/**
 * SIMSNAPSHOT.JS - exact state capture / restore for the Lagrangian-DAE simulator.
 *
 *   const state = serializeState(sim);      // structured-cloneable tree
 *   deserializeState(sim, state);           // overlay onto a live Simulation
 *   const bytes = encodeState(state);       // Uint8Array (JSON, lossless doubles)
 *   const state2 = decodeState(bytes);
 *
 * Why it exists: sampling-based MPC (residual-MPPI, js/residualMppi.js) must
 * branch many rollouts from one identical state. Every worker owns a
 * Simulation built from the same spec (so configuration, terrain, rover
 * geometry and callbacks are identical by construction); only the *dynamic*
 * state is transported:
 *
 *   - positions / velocities / rest lengths / actuator offsets / contact state,
 *   - the Lagrangian DAE integrator (multipliers, warm starts),
 *   - the energy ledger (so drift stays booked consistently across a branch),
 *   - controller state machines (SGT SETTLE/SHIFT/TIP/RELEASE, failed edges),
 *   - the command queue (actuator delay), sensor RNG streams,
 *   - the A/B crest-checkpoint tracker (Levels 1-14), speed statistics.
 *
 * Skipped (static or log-only): cfg, rover, terrain, routeLearner, the long
 * history arrays, controller `opts`, and back-references to the simulation.
 * Restoring overlays *in place*, so objects aliased elsewhere (the monitor
 * keeps a reference to sim.q) stay valid.
 */

const STATIC_SIM_KEYS = new Set(['cfg', 'rover', 'terrain', 'terrainSource', 'routeLearner', 'history', 'onlinePlanner', 'preStep']);
// 'field' = the shared obstacle list referenced by FieldObstacleTracker (static, owned by the terrain).
const STATIC_OBJECT_KEYS = new Set(['sim', 'opts', 'cfg', 'rover', 'terrain', 'terrainSource', 'routeLearner', 'field']);
const NONFINITE = { 'Infinity': Infinity, '-Infinity': -Infinity, 'NaN': NaN };
const TYPED = { Float64Array, Float32Array, Int32Array, Uint8Array, Uint32Array, Int16Array, Uint16Array, Int8Array };

function encodeValue(value, depth = 0) {
  if (depth > 60) throw new Error('simSnapshot: structure too deep (cycle?)');
  switch (typeof value) {
    case 'number':
      return Number.isFinite(value) ? value : { $n: String(value) };
    case 'string': case 'boolean': return value;
    case 'undefined': return { $u: 1 };
    case 'function': return undefined;          // callbacks come from the live object
    case 'bigint': return Number(value);
    default: break;
  }
  if (value === null) return null;
  if (Array.isArray(value)) return value.map(item => { const e = encodeValue(item, depth+1); return e === undefined ? null : e; });
  if (ArrayBuffer.isView(value)) return { $t: value.constructor.name, d: value.slice() };
  if (value instanceof Map) return { $m: [...value].map(([k, v]) => [encodeValue(k, depth+1), encodeValue(v, depth+1)]) };
  if (value instanceof Set) return { $s: [...value].map(v => encodeValue(v, depth+1)) };
  const out = {};
  const name = value.constructor && value.constructor !== Object ? value.constructor.name : null;
  for (const key of Object.keys(value)) {
    if (STATIC_OBJECT_KEYS.has(key)) continue;
    const encoded = encodeValue(value[key], depth+1);
    if (encoded !== undefined) out[key] = encoded;
  }
  return name ? { $c: name, f: out } : out;
}

/** Plain, structured-cloneable snapshot of the dynamic simulation state. */
export function serializeState(sim) {
  const fields = {};
  for (const key of Object.keys(sim)) {
    if (STATIC_SIM_KEYS.has(key)) continue;
    const encoded = encodeValue(sim[key]);
    if (encoded !== undefined) fields[key] = encoded;
  }
  return { version: 1, t: sim.t, stepCount: sim.stepCount, fields };
}

function decodeScalar(node) {
  if (node && typeof node === 'object') {
    if ('$n' in node) return NONFINITE[node.$n];
    if ('$u' in node) return undefined;
  }
  return node;
}

/** Merge `node` into `target` in place when shapes agree; return the merged value. */
function overlay(target, node, hooks, path) {
  if (node === null || typeof node !== 'object') return node;
  if (Array.isArray(node)) {
    const arr = Array.isArray(target) ? target : [];
    arr.length = node.length;
    for (let i = 0; i < node.length; i++) arr[i] = overlay(arr[i], node[i], hooks, `${path}[${i}]`);
    return arr;
  }
  if ('$n' in node || '$u' in node) return decodeScalar(node);
  if ('$t' in node) {
    const Ctor = TYPED[node.$t];
    if (!Ctor) throw new Error(`simSnapshot: unsupported typed array ${node.$t}`);
    if (target instanceof Ctor && target.length === node.d.length) { target.set(node.d); return target; }
    return Ctor.from(node.d);
  }
  if ('$m' in node) {
    const map = target instanceof Map ? target : new Map();
    map.clear();
    for (const [k, v] of node.$m) map.set(overlay(undefined, k, hooks, path), overlay(undefined, v, hooks, path));
    return map;
  }
  if ('$s' in node) return new Set(node.$s.map(v => overlay(undefined, v, hooks, path)));
  if ('$c' in node) {
    let instance = target;
    if (!instance || typeof instance !== 'object' || instance.constructor?.name !== node.$c) {
      instance = hooks.create(node.$c, path);
    }
    for (const key of Object.keys(node.f)) instance[key] = overlay(instance[key], node.f[key], hooks, `${path}.${key}`);
    return instance;
  }
  const obj = target && typeof target === 'object' && !Array.isArray(target) && !ArrayBuffer.isView(target) ? target : {};
  for (const key of Object.keys(obj)) if (!(key in node) && !STATIC_OBJECT_KEYS.has(key)) delete obj[key];
  for (const key of Object.keys(node)) obj[key] = overlay(obj[key], node[key], hooks, `${path}.${key}`);
  return obj;
}

/**
 * Restore `state` into `sim` (built from the same configuration). Lazily
 * created controller objects are rebuilt through `sim.createSgt()`.
 */
export function deserializeState(sim, state) {
  if (!state || state.version !== 1) throw new Error('simSnapshot: unsupported snapshot version');
  const hooks = {
    create(className, path) {
      if (className === 'SensitivityTippingController' && path === 'sim.sgt' && sim.createSgt) {
        const sgt = sim.createSgt();
        return sgt;
      }
      throw new Error(`simSnapshot: cannot rebuild ${className} at ${path}; step the simulation once before restoring`);
    }
  };
  // Controller objects must exist before their fields are overlaid.
  if (state.fields.sgt && state.fields.sgt !== null && state.fields.sgt.$c && !sim.sgt && sim.createSgt) sim.sgt = sim.createSgt();
  for (const key of Object.keys(state.fields)) {
    sim[key] = overlay(sim[key], state.fields[key], hooks, `sim.${key}`);
  }
  // Fields that were absent from the snapshot (not yet created) are reset.
  for (const key of Object.keys(sim)) {
    if (!(key in state.fields) && !STATIC_SIM_KEYS.has(key) && typeof sim[key] !== 'function') {
      if (key === 'sgt' || key === 'gpsNav' || key === 'gpsState') sim[key] = null;
    }
  }
  return sim;
}

// ------------------------------------------------------------------ bytes (for SharedArrayBuffer slots)
function replacer(key, value) {
  if (value && typeof value === 'object' && value.$t && value.d && ArrayBuffer.isView(value.d)) {
    return { $t: value.$t, d: Array.from(value.d) };
  }
  return value;
}

/** Lossless UTF-8 JSON encoding of a snapshot (doubles round-trip exactly). */
export function encodeState(state) {
  return new TextEncoder().encode(JSON.stringify(state, replacer));
}

export function decodeState(bytes) {
  return JSON.parse(new TextDecoder().decode(bytes));
}
