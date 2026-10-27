/**
 * PARALLELPOOL.JS - zero-copy multi-simulation worker pool.
 *
 * N headless 500 Hz Lagrangian-DAE simulations run in parallel, in
 * `worker_threads` under Node (npm run experiments / hybrid training) and in
 * Web Workers in the browser. After a one-time bootstrap message that hands
 * over the SharedArrayBuffers, *all* traffic is shared memory:
 *
 *   ctrl   Int32   per-env command word: coordinator stores a code and
 *                  Atomics.notify; the worker Atomics.wait()s, runs the
 *                  command, stores 0 and notifies back.
 *   obs    F64     48-D observation of every env after its last tick
 *   act    F64     24-D exploration noise injected at the next tick
 *   tel    F64     17-D telemetry of the last 20 Hz tick (25 DAE sub-steps)
 *   noise  F64     [K][H][24] candidate noise sequences  (coordinator -> workers)
 *   ret    F64     [K] discounted rollout returns         (workers -> coordinator)
 *   params F64     MPC cost / reward / bound parameters   (coordinator -> workers)
 *   weights F64    actor weights                          (coordinator -> workers)
 *   state  bytes   one checkpoint slot: a serialized simulation
 *                  (js/simSnapshot.js) from which every worker can branch.
 *
 * Commands (cmd codes):
 *   RESET      build a fresh simulation from the JSON spec in the spec slot
 *   STEP       advance one 20 Hz control tick (25 sub-steps) with noise act[i]
 *   SAVE       keep a local snapshot (serializeState)
 *   RESTORE    restore the local snapshot
 *   PUBLISH    write this env's state into the shared checkpoint slot
 *   BRANCH     load the shared checkpoint into this env
 *   ROLLOUT    BRANCH if the checkpoint is new, then roll candidate arg0 for
 *              H ticks with the actor prior + noise, writing its return
 *   EPISODE    run a whole episode with the actor prior; write a benchmark digest
 *   EXIT       terminate
 *
 * The coordinator awaits workers with Atomics.waitAsync (never blocks the
 * browser main thread). Workers only block in Atomics.wait.
 */
import { OBS_DIM, ACT_DIM, TEL_DIM, MPC_PARAM_DIM, ACTOR_PARAM_COUNT, packParams, DEFAULT_PARAMS } from './residualMppi.js';

export const CMD = Object.freeze({ IDLE: 0, RESET: 1, STEP: 2, SAVE: 3, RESTORE: 4, PUBLISH: 5, BRANCH: 6, ROLLOUT: 7, EPISODE: 8, EXIT: 9 });
export const DIGEST_FIELDS = Object.freeze([
  'avgSpeedMs', 'rmseM', 'maxErrorM', 'recoveryTimeS', 'controlEffortUtu', 'costOfTransport', 'solveTimeMs',
  'successRatePct', 'rodResidualMm', 'relaxedCablesPct', 'contactSlipMs', 'energyDriftJ', 'progressM', 'tips', 'aborts', 'maxOuterTensionN'
]);
export const DIGEST_DIM = DIGEST_FIELDS.length;
export const SPEC_BYTES = 8192;
export const CHECKPOINT_BYTES = 16*1024*1024;
export const ERR_BYTES = 512;
export const MAX_HORIZON = 24;

const isNode = typeof process !== 'undefined' && Boolean(process.versions?.node) && typeof window === 'undefined';

/** Allocates every shared buffer. Plain object of SharedArrayBuffers (transferable by reference to workers). */
export function allocateShared(numEnvs, numCandidates, maxHorizon = MAX_HORIZON) {
  if (typeof SharedArrayBuffer === 'undefined') {
    throw new Error('SharedArrayBuffer is unavailable (browser pages need cross-origin isolation: COOP same-origin + COEP)');
  }
  const f64 = n => new SharedArrayBuffer(n*8), i32 = n => new SharedArrayBuffer(n*4);
  return {
    numEnvs, numCandidates, maxHorizon,
    ctrl: i32(numEnvs), err: i32(numEnvs), arg: i32(numEnvs*4), ckpt: i32(4),
    specLen: i32(numEnvs), spec: new SharedArrayBuffer(numEnvs*SPEC_BYTES), errText: new SharedArrayBuffer(numEnvs*ERR_BYTES),
    obs: f64(numEnvs*OBS_DIM), act: f64(numEnvs*ACT_DIM), tel: f64(numEnvs*TEL_DIM),
    decObs: f64(numEnvs*OBS_DIM), decAct: f64(numEnvs*ACT_DIM), decFlag: i32(numEnvs),
    noise: f64(numCandidates*maxHorizon*ACT_DIM), ret: f64(numCandidates), termObs: f64(numCandidates*OBS_DIM),
    candTel: f64(numCandidates*TEL_DIM), candInfo: f64(numCandidates*4),
    params: f64(MPC_PARAM_DIM), weights: f64(ACTOR_PARAM_COUNT),
    digest: f64(numEnvs*DIGEST_DIM), state: new SharedArrayBuffer(CHECKPOINT_BYTES)
  };
}

/** Typed views over the shared buffers (same in coordinator and workers). */
export function sharedViews(s) {
  return {
    ctrl: new Int32Array(s.ctrl), err: new Int32Array(s.err), arg: new Int32Array(s.arg), ckpt: new Int32Array(s.ckpt),
    specLen: new Int32Array(s.specLen), spec: new Uint8Array(s.spec), errText: new Uint8Array(s.errText),
    obs: new Float64Array(s.obs), act: new Float64Array(s.act), tel: new Float64Array(s.tel),
    decObs: new Float64Array(s.decObs), decAct: new Float64Array(s.decAct), decFlag: new Int32Array(s.decFlag),
    noise: new Float64Array(s.noise), ret: new Float64Array(s.ret), termObs: new Float64Array(s.termObs),
    candTel: new Float64Array(s.candTel), candInfo: new Float64Array(s.candInfo),
    params: new Float64Array(s.params), weights: new Float64Array(s.weights),
    digest: new Float64Array(s.digest), state: new Uint8Array(s.state)
  };
}

export class SharedVecTensegrityEnv {
  /**
   * @param {object} options numEnvs (workers), numCandidates (rollout candidates K), maxHorizon
   */
  constructor({ numEnvs = 4, numCandidates = 8, maxHorizon = MAX_HORIZON, baseUrl = import.meta.url } = {}) {
    this.numEnvs = numEnvs;
    this.numCandidates = numCandidates;
    this.maxHorizon = maxHorizon;
    this.shared = allocateShared(numEnvs, numCandidates, maxHorizon);
    this.v = sharedViews(this.shared);
    this.workers = [];
    this.baseUrl = baseUrl;
    this.ready = null;
    this.setParams(DEFAULT_PARAMS);
  }

  async start() {
    if (this.ready) return this.ready;
    this.ready = (async () => {
      if (isNode) {
        const { Worker } = await import('node:worker_threads');
        const url = new URL('./parallelWorker.js', this.baseUrl);
        for (let id = 0; id < this.numEnvs; id++) {
          const worker = new Worker(url, { workerData: { id, shared: this.shared } });
          worker.on('error', error => console.error(`[pool] worker ${id}:`, error));
          this.workers.push(worker);
        }
      } else {
        const url = new URL('./parallelWorker.js', this.baseUrl);
        for (let id = 0; id < this.numEnvs; id++) {
          const worker = new Worker(url, { type: 'module' });
          worker.onerror = event => console.error(`[pool] worker ${id}:`, event.message);
          worker.postMessage({ id, shared: this.shared });
          this.workers.push(worker);
        }
      }
      // Wait until every worker has loaded the simulator and parked in Atomics.wait.
      await Promise.all(this.workers.map((_, id) => this.command(id, CMD.IDLE, true)));
    })();
    return this.ready;
  }

  setParams(params) {
    packParams(params, this.v.params);
  }
  setWeights(flat) { this.v.weights.set(flat); }

  specToSlot(id, spec) {
    const bytes = new TextEncoder().encode(JSON.stringify(spec));
    if (bytes.length > SPEC_BYTES) throw new Error('spec too large');
    this.v.spec.set(bytes, id*SPEC_BYTES);
    Atomics.store(this.v.specLen, id, bytes.length);
  }

  /** Issue one command to one env and await its completion. */
  async command(id, code, handshake = false, args = null) {
    const { ctrl, arg } = this.v;
    if (args) for (let k = 0; k < args.length; k++) Atomics.store(arg, id*4+k, args[k]);
    Atomics.store(this.v.err, id, 0);
    if (handshake) {
      // The worker sets ctrl[id] = -1 once it is parked; wait for that signal.
      for (;;) {
        const cur = Atomics.load(ctrl, id);
        if (cur === -1) { Atomics.store(ctrl, id, 0); return; }
        await this.sleepOn(ctrl, id, cur);
      }
    }
    Atomics.store(ctrl, id, code);
    Atomics.notify(ctrl, id);
    await this.settle(id);
  }

  async settle(id) {
    const { ctrl } = this.v;
    for (;;) {
      const cur = Atomics.load(ctrl, id);
      if (cur === 0) break;
      await this.sleepOn(ctrl, id, cur);
    }
    if (Atomics.load(this.v.err, id)) {
      const text = new TextDecoder().decode(this.v.errText.slice(id*ERR_BYTES, (id+1)*ERR_BYTES)).replace(/\0+$/, '');
      throw new Error(`worker ${id}: ${text}`);
    }
  }

  sleepOn(view, index, expected) {
    if (typeof Atomics.waitAsync === 'function') {
      const r = Atomics.waitAsync(view, index, expected, 1000);
      return r.async ? r.value : Promise.resolve();
    }
    return new Promise(resolve => setTimeout(resolve, 1));
  }

  /** Run the same command on a set of envs concurrently. */
  async broadcast(code, ids = null, args = null) {
    ids = ids || Array.from({ length: this.numEnvs }, (_, i) => i);
    // Issue all first, then await all (true parallelism).
    for (const id of ids) {
      const a = typeof args === 'function' ? args(id) : args;
      if (a) for (let k = 0; k < a.length; k++) Atomics.store(this.v.arg, id*4+k, a[k]);
      Atomics.store(this.v.err, id, 0);
      Atomics.store(this.v.ctrl, id, code);
      Atomics.notify(this.v.ctrl, id);
    }
    await Promise.all(ids.map(id => this.settle(id)));
  }

  async reset(id, spec) {
    this.specToSlot(id, spec);
    await this.command(id, CMD.RESET);
  }
  async resetAll(specFor) {
    for (let id = 0; id < this.numEnvs; id++) this.specToSlot(id, specFor(id));
    await this.broadcast(CMD.RESET);
  }

  getEnvObs(id) { return this.v.obs.subarray(id*OBS_DIM, (id+1)*OBS_DIM); }
  getEnvTelemetry(id) { return this.v.tel.subarray(id*TEL_DIM, (id+1)*TEL_DIM); }
  setEnvAction(id, action24) { this.v.act.set(action24, id*ACT_DIM); }
  getDigest(id) {
    const d = {};
    DIGEST_FIELDS.forEach((name, i) => { d[name] = this.v.digest[id*DIGEST_DIM+i]; });
    return d;
  }

  async terminate() {
    try { await this.broadcast(CMD.EXIT); } catch { /* workers may already be gone */ }
    for (const w of this.workers) { if (w.terminate) await w.terminate(); }
    this.workers = [];
  }
}
