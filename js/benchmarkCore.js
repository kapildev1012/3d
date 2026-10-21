/**
 * BENCHMARKCORE.JS - run-record schema, merge, ranking and export
 *
 * Shared by the browser (js/benchmarkPersistence.js, js/runRecorder.js), the
 * local Node server (server.mjs) and the serverless API (api/benchmarks.js),
 * so every storage layer merges records with exactly the same rules.
 *
 * State (schema v2):
 *   { version: 2, updatedAt: ISO, clearedAt: ISO | null, runs: Run[] }
 *
 * Run:
 *   { id, level, source: 'live'|'auto'|'seed'|'import'|'legacy',
 *     status: 'active'|'completed'|'failed'|'terminated',
 *     reason, final, startedAt, updatedAt, endedAt, checkpoints,
 *     controller, controllerShort, gait, gaitChanges, physicsCore, actuation,
 *     gravity, seed, simTime, obstacles,
 *     A: ModelMetrics | null,   // Model A, fixed baseline
 *     B: ModelMetrics | null }  // Model B, adaptive (primary for ranking)
 *
 * ModelMetrics:
 *   { distance, time, avgSpeed, maxG, maxTension, deformation,
 *     over, around, outcome, completionTime }
 *
 * Merge rules (every layer, commutative and idempotent):
 *   - union of run ids;
 *   - for the same id the record with the later updatedAt wins; a final
 *     record beats a non-final one with the same timestamp;
 *   - clearedAt is a tombstone: runs last updated before the newest
 *     clearedAt of any layer are dropped, so "Clear saved records" cannot be
 *     undone by another browser, the disk file or IndexedDB re-syncing.
 */

export const BENCHMARK_SCHEMA_VERSION = 2;
export const LEVEL_COUNT = 14;
export const MAX_RUNS = 20000;

/** Minimum run that is worth keeping (accidental double-resets are not runs). */
export const MIN_RUN_TIME = 0.5;      // [s] simulated
export const MIN_RUN_DISTANCE = 0.01; // [m]
/** Shortest run that may be reported as the all-time worst. */
export const MIN_WORST_TIME = 1.0;    // [s]

const finite = value => (typeof value === 'number' && Number.isFinite(value) ? value : null);
const iso = value => {
  const time = Date.parse(value);
  return Number.isFinite(time) ? new Date(time).toISOString() : null;
};
const ms = value => {
  const time = Date.parse(value);
  return Number.isFinite(time) ? time : 0;
};

export function emptyState() {
  return { version: BENCHMARK_SCHEMA_VERSION, updatedAt: null, clearedAt: null, runs: [] };
}

export function newRunId() {
  const random = (typeof crypto !== 'undefined' && crypto.randomUUID)
    ? crypto.randomUUID()
    : `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
  return `run-${random}`;
}

function normalizeModel(model) {
  if (!model || typeof model !== 'object') return null;
  return {
    distance: finite(model.distance) ?? 0,
    time: finite(model.time) ?? 0,
    avgSpeed: finite(model.avgSpeed) ?? 0,
    maxG: finite(model.maxG),
    maxTension: finite(model.maxTension),
    deformation: finite(model.deformation),
    over: finite(model.over),
    around: finite(model.around),
    outcome: typeof model.outcome === 'string' ? model.outcome : null,
    completionTime: finite(model.completionTime)
  };
}

const STATUSES = new Set(['active', 'completed', 'failed', 'terminated']);

/** Validate one run; returns null for anything that is not a usable record. */
export function normalizeRun(run) {
  if (!run || typeof run !== 'object') return null;
  const level = Math.round(Number(run.level));
  if (!(level >= 1 && level <= LEVEL_COUNT)) return null;
  const id = typeof run.id === 'string' && run.id.length <= 120 ? run.id : null;
  if (!id) return null;
  const updatedAt = iso(run.updatedAt) || iso(run.startedAt);
  if (!updatedAt) return null;
  const status = STATUSES.has(run.status) ? run.status : 'terminated';
  const str = (value, max = 80) => (typeof value === 'string' ? value.slice(0, max) : null);
  return {
    id,
    level,
    source: str(run.source, 16) || 'live',
    status,
    reason: str(run.reason, 40),
    final: Boolean(run.final) || status === 'completed' || status === 'failed',
    startedAt: iso(run.startedAt) || updatedAt,
    updatedAt,
    endedAt: iso(run.endedAt),
    checkpoints: finite(run.checkpoints) ?? 0,
    controller: str(run.controller),
    controllerShort: str(run.controllerShort, 24),
    gait: str(run.gait, 24),
    gaitChanges: finite(run.gaitChanges) ?? 0,
    physicsCore: str(run.physicsCore, 16),
    actuation: str(run.actuation, 16),
    gravity: finite(run.gravity),
    seed: finite(run.seed),
    simTime: finite(run.simTime) ?? 0,
    obstacles: finite(run.obstacles),
    A: normalizeModel(run.A),
    B: normalizeModel(run.B)
  };
}

export function normalizeState(input) {
  const state = emptyState();
  if (!input || typeof input !== 'object') return state;
  state.clearedAt = iso(input.clearedAt);
  state.updatedAt = iso(input.updatedAt);
  const runs = Array.isArray(input.runs) ? input.runs : [];
  const byId = new Map();
  for (const raw of runs) {
    const run = normalizeRun(raw);
    if (run) byId.set(run.id, pickNewer(byId.get(run.id), run));
  }
  state.runs = applyTombstone([...byId.values()], state.clearedAt);
  return state;
}

function pickNewer(a, b) {
  if (!a) return b;
  if (!b) return a;
  const ta = ms(a.updatedAt), tb = ms(b.updatedAt);
  if (tb > ta) return b;
  if (ta > tb) return a;
  if (b.final && !a.final) return b;
  return a;
}

function applyTombstone(runs, clearedAt) {
  const cut = clearedAt ? ms(clearedAt) : 0;
  const kept = cut ? runs.filter(run => ms(run.updatedAt) > cut) : runs;
  kept.sort((a, b) => ms(a.startedAt)-ms(b.startedAt));
  return kept.length > MAX_RUNS ? kept.slice(kept.length-MAX_RUNS) : kept;
}

/** Union-merge any number of states (already normalized or raw). */
export function mergeStates(...inputs) {
  const states = inputs.map(normalizeState);
  const clearedAt = states.map(s => s.clearedAt).filter(Boolean).sort().pop() || null;
  const byId = new Map();
  for (const state of states) {
    for (const run of state.runs) byId.set(run.id, pickNewer(byId.get(run.id), run));
  }
  const updatedAt = states.map(s => s.updatedAt).filter(Boolean).sort().pop() || null;
  return { version: BENCHMARK_SCHEMA_VERSION, updatedAt, clearedAt, runs: applyTombstone([...byId.values()], clearedAt) };
}

/** Is this run long enough to be stored at all? */
export function isMeaningfulRun(run) {
  const primary = primaryMetrics(run);
  return (run?.simTime ?? 0) >= MIN_RUN_TIME || (primary?.distance ?? 0) > MIN_RUN_DISTANCE;
}

/** Model B (adaptive) is the primary model; legacy single-model records fall back to A. */
export function primaryMetrics(run) {
  return run?.B || run?.A || null;
}

/**
 * Composite rank tuple (larger is better), compared lexicographically:
 *   completed -> distance -> average speed -> faster completion
 *   -> lower peak tension -> lower peak payload G.
 */
export function rankTuple(run) {
  const m = primaryMetrics(run) || {};
  const completed = run.status === 'completed' ? 1 : 0;
  const distance = Math.round((m.distance ?? 0)*100)/100;       // 1 cm resolution
  const speed = Math.round((m.avgSpeed ?? 0)*1000)/1000;        // 1 mm/s
  const fastest = completed && Number.isFinite(m.completionTime) ? -m.completionTime : -Infinity;
  const tension = Number.isFinite(m.maxTension) ? -m.maxTension : -Infinity;
  const gforce = Number.isFinite(m.maxG) ? -m.maxG : -Infinity;
  return [completed, distance, speed, fastest, tension, gforce];
}

export function compareRuns(a, b) {
  const ta = rankTuple(a), tb = rankTuple(b);
  for (let i = 0; i < ta.length; i++) {
    if (ta[i] > tb[i]) return 1;
    if (ta[i] < tb[i]) return -1;
  }
  return ms(b.startedAt)-ms(a.startedAt) > 0 ? 1 : 0; // earlier run wins ties
}

export function runsForLevel(state, level) {
  return (state?.runs || []).filter(run => run.level === level);
}

export function bestRun(runs) {
  return runs.length ? runs.reduce((best, run) => (compareRuns(run, best) > 0 ? run : best)) : null;
}

export function worstRun(runs) {
  const valid = runs.filter(run => (run.simTime ?? 0) > MIN_WORST_TIME && run.status !== 'active');
  return valid.length ? valid.reduce((worst, run) => (compareRuns(run, worst) < 0 ? run : worst)) : null;
}

export function latestRun(runs) {
  return runs.length ? runs.reduce((a, b) => (ms(b.updatedAt) > ms(a.updatedAt) ? b : a)) : null;
}

/** Per-level summary used by the matrix. */
export function summarizeLevel(state, level) {
  const runs = runsForLevel(state, level);
  return {
    level,
    count: runs.length,
    completed: runs.filter(run => run.status === 'completed').length,
    latest: latestRun(runs),
    best: bestRun(runs),
    worst: worstRun(runs),
    runs
  };
}

/** Status label shown in the matrix for a stored (not currently live) run. */
export function displayStatus(run) {
  if (!run) return 'NOT RUN';
  if (run.status === 'completed') return 'COMPLETED';
  if (run.status === 'failed') return 'FAILED';
  return 'INCOMPLETE';
}

export function formatRunSummary(run) {
  const m = primaryMetrics(run);
  if (!m) return '—';
  const time = run.status === 'completed' && Number.isFinite(m.completionTime) ? m.completionTime : m.time;
  return `${(m.distance ?? 0).toFixed(1)} m in ${(time ?? 0).toFixed(1)} s (${(m.avgSpeed ?? 0).toFixed(2)} m/s)`;
}

// ------------------------------------------------------------------ export

const CSV_COLUMNS = [
  ['id', r => r.id], ['level', r => r.level], ['source', r => r.source], ['status', r => r.status],
  ['reason', r => r.reason], ['started_at', r => r.startedAt], ['ended_at', r => r.endedAt],
  ['sim_time_s', r => r.simTime], ['controller', r => r.controller], ['gait', r => r.gait],
  ['physics_core', r => r.physicsCore], ['actuation', r => r.actuation], ['gravity_mps2', r => r.gravity],
  ['seed', r => r.seed], ['obstacles', r => r.obstacles],
  ...['B', 'A'].flatMap(model => [
    [`${model}_distance_m`, r => r[model]?.distance], [`${model}_time_s`, r => r[model]?.time],
    [`${model}_avg_speed_mps`, r => r[model]?.avgSpeed], [`${model}_max_g`, r => r[model]?.maxG],
    [`${model}_max_tension_N`, r => r[model]?.maxTension], [`${model}_deformation_m`, r => r[model]?.deformation],
    [`${model}_obstacles_over`, r => r[model]?.over], [`${model}_obstacles_around`, r => r[model]?.around],
    [`${model}_outcome`, r => r[model]?.outcome], [`${model}_completion_s`, r => r[model]?.completionTime]
  ])
];

export function toCSV(state) {
  const escape = value => {
    if (value === null || value === undefined) return '';
    const text = String(value);
    return /[",\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
  };
  const lines = [CSV_COLUMNS.map(([name]) => name).join(',')];
  for (const run of state.runs) lines.push(CSV_COLUMNS.map(([, get]) => escape(get(run))).join(','));
  return lines.join('\n')+'\n';
}

// ------------------------------------------------------------------ migration

/**
 * Convert the previous localStorage store ('tensegrity-experiment-records-v1':
 * terminal win/loss entries + per-experiment snapshots) into v2 runs, so no
 * earlier record is lost by the upgrade. Ids are deterministic, so migrating
 * twice never duplicates.
 */
export function migrateLegacyRecords(legacy) {
  const runs = [];
  if (!legacy || typeof legacy !== 'object') return runs;
  const entries = Array.isArray(legacy.entries) ? legacy.entries : [];
  entries.forEach((entry, index) => {
    const level = Math.round(Number(entry?.expId));
    if (!(level >= 1 && level <= LEVEL_COUNT)) return;
    const stamp = iso(entry.timestamp) || new Date(0).toISOString();
    const metrics = {
      distance: entry.distanceTraveled, time: entry.completionTime ?? null, avgSpeed: entry.avgVelocity,
      maxG: entry.payloadAccelMax, maxTension: entry.maxCableTension, deformation: entry.shapeDeformationMax,
      outcome: entry.outcome, completionTime: entry.completionTime
    };
    if (!Number.isFinite(metrics.time) && Number.isFinite(entry.distanceTraveled) && entry.avgVelocity > 0) {
      metrics.time = entry.distanceTraveled/entry.avgVelocity;
    }
    runs.push({
      id: `legacy-${level}-${ms(stamp)}-${index}`, level, source: 'legacy',
      status: entry.outcome === 'win' ? 'completed' : entry.outcome === 'loss' ? 'failed' : 'terminated',
      reason: entry.outcome === 'loss' ? 'deadline' : entry.outcome === 'win' ? 'goal' : 'legacy',
      final: true, startedAt: stamp, updatedAt: stamp, endedAt: stamp,
      controller: entry.controllerMode || null, controllerShort: entry.controllerShort || null,
      simTime: Number.isFinite(metrics.time) ? metrics.time : 0,
      A: entry.model === 'A' ? metrics : null,
      B: entry.model === 'A' ? null : metrics
    });
  });
  const snapshots = legacy.snapshots && typeof legacy.snapshots === 'object' ? legacy.snapshots : {};
  for (const [key, snap] of Object.entries(snapshots)) {
    const level = Math.round(Number(snap?.expId ?? key));
    if (!(level >= 1 && level <= LEVEL_COUNT) || !snap) continue;
    const stamp = iso(snap.updatedAt) || new Date(0).toISOString();
    runs.push({
      id: `legacy-snapshot-${level}-${ms(stamp)}`, level, source: 'legacy', status: 'terminated',
      reason: 'snapshot', final: true, startedAt: iso(snap.firstCapturedAt) || stamp, updatedAt: stamp, endedAt: stamp,
      controllerShort: snap.controllerShort || null, gravity: snap.gravity,
      simTime: snap.elapsed || 0, obstacles: snap.obstacles,
      B: { distance: snap.distance, time: snap.elapsed, avgSpeed: snap.avgVelocity, maxG: snap.maxG,
        maxTension: snap.maxTension, deformation: snap.deformation, outcome: snap.outcome }
    });
  }
  return runs.map(normalizeRun).filter(run => run && isMeaningfulRun(run));
}
