/**
 * PERSISTENT EXPERIMENT RECORDS
 *
 * Every terminal run (win or loss) of any model, on any terrain, with any
 * controller is scored and stored in localStorage, so the best and worst
 * outcomes in the "10 Locomotion Experiments Performance Metrics Matrix"
 * survive across sessions — a run recorded today still ranks against runs
 * performed years earlier or later.
 *
 * Ranking score (higher is better):
 *   win:  1,000,000 − completionTime·100 + avgVelocity·10
 *   loss: distanceTraveled·1000  (losses never outrank a win)
 */

export const RECORDS_STORAGE_KEY = 'tensegrity-experiment-records-v1';
export const RECORDS_SCHEMA_VERSION = 1;
const MAX_ENTRIES = 5000;

const clampNumber = value => Number.isFinite(value) ? value : 0;

/** Higher score = better run. Wins always dominate losses. */
export function scoreRun(entry) {
  if (entry.outcome === 'win') {
    return 1_000_000-clampNumber(entry.completionTime)*100+clampNumber(entry.avgVelocity)*10;
  }
  if (entry.outcome === 'loss') {
    return clampNumber(entry.distanceTraveled)*1000;
  }
  return -1_000_000+clampNumber(entry.distanceTraveled)*1000;
}

export function loadRecords(storage) {
  try {
    const raw = storage?.getItem?.(RECORDS_STORAGE_KEY);
    if (!raw) return { version: RECORDS_SCHEMA_VERSION, entries: [], snapshots: {} };
    const parsed = JSON.parse(raw);
    if (!parsed || !Array.isArray(parsed.entries)) {
      return { version: RECORDS_SCHEMA_VERSION, entries: [], snapshots: {} };
    }
    return {
      version: RECORDS_SCHEMA_VERSION,
      entries: parsed.entries.filter(entry => entry && Number.isFinite(entry.expId)),
      // Per-experiment latest-metric snapshots (additive schema): restored
      // after a refresh so the matrix keeps every terrain's last known data.
      snapshots: parsed.snapshots && typeof parsed.snapshots === 'object'
        ? parsed.snapshots : {}
    };
  } catch {
    return { version: RECORDS_SCHEMA_VERSION, entries: [], snapshots: {} };
  }
}

export function saveRecords(storage, store) {
  if (!storage?.setItem) return false;
  try {
    storage.setItem(RECORDS_STORAGE_KEY, JSON.stringify(store));
    return true;
  } catch {
    return false; // quota exceeded / private mode: records stay in memory
  }
}

/** Insert a terminal run. Returns which records it claimed. */
export function recordRun(store, entry) {
  const stamped = {
    ...entry,
    timestamp: entry.timestamp || new Date().toISOString()
  };
  const previousBest = bestRunFor(store, stamped.expId);
  const previousWorst = worstRunFor(store, stamped.expId);
  store.entries.push(stamped);
  if (store.entries.length > MAX_ENTRIES) {
    store.entries.splice(0, store.entries.length-MAX_ENTRIES);
  }
  const newBest = !previousBest || scoreRun(stamped) > scoreRun(previousBest);
  const newWorst = !previousWorst || scoreRun(stamped) < scoreRun(previousWorst);
  return { entry: stamped, newBest, newWorst };
}

function runsFor(store, expId) {
  return store.entries.filter(entry => entry.expId === expId);
}

export function bestRunFor(store, expId) {
  const runs = runsFor(store, expId);
  if (!runs.length) return null;
  return runs.reduce((best, run) => scoreRun(run) > scoreRun(best) ? run : best);
}

export function worstRunFor(store, expId) {
  const runs = runsFor(store, expId);
  if (!runs.length) return null;
  return runs.reduce((worst, run) => scoreRun(run) < scoreRun(worst) ? run : worst);
}

export function summarizeForExperiment(store, expId) {
  const runs = runsFor(store, expId);
  const wins = runs.filter(run => run.outcome === 'win').length;
  return {
    best: bestRunFor(store, expId),
    worst: worstRunFor(store, expId),
    total: runs.length,
    wins
  };
}

/** Human-friendly age for the "2 years back or 2 years later" requirement. */
export function relativeAge(isoString, now = Date.now()) {
  const then = Date.parse(isoString);
  if (!Number.isFinite(then)) return '';
  const seconds = Math.max(0, (now-then)/1000);
  if (seconds < 90) return 'just now';
  const minutes = seconds/60;
  if (minutes < 90) return `${Math.round(minutes)} min ago`;
  const hours = minutes/60;
  if (hours < 36) return `${Math.round(hours)} h ago`;
  const days = hours/24;
  if (days < 45) return `${Math.round(days)} d ago`;
  const months = days/30.44;
  if (months < 18) return `${Math.round(months)} mo ago`;
  const years = days/365.25;
  const value = years >= 100 ? '99+' : years.toFixed(1).replace(/\.0$/, '');
  return `${value} yr ago`;
}

export function clearRecords(store) {
  store.entries.length = 0;
  store.snapshots = {};
}

/**
 * Persist the latest live metrics of an experiment so its matrix row keeps
 * real data across page refreshes — even when no run ever reached a terminal
 * win/loss. Peak fields (G, tension, deformation) accumulate all-time maxima
 * across captures; everything else reflects the most recent capture.
 */
export function saveExperimentSnapshot(store, expId, data) {
  if (!Number.isFinite(expId)) return null;
  store.snapshots = store.snapshots || {};
  const previous = store.snapshots[String(expId)];
  const updatedAt = data.updatedAt || new Date().toISOString();
  const peak = (previousValue, value) => {
    const a = Number.isFinite(previousValue) ? previousValue : -Infinity;
    const b = Number.isFinite(value) ? value : -Infinity;
    const maximum = Math.max(a, b);
    return Number.isFinite(maximum) ? maximum : 0;
  };
  const snapshot = {
    expId,
    terrainLevel: data.terrainLevel ?? previous?.terrainLevel ?? null,
    gravity: Number.isFinite(data.gravity) ? data.gravity : previous?.gravity ?? null,
    controllerShort: data.controllerShort || previous?.controllerShort || null,
    modelLabel: data.modelLabel || previous?.modelLabel || 'Model B',
    // Latest capture values
    distance: clampNumber(data.distance),
    elapsed: clampNumber(data.elapsed),
    avgVelocity: clampNumber(data.avgVelocity),
    obstacles: Number.isFinite(data.obstacles) ? data.obstacles : previous?.obstacles ?? null,
    outcome: data.outcome || previous?.outcome || 'running',
    // All-time peaks across every capture of this experiment
    maxG: peak(previous?.maxG, data.maxG),
    maxTension: peak(previous?.maxTension, data.maxTension),
    deformation: peak(previous?.deformation, data.deformation),
    bestAvgVelocity: peak(previous?.bestAvgVelocity, data.avgVelocity),
    // Bookkeeping
    captures: (previous?.captures || 0)+1,
    firstCapturedAt: previous?.firstCapturedAt || updatedAt,
    updatedAt
  };
  store.snapshots[String(expId)] = snapshot;
  return snapshot;
}

export function getExperimentSnapshot(store, expId) {
  return store?.snapshots?.[String(expId)] || null;
}

/** One-line status badge for a persisted snapshot row. */
export function formatSnapshotStatus(snapshot) {
  if (!snapshot) return '';
  const parts = [`SAVED · ${snapshot.captures} capture${snapshot.captures === 1 ? '' : 's'}`];
  const age = relativeAge(snapshot.updatedAt);
  if (age) parts.push(age);
  return parts.join(' · ');
}

/** Compact one-line cell text for the matrix. */
export function formatRecordCell(run) {
  if (!run) return '—';
  const outcome = run.outcome === 'win' ? 'WIN' : run.outcome === 'loss' ? 'LOSS' : 'RUN';
  const time = Number.isFinite(run.completionTime) ? `${run.completionTime.toFixed(1)}s` : null;
  const speed = Number.isFinite(run.avgVelocity) ? `${run.avgVelocity.toFixed(2)} m/s` : null;
  const distance = Number.isFinite(run.distanceTraveled) ? `${run.distanceTraveled.toFixed(1)} m` : null;
  const detail = [time, speed || distance].filter(Boolean).join(' · ');
  const age = relativeAge(run.timestamp);
  return `${outcome}${detail ? ` · ${detail}` : ''} · ${run.modelLabel || run.model || ''}`+
    `${run.controllerShort ? ` · ${run.controllerShort}` : ''}${age ? ` · ${age}` : ''}`;
}
