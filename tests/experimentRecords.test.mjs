import test from 'node:test';
import assert from 'node:assert/strict';
import {
  clearRecords,
  formatRecordCell,
  formatSnapshotStatus,
  getExperimentSnapshot,
  loadRecords,
  recordRun,
  relativeAge,
  saveExperimentSnapshot,
  saveRecords,
  scoreRun,
  summarizeForExperiment
} from '../js/experimentRecords.js';

function fakeStorage() {
  const map = new Map();
  return {
    getItem: key => map.has(key) ? map.get(key) : null,
    setItem: (key, value) => map.set(key, String(value)),
    get size() { return map.size; }
  };
}

const run = overrides => ({
  expId: 10, terrainLevel: 10, model: 'B', modelLabel: 'Model B',
  controllerMode: 'riccati_lqr', controllerShort: 'Ric-LQR',
  outcome: 'win', completionTime: 90, distanceTraveled: 50, avgVelocity: 0.55,
  ...overrides
});

test('wins always outrank losses and faster wins score higher', () => {
  const fastWin = scoreRun(run({ completionTime: 60 }));
  const slowWin = scoreRun(run({ completionTime: 110 }));
  const farLoss = scoreRun(run({ outcome: 'loss', completionTime: null, distanceTraveled: 45 }));
  assert.ok(fastWin > slowWin, 'faster win must rank higher');
  assert.ok(slowWin > farLoss, 'even a slow win must outrank any loss');
});

test('recordRun tracks best and worst per experiment across controllers', () => {
  const store = { version: 1, entries: [] };
  let flags = recordRun(store, run({ expId: 3, controllerMode: 'cpg', completionTime: 100 }));
  assert.ok(flags.newBest && flags.newWorst, 'first run is both best and worst');

  flags = recordRun(store, run({ expId: 3, controllerMode: 'qp_mpc_proj', completionTime: 70 }));
  assert.ok(flags.newBest && !flags.newWorst);

  flags = recordRun(store, run({
    expId: 3, controllerMode: 'cpg', outcome: 'loss',
    completionTime: null, distanceTraveled: 4
  }));
  assert.ok(!flags.newBest && flags.newWorst, 'short loss must become worst');

  // A different terrain must not mix into expId 3 records.
  recordRun(store, run({ expId: 5, outcome: 'loss', completionTime: null, distanceTraveled: 0 }));
  const summary = summarizeForExperiment(store, 3);
  assert.equal(summary.total, 3);
  assert.equal(summary.best.controllerMode, 'qp_mpc_proj');
  assert.equal(summary.worst.distanceTraveled, 4);
});

test('records persist through storage round-trip regardless of session age', () => {
  const storage = fakeStorage();
  let store = loadRecords(storage);
  recordRun(store, run({ timestamp: new Date(Date.now()-2*365.25*24*3600*1000).toISOString() }));
  recordRun(store, run({ completionTime: 80 }));
  saveRecords(storage, store);

  const reloaded = loadRecords(storage);
  assert.equal(reloaded.entries.length, 2);
  const summary = summarizeForExperiment(reloaded, 10);
  assert.equal(summary.wins, 2);
  // The two-year-old slower run is ranked worst even though it was first.
  assert.ok(summary.worst.timestamp < summary.best.timestamp);
  assert.match(relativeAge(summary.worst.timestamp), /yr ago$/);
});

test('loadRecords survives corrupt or missing storage data', () => {
  const broken = { getItem: () => '{not json', setItem: () => {} };
  assert.deepEqual(loadRecords(broken).entries, []);
  assert.deepEqual(loadRecords(null).entries, []);
});

test('clearRecords empties the store and saving reflects it', () => {
  const storage = fakeStorage();
  const store = loadRecords(storage);
  recordRun(store, run());
  saveRecords(storage, store);
  clearRecords(store);
  saveRecords(storage, store);
  assert.equal(loadRecords(storage).entries.length, 0);
});

test('formatRecordCell renders outcome, metrics, model and age', () => {
  const cell = formatRecordCell(run({ timestamp: new Date().toISOString() }));
  assert.match(cell, /^WIN · 90\.0s · 0\.55 m\/s · Model B · Ric-LQR · just now$/);
  assert.equal(formatRecordCell(null), '—');
});

const snapshotPayload = overrides => ({
  terrainLevel: 14, gravity: 3.721, controllerShort: 'Natural', modelLabel: 'Model B',
  distance: 128.4, elapsed: 96.5, avgVelocity: 1.33,
  maxG: 2.4, maxTension: 512.5, deformation: 0.012,
  obstacles: 31, outcome: 'running', ...overrides
});

test('experiment snapshots keep the last metrics and accumulate all-time peaks', () => {
  const store = loadRecords(null);
  saveExperimentSnapshot(store, 14, snapshotPayload({ maxG: 2.4, avgVelocity: 1.33 }));
  saveExperimentSnapshot(store, 14, snapshotPayload({ maxG: 3.1, avgVelocity: 1.10 }));
  const snapshot = getExperimentSnapshot(store, 14);
  // Latest capture wins for run fields...
  assert.equal(snapshot.distance, 128.4);
  assert.equal(snapshot.avgVelocity, 1.10);
  assert.equal(snapshot.outcome, 'running');
  // ...while peaks accumulate across captures.
  assert.equal(snapshot.maxG, 3.1);
  assert.equal(snapshot.bestAvgVelocity, 1.33);
  assert.equal(snapshot.captures, 2);
});

test('snapshots survive a page refresh through the storage round-trip', () => {
  const storage = fakeStorage();
  const store = loadRecords(storage);
  saveExperimentSnapshot(store, 14, snapshotPayload());
  saveExperimentSnapshot(store, 3, snapshotPayload({ distance: 22 }));
  recordRun(store, run({ expId: 14 }));
  saveRecords(storage, store);

  // Simulated refresh: brand-new App reads the same storage.
  const reloaded = loadRecords(storage);
  assert.equal(reloaded.entries.length, 1, 'terminal runs still recorded');
  const snapshot14 = getExperimentSnapshot(reloaded, 14);
  assert.equal(snapshot14.distance, 128.4);
  assert.equal(snapshot14.gravity, 3.721);
  assert.match(formatSnapshotStatus(snapshot14), /^SAVED · 1 capture · just now$/);
  assert.equal(getExperimentSnapshot(reloaded, 3).distance, 22);
  assert.equal(getExperimentSnapshot(reloaded, 9), null);

  clearRecords(store);
  saveRecords(storage, store);
  assert.deepEqual(loadRecords(storage).snapshots, {}, 'clearing wipes snapshots too');
});
