import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, readFileSync, writeFileSync, chmodSync, mkdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import {
  mergeStates, normalizeState, compareRuns, bestRun, worstRun, summarizeLevel, migrateLegacyRecords,
  isMeaningfulRun, toCSV, displayStatus
} from '../js/benchmarkCore.js';
import { createFileStore } from '../lib/benchmarkFileStore.mjs';
import { RunRecorder } from '../js/runRecorder.js';
import { configureLevel, LEVELS } from '../js/levels.js';
import { SimConfig, SphericalRoverModel, TerrainModel, Simulation } from '../js/simEngine.js';

const run = (id, extra = {}) => ({
  id, level: 1, status: 'terminated', startedAt: '2026-10-01T10:00:00.000Z', updatedAt: '2026-10-01T10:00:05.000Z',
  simTime: 5, B: { distance: 1, time: 5, avgSpeed: 0.2, maxTension: 50, maxG: 1.2 }, ...extra
});

test('merge is a union of run ids and keeps the newest copy of each run', () => {
  const a = { runs: [run('r1'), run('r2', { updatedAt: '2026-10-01T10:00:09.000Z', simTime: 9 })] };
  const b = { runs: [run('r2', { simTime: 3 }), run('r3', { level: 14 })] };
  const merged = mergeStates(a, b);
  assert.deepEqual(merged.runs.map(r => r.id).sort(), ['r1', 'r2', 'r3']);
  assert.equal(merged.runs.find(r => r.id === 'r2').simTime, 9);
  // commutative and idempotent
  assert.deepEqual(mergeStates(b, a).runs.map(r => r.id).sort(), ['r1', 'r2', 'r3']);
  assert.equal(mergeStates(merged, merged).runs.length, 3);
});

test('a final record beats a checkpoint with the same timestamp', () => {
  const checkpoint = run('r1', { status: 'active', final: false });
  const final = run('r1', { status: 'completed', final: true });
  assert.equal(mergeStates({ runs: [checkpoint] }, { runs: [final] }).runs[0].status, 'completed');
  assert.equal(mergeStates({ runs: [final] }, { runs: [checkpoint] }).runs[0].status, 'completed');
});

test('clear tombstone drops older runs from every layer it meets', () => {
  const cleared = { clearedAt: '2026-10-02T00:00:00.000Z', runs: [] };
  const old = { runs: [run('old')] };
  const fresh = { runs: [run('new', { updatedAt: '2026-10-03T00:00:00.000Z' })] };
  const merged = mergeStates(old, cleared, fresh);
  assert.deepEqual(merged.runs.map(r => r.id), ['new']);
  assert.equal(merged.clearedAt, '2026-10-02T00:00:00.000Z');
});

test('invalid records are rejected, levels are 1..14', () => {
  const state = normalizeState({ runs: [run('ok'), run('bad', { level: 15 }), { id: 'x' }, null, run('', {})] });
  assert.deepEqual(state.runs.map(r => r.id), ['ok']);
});

test('ranking: completed > distance > speed > faster time > lower tension/G', () => {
  const done = run('done', { status: 'completed', B: { distance: 1, avgSpeed: 0.1, completionTime: 30 } });
  const far = run('far', { B: { distance: 40, avgSpeed: 1 } });
  assert.ok(compareRuns(done, far) > 0);
  const fast = run('fast', { status: 'completed', B: { distance: 50, avgSpeed: 1.3, completionTime: 38 } });
  const slow = run('slow', { status: 'completed', B: { distance: 50, avgSpeed: 1.3, completionTime: 41 } });
  assert.ok(compareRuns(fast, slow) > 0);
  const gentle = run('gentle', { B: { distance: 2, avgSpeed: 0.2, maxTension: 40 } });
  const harsh = run('harsh', { B: { distance: 2, avgSpeed: 0.2, maxTension: 90 } });
  assert.equal(bestRun([harsh, gentle]).id, 'gentle');
});

test('worst run ignores runs of 1 s or less and live runs', () => {
  const short = run('short', { simTime: 0.8, B: { distance: 0 } });
  const live = run('live', { status: 'active', B: { distance: 0 } });
  const weak = run('weak', { simTime: 10, B: { distance: 0.3 } });
  const good = run('good', { simTime: 10, B: { distance: 3 } });
  assert.equal(worstRun([short, live, weak, good]).id, 'weak');
});

test('level summary: count, latest, best, worst; display status labels', () => {
  const state = mergeStates({ runs: [run('a'), run('b', { updatedAt: '2026-10-05T00:00:00.000Z', status: 'failed', B: { distance: 0.5 } }), run('c', { level: 2 })] });
  const summary = summarizeLevel(state, 1);
  assert.equal(summary.count, 2);
  assert.equal(summary.latest.id, 'b');
  assert.equal(summary.best.id, 'a');
  assert.equal(displayStatus(summary.latest), 'FAILED');
  assert.equal(displayStatus(null), 'NOT RUN');
  assert.equal(displayStatus(run('t')), 'INCOMPLETE');
});

test('minimum run threshold filters accidental double resets', () => {
  assert.equal(isMeaningfulRun(run('z', { simTime: 0.1, B: { distance: 0.001 } })), false);
  assert.equal(isMeaningfulRun(run('t', { simTime: 0.6, B: { distance: 0 } })), true);
  assert.equal(isMeaningfulRun(run('d', { simTime: 0.1, B: { distance: 0.05 } })), true);
});

test('v1 records migrate deterministically (no duplicates on re-run)', () => {
  const legacy = {
    entries: [{ expId: 10, model: 'B', outcome: 'win', completionTime: 38.2, distanceTraveled: 50, avgVelocity: 1.31, timestamp: '2025-01-01T00:00:00Z' }],
    snapshots: { 3: { expId: 3, distance: 2, elapsed: 20, avgVelocity: 0.1, updatedAt: '2025-02-01T00:00:00Z' } }
  };
  const once = migrateLegacyRecords(legacy);
  assert.equal(once.length, 2);
  assert.equal(once[0].status, 'completed');
  const twice = mergeStates({ runs: once }, { runs: migrateLegacyRecords(legacy) });
  assert.equal(twice.runs.length, 2);
});

test('CSV export has a header and one line per run', () => {
  const csv = toCSV(mergeStates({ runs: [run('a'), run('b')] }));
  const lines = csv.trim().split('\n');
  assert.equal(lines.length, 3);
  assert.match(lines[0], /^id,level,source,status/);
});

test('disk store: atomic merge, concurrent writers, clear tombstone', async () => {
  const dir = mkdtempSync(path.join(tmpdir(), 'bench-'));
  const file = path.join(dir, 'data', 'benchmarks.json');
  const store = createFileStore(file);
  await Promise.all([
    store.mergeAndWrite({ runs: [run('a')] }),
    store.mergeAndWrite({ runs: [run('b')] }),
    store.mergeAndWrite({ runs: [run('c')] })
  ]);
  const onDisk = JSON.parse(readFileSync(file, 'utf8'));
  assert.deepEqual(onDisk.runs.map(r => r.id).sort(), ['a', 'b', 'c']);
  const cleared = await store.clear();
  assert.equal(cleared.state.runs.length, 0);
  const after = await store.mergeAndWrite({ runs: [run('a')] }); // stale browser copy
  assert.equal(after.state.runs.length, 0, 'tombstone wins over stale copies');
});

test('disk store: corrupt file is quarantined, read-only disk reports persisted:false', async () => {
  const dir = mkdtempSync(path.join(tmpdir(), 'bench-'));
  const file = path.join(dir, 'benchmarks.json');
  writeFileSync(file, '{ not json');
  const store = createFileStore(file);
  assert.equal((await store.readState()).runs.length, 0);
  if (process.getuid?.() === 0) return; // root ignores permissions
  const roDir = path.join(dir, 'ro');
  mkdirSync(roDir);
  chmodSync(roDir, 0o500);
  const ro = createFileStore(path.join(roDir, 'benchmarks.json'));
  const result = await ro.mergeAndWrite({ runs: [run('a')] });
  assert.equal(result.persisted, false);
});

test('recorder: checkpoints each second, commits on reset, skips empty runs', () => {
  const quiet = console.info; console.info = () => {};
  const cfg = configureLevel(new SimConfig({ enableDiagnosticsLog: false }), 1);
  const rover = new SphericalRoverModel(cfg);
  const terrain = new TerrainModel(cfg);
  const app = {
    cfg, simA: new Simulation(cfg, rover, terrain, 'fixed'), simB: new Simulation(cfg, rover, terrain, 'adaptive'),
    obstacleCount: () => 0, constructor: { controllerShortLabel: () => 'Natural' }
  };
  const saved = [];
  const recorder = new RunRecorder(app, { upsertRun: (r, o) => { saved.push({ ...r, flush: o.flush }); return true; } });
  const stepFor = seconds => {
    for (let k = 0; k < Math.round(seconds/cfg.dt); k++) { app.simA.step(); app.simB.step(); if (k % 25 === 0) recorder.tick(); }
    recorder.tick();
  };
  stepFor(2.2);
  assert.ok(saved.filter(r => r.status === 'active').length >= 1, 'checkpoint while running');
  const id = recorder.activeRun.id;
  recorder.commit('reset');
  const final = saved.filter(r => r.id === id).pop();
  assert.equal(final.status, 'terminated');
  assert.equal(final.reason, 'reset');
  assert.ok(final.final && final.flush);
  assert.ok(final.A && final.B && final.simTime > 2);
  // Immediately resetting again creates no empty record
  app.simA.reset(); app.simB.reset();
  const before = saved.length;
  recorder.tick();
  recorder.commit('reset');
  assert.equal(saved.length, before);
  console.info = quiet;
  assert.equal(LEVELS.length, 14);
});

test('Level 6 slope is 18 degrees', () => {
  const quiet = console.info; console.info = () => {};
  const cfg = configureLevel(new SimConfig({ enableDiagnosticsLog: false }), 6);
  const terrain = new TerrainModel(cfg);
  console.info = quiet;
  assert.ok(Math.abs(Math.atan(terrain.slopes[0].incline)*180/Math.PI-18) < 1e-9);
});
