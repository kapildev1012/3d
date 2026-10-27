import test from 'node:test';
import assert from 'node:assert/strict';
import { SimConfig } from '../js/simEngine.js?v=20261021-ws1';
import { configureLevel } from '../js/levels.js';
import { METHODS } from '../js/experimentProtocol.js';
import { Simulation, TerrainModel, SphericalRoverModel } from '../js/simEngine.js?v=20261021-ws1';
import { serializeState, deserializeState, encodeState, decodeState } from '../js/simSnapshot.js';

console.info = () => {};
const make = (level = 1) => {
  const cfg = new SimConfig({ seed: 3, enableDiagnosticsLog: false, monitoring: { enabled: false } });
  configureLevel(cfg, level); METHODS.sgtShield.configure(cfg); cfg.actuatorDelay = 0.1;
  return new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
};
const run = (sim, n) => { for (let i = 0; i < n; i++) sim.step(); return [...sim.q.flat(), ...sim.v.flat(), ...sim.corePosition, ...sim.coreVelocity, sim.t, sim.ledger.E, ...sim.currentActuationOffsets]; };
const maxDiff = (a, b) => a.reduce((m, x, i) => Math.max(m, Math.abs(x-b[i])), 0);

test('snapshot restore is bit-exact: in place, and into a fresh simulator through bytes', () => {
  const a = make();
  run(a, 700);                                  // 1.4 s: past the first SETTLE, actuator delay queue populated
  const snap = serializeState(a);
  const bytes = encodeState(snap);
  assert.ok(bytes.length < 200000, `checkpoint is ${bytes.length} bytes`);
  const reference = run(a, 150);
  deserializeState(a, snap);                    // roll back in place
  assert.equal(maxDiff(reference, run(a, 150)), 0);
  const b = make();
  deserializeState(b, decodeState(bytes));      // brand-new simulator (lazy SGT controller rebuilt)
  assert.equal(maxDiff(reference, run(b, 150)), 0);
});

test('snapshot carries the controller state machine, ledger and sensor RNG', () => {
  const a = make();
  run(a, 700);
  const snap = serializeState(a);
  assert.ok(snap.fields.sgt && snap.fields.sgt.$c === 'SensitivityTippingController');
  assert.ok('ledger' in snap.fields && 'commandQueue' in snap.fields);
  assert.equal(snap.t, a.t);
  run(a, 100);
  deserializeState(a, snap);
  assert.equal(a.t, snap.t);
  assert.equal(a.sgt.state, snap.fields.sgt.f.state);
});

test('crest-checkpoint tracker (obstacle course levels) is part of the snapshot', () => {
  const a = make(10);
  run(a, 30);
  const snap = serializeState(a);
  assert.ok(a.obstacleTracker, 'Level 10 has a crest tracker');
  assert.ok(snap.fields.obstacleTracker, 'tracker state serialized');
  const before = JSON.stringify(a.obstacleTracker.summary());
  run(a, 30);
  deserializeState(a, snap);
  assert.equal(JSON.stringify(a.obstacleTracker.summary()), before);
});
