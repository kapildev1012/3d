import test from 'node:test';
import assert from 'node:assert/strict';
import { trainSpec, freshInner, accumulateInner, innerSummary, isValidIncidence, TRAIN_LEVELS } from '../js/trainingLoop.js';
import { synergyMatrix } from '../js/residualMppi.js';
import { SimConfig, SphericalRoverModel } from '../js/simEngine.js?v=20261027-ob1';

console.info = () => {};

test('incidence check: the rover topology passes, the 0.15-filled placeholder is rejected', () => {
  assert.ok(isValidIncidence(synergyMatrix(new SphericalRoverModel(new SimConfig({})))));
  assert.ok(!isValidIncidence(new Float64Array(144).fill(0.15)));
  assert.ok(!isValidIncidence(new Float64Array(100)));
});

test('domain randomisation is deterministic and covers many levels and scenarios', () => {
  assert.deepEqual(trainSpec(3, 17), trainSpec(3, 17));
  const specs = Array.from({ length: 500 }, (_, e) => trainSpec(1, e));
  assert.ok(new Set(specs.map(s => s.level)).size >= TRAIN_LEVELS.length - 1);
  assert.ok(new Set(specs.map(s => s.scenario)).size >= 8);
});

test('inner-loop statistics count real and rollout physics steps and failure modes from measured digests', () => {
  const inner = freshInner();
  const digest = { progressM: 0.4, avgSpeedMs: 0.08, relaxedCablesPct: 2, rodResidualMm: 0.01, contactSlipMs: 0.1, maxOuterTensionN: 190, aborts: 1, tips: 3 };
  accumulateInner(inner, { level: 4, scenario: 'push' }, { digest, ticks: 100, meanEss: 5, meanAdvantage: 0.2, meanReturn: 1 }, { candidates: 8, durationS: 5 });
  const s = innerSummary(inner);
  assert.equal(s.realPhysicsSteps, 2500);
  assert.equal(s.rolloutPhysicsSteps, 17500);
  assert.equal(s.successRatePct, 100);                      // 0.4 m in 5 s >= 1 m per 20 s pace
  assert.equal(s.overloadEpisodes, 1);
  assert.equal(s.byLevel.L4.episodes, 1);
});
