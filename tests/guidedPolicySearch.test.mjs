/**
 * MPC-guided policy search: teacher QP, MLP regression, policy wiring and a
 * closed-loop check of the shipped (trained) policy.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { solveTeacherQp, Mlp, GuidedPolicy, OUTER_FEATURES, CORE_FEATURES } from '../js/guidedPolicySearch.js';
import { GUIDED_POLICY_WEIGHTS } from '../js/guidedPolicyWeights.js';
import { METHODS } from '../js/experimentProtocol.js';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';
import { configureLevel } from '../js/levels.js';

console.info = () => {};

test('teacher QP reduces to the SGT command (r = 1, no rate/smoothing/policy terms)', () => {
  const g = [0.9, -0.5, 0.2, -1, 1, 0.05], lo = [-0.35, -0.35, -0.35, -1, -1, -0.35];
  const u = solveTeacherQp({ g, u0: g.map(() => 0), lo, rate: g.map(() => 10) }, { horizon: 1, rDelta: 0 });
  u.forEach((v, k) => assert.ok(Math.abs(v-Math.max(lo[k], Math.min(1, g[k]))) < 1e-6));
});

test('teacher QP respects box and spool-rate limits and is pulled toward the policy by λ', () => {
  const g = [1, -1], lo = [-0.35, -1], rate = [0.1, 0.1];
  const u = solveTeacherQp({ g, u0: [0, 0], lo, rate });
  assert.ok(Math.abs(u[0]) <= 0.1+1e-12 && Math.abs(u[1]) <= 0.1+1e-12);
  const free = solveTeacherQp({ g: [0.6], u0: [0.6], lo: [-1], rate: [1] }, { lambda: 0 });
  const pulled = solveTeacherQp({ g: [0.6], u0: [0.6], lo: [-1], rate: [1], piTarget: [-0.4] }, { lambda: 2 });
  assert.ok(pulled[0] < free[0]-0.2, `${pulled[0]} vs ${free[0]}`);
});

test('MLP regression fits a smooth target', () => {
  const X = [], Y = [];
  for (let i = 0; i < 600; i++) { const a = Math.sin(i), b = Math.cos(1.7*i); X.push([a, b]); Y.push([0.6*Math.tanh(2*a-b)]); }
  const net = new Mlp([2, 16, 1], 3);
  net.fitNormalization(X);
  const before = net.train(X, Y, { epochs: 1, lr: 0 });
  const after = net.train(X, Y, { epochs: 60, lr: 1e-2 });
  assert.ok(after < 0.1*before && after < 0.005, `mse ${before} -> ${after}`);
});

test('policy outputs respect actuator bounds and round-trip through JSON', () => {
  const policy = new GuidedPolicy(null, 5);
  const u = policy.normalizedCommand({ outer: new Array(24).fill(0).map((_, k) => new Array(OUTER_FEATURES).fill(k/24)),
    core: new Array(12).fill(0).map(() => new Array(CORE_FEATURES).fill(0.3)) }, 0.35);
  assert.equal(u.length, 36);
  u.slice(0, 24).forEach(v => assert.ok(v >= -0.35-1e-12 && v <= 1));
  const copy = new GuidedPolicy(JSON.parse(JSON.stringify(policy.toJSON())));
  const x = [0.1, ...new Array(OUTER_FEATURES-1).fill(0.2)];
  assert.ok(Math.abs(copy.outer.forward(x)[0]-policy.outer.forward(x)[0]) < 1e-4, 'weights are stored to 1e-6');
});

test('guidedPolicy is a benchmark method', () => assert.ok(METHODS.guidedPolicy));

test('shipped policy rolls forward cable-only without the KKT model', { skip: !GUIDED_POLICY_WEIGHTS && 'no trained weights', timeout: 300000 }, () => {
  const cfg = new SimConfig({}); configureLevel(cfg, 1); METHODS.guidedPolicy.configure(cfg);
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
  const y0 = sim.q.reduce((s, p) => s+p[1]/12, 0);
  while (sim.t < 12) sim.step();
  const y = sim.q.reduce((s, p) => s+p[1]/12, 0);
  assert.equal(sim.controlDiagnostics.mode, 'guided_policy');
  assert.ok(y-y0 > 0.5, `progress ${(y-y0).toFixed(3)} m`);
  assert.ok(sim.sgt.tipCount >= 2);
});
