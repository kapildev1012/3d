import test from 'node:test';
import assert from 'node:assert/strict';
import {
  OBS_DIM, ACT_DIM, SYNERGY_DIM, SynergyActor, ValueNet, TwinCritic, mppiWeights, sampleNoise, makeRng, synergyMatrix,
  packParams, unpackParams, DEFAULT_PARAMS, stageReward, TEL, TEL_DIM, PARAM_IDX, synergyShare
} from '../js/residualMppi.js';
import { SimConfig, SphericalRoverModel } from '../js/simEngine.js?v=20261021-ws1';
import { SensitivityTippingController } from '../js/tippingController.js';

console.info = () => {};
const rover = new SphericalRoverModel(new SimConfig({}));
const M = synergyMatrix(rover);

test('synergy matrix: 6 struts, 4 cables at each end, rows sum to zero', () => {
  assert.equal(M.length, SYNERGY_DIM*ACT_DIM);
  for (let k = 0; k < SYNERGY_DIM; k++) {
    const row = Array.from(M.subarray(k*ACT_DIM, (k+1)*ACT_DIM));
    assert.equal(row.filter(v => v === 1).length, 4);
    assert.equal(row.filter(v => v === -1).length, 4);
    assert.equal(row.reduce((s, v) => s+v, 0), 0);
  }
});

test('parameter packing round-trips', () => {
  const view = packParams({ ...DEFAULT_PARAMS, qVelocityTrack: 7.5 });
  assert.equal(unpackParams(view).qVelocityTrack, 7.5);
  assert.equal(view[PARAM_IDX.maxResidualStrokeM], 0.030);
});

test('untrained actor outputs exactly zero (residual-RL safe start); outputs are bounded', () => {
  const actor = new SynergyActor(M, 3);
  const obs = new Float64Array(OBS_DIM).map((_, i) => Math.sin(i));
  assert.ok(actor.forward(obs, 0.03).every(v => v === 0));
  actor.w.fill(0.4);
  assert.ok(actor.forward(obs.map(v => v*5), 0.03).every(v => Math.abs(v) <= 0.03+1e-15));
});

test('actor backprop matches finite differences', () => {
  const actor = new SynergyActor(M, 5);
  const rng = makeRng(9);
  for (let i = 0; i < actor.w.length; i++) actor.w[i] += (rng()-0.5)*0.2;
  const obs = Array.from({ length: 2 }, () => Float64Array.from({ length: OBS_DIM }, () => rng()*2-1));
  const target = Array.from({ length: 2 }, () => Float64Array.from({ length: ACT_DIM }, () => (rng()-0.5)*0.04));
  const loss = () => {
    let l = 0;
    obs.forEach((o, b) => { const y = actor.forward(o, 0.03); for (let c = 0; c < ACT_DIM; c++) { const dy = y[c]/0.03-Math.max(-0.999, Math.min(0.999, target[b][c]/0.03)); l += 0.5*dy*dy; } });
    return l/(obs.length*ACT_DIM);
  };
  const base = Float64Array.from(actor.w);
  actor.opt.lr = 0;                                    // inspect the gradient, do not move
  actor.trainBatch(obs, target, 0.03);
  const grad = Float64Array.from(actor.grad).map(g => g/obs.length);
  const h = 1e-6;
  for (const i of [0, 700, 3100, 3140, 3200, 3300, 4800, 5060]) {
    actor.w[i] = base[i]+h; const up = loss();
    actor.w[i] = base[i]-h; const down = loss();
    actor.w[i] = base[i];
    const numeric = (up-down)/(2*h), analytic = grad[i]/ACT_DIM;
    assert.ok(Math.abs(numeric-analytic) <= 2e-6+1e-4*Math.abs(numeric), `w[${i}] numeric ${numeric} analytic ${analytic}`);
  }
});

test('critic backprop matches finite differences (LayerNorm + SiLU)', () => {
  const net = new ValueNet(4);
  const rng = makeRng(2);
  const obs = Array.from({ length: 3 }, () => Float64Array.from({ length: OBS_DIM }, () => rng()*2-1));
  const y = [0.7, -0.3, 1.2];
  const loss = () => obs.reduce((s, o, b) => s+0.5*(net.forward(o)-y[b])**2, 0)/obs.length;
  const base = Float64Array.from(net.w);
  net.opt.lr = 0;
  net.trainBatch(obs, y);
  const grad = Float64Array.from(net.grad).map(g => g/obs.length);
  const h = 1e-6;
  for (const i of [0, 100, 3071, 3072, 3100, 3136, 3150, net.w.length-1]) {
    net.w[i] = base[i]+h; const up = loss();
    net.w[i] = base[i]-h; const down = loss();
    net.w[i] = base[i];
    const numeric = (up-down)/(2*h);
    assert.ok(Math.abs(numeric-grad[i]) <= 1e-6+1e-4*Math.abs(numeric), `w[${i}] numeric ${numeric} analytic ${grad[i]}`);
  }
});

test('twin critic learns a simple regression and keeps a pessimistic target', () => {
  const critic = new TwinCritic(1);
  const rng = makeRng(8);
  const obs = Array.from({ length: 64 }, () => Float64Array.from({ length: OBS_DIM }, () => rng()*2-1));
  const y = obs.map(o => 0.5*o[0]-0.3*o[5]);
  const first = critic.train(obs, y);
  let last = first;
  for (let i = 0; i < 250; i++) last = critic.train(obs, y);
  assert.ok(last < 0.5*first, `loss ${first} -> ${last}`);
  assert.ok(critic.value(obs[0]) <= Math.max(critic.v1.forward(obs[0]), critic.v2.forward(obs[0])));
});

test('MPPI weights: normalized, monotone in return, ESS in [1, K]', () => {
  const { weights, ess } = mppiWeights([0, 1, 2, 3], 0.5);
  assert.ok(Math.abs(weights.reduce((s, v) => s+v, 0)-1) < 1e-12);
  assert.ok(weights[3] > weights[2] && weights[2] > weights[1] && weights[1] > weights[0]);
  assert.ok(ess >= 1 && ess <= 4);
  assert.ok(mppiWeights([5, 5, 5, 5], 0.5).ess > 3.99);       // equal returns: uniform
});

test('exploration noise: candidate 0 is the prior, bounded, correlated in time, lives in synergy + fine space', () => {
  const K = 6, H = 10, out = new Float64Array(K*H*ACT_DIM);
  sampleNoise(out, K, H, M, makeRng(4), { cap: 0.03 });
  assert.ok(out.subarray(0, H*ACT_DIM).every(v => v === 0));
  assert.ok(out.every(v => Math.abs(v) <= 0.03));
  const row = k => out.subarray(k*H*ACT_DIM, (k+1)*H*ACT_DIM);
  let corr = 0, norm = 0;
  for (let t = 1; t < H; t++) for (let c = 0; c < ACT_DIM; c++) { corr += row(1)[t*ACT_DIM+c]*row(1)[(t-1)*ACT_DIM+c]; norm += row(1)[t*ACT_DIM+c]**2; }
  assert.ok(corr/norm > 0.2, `lag-1 correlation ${corr/norm}`);
  assert.ok(synergyShare(row(1).subarray(0, ACT_DIM), M) > 0.15);
});

test('stage reward: faster is better, violations are penalized', () => {
  const p = packParams(DEFAULT_PARAMS);
  const tel = new Float64Array(TEL_DIM);
  tel[TEL.clearance] = 0.2;
  const slow = stageReward(tel, p);
  tel[TEL.vFwd] = 0.3;
  const fast = stageReward(tel, p);
  assert.ok(fast > slow);
  tel[TEL.over] = 1;
  assert.ok(stageReward(tel, p) < fast);
  tel[TEL.over] = 0; tel[TEL.slackPct] = 40;
  assert.ok(stageReward(tel, p) < fast);
});

test('residual hook: |du| <= 0.030 m per cable and the total stroke is clamped to 0.120 m', () => {
  const sim = { cfg: { actuationDeltaL: 0.15 }, t: 1, rover };
  const controller = Object.create(SensitivityTippingController.prototype);
  Object.assign(controller, { sim, opts: { relaxLimit: 0.35, residual: { maxResidualStrokeM: 0.03, maxTotalStrokeM: 0.12, act: () => new Array(24).fill(1) } }, enteredAt: 0, state: 'SGT_SHIFT', lastPlan: { margin: 0.1 } });
  const nominal = Array.from({ length: 24 }, (_, k) => (k % 3 === 0 ? 0.15 : k % 3 === 1 ? 0.10 : -0.05));
  const out = controller.applyResidual({ cableTargets: nominal, coreTargets: new Array(12).fill(0) }, [0, 1, 0], null);
  out.cableTargets.forEach((u, k) => {
    assert.ok(u <= 0.12+1e-12, `cable ${k}: ${u}`);
    assert.ok(u >= -0.35*0.15-1e-12);
    assert.ok(Math.abs(u-Math.min(nominal[k], 0.12)) <= 0.03+1e-12);
  });
});

test('command smoothing Kd: identity at 0, first-order lag otherwise', () => {
  const controller = Object.create(SensitivityTippingController.prototype);
  controller.opts = { gainKd: 0 };
  const a = { cableTargets: [0.1], coreTargets: [0] }, b = { cableTargets: [0.0], coreTargets: [0] };
  controller.smoothCommand(a);
  assert.deepEqual(controller.smoothCommand(b).cableTargets, [0]);
  controller.opts = { gainKd: 1 }; controller.previousCommand = null;
  controller.smoothCommand(a);
  assert.ok(Math.abs(controller.smoothCommand(b).cableTargets[0]-0.05) < 1e-12);
});

test('MPC + RL and MPC + RL + LLM are separate modes: only the LLM mode uses tuned parameters', async () => {
  const { residualCheckpoint, residualTunedParams, isResidualMode, isLlmMode } = await import('../js/residualDeploy.js');
  assert.ok(isResidualMode('residual_mppi') && isResidualMode('residual_mppi_llm') && !isResidualMode('sgt_tipping'));
  assert.ok(isLlmMode('residual_mppi_llm') && !isLlmMode('residual_mppi'));
  assert.deepEqual(residualTunedParams('residual_mppi'), {});
  assert.deepEqual(residualCheckpoint('residual_mppi').params, {});
  const { METHODS } = await import('../js/experimentProtocol.js');
  const { SimConfig: Cfg } = await import('../js/simEngine.js?v=20261021-ws1');
  const a = new Cfg({}), b = new Cfg({});
  METHODS.residualMppi.configure(a); METHODS.residualMppiLlm.configure(b);
  assert.equal(a.controllerMode, 'residual_mppi');
  assert.equal(b.controllerMode, 'residual_mppi_llm');
  assert.ok(b.tensionShield.outerTensionMax <= 150);
});

test('deployed MPC + RL controllers load actor-only checkpoints, run cable-only and use the fast gait timing', async () => {
  const { buildConfig } = await import('../js/experimentProtocol.js');
  const { Simulation: Sim, TerrainModel: Terrain, SphericalRoverModel: Rover } = await import('../js/simEngine.js?v=20261021-ws1');
  for (const method of ['residualMppi', 'residualMppiLlm']) {
    const cfg = buildConfig({ method, level: 1, seed: 1 });
    const sim = new Sim(cfg, new Rover(cfg), new Terrain(cfg), 'adaptive');
    for (let k = 0; k < 300; k++) sim.step();
    assert.ok(sim.sgt, `${method} built its controller`);
    assert.equal(sim.sgt.opts.releaseTime, 0.18);
    assert.equal(sim.ledger.work.assist, 0);
  }
});
