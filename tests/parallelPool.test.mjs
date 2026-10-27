import test from 'node:test';
import assert from 'node:assert/strict';
import { SharedVecTensegrityEnv, CMD } from '../js/parallelPool.js';
import { OBS_DIM, ACT_DIM, TEL, sampleNoise, makeRng, synergyMatrix } from '../js/residualMppi.js';
import { SimConfig, SphericalRoverModel } from '../js/simEngine.js?v=20261027-ob1';

console.info = () => {};
const M = synergyMatrix(new SphericalRoverModel(new SimConfig({})));

test('shared-memory pool: zero-copy views, branching rollouts reproduce the real trajectory bit-exactly', async () => {
  const pool = new SharedVecTensegrityEnv({ numEnvs: 2, numCandidates: 2 });
  try {
    await pool.start();
    assert.ok(pool.v.obs.buffer instanceof SharedArrayBuffer && pool.v.ctrl.buffer instanceof SharedArrayBuffer);
    await pool.resetAll(() => ({ level: 1, seed: 5, scenario: 'nominal' }));
    // Advance until the SGT state machine is shifting/tipping (the residual hook only acts there).
    for (let k = 0; k < 120 && !(k > 30 && pool.v.decFlag[0]); k++) await pool.broadcast(CMD.STEP, [0]);
    assert.equal(pool.v.decFlag[0], 1, 'residual hook active at the checkpoint')
    assert.ok(pool.getEnvTelemetry(0)[TEL.vFwd] !== undefined);
    await pool.broadcast(CMD.PUBLISH, [0]);
    assert.ok(pool.v.ckpt[1] > 1000 && pool.v.ckpt[1] < 200000, `checkpoint ${pool.v.ckpt[1]} B`);
    const H = 4;
    pool.v.noise.fill(0);
    sampleNoise(pool.v.noise, 2, pool.maxHorizon, M, makeRng(2), { cap: 0.03 });      // cand 0 = zero noise, cand 1 = noisy
    for (const cand of [0, 1]) await pool.broadcast(CMD.ROLLOUT, [1], () => [cand, H, 0, 0]);
    const term = c => Float64Array.from(pool.v.termObs.subarray(c*OBS_DIM, (c+1)*OBS_DIM));
    const noisyTerm = term(1);
    // the real env continues with zero noise for H ticks: must equal candidate 0 exactly
    const realBefore = Float64Array.from(pool.getEnvObs(0));
    for (let k = 0; k < H; k++) await pool.broadcast(CMD.STEP, [0]);
    const real = pool.getEnvObs(0), zero = term(0);
    for (let i = 0; i < OBS_DIM; i++) assert.equal(real[i], zero[i], `obs[${i}]`);
    assert.notDeepEqual(Array.from(noisyTerm), Array.from(zero));                      // noise changes the outcome
    assert.notDeepEqual(Array.from(realBefore), Array.from(real));
    assert.ok(Number.isFinite(pool.v.ret[0]) && pool.v.candInfo[0] === H);
  } finally { await pool.terminate(); }
});

test('worker errors are reported to the coordinator instead of hanging', async () => {
  const pool = new SharedVecTensegrityEnv({ numEnvs: 2, numCandidates: 1 });
  try {
    await pool.start();
    await assert.rejects(() => pool.broadcast(77, [1]), /unknown command 77/);
    // the worker survives and still answers
    await pool.reset(1, { level: 1, seed: 1, scenario: 'nominal' });
  } finally { await pool.terminate(); }
});

test('episode digest: all Comparative-benchmark metrics are produced', async () => {
  const pool = new SharedVecTensegrityEnv({ numEnvs: 2, numCandidates: 1 });
  try {
    await pool.start();
    await pool.reset(0, { level: 1, seed: 2, scenario: 'push' });
    await pool.broadcast(CMD.EPISODE, [0], () => [0, 0, 2000, 0]);
    const d = pool.getDigest(0);
    for (const key of ['avgSpeedMs', 'rmseM', 'maxErrorM', 'controlEffortUtu', 'solveTimeMs', 'successRatePct', 'rodResidualMm', 'relaxedCablesPct', 'contactSlipMs', 'energyDriftJ']) {
      assert.ok(Number.isFinite(d[key]), `${key} = ${d[key]}`);
    }
    assert.ok(d.rodResidualMm < 0.05, `rod residual ${d.rodResidualMm} mm`);
  } finally { await pool.terminate(); }
});
