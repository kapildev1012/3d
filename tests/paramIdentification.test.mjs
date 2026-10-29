import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import {
  forwardTensegrity, columnMajor, planarProblem, runParticleFilter, residualResample, resampleIndices,
  makeRandom, FINAL_CODES_TRUE
} from '../js/paramIdentification.js';
import { roverBase, roverForward, roverProblem, ROVER_TRUE } from '../js/roverParamIdentification.js';

console.info = () => {};
const fixture = JSON.parse(readFileSync(new URL('./fixtures/final_codes_forward.json', import.meta.url), 'utf8'));
const hasOctave = spawnSync('octave', ['--version'], { encoding: 'utf8' }).status === 0;

test('forward model matches the original forward_tensegrity.m on 300 Octave cases', () => {
  let compared = 0;
  fixture.theta.forEach((theta, i) => {
    const js = forwardTensegrity(theta, fixture.force[i], { stability: 'original' });
    assert.equal(js.output, fixture.output[i], `case ${i}: output flag`);
    if (js.output !== 1) return;
    const y = columnMajor(js.positions);
    for (let d = 0; d < 18; d++) {
      assert.ok(Math.abs(y[d]-fixture.positions[i][d]) < 1e-9, `case ${i}, coordinate ${d}: ${y[d]} vs ${fixture.positions[i][d]}`);
    }
    compared++;
  });
  assert.ok(compared >= 290, `only ${compared} converged cases compared`);
});

test('a failed solve returns output 0 and NaN positions instead of throwing', () => {
  const i = fixture.output.findIndex(o => o === 0);
  assert.ok(i >= 0, 'fixture contains a failing case');
  const r = forwardTensegrity(fixture.theta[i], fixture.force[i]);
  assert.equal(r.output, 0);
  assert.ok(r.positions.flat().every(Number.isNaN));
});

test('corrected stability test rejects an equilibrium the original accepts with a negative eigenvalue', () => {
  // Found by tools/final_codes_validate.mjs --lo 0.2 --hi 2.5: eigs(K,1,'smallestabs') is positive,
  // but the reduced tangent stiffness also has a negative eigenvalue.
  const theta = [0.31767709464766086, 1.1185908939223737, 0.8839727503946051, 1.0901261320803315, 0.5198469322174788];
  const force = [-0.001842578230425716, 0.0017373413546010845, 0, 3];
  const original = forwardTensegrity(theta, force, { stability: 'original' });
  const corrected = forwardTensegrity(theta, force, { stability: 'positive-definite' });
  assert.equal(original.output, 1);
  assert.ok(original.solve.smallestEigenvalue > 0);
  assert.equal(corrected.output, 0);
  assert.equal(corrected.solve.reason, 'unstable equilibrium');
});

test('residual resampling keeps floor(N w) copies and returns N particles', () => {
  const rng = makeRandom(3);
  const w = [0.42, 0.31, 0.15, 0.08, 0.04];
  const counts = residualResample(w, rng);
  assert.equal(counts.reduce((s, v) => s+v, 0), 5);
  w.forEach((wi, i) => assert.ok(counts[i] >= Math.floor(5*wi)));
  assert.deepEqual(resampleIndices([2, 0, 1]), [0, 0, 2]);
});

test('particle filter of MainFile.m recovers the five planar parameters', { timeout: 120000 }, () => {
  const r = runParticleFilter(planarProblem({ seed: 46 }), { particles: 100 });
  assert.equal(r.forwardSolves, 10000);
  r.final.forEach((v, p) => assert.ok(Math.abs(v-FINAL_CODES_TRUE[p]) < 0.03*FINAL_CODES_TRUE[p],
    `parameter ${p+1}: ${v.toFixed(4)} vs ${FINAL_CODES_TRUE[p]}`));
  // strut rest lengths are identified to 0.5 %
  for (let p = 0; p < 3; p++) assert.ok(Math.abs(r.errorPct[p]) < 0.5);
});

test('rover static model: prestressed, statically determinate supports, cables taut under load', () => {
  const base = roverBase();
  assert.equal(base.fixed.flat().filter(Boolean).length, 6);
  const r = roverForward(ROVER_TRUE, [{ node: 5, f: [10, -5, 8] }], base);
  assert.ok(r.ok);
  base.elements.forEach((e, k) => {
    const f = r.solve.elementForces[k];
    if (e.kind === 'bar') assert.ok(f < 0, 'bars in compression'); else assert.ok(f > 0, 'cables in tension');
  });
});

test('annealed particle filter identifies the rover cable parameters', { timeout: 600000 }, () => {
  const problem = roverProblem({ steps: 80, seed: 99 });
  const r = runParticleFilter(problem, { particles: 100, ...problem.filterDefaults });
  r.errorPct.forEach((e, p) => assert.ok(Math.abs(e) < 2, `rover parameter ${p+1}: ${e.toFixed(2)} % error`));
});

test('JS port agrees with the original MATLAB code (Octave)', { skip: !hasOctave && 'GNU Octave not installed', timeout: 300000 }, () => {
  const run = spawnSync(process.execPath, [new URL('../tools/final_codes_validate.mjs', import.meta.url).pathname, '--cases', '40'], { encoding: 'utf8' });
  assert.equal(run.status, 0, run.stdout+run.stderr);
});
