#!/usr/bin/env node
/**
 * Cross-validate the JavaScript port (js/paramIdentification.js) against the
 * ORIGINAL final_codes/original/forward_tensegrity.m.
 *
 *   node tools/final_codes_validate.mjs [--cases 300] [--write-fixture]
 *   (needs GNU Octave >= 7 or MATLAB on PATH)
 *
 * 1. draws random cases: parameters uniform in [--lo, --hi] = [0.5, 1.6] (wider than the
 *    filter's prior [0.7, 1.4], so some solves fail) and forces as in MainFile.m;
 * 2. runs tools/final_codes_validate.m on the original code;
 * 3. runs forwardTensegrity() with stability 'original' and compares the
 *    output flag (must match exactly) and the 18 nodal coordinates;
 *    Where both answers are exact equilibria but differ (far outside the prior a
 *    structure can have two stable shapes and rounding picks one), the case is
 *    counted as an alternate equilibrium, not as an error;
 * 4. counts the cases where the corrected stability test (positive-definite
 *    tangent stiffness) would decide differently.
 * Writes results/final_codes/validation.json; --write-fixture also stores the
 * Octave results in tests/fixtures/final_codes_forward.json for the unit tests.
 */
import { writeFileSync, readFileSync, mkdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { forwardTensegrity, columnMajor, makeRandom, planarModel } from '../js/paramIdentification.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const opt = (name, fallback) => { const i = args.indexOf(`--${name}`); return i >= 0 ? args[i+1] : fallback; };
const n = Number(opt('cases', 300)), lo = Number(opt('lo', 0.5)), hi = Number(opt('hi', 1.6));
const out = path.join(root, 'results', 'final_codes');
mkdirSync(out, { recursive: true });

const rng = makeRandom(2026);
const theta = [], force = [];
for (let i = 0; i < n; i++) {
  theta.push(Array.from({ length: 5 }, () => lo+(hi-lo)*rng.rand()));
  force.push([-0.01+0.02*rng.rand(), -0.01+0.02*rng.rand(), 0, 2+rng.randi(4)]);
}
const casesFile = path.join(out, 'cases.json'), octaveFile = path.join(out, 'octave_forward.json');
writeFileSync(casesFile, JSON.stringify({ theta, force }));

const t0 = Date.now();
const octave = spawnSync('octave', ['--no-gui', '--quiet', '--eval',
  `cd('${path.join(root, 'tools')}'); final_codes_validate('${casesFile}', '${octaveFile}')`],
{ encoding: 'utf8', timeout: 1800000 });
if (octave.error || octave.status !== 0) {
  console.error('[final_codes] Octave run failed:', octave.error?.message || octave.stderr);
  process.exit(2);
}
const octaveMs = Date.now()-t0;
const m = JSON.parse(readFileSync(octaveFile, 'utf8'));

// largest out-of-balance force [N] over the free DOFs of positions P (column-major 18-vector)
const imbalance = (th, F, P) => {
  const model = planarModel(th), pos = [0, 1, 2, 3, 4, 5].map(k => [P[k], P[6+k], P[12+k]]);
  const f = pos.map(() => [0, 0, 0]);
  for (const e of model.elements) {
    const d = [0, 1, 2].map(c => pos[e.j][c]-pos[e.i][c]), L = Math.hypot(...d), t = e.k*(L-e.L);
    for (let c = 0; c < 3; c++) { f[e.i][c] += t*d[c]/L; f[e.j][c] -= t*d[c]/L; }
  }
  f[F[3]-1][0] += F[0]; f[F[3]-1][1] += F[1]; f[F[3]-1][2] += F[2];
  let max = 0;
  f.forEach((v, n) => v.forEach((x, c) => { if (!model.fixed[n][c]) max = Math.max(max, Math.abs(x)); }));
  return max;
};

let flagMismatch = 0, maxErr = 0, pdDisagree = 0, jsMs = 0, alternate = 0;
const rows = theta.map((th, i) => {
  const t = Date.now();
  const js = forwardTensegrity(th, force[i], { stability: 'original' });
  jsMs += Date.now()-t;
  const pd = forwardTensegrity(th, force[i], { stability: 'positive-definite' });
  if (pd.output !== js.output) pdDisagree++;
  if (js.output !== m.output[i]) flagMismatch++;
  let err = 0;
  if (js.output === 1 && m.output[i] === 1) {
    const y = columnMajor(js.positions);
    for (let d = 0; d < 18; d++) err = Math.max(err, Math.abs(y[d]-m.positions[i][d]));
    // a different answer that is still an exact equilibrium = another branch of a multi-stable case
    if (err > 1e-9 && imbalance(th, force[i], m.positions[i]) < 1e-8 && imbalance(th, force[i], y) < 1e-8) {
      alternate++;
      return { theta: th, force: force[i], octave: 1, js: 1, positiveDefinite: pd.output, maxAbsErrMm: err, alternateEquilibrium: true };
    }
    maxErr = Math.max(maxErr, err);
  }
  return { theta: th, force: force[i], octave: m.output[i], js: js.output, positiveDefinite: pd.output, maxAbsErrMm: err };
});

const summary = {
  generatedAt: new Date().toISOString(), cases: n,
  converged: m.output.filter(v => v === 1).length,
  outputFlagMismatches: flagMismatch,
  maxAbsPositionErrorMm: maxErr,
  alternateEquilibria: alternate,
  positiveDefiniteDisagreements: pdDisagree,
  octaveSecondsPerSolve: m.seconds.reduce((s, v) => s+v, 0)/n,
  jsMillisecondsPerSolve: jsMs/n,
  octaveWallSeconds: octaveMs/1000
};
writeFileSync(path.join(out, 'validation.json'), JSON.stringify({ summary, rows }, null, 1));
if (args.includes('--write-fixture')) {
  writeFileSync(path.join(root, 'tests', 'fixtures', 'final_codes_forward.json'),
    JSON.stringify({ source: 'final_codes/original/forward_tensegrity.m run in GNU Octave by tools/final_codes_validate.mjs',
      generatedAt: summary.generatedAt, theta, force, output: m.output, positions: m.positions }));
}
console.log(summary);
const ok = flagMismatch === 0 && maxErr < 1e-9;
console.log(ok ? '[final_codes] JS port matches the original' : '[final_codes] MISMATCH');
process.exit(ok ? 0 : 1);
