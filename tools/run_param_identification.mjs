#!/usr/bin/env node
/**
 * Particle-filter parameter identification ("Final Codes", js/paramIdentification.js)
 * from the command line.
 *
 *   node tools/run_param_identification.mjs [--problem planar|rover|both] [--steps N]
 *        [--particles N] [--seed S] [--noise-mm σ]
 *
 * planar: the 6-node model of final_codes/forward_tensegrity.m with the settings of
 *         MainFile.m (100 load cases, 100 particles, seed 46);
 * rover:  outer / payload cable stiffness and pre-stretch of the six-bar rover
 *         (80 load cases, 100 particles, annealed likelihood).
 * Writes results/param_id/<problem>.json and <problem>.svg (convergence chart).
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { planarProblem, runParticleFilter } from '../js/paramIdentification.js';
import { roverProblem } from '../js/roverParamIdentification.js';
import { convergenceSvg } from '../js/paramIdPlot.js';

console.info = () => {};
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const opt = (name, fallback) => { const i = args.indexOf(`--${name}`); return i >= 0 ? args[i+1] : fallback; };
const which = opt('problem', 'both');
const seed = Number(opt('seed', 46)), noiseMm = Number(opt('noise-mm', 0));
const out = path.join(root, 'results', 'param_id');
mkdirSync(out, { recursive: true });

const runs = [];
if (which === 'planar' || which === 'both') runs.push(['planar', planarProblem({ steps: Number(opt('steps', 100)), seed, sigmaObs: noiseMm }), 100]);
if (which === 'rover' || which === 'both') runs.push(['rover', roverProblem({ steps: Number(opt('steps', 80)), seed, sigmaObs: noiseMm*1e-3 }), 100]);

for (const [id, problem, defaultParticles] of runs) {
  const particles = Number(opt('particles', defaultParticles));
  process.stdout.write(`[param-id] ${problem.title}: ${problem.loads.length} load cases x ${particles} particles ...`);
  const r = runParticleFilter(problem, { particles, ...problem.filterDefaults });
  console.log(` ${(r.elapsedMs/1000).toFixed(1)} s, ${r.forwardSolves} solves (${r.failedSolves} failed)`);
  const rows = problem.params.map((p, i) => ({ parameter: p.label, true: problem.trueTheta[i], estimate: r.final[i], twoSigma: 2*r.std[i],
    errorPct: r.errorPct[i], trueWithin2Sigma: r.within2Sigma[i] }));
  for (const row of rows) {
    console.log(`  ${row.parameter.padEnd(28)} true ${row.true.toFixed(2)}  estimate ${row.estimate.toFixed(4)} ± ${row.twoSigma.toFixed(4)}  error ${row.errorPct.toFixed(2).padStart(6)} %`);
  }
  const settings = { seed, noiseMm, particles, steps: problem.loads.length, sigmaAssumed: problem.sigmaAssumed, filter: problem.filterDefaults };
  writeFileSync(path.join(out, `${id}.json`), JSON.stringify({ generatedAt: new Date().toISOString(), problem: problem.title, units: problem.units,
    settings, rows, summary: { ...r, mean: undefined, stdHistory: undefined }, history: { mean: r.mean, std: r.stdHistory, neff: r.neff } }, null, 1));
  writeFileSync(path.join(out, `${id}.svg`), convergenceSvg({ mean: r.mean, std: r.stdHistory, trueTheta: problem.trueTheta, lower: problem.lower,
    upper: problem.upper, labels: problem.params.map(p => p.label), title: `${problem.title}: estimate ± 2σ per load case (dashed: true value)` },
  { width: 760, height: 380, theme: 'light', legend: true }));
}
console.log(`[param-id] wrote ${path.relative(root, out)}/`);
