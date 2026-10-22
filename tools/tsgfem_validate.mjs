#!/usr/bin/env node
/**
 * Cross-validate js/tsgfem.js against the original TsgFEM MATLAB functions.
 *
 *   node tools/tsgfem_validate.mjs          (needs GNU Octave ≥ 7 or MATLAB on PATH)
 *
 * 1. exports the rover (TsgFEM equilibrium shape + payload core, steel parts)
 *    to results/tsgfem/rover_model.json;
 * 2. runs tools/tsgfem_validate.m with third_party/TsgFEM/Function_library
 *    (tenseg_equilibrium_matrix1, tenseg_svd, tenseg_prestress_design,
 *    tenseg_minimass, tenseg_stiff_matx2, tenseg_mass_matrix, eig);
 * 3. compares with analyzeRover() from the JavaScript port and writes
 *    results/tsgfem/validation.json. Exit code 1 if any quantity disagrees.
 */
import { writeFileSync, readFileSync, mkdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { roverStructuralModel } from '../js/structuralAnalysis.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const out = path.join(root, 'results', 'tsgfem');
mkdirSync(out, { recursive: true });
console.info = () => {};

const { input, js } = roverStructuralModel({ material: 'steel' });
writeFileSync(path.join(out, 'rover_model.json'), JSON.stringify(input, null, 1));

const octave = spawnSync('octave', ['--no-gui', '--quiet', '--eval',
  `cd('${path.join(root, 'tools')}'); tsgfem_validate('${path.join(out, 'rover_model.json')}', '${path.join(out, 'tsgfem_matlab.json')}')`],
{ encoding: 'utf8', timeout: 300000 });
if (octave.error || octave.status !== 0) {
  console.error('[tsgfem] Octave run failed:', octave.error?.message || octave.stderr);
  process.exit(2);
}
const m = JSON.parse(readFileSync(path.join(out, 'tsgfem_matlab.json'), 'utf8'));

const rel = (a, b) => Math.abs(a-b)/Math.max(Math.abs(b), 1e-12);
const checks = [
  ['self-stress modes (grouped)', js.equilibrium.groupedSelfStressModes, m.selfStressModesGrouped, 0],
  ['equilibrium-matrix rank', js.equilibrium.rank, m.rank, 0],
  ['bar force [N]', js.prestress.barForce, m.barForce, 1e-6],
  ['cable force [N]', js.prestress.cableForce, m.cableForce, 1e-6],
  ['bar force density [N/m]', js.prestress.forceDensity.bar, m.qBar, 1e-6],
  ['min-mass bar radius [m]', js.minimalMass.barRadius, m.barRadius, 1e-5],
  ['min-mass cable radius [m]', js.minimalMass.cableRadius, m.cableRadius, 1e-6],
  ['min-mass structure [kg]', js.minimalMass.structureMass, m.structureMass, 1e-5],
  ...m.frequencies.slice(6, 16).map((f, k) => [`mode ${k+7} [Hz]`, js.modal.elasticHz[k], f, 1e-5]),
  ['highest mode [Hz]', js.modal.highestHz, m.frequencies[m.frequencies.length-1], 1e-5]
].map(([name, a, b, tol]) => ({ name, js: a, tsgfem: b, relError: rel(a, b), ok: tol === 0 ? a === b : rel(a, b) <= tol }));

writeFileSync(path.join(out, 'validation.json'), JSON.stringify({ generatedAt: new Date().toISOString(), checks }, null, 1));
for (const c of checks) console.log(`${c.ok ? 'ok  ' : 'FAIL'} ${c.name.padEnd(30)} js ${Number(c.js).toPrecision(8).padStart(14)}  TsgFEM ${Number(c.tsgfem).toPrecision(8).padStart(14)}  rel ${c.relError.toExponential(1)}`);
process.exit(checks.every(c => c.ok) ? 0 : 1);
