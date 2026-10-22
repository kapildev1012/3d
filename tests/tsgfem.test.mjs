/**
 * Tests for the TsgFEM port (js/tsgfem.js) and the rover structural model.
 * If GNU Octave is installed, also cross-checks against the original MATLAB
 * toolbox in third_party/TsgFEM (tools/tsgfem_validate.mjs).
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import * as T from '../js/tsgfem.js';
import { roverStructuralModel } from '../js/structuralAnalysis.js';

console.info = () => {};
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

// TsgFEM's lander (examples/lander_statics/main_lander.m): expanded octahedron, L = 1
function lander() {
  const L = 1;
  const N = [
    [-L/4, 0, L/2], [L/4, 0, L/2], [-L/4, 0, -L/2], [L/4, 0, -L/2],
    [L/2, L/4, 0], [L/2, -L/4, 0], [-L/2, L/4, 0], [-L/2, -L/4, 0],
    [0, L/2, L/4], [0, L/2, -L/4], [0, -L/2, L/4], [0, -L/2, -L/4]
  ];
  const bars = [[0, 2], [1, 3], [4, 6], [5, 7], [8, 10], [9, 11]];
  // every node joins the 4 nearest non-bar nodes (edge length sqrt(3/8)·L... same for all)
  const strings = [];
  for (let i = 0; i < 12; i++) {
    for (let j = i+1; j < 12; j++) {
      const d = Math.hypot(...[0, 1, 2].map(a => N[j][a]-N[i][a]));
      if (Math.abs(d-Math.sqrt(3/8)) < 1e-9) strings.push([i, j]);
    }
  }
  return { N, bars, strings };
}

test('lander: 24 strings, one grouped self-stress mode with q_bar/q_string = -1.5', () => {
  const { N, bars, strings } = lander();
  assert.equal(strings.length, 24);
  const pairs = [...bars, ...strings];
  const Gp = T.groupMatrix([bars.map((_, k) => k), strings.map((_, k) => 6+k)], pairs.length);
  const eq = T.equilibriumMatrix(N, pairs, { Gp });
  const sv = T.svdNullspaces(eq.Ag);
  assert.equal(sv.selfStressModes, 1);
  const [qb, qs] = sv.V2[0];
  assert.ok(Math.abs(qb/qs+1.5) < 1e-9, `ratio ${qb/qs}`);
  const pre = T.prestressDesign(eq, sv.V2, [{ group: 1, force: 40 }]);
  assert.ok(pre.residual < 1e-9);
  assert.ok(pre.t[0] < 0 && pre.t[6] > 0, 'bars in compression, strings in tension');
});

test('rover: modelled icosahedron (d/h = 1/φ) is not prestressable, form-found shape (d/h = 0.5) is', () => {
  const { js } = roverStructuralModel({ material: 'steel' });
  assert.ok(Math.abs(js.formFinding.modelledRatio-0.618034) < 1e-3);
  assert.equal(js.formFinding.modelledSelfStressModes, 0);
  assert.equal(js.formFinding.equilibriumSelfStressModes, 1);
  assert.equal(js.formFinding.usedEquilibriumShape, true);
  assert.ok(js.prestress.residual < 1e-8);
  assert.ok(js.prestress.barForce < 0);
  assert.ok(Math.abs(js.prestress.cableForce-40) < 1e-9);
});

test('rover: installed steel parts are far from failure; modal frequencies are physical', () => {
  const { js } = roverStructuralModel({ material: 'steel' });
  assert.ok(js.installed.bucklingSafety > 10);
  assert.ok(js.installed.cableSafety > 10);
  assert.ok(js.installed.coreCableSafety === null || Number.isFinite(js.installed.coreCableSafety));
  assert.equal(js.modal.rigidModes.length, 6);
  for (const f of js.modal.rigidModes) assert.ok(f < 1e-2*js.modal.firstElasticHz, `rigid mode ${f} Hz`);
  assert.ok(js.modal.firstElasticHz > 1 && js.modal.firstElasticHz < 50);
  assert.ok(js.modal.highestHz > 100*js.modal.firstElasticHz, 'bar axial modes far above shell modes');
  assert.ok(Number.isFinite(js.minimalMass.structureMass) && js.minimalMass.structureMass > 0);
});

test('tangent stiffness is symmetric and mass matrix matches TsgFEM consistent form', () => {
  const { js } = roverStructuralModel({ material: 'steel' });
  const f = js.fem;
  const K = T.tangentStiffness(f.nodes, f.pairs, f.E, f.A, f.l0);
  for (let i = 0; i < K.length; i++) for (let j = 0; j < i; j++) assert.ok(Math.abs(K[i][j]-K[j][i]) < 1e-6*Math.abs(K[i][i]+1));
  const M = T.massMatrix(2, [[0, 1]], [6]);
  assert.equal(M[0][0], 2); assert.equal(M[0][3], 1);
});

const hasOctave = spawnSync('octave', ['--version'], { encoding: 'utf8' }).status === 0;
test('JS port agrees with the original TsgFEM (Octave)', { skip: !hasOctave && 'GNU Octave not installed', timeout: 300000 }, () => {
  const run = spawnSync(process.execPath, [path.join(root, 'tools', 'tsgfem_validate.mjs')], { encoding: 'utf8', timeout: 300000 });
  assert.equal(run.status, 0, run.stdout+run.stderr);
});
