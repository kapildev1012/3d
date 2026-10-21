#!/usr/bin/env node
/**
 * Time-step convergence of the per-step energy drift (seed 1, Level 1, 20 s).
 * The work ledger measures, every step and BEFORE the corrector, the energy
 * change that no booked work explains. If this is discretization error it
 * must shrink with dt; an unbooked mechanism would not.
 * Writes results/paper/drift_check.json (read by tools/make_numbers.mjs).
 */
import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { buildConfig } from '../js/experimentProtocol.js';
import { Simulation, SphericalRoverModel, TerrainModel } from '../js/simEngine.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
console.info = () => {};
const cases = [
  { key: 'dae2', method: 'sgt', ablation: null, dt: 0.002 },
  { key: 'dae1', method: 'sgt', ablation: null, dt: 0.001 },
  { key: 'legacy2', method: 'sgt', ablation: 'legacyCore', dt: 0.002 },
  { key: 'legacy1', method: 'sgt', ablation: 'legacyCore', dt: 0.001 }
];
const only = process.argv[2];
const out = {};
for (const c of cases) {
  if (only && !only.split(',').includes(c.key)) continue;
  const cfg = buildConfig({ method: c.method, seed: 1, ablation: c.ablation });
  cfg.dt = c.dt;
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
  const N = Math.round(20/cfg.dt);
  for (let k = 0; k < N; k++) sim.step();
  const L = sim.ledger;
  out[c.key] = { dt: c.dt, driftAbs: L.driftAbs, driftSigned: L.driftSigned, maxStepDrift: L.maxStepDrift,
    driftRateW: L.driftAbs/20, cableWork: L.work.cables+L.work.core, progress: sim.currentDiag.centroid[1] };
  console.log(c.key, JSON.stringify(out[c.key]));
}
const file = path.join(root, 'results', 'paper', only ? `drift_check_${only.replace(/,/g, '_')}.json` : 'drift_check.json');
writeFileSync(file, JSON.stringify(out, null, 2));
console.log('[drift] wrote', file);
