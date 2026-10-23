#!/usr/bin/env node
/**
 * Shell sag and payload-cable pretension loss under the SGT gait (Level 1,
 * cable-only, 20 s, no filter). Writes results/shield_study/sag.json.
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';
import { configureLevel } from '../js/levels.js';

console.info = () => {};
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const cfg = new SimConfig({}); configureLevel(cfg, 1); cfg.applyResearchProfile();
const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
const height = () => sim.q.reduce((s, p) => s+p[2]-sim.terrain.eval(p[0], p[1]).h, 0)/sim.q.length;
sim.step();
const h0 = height(), t0 = sim.coreCableForces.reduce((s, v) => s+v, 0)/12;
let hs = 0, ts = 0, n = 0;
while (sim.t < 20) { sim.step(); if (sim.t >= 1) { hs += height(); ts += sim.coreCableForces.reduce((s, v) => s+v, 0)/12; n++; } }
const out = { initialHeight: h0, meanHeight: hs/n, sagPercent: 100*(1-hs/n/h0), initialCoreTension: t0, meanCoreTension: ts/n, pretensionDesign: cfg.pretensionCore };
mkdirSync(path.join(root, 'results', 'shield_study'), { recursive: true });
writeFileSync(path.join(root, 'results', 'shield_study', 'sag.json'), JSON.stringify(out, null, 1));
console.log(out);
