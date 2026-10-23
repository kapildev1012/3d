#!/usr/bin/env node
/**
 * Frames for the paper's teaser figure: SGT + suspension filter on Level 1
 * (shell, payload, cable rest-length offsets, state) and the payload-cable
 * tension with and without the filter. Writes results/paper/teaser_frames.json.
 */
import { writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';
import { configureLevel } from '../js/levels.js';

console.info = () => {};
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
function run(shield, T, keepFrames) {
  const cfg = new SimConfig({}); configureLevel(cfg, 1); cfg.applyResearchProfile();
  if (shield) cfg.tensionShield = true;
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
  const frames = [], tension = [];
  let k = 0;
  while (sim.t < T) {
    sim.step(); k++;
    if (k % 25 === 0) {
      tension.push({ t: sim.t, mean: sim.coreCableForces.reduce((s, v) => s+v, 0)/12, slack: sim.coreCableForces.filter(v => v < 1).length });
      if (keepFrames) frames.push({
        t: sim.t, state: sim.sgt?.state, tips: sim.sgt?.tipCount,
        q: sim.q.map(p => p.map(v => +v.toFixed(4))), core: sim.corePosition.map(v => +v.toFixed(4)),
        cable: sim.currentActuationOffsets.map(v => +v.toFixed(4)), coreCable: sim.currentCoreActuationOffsets.map(v => +v.toFixed(4)),
        coreT: sim.coreCableForces.map(v => +v.toFixed(2)), face: sim.sgt?.lastPlan?.geometry?.face || null, edge: sim.sgt?.lastPlan?.geometry?.edge || null
      });
    }
  }
  return { frames, tension, bars: sim.rover.bars, outer: sim.rover.outerStrings };
}
const on = run(true, 20, true), off = run(false, 20, false);
writeFileSync(path.join(root, 'results', 'paper', 'teaser_frames.json'),
  JSON.stringify({ bars: on.bars, outer: on.outer, frames: on.frames, tensionOn: on.tension, tensionOff: off.tension }));
console.log('frames', on.frames.length, 'states', [...new Set(on.frames.map(f => f.state))].join(','), 'tips', on.frames.at(-1).tips);
