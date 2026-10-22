#!/usr/bin/env node
/**
 * One cable-only run with payload-safety metrics (used by run_shield_study.mjs).
 *   node tools/shield_metrics.mjs <controllerMode> <on|off> [scenario] [level] [seconds] [seed]
 * Prints one JSON line. Metrics are taken after the first second (initial settling).
 */
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';
import { configureLevel } from '../js/levels.js';
import { SCENARIOS } from '../js/experimentProtocol.js';

console.info = () => {};
const [mode = 'sgt_tipping', shield = 'off', scenario = 'nominal', level = '1', T = '20', seed = '42'] = process.argv.slice(2);
const cfg = new SimConfig({});
configureLevel(cfg, Number(level));
cfg.applyResearchProfile();
cfg.controllerMode = mode;
cfg.seed = Number(seed);
SCENARIOS[scenario].apply(cfg);
if (shield === 'on') cfg.tensionShield = true;
const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
const impulse = SCENARIOS[scenario].impulse;

const segDist = (p, a, b) => {
  const ab = [0, 1, 2].map(k => b[k]-a[k]), ap = [0, 1, 2].map(k => p[k]-a[k]);
  const t = Math.max(0, Math.min(1, (ap[0]*ab[0]+ap[1]*ab[1]+ap[2]*ab[2])/(ab[0]**2+ab[1]**2+ab[2]**2)));
  return Math.hypot(...[0, 1, 2].map(k => ap[k]-t*ab[k]));
};
let minClear = Infinity, steps = 0, slackCables = 0, anySlack = 0, maxOuter = 0, maxCore = 0, impulseDone = false;
const rel = [], accel = [], coreT = [];
let prevV = null;
const y0 = sim.q.reduce((s, p) => s+p[1], 0)/12;
while (sim.t < Number(T)) {
  if (impulse && !impulseDone && sim.t >= impulse.time) {
    for (const v of sim.v) for (let k = 0; k < 3; k++) v[k] += impulse.dv[k];
    for (let k = 0; k < 3; k++) sim.coreVelocity[k] += impulse.dv[k];
    impulseDone = true;
  }
  sim.step();
  if (sim.t < 1) { prevV = sim.coreVelocity.slice(); continue; }
  steps++;
  const c = sim.corePosition;
  for (const [i, j] of sim.rover.bars) minClear = Math.min(minClear, segDist(c, sim.q[i], sim.q[j]));
  const ct = sim.coreCableForces;
  const ns = ct.filter(t => t < 1).length;
  slackCables += ns; if (ns) anySlack++;
  maxCore = Math.max(maxCore, ...ct);
  coreT.push(ct.reduce((s, t) => s+t, 0)/ct.length);
  maxOuter = Math.max(maxOuter, ...(sim.currentDiag?.outerCableForces || [0]));
  const cen = sim.q.reduce((s, p) => [s[0]+p[0]/12, s[1]+p[1]/12, s[2]+p[2]/12], [0, 0, 0]);
  rel.push([c[0]-cen[0], c[1]-cen[1], c[2]-cen[2]]);
  accel.push(Math.hypot(...[0, 1, 2].map(k => (sim.coreVelocity[k]-prevV[k])/cfg.dt))/9.81);
  prevV = sim.coreVelocity.slice();
}
const mean = rel.reduce((s, r) => [s[0]+r[0]/rel.length, s[1]+r[1]/rel.length, s[2]+r[2]/rel.length], [0, 0, 0]);
const rattle = Math.sqrt(rel.reduce((s, r) => s+(r[0]-mean[0])**2+(r[1]-mean[1])**2+(r[2]-mean[2])**2, 0)/rel.length);
accel.sort((a, b) => a-b);
const ctl = sim.controllerTiming;
console.log(JSON.stringify({
  mode, shield, scenario, level: Number(level), seed: Number(seed), seconds: Number(T),
  progress: sim.q.reduce((s, p) => s+p[1], 0)/12-y0,
  tips: sim.sgt?.tipCount ?? 0, aborts: sim.sgt?.abortCount ?? 0,
  slackPct: 100*slackCables/(12*steps), anySlackPct: 100*anySlack/steps,
  meanCoreTension: coreT.reduce((s, v) => s+v, 0)/coreT.length, maxCoreTension: maxCore,
  minClearance: minClear, maxOuterTension: maxOuter, rattleMm: 1000*rattle,
  payloadAccelP99g: accel[Math.floor(0.99*accel.length)], payloadAccelMaxg: accel[accel.length-1],
  controllerMs: ctl.totalMs/Math.max(1, ctl.count)
}));
