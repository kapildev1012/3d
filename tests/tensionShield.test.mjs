/**
 * Tension-certified safety filter: constraint satisfaction on the predicted
 * (linearized) state, minimal change when already safe, and a closed-loop
 * check that it keeps the payload suspension taut.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { shieldCommand, SHIELD_DEFAULTS } from '../js/tensionShield.js';
import { SensitivityTippingController } from '../js/tippingController.js';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';
import { configureLevel } from '../js/levels.js';

console.info = () => {};

function makeSim(setup = () => {}) {
  const cfg = new SimConfig({}); configureLevel(cfg, 1); cfg.applyResearchProfile(); setup(cfg);
  return new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
}
const bounds = sim => ({
  lo: [...new Array(24).fill(-0.35*sim.cfg.actuationDeltaL), ...new Array(12).fill(-sim.cfg.coreActuationDeltaL)],
  hi: [...new Array(24).fill(sim.cfg.actuationDeltaL), ...new Array(12).fill(sim.cfg.coreActuationDeltaL)]
});
// Predicted payload-cable tension after the command, with the same linear model
function predictedCoreTension(sim, sens, u) {
  const uNow = [...sim.currentActuationOffsets, ...sim.currentCoreActuationOffsets];
  const n = sim.rover.nOuter;
  return sim.q.map((p, i) => {
    const c = sim.corePosition, d = [c[0]-p[0], c[1]-p[1], c[2]-p[2]], L = Math.hypot(...d), e = d.map(v => v/L);
    let dL = 0;
    for (let a = 0; a < 36; a++) {
      const X = sens.displacements[a];
      dL += (e[0]*(X[3*n]-X[3*i])+e[1]*(X[3*n+1]-X[3*i+1])+e[2]*(X[3*n+2]-X[3*i+2]))*(u[a]-uNow[a]);
    }
    return sim.cfg.kCore*(L-sim.baseCoreRestLengths[i]+u[24+i]+dL);
  });
}

test('filter enforces the payload-tension barrier on the predicted state', () => {
  const sim = makeSim();
  while (sim.t < 3) sim.step();                       // shell has sagged; payload cables partly slack
  const sgt = new SensitivityTippingController(sim);
  const face = sgt.supportGeometry([0, 1]).face;
  const sens = sgt.sensitivity(face);
  // Adversarial nominal: relax every payload cable fully
  const nominal = { cableTargets: new Array(24).fill(0), coreTargets: new Array(12).fill(-sim.cfg.coreActuationDeltaL) };
  const out = shieldCommand(sim, sens, nominal, bounds(sim));
  const u = [...out.cableTargets, ...out.coreTargets];
  const before = predictedCoreTension(sim, sens, [...sim.currentActuationOffsets, ...sim.currentCoreActuationOffsets]);
  const after = predictedCoreTension(sim, sens, u);
  const g = SHIELD_DEFAULTS.gamma, tmin = SHIELD_DEFAULTS.coreTensionMin;
  after.forEach((t, i) => assert.ok(t-tmin >= (1-g)*(before[i]-tmin)-0.25 /* solver tolerance ≈ 0.1 mm × k */, `cable ${i}: ${t.toFixed(2)} N`));
  u.forEach((v, k) => assert.ok(v >= bounds(sim).lo[k]-1e-12 && v <= bounds(sim).hi[k]+1e-12));
  assert.ok(out.info.converged);
});

test('filter leaves a safe command (almost) unchanged', () => {
  const sim = makeSim();
  sim.step();                                          // at rest: all payload cables taut (~38 N)
  const sgt = new SensitivityTippingController(sim);
  const sens = sgt.sensitivity(sgt.supportGeometry([0, 1]).face);
  const nominal = { cableTargets: new Array(24).fill(0), coreTargets: new Array(12).fill(0) };
  const out = shieldCommand(sim, sens, nominal, bounds(sim));
  assert.ok(out.info.correction < 1e-6, `correction ${out.info.correction}`);
});

test('closed loop: SGT + filter keeps the payload cables taut and still rolls', { timeout: 300000 }, () => {
  const sim = makeSim(cfg => { cfg.tensionShield = true; });
  let slack = 0, steps = 0;
  const y0 = sim.q.reduce((s, p) => s+p[1], 0)/12;
  while (sim.t < 10) {
    sim.step();
    if (sim.t < 1) continue;
    steps++;
    slack += sim.coreCableForces.filter(t => t < 1).length;
  }
  const progress = sim.q.reduce((s, p) => s+p[1], 0)/12-y0;
  assert.ok(100*slack/(12*steps) < 8, `slack ${(100*slack/(12*steps)).toFixed(1)} %`);
  assert.ok(progress > 0.8, `progress ${progress.toFixed(2)} m`);
  assert.ok(sim.sgt.shieldInfo, 'filter ran');
});
