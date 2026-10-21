import test from 'node:test';
import assert from 'node:assert/strict';
import { ShellDAE, flatten, solveDense } from '../js/lagrangianDae.js';
import { SimConfig, SphericalRoverModel, TerrainModel, Simulation } from '../js/simEngine.js';

const makeDae = () => {
  const cfg = new SimConfig();
  const rover = new SphericalRoverModel(cfg);
  const dae = new ShellDAE({
    nNodes: 12, bars: rover.bars, barLengths: rover.l0_bars,
    barMass: cfg.strutMass, barRadius: 0.012, nodeMass: cfg.nodeMass, gravity: cfg.gravity
  });
  return { cfg, rover, dae };
};

test('closed-form M^-1 inverts the consistent rigid-bar mass matrix (Eq. 22)', () => {
  const { dae } = makeDae();
  const x = Float64Array.from({ length: 36 }, (_, i) => Math.sin(1.7*i+0.3));
  const back = dae.solveM(dae.mulM(x));
  for (let i = 0; i < 36; i++) assert.ok(Math.abs(back[i]-x[i]) < 1e-12);
  // total mass and gravity split: half of each bar at each end + node mass
  assert.ok(Math.abs(dae.totalMass-(12*0.2+6*0.3)) < 1e-12);
});

test('constrained accelerations satisfy Rq qdd = -xi2 to machine precision (Eq. 34-35)', () => {
  const { rover, dae } = makeDae();
  const q = flatten(rover.q0_outer);
  const v = Float64Array.from({ length: 36 }, (_, i) => 0.4*Math.cos(3*i));
  dae.correct(q, v, null);                         // admissible velocity
  const F = Float64Array.from({ length: 36 }, (_, i) => 5*Math.sin(2*i));
  const { qdd } = dae.accelerations(q, v, F);
  const { b, bd } = dae.residuals(q, v);
  const Rqa = dae.gatherRq(b, qdd);
  for (let k = 0; k < 6; k++) {
    let bdbd = 0;
    for (let a = 0; a < 3; a++) bdbd += bd[3*k+a]**2;
    assert.ok(Math.abs(Rqa[k]+2*bdbd) < 1e-10, `bar ${k}: ${Rqa[k]+2*bdbd}`);
  }
});

test('Algorithm 1 restores bar lengths, velocity constraints and energy (Eq. 56-57)', () => {
  const { rover, dae } = makeDae();
  const q = flatten(rover.q0_outer);
  const v = Float64Array.from({ length: 36 }, (_, i) => 0.3*Math.sin(5*i));
  dae.correct(q, v, null);
  const E0 = dae.energy(q, v).E;
  // perturb state off the manifold and off the energy level
  for (let i = 0; i < 36; i++) { q[i] += 1e-4*Math.cos(i); v[i] += 1e-3*Math.sin(2*i); }
  const info = dae.correct(q, v, E0, { gamma: 1e-12, energyGamma: 1e-10 });
  assert.ok(dae.maxLengthError(q) < 1e-10, `bar error ${dae.maxLengthError(q)}`);
  assert.ok(info.Rv < 1e-10, `velocity residual ${info.Rv}`);
  assert.ok(Math.abs(dae.energy(q, v).E-(E0+info.dW)) < 1e-9);
  assert.ok(info.iters <= 10);
});

test('dense solver handles a well-posed system', () => {
  const A = [4, 1, 0, 1, 3, 1, 0, 1, 2];
  const x = solveDense(A, [1, 2, 3], 3);
  const r = [4*x[0]+x[1]-1, x[0]+3*x[1]+x[2]-2, x[1]+2*x[2]-3];
  for (const value of r) assert.ok(Math.abs(value) < 1e-12);
});

test('DAE core keeps bars exact and the energy ledger closed during a drop', () => {
  const cfg = new SimConfig({ terrainLevel: 1, groundRMS: 0.001, actuationMode: 'none', physicsCore: 'dae',
    enableDiagnosticsLog: false, monitoring: { enabled: false } });
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'fixed');
  for (const p of sim.q) p[2] += 0.6;
  sim.corePosition[2] += 0.6;
  let maxBar = 0;
  for (let k = 0; k < 600; k++) maxBar = Math.max(maxBar, sim.step().constraintError);
  assert.ok(maxBar < 1e-9, `bar error ${maxBar}`);
  assert.ok(sim.ledger.maxAbsResidual < 1e-8, `energy residual ${sim.ledger.maxAbsResidual}`);
  // passive contact can only dissipate
  assert.ok(sim.ledger.work.contact <= 1e-9, `contact work ${sim.ledger.work.contact}`);
});
