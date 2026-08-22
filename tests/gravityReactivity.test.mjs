import test from 'node:test';
import assert from 'node:assert/strict';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';

// Shared baseline: nearly flat Level 1 ground, passive physics, no logging —
// so every difference below is attributable to gravity alone.
const makeConfig = gravity => new SimConfig({
  dt: 0.002,
  terrainLevel: 1,
  groundRMS: 0.001,
  actuationMode: 'none',
  enableDiagnosticsLog: false,
  monitoring: { enabled: false },
  gravity: [0, 0, -gravity],
  seed: 42
});

const makeDroppedSim = gravity => {
  const config = makeConfig(gravity);
  const sim = new Simulation(config, new SphericalRoverModel(config), new TerrainModel(config), 'fixed');
  for (const position of sim.q) position[2] += 1.20;
  sim.corePosition[2] += 1.20;
  return sim;
};

test('impact kinetics scale with the configured gravity preset', () => {
  const settleSteps = 750; // 1.5 s covers free fall plus impact
  const measure = gravity => {
    const sim = makeDroppedSim(gravity);
    for (let step = 0; step < settleSteps; step++) sim.step();
    return sim.metrics.peakKineticEnergy;
  };
  const earth = measure(9.81);
  const mars = measure(3.721);
  const moon = measure(1.625);

  for (const energy of [earth, mars, moon]) {
    assert.ok(Number.isFinite(energy) && energy > 0, 'drop must generate kinetic energy');
  }
  // Free-fall energy is m·g·h, so ratios track the preset ratios closely.
  const earthOverMoon = earth/moon;
  const earthOverMars = earth/mars;
  assert.ok(earthOverMoon > 3.2 && earthOverMoon < 9.0,
    `earth/moon energy ratio ${earthOverMoon.toFixed(2)} expected ≈ ${(9.81/1.625).toFixed(2)}`);
  assert.ok(earthOverMars > 1.5 && earthOverMars < 4.2,
    `earth/mars energy ratio ${earthOverMars.toFixed(2)} expected ≈ ${(9.81/3.721).toFixed(2)}`);
});

test('changing gravity mid-run immediately retunes the running solver', () => {
  const early = makeDroppedSim(9.81);   // stays on Earth gravity
  const retuned = makeDroppedSim(9.81); // switches to Moon gravity mid-fall

  // Both fall identically until t = 50 ms...
  for (let step = 0; step < 25; step++) { early.step(); retuned.step(); }
  // ...then Model B's universe switches to lunar gravity while airborne.
  retuned.cfg.gravity = [0, 0, -1.625];
  for (let step = 0; step < 400; step++) { early.step(); retuned.step(); }

  const zEarly = early.currentDiag.centroid[2];
  const zRetuned = retuned.currentDiag.centroid[2];
  for (const z of [zEarly, zRetuned]) assert.ok(Number.isFinite(z));
  // The retuned shell falls slower, so after the same wall-clock steps it is
  // still visibly higher than the Earth-gravity twin.
  assert.ok(zRetuned-zEarly > 0.05,
    `expected divergent heights, got ${zRetuned.toFixed(4)} vs ${zEarly.toFixed(4)}`);
  // Contact Hertz loads carry the weight difference too.
  const weightEarly = early.metrics.maxCableTension;
  assert.ok(Number.isFinite(weightEarly));
});

test('every terrain level 1-14 tolerates a live gravity retune without instability', () => {
  for (let level = 1; level <= 14; level++) {
    const config = new SimConfig({
      dt: 0.004,
      terrainLevel: level,
      experimentId: level === 14 ? 14 : level,
      abCourseEnabled: false,
      courseStartY: level === 14 ? -450 : undefined,
      targetGoalY: level === 14 ? 450 : 25,
      courseGoalY: level === 14 ? 450 : undefined,
      actuationMode: 'none',
      enableDiagnosticsLog: false,
      monitoring: { enabled: false }
    });
    const sim = new Simulation(
      config, new SphericalRoverModel(config), new TerrainModel(config), 'fixed');
    for (let step = 0; step < 10; step++) sim.step();
    config.gravity = [0, 0, -1.625];
    for (let step = 0; step < 10; step++) sim.step();

    assert.equal(sim.currentDiag.centroidAccelG > 0, true,
      `level ${level} lost its G reading`);
    for (const position of sim.q) {
      assert.ok(position.every(Number.isFinite), `level ${level} produced NaN node state`);
    }
    assert.ok([sim.metrics.maxCableTension, sim.metrics.payloadAccelMax]
      .every(value => Number.isFinite(value)), `level ${level} metrics not finite`);
  }
});
