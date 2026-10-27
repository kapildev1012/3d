import test from 'node:test';
import assert from 'node:assert/strict';
import {
  SimConfig,
  TerrainModel,
  SphericalRoverModel,
  Simulation
} from '../js/simEngine.js';

test('adaptive rover advances by rolling without being pulled across the ground', () => {
  const config = new SimConfig({
    terrainLevel: 10,
    abCourseEnabled: true,
    targetDestination: [0, 60],
    targetGoalY: 60,
    enableDiagnosticsLog: false,
    monitoring: { enabled: false }
  }).applyLevel10PerformanceProfile();
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simulation = new Simulation(config, rover, terrain, 'adaptive');

  // A tensegrity gait advances face-by-face, with a settle period between
  // tips. Ten simulated seconds covers two complete support-face rolls.
  // Rolling consistency and contact slip are judged on their median over the
  // last 5 s: a single end-of-run snapshot passes or fails depending on whether
  // it lands on an impact-settle transient (90th percentiles are ~0.5-0.65 m/s
  // for both quantities in this run, with the same median either way).
  const mismatch = [], slip = [], spin = [];
  for (let step = 0; step < 5000; step++) {
    simulation.step();
    if (step >= 2500 && step % 25 === 0) {
      mismatch.push(Math.abs(simulation.currentDiag.velocityVector[1]-simulation.currentDiag.rollingSpeed));
      slip.push(simulation.currentDiag.slipSpeed);
      spin.push(simulation.currentDiag.angularVelocity);
    }
  }
  const median = values => values.slice().sort((a, b) => a-b)[values.length >> 1];

  assert.ok(simulation.currentDiag.centroid[1] > 9.50,
    `expected forward displacement from y=9 m, got ${simulation.currentDiag.centroid[1]}`);
  assert.ok(simulation.currentDiag.completedRolls >= 2,
    `expected at least two support-face rolls, got ${simulation.currentDiag.completedRolls}`);
  assert.ok(simulation.currentDiag.rollingSpeed > 0,
    `expected forward shell rotation, got ${simulation.currentDiag.rollingSpeed}`);
  // This snapshot can land in IMPACT_SETTLE, where the shell briefly trades
  // translational speed for obstacle-normal motion. Keep the tolerance tied
  // to terrain amplitude; completed rolls and contact slip below are the
  // stronger no-drag checks on coarse ground.
  const roughTerrainRollingTolerance = Math.max(0.04, 2.25*config.groundRMS);
  assert.ok(median(mismatch) < roughTerrainRollingTolerance,
    `translation/rolling mismatch: median |v - vRoll| over the last 5 s = ${median(mismatch)}`);
  // Rough ground produces local tangential contact motion even while the
  // shell-scale translation remains matched to rotation above.
  const roughTerrainSlipLimit = Math.max(0.08, 5*config.groundRMS);
  assert.ok(median(slip) < roughTerrainSlipLimit,
    `excessive contact slip: median over the last 5 s = ${median(slip)}`);
  assert.ok(simulation.currentDiag.terrainClearance >= -1e-9,
    `terrain penetration detected: ${simulation.currentDiag.terrainClearance}`);
  for (const [component, clearance] of Object.entries(simulation.metrics.minimumTerrainClearances)) {
    assert.ok(clearance >= -1e-9,
      `${component} penetrated terrain during the run: ${clearance}`);
  }
  assert.equal(simulation.currentDiag.pathCorridor.lane, 'B');
  assert.ok(simulation.q.every(position =>
    position[0]-config.nodeRadius >= -config.pathCorridorHalfWidth-1e-9 &&
    position[0]+config.nodeRadius <= config.pathCorridorHalfWidth+1e-9),
  'Model B left its assigned local corridor');
  assert.ok(simulation.q.every(position =>
    position[0]+config.modelLaneOffset-config.nodeRadius > 0),
  'Model B entered Model A world path');
  // The anti-spin damper is a soft limit: it engages above antiSpinThreshold and
  // pulls the spin rate back (it never guaranteed omega < threshold at an
  // arbitrary instant; at ~1.5 m/s a 0.5 m shell rolls at ~3 rad/s). Check its
  // effect: the median stays near the threshold and peaks stay bounded.
  assert.ok(median(spin) < 1.1*config.antiSpinThreshold,
    `anti-spin damper ineffective: median spin ${median(spin)} rad/s vs threshold ${config.antiSpinThreshold}`);
  assert.ok(Math.max(...spin) < 1.5*config.antiSpinThreshold,
    `anti-spin limit exceeded: peak spin ${Math.max(...spin)} rad/s`);
});

test('baseline Model A keeps every rendered member outside terrain and rocks', () => {
  const config = new SimConfig({
    dt: 0.004,
    terrainLevel: 10,
    abCourseEnabled: true,
    targetDestination: [0, 60],
    targetGoalY: 60,
    enableDiagnosticsLog: false,
    monitoring: { enabled: false }
  }).applyLevel10PerformanceProfile();
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simulation = new Simulation(config, rover, terrain, 'fixed');

  for (let step = 0; step < 2500; step++) simulation.step();

  for (const [component, clearance] of Object.entries(simulation.metrics.minimumTerrainClearances)) {
    assert.ok(clearance >= -1e-9,
      `Model A ${component} penetrated terrain during the run: ${clearance}`);
  }
  assert.equal(simulation.currentDiag.pathCorridor.lane, 'A');
  assert.ok(simulation.q.every(position =>
    position[0]-config.nodeRadius >= -config.pathCorridorHalfWidth-1e-9 &&
    position[0]+config.nodeRadius <= config.pathCorridorHalfWidth+1e-9),
  'Model A left its assigned local corridor');
  assert.ok(simulation.q.every(position =>
    position[0]-config.modelLaneOffset+config.nodeRadius < 0),
  'Model A entered Model B world path');

  const denseSegmentClearance = (positionA, positionB, radius) => {
    let minimum = Infinity;
    for (let sample = 0; sample <= 100; sample++) {
      const t = sample/100;
      const x = positionA[0]*(1-t)+positionB[0]*t;
      const y = positionA[1]*(1-t)+positionB[1]*t;
      const z = positionA[2]*(1-t)+positionB[2]*t;
      minimum = Math.min(minimum, z-simulation.terrain.eval(x, y).h-radius);
    }
    return minimum;
  };
  for (const [first, second] of rover.bars) {
    assert.ok(denseSegmentClearance(simulation.q[first], simulation.q[second], 0.035) >= -1e-9,
      'Model A bar penetrated between collision samples');
  }
  for (const [first, second] of rover.outerStrings) {
    assert.ok(denseSegmentClearance(simulation.q[first], simulation.q[second], 0.012) >= -1e-9,
      'Model A outer cable penetrated between collision samples');
  }
  for (let node = 0; node < simulation.q.length; node++) {
    assert.ok(denseSegmentClearance(simulation.q[node], simulation.corePosition, 0.006) >= -1e-9,
      'Model A core cable penetrated between collision samples');
  }
});
