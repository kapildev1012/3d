import test from 'node:test';
import assert from 'node:assert/strict';
import {
  LEVEL10_PERFORMANCE,
  SimConfig,
  TerrainModel,
  SphericalRoverModel,
  Simulation
} from '../js/simEngine.js';
import { AdaptiveRouteLearner } from '../js/adaptiveLearning.js';

test('Level 10 wins by reaching the 50 m goal before the 120 s deadline', { timeout: 150_000 }, () => {
  // A 250 Hz test step halves regression time; the shipped simulator retains
  // its 500 Hz fixed step, which is verified separately during acceptance.
  const config = new SimConfig({
    dt: 0.004,
    experimentId: 10,
    terrainLevel: 10,
    abCourseEnabled: true,
    targetDestination: [0, 60],
    targetGoalY: 60,
    enableDiagnosticsLog: false,
    monitoring: { enabled: false }
  }).applyLevel10PerformanceProfile();
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const learner = new AdaptiveRouteLearner({ deadlineSeconds: config.missionDeadlineSeconds });
  const simulation = new Simulation(config, rover, terrain, 'adaptive', learner);

  assert.equal(config.targetSpeed, LEVEL10_PERFORMANCE.targetSpeed);
  assert.equal(config.T_end, 120);
  assert.equal(config.missionDeadlineSeconds, 120);
  assert.equal(config.groundRMS, 0.06);
  assert.ok(config.mu_g >= 6.0);
  assert.equal(terrain.rmsScale, config.groundRMS);
  assert.ok(terrain.rmsScale > 0.018, 'Level 10 must retain the increased terrain roughness');
  assert.ok(terrain.course.obstacles.every(obstacle =>
    Math.abs(obstacle.height-0.5*(2*rover.R_outer)) < 1e-12));

  const maximumSteps = Math.ceil(config.T_end/config.dt);
  for (let step = 0; step < maximumSteps && !simulation.metrics.courseComplete; step++) {
    simulation.step();
  }

  assert.equal(simulation.metrics.courseComplete, true,
    `rover stopped at y=${simulation.currentDiag.centroid[1].toFixed(3)} m`);
  assert.equal(simulation.metrics.runOutcome, 'win');
  assert.equal(learner.wins, 1);
  assert.ok(simulation.metrics.completionTime < 120,
    `completion took ${simulation.metrics.completionTime?.toFixed(3)} s`);
  assert.ok(simulation.metrics.completedRolls >= 24,
    `expected repeated shell rolls, got ${simulation.metrics.completedRolls}`);
  assert.equal(simulation.metrics.obstacleSummary.over, 10);
  assert.equal(simulation.metrics.obstacleSummary.checkpointsReached, 10);
  assert.equal(simulation.metrics.obstacleSummary.allCheckpointsReached, true);
  assert.equal(simulation.metrics.obstacleSummary.around, 0);
  assert.equal(simulation.metrics.obstacleSummary.bypassViolations, 0);
  assert.ok(simulation.metrics.maximumPathOffset <= config.pathCorridorHalfWidth+1e-9,
    `Model B left its path corridor: ${simulation.metrics.maximumPathOffset}`);
  assert.ok(simulation.q.every(position =>
    position[0]-config.nodeRadius >= -config.pathCorridorHalfWidth-1e-9 &&
    position[0]+config.nodeRadius <= config.pathCorridorHalfWidth+1e-9),
  'Model B structure crossed a corridor boundary');
  for (const [component, clearance] of Object.entries(simulation.metrics.minimumTerrainClearances)) {
    assert.ok(clearance >= -1e-9,
      `${component} penetrated terrain during Level 10: ${clearance}`);
  }
});
