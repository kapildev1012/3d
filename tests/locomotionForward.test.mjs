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
    enableDiagnosticsLog: false
  });
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simulation = new Simulation(config, rover, terrain, 'adaptive');

  // A tensegrity gait advances face-by-face, with a settle period between
  // tips. Ten simulated seconds covers two complete support-face rolls.
  for (let step = 0; step < 5000; step++) simulation.step();

  assert.ok(simulation.currentDiag.centroid[1] > 9.50,
    `expected forward displacement from y=9 m, got ${simulation.currentDiag.centroid[1]}`);
  assert.ok(simulation.currentDiag.completedRolls >= 2,
    `expected at least two support-face rolls, got ${simulation.currentDiag.completedRolls}`);
  assert.ok(simulation.currentDiag.rollingSpeed > 0,
    `expected forward shell rotation, got ${simulation.currentDiag.rollingSpeed}`);
  assert.ok(Math.abs(simulation.currentDiag.velocityVector[1]-simulation.currentDiag.rollingSpeed) < 0.04,
    `translation/rolling mismatch: v=${simulation.currentDiag.velocityVector[1]}, `+
    `vRoll=${simulation.currentDiag.rollingSpeed}`);
  assert.ok(simulation.currentDiag.slipSpeed < 0.08,
    `excessive contact slip: ${simulation.currentDiag.slipSpeed}`);
  assert.ok(simulation.currentDiag.terrainClearance >= -1e-9,
    `terrain penetration detected: ${simulation.currentDiag.terrainClearance}`);
  assert.ok(simulation.currentDiag.angularVelocity < config.antiSpinThreshold,
    `anti-spin limit exceeded: ${simulation.currentDiag.angularVelocity}`);
});
