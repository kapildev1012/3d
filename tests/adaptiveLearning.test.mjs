import test from 'node:test';
import assert from 'node:assert/strict';
import { AdaptiveRouteLearner } from '../js/adaptiveLearning.js';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';

test('GPS route learner converts a failed attempt into bounded gradient updates', () => {
  const learner = new AdaptiveRouteLearner({ deadlineSeconds: 120 });
  for (let time = 0; time <= 12; time += 0.05) {
    learner.observe({
      time,
      x: 0.18,
      y: 10+0.08*time,
      speed: 0.08,
      slip: 0.20,
      rollingError: 0.15,
      grade: 0.10
    });
  }
  const result = learner.finishRun({
    reached: false,
    time: 120,
    finalY: 28,
    maxSlip: 0.25,
    rollingError: 0.18,
    lateralTravel: 8,
    reason: 'timeout'
  });

  assert.equal(result.outcome, 'loss');
  assert.equal(learner.losses, 1);
  assert.ok(learner.global.speedScale > 1);
  assert.ok(learner.global.torqueScale > 1);
  assert.ok(learner.global.speedScale <= learner.settings.maximumSpeedScale);
  assert.ok(learner.segments.some(segment => segment.visits > 0));
});

test('learned GPS state persists and records progressively better wins', () => {
  const first = new AdaptiveRouteLearner({ deadlineSeconds: 120 });
  first.observe({ time: 0, x: 0.10, y: 10, speed: 0.7, slip: 0.04, rollingError: 0.03, grade: 0 });
  first.observe({ time: 1, x: 0.10, y: 11, speed: 0.8, slip: 0.03, rollingError: 0.02, grade: 0 });
  first.finishRun({ reached: true, time: 82, finalY: 60, maxSlip: 0.08, rollingError: 0.03, lateralTravel: 2, reason: 'goal' });

  const restored = new AdaptiveRouteLearner({ deadlineSeconds: 120 }, first.serialize());
  assert.equal(restored.runCount, 1);
  assert.equal(restored.wins, 1);
  assert.equal(restored.bestTime, 82);
  restored.finishRun({ reached: true, time: 78, finalY: 60, maxSlip: 0.07, rollingError: 0.02, lateralTravel: 1.5, reason: 'goal' });
  assert.equal(restored.bestTime, 78);
  assert.equal(restored.lastRun.improvedBest, true);
  const command = restored.commandAt({ x: 0, y: 10.5, grade: 0.08 });
  assert.match(command.segmentLabel, /^S\d{2}$/);
  assert.ok(command.speedScale >= restored.settings.minimumSpeedScale);
  assert.ok(command.speedScale <= restored.settings.maximumSpeedScale);
});

test('mission deadline produces a loss instead of a forced 90 second completion', () => {
  const config = new SimConfig({
    dt: 0.004,
    T_end: 0.012,
    missionDeadlineSeconds: 0.012,
    terrainLevel: 10,
    abCourseEnabled: true,
    targetDestination: [0, 60],
    targetGoalY: 60,
    enableDiagnosticsLog: false,
    monitoring: { enabled: false }
  });
  const learner = new AdaptiveRouteLearner({ deadlineSeconds: config.missionDeadlineSeconds });
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simulation = new Simulation(config, rover, terrain, 'adaptive', learner);

  while (!simulation.metrics.runTerminal) simulation.step();

  assert.equal(simulation.metrics.runOutcome, 'loss');
  assert.equal(simulation.metrics.courseComplete, false);
  assert.equal(learner.losses, 1);
  assert.equal(learner.lastRun.reason, 'timeout');
});
