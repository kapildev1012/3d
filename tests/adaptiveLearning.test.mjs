import test from 'node:test';
import assert from 'node:assert/strict';
import { AdaptiveRouteLearner, LEVEL14_LEARNING_DEFAULTS } from '../js/adaptiveLearning.js';
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

test('Level 14 defaults cover the full 1km open-world expedition with 5m segments and gentle gains', () => {
  assert.equal(LEVEL14_LEARNING_DEFAULTS.courseStartY, -450);
  assert.equal(LEVEL14_LEARNING_DEFAULTS.courseGoalY, 450);
  assert.equal(LEVEL14_LEARNING_DEFAULTS.deadlineSeconds, 5000);

  const learner = new AdaptiveRouteLearner({ ...LEVEL14_LEARNING_DEFAULTS });
  assert.equal(learner.segmentCount, Math.ceil(900/5)); // 180 segments over the 900m run

  // A 1km-scale run: slow progress over a long observation window.
  for (let time = 0; time <= 60; time += 0.5) {
    learner.observe({
      time,
      x: 0.3,
      y: -450+0.09*time,
      speed: 0.09,
      slip: 0.18,
      rollingError: 0.14,
      grade: 0.12,
      energy: 4.2
    });
  }
  learner.observeCheckpoint(-448.2, 120);
  const result = learner.finishRun({
    reached: false,
    time: 5000,
    finalY: 240,
    maxSlip: 0.22,
    rollingError: 0.16,
    lateralTravel: 30,
    reason: 'timeout'
  });

  assert.equal(result.outcome, 'loss');
  assert.ok(learner.segments.some(segment => segment.visits > 0));
  assert.ok(learner.segments.some(segment => segment.checkpointReached));
  assert.ok(learner.segments.some(segment => segment.energyEMA > 0));
  // Gentle learning rate must keep every scale inside its Level 14 bounds.
  for (const segment of learner.segments) {
    assert.ok(segment.speedScale >= LEVEL14_LEARNING_DEFAULTS.minimumSpeedScale &&
      segment.speedScale <= LEVEL14_LEARNING_DEFAULTS.maximumSpeedScale);
    assert.ok(segment.torqueScale >= LEVEL14_LEARNING_DEFAULTS.minimumTorqueScale &&
      segment.torqueScale <= LEVEL14_LEARNING_DEFAULTS.maximumTorqueScale);
    assert.ok(segment.tractionScale >= LEVEL14_LEARNING_DEFAULTS.minimumTractionScale &&
      segment.tractionScale <= LEVEL14_LEARNING_DEFAULTS.maximumTractionScale);
  }
});

test('Level 14 learned state round-trips through serialize/restore', () => {
  const first = new AdaptiveRouteLearner({ ...LEVEL14_LEARNING_DEFAULTS });
  first.observe({ time: 0, x: 0.2, y: 10, speed: 0.2, slip: 0.05, rollingError: 0.04, grade: 0.05, energy: 3 });
  first.observe({ time: 5, x: 0.25, y: 14, speed: 0.22, slip: 0.04, rollingError: 0.03, grade: 0.04, energy: 3.5 });
  first.observeCheckpoint(12.5, 90);
  first.finishRun({ reached: true, time: 4200, finalY: 800, maxSlip: 0.10, rollingError: 0.05, lateralTravel: 12, reason: 'goal' });

  const restored = new AdaptiveRouteLearner({ ...LEVEL14_LEARNING_DEFAULTS }, first.serialize());
  assert.equal(restored.runCount, 1);
  assert.equal(restored.wins, 1);
  assert.equal(restored.bestTime, 4200);
  const visited = restored.segments.filter(segment => segment.visits > 0);
  assert.ok(visited.length > 0);
  assert.ok(visited.some(segment => segment.checkpointReached));
  assert.ok(visited.some(segment => segment.energyEMA > 0));

  const command = restored.commandAt({ x: 0, y: 200, grade: 0.15 });
  assert.ok(command.speedScale >= LEVEL14_LEARNING_DEFAULTS.minimumSpeedScale);
  assert.ok(command.torqueScale <= LEVEL14_LEARNING_DEFAULTS.maximumTorqueScale);
  assert.ok(command.tractionScale >= LEVEL14_LEARNING_DEFAULTS.minimumTractionScale);
});

test('Level 14 builds an expedition waypoint chain that feeds the learner', () => {
  // Scaled-down corridor (goal at 40 m, beacon every 10 m) keeps the physics
  // cheap while exercising the exact same waypoint code path as the 1 km map.
  const config = new SimConfig({
    dt: 0.004,
    T_end: 0.05,
    missionDeadlineSeconds: 0.05,
    experimentId: 14,
    terrainLevel: 14,
    abCourseEnabled: false,
    courseStartY: 10,
    targetGoalY: 40,
    courseGoalY: 40,
    targetDestination: [0, 40],
    waypointSpacing: 10,
    enableDiagnosticsLog: false,
    monitoring: { enabled: false }
  });
  const learner = new AdaptiveRouteLearner({ ...LEVEL14_LEARNING_DEFAULTS,
    courseGoalY: 40 });
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simulation = new Simulation(config, rover, terrain, 'adaptive', learner);

  assert.equal(simulation.expeditionWaypoints.length, 2); // 20 m and 30 m
  assert.equal(simulation.expeditionWaypoints[0].id, 'WP1');

  while (!simulation.metrics.runTerminal && !simulation.metrics.courseComplete) {
    simulation.step();
    if (simulation.t > config.T_end+1) break;
  }

  // Waypoint progress is reported for the HUD and recorded in the learner.
  assert.ok(simulation.metrics.expeditionWaypoints.total === 2);
  assert.equal(learner.segments.some(segment => segment.checkpointReached),
    simulation.nextWaypointIndex > 0);
});
