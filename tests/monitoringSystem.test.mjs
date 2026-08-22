import test from 'node:test';
import assert from 'node:assert/strict';
import {
  MONITORING_DEFAULTS,
  mergeMonitoringSettings,
  rigidAlignedFormationError,
  buildCableTelemetry,
  RealtimeMonitor
} from '../js/monitoringSystem.js';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';

test('formation error removes rigid translation and rotation but detects deformation', () => {
  const reference = [[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [1, 1, 1]];
  const angle = Math.PI/3;
  const cosine = Math.cos(angle);
  const sine = Math.sin(angle);
  const transformed = reference.map(([x, y, z]) => [
    cosine*x-sine*y+4,
    sine*x+cosine*y-2,
    z+7
  ]);
  assert.ok(rigidAlignedFormationError(transformed, reference) < 1e-10);
  transformed[2][2] += 0.20;
  assert.ok(rigidAlignedFormationError(transformed, reference) > 0.05);
});

test('cable monitor reports length, strain, slack, force and overload per cable', () => {
  const rover = {
    outerStrings: [[0, 1], [1, 2]],
    l0_outerStrings: [1, 1]
  };
  const cables = buildCableTelemetry({
    rover,
    q: [[0, 0, 0], [1.1, 0, 0], [1.7, 0, 0]],
    forces: [120, 0],
    actuationOffsets: [0, 0],
    relaxedFlags: [false, true],
    settings: { ...MONITORING_DEFAULTS, cableOverloadForce: 100 }
  });
  assert.equal(cables.length, 2);
  assert.ok(Math.abs(cables[0].deltaLength-0.1) < 1e-12);
  assert.ok(Math.abs(cables[0].strainPercent-10) < 1e-10);
  assert.equal(cables[0].overloaded, true);
  assert.equal(cables[1].slack, true);
  assert.equal(cables[1].force, 0, 'tension-only monitoring must never report compression force');
});

test('adaptive simulation exposes raw monitoring, active contacts and separate exports', () => {
  const config = new SimConfig({
    terrainLevel: 1,
    targetDestination: [0, 0],
    targetGoalY: 0,
    enableDiagnosticsLog: false,
    monitoring: { rawLogging: true, maxRawSamples: 600 }
  });
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simulation = new Simulation(config, rover, terrain, 'adaptive');
  for (let step = 0; step < 500; step++) simulation.step();

  const monitoring = simulation.currentDiag.monitoring;
  assert.ok(monitoring);
  assert.equal(monitoring.cables.length, 24);
  assert.ok(monitoring.activeContactCount >= 1);
  assert.ok(monitoring.contacts.every(contact => contact.normalForce >= config.monitoring.contactForceThreshold));
  assert.equal(simulation.monitor.records.length, 500);
  assert.ok(simulation.monitor.exportCsv('goal_tracking').startsWith('time,com_x'));
  assert.ok(simulation.monitor.exportCsv('formation_metrics').includes('formation_error'));
  assert.ok(simulation.monitor.exportCsv('cable_metrics').split('\n').length > 500*24);
  assert.ok(simulation.monitor.exportCsv('contact_events').startsWith('kind,id,object'));
  assert.ok(simulation.monitor.exportCsv('complete_simulation_log').includes('node_positions'));
  assert.equal(JSON.parse(simulation.monitor.exportJson()).samples.length, 500);
});

test('goal success latches time, traveled distance and final error', () => {
  const config = new SimConfig({ terrainLevel: 1, targetDestination: [0, 0], targetGoalY: 0 });
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const initial = rover.q0_outer.map(point => point.slice());
  const center = [0, 0, 0];
  const monitor = new RealtimeMonitor(config, rover, terrain, initial, center);
  const result = monitor.sample({
    time: 2,
    q: initial,
    centroid: center,
    velocity: [0, 0, 0],
    cableForces: new Array(24).fill(0),
    actuationOffsets: new Array(24).fill(0),
    relaxedFlags: new Array(24).fill(true),
    contacts: [],
    terrainClearance: 0,
    distanceTraveled: 3.5
  });
  assert.equal(result.goalReached, true);
  assert.equal(result.status, 'Goal Reached');
  assert.equal(result.goalResult.time, 2);
  assert.equal(result.goalResult.distanceTraveled, 3.5);
});

test('mergeMonitoringSettings overrides defaults without mutating them', () => {
  const merged = mergeMonitoringSettings({ goalThreshold: 0.9 });
  assert.equal(merged.goalThreshold, 0.9);
  assert.equal(merged.contactForceThreshold, MONITORING_DEFAULTS.contactForceThreshold);
  assert.ok(Object.isFrozen(MONITORING_DEFAULTS));
  merged.goalThreshold = 0.7;
  assert.equal(merged.goalThreshold, 0.7, 'runtime sliders must be able to retune thresholds');
  assert.equal(MONITORING_DEFAULTS.goalThreshold, 0.50, 'defaults must stay untouched');
  assert.equal(mergeMonitoringSettings().goalThreshold, MONITORING_DEFAULTS.goalThreshold);
});

test('monitor accumulates run-wide peaks across samples and exports them', () => {
  const flatTerrain = { eval: () => ({ h: 0 }) };
  const miniRover = {
    outerStrings: [[0, 1]],
    l0_outerStrings: [1],
    bars: [[0, 1]],
    l0_bars: [1],
    R_outer: 0.5
  };
  const monitor = new RealtimeMonitor(
    { targetDestination: [3, 0], monitoring: { rawLogging: false } },
    miniRover,
    flatTerrain,
    [[0, 0, 0.5], [1, 0, 0.5]],
    [0.5, 0, 0.5]
  );

  const baseState = {
    q: [[0, 0, 0.5], [1, 0, 0.5]],
    centroid: [0.5, 0, 0.5],
    velocity: [0, 0, 0],
    actuationOffsets: [0],
    relaxedFlags: [false],
    contacts: [],
    terrainClearance: 0,
    distanceTraveled: 0
  };

  const first = monitor.sample({ ...baseState, time: 0.1, cableForces: [100] });
  assert.equal(first.peakCableForce, 100);

  const stretched = [[0, 0, 0.5], [1.2, 0, 0.5]];
  const second = monitor.sample({
    ...baseState, time: 0.2, q: stretched, centroid: [0.6, 0, 0.5], cableForces: [250]
  });
  assert.equal(second.peakCableForce, 250, 'peaks must keep the run maximum');
  assert.ok(second.peakCableStrainPercent >= 20-1e-9, 'strain peak must track the stretched cable');
  assert.ok(second.peakFormationError > 0);

  const atGoal = monitor.sample({
    ...baseState, time: 0.3, cableForces: [10], centroid: [3, 0, 0.5], distanceTraveled: 4
  });
  assert.equal(atGoal.peakCableForce, 250, 'later low forces must not erase earlier peaks');
  assert.equal(atGoal.goalReached, true);

  const exported = JSON.parse(monitor.exportJson());
  assert.equal(exported.peaks.cableForce, 250);
  assert.ok(exported.peaks.formationError > 0);
});
