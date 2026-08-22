import test from 'node:test';
import assert from 'node:assert/strict';
import { LEVEL10_PERFORMANCE, SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';

const levelRms = [0, 0.035, 0.05, 0.07, 0.09, 0.075, 0.06, 0.10, 0.10, 0.10, 0.06];

test('all ten levels use Level 10 target speed and a rough Mars foundation', () => {
  for (let level = 1; level <= 10; level++) {
    const config = new SimConfig({
      terrainLevel: Math.min(7, level),
      abCourseEnabled: level === 10,
      groundRMS: levelRms[level]
    });
    if (level === 10) config.applyLevel10PerformanceProfile();
    else config.applyStandardPerformanceProfile();
    const terrain = new TerrainModel(config);

    assert.equal(config.targetSpeed, LEVEL10_PERFORMANCE.targetSpeed, `Level ${level} speed mismatch`);
    assert.equal(config.modelLaneOffset, 1.5);
    assert.ok(config.pathCorridorHalfWidth < config.modelLaneOffset,
      `Level ${level} corridors would overlap`);
    assert.ok(terrain.rmsScale >= 0.03, `Level ${level} terrain is too smooth`);
    assert.ok(terrain.rocks.some(rock => rock.kind === 'mars-scatter'));
    assert.ok(terrain.rocks.some(rock => rock.kind === 'mars-ridge'));
    assert.equal(terrain.solidTerrainObjects('fixed').length,
      terrain.rocks.length+(terrain.course?.obstacles.length || 0)+terrain.courseGritRocks.length);
    assert.equal(terrain.solidTerrainObjects('adaptive').length,
      terrain.solidTerrainObjects('fixed').length+terrain.bPathRocks.length);
    for (const [x, y] of [[0, 4], [-1.2, 12], [1.5, 24]]) {
      const sample = terrain.eval(x, y);
      assert.ok([sample.h, sample.dhdx, sample.dhdy].every(Number.isFinite));
    }
  }
});

test('Mars outcrops are deterministic and keep the Level 10 centre corridor open', () => {
  const config = new SimConfig({ terrainLevel: 10, abCourseEnabled: true, groundRMS: 0.03, seed: 42 });
  const first = new TerrainModel(config);
  const second = new TerrainModel(config);
  assert.deepEqual(first.rocks, second.rocks);
  assert.deepEqual(first.courseGritRocks, second.courseGritRocks);
  assert.ok(first.rocks.length >= 50);
  assert.equal(first.courseGritRocks.length, 70,
    'each of the ten Level 10 obstacles must receive seven physical sand-grit contacts');
  assert.ok(first.courseGritRocks.every(grain =>
    grain.kind === 'course-grit' && grain.h >= 0.012 && grain.h <= 0.032));
  assert.ok(first.rocks.every(rock => Math.abs(rock.x)-2*(rock.rx || rock.r) > 0.8),
    'photo-inspired side outcrops must not block the learned Level 10 centre route');
});

test('Model B gets a deterministic dense rock path without changing Model A terrain', () => {
  const config = new SimConfig({ terrainLevel: 10, abCourseEnabled: true, groundRMS: 0.03, seed: 42 });
  const first = new TerrainModel(config);
  const second = new TerrainModel(config);

  assert.equal(first.bPathRocks.length, 42);
  assert.deepEqual(first.bPathRocks, second.bPathRocks);
  assert.ok(first.bPathRocks.every(rock =>
    rock.kind === 'b-path' && Math.abs(rock.x) <= 0.775 && rock.h >= 0.035 && rock.h <= 0.09));

  const rock = first.bPathRocks[0];
  const fixedSurface = first.eval(rock.x, rock.y, 'fixed');
  const adaptiveSurface = first.eval(rock.x, rock.y, 'adaptive');
  assert.ok(adaptiveSurface.h >= fixedSurface.h+rock.h-1e-12,
    'B must collide with its path rock at the rendered crest');
  assert.equal(first.objectAt(rock.x, rock.y, 'adaptive'), 'b-rock-1');
  assert.notEqual(first.objectAt(rock.x, rock.y, 'fixed'), 'b-rock-1');

  const fixedView = first.forModel('fixed');
  const adaptiveView = first.forModel('adaptive');
  assert.equal(fixedView.eval(rock.x, rock.y).h, fixedSurface.h);
  assert.equal(adaptiveView.eval(rock.x, rock.y).h, adaptiveSurface.h);
});

test('Level 14 expedition builds a solid obstacle chain and corridor stone gauntlet', () => {
  const config = new SimConfig({
    terrainLevel: 14,
    experimentId: 14,
    abCourseEnabled: false,
    courseStartY: 10,
    targetGoalY: 800,
    courseGoalY: 800,
    seed: 42
  });
  const terrain = new TerrainModel(config);

  // Obstacle chain: Level-10-style solid crests spaced down the corridor,
  // bounded by the goal line.
  assert.ok(terrain.expeditionObstacles.length >= 25,
    `expected a dense obstacle chain, got ${terrain.expeditionObstacles.length}`);
  assert.ok(terrain.expeditionObstacles.every(obstacle =>
    obstacle.y > config.courseStartY && obstacle.y < config.targetGoalY));
  assert.ok(terrain.expeditionObstacles.every(obstacle =>
    ['jagged-rock', 'eroded-block', 'tilted-slab', 'broken-ridge'].includes(obstacle.type)));
  assert.ok(terrain.expeditionObstacles.every(obstacle =>
    obstacle.height > 0.4 && obstacle.radiusX >= 1.5 && obstacle.radiusY >= 1.6));

  // Every expedition obstacle contributes real, solid height at its crest.
  // Compare with the obstacle removed: craters or valleys may legally drop
  // the absolute elevation, but the crest bump itself must stay substantial.
  for (const obstacle of terrain.expeditionObstacles.slice(0, 5)) {
    const savedChain = terrain.expeditionObstacles;
    terrain.expeditionObstacles = [];
    const baseH = terrain.eval(obstacle.x, obstacle.y).h;
    terrain.expeditionObstacles = savedChain;
    const crestH = terrain.eval(obstacle.x, obstacle.y).h;
    assert.ok(crestH-baseH > 0.2,
      `obstacle ${obstacle.id} adds no solid crest (${(crestH-baseH).toFixed(3)} m)`);
  }

  // Stone gauntlet: dense corridor stones between start and goal.
  const gauntlet = terrain.rocks.filter(rock => rock.kind === 'expedition-gauntlet');
  assert.ok(gauntlet.length >= 500,
    `expected hundreds of corridor stones, got ${gauntlet.length}`);
  assert.ok(gauntlet.every(rock => rock.y > config.courseStartY && rock.y < config.targetGoalY));
  assert.ok(gauntlet.every(rock => Math.abs(rock.x) <= 2.5+1e-9));

  // Deterministic generation.
  const second = new TerrainModel(config);
  assert.deepEqual(terrain.expeditionObstacles, second.expeditionObstacles);
  assert.deepEqual(
    terrain.rocks.filter(r => r.kind === 'expedition-gauntlet'),
    second.rocks.filter(r => r.kind === 'expedition-gauntlet'));
});

test('Level 14 simulation registers the expedition chain as sequential checkpoints', () => {
  // Scaled-down corridor keeps physics cheap while exercising the same
  // tracker path as the full 800 m expedition. The chain starts 25 m past
  // the start line, so the goal must leave room for at least one crest.
  const config = new SimConfig({
    dt: 0.004,
    T_end: 0.05,
    missionDeadlineSeconds: 0.05,
    experimentId: 14,
    terrainLevel: 14,
    abCourseEnabled: false,
    courseStartY: 10,
    targetGoalY: 60,
    courseGoalY: 60,
    waypointSpacing: 10,
    enableDiagnosticsLog: false,
    monitoring: { enabled: false }
  });
  const rover = new SphericalRoverModel(config);
  const terrain = new TerrainModel(config);
  const simA = new Simulation(config, rover, terrain, 'fixed');
  const simB = new Simulation(config, rover, terrain, 'adaptive', null);

  // The chain generated between start+25 and goal-10.
  assert.ok(terrain.expeditionObstacles.length >= 1);
  // Both lanes track the expedition chain through ObstaclePassTracker.
  assert.ok(simA.obstacleTracker, 'Model A must track the expedition chain');
  assert.ok(simB.obstacleTracker, 'Model B must track the expedition chain');
  assert.equal(simA.expeditionCourse.obstacles, terrain.expeditionObstacles);

  // The adaptive controller perceives E01 as its first sequential target.
  const firstCrest = terrain.expeditionObstacles[0];
  const sense = simB.senseObstacleAhead(firstCrest.x, firstCrest.y-2, 1);
  assert.equal(sense.detected, true);
  assert.equal(sense.checkpoint, true);
  assert.equal(sense.obstacle.id, firstCrest.id);
});

const LEVEL14_FIELD_CONFIG = () => new SimConfig({
  terrainLevel: 14,
  experimentId: 14,
  abCourseEnabled: false,
  courseStartY: -450,
  targetGoalY: 450,
  courseGoalY: 450,
  seed: 42
});

test('Level 14 baseline Model A rolls toward the goal instead of slipping backward', { timeout: 180_000 }, () => {
  const config = LEVEL14_FIELD_CONFIG();
  config.dt = 0.004;
  config.groundRMS = 0.18;
  config.targetDestination = [0, 450];
  config.enableDiagnosticsLog = false;
  config.monitoring = { enabled: false };
  config.applyStandardPerformanceProfile();
  const simulation = new Simulation(
    config, new SphericalRoverModel(config), new TerrainModel(config), 'fixed');

  const startY = simulation.currentDiag.centroid[1];
  let maxY = startY;
  for (let step = 0; step < 4000; step++) {
    simulation.step();
    maxY = Math.max(maxY, simulation.currentDiag.centroid[1]);
  }

  // The rolling-grip solve must couple shell rotation into translation for
  // BOTH models: the baseline has a weaker gait (x0.82 actuation, no learned
  // scales), but it must still make forward progress toward y = +450.
  assert.ok(simulation.metrics.completedRolls >= 2,
    `expected repeated support-face rolls, got ${simulation.metrics.completedRolls}`);
  assert.ok(maxY >= startY+1.5,
    `baseline never advanced: started ${startY.toFixed(2)}, best ${maxY.toFixed(2)}`);
  assert.ok(simulation.currentDiag.centroid[1] >= startY+0.5,
    `baseline ended behind its start: ${simulation.currentDiag.centroid[1].toFixed(2)} vs ${startY.toFixed(2)}`);
});

test('Level 14 spans a true 1km x 1km field with dense size-classed scatter', () => {
  const config = LEVEL14_FIELD_CONFIG();
  const terrain = new TerrainModel(config);
  const FIELD_EDGE = 480;

  const byKind = kind => terrain.rocks.filter(rock => rock.kind === kind);

  // 🪨 Pebbles & small gravel: 5–15 cm, carpeting the entire surface.
  const pebbles = byKind('expedition-pebble');
  assert.ok(pebbles.length >= 6000, `expected ~12000 pebbles, got ${pebbles.length}`);
  assert.ok(pebbles.every(rock =>
    rock.rx >= 0.05-1e-9 && rock.rx <= 0.15+1e-9 &&
    Math.abs(rock.x) <= FIELD_EDGE && Math.abs(rock.y) <= FIELD_EDGE),
  'pebbles must be 5–15 cm and inside the rendered field');

  // 🗿 Medium & sharp angular rocks: 20–60 cm packed across the field; only
  // a thin trickle may sit inside the immediate mission lane band.
  const medium = byKind('expedition-rock');
  assert.ok(medium.length >= 2000, `expected ~4800 medium rocks, got ${medium.length}`);
  assert.ok(medium.every(rock =>
    rock.rx >= 0.20-1e-9 && rock.rx <= 0.60+1e-9 &&
    Math.abs(rock.y) <= FIELD_EDGE),
  'medium rocks must be 20–60 cm and inside the field');
  assert.ok(medium.some(rock => rock.sharp), 'expected sharp angular rocks');
  assert.ok(medium.some(rock => !rock.sharp), 'expected smooth medium rocks too');
  const mediumInLane = medium.filter(rock => Math.abs(rock.x) < 2.5).length;
  assert.ok(mediumInLane <= Math.max(12, 0.02*medium.length),
    `too many medium rocks block the lane band: ${mediumInLane}`);

  // 🏔️ Big boulders & monoliths (1.5–4 m): packed in tight clusters plus
  // solo giants, never blocking the mission corridor.
  const boulders = byKind('expedition-boulder');
  assert.ok(boulders.length >= 300, `expected ~1100 boulders, got ${boulders.length}`);
  assert.ok(boulders.every(rock =>
    rock.rx >= 1.5-1e-9 && rock.rx <= 4.0+1e-9 &&
    Math.abs(rock.x) >= 4.5-1e-9 && Math.abs(rock.y) <= FIELD_EDGE),
  'boulders must be 1.5–4 m and keep the travel lane clear');
  assert.ok(boulders.some(rock => rock.monolith), 'expected monolithic boulders');
  // Clustering: boulders share ground with close neighbours somewhere.
  let clusteredBoulders = 0;
  for (const boulder of boulders) {
    if (boulders.some(other => other !== boulder &&
      Math.hypot(other.x-boulder.x, other.y-boulder.y) < Math.max(boulder.rx, other.rx)*3)) {
      clusteredBoulders++;
    }
  }
  assert.ok(clusteredBoulders >= 40,
    `expected tightly packed boulder clusters, found only ${clusteredBoulders} adjacent stones`);

  // Cosmetic micro gravel (2–6 cm): dense visual clutter that stays out of
  // the physics solver but remains fully deterministic.
  assert.ok(Array.isArray(terrain.decorChips) && terrain.decorChips.length >= 9000,
    `expected ~9000 micro chips, got ${terrain.decorChips?.length}`);
  assert.ok(terrain.decorChips.every(chip =>
    chip.kind === 'expedition-chip' &&
    chip.rx >= 0.02-1e-9 && chip.rx <= 0.06+1e-9 && chip.h <= 0.04+1e-9));
  assert.ok(!terrain.scatterIds.has(terrain.decorChips[0]),
    'decor chips must stay outside the physical scatter index');

  // Every scatter element sits on solid ground: its analytic crest must rise
  // above the surrounding surface (no floating stones). The height field
  // reads through the spatial index, so removals must rebuild it first.
  for (const rock of [...pebbles.slice(0, 40), ...medium.slice(0, 20), ...boulders.slice(0, 10)]) {
    const saved = terrain.rocks;
    terrain.rocks = saved.filter(other => other !== rock);
    terrain.rebuildRockIndex();
    const baseH = terrain.evalBase(rock.x, rock.y).h;
    terrain.rocks = saved;
    terrain.rebuildRockIndex();
    const crestH = terrain.evalBase(rock.x, rock.y).h;
    assert.ok(crestH-baseH > 0.008,
      `stone at (${rock.x.toFixed(1)}, ${rock.y.toFixed(1)}) adds no height`);
  }

  // Deterministic generation across rebuilds.
  const second = new TerrainModel(config);
  assert.deepEqual(
    terrain.rocks.filter(r => r.kind.startsWith('expedition-')),
    second.rocks.filter(r => r.kind.startsWith('expedition-')));
});

test('Level 14 sand and dust beds modulate grip and drag through sandAt', () => {
  const terrain = new TerrainModel(LEVEL14_FIELD_CONFIG());
  assert.ok(terrain.sandPatches.length >= 20, 'expected ~22 sand/dust beds');
  assert.ok(terrain.sandPatches.every(patch =>
    patch.frictionScale > 0.5 && patch.frictionScale < 1 && patch.dragScale > 1));

  let centresTested = 0;
  for (const patch of terrain.sandPatches) {
    const centre = terrain.sandAt(patch.x, patch.y);
    assert.equal(centre.inSand, true);
    assert.ok(centre.frictionScale < 1, 'loose sand must reduce Coulomb grip');
    assert.ok(centre.dragScale > 1, 'sand beds must add viscous drag');
    centresTested++;
  }
  assert.ok(centresTested >= 20);

  // Far away from every patch (patch radii are bounded well under 100 m)
  // the surface reads as normal compact regolith.
  const outside = terrain.sandAt(5000, 5000);
  assert.deepEqual(outside, { inSand: false, frictionScale: 1, dragScale: 1, depth: 0 });
});

test('sharp angular stones produce steeper contact normals than round ones', () => {
  const config = new SimConfig({ terrainLevel: 1, groundRMS: 0.001, seed: 7 });
  const terrain = new TerrainModel(config);
  const roundRock = { x: -3, y: 8, rx: 0.3, ry: 0.3, h: 0.2, sharp: false };
  const sharpRock = { x: 3, y: 8, rx: 0.3, ry: 0.3, h: 0.2, sharp: true };
  terrain.rocks.push(roundRock, sharpRock);
  terrain.rebuildRockIndex();

  const probeDistance = 0.12; // within the steep crest zone
  const roundSlope = Math.abs(terrain.eval(roundRock.x+probeDistance, roundRock.y).dhdx);
  const sharpSlope = Math.abs(terrain.eval(sharpRock.x+probeDistance, sharpRock.y).dhdx);
  assert.ok(sharpSlope > roundSlope*1.8,
    `sharp slope ${sharpSlope.toFixed(3)} should far exceed round ${roundSlope.toFixed(3)}`);
});

test('the scatter spatial hash returns every stone the height field can see', () => {
  const terrain = new TerrainModel(LEVEL14_FIELD_CONFIG());
  const reach = terrain.maxRockReach;
  const exactContributors = (x, y) => terrain.scatterList
    .filter(rock => {
      const rx = Math.max(0.04, rock.rx || rock.r || 0.2);
      const ry = Math.max(0.04, rock.ry || rock.r || 0.2);
      const nx = (x-rock.x)/rx;
      const ny = (y-rock.y)/ry;
      return nx*nx+ny*ny < 4; // evaluateRockOutcrop cutoff
    })
    .map(rock => rock._sq)
    .sort((a, b) => a-b);
  const probes = [
    [0, 0], [0.4, -217.3], [-2.2, 88.8], [130, 240], [-455, -460],
    [1.1, 449], [17, -13], [-96.5, 12.25], [-185.08, -76.02], [3.1, 210]
  ];
  for (const [x, y] of probes) {
    const queried = terrain.scatterQuery(x, y, reach, true)
      .map(rock => rock._sq).sort((a, b) => a-b);
    // Superset property: every stone whose footprint covers the probe must
    // be returned (extra footprint-overlapping neighbours are allowed and
    // contribute exactly zero through evaluateRockOutcrop's cutoff).
    const missing = exactContributors(x, y)
      .filter(id => !queried.includes(id));
    assert.equal(missing.length, 0,
      `grid query missed ${missing.length} contributors at (${x}, ${y})`);

    // The fixed lane excludes Model B's private path stones everywhere.
    const fixedQueried = terrain.scatterQuery(x, y, reach, false);
    assert.ok(fixedQueried.every(rock => rock.kind !== 'b-path'));

    // Deterministic repeat.
    const again = terrain.scatterQuery(x, y, reach, true)
      .map(rock => rock._sq).sort((a, b) => a-b);
    assert.deepEqual(queried, again);
  }
});

test('Level 14 multi-octave noise field shapes dunes, ridges and terraced ledges', () => {
  const terrain = new TerrainModel(LEVEL14_FIELD_CONFIG());
  assert.ok(terrain.noiseFields, 'open-world expedition must define noise fields');
  assert.ok(terrain.noiseFields.dunes.octaves >= 3);
  assert.ok(terrain.noiseFields.ridges.octaves >= 4);

  // Heights stay finite with bounded slopes across the whole km².
  let hMin = Infinity, hMax = -Infinity;
  for (let i = 0; i < 4000; i++) {
    const x = (i*7919 % 997)-498;   // deterministic quasi-random sweep
    const y = ((i*104729) % 997)-498;
    const s = terrain.eval(x, y);
    assert.ok([s.h, s.dhdx, s.dhdy].every(Number.isFinite));
    hMin = Math.min(hMin, s.h);
    hMax = Math.max(hMax, s.h);
  }
  // Dune basins + ridge crests + plateau ledges give real vertical relief.
  assert.ok(hMax-hMin > 8, `expected multi-metre relief, got ${(hMax-hMin).toFixed(1)} m`);

  // The noise layer eases out in the mission lane: sampling two points that
  // share y but sit inside vs outside the lane, the lane point must not see
  // the full procedural amplitude. Compare against the analytic layer alone
  // by probing the derivative of the noise contribution indirectly — the
  // corridor slope budget stays under the impassable cliff threshold.
  let maxCorridorSlope = 0;
  for (let y = -440; y <= 440; y += 11) {
    const s = terrain.eval(0, y);
    maxCorridorSlope = Math.max(maxCorridorSlope, Math.hypot(s.dhdx, s.dhdy));
  }
  assert.ok(maxCorridorSlope < 6.0,
    `lane centre must remain traversable (noise faded + stone crests), got slope ${maxCorridorSlope.toFixed(2)}`);

  // Deterministic under seed.
  const again = new TerrainModel(LEVEL14_FIELD_CONFIG());
  assert.equal(
    JSON.stringify(terrain.eval(-217.5, 88.25)),
    JSON.stringify(again.eval(-217.5, 88.25)));
});
