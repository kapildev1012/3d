import test from 'node:test';
import assert from 'node:assert/strict';
import { LEVEL10_PERFORMANCE, SimConfig, TerrainModel } from '../js/simEngine.js';

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
