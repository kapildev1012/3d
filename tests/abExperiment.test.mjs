import test from 'node:test';
import assert from 'node:assert/strict';
import {
  createABCourse,
  evaluateCourseObstacle,
  senseCourseObstacle,
  obstacleActuation,
  ObstaclePassTracker
} from '../js/abExperiment.js';

test('course is deterministic and spans the prescribed 50 m measured section', () => {
  const courseA = createABCourse();
  const courseB = createABCourse();
  assert.deepEqual(courseA, courseB);
  assert.equal(courseA.startY, 10);
  assert.equal(courseA.goalY, 60);
  assert.equal(courseA.goalY-courseA.startY, 50);
  assert.equal(courseA.obstacles.length, 12);
  assert.ok(courseA.obstacles.every(obstacle => obstacle.y > 10 && obstacle.y < 60));
  for (let bandStart = 10; bandStart < 60; bandStart += 10) {
    const count = courseA.obstacles.filter(obstacle => obstacle.y >= bandStart && obstacle.y < bandStart+10).length;
    assert.ok(count >= 2 && count <= 3, `expected 2–3 obstacles in ${bandStart}–${bandStart+10} m, got ${count}`);
  }
});

test('obstacle surface is smooth, compact and reaches its specified crest', () => {
  const obstacle = createABCourse().obstacles[4];
  const crest = evaluateCourseObstacle(obstacle, obstacle.x, obstacle.y);
  assert.ok(Math.abs(crest.h-obstacle.height) < 1e-12);
  assert.ok(Math.abs(crest.dhdx) < 1e-12);
  assert.ok(Math.abs(crest.dhdy) < 1e-12);
  assert.deepEqual(evaluateCourseObstacle(obstacle, obstacle.x+2*obstacle.radiusX, obstacle.y), { h: 0, dhdx: 0, dhdy: 0 });
});

test('sensing selects the next obstacle and proportional adaptation stays bounded', () => {
  const course = createABCourse();
  const first = course.obstacles[0];
  const sensed = senseCourseObstacle(course, first.x, first.y-1.0, 1);
  assert.equal(sensed.obstacle.id, first.id);
  const small = obstacleActuation(course.obstacles.find(obstacle => obstacle.difficulty === 'small'));
  const large = obstacleActuation(course.obstacles.find(obstacle => obstacle.difficulty === 'large'));
  assert.ok(large.delta > small.delta);
  assert.ok(large.cableCount > small.cableCount);
  assert.ok(large.delta <= 0.12);
  assert.ok(large.cableCount < 12, 'adaptation must never relax/actuate the entire network');
});

test('OVER and AROUND are explicitly distinguished; B around is a violation', () => {
  const course = createABCourse();
  const obstacle = course.obstacles[0];
  const overTracker = new ObstaclePassTracker(course, 'adaptive');
  overTracker.update({ x: obstacle.x, y: obstacle.y, z: 0.65, baseHeight: 0 });
  const over = overTracker.update({ x: obstacle.x, y: obstacle.y+obstacle.radiusY+0.6, z: 0.55, baseHeight: 0 });
  assert.equal(over.records[0].status, 'over');
  assert.equal(over.bypassViolations, 0);

  const aroundTracker = new ObstaclePassTracker(course, 'adaptive');
  aroundTracker.update({ x: obstacle.x+obstacle.radiusX+0.8, y: obstacle.y, z: 0.55, baseHeight: 0 });
  const around = aroundTracker.update({ x: obstacle.x+obstacle.radiusX+0.8, y: obstacle.y+obstacle.radiusY+0.6, z: 0.55, baseHeight: 0 });
  assert.equal(around.records[0].status, 'around');
  assert.equal(around.bypassViolations, 1);
});

test('small, medium and large obstacle profiles remain finite and traversable', () => {
  const course = createABCourse();
  for (const difficulty of ['small', 'medium', 'large']) {
    const obstacles = course.obstacles.filter(obstacle => obstacle.difficulty === difficulty);
    assert.ok(obstacles.length > 0);
    for (const obstacle of obstacles) {
      for (let fraction = -1; fraction <= 1; fraction += 0.1) {
        const sample = evaluateCourseObstacle(obstacle, obstacle.x, obstacle.y+fraction*obstacle.radiusY);
        assert.ok([sample.h, sample.dhdx, sample.dhdy].every(Number.isFinite));
        assert.ok(sample.h >= 0 && sample.h <= obstacle.height+1e-12);
      }
    }
  }
});

test('recovery attempts are counted without changing obstacle geometry', () => {
  const course = createABCourse();
  const tracker = new ObstaclePassTracker(course, 'adaptive');
  const snapshot = JSON.stringify(course.obstacles);
  tracker.markRetry('O01');
  tracker.markRetry('O01');
  assert.equal(tracker.summary().retries, 1);
  assert.equal(JSON.stringify(course.obstacles), snapshot);
});

test('synthetic full-course centreline traversal validates all 12 as OVER', () => {
  const course = createABCourse();
  const tracker = new ObstaclePassTracker(course, 'adaptive');
  for (const obstacle of course.obstacles) {
    tracker.update({ x: obstacle.x, y: obstacle.y, z: 0.55+obstacle.height, baseHeight: 0 });
    tracker.update({
      x: obstacle.x,
      y: obstacle.y+obstacle.radiusY+0.55,
      z: 0.55,
      baseHeight: 0
    });
  }
  const result = tracker.summary();
  assert.equal(result.over, 12);
  assert.equal(result.around, 0);
  assert.equal(result.bypassViolations, 0);
});
