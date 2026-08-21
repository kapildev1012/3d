/**
 * Deterministic 50 m A-vs-B obstacle experiment.
 *
 * Coordinates are expressed in metres. The first 10 m are an unmeasured
 * settling/approach zone; scoring starts at y=10 m and ends at y=60 m.
 */

const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export const AB_COURSE = Object.freeze({
  roverDiameter: 1.0,
  obstacleHeightRatio: 0.50,
  minimumLateralRadiusRatio: 1.50,
  minimumLongitudinalRadiusRatio: 1.60,
  minY: 0,
  startY: 10,
  goalY: 60,
  maxY: 70,
  measuredLength: 50,
  width: 8,
  obstacleCorridorHalfWidth: 0.42,
  detectionDistance: 2.40,
  maxRetries: 2,
  obstacles: Object.freeze([
    { id: 'O01', type: 'jagged-rock',  difficulty: 'small',  x:  0.04, y: 13.4, radiusX: 0.58, radiusY: 0.48, height: 0.50, yaw:  0.19, skewX:  0.23, skewY: -0.14, lobe:  0.12, twist: -0.16 },
    { id: 'O02', type: 'eroded-block', difficulty: 'medium', x: -0.20, y: 17.8, radiusX: 0.66, radiusY: 0.52, height: 0.50, yaw: -0.27, skewX: -0.17, skewY:  0.21, lobe: -0.14, twist:  0.18 },
    { id: 'O03', type: 'tilted-slab',  difficulty: 'medium', x:  0.19, y: 22.6, radiusX: 0.74, radiusY: 0.57, height: 0.50, yaw:  0.31, skewX:  0.12, skewY:  0.25, lobe:  0.16, twist:  0.13 },
    { id: 'O04', type: 'broken-ridge', difficulty: 'large',  x: -0.15, y: 27.3, radiusX: 0.82, radiusY: 0.64, height: 0.50, yaw: -0.23, skewX: -0.25, skewY: -0.11, lobe: -0.12, twist: -0.19 },
    { id: 'O05', type: 'jagged-rock',  difficulty: 'small',  x:  0.12, y: 32.3, radiusX: 0.62, radiusY: 0.50, height: 0.50, yaw:  0.14, skewX:  0.20, skewY:  0.18, lobe: -0.15, twist:  0.16 },
    { id: 'O06', type: 'eroded-block', difficulty: 'large',  x: -0.22, y: 37.1, radiusX: 0.86, radiusY: 0.66, height: 0.50, yaw:  0.28, skewX: -0.15, skewY: -0.24, lobe:  0.14, twist:  0.20 },
    { id: 'O07', type: 'tilted-slab',  difficulty: 'medium', x:  0.18, y: 42.2, radiusX: 0.76, radiusY: 0.59, height: 0.50, yaw: -0.32, skewX:  0.26, skewY: -0.10, lobe:  0.13, twist: -0.17 },
    { id: 'O08', type: 'broken-ridge', difficulty: 'large',  x: -0.13, y: 47.1, radiusX: 0.90, radiusY: 0.70, height: 0.50, yaw:  0.22, skewX: -0.22, skewY:  0.16, lobe: -0.16, twist: -0.14 },
    { id: 'O09', type: 'jagged-rock',  difficulty: 'medium', x:  0.16, y: 52.0, radiusX: 0.70, radiusY: 0.55, height: 0.50, yaw: -0.18, skewX:  0.16, skewY:  0.24, lobe:  0.15, twist:  0.19 },
    { id: 'O10', type: 'eroded-block', difficulty: 'large',  x: -0.10, y: 57.0, radiusX: 0.88, radiusY: 0.68, height: 0.50, yaw:  0.26, skewX: -0.24, skewY: -0.15, lobe: -0.13, twist:  0.17 }
  ].map(Object.freeze))
});

export function createABCourse(roverDiameter = AB_COURSE.roverDiameter) {
  const obstacleHeight = AB_COURSE.obstacleHeightRatio*roverDiameter;
  const minimumLateralRadius = AB_COURSE.minimumLateralRadiusRatio*roverDiameter;
  const minimumLongitudinalRadius = AB_COURSE.minimumLongitudinalRadiusRatio*roverDiameter;
  return {
    ...AB_COURSE,
    roverDiameter,
    obstacles: AB_COURSE.obstacles.map(obstacle => ({
      ...obstacle,
      height: obstacleHeight,
      radiusX: Math.max(obstacle.radiusX, minimumLateralRadius),
      radiusY: Math.max(obstacle.radiusY, minimumLongitudinalRadius)
    }))
  };
}

/** Smooth, compact, asymmetrically warped obstacle with analytic derivatives. */
export function evaluateCourseObstacle(obstacle, x, y) {
  const cosYaw = Math.cos(obstacle.yaw || 0);
  const sinYaw = Math.sin(obstacle.yaw || 0);
  const dx = x-obstacle.x;
  const dy = y-obstacle.y;
  const u = cosYaw*dx+sinYaw*dy;
  const v = -sinYaw*dx+cosYaw*dy;
  const normalizedX = u/obstacle.radiusX;
  const normalizedY = v/obstacle.radiusY;
  const baseR2 = normalizedX*normalizedX+normalizedY*normalizedY;
  const rawWarp = 1
    +(obstacle.skewX || 0)*normalizedX
    +(obstacle.skewY || 0)*normalizedY
    +(obstacle.lobe || 0)*(normalizedX*normalizedX-normalizedY*normalizedY)
    +(obstacle.twist || 0)*normalizedX*normalizedY;
  const warp = Math.max(0.42, rawWarp);
  const r2 = baseR2*warp;
  if (r2 >= 1) return { h: 0, dhdx: 0, dhdy: 0 };

  // The polynomial warp breaks circular/elliptical symmetry while retaining
  // a C1-continuous edge. Distinct exponents create eroded blocks, slabs,
  // jagged rocks, and broken ridges without collision discontinuities.
  const derivativeWarpX = rawWarp <= 0.42 ? 0 : (
    (obstacle.skewX || 0)
    +2*(obstacle.lobe || 0)*normalizedX
    +(obstacle.twist || 0)*normalizedY
  )/obstacle.radiusX;
  const derivativeWarpY = rawWarp <= 0.42 ? 0 : (
    (obstacle.skewY || 0)
    -2*(obstacle.lobe || 0)*normalizedY
    +(obstacle.twist || 0)*normalizedX
  )/obstacle.radiusY;
  const derivativeBaseX = 2*normalizedX/obstacle.radiusX;
  const derivativeBaseY = 2*normalizedY/obstacle.radiusY;
  const derivativeR2U = derivativeBaseX*warp+baseR2*derivativeWarpX;
  const derivativeR2V = derivativeBaseY*warp+baseR2*derivativeWarpY;
  const core = 1-r2;
  const exponent = obstacle.type === 'eroded-block' ? 0.58
    : obstacle.type === 'tilted-slab' ? 0.82
      : obstacle.type === 'broken-ridge' ? 1.18 : 0.96;
  const profile = Math.pow(core, exponent+1);
  const dProfileDr2 = -(exponent+1)*Math.pow(core, exponent);
  const dhdu = obstacle.height*dProfileDr2*derivativeR2U;
  const dhdv = obstacle.height*dProfileDr2*derivativeR2V;
  return {
    h: obstacle.height*profile,
    dhdx: dhdu*cosYaw-dhdv*sinYaw,
    dhdy: dhdu*sinYaw+dhdv*cosYaw
  };
}

export function senseCourseObstacle(course, x, y, directionY = 1) {
  const direction = directionY >= 0 ? 1 : -1;
  let nearest = null;
  for (const obstacle of course.obstacles) {
    const ahead = (obstacle.y-y)*direction;
    if (ahead < -obstacle.radiusY || ahead > course.detectionDistance) continue;
    const lateralError = x-obstacle.x;
    if (Math.abs(lateralError) > obstacle.radiusX+0.85) continue;
    if (!nearest || ahead < nearest.distance) {
      nearest = {
        detected: true,
        obstacle,
        distance: Math.max(0, ahead-obstacle.radiusY),
        height: obstacle.height,
        lateralError,
        steerSign: lateralError >= 0 ? 1 : -1
      };
    }
  }
  return nearest || { detected: false, obstacle: null, distance: Infinity, height: 0, lateralError: 0, steerSign: 0 };
}

/** Minimum-necessary, obstacle-proportional cable deformation. */
export function obstacleActuation(obstacle, maximumDelta = 0.12) {
  if (!obstacle) return { delta: 0, cableCount: 0, relaxationRatio: 0 };
  const difficultySeverity = { small: 0.25, medium: 0.60, large: 1.0 }[obstacle.difficulty] ?? 0.60;
  const heightScale = clamp(obstacle.height/(AB_COURSE.obstacleHeightRatio*AB_COURSE.roverDiameter), 0, 1);
  const severity = difficultySeverity*heightScale;
  return {
    delta: clamp(0.045+0.065*severity, 0, maximumDelta),
    cableCount: severity < 0.34 ? 3 : severity < 0.72 ? 4 : 6,
    relaxationRatio: 0.15+0.20*severity
  };
}

export class ObstaclePassTracker {
  constructor(course = createABCourse(), modelType = 'fixed') {
    this.course = course;
    this.modelType = modelType;
    this.nextCheckpointIndex = 0;
    this.records = new Map(course.obstacles.map(obstacle => [obstacle.id, {
      id: obstacle.id,
      difficulty: obstacle.difficulty,
      status: 'pending',
      attempts: 0,
      minCenterOffset: Infinity,
      maxRelativeElevation: 0,
      enteredFootprint: false,
      crestReached: false,
      crestReachedAt: null,
      minimumCrestDistance: Infinity,
      maximumCrestElevation: 0
    }]));
  }

  currentCheckpoint() {
    return this.course.obstacles[this.nextCheckpointIndex] || null;
  }

  update({ x, y, z, baseHeight = 0, time = null }) {
    // Model B must physically reach each obstacle crest in order. A crest is
    // accepted only inside the obstacle's central crest zone with the rover
    // centroid elevated by the rock height; merely going around or crossing
    // the obstacle's Y coordinate cannot advance the checkpoint.
    const checkpoint = this.currentCheckpoint();
    if (checkpoint) {
      const relativeElevation = z-baseHeight;
      const minimumCrestElevation = 0.38*this.course.roverDiameter+0.58*checkpoint.height;
      const record = this.records.get(checkpoint.id);
      const crestDistance = Math.hypot(
        (x-checkpoint.x)/(0.90*checkpoint.radiusX),
        (y-checkpoint.y)/(0.72*checkpoint.radiusY)
      );
      record.minimumCrestDistance = Math.min(record.minimumCrestDistance, crestDistance);
      if (crestDistance <= 1) {
        record.maximumCrestElevation = Math.max(record.maximumCrestElevation, relativeElevation);
      }
      if (record.maximumCrestElevation >= minimumCrestElevation
        && record.minimumCrestDistance <= 1) {
        record.crestReached = true;
        record.crestReachedAt = Number.isFinite(time) ? time : record.crestReachedAt;
        record.status = 'crest';
        this.nextCheckpointIndex += 1;
      }
    }

    for (const obstacle of this.course.obstacles) {
      const record = this.records.get(obstacle.id);
      if (record.status === 'over' || record.status === 'around') continue;
      const lateral = Math.abs(x-obstacle.x);
      const longitudinal = Math.abs(y-obstacle.y);
      if (longitudinal <= obstacle.radiusY+0.50) {
        record.minCenterOffset = Math.min(record.minCenterOffset, lateral);
        record.maxRelativeElevation = Math.max(record.maxRelativeElevation, z-baseHeight);
        if (lateral <= obstacle.radiusX) {
          if (!record.enteredFootprint) record.attempts += 1;
          record.enteredFootprint = true;
          record.status = record.crestReached ? 'crest' : 'engaged';
        }
      }
      if (y > obstacle.y+obstacle.radiusY+0.50) {
        const centerBand = Math.max(0.20, 0.95*obstacle.radiusX);
        if (record.enteredFootprint && record.minCenterOffset <= centerBand && record.crestReached) {
          record.status = 'over';
        } else {
          record.status = 'around';
        }
      }
    }
    return this.summary();
  }

  markRetry(obstacleId) {
    const record = this.records.get(obstacleId);
    if (record) {
      record.attempts += 1;
      record.status = 'retry';
    }
  }

  summary() {
    const records = [...this.records.values()];
    const over = records.filter(record => record.status === 'over').length;
    const around = records.filter(record => record.status === 'around').length;
    const retries = records.reduce((sum, record) => sum+Math.max(0, record.attempts-1), 0);
    return {
      total: records.length,
      over,
      around,
      retries,
      checkpointsReached: this.nextCheckpointIndex,
      nextCheckpointId: this.currentCheckpoint()?.id || null,
      allCheckpointsReached: this.nextCheckpointIndex >= this.course.obstacles.length,
      pending: records.filter(record => !['over', 'around'].includes(record.status)).length,
      bypassViolations: this.modelType === 'adaptive' ? around : 0,
      records: records.map(record => ({ ...record }))
    };
  }
}
