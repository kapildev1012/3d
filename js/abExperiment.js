/**
 * Deterministic 50 m A-vs-B obstacle experiment.
 *
 * Coordinates are expressed in metres. The first 10 m are an unmeasured
 * settling/approach zone; scoring starts at y=10 m and ends at y=60 m.
 */

const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export const AB_COURSE = Object.freeze({
  minY: 0,
  startY: 10,
  goalY: 60,
  maxY: 70,
  measuredLength: 50,
  width: 8,
  obstacleCorridorHalfWidth: 0.42,
  detectionDistance: 1.65,
  maxRetries: 2,
  obstacles: Object.freeze([
    { id: 'O01', type: 'rounded-rock', difficulty: 'small',  x:  0.00, y: 13.5, radiusX: 0.42, radiusY: 0.34, height: 0.08, yaw:  0.10 },
    { id: 'O02', type: 'low-block',    difficulty: 'small',  x: -0.16, y: 17.2, radiusX: 0.48, radiusY: 0.38, height: 0.10, yaw: -0.18 },
    { id: 'O03', type: 'mound',        difficulty: 'medium', x:  0.18, y: 21.8, radiusX: 0.58, radiusY: 0.50, height: 0.15, yaw:  0.06 },
    { id: 'O04', type: 'ramp',         difficulty: 'medium', x: -0.12, y: 25.7, radiusX: 0.62, radiusY: 0.58, height: 0.18, yaw: -0.08 },
    { id: 'O05', type: 'rounded-rock', difficulty: 'medium', x:  0.20, y: 29.4, radiusX: 0.54, radiusY: 0.46, height: 0.17, yaw:  0.20 },
    { id: 'O06', type: 'low-block',    difficulty: 'large',  x: -0.18, y: 34.1, radiusX: 0.66, radiusY: 0.56, height: 0.23, yaw:  0.12 },
    { id: 'O07', type: 'mound',        difficulty: 'small',  x:  0.08, y: 38.2, radiusX: 0.49, radiusY: 0.42, height: 0.11, yaw: -0.12 },
    { id: 'O08', type: 'ramp',         difficulty: 'large',  x:  0.17, y: 42.5, radiusX: 0.72, radiusY: 0.68, height: 0.26, yaw:  0.08 },
    { id: 'O09', type: 'rounded-rock', difficulty: 'medium', x: -0.20, y: 46.0, radiusX: 0.58, radiusY: 0.50, height: 0.19, yaw: -0.16 },
    { id: 'O10', type: 'low-block',    difficulty: 'small',  x:  0.14, y: 50.1, radiusX: 0.46, radiusY: 0.38, height: 0.10, yaw:  0.05 },
    { id: 'O11', type: 'mound',        difficulty: 'large',  x: -0.10, y: 54.2, radiusX: 0.70, radiusY: 0.62, height: 0.25, yaw:  0.14 },
    { id: 'O12', type: 'ramp',         difficulty: 'medium', x:  0.05, y: 57.6, radiusX: 0.61, radiusY: 0.52, height: 0.18, yaw: -0.06 }
  ].map(Object.freeze))
});

export function createABCourse() {
  return {
    ...AB_COURSE,
    obstacles: AB_COURSE.obstacles.map(obstacle => ({ ...obstacle }))
  };
}

/** Smooth, compact obstacle surface with analytic first derivatives. */
export function evaluateCourseObstacle(obstacle, x, y) {
  const cosYaw = Math.cos(obstacle.yaw || 0);
  const sinYaw = Math.sin(obstacle.yaw || 0);
  const dx = x-obstacle.x;
  const dy = y-obstacle.y;
  const u = cosYaw*dx+sinYaw*dy;
  const v = -sinYaw*dx+cosYaw*dy;
  const r2 = (u*u)/(obstacle.radiusX*obstacle.radiusX)+(v*v)/(obstacle.radiusY*obstacle.radiusY);
  if (r2 >= 1) return { h: 0, dhdx: 0, dhdy: 0 };

  // C1-continuous footprint. Type changes the crest without introducing a
  // collision discontinuity that would cause bounce or solver jitter.
  const core = 1-r2;
  const exponent = obstacle.type === 'low-block' ? 0.65 : obstacle.type === 'ramp' ? 1.35 : 1.0;
  const profile = Math.pow(core, exponent+1);
  const dProfileDr2 = -(exponent+1)*Math.pow(core, exponent);
  const dr2du = 2*u/(obstacle.radiusX*obstacle.radiusX);
  const dr2dv = 2*v/(obstacle.radiusY*obstacle.radiusY);
  const dhdu = obstacle.height*dProfileDr2*dr2du;
  const dhdv = obstacle.height*dProfileDr2*dr2dv;
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
  const severity = clamp((obstacle.height-0.06)/0.22, 0, 1);
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
    this.records = new Map(course.obstacles.map(obstacle => [obstacle.id, {
      id: obstacle.id,
      difficulty: obstacle.difficulty,
      status: 'pending',
      attempts: 0,
      minCenterOffset: Infinity,
      maxRelativeElevation: 0,
      enteredFootprint: false
    }]));
  }

  update({ x, y, z, baseHeight = 0 }) {
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
          record.status = 'engaged';
        }
      }
      if (y > obstacle.y+obstacle.radiusY+0.50) {
        const centerBand = Math.max(0.20, 0.72*obstacle.radiusX);
        const climbed = record.maxRelativeElevation >= 0.45*obstacle.height;
        if (record.enteredFootprint && record.minCenterOffset <= centerBand && climbed) {
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
      pending: records.filter(record => !['over', 'around'].includes(record.status)).length,
      bypassViolations: this.modelType === 'adaptive' ? around : 0,
      records: records.map(record => ({ ...record }))
    };
  }
}

