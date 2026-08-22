/**
 * Persistent run-to-run route learning for the Level 10 locomotion mission.
 *
 * The learner is deliberately outside the 500 Hz physics solver. It observes
 * controller-rate summaries, remembers 2 m GPS-like course segments, and
 * applies small bounded gradient updates only after a complete attempt. This
 * keeps one attempt physically deterministic while allowing later attempts to
 * improve from slow, slipping, misaligned, or failed sections.
 */

const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value));
const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

export const ROUTE_LEARNING_DEFAULTS = Object.freeze({
  version: 2,
  deadlineSeconds: 120,
  courseStartY: 10,
  courseGoalY: 60,
  segmentLength: 2,
  learningRate: 0.08,
  minimumSpeedScale: 0.90,
  maximumSpeedScale: 1.22,
  minimumTorqueScale: 0.92,
  maximumTorqueScale: 1.30,
  minimumTractionScale: 0.90,
  maximumTractionScale: 1.24,
  maximumWaypointOffset: 0.24
});

export const LEVEL14_LEARNING_DEFAULTS = Object.freeze({
  version: 4,
  versionNote: 'open-world 1km x 1km expanse, start -450 m to goal +450 m',
  deadlineSeconds: 5000,
  courseStartY: -450,
  courseGoalY: 450,
  segmentLength: 5,
  learningRate: 0.02,
  minimumSpeedScale: 0.85,
  maximumSpeedScale: 1.30,
  minimumTorqueScale: 0.85,
  maximumTorqueScale: 1.40,
  minimumTractionScale: 0.85,
  maximumTractionScale: 1.35,
  maximumWaypointOffset: 0.50
});

const freshSegment = (index, settings) => ({
  index,
  startY: settings.courseStartY+index*settings.segmentLength,
  endY: Math.min(settings.courseGoalY, settings.courseStartY+(index+1)*settings.segmentLength),
  visits: 0,
  speedScale: 1,
  torqueScale: 1,
  tractionScale: 1,
  waypointX: 0,
  slipEMA: 0,
  rollingErrorEMA: 0,
  gradeEMA: 0,
  progressRateEMA: 0,
  energyEMA: 0,
  checkpointReached: false
});

export class AdaptiveRouteLearner {
  constructor(options = {}, savedState = null) {
    this.settings = { ...ROUTE_LEARNING_DEFAULTS, ...options };
    const distance = Math.max(0.1, this.settings.courseGoalY-this.settings.courseStartY);
    this.segmentCount = Math.max(1, Math.ceil(distance/this.settings.segmentLength));
    this.runCount = 0;
    this.wins = 0;
    this.losses = 0;
    this.bestTime = null;
    this.lastRun = null;
    this.revision = 0;
    this.global = {
      speedScale: 1,
      torqueScale: 1,
      tractionScale: 1,
      alignmentScale: 1
    };
    this.segments = Array.from({ length: this.segmentCount }, (_, index) =>
      freshSegment(index, this.settings));
    this.restore(savedState);
    this.beginRun();
  }

  restore(savedState) {
    if (!savedState || savedState.version !== this.settings.version) return false;
    this.runCount = Math.max(0, Math.floor(finite(savedState.runCount)));
    this.wins = Math.max(0, Math.floor(finite(savedState.wins)));
    this.losses = Math.max(0, Math.floor(finite(savedState.losses)));
    this.bestTime = Number.isFinite(savedState.bestTime) ? savedState.bestTime : null;
    this.lastRun = savedState.lastRun && typeof savedState.lastRun === 'object'
      ? { ...savedState.lastRun } : null;
    const global = savedState.global || {};
    this.global.speedScale = clamp(finite(global.speedScale, 1),
      this.settings.minimumSpeedScale, this.settings.maximumSpeedScale);
    this.global.torqueScale = clamp(finite(global.torqueScale, 1),
      this.settings.minimumTorqueScale, this.settings.maximumTorqueScale);
    this.global.tractionScale = clamp(finite(global.tractionScale, 1),
      this.settings.minimumTractionScale, this.settings.maximumTractionScale);
    this.global.alignmentScale = clamp(finite(global.alignmentScale, 1), 0.90, 1.25);
    if (Array.isArray(savedState.segments)) {
      for (let index = 0; index < Math.min(this.segments.length, savedState.segments.length); index++) {
        const source = savedState.segments[index] || {};
        const target = this.segments[index];
        target.visits = Math.max(0, Math.floor(finite(source.visits)));
        target.speedScale = clamp(finite(source.speedScale, 1),
          this.settings.minimumSpeedScale, this.settings.maximumSpeedScale);
        target.torqueScale = clamp(finite(source.torqueScale, 1),
          this.settings.minimumTorqueScale, this.settings.maximumTorqueScale);
        target.tractionScale = clamp(finite(source.tractionScale, 1),
          this.settings.minimumTractionScale, this.settings.maximumTractionScale);
        target.waypointX = clamp(finite(source.waypointX),
          -this.settings.maximumWaypointOffset, this.settings.maximumWaypointOffset);
        target.slipEMA = Math.max(0, finite(source.slipEMA));
        target.rollingErrorEMA = Math.max(0, finite(source.rollingErrorEMA));
        target.gradeEMA = Math.max(0, finite(source.gradeEMA));
        target.progressRateEMA = Math.max(0, finite(source.progressRateEMA));
        target.energyEMA = Math.max(0, finite(source.energyEMA));
        target.checkpointReached = Boolean(source.checkpointReached);
      }
    }
    return true;
  }

  resetLearning() {
    this.runCount = 0;
    this.wins = 0;
    this.losses = 0;
    this.bestTime = null;
    this.lastRun = null;
    this.global = { speedScale: 1, torqueScale: 1, tractionScale: 1, alignmentScale: 1 };
    this.segments = Array.from({ length: this.segmentCount }, (_, index) =>
      freshSegment(index, this.settings));
    this.revision++;
    this.beginRun();
  }

  beginRun() {
    this.activeRun = {
      started: false,
      lastTime: null,
      lastY: null,
      segments: new Map()
    };
    return this.snapshot();
  }

  segmentIndex(y) {
    return clamp(Math.floor((finite(y)-this.settings.courseStartY)/this.settings.segmentLength),
      0, this.segmentCount-1);
  }

  commandAt({ x = 0, y = this.settings.courseStartY, grade = 0, obstacle = null } = {}) {
    const segment = this.segments[this.segmentIndex(y)];
    const uphill = clamp(finite(grade), 0, 0.45);
    const obstacleX = Number.isFinite(obstacle?.x) ? obstacle.x : null;
    const waypointX = obstacleX === null ? segment.waypointX : obstacleX;
    const learnedRisk = clamp(
      0.55*segment.slipEMA+0.75*segment.rollingErrorEMA+0.35*segment.gradeEMA,
      0,
      1
    );
    return {
      segmentIndex: segment.index,
      segmentLabel: `S${String(segment.index+1).padStart(2, '0')}`,
      waypointX,
      lateralError: waypointX-finite(x),
      speedScale: clamp(this.global.speedScale*segment.speedScale*(1+0.08*uphill),
        this.settings.minimumSpeedScale, this.settings.maximumSpeedScale),
      torqueScale: clamp(this.global.torqueScale*segment.torqueScale*(1+0.18*uphill),
        this.settings.minimumTorqueScale, this.settings.maximumTorqueScale),
      tractionScale: clamp(this.global.tractionScale*segment.tractionScale,
        this.settings.minimumTractionScale, this.settings.maximumTractionScale),
      alignmentScale: this.global.alignmentScale,
      actuationScale: clamp(1+0.12*learnedRisk+0.10*uphill, 1, 1.16),
      learnedRisk
    };
  }

  observe({ time, x, y, speed, slip, rollingError, grade, energy = 0 } = {}) {
    if (!this.activeRun) this.beginRun();
    const run = this.activeRun;
    const currentTime = Math.max(0, finite(time));
    const currentY = finite(y, this.settings.courseStartY);
    const index = this.segmentIndex(currentY);
    const sample = run.segments.get(index) || {
      elapsed: 0,
      progress: 0,
      samples: 0,
      speedSum: 0,
      slipSum: 0,
      rollingErrorSum: 0,
      gradeSum: 0,
      xSum: 0,
      energySum: 0,
      checkpointReached: false
    };
    if (run.lastTime !== null) {
      sample.elapsed += clamp(currentTime-run.lastTime, 0, 0.25);
      sample.progress += Math.max(0, currentY-run.lastY);
    }
    sample.samples++;
    sample.speedSum += Math.max(0, finite(speed));
    sample.slipSum += Math.max(0, finite(slip));
    sample.rollingErrorSum += Math.max(0, finite(rollingError));
    sample.gradeSum += Math.max(0, finite(grade));
    sample.xSum += finite(x);
    sample.energySum += Math.max(0, finite(energy));
    run.segments.set(index, sample);
    run.started = true;
    run.lastTime = currentTime;
    run.lastY = currentY;
  }

  observeCheckpoint(y, energy = 0) {
    if (!this.activeRun) return;
    const index = this.segmentIndex(y);
    const sample = this.activeRun.segments.get(index);
    if (sample) {
      sample.checkpointReached = true;
      sample.energySum += energy;
    }
  }

  finishRun({ reached = false, time = 0, finalY = 0, maxSlip = 0, rollingError = 0,
    lateralTravel = 0, reason = reached ? 'goal' : 'timeout' } = {}) {
    const elapsed = Math.max(0, finite(time));
    const withinDeadline = reached && elapsed <= this.settings.deadlineSeconds+1e-9;
    const outcome = withinDeadline ? 'win' : 'loss';
    const previousBest = this.bestTime;
    const courseDistance = this.settings.courseGoalY-this.settings.courseStartY;
    const remainingRatio = clamp((this.settings.courseGoalY-finite(finalY))/courseDistance, 0, 1);
    const deadlineRatio = elapsed/Math.max(1, this.settings.deadlineSeconds);
    const slipRatio = clamp(finite(maxSlip)/0.45, 0, 1.5);
    const rollingRatio = clamp(finite(rollingError)/0.35, 0, 1.5);
    const lateralRatio = clamp(finite(lateralTravel)/Math.max(1, courseDistance), 0, 1);
    const lr = this.settings.learningRate;

    // Gradient direction is computed from measurable run loss. A timeout or
    // slow run adds speed/torque authority; slip opposes traction growth; and
    // lateral waste strengthens GPS alignment. All updates are hard-bounded.
    const timePressure = outcome === 'loss'
      ? 0.40+0.55*remainingRatio
      : 0.04+Math.max(0, deadlineRatio-0.60);
    const speedGradient = timePressure-0.10*slipRatio;
    const torqueGradient = 0.08+0.22*rollingRatio+(outcome === 'loss' ? 0.16 : 0);
    const tractionGradient = 0.16*rollingRatio-0.10*slipRatio+(outcome === 'loss' ? 0.08 : 0);
    const alignmentGradient = 0.10*lateralRatio+(outcome === 'loss' ? 0.04 : 0);
    this.global.speedScale = clamp(this.global.speedScale+lr*speedGradient,
      this.settings.minimumSpeedScale, this.settings.maximumSpeedScale);
    this.global.torqueScale = clamp(this.global.torqueScale+lr*torqueGradient,
      this.settings.minimumTorqueScale, this.settings.maximumTorqueScale);
    this.global.tractionScale = clamp(this.global.tractionScale+lr*tractionGradient,
      this.settings.minimumTractionScale, this.settings.maximumTractionScale);
    this.global.alignmentScale = clamp(this.global.alignmentScale+lr*alignmentGradient, 0.90, 1.25);

    for (const [index, observation] of this.activeRun?.segments || []) {
      const segment = this.segments[index];
      const samples = Math.max(1, observation.samples);
      const avgSlip = observation.slipSum/samples;
      const avgRollingError = observation.rollingErrorSum/samples;
      const avgGrade = observation.gradeSum/samples;
      const avgX = observation.xSum/samples;
      const progressRate = observation.progress/Math.max(0.05, observation.elapsed);
      const expectedRate = 0.55;
      const paceDeficit = clamp(1-progressRate/expectedRate, -0.5, 1);
      const alpha = segment.visits ? 0.25 : 1;
      segment.slipEMA += alpha*(avgSlip-segment.slipEMA);
      segment.rollingErrorEMA += alpha*(avgRollingError-segment.rollingErrorEMA);
      segment.gradeEMA += alpha*(avgGrade-segment.gradeEMA);
      segment.progressRateEMA += alpha*(progressRate-segment.progressRateEMA);
      const avgEnergy = observation.energySum ? observation.energySum/samples : 0;
      segment.energyEMA += alpha*(avgEnergy-segment.energyEMA);
      segment.speedScale = clamp(segment.speedScale+lr*(0.08*paceDeficit-0.03*avgSlip), 0.92, 1.18);
      segment.torqueScale = clamp(segment.torqueScale+lr*(0.10*paceDeficit+0.18*avgRollingError+0.08*avgGrade), 0.94, 1.24);
      segment.tractionScale = clamp(segment.tractionScale+lr*(0.12*avgRollingError-0.05*avgSlip), 0.92, 1.18);
      segment.waypointX = outcome === 'win'
        ? clamp(0.80*segment.waypointX+0.20*avgX,
          -this.settings.maximumWaypointOffset, this.settings.maximumWaypointOffset)
        : 0.75*segment.waypointX;
      if (observation.checkpointReached) {
        segment.checkpointReached = true;
      }
      segment.visits++;
    }

    this.runCount++;
    if (outcome === 'win') {
      this.wins++;
      this.bestTime = previousBest === null ? elapsed : Math.min(previousBest, elapsed);
    } else {
      this.losses++;
    }
    const loss = outcome === 'win'
      ? elapsed/this.settings.deadlineSeconds+0.20*slipRatio+0.10*lateralRatio
      : 1+remainingRatio+0.20*slipRatio+0.10*lateralRatio;
    this.lastRun = {
      attempt: this.runCount,
      outcome,
      reason,
      time: elapsed,
      finalY: finite(finalY),
      remaining: Math.max(0, this.settings.courseGoalY-finite(finalY)),
      loss,
      improvedBest: outcome === 'win' && (previousBest === null || elapsed < previousBest),
      gradient: {
        speed: lr*speedGradient,
        torque: lr*torqueGradient,
        traction: lr*tractionGradient,
        alignment: lr*alignmentGradient
      }
    };
    this.revision++;
    this.activeRun = null;
    return this.lastRun;
  }

  snapshot(currentCommand = null) {
    return {
      version: this.settings.version,
      deadlineSeconds: this.settings.deadlineSeconds,
      runCount: this.runCount,
      nextAttempt: this.runCount+1,
      wins: this.wins,
      losses: this.losses,
      bestTime: this.bestTime,
      lastRun: this.lastRun ? { ...this.lastRun } : null,
      global: { ...this.global },
      currentCommand: currentCommand ? { ...currentCommand } : null,
      segments: this.segments.map(segment => ({ ...segment })),
      revision: this.revision
    };
  }

  serialize() {
    return this.snapshot();
  }
}
