/**
 * RUNRECORDER.JS - continuous run recording for the 14-level matrix
 *
 * A run starts when the simulation clock leaves 0 and is:
 *   - checkpointed (status 'active') every 1.0 s of simulated time;
 *   - finalized as
 *       completed   Model B reached its goal (course win or goal radius),
 *       failed      mission deadline, level time limit without the goal,
 *                   or a stall (< 5 cm progress in 30 s of simulated time),
 *       terminated  the user reset, switched level/controller/preset/physics,
 *                   rebuilt the rover, or closed/refreshed the page.
 * Pause and gait changes commit a checkpoint (to every storage layer) but
 * keep the run open: it is still the same physical trajectory.
 * Runs shorter than 0.5 s of simulated time and 1 cm of travel are never
 * stored, so double resets do not create empty records.
 */
import { newRunId, isMeaningfulRun } from './benchmarkCore.js?v=20261021-ws1';

const CHECKPOINT_PERIOD = 1.0; // [s] simulated
const STALL_WINDOW = 30.0;     // [s] simulated
const STALL_DISTANCE = 0.05;   // [m]

const num = value => (Number.isFinite(value) ? value : null);

export class RunRecorder {
  constructor(app, persistence) {
    this.app = app;
    this.persistence = persistence;
    this.active = null;
    this.boundSim = null;
    this.lastSimTime = 0;
    this.awaitingReset = false;
    this.source = 'live';
    this.lastFinalized = null;
  }

  get activeRun() { return this.active; }

  /** Called once per animation frame (and by the auto-benchmark after each chunk). */
  tick() {
    const { simB } = this.app;
    if (!simB) return;
    if (simB !== this.boundSim) {
      // Sims were rebuilt (level switch, parameter rebuild). Callers commit
      // explicitly first; this is the safety net.
      if (this.active) this.finalize('rebuild', 'terminated');
      this.boundSim = simB;
      this.lastSimTime = simB.t;
      this.awaitingReset = false;
    }
    if (simB.t < this.lastSimTime-1e-9) {
      if (this.active) this.finalize('reset', 'terminated');
      this.awaitingReset = false;
    }
    this.lastSimTime = simB.t;
    if (this.awaitingReset || simB.t <= 0) return;
    if (!this.active) this.start();

    const run = this.active;
    this.capture(run);
    const outcome = this.detectOutcome(run);
    if (outcome) {
      this.finalize(outcome.reason, outcome.status);
      return;
    }
    if (simB.t-run.lastCheckpointSimTime >= CHECKPOINT_PERIOD) {
      run.lastCheckpointSimTime = simB.t;
      run.checkpoints += 1;
      this.persist(run, false);
    }
  }

  start() {
    const app = this.app;
    const cfg = app.cfg;
    const now = new Date().toISOString();
    this.active = {
      id: newRunId(),
      level: cfg.experimentId,
      source: this.source,
      status: 'active',
      reason: null,
      final: false,
      startedAt: now,
      updatedAt: now,
      endedAt: null,
      checkpoints: 0,
      controller: cfg.controllerMode || 'natural_support_face',
      controllerShort: app.constructor.controllerShortLabel(cfg.controllerMode || 'natural_support_face'),
      gait: cfg.actuationMode,
      gaitChanges: 0,
      physicsCore: cfg.physicsCore,
      actuation: cfg.pureCableActuation ? 'cable-only' : 'assisted',
      gravity: Math.abs(cfg.gravity?.[2] ?? 9.81),
      material: cfg.material || 'lightweight',
      seed: cfg.seed,
      simTime: 0,
      obstacles: app.obstacleCount?.() ?? null,
      A: null,
      B: null,
      lastCheckpointSimTime: 0,
      progressMark: { t: 0, distance: 0 }
    };
  }

  modelMetrics(sim) {
    const m = sim?.metrics;
    if (!m) return null;
    const summary = m.obstacleSummary || {};
    return {
      distance: num(m.distanceTraveled) ?? 0,
      time: num(m.timeElapsed) ?? sim.t,
      avgSpeed: num(m.avgVelocity) ?? 0,
      maxG: num(m.payloadAccelMax),
      maxTension: num(m.maxCableTension),
      deformation: num(m.shapeDeformationMax),
      over: num(summary.over),
      around: num(summary.around),
      outcome: m.runOutcome || null,
      completionTime: num(m.completionTime)
    };
  }

  capture(run) {
    const { simA, simB } = this.app;
    run.simTime = simB.t;
    run.A = this.modelMetrics(simA);
    run.B = this.modelMetrics(simB);
    run.updatedAt = new Date().toISOString();
  }

  detectOutcome(run) {
    const { simB, cfg } = this.app;
    const metrics = simB.metrics;
    if (metrics.runOutcome === 'win') return { status: 'completed', reason: 'goal' };
    if (metrics.runOutcome === 'loss') return { status: 'failed', reason: 'deadline' };
    const goalReached = simB.monitor?.goalReached || simB.currentDiag?.monitoring?.goalReached;
    if (goalReached && !simB.terrain?.course) return { status: 'completed', reason: 'goal' };
    if (simB.t >= cfg.T_end-0.5*cfg.dt) return { status: 'failed', reason: 'time-limit' };
    const distance = run.B?.distance ?? 0;
    if (distance-run.progressMark.distance >= STALL_DISTANCE) {
      run.progressMark = { t: simB.t, distance };
    } else if (simB.t-run.progressMark.t >= STALL_WINDOW) {
      return { status: 'failed', reason: 'stall' };
    }
    return null;
  }

  persist(run, flush) {
    if (!isMeaningfulRun(run)) return false;
    const { lastCheckpointSimTime, progressMark, ...record } = run;
    return this.persistence.upsertRun(record, { flush });
  }

  /** Close the open run. Returns the stored record (or null if it was too short). */
  finalize(reason, status = 'terminated') {
    const run = this.active;
    if (!run) return null;
    this.capture(run);
    run.status = status;
    run.reason = reason;
    run.final = true;
    run.endedAt = run.updatedAt;
    this.active = null;
    // After a decisive outcome the trajectory keeps evolving (the other
    // model may still be running) but belongs to no new run until a reset.
    this.awaitingReset = status === 'completed' || status === 'failed';
    const stored = this.persist(run, true);
    this.lastFinalized = stored ? run : null;
    this.app.onRunFinalized?.(run, stored);
    return stored ? run : null;
  }

  /** User actions that end the run: reset, level/controller/preset/physics change, unload. */
  commit(reason) {
    if (!this.active && this.app.simB?.t > 0 && !this.awaitingReset) this.tick();
    this.finalize(reason, 'terminated');
    this.awaitingReset = false;
  }

  /** Pause / gait change: durable checkpoint, run stays open. */
  checkpoint(reason) {
    const run = this.active;
    if (!run) return;
    this.capture(run);
    if (reason === 'gait-change') {
      run.gaitChanges += 1;
      run.gait = this.app.cfg.actuationMode;
    }
    run.checkpoints += 1;
    run.lastCheckpointSimTime = this.app.simB.t;
    // Pause flushes to disk immediately; frequent gait changes (W/A/S/D) are
    // saved locally at once and reach the disk with the next debounced sync.
    this.persist(run, reason === 'pause');
  }
}
