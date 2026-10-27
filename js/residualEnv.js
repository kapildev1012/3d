/**
 * RESIDUALENV.JS - one headless 500 Hz simulation with the residual-MPPI hook.
 *
 * Shared by the worker threads / Web Workers (js/parallelWorker.js) and by
 * single-threaded callers (UI, tests, tools). A *tick* is one 20 Hz control
 * period = 25 DAE sub-steps. At every tick in which the SGT state machine is
 * shifting/tipping, the hook evaluates
 *
 *      du = clip( actor(o) + eps ),      eps = exploration noise (0 when deploying)
 *
 * and the controller applies u = clip(u_SGT + du) (|du| <= 0.030 m, total
 * stroke <= 0.120 m) followed by the tension shield, so every rollout stays
 * inside the certified set.
 */
import { METHODS, SCENARIOS } from './experimentProtocol.js';
import { configureLevel } from './levels.js';
import { SimConfig, Simulation, TerrainModel, SphericalRoverModel } from './simEngine.js?v=20261027-ob1';
import { serializeState, deserializeState } from './simSnapshot.js';
import {
  OBS_DIM, ACT_DIM, TEL, TEL_DIM, PARAM_IDX, DEFAULT_PARAMS, packParams, SynergyActor, synergyMatrix,
  residualObservation, stageReward
} from './residualMppi.js';

export const CONTROL_SUBSTEPS = 25;

/** SGT options carried by the residual-MPPI parameter vector (gains + gait timing). */
export function residualGaitOptions(P) {
  const o = { gainKp: P.sgtGainKp, gainKd: P.sgtGainKd };
  if (Number.isFinite(P.sgtSettleSpeed)) o.settleSpeed = P.sgtSettleSpeed;
  if (Number.isFinite(P.sgtSettleMinTime)) o.settleMinTime = P.sgtSettleMinTime;
  if (Number.isFinite(P.sgtSettleMaxTime)) o.settleMaxTime = P.sgtSettleMaxTime;
  if (Number.isFinite(P.sgtReleaseTime)) o.releaseTime = P.sgtReleaseTime;
  return o;
}                 // 25 x 2 ms = 50 ms = 20 Hz
const clip = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

function segmentDistance(p, a, b) {
  const ab = [b[0]-a[0], b[1]-a[1], b[2]-a[2]], ap = [p[0]-a[0], p[1]-a[1], p[2]-a[2]];
  const len2 = ab[0]*ab[0]+ab[1]*ab[1]+ab[2]*ab[2];
  const t = clip((ap[0]*ab[0]+ap[1]*ab[1]+ap[2]*ab[2])/len2, 0, 1);
  return Math.hypot(ap[0]-t*ab[0], ap[1]-t*ab[1], ap[2]-t*ab[2]);
}

/** Simulation configuration for one run: protocol config + residual-MPPI controller. */
export function buildEnvConfig(spec, hook, P) {
  // Same order as the UI and tools/train_guided_policy.mjs: level preset first (terrain, goal, A/B course
  // with crest checkpoints on Level 10), then the cable-only research profile, then the stress scenario.
  const cfg = new SimConfig({ seed: spec.seed ?? 1, enableDiagnosticsLog: false, monitoring: { enabled: false } });
  configureLevel(cfg, spec.level ?? 1);
  METHODS.sgt.configure(cfg);
  SCENARIOS[spec.scenario || 'nominal']?.apply(cfg);
  cfg.seed = spec.seed ?? 1;
  // variant 'residual' (default): SGT + actor residual, total stroke <= maxTotalStrokeM
  //         'sgt120'  : residual path with a zero residual (isolates the 0.120 m stroke clamp)
  //         'sgt'     : plain SGT, nominal strokes (reference)
  const variant = spec.variant || 'residual';
  cfg.controllerMode = variant === 'sgt' ? 'sgt_tipping' : 'residual_mppi';
  cfg.residualMppi = hook;
  cfg.sgtOptions = { ...(cfg.sgtOptions || {}), ...residualGaitOptions(P) };
  // Tension shield inside every rollout: payload tension >= minCableTensionN, outer tension <= maxCableTensionN.
  cfg.tensionShield = spec.shield === false ? false : { coreTensionMin: P.minCableTensionN, outerTensionMax: P.maxCableTensionN };
  if (spec.target) { cfg.targetDestination = spec.target.slice(); cfg.targetGoalY = spec.target[1]; }
  return cfg;
}

export class ResidualEnv {
  /**
   * @param {Float64Array} params packed parameter vector (shared or private)
   * @param {Float64Array} weights actor weights (shared or private)
   */
  constructor(params = packParams(DEFAULT_PARAMS), weights = null) {
    this.params = params;
    this.weightsView = weights;
    this.eps = new Float64Array(ACT_DIM);
    this.du = new Float64Array(ACT_DIM);
    this.duPrev = new Float64Array(ACT_DIM);
    this.obsTmp = new Float64Array(OBS_DIM);
    this.lastObs = new Float64Array(OBS_DIM);
    this.lastDu = new Float64Array(ACT_DIM);
    this.called = false;
    this.tel = new Float64Array(TEL_DIM);
    this.sim = null;
    this.local = null;
    const env = this;
    this.hook = {
      get maxResidualStrokeM() { return env.params[PARAM_IDX.maxResidualStrokeM]; },
      get maxTotalStrokeM() { return env.params[PARAM_IDX.maxTotalStrokeM]; },
      act: ctx => env.decide(ctx)
    };
  }

  get P() { return unpackQuick(this.params); }

  reset(spec) {
    this.spec = spec;
    const P = this.P;
    const cfg = buildEnvConfig(spec, this.hook, P);
    this.cfg = cfg;
    this.sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
    this.synergy = synergyMatrix(this.sim.rover);
    if (this.weightsView) this.actor = new SynergyActor(this.synergy).bind(this.weightsView);
    else if (!this.actor) this.actor = new SynergyActor(this.synergy);
    this.actor.M = this.synergy;
    const c = this.centroid();
    this.x0 = c[0]; this.y0 = c[1];
    // Per-episode extrema start from zero (a worker runs many episodes; carrying the peak over
    // made every later episode look overloaded to the safety / rollback logic).
    this.maxErr = 0; this.prevMargin = 0; this.prevEnergy = 0; this.cumUtu = 0; this.maxOuterTension = 0;
    this.duPrev.fill(0); this.eps.fill(0); this.tickCount = 0;
    this.ep = { n: 0, slack: 0, slip: 0, rodMax: 0, e2: 0, t0: this.sim.t };   // whole-episode stats (training digests)
    this.observe(this.lastObs);
    this.tel.fill(0);
    return this;
  }

  setActor(actor) { this.actor = actor; this.actor.M = this.synergy || this.actor.M; return this; }

  centroid() {
    const sim = this.sim, n = sim.rover.nOuter;
    const c = [0, 0, 0];
    for (const p of sim.q) { c[0] += p[0]; c[1] += p[1]; c[2] += p[2]; }
    return c.map(v => v/n);
  }

  /** Observation from the current simulation state (between ticks). */
  observe(out) {
    const sgt = this.sim.sgt;
    return residualObservation({
      sim: this.sim, margin: sgt?.lastPlan?.margin ?? 0, state: sgt?.state ?? 'SGT_SETTLE',
      elapsed: sgt ? this.sim.t-sgt.enteredAt : 0, params: this.params
    }, out);
  }

  /** The residual hook: actor prior + exploration noise, bounded by the stroke cap. */
  decide(ctx) {
    const rho = this.params[PARAM_IDX.maxResidualStrokeM];
    if (this.spec?.variant === 'sgt120') { this.du.fill(0); this.called = true; this.lastObs.fill(0); this.lastDu.fill(0); return this.du; }
    ctx.params = this.params;                              // MPC context of the running terrain
    residualObservation(ctx, this.obsTmp);
    this.actor.forward(this.obsTmp, rho, this.du);
    for (let c = 0; c < ACT_DIM; c++) this.du[c] = clip(this.du[c]+this.eps[c], -rho, rho);
    this.called = true;
    this.lastObs.set(this.obsTmp);
    this.lastDu.set(this.du);
    return this.du;
  }

  /** One 20 Hz control tick. Fills this.tel and returns it. */
  tick(eps = null) {
    const sim = this.sim, P = this.params;
    if (eps) this.eps.set(eps); else this.eps.fill(0);
    this.called = false;
    const c0 = this.centroid();
    const energy0 = sim.metrics.energyCost;
    for (let k = 0; k < CONTROL_SUBSTEPS; k++) sim.step();
    this.tickCount++;
    const c1 = this.centroid();
    const dtc = CONTROL_SUBSTEPS*sim.cfg.dt;
    const tel = this.tel;
    const rho = P[PARAM_IDX.maxResidualStrokeM];

    tel[TEL.vFwd] = (c1[1]-c0[1])/dtc;
    tel[TEL.cross] = Math.abs(c1[0]-this.x0);
    this.maxErr = Math.max(this.maxErr, tel[TEL.cross]);
    tel[TEL.maxErr] = this.maxErr;
    let uu = 0;
    for (const u of sim.currentActuationOffsets) uu += u*u;
    for (const u of sim.currentCoreActuationOffsets) uu += u*u;
    tel[TEL.uTu] = uu;
    this.cumUtu += uu*dtc;
    const dy = c1[1]-this.y0;
    const totalMass = sim.dynamicNodeMass*sim.rover.nOuter+sim.plantCoreMass;
    const mg = totalMass*Math.abs(sim.cfg.gravity[2]);
    tel[TEL.cot] = dy > 0.05 ? sim.metrics.energyCost/(mg*dy) : 0;
    tel[TEL.energyNorm] = (sim.metrics.energyCost-energy0)/(mg*0.1);

    let rod = 0;
    sim.rover.bars.forEach(([i, j], k) => {
      const a = sim.q[i], b = sim.q[j];
      rod = Math.max(rod, Math.abs(Math.hypot(b[0]-a[0], b[1]-a[1], b[2]-a[2])-sim.rover.l0_bars[k]));
    });
    tel[TEL.rodMm] = rod*1000;
    const outerForces = sim.currentDiag?.outerCableForces || [];
    let slack = 0, maxOuter = 0;
    for (const t of outerForces) { if (t < 1) slack++; if (t > maxOuter) maxOuter = t; }
    for (const t of sim.coreCableForces) if (t < 1) slack++;
    tel[TEL.slackPct] = 100*slack/(outerForces.length+sim.coreCableForces.length);
    tel[TEL.slip] = sim.currentDiag?.slipSpeed || 0;
    const margin = Number.isFinite(sim.comMargin) ? clip(sim.comMargin, -0.5, 0.5) : 0.3;
    tel[TEL.comMargin] = margin;
    tel[TEL.marginRate] = (margin-this.prevMargin)/dtc;
    this.prevMargin = margin;
    tel[TEL.epsJ] = Math.abs(sim.currentDiag?.energyLedger?.residual ?? 0);
    let clearance = Infinity;
    for (const [i, j] of sim.rover.bars) clearance = Math.min(clearance, segmentDistance(sim.corePosition, sim.q[i], sim.q[j]));
    tel[TEL.clearance] = clearance;
    tel[TEL.over] = (maxOuter > P[PARAM_IDX.maxCableTensionN] || tel[TEL.rodMm] > P[PARAM_IDX.rodLimitMm]) ? 1 : 0;
    tel[TEL.done] = sim.metrics.runTerminal ? 1 : 0;
    this.maxOuterTension = Math.max(this.maxOuterTension || 0, maxOuter);

    let dd = 0, d2 = 0;
    if (this.called) {
      for (let c = 0; c < ACT_DIM; c++) { d2 += this.lastDu[c]**2; dd += (this.lastDu[c]-this.duPrev[c])**2; }
      this.duPrev.set(this.lastDu);
    } else {
      for (let c = 0; c < ACT_DIM; c++) dd += this.duPrev[c]**2;
      this.duPrev.fill(0);
    }
    tel[TEL.duSq] = d2/(ACT_DIM*rho*rho);
    tel[TEL.dduSq] = dd/(ACT_DIM*4*rho*rho);
    if (this.ep) {
      this.ep.n++; this.ep.slack += tel[TEL.slackPct]; this.ep.slip += tel[TEL.slip];
      this.ep.rodMax = Math.max(this.ep.rodMax, tel[TEL.rodMm]); this.ep.e2 += tel[TEL.cross]**2;
    }
    return tel;
  }

  /** Digest of everything since reset() (used for training episodes driven tick by tick). */
  episodeDigest() {
    const sim = this.sim, ep = this.ep || { n: 0, slack: 0, slip: 0, rodMax: 0, e2: 0, t0: 0 };
    const dy = this.centroid()[1]-this.y0, elapsed = Math.max(1e-9, sim.t-ep.t0), n = Math.max(1, ep.n);
    const totalMass = sim.dynamicNodeMass*sim.rover.nOuter+sim.plantCoreMass;
    const timing = sim.controllerTiming;
    return {
      avgSpeedMs: dy/elapsed, rmseM: Math.sqrt(ep.e2/n), maxErrorM: this.maxErr, recoveryTimeS: NaN,
      controlEffortUtu: this.cumUtu, costOfTransport: dy > 0.05 ? sim.metrics.energyCost/(totalMass*Math.abs(sim.cfg.gravity[2])*dy) : NaN,
      solveTimeMs: timing.count ? timing.totalMs/timing.count : NaN, successRatePct: NaN,
      rodResidualMm: ep.rodMax, relaxedCablesPct: ep.slack/n, contactSlipMs: ep.slip/n,
      energyDriftJ: sim.currentDiag?.energyLedger?.maxAbsResidual ?? NaN, progressM: dy,
      tips: sim.sgt?.tipCount ?? 0, aborts: sim.sgt?.abortCount ?? 0, maxOuterTensionN: this.maxOuterTension || 0
    };
  }

  reward() { return stageReward(this.tel, this.params); }

  /** Local checkpoint including the runner's own bookkeeping. */
  save() {
    this.local = this.exportState();
    return this.local;
  }
  restore(state = this.local) { this.importState(state); }

  exportState() {
    return {
      sim: serializeState(this.sim),
      env: { maxErr: this.maxErr, prevMargin: this.prevMargin, cumUtu: this.cumUtu, tickCount: this.tickCount,
        duPrev: Array.from(this.duPrev), maxOuterTension: this.maxOuterTension || 0, x0: this.x0, y0: this.y0, ep: this.ep ? { ...this.ep } : null }
    };
  }
  importState(state) {
    deserializeState(this.sim, state.sim);
    const e = state.env;
    this.maxErr = e.maxErr; this.prevMargin = e.prevMargin; this.cumUtu = e.cumUtu; this.tickCount = e.tickCount;
    this.duPrev.set(e.duPrev); this.maxOuterTension = e.maxOuterTension; this.x0 = e.x0; this.y0 = e.y0;
    if (e.ep) this.ep = { ...e.ep };
    this.eps.fill(0);
    this.observe(this.lastObs);
  }

  /**
   * Whole-episode evaluation (actor prior only unless `noiseFn` is given),
   * returning the Comparative-benchmark digest for this run.
   */
  runEpisode(duration = 20, { successDistance = 1.0 } = {}) {
    if (!(duration > 0)) return this.episodeDigest();     // read-out of a tick-by-tick training episode
    const sim = this.sim, scenario = SCENARIOS[this.spec.scenario || 'nominal'];
    const ticks = Math.round(duration/(CONTROL_SUBSTEPS*sim.cfg.dt));
    let impulseDone = false, yBefore = null, recovery = NaN;
    const dTime = scenario.impulse?.time ?? sim.cfg.disturbance?.time ?? null;
    const dEnd = scenario.impulse ? dTime : (sim.cfg.disturbance ? dTime+(sim.cfg.disturbance.duration ?? 0) : null);
    let sumE2 = 0, nE = 0, slackSum = 0, slipSum = 0, rodMax = 0;
    for (let k = 0; k < ticks; k++) {
      if (scenario.impulse && !impulseDone && sim.t >= scenario.impulse.time) { sim.injectImpulse(scenario.impulse.dv); impulseDone = true; }
      if (dTime !== null && yBefore === null && sim.t >= dTime) yBefore = this.centroid()[1];
      this.tick(null);
      const tel = this.tel;
      sumE2 += tel[TEL.cross]**2; nE++;
      slackSum += tel[TEL.slackPct]; slipSum += tel[TEL.slip];
      rodMax = Math.max(rodMax, tel[TEL.rodMm]);
      const cy = this.centroid()[1];
      if (dEnd !== null && Number.isNaN(recovery) && sim.t >= dEnd && yBefore !== null && cy >= yBefore+0.05) recovery = sim.t-dTime;
      if (tel[TEL.done]) break;
    }
    const dy = this.centroid()[1]-this.y0;
    const totalMass = sim.dynamicNodeMass*sim.rover.nOuter+sim.plantCoreMass;
    const timing = sim.controllerTiming;
    return {
      avgSpeedMs: dy/duration, rmseM: Math.sqrt(sumE2/Math.max(1, nE)), maxErrorM: this.maxErr, recoveryTimeS: recovery,
      controlEffortUtu: this.cumUtu, costOfTransport: dy > 0.05 ? sim.metrics.energyCost/(totalMass*Math.abs(sim.cfg.gravity[2])*dy) : NaN,
      solveTimeMs: timing.count ? timing.totalMs/timing.count : NaN, successRatePct: dy >= successDistance ? 100 : 0,
      rodResidualMm: rodMax, relaxedCablesPct: slackSum/Math.max(1, nE), contactSlipMs: slipSum/Math.max(1, nE),
      energyDriftJ: sim.currentDiag?.energyLedger?.maxAbsResidual ?? NaN, progressM: dy,
      tips: sim.sgt?.tipCount ?? 0, aborts: sim.sgt?.abortCount ?? 0, maxOuterTensionN: this.maxOuterTension || 0
    };
  }
}

function unpackQuick(view) {
  const out = {};
  for (const name of Object.keys(PARAM_IDX)) out[name] = view[PARAM_IDX[name]];
  return out;
}
