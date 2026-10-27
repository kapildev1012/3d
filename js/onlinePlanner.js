/**
 * ONLINEPLANNER.JS - live (receding-horizon) residual-MPPI planning for the
 * deployed MPC + RL and MPC + RL + LLM controllers.
 *
 * The distilled actor alone is only the RL half. This module adds the MPC half
 * at run time, single-threaded, so it works in the benchmark protocol, in Node
 * tools and (optionally) in the browser:
 *
 *   every H control ticks (H = 10 ticks = 0.5 s):
 *     1. snapshot the real simulation (js/simSnapshot.js, bit-exact);
 *     2. for K candidates, restore the snapshot into a shadow simulation and roll
 *        it out for H ticks with u = SGT + actor(o) + eps_k under the tension
 *        shield; candidate 0 is eps = 0 (the actor prior itself);
 *     3. score R_k = sum_t gamma^t r_t with the MPC stage reward (residualMppi.js);
 *     4. execute the best candidate's eps sequence (elite selection), but only if
 *        it beats the prior by `minGain`; otherwise execute the prior unchanged.
 *     5. a second, narrower refinement round samples around the elite.
 *
 * The planner's model is the NOMINAL plant: the shadow configuration drops the
 * scenario's disturbance force, sensor noise and actuator delay, so planning
 * never sees future disturbances (no clairvoyance); plant changes (friction,
 * stiffness, payload) are likewise unknown to it. Because candidate 0 is the
 * prior, the executed plan is never worse than the prior *under the model*.
 */
import { SimConfig, Simulation } from './simEngine.js?v=20261027-ob1';
import { serializeState, deserializeState } from './simSnapshot.js';
import { OBS_DIM, ACT_DIM, TEL, TEL_DIM, PARAM_IDX, residualObservation, stageReward, sampleNoise, makeRng, packParams, DEFAULT_PARAMS } from './residualMppi.js';

export const CONTROL_SUBSTEPS = 25;                        // 20 Hz control tick
const clip = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

function segmentDistance(p, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
  const t = clip((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / (ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]), 0, 1);
  return Math.hypot(ap[0] - t * ab[0], ap[1] - t * ab[1], ap[2] - t * ab[2]);
}

export class OnlineResidualPlanner {
  /**
   * @param {Simulation} sim the real simulation (its sgt uses this.hook)
   * @param {MorphologyActorCritic} model actor (+ params) used for the prior
   */
  constructor(sim, model, options = {}) {
    this.sim = sim;
    this.model = model;
    this.opts = { candidates: 8, horizon: 10, gamma: 0.98, minGain: 0.02, refine: true, sigmaSynergy: 0.008, sigmaFine: 0.005, alpha: 0.6, seed: 7, ...options };
    this.rng = makeRng(this.opts.seed);
    this.P = packParams({ ...DEFAULT_PARAMS, ...(model.params || {}) });
    this.rho = model.params?.maxResidualStrokeM ?? DEFAULT_PARAMS.maxResidualStrokeM;
    const H = this.opts.horizon;
    this.plan = new Float64Array(H * ACT_DIM);              // executed eps sequence
    this.noise = new Float64Array(this.opts.candidates * H * ACT_DIM);
    this.planT = -Infinity;
    this.nextPlanT = 0;
    this.obs = new Float64Array(OBS_DIM);
    this.out = new Float64Array(ACT_DIM);
    this.stats = { plans: 0, improved: 0, meanGain: 0, lastGain: 0, rolloutTicks: 0, wallMs: 0 };
    const planner = this;
    this.shadowEps = null;                                  // eps sequence the shadow is following
    this.shadowT0 = 0;
    // Hook used by the REAL controller: actor prior + planned eps.
    this.hook = {
      maxResidualStrokeM: this.rho,
      maxTotalStrokeM: model.params?.maxTotalStrokeM ?? DEFAULT_PARAMS.maxTotalStrokeM,
      act: ctx => planner.act(ctx, planner.plan, planner.planT)
    };
    // Hook used by the SHADOW controller during rollouts.
    this.shadowHook = {
      maxResidualStrokeM: this.rho,
      maxTotalStrokeM: this.hook.maxTotalStrokeM,
      act: ctx => planner.act(ctx, planner.shadowEps, planner.shadowT0)
    };
  }

  act(ctx, eps, t0) {
    ctx.params = this.model.params;
    residualObservation(ctx, this.obs);
    const prior = this.model.actPrior(this.obs, this.rho, this.out);
    const H = this.opts.horizon;
    const k = Math.floor((ctx.sim.t - t0) / (CONTROL_SUBSTEPS * ctx.sim.cfg.dt) + 1e-9);
    const du = prior || this.out.fill(0);
    if (eps && k >= 0 && k < H) for (let c = 0; c < ACT_DIM; c++) du[c] = clip(du[c] + eps[k * ACT_DIM + c], -this.rho, this.rho);
    return du;
  }

  /** Shadow simulation with the nominal plant model (built once, lazily). */
  shadow() {
    if (this.shadowSim) return this.shadowSim;
    const main = this.sim;
    const cfg = Object.assign(new SimConfig({}), main.cfg);
    cfg.disturbance = null; cfg.sensorNoise = null; cfg.actuatorDelay = 0;
    cfg.plantFrictionScale = 1; cfg.plantStiffnessScale = 1; cfg.plantPayloadMassScale = 1;
    cfg.residualMppi = this.shadowHook;
    cfg.residualOnlinePlanning = false;
    cfg.enableDiagnosticsLog = false;
    cfg.monitoring = { ...(cfg.monitoring || {}), enabled: false };
    this.shadowSim = new Simulation(cfg, main.rover, main.terrain, main.modelType);
    return this.shadowSim;
  }

  rollout(snapshot, eps) {
    const sim = this.shadow();
    deserializeState(sim, snapshot);
    this.shadowEps = eps;
    this.shadowT0 = sim.t;
    const H = this.opts.horizon, gamma = this.opts.gamma;
    const tel = new Float64Array(TEL_DIM);
    let R = 0, discount = 1, prevMargin = Number.isFinite(sim.comMargin) ? clip(sim.comMargin, -0.5, 0.5) : 0.3;
    let prevDu = null;
    const P = this.P;
    for (let t = 0; t < H; t++) {
      const c0 = centroid(sim), e0 = sim.metrics.energyCost;
      for (let s = 0; s < CONTROL_SUBSTEPS; s++) sim.step();
      this.stats.rolloutTicks++;
      const c1 = centroid(sim), dtc = CONTROL_SUBSTEPS * sim.cfg.dt;
      tel.fill(0);
      tel[TEL.vFwd] = (c1[1] - c0[1]) / dtc;
      tel[TEL.cross] = Math.abs(c1[0] - (this.laneX ?? 0));
      let uu = 0;
      for (const u of sim.currentActuationOffsets) uu += u * u;
      for (const u of sim.currentCoreActuationOffsets) uu += u * u;
      tel[TEL.uTu] = uu;
      const mg = (sim.dynamicNodeMass * sim.rover.nOuter + sim.plantCoreMass) * Math.abs(sim.cfg.gravity[2]);
      tel[TEL.energyNorm] = (sim.metrics.energyCost - e0) / (mg * 0.1);
      const forces = sim.currentDiag?.outerCableForces || [];
      let slack = 0, maxOuter = 0;
      for (const f of forces) { if (f < 1) slack++; if (f > maxOuter) maxOuter = f; }
      for (const f of sim.coreCableForces) if (f < 1) slack++;
      tel[TEL.slackPct] = 100 * slack / Math.max(1, forces.length + sim.coreCableForces.length);
      tel[TEL.slip] = sim.currentDiag?.slipSpeed || 0;
      const margin = Number.isFinite(sim.comMargin) ? clip(sim.comMargin, -0.5, 0.5) : 0.3;
      tel[TEL.comMargin] = margin; tel[TEL.marginRate] = (margin - prevMargin) / dtc; prevMargin = margin;
      let clearance = Infinity;
      for (const [i, j] of sim.rover.bars) clearance = Math.min(clearance, segmentDistance(sim.corePosition, sim.q[i], sim.q[j]));
      tel[TEL.clearance] = clearance;
      tel[TEL.over] = maxOuter > P[PARAM_IDX.maxCableTensionN] ? 1 : 0;
      tel[TEL.done] = sim.metrics.runTerminal ? 1 : 0;
      // smoothness of the executed residual (eps part; the prior is shared by all candidates)
      if (eps) {
        let d2 = 0, dd = 0;
        for (let c = 0; c < ACT_DIM; c++) { const e = eps[t * ACT_DIM + c]; d2 += e * e; const p = prevDu ? prevDu[c] : 0; dd += (e - p) ** 2; }
        tel[TEL.duSq] = d2 / (ACT_DIM * this.rho * this.rho); tel[TEL.dduSq] = dd / (ACT_DIM * 4 * this.rho * this.rho);
        prevDu = eps.subarray(t * ACT_DIM, (t + 1) * ACT_DIM);
      }
      R += discount * stageReward(tel, P);
      discount *= gamma;
      if (tel[TEL.done]) break;
    }
    return R;
  }

  /** Called by Simulation.step() before each step (state is between steps, i.e. consistent). */
  maybePlan(sim) {
    if (sim.t + 1e-9 < this.nextPlanT) return;
    const H = this.opts.horizon, K = this.opts.candidates;
    const tick = CONTROL_SUBSTEPS * sim.cfg.dt;
    this.nextPlanT = sim.t + H * tick;
    // No residual outside SHIFT/TIP: skip planning while the gait is settling/releasing far from a tip.
    const started = performance.now();
    if (this.laneX === undefined) this.laneX = centroid(sim)[0];
    const snapshot = serializeState(sim);
    sampleNoise(this.noise, K, H, this.model.actor.M, this.rng, { cap: this.rho, sigmaSynergy: this.opts.sigmaSynergy, sigmaFine: this.opts.sigmaFine, alpha: this.opts.alpha });
    const returns = new Float64Array(K);
    for (let k = 0; k < K; k++) returns[k] = this.rollout(snapshot, k === 0 ? null : this.noise.subarray(k * H * ACT_DIM, (k + 1) * H * ACT_DIM));
    let best = 0;
    for (let k = 1; k < K; k++) if (returns[k] > returns[best]) best = k;
    let bestEps = best === 0 ? null : this.noise.slice(best * H * ACT_DIM, (best + 1) * H * ACT_DIM);
    let bestR = returns[best];
    // Refinement round: perturb the elite with half the noise.
    if (this.opts.refine && bestEps && K > 2) {
      const refine = new Float64Array(K * H * ACT_DIM);
      sampleNoise(refine, K, H, this.model.actor.M, this.rng, { cap: this.rho, sigmaSynergy: 0.5 * this.opts.sigmaSynergy, sigmaFine: 0.5 * this.opts.sigmaFine, alpha: this.opts.alpha });
      for (let k = 1; k < Math.ceil(K / 2); k++) {
        const cand = refine.subarray(k * H * ACT_DIM, (k + 1) * H * ACT_DIM);
        for (let i = 0; i < cand.length; i++) cand[i] = clip(cand[i] + bestEps[i], -this.rho, this.rho);
        const R = this.rollout(snapshot, cand);
        if (R > bestR) { bestR = R; bestEps = cand.slice(); }
      }
    }
    // Restore nothing on the real sim (rollouts ran on the shadow); commit the plan.
    const gain = bestR - returns[0];
    if (bestEps && gain > this.opts.minGain) { this.plan.set(bestEps); this.stats.improved++; } else this.plan.fill(0);
    this.planT = sim.t;
    this.stats.plans++;
    this.stats.lastGain = Math.max(0, gain);
    this.stats.meanGain += (this.stats.lastGain - this.stats.meanGain) / this.stats.plans;
    this.stats.wallMs += performance.now() - started;
  }
}

function centroid(sim) {
  const c = [0, 0, 0];
  for (const p of sim.q) { c[0] += p[0]; c[1] += p[1]; c[2] += p[2]; }
  const n = sim.q.length;
  return [c[0] / n, c[1] / n, c[2] / n];
}
