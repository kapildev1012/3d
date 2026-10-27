/**
 * MPPIPLANNER.JS - residual-MPPI training and evaluation on the worker pool.
 *
 * One planning cycle (every H = 10 control ticks = 0.5 s of simulated time):
 *
 *   1. env 0 (the real trajectory) PUBLISHes its state to the shared
 *      checkpoint slot.
 *   2. K structured noise sequences eps_k (6-D synergy + 24-D fine, time
 *      correlated, candidate 0 noise-free) are written into shared memory.
 *   3. The rollout workers BRANCH from the checkpoint and simulate candidate k
 *      for H ticks with u = SGT + actor(o) + eps_k under the tension shield.
 *      Return: R_k = sum_t gamma^t r_t + gamma^H min(V1,V2)(s_H).
 *   4. Exponential weights w_k ~ exp(R_k / lambda) give the executed noise
 *      eps_bar_t = sum_k w_k eps_k,t, which env 0 applies on top of its own
 *      closed-loop actor for the next H ticks.
 *   5. The actor is distilled from the executed actions, the twin critic from
 *      TD(0) transitions and the MPPI return target.
 */
import { CMD } from './parallelPool.js';
import {
  OBS_DIM, ACT_DIM, TEL, TEL_DIM, PARAM_IDX, packParams, unpackParams, stageReward, mppiWeights, sampleNoise,
  makeRng, Replay, synergyShare, DEFAULT_PARAMS
} from './residualMppi.js';

const clip = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

export class ResidualMppiTrainer {
  /**
   * @param {SharedVecTensegrityEnv} pool  numEnvs >= 2 (env 0 = real, others = rollout branches)
   * @param {MorphologyActorCritic} model
   */
  constructor(pool, model, options = {}) {
    if (pool.numEnvs < 2) throw new Error('residual-MPPI needs at least 2 workers (1 real + >=1 rollout)');
    this.pool = pool;
    this.model = model;
    this.opts = {
      horizon: 10, temperature: 0.5, updatesPerPlan: 24, batch: 48, valueWarmup: 150,
      sigmaSynergy: 0.008, sigmaFine: 0.005, alpha: 0.6, actorLr: 8e-4, criticLr: 1.5e-3, seed: 11, ...options
    };
    this.rng = makeRng(this.opts.seed);
    this.transitions = new Replay(30000, this.opts.seed+1);
    this.targets = new Replay(5000, this.opts.seed+2);       // MPPI return targets (s_0, R)
    this.actions = new Replay(8000, this.opts.seed+3);       // executed actions (o, du)
    this.criticSteps = 0;
    this.model.actor.opt.lr = this.opts.actorLr;
    this.model.critic.v1.opt.lr = this.model.critic.v2.opt.lr = this.opts.criticLr;
    this.log = [];
  }

  syncToPool() {
    this.pool.setParams(this.model.params);
    this.pool.setWeights(this.model.actor.w);
  }

  /** Terminal-value bootstrap, ramped in as the critic gets trained. */
  terminalValue(obs) {
    const ramp = clip(this.criticSteps/this.opts.valueWarmup, 0, 1);
    return ramp > 0 ? ramp*this.model.critic.target(obs) : 0;
  }

  /**
   * One MPPI planning cycle from env 0's current state. Returns the planned
   * executed noise [H][24] and diagnostics.
   */
  async plan(H = this.opts.horizon) {
    const { pool, model } = this;
    const K = pool.numCandidates, W = pool.numEnvs;
    const P = unpackParams(pool.v.params);
    this.syncToPool();
    await pool.broadcast(CMD.PUBLISH, [0]);
    sampleNoise(pool.v.noise.subarray(0, K*pool.maxHorizon*ACT_DIM), K, pool.maxHorizon, model.actor.M, this.rng, {
      sigmaSynergy: this.opts.sigmaSynergy, sigmaFine: this.opts.sigmaFine, alpha: this.opts.alpha, cap: P.maxResidualStrokeM
    });
    const rollers = Array.from({ length: W-1 }, (_, i) => i+1);
    for (let first = 0; first < K; first += rollers.length) {
      const ids = rollers.filter((_, i) => first+i < K);
      await pool.broadcast(CMD.ROLLOUT, ids, id => [first+(id-1), H, 0, 0]);
    }
    const returns = new Float64Array(K);
    const gammaH = P.gamma**H;
    let violating = 0;
    for (let k = 0; k < K; k++) {
      const terminal = pool.v.termObs.subarray(k*OBS_DIM, (k+1)*OBS_DIM);
      returns[k] = pool.v.ret[k]+gammaH*this.terminalValue(terminal);
      if (pool.v.candInfo[k*4+1] > 0) violating++;
    }
    const { weights, ess, lambda } = mppiWeights(returns, this.opts.temperature);
    const noise = new Float64Array(H*ACT_DIM);
    for (let t = 0; t < H; t++) {
      for (let k = 0; k < K; k++) {
        const base = (k*pool.maxHorizon+t)*ACT_DIM;
        for (let c = 0; c < ACT_DIM; c++) noise[t*ACT_DIM+c] += weights[k]*pool.v.noise[base+c];
      }
    }
    let best = 0, expected = 0;
    for (let k = 0; k < K; k++) { if (returns[k] > returns[best]) best = k; expected += weights[k]*returns[k]; }
    return { noise, returns, weights, ess, lambda, best, expected, nominalReturn: returns[0], advantage: returns[best]-returns[0], violating,
      candTel: Array.from(pool.v.candTel.slice(0, K*TEL_DIM)) };
  }

  /** Execute the planned noise for H ticks on the real env; record transitions and training data. */
  async execute(plan, H = this.opts.horizon) {
    const { pool } = this;
    const P = unpackParams(pool.v.params);
    let ret = 0, steps = 0, done = false;
    for (let t = 0; t < H && !done; t++) {
      const before = Float64Array.from(pool.getEnvObs(0));
      pool.setEnvAction(0, plan.noise.subarray(t*ACT_DIM, (t+1)*ACT_DIM));
      await pool.broadcast(CMD.STEP, [0]);
      const tel = pool.getEnvTelemetry(0);
      const r = stageReward(tel, pool.v.params);
      const after = Float64Array.from(pool.getEnvObs(0));
      done = tel[TEL.done] > 0;
      this.transitions.push({ s: before, r, s2: after, done });
      if (pool.v.decFlag[0]) {
        this.actions.push({ o: Float64Array.from(pool.v.decObs.subarray(0, OBS_DIM)), a: Float64Array.from(pool.v.decAct.subarray(0, ACT_DIM)) });
      }
      ret += r*P.gamma**t; steps++;
    }
    return { ret, steps, done };
  }

  /** Gradient updates for the actor (distillation) and twin critic (TD + MPPI targets). */
  train(plan, startObs) {
    const { model } = this, P = model.params, rho = P.maxResidualStrokeM;
    this.targets.push({ s: startObs, R: plan.expected });
    let actorLoss = NaN, criticLoss = NaN;
    for (let u = 0; u < this.opts.updatesPerPlan; u++) {
      if (this.actions.size >= 16) {
        const batch = this.actions.sample(this.opts.batch);
        actorLoss = model.actor.trainBatch(batch.map(b => b.o), batch.map(b => b.a), rho);
      }
      if (this.transitions.size >= 16) {
        const tb = this.transitions.sample(this.opts.batch), mb = this.targets.sample(Math.min(12, this.targets.size));
        const obs = [], y = [];
        for (const tr of tb) {
          obs.push(tr.s);
          y.push(tr.r+(tr.done ? 0 : P.gamma*model.critic.target(tr.s2)));
        }
        for (const m of mb) { obs.push(m.s); y.push(m.R); }
        criticLoss = model.critic.train(obs, y);
        this.criticSteps++;
      }
    }
    return { actorLoss, criticLoss };
  }

  /**
   * One training episode on spec (level, seed, scenario). `duration` seconds of
   * simulated time, planning every `horizon` ticks.
   */
  async runEpisode(spec, duration, { train = true, onPlan = null } = {}) {
    const { pool } = this;
    pool.setParams(this.model.params);
    this.syncToPool();
    await pool.resetAll(() => spec);
    const H = this.opts.horizon, tickDt = 0.05;
    const cycles = Math.ceil(duration/(H*tickDt));
    const stats = { plans: 0, ess: 0, spread: 0, violating: 0, gainOverNominal: 0, advantage: 0, synergyShare: 0, retSum: 0 };
    let total = 0;
    for (let c = 0; c < cycles; c++) {
      const startObs = Float64Array.from(pool.getEnvObs(0));
      const plan = await this.plan(H);
      const run = await this.execute(plan, H);
      const losses = train ? this.train(plan, startObs) : {};
      stats.plans++; stats.ess += plan.ess; stats.violating += plan.violating;
      stats.spread += Math.max(...plan.returns)-Math.min(...plan.returns);
      stats.gainOverNominal += plan.expected-plan.nominalReturn;
      stats.advantage += plan.advantage;
      stats.retSum += run.ret; total += run.steps;
      const executed = plan.noise.slice(0, ACT_DIM);
      stats.synergyShare += synergyShare(executed, this.model.actor.M);
      this.log.push({ spec, cycle: c, ess: plan.ess, expected: plan.expected, nominal: plan.nominalReturn, executed: run.ret, ...losses });
      if (onPlan) onPlan({ cycle: c, cycles, plan, run, losses });
      if (run.done) break;
    }
    await pool.broadcast(CMD.EPISODE, [0], () => [0, 0, 0, 0]);           // duration 0: read the digest of what env 0 did
    const digest = pool.getDigest(0);
    const n = Math.max(1, stats.plans);
    return { digest, plans: stats.plans, ticks: total, meanEss: stats.ess/n, meanSpread: stats.spread/n,
      violatingCandidates: stats.violating, meanGainOverNominal: stats.gainOverNominal/n, meanAdvantage: stats.advantage/n,
      synergyShare: stats.synergyShare/n, meanReturn: stats.retSum/n };
  }
}

/**
 * Evaluate the actor prior (no search) on a list of specs, in parallel over
 * all workers. Returns one digest per spec (same order).
 */
export async function evaluateSpecs(pool, model, specs, duration) {
  pool.setParams(model.params);
  pool.setWeights(model.actor.w);
  const W = pool.numEnvs, out = new Array(specs.length);
  for (let first = 0; first < specs.length; first += W) {
    const ids = Array.from({ length: Math.min(W, specs.length-first) }, (_, i) => i);
    for (const id of ids) pool.specToSlot(id, specs[first+id]);
    await pool.broadcast(CMD.RESET, ids);
    await pool.broadcast(CMD.EPISODE, ids, () => [0, 0, Math.round(duration*1000), 0]);
    for (const id of ids) out[first+id] = { spec: specs[first+id], ...pool.getDigest(id) };
  }
  return out;
}

/** Mean of the finite values. */
const mean = values => {
  const f = values.filter(Number.isFinite);
  return f.length ? f.reduce((s, v) => s+v, 0)/f.length : NaN;
};

/**
 * Aggregate raw digests into BenchmarkDigest rows (one per level x disturbance)
 * exactly as in the Comparative benchmark table.
 */
export function aggregateDigests(results) {
  const groups = new Map();
  for (const r of results) {
    const key = `${r.spec.level}|${r.spec.scenario || 'nominal'}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(r);
  }
  return [...groups.entries()].map(([key, rows]) => {
    const [level, disturbance] = key.split('|');
    const g = name => mean(rows.map(r => r[name]));
    return {
      level: Number(level), disturbance, seeds: rows.length,
      avgSpeedMs: g('avgSpeedMs'), rmseM: g('rmseM'), maxErrorM: g('maxErrorM'), recoveryTimeS: g('recoveryTimeS'),
      controlEffortUtu: g('controlEffortUtu'), costOfTransport: g('costOfTransport'), solveTimeMs: g('solveTimeMs'),
      successRatePct: g('successRatePct'), rodResidualMm: g('rodResidualMm'), relaxedCablesPct: g('relaxedCablesPct'),
      contactSlipMs: g('contactSlipMs'), energyDriftJ: g('energyDriftJ'), progressM: g('progressM'),
      maxOuterTensionN: Math.max(...rows.map(r => r.maxOuterTensionN || 0))
    };
  });
}

export const STRESS_SCENARIOS = Object.freeze(['nominal', 'push', 'gaussian', 'laplace', 'outlier', 'stiffness', 'friction', 'payload', 'delay']);
export const STRESS_LEVELS = Object.freeze([1, 4, 7, 10]);

export function stressMatrix({ levels = STRESS_LEVELS, scenarios = STRESS_SCENARIOS, seeds = [1] } = {}) {
  const specs = [];
  for (const level of levels) for (const scenario of scenarios) for (const seed of seeds) specs.push({ level, scenario, seed });
  return specs;
}

export { DEFAULT_PARAMS };
