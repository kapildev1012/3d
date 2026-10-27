/**
 * RESIDUALMPPI.JS - Morphology-aware residual-MPPI with a twin value critic
 * for the six-bar tensegrity rover (cable-only, Lagrangian DAE).
 *
 * Structure (TD-MPC2 / residual-RL style):
 *
 *   u_t = clip( u_nominal(SGT, MPC) + du_t ),   |du_t| <= rho = 0.030 m,
 *                                               total stroke <= 0.120 m
 *
 *   du = rho * tanh( 0.7 * M' tanh(z) + 0.3 * r ),   [z; r] = heads(trunk(o))
 *
 *   o  in R^48  observation (COM, velocity, spin, goal error, COM margin,
 *               gait phase, 24 outer-cable strains, 12 payload tensions)
 *   z  in R^6   morphological synergies: one per strut. The icosahedral shell
 *               has six struts and each strut joins two nodes carrying four
 *               outer cables; synergy k pulls the cables at one end of strut k
 *               and releases those at the other (a rolling/steering mode).
 *               M in R^{6x24} is that incidence matrix (rows scaled to +-1).
 *   r  in R^24  fine per-cable residual.
 *
 * Planning (js/parallelPool.js supplies K branch rollouts from one
 * checkpoint): K noisy residual sequences eps_k over H = 10 control ticks
 * (0.5 s) are scored by the physics reward plus the learned terminal value
 * min(V1, V2)(s_H) and combined with exponential (MPPI) weights; the
 * weighted noise is executed on top of the actor's prior, and the actor is
 * distilled from the executed actions while the twin critic is trained by
 * TD(0) and by the MPPI return targets. The tension shield runs inside every
 * rollout (cfg.tensionShield), so rollouts never leave the certified set.
 *
 * Everything here is plain JS with no dependency on the simulator except
 * residualObservation / stageReward, so the same module runs in Node worker
 * threads, browser Web Workers and the UI.
 */

/**
 * Observation = 48-D rover state + 14-D terrain one-hot + 11-D live MPC parameters.
 * The terrain and MPC context make one shared policy terrain- and MPC-conditioned:
 * when the meta-controller retunes terrain k, the policy sees the new parameters.
 */
export const OBS_BASE_DIM = 48;
export const TERRAIN_DIM = 14;
export const MPC_CONTEXT_KEYS = Object.freeze(['horizonSteps', 'qVelocityTrack', 'qLateralRmse', 'qComMargin', 'qClearance',
  'rControlEffort', 'rSmoothnessDeltaU', 'minCableTensionN', 'maxCableTensionN', 'sgtGainKp', 'sgtGainKd']);
const MPC_CONTEXT_SCALE = Object.freeze([16, 10, 10, 10, 10, 1, 1, 25, 200, 2.5, 2]);
export const OBS_DIM = OBS_BASE_DIM+TERRAIN_DIM+MPC_CONTEXT_KEYS.length;   // 73
export const ACT_DIM = 24;
export const SYNERGY_DIM = 6;
export const HIDDEN = 64;

/** Telemetry layout written by the workers at every 20 Hz control tick. */
export const TEL = Object.freeze({
  vFwd: 0, cross: 1, maxErr: 2, uTu: 3, cot: 4, rodMm: 5, slackPct: 6, slip: 7,
  comMargin: 8, epsJ: 9, over: 10, done: 11, duSq: 12, dduSq: 13, clearance: 14,
  energyNorm: 15, marginRate: 16
});
export const TEL_DIM = 17;

/** Packed parameters shared with the workers (SharedArrayBuffer Float64Array). */
export const PARAM_NAMES = Object.freeze([
  // MPC cost weights and gait/shield parameters (TunableMpcConfig)
  'horizonSteps', 'qVelocityTrack', 'qLateralRmse', 'qComMargin', 'qClearance',
  'rControlEffort', 'rSmoothnessDeltaU', 'minCableTensionN', 'maxCableTensionN', 'sgtGainKp', 'sgtGainKd',
  // SGT gait timing (SETTLE -> SHIFT trigger and RELEASE hold); the main lever on cycle rate
  'sgtSettleSpeed', 'sgtSettleMinTime', 'sgtSettleMaxTime', 'sgtReleaseTime',
  // RL reward shaping weights
  'wVelocity', 'wRmse', 'wEffort', 'wCot', 'wResidual', 'wSlack', 'wSlip', 'wComMargin',
  // residual bounds
  'maxResidualStrokeM', 'maxTotalStrokeM',
  // planner
  'gamma', 'rodLimitMm', 'targetSpeed'
]);
export const MPC_PARAM_DIM = PARAM_NAMES.length;

export const DEFAULT_PARAMS = Object.freeze({
  horizonSteps: 10, qVelocityTrack: 4.0, qLateralRmse: 2.5, qComMargin: 1.8, qClearance: 1.2,
  rControlEffort: 0.08, rSmoothnessDeltaU: 0.15, minCableTensionN: 10.0, maxCableTensionN: 150.0,
  sgtGainKp: 1.0, sgtGainKd: 0.0,
  // 'fast' timing (plain SGT uses 0.12 m/s, 0.20 s, 1.20 s, 0.35 s): ~3x progress on Level 1 in the timing sweep
  sgtSettleSpeed: 0.30, sgtSettleMinTime: 0.10, sgtSettleMaxTime: 0.45, sgtReleaseTime: 0.18,
  wVelocity: 2.5, wRmse: 1.2, wEffort: 0.05, wCot: 0.4, wResidual: 6.0, wSlack: 2.0, wSlip: 1.0, wComMargin: 0.5,
  maxResidualStrokeM: 0.030, maxTotalStrokeM: 0.120,
  gamma: 0.98, rodLimitMm: 0.05, targetSpeed: 1.30
});

export function packParams(params, out = new Float64Array(MPC_PARAM_DIM)) {
  PARAM_NAMES.forEach((name, i) => { out[i] = params[name] ?? DEFAULT_PARAMS[name]; });
  return out;
}
export function unpackParams(view) {
  const out = {};
  PARAM_NAMES.forEach((name, i) => { out[name] = view[i]; });
  return out;
}

// ------------------------------------------------------------------ morphology
/**
 * 6 x 24 synergy matrix from the shell topology: row k = +1 on the outer cables
 * incident to the first node of strut k, -1 on those at the second node.
 */
export function synergyMatrix(rover) {
  const M = new Float64Array(SYNERGY_DIM*ACT_DIM);
  rover.bars.forEach(([a, b], k) => {
    if (k >= SYNERGY_DIM) return;
    rover.outerStrings.forEach(([i, j], c) => {
      if (i === a || j === a) M[k*ACT_DIM+c] += 1;
      if (i === b || j === b) M[k*ACT_DIM+c] -= 1;
    });
  });
  return M;
}

// ------------------------------------------------------------------ observation and reward
const clip = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

/** Write the normalized 11-D MPC context (params: object or packed vector) at out[offset..]. */
export function mpcContext(params, out, offset = 0) {
  MPC_CONTEXT_KEYS.forEach((key, i) => {
    const raw = params == null ? DEFAULT_PARAMS[key]
      : ArrayBuffer.isView(params) ? params[PARAM_IDX[key]] : (params[key] ?? DEFAULT_PARAMS[key]);
    out[offset+i] = clip(raw/MPC_CONTEXT_SCALE[i], -3, 3);
  });
  return out;
}

/**
 * 73-D observation from the live simulation and the controller context:
 * [0..47] rover state, [48..61] terrain one-hot (cfg.experimentId), [62..72] MPC context
 * (ctx.params: the parameters the controller is running with; defaults if absent).
 */
export function residualObservation(ctx, out = new Float64Array(OBS_DIM)) {
  const sim = ctx.sim, n = sim.rover.nOuter;
  let cx = 0, cy = 0, cz = 0, vx = 0, vy = 0, vz = 0;
  for (let i = 0; i < n; i++) {
    cx += sim.q[i][0]; cy += sim.q[i][1]; cz += sim.q[i][2];
    vx += sim.v[i][0]; vy += sim.v[i][1]; vz += sim.v[i][2];
  }
  cx /= n; cy /= n; cz /= n; vx /= n; vy /= n; vz /= n;
  const omega = sim.calcAngularVelocityVector(cx, cy, cz, vx, vy, vz);
  const goal = sim.cfg.targetDestination || [0, sim.cfg.targetGoalY || 25];
  const ground = sim.terrain.eval(cx, cy).h || 0;
  out[0] = (cz-ground-sim.cfg.outerRadius)*4;
  out[1] = vx*8; out[2] = vy*8; out[3] = vz*8;
  out[4] = omega[0]*2; out[5] = omega[1]*2; out[6] = omega[2]*2;
  out[7] = clip((goal[0]-cx)/10, -2, 2); out[8] = clip((goal[1]-cy)/10, -2, 2);
  out[9] = clip(ctx.margin ?? 0, -0.3, 0.3)/0.3;
  out[10] = ctx.state === 'SGT_TIP' ? 1 : ctx.state === 'SGT_SHIFT' ? 0.5 : 0;
  out[11] = clip((ctx.elapsed ?? 0)/3, 0, 2);
  sim.rover.outerStrings.forEach(([i, j], c) => {
    const dx = sim.q[j][0]-sim.q[i][0], dy = sim.q[j][1]-sim.q[i][1], dz = sim.q[j][2]-sim.q[i][2];
    const length = Math.hypot(dx, dy, dz);
    const rest = sim.rover.l0_outerStrings[c]-sim.currentActuationOffsets[c];
    out[12+c] = clip((length-rest)/Math.max(rest, 1e-6)*20, -3, 3);
  });
  for (let k = 0; k < 12; k++) out[36+k] = clip((sim.coreCableForces?.[k] ?? 0)/50, 0, 4);
  const level = Math.round(sim.cfg.experimentId || 1);
  for (let k = 0; k < TERRAIN_DIM; k++) out[OBS_BASE_DIM+k] = k === level-1 ? 1 : 0;
  mpcContext(ctx.params, out, OBS_BASE_DIM+TERRAIN_DIM);
  return out;
}

/**
 * Widen first-layer weights trained on an older, smaller observation (e.g. the 48-D one)
 * to OBS_DIM: old columns are kept, new input columns start at zero, so a migrated
 * network computes exactly what it computed before.
 */
export function migrateInputWeights(flat, oldDim, hidden = HIDDEN) {
  const extra = OBS_DIM-oldDim;
  const out = new Float64Array(flat.length+hidden*extra);
  for (let h = 0; h < hidden; h++) out.set(flat.subarray ? flat.subarray(h*oldDim, (h+1)*oldDim) : flat.slice(h*oldDim, (h+1)*oldDim), h*OBS_DIM);
  out.set(flat.subarray ? flat.subarray(hidden*oldDim) : flat.slice(hidden*oldDim), hidden*OBS_DIM);
  return out;
}
const widen = (w, size) => {
  if (w.length === size) return w;
  for (const old of [48]) if (w.length === size-HIDDEN*(OBS_DIM-old)) return migrateInputWeights(Float64Array.from(w), old);
  throw new Error(`weight vector has ${w.length} entries, expected ${size}`);
};

/**
 * Stage reward of one control tick from its telemetry (higher is better).
 * `p` is the packed parameter vector (MPC cost weights + RL shaping weights).
 *
 *   r = Q_vel v - Q_rmse |e| - Q_com clip(m) - Q_clr pen(clearance)
 *       - R_eff |du|^2 - R_smooth |du - du_prev|^2             (MPC cost)
 *     + w_vel v - w_rmse |e| - w_eff u'u - w_cot dE/(m g)
 *       - w_res max(0, rod - limit) - w_slack slack - w_slip slip
 *       + w_com (-dm/dt)                                       (RL shaping)
 */
export function stageReward(tel, p) {
  const P = PARAM_IDX;
  const margin = clip(tel[TEL.comMargin], -0.1, 0.3);
  const clearancePenalty = Math.max(0, 0.08-tel[TEL.clearance])/0.08;
  let r = p[P.qVelocityTrack]*tel[TEL.vFwd]
    - p[P.qLateralRmse]*tel[TEL.cross]
    - p[P.qComMargin]*margin
    - p[P.qClearance]*clearancePenalty
    - p[P.rControlEffort]*tel[TEL.duSq]
    - p[P.rSmoothnessDeltaU]*tel[TEL.dduSq];
  r += p[P.wVelocity]*tel[TEL.vFwd]
    - p[P.wRmse]*tel[TEL.cross]
    - p[P.wEffort]*tel[TEL.uTu]/0.81
    - p[P.wCot]*tel[TEL.energyNorm]
    - p[P.wResidual]*Math.max(0, tel[TEL.rodMm]-p[P.rodLimitMm])
    - p[P.wSlack]*tel[TEL.slackPct]/100
    - p[P.wSlip]*tel[TEL.slip]
    + p[P.wComMargin]*clip(-tel[TEL.marginRate], -1, 1);
  if (tel[TEL.over] > 0) r -= 5;
  return r;
}
export const PARAM_IDX = Object.freeze(Object.fromEntries(PARAM_NAMES.map((name, i) => [name, i])));

// ------------------------------------------------------------------ Adam on a flat vector
export class Adam {
  constructor(size, lr = 1e-3, b1 = 0.9, b2 = 0.999) {
    this.m = new Float64Array(size); this.v = new Float64Array(size);
    this.lr = lr; this.b1 = b1; this.b2 = b2; this.t = 0;
  }
  step(params, grad, scale = 1) {
    this.t++;
    const c1 = 1-this.b1**this.t, c2 = 1-this.b2**this.t;
    for (let i = 0; i < params.length; i++) {
      const g = grad[i]*scale;
      this.m[i] = this.b1*this.m[i]+(1-this.b1)*g;
      this.v[i] = this.b2*this.v[i]+(1-this.b2)*g*g;
      params[i] -= this.lr*(this.m[i]/c1)/(Math.sqrt(this.v[i]/c2)+1e-8);
    }
  }
}

function randn(rng) {
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);
}
export function makeRng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s+0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t+Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0)/4294967296;
  };
}
export { randn };

// ------------------------------------------------------------------ actor
const ACTOR_SIZE = HIDDEN*OBS_DIM+HIDDEN + SYNERGY_DIM*HIDDEN+SYNERGY_DIM + ACT_DIM*HIDDEN+ACT_DIM;

/**
 * Synergy actor: trunk tanh(W1 o + b1) -> {z in R^6, r in R^24};
 * du = rho * tanh(0.7 M' tanh(z) + 0.3 r). Output heads start at zero, so the
 * untrained policy is exactly the nominal SGT command (residual-RL safe start).
 */
export class SynergyActor {
  constructor(synergy, seed = 1) {
    this.M = synergy;
    this.w = new Float64Array(ACTOR_SIZE);
    const rng = makeRng(seed), scale = Math.sqrt(1/OBS_DIM);
    for (let i = 0; i < HIDDEN*OBS_DIM; i++) this.w[i] = (rng()*2-1)*scale*1.7;
    this.slice();
    this.grad = new Float64Array(ACTOR_SIZE);
    this.opt = new Adam(ACTOR_SIZE, 1e-3);
    this.mixSynergy = 0.7; this.mixFine = 0.3;
    this.scratch = { a1: new Float64Array(HIDDEN), tz: new Float64Array(SYNERGY_DIM), y: new Float64Array(ACT_DIM), pre: new Float64Array(ACT_DIM) };
  }
  /** (Re)create typed views into the flat weight vector. */
  slice() {
    let o = 0;
    const take = n => { const v = this.w.subarray(o, o+n); o += n; return v; };
    this.W1 = take(HIDDEN*OBS_DIM); this.b1 = take(HIDDEN);
    this.Wz = take(SYNERGY_DIM*HIDDEN); this.bz = take(SYNERGY_DIM);
    this.Wr = take(ACT_DIM*HIDDEN); this.br = take(ACT_DIM);
  }
  /** Bind to externally owned (shared) weights. */
  bind(flat) { this.w = flat; this.slice(); return this; }
  forward(obs, rho = 0.030, out = new Float64Array(ACT_DIM), keep = false) {
    const { a1, tz, y, pre } = this.scratch;
    for (let h = 0; h < HIDDEN; h++) {
      let z = this.b1[h];
      const off = h*OBS_DIM;
      for (let i = 0; i < OBS_DIM; i++) z += this.W1[off+i]*obs[i];
      a1[h] = Math.tanh(z);
    }
    for (let k = 0; k < SYNERGY_DIM; k++) {
      let z = this.bz[k];
      const off = k*HIDDEN;
      for (let h = 0; h < HIDDEN; h++) z += this.Wz[off+h]*a1[h];
      tz[k] = Math.tanh(z);
    }
    for (let c = 0; c < ACT_DIM; c++) {
      let s = 0;
      for (let k = 0; k < SYNERGY_DIM; k++) s += tz[k]*this.M[k*ACT_DIM+c];
      let r = this.br[c];
      const off = c*HIDDEN;
      for (let h = 0; h < HIDDEN; h++) r += this.Wr[off+h]*a1[h];
      pre[c] = this.mixSynergy*s+this.mixFine*r;
      y[c] = Math.tanh(pre[c]);
      out[c] = rho*y[c];
    }
    return out;
  }
  /** One Adam step on the mean squared error to the target residuals (metres). Returns the loss. */
  trainBatch(obsBatch, targetBatch, rho = 0.030) {
    this.grad.fill(0);
    const gW1 = this.grad.subarray(0, HIDDEN*OBS_DIM);
    let o = HIDDEN*OBS_DIM;
    const gb1 = this.grad.subarray(o, o+HIDDEN); o += HIDDEN;
    const gWz = this.grad.subarray(o, o+SYNERGY_DIM*HIDDEN); o += SYNERGY_DIM*HIDDEN;
    const gbz = this.grad.subarray(o, o+SYNERGY_DIM); o += SYNERGY_DIM;
    const gWr = this.grad.subarray(o, o+ACT_DIM*HIDDEN); o += ACT_DIM*HIDDEN;
    const gbr = this.grad.subarray(o, o+ACT_DIM);
    const { a1, tz, y } = this.scratch;
    const dpre = new Float64Array(ACT_DIM), dtz = new Float64Array(SYNERGY_DIM), da1 = new Float64Array(HIDDEN);
    let loss = 0;
    const n = obsBatch.length;
    for (let b = 0; b < n; b++) {
      const obs = obsBatch[b], target = targetBatch[b];
      this.forward(obs, rho);
      for (let c = 0; c < ACT_DIM; c++) {
        const ty = clip(target[c]/rho, -0.999, 0.999);
        const dy = y[c]-ty;
        loss += 0.5*dy*dy;
        dpre[c] = dy*(1-y[c]*y[c]);
      }
      dtz.fill(0); da1.fill(0);
      for (let c = 0; c < ACT_DIM; c++) {
        const ds = this.mixSynergy*dpre[c], dr = this.mixFine*dpre[c];
        for (let k = 0; k < SYNERGY_DIM; k++) dtz[k] += ds*this.M[k*ACT_DIM+c];
        gbr[c] += dr;
        const off = c*HIDDEN;
        for (let h = 0; h < HIDDEN; h++) { gWr[off+h] += dr*a1[h]; da1[h] += dr*this.Wr[off+h]; }
      }
      for (let k = 0; k < SYNERGY_DIM; k++) {
        const dz = dtz[k]*(1-tz[k]*tz[k]);
        gbz[k] += dz;
        const off = k*HIDDEN;
        for (let h = 0; h < HIDDEN; h++) { gWz[off+h] += dz*a1[h]; da1[h] += dz*this.Wz[off+h]; }
      }
      for (let h = 0; h < HIDDEN; h++) {
        const dz = da1[h]*(1-a1[h]*a1[h]);
        gb1[h] += dz;
        const off = h*OBS_DIM;
        for (let i = 0; i < OBS_DIM; i++) gW1[off+i] += dz*obs[i];
      }
    }
    this.opt.step(this.w, this.grad, 1/n);
    return loss/(n*ACT_DIM);
  }
  toJSON() { return { w: Array.from(this.w), mixSynergy: this.mixSynergy, mixFine: this.mixFine }; }
  load(json) { this.w.set(widen(json.w, this.w.length)); this.mixSynergy = json.mixSynergy ?? 0.7; this.mixFine = json.mixFine ?? 0.3; return this; }
}
export const ACTOR_PARAM_COUNT = ACTOR_SIZE;

// ------------------------------------------------------------------ twin critic
const CRITIC_SIZE = HIDDEN*OBS_DIM+HIDDEN+HIDDEN+1;
const silu = z => z/(1+Math.exp(-z));
const dsilu = z => { const s = 1/(1+Math.exp(-z)); return s*(1+z*(1-s)); };

/** V(s) = w2 . silu(LayerNorm(W1 s + b1)) + b2 (TD-MPC2-style value head). */
export class ValueNet {
  constructor(seed = 1) {
    this.w = new Float64Array(CRITIC_SIZE);
    const rng = makeRng(seed), scale = Math.sqrt(2/OBS_DIM);
    for (let i = 0; i < HIDDEN*OBS_DIM; i++) this.w[i] = (rng()*2-1)*scale;
    for (let h = 0; h < HIDDEN; h++) this.w[HIDDEN*OBS_DIM+HIDDEN+h] = (rng()*2-1)*0.05;
    this.grad = new Float64Array(CRITIC_SIZE);
    this.opt = new Adam(CRITIC_SIZE, 1e-3);
    this.pre = new Float64Array(HIDDEN); this.zn = new Float64Array(HIDDEN);
    this.sigma = 1;
  }
  forward(obs) {
    const { w, pre, zn } = this;
    let mean = 0;
    for (let h = 0; h < HIDDEN; h++) {
      let z = w[HIDDEN*OBS_DIM+h];
      const off = h*OBS_DIM;
      for (let i = 0; i < OBS_DIM; i++) z += w[off+i]*obs[i];
      pre[h] = z; mean += z;
    }
    mean /= HIDDEN;
    let variance = 0;
    for (let h = 0; h < HIDDEN; h++) variance += (pre[h]-mean)**2;
    this.sigma = Math.sqrt(variance/HIDDEN+1e-5);
    let value = w[CRITIC_SIZE-1];
    const w2 = HIDDEN*OBS_DIM+HIDDEN;
    for (let h = 0; h < HIDDEN; h++) {
      zn[h] = (pre[h]-mean)/this.sigma;
      value += w[w2+h]*silu(zn[h]);
    }
    return value;
  }
  /** Mean squared error step toward `targets`. Returns the loss. */
  trainBatch(obsBatch, targets, weights = null) {
    this.grad.fill(0);
    const { w, grad, zn } = this;
    const w2 = HIDDEN*OBS_DIM+HIDDEN;
    const dzn = new Float64Array(HIDDEN), dpre = new Float64Array(HIDDEN);
    let loss = 0;
    const n = obsBatch.length;
    for (let b = 0; b < n; b++) {
      const obs = obsBatch[b];
      const v = this.forward(obs);
      const wt = weights ? weights[b] : 1;
      const err = clip(v-targets[b], -20, 20)*wt;
      loss += 0.5*err*err;
      grad[CRITIC_SIZE-1] += err;
      let m1 = 0, m2 = 0;
      for (let h = 0; h < HIDDEN; h++) {
        grad[w2+h] += err*silu(zn[h]);
        dzn[h] = err*w[w2+h]*dsilu(zn[h]);
        m1 += dzn[h]; m2 += dzn[h]*zn[h];
      }
      m1 /= HIDDEN; m2 /= HIDDEN;
      for (let h = 0; h < HIDDEN; h++) {
        dpre[h] = (dzn[h]-m1-zn[h]*m2)/this.sigma;
        grad[HIDDEN*OBS_DIM+h] += dpre[h];
        const off = h*OBS_DIM;
        for (let i = 0; i < OBS_DIM; i++) grad[off+i] += dpre[h]*obs[i];
      }
    }
    this.opt.step(w, grad, 1/n);
    return loss/n;
  }
  copyFrom(other, tau = 1) {
    for (let i = 0; i < this.w.length; i++) this.w[i] += tau*(other.w[i]-this.w[i]);
  }
}
export const CRITIC_PARAM_COUNT = CRITIC_SIZE;

/** Twin critics V1, V2 with Polyak target copies; the pessimistic min is used for bootstrapping. */
export class TwinCritic {
  constructor(seed = 1) {
    this.v1 = new ValueNet(seed); this.v2 = new ValueNet(seed+101);
    this.t1 = new ValueNet(seed); this.t2 = new ValueNet(seed+101);
    this.t1.copyFrom(this.v1); this.t2.copyFrom(this.v2);
  }
  /** min(V1, V2) of the target networks (used for bootstrapped targets). */
  target(obs) { return Math.min(this.t1.forward(obs), this.t2.forward(obs)); }
  value(obs) { return Math.min(this.v1.forward(obs), this.v2.forward(obs)); }
  train(obsBatch, targets, tau = 0.02) {
    const l1 = this.v1.trainBatch(obsBatch, targets), l2 = this.v2.trainBatch(obsBatch, targets);
    this.t1.copyFrom(this.v1, tau); this.t2.copyFrom(this.v2, tau);
    return 0.5*(l1+l2);
  }
  toJSON() { return { v1: Array.from(this.v1.w), v2: Array.from(this.v2.w) }; }
  load(json) {
    this.v1.w.set(widen(json.v1, this.v1.w.length)); this.v2.w.set(widen(json.v2, this.v2.w.length));
    this.t1.copyFrom(this.v1); this.t2.copyFrom(this.v2);
    return this;
  }
}

/** Actor + twin critic bundle (the learner state). */
export class MorphologyActorCritic {
  constructor(synergy, seed = 1) {
    this.actor = new SynergyActor(synergy, seed);
    this.critic = new TwinCritic(seed+7);
    this.params = { ...DEFAULT_PARAMS };
  }
  actPrior(obs, rho = this.params.maxResidualStrokeM, out) { return this.actor.forward(obs, rho, out); }
  evaluateValue(obs) { return this.critic.value(obs); }
  toJSON() { return { actor: this.actor.toJSON(), critic: this.critic.toJSON(), params: this.params }; }
  load(json) {
    this.actor.load(json.actor); if (json.critic) this.critic.load(json.critic);   // deployment checkpoints carry the actor only
    if (json.params) this.params = { ...this.params, ...json.params };
    return this;
  }
}

// ------------------------------------------------------------------ MPPI
/** Exponential-utility weights; temperature is relative to the spread of the returns. */
export function mppiWeights(returns, temperature = 0.5) {
  const K = returns.length;
  let max = -Infinity, mean = 0;
  for (const r of returns) { if (r > max) max = r; mean += r/K; }
  let variance = 0;
  for (const r of returns) variance += (r-mean)**2/K;
  const lambda = Math.max(temperature*Math.sqrt(variance), 1e-6);
  const w = new Float64Array(K);
  let sum = 0;
  for (let k = 0; k < K; k++) { w[k] = Math.exp((returns[k]-max)/lambda); sum += w[k]; }
  for (let k = 0; k < K; k++) w[k] /= sum;
  let ess = 0;
  for (let k = 0; k < K; k++) ess += w[k]*w[k];
  return { weights: w, ess: 1/ess, lambda };
}

/**
 * Structured exploration noise for K candidates over H ticks: low-dimensional
 * synergy noise (6-D, mapped through M) plus fine per-cable noise, correlated
 * in time with factor alpha. Candidate 0 is the noise-free prior.
 */
export function sampleNoise(out, K, H, synergy, rng, { sigmaSynergy = 0.010, sigmaFine = 0.006, alpha = 0.6, cap = 0.030 } = {}) {
  const innov = Math.sqrt(1-alpha*alpha);
  const prevZ = new Float64Array(SYNERGY_DIM), prevR = new Float64Array(ACT_DIM);
  for (let k = 0; k < K; k++) {
    prevZ.fill(0); prevR.fill(0);
    for (let t = 0; t < H; t++) {
      const base = (k*H+t)*ACT_DIM;
      if (k === 0) { for (let c = 0; c < ACT_DIM; c++) out[base+c] = 0; continue; }
      for (let j = 0; j < SYNERGY_DIM; j++) prevZ[j] = alpha*prevZ[j]+innov*randn(rng);
      for (let c = 0; c < ACT_DIM; c++) prevR[c] = alpha*prevR[c]+innov*randn(rng);
      for (let c = 0; c < ACT_DIM; c++) {
        let s = 0;
        for (let j = 0; j < SYNERGY_DIM; j++) s += prevZ[j]*synergy[j*ACT_DIM+c];
        out[base+c] = clip(sigmaSynergy*s+sigmaFine*prevR[c], -cap, cap);
      }
    }
  }
  return out;
}

/** Replay of (s, r, s', done) control-tick transitions plus MPPI return targets. */
export class Replay {
  constructor(capacity = 20000, seed = 3) {
    this.capacity = capacity; this.rng = makeRng(seed);
    this.items = []; this.next = 0;
  }
  push(item) {
    if (this.items.length < this.capacity) this.items.push(item);
    else { this.items[this.next] = item; this.next = (this.next+1)%this.capacity; }
  }
  get size() { return this.items.length; }
  sample(n) {
    const out = [];
    for (let i = 0; i < n && this.items.length; i++) out.push(this.items[Math.floor(this.rng()*this.items.length)]);
    return out;
  }
}

/** Per-synergy diagnostics: how much of the executed residual lies in the 6-D synergy subspace. */
export function synergyShare(du, synergy) {
  const coeff = new Float64Array(SYNERGY_DIM);
  let norm = 0;
  for (let k = 0; k < SYNERGY_DIM; k++) {
    let s = 0, m = 0;
    for (let c = 0; c < ACT_DIM; c++) { s += du[c]*synergy[k*ACT_DIM+c]; m += synergy[k*ACT_DIM+c]**2; }
    coeff[k] = m > 0 ? s/m : 0;
  }
  const rebuilt = new Float64Array(ACT_DIM);
  for (let k = 0; k < SYNERGY_DIM; k++) for (let c = 0; c < ACT_DIM; c++) rebuilt[c] += coeff[k]*synergy[k*ACT_DIM+c];
  let total = 0, inSub = 0;
  for (let c = 0; c < ACT_DIM; c++) { total += du[c]**2; inSub += rebuilt[c]**2; }
  norm = total;
  return norm > 1e-18 ? inSub/norm : 0;
}

/** Deployable hook (cfg.residualMppi): actor prior only, no search. */
export function residualPolicyHook(model, overrides = {}) {
  const rho = overrides.maxResidualStrokeM ?? model.params.maxResidualStrokeM;
  const obs = new Float64Array(OBS_DIM), out = new Float64Array(ACT_DIM);
  return {
    maxResidualStrokeM: rho,
    maxTotalStrokeM: overrides.maxTotalStrokeM ?? model.params.maxTotalStrokeM,
    act(ctx) {
      ctx.params = model.params;
      residualObservation(ctx, obs);
      return model.actPrior(obs, rho, out);
    }
  };
}
