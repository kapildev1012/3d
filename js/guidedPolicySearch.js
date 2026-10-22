/**
 * GUIDEDPOLICYSEARCH.JS - MPC-guided policy search for the six-bar rover
 *
 * Guided Policy Search (Levine & Koltun 2013; MPC-guided variant: Zhang,
 * Kahn, Levine & Abbeel, "Learning deep control policies for autonomous
 * aerial vehicles with MPC-guided policy search", ICRA 2016; applied to
 * SUPERball by Zhang et al., "Deep reinforcement learning for tensegrity
 * robot locomotion", ICRA 2017). Two parts:
 *
 *  Teacher (C-step): a receding-horizon QP-MPC on the pinned quasi-static
 *  KKT sensitivity model of the SGT gait. For the locked tipping edge with
 *  outward normal n, ĝ = nᵀS/max|nᵀS| is the normalized COM-shift gradient
 *  of every actuator. Over N steps of Δt with normalized commands ũ_k:
 *
 *    min Σ_k [ −ĝᵀũ_k + ½ r‖ũ_k‖² ] + ½ r_Δ Σ_k ‖ũ_k − ũ_{k−1}‖²
 *        + ½ λ ‖ũ_1 − π_θ(o)‖²
 *    s.t. lo ≤ ũ_k ≤ 1,  |ũ_k − ũ_{k−1}| ≤ ρ  (spool speed),  ũ_0 = current
 *
 *  With r = 1, r_Δ = 0, λ = 0 and no rate bound the first step is exactly
 *  the SGT command clamp(ĝ). λ > 0 is the guided-policy-search proximity
 *  term (the quadratic form of the KL constraint for fixed-covariance
 *  Gaussian policies) that keeps the teacher near the learner (PLATO style).
 *
 *  Learner (S-step): a policy π_θ(o) trained by supervised regression on
 *  the teacher's actions. It does not use the KKT model: each actuator's
 *  command is produced by one small MLP shared by all 24 outer cables and
 *  one shared by the 12 payload cables, from local features in the frame
 *  of the desired rolling direction (permutation-equivariant, so the
 *  policy is independent of which face the rover rests on).
 *
 *  At run time the policy replaces the KKT solve; the SGT state machine
 *  (settle → shift → tip → release) and support/edge detection are kept.
 */

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export const TEACHER_DEFAULTS = Object.freeze({
  horizon: 6, r: 1.0, rDelta: 0.2, lambda: 0, iterations: 60
});

// ------------------------------------------------------------------ teacher

/**
 * Per-actuator separable QP (each actuator's trajectory is independent),
 * solved by projected gradient with a sequential box/rate projection.
 * @returns normalized first-step commands ũ_1 (length = g.length)
 */
export function solveTeacherQp({ g, u0, lo, rate, piTarget = null }, options = {}) {
  const o = { ...TEACHER_DEFAULTS, ...options };
  const N = o.horizon, m = g.length;
  const out = new Array(m);
  for (let a = 0; a < m; a++) {
    const lower = lo[a], upper = 1, rho = rate[a];
    const pi = piTarget ? piTarget[a] : 0;
    const lam = piTarget ? o.lambda : 0;
    // warm start: ramp toward the unconstrained stationary point
    const target = clamp((g[a]+lam*pi)/(o.r+lam), lower, upper);
    const u = new Array(N);
    let prev = u0[a];
    for (let k = 0; k < N; k++) { prev = clamp(prev+clamp(target-prev, -rho, rho), lower, upper); u[k] = prev; }
    const L = o.r+lam+4*o.rDelta;                         // Lipschitz bound of the gradient
    const step = 1/L;
    for (let it = 0; it < o.iterations; it++) {
      const grad = new Array(N);
      for (let k = 0; k < N; k++) {
        const before = k === 0 ? u0[a] : u[k-1];
        let gk = -g[a]+o.r*u[k]+o.rDelta*(u[k]-before);
        if (k+1 < N) gk -= o.rDelta*(u[k+1]-u[k]);
        if (k === 0) gk += lam*(u[0]-pi);
        grad[k] = gk;
      }
      let p = u0[a], moved = 0;
      for (let k = 0; k < N; k++) {
        const next = clamp(clamp(u[k]-step*grad[k], p-rho, p+rho), lower, upper);
        moved = Math.max(moved, Math.abs(next-u[k]));
        u[k] = next; p = next;
      }
      if (moved < 1e-7) break;
    }
    out[a] = u[0];
  }
  return out;
}

// ------------------------------------------------------------------ features

export const OUTER_FEATURES = 20;
export const CORE_FEATURES = 19;

/**
 * Local, frame-invariant features of every actuator.
 * ctx: { sim, geometry (support face + locked edge), direction, com, margin, state, elapsed }
 */
export function actuatorFeatures(ctx) {
  const { sim, geometry, direction, com, margin, state, elapsed } = ctx;
  const rover = sim.rover, cfg = sim.cfg, R = Math.max(rover.R_outer || 0.5, 1e-6);
  const dl = Math.hypot(direction[0], direction[1]) || 1;
  const d = [direction[0]/dl, direction[1]/dl], side = [-d[1], d[0]];
  const n = geometry.normal || d, t = [-n[1], n[0]], mid = geometry.mid || [com[0], com[1]];
  const face = new Set(geometry.face || []), edge = new Set(geometry.edge || []);
  const rel = p => [((p[0]-com[0])*d[0]+(p[1]-com[1])*d[1])/R, ((p[0]-com[0])*side[0]+(p[1]-com[1])*side[1])/R, (p[2]-com[2])/R];
  const clearance = p => (p[2]-sim.terrain.eval(p[0], p[1]).h-cfg.nodeRadius)/R;
  const edgeNormal = p => ((p[0]-mid[0])*n[0]+(p[1]-mid[1])*n[1])/R;
  const edgeTangent = p => Math.abs((p[0]-mid[0])*t[0]+(p[1]-mid[1])*t[1])/R;
  const vel = sim.v.reduce((s, v) => [s[0]+v[0], s[1]+v[1]], [0, 0]).map(x => x/sim.v.length);
  const globals = [
    (vel[0]*d[0]+vel[1]*d[1])/0.5, (vel[0]*side[0]+vel[1]*side[1])/0.5,
    state === 'SGT_SHIFT' ? 1 : 0, state === 'SGT_TIP' ? 1 : 0,
    clamp((margin ?? 0)/R, -1, 1), clamp((elapsed ?? 0)/1.5, 0, 2)
  ];
  const umax = Math.max(cfg.actuationDeltaL, 1e-6), cmax = Math.max(cfg.coreActuationDeltaL, 1e-6);
  const outer = rover.outerStrings.map(([i, j], s) => {
    const qi = sim.q[i], qj = sim.q[j];
    const m = [(qi[0]+qj[0])/2, (qi[1]+qj[1])/2, (qi[2]+qj[2])/2];
    const L = Math.max(1e-9, Math.hypot(qj[0]-qi[0], qj[1]-qi[1], qj[2]-qi[2]));
    const e = [(qj[0]-qi[0])/L, (qj[1]-qi[1])/L, (qj[2]-qi[2])/L];
    const en = e[0]*n[0]+e[1]*n[1];
    const ci = clearance(qi), cj = clearance(qj);
    return [
      ...rel(m), Math.min(ci, cj), Math.max(ci, cj),
      edgeNormal(m), edgeTangent(m),
      (face.has(i) ? 1 : 0)+(face.has(j) ? 1 : 0), (edge.has(i) ? 1 : 0)+(edge.has(j) ? 1 : 0),
      en*e[2], en*en, e[2]*e[2],
      sim.currentActuationOffsets[s]/umax, L/rover.l0_outerStrings[s]-1,
      ...globals
    ];
  });
  const core = sim.corePosition;
  const coreFeatures = sim.q.map((p, i) => {
    const L = Math.max(1e-9, Math.hypot(core[0]-p[0], core[1]-p[1], core[2]-p[2]));
    const e = [(core[0]-p[0])/L, (core[1]-p[1])/L, (core[2]-p[2])/L];
    return [
      ...rel(p), clearance(p), edgeNormal(p), edgeTangent(p),
      face.has(i) ? 1 : 0, edge.has(i) ? 1 : 0,
      e[0]*d[0]+e[1]*d[1], e[0]*side[0]+e[1]*side[1], e[2],
      (sim.currentCoreActuationOffsets?.[i] || 0)/cmax, L/R-1,
      ...globals
    ].slice(0, CORE_FEATURES);
  });
  return { outer, core: coreFeatures };
}

// ------------------------------------------------------------------ MLP

function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a+0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t+Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0)/4294967296;
  };
}

/** Small fully connected tanh network with a tanh output in [-1, 1]. */
export class Mlp {
  constructor(sizes, seed = 1, weights = null) {
    this.sizes = sizes;
    if (weights?.W) { this.W = weights.W; this.b = weights.b; this.mu = weights.mu; this.sd = weights.sd; return; }
    const rand = mulberry32(seed);
    this.W = []; this.b = [];
    for (let l = 0; l+1 < sizes.length; l++) {
      const scale = Math.sqrt(1/sizes[l]);
      this.W.push(Array.from({ length: sizes[l+1] }, () => Array.from({ length: sizes[l] }, () => (2*rand()-1)*scale*1.7)));
      this.b.push(new Array(sizes[l+1]).fill(0));
    }
    this.mu = new Array(sizes[0]).fill(0);
    this.sd = new Array(sizes[0]).fill(1);
  }

  forward(x, keep = false) {
    let a = x.map((v, i) => (v-this.mu[i])/this.sd[i]);
    const acts = [a];
    for (let l = 0; l < this.W.length; l++) {
      const z = this.W[l].map((row, j) => row.reduce((s, w, i) => s+w*a[i], this.b[l][j]));
      a = z.map(Math.tanh);
      acts.push(a);
    }
    return keep ? acts : a;
  }

  /** Normalization statistics from the training inputs. */
  fitNormalization(X) {
    const n = X.length, d = this.sizes[0];
    this.mu = new Array(d).fill(0); this.sd = new Array(d).fill(0);
    for (const x of X) for (let i = 0; i < d; i++) this.mu[i] += x[i]/n;
    for (const x of X) for (let i = 0; i < d; i++) this.sd[i] += (x[i]-this.mu[i])**2/n;
    this.sd = this.sd.map(v => Math.max(Math.sqrt(v), 1e-3));
  }

  /** Mini-batch Adam on the mean-squared error; returns the final training loss. */
  train(X, Y, { epochs = 30, batch = 128, lr = 3e-3, seed = 1, weights = null } = {}) {
    const rand = mulberry32(seed);
    const mW = this.W.map(L => L.map(r => r.map(() => 0))), vW = this.W.map(L => L.map(r => r.map(() => 0)));
    const mb = this.b.map(L => L.map(() => 0)), vb = this.b.map(L => L.map(() => 0));
    let step = 0, loss = 0;
    const idx = X.map((_, i) => i);
    for (let ep = 0; ep < epochs; ep++) {
      for (let i = idx.length-1; i > 0; i--) { const j = Math.floor(rand()*(i+1)); [idx[i], idx[j]] = [idx[j], idx[i]]; }
      loss = 0;
      for (let start = 0; start < idx.length; start += batch) {
        const gW = this.W.map(L => L.map(r => r.map(() => 0))), gb = this.b.map(L => L.map(() => 0));
        const members = idx.slice(start, start+batch);
        let wsum = 0;
        for (const i of members) {
          const w = weights ? weights[i] : 1;
          wsum += w;
          const acts = this.forward(X[i], true);
          const out = acts[acts.length-1];
          let delta = out.map((y, k) => w*2*(y-Y[i][k])*(1-y*y));
          loss += w*out.reduce((s, y, k) => s+(y-Y[i][k])**2, 0);
          for (let l = this.W.length-1; l >= 0; l--) {
            const a = acts[l];
            for (let j = 0; j < this.W[l].length; j++) {
              gb[l][j] += delta[j];
              const row = gW[l][j];
              for (let k = 0; k < a.length; k++) row[k] += delta[j]*a[k];
            }
            if (l > 0) {
              const prev = acts[l];
              delta = prev.map((v, k) => this.W[l].reduce((s, row, j) => s+row[k]*delta[j], 0)*(1-v*v));
            }
          }
        }
        step++;
        const b1 = 0.9, b2 = 0.999, eps = 1e-8, scale = 1/Math.max(wsum, 1e-9);
        const c1 = 1-b1**step, c2 = 1-b2**step;
        for (let l = 0; l < this.W.length; l++) {
          for (let j = 0; j < this.W[l].length; j++) {
            for (let k = 0; k < this.W[l][j].length; k++) {
              const g = gW[l][j][k]*scale;
              mW[l][j][k] = b1*mW[l][j][k]+(1-b1)*g; vW[l][j][k] = b2*vW[l][j][k]+(1-b2)*g*g;
              this.W[l][j][k] -= lr*(mW[l][j][k]/c1)/(Math.sqrt(vW[l][j][k]/c2)+eps);
            }
            const g = gb[l][j]*scale;
            mb[l][j] = b1*mb[l][j]+(1-b1)*g; vb[l][j] = b2*vb[l][j]+(1-b2)*g*g;
            this.b[l][j] -= lr*(mb[l][j]/c1)/(Math.sqrt(vb[l][j]/c2)+eps);
          }
        }
      }
      loss /= weights ? weights.reduce((s, w) => s+w, 0) : X.length;
    }
    return loss;
  }

  toJSON() {
    const round = v => Math.round(v*1e6)/1e6;
    return { sizes: this.sizes, W: this.W.map(L => L.map(r => r.map(round))), b: this.b.map(L => L.map(round)),
      mu: this.mu.map(round), sd: this.sd.map(round) };
  }
}

// ------------------------------------------------------------------ policy

/** Normalized command bounds of the actuators (outer may relax by relaxLimit). */
export function actuatorBounds(sim, relaxLimit = 0.35, coreStrokeRatio = 1) {
  const umax = sim.cfg.actuationDeltaL, cmax = sim.cfg.coreActuationDeltaL*coreStrokeRatio;
  const dt = Math.max(sim.cfg.controllerDt || 0.05, 1e-3);
  return {
    umax, cmax,
    lo: [...new Array(24).fill(-relaxLimit), ...new Array(12).fill(-1)],
    rate: [...new Array(24).fill((sim.cfg.cableLinearVelocity || 0.2)*dt/Math.max(umax, 1e-6)),
      ...new Array(12).fill((sim.cfg.coreCableLinearVelocity || 0.2)*dt/Math.max(cmax, 1e-6))]
  };
}

export class GuidedPolicy {
  constructor(weights = null, seed = 1) {
    this.outer = new Mlp(weights?.outer?.sizes || [OUTER_FEATURES, 32, 32, 1], seed, weights?.outer);
    this.core = new Mlp(weights?.core?.sizes || [CORE_FEATURES, 24, 24, 1], seed+1, weights?.core);
    this.meta = weights?.meta || null;
    this.trained = Boolean(weights?.outer?.W);
  }

  /** Normalized commands ũ (24 outer in [-relax,1], 12 core in [-1,1]). */
  normalizedCommand(features, relaxLimit = 0.35) {
    const outer = features.outer.map(x => clamp(this.outer.forward(x)[0], -relaxLimit, 1));
    const core = features.core.map(x => this.core.forward(x)[0]);
    return [...outer, ...core];
  }

  act(ctx) {
    const features = actuatorFeatures(ctx);
    const relax = ctx.relaxLimit ?? 0.35;
    const u = this.normalizedCommand(features, relax);
    const { umax, cmax } = actuatorBounds(ctx.sim, relax, ctx.coreStrokeRatio ?? 1);
    return {
      normalized: u, features,
      cableTargets: u.slice(0, 24).map(v => v*umax),
      coreTargets: u.slice(24).map(v => v*cmax)
    };
  }

  toJSON() { return { outer: this.outer.toJSON(), core: this.core.toJSON(), meta: this.meta }; }
}
