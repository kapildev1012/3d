/**
 * GPSNAVIGATION.JS - MPC + GPS waypoint navigation for the rolling rover
 *
 * The other controllers steer from the simulator's true centre of mass. The
 * MPC + GPS controller only sees what a GNSS receiver would report:
 *
 *  1. GpsReceiver: fixes at `rate` Hz with horizontal noise σ (white Gaussian
 *     + slowly drifting bias, Gauss–Markov), a fixed latency and random
 *     dropouts. Seeded, so runs are reproducible.
 *  2. GpsKalmanFilter: constant-velocity Kalman filter on [x, y, vx, vy],
 *     predicted at the 20 Hz control rate. A fix taken `latency` seconds ago
 *     is fused with H = [I, −L·I] (position back-propagated along velocity),
 *     and fixes that fail a χ² gate (outliers, multipath) are rejected.
 *  3. solveSteeringMpc: receding-horizon MPC on the path frame. State
 *     [e, ψ] (cross-track error from the start→goal line, heading error of
 *     the rolling direction), input ω (heading rate) with hard bounds
 *     |ω| ≤ ωmax, solved by projected gradient on the horizon:
 *        e⁺ = e + Δt·v·sin ψ,  ψ⁺ = ψ + Δt·ω
 *        J  = Σ qe e² + qψ ψ² + rω ω² + rΔ (ω−ω₋)² + terminal
 *     The first predicted heading sets the rolling direction; the existing
 *     QP-MPC (advancedControllers.js) sets the cable effort along it.
 */

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

export const GPS_DEFAULTS = Object.freeze({
  rate: 5,            // Hz, fix rate
  sigma: 0.30,        // m, white horizontal noise (1σ per axis); RTK ≈ 0.02
  biasSigma: 0.20,    // m, steady-state 1σ of the slowly varying bias
  biasTau: 60,        // s, bias correlation time
  latency: 0.20,      // s, fix age when it reaches the controller
  dropout: 0.03,      // probability that a fix is lost
  accelNoise: 0.8,    // m/s², Kalman process noise (white acceleration)
  gate: 13.8          // χ²(2) gate for outlier rejection (99.9 %)
});

export const GPS_MPC_DEFAULTS = Object.freeze({
  horizon: 15,
  dt: 0.2,            // s per MPC step (3 s look-ahead)
  omegaMax: 0.6,      // rad/s, heading-rate bound of the rolling sphere
  psiMax: 1.1,        // rad, never roll more than ~63° off the path
  qe: 4.0, qpsi: 1.2, rw: 0.6, rdw: 2.0, qeT: 12.0, qpsiT: 3.0,
  minSpeed: 0.15      // m/s, speed floor for the e-dynamics
});

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

export class GpsReceiver {
  constructor(options = {}, seed = 1) {
    this.o = { ...GPS_DEFAULTS, ...options };
    this.rand = mulberry32((seed*2654435761) ^ 0x47505331);
    this.spare = null;
    this.history = [];           // [{t, x, y}] true positions for the latency buffer
    this.nextFix = 0;
    this.bias = [this.gauss()*this.o.biasSigma, this.gauss()*this.o.biasSigma];
    this.lastBiasT = 0;
    this.fixes = 0;
    this.dropped = 0;
  }

  gauss() {
    if (this.spare !== null) { const s = this.spare; this.spare = null; return s; }
    let u = 0, v = 0;
    while (u <= 1e-12) u = this.rand();
    v = this.rand();
    const r = Math.sqrt(-2*Math.log(u));
    this.spare = r*Math.sin(2*Math.PI*v);
    return r*Math.cos(2*Math.PI*v);
  }

  /** Record the true position; returns a fix {t, x, y, sigma} when one is delivered at time t. */
  sample(t, truePosition) {
    this.history.push({ t, x: truePosition[0], y: truePosition[1] });
    while (this.history.length > 2 && this.history[1].t < t-this.o.latency-0.5) this.history.shift();
    if (t+1e-9 < this.nextFix) return null;
    this.nextFix = Math.max(this.nextFix+1/this.o.rate, t+0.5/this.o.rate);
    // Gauss–Markov bias update
    const dt = Math.max(0, t-this.lastBiasT);
    this.lastBiasT = t;
    const phi = Math.exp(-dt/this.o.biasTau);
    const drive = this.o.biasSigma*Math.sqrt(Math.max(0, 1-phi*phi));
    this.bias = this.bias.map(b => phi*b+drive*this.gauss());
    if (this.rand() < this.o.dropout) { this.dropped++; return null; }
    const tm = t-this.o.latency;
    const past = this.positionAt(tm);
    this.fixes++;
    return {
      t: tm, receivedAt: t, sigma: this.o.sigma,
      x: past.x+this.bias[0]+this.o.sigma*this.gauss(),
      y: past.y+this.bias[1]+this.o.sigma*this.gauss()
    };
  }

  positionAt(t) {
    const h = this.history;
    if (t <= h[0].t) return h[0];
    for (let k = h.length-1; k > 0; k--) {
      if (h[k-1].t <= t) {
        const a = h[k-1], b = h[k];
        const w = b.t > a.t ? (t-a.t)/(b.t-a.t) : 1;
        return { x: a.x+w*(b.x-a.x), y: a.y+w*(b.y-a.y) };
      }
    }
    return h[h.length-1];
  }
}

/** Constant-velocity Kalman filter, state [x, y, vx, vy]. */
export class GpsKalmanFilter {
  constructor(options = {}) {
    this.o = { ...GPS_DEFAULTS, ...options };
    this.x = null;
    this.P = null;
    this.t = 0;
    this.accepted = 0;
    this.rejected = 0;
    this.lastInnovation = 0;
  }

  get initialized() { return this.x !== null; }

  init(fix) {
    const r = this.o.sigma**2+this.o.biasSigma**2;
    this.x = [fix.x, fix.y, 0, 0];
    this.P = [[r, 0, 0, 0], [0, r, 0, 0], [0, 0, 0.25, 0], [0, 0, 0, 0.25]];
    this.t = fix.receivedAt;
  }

  predict(t) {
    if (!this.initialized) return;
    const dt = Math.max(0, t-this.t);
    this.t = t;
    if (dt === 0) return;
    const [x, y, vx, vy] = this.x;
    this.x = [x+dt*vx, y+dt*vy, vx, vy];
    // P ← F P Fᵀ + Q (per axis: [[1,dt],[0,1]], white-acceleration Q)
    const q = this.o.accelNoise**2;
    const Q = [dt**4/4*q, dt**3/2*q, dt*dt*q];
    for (const [p, v] of [[0, 2], [1, 3]]) {
      const Ppp = this.P[p][p], Ppv = this.P[p][v], Pvv = this.P[v][v];
      this.P[p][p] = Ppp+2*dt*Ppv+dt*dt*Pvv+Q[0];
      this.P[p][v] = this.P[v][p] = Ppv+dt*Pvv+Q[1];
      this.P[v][v] = Pvv+Q[2];
    }
  }

  /** Fuse a delayed fix: z = p(t−L) ≈ p − L·v. Returns true when accepted. */
  update(fix) {
    if (!this.initialized) { this.init(fix); this.accepted++; return true; }
    const L = Math.max(0, this.t-fix.t);
    const R = this.o.sigma**2+this.o.biasSigma**2*0.25;
    let ok = true;
    // axes are decoupled in this model: two independent 2-state updates
    const innovations = [];
    for (const [p, v, z] of [[0, 2, fix.x], [1, 3, fix.y]]) {
      const yv = z-(this.x[p]-L*this.x[v]);
      const S = this.P[p][p]-2*L*this.P[p][v]+L*L*this.P[v][v]+R;
      innovations.push(yv*yv/S);
    }
    const nis = innovations[0]+innovations[1];
    this.lastInnovation = nis;
    if (nis > this.o.gate && this.accepted > 3) { this.rejected++; ok = false; return ok; }
    for (const [p, v, z] of [[0, 2, fix.x], [1, 3, fix.y]]) {
      const yv = z-(this.x[p]-L*this.x[v]);
      const PHt = [this.P[p][p]-L*this.P[p][v], this.P[v][p]-L*this.P[v][v]];
      const S = PHt[0]-L*PHt[1]+R;
      const K = [PHt[0]/S, PHt[1]/S];
      this.x[p] += K[0]*yv;
      this.x[v] += K[1]*yv;
      const Ppp = this.P[p][p], Ppv = this.P[p][v], Pvv = this.P[v][v];
      // P ← (I − K H) P with H = [1, −L]
      this.P[p][p] = Ppp-K[0]*(Ppp-L*Ppv);
      this.P[p][v] = Ppv-K[0]*(Ppv-L*Pvv);
      this.P[v][p] = this.P[p][v];
      this.P[v][v] = Pvv-K[1]*(Ppv-L*Pvv);
    }
    this.accepted++;
    return ok;
  }

  get position() { return this.x ? [this.x[0], this.x[1]] : null; }
  get velocity() { return this.x ? [this.x[2], this.x[3]] : [0, 0]; }
  get positionSigma() { return this.P ? Math.sqrt(0.5*(this.P[0][0]+this.P[1][1])) : Infinity; }
}

/**
 * Steering MPC on the path frame (projected gradient, hard bounds on ω and ψ).
 * @param ctx { e, psi, speeds[] (predicted forward speed per step), previousOmega }
 */
export function solveSteeringMpc(ctx, options = {}) {
  const o = { ...GPS_MPC_DEFAULTS, ...options };
  const N = o.horizon;
  const speedAt = k => Math.max(o.minSpeed, ctx.speeds?.[Math.min(k, (ctx.speeds?.length || 1)-1)] ?? o.minSpeed);
  const rollout = omegas => {
    let e = ctx.e, psi = ctx.psi, cost = 0, prev = ctx.previousOmega || 0;
    const psis = [];
    for (let k = 0; k < N; k++) {
      const w = omegas[k];
      cost += o.qe*e*e+o.qpsi*psi*psi+o.rw*w*w+o.rdw*(w-prev)*(w-prev);
      e += o.dt*speedAt(k)*Math.sin(psi);
      psi = clamp(psi+o.dt*w, -o.psiMax, o.psiMax);
      psis.push(psi);
      prev = w;
    }
    cost += o.qeT*e*e+o.qpsiT*psi*psi;
    return { cost, psis, eN: e };
  };
  // Warm start: proportional heading-rate law toward the pure-pursuit heading
  const lookAhead = Math.max(1.0, 2.5*speedAt(0));
  const psiTarget = clamp(-Math.atan2(ctx.e, lookAhead), -o.psiMax, o.psiMax);
  let omegas = new Array(N).fill(0).map((_, k) => clamp((psiTarget-ctx.psi)/Math.max(o.dt*(k+3), 1e-3), -o.omegaMax, o.omegaMax));
  let best = rollout(omegas);
  let iterations = 0;
  for (let iter = 0; iter < 20; iter++) {
    const eps = 1e-4;
    const grad = omegas.map((w, k) => {
      const plus = omegas.slice(); plus[k] = w+eps;
      const minus = omegas.slice(); minus[k] = w-eps;
      return (rollout(plus).cost-rollout(minus).cost)/(2*eps);
    });
    let improved = false;
    for (const step of [0.25, 0.08, 0.025, 0.008]) {
      const next = omegas.map((w, k) => clamp(w-step*grad[k], -o.omegaMax, o.omegaMax));
      const candidate = rollout(next);
      iterations++;
      if (candidate.cost < best.cost-1e-9) { omegas = next; best = candidate; improved = true; break; }
    }
    if (!improved) break;
  }
  return { omega: omegas[0], psiCommand: best.psis[0], cost: best.cost, iterations, omegas, predictedPsi: best.psis, terminalCrossTrack: best.eN };
}

/**
 * Per-rover navigator: GPS receiver + Kalman filter + steering MPC.
 * step() is called once per control tick with the true centroid (fed only to
 * the receiver) and returns the estimate and the commanded rolling direction.
 */
export class GpsMpcNavigator {
  constructor({ gps = {}, mpc = {}, seed = 1, start = null, goal = [0, 25] } = {}) {
    this.receiver = new GpsReceiver(gps, seed);
    this.filter = new GpsKalmanFilter(gps);
    this.mpcOptions = mpc;
    this.start = start;
    this.goal = goal;
    this.previousOmega = 0;
    this.lastFix = null;
    this.last = null;
  }

  step(t, trueCentroid, { forwardSpeeds = null } = {}) {
    const fix = this.receiver.sample(t, trueCentroid);
    this.filter.predict(t);
    let fused = false;
    if (fix) { fused = this.filter.update(fix); this.lastFix = fix; }
    if (!this.filter.initialized) return null;
    const position = this.filter.position;
    const velocity = this.filter.velocity;
    if (!this.start) this.start = position.slice();
    // Path frame: line start → goal; heading error from course over ground
    const path = [this.goal[0]-this.start[0], this.goal[1]-this.start[1]];
    const pathLength = Math.hypot(...path) || 1;
    const tangent = [path[0]/pathLength, path[1]/pathLength];
    const theta = Math.atan2(tangent[1], tangent[0]);
    const rel = [position[0]-this.start[0], position[1]-this.start[1]];
    const crossTrack = tangent[0]*rel[1]-tangent[1]*rel[0];       // + = left of path
    const alongTrack = tangent[0]*rel[0]+tangent[1]*rel[1];
    const speed = Math.hypot(...velocity);
    // Heading state: course over ground when moving, else the last command
    const course = speed > 0.12 ? Math.atan2(velocity[1], velocity[0]) : (this.last ? theta+this.last.psiCommand : theta);
    const psi = clamp(wrap(course-theta), -GPS_MPC_DEFAULTS.psiMax, GPS_MPC_DEFAULTS.psiMax);
    // e is defined positive to the left, so e⁺ = e + v sin ψ with ψ measured CCW
    const solution = solveSteeringMpc({
      e: crossTrack, psi, previousOmega: this.previousOmega,
      speeds: forwardSpeeds || [Math.max(speed, GPS_MPC_DEFAULTS.minSpeed)]
    }, this.mpcOptions);
    this.previousOmega = solution.omega;
    const heading = theta+solution.psiCommand;
    const direction = [Math.cos(heading), Math.sin(heading), 0];
    const truthError = Math.hypot(position[0]-trueCentroid[0], position[1]-trueCentroid[1]);
    this.last = {
      position, velocity, crossTrack, alongTrack, psi, psiCommand: solution.psiCommand,
      omega: solution.omega, direction, fused, fix, truthError,
      sigma: this.filter.positionSigma, fixes: this.receiver.fixes, dropped: this.receiver.dropped,
      rejected: this.filter.rejected, mpcCost: solution.cost, mpcIterations: solution.iterations,
      remaining: Math.hypot(this.goal[0]-position[0], this.goal[1]-position[1])
    };
    return this.last;
  }
}
