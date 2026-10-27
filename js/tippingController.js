import { shieldCommand } from './tensionShield.js?v=20261027-ob1';
/**
 * TIPPINGCONTROLLER.JS - Sensitivity-Gradient Tipping (SGT) gait
 *
 * Physically admissible punctuated rolling for the six-bar shell: locomotion
 * is produced only by cable rest-length changes (outer cables and payload
 * suspension cables); no external force or torque is applied.
 *
 * At every controller tick the planner linearizes the quasi-static
 * equilibrium of the shell + payload core with the three support nodes
 * pinned (sticking friction) and the six bars rigid:
 *
 *   K dq = B du,    A dq = 0,     A = [Rq ; pinned support rows]
 *
 *   K  tangent stiffness: cables  k e e' + (t/L)(I - e e')  (taut only),
 *      bars   -(P/L)(I - e e')  (geometric softening by compression P),
 *   B  dF/du for every actuated cable (du > 0 shortens the rest length).
 *
 * The KKT system [K A'; A 0] is factorized once and solved for every
 * actuator column, giving the COM sensitivity  S = dCOM/du  (3 x nu). The
 * command is the box-constrained ascent direction of the COM displacement
 * along the outward normal n of the chosen tipping edge:
 *
 *   u_k = umax_k * clamp(g_k / max|g|, -relaxLimit, 1),   g = n' S_xy
 *
 * The state machine (SETTLE -> SHIFT -> TIP -> RELEASE) holds the command
 * until the support face changes (one completed tip), releases all cables,
 * waits for the shell to settle and repeats.
 */

const STATES = Object.freeze(['SGT_SETTLE', 'SGT_SHIFT', 'SGT_TIP', 'SGT_RELEASE']);
export const SGT_STATES = STATES;

export const SGT_DEFAULTS = Object.freeze({
  settleSpeed: 0.12,        // max node speed to start a cycle [m/s]
  settleMinTime: 0.20,      // [s]
  settleMaxTime: 1.20,      // start the next cycle even if a light mode still rings [s]
  minAlignment: 0.25,       // only edges whose outward normal points forward
  shiftTimeout: 3.0,        // abort a cycle that cannot tip [s]
  tipHoldTimeout: 1.5,      // [s]
  releaseTime: 0.35,        // [s]
  relaxLimit: 0.35,         // max lengthening as a fraction of umax
  coreStrokeRatio: 1.0,     // payload stroke scale
  activeCables: 24,         // number of outer cables used per cycle (sparsity)
  regularization: 1e-6,     // relative Tikhonov term on K
  edgeSelection: 'feasibility', // 'feasibility' | 'alignment' (ablation)
  ignoreBarCompression: false   // ablation: drop the bars' geometric stiffness
});

export class SensitivityTippingController {
  constructor(sim, options = {}) {
    this.sim = sim;
    this.opts = { ...SGT_DEFAULTS, ...options };
    this.state = 'SGT_SETTLE';
    this.enteredAt = sim.t;
    this.cycleSignature = '';
    this.targetEdge = [];
    this.tipCount = 0;
    this.abortCount = 0;
    this.lastPlan = null;
  }

  enter(state) {
    if (state !== this.state) {
      this.state = state;
      this.enteredAt = this.sim.t;
    }
  }

  /** Support face (3 lowest nodes) and the edge whose outward normal best matches d. */
  supportGeometry(direction) {
    const sim = this.sim;
    const ranked = sim.q.map((p, node) => ({
      node, clearance: p[2]-sim.terrain.eval(p[0], p[1]).h-sim.cfg.nodeRadius
    })).sort((a, b) => a.clearance-b.clearance);
    const face = ranked.slice(0, 3).map(item => item.node);
    let best = null;
    const edges = [];
    for (let a = 0; a < 3; a++) {
      for (let b = a+1; b < 3; b++) {
        const i = face[a], j = face[b];
        const third = face.find(node => node !== i && node !== j);
        const qi = sim.q[i], qj = sim.q[j], qk = sim.q[third];
        const ex = qj[0]-qi[0], ey = qj[1]-qi[1];
        const len = Math.hypot(ex, ey);
        if (len < 1e-9) continue;
        let nx = -ey/len, ny = ex/len;
        const mx = 0.5*(qi[0]+qj[0]), my = 0.5*(qi[1]+qj[1]);
        if ((qk[0]-mx)*nx+(qk[1]-my)*ny > 0) { nx = -nx; ny = -ny; }
        const alignment = nx*direction[0]+ny*direction[1];
        const candidate = { edge: [i, j], normal: [nx, ny], mid: [mx, my], alignment };
        edges.push(candidate);
        if (!best || alignment > best.alignment) best = candidate;
      }
    }
    return { face, signature: face.slice().sort((a, b) => a-b).join('-'), edges, ...best };
  }

  /** Mass-weighted centre of mass of shell + payload core. */
  centreOfMass() {
    const sim = this.sim;
    const m = sim.dynamicNodeMass, mc = sim.cfg.coreMass;
    const c = [0, 0, 0];
    for (const p of sim.q) for (let a = 0; a < 3; a++) c[a] += m*p[a];
    for (let a = 0; a < 3; a++) c[a] += mc*sim.corePosition[a];
    const total = m*sim.q.length+mc;
    return c.map(value => value/total);
  }

  /**
   * COM sensitivity to every actuator, from the pinned quasi-static KKT system.
   * Actuators: 0..23 outer cables, 24..35 payload suspension cables.
   */
  sensitivity(face) {
    const sim = this.sim, cfg = sim.cfg, rover = sim.rover;
    const n = rover.nOuter+1;           // + payload core node
    const core = n-1;
    const nq = 3*n;
    const pos = i => (i === core ? sim.corePosition : sim.q[i]);
    const K = new Float64Array(nq*nq);
    const addBlock = (i, j, Ke) => {
      for (let a = 0; a < 3; a++) {
        for (let b = 0; b < 3; b++) {
          const value = Ke[3*a+b];
          K[(3*i+a)*nq+3*i+b] += value;
          K[(3*j+a)*nq+3*j+b] += value;
          K[(3*i+a)*nq+3*j+b] -= value;
          K[(3*j+a)*nq+3*i+b] -= value;
        }
      }
    };
    const member = (i, j) => {
      const pi = pos(i), pj = pos(j);
      const d = [pj[0]-pi[0], pj[1]-pi[1], pj[2]-pi[2]];
      const L = Math.max(1e-9, Math.hypot(d[0], d[1], d[2]));
      return { e: d.map(x => x/L), L };
    };
    const cableBlock = (e, L, k, t) => {
      const Ke = new Float64Array(9);
      for (let a = 0; a < 3; a++) {
        for (let b = 0; b < 3; b++) {
          const eab = e[a]*e[b];
          Ke[3*a+b] = k*eab+(t/L)*((a === b ? 1 : 0)-eab);
        }
      }
      return Ke;
    };

    const actuators = [];
    // Outer cables
    rover.outerStrings.forEach(([i, j], idx) => {
      const { e, L } = member(i, j);
      const restLength = rover.l0_outerStrings[idx]-sim.currentActuationOffsets[idx];
      const taut = L > restLength;
      const t = taut ? cfg.kS*(L-restLength) : 0;
      if (taut) addBlock(i, j, cableBlock(e, L, cfg.kS, t));
      actuators.push({ kind: 'outer', index: idx, i, j, e, gain: taut ? cfg.kS : 0 });
    });
    // Payload suspension cables (node i <-> core)
    for (let i = 0; i < rover.nOuter; i++) {
      const { e, L } = member(i, core);
      const restLength = sim.coreRestLengths[i];
      const taut = L > restLength;
      const t = taut ? cfg.kCore*(L-restLength) : 0;
      if (taut) addBlock(i, core, cableBlock(e, L, cfg.kCore, t));
      actuators.push({ kind: 'core', index: i, i, j: core, e, gain: taut ? cfg.kCore : 0 });
    }
    // Bars: geometric stiffness of the compression P (axial rigidity is a constraint)
    const barForces = sim.lastBarForces || sim.currentDiag?.strutForces || [];
    rover.bars.forEach(([i, j], k) => {
      const { e, L } = member(i, j);
      const P = this.opts.ignoreBarCompression ? 0 : (Number.isFinite(barForces[k]) ? barForces[k] : 0);
      const Ke = new Float64Array(9);
      for (let a = 0; a < 3; a++) {
        for (let b = 0; b < 3; b++) Ke[3*a+b] = -(P/L)*((a === b ? 1 : 0)-e[a]*e[b]);
      }
      addBlock(i, j, Ke);
    });
    // Tikhonov regularization (relative)
    let trace = 0;
    for (let r = 0; r < nq; r++) trace += K[r*nq+r];
    const reg = this.opts.regularization*Math.max(trace/nq, 1);
    for (let r = 0; r < nq; r++) K[r*nq+r] += reg;

    // Constraint rows: bars (Rq) and pinned support nodes
    const rows = [];
    rover.bars.forEach(([i, j]) => {
      const row = new Float64Array(nq);
      const pi = pos(i), pj = pos(j);
      for (let a = 0; a < 3; a++) {
        row[3*i+a] = -(pj[a]-pi[a]);
        row[3*j+a] = pj[a]-pi[a];
      }
      rows.push(row);
    });
    for (const node of face) {
      for (let a = 0; a < 3; a++) {
        const row = new Float64Array(nq);
        row[3*node+a] = 1;
        rows.push(row);
      }
    }
    const nc = rows.length;
    const dim = nq+nc;
    const KKT = new Float64Array(dim*dim);
    for (let r = 0; r < nq; r++) for (let c = 0; c < nq; c++) KKT[r*dim+c] = K[r*nq+c];
    rows.forEach((row, k) => {
      for (let c = 0; c < nq; c++) {
        KKT[(nq+k)*dim+c] = row[c];
        KKT[c*dim+nq+k] = row[c];
      }
    });
    const lu = luFactor(KKT, dim);

    const masses = new Float64Array(n).fill(sim.dynamicNodeMass);
    masses[core] = cfg.coreMass;
    const totalMass = masses.reduce((s, m) => s+m, 0);
    const displacements = [];
    const S = actuators.map(act => {
      const rhs = new Float64Array(dim);
      if (act.gain > 0) {
        for (let a = 0; a < 3; a++) {
          rhs[3*act.i+a] += act.gain*act.e[a];
          rhs[3*act.j+a] -= act.gain*act.e[a];
        }
      }
      const x = luSolve(lu, rhs);
      displacements.push(x.subarray ? x.subarray(0, nq) : x.slice(0, nq));
      const com = [0, 0, 0];
      for (let node = 0; node < n; node++) {
        for (let a = 0; a < 3; a++) com[a] += masses[node]*x[3*node+a];
      }
      return com.map(value => value/totalMass);
    });
    return { actuators, S, displacements };
  }

  /** Achievable COM shift along n for a gradient g under the box limits. */
  achievable(g, actuators) {
    const cfg = this.sim.cfg;
    let shift = 0;
    actuators.forEach((act, k) => {
      const umax = act.kind === 'outer' ? cfg.actuationDeltaL : cfg.coreActuationDeltaL*this.opts.coreStrokeRatio;
      const lo = act.kind === 'outer' ? -this.opts.relaxLimit*umax : -umax;
      shift += g[k] > 0 ? g[k]*umax : g[k]*lo;
    });
    return shift;
  }

  plan(direction) {
    const sim = this.sim, cfg = sim.cfg;
    const support = this.supportGeometry(direction);
    // Learned policy (guided policy search): no KKT solve; edge by alignment.
    if (this.opts.policy && !this.opts.teacher) return this.planWithPolicy(direction, support);
    const { actuators, S } = this.sensitivity(support.face);
    const com = this.centreOfMass();
    const gradientFor = edge => S.map(col => col[0]*edge.normal[0]+col[1]*edge.normal[1]);
    const marginFor = edge => -((com[0]-edge.mid[0])*edge.normal[0]+(com[1]-edge.mid[1])*edge.normal[1]);
    // Edge selection, locked for the cycle: among forward-facing edges pick
    // the one whose achievable COM shift exceeds its margin by the most.
    let chosenEdge = null;
    if (this.cycleEdgeKey) {
      chosenEdge = support.edges.find(edge => edge.edge.slice().sort((a, b) => a-b).join('-') === this.cycleEdgeKey) || null;
    }
    if (!chosenEdge && this.opts.edgeSelection === 'alignment') {
      chosenEdge = support.edges.reduce((a, b) => (b.alignment > a.alignment ? b : a));
      this.cycleEdgeKey = chosenEdge.edge.slice().sort((a, b) => a-b).join('-');
    }
    if (!chosenEdge) {
      let bestScore = -Infinity;
      for (const edge of support.edges) {
        if (edge.alignment < this.opts.minAlignment) continue;
        const score = this.achievable(gradientFor(edge), actuators)-marginFor(edge);
        if (score > bestScore) { bestScore = score; chosenEdge = edge; }
      }
      chosenEdge = chosenEdge || support.edges.reduce((a, b) => (b.alignment > a.alignment ? b : a));
      this.cycleEdgeKey = chosenEdge.edge.slice().sort((a, b) => a-b).join('-');
    }
    const geometry = { ...support, ...chosenEdge };
    const g = gradientFor(chosenEdge);
    const outer = actuators.map((act, k) => ({ act, g: g[k] })).filter(item => item.act.kind === 'outer');
    const coreActs = actuators.map((act, k) => ({ act, g: g[k] })).filter(item => item.act.kind === 'core');
    const gMaxOuter = Math.max(1e-12, ...outer.map(item => Math.abs(item.g)));
    const gMaxCore = Math.max(1e-12, ...coreActs.map(item => Math.abs(item.g)));
    const cableTargets = new Array(sim.rover.outerStrings.length).fill(0);
    const coreTargets = new Array(sim.rover.nOuter).fill(0);
    // Sparse selection: the strongest |g| cables only
    const chosen = outer.slice().sort((a, b) => Math.abs(b.g)-Math.abs(a.g)).slice(0, this.opts.activeCables);
    const umax = cfg.actuationDeltaL;
    for (const item of chosen) {
      cableTargets[item.act.index] = umax*Math.max(-this.opts.relaxLimit, Math.min(1, (this.opts.gainKp ?? 1)*item.g/gMaxOuter));
    }
    const coreMax = cfg.coreActuationDeltaL*this.opts.coreStrokeRatio;
    for (const item of coreActs) coreTargets[item.act.index] = coreMax*Math.max(-1, Math.min(1, item.g/gMaxCore));

    const margin = marginFor(chosenEdge);
    // MPC teacher (guided policy search C-step): receding-horizon QP on the
    // same sensitivity model, optionally pulled toward the learner's action.
    if (this.opts.teacher) {
      const ctx = this.policyContext(direction, geometry, com, margin);
      const pi = this.opts.policy ? this.opts.policy.act(ctx) : null;
      const gNorm = [...outer.map(item => item.g/gMaxOuter), ...coreActs.map(item => item.g/gMaxCore)];
      const bounds = this.opts.teacher.bounds(sim, this.opts.relaxLimit, this.opts.coreStrokeRatio);
      const u0 = [...sim.currentActuationOffsets.map(v => v/bounds.umax),
        ...sim.coreRestLengths.map((_, k) => (sim.currentCoreActuationOffsets?.[k] || 0)/Math.max(bounds.cmax, 1e-9))];
      const args = { g: gNorm, u0, lo: bounds.lo, rate: bounds.rate };
      // Executed action: teacher pulled toward the learner; label: unregularized teacher.
      const u = this.opts.teacher.solve({ ...args, piTarget: pi?.normalized || null });
      const executed = this.opts.executePolicy && pi ? pi.normalized : u;
      for (let k = 0; k < 24; k++) cableTargets[k] = executed[k]*bounds.umax;
      for (let k = 0; k < 12; k++) coreTargets[k] = executed[24+k]*bounds.cmax;
      if (this.opts.onSample) {
        const label = pi && this.opts.teacher.regularized ? this.opts.teacher.solve({ ...args, piTarget: null }) : u;
        this.opts.onSample(ctx, label, pi);
      }
    }
    let predictedShift = 0;
    cableTargets.forEach((u, k) => { predictedShift += g[k]*(u-sim.currentActuationOffsets[k]); });
    coreTargets.forEach((u, k) => { predictedShift += g[24+k]*(u-sim.currentCoreActuationOffsets[k]); });
    this.lastPlan = { geometry, margin, predictedShift, gMaxOuter, gMaxCore,
      achievable: this.achievable(g, actuators),
      activeCables: chosen.map(item => item.act.index) };
    return { geometry, cableTargets, coreTargets, margin };
  }

  policyContext(direction, geometry, com, margin) {
    return { sim: this.sim, geometry, direction, com, margin, state: this.state,
      elapsed: this.sim.t-this.enteredAt, relaxLimit: this.opts.relaxLimit, coreStrokeRatio: this.opts.coreStrokeRatio };
  }

  /** Command from the learned policy; edge locked per cycle by alignment. */
  planWithPolicy(direction, support) {
    const sim = this.sim;
    let chosenEdge = this.cycleEdgeKey
      ? support.edges.find(edge => edge.edge.slice().sort((a, b) => a-b).join('-') === this.cycleEdgeKey) || null
      : null;
    const keyOf = edge => edge.edge.slice().sort((a, b) => a-b).join('-');
    const com = this.centreOfMass();
    if (!chosenEdge) {
      // Skip edges that already failed (aborted) from this support face.
      const failed = this.failedEdges?.get(support.signature) || new Set();
      let candidates = support.edges.filter(edge => !failed.has(keyOf(edge)));
      if (!candidates.length) candidates = support.edges;
      const forward = candidates.filter(edge => edge.alignment >= this.opts.minAlignment);
      const pool = forward.length ? forward : candidates;
      if (this.opts.policyEdgeSelection === 'feasibility') {
        // One KKT solve per tip cycle (not per tick): pick the edge whose
        // achievable COM shift exceeds its margin the most, as the teacher does.
        const { actuators, S } = this.sensitivity(support.face);
        let best = -Infinity;
        for (const edge of pool) {
          const g = S.map(col => col[0]*edge.normal[0]+col[1]*edge.normal[1]);
          const margin = -((com[0]-edge.mid[0])*edge.normal[0]+(com[1]-edge.mid[1])*edge.normal[1]);
          const score = this.achievable(g, actuators)-margin;
          if (score > best) { best = score; chosenEdge = edge; }
        }
      }
      chosenEdge = chosenEdge || pool.reduce((a, b) => (b.alignment > a.alignment ? b : a));
      this.cycleEdgeKey = keyOf(chosenEdge);
    }
    const margin = -((com[0]-chosenEdge.mid[0])*chosenEdge.normal[0]+(com[1]-chosenEdge.mid[1])*chosenEdge.normal[1]);
    const geometry = { ...support, ...chosenEdge };
    const command = this.opts.policy.act(this.policyContext(direction, geometry, com, margin));
    this.lastPlan = { geometry, margin, predictedShift: 0, achievable: null,
      activeCables: command.cableTargets.map((u, k) => (Math.abs(u) > 1e-4 ? k : -1)).filter(k => k >= 0) };
    return { geometry, cableTargets: command.cableTargets, coreTargets: command.coreTargets, margin };
  }

  update(centroid, velocity, omega, direction) {
    const sim = this.sim;
    const elapsed = sim.t-this.enteredAt;
    const maxNodeSpeed = sim.v.reduce((m, vel) => Math.max(m, Math.hypot(vel[0], vel[1], vel[2])), 0);
    const zeros = () => ({ cableTargets: new Array(sim.rover.outerStrings.length).fill(0),
      coreTargets: new Array(sim.rover.nOuter).fill(0) });
    let command = zeros();
    let geometry = this.lastPlan?.geometry || null;

    if (this.state === 'SGT_SETTLE') {
      if ((elapsed >= this.opts.settleMinTime && maxNodeSpeed < this.opts.settleSpeed) ||
        elapsed >= this.opts.settleMaxTime) {
        geometry = this.supportGeometry(direction);
        this.cycleSignature = geometry.signature;
        this.cycleEdgeKey = null;
        this.enter('SGT_SHIFT');
      }
    }
    if (this.state === 'SGT_SHIFT' || this.state === 'SGT_TIP') {
      const planned = this.plan(direction);
      geometry = planned.geometry;
      command = planned;
      const tipped = geometry.signature !== this.cycleSignature;
      if (tipped) {
        this.tipCount++;
        this.failedEdges = null;
        this.enter('SGT_RELEASE');
        command = zeros();
      } else if (this.state === 'SGT_SHIFT' && planned.margin < 0) {
        this.enter('SGT_TIP');
      } else if ((this.state === 'SGT_SHIFT' && elapsed >= this.opts.shiftTimeout) ||
        (this.state === 'SGT_TIP' && elapsed >= this.opts.tipHoldTimeout)) {
        this.abortCount++;
        if (this.opts.policy && !this.opts.teacher && this.cycleEdgeKey) {
          if (!this.failedEdges) this.failedEdges = new Map();
          if (!this.failedEdges.has(this.cycleSignature)) this.failedEdges.set(this.cycleSignature, new Set());
          this.failedEdges.get(this.cycleSignature).add(this.cycleEdgeKey);
        }
        this.enter('SGT_RELEASE');
        command = zeros();
      }
    } else if (this.state === 'SGT_RELEASE') {
      if (elapsed >= this.opts.releaseTime) this.enter('SGT_SETTLE');
    }
    const active = this.state === 'SGT_SHIFT' || this.state === 'SGT_TIP';
    if (active) command = this.smoothCommand(command);
    else this.previousCommand = null;
    if (active && this.opts.residual) command = this.applyResidual(command, direction, geometry);
    if (this.opts.shield) command = this.applyShield(command, direction);
    return { ...command, geometry, state: this.state };
  }

  /** First-order command smoothing (gainKd): u <- u_prev + (u - u_prev)/(1 + Kd). Kd = 0 is the identity. */
  smoothCommand(command) {
    const kd = this.opts.gainKd ?? 0;
    const previous = this.previousCommand;
    if (!(kd > 0) || !previous) {
      this.previousCommand = { cableTargets: command.cableTargets.slice(), coreTargets: command.coreTargets.slice() };
      return command;
    }
    const blend = (now, before) => now.map((u, k) => before[k]+(u-before[k])/(1+kd));
    const out = { ...command, cableTargets: blend(command.cableTargets, previous.cableTargets),
      coreTargets: blend(command.coreTargets, previous.coreTargets) };
    this.previousCommand = { cableTargets: out.cableTargets.slice(), coreTargets: out.coreTargets.slice() };
    return out;
  }

  /**
   * Residual policy (js/residualMppi.js): u = clamp(u_nominal + du), with
   * |du| <= maxResidualStrokeM per cable and the total stroke clamped to
   * maxTotalStrokeM. The tension shield, if enabled, still filters the result.
   */
  applyResidual(command, direction, geometry) {
    const sim = this.sim, cfg = sim.cfg;
    const hook = this.opts.residual;
    const act = typeof hook === 'function' ? hook : hook.act;
    const cap = hook.maxResidualStrokeM ?? 0.030, total = hook.maxTotalStrokeM ?? 0.120;
    const umax = cfg.actuationDeltaL, lo = -this.opts.relaxLimit*umax, hi = Math.min(umax, total);
    const nominal = command.cableTargets;
    const du = act({ sim, controller: this, geometry, direction, nominal, margin: this.lastPlan?.margin ?? 0,
      state: this.state, elapsed: sim.t-this.enteredAt });
    this.lastResidual = du ? Array.from(du) : null;
    const cableTargets = nominal.map((u, k) => {
      const d = du ? Math.max(-cap, Math.min(cap, du[k])) : 0;
      return Math.max(lo, Math.min(hi, Math.min(u, hi)+d));
    });
    return { ...command, cableTargets };
  }

  /** Tension-certified safety filter (tensionShield.js) on the final command, in every state. */
  applyShield(command, direction) {
    const sim = this.sim, cfg = sim.cfg;
    const face = this.supportGeometry(direction).face;
    const sens = this.sensitivity(face);
    // COM-shift gradient along the locked tipping edge (only while shifting/tipping).
    const normal = this.lastPlan?.geometry?.normal;
    if (normal && (this.state === 'SGT_SHIFT' || this.state === 'SGT_TIP')) {
      sens.shiftGradient = sens.S.map(col => col[0]*normal[0]+col[1]*normal[1]);
    }
    const umax = cfg.actuationDeltaL, cmax = cfg.coreActuationDeltaL*this.opts.coreStrokeRatio;
    // Residual-MPPI: the total commanded outer stroke stays within maxTotalStrokeM after filtering, too.
    const outerHi = this.opts.residual ? Math.min(umax, this.opts.residual.maxTotalStrokeM ?? 0.120) : umax;
    const bounds = {
      lo: [...new Array(24).fill(-this.opts.relaxLimit*umax), ...new Array(12).fill(-cmax)],
      hi: [...new Array(24).fill(outerHi), ...new Array(12).fill(cmax)]
    };
    const result = shieldCommand(sim, sens, command, bounds, typeof this.opts.shield === 'object' ? this.opts.shield : {});
    this.shieldInfo = result.info;
    return { ...command, cableTargets: result.cableTargets, coreTargets: result.coreTargets };
  }
}

/** Dense LU with partial pivoting (row-major, in place copy). */
export function luFactor(Ain, n) {
  const A = Float64Array.from(Ain);
  const piv = new Int32Array(n);
  for (let i = 0; i < n; i++) piv[i] = i;
  for (let col = 0; col < n; col++) {
    let p = col, best = Math.abs(A[col*n+col]);
    for (let r = col+1; r < n; r++) {
      const v = Math.abs(A[r*n+col]);
      if (v > best) { best = v; p = r; }
    }
    if (best < 1e-300) continue;
    if (p !== col) {
      for (let c = 0; c < n; c++) { const t = A[col*n+c]; A[col*n+c] = A[p*n+c]; A[p*n+c] = t; }
      const t = piv[col]; piv[col] = piv[p]; piv[p] = t;
    }
    const d = A[col*n+col];
    for (let r = col+1; r < n; r++) {
      const f = A[r*n+col]/d;
      A[r*n+col] = f;
      if (f === 0) continue;
      for (let c = col+1; c < n; c++) A[r*n+c] -= f*A[col*n+c];
    }
  }
  return { A, piv, n };
}

export function luSolve({ A, piv, n }, b) {
  const x = new Float64Array(n);
  for (let i = 0; i < n; i++) x[i] = b[piv[i]];
  for (let i = 0; i < n; i++) {
    let s = x[i];
    for (let c = 0; c < i; c++) s -= A[i*n+c]*x[c];
    x[i] = s;
  }
  for (let i = n-1; i >= 0; i--) {
    let s = x[i];
    for (let c = i+1; c < n; c++) s -= A[i*n+c]*x[c];
    const d = A[i*n+i];
    x[i] = Math.abs(d) > 1e-300 ? s/d : 0;
  }
  return x;
}
