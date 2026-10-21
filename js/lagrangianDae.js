/**
 * LAGRANGIANDAE.JS - Non-minimal Cartesian Lagrangian dynamics for the shell
 *
 * Port of the constrained formulation in "Tensegrity dynamics with non-minimal
 * coordinates" (repository: tensegrityDynamicspreprint.pdf, Sections 2.1-2.4)
 * to the 12-node / 6-bar shell used by simEngine.js.
 *
 *   q = vec(N) in R^{3n},  M qdd = F + Rq' lambda,  R(q) = 0
 *
 *   M        (Eq. 22) consistent rigid-bar inertia: for bar k with mass m_k,
 *            radius r_k, length l_k and I_k = m_k/12 (3 r_k^2 + l_k^2),
 *              M_k = m_k Xbar_k'Xbar_k + I_k/l_k^2 X_k'X_k
 *            plus nodal point masses. Bars are node-disjoint in this rover,
 *            so M is block-diagonal with one 2x2 (x I3) block per bar and is
 *            inverted in closed form.
 *   R(q)     (Eq. 36) R_k = |q_j - q_i|^2 - l_k^2 for every rigid bar
 *   lambda   (Eq. 34-35) -(Rq M^-1 Rq')^-1 (xi2 + Rq M^-1 F), xi2_k = 2|b_dot_k|^2
 *   Algorithm 1 (Eq. 56-57) minimum-norm correction of (q, qdot, W) so that
 *            R = 0, Rq qdot = 0 and E(q,qdot) = E0 + W after every step.
 *
 * Everything works on flat Float64Arrays of length 3n for speed; helper
 * functions convert from/to the [[x,y,z],...] layout used by simEngine.js.
 */

export class ShellDAE {
  /**
   * @param {object} p
   * @param {number}   p.nNodes
   * @param {number[][]} p.bars       [[i, j], ...] node pairs (node-disjoint)
   * @param {number[]} p.barLengths   rest lengths l_k [m]
   * @param {number}   p.barMass      bar mass m_k [kg] (scalar or array)
   * @param {number}   p.barRadius    bar radius r_k [m] (scalar or array)
   * @param {number}   p.nodeMass     point mass at every node [kg] (scalar or array)
   * @param {number[]} p.gravity      [gx, gy, gz]
   */
  constructor(p) {
    this.n = p.nNodes;
    this.nq = 3*this.n;
    this.bars = p.bars.map(pair => pair.slice());
    this.m = this.bars.length;
    const arr = (value, count) => Array.isArray(value) ? value.slice() : new Array(count).fill(value);
    this.barMass = arr(p.barMass, this.m);
    this.barRadius = arr(p.barRadius ?? 0.01, this.m);
    this.nodeMass = arr(p.nodeMass ?? 0, this.n);
    this.barLengths = p.barLengths.slice();
    this.setGravity(p.gravity || [0, 0, -9.81]);

    const owner = new Array(this.n).fill(-1);
    this.bars.forEach(([i, j], k) => {
      if (owner[i] >= 0 || owner[j] >= 0) {
        throw new Error('ShellDAE: bars must be node-disjoint (closed-form block inverse)');
      }
      owner[i] = k; owner[j] = k;
    });
    this.owner = owner;
    this.buildMass();
  }

  setGravity(g) {
    this.g = g.slice();
  }

  /** Scalar 2x2 mass blocks [[A_i, B],[B, A_j]] (each times I3) and inverses. */
  buildMass() {
    this.block = this.bars.map(([i, j], k) => {
      const mb = this.barMass[k];
      const l = this.barLengths[k];
      const r = this.barRadius[k];
      const I = mb/12*(3*r*r+l*l);
      const a = 0.25*mb+I/(l*l);
      const b = 0.25*mb-I/(l*l);
      const Ai = a+this.nodeMass[i];
      const Aj = a+this.nodeMass[j];
      const det = Ai*Aj-b*b;
      if (!(det > 0)) throw new Error('ShellDAE: mass block is not positive definite');
      return { i, j, Ai, Aj, B: b, invAi: Aj/det, invAj: Ai/det, invB: -b/det };
    });
    // Gravity vector G (Eq. 23): half the bar mass at each end plus point mass.
    this.nodeGravityMass = new Float64Array(this.n);
    for (let node = 0; node < this.n; node++) this.nodeGravityMass[node] = this.nodeMass[node];
    this.bars.forEach(([i, j], k) => {
      this.nodeGravityMass[i] += 0.5*this.barMass[k];
      this.nodeGravityMass[j] += 0.5*this.barMass[k];
    });
    this.totalMass = this.nodeGravityMass.reduce((sum, value) => sum+value, 0);
  }

  /** y = M x  (flat arrays). */
  mulM(x, out = new Float64Array(this.nq)) {
    for (let node = 0; node < this.n; node++) {
      if (this.owner[node] >= 0) continue;
      const m = this.nodeMass[node];
      for (let a = 0; a < 3; a++) out[3*node+a] = m*x[3*node+a];
    }
    for (const blk of this.block) {
      for (let a = 0; a < 3; a++) {
        const xi = x[3*blk.i+a], xj = x[3*blk.j+a];
        out[3*blk.i+a] = blk.Ai*xi+blk.B*xj;
        out[3*blk.j+a] = blk.B*xi+blk.Aj*xj;
      }
    }
    return out;
  }

  /** y = M^-1 x  (closed form). */
  solveM(x, out = new Float64Array(this.nq)) {
    for (let node = 0; node < this.n; node++) {
      if (this.owner[node] >= 0) continue;
      const m = this.nodeMass[node];
      for (let a = 0; a < 3; a++) out[3*node+a] = x[3*node+a]/m;
    }
    for (const blk of this.block) {
      for (let a = 0; a < 3; a++) {
        const xi = x[3*blk.i+a], xj = x[3*blk.j+a];
        out[3*blk.i+a] = blk.invAi*xi+blk.invB*xj;
        out[3*blk.j+a] = blk.invB*xi+blk.invAj*xj;
      }
    }
    return out;
  }

  /** Bar vectors b_k = q_j - q_i (flat 3m array). */
  barVectors(q) {
    const b = new Float64Array(3*this.m);
    this.bars.forEach(([i, j], k) => {
      for (let a = 0; a < 3; a++) b[3*k+a] = q[3*j+a]-q[3*i+a];
    });
    return b;
  }

  /** Constraint residuals R_k = |b_k|^2 - l_k^2 and velocity residuals Rq qdot. */
  residuals(q, v) {
    const b = this.barVectors(q);
    const bd = this.barVectors(v);
    const R = new Float64Array(this.m);
    const Rv = new Float64Array(this.m);
    for (let k = 0; k < this.m; k++) {
      let bb = 0, bbd = 0;
      for (let a = 0; a < 3; a++) {
        bb += b[3*k+a]*b[3*k+a];
        bbd += b[3*k+a]*bd[3*k+a];
      }
      R[k] = bb-this.barLengths[k]*this.barLengths[k];
      Rv[k] = 2*bbd;
    }
    return { R, Rv, b, bd };
  }

  /** Maximum absolute bar-length error | |b_k| - l_k | [m]. */
  maxLengthError(q) {
    const b = this.barVectors(q);
    let worst = 0;
    for (let k = 0; k < this.m; k++) {
      const len = Math.hypot(b[3*k], b[3*k+1], b[3*k+2]);
      worst = Math.max(worst, Math.abs(len-this.barLengths[k]));
    }
    return worst;
  }

  /** Rq' y : scatter m multipliers into a 3n force vector. */
  scatterRqT(b, y, out = new Float64Array(this.nq)) {
    out.fill(0);
    this.bars.forEach(([i, j], k) => {
      for (let a = 0; a < 3; a++) {
        const f = 2*y[k]*b[3*k+a];
        out[3*i+a] -= f;
        out[3*j+a] += f;
      }
    });
    return out;
  }

  /** Rq x : gather (m) from a 3n vector. */
  gatherRq(b, x, out = new Float64Array(this.m)) {
    this.bars.forEach(([i, j], k) => {
      let s = 0;
      for (let a = 0; a < 3; a++) s += 2*b[3*k+a]*(x[3*j+a]-x[3*i+a]);
      out[k] = s;
    });
    return out;
  }

  /**
   * Constrained accelerations (Eq. 34-35).
   * @param q,v  flat positions / velocities
   * @param F    flat applied force (everything except the bar constraint)
   * @returns {{qdd, lambda, constraintForce, barForces}}
   *   barForces[k] = axial bar force 2 lambda_k l_k [N] (+ = compression)
   */
  accelerations(q, v, F) {
    const { b, bd } = this.residuals(q, v);
    const y = this.solveM(F);
    const m = this.m;
    // S = Rq M^-1 Rq'  (m x m), dense build: column k = Rq M^-1 (Rq' e_k)
    const S = new Float64Array(m*m);
    const e = new Float64Array(m);
    const col = new Float64Array(this.nq);
    const tmp = new Float64Array(this.nq);
    const RqCol = new Float64Array(m);
    for (let k = 0; k < m; k++) {
      e.fill(0); e[k] = 1;
      this.scatterRqT(b, e, col);
      this.solveM(col, tmp);
      this.gatherRq(b, tmp, RqCol);
      for (let r = 0; r < m; r++) S[r*m+k] = RqCol[r];
    }
    const rhs = this.gatherRq(b, y);
    for (let k = 0; k < m; k++) {
      let bdbd = 0;
      for (let a = 0; a < 3; a++) bdbd += bd[3*k+a]*bd[3*k+a];
      rhs[k] = -(rhs[k]+2*bdbd);
    }
    const lambda = solveDense(S, rhs, m);
    const constraintForce = this.scatterRqT(b, lambda);
    const total = new Float64Array(this.nq);
    for (let idx = 0; idx < this.nq; idx++) total[idx] = F[idx]+constraintForce[idx];
    const qdd = this.solveM(total);
    const barForces = new Float64Array(m);
    for (let k = 0; k < m; k++) {
      const len = Math.hypot(b[3*k], b[3*k+1], b[3*k+2]);
      barForces[k] = 2*lambda[k]*len;
    }
    return { qdd, lambda, constraintForce, barForces };
  }

  /** Shell energy with consistent inertia: T = 1/2 v'Mv, Vg = -G'q. */
  energy(q, v) {
    const Mv = this.mulM(v);
    let T = 0, Vg = 0;
    for (let idx = 0; idx < this.nq; idx++) T += 0.5*v[idx]*Mv[idx];
    for (let node = 0; node < this.n; node++) {
      for (let a = 0; a < 3; a++) Vg -= this.nodeGravityMass[node]*this.g[a]*q[3*node+a];
    }
    return { E: T+Vg, T, Vg, Mv };
  }

  /**
   * Algorithm 1: iterative minimum-norm correction (Eq. 56-57).
   * @param q,v       flat state, corrected in place
   * @param target    E0 + W_f (energy the state must carry), or null to skip
   *                  the energy row (constraint-only correction)
   * @param opts      { gamma = 1e-10, maxIter = 10, energyGamma,
   *                    potential: q => ({V, grad}) extra potential (e.g. contact)
   *                    that belongs to the energy row }
   * @returns {{iters, dW, R, Rv, Eres}}  dW = ledger correction (Eq. 50)
   */
  correct(q, v, target = null, opts = {}) {
    const gamma = opts.gamma ?? 1e-10;
    const egamma = opts.energyGamma ?? gamma;
    const maxIter = opts.maxIter ?? 10;
    const useE = target !== null && Number.isFinite(target);
    const m = this.m, nq = this.nq;
    let dWTotal = 0;
    let iters = 0;
    let state;
    for (;;) {
      state = this.residuals(q, v);
      let Rn = 0, Vn = 0;
      for (let k = 0; k < m; k++) { Rn += state.R[k]*state.R[k]; Vn += state.Rv[k]*state.Rv[k]; }
      Rn = Math.sqrt(Rn); Vn = Math.sqrt(Vn);
      let Eres = 0, en = null, ext = null;
      if (useE) {
        en = this.energy(q, v);
        ext = opts.potential ? opts.potential(q) : null;
        Eres = en.E+(ext ? ext.V : 0)-(target+dWTotal);
      }
      state.Rn = Rn; state.Vn = Vn; state.Eres = Eres;
      if ((Rn <= gamma && Vn <= gamma && (!useE || Math.abs(Eres) <= egamma)) || iters >= maxIter) break;

      // Build A_c (rows: m position, m velocity, [1 energy]) x (2nq [+1]).
      const rows = useE ? 2*m+1 : 2*m;
      const cols = useE ? 2*nq+1 : 2*nq;
      const Ac = new Float64Array(rows*cols);
      const bc = new Float64Array(rows);
      const { b, bd } = state;
      this.bars.forEach(([i, j], k) => {
        for (let a = 0; a < 3; a++) {
          // position row: Rq
          Ac[k*cols+3*i+a] = -2*b[3*k+a];
          Ac[k*cols+3*j+a] = 2*b[3*k+a];
          // velocity row: [Q, Rq], Q = d(Rq qdot)/dq
          Ac[(m+k)*cols+3*i+a] = -2*bd[3*k+a];
          Ac[(m+k)*cols+3*j+a] = 2*bd[3*k+a];
          Ac[(m+k)*cols+nq+3*i+a] = -2*b[3*k+a];
          Ac[(m+k)*cols+nq+3*j+a] = 2*b[3*k+a];
        }
        bc[k] = -state.R[k];
        bc[m+k] = -state.Rv[k];
      });
      if (useE) {
        const r = 2*m;
        for (let node = 0; node < this.n; node++) {
          for (let a = 0; a < 3; a++) {
            Ac[r*cols+3*node+a] = -this.nodeGravityMass[node]*this.g[a]   // dVg/dq
              +(ext && ext.grad ? ext.grad[3*node+a] : 0);                 // dV_ext/dq
            Ac[r*cols+nq+3*node+a] = en.Mv[3*node+a];                   // dT/dqdot
          }
        }
        Ac[r*cols+2*nq] = -1;
        bc[r] = -Eres;
      }
      // d = Ac' (Ac Ac')^-1 bc
      const AAt = new Float64Array(rows*rows);
      for (let r1 = 0; r1 < rows; r1++) {
        for (let r2 = r1; r2 < rows; r2++) {
          let s = 0;
          const o1 = r1*cols, o2 = r2*cols;
          for (let c = 0; c < cols; c++) s += Ac[o1+c]*Ac[o2+c];
          AAt[r1*rows+r2] = s; AAt[r2*rows+r1] = s;
        }
      }
      const z = solveDense(AAt, bc, rows);
      for (let c = 0; c < cols; c++) {
        let s = 0;
        for (let r1 = 0; r1 < rows; r1++) s += Ac[r1*cols+c]*z[r1];
        if (c < nq) q[c] += s;
        else if (c < 2*nq) v[c-nq] += s;
        else dWTotal += s;
      }
      iters++;
    }
    return { iters, dW: dWTotal, R: state.Rn, Rv: state.Vn, Eres: state.Eres };
  }
}

/** Gaussian elimination with partial pivoting and a tiny Tikhonov fallback. */
export function solveDense(Ain, bin, n) {
  const A = Float64Array.from(Ain);
  const b = Float64Array.from(bin);
  let scale = 0;
  for (let i = 0; i < n; i++) scale = Math.max(scale, Math.abs(A[i*n+i]));
  for (let col = 0; col < n; col++) {
    let piv = col, best = Math.abs(A[col*n+col]);
    for (let r = col+1; r < n; r++) {
      const val = Math.abs(A[r*n+col]);
      if (val > best) { best = val; piv = r; }
    }
    if (best < 1e-14*Math.max(scale, 1)) {
      A[col*n+col] += 1e-12*Math.max(scale, 1); // redundant row -> least-squares-like fallback
      piv = col;
    }
    if (piv !== col) {
      for (let c = 0; c < n; c++) { const t = A[col*n+c]; A[col*n+c] = A[piv*n+c]; A[piv*n+c] = t; }
      const t = b[col]; b[col] = b[piv]; b[piv] = t;
    }
    const d = A[col*n+col];
    for (let r = col+1; r < n; r++) {
      const f = A[r*n+col]/d;
      if (f === 0) continue;
      for (let c = col; c < n; c++) A[r*n+c] -= f*A[col*n+c];
      b[r] -= f*b[col];
    }
  }
  const x = new Float64Array(n);
  for (let r = n-1; r >= 0; r--) {
    let s = b[r];
    for (let c = r+1; c < n; c++) s -= A[r*n+c]*x[c];
    x[r] = s/A[r*n+r];
  }
  return x;
}

export function flatten(nodes, out = new Float64Array(3*nodes.length)) {
  for (let i = 0; i < nodes.length; i++) {
    out[3*i] = nodes[i][0]; out[3*i+1] = nodes[i][1]; out[3*i+2] = nodes[i][2];
  }
  return out;
}

export function unflatten(flat, nodes) {
  for (let i = 0; i < nodes.length; i++) {
    nodes[i][0] = flat[3*i]; nodes[i][1] = flat[3*i+1]; nodes[i][2] = flat[3*i+2];
  }
  return nodes;
}
