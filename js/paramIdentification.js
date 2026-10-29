/**
 * PARAMIDENTIFICATION.JS - JavaScript port of "Final Codes" (final_codes/*.m)
 *
 *   forward_tensegrity.m   static FEM of a pin-jointed strut/spring tensegrity:
 *                          form finding (element lengths ramped from the drawn
 *                          geometry to the rest lengths in 20 increments), then
 *                          load application (20 increments). Every increment is
 *                          a Newton iteration on the tangent stiffness
 *                            K_e = k w w' + (F/L)(I - w w'),   F = k (L - L_rest).
 *   MainFile.m             sequential Bayesian estimation of the parameters with
 *                          a particle filter: Gaussian jitter, retry of failed
 *                          solves, log-sum-exp weights, weighted mean/variance,
 *                          residual resampling when N_eff < N/2, jitter decay.
 *
 * The solver is generic (any nodes, elements and supports), so the same filter
 * also identifies the 3D rover (js/roverParamIdentification.js). The planar
 * model below reproduces forward_tensegrity.m element by element; the
 * validation against the original MATLAB code (run in Octave) is in
 * tools/final_codes_validate.mjs and tests/paramIdentification.test.mjs.
 *
 * Differences from the original, all optional:
 *   - stability: 'positive-definite' (default) accepts an equilibrium only if the
 *     reduced tangent stiffness is positive definite. The original
 *     eigs(K,1,'smallestabs') > 0 tests the sign of the eigenvalue closest to
 *     zero, which misses a negative eigenvalue of larger magnitude;
 *     stability: 'original' reproduces it exactly.
 *   - a failed solve returns ok = false with NaN positions instead of an
 *     unassigned output (which stops MainFile.m with an error).
 *   - weightUpdate: 'original' (default) recomputes the weights from the current
 *     likelihood only, as MainFile.m does; 'sequential' multiplies them with the
 *     previous weights (standard sequential importance sampling). With the
 *     default settings N_eff < N/2 at every step, so both resample every step and
 *     give identical results.
 *   - anneal: { from, rate } widens the likelihood at the start and narrows it
 *     to sigmaAssumed (tempering): sigma_k = max(sigmaAssumed, from * rate^k).
 *     Used for the rover, whose strongly correlated parameters (outer cable
 *     stiffness vs pre-stretch, correlation -0.95) form a narrow ridge: with the
 *     sharp likelihood from the first step the cloud is captured at a wrong point
 *     of the ridge and the fixed jitter walks along it only slowly.
 */

// ------------------------------------------------------------------ random numbers
/** Seeded generator: rand() in [0,1), randn() standard normal, randi(n) in 1..n. */
export function makeRandom(seed = 46) {
  let s = seed >>> 0;
  const rand = () => {
    s = (s+0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t+Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0)/4294967296;
  };
  let spare = null;
  const randn = () => {
    if (spare !== null) { const v = spare; spare = null; return v; }
    let u, v, q;
    do { u = 2*rand()-1; v = 2*rand()-1; q = u*u+v*v; } while (q === 0 || q >= 1);
    const f = Math.sqrt(-2*Math.log(q)/q);
    spare = v*f;
    return u*f;
  };
  const randi = n => 1+Math.floor(rand()*n);
  return { rand, randn, randi };
}

// ------------------------------------------------------------------ dense linear algebra
/** Solve A x = b in place (Gaussian elimination, partial pivoting). Returns false if singular. */
function solveInPlace(A, b, n) {
  for (let c = 0; c < n; c++) {
    let p = c, best = Math.abs(A[c*n+c]);
    for (let r = c+1; r < n; r++) { const v = Math.abs(A[r*n+c]); if (v > best) { best = v; p = r; } }
    if (!(best > 0) || !Number.isFinite(best)) return false;
    if (p !== c) {
      for (let k = c; k < n; k++) { const t = A[c*n+k]; A[c*n+k] = A[p*n+k]; A[p*n+k] = t; }
      const t = b[c]; b[c] = b[p]; b[p] = t;
    }
    const inv = 1/A[c*n+c];
    for (let r = c+1; r < n; r++) {
      const f = A[r*n+c]*inv;
      if (f === 0) continue;
      for (let k = c+1; k < n; k++) A[r*n+k] -= f*A[c*n+k];
      b[r] -= f*b[c];
    }
  }
  for (let r = n-1; r >= 0; r--) {
    let s = b[r];
    for (let k = r+1; k < n; k++) s -= A[r*n+k]*b[k];
    b[r] = s/A[r*n+r];
  }
  return true;
}

/** True if the symmetric part of A (n x n) is positive definite (Cholesky). */
function isPositiveDefinite(A, n) {
  const L = new Float64Array(n*n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = 0.5*(A[i*n+j]+A[j*n+i]);
      for (let k = 0; k < j; k++) s -= L[i*n+k]*L[j*n+k];
      if (i === j) {
        if (!(s > 0)) return false;
        L[i*n+i] = Math.sqrt(s);
      } else {
        L[i*n+j] = s/L[j*n+j];
      }
    }
  }
  return true;
}

/** Eigenvalues of the symmetric part of A (cyclic Jacobi). */
export function symmetricEigenvalues(A, n) {
  const M = new Float64Array(n*n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) M[i*n+j] = 0.5*(A[i*n+j]+A[j*n+i]);
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0, diag = 0;
    for (let i = 0; i < n; i++) {
      diag += M[i*n+i]**2;
      for (let j = i+1; j < n; j++) off += M[i*n+j]**2;
    }
    if (off <= 1e-30*Math.max(diag, 1e-300)) break;
    for (let p = 0; p < n; p++) {
      for (let q = p+1; q < n; q++) {
        const apq = M[p*n+q];
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (M[q*n+q]-M[p*n+p])/(2*apq);
        const t = Math.sign(theta || 1)/(Math.abs(theta)+Math.sqrt(theta*theta+1));
        const c = 1/Math.sqrt(t*t+1), s = t*c;
        for (let k = 0; k < n; k++) {
          const mkp = M[k*n+p], mkq = M[k*n+q];
          M[k*n+p] = c*mkp-s*mkq; M[k*n+q] = s*mkp+c*mkq;
        }
        for (let k = 0; k < n; k++) {
          const mpk = M[p*n+k], mqk = M[q*n+k];
          M[p*n+k] = c*mpk-s*mqk; M[q*n+k] = s*mpk+c*mqk;
        }
      }
    }
  }
  return Array.from({ length: n }, (_, i) => M[i*n+i]).sort((a, b) => a-b);
}

// ------------------------------------------------------------------ static solver
/**
 * Static equilibrium of a pin-jointed tensegrity (port of forward_tensegrity.m).
 * model: { nodes: [[x,y,z]...], elements: [{ i, j, k, L }...] (0-based node
 * indices, axial stiffness k, rest length L), fixed: [[bx,by,bz]...] (true = fixed) }.
 * load: [{ node, f: [Fx,Fy,Fz] }...] applied after form finding (or null).
 */
export function solveStatic(model, {
  load = null, increments = 20, iterations = 10, forceTol = 1e-6, dispTol = 1e-6,
  stability = 'positive-definite'
} = {}) {
  const { nodes, elements, fixed } = model;
  const n = nodes.length, dof = 3*n, ne = elements.length;
  const free = [];
  for (let i = 0; i < n; i++) for (let a = 0; a < 3; a++) if (!fixed[i][a]) free.push(3*i+a);
  const nf = free.length;
  const freeIndex = new Int32Array(dof).fill(-1);
  free.forEach((d, r) => { freeIndex[d] = r; });
  const ia = Int32Array.from(elements, e => e.i), ib = Int32Array.from(elements, e => e.j);
  const kv = Float64Array.from(elements, e => e.k), Lrest = Float64Array.from(elements, e => e.L);

  const pos = Float64Array.from(nodes.flat());
  const L0 = new Float64Array(ne), w = new Float64Array(3*ne), Fe = new Float64Array(ne);
  const K = new Float64Array(dof*dof), F = new Float64Array(dof);
  const Kff = new Float64Array(nf*nf), Kwork = new Float64Array(nf*nf), du = new Float64Array(nf);
  const fext = new Float64Array(dof), Fu = new Float64Array(dof);
  if (load) for (const { node, f } of load) for (let a = 0; a < 3; a++) fext[3*node+a] += f[a];

  const geometry = () => {
    for (let e = 0; e < ne; e++) {
      const a = 3*ia[e], b = 3*ib[e];
      const dx = pos[b]-pos[a], dy = pos[b+1]-pos[a+1], dz = pos[b+2]-pos[a+2];
      const L = Math.hypot(dx, dy, dz);
      L0[e] = L; w[3*e] = dx/L; w[3*e+1] = dy/L; w[3*e+2] = dz/L;
    }
  };
  const lengthsNow = () => { geometry(); return Float64Array.from(L0); };

  // one load increment: Newton iterations until |sum of nodal forces| and sum |du| are below tolerance
  let Fusum = Infinity, dusum = Infinity, iterationsUsed = 0;
  const increment = (Lv, loadFactor) => {
    for (let it = 0; it < iterations; it++) {
      iterationsUsed++;
      K.fill(0); F.fill(0);
      geometry();
      for (let e = 0; e < ne; e++) {
        const f = kv[e]*(L0[e]-Lv[e]); Fe[e] = f;
        const g = f/L0[e], a = 3*ia[e], b = 3*ib[e];
        for (let r = 0; r < 3; r++) {
          const wr = w[3*e+r];
          for (let c = 0; c < 3; c++) {
            const wc = w[3*e+c];
            const ke = kv[e]*wr*wc+g*((r === c ? 1 : 0)-wr*wc);
            K[(a+r)*dof+a+c] += ke; K[(a+r)*dof+b+c] -= ke;
            K[(b+r)*dof+a+c] -= ke; K[(b+r)*dof+b+c] += ke;
          }
          F[a+r] += f*wr; F[b+r] -= f*wr;
        }
      }
      if (loadFactor) for (let d = 0; d < dof; d++) F[d] += fext[d]*loadFactor;
      for (let r = 0; r < nf; r++) {
        du[r] = F[free[r]];
        for (let c = 0; c < nf; c++) Kff[r*nf+c] = K[free[r]*dof+free[c]];
      }
      Kwork.set(Kff);
      if (!solveInPlace(Kwork, du, nf)) return false;
      for (let r = 0; r < nf; r++) pos[free[r]] += du[r];
      // sum over nodes of |du_node|
      dusum = 0;
      for (let i = 0; i < n; i++) {
        let s = 0;
        for (let a = 0; a < 3; a++) { const r = freeIndex[3*i+a]; if (r >= 0) s += du[r]*du[r]; }
        dusum += Math.sqrt(s);
      }
      // Force criterion as in the original: norm(sum(Fu_vek)), the internal nodal forces summed
      // over all nodes. Every element adds +f w at one node and -f w at the other, so this sum
      // cancels to rounding error and convergence is decided by the displacement criterion.
      geometry();
      Fu.fill(0);
      for (let e = 0; e < ne; e++) {
        const f = kv[e]*(L0[e]-Lv[e]); Fe[e] = f;
        const a = 3*ia[e], b = 3*ib[e];
        for (let c = 0; c < 3; c++) { Fu[a+c] += f*w[3*e+c]; Fu[b+c] -= f*w[3*e+c]; }
      }
      const sum = [0, 0, 0];
      for (let i = 0; i < n; i++) for (let c = 0; c < 3; c++) sum[c] += Fu[3*i+c];
      Fusum = Math.hypot(...sum);
      if (!Number.isFinite(dusum)) return false;
      if (Fusum <= forceTol && dusum <= dispTol) break;
    }
    return true;
  };

  const fail = reason => ({ ok: false, converged: false, stable: false, reason, positions: nodes.map(() => [NaN, NaN, NaN]),
    elementForces: Array.from(Fe), iterations: iterationsUsed });

  // 1) form finding: element lengths ramp from the drawn geometry to the rest lengths
  const Linit = lengthsNow(), Lv = new Float64Array(ne);
  for (let j = 1; j <= increments; j++) {
    for (let e = 0; e < ne; e++) Lv[e] = Lrest[e]+(Linit[e]-Lrest[e])*(increments-j)/increments;
    if (!increment(Lv, 0)) return fail('singular stiffness in form finding');
  }
  const formFound = Array.from({ length: n }, (_, i) => [pos[3*i], pos[3*i+1], pos[3*i+2]]);
  // 2) loading: external forces ramp in 20 increments at the rest lengths
  for (let j = 1; j <= increments; j++) {
    if (!increment(Lrest, j/increments)) return fail('singular stiffness under load');
  }

  const converged = Fusum <= forceTol && dusum <= dispTol;
  let stable = true, smallestEigenvalue = null;
  if (stability === 'original') {
    const ev = symmetricEigenvalues(Kff, nf);
    smallestEigenvalue = ev.reduce((m, v) => (Math.abs(v) < Math.abs(m) ? v : m), Infinity);
    stable = smallestEigenvalue > 0;
  } else if (stability === 'positive-definite') {
    stable = isPositiveDefinite(Kff, nf);
  }
  const positions = Array.from({ length: n }, (_, i) => [pos[3*i], pos[3*i+1], pos[3*i+2]]);
  const ok = converged && stable;
  return {
    ok, converged, stable, smallestEigenvalue, iterations: iterationsUsed, formFound,
    positions: ok ? positions : nodes.map(() => [NaN, NaN, NaN]), rawPositions: positions,
    elementForces: Array.from(Fe), reason: ok ? null : (!converged ? 'not converged' : 'unstable equilibrium')
  };
}

// ------------------------------------------------------------------ the planar model of forward_tensegrity.m
/** True parameters of MainFile.m: [del_s1 del_s2 del_s3 del_f1 del_f2]. */
export const FINAL_CODES_TRUE = Object.freeze([0.8, 1.2, 0.7, 1.2, 0.8]);
export const PLANAR_PARAMS = Object.freeze([
  { key: 'del_s1', label: 'strut 1 rest length ×', symbol: 'δs1' },
  { key: 'del_s2', label: 'strut 2 rest length ×', symbol: 'δs2' },
  { key: 'del_s3', label: 'strut 3 rest length ×', symbol: 'δs3' },
  { key: 'del_f1', label: 'spring type 1 stiffness ×', symbol: 'δf1' },
  { key: 'del_f2', label: 'spring type 2 stiffness ×', symbol: 'δf2' }
]);
export const PLANAR_NODES = Object.freeze([[0, 0, 0], [1, 0, 0], [0.2, -0.4, 0], [0.3, 0.2, 0], [0.8, -0.3, 0], [0.6, 0.4, 0]]);

/** Nodes, elements (3 struts, 6 springs) and supports of forward_tensegrity.m, in mm and N. */
export function planarModel(theta) {
  const [ds1, ds2, ds3, df1, df2] = theta;
  const ks = 1e6, kf1 = df1*0.2, kf2 = df2*0.3, Lf = 0.2;
  const el = (i, j, k, L, kind) => ({ i: i-1, j: j-1, k, L, kind });
  const elements = [
    el(1, 2, ks, ds1*1, 'strut'), el(3, 4, ks, ds2*0.4, 'strut'), el(5, 6, ks, ds3*0.5, 'strut'),
    el(1, 4, kf1, Lf, 'spring1'), el(1, 3, kf1, Lf, 'spring1'), el(4, 6, kf2, Lf, 'spring2'),
    el(3, 5, kf1, Lf, 'spring1'), el(2, 6, kf1, Lf, 'spring1'), el(2, 5, kf1, Lf, 'spring1')
  ];
  const fixed = PLANAR_NODES.map(() => [false, false, true]);   // planar: all z fixed
  fixed[0] = [true, true, true];                                // node 1 fixed
  fixed[1] = [false, true, true];                               // node 2 roller (y, z fixed)
  return { nodes: PLANAR_NODES.map(p => [...p]), elements, fixed };
}

/**
 * forward_tensegrity(del_sf, F_inp): F_inp = [Fx, Fy, Fz, node] (node 1-based).
 * Returns { output: 1|0, positions: 6 x 3 (NaN when output = 0), solve }.
 */
export function forwardTensegrity(theta, Finp, options = {}) {
  const [fx, fy, fz, node] = Finp;
  const solve = solveStatic(planarModel(theta), { ...options, load: [{ node: node-1, f: [fx, fy, fz] }] });
  return { output: solve.ok ? 1 : 0, positions: solve.positions, solve };
}

/** Flatten positions like reshape(nodal_positions, 1, 18) in MATLAB (column-major: all x, all y, all z). */
export function columnMajor(positions) {
  const out = new Float64Array(positions.length*3);
  for (let a = 0; a < 3; a++) for (let i = 0; i < positions.length; i++) out[a*positions.length+i] = positions[i][a];
  return out;
}

/**
 * The identification problem of MainFile.m: exp_no load cases (random force on
 * node 3..6), synthetic measurements of all 18 nodal coordinates at the true
 * parameters, uniform prior on [0.7, 1.4].
 */
export function planarProblem({ steps = 100, trueTheta = FINAL_CODES_TRUE, seed = 46, sigmaObs = 0, sigmaAssumed = 0.001,
  lower = 0.7, upper = 1.4, stability = 'positive-definite' } = {}) {
  const rng = makeRandom(seed);
  const loads = [];
  for (let k = 0; k < steps; k++) loads.push([-0.01+0.02*rng.rand(), -0.01+0.02*rng.rand(), 0, 2+rng.randi(4)]);
  const forward = (theta, load) => {
    const r = forwardTensegrity(theta, load, { stability });
    return { ok: r.output === 1, y: columnMajor(r.positions), solve: r.solve };
  };
  const observations = loads.map(load => {
    const y = forward(trueTheta, load).y;
    return y.map(v => v+sigmaObs*rng.randn());
  });
  return {
    id: 'planar', title: 'Planar tensegrity (Final Codes)', units: 'mm',
    params: PLANAR_PARAMS, trueTheta: [...trueTheta], lower: trueTheta.map(() => lower), upper: trueTheta.map(() => upper),
    loads, observations, sigmaAssumed, forward, rng, filterDefaults: {}
  };
}

// ------------------------------------------------------------------ particle filter (MainFile.m)
/** Residual resampling: deterministic floor(N w) copies, the rest drawn from the residual weights. */
export function residualResample(weights, rng) {
  const np = weights.length;
  const offsprings = weights.map(w => Math.floor(np*w));
  const nnp = np-offsprings.reduce((s, v) => s+v, 0);
  if (nnp > 0) {
    const residual = weights.map((w, i) => w-offsprings[i]/np);
    const sum = residual.reduce((s, v) => s+v, 0);
    if (sum > 0) {
      const cdf = [];
      let acc = 0;
      for (const r of residual) { acc += r/sum; cdf.push(acc); }
      for (let k = 0; k < nnp; k++) {
        const u = rng.rand();
        let i = cdf.findIndex(c => c >= u);
        if (i < 0) i = np-1;
        offsprings[i]++;
      }
    }
  }
  return offsprings;
}

/** Expand offspring counts into particle indices (f_resample_indices). */
export function resampleIndices(offsprings) {
  const out = [];
  offsprings.forEach((count, i) => { for (let c = 0; c < count; c++) out.push(i); });
  return out;
}

/**
 * Particle filter of MainFile.m as a generator: yields one record per forcing
 * step and returns the summary. `problem` = { trueTheta, lower[], upper[],
 * loads[], observations[], sigmaAssumed, forward(theta, load) -> { ok, y }, rng }.
 */
export function* particleFilter(problem, {
  particles = 100, steps = problem.loads.length, sigmaJitter = 0.01, sigmaJitterMin = 0.005, degradeRate = 0.995,
  maxAttempts = 100, weightUpdate = 'original', anneal = null, rng = problem.rng || makeRandom(46)
} = {}) {
  const P = problem.trueTheta.length, N = particles;
  const { lower, upper } = problem;
  const clamp = (v, p) => Math.max(Math.min(v, upper[p]), lower[p]);
  const theta = Array.from({ length: N }, () => new Float64Array(P));
  for (let p = 0; p < P; p++) for (let i = 0; i < N; i++) theta[i][p] = lower[p]+(upper[p]-lower[p])*rng.rand();
  let weights = new Float64Array(N).fill(1/N);
  let jitter = sigmaJitter, solves = 0, failedTotal = 0;
  const mean = [], variance = [], neffHist = [], started = Date.now();
  // anneal = { from, rate }: likelihood width sigma_k = max(sigmaAssumed, from * rate^k), broad at
  // first so the cloud is not captured by the first data set, sharpening to sigmaAssumed.
  const sigmaAt = k => (anneal ? Math.max(problem.sigmaAssumed, anneal.from*anneal.rate**k) : problem.sigmaAssumed);

  for (let k = 0; k < steps; k++) {
    const load = problem.loads[k], Yk = problem.observations[k], R = sigmaAt(k)**2;
    // A. jitter (not before the first step)
    if (k > 0) for (let p = 0; p < P; p++) for (let i = 0; i < N; i++) theta[i][p] = clamp(theta[i][p]+jitter*rng.randn(), p);
    // B. likelihood, retrying a failed solve with a perturbed copy
    const logw = new Float64Array(N);
    let failed = 0;
    for (let i = 0; i < N; i++) {
      let trial = theta[i], result = null, attempt = 0;
      while ((!result || !result.ok) && attempt < maxAttempts) {
        attempt++;
        result = problem.forward(Array.from(trial), load); solves++;
        if (!result.ok) {
          trial = new Float64Array(P);
          for (let p = 0; p < P; p++) trial[p] = clamp(theta[i][p]+jitter*rng.randn(), p);
        }
      }
      theta[i] = trial;
      if (!result.ok) { logw[i] = -1e10; failed++; continue; }
      let ss = 0;
      for (let d = 0; d < Yk.length; d++) { const r = Yk[d]-result.y[d]; ss += r*r; }
      logw[i] = -0.5*ss/R;
    }
    failedTotal += failed;
    // C. log-sum-exp normalisation
    if (weightUpdate === 'sequential') for (let i = 0; i < N; i++) logw[i] += Math.log(Math.max(weights[i], 1e-300));
    const maxLog = Math.max(...logw);
    const next = Float64Array.from(logw, l => Math.exp(l-maxLog));
    const sum = next.reduce((s, v) => s+v, 0);
    if (!(sum >= 1e-18)) next.fill(1/N); else for (let i = 0; i < N; i++) next[i] /= sum;
    weights = next;
    // D. weighted mean and variance
    const mu = new Array(P).fill(0), va = new Array(P).fill(0);
    for (let i = 0; i < N; i++) for (let p = 0; p < P; p++) mu[p] += weights[i]*theta[i][p];
    for (let i = 0; i < N; i++) for (let p = 0; p < P; p++) va[p] += weights[i]*(theta[i][p]-mu[p])**2;
    if (mu.some(v => !Number.isFinite(v))) {
      const prev = mean.length ? [mean.at(-1), variance.at(-1)] : null;
      for (let p = 0; p < P; p++) {
        if (prev) { mu[p] = prev[0][p]; va[p] = prev[1][p]; continue; }
        const col = theta.map(t => t[p]), m = col.reduce((s, v) => s+v, 0)/N;
        mu[p] = m; va[p] = col.reduce((s, v) => s+(v-m)**2, 0)/(N-1);
      }
    }
    mean.push(mu); variance.push(va);
    // E. effective sample size and residual resampling
    const neff = 1/weights.reduce((s, w) => s+w*w, 0);
    neffHist.push(neff);
    let resampled = false;
    if (neff < 0.5*N) {
      const idx = resampleIndices(residualResample(Array.from(weights), rng));
      const copy = theta.map(t => Float64Array.from(t));
      for (let i = 0; i < N; i++) theta[i] = Float64Array.from(copy[idx[i]]);
      weights = new Float64Array(N).fill(1/N);
      resampled = true;
    }
    jitter = Math.max(sigmaJitterMin, jitter*degradeRate);
    yield { k: k+1, steps, mean: mu, std: va.map(Math.sqrt), neff, resampled, failed, solves, sigma: Math.sqrt(R), elapsedMs: Date.now()-started };
  }

  const final = mean.at(-1), std = variance.at(-1).map(Math.sqrt);
  return {
    problem: problem.id, title: problem.title, steps, particles: N, weightUpdate, anneal,
    trueTheta: problem.trueTheta, final, std,
    errorPct: final.map((v, p) => 100*(v-problem.trueTheta[p])/problem.trueTheta[p]),
    within2Sigma: final.map((v, p) => Math.abs(v-problem.trueTheta[p]) <= 2*std[p]),
    mean, stdHistory: variance.map(v => v.map(Math.sqrt)), neff: neffHist, forwardSolves: solves, failedSolves: failedTotal,
    elapsedMs: Date.now()-started
  };
}

/** Run the particle filter to completion (onStep is called after every forcing step). */
export function runParticleFilter(problem, options = {}) {
  const it = particleFilter(problem, options);
  for (;;) {
    const r = it.next();
    if (r.done) return r.value;
    options.onStep?.(r.value);
  }
}
