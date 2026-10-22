/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/**
 * TSGFEM.JS - JavaScript port of the statics / modal core of TsgFEM
 *
 *   Ma, S., Chen, M., Skelton, R.E. (2022). Tensegrity Finite Element Method.
 *   Journal of Open Source Software 7(75), 3390. doi:10.21105/joss.03390
 *   https://github.com/Muhao-Chen/Tensegrity_Finite_Element_Method_TsgFEM
 *
 * Ported (same equations, same conventions; MATLAB source in
 * third_party/TsgFEM/Function_library):
 *   material_lib             -> MATERIAL_LIB (+ 'UHS_steel_string' added here)
 *   tenseg_equilibrium_matrix1 -> equilibriumMatrix  (A_1a, A_1ag, l, l_gp)
 *   tenseg_svd               -> svdNullspaces       (rank, self-stress V2, mechanisms U2)
 *   tenseg_prestress_design  -> prestressDesign     (q, t from designed group forces)
 *   tenseg_minimass + minimass_buckle_hollow/solid -> minimalMass
 *   tenseg_stiff_matx2       -> tangentStiffness    K_T = Σ q_i I + E_i A_i/l_i³ b_i b_iᵀ
 *   tenseg_mass_matrix       -> massMatrix          (consistent or lumped)
 *   tenseg_mode              -> modalAnalysis       eig(Ia'K_T Ia, Ia'M Ia), f in Hz
 * Conventions: C is the ne×nn connectivity matrix (row: -1 at start node,
 * +1 at end node), H = N Cᵀ, tension positive. Dense linear algebra below
 * (symmetric Jacobi eigen-solver, Cholesky) replaces MATLAB's svd/eig.
 */

// ------------------------------------------------------------------ materials (material_lib.m)

/** E [Pa], yield/strength sigma [Pa], density rho [kg/m³]. Densities are the real ones
 *  (TsgFEM's examples amplify some densities ×50 for animation; not used here). */
export const MATERIAL_LIB = Object.freeze({
  bars: {
    Steel_Q345: { E: 300e6/1456e-6, sigma: 300e6, rho: 7870, label: 'Steel Q345 (S355-class)' },
    Steel: { E: 200e9, sigma: 300e6, rho: 8000, label: 'Steel' },
    Carbon_Rod: { E: 138e9, sigma: 1.72e9, rho: 1500, label: 'Carbon fibre rod' },
    Aluminum: { E: 60e9, sigma: 110e6, rho: 2700, label: 'Aluminium' },
    UHMWPE: { E: 120e9, sigma: 2.7e9, rho: 970, label: 'UHMWPE' },
    Wood: { E: 8.1e9, sigma: 78e6, rho: 2700, label: 'Wood' }
  },
  strings: {
    Steel_string: { E: 1223.5e6/0.016099, sigma: 1223.5e6, rho: 7870, label: 'Steel string (TsgFEM)' },
    // Added for this project: ultra-high-strength wire rope, 1960 MPa grade,
    // effective rope modulus 100 GPa, strength on the metallic area.
    UHS_steel_string: { E: 100e9, sigma: 1960e6, rho: 7850, label: 'UHS steel wire rope (1960 MPa)' },
    Steel: { E: 200e9, sigma: 300e6, rho: 8000, label: 'Steel' },
    UHMWPE: { E: 120e9, sigma: 2.7e9, rho: 970, label: 'UHMWPE (Dyneema)' },
    Aluminum: { E: 60e9, sigma: 110e6, rho: 2700, label: 'Aluminium' },
    Rubber_band: { E: 2e6, sigma: 1e7, rho: 1700, label: 'Rubber band' }
  }
});

// ------------------------------------------------------------------ dense linear algebra

const zeros = (r, c) => Array.from({ length: r }, () => new Float64Array(c));
const transpose = A => { const r = A.length, c = A[0].length, T = zeros(c, r); for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) T[j][i] = A[i][j]; return T; };
const matmul = (A, B) => {
  const r = A.length, n = B.length, c = B[0].length, R = zeros(r, c);
  for (let i = 0; i < r; i++) for (let k = 0; k < n; k++) { const a = A[i][k]; if (a !== 0) for (let j = 0; j < c; j++) R[i][j] += a*B[k][j]; }
  return R;
};

/** Cyclic Jacobi eigen-decomposition of a symmetric matrix: { values (ascending), vectors (columns) }. */
export function symmetricEigen(input, { tol = 1e-13, maxSweeps = 100 } = {}) {
  const n = input.length;
  const A = input.map(row => Float64Array.from(row));
  const V = zeros(n, n);
  for (let i = 0; i < n; i++) V[i][i] = 1;
  const scale = Math.max(1e-300, ...A.map((row, i) => Math.abs(row[i])), ...A.flatMap(row => [...row].map(Math.abs)));
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p+1; q < n; q++) off += A[p][q]*A[p][q];
    if (Math.sqrt(off) <= tol*scale*n) break;
    for (let p = 0; p < n-1; p++) {
      for (let q = p+1; q < n; q++) {
        const apq = A[p][q];
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (A[q][q]-A[p][p])/(2*apq);
        const t = Math.sign(theta || 1)/(Math.abs(theta)+Math.sqrt(theta*theta+1));
        const c = 1/Math.sqrt(t*t+1), s = t*c;
        for (let k = 0; k < n; k++) {
          const akp = A[k][p], akq = A[k][q];
          A[k][p] = c*akp-s*akq; A[k][q] = s*akp+c*akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p][k], aqk = A[q][k];
          A[p][k] = c*apk-s*aqk; A[q][k] = s*apk+c*aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c*vkp-s*vkq; V[k][q] = s*vkp+c*vkq;
        }
      }
    }
  }
  const order = [...Array(n).keys()].sort((i, j) => A[i][i]-A[j][j]);
  return { values: order.map(i => A[i][i]), vectors: order.map(i => V.map(row => row[i])) };
}

function cholesky(M) {
  const n = M.length, L = zeros(n, n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = M[i][j];
      for (let k = 0; k < j; k++) s -= L[i][k]*L[j][k];
      if (i === j) {
        if (!(s > 0)) throw new Error('mass matrix is not positive definite');
        L[i][i] = Math.sqrt(s);
      } else {
        L[i][j] = s/L[j][j];
      }
    }
  }
  return L;
}

function lowerInverse(L) {
  const n = L.length, X = zeros(n, n);
  for (let j = 0; j < n; j++) {
    X[j][j] = 1/L[j][j];
    for (let i = j+1; i < n; i++) {
      let s = 0;
      for (let k = j; k < i; k++) s -= L[i][k]*X[k][j];
      X[i][j] = s/L[i][i];
    }
  }
  return X;
}

/** Least-squares solve min‖Mx − b‖ via normal equations (small systems). */
function leastSquares(M, b) {
  const Mt = transpose(M);
  const MtM = matmul(Mt, M);
  const Mtb = Mt.map(row => row.reduce((s, v, i) => s+v*b[i], 0));
  const n = MtM.length;
  const A = MtM.map((row, i) => [...row, Mtb[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col+1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    [A[col], A[piv]] = [A[piv], A[col]];
    const d = A[col][col] || 1e-300;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = A[r][col]/d;
      for (let k = col; k <= n; k++) A[r][k] -= f*A[col][k];
    }
  }
  return A.map((row, i) => row[n]/(row[i] || 1e-300));
}

// ------------------------------------------------------------------ TsgFEM functions

/** Connectivity matrix from index pairs (tenseg_ind2C). */
export function connectivity(pairs, nn) {
  return pairs.map(([i, j]) => { const row = new Float64Array(nn); row[i] = -1; row[j] = 1; return row; });
}

/**
 * tenseg_equilibrium_matrix1: A_1a (3nn_free × ne) with A_1a q = w, member lengths l,
 * and the grouped versions for group matrix Gp (ne × ng).
 */
export function equilibriumMatrix(N, pairs, { Gp = null, free = null } = {}) {
  const nn = N.length, ne = pairs.length;
  const H = pairs.map(([i, j]) => [0, 1, 2].map(a => N[j][a]-N[i][a]));
  const l = H.map(h => Math.hypot(...h));
  const freeDofs = free || [...Array(3*nn).keys()];
  const A = zeros(freeDofs.length, ne);
  const row = new Map(freeDofs.map((dof, r) => [dof, r]));
  pairs.forEach(([i, j], e) => {
    for (let a = 0; a < 3; a++) {
      // Cᵀ ⊗ I · blkdiag(H): node i (C=-1) gets -h, node j (C=+1) gets +h
      if (row.has(3*i+a)) A[row.get(3*i+a)][e] += -H[e][a];
      if (row.has(3*j+a)) A[row.get(3*j+a)][e] += H[e][a];
    }
  });
  const G = Gp || zeros(ne, ne).map((r, i) => { r[i] = 1; return r; });
  const Ag = matmul(A, G);
  const pinvG = G[0].length === ne ? null : null;
  const lGp = groupAverage(G, l);
  return { A, Ag, l, lGp, H, G, pinvG };
}

/** pinv(Gp)*v for a 0/1 group matrix = mean of v over each group. */
function groupAverage(G, v) {
  const ng = G[0].length;
  return [...Array(ng).keys()].map(g => {
    let sum = 0, n = 0;
    G.forEach((row, e) => { if (row[g]) { sum += v[e]; n++; } });
    return n ? sum/n : 0;
  });
}

/** Group matrix from group index lists (tenseg_str_gp); groups use 0-based member ids. */
export function groupMatrix(groups, ne) {
  if (!groups?.length) return null;
  const covered = new Set(groups.flat());
  const all = [...groups, ...[...Array(ne).keys()].filter(e => !covered.has(e)).map(e => [e])];
  const G = zeros(ne, all.length);
  all.forEach((members, g) => members.forEach(e => { G[e][g] = 1; }));
  return G;
}

/**
 * tenseg_svd: rank r of A_1ag, self-stress modes V2 (null space of A_1ag)
 * and mechanism count (null space of A_1agᵀ, includes rigid-body modes).
 */
export function svdNullspaces(Ag, { tol = 1e-6 } = {}) {
  // Singular values come from eig(AᵀA): an exact zero shows up near 1e-8·σmax
  // (squared conditioning), so the rank threshold is 1e-6·σmax.
  const At = transpose(Ag);
  const { values, vectors } = symmetricEigen(matmul(At, Ag));      // AᵀA: right singular vectors
  const sigmaMax = Math.sqrt(Math.max(...values.map(Math.abs), 0));
  const zero = values.map(v => Math.sqrt(Math.max(v, 0)) <= tol*Math.max(sigmaMax, 1e-300));
  const rank = zero.filter(z => !z).length;
  const V2 = vectors.filter((_, k) => zero[k]);
  const singularValues = values.map(v => Math.sqrt(Math.max(v, 0))).reverse();
  return { rank, selfStressModes: V2.length, V2, mechanisms: Ag.length-rank, singularValues };
}

/**
 * tenseg_prestress_design: force densities from designed forces fd on the
 * groups index_gp (w0 = 0): q_gp = V2 z with e_dᵀ V2 z = fd / l_d.
 */
export function prestressDesign(eq, V2, designed) {
  const ng = eq.G[0].length;
  const groups = designed.map(d => d.group);
  const qd = designed.map(d => d.force/eq.lGp[d.group]);
  const EdV2 = groups.map(g => V2.map(mode => mode[g]));          // (nd × nmodes)
  const z = leastSquares(EdV2, qd);
  const qGp = [...Array(ng).keys()].map(g => V2.reduce((s, mode, k) => s+mode[g]*z[k], 0));
  const q = eq.G.map(row => row.reduce((s, v, g) => s+v*qGp[g], 0));
  const t = q.map((qi, e) => qi*eq.l[e]);
  const residual = Math.max(...matmul(eq.A, q.map(v => [v])).map(r => Math.abs(r[0])));
  return { qGp, q, t, residual };
}

/** minimass_buckle_hollow: outer radius r of a tube (wall `thick`) with π³E(r⁴−(r−t)⁴)/(4l²) = f/c. */
export function hollowBarRadius(force, length, thick, E, c) {
  const target = force/c;
  const g = r => Math.PI**3*E/4/(length*length)*(r**4-(r-thick)**4)-target;
  const dg = r => Math.PI**3*E/4/(length*length)*(4*r**3-4*(r-thick)**3);
  let r = Math.max(thick, 1e-4);
  for (let k = 0; k < 100; k++) {
    const step = g(r)/dg(r);
    r = Math.max(thick, r-step);
    if (Math.abs(step) < 1e-12) break;
  }
  return { r, A: Math.PI*(r*r-(r-thick)**2) };
}

/**
 * tenseg_minimass: bars sized against buckling (safety coefficient c_b), strings
 * against yield (c_s); returns areas, radii, rest lengths l0 = E A l/(t + E A), masses.
 */
export function minimalMass({ t, l, bars, strings, barMaterial, stringMaterial, c_b = 0.5, c_s = 0.3, thick = 0, hollow = true }) {
  const ne = t.length;
  const A = new Array(ne).fill(0), radius = new Array(ne).fill(0), E = new Array(ne).fill(0), rho = new Array(ne).fill(0);
  for (const e of bars) {
    const f = Math.abs(t[e]);
    if (hollow && thick > 0) {
      const sized = hollowBarRadius(f, l[e], thick, barMaterial.E, c_b);
      A[e] = sized.A; radius[e] = sized.r;
    } else {
      A[e] = 2*l[e]*Math.sqrt(f/c_b/Math.PI/barMaterial.E);
      radius[e] = Math.sqrt(A[e]/Math.PI);
    }
    E[e] = barMaterial.E; rho[e] = barMaterial.rho;
  }
  for (const e of strings) {
    A[e] = Math.max(t[e], 0)/stringMaterial.sigma/c_s;
    radius[e] = Math.sqrt(A[e]/Math.PI);
    E[e] = stringMaterial.E; rho[e] = stringMaterial.rho;
  }
  // l0 = E A l/(t + E A); a zero-force string gets no area (mass 0, l0 = l)
  const l0 = t.map((ti, e) => (A[e] > 0 ? E[e]*A[e]*l[e]/(ti+E[e]*A[e]) : l[e]));
  const mass = rho.map((r, e) => r*A[e]*l0[e]);
  return { A, radius, E, l0, rho, mass, total: mass.reduce((s, m) => s+m, 0) };
}

/** tenseg_stiff_matx2: tangent stiffness K_T (3nn × 3nn). */
export function tangentStiffness(N, pairs, E, A, l0) {
  const nn = N.length, K = zeros(3*nn, 3*nn);
  pairs.forEach(([i, j], e) => {
    const b = [0, 1, 2].map(a => N[j][a]-N[i][a]);
    const l = Math.hypot(...b);
    const q = E[e]*A[e]*(1/l0[e]-1/l);
    const k = E[e]*A[e]/(l**3);
    for (let a = 0; a < 3; a++) {
      for (let c = 0; c < 3; c++) {
        const v = (a === c ? q : 0)+k*b[a]*b[c];
        K[3*i+a][3*i+c] += v; K[3*j+a][3*j+c] += v;
        K[3*i+a][3*j+c] -= v; K[3*j+a][3*i+c] -= v;
      }
    }
  });
  return K;
}

/** tenseg_mass_matrix: consistent (lumped = false) or lumped member mass + extra node masses. */
export function massMatrix(nn, pairs, memberMass, { lumped = false, nodeMass = null } = {}) {
  const M = zeros(3*nn, 3*nn);
  pairs.forEach(([i, j], e) => {
    const m = memberMass[e];
    for (let a = 0; a < 3; a++) {
      if (lumped) { M[3*i+a][3*i+a] += m/2; M[3*j+a][3*j+a] += m/2; }
      else {
        M[3*i+a][3*i+a] += m/3; M[3*j+a][3*j+a] += m/3;
        M[3*i+a][3*j+a] += m/6; M[3*j+a][3*i+a] += m/6;
      }
    }
  });
  if (nodeMass) nodeMass.forEach((m, n) => { for (let a = 0; a < 3; a++) M[3*n+a][3*n+a] += m; });
  return M;
}

/** tenseg_mode: natural frequencies [Hz] (ascending) and mode shapes of K v = ω² M v. */
export function modalAnalysis(K, M) {
  const L = cholesky(M);
  const Li = lowerInverse(L);
  const reduced = matmul(matmul(Li, K), transpose(Li));
  const sym = reduced.map((row, i) => row.map((v, j) => 0.5*(v+reduced[j][i])));
  const { values, vectors } = symmetricEigen(sym);
  const LiT = transpose(Li);
  const shapes = vectors.map(y => LiT.map(row => row.reduce((s, v, k) => s+v*y[k], 0)));
  const frequencies = values.map(v => Math.sqrt(Math.max(v, 0))/(2*Math.PI));
  return { eigenvalues: values, frequencies, shapes };
}

// ------------------------------------------------------------------ form-finding

/**
 * Six-bar form-finding. A six-bar sphere with node coordinates that are
 * permutations of (0, ±d, ±h) (bar half-length h, offset d of the parallel
 * bar pair) admits a self-stress state only for d = h/2 (the "expanded
 * octahedron", TsgFEM's lander). The regular icosahedron (d = h/φ = 0.618 h)
 * does not. Returns the d/h ratio of the given nodes and the prestressable
 * shape with identical topology, bar length and orientation.
 */
export function sixBarFormFind(nodes, bars) {
  // Frame of the three orthogonal bar directions (one per parallel bar pair).
  const dirs = [];
  for (const [i, j] of bars) {
    const d = [0, 1, 2].map(k => nodes[j][k]-nodes[i][k]);
    const n = Math.hypot(...d);
    const u = d.map(v => v/n);
    if (!dirs.some(w => Math.abs(w[0]*u[0]+w[1]*u[1]+w[2]*u[2]) > 0.9)) dirs.push(u);
  }
  if (dirs.length !== 3) throw new Error('six-bar form-finding expects three bar directions');
  // Gram–Schmidt for an exactly orthonormal frame R (rows)
  const R = [];
  for (const u of dirs) {
    const v = u.slice();
    for (const w of R) { const dot = v[0]*w[0]+v[1]*w[1]+v[2]*w[2]; for (let k = 0; k < 3; k++) v[k] -= dot*w[k]; }
    const n = Math.hypot(...v);
    R.push(v.map(x => x/n));
  }
  const toLocal = p => R.map(r => r[0]*p[0]+r[1]*p[1]+r[2]*p[2]);
  const toWorld = q => [0, 1, 2].map(k => R[0][k]*q[0]+R[1][k]*q[1]+R[2][k]*q[2]);
  const local = nodes.map(toLocal);
  const h = local.reduce((s, q) => s+Math.max(...q.map(Math.abs)), 0)/local.length;
  const ratios = local.map(q => { const a = q.map(Math.abs).sort((x, y) => y-x); return a[1]/a[0]; });
  const ratio = ratios.reduce((x, y) => x+y, 0)/ratios.length;
  // Snap every node to (±h along its bar, ±h/2 across, 0): the expanded octahedron
  const equilibrium = local.map(q => {
    const order = q.map((v, k) => [Math.abs(v), k]).sort((x, y) => y[0]-x[0]);
    const snapped = [0, 0, 0];
    snapped[order[0][1]] = Math.sign(q[order[0][1]])*h;
    snapped[order[1][1]] = Math.sign(q[order[1][1]])*h/2;
    return toWorld(snapped);
  });
  return { ratio, targetRatio: 0.5, h, equilibrium };
}

// ------------------------------------------------------------------ rover-level analysis

/**
 * Full TsgFEM statics + modal analysis of the six-bar rover (shell + payload core).
 * @param geometry { nodes: [[x,y,z]x12], core: [x,y,z], bars, outerStrings }
 * @param design   { cablePretension, corePretension, barMaterial, stringMaterial,
 *                   installed: { barArea, barInertia, cableArea, bucklingLoad, cableBreaking },
 *                   cableK, coreK, hubMass, payloadMass, c_b, c_s, thick }
 */
export function analyzeRover(geometry, design) {
  // Statics on the modelled rest shape first: does it admit a self-stress state?
  const modelled = svdNullspaces(equilibriumMatrix(geometry.nodes, [...geometry.bars, ...geometry.outerStrings]).A);
  const form = sixBarFormFind(geometry.nodes, geometry.bars);
  const shellNodes = modelled.selfStressModes > 0 ? geometry.nodes : form.equilibrium;
  const nodes = [...shellNodes, geometry.core];
  const core = nodes.length-1;
  const bars = geometry.bars.map(b => [...b]);
  const outer = geometry.outerStrings.map(s => [...s]);
  const coreCables = geometry.nodes.map((_, i) => [i, core]);
  const pairs = [...bars, ...outer, ...coreCables];
  const ne = pairs.length;
  const barIdx = bars.map((_, k) => k);
  const outerIdx = outer.map((_, k) => bars.length+k);
  const coreIdx = coreCables.map((_, k) => bars.length+outer.length+k);
  // Symmetric groups: all bars, all outer cables, all payload cables (tenseg_str_gp)
  const Gp = groupMatrix([barIdx, outerIdx, coreIdx], ne);
  const eq = equilibriumMatrix(nodes, pairs, { Gp });
  const ungrouped = svdNullspaces(eq.A);
  const grouped = svdNullspaces(eq.Ag);
  const designedForces = [
    { group: 1, force: design.cablePretension },
    ...(grouped.selfStressModes > 1 ? [{ group: 2, force: design.corePretension }] : [])
  ];
  const pre = prestressDesign(eq, grouped.V2, designedForces);
  const barForce = pre.t[barIdx[0]], cableForce = pre.t[outerIdx[0]], coreForce = pre.t[coreIdx[0]];

  // Minimal-mass design (TsgFEM) for comparison with the installed parts
  const mm = minimalMass({
    t: pre.t, l: eq.l, bars: barIdx, strings: [...outerIdx, ...coreIdx],
    barMaterial: design.barMaterial, stringMaterial: design.stringMaterial,
    c_b: design.c_b ?? 0.5, c_s: design.c_s ?? 0.3, thick: design.thick ?? 0.0015, hollow: true
  });

  // Modal analysis of the installed rover: bars elastic (EA of the tube), cables =
  // series spring + rope, given as effective axial stiffness k -> EA_eff = k l0.
  const inst = design.installed;
  const E = new Array(ne), A = new Array(ne), l0 = new Array(ne), mass = new Array(ne).fill(0);
  for (const e of barIdx) {
    E[e] = design.barMaterial.E; A[e] = inst.barArea;
    l0[e] = eq.l[e]*(1+Math.abs(barForce)/(E[e]*A[e]));          // compressed bar: l0 > l
    mass[e] = inst.barMass;
  }
  const springMember = (e, k, force) => {
    l0[e] = eq.l[e]-Math.max(force, 0)/k;                         // pretension = k (l − l0)
    E[e] = 1; A[e] = k*l0[e];                                     // EA_eff = k l0
    mass[e] = inst.cableMassPerMetre*eq.l[e];
  };
  for (const e of outerIdx) springMember(e, design.cableK, cableForce);
  for (const e of coreIdx) springMember(e, design.coreK, coreForce);
  const nodeMass = nodes.map((_, n) => (n === core ? design.payloadMass : design.hubMass));
  const K = tangentStiffness(nodes, pairs, E, A, l0);
  const M = massMatrix(nodes.length, pairs, mass, { lumped: false, nodeMass });
  const modal = modalAnalysis(K, M);
  const rigidCount = 6;
  const elastic = modal.frequencies.slice(rigidCount);

  const shellCheck = svdNullspaces(equilibriumMatrix(shellNodes, [...geometry.bars, ...geometry.outerStrings]).A);
  return {
    formFinding: {
      modelledRatio: form.ratio, equilibriumRatio: form.targetRatio,
      modelledSelfStressModes: modelled.selfStressModes, equilibriumSelfStressModes: shellCheck.selfStressModes,
      usedEquilibriumShape: shellNodes !== geometry.nodes,
      cableLength: { modelled: geometry.outerStrings.map(([i, j]) => Math.hypot(...[0, 1, 2].map(a => geometry.nodes[j][a]-geometry.nodes[i][a])))[0],
        equilibrium: eq.l[outerIdx[0]] },
      shellNodes
    },
    counts: { nodes: nodes.length, members: ne, dof: 3*nodes.length, bars: bars.length, cables: outer.length+coreCables.length },
    equilibrium: {
      rank: ungrouped.rank, selfStressModes: ungrouped.selfStressModes, mechanisms: ungrouped.mechanisms-rigidCount,
      groupedSelfStressModes: grouped.selfStressModes
    },
    prestress: {
      barForce, cableForce, coreForce, residual: pre.residual,
      forceDensity: { bar: pre.q[barIdx[0]], cable: pre.q[outerIdx[0]], core: pre.q[coreIdx[0]] }
    },
    minimalMass: {
      barRadius: mm.radius[barIdx[0]], barArea: mm.A[barIdx[0]], cableRadius: mm.radius[outerIdx[0]],
      coreCableRadius: mm.radius[coreIdx[0]], structureMass: mm.total,
      barsMass: barIdx.reduce((s, e) => s+mm.mass[e], 0), cablesMass: [...outerIdx, ...coreIdx].reduce((s, e) => s+mm.mass[e], 0)
    },
    installed: {
      bucklingSafety: inst.bucklingLoad/Math.max(Math.abs(barForce), 1e-9),
      cableSafety: inst.cableBreaking/Math.max(cableForce, 1e-9),
      // A payload cable with (numerically) zero designed force has no meaningful margin.
      coreCableSafety: coreForce > 1e-6*Math.abs(cableForce) ? inst.cableBreaking/coreForce : null
    },
    // Full member data, for exporting the model to the MATLAB/Octave TsgFEM.
    fem: { nodes, pairs, groups: { bars: barIdx, outer: outerIdx, core: coreIdx }, E, A, l0, memberMass: mass, nodeMass },
    modal: {
      rigidModes: modal.frequencies.slice(0, rigidCount),
      elasticHz: elastic,
      firstElasticHz: elastic[0],
      highestHz: modal.frequencies[modal.frequencies.length-1]
    }
  };
}
