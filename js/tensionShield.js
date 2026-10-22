/**
 * TENSIONSHIELD.JS - Tension-certified safety filter for cable-only locomotion
 *
 * Problem. While the six-bar shell rolls, gravity and the outer-cable
 * actuation compress it (the centroid drops ~15 %); the node-to-core
 * distances shrink and the 12 payload suspension cables lose their
 * pretension. Without a filter several of them are slack almost all the
 * time, i.e. the "suspended" payload rattles loosely inside the cage.
 *
 * Filter. Any tipping controller (SGT, the MPC teacher, the learned
 * guided-policy) proposes rest-length offsets u_nom ∈ R³⁶ (24 outer + 12
 * payload cables). The filter returns
 *
 *    u* = argmin ½ (u − u_nom)ᵀ W (u − u_nom)
 *    s.t. h_c(u) ≥ (1 − γ) h_c(now)   for every safety function h_c
 *         lo ≤ u ≤ hi
 *
 * a discrete-time control-barrier condition (forward invariance of
 * {h ≥ 0}; recovery at rate γ when currently violated). The h_c are
 * predicted with the same pinned quasi-static KKT sensitivities X = dq/du
 * that SGT already factorizes, so the filter adds no model and no second
 * factorization:
 *
 *   payload cable i:   h = T_i − T_min,   T_i(u) = k_c (L_i − l₀ᵢ + u_i + Σ_a ∂L_i/∂u_a Δu_a)
 *   outer cable c:     h = T_max − T_c(u)  (same form with k_s)
 *   payload–rod j:     h = d_j(u) − d_min,  ∂d_j/∂u_a = n̂ᵀ (x_core,a − (1−s) x_i,a − s x_k,a)
 *
 * W weights outer cables w_outer ≫ payload cables, so the filter restores
 * suspension integrity mainly with the payload cables and leaves the
 * locomotion command nearly untouched. The QP (36 variables, ~43 rows plus
 * the box) is solved by accelerated dual projected gradient (FISTA) with the
 * box kept exact in the primal.
 *
 * Scope (results/shield_study, 10 conditions × {SGT, learned policy}): the
 * barriers hold on the quasi-static prediction, and closed-loop payload-cable
 * slack drops from ~35 % to ~2 % of the time in every condition. The
 * clearance and outer-load limits are NOT guaranteed in closed loop: fast
 * tumbles and impacts are dynamic events outside the quasi-static model
 * (a second-order barrier on the DAE mass matrix would be needed).
 */

export const SHIELD_DEFAULTS = Object.freeze({
  coreTensionMin: 10,     // N, payload cables stay taut
  outerTensionMax: 150,   // N, outer cable load limit
  clearanceMin: 0.08,     // m, payload centre to rod axis (core radius 0.05 m + 0.03 m)
  gamma: 0.3,             // barrier decay rate per control tick
  wOuter: 10,             // weight on changing outer-cable commands
  wCore: 1,               // weight on changing payload-cable commands
  preserveShift: 1.0,     // fraction of the nominal predicted COM shift to keep (0 = off)
  sweeps: 3000,
  tolerance: 1e-4
});

const dot3 = (a, b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];

/**
 * @param sim     simulation (q, corePosition, cables, offsets, cfg)
 * @param sens    { displacements: [nu][3n] dq/du columns, nOuterNodes } from the KKT solve
 * @param nominal { cableTargets[24], coreTargets[12] } physical rest-length offsets [m]
 * @param bounds  { lo[36], hi[36] } physical bounds [m]
 */
export function shieldCommand(sim, sens, nominal, bounds, options = {}) {
  const o = { ...SHIELD_DEFAULTS, ...options };
  const cfg = sim.cfg, rover = sim.rover;
  const n = rover.nOuter, core = n, nu = 24+12;
  const X = sens.displacements;                                  // X[a] = flat dq/du_a (3(n+1))
  const node = (a, i) => [X[a][3*i], X[a][3*i+1], X[a][3*i+2]];
  const pos = i => (i === core ? sim.corePosition : sim.q[i]);
  const uNow = [...sim.currentActuationOffsets, ...sim.currentCoreActuationOffsets];
  const uNom = [...nominal.cableTargets, ...nominal.coreTargets];
  const rows = [];                                               // { a[36], b, kind, index, hNow }
  const cableRow = (i, j, actuator, k, l0, kind, index, sign, limit) => {
    // T(u) = k (L − l0 + u_self + Σ dL/du_a Δu_a);  sign=+1: T ≥ limit, sign=−1: T ≤ limit
    const pi = pos(i), pj = pos(j);
    const d = [pj[0]-pi[0], pj[1]-pi[1], pj[2]-pi[2]];
    const L = Math.max(1e-9, Math.hypot(...d));
    const e = d.map(v => v/L);
    const tNow = k*(L-l0+uNow[actuator]);
    const grad = new Array(nu);
    for (let a = 0; a < nu; a++) {
      const dj = node(a, j), di = node(a, i);
      grad[a] = k*(e[0]*(dj[0]-di[0])+e[1]*(dj[1]-di[1])+e[2]*(dj[2]-di[2]))+(a === actuator ? k : 0);
    }
    const hNow = sign*(tNow-limit);
    // h(u) = hNow + sign·grad·(u − uNow) ≥ (1−γ) hNow
    const a = grad.map(g => sign*g);
    const b = (1-o.gamma)*hNow-hNow+a.reduce((s, ai, idx) => s+ai*uNow[idx], 0);
    rows.push({ a, b, kind, index, hNow });
  };
  for (let i = 0; i < n; i++) {
    cableRow(i, core, 24+i, cfg.kCore, sim.baseCoreRestLengths[i], 'core', i, +1, o.coreTensionMin);
  }
  rover.outerStrings.forEach(([i, j], c) => {
    cableRow(i, j, c, cfg.kS, rover.l0_outerStrings[c], 'outer', c, -1, o.outerTensionMax);
  });
  rover.bars.forEach(([i, j], bar) => {
    const c = sim.corePosition, pi = sim.q[i], pj = sim.q[j];
    const ab = [pj[0]-pi[0], pj[1]-pi[1], pj[2]-pi[2]];
    const ap = [c[0]-pi[0], c[1]-pi[1], c[2]-pi[2]];
    const s = Math.max(0, Math.min(1, dot3(ap, ab)/Math.max(dot3(ab, ab), 1e-12)));
    const closest = [pi[0]+s*ab[0], pi[1]+s*ab[1], pi[2]+s*ab[2]];
    const diff = [c[0]-closest[0], c[1]-closest[1], c[2]-closest[2]];
    const dist = Math.max(1e-9, Math.hypot(...diff));
    const nhat = diff.map(v => v/dist);
    const a = new Array(nu);
    for (let k = 0; k < nu; k++) {
      const xc = node(k, core), xi = node(k, i), xj = node(k, j);
      a[k] = dot3(nhat, [xc[0]-(1-s)*xi[0]-s*xj[0], xc[1]-(1-s)*xi[1]-s*xj[1], xc[2]-(1-s)*xi[2]-s*xj[2]]);
    }
    const hNow = dist-o.clearanceMin;
    const b = (1-o.gamma)*hNow-hNow+a.reduce((sum, ak, idx) => sum+ak*uNow[idx], 0);
    rows.push({ a, b, kind: 'clearance', index: bar, hNow });
  });
  // Shift-preserving row: keep the nominal command's predicted COM shift
  // along the tipping normal, gᵀ(u − u_now) ≥ ρ gᵀ(u_nom − u_now), so the
  // filter re-routes the tipping effort instead of simply cancelling it.
  if (o.preserveShift && sens.shiftGradient) {
    const g = sens.shiftGradient;
    const nominalShift = g.reduce((acc, gk, k) => acc+gk*(uNom[k]-uNow[k]), 0);
    if (nominalShift > 0) {
      rows.push({ a: g.slice(), b: o.preserveShift*nominalShift+g.reduce((acc, gk, k) => acc+gk*uNow[k], 0), kind: 'shift', index: 0, hNow: 0 });
    }
  }

  // Dual projected gradient with Nesterov momentum (FISTA). The box is kept
  // exact in the primal: for diagonal W, argmin over lo ≤ u ≤ hi of the
  // Lagrangian is u(λ) = clip(v + W⁻¹ Gᵀλ, lo, hi). Rows are scaled to unit
  // W⁻¹-norm so a single tolerance applies to tension, clearance and shift.
  const winv = new Float64Array(nu).map((_, k) => 1/(k < 24 ? o.wOuter : o.wCore));
  const lo = bounds.lo, hi = bounds.hi;
  const solve = (activeRows) => {
    const m = activeRows.length;
    const G = new Float64Array(m*nu), h = new Float64Array(m);
    activeRows.forEach((row, r) => {
      let norm = 0;
      for (let k = 0; k < nu; k++) norm += row.a[k]*row.a[k]*winv[k];
      const scale = norm > 1e-18 ? 1/Math.sqrt(norm) : 0;
      for (let k = 0; k < nu; k++) G[r*nu+k] = row.a[k]*scale;
      h[r] = row.b*scale;
    });
    const primal = (lam, u) => {
      for (let k = 0; k < nu; k++) {
        let acc = 0;
        for (let r = 0; r < m; r++) acc += G[r*nu+k]*lam[r];
        u[k] = Math.max(lo[k], Math.min(hi[k], uNom[k]+winv[k]*acc));
      }
    };
    const violation = u => {
      let worst = 0;
      for (let r = 0; r < m; r++) {
        let gu = 0;
        for (let k = 0; k < nu; k++) gu += G[r*nu+k]*u[k];
        worst = Math.max(worst, h[r]-gu);
      }
      return worst;
    };
    // Lipschitz constant of the dual gradient: λmax(G W⁻¹ Gᵀ) by power iteration
    const M = new Float64Array(m*m);
    for (let r = 0; r < m; r++) {
      for (let q = r; q < m; q++) {
        let acc = 0;
        for (let k = 0; k < nu; k++) acc += G[r*nu+k]*winv[k]*G[q*nu+k];
        M[r*m+q] = acc; M[q*m+r] = acc;
      }
    }
    let x = new Float64Array(m).fill(1/Math.sqrt(Math.max(m, 1))), Lip = 1;
    for (let it = 0; it < 40; it++) {
      const y = new Float64Array(m);
      for (let r = 0; r < m; r++) { let acc = 0; for (let q = 0; q < m; q++) acc += M[r*m+q]*x[q]; y[r] = acc; }
      let norm = 0; for (let r = 0; r < m; r++) norm += y[r]*y[r];
      norm = Math.sqrt(norm) || 1;
      Lip = norm;
      for (let r = 0; r < m; r++) x[r] = y[r]/norm;
    }
    const step = 1/Math.max(1.05*Lip, 1e-9);
    let lambda = new Float64Array(m), tk = 1;
    const yv = new Float64Array(m), u = new Float64Array(nu), next = new Float64Array(m);
    let iterations = 0, worst = Infinity;
    for (; iterations < o.sweeps; iterations++) {
      primal(yv, u);
      for (let r = 0; r < m; r++) {
        let gu = 0;
        for (let k = 0; k < nu; k++) gu += G[r*nu+k]*u[k];
        next[r] = Math.max(0, yv[r]+step*(h[r]-gu));
      }
      const tNext = (1+Math.sqrt(1+4*tk*tk))/2;
      for (let r = 0; r < m; r++) { yv[r] = next[r]+((tk-1)/tNext)*(next[r]-lambda[r]); lambda[r] = next[r]; }
      tk = tNext;
      if (iterations % 10 === 9) {
        primal(lambda, u);
        worst = violation(u);
        if (worst < o.tolerance) break;
      }
    }
    primal(lambda, u);
    worst = violation(u);
    return { u, lambda, sweeps: iterations+1, converged: worst < o.tolerance };
  };
  const general = rows.filter(r => r.kind !== 'box');
  let solution = solve(general);
  let shiftDropped = false;
  if (!solution.converged && general.some(r => r.kind === 'shift')) {
    // Safety before progress: drop the shift row if both cannot hold.
    solution = solve(general.filter(r => r.kind !== 'shift'));
    shiftDropped = true;
  }
  const { u, lambda, sweeps } = solution;
  const worst = solution.converged ? 0 : 1;
  const usedRows = shiftDropped ? general.filter(r => r.kind !== 'shift') : general;
  const active = usedRows.reduce((count, row, r) => count+(lambda[r] > 1e-9 ? 1 : 0), 0);
  const violated = rows.filter(row => row.kind !== 'box' && row.hNow < 0).length;
  const correction = Math.sqrt(Array.from(u).reduce((acc, v, k) => acc+(v-uNom[k])**2, 0));
  return {
    cableTargets: Array.from(u.slice(0, 24)), coreTargets: Array.from(u.slice(24)),
    info: { active, violatedNow: violated, sweeps, converged: worst === 0, shiftDropped, correction,
      minCoreTension: Math.min(...rows.filter(r => r.kind === 'core').map(r => r.hNow+o.coreTensionMin)) }
  };
}
