/**
 * ROVERPARAMIDENTIFICATION.JS - the "Final Codes" particle filter applied to the
 * 3D six-bar rover.
 *
 * Static model: the TsgFEM equilibrium shape of the rover (js/structuralAnalysis.js,
 * 12 shell nodes + payload core, 6 bars, 24 outer cables, 12 payload cables), solved
 * with the same form-finding + loading Newton scheme as forward_tensegrity.m
 * (js/paramIdentification.js solveStatic). Rigid-body motion is removed with a
 * statically determinate 3-2-1 support (one shell node fixed, a second held in two
 * directions, a third in one), so the supports add no stiffness.
 *
 * Identified parameters (factors on the design values, like MainFile.m):
 *   kOuter   outer cable stiffness          kCore   payload cable stiffness
 *   preOuter outer cable pre-stretch        preCore payload cable pre-stretch
 *            (l - l0, sets the pretension)
 * Measurements: all 13 node positions under random point loads (up to +-20 N per
 * component on a random unsupported node). A cable cannot push: a solve that
 * leaves any cable slack counts as failed, like an unstable or unconverged solve.
 *
 * The filter is MainFile.m's, with one setting added: likelihood annealing
 * (filterDefaults). A Fisher-information check at the true values gives
 * Cramer-Rao standard deviations of 0.1-0.2 % for all four factors at 0.1 mm,
 * so they are identifiable, but outer stiffness and outer pre-stretch are -0.95
 * correlated (payload pair -0.83). Without annealing the cloud is captured at a
 * wrong point of that ridge in about half of the runs.
 */
import { roverStructuralModel } from './structuralAnalysis.js?v=20261027-ob1';
import { solveStatic, makeRandom } from './paramIdentification.js?v=20261029-pf1';

export const ROVER_PARAMS = Object.freeze([
  { key: 'kOuter', label: 'outer cable stiffness ×', symbol: 'k_o' },
  { key: 'kCore', label: 'payload cable stiffness ×', symbol: 'k_c' },
  { key: 'preOuter', label: 'outer cable pre-stretch ×', symbol: 'p_o' },
  { key: 'preCore', label: 'payload cable pre-stretch ×', symbol: 'p_c' }
]);
export const ROVER_TRUE = Object.freeze([1.10, 0.90, 1.20, 0.85]);

/** Statically determinate 3-2-1 support on three well-separated shell nodes. */
export function supports321(nodes, candidates) {
  const sub = (a, b) => [0, 1, 2].map(i => a[i]-b[i]);
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const argmax = v => v.reduce((m, x, i) => (Math.abs(x) > Math.abs(v[m]) ? i : m), 0);
  const a = candidates[0];
  const b = candidates.reduce((best, n) => (Math.hypot(...sub(nodes[n], nodes[a])) > Math.hypot(...sub(nodes[best], nodes[a])) ? n : best), candidates[1]);
  const d = sub(nodes[b], nodes[a]);
  let c = null, area = 0;
  for (const n of candidates) {
    if (n === a || n === b) continue;
    const s = Math.hypot(...cross(d, sub(nodes[n], nodes[a])));
    if (s > area) { area = s; c = n; }
  }
  const normal = cross(d, sub(nodes[c], nodes[a]));
  const fixed = nodes.map(() => [false, false, false]);
  fixed[a] = [true, true, true];
  const free = argmax(d);                              // b slides along its dominant direction
  fixed[b] = [0, 1, 2].map(i => i !== free);
  fixed[c][argmax(normal)] = true;                     // c held normal to the plane a-b-c
  return { fixed, nodes: { a, b, c } };
}

/** Base data of the rover static model (cached per material). */
const baseCache = new Map();
export function roverBase(material = 'lightweight') {
  if (baseCache.has(material)) return baseCache.get(material);
  const { js, design } = roverStructuralModel({ material });
  const f = js.fem;
  const len = ([i, j]) => Math.hypot(...[0, 1, 2].map(a => f.nodes[j][a]-f.nodes[i][a]));
  const kind = new Array(f.pairs.length);
  f.groups.bars.forEach(e => { kind[e] = 'bar'; });
  f.groups.outer.forEach(e => { kind[e] = 'outer'; });
  f.groups.core.forEach(e => { kind[e] = 'core'; });
  const core = f.nodes.length-1;
  const elements = f.pairs.map((pair, e) => {
    const l = len(pair), k = f.E[e]*f.A[e]/f.l0[e];
    // payload cables: the TsgFEM self-stress state carries none; use the simulator's design pretension
    const stretch = kind[e] === 'core' ? design.corePretension/k : l-f.l0[e];
    return { i: pair[0], j: pair[1], kind: kind[e], k, l, l0: f.l0[e], stretch };
  });
  const shell = f.nodes.map((_, n) => n).filter(n => n !== core);
  const { fixed, nodes: support } = supports321(f.nodes, shell);
  const base = { material, nodes: f.nodes.map(p => [...p]), elements, fixed, support, core, shell,
    pretension: { outer: design.cablePretension, core: design.corePretension } };
  baseCache.set(material, base);
  return base;
}

/** Static model for parameter factors theta = [kOuter, kCore, preOuter, preCore]. */
export function roverModel(theta, base = roverBase()) {
  const [kO, kC, pO, pC] = theta;
  const elements = base.elements.map(e => {
    if (e.kind === 'bar') return { i: e.i, j: e.j, k: e.k, L: e.l0, kind: e.kind };
    const outer = e.kind === 'outer';
    return { i: e.i, j: e.j, k: e.k*(outer ? kO : kC), L: e.l-e.stretch*(outer ? pO : pC), kind: e.kind };
  });
  return { nodes: base.nodes, elements, fixed: base.fixed };
}

/** Solve the rover under one load case; ok = stable, converged and every cable in tension. */
export function roverForward(theta, load, base = roverBase(), options = {}) {
  const model = roverModel(theta, base);
  const s = solveStatic(model, { ...options, load, dispTol: options.dispTol ?? 1e-7 });
  let ok = s.ok;
  if (ok) {
    for (let e = 0; e < model.elements.length; e++) {
      if (model.elements[e].kind !== 'bar' && !(s.elementForces[e] > 0)) { ok = false; break; }
    }
  }
  const y = new Float64Array(3*base.nodes.length);
  s.positions.forEach((p, n) => { for (let a = 0; a < 3; a++) y[3*n+a] = ok ? p[a] : NaN; });
  return { ok, y, solve: s, slack: s.ok && !ok };
}

/**
 * Identification problem for the rover: `steps` random load cases at the true
 * parameters (load cases that would slacken a cable are redrawn), node positions
 * in metres with optional sensor noise, uniform prior per parameter.
 */
export function roverProblem({ steps = 60, trueTheta = ROVER_TRUE, seed = 46, sigmaObs = 0, sigmaAssumed = 1e-4,
  lower = [0.6, 0.6, 0.6, 0.6], upper = [1.5, 1.5, 1.5, 1.5], maxForce = 20, material = 'lightweight' } = {}) {
  const rng = makeRandom(seed);
  const base = roverBase(material);
  const loadable = base.nodes.map((_, n) => n).filter(n => n !== base.support.a && n !== base.support.b && n !== base.support.c);
  const loads = [], observations = [];
  let redrawn = 0;
  while (loads.length < steps) {
    const node = loadable[Math.floor(rng.rand()*loadable.length)];
    const load = [{ node, f: [0, 1, 2].map(() => maxForce*(2*rng.rand()-1)) }];
    const truth = roverForward(trueTheta, load, base);
    if (!truth.ok) { redrawn++; continue; }
    loads.push(load);
    observations.push(truth.y.map(v => v+sigmaObs*rng.randn()));
  }
  return {
    id: 'rover', title: `Six-bar rover (${material})`, units: 'm', params: ROVER_PARAMS,
    trueTheta: [...trueTheta], lower: [...lower], upper: [...upper], loads, observations, sigmaAssumed, rng,
    forward: (theta, load) => roverForward(theta, load, base), base, redrawnLoads: redrawn,
    // likelihood annealing from 3 mm: the outer-cable stiffness and pre-stretch are -0.95 correlated
    filterDefaults: { anneal: { from: 3e-3, rate: 0.93 } }
  };
}
