/**
 * PARAMIDPANELCORE.JS - builds the identification problem for the Tools tab
 * (shared by the panel and its worker).
 */
import { planarProblem } from './paramIdentification.js?v=20261029-pf1';
import { roverProblem } from './roverParamIdentification.js?v=20261029-pf1';

/** settings: { mode: 'planar' | 'rover', steps, seed, noiseMm } */
export function buildIdentificationProblem(s) {
  return s.mode === 'rover'
    ? roverProblem({ steps: s.steps, seed: s.seed, sigmaObs: s.noiseMm*1e-3 })
    : planarProblem({ steps: s.steps, seed: s.seed, sigmaObs: s.noiseMm });
}

/** The cloneable part of a problem (no functions), for postMessage and the JSON download. */
export function problemInfo(p) {
  return { id: p.id, title: p.title, units: p.units, params: p.params, trueTheta: p.trueTheta, lower: p.lower, upper: p.upper,
    sigmaAssumed: p.sigmaAssumed, filterDefaults: p.filterDefaults, loads: p.loads };
}
