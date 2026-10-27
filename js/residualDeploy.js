/**
 * RESIDUALDEPLOY.JS - deployment hooks for the two learned controllers.
 *
 *   controllerMode 'residual_mppi'      MPC + RL        residual-MPPI actor trained with the
 *                                                       MPC/reward parameters held fixed
 *                                                       (js/residualMppiWeightsRl.js), run with
 *                                                       the default gait/shield parameters.
 *   controllerMode 'residual_mppi_llm'  MPC + RL + LLM  actor from the LLM co-designer loop
 *                                                       (js/residualMppiWeights.js), run with the
 *                                                       parameters the co-designer selected.
 *
 * The distilled actor prior runs in the browser/UI and in single-threaded
 * experiments without any workers (the MPPI search itself is a training-time /
 * optional test-time tool, see js/mppiPlanner.js). cfg.residualMppi overrides
 * the packaged weights. If the fixed-parameter (RL-only) checkpoint has not been
 * trained yet, MPC + RL falls back to the co-designer actor but still runs with
 * the default parameters, so the two modes stay distinct.
 */
import { MorphologyActorCritic, synergyMatrix, residualPolicyHook, DEFAULT_PARAMS } from './residualMppi.js?v=20261027-ob1';
import { RESIDUAL_MPPI_WEIGHTS } from './residualMppiWeights.js?v=20261027-ob1';
import { RESIDUAL_MPPI_WEIGHTS_RL } from './residualMppiWeightsRl.js?v=20261027-ob1';
import { OnlineResidualPlanner } from './onlinePlanner.js?v=20261027-ob1';

export const isLlmMode = mode => mode === 'residual_mppi_llm';
export const isResidualMode = mode => mode === 'residual_mppi' || mode === 'residual_mppi_llm';

/** Packaged checkpoint for a mode: { weights, params, source }. */
export function residualCheckpoint(mode = 'residual_mppi', level = null) {
  if (isLlmMode(mode)) {
    // 14-terrain checkpoints carry one configuration per level (paramsByLevel).
    const params = (level && RESIDUAL_MPPI_WEIGHTS?.paramsByLevel?.[level]) || RESIDUAL_MPPI_WEIGHTS?.params || {};
    return { weights: RESIDUAL_MPPI_WEIGHTS, params, source: RESIDUAL_MPPI_WEIGHTS ? 'llm' : 'untrained' };
  }
  if (RESIDUAL_MPPI_WEIGHTS_RL) return { weights: RESIDUAL_MPPI_WEIGHTS_RL, params: {}, source: 'rl' };
  return { weights: RESIDUAL_MPPI_WEIGHTS, params: {}, source: RESIDUAL_MPPI_WEIGHTS ? 'llm-actor-default-params' : 'untrained' };
}

export function residualHookFor(sim, mode = sim.cfg.controllerMode) {
  if (sim.cfg.residualMppi) return sim.cfg.residualMppi;
  const level = sim.cfg.experimentId || 1;
  const { weights, params: tuned } = residualCheckpoint(mode, level);
  const planning = Boolean(sim.cfg.residualOnlinePlanning);
  if (!weights && !planning) return { maxResidualStrokeM: DEFAULT_PARAMS.maxResidualStrokeM, maxTotalStrokeM: DEFAULT_PARAMS.maxTotalStrokeM, act: () => null };
  const model = new MorphologyActorCritic(synergyMatrix(sim.rover));
  if (weights) model.load(weights);
  // The policy is MPC-conditioned: it must see the configuration it actually runs with.
  if (isLlmMode(mode)) model.params = { ...model.params, ...tuned };
  // MPC + RL always runs with the default stroke limits; MPC + RL + LLM uses the tuned ones.
  if (!isLlmMode(mode)) model.params = { ...model.params, maxResidualStrokeM: DEFAULT_PARAMS.maxResidualStrokeM, maxTotalStrokeM: DEFAULT_PARAMS.maxTotalStrokeM };
  if (planning) {
    // Live receding-horizon residual-MPPI around the actor prior (js/onlinePlanner.js).
    const planner = new OnlineResidualPlanner(sim, model, sim.cfg.onlinePlanning || {});
    sim.onlinePlanner = planner;
    sim.preStep = s => planner.maybePlan(s);
    return planner.hook;
  }
  return residualPolicyHook(model);
}

/** Gait/shield parameters for a mode: LLM-tuned for 'residual_mppi_llm', defaults otherwise. */
export function residualTunedParams(mode = 'residual_mppi_llm', level = null) {
  return residualCheckpoint(mode, level).params;
}
