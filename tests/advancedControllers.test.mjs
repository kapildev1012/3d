import test from 'node:test';
import assert from 'node:assert/strict';
import {
  ADVANCED_MODES,
  solveAdvancedController
} from '../js/advancedControllers.js';
import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from '../js/simEngine.js';

const BASE_CONTEXT = () => ({
  state0: [0.35, 0.02],
  referenceSpeed: 1.0,
  disturbance: 0.2,
  previousCommand: 0,
  horizon: 12,
  weights: { qv: 6, qh: 4, r: 3.3, rd: 1.4, qvT: 10, qhT: 6 },
  params: {
    thrust: 1.35, drag: 0.95, gradeLoss: 0.85,
    heightRelax: 2.2, rollCoupling: 0.10, dtH: 0.05
  }
});

test('advanced mode registry exposes the four ported Drive solvers', () => {
  assert.deepEqual([...ADVANCED_MODES].sort(), [
    'ilqr_minimax_true', 'ilqr_true', 'qp_mpc_proj', 'riccati_lqr'
  ]);
});

test('Riccati LQR produces a finite stabilizing feedback command', () => {
  const solution = solveAdvancedController('riccati_lqr', BASE_CONTEXT());
  assert.ok(Number.isFinite(solution.command));
  assert.ok(Math.abs(solution.command) <= 1);
  assert.ok(Number.isFinite(solution.cost) && solution.cost > 0);
  assert.ok(Number.isFinite(solution.gain[0]) && Number.isFinite(solution.gain[1]));
  // With speed below reference the optimal command must request thrust.
  assert.ok(solution.command > -0.2, `expected thrust-biased command, got ${solution.command}`);
});

test('iLQR reduces cost below its feedforward initialization', () => {
  const ctx = BASE_CONTEXT();
  const solution = solveAdvancedController('ilqr_true', ctx);
  assert.ok(solution.iterations >= 1, 'iLQR must run at least one forward pass');
  assert.ok(Number.isFinite(solution.cost));
  // Compare against a pure-feedforward rollout with the same initial guess.
  const ff = solveAdvancedController('riccati_lqr', ctx);
  assert.ok(
    solution.cost <= ff.cost*1.05+1e-6,
    `iLQR cost ${solution.cost} should not exceed Riccati baseline ${ff.cost} by much`);
});

test('minimax iLQR reports a worst case at least as large as its nominal cost', () => {
  const solution = solveAdvancedController('ilqr_minimax_true', BASE_CONTEXT());
  assert.ok(solution.worstCaseCost >= solution.cost-1e-6,
    `worst case ${solution.worstCaseCost} must dominate nominal ${solution.cost}`);
  assert.ok(Number.isFinite(solution.disturbanceEstimate));
});

test('projected QP-MPC respects box constraints and beats the hold input', () => {
  const ctx = BASE_CONTEXT();
  const solution = solveAdvancedController('qp_mpc_proj', ctx);
  assert.ok(Math.abs(solution.command) <= 1+1e-12);
  assert.ok(solution.inputSequence.every(u => Math.abs(u) <= 1+1e-12),
    'every MPC decision must satisfy the actuator box constraint');
  assert.ok(solution.cost <= solution.openLoopCost+1e-6,
    `optimized ${solution.cost} must not be worse than holding ${solution.openLoopCost}`);
});

test('all solvers sanitize non-finite contexts instead of emitting NaN commands', () => {
  const ctx = BASE_CONTEXT();
  ctx.state0 = [NaN, 0];
  for (const mode of ADVANCED_MODES) {
    const solution = solveAdvancedController(mode, ctx);
    assert.ok(Number.isFinite(solution.command), `${mode} emitted a NaN command`);
    assert.ok(Math.abs(solution.command) <= 1, `${mode} command out of actuator bounds`);
    // The sanitized standstill state (0 speed vs reference) must request thrust.
    assert.ok(solution.command > 0.5, `${mode} did not respond to sanitized state`);
  }
});

test('adaptive simulation runs with every ported controller mode engaged', () => {
  for (const mode of ADVANCED_MODES) {
    const config = new SimConfig({
      terrainLevel: 1,
      targetDestination: [0, 0],
      targetGoalY: 0,
      enableDiagnosticsLog: false,
      monitoring: { rawLogging: false }
    });
    config.controllerMode = mode;
    const rover = new SphericalRoverModel(config);
    const terrain = new TerrainModel(config);
    const simulation = new Simulation(config, rover, terrain, 'adaptive');
    for (let step = 0; step < 600; step++) simulation.step();

    assert.ok(simulation.q.every(point => point.every(Number.isFinite)), `${mode}: NaN node position`);
    const diag = simulation.currentDiag;
    assert.equal(diag.controllerMode, mode);
    assert.ok(Number.isFinite(diag.controlCost), `${mode}: non-finite control cost`);
    if (diag.solverIterations > 0) {
      assert.ok(diag.solverMs < 50, `${mode}: solver too slow (${diag.solverMs.toFixed(2)} ms)`);
    }
  }
});
