import {
  add,
  addDiagonal,
  blockMatrix,
  clampVector,
  diagonal,
  dot,
  finiteDifferenceJacobian,
  finiteHorizonRiccati,
  identity,
  matVec,
  multiply,
  norm,
  outer,
  quadraticForm,
  scale,
  solve,
  subtract,
  symmetrize,
  transpose,
  trapezoidalDiscretize,
  vecAdd,
  vecScale,
  vecSubtract,
  zeros
} from './linearAlgebra.js';

export const DRIVE_CONTROLLER_LABELS = Object.freeze({
  lqr: 'LQR Rolling Direction',
  lqr_payload: 'LQR Rolling Direction · central payload',
  ilqr: 'iLQR Rolling Direction',
  ilqr_minimax: 'iLQR minimax Rolling Direction',
  ilqr_minimax_penalty: 'iLQR minimax · input penalty',
  qp_mpc: 'QP-MPC Rolling Direction',
  qp_mpc_payload: 'QP-MPC · central payload',
  neural: 'Neural Rolling Direction'
});

function copyMatrix(matrix) {
  return matrix.map(row => row.slice());
}

function positiveDefinite(matrix, margin) {
  const symmetric = symmetrize(matrix);
  const lowerBound = Math.min(...symmetric.map((row, i) =>
    row[i] - row.reduce((sum, value, j) => sum + (i === j ? 0 : Math.abs(value)), 0)));
  return addDiagonal(symmetric, Math.max(0, margin - lowerBound));
}

function negativeDefinite(matrix, margin) {
  const symmetric = symmetrize(matrix);
  const upperBound = Math.max(...symmetric.map((row, i) =>
    row[i] + row.reduce((sum, value, j) => sum + (i === j ? 0 : Math.abs(value)), 0)));
  return addDiagonal(symmetric, Math.min(0, -margin - upperBound));
}

function constantReference(reference, horizon) {
  if (Array.isArray(reference[0])) return reference;
  return Array.from({ length: horizon + 1 }, () => reference.slice());
}

function linearModel(model, state, input) {
  if (model.Ac && model.Bc) return { ...trapezoidalDiscretize(model.Ac, model.Bc, model.dt), c: model.c || null };
  if (model.A && model.B) return { A: model.A, B: model.B, c: model.c || null };
  const zeroInput = input || new Array(model.inputSize).fill(0);
  const A = finiteDifferenceJacobian(candidate => model.step(candidate, zeroInput), state);
  const B = finiteDifferenceJacobian(candidate => model.step(state, candidate), zeroInput);
  const nominal = model.step(state, zeroInput);
  const affine = vecSubtract(nominal, vecAdd(matVec(A, state), matVec(B, zeroInput)));
  return { A, B, c: affine };
}

function linearStep(linear, state, input) {
  let next = vecAdd(matVec(linear.A, state), matVec(linear.B, input));
  if (linear.c) next = vecAdd(next, linear.c);
  return next;
}

function stateInputJacobians(model, state, input, disturbance = null) {
  return {
    A: model.A || finiteDifferenceJacobian(candidate => model.step(candidate, input, disturbance), state),
    B: model.B || finiteDifferenceJacobian(candidate => model.step(state, candidate, disturbance), input),
    W: disturbance === null ? null : (model.W || finiteDifferenceJacobian(
      candidate => model.step(state, input, candidate), disturbance))
  };
}

function projectActuators(input, constraints = {}) {
  const minimum = constraints.minimum ?? -Infinity;
  const maximum = constraints.maximum ?? Infinity;
  const result = clampVector(input, minimum, maximum);
  for (const index of constraints.passive || []) result[index] = 0;
  for (const [a, b] of constraints.paired || []) {
    const magnitude = 0.5 * (Math.abs(result[a]) + Math.abs(result[b]));
    const sign = Math.sign(result[a] - result[b]) || 1;
    result[a] = sign * magnitude;
    result[b] = -sign * magnitude;
  }
  for (const group of constraints.similar || []) {
    const mean = group.reduce((sum, index) => sum + result[index], 0) / Math.max(1, group.length);
    for (const index of group) result[index] = mean;
  }
  return clampVector(result, minimum, maximum);
}

function trajectoryCost(states, inputs, references, Q, R, terminalQ, changePenalty, previousInput) {
  let cost = quadraticForm(vecSubtract(states.at(-1), references.at(-1)), terminalQ);
  let last = previousInput || new Array(inputs[0]?.length || 0).fill(0);
  for (let k = 0; k < inputs.length; k++) {
    const error = vecSubtract(states[k], references[k]);
    cost += quadraticForm(error, Q) + quadraticForm(inputs[k], R);
    if (changePenalty) cost += quadraticForm(vecSubtract(inputs[k], last), changePenalty);
    last = inputs[k];
  }
  return cost;
}

function rolloutLinear(linear, state, inputs) {
  const states = [state.slice()];
  for (const input of inputs) states.push(linearStep(linear, states.at(-1), input));
  return states;
}

function rolloutNonlinear(model, state, inputs, disturbances = null) {
  const states = [state.slice()];
  for (let k = 0; k < inputs.length; k++) {
    states.push(model.step(states.at(-1), inputs[k], disturbances?.[k]));
  }
  return states;
}

function controllerResult(mode, input, states, cost, extra = {}) {
  return {
    mode,
    label: DRIVE_CONTROLLER_LABELS[mode],
    input,
    predictedStates: states,
    cost,
    ...extra
  };
}

/** Finite-horizon, affine-tracking Riccati controller from LQR_RollingDirection.m. */
export class LQRRollingDirectionController {
  constructor(model, options = {}) {
    this.model = model;
    this.horizon = options.horizon ?? 12;
    this.Q = copyMatrix(options.Q || model.Q);
    this.R = copyMatrix(options.R || model.R);
    this.terminalQ = copyMatrix(options.terminalQ || this.Q);
    this.constraints = options.constraints || model.constraints || {};
    this.mode = options.mode || 'lqr';
  }

  solve({ state, reference, previousInput }) {
    const references = constantReference(reference, this.horizon);
    const linear = linearModel(this.model, state, previousInput);
    const { P, K } = finiteHorizonRiccati(linear.A, linear.B, this.Q, this.R, this.horizon, this.terminalQ);
    const error = vecSubtract(state, references[0]);
    const raw = vecScale(matVec(K[0], error), -1);
    const input = projectActuators(raw, this.constraints);
    const inputs = Array.from({ length: this.horizon }, () => input.slice());
    const states = rolloutLinear(linear, state, inputs);
    return controllerResult(this.mode, input, states,
      trajectoryCost(states, inputs, references, this.Q, this.R, this.terminalQ),
      { gain: K[0], valueMatrix: P[0] });
  }
}

/** Mass/height-weighted LQR variant from LQR_RollingDirection_centralPayload.m. */
export class CentralPayloadLQRController extends LQRRollingDirectionController {
  constructor(model, options = {}) {
    const Q = copyMatrix(options.Q || model.Q);
    for (const index of options.payloadStateIndices || model.payloadStateIndices || []) Q[index][index] += options.payloadWeight ?? 10;
    super(model, { ...options, Q, mode: 'lqr_payload' });
  }
}

/**
 * Box-constrained receding-horizon quadratic program. The MATLAB source uses
 * YALMIP/Gurobi; this port solves the same convex shooting form with projected
 * adjoint-gradient iterations and trapezoidal model discretization.
 */
export class QPMPCController {
  constructor(model, options = {}) {
    this.model = model;
    this.horizon = options.horizon ?? 12;
    this.Q = copyMatrix(options.Q || model.Q);
    this.R = copyMatrix(options.R || model.R);
    this.terminalQ = copyMatrix(options.terminalQ || this.Q);
    this.changePenalty = options.changePenalty || null;
    this.constraints = options.constraints || model.constraints || {};
    this.iterations = options.iterations ?? 24;
    this.learningRate = options.learningRate ?? 0.08;
    this.mode = options.mode || 'qp_mpc';
  }

  solve({ state, reference, previousInput, warmStart }) {
    const references = constantReference(reference, this.horizon);
    const linear = linearModel(this.model, state, previousInput);
    const inputSize = linear.B[0].length;
    let inputs = Array.from({ length: this.horizon }, (_, k) =>
      projectActuators(warmStart?.[k] || previousInput || new Array(inputSize).fill(0), this.constraints));
    let states = rolloutLinear(linear, state, inputs);
    let bestCost = trajectoryCost(states, inputs, references, this.Q, this.R, this.terminalQ, this.changePenalty, previousInput);
    let stepSize = this.learningRate;

    for (let iteration = 0; iteration < this.iterations; iteration++) {
      const costates = new Array(this.horizon + 1);
      costates[this.horizon] = vecScale(matVec(this.terminalQ,
        vecSubtract(states[this.horizon], references[this.horizon])), 2);
      const gradients = new Array(this.horizon);
      for (let k = this.horizon - 1; k >= 0; k--) {
        const stateGradient = vecScale(matVec(this.Q, vecSubtract(states[k], references[k])), 2);
        let inputGradient = vecAdd(vecScale(matVec(this.R, inputs[k]), 2),
          matVec(transpose(linear.B), costates[k + 1]));
        if (this.changePenalty) {
          const before = k === 0 ? (previousInput || new Array(inputSize).fill(0)) : inputs[k - 1];
          inputGradient = vecAdd(inputGradient,
            vecScale(matVec(this.changePenalty, vecSubtract(inputs[k], before)), 2));
          if (k + 1 < this.horizon) {
            inputGradient = vecAdd(inputGradient,
              vecScale(matVec(this.changePenalty, vecSubtract(inputs[k], inputs[k + 1])), 2));
          }
        }
        gradients[k] = inputGradient;
        costates[k] = vecAdd(stateGradient, matVec(transpose(linear.A), costates[k + 1]));
      }
      const candidateInputs = inputs.map((input, k) =>
        projectActuators(vecSubtract(input, vecScale(gradients[k], stepSize)), this.constraints));
      const candidateStates = rolloutLinear(linear, state, candidateInputs);
      const candidateCost = trajectoryCost(candidateStates, candidateInputs, references,
        this.Q, this.R, this.terminalQ, this.changePenalty, previousInput);
      if (candidateCost <= bestCost) {
        inputs = candidateInputs;
        states = candidateStates;
        if (bestCost - candidateCost < 1e-7) break;
        bestCost = candidateCost;
        stepSize = Math.min(this.learningRate, stepSize * 1.15);
      } else {
        stepSize *= 0.5;
        if (stepSize < 1e-6) break;
      }
    }
    return controllerResult(this.mode, inputs[0], states, bestCost, { inputTrajectory: inputs });
  }
}

export class CentralPayloadQPMPCController extends QPMPCController {
  constructor(model, options = {}) {
    const Q = copyMatrix(options.Q || model.Q);
    for (const index of options.payloadStateIndices || model.payloadStateIndices || []) Q[index][index] += options.payloadWeight ?? 12;
    super(model, { ...options, Q, mode: 'qp_mpc_payload' });
  }
}

/** Nonlinear iterative LQR with finite-difference dynamics and backtracking. */
export class IterativeLQRController {
  constructor(model, options = {}) {
    this.model = model;
    this.horizon = options.horizon ?? 12;
    this.Q = copyMatrix(options.Q || model.Q);
    this.R = copyMatrix(options.R || model.R);
    this.terminalQ = copyMatrix(options.terminalQ || this.Q);
    this.changePenalty = options.changePenalty || null;
    this.constraints = options.constraints || model.constraints || {};
    this.iterations = options.iterations ?? 6;
    this.regularization = options.regularization ?? 1e-4;
    this.mode = options.mode || 'ilqr';
  }

  solve({ state, reference, previousInput, warmStart }) {
    const references = constantReference(reference, this.horizon);
    const inputSize = this.model.inputSize || this.model.B?.[0]?.length;
    let inputs = Array.from({ length: this.horizon }, (_, k) =>
      projectActuators(warmStart?.[k] || previousInput || new Array(inputSize).fill(0), this.constraints));
    let states = rolloutNonlinear(this.model, state, inputs);
    let bestCost = trajectoryCost(states, inputs, references, this.Q, this.R, this.terminalQ, this.changePenalty, previousInput);

    for (let iteration = 0; iteration < this.iterations; iteration++) {
      const feedforward = new Array(this.horizon);
      const feedback = new Array(this.horizon);
      let Vx = vecScale(matVec(this.terminalQ, vecSubtract(states.at(-1), references.at(-1))), 2);
      let Vxx = scale(this.terminalQ, 2);

      for (let k = this.horizon - 1; k >= 0; k--) {
        const stateAt = states[k];
        const inputAt = inputs[k];
        const { A, B } = stateInputJacobians(this.model, stateAt, inputAt);
        const error = vecSubtract(stateAt, references[k]);
        const Qx = vecAdd(vecScale(matVec(this.Q, error), 2), matVec(transpose(A), Vx));
        let Qu = vecAdd(vecScale(matVec(this.R, inputAt), 2), matVec(transpose(B), Vx));
        let Quu = add(scale(this.R, 2), multiply(multiply(transpose(B), Vxx), B));
        if (this.changePenalty) {
          const before = k === 0 ? (previousInput || new Array(inputSize).fill(0)) : inputs[k - 1];
          Qu = vecAdd(Qu, vecScale(matVec(this.changePenalty, vecSubtract(inputAt, before)), 2));
          Quu = add(Quu, scale(this.changePenalty, 2));
        }
        const Qxx = add(scale(this.Q, 2), multiply(multiply(transpose(A), Vxx), A));
        const Qux = multiply(multiply(transpose(B), Vxx), A);
        Quu = positiveDefinite(Quu, this.regularization);
        feedforward[k] = vecScale(solve(Quu, Qu), -1);
        feedback[k] = scale(solve(Quu, Qux), -1);
        const KT = transpose(feedback[k]);
        Vx = vecAdd(Qx, vecAdd(
          matVec(KT, vecAdd(Qu, matVec(Quu, feedforward[k]))),
          matVec(transpose(Qux), feedforward[k])
        ));
        Vxx = symmetrize(add(Qxx, add(
          multiply(multiply(KT, Quu), feedback[k]),
          add(multiply(KT, Qux), multiply(transpose(Qux), feedback[k]))
        )));
      }

      let accepted = false;
      for (const alpha of [1, 0.5, 0.25, 0.1, 0.05]) {
        const candidateStates = [state.slice()];
        const candidateInputs = [];
        for (let k = 0; k < this.horizon; k++) {
          const correction = matVec(feedback[k], vecSubtract(candidateStates[k], states[k]));
          const candidate = vecAdd(inputs[k], vecAdd(vecScale(feedforward[k], alpha), correction));
          candidateInputs[k] = projectActuators(candidate, this.constraints);
          candidateStates.push(this.model.step(candidateStates[k], candidateInputs[k]));
        }
        const candidateCost = trajectoryCost(candidateStates, candidateInputs, references,
          this.Q, this.R, this.terminalQ, this.changePenalty, previousInput);
        if (candidateCost < bestCost) {
          const improvement = bestCost - candidateCost;
          inputs = candidateInputs;
          states = candidateStates;
          bestCost = candidateCost;
          accepted = true;
          if (improvement < 1e-6) iteration = this.iterations;
          break;
        }
      }
      if (!accepted) break;
    }
    return controllerResult(this.mode, inputs[0], states, bestCost, { inputTrajectory: inputs });
  }
}

function robustTrajectoryCost(states, inputs, disturbances, references, Q, R, G, terminalQ, changePenalty, previousInput) {
  let cost = trajectoryCost(states, inputs, references, Q, R, terminalQ, changePenalty, previousInput);
  for (const disturbance of disturbances) cost -= quadraticForm(disturbance, G);
  return cost;
}

/** Saddle-point iLQR pass from iLQRminimax_RollingDirection_2.m. */
export class MinimaxIterativeLQRController {
  constructor(model, options = {}) {
    this.model = model;
    this.horizon = options.horizon ?? 12;
    this.Q = copyMatrix(options.Q || model.Q);
    this.R = copyMatrix(options.R || model.R);
    this.G = copyMatrix(options.G || diagonal(new Array(model.stateSize).fill(1e3)));
    this.terminalQ = copyMatrix(options.terminalQ || this.Q);
    this.changePenalty = options.changePenalty || null;
    this.constraints = options.constraints || model.constraints || {};
    this.disturbanceBounds = options.disturbanceBounds || model.disturbanceBounds || new Array(model.stateSize).fill(0.02);
    this.iterations = options.iterations ?? 3;
    this.regularization = options.regularization ?? 1e-3;
    this.mode = options.mode || 'ilqr_minimax';
  }

  solve({ state, reference, previousInput, warmStart }) {
    const references = constantReference(reference, this.horizon);
    const inputSize = this.model.inputSize || this.model.B?.[0]?.length;
    const disturbanceSize = this.model.disturbanceSize || state.length;
    let inputs = Array.from({ length: this.horizon }, (_, k) =>
      projectActuators(warmStart?.[k] || previousInput || new Array(inputSize).fill(0), this.constraints));
    let disturbances = Array.from({ length: this.horizon }, () => new Array(disturbanceSize).fill(0));
    let states = rolloutNonlinear(this.model, state, inputs, disturbances);

    for (let iteration = 0; iteration < this.iterations; iteration++) {
      let Vx = vecScale(matVec(this.terminalQ, vecSubtract(states.at(-1), references.at(-1))), 2);
      let Vxx = scale(this.terminalQ, 2);
      const policies = new Array(this.horizon);
      for (let k = this.horizon - 1; k >= 0; k--) {
        const x = states[k];
        const u = inputs[k];
        const w = disturbances[k];
        const { A, B, W } = stateInputJacobians(this.model, x, u, w);
        const error = vecSubtract(x, references[k]);
        const Qx = vecAdd(vecScale(matVec(this.Q, error), 2), matVec(transpose(A), Vx));
        let Qu = vecAdd(vecScale(matVec(this.R, u), 2), matVec(transpose(B), Vx));
        const Qw = vecAdd(vecScale(matVec(this.G, w), -2), matVec(transpose(W), Vx));
        let Quu = add(scale(this.R, 2), multiply(multiply(transpose(B), Vxx), B));
        if (this.changePenalty) {
          const before = k === 0 ? (previousInput || new Array(inputSize).fill(0)) : inputs[k - 1];
          Qu = vecAdd(Qu, vecScale(matVec(this.changePenalty, vecSubtract(u, before)), 2));
          Quu = add(Quu, scale(this.changePenalty, 2));
        }
        Quu = positiveDefinite(Quu, this.regularization);
        const Qww = negativeDefinite(add(scale(this.G, -2),
          multiply(multiply(transpose(W), Vxx), W)), this.regularization);
        const Quw = multiply(multiply(transpose(B), Vxx), W);
        const Qux = multiply(multiply(transpose(B), Vxx), A);
        const Qwx = multiply(multiply(transpose(W), Vxx), A);
        const H = blockMatrix(Quu, Quw, transpose(Quw), Qww);
        const gradient = Qu.concat(Qw);
        const cross = Qux.map((row, i) => row.slice()).concat(Qwx.map(row => row.slice()));
        const feedforward = vecScale(solve(H, gradient), -1);
        const feedback = scale(solve(H, cross), -1);
        policies[k] = {
          ku: feedforward.slice(0, inputSize),
          kw: feedforward.slice(inputSize),
          Ku: feedback.slice(0, inputSize),
          Kw: feedback.slice(inputSize)
        };
        const Qxx = add(scale(this.Q, 2), multiply(multiply(transpose(A), Vxx), A));
        const HTimesFeedback = multiply(H, feedback);
        const crossT = transpose(cross);
        Vx = vecAdd(Qx, vecAdd(matVec(transpose(feedback), gradient), matVec(crossT, feedforward)));
        Vxx = symmetrize(add(Qxx, add(multiply(transpose(feedback), HTimesFeedback),
          add(multiply(transpose(feedback), cross), multiply(crossT, feedback)))));
      }

      const nextStates = [state.slice()];
      const nextInputs = [];
      const nextDisturbances = [];
      for (let k = 0; k < this.horizon; k++) {
        const deviation = vecSubtract(nextStates[k], states[k]);
        nextInputs[k] = projectActuators(vecAdd(inputs[k],
          vecAdd(policies[k].ku, matVec(policies[k].Ku, deviation))), this.constraints);
        nextDisturbances[k] = clampVector(vecAdd(disturbances[k],
          vecAdd(policies[k].kw, matVec(policies[k].Kw, deviation))),
        this.disturbanceBounds.map(value => -value), this.disturbanceBounds);
        nextStates.push(this.model.step(nextStates[k], nextInputs[k], nextDisturbances[k]));
      }
      inputs = nextInputs;
      disturbances = nextDisturbances;
      states = nextStates;
    }

    const cost = robustTrajectoryCost(states, inputs, disturbances, references,
      this.Q, this.R, this.G, this.terminalQ, this.changePenalty, previousInput);
    return controllerResult(this.mode, inputs[0], states, cost,
      { inputTrajectory: inputs, disturbanceTrajectory: disturbances, worstCaseDisturbance: disturbances[0] });
  }
}

/** Minimax variant with explicit delta-input/delta-disturbance penalties. */
export class InputPenaltyMinimaxController extends MinimaxIterativeLQRController {
  constructor(model, options = {}) {
    const inputSize = model.inputSize || model.B?.[0]?.length;
    const changePenalty = options.changePenalty || diagonal(new Array(inputSize).fill(1));
    const G = options.G || diagonal(new Array(model.stateSize).fill(1e-3));
    super(model, { ...options, G, changePenalty, mode: 'ilqr_minimax_penalty' });
  }
}

function denseLayer(input, weights, bias, activation = value => value) {
  return weights.map((row, neuron) => activation(dot(row, input) + (bias?.[neuron] || 0)));
}

/** Neural geometry interface from NN_RollingDirection.m. */
export class NeuralRollingDirectionController {
  constructor(model, options = {}) {
    this.model = model;
    this.network = options.network || null;
    this.constraints = options.constraints || model.constraints || {};
    this.cables = options.cables || model.cables || [];
    this.mode = 'neural';
  }

  geometryFeatures({ nodes, velocities, cableLengths, desiredDirection }) {
    const dimension = nodes[0]?.length || 3;
    const centroid = new Array(dimension).fill(0);
    for (const node of nodes) node.forEach((value, axis) => { centroid[axis] += value / nodes.length; });
    const angle = Math.atan2(desiredDirection[1], desiredDirection[0]);
    const cosine = Math.cos(-angle);
    const sine = Math.sin(-angle);
    const features = [];
    for (const node of nodes) {
      const x = node[0] - centroid[0];
      const y = node[1] - centroid[1];
      features.push(cosine * x - sine * y, sine * x + cosine * y, node[2] - centroid[2]);
    }
    // The shared MATLAB network used positions and cable state; velocities
    // remain optional because its trained net2 .mat weights are not present.
    features.push(...cableLengths);
    return features;
  }

  solve(context) {
    const features = this.geometryFeatures(context);
    let input;
    if (this.network) {
      let activations = features;
      for (let layer = 0; layer < this.network.weights.length; layer++) {
        const last = layer === this.network.weights.length - 1;
        activations = denseLayer(activations, this.network.weights[layer],
          this.network.biases?.[layer], last ? Math.tanh : Math.tanh);
      }
      input = activations;
    } else {
      // Deterministic geometry fallback when the external net2 weight file is
      // unavailable. It retains the rotated-node feature contract and cable
      // pairing safeguards, but does not claim the missing learned weights.
      const centroid = context.nodes.reduce((sum, node) => sum.map((value, axis) => value + node[axis] / context.nodes.length), [0, 0, 0]);
      input = this.cables.map(([i, j]) => {
        const midpoint = context.nodes[i].map((value, axis) => 0.5 * (value + context.nodes[j][axis]) - centroid[axis]);
        const forward = midpoint[0] * context.desiredDirection[0] + midpoint[1] * context.desiredDirection[1];
        const vertical = midpoint[2];
        return Math.tanh(2.2 * forward - 1.1 * vertical);
      });
    }
    input = projectActuators(input.slice(0, this.model.inputSize), this.constraints);
    return controllerResult(this.mode, input, [], NaN, { neuralFallback: !this.network, features });
  }
}

export function createDriveControllerCatalog(model, options = {}) {
  const shared = {
    horizon: options.horizon,
    constraints: options.constraints || model.constraints
  };
  return {
    lqr: new LQRRollingDirectionController(model, shared),
    lqr_payload: new CentralPayloadLQRController(model, { ...shared, payloadStateIndices: options.payloadStateIndices }),
    ilqr: new IterativeLQRController(model, { ...shared, iterations: options.ilqrIterations ?? 3 }),
    ilqr_minimax: new MinimaxIterativeLQRController(model, { ...shared, iterations: options.minimaxIterations ?? 1 }),
    ilqr_minimax_penalty: new InputPenaltyMinimaxController(model, { ...shared, iterations: options.minimaxIterations ?? 1 }),
    qp_mpc: new QPMPCController(model, { ...shared, iterations: options.qpIterations ?? 16 }),
    qp_mpc_payload: new CentralPayloadQPMPCController(model, { ...shared, iterations: options.qpIterations ?? 16,
      payloadStateIndices: options.payloadStateIndices }),
    neural: new NeuralRollingDirectionController(model, { ...shared, cables: options.cables || model.cables, network: options.network })
  };
}
