import {
  DRIVE_CONTROLLER_LABELS,
  createDriveControllerCatalog
} from './controllerAlgorithms.js';
import {
  diagonal,
  identity,
  matVec,
  trapezoidalDiscretize,
  vecAdd,
  zeros
} from './linearAlgebra.js';

const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value));

function average(values) {
  return values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length);
}

function buildReducedModel(simulation) {
  const { rover, cfg } = simulation;
  const cableCount = rover.outerStrings.length;
  const rodCount = rover.bars.length;
  const inputSize = cableCount + rodCount;
  const cableOffsetStart = 8;
  const rodOffsetStart = cableOffsetStart + cableCount;
  const stateSize = rodOffsetStart + rodCount;
  const Ac = zeros(stateSize);
  const Bc = zeros(stateSize, inputSize);

  // State: centroid xyz, velocity xyz, payload z/vz, cable offsets, rod offsets.
  Ac[0][3] = 1;
  Ac[1][4] = 1;
  Ac[2][5] = 1;
  Ac[6][7] = 1;
  for (let axis = 3; axis <= 5; axis++) Ac[axis][axis] = -Math.max(0.05, cfg.damping || 0.3);
  Ac[7][7] = -0.8;
  const localCentroid = [0, 0, 0];
  for (const node of rover.q0_outer) node.forEach((value, axis) => { localCentroid[axis] += value / rover.nOuter; });

  rover.outerStrings.forEach(([i, j], cable) => {
    const midpoint = rover.q0_outer[i].map((value, axis) =>
      0.5 * (value + rover.q0_outer[j][axis]) - localCentroid[axis]);
    const radial = Math.max(1e-6, Math.hypot(...midpoint));
    // Contracting high/front cables creates a rolling moment through contact.
    const influence = 2.2 / Math.max(0.1, rover.R_outer);
    Bc[3][cable] = influence * midpoint[0] * (0.35 + Math.abs(midpoint[2]) / radial);
    Bc[4][cable] = influence * midpoint[1] * (0.35 + Math.abs(midpoint[2]) / radial);
    Bc[5][cable] = -0.30 * influence * midpoint[2];
    Bc[7][cable] = -0.18 * influence * midpoint[2];
    Bc[cableOffsetStart + cable][cable] = 1;
    Ac[cableOffsetStart + cable][cableOffsetStart + cable] = -1 / Math.max(0.05, cfg.actuatorTau || 0.2);
  });

  rover.bars.forEach(([i, j], rod) => {
    const column = cableCount + rod;
    // Rod actuation is represented but strongly penalized, matching the source.
    const vector = rover.q0_outer[j].map((value, axis) => value - rover.q0_outer[i][axis]);
    Bc[3][column] = 0.04 * vector[0];
    Bc[4][column] = 0.04 * vector[1];
    Bc[5][column] = 0.04 * vector[2];
    Bc[rodOffsetStart + rod][column] = 1;
  });

  const discrete = trapezoidalDiscretize(Ac, Bc, Math.max(0.01, cfg.controllerDt || 0.05));
  const W = identity(stateSize).map(row => row.map(value => value * Math.max(0.01, cfg.controllerDt || 0.05)));
  const qWeights = [2, 2, 8, 8, 8, 4, 4, 2]
    .concat(new Array(cableCount).fill(0.5), new Array(rodCount).fill(1));
  const rWeights = new Array(cableCount).fill(0.05).concat(new Array(rodCount).fill(1e6));
  const cableLimit = cfg.cableLinearVelocity || 0.1;
  const rodLimit = cfg.rodLinearVelocity || 0.15;
  const minimum = new Array(cableCount).fill(-cableLimit).concat(new Array(rodCount).fill(-rodLimit));
  const maximum = minimum.map(value => -value);
  const passive = [];
  if ((cfg.actuatorMode || 'cables') === 'cables') {
    for (let rod = 0; rod < rodCount; rod++) passive.push(cableCount + rod);
  } else if (cfg.actuatorMode === 'rods') {
    for (let cable = 0; cable < cableCount; cable++) passive.push(cable);
  }

  const model = {
    ...discrete,
    Ac,
    Bc,
    W,
    dt: Math.max(0.01, cfg.controllerDt || 0.05),
    stateSize,
    inputSize,
    disturbanceSize: stateSize,
    Q: diagonal(qWeights),
    R: diagonal(rWeights),
    constraints: { minimum, maximum, passive, paired: [], similar: [] },
    disturbanceBounds: new Array(stateSize).fill(cfg.disturbanceBound || 0.02),
    payloadStateIndices: [2, 6, 7],
    cables: rover.outerStrings,
    layout: { cableCount, rodCount, cableOffsetStart, rodOffsetStart },
    step(state, input, disturbance = null) {
      let next = vecAdd(matVec(this.A, state), matVec(this.B, input));
      if (disturbance) next = vecAdd(next, matVec(this.W, disturbance));
      return next;
    }
  };
  return model;
}

function stateFromSimulation(simulation, centroid, velocity) {
  const { model } = simulation.__driveReferenceAdapter;
  const coreZ = simulation.corePosition?.[2] ?? centroid[2];
  const coreVz = simulation.coreVelocity?.[2] ?? velocity[2];
  return centroid.concat(velocity, [coreZ, coreVz],
    simulation.currentActuationOffsets.slice(), simulation.currentRodOffsets.slice());
}

function referenceFromSimulation(simulation, centroid, desiredDirection) {
  const cfg = simulation.cfg;
  const mode = cfg.actuationMode;
  const destination = mode === 'roll_backward'
    ? [centroid[0], centroid[1] - 8]
    : mode === 'steer_left'
      ? [centroid[0] - 6, centroid[1] + 6]
      : mode === 'steer_right'
        ? [centroid[0] + 6, centroid[1] + 6]
        : (cfg.targetDestination || [0, cfg.targetGoalY || 25]);
  const desiredSpeed = mode === 'none' ? 0 : (cfg.targetSpeed || 0.2);
  const payloadHeight = cfg.payloadTargetHeight || centroid[2];
  const { cableCount, rodCount } = simulation.__driveReferenceAdapter.model.layout;
  return [destination[0], destination[1], payloadHeight,
    desiredDirection[0] * desiredSpeed, desiredDirection[1] * desiredSpeed, 0,
    payloadHeight, 0,
    ...new Array(cableCount + rodCount).fill(0)];
}

function payloadTargets(simulation, desiredDirection, controllerMode, intensity) {
  const targets = new Array(simulation.rover.nOuter).fill(0);
  if (!controllerMode.endsWith('_payload')) return targets;
  const centroid = simulation.q.reduce((sum, node) => sum.map((value, axis) => value + node[axis] / simulation.q.length), [0, 0, 0]);
  const projected = simulation.q.map((node, index) => ({
    index,
    value: (node[0] - centroid[0]) * desiredDirection[0] + (node[1] - centroid[1]) * desiredDirection[1]
  })).sort((left, right) => right.value - left.value);
  const stroke = clamp(intensity, 0, 1) * (simulation.cfg.coreActuationDeltaL || 0.065);
  for (const item of projected.slice(0, 4)) targets[item.index] = stroke;
  for (const item of projected.slice(-2)) targets[item.index] = -0.35 * stroke;
  return targets;
}

export class SimulationControllerAdapter {
  constructor(simulation) {
    this.simulation = simulation;
    this.model = buildReducedModel(simulation);
    this.catalog = createDriveControllerCatalog(this.model, {
      // Dense browser-side saddle/QP solves stay responsive at 20 Hz while
      // retaining the finite receding-horizon structure of the MATLAB code.
      horizon: Math.max(2, Math.min(6, simulation.cfg.controlHorizon || 12)),
      payloadStateIndices: this.model.payloadStateIndices,
      cables: simulation.rover.outerStrings,
      ilqrIterations: 2,
      minimaxIterations: 1,
      qpIterations: 12
    });
    this.previousInput = new Array(this.model.inputSize).fill(0);
    this.warmStarts = new Map();
  }

  solve(controllerMode, centroid, velocity, obstacle) {
    const simulation = this.simulation;
    const controller = this.catalog[controllerMode];
    if (!controller) throw new Error(`Unknown Drive controller: ${controllerMode}`);
    const desiredDirection = simulation.desiredDirectionForMode(centroid, obstacle);
    const state = stateFromSimulation(simulation, centroid, velocity);
    const reference = referenceFromSimulation(simulation, centroid, desiredDirection);
    let result;
    if (controllerMode === 'neural') {
      result = controller.solve({
        nodes: simulation.q,
        velocities: simulation.v,
        cableLengths: simulation.rover.outerStrings.map(([i, j]) => Math.hypot(
          simulation.q[i][0] - simulation.q[j][0],
          simulation.q[i][1] - simulation.q[j][1],
          simulation.q[i][2] - simulation.q[j][2]
        )),
        desiredDirection
      });
      // Neural output is normalized; convert it to legal spool speed.
      result.input = result.input.map((value, i) => value * (this.model.constraints.maximum[i] || 0));
    } else {
      result = controller.solve({
        state,
        reference,
        previousInput: this.previousInput,
        warmStart: this.warmStarts.get(controllerMode)
      });
    }

    const { cableCount, rodCount } = this.model.layout;
    const dt = this.model.dt;
    const cableTargets = simulation.currentActuationOffsets.map((offset, cable) => {
      const base = simulation.rover.l0_outerStrings[cable];
      const minimumOffset = base - base * simulation.cfg.cableMaxRatio;
      const maximumOffset = base - base * simulation.cfg.cableMinRatio;
      return clamp(offset + (result.input[cable] || 0) * dt, minimumOffset, maximumOffset);
    });
    const rodTargets = simulation.currentRodOffsets.map((offset, rod) => {
      const base = simulation.rover.l0_bars[rod];
      return clamp(offset + (result.input[cableCount + rod] || 0) * dt,
        base * (simulation.cfg.rodMinRatio - 1), base * (simulation.cfg.rodMaxRatio - 1));
    });
    this.previousInput = result.input.slice();
    if (result.inputTrajectory) this.warmStarts.set(controllerMode,
      result.inputTrajectory.slice(1).concat([result.inputTrajectory.at(-1)]));
    const effort = average(result.input.slice(0, cableCount).map(value => Math.abs(value))) /
      Math.max(1e-9, simulation.cfg.cableLinearVelocity || 0.1);
    const predictedPath = (result.predictedStates || []).filter((_, index) => index % 2 === 0)
      .map(predicted => predicted.slice(0, 3));
    const disturbance = result.worstCaseDisturbance || [];
    return {
      cableTargets,
      rodTargets,
      coreTargets: payloadTargets(simulation, desiredDirection, controllerMode, effort),
      diagnostics: {
        mode: controllerMode,
        modeLabel: DRIVE_CONTROLLER_LABELS[controllerMode],
        desiredDirection,
        predictedPath,
        controlCost: Number.isFinite(result.cost) ? result.cost : 0,
        activeCableCount: result.input.slice(0, cableCount).filter(value => Math.abs(value) > 1e-5).length,
        activeRodCount: result.input.slice(cableCount, cableCount + rodCount).filter(value => Math.abs(value) > 1e-5).length,
        disturbanceEstimate: disturbance.length ? Math.hypot(...disturbance) : 0,
        horizon: this.catalog[controllerMode].horizon || simulation.cfg.controlHorizon,
        neuralFallback: Boolean(result.neuralFallback)
      }
    };
  }
}

export function attachDriveControllerSuite(simulation) {
  if (!simulation.__driveReferenceAdapter) simulation.__driveReferenceAdapter = new SimulationControllerAdapter(simulation);
  return simulation.__driveReferenceAdapter;
}
