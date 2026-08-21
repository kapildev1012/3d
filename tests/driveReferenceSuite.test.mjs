import test from 'node:test';
import assert from 'node:assert/strict';

import {
  DRIVE_CONTROLLER_LABELS,
  createDriveControllerCatalog
} from '../js/reference/controllerAlgorithms.js';
import {
  SIX_BAR_CABLES,
  SIX_BAR_RODS,
  SIX_BAR_SOURCE_NODES,
  cableForces,
  collisionAvoidanceForce,
  connectivityMatrix,
  memberLength,
  relaxingStringForce,
  rodConstraints
} from '../js/reference/tensegrityMath.js';
import {
  diagonal,
  identity,
  matVec,
  trapezoidalDiscretize,
  vecAdd,
  zeros
} from '../js/reference/linearAlgebra.js';
import { SimConfig, SphericalRoverModel, Simulation, TerrainModel } from '../js/simEngine.js';
import { attachDriveControllerSuite } from '../js/reference/simulationControllerAdapter.js';

test('Drive six-bar topology and rigid-rod constraints are preserved', () => {
  assert.equal(SIX_BAR_SOURCE_NODES.length, 12);
  assert.equal(SIX_BAR_RODS.length, 6);
  assert.equal(SIX_BAR_CABLES.length, 24);
  const connectivity = connectivityMatrix(SIX_BAR_CABLES, 12);
  assert.equal(connectivity.length, 24);
  assert.ok(connectivity.every(row => row.reduce((sum, value) => sum + value, 0) === 0));
  const lengths = SIX_BAR_RODS.map(edge => memberLength(SIX_BAR_SOURCE_NODES, edge));
  const velocities = SIX_BAR_SOURCE_NODES.map(() => [0, 0, 0]);
  const constraints = rodConstraints(SIX_BAR_SOURCE_NODES, velocities, SIX_BAR_RODS, lengths);
  assert.ok(constraints.G.every(value => Math.abs(value) < 1e-12));
  assert.ok(constraints.Gdot.every(value => Math.abs(value) < 1e-12));
  assert.deepEqual([constraints.J.length, constraints.J[0].length], [6, 36]);
});

test('Drive cable, adaptive relaxation, and avoidance force laws are active', () => {
  const nodes = [[0, 0, 0], [2, 0, 0]];
  const result = cableForces(nodes, [[0, 1]], [1], 100, 1e-6);
  assert.ok(result.tensions[0] > 99.99);
  assert.ok(result.forces[0][0] > 0 && result.forces[1][0] < 0);

  const before = relaxingStringForce(15.5);
  const after = relaxingStringForce(50);
  assert.ok(Math.abs(before - 0.0341 * 15.5 ** 2) < 1e-9);
  assert.equal(after, 8);
  assert.ok(Number.isFinite(relaxingStringForce(30)));

  const repulsion = collisionAvoidanceForce([2, 0], [0, 0], {
    radius: 8, gain: 20, exponent: 0.4
  });
  assert.ok(repulsion[0] > 0, 'force must point away from obstacle');
});

function makeControlModel() {
  const Ac = zeros(4);
  const Bc = zeros(4, 2);
  Ac[0][2] = 1;
  Ac[1][3] = 1;
  Ac[2][2] = -0.25;
  Ac[3][3] = -0.25;
  Bc[2][0] = 1;
  Bc[3][1] = 1;
  const { A, B } = trapezoidalDiscretize(Ac, Bc, 0.1);
  const W = identity(4).map(row => row.map(value => 0.1 * value));
  return {
    A, B, W, stateSize: 4, inputSize: 2, disturbanceSize: 4, dt: 0.1,
    Q: diagonal([4, 4, 1, 1]),
    R: diagonal([0.2, 0.2]),
    constraints: { minimum: [-0.5, -0.5], maximum: [0.5, 0.5] },
    disturbanceBounds: [0.02, 0.02, 0.02, 0.02],
    payloadStateIndices: [1, 3],
    cables: [[0, 1], [2, 3]],
    step(state, input, disturbance = null) {
      let next = vecAdd(matVec(A, state), matVec(B, input));
      if (disturbance) next = vecAdd(next, matVec(W, disturbance));
      return next;
    }
  };
}

test('all eight Drive controller ports produce bounded finite commands', () => {
  const model = makeControlModel();
  const catalog = createDriveControllerCatalog(model, {
    horizon: 4, ilqrIterations: 2, minimaxIterations: 1, qpIterations: 10,
    payloadStateIndices: [1, 3], cables: model.cables
  });
  assert.deepEqual(Object.keys(catalog).sort(), Object.keys(DRIVE_CONTROLLER_LABELS).sort());
  const state = [0, 0, 0, 0];
  const reference = [1, 1, 0, 0];
  for (const [mode, controller] of Object.entries(catalog)) {
    const result = mode === 'neural'
      ? controller.solve({
        nodes: [[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0]],
        velocities: new Array(4).fill(0).map(() => [0, 0, 0]),
        cableLengths: [2, 2], desiredDirection: [1, 0, 0]
      })
      : controller.solve({ state, reference, previousInput: [0, 0] });
    assert.equal(result.input.length, 2, mode);
    assert.ok(result.input.every(value => Number.isFinite(value)), mode);
    assert.ok(result.input.every(value => Math.abs(value) <= 0.5000001), mode);
  }
});

test('additive adapter drives the existing Simulation without modifying it', () => {
  const cfg = new SimConfig({
    controllerMode: 'lqr', controlHorizon: 3, controllerDt: 0.05,
    targetDestination: [0, 3]
  });
  const rover = new SphericalRoverModel(cfg);
  const simulation = new Simulation(cfg, rover, new TerrainModel(cfg), 'adaptive');
  const adapter = attachDriveControllerSuite(simulation);
  const result = adapter.solve('lqr', [0, 0, 0.55], [0, 0, 0], { detected: false });
  assert.equal(result.cableTargets.length, 24);
  assert.equal(result.rodTargets.length, 6);
  assert.equal(result.coreTargets.length, 12);
  assert.ok(result.cableTargets.every(Number.isFinite));
  assert.equal(result.diagnostics.mode, 'lqr');
});
