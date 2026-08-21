import {
  add,
  diagonal,
  dot,
  identity,
  matVec,
  multiply,
  norm,
  outer,
  scale,
  solve,
  subtract,
  symmetricEigenvalues,
  transpose,
  vecAdd,
  vecScale,
  zeros
} from './linearAlgebra.js';

// Direct port of the six_bar_model.m node order and connectivity.
export const SIX_BAR_SOURCE_NODES = Object.freeze([
  [0.1415, -0.1884, -0.3085], [-0.3785, -0.0459, 0.0728],
  [0.3785, 0.0459, -0.0728], [-0.1415, 0.1884, 0.3085],
  [-0.2290, 0.3049, -0.0728], [-0.0924, -0.2168, 0.3085],
  [0.0924, 0.2168, -0.3085], [0.2290, -0.3049, 0.0728],
  [0.2339, 0.0283, 0.3085], [-0.1495, -0.3507, -0.0728],
  [0.1495, 0.3507, 0.0728], [-0.2339, -0.0283, -0.3085]
]);

export const SIX_BAR_RODS = Object.freeze([[0, 1], [2, 3], [4, 5], [6, 7], [8, 9], [10, 11]]);

export const SIX_BAR_CABLES = Object.freeze([
  [1, 5], [1, 4], [1, 9], [1, 11], [0, 6], [0, 7], [0, 9], [0, 11],
  [2, 6], [2, 7], [2, 10], [2, 8], [3, 5], [3, 4], [3, 10], [3, 8],
  [5, 9], [5, 8], [4, 11], [4, 10], [6, 11], [6, 10], [7, 9], [7, 8]
]);

export const PAPER_PARAMETERS = Object.freeze({
  dt: 0.05,
  mass: 1,
  damping: 1.5,
  navigation: [0, 3],
  maximumInput: 10,
  measurementNoiseSigma: 0.5,
  stringLength: 15,
  stringGain: 0.0341,
  stringExponent: 2,
  barGain: -50,
  barExponent: -0.5,
  relaxationStart: 15.5,
  relaxationEnd: 50,
  relaxedForce: 8,
  avoidanceRadius: 8,
  avoidanceGain: 20,
  avoidanceExponent: 0.4
});

export function flattenNodes(nodes) {
  return nodes.flatMap(node => node);
}

export function unflattenNodes(vector, dimension = 3) {
  const nodes = [];
  for (let i = 0; i < vector.length; i += dimension) nodes.push(vector.slice(i, i + dimension));
  return nodes;
}

export function connectivityMatrix(edges, nodeCount) {
  return edges.map(([start, end]) => {
    const row = new Array(nodeCount).fill(0);
    row[start] = -1;
    row[end] = 1;
    return row;
  });
}

export function memberVector(nodes, edge) {
  const [start, end] = edge;
  return nodes[end].map((value, axis) => value - nodes[start][axis]);
}

export function memberLength(nodes, edge) {
  return norm(memberVector(nodes, edge));
}

export function memberCoordinateDifferences(nodes, edges) {
  const dimension = nodes[0]?.length || 0;
  const byAxis = Array.from({ length: dimension }, () => []);
  for (const edge of edges) {
    const difference = memberVector(nodes, edge);
    difference.forEach((value, axis) => byAxis[axis].push(value));
  }
  return byAxis;
}

/** Weighted graph Laplacian / force-density matrix from paper Eq. (4). */
export function forceDensityMatrix(nodeCount, edges, edgeForces, nodes) {
  const matrix = zeros(nodeCount);
  edges.forEach(([i, j], edge) => {
    const length = Math.max(1e-9, memberLength(nodes, [i, j]));
    const density = edgeForces[edge] / length;
    matrix[i][i] += density;
    matrix[j][j] += density;
    matrix[i][j] -= density;
    matrix[j][i] -= density;
  });
  return matrix;
}

/** Standard rigidity/geometry matrix; one row per member. */
export function geometryMatrix(nodes, edges) {
  const dimension = nodes[0]?.length || 0;
  return edges.map(([i, j]) => {
    const row = new Array(nodes.length * dimension).fill(0);
    const difference = nodes[i].map((value, axis) => value - nodes[j][axis]);
    for (let axis = 0; axis < dimension; axis++) {
      row[i * dimension + axis] = difference[axis];
      row[j * dimension + axis] = -difference[axis];
    }
    return row;
  });
}

/** Position/velocity-level constraints and Jacobians from RodConstraints.m. */
export function rodConstraints(nodes, velocities, rods, targetLengths) {
  const dimension = nodes[0]?.length || 0;
  const G = [];
  const Gdot = [];
  const J = zeros(rods.length, nodes.length * dimension);
  const Jdot = zeros(rods.length, nodes.length * dimension);
  rods.forEach(([i, j], rod) => {
    const displacement = nodes[i].map((value, axis) => value - nodes[j][axis]);
    const relativeVelocity = velocities[i].map((value, axis) => value - velocities[j][axis]);
    G.push(dot(displacement, displacement) - targetLengths[rod] ** 2);
    Gdot.push(2 * dot(displacement, relativeVelocity));
    for (let axis = 0; axis < dimension; axis++) {
      J[rod][i * dimension + axis] = 2 * displacement[axis];
      J[rod][j * dimension + axis] = -2 * displacement[axis];
      Jdot[rod][i * dimension + axis] = 2 * relativeVelocity[axis];
      Jdot[rod][j * dimension + axis] = -2 * relativeVelocity[axis];
    }
  });
  return { G, Gdot, J, Jdot };
}

export function smoothMaximum(value, beta = 1e-3) {
  return (Math.sqrt(value * value + beta * beta) + value) / 2;
}

export function smoothStep(value, beta = 1e-3) {
  return 0.5 * (value / Math.sqrt(value * value + beta * beta) + 1);
}

/** Tension-only cable forces used by Dynamics_Generator.m. */
export function cableForces(nodes, cables, restLengths, stiffness, beta = 1e-3) {
  const forces = nodes.map(node => new Array(node.length).fill(0));
  const tensions = [];
  cables.forEach(([i, j], cable) => {
    const displacement = nodes[j].map((value, axis) => value - nodes[i][axis]);
    const length = Math.max(1e-9, norm(displacement));
    const extension = length - restLengths[cable];
    const k = Array.isArray(stiffness) ? stiffness[cable] : stiffness;
    const tension = k * smoothMaximum(extension, beta);
    const unit = displacement.map(value => value / length);
    tensions.push(tension);
    for (let axis = 0; axis < unit.length; axis++) {
      forces[i][axis] += tension * unit[axis];
      forces[j][axis] -= tension * unit[axis];
    }
  });
  return { forces, tensions };
}

export function gravityForces(masses, gravity = 9.81, dimension = 3) {
  return masses.map(mass => {
    const force = new Array(dimension).fill(0);
    force[dimension - 1] = -mass * gravity;
    return force;
  });
}

export function generalDampingForces(velocities, damping = 0.5) {
  return velocities.map(velocity => velocity.map(value => -damping * value));
}

/** Smooth compliant floor from FloorForceVertical_XIncline.m. */
export function inclineFloorForces(nodes, velocities, options = {}) {
  const {
    baseFloor = 0,
    stiffness = 5e4,
    damping = 0.1,
    beta = 1e-3,
    inclineDegrees = 0
  } = options;
  const tangent = Math.tan(inclineDegrees * Math.PI / 180);
  return nodes.map((node, i) => {
    const force = new Array(node.length).fill(0);
    const penetration = baseFloor + tangent * node[0] - node[node.length - 1];
    force[node.length - 1] = smoothMaximum(penetration * stiffness, beta) +
      smoothStep(penetration, beta) * (-velocities[i][node.length - 1]) * damping;
    return force;
  });
}

/** Contact-gated XY damping from FloorForceHorizontal_ViscousFriction.m. */
export function viscousFloorFriction(nodes, velocities, options = {}) {
  const { baseFloor = 0, damping = 0.5, beta = 1e-3 } = options;
  return nodes.map((node, i) => {
    const penetration = baseFloor - node[node.length - 1];
    const contact = smoothStep(penetration, beta);
    const force = new Array(node.length).fill(0);
    for (let axis = 0; axis < Math.min(2, node.length - 1); axis++) {
      force[axis] = -contact * damping * velocities[i][axis];
    }
    return force;
  });
}

export function sumNodalForces(...forceSets) {
  if (!forceSets.length) return [];
  return forceSets[0].map((node, i) => node.map((_, axis) =>
    forceSets.reduce((sum, forces) => sum + forces[i][axis], 0)));
}

/**
 * Lagrange-multiplier acceleration projection from Dynamics_Generator.m.
 * Baumgarte terms keep position and velocity constraints from drifting.
 */
export function constrainedAcceleration(forces, masses, constraints, options = {}) {
  const { positionGain = 1e-4, velocityGain = 1e-4 } = options;
  const flatForces = flattenNodes(forces);
  const inverseMasses = masses.flatMap(mass => new Array(forces[0].length).fill(1 / mass));
  const W = diagonal(inverseMasses);
  const { G, Gdot, J, Jdot } = constraints;
  if (!J.length) return unflattenNodes(matVec(W, flatForces), forces[0].length);
  const JW = multiply(J, W);
  const constraintMass = multiply(JW, transpose(J));
  const curvature = matVec(Jdot, constraints.flatVelocity || new Array(flatForces.length).fill(0));
  const bias = curvature.map((value, i) => value - positionGain * G[i] - velocityGain * Gdot[i]);
  const projectedForceTerm = matVec(JW, flatForces);
  const multipliers = solve(constraintMass, vecAdd(bias, projectedForceTerm), 1e-8);
  const constraintReaction = matVec(transpose(J), multipliers);
  const acceleration = matVec(W, flatForces.map((force, i) => force - constraintReaction[i]));
  return unflattenNodes(acceleration, forces[0].length);
}

export function tensegrityAcceleration(state, model, options = {}) {
  const nodes = state.nodes;
  const velocities = state.velocities;
  const cables = cableForces(nodes, model.cables, state.cableRestLengths, model.cableStiffness, options.beta);
  const constraints = rodConstraints(nodes, velocities, model.rods, state.rodLengths);
  constraints.flatVelocity = flattenNodes(velocities);
  const total = sumNodalForces(
    cables.forces,
    gravityForces(model.masses, options.gravity ?? 9.81, nodes[0].length),
    generalDampingForces(velocities, options.damping ?? 0.5),
    inclineFloorForces(nodes, velocities, options.floor),
    viscousFloorFriction(nodes, velocities, options.floor)
  );
  return {
    acceleration: constrainedAcceleration(total, model.masses, constraints, options.constraint),
    constraints,
    cableTensions: cables.tensions,
    totalForces: total
  };
}

/** Nonlinear virtual-member law, paper Eq. (10). */
export function powerLawEdgeForce(length, gain, exponent) {
  return gain * Math.max(length, 1e-9) ** exponent;
}

/** Adaptive Hermite string relaxation, paper Eq. (19)-(20). */
export function relaxingStringForce(length, options = PAPER_PARAMETERS) {
  const gain = options.stringGain ?? options.gain;
  const exponent = options.stringExponent ?? options.exponent;
  const z1 = options.relaxationStart ?? options.z1;
  const z2 = options.relaxationEnd ?? options.z2;
  const beta = options.relaxedForce ?? options.beta;
  if (length <= z1) return powerLawEdgeForce(length, gain, exponent);
  if (length >= z2) return beta;
  const s = (length - z1) / (z2 - z1);
  const h00 = 2 * s ** 3 - 3 * s ** 2 + 1;
  const h10 = s ** 3 - 2 * s ** 2 + s;
  const h01 = -2 * s ** 3 + 3 * s ** 2;
  return h00 * gain * z1 ** exponent +
    h10 * (z2 - z1) * gain * z1 ** (exponent - 1) * exponent +
    h01 * beta;
}

/** Edge potential psi(z)=integral_0^z f(s)ds from paper Eq. (12)-(13). */
export function powerLawPotential(length, gain, exponent) {
  if (Math.abs(exponent + 1) < 1e-9) return gain * Math.log(Math.max(length, 1e-9));
  return gain * length ** (exponent + 1) / (exponent + 1);
}

/** Collision/obstacle response from Eq. (17)-(18), with repulsive default. */
export function collisionAvoidanceForce(agent, obstaclePoint, options = PAPER_PARAMETERS) {
  const radius = options.avoidanceRadius ?? options.radius;
  const gain = options.avoidanceGain ?? options.gain;
  const exponent = options.avoidanceExponent ?? options.exponent;
  const displacement = agent.map((value, axis) => value - obstaclePoint[axis]);
  const distance = Math.max(1e-6, norm(displacement));
  if (distance > radius) return new Array(agent.length).fill(0);
  const magnitude = gain * (distance ** (-exponent) - radius ** (-exponent));
  // The PDF prints a leading minus although p points away from the obstacle.
  // main.m corrects this to a repulsive sign; paperSign reproduces the print.
  const sign = options.paperSign ? -1 : 1;
  return displacement.map(value => sign * magnitude * value / distance);
}

export function saturateVector(vector, maximumMagnitude) {
  const magnitude = norm(vector);
  return magnitude > maximumMagnitude ? vecScale(vector, maximumMagnitude / magnitude) : vector.slice();
}

/** Lemma-2 sign test plus the FDM spectrum used by the paper. */
export function structuralStabilityReport(nodes, edges, gains, exponents) {
  const memberForces = edges.map((edge, i) => powerLawEdgeForce(memberLength(nodes, edge), gains[i], exponents[i]));
  const D = forceDensityMatrix(nodes.length, edges, memberForces, nodes);
  const eigenvalues = symmetricEigenvalues(D);
  const memberConditions = gains.map((gain, i) => gain * (exponents[i] - 1));
  return {
    stableByMemberSigns: memberConditions.every(value => value >= -1e-9),
    forceDensityPositiveSemidefinite: eigenvalues[0] >= -1e-8,
    memberConditions,
    forceDensityMatrix: D,
    eigenvalues
  };
}

export function selfEquilibriumResidual(nodes, edges, edgeForces) {
  const D = forceDensityMatrix(nodes.length, edges, edgeForces, nodes);
  const coordinates = transpose(nodes);
  return coordinates.map(axis => matVec(D, axis));
}

/** Semi-implicit paper dynamics m*qddot=-c*qdot+sat(u), Eq. (9). */
export function stepPaperAgents(nodes, velocities, controls, dt, options = PAPER_PARAMETERS) {
  const mass = options.mass;
  const damping = options.damping;
  const maximum = options.maximumInput;
  const nextVelocities = velocities.map((velocity, i) => {
    const input = saturateVector(controls[i], maximum);
    return velocity.map((value, axis) => value + dt * (input[axis] - damping * value) / mass);
  });
  const nextNodes = nodes.map((node, i) => node.map((value, axis) => value + dt * nextVelocities[i][axis]));
  return { nodes: nextNodes, velocities: nextVelocities };
}
