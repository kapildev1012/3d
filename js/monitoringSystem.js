/**
 * Real-time monitoring and export for the tensegrity solver.
 *
 * The monitor never changes dynamics. It evaluates raw solver state after a
 * physics step, keeps a display-rate history, and optionally preserves every
 * physics sample for lossless CSV/JSON export.
 */

const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
const distance3 = (a, b) => Math.hypot(a[0]-b[0], a[1]-b[1], a[2]-b[2]);
const meanPoint = points => {
  const result = [0, 0, 0];
  if (!points.length) return result;
  for (const point of points) {
    result[0] += point[0]; result[1] += point[1]; result[2] += point[2];
  }
  return result.map(value => value/points.length);
};

export const MONITORING_DEFAULTS = Object.freeze({
  enabled: true,
  rawLogging: true,
  maxRawSamples: 50_000,
  chartSamplePeriod: 0.05,
  chartWindowSeconds: 60,
  goalThreshold: 0.50,
  contactForceThreshold: 0.25,
  formationStableThreshold: 0.080,
  formationWarningThreshold: 0.180,
  formationCollapseThreshold: 0.350,
  rodLengthWarning: 0.005,
  penetrationWarning: 0.002,
  cableModerateStrainPercent: 10,
  cableHighStrainPercent: 25,
  cableMaximumStrainPercent: 40,
  cableOverloadForce: 720,
  slackForceThreshold: 0.25,
  maximumNodeSeparationRatio: 2.6,
  collapseRadiusRatio: 0.22
});

function normalizeQuaternion(quaternion) {
  const length = Math.hypot(...quaternion);
  return length > 1e-12 ? quaternion.map(value => value/length) : [1, 0, 0, 0];
}

function dominantEigenvector4(matrix) {
  const diagonalized = matrix.map(row => row.slice());
  const eigenvectors = new Array(4).fill(0).map((_, row) =>
    new Array(4).fill(0).map((__, column) => row === column ? 1 : 0));
  for (let sweep = 0; sweep < 24; sweep++) {
    let p = 0, q = 1, maximum = 0;
    for (let row = 0; row < 4; row++) {
      for (let column = row+1; column < 4; column++) {
        if (Math.abs(diagonalized[row][column]) > maximum) {
          maximum = Math.abs(diagonalized[row][column]); p = row; q = column;
        }
      }
    }
    if (maximum < 1e-14) break;
    const angle = 0.5*Math.atan2(2*diagonalized[p][q], diagonalized[q][q]-diagonalized[p][p]);
    const cosine = Math.cos(angle);
    const sine = Math.sin(angle);
    const app = diagonalized[p][p];
    const aqq = diagonalized[q][q];
    const apq = diagonalized[p][q];
    diagonalized[p][p] = cosine*cosine*app-2*sine*cosine*apq+sine*sine*aqq;
    diagonalized[q][q] = sine*sine*app+2*sine*cosine*apq+cosine*cosine*aqq;
    diagonalized[p][q] = diagonalized[q][p] = 0;
    for (let index = 0; index < 4; index++) {
      if (index === p || index === q) continue;
      const aip = diagonalized[index][p];
      const aiq = diagonalized[index][q];
      diagonalized[index][p] = diagonalized[p][index] = cosine*aip-sine*aiq;
      diagonalized[index][q] = diagonalized[q][index] = sine*aip+cosine*aiq;
    }
    for (let row = 0; row < 4; row++) {
      const vip = eigenvectors[row][p];
      const viq = eigenvectors[row][q];
      eigenvectors[row][p] = cosine*vip-sine*viq;
      eigenvectors[row][q] = sine*vip+cosine*viq;
    }
  }
  let bestColumn = 0;
  for (let column = 1; column < 4; column++) {
    if (diagonalized[column][column] > diagonalized[bestColumn][bestColumn]) bestColumn = column;
  }
  return normalizeQuaternion(eigenvectors.map(row => row[bestColumn]));
}

function rotateByQuaternion(vector, quaternion) {
  const [w, x, y, z] = quaternion;
  const [vx, vy, vz] = vector;
  const tx = 2*(y*vz-z*vy);
  const ty = 2*(z*vx-x*vz);
  const tz = 2*(x*vy-y*vx);
  return [
    vx+w*tx+(y*tz-z*ty),
    vy+w*ty+(z*tx-x*tz),
    vz+w*tz+(x*ty-y*tx)
  ];
}

/** Horn quaternion alignment: translation and rotation do not count as deformation. */
export function rigidAlignedFormationError(currentNodes, referenceNodes) {
  if (!currentNodes?.length || currentNodes.length !== referenceNodes?.length) return Infinity;
  const currentCenter = meanPoint(currentNodes);
  const referenceCenter = meanPoint(referenceNodes);
  const current = currentNodes.map(point => point.map((value, axis) => value-currentCenter[axis]));
  const reference = referenceNodes.map(point => point.map((value, axis) => value-referenceCenter[axis]));

  const covariance = new Array(3).fill(0).map(() => [0, 0, 0]);
  for (let node = 0; node < current.length; node++) {
    for (let row = 0; row < 3; row++) {
      for (let column = 0; column < 3; column++) {
        covariance[row][column] += current[node][row]*reference[node][column];
      }
    }
  }
  const [[sxx, sxy, sxz], [syx, syy, syz], [szx, szy, szz]] = covariance;
  const matrix = [
    [sxx+syy+szz, syz-szy, szx-sxz, sxy-syx],
    [syz-szy, sxx-syy-szz, sxy+syx, szx+sxz],
    [szx-sxz, sxy+syx, -sxx+syy-szz, syz+szy],
    [sxy-syx, szx+sxz, syz+szy, -sxx-syy+szz]
  ];
  const quaternion = dominantEigenvector4(matrix);
  let sumSquares = 0;
  for (let node = 0; node < current.length; node++) {
    const aligned = rotateByQuaternion(current[node], quaternion);
    const dx = aligned[0]-reference[node][0];
    const dy = aligned[1]-reference[node][1];
    const dz = aligned[2]-reference[node][2];
    sumSquares += dx*dx+dy*dy+dz*dz;
  }
  return Math.sqrt(sumSquares/current.length);
}

function closestSegmentDistance(a0, a1, b0, b1) {
  const u = a1.map((value, axis) => value-a0[axis]);
  const v = b1.map((value, axis) => value-b0[axis]);
  const w = a0.map((value, axis) => value-b0[axis]);
  const aa = u.reduce((sum, value) => sum+value*value, 0);
  const bb = u.reduce((sum, value, axis) => sum+value*v[axis], 0);
  const cc = v.reduce((sum, value) => sum+value*value, 0);
  const dd = u.reduce((sum, value, axis) => sum+value*w[axis], 0);
  const ee = v.reduce((sum, value, axis) => sum+value*w[axis], 0);
  const denominator = aa*cc-bb*bb;
  let s = denominator > 1e-12 ? clamp((bb*ee-cc*dd)/denominator, 0, 1) : 0;
  let t = cc > 1e-12 ? clamp((bb*s+ee)/cc, 0, 1) : 0;
  s = aa > 1e-12 ? clamp((bb*t-dd)/aa, 0, 1) : 0;
  const pa = a0.map((value, axis) => value+s*u[axis]);
  const pb = b0.map((value, axis) => value+t*v[axis]);
  return distance3(pa, pb);
}

export function buildCableTelemetry({ rover, q, forces, actuationOffsets, relaxedFlags, settings }) {
  return rover.outerStrings.map(([nodeA, nodeB], index) => {
    const baseRestLength = rover.l0_outerStrings[index];
    const commandedRestLength = clamp(
      baseRestLength-(actuationOffsets?.[index] || 0),
      0.5*baseRestLength,
      1.5*baseRestLength
    );
    const currentLength = distance3(q[nodeA], q[nodeB]);
    const deltaLength = currentLength-commandedRestLength;
    const strainPercent = 100*deltaLength/Math.max(commandedRestLength, 1e-9);
    const force = Math.max(0, forces?.[index] || 0);
    const slack = Boolean(relaxedFlags?.[index]) || force <= settings.slackForceThreshold || deltaLength <= 0;
    const overloaded = force >= settings.cableOverloadForce || strainPercent >= settings.cableMaximumStrainPercent;
    const state = slack ? 'slack'
      : overloaded ? 'overload'
        : strainPercent >= settings.cableHighStrainPercent ? 'high'
          : strainPercent >= settings.cableModerateStrainPercent ? 'moderate' : 'nominal';
    return {
      id: `C${String(index+1).padStart(2, '0')}`,
      index,
      nodeA,
      nodeB,
      restLength: commandedRestLength,
      nominalLength: baseRestLength,
      currentLength,
      deltaLength,
      strainPercent,
      force,
      slack,
      overloaded,
      state
    };
  });
}

function csvEscape(value) {
  if (value === null || value === undefined) return '';
  const text = typeof value === 'string' ? value : String(value);
  return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function toCsv(headers, rows) {
  return [headers.join(','), ...rows.map(row => headers.map(header => csvEscape(row[header])).join(','))].join('\n');
}

export class RealtimeMonitor {
  constructor(config, rover, terrain, initialNodes, initialCentroid) {
    this.config = config;
    this.rover = rover;
    this.terrain = terrain;
    this.settings = { ...MONITORING_DEFAULTS, ...(config.monitoring || {}) };
    this.referenceNodes = initialNodes.map(point => point.slice());
    this.start = initialCentroid.slice();
    const target = config.targetDestination || [0, config.targetGoalY || 25];
    const startGround = terrain.eval(initialCentroid[0], initialCentroid[1]).h;
    const goalGround = terrain.eval(target[0], target[1]).h;
    this.goal = [target[0], target[1], goalGround+(initialCentroid[2]-startGround)];
    this.records = [];
    this.history = {
      t: [], goalError: [], formationError: [], cableDelta: [], cableForce: [],
      contactCount: [], comX: [], comY: [], status: []
    };
    this.contactEvents = [];
    this.activeContactEvents = new Map();
    this.nextChartSample = 0;
    this.goalReached = false;
    this.goalResult = null;
    this.latest = null;
  }

  evaluateRodHealth(q) {
    let maximumLengthError = 0;
    const rodErrors = this.rover.bars.map(([nodeA, nodeB], index) => {
      const currentLength = distance3(q[nodeA], q[nodeB]);
      const error = Math.abs(currentLength-this.rover.l0_bars[index]);
      maximumLengthError = Math.max(maximumLengthError, error);
      return { id: `R${index+1}`, nodeA, nodeB, currentLength, restLength: this.rover.l0_bars[index], error };
    });
    let intersections = 0;
    for (let first = 0; first < this.rover.bars.length; first++) {
      for (let second = first+1; second < this.rover.bars.length; second++) {
        const [a0, a1] = this.rover.bars[first];
        const [b0, b1] = this.rover.bars[second];
        if (closestSegmentDistance(q[a0], q[a1], q[b0], q[b1]) < 0.055) intersections++;
      }
    }
    return { rodErrors, maximumLengthError, intersections };
  }

  updateContactEvents(time, contacts) {
    const activeKeys = new Set();
    for (const contact of contacts) {
      if ((contact.normalForce || 0) < this.settings.contactForceThreshold) continue;
      const key = `${contact.kind || 'node'}:${contact.id}`;
      activeKeys.add(key);
      if (!this.activeContactEvents.has(key)) {
        this.activeContactEvents.set(key, { ...contact, startTime: time, lastTime: time });
      } else {
        Object.assign(this.activeContactEvents.get(key), contact, { lastTime: time });
      }
    }
    for (const [key, event] of this.activeContactEvents) {
      if (activeKeys.has(key)) continue;
      this.contactEvents.push({ ...event, endTime: time, duration: Math.max(0, time-event.startTime) });
      this.activeContactEvents.delete(key);
    }
  }

  sample(state) {
    const { time, q, centroid, velocity, cableForces, actuationOffsets, relaxedFlags,
      contacts = [], constraintError = 0, terrainClearance = 0, distanceTraveled = 0 } = state;
    const goalError = distance3(centroid, this.goal);
    const remainingX = this.goal[0]-centroid[0];
    const remainingY = this.goal[1]-centroid[1];
    const formationError = rigidAlignedFormationError(q, this.referenceNodes);
    const cables = buildCableTelemetry({
      rover: this.rover, q, forces: cableForces, actuationOffsets, relaxedFlags, settings: this.settings
    });
    const rodHealth = this.evaluateRodHealth(q);
    const activeContacts = contacts.filter(contact => (contact.normalForce || 0) >= this.settings.contactForceThreshold);
    this.updateContactEvents(time, activeContacts);

    let maximumNodeSeparation = 0;
    const center = meanPoint(q);
    let meanRadius = 0;
    for (let first = 0; first < q.length; first++) {
      meanRadius += distance3(q[first], center);
      for (let second = first+1; second < q.length; second++) {
        maximumNodeSeparation = Math.max(maximumNodeSeparation, distance3(q[first], q[second]));
      }
    }
    meanRadius /= Math.max(1, q.length);
    const maximumCableForce = Math.max(0, ...cables.map(cable => cable.force));
    const averageCableForce = cables.reduce((sum, cable) => sum+cable.force, 0)/Math.max(1, cables.length);
    const maximumCableStrain = Math.max(0, ...cables.map(cable => cable.strainPercent));
    const slackCableCount = cables.filter(cable => cable.slack).length;
    const overloadedCableCount = cables.filter(cable => cable.overloaded).length;
    const warnings = [];
    if (rodHealth.maximumLengthError > this.settings.rodLengthWarning) warnings.push('ROD LENGTH');
    if (rodHealth.intersections > 0) warnings.push('ROD INTERSECTION');
    if (terrainClearance < -this.settings.penetrationWarning) warnings.push('TERRAIN PENETRATION');
    if (maximumCableStrain >= this.settings.cableMaximumStrainPercent) warnings.push('CABLE STRAIN');
    if (overloadedCableCount) warnings.push('CABLE OVERLOAD');
    if (maximumNodeSeparation > this.settings.maximumNodeSeparationRatio*2*this.rover.R_outer) warnings.push('NODE SEPARATION');
    const collapsed = !Number.isFinite(formationError) ||
      formationError >= this.settings.formationCollapseThreshold ||
      meanRadius < this.settings.collapseRadiusRatio*this.rover.R_outer ||
      rodHealth.intersections > 0;
    const deformed = !collapsed && (formationError >= this.settings.formationWarningThreshold || warnings.length > 0);
    if (!this.goalReached && goalError <= this.settings.goalThreshold) {
      this.goalReached = true;
      this.goalResult = { time, distanceTraveled, finalError: goalError };
    }
    const status = this.goalReached ? 'Goal Reached' : collapsed ? 'Collapsed' : deformed ? 'Deformed' : 'Stable';
    const stabilityLevel = collapsed || deformed ? 'red'
      : formationError >= this.settings.formationStableThreshold ? 'yellow' : 'green';
    const speed = Math.hypot(...velocity);
    const result = {
      time,
      start: this.start.slice(),
      goal: this.goal.slice(),
      com: centroid.slice(),
      goalError,
      remainingX,
      remainingY,
      goalThreshold: this.settings.goalThreshold,
      goalReached: this.goalReached,
      goalResult: this.goalResult ? { ...this.goalResult } : null,
      formationError,
      stabilityLevel,
      status,
      warnings,
      speed,
      cables,
      maximumCableForce,
      averageCableForce,
      maximumCableStrain,
      slackCableCount,
      overloadedCableCount,
      contacts: activeContacts.map(contact => ({ ...contact })),
      activeContactCount: activeContacts.length,
      grounded: activeContacts.length > 0,
      rodErrors: rodHealth.rodErrors,
      maximumRodError: rodHealth.maximumLengthError,
      rodIntersections: rodHealth.intersections,
      terrainClearance,
      maximumNodeSeparation,
      meanRadius
    };
    this.latest = result;

    const completeRecord = {
      ...result,
      nodes: q.map(point => point.slice()),
      cableLengths: cables.map(cable => cable.currentLength),
      cableDeltaLengths: cables.map(cable => cable.deltaLength),
      cableStrainPercentages: cables.map(cable => cable.strainPercent),
      cableForces: cables.map(cable => cable.force)
    };
    if (this.settings.rawLogging && this.records.length < this.settings.maxRawSamples) this.records.push(completeRecord);
    if (time+1e-12 >= this.nextChartSample) {
      this.history.t.push(time);
      this.history.goalError.push(goalError);
      this.history.formationError.push(formationError);
      this.history.cableDelta.push(cables.map(cable => cable.deltaLength));
      this.history.cableForce.push(cables.map(cable => cable.force));
      this.history.contactCount.push(activeContacts.length);
      this.history.comX.push(centroid[0]);
      this.history.comY.push(centroid[1]);
      this.history.status.push(status);
      this.nextChartSample = time+this.settings.chartSamplePeriod;
    }
    return result;
  }

  contactEventRecords() {
    const active = [...this.activeContactEvents.values()].map(event => ({
      ...event,
      endTime: this.latest?.time ?? event.lastTime,
      duration: Math.max(0, (this.latest?.time ?? event.lastTime)-event.startTime)
    }));
    return [...this.contactEvents, ...active];
  }

  exportCsv(kind) {
    if (kind === 'goal_tracking') {
      const headers = ['time', 'com_x', 'com_y', 'com_z', 'goal_x', 'goal_y', 'goal_z', 'goal_error', 'remaining_x', 'remaining_y', 'speed', 'status'];
      return toCsv(headers, this.records.map(record => ({
        time: record.time, com_x: record.com[0], com_y: record.com[1], com_z: record.com[2],
        goal_x: record.goal[0], goal_y: record.goal[1], goal_z: record.goal[2], goal_error: record.goalError,
        remaining_x: record.remainingX, remaining_y: record.remainingY, speed: record.speed, status: record.status
      })));
    }
    if (kind === 'formation_metrics') {
      const headers = ['time', 'formation_error', 'stability_level', 'status', 'max_rod_error', 'rod_intersections', 'terrain_clearance', 'max_node_separation', 'warnings'];
      return toCsv(headers, this.records.map(record => ({
        time: record.time, formation_error: record.formationError, stability_level: record.stabilityLevel,
        status: record.status, max_rod_error: record.maximumRodError, rod_intersections: record.rodIntersections,
        terrain_clearance: record.terrainClearance, max_node_separation: record.maximumNodeSeparation,
        warnings: record.warnings.join('|')
      })));
    }
    if (kind === 'cable_metrics') {
      const headers = ['time', 'cable_id', 'node_a', 'node_b', 'rest_length', 'current_length', 'delta_length', 'strain_percent', 'force_n', 'state'];
      const rows = this.records.flatMap(record => record.cables.map(cable => ({
        time: record.time, cable_id: cable.id, node_a: `N${cable.nodeA+1}`, node_b: `N${cable.nodeB+1}`,
        rest_length: cable.restLength, current_length: cable.currentLength, delta_length: cable.deltaLength,
        strain_percent: cable.strainPercent, force_n: cable.force, state: cable.state
      })));
      return toCsv(headers, rows);
    }
    if (kind === 'contact_events') {
      const headers = ['kind', 'id', 'object', 'start_time', 'end_time', 'duration', 'position', 'normal', 'normal_force', 'friction_force'];
      return toCsv(headers, this.contactEventRecords().map(event => ({
        kind: event.kind, id: event.id, object: event.objectId, start_time: event.startTime,
        end_time: event.endTime, duration: event.duration, position: JSON.stringify(event.position),
        normal: JSON.stringify(event.normal), normal_force: event.normalForce,
        friction_force: JSON.stringify(event.frictionForce)
      })));
    }
    const headers = ['time', 'com_position', 'goal_position', 'goal_error', 'formation_error', 'speed', 'node_positions', 'cable_lengths', 'cable_delta_lengths', 'cable_strain_percentages', 'cable_forces', 'active_contacts', 'contact_forces', 'status', 'warnings'];
    return toCsv(headers, this.records.map(record => ({
      time: record.time, com_position: JSON.stringify(record.com), goal_position: JSON.stringify(record.goal),
      goal_error: record.goalError, formation_error: record.formationError, speed: record.speed,
      node_positions: JSON.stringify(record.nodes), cable_lengths: JSON.stringify(record.cableLengths),
      cable_delta_lengths: JSON.stringify(record.cableDeltaLengths), cable_strain_percentages: JSON.stringify(record.cableStrainPercentages),
      cable_forces: JSON.stringify(record.cableForces), active_contacts: JSON.stringify(record.contacts.map(contact => contact.id)),
      contact_forces: JSON.stringify(record.contacts.map(contact => contact.normalForce)), status: record.status,
      warnings: record.warnings.join('|')
    })));
  }

  exportJson() {
    return JSON.stringify({
      settings: this.settings,
      start: this.start,
      goal: this.goal,
      goalResult: this.goalResult,
      contactEvents: this.contactEventRecords(),
      samples: this.records
    });
  }
}
