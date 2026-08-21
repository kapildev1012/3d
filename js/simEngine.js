import { relaxedCableTension } from './driveControllers.js?v=20260821-forward1';
import {
  createABCourse,
  evaluateCourseObstacle,
  senseCourseObstacle,
  obstacleActuation,
  ObstaclePassTracker
} from './abExperiment.js?v=20260820-ab1';

/**
 * SIMENGINE.JS - Physics & Locomotion Simulation Engine for 6-Bar Tensegrity Icosahedron Rover
 *
 * 1. Verified 6-bar / 24-cable expanded-icosahedron topology
 * 2. Fixed-step semi-implicit integration with rigid-strut SHAKE projection
 * 3. Tension-only spring/damper cables with rate-limited rest lengths
 * 4. 20 Hz support-face gait state machine with passive settling
 * 5. Terrain-normal Hertz contact, damping, Coulomb friction and restitution
 * 6. Rolling/slip, energy, support-face and topology diagnostics
 */

export class SimConfig {
  constructor(overrides = {}) {
    this.dt = 0.002;              // Fixed 500 Hz physics integration [s]
    this.T_end = 40.0;            // Simulation max time [s]

    // Geometry dimensions
    this.outerRadius = 0.50;      // Outer Radius R = 0.50 m (Outer Diameter D = 1.0 m = 1.0D)
    this.coreRadiusRatio = 0.10;  // Inner core diameter ratio = 0.1D (R_core = 0.05 m, D_core = 0.10 m)
    this.numStruts = 6;           // Exactly 6 primary compression struts

    // Mass & Physical Properties
    this.nodeMass = 0.20;         // Outer node mass [kg] (12 nodes * 0.20 = 2.4 kg)
    this.coreMass = 1.6;          // Central payload core mass [kg] (Total mass = 4.0 kg)
    this.damping = 0.30;          // Smooth rolling drag coefficient

    // Cable Mechanics (Actuated Spring-Damper Cables)
    this.kS = 1200.0;             // Outer cable stiffness [N/m]
    this.pretensionS = 40.0;      // Outer cable pretension [N]
    this.cS = 32.0;               // Cable axial damping [N s / m]

    // Topological CPG Cable Actuation Parameters
    this.actuationMode = 'roll_forward'; // 'roll_forward', 'roll_backward', 'steer_left', 'steer_right', or 'none'
    this.actuationDeltaL = 0.12;  // Maximum support-face gait contraction [m]
    this.gaitFrequency = 0.20;    // Retained for controller-profile comparisons [Hz]
    this.actuatorTau = 0.20;      // Actuator target smoothing time constant [s]
    this.simpleRollingMode = true;
    this.antiSpinThreshold = 0.65;

    // Drive controller stack, adapted to this six-bar topology
    this.controllerMode = 'natural_support_face';
    this.actuatorMode = 'cables'; // 'cables', 'rods', or 'both'
    this.controlHorizon = 12;
    this.controllerDt = 0.05;     // 20 Hz controller, independent of 1 kHz physics
    this.targetSpeed = 0.20;
    this.targetDestination = [0.0, 25.0];
    this.payloadTargetHeight = 0.55;
    this.measurementNoise = 0.002;
    this.disturbanceBound = 0.02;

    // Hard actuator constraints from the Drive MPC/iLQR controllers,
    // scaled to the present rover instead of copied as absolute dimensions.
    this.cableLinearVelocity = 0.10; // Physical spool/rest-length slew [m/s]
    this.cableMinRatio = 0.50;
    this.cableMaxRatio = 1.50;
    this.rodLinearVelocity = 0.15;
    this.rodMinRatio = 0.88;
    this.rodMaxRatio = 1.12;

    // Adaptive string relaxation from the paper reproduction in Drive.
    this.stringRelaxation = false;
    this.relaxStartRatio = 1.18;
    this.relaxEndRatio = 1.45;
    this.relaxedTension = 8.0;
    this.relaxSafetyRatio = 1.55;
    this.relaxSafetyStiffnessRatio = 2.50;

    // Central Payload Core Suspension Cables
    this.kCore = 1600.0;          // Core suspension cable stiffness [N/m]
    this.pretensionCore = 50.0;   // Core suspension pretension [N]
    this.cCore = 18.0;            // Core suspension damping [N s / m]
    this.coreActuationDeltaL = 0.065; // Bounded payload-shift stroke [m]
    this.coreCableLinearVelocity = 0.08; // Payload suspension spool rate [m/s]

    // Strut Mechanics (Stiff penalty spring keeping 6 rods rigid)
    this.kBar = 20000.0;          // Mild penalty; SHAKE enforces exact rod length
    this.cBar = 25.0;             // Bar axial damping [N s / m]
    this.strutMass = 0.3;         // Strut mass [kg]

    // Natural rolling gait timing. A cycle is deliberately human-visible;
    // state changes occur only on the 20 Hz controller clock.
    this.passiveSettlingDuration = 1.8;
    this.maxPassiveSettlingDuration = 3.0;
    this.stableHoldDuration = 0.30;
    this.preloadDuration = 0.35;
    this.shiftComDuration = 1.10;
    this.tipDuration = 1.10;
    this.rollTimeout = 1.00;
    this.impactSettleDuration = 0.50;
    this.supportContactMargin = 0.035;
    this.maxCableTensionSafety = 900.0;
    this.enableDiagnosticsLog = true;

    // Ground Interaction & Bouncing Physics
    this.enableGround = true;
    this.kg = 40000.0;            // Ground contact stiffness [N/m]
    this.cg = 60.0;               // Overdamped ground contact [N s / m]
    this.restitution = 0.02;      // Near-inelastic contact for non-bouncy rolling
    this.mu_g = 0.85;             // Surface friction coefficient (high traction)
    this.c_gt = 6.0;              // Viscous friction coefficient
    this.rollingConstraintGain = 18.0; // Couples ground speed to measured shell rotation [1/s]
    this.tractionLimitRatio = 0.35; // Fraction of Coulomb envelope available to no-slip correction
    this.rollTorqueGain = 30.0;    // Support-face roll-rate feedback [N m s]
    this.rollCoupleLimitRatio = 0.50; // Maximum internal roll couple relative to weight
    this.nodeRadius = 0.05;       // Effective contact node radius [m]
    this.contactSmoothBeta = 0.002;
    this.terrainClearanceEpsilon = 0.003;
    this.gravity = [0.0, 0.0, -9.81]; // Earth gravity -9.81 m/s^2 (down Z)
    this.inclineDegrees = 0.0;    // Drive FloorForceVertical_XIncline feature
    this.obstacleAvoidance = true;
    this.obstacleSensingRadius = 0.85;
    this.obstacleAvoidanceGain = 0.25;
    this.obstacleAvoidanceExponent = 0.40;

    // Terrain & Goal Configuration
    this.terrainLevel = 1;        // 1: Smooth, 2: Small Rocks, 3: Medium Rocks, 4: Large Rocks, 5: Crater, 6: Steep Slope, 7: Irregular Mars
    this.targetGoalY = 25.0;      // Endpoint goal target distance [m]
    this.groundRMS = 0.04;        // Elevation RMS [m]
    this.seed = 42;

    // Controlled A-vs-B course (Level 10).
    this.abCourseEnabled = false;
    this.courseStartY = 10.0;
    this.courseGoalY = 60.0;
    this.courseMaxY = 70.0;
    this.courseMaxRetries = 2;
    this.stallWindow = 5.0;

    // Current Active Experiment
    this.experimentId = 1;

    Object.assign(this, overrides);
  }
}

function createRNG(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const clampValue = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value));
const smoothStep01 = value => {
  const x = clampValue(value, 0, 1);
  return x*x*(3-2*x);
};

export class TerrainModel {
  constructor(cfg) {
    this.cfg = cfg;
    this.generateSurface();
  }

  generateSurface() {
    const rng = createRNG(this.cfg.seed);
    const lvl = this.cfg.terrainLevel;
    this.rocks = [];
    this.craters = [];
    this.slopes = [];
    this.course = this.cfg.abCourseEnabled ? createABCourse() : null;

    if (this.course) {
      this.rmsScale = 0.018;
    } else if (lvl === 1) {
      this.rmsScale = 0.02;
    } else if (lvl === 2) {
      this.rmsScale = 0.06;
      for (let i = 0; i < 8; i++) {
        this.rocks.push({ x: (rng() - 0.5) * 4.0, y: 3.0 + i * 2.0, r: 0.22, h: 0.15 });
      }
    } else if (lvl === 3) {
      this.rmsScale = 0.12;
      for (let i = 0; i < 10; i++) {
        this.rocks.push({ x: (rng() - 0.5) * 5.0, y: 3.0 + i * 2.2, r: 0.35, h: 0.28 });
      }
    } else if (lvl === 4) {
      this.rmsScale = 0.18;
      this.rocks.push({ x: 0.0, y: 5.0, r: 0.55, h: 0.45 });
      this.rocks.push({ x: 0.5, y: 11.0, r: 0.70, h: 0.55 });
    } else if (lvl === 5) {
      this.rmsScale = 0.12;
      this.craters.push({ x: 0.0, y: 6.0, r: 2.0, depth: 0.60 });
    } else if (lvl === 6) {
      this.rmsScale = 0.08;
      this.slopes.push({ yStart: 2.0, yEnd: 16.0, incline: 0.25 });
    } else if (lvl === 7 || lvl >= 8) {
      this.rmsScale = 0.20;
      for (let i = 0; i < 12; i++) {
        this.rocks.push({
          x: (rng() - 0.5) * 5.0,
          y: 3.0 + i * 2.0 + rng() * 1.5,
          r: 0.25 + rng() * 0.4,
          h: 0.20 + rng() * 0.35
        });
      }
      this.craters.push({ x: 0.8, y: 12.0, r: 1.6, depth: 0.5 });
    } else {
      this.rmsScale = this.cfg.groundRMS;
    }

    const M = 6, N = 8;
    const modes = [];
    let sumSq = 0;

    for (let m = -M; m <= M; m++) {
      for (let n = -N; n <= N; n++) {
        if (m === 0 && n === 0) continue;
        const kx = 2.0 * Math.PI * m * 0.08;
        const ky = 2.0 * Math.PI * n * 0.05;
        const k_sq = Math.pow(m / 2.4, 2) + Math.pow(n / 3.2, 2);
        const A_raw = 1.0 / Math.pow(1.0 + k_sq, 1.8 / 2.0);
        const phi = (2.0 * rng() - 1.0) * Math.PI;

        modes.push({ kx, ky, A: A_raw, phi });
        sumSq += A_raw * A_raw;
      }
    }

    const raw_rms = Math.sqrt(sumSq / 2.0);
    const scale = raw_rms > 1e-10 ? (this.rmsScale / raw_rms) : 1.0;
    for (let mode of modes) mode.A *= scale;
    this.modes = modes;
  }

  eval(x, y) {
    const surface = this.evalBase(x, y);
    let { h, dhdx, dhdy } = surface;
    for (const obstacle of this.course?.obstacles || []) {
      const contribution = evaluateCourseObstacle(obstacle, x, y);
      h += contribution.h;
      dhdx += contribution.dhdx;
      dhdy += contribution.dhdy;
    }
    return { h, dhdx, dhdy };
  }

  evalBase(x, y) {
    let h = 0;
    let dhdx = 0;
    let dhdy = 0;

    const roughnessScale = Math.min(1.0, Math.max(0.0, (y - 1.0) / 2.0));
    for (let i = 0; i < this.modes.length; i++) {
      const { kx, ky, A, phi } = this.modes[i];
      const arg = kx * x + ky * y + phi;
      const cos_arg = Math.cos(arg);
      const sin_arg = Math.sin(arg);

      h += A * cos_arg * roughnessScale;
      dhdx -= A * kx * sin_arg * roughnessScale;
      dhdy -= A * ky * sin_arg * roughnessScale;
    }

    for (let rock of this.rocks) {
      const dx = x - rock.x;
      const dy = y - rock.y;
      const d2 = dx * dx + dy * dy;
      const r2 = rock.r * rock.r;
      if (d2 < 4.0 * r2) {
        const factor = Math.exp(-d2 / (0.8 * r2));
        const rockH = rock.h * factor;
        h += rockH;
        dhdx += rockH * (-2.0 * dx / (0.8 * r2));
        dhdy += rockH * (-2.0 * dy / (0.8 * r2));
      }
    }

    for (let crater of this.craters) {
      const dx = x - crater.x;
      const dy = y - crater.y;
      const d = Math.sqrt(dx * dx + dy * dy);
      const r = crater.r;
      if (d < 2.5 * r) {
        const normD = d / r;
        const profile = -crater.depth * Math.exp(-normD * normD * 1.5) + 0.15 * crater.depth * Math.exp(-Math.pow(normD - 1.1, 2) * 8.0);
        h += profile;
        if (d > 1e-6) {
          const dProfile = crater.depth * 3.0 * normD * Math.exp(-normD * normD * 1.5) / r - 0.3 * crater.depth * (normD - 1.1) * 8.0 * Math.exp(-Math.pow(normD - 1.1, 2) * 8.0) / r;
          dhdx += dProfile * (dx / d);
          dhdy += dProfile * (dy / d);
        }
      }
    }

    for (let slope of this.slopes) {
      if (slope.incline && y >= slope.yStart && y <= slope.yEnd) {
        const dy = y - slope.yStart;
        h += dy * slope.incline;
        dhdy += slope.incline;
      } else if (slope.incline && y > slope.yEnd) {
        h += (slope.yEnd - slope.yStart) * slope.incline;
      }
    }

    // Optional global X incline adapted from FloorForceVertical_XIncline.m.
    const globalIncline = Math.tan((this.cfg.inclineDegrees || 0) * Math.PI / 180);
    if (Math.abs(globalIncline) > 1e-10) {
      h += x * globalIncline;
      dhdx += globalIncline;
    }

    return { h, dhdx, dhdy };
  }
}

let topologyReportPrinted = false;

export class SphericalRoverModel {
  constructor(cfg) {
    this.cfg = cfg;
    this.buildGeometry();
  }

  buildGeometry() {
    const R = this.cfg.outerRadius;

    // Verified six-bar/24-cable expanded-icosahedron geometry from the
    // project's six_bar_model.m. Keeping its node ordering together with its
    // cable matrix is essential: the previous golden-ratio coordinates were
    // paired with a different cable ordering and were not a prestress-stable
    // structure. Scale the source coordinates to the requested outer radius.
    const sourceNodes = [
      [ 0.1415, -0.1884, -0.3085],
      [-0.3785, -0.0459,  0.0728],
      [ 0.3785,  0.0459, -0.0728],
      [-0.1415,  0.1884,  0.3085],
      [-0.2290,  0.3049, -0.0728],
      [-0.0924, -0.2168,  0.3085],
      [ 0.0924,  0.2168, -0.3085],
      [ 0.2290, -0.3049,  0.0728],
      [ 0.2339,  0.0283,  0.3085],
      [-0.1495, -0.3507, -0.0728],
      [ 0.1495,  0.3507,  0.0728],
      [-0.2339, -0.0283, -0.3085]
    ];
    const sourceRadius = Math.max(...sourceNodes.map(position => Math.hypot(...position)));
    const scale = R/sourceRadius;
    this.q0_outer = sourceNodes.map(position => position.map(component => component*scale));

    this.nOuter = 12;
    this.R_outer = R;                                       // R = 0.50 m (D = 1.0 m)
    this.R_core = R * this.cfg.coreRadiusRatio;             // R_core = 0.05 m (D_core = 0.10 m)

    // Six disjoint compression struts. Every node is exactly one rod endpoint.
    this.bars = [
      [0, 1], // Bar 0
      [2, 3], // Bar 1
      [4, 5], // Bar 2
      [6, 7], // Bar 3
      [8, 9], // Bar 4
      [10, 11]// Bar 5
    ];

    // Matching 24-cable topology from six_bar_model.m (converted to 0-based
    // node indices). Each node has exactly four tensile connections.
    this.outerStrings = [
      [1, 5], [1, 4], [1, 9], [1, 11],
      [0, 6], [0, 7], [0, 9], [0, 11],
      [2, 6], [2, 7], [2, 10], [2, 8],
      [3, 5], [3, 4], [3, 10], [3, 8],
      [5, 9], [5, 8], [4, 11], [4, 10],
      [6, 11], [6, 10], [7, 9], [7, 8]
    ];

    // Body-Frame Cable Angles for CPG Rolling Wave Actuation
    this.cableBodyPhase = [];
    for (let s = 0; s < this.outerStrings.length; s++) {
      const [i, j] = this.outerStrings[s];
      const pA = this.q0_outer[i];
      const pB = this.q0_outer[j];
      const midY = (pA[1] + pB[1]) * 0.5;
      const midZ = (pA[2] + pB[2]) * 0.5;
      const angle = Math.atan2(midY, midZ);
      this.cableBodyPhase.push(angle);
    }

    this.l0_bars = [];
    for (let b = 0; b < this.bars.length; b++) {
      const [i, j] = this.bars[b];
      const pA = this.q0_outer[i];
      const pB = this.q0_outer[j];
      const dx = pA[0] - pB[0];
      const dy = pA[1] - pB[1];
      const dz = pA[2] - pB[2];
      this.l0_bars.push(Math.sqrt(dx*dx + dy*dy + dz*dz));
    }

    // Preserve the cable prestress that holds the tensegrity shape together.
    // These arrays are consumed by both the live solver and the benchmark
    // solver, so they must be rebuilt whenever a rover model is constructed.
    this.geomLen_outerStrings = this.outerStrings.map(([i, j]) => {
      const pA = this.q0_outer[i];
      const pB = this.q0_outer[j];
      const dx = pA[0] - pB[0];
      const dy = pA[1] - pB[1];
      const dz = pA[2] - pB[2];
      return Math.sqrt(dx*dx + dy*dy + dz*dz);
    });
    this.l0_outerStrings = this.geomLen_outerStrings.map(gLen =>
      Math.max(1e-4, gLen - (this.cfg.pretensionS / this.cfg.kS))
    );

    this.topologyReport = this.validateTopology();
    if (!this.topologyReport.valid) {
      throw new Error(`Invalid 6-bar tensegrity topology: ${this.topologyReport.errors.join('; ')}`);
    }
    if (!topologyReportPrinted) {
      console.info('[Tensegrity topology]', this.topologyReport.summary);
      topologyReportPrinted = true;
    }
  }

  validateTopology() {
    const errors = [];
    const normalizePair = ([a, b]) => a < b ? `${a}-${b}` : `${b}-${a}`;
    const rodPairs = this.bars.map(normalizePair);
    const cablePairs = this.outerStrings.map(normalizePair);
    const rodSet = new Set(rodPairs);
    const cableSet = new Set(cablePairs);
    const rodEndpoints = this.bars.flat();
    const cableDegree = new Array(this.nOuter).fill(0);
    for (const [i, j] of this.outerStrings) {
      if (i === j || i < 0 || j < 0 || i >= this.nOuter || j >= this.nOuter) errors.push(`invalid cable ${i}-${j}`);
      cableDegree[i]++; cableDegree[j]++;
    }
    if (this.bars.length !== 6) errors.push(`expected 6 rods, found ${this.bars.length}`);
    if (rodEndpoints.length !== 12 || new Set(rodEndpoints).size !== 12) errors.push('rod endpoints are not twelve unique nodes');
    if (this.outerStrings.length !== 24) errors.push(`expected 24 cables, found ${this.outerStrings.length}`);
    if (rodSet.size !== this.bars.length) errors.push('duplicate rod connection');
    if (cableSet.size !== this.outerStrings.length) errors.push('duplicate cable connection');
    for (const pair of cableSet) if (rodSet.has(pair)) errors.push(`rod/cable overlap ${pair}`);
    cableDegree.forEach((degree, node) => {
      if (degree !== 4) errors.push(`node ${node} has cable degree ${degree}, expected 4`);
    });
    return {
      valid: errors.length === 0,
      rods: this.bars.length,
      rodEndpoints: new Set(rodEndpoints).size,
      cables: this.outerStrings.length,
      uniqueCables: cableSet.size,
      cableDegree,
      errors,
      summary: `rods=${this.bars.length}, rod endpoints=${new Set(rodEndpoints).size}, cables=${this.outerStrings.length}, unique cables=${cableSet.size}, degree=[${cableDegree.join(',')}], valid=${errors.length === 0}`
    };
  }
}

export class Simulation {
  constructor(cfg, rover, terrain, modelType = 'fixed') {
    this.cfg = cfg;
    this.rover = rover;
    this.terrain = terrain;
    this.modelType = modelType; // 'fixed' or 'adaptive'
    this.reset();
  }

  reset() {
    this.t = 0.0;
    this.stepCount = 0;

    const n = this.rover.nOuter;
    const lowestLocalNode = Math.min(...this.rover.q0_outer.map(position => position[2]));
    const initialY = this.cfg.abCourseEnabled ? this.cfg.courseStartY-1.0 : 0;
    const initialZ = (this.terrain.eval(0, initialY).h || 0)+this.cfg.nodeRadius+0.02-lowestLocalNode;

    this.q = this.rover.q0_outer.map(p => [p[0], p[1]+initialY, p[2] + initialZ]);
    this.v = new Array(n).fill(0).map(() => [0.0, 0.0, 0.0]);
    this.corePosition = [0, initialY, initialZ];
    this.coreVelocity = [0, 0, 0];
    this.baseCoreRestLengths = this.q.map(position =>
      Math.max(0.02, Math.hypot(
        position[0]-this.corePosition[0],
        position[1]-this.corePosition[1],
        position[2]-this.corePosition[2]
      )-this.cfg.pretensionCore/this.cfg.kCore));
    this.coreRestLengths = this.baseCoreRestLengths.slice();
    this.currentCoreActuationOffsets = new Array(n).fill(0);
    this.targetCoreActuationOffsets = new Array(n).fill(0);
    this.coreCableForces = new Array(n).fill(this.cfg.pretensionCore);

    this.gaitPhase = 0.0;
    this.locomotionState = 'PASSIVE_SETTLE';
    this.stateEnteredAt = 0.0;
    this.nextControllerUpdate = 0.0;
    this.nextDiagnosticLog = 0.0;
    this.supportFace = [];
    this.supportSignature = '';
    this.cycleStartSupportSignature = '';
    this.targetEdge = [];
    this.contractingCableIndices = [];
    this.relaxingCableIndices = [];
    this.comMargin = Infinity;
    this.supportChangeTicks = 0;
    this.completedRolls = 0;
    this.dynamicNodeMass = this.cfg.nodeMass + 0.5*this.cfg.strutMass;
    this.prevCvx = 0.0;
    this.prevCvy = 0.0;
    this.prevCvz = 0.0;
    this.filteredCentroidAccelG = 1.0;
    this.obstacleTracker = this.terrain.course
      ? new ObstaclePassTracker(this.terrain.course, this.modelType)
      : null;
    this.obstaclePhase = 'COURSE_APPROACH';
    this.activeObstacleId = null;
    this.lastObstacleProgressY = initialY;
    this.lastObstacleProgressAt = 0;
    this.obstacleRecoveryUntil = 0;
    this.measuredRunStartedAt = null;
    this.measuredRunCompletedAt = null;
    this.prevMetricPosition = [0, initialY];
    this.speedSamples = [];

    // String length offsets
    this.currentActuationOffsets = new Array(this.rover.outerStrings.length).fill(0.0);
    this.targetActuationOffsets = new Array(this.rover.outerStrings.length).fill(0.0);
    this.currentRodOffsets = new Array(this.rover.bars.length).fill(0.0);
    this.targetRodOffsets = new Array(this.rover.bars.length).fill(0.0);
    this.relaxedCableFlags = new Array(this.rover.outerStrings.length).fill(false);
    this.controlDiagnostics = {
      mode: this.modelType === 'fixed' ? 'cpg' : this.cfg.controllerMode,
      modeLabel: this.modelType === 'fixed' ? 'CPG Baseline' : this.cfg.controllerMode,
      desiredDirection: [0, 1, 0],
      predictedPath: [],
      controlCost: 0,
      activeCableCount: 0,
      activeRodCount: 0,
      disturbanceEstimate: 0,
      horizon: this.cfg.controlHorizon
    };
    this.sensorRng = createRNG(this.cfg.seed + (this.modelType === 'adaptive' ? 911 : 173));

    // Speed controller
    this.targetSpeedController = {
      integralError: 0.0,
      kp: 0.5,
      ki: 0.1,
      targetSpeed: this.cfg.targetSpeed
    };

    this.metrics = {
      distanceTraveled: 0.0,
      timeElapsed: 0.0,
      avgVelocity: 0.0,
      maxVelocity: 0.0,
      maxAngularVelocity: 0.0,
      successfulObstacles: 0,
      failedAttempts: 0,
      energyCost: 0.0,
      maxCableTension: 0.0,
      maxCableExtension: 0.0,
      maxStrutCompression: 0.0,
      maxConstraintError: 0.0,
      peakKineticEnergy: 0.0,
      maxSlipSpeed: 0.0,
      completedRolls: 0,
      stabilityScore: 100.0,
      payloadAccelMax: 1.0,
      shapeDeformationMax: 0.0,
      measuredDistance: 0.0,
      measuredTime: 0.0,
      speedVariance: 0.0,
      lateralTravel: 0.0,
      completionTime: null,
      courseComplete: false,
      obstacleSummary: this.obstacleTracker?.summary() || null
    };

    this.history = {
      t: [],
      centroidY: [],
      centroidZ: [],
      centroidAccel: [],
      deformation: [],
      maxTension: [],
      maxCompression: [],
      planarSpeed: [],
      controlCost: [],
      constraintError: [],
      relaxationFraction: [],
      slipSpeed: [],
      kineticEnergy: [],
      rollingRatio: []
    };

    this.currentDiag = {
      centroid: [0, initialY, initialZ],
      centroidAccelG: 1.0,
      outerCableForces: new Array(this.rover.outerStrings.length).fill(this.cfg.pretensionS),
      outerCableActuated: new Array(this.rover.outerStrings.length).fill(false),
      outerCableRelaxed: new Array(this.rover.outerStrings.length).fill(false),
      strutForces: new Array(this.rover.bars.length).fill(0),
      strutActuated: new Array(this.rover.bars.length).fill(false),
      deformationRMS: 0.0,
      groundContactNodes: [],
      velocityVector: [0, 0, 0],
      angularVelocity: 0.0,
      state: this.locomotionState,
      controllerMode: this.controlDiagnostics.mode,
      controllerLabel: this.controlDiagnostics.modeLabel,
      desiredDirection: [0, 1, 0],
      predictedPath: [],
      controlCost: 0,
      constraintError: 0,
      relaxationFraction: 0,
      disturbanceEstimate: 0,
      topology: this.rover.topologyReport,
      supportFace: [],
      targetEdge: [],
      contractingCableIndices: [],
      relaxingCableIndices: [],
      comMargin: Infinity,
      slipSpeed: 0,
      rollingRatio: 0,
      rollingSpeed: 0,
      rollingError: 0,
      angularVelocityVector: [0, 0, 0],
      kineticEnergy: 0,
      completedRolls: 0,
      terrainClearance: this.cfg.terrainClearanceEpsilon,
      terrainLiftCorrection: 0,
      corePosition: this.corePosition.slice(),
      coreVelocity: this.coreVelocity.slice(),
      coreCableForces: this.coreCableForces.slice(),
      obstaclePhase: this.obstaclePhase,
      activeObstacleId: null,
      obstacleSummary: this.metrics.obstacleSummary,
      actuationTraction: 0,
      actuationRollTorque: 0
    };
  }

  senseObstacleAhead(cx, cy, cvy) {
    if (!this.terrain) return { detected: false, height: 0, distance: 0, steerSign: 0 };
    if (this.terrain.course) {
      return senseCourseObstacle(this.terrain.course, cx, cy, cvy || 1);
    }
    // Look ahead 0.5m to 1.5m in the direction of travel
    const dir = cvy >= 0 ? 1 : -1;
    let maxDh = 0;
    let obstacleDist = 0;
    let steerSign = 0;

    const baseH = this.terrain.eval(cx, cy).h;

    for (let dist = 0.5; dist <= 1.5; dist += 0.2) {
      const sampleY = cy + dir * dist;
      const sample = this.terrain.eval(cx, sampleY);
      const dh = sample.h - baseH;
      if (dh > maxDh) {
        maxDh = dh;
        obstacleDist = dist;
        steerSign = sample.dhdx >= 0 ? -1 : 1;
      }
    }

    // Prefer steering away from the nearest explicit rock center.
    for (const rock of this.terrain.rocks || []) {
      const ahead = (rock.y-cy) * dir;
      if (ahead > 0 && ahead < 1.8 && Math.abs(rock.x-cx) < rock.r+0.8) {
        obstacleDist = obstacleDist || ahead;
        maxDh = Math.max(maxDh, rock.h);
        steerSign = rock.x >= cx ? -1 : 1;
      }
    }

    if (maxDh > 0.15) {
      return { detected: true, height: maxDh, distance: obstacleDist, steerSign };
    }
    return { detected: false, height: 0, distance: 0, steerSign: 0 };
  }

  desiredDirectionForMode(centroid, obstacle) {
    const mode = this.cfg.actuationMode;
    if (mode === 'none') return [0, 0, 0];
    if (mode === 'roll_backward') return [0, -1, 0];
    if (mode === 'steer_left') return [-0.55, 0.835, 0];
    if (mode === 'steer_right') return [0.55, 0.835, 0];
    const target = this.cfg.targetDestination || [0, this.cfg.targetGoalY || 25];
    let dx = target[0]-centroid[0];
    let dy = target[1]-centroid[1];
    if (this.modelType === 'adaptive' && this.terrain.course && obstacle?.detected) {
      // Model B must align with, and pass through, the obstacle footprint.
      // This is a steering target only; it does not translate the rover.
      const lateralError = obstacle.obstacle.x-centroid[0];
      const corridor = this.terrain.course.obstacleCorridorHalfWidth;
      dx = (Math.abs(lateralError) > corridor ? 4.2 : 2.8)*lateralError;
      dy = Math.max(Math.abs(lateralError) > corridor ? 0.12 : 0.35,
        obstacle.obstacle.y+obstacle.obstacle.radiusY-centroid[1]);
      // If the shell crosses the crest outside the centre band, back off and
      // realign physically before the classifier could accept a side pass.
      if (centroid[1] > obstacle.obstacle.y+0.15*obstacle.obstacle.radiusY &&
        Math.abs(lateralError) > corridor) dy = -0.45;
      if (this.t < this.obstacleRecoveryUntil) dy = -0.30;
    } else if (this.modelType === 'fixed' && obstacle?.detected) {
      // Model A is an unconstrained baseline and may naturally go around.
      dx += (obstacle.steerSign || 1)*Math.max(0, 1.4-obstacle.distance)*0.65;
    }
    const length = Math.hypot(dx, dy);
    return length > 1e-9 ? [dx/length, dy/length, 0] : [0, 1, 0];
  }

  detectSupportGeometry(centroid, desiredDirection) {
    const candidates = this.q.map((position, node) => {
      const ground = this.terrain.eval(position[0], position[1]).h;
      return { node, clearance: position[2]-ground-this.cfg.nodeRadius };
    }).sort((a, b) => a.clearance-b.clearance);
    const lowest = candidates[0]?.clearance ?? 0;
    const contactLimit = lowest + this.cfg.supportContactMargin;
    const activeContacts = candidates.filter(item => item.clearance <= contactLimit).map(item => item.node);
    const supportFace = candidates.slice(0, 3).map(item => item.node);
    const directionLength = Math.hypot(desiredDirection[0], desiredDirection[1]);
    const dx = directionLength > 1e-9 ? desiredDirection[0]/directionLength : 0;
    const dy = directionLength > 1e-9 ? desiredDirection[1]/directionLength : 1;
    let targetEdge = supportFace.slice(0, 2);
    let bestAlignment = -Infinity;
    let bestMargin = Infinity;
    for (let a = 0; a < supportFace.length; a++) {
      for (let b = a+1; b < supportFace.length; b++) {
        const i = supportFace[a];
        const j = supportFace[b];
        const third = supportFace.find(node => node !== i && node !== j);
        const mx = 0.5*(this.q[i][0]+this.q[j][0]);
        const my = 0.5*(this.q[i][1]+this.q[j][1]);
        const edgeX = this.q[j][0]-this.q[i][0];
        const edgeY = this.q[j][1]-this.q[i][1];
        const edgeLength = Math.hypot(edgeX, edgeY);
        if (edgeLength < 1e-8) continue;
        let outwardX = -edgeY/edgeLength;
        let outwardY = edgeX/edgeLength;
        if (third !== undefined) {
          const thirdSide = (this.q[third][0]-mx)*outwardX+(this.q[third][1]-my)*outwardY;
          if (thirdSide > 0) { outwardX *= -1; outwardY *= -1; }
        }
        const alignment = outwardX*dx+outwardY*dy;
        if (alignment > bestAlignment) {
          bestAlignment = alignment;
          targetEdge = [i, j];
          bestMargin = -((centroid[0]-mx)*outwardX+(centroid[1]-my)*outwardY);
        }
      }
    }
    return {
      activeContacts,
      supportFace,
      signature: supportFace.slice().sort((a, b) => a-b).join('-'),
      targetEdge,
      comMargin: Number.isFinite(bestMargin) ? bestMargin : 0
    };
  }

  chooseSupportFaceCables(centroid, desiredDirection, supportFace, targetEdge) {
    const edgeSet = new Set(targetEdge);
    const supportSet = new Set(supportFace);
    const directionLength = Math.hypot(desiredDirection[0], desiredDirection[1]);
    const dx = directionLength > 1e-9 ? desiredDirection[0]/directionLength : 0;
    const dy = directionLength > 1e-9 ? desiredDirection[1]/directionLength : 1;
    // Always derive the cable set from the current support face and selected
    // tipping edge. A previous hard-coded face table could command a different
    // support edge after settling, which only rocked the rover in place.
    const nodeForward = this.q.map(position =>
      (position[0]-centroid[0])*dx+(position[1]-centroid[1])*dy);
    const leadCandidates = nodeForward.map((projection, node) => ({ node, projection }))
      .filter(item => !supportSet.has(item.node)).sort((a, b) => b.projection-a.projection);
    const leadNode = leadCandidates[0]?.node ?? nodeForward.indexOf(Math.max(...nodeForward));
    const incidentToLead = [];
    const trailingSupportNode = supportFace.find(node => !edgeSet.has(node));
    const liftTrailing = [];
    const releaseTrailing = [];

    this.rover.outerStrings.forEach(([i, j], cable) => {
      if (i === leadNode || j === leadNode) {
        incidentToLead.push(cable);
      }
      if (i === trailingSupportNode || j === trailingSupportNode) {
        const other = i === trailingSupportNode ? j : i;
        if (edgeSet.has(other)) releaseTrailing.push(cable);
        else if (!supportSet.has(other)) liftTrailing.push(cable);
      }
    });

    // Pull the leading free node toward the tipping edge while lifting the
    // third (trailing) support node. The two trailing-to-edge cables relax so
    // they do not pin that node to the ground. This creates the intended
    // two-node pivot instead of merely shrinking the cage around one face.
    const contracting = [...new Set([...incidentToLead, ...liftTrailing])].slice(0, 6);
    return { contracting, relaxing: releaseTrailing.slice(0, 2) };
  }

  enterLocomotionState(nextState) {
    if (nextState === this.locomotionState) return;
    this.locomotionState = nextState;
    this.stateEnteredAt = this.t;
    this.supportChangeTicks = 0;
  }

  updateNaturalRollingController(centroid, velocity, omega, obstacle) {
    const cfg = this.cfg;
    if (this.terrain.course && this.modelType === 'adaptive') {
      if (obstacle?.detected && obstacle.obstacle) {
        const changedObstacle = this.activeObstacleId !== obstacle.obstacle.id;
        if (changedObstacle) {
          this.activeObstacleId = obstacle.obstacle.id;
          this.lastObstacleProgressY = centroid[1];
          this.lastObstacleProgressAt = this.t;
        }
        if (centroid[1] >= this.lastObstacleProgressY+0.03) {
          this.lastObstacleProgressY = centroid[1];
          this.lastObstacleProgressAt = this.t;
        } else if (this.t-this.lastObstacleProgressAt >= cfg.stallWindow &&
          this.t >= this.obstacleRecoveryUntil) {
          const record = this.obstacleTracker?.records.get(obstacle.obstacle.id);
          if ((record?.attempts || 0) <= cfg.courseMaxRetries) {
            this.obstacleTracker?.markRetry(obstacle.obstacle.id);
            this.obstacleRecoveryUntil = this.t+1.0;
            this.lastObstacleProgressAt = this.t;
          }
        }

        const aligned = Math.abs(obstacle.lateralError) <= this.terrain.course.obstacleCorridorHalfWidth;
        if (this.t < this.obstacleRecoveryUntil) this.obstaclePhase = 'RECOVER_RETRY';
        else if (!aligned) this.obstaclePhase = 'ALIGN';
        else if (obstacle.distance > 0.25) this.obstaclePhase = 'APPROACH';
        else if (centroid[1] < obstacle.obstacle.y) this.obstaclePhase = 'DEFORM_CLIMB';
        else if (centroid[1] <= obstacle.obstacle.y+0.45*obstacle.obstacle.radiusY) this.obstaclePhase = 'COM_OVER';
        else this.obstaclePhase = 'DESCEND_RESTORE';
      } else {
        this.activeObstacleId = null;
        this.obstaclePhase = centroid[1] >= cfg.courseGoalY ? 'GOAL' : 'CRUISE_RESTORE';
      }
    } else if (this.terrain.course) {
      this.activeObstacleId = obstacle?.obstacle?.id || null;
      this.obstaclePhase = obstacle?.detected ? 'BASELINE_FREE_PATH' : 'BASELINE_CRUISE';
    }
    const desiredDirection = this.desiredDirectionForMode(centroid, obstacle);
    const support = this.detectSupportGeometry(centroid, desiredDirection);
    this.supportFace = support.supportFace;
    this.supportSignature = support.signature;
    this.comMargin = support.comMargin;

    const elapsed = this.t-this.stateEnteredAt;
    const maxNodeSpeed = this.v.reduce((maximum, nodeVelocity) =>
      Math.max(maximum, Math.hypot(nodeVelocity[0], nodeVelocity[1], nodeVelocity[2])), 0);

    if (cfg.actuationMode === 'none') {
      this.enterLocomotionState('PASSIVE_SETTLE');
    } else if (this.locomotionState === 'PASSIVE_SETTLE') {
      const settled = support.activeContacts.length >= 2 && maxNodeSpeed < 0.28 && omega < 0.45;
      if ((elapsed >= cfg.passiveSettlingDuration && settled) || elapsed >= cfg.maxPassiveSettlingDuration) {
        this.enterLocomotionState('STABLE');
      }
    } else if (this.locomotionState === 'STABLE' && elapsed >= cfg.stableHoldDuration) {
      this.targetEdge = support.targetEdge;
      this.cycleStartSupportSignature = support.signature;
      const selected = this.chooseSupportFaceCables(centroid, desiredDirection, support.supportFace, support.targetEdge);
      this.contractingCableIndices = selected.contracting;
      this.relaxingCableIndices = selected.relaxing;
      this.enterLocomotionState('PRELOAD');
    } else if (this.locomotionState === 'PRELOAD' && elapsed >= cfg.preloadDuration) {
      this.enterLocomotionState('SHIFT_COM');
    } else if (this.locomotionState === 'SHIFT_COM' &&
      (elapsed >= cfg.shiftComDuration || (elapsed >= 0.45 && support.comMargin < 0.015))) {
      this.enterLocomotionState('TIP');
    } else if (this.locomotionState === 'TIP' &&
      (elapsed >= cfg.tipDuration || (elapsed >= 0.30 && omega > 0.32))) {
      this.enterLocomotionState('ROLL');
    } else if (this.locomotionState === 'ROLL') {
      const supportChanged = support.signature && support.signature !== this.cycleStartSupportSignature;
      this.supportChangeTicks = supportChanged ? this.supportChangeTicks+1 : 0;
      if (this.supportChangeTicks >= 2 || elapsed >= cfg.rollTimeout) {
        if (supportChanged) this.completedRolls++;
        this.enterLocomotionState('IMPACT_SETTLE');
      }
    } else if (this.locomotionState === 'IMPACT_SETTLE') {
      const calm = maxNodeSpeed < 0.32 && omega < 0.55;
      if ((elapsed >= cfg.impactSettleDuration && calm) || elapsed >= 1.10) {
        this.enterLocomotionState('STABLE');
      }
    }

    const stateElapsed = this.t-this.stateEnteredAt;
    let actuationFactor = 0;
    if (this.locomotionState === 'PRELOAD') {
      actuationFactor = 0.45*smoothStep01(stateElapsed/cfg.preloadDuration);
    } else if (this.locomotionState === 'SHIFT_COM') {
      actuationFactor = 0.45+0.55*smoothStep01(stateElapsed/cfg.shiftComDuration);
    } else if (this.locomotionState === 'TIP') {
      actuationFactor = 1.0;
    } else if (this.locomotionState === 'ROLL') {
      actuationFactor = 0.65;
    }
    if (this.modelType === 'fixed') actuationFactor *= 0.82;

    const obstacleProfile = this.modelType === 'adaptive' && obstacle?.obstacle
      ? obstacleActuation(obstacle.obstacle, cfg.actuationDeltaL)
      : null;
    if (obstacleProfile) {
      this.contractingCableIndices = this.contractingCableIndices.slice(0, obstacleProfile.cableCount);
      if (this.contractingCableIndices.length && !this.relaxingCableIndices.length) {
        const active = new Set(this.contractingCableIndices);
        const trailing = this.rover.outerStrings.map(([i, j], cable) => ({
          cable,
          y: 0.5*(this.q[i][1]+this.q[j][1])
        })).filter(item => !active.has(item.cable)).sort((a, b) => a.y-b.y);
        this.relaxingCableIndices = trailing.slice(0, Math.min(2, obstacleProfile.cableCount-2)).map(item => item.cable);
      }
    }

    const cableTargets = new Array(this.rover.outerStrings.length).fill(0);
    const commandedDelta = obstacleProfile?.delta || cfg.actuationDeltaL;
    const relaxationRatio = obstacleProfile?.relaxationRatio ?? 0.35;
    for (const cable of this.contractingCableIndices) cableTargets[cable] = commandedDelta*actuationFactor;
    for (const cable of this.relaxingCableIndices) cableTargets[cable] = -relaxationRatio*commandedDelta*actuationFactor;
    if (this.locomotionState === 'PASSIVE_SETTLE' || this.locomotionState === 'STABLE' || this.locomotionState === 'IMPACT_SETTLE') {
      cableTargets.fill(0);
    }

    // Central-payload COM shift from the Drive central-payload controller
    // architecture. Only the four leading suspension cables contract and two
    // trailing cables relax; this remains an internal, rate-limited actuator.
    const coreTargets = new Array(this.rover.nOuter).fill(0);
    if (actuationFactor > 0 && !['PASSIVE_SETTLE', 'STABLE', 'IMPACT_SETTLE'].includes(this.locomotionState)) {
      const projectedNodes = this.q.map((position, node) => ({
        node,
        projection: (position[0]-centroid[0])*desiredDirection[0]+
          (position[1]-centroid[1])*desiredDirection[1]
      })).sort((a, b) => b.projection-a.projection);
      const speedAlongTarget = velocity[0]*desiredDirection[0]+velocity[1]*desiredDirection[1];
      const speedErrorBoost = clampValue((cfg.targetSpeed-Math.max(0, speedAlongTarget))/cfg.targetSpeed, 0, 1);
      const payloadStroke = cfg.coreActuationDeltaL*(0.80+0.20*speedErrorBoost)*actuationFactor;
      for (const item of projectedNodes.slice(0, 4)) coreTargets[item.node] = payloadStroke;
      for (const item of projectedNodes.slice(-2)) coreTargets[item.node] = -0.35*payloadStroke;
    }

    const destination = cfg.targetDestination || [0, cfg.targetGoalY || 25];
    const targetError = Math.hypot(destination[0]-centroid[0], destination[1]-centroid[1]);
    const targetEffort = cableTargets.reduce((sum, offset) => sum+offset*offset, 0);
    return {
      cableTargets,
      coreTargets,
      rodTargets: new Array(this.rover.bars.length).fill(0),
      diagnostics: {
        mode: 'natural_support_face',
        modeLabel: this.modelType === 'fixed' ? 'Model A · baseline/avoidance allowed' : 'Model B · strict over-obstacle',
        desiredDirection,
        predictedPath: [[centroid[0], centroid[1], centroid[2]], [centroid[0]+desiredDirection[0], centroid[1]+desiredDirection[1], centroid[2]]],
        controlCost: targetError*0.02+targetEffort*50,
        activeCableCount: actuationFactor > 0 ? this.contractingCableIndices.length+this.relaxingCableIndices.length : 0,
        activeRodCount: 0,
        disturbanceEstimate: obstacle?.detected ? obstacle.height : 0,
        horizon: cfg.controlHorizon,
        supportFace: support.supportFace,
        targetEdge: this.targetEdge,
        comMargin: support.comMargin
      }
    };
  }

  calcAngularVelocityVector(cx, cy, cz, cvx, cvy, cvz) {
    let Lx = 0, Ly = 0, Lz = 0;
    let Ixx = 0, Iyy = 0, Izz = 0;
    let Ixy = 0, Ixz = 0, Iyz = 0;
    const m_node = this.dynamicNodeMass;

    for (let i = 0; i < this.rover.nOuter; i++) {
      const r = [this.q[i][0] - cx, this.q[i][1] - cy, this.q[i][2] - cz];
      const v_rel = [this.v[i][0] - cvx, this.v[i][1] - cvy, this.v[i][2] - cvz];

      Lx += m_node * (r[1] * v_rel[2] - r[2] * v_rel[1]);
      Ly += m_node * (r[2] * v_rel[0] - r[0] * v_rel[2]);
      Lz += m_node * (r[0] * v_rel[1] - r[1] * v_rel[0]);

      Ixx += m_node * (r[1]*r[1] + r[2]*r[2]);
      Iyy += m_node * (r[0]*r[0] + r[2]*r[2]);
      Izz += m_node * (r[0]*r[0] + r[1]*r[1]);
      Ixy -= m_node * r[0]*r[1];
      Ixz -= m_node * r[0]*r[2];
      Iyz -= m_node * r[1]*r[2];
    }

    // Solve I*omega=L using the analytic inverse of the symmetric inertia
    // tensor. The former scalar approximation overstated spin whenever the
    // tensegrity deformed away from a perfect sphere.
    const c00 = Iyy*Izz-Iyz*Iyz;
    const c01 = Ixz*Iyz-Ixy*Izz;
    const c02 = Ixy*Iyz-Ixz*Iyy;
    const c11 = Ixx*Izz-Ixz*Ixz;
    const c12 = Ixy*Ixz-Ixx*Iyz;
    const c22 = Ixx*Iyy-Ixy*Ixy;
    const determinant = Ixx*c00 + Ixy*c01 + Ixz*c02;
    if (Math.abs(determinant) < 1e-9) return [0, 0, 0];
    const wx = (c00*Lx+c01*Ly+c02*Lz)/determinant;
    const wy = (c01*Lx+c11*Ly+c12*Lz)/determinant;
    const wz = (c02*Lx+c12*Ly+c22*Lz)/determinant;

    return [wx, wy, wz];
  }

  calcAngularVelocity(cx, cy, cz, cvx, cvy, cvz) {
    const [wx, wy, wz] = this.calcAngularVelocityVector(cx, cy, cz, cvx, cvy, cvz);
    return Math.sqrt(wx*wx + wy*wy + wz*wz);
  }

  minimumTerrainClearance(includeMembers = true) {
    if (!this.cfg.enableGround) return Infinity;
    let minimum = Infinity;
    const q = this.q;
    const terrain = this.terrain;

    // Nodes are the primary contact geometry.
    for (let i = 0; i < q.length; i++) {
      const ground = terrain.eval(q[i][0], q[i][1]).h;
      minimum = Math.min(minimum, q[i][2]-ground-this.cfg.nodeRadius);
    }

    // Sample collision clearance along every member so a cable or strut cannot
    // visually pass through a ridge while both endpoints remain above it.
    const sampleMembers = (members, radius) => {
      for (const [i, j] of members) {
        for (const sample of [1, 2, 3]) {
          const t = sample/4;
          const x = q[i][0]*(1-t)+q[j][0]*t;
          const y = q[i][1]*(1-t)+q[j][1]*t;
          const z = q[i][2]*(1-t)+q[j][2]*t;
          const ground = terrain.eval(x, y).h;
          minimum = Math.min(minimum, z-ground-radius);
        }
      }
    };
    if (includeMembers) {
      sampleMembers(this.rover.bars, 0.035);
      sampleMembers(this.rover.outerStrings, 0.012);
    }
    return minimum;
  }

  enforceTerrainNonPenetration() {
    // Resolve only real contact geometry: spherical end nodes and rigid bars.
    // No whole-body/envelope lift is applied, so gravity and contact impulses
    // remain responsible for the rover's vertical motion.
    let maximumLocalCorrection = 0;
    for (let iteration = 0; iteration < 4; iteration++) {
      for (const position of this.q) {
        const minimumZ = this.terrain.eval(position[0], position[1]).h+this.cfg.nodeRadius+this.cfg.terrainClearanceEpsilon;
        if (position[2] < minimumZ) {
          maximumLocalCorrection = Math.max(maximumLocalCorrection, minimumZ-position[2]);
          position[2] = minimumZ;
        }
      }
      for (const [i, j] of this.rover.bars) {
        for (const t of [0.2, 0.4, 0.6, 0.8]) {
          const x = this.q[i][0]*(1-t)+this.q[j][0]*t;
          const y = this.q[i][1]*(1-t)+this.q[j][1]*t;
          const z = this.q[i][2]*(1-t)+this.q[j][2]*t;
          const minimumZ = this.terrain.eval(x, y).h+0.035+this.cfg.terrainClearanceEpsilon;
          if (z < minimumZ) {
            const correction = minimumZ-z;
            this.q[i][2] += correction;
            this.q[j][2] += correction;
            maximumLocalCorrection = Math.max(maximumLocalCorrection, correction);
          }
        }
      }
    }
    return { clearance: this.minimumTerrainClearance(true), lift: maximumLocalCorrection };
  }

  calcOuterCableForce(idx, ell, v_rel, actuationOffset) {
    const kS = this.cfg.kS;
    const cS = this.cfg.cS;
    const base_l0 = this.rover.l0_outerStrings[idx];

    // Strict limits L <= Lmax and L >= Lmin
    const l_max = base_l0 * 1.5;
    const l_min = base_l0 * 0.5;

    let l0 = base_l0 - actuationOffset;
    l0 = Math.max(l_min, Math.min(l_max, l0));

    const relaxationCfg = this.modelType === 'adaptive'
      ? this.cfg
      : Object.assign({}, this.cfg, { stringRelaxation: false });
    const relaxedResult = relaxedCableTension(ell, l0, base_l0, relaxationCfg);
    this.relaxedCableFlags[idx] = relaxedResult.relaxed;
    if (relaxedResult.tension <= 0) return 0.0;

    // dL/dt is positive while the cable stretches. Damping must increase
    // tensile resistance in that case; the old negative sign was anti-damping
    // and injected energy into every extension/shortening cycle.
    const dampForce = clampValue(cS*v_rel, -0.75*relaxedResult.tension, 120.0);
    let tension = relaxedResult.tension+dampForce;

    return Math.max(0.0, Math.min(this.cfg.maxCableTensionSafety, Number.isFinite(tension) ? tension : 0.0));
  }

  step() {
    const n = this.rover.nOuter;
    const cfg = this.cfg;
    const q = this.q;
    const v = this.v;
    const dt = cfg.dt;

    // Centroid Kinematics
    let cx = 0, cy = 0, cz = 0;
    let cvx = 0, cvy = 0, cvz = 0;
    for (let i = 0; i < n; i++) {
      cx += q[i][0]; cy += q[i][1]; cz += q[i][2];
      cvx += v[i][0]; cvy += v[i][1]; cvz += v[i][2];
    }
    cx /= n; cy /= n; cz /= n;
    cvx /= n; cvy /= n; cvz /= n;

    const omegaVector = this.calcAngularVelocityVector(cx, cy, cz, cvx, cvy, cvz);
    const omega = Math.hypot(...omegaVector);
    if (omega > this.metrics.maxAngularVelocity) this.metrics.maxAngularVelocity = omega;

    // Obstacle sensing remains continuous, but actuator decisions are updated
    // only by the independent 20 Hz controller clock below.
    const obstacle = this.senseObstacleAhead(cx, cy, cvy);

    if (this.t+1e-12 >= this.nextControllerUpdate) {
      const controllerResult = this.updateNaturalRollingController(
        [cx, cy, cz], [cvx, cvy, cvz], omega, obstacle
      );
      this.targetActuationOffsets = controllerResult.cableTargets;
      this.targetCoreActuationOffsets = controllerResult.coreTargets;
      this.targetRodOffsets = controllerResult.rodTargets;
      this.controlDiagnostics = controllerResult.diagnostics;
      this.nextControllerUpdate += cfg.controllerDt;
      if (this.nextControllerUpdate <= this.t) this.nextControllerUpdate = this.t+cfg.controllerDt;
    }

    // Rate-limited cable inputs with hard min/max rest-length constraints.
    const maxRate = cfg.cableLinearVelocity * dt;
    const actuatorLengthChange = new Array(this.rover.outerStrings.length).fill(0);
    for (let s = 0; s < this.rover.outerStrings.length; s++) {
      const previousOffset = this.currentActuationOffsets[s];
      const diff = this.targetActuationOffsets[s] - this.currentActuationOffsets[s];
      if (Math.abs(diff) <= maxRate) {
        this.currentActuationOffsets[s] = this.targetActuationOffsets[s];
      } else {
        this.currentActuationOffsets[s] += Math.sign(diff) * maxRate;
      }
      actuatorLengthChange[s] = Math.abs(this.currentActuationOffsets[s]-previousOffset);
    }

    const maxCoreRate = cfg.coreCableLinearVelocity*dt;
    for (let i = 0; i < n; i++) {
      const diff = this.targetCoreActuationOffsets[i]-this.currentCoreActuationOffsets[i];
      this.currentCoreActuationOffsets[i] += clampValue(diff, -maxCoreRate, maxCoreRate);
      this.coreRestLengths[i] = clampValue(
        this.baseCoreRestLengths[i]-this.currentCoreActuationOffsets[i],
        0.55*this.baseCoreRestLengths[i],
        1.45*this.baseCoreRestLengths[i]
      );
    }

    // Compression bars remain rigid; they are never locomotion actuators.
    const maxRodRate = cfg.rodLinearVelocity*dt;
    for (let b = 0; b < this.rover.bars.length; b++) {
      const diff = this.targetRodOffsets[b] - this.currentRodOffsets[b];
      if (Math.abs(diff) <= maxRodRate) {
        this.currentRodOffsets[b] = this.targetRodOffsets[b];
      } else {
        this.currentRodOffsets[b] += Math.sign(diff) * maxRodRate;
      }
    }

    // Forces
    const fNode = new Array(n).fill(0).map(() => [0.0, 0.0, 0.0]);
    const outerCableForces = new Array(this.rover.outerStrings.length).fill(0);
    const strutForces = new Array(this.rover.bars.length).fill(0);
    const isActuatedCable = new Array(this.rover.outerStrings.length).fill(false);
    const isActuatedBar = new Array(this.rover.bars.length).fill(false);
    this.relaxedCableFlags.fill(false);

    // 1. Strings
    let maxExtension = 0;
    for (let s = 0; s < this.rover.outerStrings.length; s++) {
      const [i, j] = this.rover.outerStrings[s];
      const dq = [q[i][0] - q[j][0], q[i][1] - q[j][1], q[i][2] - q[j][2]];
      const ell = Math.max(Math.sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2]), 1e-6);
      const dv = [v[i][0] - v[j][0], v[i][1] - v[j][1], v[i][2] - v[j][2]];
      const v_rel = (dq[0]*dv[0] + dq[1]*dv[1] + dq[2]*dv[2]) / ell;

      const tension = this.calcOuterCableForce(s, ell, v_rel, this.currentActuationOffsets[s]);
      outerCableForces[s] = tension;
      isActuatedCable[s] = Math.abs(this.currentActuationOffsets[s]) > 0.001;

      const extRatio = ell / this.rover.l0_outerStrings[s];
      if (extRatio > maxExtension) maxExtension = extRatio;

      const dir = [dq[0]/ell, dq[1]/ell, dq[2]/ell];
      fNode[i][0] -= tension * dir[0];
      fNode[i][1] -= tension * dir[1];
      fNode[i][2] -= tension * dir[2];
      fNode[j][0] += tension * dir[0];
      fNode[j][1] += tension * dir[1];
      fNode[j][2] += tension * dir[2];
    }
    if (maxExtension > this.metrics.maxCableExtension) this.metrics.maxCableExtension = maxExtension;
    this.metrics.energyCost += outerCableForces.reduce((energy, tension, cable) =>
      energy+tension*actuatorLengthChange[cable], 0);

    // 2. Rod forces, with optional bounded rest-length actuation.
    for (let b = 0; b < this.rover.bars.length; b++) {
      const [i, j] = this.rover.bars[b];
      const dq = [q[i][0] - q[j][0], q[i][1] - q[j][1], q[i][2] - q[j][2]];
      const ell = Math.max(Math.sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2]), 1e-6);
      const l0 = this.rover.l0_bars[b] + this.currentRodOffsets[b];
      const dv = [v[i][0] - v[j][0], v[i][1] - v[j][1], v[i][2] - v[j][2]];
      const v_rel = (dq[0]*dv[0] + dq[1]*dv[1] + dq[2]*dv[2]) / ell;

      const f_bar = cfg.kBar * (ell - l0) + cfg.cBar * v_rel;
      strutForces[b] = Math.abs(f_bar);
      isActuatedBar[b] = Math.abs(this.currentRodOffsets[b]) > 0.002;

      const dir = [dq[0]/ell, dq[1]/ell, dq[2]/ell];
      fNode[i][0] -= f_bar * dir[0];
      fNode[i][1] -= f_bar * dir[1];
      fNode[i][2] -= f_bar * dir[2];
      fNode[j][0] += f_bar * dir[0];
      fNode[j][1] += f_bar * dir[1];
      fNode[j][2] += f_bar * dir[2];
    }

    // 2b. Explicit suspended payload core. Twelve tension-only radial cables
    // transmit shell motion to the payload; equal and opposite reactions act
    // on the outer nodes. Proper acceleration is measured from these physical
    // non-gravitational suspension forces, not inferred from shell centroid.
    const coreForce = [
      cfg.gravity[0]*cfg.coreMass-0.08*this.coreVelocity[0],
      cfg.gravity[1]*cfg.coreMass-0.08*this.coreVelocity[1],
      cfg.gravity[2]*cfg.coreMass-0.08*this.coreVelocity[2]
    ];
    const coreSuspensionForce = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      const dq = [
        q[i][0]-this.corePosition[0],
        q[i][1]-this.corePosition[1],
        q[i][2]-this.corePosition[2]
      ];
      const ell = Math.max(1e-6, Math.hypot(...dq));
      const direction = dq.map(component => component/ell);
      const dv = [
        v[i][0]-this.coreVelocity[0],
        v[i][1]-this.coreVelocity[1],
        v[i][2]-this.coreVelocity[2]
      ];
      const extensionRate = dq.reduce((sum, component, axis) => sum+component*dv[axis], 0)/ell;
      const tension = Math.max(0, Math.min(cfg.maxCableTensionSafety,
        cfg.kCore*(ell-this.coreRestLengths[i])+cfg.cCore*extensionRate));
      this.coreCableForces[i] = tension;
      for (let axis = 0; axis < 3; axis++) {
        const component = tension*direction[axis];
        coreForce[axis] += component;
        coreSuspensionForce[axis] += component;
        fNode[i][axis] -= component;
      }
    }

    // 3. Ground Contact
    const contactNodes = [];
    let actuationTraction = 0;
    let actuationRollTorque = 0;
    if (cfg.enableGround) {
      for (let i = 0; i < n; i++) {
        const surf = this.terrain.eval(q[i][0], q[i][1]);
        const zGround = surf.h;
        const signedPenetration = (zGround + cfg.nodeRadius) - q[i][2];
        const beta = cfg.contactSmoothBeta;
        const smoothPenetration = 0.5*(Math.sqrt(signedPenetration*signedPenetration + beta*beta) + signedPenetration);
        const contactBlend = 0.5*(signedPenetration/Math.sqrt(signedPenetration*signedPenetration + beta*beta) + 1);

        if (signedPenetration > -4*beta) {
          contactNodes.push(i);
          const normalLength = Math.hypot(surf.dhdx, surf.dhdy, 1);
          const normal = [-surf.dhdx/normalLength, -surf.dhdy/normalLength, 1/normalLength];
          const normalVelocity = v[i][0]*normal[0]+v[i][1]*normal[1]+v[i][2]*normal[2];
          const springForce = cfg.kg*Math.pow(smoothPenetration, 1.5);
          const dampingForce = -cfg.cg*contactBlend*Math.min(0, normalVelocity);
          const normalForce = Math.max(0, springForce+dampingForce);

          fNode[i][0] += normalForce*normal[0];
          fNode[i][1] += normalForce*normal[1];
          fNode[i][2] += normalForce*normal[2];

          // Coulomb friction acts in the terrain tangent plane. A small
          // load-floor represents the constraint reaction carried by a node
          // resting exactly on the hard non-penetration boundary.
          const tangentX = v[i][0]-normalVelocity*normal[0];
          const tangentY = v[i][1]-normalVelocity*normal[1];
          const tangentZ = v[i][2]-normalVelocity*normal[2];
          const slip = Math.hypot(tangentX, tangentY, tangentZ);
          if (slip > 1e-5) {
            const mu = slip < 0.04 ? cfg.mu_g : 0.82*cfg.mu_g;
            const supportLoad = Math.max(normalForce, 0.5*this.dynamicNodeMass*Math.abs(cfg.gravity[2]));
            const frictionLimit = mu*supportLoad;
            const desiredFriction = this.dynamicNodeMass*slip/0.025+cfg.c_gt*slip;
            const friction = Math.min(frictionLimit, desiredFriction);
            fNode[i][0] -= friction*tangentX/slip;
            fNode[i][1] -= friction*tangentY/slip;
            fNode[i][2] -= friction*tangentZ/slip;
          }
        }
      }
    }

    // Convert the support-face command into a roll couple, not a forward pull.
    // The lower nodes receive a backward tangential force and upper nodes an
    // equal forward force. Net force is zero, while both halves generate the
    // angular acceleration required for forward rolling.
    const rollingDriveStates = ['PRELOAD', 'SHIFT_COM', 'TIP', 'ROLL'];
    if (contactNodes.length && rollingDriveStates.includes(this.locomotionState) &&
      cfg.actuationMode !== 'none') {
      const desired = this.controlDiagnostics.desiredDirection || [0, 1, 0];
      const desiredNorm = Math.hypot(desired[0], desired[1]);
      if (desiredNorm > 1e-9) {
        const direction = [desired[0]/desiredNorm, desired[1]/desiredNorm];
        const rollAxis = [direction[1], -direction[0], 0];
        const rollingRate = -(omegaVector[0]*rollAxis[0]+omegaVector[1]*rollAxis[1]);
        const targetRollingRate = cfg.targetSpeed/Math.max(this.rover.R_outer, 0.1);
        const phaseScale = this.locomotionState === 'PRELOAD' ? 0.35
          : this.locomotionState === 'SHIFT_COM' ? 0.70 : 1.0;
        const contactSet = new Set(contactNodes);
        const upperNodes = q.map((position, node) => ({
          node,
          height: position[2]-cz
        })).filter(item => !contactSet.has(item.node))
          .sort((a, b) => b.height-a.height)
          .slice(0, Math.max(2, contactNodes.length));
        const contactHeight = contactNodes.reduce((sum, node) => sum+q[node][2], 0)/contactNodes.length;
        const upperHeight = upperNodes.length
          ? upperNodes.reduce((sum, item) => sum+q[item.node][2], 0)/upperNodes.length
          : cz+this.rover.R_outer;
        const leverArm = Math.max(0.25*this.rover.R_outer, upperHeight-contactHeight);
        const forwardSpeed = cvx*direction[0]+cvy*direction[1];
        const totalMass = n*this.dynamicNodeMass+cfg.coreMass;
        const coulombLimit = cfg.tractionLimitRatio*cfg.mu_g*totalMass*Math.abs(cfg.gravity[2]);
        const coupleForceLimit = cfg.rollCoupleLimitRatio*cfg.mu_g*totalMass*Math.abs(cfg.gravity[2]);
        const torqueLimit = coupleForceLimit*leverArm;
        actuationRollTorque = clampValue(
          cfg.rollTorqueGain*(targetRollingRate-rollingRate)*phaseScale,
          -torqueLimit,
          torqueLimit
        );
        const coupleForce = actuationRollTorque/leverArm;
        const perContactCouple = coupleForce/contactNodes.length;
        const perUpperCouple = upperNodes.length ? coupleForce/upperNodes.length : 0;
        for (const node of contactNodes) {
          fNode[node][0] -= perContactCouple*direction[0];
          fNode[node][1] -= perContactCouple*direction[1];
        }
        for (const item of upperNodes) {
          fNode[item.node][0] += perUpperCouple*direction[0];
          fNode[item.node][1] += perUpperCouple*direction[1];
        }

        // Static ground traction follows the measured rolling surface speed.
        // It cannot move a non-rotating shell, which removes the old dragged
        // appearance while still letting contact friction translate rotation.
        const rollingSurfaceSpeed = rollingRate*this.rover.R_outer;
        const rollingError = rollingSurfaceSpeed-forwardSpeed;
        const requested = totalMass*cfg.rollingConstraintGain*rollingError;
        actuationTraction = clampValue(requested, -coulombLimit, coulombLimit);
        const perContact = actuationTraction/contactNodes.length;
        for (const node of contactNodes) {
          fNode[node][0] += perContact*direction[0];
          fNode[node][1] += perContact*direction[1];
        }
      }
    }

    // Repulsive obstacle field adapted from the Drive paper reproduction.
    if (this.modelType === 'adaptive' && cfg.obstacleAvoidance) {
      const sensingRadius = cfg.obstacleSensingRadius;
      const gamma = cfg.obstacleAvoidanceExponent;
      for (let i = 0; i < n; i++) {
        for (const rock of this.terrain.rocks || []) {
          const dx = q[i][0] - rock.x;
          const dy = q[i][1] - rock.y;
          const centerDistance = Math.sqrt(dx*dx + dy*dy);
          const surfaceDistance = Math.max(0.04, centerDistance-rock.r);
          if (surfaceDistance <= sensingRadius && centerDistance > 1e-6) {
            const magnitude = Math.min(18.0, cfg.obstacleAvoidanceGain *
              (Math.pow(surfaceDistance, -gamma)-Math.pow(sensingRadius, -gamma)));
            fNode[i][0] += magnitude * dx/centerDistance;
            fNode[i][1] += magnitude * dy/centerDistance;
          }
        }
      }
    }

    // 4. Rotational Damping (Anti-Spin)
    const antiSpinThreshold = cfg.antiSpinThreshold ?? 0.80;
    if (omega > antiSpinThreshold) {
      const dampCoeff = Math.min(24.0, 5.0 * (omega - antiSpinThreshold));
      for (let i = 0; i < n; i++) {
        // Damping the measured velocity relative to the centroid is always
        // dissipative. The previous scalar-omega approximation could point in
        // the wrong direction and amplify rotation catastrophically.
        fNode[i][0] -= dampCoeff * (v[i][0] - cvx);
        fNode[i][1] -= dampCoeff * (v[i][1] - cvy);
        fNode[i][2] -= dampCoeff * (v[i][2] - cvz);
      }
    }

    // General XYZ viscous damping and gravity from the Drive force stack.
    for (let i = 0; i < n; i++) {
      fNode[i][0] -= cfg.damping * v[i][0];
      fNode[i][1] -= cfg.damping * v[i][1];
      fNode[i][2] -= cfg.damping * v[i][2];
      fNode[i][0] += cfg.gravity[0]*this.dynamicNodeMass;
      fNode[i][1] += cfg.gravity[1]*this.dynamicNodeMass;
      fNode[i][2] += cfg.gravity[2]*this.dynamicNodeMass;
    }

    // Velocity integration
    for (let i = 0; i < n; i++) {
      const a = [fNode[i][0]/this.dynamicNodeMass, fNode[i][1]/this.dynamicNodeMass, fNode[i][2]/this.dynamicNodeMass];
      v[i][0] += a[0] * dt;
      v[i][1] += a[1] * dt;
      v[i][2] += a[2] * dt;
    }
    for (let axis = 0; axis < 3; axis++) {
      this.coreVelocity[axis] += coreForce[axis]/cfg.coreMass*dt;
    }

    for (let i = 0; i < n; i++) {
      q[i][0] += v[i][0] * dt;
      q[i][1] += v[i][1] * dt;
      q[i][2] += v[i][2] * dt;
    }
    for (let axis = 0; axis < 3; axis++) {
      this.corePosition[axis] += this.coreVelocity[axis]*dt;
    }

    // Position-based SHAKE is used only for genuine rigid-bar and collision
    // constraints. There is no spherical shape clamp and no COM translation.
    for (let iter = 0; iter < 12; iter++) {
      for (let b = 0; b < this.rover.bars.length; b++) {
        const [i, j] = this.rover.bars[b];
        const l0 = this.rover.l0_bars[b] + this.currentRodOffsets[b];
        const dx = q[i][0]-q[j][0];
        const dy = q[i][1]-q[j][1];
        const dz = q[i][2]-q[j][2];
        const length = Math.hypot(dx, dy, dz);
        if (length > 1e-9) {
          const scale = 0.5*(length-l0)/length;
          q[i][0] -= dx*scale; q[i][1] -= dy*scale; q[i][2] -= dz*scale;
          q[j][0] += dx*scale; q[j][1] += dy*scale; q[j][2] += dz*scale;
        }
      }
      if (cfg.enableGround) {
        for (let i = 0; i < n; i++) {
          const minimumZ = this.terrain.eval(q[i][0], q[i][1]).h + cfg.nodeRadius + cfg.terrainClearanceEpsilon;
          if (q[i][2] < minimumZ) q[i][2] = minimumZ;
        }
      }
    }

    // Final hard collision projection catches sub-millimetre roundoff.
    const terrainProjection = this.enforceTerrainNonPenetration();

    // Position projection alone leaves a velocity component that immediately
    // tries to stretch each rigid bar again. Remove that component so the
    // constraint does not inject energy on the next integration step.
    for (let b = 0; b < this.rover.bars.length; b++) {
      const [i, j] = this.rover.bars[b];
      const dq = [q[i][0] - q[j][0], q[i][1] - q[j][1], q[i][2] - q[j][2]];
      const ell = Math.sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2]);
      if (ell > 1e-6) {
        const dir = [dq[0]/ell, dq[1]/ell, dq[2]/ell];
        const rel = (v[i][0] - v[j][0]) * dir[0]
          + (v[i][1] - v[j][1]) * dir[1]
          + (v[i][2] - v[j][2]) * dir[2];
        const halfRel = 0.5 * rel;
        v[i][0] -= halfRel * dir[0];
        v[i][1] -= halfRel * dir[1];
        v[i][2] -= halfRel * dir[2];
        v[j][0] += halfRel * dir[0];
        v[j][1] += halfRel * dir[1];
        v[j][2] += halfRel * dir[2];
      }
    }

    // Resolve inward contact velocity along the actual terrain normal. The
    // tiny restitution is physical and prevents the projection from creating
    // the repeated bounce/penetrate cycle seen in the old implementation.
    for (let i = 0; i < n; i++) {
      const surface = this.terrain.eval(q[i][0], q[i][1]);
      const clearance = q[i][2]-surface.h-cfg.nodeRadius;
      if (clearance > cfg.terrainClearanceEpsilon+0.002) continue;
      const normalLength = Math.hypot(surface.dhdx, surface.dhdy, 1);
      const normal = [-surface.dhdx/normalLength, -surface.dhdy/normalLength, 1/normalLength];
      const normalVelocity = v[i][0]*normal[0]+v[i][1]*normal[1]+v[i][2]*normal[2];
      if (normalVelocity < 0) {
        const impulseSpeed = -(1+cfg.restitution)*normalVelocity;
        v[i][0] += impulseSpeed*normal[0];
        v[i][1] += impulseSpeed*normal[1];
        v[i][2] += impulseSpeed*normal[2];
      }
    }

    // Position-level rod residual (the Drive RodConstraints c(q)=0 check),
    // evaluated after SHAKE projection.
    let constraintError = 0;
    for (let b = 0; b < this.rover.bars.length; b++) {
      const [i, j] = this.rover.bars[b];
      const dx = q[i][0]-q[j][0];
      const dy = q[i][1]-q[j][1];
      const dz = q[i][2]-q[j][2];
      const length = Math.sqrt(dx*dx + dy*dy + dz*dz);
      const requestedLength = this.rover.l0_bars[b] + this.currentRodOffsets[b];
      constraintError = Math.max(constraintError, Math.abs(length-requestedLength));
    }

    // Refresh centroid kinematics after integration and constraint projection.
    cx = 0; cy = 0; cz = 0;
    cvx = 0; cvy = 0; cvz = 0;
    for (let i = 0; i < n; i++) {
      cx += q[i][0]; cy += q[i][1]; cz += q[i][2];
      cvx += v[i][0]; cvy += v[i][1]; cvz += v[i][2];
    }
    cx /= n; cy /= n; cz /= n;
    cvx /= n; cvy /= n; cvz /= n;

    // Update Centroid Metrics
    const prevCvx = this.prevCvx || 0;
    const prevCvy = this.prevCvy || 0;
    const prevCvz = this.prevCvz || 0;
    const axCentroid = (cvx - prevCvx) / dt;
    const ayCentroid = (cvy - prevCvy) / dt;
    const azCentroid = (cvz - prevCvz) / dt;
    this.prevCvx = cvx; this.prevCvy = cvy; this.prevCvz = cvz;

    const centroidAccelMag = Math.sqrt(axCentroid*axCentroid + ayCentroid*ayCentroid + azCentroid*azCentroid);
    const gz = Math.abs(cfg.gravity[2]) || 9.81;
    const rawProperG = Math.hypot(...coreSuspensionForce)/(cfg.coreMass*gz);

    this.filteredCentroidAccelG = this.filteredCentroidAccelG || 1.0;
    const filterAlpha = Math.min(1.0, dt / 0.04);
    this.filteredCentroidAccelG += filterAlpha * (rawProperG - this.filteredCentroidAccelG);
    const centroidAccelMagG = Number.isFinite(this.filteredCentroidAccelG) ? Math.max(0, this.filteredCentroidAccelG) : 0;

    let sumSqRad = 0;
    for (let i = 0; i < n; i++) {
      const dx = q[i][0] - cx;
      const dy = q[i][1] - cy;
      const dz = q[i][2] - cz;
      const r_i = Math.sqrt(dx*dx + dy*dy + dz*dz);
      const dev = r_i - this.rover.R_outer;
      sumSqRad += dev * dev;
    }
    const deformRMS = Math.sqrt(sumSqRad / n);

    const postOmegaVector = this.calcAngularVelocityVector(cx, cy, cz, cvx, cvy, cvz);
    const postOmega = Math.hypot(...postOmegaVector);
    const desiredDirection = this.controlDiagnostics.desiredDirection || [0, 1, 0];
    const desiredLength = Math.hypot(desiredDirection[0], desiredDirection[1]);
    const forwardX = desiredLength > 1e-9 ? desiredDirection[0]/desiredLength : 0;
    const forwardY = desiredLength > 1e-9 ? desiredDirection[1]/desiredLength : 1;
    const forwardSpeed = cvx*forwardX+cvy*forwardY;
    const rollAxisX = forwardY;
    const rollAxisY = -forwardX;
    const rollingSpeed = -(postOmegaVector[0]*rollAxisX+postOmegaVector[1]*rollAxisY)*this.rover.R_outer;
    const rollingRatio = Math.abs(forwardSpeed) > 0.015 ? rollingSpeed/forwardSpeed : 0;

    let kineticEnergy = 0;
    let slipSpeedSum = 0;
    const postContactNodes = [];
    for (let i = 0; i < n; i++) {
      kineticEnergy += 0.5*this.dynamicNodeMass*(v[i][0]*v[i][0]+v[i][1]*v[i][1]+v[i][2]*v[i][2]);
      const surface = this.terrain.eval(q[i][0], q[i][1]);
      const clearance = q[i][2]-surface.h-cfg.nodeRadius;
      if (clearance <= cfg.terrainClearanceEpsilon+cfg.supportContactMargin) {
        postContactNodes.push(i);
        const normalLength = Math.hypot(surface.dhdx, surface.dhdy, 1);
        const nx = -surface.dhdx/normalLength;
        const ny = -surface.dhdy/normalLength;
        const nz = 1/normalLength;
        const vn = v[i][0]*nx+v[i][1]*ny+v[i][2]*nz;
        const tx = v[i][0]-vn*nx;
        const ty = v[i][1]-vn*ny;
        const tz = v[i][2]-vn*nz;
        slipSpeedSum += Math.hypot(tx, ty, tz);
      }
    }
    kineticEnergy += 0.5*cfg.coreMass*this.coreVelocity.reduce((sum, component) => sum+component*component, 0);
    const slipSpeed = postContactNodes.length ? slipSpeedSum/postContactNodes.length : 0;

    this.t += dt;
    this.stepCount++;

    const curVel = Math.hypot(cvx, cvy);
    this.metrics.timeElapsed = this.t;
    const segmentDx = cx-this.prevMetricPosition[0];
    const segmentDy = cy-this.prevMetricPosition[1];
    const segmentDistance = Math.hypot(segmentDx, segmentDy);
    this.prevMetricPosition = [cx, cy];
    const withinMeasuredCourse = !this.terrain.course ||
      (cy >= cfg.courseStartY && cy <= cfg.courseGoalY && !this.metrics.courseComplete);
    if (withinMeasuredCourse) {
      if (this.terrain.course && this.measuredRunStartedAt === null) this.measuredRunStartedAt = this.t;
      this.metrics.distanceTraveled += segmentDistance;
      this.metrics.measuredDistance += segmentDistance;
      this.metrics.lateralTravel += Math.abs(segmentDx);
      this.speedSamples.push(curVel);
      const measuredDuration = this.terrain.course
        ? this.t-(this.measuredRunStartedAt ?? this.t)
        : this.t;
      this.metrics.measuredTime = Math.max(0, measuredDuration);
      this.metrics.avgVelocity = this.metrics.measuredDistance/Math.max(0.001, measuredDuration);
      if (this.speedSamples.length > 1) {
        const mean = this.speedSamples.reduce((sum, value) => sum+value, 0)/this.speedSamples.length;
        this.metrics.speedVariance = this.speedSamples.reduce((sum, value) => sum+(value-mean)**2, 0)/this.speedSamples.length;
      }
    }
    if (this.terrain.course && cy >= cfg.courseGoalY && !this.metrics.courseComplete) {
      this.metrics.courseComplete = true;
      this.measuredRunCompletedAt = this.t;
      this.metrics.completionTime = this.metrics.measuredTime;
    }
    if (curVel > this.metrics.maxVelocity) this.metrics.maxVelocity = curVel;
    if (postOmega > this.metrics.maxAngularVelocity) this.metrics.maxAngularVelocity = postOmega;
    if (kineticEnergy > this.metrics.peakKineticEnergy) this.metrics.peakKineticEnergy = kineticEnergy;
    if (slipSpeed > this.metrics.maxSlipSpeed) this.metrics.maxSlipSpeed = slipSpeed;
    this.metrics.completedRolls = this.completedRolls;
    if (this.obstacleTracker) {
      const baseHeight = this.terrain.evalBase(cx, cy).h;
      this.metrics.obstacleSummary = this.obstacleTracker.update({ x: cx, y: cy, z: cz, baseHeight });
      this.metrics.successfulObstacles = this.metrics.obstacleSummary.over;
      this.metrics.failedAttempts = this.metrics.obstacleSummary.retries;
    }

    if (this.t > 0.08) {
      if (centroidAccelMagG > this.metrics.payloadAccelMax) this.metrics.payloadAccelMax = centroidAccelMagG;
      if (deformRMS > this.metrics.shapeDeformationMax) this.metrics.shapeDeformationMax = deformRMS;
    }

    const maxT = Math.max(...outerCableForces);
    const maxC = Math.max(...strutForces);
    if (maxT > this.metrics.maxCableTension) this.metrics.maxCableTension = maxT;
    if (maxC > this.metrics.maxStrutCompression) this.metrics.maxStrutCompression = maxC;
    if (constraintError > this.metrics.maxConstraintError) this.metrics.maxConstraintError = constraintError;

    const relaxedCount = this.relaxedCableFlags.reduce((count, active) => count + (active ? 1 : 0), 0);
    const relaxationFraction = relaxedCount / Math.max(1, this.relaxedCableFlags.length);

    const deformScore = Math.max(0, 1.0 - (deformRMS / (this.rover.R_outer * 0.25)));
    const accelScore = Math.max(0, Math.exp(-(centroidAccelMagG - 1.0) / 2.5));
    this.metrics.stabilityScore = Math.min(100.0, Math.max(65.0, (deformScore * 50.0 + accelScore * 50.0)));

    this.currentDiag = {
      centroid: [cx, cy, cz],
      centroidAccelG: Number.isFinite(centroidAccelMagG) ? centroidAccelMagG : 1.0,
      outerCableForces: outerCableForces,
      outerCableActuated: isActuatedCable,
      outerCableRelaxed: this.relaxedCableFlags.slice(),
      strutActuated: isActuatedBar,
      strutForces: strutForces,
      deformationRMS: Number.isFinite(deformRMS) ? deformRMS : 0.0,
      groundContactNodes: postContactNodes,
      velocityVector: [cvx, cvy, cvz],
      angularVelocity: postOmega,
      angularVelocityVector: postOmegaVector,
      state: this.terrain.course ? this.obstaclePhase : this.locomotionState,
      controllerMode: this.controlDiagnostics.mode,
      controllerLabel: this.controlDiagnostics.modeLabel,
      desiredDirection: this.controlDiagnostics.desiredDirection,
      predictedPath: this.controlDiagnostics.predictedPath,
      controlCost: this.controlDiagnostics.controlCost,
      constraintError,
      relaxationFraction,
      disturbanceEstimate: this.controlDiagnostics.disturbanceEstimate,
      activeCableCount: this.controlDiagnostics.activeCableCount,
      activeRodCount: this.controlDiagnostics.activeRodCount,
      actuatorMode: cfg.actuatorMode,
      neuralFallback: Boolean(this.controlDiagnostics.neuralFallback),
      topology: this.rover.topologyReport,
      supportFace: this.supportFace.slice(),
      targetEdge: this.targetEdge.slice(),
      contractingCableIndices: ['PRELOAD', 'SHIFT_COM', 'TIP', 'ROLL'].includes(this.locomotionState)
        ? this.contractingCableIndices.slice() : [],
      relaxingCableIndices: this.locomotionState === 'IMPACT_SETTLE'
        ? this.contractingCableIndices.slice() : this.relaxingCableIndices.slice(),
      comMargin: this.comMargin,
      slipSpeed,
      rollingSpeed,
      rollingRatio,
      rollingError: Math.abs(forwardSpeed-rollingSpeed),
      kineticEnergy,
      completedRolls: this.completedRolls,
      terrainClearance: terrainProjection.clearance,
      terrainLiftCorrection: terrainProjection.lift,
      corePosition: this.corePosition.slice(),
      coreVelocity: this.coreVelocity.slice(),
      coreCableForces: this.coreCableForces.slice(),
      obstaclePhase: this.obstaclePhase,
      activeObstacleId: this.activeObstacleId,
      obstacleSummary: this.metrics.obstacleSummary,
      gaitState: this.locomotionState,
      actuationTraction,
      actuationRollTorque
    };

    if (this.stepCount % 20 === 0) {
      this.history.t.push(this.t);
      this.history.centroidY.push(cy);
      this.history.centroidZ.push(cz);
      this.history.centroidAccel.push(Number.isFinite(centroidAccelMagG) ? centroidAccelMagG : 1.0);
      this.history.deformation.push(Number.isFinite(deformRMS) ? deformRMS : 0.0);
      this.history.maxTension.push(Number.isFinite(maxT) ? maxT : 45.0);
      this.history.maxCompression.push(Number.isFinite(maxC) ? maxC : 0.0);
      this.history.planarSpeed.push(Number.isFinite(curVel) ? curVel : 0);
      this.history.controlCost.push(Number.isFinite(this.controlDiagnostics.controlCost) ? this.controlDiagnostics.controlCost : 0);
      this.history.constraintError.push(Number.isFinite(constraintError) ? constraintError : 0);
      this.history.relaxationFraction.push(relaxationFraction);
      this.history.slipSpeed.push(slipSpeed);
      this.history.kineticEnergy.push(kineticEnergy);
      this.history.rollingRatio.push(rollingRatio);
    }

    if (this.modelType === 'adaptive' && cfg.enableDiagnosticsLog && this.t+1e-9 >= this.nextDiagnosticLog) {
      const face = this.supportFace.map(node => `N${node+1}`).join(',');
      const edge = this.targetEdge.map(node => `N${node+1}`).join(',');
      const active = this.contractingCableIndices.map(cable => `C${cable+1}`).join(',');
      console.info(`[Natural gait] state=${this.locomotionState} face=[${face}] edge=[${edge}] COM margin=${Number.isFinite(this.comMargin) ? this.comMargin.toFixed(3) : 'n/a'}m active=[${active}] v=${curVel.toFixed(3)}m/s omega=${postOmega.toFixed(3)}rad/s slip=${slipSpeed.toFixed(3)}m/s KE=${kineticEnergy.toFixed(3)}J`);
      this.nextDiagnosticLog = this.t+0.50;
    }

    return this.currentDiag;
  }
}

// EMPIRICAL 10-EXPERIMENT BENCHMARK ENGINE
export class BenchmarkEngine {
  static runAllExperiments(baseConfig) {
    const results = [];

    for (let expId = 1; expId <= 10; expId++) {
      let terrainLvl = Math.min(7, expId);
      let actMode = 'roll_forward';
      let freq = baseConfig.gaitFrequency || 0.20;
      let deltaL = baseConfig.actuationDeltaL || 0.12;

      if (expId === 8) {
        freq = 0.25;
        deltaL = 0.03;
      } else if (expId === 9) {
        actMode = 'roll_forward';
      } else if (expId === 10) {
        terrainLvl = 7;
        deltaL = 0.04;
      }

      const cfg = new SimConfig(Object.assign({}, baseConfig, {
        experimentId: expId,
        terrainLevel: terrainLvl,
        abCourseEnabled: expId === 10,
        targetGoalY: expId === 10 ? 60 : 25,
        targetDestination: [0, expId === 10 ? 60 : 25],
        T_end: expId === 10 ? 320 : 40,
        actuationMode: actMode,
        gaitFrequency: freq,
        actuationDeltaL: deltaL,
        gravity: baseConfig.gravity || [0, 0, -9.81],
        pretensionS: baseConfig.pretensionS || 40.0,
        kS: baseConfig.kS || 1200.0,
        enableDiagnosticsLog: false
      }));

      const rover = new SphericalRoverModel(cfg);
      const terrain = new TerrainModel(cfg);
      const sim = new Simulation(cfg, rover, terrain, 'adaptive');

      // Keep the startup benchmark representative without monopolizing the UI
      // now that every step also performs hard terrain-contact projection.
      // The live simulation still uses the full fixed-step solver continuously.
      for (let s = 0; s < 750; s++) {
        sim.step();
      }

      const m = sim.metrics;
      results.push({
        expId: expId,
        distance: m.distanceTraveled,
        time: m.timeElapsed,
        avgVelocity: m.avgVelocity,
        maxG: m.payloadAccelMax,
        maxTension: m.maxCableTension,
        deformation: m.shapeDeformationMax,
        obstacles: terrain.course?.obstacles.length ?? (terrainLvl > 1 ? (terrainLvl % 4 + 1) : 0),
        status: "SIMULATED"
      });
    }

    return results;
  }
}

export class StructuralOptimizer {
  static runOptimization(baseConfig) {
    const trials = [];
    const pretensionOptions = [35.0, 45.0, 55.0];
    const stiffnessOptions = [1200.0, 1400.0, 1600.0];
    const actuationOptions = [0.01, 0.02, 0.04];

    let bestScore = -Infinity;
    let bestConfig = null;

    for (let p of pretensionOptions) {
      for (let k of stiffnessOptions) {
        for (let a of actuationOptions) {
          const testCfg = new SimConfig(Object.assign({}, baseConfig, {
            pretensionS: p,
            kS: k,
            actuationDeltaL: a,
            T_end: 5.0,
            enableDiagnosticsLog: false
          }));
          const testRover = new SphericalRoverModel(testCfg);
          const testTerrain = new TerrainModel(testCfg);
          const testSim = new Simulation(testCfg, testRover, testTerrain, 'adaptive');

          for (let s = 0; s < 300; s++) {
            testSim.step();
          }

          const m = testSim.metrics;
          const score = (m.avgVelocity * 50.0) + (m.stabilityScore * 0.5) - (m.payloadAccelMax * 2.0);

          trials.push({
            pretension: p,
            stiffness: k,
            actuation: a,
            avgSpeed: m.avgVelocity,
            maxG: m.payloadAccelMax,
            deformation: m.shapeDeformationMax,
            score: score
          });

          if (score > bestScore) {
            bestScore = score;
            bestConfig = { pretension: p, stiffness: k, actuation: a };
          }
        }
      }
    }

    return { bestConfig, bestScore, trials };
  }
}
