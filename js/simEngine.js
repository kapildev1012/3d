import {
  ADVANCED_CONTROLLER_LABELS,
  ADVANCED_MODES,
  solveAdvancedController
} from './advancedControllers.js?v=20260822-grip1';
import { relaxedCableTension } from './driveControllers.js?v=20260822-grip1';
import {
  createABCourse,
  evaluateCourseObstacle,
  senseCourseObstacle,
  obstacleActuation,
  ObstaclePassTracker
} from './abExperiment.js?v=20260822-grip1';
import { mergeMonitoringSettings, RealtimeMonitor } from './monitoringSystem.js?v=20260822-grip1';

export const LEVEL10_PERFORMANCE = Object.freeze({
  targetSpeed: 1.30,
  timeLimit: 120.0,
  groundRMS: 0.06,
  controllerDt: 0.02,
  cableLinearVelocity: 0.32,
  coreCableLinearVelocity: 0.20,
  actuatorTau: 0.06,
  passiveSettlingDuration: 0.35,
  maxPassiveSettlingDuration: 0.65,
  stableHoldDuration: 0.05,
  preloadDuration: 0.08,
  shiftComDuration: 0.18,
  tipDuration: 0.16,
  rollTimeout: 0.28,
  impactSettleDuration: 0.08,
  antiSpinThreshold: 2.80,
  rollingConstraintGain: 42.0,
  tractionLimitRatio: 2.50,
  rollTorqueGain: 200.0,
  rollCoupleLimitRatio: 3.00,
  mu_g: 6.00,
  c_gt: 20.0,
  courseSpeedGain: 70.0,
  courseGradeCompensationGain: 1.35,
  adaptiveContactGrip: 0.88,
  stallWindow: 6.0
});

/**
 * SIMENGINE.JS - Physics & Locomotion Simulation Engine for 6-Bar Tensegrity Icosahedron Rover
 *
 * 1. Verified 6-bar / 24-cable expanded-icosahedron topology
 * 2. Fixed-step semi-implicit integration with rigid-strut SHAKE projection
 * 3. Tension-only spring/damper cables with rate-limited rest lengths
 * 4. Fixed-rate support-face gait state machine with passive settling
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
    this.controllerInputPenalty = 0.35; // Drive-solver input-change penalty weight
    this.targetSpeed = 0.20;
    this.targetDestination = [0.0, 25.0];
    this.payloadTargetHeight = 0.55;
    this.measurementNoise = 0.002;
    this.disturbanceBound = 0.02;

    // Rolling physics-history window (samples arrive every 20 steps). Long
    // expeditions would otherwise grow 14 series without bound.
    this.maxHistorySamples = 4000;

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
    // state changes occur only on the configured controller clock.
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

    // The A/B worlds are two independent physical corridors. Simulation
    // coordinates remain lane-local (centre x=0); the visualizer applies the
    // signed world offset. A hard envelope is a final safety constraint,
    // while the controller steers back toward its own centreline before the
    // shell reaches that envelope.
    this.modelLaneOffset = 1.50;
    this.pathCorridorHalfWidth = 1.25;
    this.pathCenteringGain = 1.65;
    this.pathCenteringDamping = 0.90;
    this.corridorSafetyMargin = 0.015;
    this.adaptiveContactGrip = 0.72;

    // Terrain & Goal Configuration
    this.terrainLevel = 1;        // 1: Smooth, 2: Small Rocks, 3: Medium Rocks, 4: Large Rocks, 5: Crater, 6: Steep Slope, 7: Irregular Mars
    this.targetGoalY = 25.0;      // Endpoint goal target distance [m]
    this.groundRMS = 0.06;        // Physical Mars roughness RMS [m]
    this.seed = 42;

    // Controlled A-vs-B course (Level 10).
    this.abCourseEnabled = false;
    this.courseStartY = 10.0;
    this.courseGoalY = 60.0;
    this.courseMaxY = 70.0;
    this.courseMaxRetries = 2;
    this.missionDeadlineSeconds = 120.0;
    this.stallWindow = 5.0;
    this.courseSpeedGain = 0.0;
    this.courseGradeCompensationGain = 0.0;

    // Current Active Experiment
    this.experimentId = 1;

    const monitoringOverrides = overrides.monitoring || {};
    Object.assign(this, overrides);
    this.monitoring = mergeMonitoringSettings(monitoringOverrides);
  }

  applyLevel10PerformanceProfile() {
    Object.assign(this, LEVEL10_PERFORMANCE, { T_end: LEVEL10_PERFORMANCE.timeLimit });
    return this;
  }

  applyStandardPerformanceProfile() {
    Object.assign(this, {
      // Every terrain level starts with the same locomotion capability as
      // Level 10. Terrain difficulty, not an artificial low speed, now
      // distinguishes the experiments.
      targetSpeed: LEVEL10_PERFORMANCE.targetSpeed,
      T_end: 40.0,
      controllerDt: 0.05,
      cableLinearVelocity: 0.10,
      coreCableLinearVelocity: 0.08,
      actuatorTau: 0.20,
      passiveSettlingDuration: 1.8,
      maxPassiveSettlingDuration: 3.0,
      stableHoldDuration: 0.30,
      preloadDuration: 0.35,
      shiftComDuration: 1.10,
      tipDuration: 1.10,
      rollTimeout: 1.00,
      impactSettleDuration: 0.50,
      antiSpinThreshold: 0.65,
      rollingConstraintGain: 26.0,
      tractionLimitRatio: 0.75,
      rollTorqueGain: 30.0,
      rollCoupleLimitRatio: 0.85,
      mu_g: 1.15,
      c_gt: 12.0,
      adaptiveContactGrip: 0.78,
      courseSpeedGain: 0.0,
      courseGradeCompensationGain: 0.0,
      stallWindow: 5.0
    });
    return this;
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

// ── Deterministic gradient (Perlin-style) noise with exact derivatives ──
// Lattice points carry one of eight fixed unit gradients selected by an
// integer hash; cells blend them through the quintic fade so the field is
// C2-continuous — the physics solver needs usable dhdx/dhdy everywhere.
function perlinHashGradient(ix, iy, seed) {
  let h = (Math.imul(ix, 0x27d4eb2d) ^ Math.imul(iy, 0x165667b1) ^ Math.imul(seed, 0x9e3779b1)) | 0;
  h = Math.imul(h ^ (h >>> 15), 0x85ebca6b);
  return GRADIENT_TABLE[(h ^ (h >>> 13)) & 7];
}
const GRADIENT_TABLE = Object.freeze([
  [1, 0], [-1, 0], [0, 1], [0, -1],
  [Math.SQRT1_2, Math.SQRT1_2], [-Math.SQRT1_2, Math.SQRT1_2],
  [Math.SQRT1_2, -Math.SQRT1_2], [-Math.SQRT1_2, -Math.SQRT1_2]
]);

// Single Perlin octave in unit-cell space. Returns value plus partial
// derivatives with respect to the (already scaled) inputs.
function perlinOctave(x, y, seed) {
  const ix = Math.floor(x);
  const iy = Math.floor(y);
  const fx = x-ix;
  const fy = y-iy;
  const u = fx*fx*fx*(fx*(fx*6-15)+10);
  const v = fy*fy*fy*(fy*(fy*6-15)+10);
  const du = 30*fx*fx*(fx-1)*(fx-1);
  const dv = 30*fy*fy*(fy-1)*(fy-1);

  const g00 = perlinHashGradient(ix, iy, seed);
  const g10 = perlinHashGradient(ix+1, iy, seed);
  const g01 = perlinHashGradient(ix, iy+1, seed);
  const g11 = perlinHashGradient(ix+1, iy+1, seed);
  const n00 = g00[0]*fx+g00[1]*fy;
  const n10 = g10[0]*(fx-1)+g10[1]*fy;
  const n01 = g01[0]*fx+g01[1]*(fy-1);
  const n11 = g11[0]*(fx-1)+g11[1]*(fy-1);

  const sxRow = n10-n00;
  const sxHigh = n11-n01;
  const value = (n00+u*sxRow)+(v*((n01+u*sxHigh)-(n00+u*sxRow)));
  const dhdx = du*(sxRow+v*(sxHigh-sxRow));
  const dhdy = dv*((n01+u*sxHigh)-(n00+u*sxRow));
  return { value, dhdx, dhdy };
}

// Fractal Brownian motion: summed Perlin octaves with geometric gain and
// lacunarity, derivatives carried through every octave.
function fbmNoise(x, y, field) {
  let sum = 0;
  let dx = 0;
  let dy = 0;
  let amplitude = 1;
  let frequency = field.baseFrequency;
  for (let octave = 0; octave < field.octaves; octave++) {
    const o = perlinOctave(x*frequency+octave*17.31, y*frequency-octave*9.7,
      (field.seed+octave*0x9e37) >>> 0);
    sum += amplitude*o.value;
    dx += amplitude*frequency*o.dhdx;
    dy += amplitude*frequency*o.dhdy;
    amplitude *= field.gain;
    frequency *= field.lacunarity;
  }
  return { value: sum, dhdx: dx, dhdy: dy };
}

// Ridged transform: inverted absolute value sharpens noise crests into
// jagged ridge lines. Smoothed sign keeps the derivative bounded at zero.
function ridgedFbm(x, y, field) {
  const n = fbmNoise(x, y, field);
  const softSign = Math.tanh(6*n.value);
  return {
    value: 1-Math.abs(n.value),
    dhdx: -softSign*n.dhdx,
    dhdy: -softSign*n.dhdy
  };
}
// Ten-centimetre-or-better spacing on the longest one-metre struts prevents
// the straight member between two clear endpoints from clipping a narrow
// rock crest. Feature-centre candidates below add an exact sample for every
// nearby discrete rock or obstacle.
const MEMBER_COLLISION_SAMPLES = Object.freeze([
  0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90
]);
const MEMBER_COLLISION_SKIN = 0.006;

function evaluateRockOutcrop(rock, x, y) {
  const rx = Math.max(0.04, rock.rx || rock.r || 0.2);
  const ry = Math.max(0.04, rock.ry || rock.r || 0.2);
  const dx = x-rock.x;
  const dy = y-rock.y;
  const normalizedRadiusSquared = dx*dx/(rx*rx)+dy*dy/(ry*ry);
  if (normalizedRadiusSquared >= 4) return { h: 0, dhdx: 0, dhdy: 0 };
  // Sharp angular stones use a narrower Gaussian core, which steepens the
  // contact normals near the crest without breaking C1 continuity.
  const falloff = rock.sharp ? 0.42 : 0.8;
  const factor = Math.exp(-normalizedRadiusSquared/falloff);
  const h = rock.h*factor;
  return {
    h,
    dhdx: h*(-2*dx/(falloff*rx*rx)),
    dhdy: h*(-2*dy/(falloff*ry*ry))
  };
}

/**
 * Evaluate a marsh (boggy depression) at position (x,y).
 * A marsh is a shallow, soft-edged depression with an uneven muddy floor.
 * Properties: {x, y, rx, ry, depth, muddiness, seed}
 *   depth     – maximum depression depth [m]
 *   muddiness – 0..1 controls floor roughness bumps inside the marsh
 */
function evaluateMarsh(marsh, x, y) {
  const rx = marsh.rx || 1.5;
  const ry = marsh.ry || 1.5;
  const dx = x-marsh.x;
  const dy = y-marsh.y;
  const r2 = dx*dx/(rx*rx)+dy*dy/(ry*ry);
  if (r2 >= 1.0) return { h: 0, dhdx: 0, dhdy: 0, inMarsh: false };

  // Smooth depression: deepest at centre, C1-continuous at the edge
  const edge = 1.0-r2;          // 1 at centre, 0 at boundary
  const profile = edge*edge;    // quartic ring → smooth bowl
  const depth = marsh.depth || 0.12;

  // Low-amplitude muddy ripples inside the marsh for uneven footing
  const muddiness = marsh.muddiness || 0.5;
  const freq = 6.8;
  const ripple = muddiness*0.025*(
    Math.sin(freq*dx+1.7*dy+((marsh.seed||0)*3.1))+
    0.6*Math.sin(2.3*freq*dy-0.9*dx+((marsh.seed||0)*7.3))
  )*profile;

  const h = -depth*profile+ripple;

  // Analytic partial derivatives
  const dEdgeDx = -2*dx/(rx*rx);
  const dEdgeDy = -2*dy/(ry*ry);
  const dProfileDx = 2*edge*dEdgeDx;
  const dProfileDy = 2*edge*dEdgeDy;

  const dRippleDx = muddiness*0.025*(
    freq*Math.cos(freq*dx+1.7*dy+((marsh.seed||0)*3.1))+
    0.6*(-0.9)*Math.cos(2.3*freq*dy-0.9*dx+((marsh.seed||0)*7.3))
  )*profile+ripple/Math.max(1e-6,profile)*dProfileDx*0.5;

  const dRippleDy = muddiness*0.025*(
    1.7*Math.cos(freq*dx+1.7*dy+((marsh.seed||0)*3.1))+
    0.6*2.3*freq*Math.cos(2.3*freq*dy-0.9*dx+((marsh.seed||0)*7.3))
  )*profile+ripple/Math.max(1e-6,profile)*dProfileDy*0.5;

  return {
    h,
    dhdx: -depth*dProfileDx+dRippleDx,
    dhdy: -depth*dProfileDy+dRippleDy,
    inMarsh: true
  };
}

/**
 * Evaluate a mountain peak at position (x,y).
 * Mountains are large Gaussian mounds with optional ridge extensions.
 * Properties: {x, y, rx, ry, peakHeight, ridgeAngle, ridgeLength, ridgeFalloff}
 */
function evaluateMountain(mountain, x, y) {
  const rx = mountain.rx || 30;
  const ry = mountain.ry || 30;
  const dx = x - mountain.x;
  const dy = y - mountain.y;
  const r2 = dx*dx/(rx*rx) + dy*dy/(ry*ry);
  if (r2 >= 9) return { h: 0, dhdx: 0, dhdy: 0 }; // cutoff at 3*radius

  const peak = mountain.peakHeight || 15;
  const factor = Math.exp(-r2 * 0.6);
  const h_base = peak * factor;

  // Ridge extension: elongated Gaussian along ridgeAngle
  let h_ridge = 0, dRidgeDx = 0, dRidgeDy = 0;
  if (mountain.ridgeLength) {
    const cosA = Math.cos(mountain.ridgeAngle || 0);
    const sinA = Math.sin(mountain.ridgeAngle || 0);
    const along = dx*cosA + dy*sinA;
    const across = -dx*sinA + dy*cosA;
    const rl = mountain.ridgeLength;
    const rw = mountain.ridgeWidth || rl*0.15;
    const rf = mountain.ridgeFalloff || 0.4;
    const rr2 = along*along/(rl*rl) + across*across/(rw*rw);
    if (rr2 < 9) {
      const ridgeFactor = Math.exp(-rr2 * rf);
      const rh = (mountain.ridgeHeight || peak*0.4) * ridgeFactor;
      h_ridge = rh;
      const drr2_da = 2*along/(rl*rl);
      const drr2_dc = 2*across/(rw*rw);
      const dRidgeDa = -rh * rf * drr2_da;
      const dRidgeDc = -rh * rf * drr2_dc;
      dRidgeDx = dRidgeDa*cosA - dRidgeDc*sinA;
      dRidgeDy = dRidgeDa*sinA + dRidgeDc*cosA;
    }
  }

  const h = h_base + h_ridge;
  const dBaseDx = h_base * (-2*dx/(rx*rx)) * 0.6;
  const dBaseDy = h_base * (-2*dy/(ry*ry)) * 0.6;

  return {
    h,
    dhdx: dBaseDx + dRidgeDx,
    dhdy: dBaseDy + dRidgeDy
  };
}

/**
 * Evaluate a valley/canyon at position (x,y).
 * A valley is a linear depression with smooth Gaussian cross-section.
 */
function evaluateValley(valley, x, y) {
  const cosA = Math.cos(valley.angle || 0);
  const sinA = Math.sin(valley.angle || 0);
  const dx = x - valley.x;
  const dy = y - valley.y;
  const along = dx*cosA + dy*sinA;
  const across = -dx*sinA + dy*cosA;

  const halfLen = (valley.length || 100) * 0.5;
  if (Math.abs(along) > halfLen * 1.5) return { h: 0, dhdx: 0, dhdy: 0 };

  const w = valley.width || 15;
  const depth = valley.depth || 3;

  // Longitudinal taper
  const taper = Math.exp(-Math.pow(along/halfLen, 4) * 2);
  // Cross-section: Gaussian depression
  const crossR2 = across*across/(w*w);
  if (crossR2 >= 6) return { h: 0, dhdx: 0, dhdy: 0 };
  const crossProfile = Math.exp(-crossR2);
  const h = -depth * crossProfile * taper;

  const dCrossDc = depth * 2*across/(w*w) * crossProfile * taper;
  const dTaperDa = taper * (-8*Math.pow(along,3)/Math.pow(halfLen,4));
  const dAlongH = -depth * crossProfile * dTaperDa;

  return {
    h,
    dhdx: (-dCrossDc*sinA + dAlongH*cosA),
    dhdy: (dCrossDc*cosA + dAlongH*sinA)
  };
}

export class TerrainModel {
  constructor(cfg) {
    this.cfg = cfg;
    this.generateSurface();
  }

  generateSurface() {
    const rng = createRNG(this.cfg.seed);
    const lvl = this.cfg.terrainLevel;
    this.rocks = [];
    this.bPathRocks = [];
    this.courseGritRocks = [];
    this.craters = [];
    this.slopes = [];
    this.marshes = [];
    this.mountains = [];
    this.valleys = [];
    this.sandPatches = [];
    this.decorChips = [];
    this.expeditionObstacles = [];
    this.course = this.cfg.abCourseEnabled ? createABCourse(2*this.cfg.outerRadius) : null;

    if (this.course) {
      // The benchmark previously forced an almost-flat 0.018 m surface and
      // ignored the roughness control. Honour the configured RMS so the
      // visible course and the contact solver use the same rough terrain.
      this.rmsScale = clampValue(this.cfg.groundRMS, 0.0, 0.35);
    } else if (lvl === 1) {
      this.rmsScale = Math.max(0.035, this.cfg.groundRMS);
    } else if (lvl === 2) {
      this.rmsScale = Math.max(0.05, this.cfg.groundRMS);
      for (let i = 0; i < 8; i++) {
        this.rocks.push({ x: (rng()-0.5)*4, y: 3+i*2, rx: 0.24, ry: 0.18, h: 0.15, kind: 'challenge' });
      }
    } else if (lvl === 3) {
      this.rmsScale = Math.max(0.07, this.cfg.groundRMS);
      for (let i = 0; i < 10; i++) {
        this.rocks.push({ x: (rng()-0.5)*5, y: 3+i*2.2, rx: 0.40, ry: 0.30, h: 0.28, kind: 'challenge' });
      }
    } else if (lvl === 4) {
      this.rmsScale = Math.max(0.09, this.cfg.groundRMS);
      this.rocks.push({ x: 0, y: 5, rx: 0.62, ry: 0.48, h: 0.45, kind: 'challenge' });
      this.rocks.push({ x: 0.5, y: 11, rx: 0.78, ry: 0.58, h: 0.55, kind: 'challenge' });
    } else if (lvl === 5) {
      this.rmsScale = Math.max(0.075, this.cfg.groundRMS);
      this.craters.push({ x: 0.0, y: 6.0, r: 2.0, depth: 0.60 });
    } else if (lvl === 6) {
      this.rmsScale = Math.max(0.06, this.cfg.groundRMS);
      this.slopes.push({ yStart: 2.0, yEnd: 16.0, incline: 0.25 });
    } else if (lvl === 7) {
      this.rmsScale = Math.max(0.10, this.cfg.groundRMS);
      for (let i = 0; i < 12; i++) {
        this.rocks.push({
          x: (rng() - 0.5) * 5.0,
          y: 3.0 + i * 2.0 + rng() * 1.5,
          rx: 0.28+rng()*0.42,
          ry: 0.22+rng()*0.30,
          h: 0.20+rng()*0.35,
          kind: 'challenge'
        });
      }
      this.craters.push({ x: 0.8, y: 12.0, r: 1.6, depth: 0.5 });

    // ── Level 8: Mixed-Size Boulder Scatter ──
    // Tiny pebbles, medium rocks, and large boulders in one field.
    } else if (lvl === 8) {
      this.rmsScale = Math.max(0.10, this.cfg.groundRMS);
      // Tiny pebbles (20 count)
      for (let i = 0; i < 20; i++) {
        this.rocks.push({
          x: (rng()-0.5)*6, y: 2+rng()*22,
          rx: 0.06+rng()*0.08, ry: 0.05+rng()*0.06, h: 0.04+rng()*0.06,
          kind: 'challenge'
        });
      }
      // Medium rocks (8 count)
      for (let i = 0; i < 8; i++) {
        this.rocks.push({
          x: (rng()-0.5)*5, y: 3+i*2.8+rng()*1.2,
          rx: 0.25+rng()*0.20, ry: 0.20+rng()*0.15, h: 0.18+rng()*0.20,
          kind: 'challenge'
        });
      }
      // Large boulders (3 count)
      for (let i = 0; i < 3; i++) {
        this.rocks.push({
          x: (rng()-0.5)*3.5, y: 5+i*7+rng()*2,
          rx: 0.60+rng()*0.30, ry: 0.48+rng()*0.24, h: 0.40+rng()*0.25,
          kind: 'challenge'
        });
      }

    // ── Level 9: Mars Marsh Wetlands ──
    // Boggy depressions with soft, uneven floors. Reduced traction zones.
    } else if (lvl === 9) {
      this.rmsScale = Math.max(0.06, this.cfg.groundRMS);
      // Two large marsh zones
      this.marshes.push({ x: -0.3, y: 6.0,  rx: 2.2, ry: 1.8, depth: 0.14, muddiness: 0.7, seed: rng()*1000 });
      this.marshes.push({ x:  0.5, y: 14.0, rx: 2.5, ry: 2.0, depth: 0.18, muddiness: 0.9, seed: rng()*1000 });
      // Scattered small marsh pools
      for (let i = 0; i < 4; i++) {
        this.marshes.push({
          x: (rng()-0.5)*4, y: 3+rng()*20,
          rx: 0.8+rng()*0.6, ry: 0.7+rng()*0.5,
          depth: 0.06+rng()*0.10, muddiness: 0.4+rng()*0.5, seed: rng()*1000
        });
      }
      // A few rocks poking out of the marsh
      for (let i = 0; i < 5; i++) {
        this.rocks.push({
          x: (rng()-0.5)*3, y: 4+rng()*16,
          rx: 0.15+rng()*0.12, ry: 0.12+rng()*0.10, h: 0.10+rng()*0.12,
          kind: 'challenge'
        });
      }

    // ── Level 10: kept as A-vs-B (handled by course branch above) ──
    // Falls through to default since abCourseEnabled is set

    // ── Level 11: Uneven Rubble Field ──
    // Dense irregular rubble with high Fourier roughness and clustered debris.
    } else if (lvl === 11) {
      this.rmsScale = Math.max(0.14, this.cfg.groundRMS);
      // Dense rubble clusters — 5 clusters of 6-8 stones each
      for (let c = 0; c < 5; c++) {
        const cx = (rng()-0.5)*4.5;
        const cy = 3+c*4.5+rng()*2;
        const count = 6+Math.floor(rng()*3);
        for (let j = 0; j < count; j++) {
          this.rocks.push({
            x: cx+(rng()-0.5)*1.8,
            y: cy+(rng()-0.5)*1.4,
            rx: 0.10+rng()*0.22,
            ry: 0.08+rng()*0.18,
            h: 0.08+rng()*0.28,
            kind: 'challenge'
          });
        }
      }
      // Additional solo boulders
      for (let i = 0; i < 4; i++) {
        this.rocks.push({
          x: (rng()-0.5)*5, y: 2+rng()*24,
          rx: 0.35+rng()*0.30, ry: 0.28+rng()*0.22, h: 0.22+rng()*0.30,
          kind: 'challenge'
        });
      }

    // ── Level 12: Mars Bog & Boulder Gauntlet ──
    // Marsh depressions between boulder clusters — combined challenge.
    } else if (lvl === 12) {
      this.rmsScale = Math.max(0.11, this.cfg.groundRMS);
      // Alternating marsh–boulder pattern
      for (let i = 0; i < 3; i++) {
        // Marsh zone
        this.marshes.push({
          x: (rng()-0.5)*2.5, y: 3+i*8,
          rx: 1.6+rng()*0.8, ry: 1.3+rng()*0.6,
          depth: 0.10+rng()*0.10, muddiness: 0.5+rng()*0.4, seed: rng()*1000
        });
        // Boulder cluster after the marsh
        for (let j = 0; j < 6; j++) {
          this.rocks.push({
            x: (rng()-0.5)*4, y: 5.5+i*8+rng()*2,
            rx: 0.18+rng()*0.35, ry: 0.14+rng()*0.28,
            h: 0.12+rng()*0.35, kind: 'challenge'
          });
        }
      }
      // One large crater
      this.craters.push({ x: 0.3, y: 20.0, r: 1.8, depth: 0.55 });

    // ── Level 13: Extreme Mars Composite ──
    // Everything combined: marsh, varied boulders, crater, slope, high roughness.
    } else if (lvl === 13) {
      this.rmsScale = Math.max(0.15, this.cfg.groundRMS);
      // Marsh zones
      this.marshes.push({ x: -0.8, y: 4.0,  rx: 1.8, ry: 1.4, depth: 0.16, muddiness: 0.8, seed: rng()*1000 });
      this.marshes.push({ x:  1.0, y: 18.0, rx: 2.0, ry: 1.6, depth: 0.20, muddiness: 1.0, seed: rng()*1000 });
      // Slope section
      this.slopes.push({ yStart: 8.0, yEnd: 14.0, incline: 0.20 });
      // Crater
      this.craters.push({ x: -0.5, y: 22.0, r: 1.4, depth: 0.50 });
      // Multi-size rocks: tiny through massive
      for (let i = 0; i < 12; i++) {
        this.rocks.push({
          x: (rng()-0.5)*5, y: 2+rng()*24,
          rx: 0.05+rng()*0.08, ry: 0.04+rng()*0.06, h: 0.03+rng()*0.05,
          kind: 'challenge'
        }); // tiny pebbles
      }
      for (let i = 0; i < 8; i++) {
        this.rocks.push({
          x: (rng()-0.5)*5, y: 2+rng()*24,
          rx: 0.22+rng()*0.25, ry: 0.18+rng()*0.20, h: 0.15+rng()*0.25,
          kind: 'challenge'
        }); // medium rocks
      }
      for (let i = 0; i < 3; i++) {
        this.rocks.push({
          x: (rng()-0.5)*3, y: 4+rng()*20,
          rx: 0.55+rng()*0.35, ry: 0.45+rng()*0.28, h: 0.38+rng()*0.30,
          kind: 'challenge'
        }); // large boulders
      }
    // ── Level 14: 1km×1km Open-World Martian Expedition ──
    // A true 1000 m × 1000 m expanse centred on the origin: (x, y) ∈
    // [-500 m, +500 m] on both axes. The mission corridor runs north from
    // courseStartY to targetGoalY through multi-octave dune fields, crater
    // rims with central uplift, jagged ridge lines, drop-off ledges and rocky
    // plateau inclines, all benchmarked by a solid crest-checkpoint chain.
    } else if (lvl === 14) {
      this.rmsScale = Math.max(0.24, this.cfg.groundRMS);
      const WORLD_EDGE = 480;   // scenery bound inside the ±500 m field
      const missionStartY = this.cfg.courseStartY ?? -450;
      const missionGoalY = this.cfg.targetGoalY ?? this.cfg.courseGoalY ?? 450;

      // — Expedition obstacle chain: Level-10-style solid obstacles spaced
      // down the whole corridor. These are scored crest checkpoints for the
      // adaptive model, exactly like the Level 10 benchmark, but at
      // expedition scale. Ranges derive from the configured mission lines so
      // scaled-down regression configs generate proportional chains.
      const obstacleRng = createRNG((this.cfg.seed ^ 0x0b57ac1e) >>> 0);
      const obstacleTypes = ['jagged-rock', 'eroded-block', 'tilted-slab', 'broken-ridge'];
      const difficulties = ['small', 'medium', 'large'];
      const chainStartY = missionStartY+25;
      const chainEndY = Math.min(missionGoalY-10, missionStartY+905);
      let obstacleY = chainStartY;
      while (obstacleY < chainEndY) {
        const type = obstacleTypes[Math.floor(obstacleRng()*obstacleTypes.length)];
        const difficulty = difficulties[Math.floor(obstacleRng()*difficulties.length)];
        const severity = { small: 1.0, medium: 1.25, large: 1.5 }[difficulty];
        this.expeditionObstacles.push({
          id: `E${String(this.expeditionObstacles.length+1).padStart(2, '0')}`,
          type, difficulty,
          x: (obstacleRng()-0.5)*0.9,
          y: obstacleY,
          // Same 0.50×diameter height ratio as the Level 10 course, scaled by
          // difficulty; radii keep the Level 10 minimums so the rover can
          // actually crest them.
          height: 0.50*severity*(2*this.cfg.outerRadius),
          radiusX: (1.50+0.30*obstacleRng())*severity*(2*this.cfg.outerRadius),
          radiusY: (1.60+0.35*obstacleRng())*severity*(2*this.cfg.outerRadius),
          yaw: (obstacleRng()-0.5)*0.65,
          skewX: (obstacleRng()-0.5)*0.5,
          skewY: (obstacleRng()-0.5)*0.5,
          lobe: (obstacleRng()-0.5)*0.4,
          twist: (obstacleRng()-0.5)*0.4
        });
        obstacleY += 20+obstacleRng()*15;
      }

      // — Corridor stone gauntlet: extremely dense stone fields blanketing
      // the travel corridor with no clear path through.
      const gauntletRng = createRNG((this.cfg.seed ^ 0x5e0a5e0a) >>> 0);
      for (let y = missionStartY+5; y < missionGoalY-5; y += 1.4) {
        const clusterCount = 4+Math.floor(gauntletRng()*4);
        for (let j = 0; j < clusterCount; j++) {
          const lateral = (gauntletRng()-0.5)*5.0;
          this.rocks.push({
            x: lateral,
            y: y+gauntletRng()*1.5,
            rx: 0.10+gauntletRng()*0.30,
            ry: 0.08+gauntletRng()*0.22,
            h: 0.08+gauntletRng()*0.28,
            yaw: (gauntletRng()-0.5)*Math.PI,
            colorSeed: gauntletRng(),
            kind: 'expedition-gauntlet'
          });
        }
      }

      // — Multi-octave procedural heightfield: fractal dune fields, jagged
      // ridgelines, and terraced plateau ledges layered over the whole km².
      // Fields are deterministic under the world seed and evaluated with
      // exact derivatives inside evalBase().
      const fieldRng = createRNG((this.cfg.seed ^ 0x6e01d5) >>> 0);
      const seedFrom = offset => (Math.floor(fieldRng()*0xffffffff) ^ offset) >>> 0;
      this.noiseFields = {
        // Rolling sand dune basins: long wavelength, strong amplitude.
        dunes: {
          baseFrequency: 1/95, octaves: 4, gain: 0.5, lacunarity: 2.05,
          seed: seedFrom(0xd00e), amplitude: 2.4
        },
        // Jagged ridge lines: ridged fBm, masked to a band network.
        ridges: {
          baseFrequency: 1/60, octaves: 5, gain: 0.52, lacunarity: 2.1,
          seed: seedFrom(0x21d6), amplitude: 1.8
        },
        // Rocky plateau inclines: low-frequency mask carving terraced ledges.
        ledges: {
          baseFrequency: 1/140, octaves: 3, gain: 0.45, lacunarity: 2.0,
          seed: seedFrom(0x13a7), amplitude: 4.0
        }
      };

      // — Multi-feature landscape: major shield volcanoes with ridge
      // extensions, secondary peaks, deep valleys/canyons, layered slope
      // bands, large craters with rim uplift, and extensive marsh basins,
      // all scattered across the full square kilometre.
      const mountainRng = createRNG((this.cfg.seed ^ 0xa1b2c3d4) >>> 0);
      const peakPositions = [
        { x: -210, y: -260, peakHeight: 26, rx: 85, ry: 65, ridgeAngle: 0.45, ridgeLength: 130, ridgeWidth: 18, ridgeHeight: 12 },
        { x: 195, y: -95, peakHeight: 34, rx: 100, ry: 72, ridgeAngle: -0.35, ridgeLength: 150, ridgeWidth: 22, ridgeHeight: 16 },
        { x: -160, y: 120, peakHeight: 30, rx: 75, ry: 92, ridgeAngle: 0.85, ridgeLength: 110, ridgeWidth: 15, ridgeHeight: 14 },
        { x: 230, y: 260, peakHeight: 38, rx: 108, ry: 82, ridgeAngle: -0.6, ridgeLength: 175, ridgeWidth: 25, ridgeHeight: 18 },
        { x: -240, y: 400, peakHeight: 27, rx: 88, ry: 66, ridgeAngle: 1.0, ridgeLength: 125, ridgeWidth: 20, ridgeHeight: 13 },
        { x: 60, y: 430, peakHeight: 22, rx: 62, ry: 52, ridgeAngle: 0.2, ridgeLength: 90, ridgeWidth: 14, ridgeHeight: 10 }
      ];
      // Secondary smaller peaks; big cores stay clear of the travel lane.
      for (let i = 0; i < 14; i++) {
        peakPositions.push({
          x: (mountainRng()-0.5)*2*WORLD_EDGE,
          y: (mountainRng()-0.5)*2*WORLD_EDGE,
          peakHeight: 7 + mountainRng()*17,
          rx: 26 + mountainRng()*48,
          ry: 22 + mountainRng()*40,
          ridgeAngle: (mountainRng()-0.5)*Math.PI,
          ridgeLength: 35 + mountainRng()*80,
          ridgeWidth: 8 + mountainRng()*12,
          ridgeHeight: 3.5 + mountainRng()*9
        });
      }
      this.mountains = peakPositions;

      // — Valleys / canyons: centres kept off-corridor so their tapering
      // ends grade into the route instead of cutting it with a sheer wall.
      this.valleys = [
        { x: -70, y: -300, angle: 0.35, length: 220, width: 20, depth: 5 },
        { x: 85, y: -60, angle: -0.5, length: 180, width: 16, depth: 4 },
        { x: -95, y: 190, angle: 0.7, length: 200, width: 24, depth: 6 },
        { x: 120, y: 420, angle: -0.25, length: 170, width: 18, depth: 4.5 }
      ];

      // — Slope segments covering the full expanse: angles -18° to +18° —
      const slopeAngles = [-18, -12, -8, 8, 12, 15, -15, 10, -5, 18, -10, 5];
      for (let i = 0; i < slopeAngles.length; i++) {
        const yStart = -WORLD_EDGE+8+i*80;
        const yEnd = yStart+50+mountainRng()*30;
        this.slopes.push({
          yStart, yEnd,
          incline: Math.tan(slopeAngles[i]*Math.PI/180)
        });
      }

      // — Large craters across the km²: rims and central uplift stay clear
      // of the immediate lane (|x| > r+6) so no crater wall fully blocks it.
      for (let i = 0; i < 10; i++) {
        const craterR = 8+mountainRng()*24;
        const side = mountainRng() < 0.5 ? -1 : 1;
        this.craters.push({
          x: side*(craterR+7)+ (mountainRng()-0.5)*2*(WORLD_EDGE-craterR-10),
          y: (mountainRng()-0.5)*2*WORLD_EDGE,
          r: craterR,
          depth: 2+mountainRng()*5.5
        });
      }

      // — Extensive marsh zones —
      for (let i = 0; i < 12; i++) {
        this.marshes.push({
          x: (mountainRng()-0.5)*2*WORLD_EDGE*0.85,
          y: (mountainRng()-0.5)*2*WORLD_EDGE*0.85,
          rx: 9+mountainRng()*28,
          ry: 8+mountainRng()*24,
          depth: 0.3+mountainRng()*1.2,
          muddiness: 0.4+mountainRng()*0.6,
          seed: mountainRng()*10000
        });
      }

      // — Fine sand & dust beds: loose-granular patches with locally reduced
      // Coulomb grip and extra viscous drag. Physics queries them through
      // sandAt(); the renderer tints them as pale dust sheets.
      this.sandPatches = [];
      for (let i = 0; i < 22; i++) {
        this.sandPatches.push({
          x: (mountainRng()-0.5)*2*WORLD_EDGE*0.9,
          y: (mountainRng()-0.5)*2*WORLD_EDGE*0.9,
          rx: 24+mountainRng()*66,
          ry: 18+mountainRng()*52,
          frictionScale: 0.72+mountainRng()*0.22,
          dragScale: 1.0+mountainRng()*1.3,
          seed: mountainRng()*10000
        });
      }

      // — Extremely high-density surface scatter across the entire 1 km²:
      // 🪨 pebbles & small gravel (5–15 cm),
      // 🗿 medium & sharp angular rocks (20–60 cm),
      // 🏔️ big boulders & monolithic Martian rocks (1.5–4 m).
      // No keep-out lanes — the surface is heavily cluttered with tightly
      // packed irregular dark rocks everywhere, zero clear paths or trails.
      const scatterRng = createRNG((this.cfg.seed ^ 0x51cabbe5) >>> 0);
      const scatterInField = margin => ({
        x: (scatterRng()-0.5)*2*(WORLD_EDGE-margin),
        y: (scatterRng()-0.5)*2*(WORLD_EDGE-margin)
      });
      // Pebbles & small gravel (5–15 cm): carpeting the entire surface.
      for (let i = 0; i < 12000; i++) {
        const p = scatterInField(2);
        this.rocks.push({
          x: p.x, y: p.y,
          rx: 0.05+scatterRng()*0.10,
          ry: 0.04+scatterRng()*0.09,
          h: 0.03+scatterRng()*0.07,
          yaw: (scatterRng()-0.5)*Math.PI,
          colorSeed: scatterRng(),
          kind: 'expedition-pebble'
        });
      }
      // Medium & sharp angular rocks (20–60 cm): packed across the field;
      // only a thin trickle stays inside the immediate lane band so the
      // mission corridor remains traversable.
      for (let i = 0; i < 4800; i++) {
        const p = scatterInField(2);
        if (Math.abs(p.x) < 2.5 && scatterRng() < 0.88) {
          p.x = Math.sign(p.x || 1)*(2.5+scatterRng()*10);
        }
        const size = 0.20+scatterRng()*0.40;
        this.rocks.push({
          x: p.x, y: p.y,
          rx: size,
          ry: size*(0.70+scatterRng()*0.28),
          h: 0.14+scatterRng()*0.32,
          yaw: (scatterRng()-0.5)*Math.PI,
          colorSeed: scatterRng(),
          sharp: scatterRng() < 0.5,
          kind: 'expedition-rock'
        });
      }
      // Big boulders & monoliths (1.5–4 m): dense clusters plus solo giants
      // scattered uniformly — with a hard keep-out so no massive rock walls
      // off the mission corridor.
      const keepBoulderOutOfLane = stone => {
        if (Math.abs(stone.x) < 4.5+stone.rx) {
          stone.x = Math.sign(stone.x || 1)*(4.5+stone.rx+scatterRng()*12);
        }
        return stone;
      };
      for (let cluster = 0; cluster < 160; cluster++) {
        const cx = (scatterRng()-0.5)*2*(WORLD_EDGE-8);
        const cy = (scatterRng()-0.5)*2*(WORLD_EDGE-8);
        const members = 2+Math.floor(scatterRng()*4);
        for (let m = 0; m < members; m++) {
          const size = 1.5+scatterRng()*2.5;
          const stone = {
            x: cx+(scatterRng()-0.5)*size*2.2,
            y: cy+(scatterRng()-0.5)*size*2.2,
            rx: size,
            ry: size*(0.72+scatterRng()*0.26),
            h: 0.55*size+scatterRng()*0.45*size,
            yaw: (scatterRng()-0.5)*Math.PI,
            colorSeed: scatterRng(),
            monolith: size > 3.0,
            kind: 'expedition-boulder'
          };
          if (Math.abs(stone.x) > WORLD_EDGE-4 || Math.abs(stone.y) > WORLD_EDGE-4) continue;
          this.rocks.push(keepBoulderOutOfLane(stone));
        }
      }
      for (let i = 0; i < 600; i++) {
        const size = 1.5+scatterRng()*2.5;
        this.rocks.push(keepBoulderOutOfLane({
          x: (scatterRng()-0.5)*2*(WORLD_EDGE-6),
          y: (scatterRng()-0.5)*2*(WORLD_EDGE-6),
          rx: size,
          ry: size*(0.72+scatterRng()*0.26),
          h: 0.55*size+scatterRng()*0.45*size,
          yaw: (scatterRng()-0.5)*Math.PI,
          colorSeed: scatterRng(),
          monolith: size > 3.0,
          kind: 'expedition-boulder'
        }));
      }

      // — Visual-only micro gravel (2–6 cm): a deterministic cosmetic layer
      // that packs the ground to photographic clutter density. Chips live
      // outside the physics rock array (far below the node contact scale),
      // but the renderer anchors every one onto the exact heightfield like
      // every physical stone.
      this.decorChips = [];
      for (let i = 0; i < 18000; i++) {
        this.decorChips.push({
          x: (scatterRng()-0.5)*2*(WORLD_EDGE-1),
          y: (scatterRng()-0.5)*2*(WORLD_EDGE-1),
          rx: 0.02+scatterRng()*0.04,
          ry: 0.018+scatterRng()*0.034,
          h: 0.012+scatterRng()*0.028,
          yaw: (scatterRng()-0.5)*Math.PI,
          colorSeed: scatterRng(),
          kind: 'expedition-chip'
        });
      }
    } else {
      this.rmsScale = this.cfg.groundRMS;
    }

    // Photo-inspired Mars foundation shared by every level: granular sand,
    // scattered embedded stones, and low broken ridgelines. Level 10 keeps
    // these outcrops outside its strict centre corridor so its benchmark
    // obstacles and learning objective remain unchanged.
    const marsRng = createRNG((this.cfg.seed ^ 0x9e3779b9) >>> 0);
    const terrainMaxY = this.course ? this.cfg.courseMaxY : (lvl === 14 ? 480 : 44);
    const scatterCount = this.course ? 28 : (lvl === 14 ? 800 : 16+2*Math.min(10, Math.max(1, lvl)));
    for (let i = 0; i < scatterCount; i++) {
      const side = marsRng() < 0.5 ? -1 : 1;
      let x = this.course ? side*(1.95+1.75*marsRng())
        : (lvl === 14 ? (marsRng()-0.5)*960 : (marsRng()-0.5)*7.2);
      const y = lvl === 14
        ? (marsRng()-0.5)*960
        : 1.5+marsRng()*(terrainMaxY-3);
      let h = 0.045+0.13*marsRng();
      if (!this.course && Math.abs(x) < 0.55) h *= 0.55;
      this.rocks.push({
        x,
        y,
        rx: 0.12+0.24*marsRng(),
        ry: 0.09+0.18*marsRng(),
        h,
        yaw: (marsRng()-0.5)*Math.PI,
        colorSeed: marsRng(),
        kind: 'mars-scatter'
      });
    }

    const ridgeCount = this.course ? 7 : (lvl === 14 ? 90 : 5);
    for (let ridge = 0; ridge < ridgeCount; ridge++) {
      const side = marsRng() < 0.5 ? -1 : 1;
      const centerX = side*(this.course ? 2.25+0.85*marsRng()
        : lvl === 14 ? 3.0+6.0*marsRng() : 1.65+1.35*marsRng());
      const centerY = lvl === 14
        ? (marsRng()-0.5)*940
        : 3+marsRng()*(terrainMaxY-6);
      const yaw = (marsRng()-0.5)*0.7;
      for (let stone = 0; stone < 4; stone++) {
        const along = (stone-1.5)*(0.32+0.12*marsRng());
        this.rocks.push({
          x: centerX+along*Math.cos(yaw)+0.10*(marsRng()-0.5),
          y: centerY+along*Math.sin(yaw)+0.12*(marsRng()-0.5),
          rx: 0.28+0.24*marsRng(),
          ry: 0.17+0.16*marsRng(),
          h: 0.16+0.30*marsRng(),
          yaw,
          colorSeed: marsRng(),
          kind: 'mars-ridge'
        });
      }
    }

    // Coarse sand and embedded grit are part of the physical obstacle
    // surface, not a visual-only texture. Each irregular formation receives
    // deterministic low-profile grains across its upper body so contacts gain
    // small local normals and the rover cannot skate over a smooth analytic
    // mound.
    if (this.course) {
      const gritRng = createRNG((this.cfg.seed ^ 0x68e31da4) >>> 0);
      for (const obstacle of this.course.obstacles) {
        for (let grain = 0; grain < 7; grain++) {
          const angle = (gritRng()-0.5)*Math.PI*2;
          const radius = 0.18+0.54*gritRng();
          const localX = radius*obstacle.radiusX*Math.cos(angle);
          const localY = radius*obstacle.radiusY*Math.sin(angle);
          const cosYaw = Math.cos(obstacle.yaw || 0);
          const sinYaw = Math.sin(obstacle.yaw || 0);
          this.courseGritRocks.push({
            obstacleId: obstacle.id,
            x: obstacle.x+cosYaw*localX-sinYaw*localY,
            y: obstacle.y+sinYaw*localX+cosYaw*localY,
            rx: 0.055+0.055*gritRng(),
            ry: 0.045+0.045*gritRng(),
            h: 0.012+0.020*gritRng(),
            yaw: angle,
            colorSeed: gritRng(),
            kind: 'course-grit'
          });
        }
      }
    }

    // Model B receives an additional deterministic field of small embedded
    // stones along its physical route. These are kept separate from the
    // shared Mars scenery so Model A retains the original benchmark surface,
    // while every B contact/clearance query uses the denser rock path.
    if (this.course) {
      const bPathRng = createRNG((this.cfg.seed ^ 0xb5297a4d) >>> 0);
      const rockCount = 42;
      const firstY = this.course.startY+0.60;
      const lastY = this.course.goalY-0.80;
      for (let i = 0; i < rockCount; i++) {
        const progress = i/Math.max(1, rockCount-1);
        this.bPathRocks.push({
          x: (bPathRng()-0.5)*1.55,
          y: firstY+progress*(lastY-firstY)+0.22*(bPathRng()-0.5),
          rx: 0.075+0.075*bPathRng(),
          ry: 0.065+0.070*bPathRng(),
          h: 0.035+0.055*bPathRng(),
          yaw: (bPathRng()-0.5)*Math.PI,
          colorSeed: bPathRng(),
          kind: 'b-path'
        });
      }
    }

    const M = 6, N = 8;
    const modes = [];
    let sumSq = 0;

    for (let m = -M; m <= M; m++) {
      for (let n = -N; n <= N; n++) {
        if (m === 0 && n === 0) continue;
        const kx = 2.0 * Math.PI * m * (lvl === 14 ? 0.003 : 0.08);
        const ky = 2.0 * Math.PI * n * (lvl === 14 ? 0.002 : 0.05);
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

    this.rebuildRockIndex();
  }

  /**
   * Uniform spatial hash over every scattered stone (shared rocks plus Model
   * B's private path rocks). The open-world expedition pushes the scatter
   * count well past 1500, and the height field is evaluated hundreds of
   * times per physics step — a grid lookup keeps each query proportional to
   * the stones actually nearby instead of O(all rocks).
   * evaluateRockOutcrop() is exactly zero beyond 2×max(rx,ry), so indexing
   * each stone into the cells its footprint covers never clips a real
   * contribution.
   */
  rebuildRockIndex() {
    const cell = 8.0;
    this.scatterCellSize = cell;
    this.scatterGrid = new Map();
    this.scatterList = [];
    this.scatterIds = new Map();
    let uid = 0;
    const insert = (rock, objectId) => {
      const rx = Math.max(0.04, rock.rx || rock.r || 0.2);
      const ry = Math.max(0.04, rock.ry || rock.r || 0.2);
      const reachX = 2*rx;
      const reachY = 2*ry;
      Object.defineProperty(rock, '_sq', {
        value: uid++, writable: false, enumerable: false, configurable: true
      });
      if (objectId) this.scatterIds.set(rock, objectId);
      this.scatterList.push(rock);
      const ix0 = Math.floor((rock.x-reachX)/cell);
      const ix1 = Math.floor((rock.x+reachX)/cell);
      const iy0 = Math.floor((rock.y-reachY)/cell);
      const iy1 = Math.floor((rock.y+reachY)/cell);
      for (let iy = iy0; iy <= iy1; iy++) {
        for (let ix = ix0; ix <= ix1; ix++) {
          const key = (ix+4096)*8192+(iy+4096);
          let bucket = this.scatterGrid.get(key);
          if (!bucket) {
            bucket = [];
            this.scatterGrid.set(key, bucket);
          }
          bucket.push(rock);
        }
      }
    };
    this.rocks.forEach((rock, index) => insert(rock, `rock-${index+1}`));
    this.bPathRocks.forEach((rock, index) => insert(rock, `b-rock-${index+1}`));
    this.scatterStamps = new Int32Array(uid);
    this.scatterStampEpoch = 0;
    this.scatterScratch = [];
    this.maxSolidScatterRadius = this.scatterList.reduce((maximum, rock) =>
      Math.max(maximum, rock.rx || rock.r || 0.2, rock.ry || rock.r || 0.2), 0.2);
    this.maxRockReach = this.scatterList.reduce((maximum, rock) =>
      Math.max(maximum, 2*Math.max(rock.rx || rock.r || 0.2, rock.ry || rock.r || 0.2)), 0.6);
  }

  /**
   * Stones whose footprints may contribute at/around (x, y) within `reach`.
   * Returns a shared scratch array valid until the next query; each stone is
   * yielded at most once per query. Model B's path stones are included only
   * for the adaptive lane, matching eval()/objectAt() semantics.
   */
  scatterQuery(x, y, reach, includeBPath = false) {
    const epoch = ++this.scatterStampEpoch;
    const result = this.scatterScratch;
    result.length = 0;
    const cell = this.scatterCellSize;
    const ix0 = Math.floor((x-reach)/cell);
    const ix1 = Math.floor((x+reach)/cell);
    const iy0 = Math.floor((y-reach)/cell);
    const iy1 = Math.floor((y+reach)/cell);
    for (let iy = iy0; iy <= iy1; iy++) {
      for (let ix = ix0; ix <= ix1; ix++) {
        const bucket = this.scatterGrid.get((ix+4096)*8192+(iy+4096));
        if (!bucket) continue;
        for (let i = 0; i < bucket.length; i++) {
          const rock = bucket[i];
          if (this.scatterStamps[rock._sq] === epoch) continue;
          this.scatterStamps[rock._sq] = epoch;
          if (!includeBPath && rock.kind === 'b-path') continue;
          result.push(rock);
        }
      }
    }
    return result;
  }

  rocksNear(x, y, includeBPath = false) {
    return this.scatterQuery(x, y, this.maxRockReach, includeBPath);
  }

  eval(x, y, modelType = 'fixed') {
    const surface = this.evalBase(x, y, modelType);
    let { h, dhdx, dhdy } = surface;
    for (const obstacle of this.course?.obstacles || []) {
      const contribution = evaluateCourseObstacle(obstacle, x, y);
      h += contribution.h;
      dhdx += contribution.dhdx;
      dhdy += contribution.dhdy;
    }
    // Expedition obstacle chain shares the Level 10 obstacle analytic, so
    // both lanes climb the same solid, C1-continuous crest geometry.
    for (const obstacle of this.expeditionObstacles || []) {
      const contribution = evaluateCourseObstacle(obstacle, x, y);
      h += contribution.h;
      dhdx += contribution.dhdx;
      dhdy += contribution.dhdy;
    }
    for (const grain of this.courseGritRocks) {
      const contribution = evaluateRockOutcrop(grain, x, y);
      h += contribution.h;
      dhdx += contribution.dhdx;
      dhdy += contribution.dhdy;
    }
    return { h, dhdx, dhdy };
  }

  objectAt(x, y, modelType = 'fixed') {
    let objectId = 'ground';
    let maximumContribution = 1e-5;
    for (const obstacle of this.course?.obstacles || []) {
      const contribution = evaluateCourseObstacle(obstacle, x, y).h;
      if (contribution > maximumContribution) {
        maximumContribution = contribution;
        objectId = obstacle.id;
      }
    }
    (this.expeditionObstacles || []).forEach((obstacle, index) => {
      const contribution = evaluateCourseObstacle(obstacle, x, y).h;
      if (contribution > maximumContribution) {
        maximumContribution = contribution;
        objectId = obstacle.id;
      }
    });
    this.courseGritRocks.forEach((grain, index) => {
      const contribution = evaluateRockOutcrop(grain, x, y).h;
      if (contribution > maximumContribution) {
        maximumContribution = contribution;
        objectId = `grit-${grain.obstacleId}-${index+1}`;
      }
    });
    // Spatial-hash window instead of a full scan; IDs stay identical.
    const includeBPath = modelType === 'adaptive';
    for (const rock of this.scatterQuery(
      x, y, Math.max(0.3, 2*this.maxSolidScatterRadius), includeBPath)) {
      const contribution = evaluateRockOutcrop(rock, x, y).h;
      if (contribution > maximumContribution) {
        maximumContribution = contribution;
        objectId = this.scatterIds.get(rock);
      }
    }
    return objectId;
  }

  evalBase(x, y, modelType = 'fixed') {
    let h = 0;
    let dhdx = 0;
    let dhdy = 0;

    // Micro-roughness fades in just north of the mission start line so the
    // rover gets a smooth launch pad, then reaches full strength quickly.
    // Anchored to courseStartY because the Level 14 expedition begins far
    // south of the origin (-450 m); the old absolute y>1 gate left the whole
    // southern half of the open world glass-smooth.
    const roughnessOrigin = this.cfg.terrainLevel === 14
      ? (this.cfg.courseStartY ?? -450)+10 : 1.0;
    const roughnessScale = Math.min(1.0, Math.max(0.0,
      (y-roughnessOrigin)/(this.cfg.terrainLevel === 14 ? 10.0 : 2.0)));
    for (let i = 0; i < this.modes.length; i++) {
      const { kx, ky, A, phi } = this.modes[i];
      const arg = kx * x + ky * y + phi;
      const cos_arg = Math.cos(arg);
      const sin_arg = Math.sin(arg);

      h += A * cos_arg * roughnessScale;
      dhdx -= A * kx * sin_arg * roughnessScale;
      dhdy -= A * ky * sin_arg * roughnessScale;
    }

    // Windowed rock lookup: only stones whose footprint can reach this
    // query point, gathered from the spatial hash. Model B's private path
    // stones ride the same grid and join only for the adaptive lane.
    const nearbyRocks = this.rocksNear(x, y, modelType === 'adaptive');
    for (let rock of nearbyRocks) {
      const contribution = evaluateRockOutcrop(rock, x, y);
      h += contribution.h;
      dhdx += contribution.dhdx;
      dhdy += contribution.dhdy;
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

    // Marsh (boggy depression) zones
    for (const marsh of this.marshes) {
      const contribution = evaluateMarsh(marsh, x, y);
      h += contribution.h;
      dhdx += contribution.dhdx;
      dhdy += contribution.dhdy;
    }

    // Mountain peaks
    if (this.mountains) {
      for (const mtn of this.mountains) {
        const contribution = evaluateMountain(mtn, x, y);
        h += contribution.h;
        dhdx += contribution.dhdx;
        dhdy += contribution.dhdy;
      }
    }

    // Level 14 open-world procedural layer: rolling dune basins, jagged
    // ridgelines, and drop-off plateau ledges across the whole km². The
    // layer eases out toward the mission lane (|x| < ~2 m) so the 900 m
    // crossing stays physically traversable while the open field keeps its
    // full drama beyond the travel corridor.
    if (this.noiseFields && this.cfg.terrainLevel === 14) {
      const f = this.noiseFields;
      const laneFade = smoothStep01((Math.abs(x)-1.6)/6.5);

      // Dunes: plain fBm everywhere.
      const dunes = fbmNoise(x, y, f.dunes);
      h += laneFade*f.dunes.amplitude*dunes.value;
      dhdx += laneFade*f.dunes.amplitude*dunes.dhdx;
      dhdy += laneFade*f.dunes.amplitude*dunes.dhdy;

      // Ridges: ridged fBm gated by a low-frequency mask so they form a
      // sparse jagged network rather than carpeting the map.
      if (laneFade > 1e-6) {
        const ridgeMask = fbmNoise(x*0.35+40.7, y*0.35-13.2, f.ridges);
        const gate = smoothStep01(1.15*(ridgeMask.value-0.08));
        if (gate > 1e-6) {
          const ridges = ridgedFbm(x, y, f.ridges);
          const ridgeHeight = Math.max(0, ridges.value-0.55)/0.45;
          h += laneFade*f.ridges.amplitude*gate*Math.pow(ridgeHeight, 1.6);
          dhdx += laneFade*f.ridges.amplitude*gate*1.6*Math.pow(Math.max(0, ridgeHeight), 0.6)*ridges.dhdx/0.45;
          dhdy += laneFade*f.ridges.amplitude*gate*1.6*Math.pow(Math.max(0, ridgeHeight), 0.6)*ridges.dhdy/0.45;
        }
      }

      // Ledges / plateaus: quantize a low-frequency field into terraces with
      // smooth drop-offs between levels. The corridor keeps a quarter-strength
      // tread offset so elevation stays continuous into the lane.
      const ledges = fbmNoise(x, y, f.ledges);
      const ledgeAmp = f.ledges.amplitude*(0.25+0.75*laneFade);
      const terraceLevels = 5;
      const step = (ledgeAmp*2)/terraceLevels;
      const raw = ledgeAmp*ledges.value+ledgeAmp;   // [0, 2·ledgeAmp]
      const level = clampValue(raw/step, 0, terraceLevels);
      const li = Math.floor(level);
      const lt = level-li;
      const edge = smoothStep01((lt-0.82)/0.18);   // last 18% = drop-off
      const terraceH = (li+edge)*step-ledgeAmp;
      h += terraceH;
      // Derivative: flat on the tread, steep through the scarp. The scarp
      // slope direction follows the underlying ledge field gradient.
      const dEdge = lt > 0.82 && lt < 1
        ? (6*((lt-0.82)/0.18)*(1-(lt-0.82)/0.18))/0.18 : 0;
      const gradMag = Math.hypot(ledges.dhdx, ledges.dhdy) || 1;
      const dirX = ledges.dhdx/gradMag;
      const dirY = ledges.dhdy/gradMag;
      dhdx += dEdge*step*dirX;
      dhdy += dEdge*step*dirY;
    }

    // Valley/canyon depressions
    if (this.valleys) {
      for (const val of this.valleys) {
        const contribution = evaluateValley(val, x, y);
        h += contribution.h;
        dhdx += contribution.dhdx;
        dhdy += contribution.dhdy;
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

  /**
   * Check whether point (x,y) is inside any marsh zone.
   * Returns {inMarsh: bool, muddiness: number, frictionScale: number}.
   * Physics can use frictionScale to reduce traction in boggy areas.
   */
  marshAt(x, y) {
    for (const marsh of this.marshes) {
      const rx = marsh.rx || 1.5;
      const ry = marsh.ry || 1.5;
      const dx = x-marsh.x;
      const dy = y-marsh.y;
      const r2 = dx*dx/(rx*rx)+dy*dy/(ry*ry);
      if (r2 < 1.0) {
        const depth = 1.0-r2;  // 1 at centre, 0 at edge
        const muddiness = marsh.muddiness || 0.5;
        // Friction drops to 40-80% normal depending on depth and muddiness
        const frictionScale = 1.0-0.55*muddiness*depth;
        return { inMarsh: true, muddiness, frictionScale, depth: depth };
      }
    }
    return { inMarsh: false, muddiness: 0, frictionScale: 1.0, depth: 0 };
  }

  /**
   * Check whether point (x,y) lies inside a fine sand / dust bed.
   * Returns {inSand, frictionScale, dragScale, depth}. Loose granular beds
   * lower the Coulomb grip (frictionScale < 1) and raise viscous drag
   * (dragScale > 1); both blend smoothly to neutral at the patch edge.
   */
  sandAt(x, y) {
    for (const sand of this.sandPatches || []) {
      const rx = sand.rx || 20;
      const ry = sand.ry || 20;
      const dx = x-sand.x;
      const dy = y-sand.y;
      const r2 = dx*dx/(rx*rx)+dy*dy/(ry*ry);
      if (r2 < 1.0) {
        const depth = 1.0-r2;   // 1 at centre, 0 at edge
        const blend = Math.min(1, 1.25*depth); // slightly inset plateau
        return {
          inSand: true,
          frictionScale: 1.0-(1.0-(sand.frictionScale || 0.85))*blend,
          dragScale: 1.0+((sand.dragScale || 1.4)-1.0)*blend,
          depth
        };
      }
    }
    return { inSand: false, frictionScale: 1.0, dragScale: 1.0, depth: 0 };
  }

  solidTerrainObjects(modelType = 'fixed') {
    return [
      ...(this.course?.obstacles || []),
      ...(this.expeditionObstacles || []),
      ...this.courseGritRocks,
      ...this.rocks,
      ...(modelType === 'adaptive' ? this.bPathRocks : [])
    ];
  }

  memberCollisionParameters(positionA, positionB, modelType = 'fixed') {
    const parameters = [...MEMBER_COLLISION_SAMPLES];
    const segmentX = positionB[0]-positionA[0];
    const segmentY = positionB[1]-positionA[1];
    const planarLengthSquared = segmentX*segmentX+segmentY*segmentY;
    if (planarLengthSquared < 1e-10) return parameters;

    // Candidate features come from the spatial hash around the segment
    // bounding box plus the small analytic obstacle chains, so a 1600+ stone
    // field costs the same as the old handful.
    const midX = 0.5*(positionA[0]+positionB[0]);
    const midY = 0.5*(positionA[1]+positionB[1]);
    const halfLength = 0.5*Math.sqrt(planarLengthSquared);
    const features = [
      ...(this.course?.obstacles || []),
      ...(this.expeditionObstacles || []),
      ...this.courseGritRocks,
      ...this.scatterQuery(midX, midY,
        halfLength+this.maxSolidScatterRadius+0.05, modelType === 'adaptive')
    ];
    for (const feature of features) {
      const projected = ((feature.x-positionA[0])*segmentX
        +(feature.y-positionA[1])*segmentY)/planarLengthSquared;
      if (projected <= 0 || projected >= 1) continue;
      const nearestX = positionA[0]+projected*segmentX;
      const nearestY = positionA[1]+projected*segmentY;
      const radiusX = feature.radiusX || feature.rx || feature.r || 0.2;
      const radiusY = feature.radiusY || feature.ry || feature.r || 0.2;
      const reach = (feature.radiusX ? 1.7 : 2.1)*Math.max(radiusX, radiusY)+0.04;
      const dx = nearestX-feature.x;
      const dy = nearestY-feature.y;
      if (dx*dx+dy*dy <= reach*reach) parameters.push(projected);
    }
    return [...new Set(parameters.map(value => Number(value.toFixed(7))))]
      .sort((first, second) => first-second);
  }

  forModel(modelType = 'fixed') {
    const source = this;
    const view = Object.create(source);
    view.eval = (x, y) => source.eval(x, y, modelType);
    view.evalBase = (x, y) => source.evalBase(x, y, modelType);
    view.objectAt = (x, y) => source.objectAt(x, y, modelType);
    view.solidTerrainObjects = () => source.solidTerrainObjects(modelType);
    view.memberCollisionParameters = (positionA, positionB) =>
      source.memberCollisionParameters(positionA, positionB, modelType);
    return view;
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
  constructor(cfg, rover, terrain, modelType = 'fixed', routeLearner = null) {
    this.cfg = cfg;
    this.rover = rover;
    this.modelType = modelType; // 'fixed' or 'adaptive'
    this.terrainSource = terrain;
    this.terrain = typeof terrain.forModel === 'function'
      ? terrain.forModel(modelType)
      : terrain;
    this.routeLearner = modelType === 'adaptive' ? routeLearner : null;
    this.reset(false);
  }

  reset(learnFromCurrent = true) {
    if (learnFromCurrent && this.routeLearner && this.t > 0 && !this.runFinalized) {
      this.finalizeRun('reset');
    }
    this.t = 0.0;
    this.stepCount = 0;
    this.runFinalized = false;
    this.routeLearner?.beginRun();

    const n = this.rover.nOuter;
    const lowestLocalNode = Math.min(...this.rover.q0_outer.map(position => position[2]));
    // Course missions and the Level 14 open-world expedition both begin one
    // metre behind their configured start line; every other level spawns at
    // the origin as before.
    const initialY = (this.cfg.abCourseEnabled || this.cfg.experimentId === 14)
      ? this.cfg.courseStartY-1.0 : 0;
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
    // The expedition reuses the Level 10 crest-checkpoint tracker against its
    // own obstacle chain: Model B must crest E01…Enn in order before the
    // 800 m goal line scores.
    this.expeditionCourse = (!this.terrain.course && this.terrain.expeditionObstacles?.length)
      ? {
        roverDiameter: 2*this.cfg.outerRadius,
        detectionDistance: 2.40,
        maxY: this.cfg.courseMaxY || (this.cfg.targetGoalY || 800)+20,
        obstacles: this.terrain.expeditionObstacles
      }
      : null;
    const trackedCourse = this.terrain.course || this.expeditionCourse;
    this.obstacleTracker = trackedCourse
      ? new ObstaclePassTracker(trackedCourse, this.modelType)
      : null;
    // The Level 14 expedition has no A/B course object, so it gets its own
    // waypoint chain: one beacon every 50 m between the start and goal lines.
    // These drive the HUD progress counter and the learner's checkpoint memory.
    this.expeditionWaypoints = this.buildExpeditionWaypoints();
    this.nextWaypointIndex = 0;
    this.obstaclePhase = 'COURSE_APPROACH';
    this.activeObstacleId = null;
    this.lastObstacleProgressY = initialY;
    this.lastObstacleProgressAt = 0;
    this.obstacleRecoveryUntil = 0;
    this.measuredRunStartedAt = null;
    this.measuredRunCompletedAt = null;
    this.prevMetricPosition = [0, initialY];
    this.speedSampleStats = { count: 0, mean: 0, m2: 0 };
    this.learningCommand = this.routeLearner?.commandAt({ x: 0, y: initialY, grade: 0 }) || null;
    this.monitor = this.modelType === 'adaptive' && this.cfg.monitoring?.enabled
      ? new RealtimeMonitor(this.cfg, this.rover, this.terrain, this.q, [0, initialY, initialZ])
      : null;

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
      minimumTerrainClearances: {
        nodes: Infinity,
        bars: Infinity,
        outerCables: Infinity,
        core: Infinity,
        coreCables: Infinity
      },
      maximumPathOffset: 0.0,
      corridorBoundaryHits: 0,
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
      runTerminal: false,
      runOutcome: 'running',
      runDeadline: this.cfg.missionDeadlineSeconds || this.cfg.T_end,
      goalReached: false,
      goalResult: null,
      learning: this.routeLearner?.snapshot(this.learningCommand) || null,
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
      terrainClearanceComponents: null,
      terrainLiftCorrection: 0,
      pathCorridor: {
        lane: this.modelType === 'fixed' ? 'A' : 'B',
        worldOffset: this.modelType === 'fixed'
          ? -this.cfg.modelLaneOffset : this.cfg.modelLaneOffset,
        halfWidth: this.cfg.pathCorridorHalfWidth,
        centreX: 0,
        correction: 0,
        boundaryHit: false
      },
      corePosition: this.corePosition.slice(),
      coreVelocity: this.coreVelocity.slice(),
      coreCableForces: this.coreCableForces.slice(),
      obstaclePhase: this.obstaclePhase,
      activeObstacleId: null,
      obstacleSummary: this.metrics.obstacleSummary,
      actuationTraction: 0,
      actuationRollTorque: 0,
      monitoring: this.monitor?.latest || null,
      learning: this.metrics.learning,
      contacts: [],
      cableTelemetry: []
    };
  }

  // One beacon every waypointSpacing meters along the expedition corridor.
  // Waypoints are pure progress markers — they gate nothing in physics, they
  // just give the HUD and the route learner a per-50m record of the attempt.
  buildExpeditionWaypoints() {
    if (this.cfg.experimentId !== 14 || this.terrain.course) return [];
    const startY = this.cfg.courseStartY || 10;
    const goalY = this.cfg.targetGoalY || this.cfg.courseGoalY || 1000;
    const spacing = this.cfg.waypointSpacing || 50;
    const waypoints = [];
    for (let y = startY+spacing; y < goalY; y += spacing) {
      waypoints.push({ id: `WP${waypoints.length+1}`, y });
    }
    return waypoints;
  }

  updateExpeditionWaypoints(cx, cy) {
    while (this.nextWaypointIndex < this.expeditionWaypoints.length &&
      cy >= this.expeditionWaypoints[this.nextWaypointIndex].y) {
      const waypoint = this.expeditionWaypoints[this.nextWaypointIndex];
      this.nextWaypointIndex += 1;
      const kineticEnergy = this.currentDiag?.kineticEnergy || 0;
      this.routeLearner?.observeCheckpoint(cy, kineticEnergy);
      return waypoint; // one per step so HUD can flash the latest pass
    }
    return null;
  }

  finalizeRun(reason = 'timeout') {
    if (this.runFinalized || !this.metrics) return this.metrics?.learning?.lastRun || null;
    const reached = Boolean(this.metrics.courseComplete);
    const deadline = this.cfg.missionDeadlineSeconds || this.cfg.T_end;
    const outcome = reached && this.t <= deadline+1e-9 ? 'win' : 'loss';
    this.metrics.runOutcome = outcome;
    this.metrics.runTerminal = reason !== 'reset' && reason !== 'configuration_change';
    this.metrics.completionTime = reached ? this.t : null;
    const finalY = this.currentDiag?.centroid?.[1] ?? this.prevMetricPosition?.[1] ?? 0;
    const result = this.routeLearner?.finishRun({
      reached,
      time: this.t,
      finalY,
      maxSlip: this.metrics.maxSlipSpeed,
      rollingError: this.currentDiag?.rollingError || 0,
      lateralTravel: this.metrics.lateralTravel,
      reason
    }) || {
      attempt: 1,
      outcome,
      reason,
      time: this.t,
      finalY,
      remaining: Math.max(0, (this.cfg.courseGoalY || this.cfg.targetGoalY)-finalY)
    };
    this.metrics.learning = this.routeLearner?.snapshot(this.learningCommand) || {
      deadlineSeconds: deadline,
      runCount: 1,
      nextAttempt: 2,
      wins: outcome === 'win' ? 1 : 0,
      losses: outcome === 'loss' ? 1 : 0,
      bestTime: outcome === 'win' ? this.t : null,
      lastRun: result,
      currentCommand: this.learningCommand
    };
    this.runFinalized = true;
    return result;
  }

  senseObstacleAhead(cx, cy, cvy) {
    if (!this.terrain) return { detected: false, height: 0, distance: 0, steerSign: 0 };
    const scoredCourse = this.terrain.course || this.expeditionCourse;
    if (scoredCourse) {
      if (this.modelType === 'adaptive' && this.obstacleTracker) {
        const checkpoint = this.obstacleTracker.currentCheckpoint();
        if (!checkpoint) return { detected: false, height: 0, distance: Infinity, steerSign: 0, checkpoint: true };
        const longitudinalError = checkpoint.y-cy;
        const lateralError = cx-checkpoint.x;
        return {
          detected: true,
          obstacle: checkpoint,
          distance: Math.max(0, Math.abs(longitudinalError)-checkpoint.radiusY),
          height: checkpoint.height,
          lateralError,
          steerSign: lateralError >= 0 ? 1 : -1,
          checkpoint: true,
          longitudinalError
        };
      }
      return senseCourseObstacle(scoredCourse, cx, cy, cvy || 1);
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
      const rockRadius = Math.max(rock.rx || rock.r || 0.2, rock.ry || rock.r || 0.2);
      if (ahead > 0 && ahead < 1.8 && Math.abs(rock.x-cx) < rockRadius+0.8) {
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

  desiredDirectionForMode(centroid, obstacle, velocity = [0, 0, 0], learning = null) {
    const mode = this.cfg.actuationMode;
    if (mode === 'none') return [0, 0, 0];
    const corridorDirection = (baseX, baseY, pathTargetX = 0, gainScale = 1) => {
      const centreLimit = Math.max(0.12,
        this.cfg.pathCorridorHalfWidth-this.rover.R_outer-this.cfg.corridorSafetyMargin);
      const lateralError = clampValue(pathTargetX, -0.72*centreLimit, 0.72*centreLimit)-centroid[0];
      let correctedX = baseX
        +this.cfg.pathCenteringGain*gainScale*lateralError
        -this.cfg.pathCenteringDamping*velocity[0];
      const boundaryRatio = Math.abs(centroid[0])/centreLimit;
      if (boundaryRatio > 0.68) {
        correctedX -= Math.sign(centroid[0])*2.2*
          smoothStep01((boundaryRatio-0.68)/0.32);
      }
      correctedX = clampValue(correctedX, -1.35, 1.35);
      const length = Math.hypot(correctedX, baseY);
      return length > 1e-9 ? [correctedX/length, baseY/length, 0] : [0, 1, 0];
    };
    if (mode === 'roll_backward') return corridorDirection(0, -1);
    if (mode === 'steer_left') return corridorDirection(-0.55, 0.835);
    if (mode === 'steer_right') return corridorDirection(0.55, 0.835);
    const target = this.cfg.targetDestination || [0, this.cfg.targetGoalY || 25];
    let dx = 0;
    let dy = Math.sign(target[1]-centroid[1]) || 1;
    let pathTargetX = target[0];
    let pathGainScale = 1;
    if (this.modelType === 'adaptive' && this.terrain.course && obstacle?.detected) {
      // Model B must align with, and pass through, the obstacle footprint.
      // Use a bounded, forward-only course correction. Large diagonal and
      // reverse commands made the spherical shell weave and waste seconds at
      // each crest even after it had already entered the centre band.
      const routeTargetX = obstacle.checkpoint
        ? obstacle.obstacle.x
        : Number.isFinite(learning?.waypointX) ? learning.waypointX : obstacle.obstacle.x;
      pathTargetX = routeTargetX;
      pathGainScale = learning?.alignmentScale || 1;
      // Keep positive roll authority through the summit. Reverse only after a
      // clear missed-crest overshoot; reducing drive at the centre can strand
      // a tensegrity shell on the steep face before its payload reaches top.
      dy = obstacle.checkpoint && obstacle.longitudinalError < -0.55 ? -0.42 : 1.0;
      if (this.t < this.obstacleRecoveryUntil) dy = -0.30;
    } else if (this.modelType === 'fixed' && obstacle?.detected) {
      // Model A may choose a local side of the obstacle, but the correction
      // below always keeps that manoeuvre inside the assigned A corridor.
      dx += (obstacle.steerSign || 1)*Math.max(0, 1.4-obstacle.distance)*0.65;
    }
    return corridorDirection(dx, dy, pathTargetX, pathGainScale);
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
    const surfaceAhead = this.terrain.eval(centroid[0], centroid[1]);
    this.learningCommand = this.routeLearner?.commandAt({
      x: centroid[0],
      y: centroid[1],
      grade: surfaceAhead.dhdy,
      obstacle: obstacle?.obstacle || null
    }) || null;
    const kineticEnergy = this.currentDiag?.kineticEnergy || 0;
    this.routeLearner?.observe({
      time: this.t,
      x: centroid[0],
      y: centroid[1],
      speed: Math.hypot(velocity[0], velocity[1]),
      slip: this.currentDiag?.slipSpeed || 0,
      rollingError: this.currentDiag?.rollingError || 0,
      grade: surfaceAhead.dhdy,
      energy: kineticEnergy
    });
    const desiredDirection = this.desiredDirectionForMode(centroid, obstacle, velocity, this.learningCommand);

    // Drive-ported optimal solvers (Riccati LQR / iLQR / minimax iLQR /
    // projected QP-MPC) run on the reduced-order rolling model and decide the
    // actuation effort; the support-face state machine below still selects
    // which cables realize that effort geometrically.
    let advancedSolution = null;
    if (this.modelType === 'adaptive' && ADVANCED_MODES.has(cfg.controllerMode)) {
      const planarSpeed = velocity[0]*desiredDirection[0]+velocity[1]*desiredDirection[1];
      const slopeDisturbance = Math.min(0.6, Math.abs(surfaceAhead?.dhdy || 0));
      try {
        advancedSolution = solveAdvancedController(cfg.controllerMode, {
          state0: [planarSpeed, centroid[2]-(cfg.payloadTargetHeight || 0.55)],
          referenceSpeed: cfg.targetSpeed*(this.learningCommand?.speedScale || 1),
          disturbance: clampValue(
            slopeDisturbance+(obstacle?.detected ? Math.max(0, obstacle.height)*0.6 : 0), 0, 1),
          previousCommand: this.lastAdvancedCommand || 0,
          horizon: clampValue(Math.round((cfg.controlHorizon || 12)*0.9), 6, 18),
          weights: { qv: 6.0, qh: 4.0, r: 0.5+cfg.controllerInputPenalty*8,
            rd: cfg.controllerInputPenalty*4, qvT: 10.0, qhT: 6.0 },
          params: {
            thrust: 1.35, drag: 0.95, gradeLoss: 0.85,
            heightRelax: 2.2, rollCoupling: 0.10,
            dtH: Math.max(0.04, cfg.controllerDt)
          }
        });
        this.lastAdvancedCommand = advancedSolution.command;
      } catch {
        advancedSolution = null;
        this.lastAdvancedCommand = 0;
      }
    }

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
    // Optimal-effort modulation from the ported Drive solver: u ∈ [-1,1]
    // scales how aggressively the selected cables preload/tip the cage.
    if (advancedSolution && actuationFactor > 0) {
      actuationFactor = clampValue(
        actuationFactor*(0.62+0.48*advancedSolution.command), 0.18, 1.12);
    }
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
    const commandedDelta = Math.min(
      1.18*cfg.actuationDeltaL,
      (obstacleProfile?.delta || cfg.actuationDeltaL)*(this.learningCommand?.actuationScale || 1)
    );
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
      const learnedTargetSpeed = cfg.targetSpeed*(this.learningCommand?.speedScale || 1);
      const speedErrorBoost = clampValue(
        (learnedTargetSpeed-Math.max(0, speedAlongTarget))/Math.max(0.1, learnedTargetSpeed), 0, 1);
      const payloadStroke = cfg.coreActuationDeltaL*(0.80+0.20*speedErrorBoost)*actuationFactor;
      for (const item of projectedNodes.slice(0, 4)) coreTargets[item.node] = payloadStroke;
      for (const item of projectedNodes.slice(-2)) coreTargets[item.node] = -0.35*payloadStroke;
    }

    const destination = cfg.targetDestination || [0, cfg.targetGoalY || 25];
    const targetError = Math.hypot(destination[0]-centroid[0], destination[1]-centroid[1]);
    const targetEffort = cableTargets.reduce((sum, offset) => sum+offset*offset, 0);
    const advancedMode = this.modelType === 'adaptive' && ADVANCED_MODES.has(cfg.controllerMode) && advancedSolution;
    return {
      cableTargets,
      coreTargets,
      rodTargets: new Array(this.rover.bars.length).fill(0),
      diagnostics: {
        mode: advancedMode ? cfg.controllerMode : 'natural_support_face',
        modeLabel: advancedMode
          ? `${ADVANCED_CONTROLLER_LABELS[cfg.controllerMode]} · Support-Face Gait`
          : this.modelType === 'fixed'
            ? 'Model A · locked A corridor'
            : 'Model B · locked B corridor · strict over-obstacle',
        desiredDirection,
        predictedPath: [[centroid[0], centroid[1], centroid[2]], [centroid[0]+desiredDirection[0], centroid[1]+desiredDirection[1], centroid[2]]],
        controlCost: advancedMode
          ? advancedSolution.cost+targetError*0.02
          : targetError*0.02+targetEffort*50,
        learning: this.routeLearner?.snapshot(this.learningCommand) || null,
        activeCableCount: actuationFactor > 0 ? this.contractingCableIndices.length+this.relaxingCableIndices.length : 0,
        activeRodCount: 0,
        disturbanceEstimate: advancedMode ? advancedSolution.disturbanceEstimate :
          (obstacle?.detected ? obstacle.height : 0),
        horizon: cfg.controlHorizon,
        supportFace: support.supportFace,
        targetEdge: this.targetEdge,
        comMargin: support.comMargin,
        solverIterations: advancedMode ? advancedSolution.iterations : 0,
        solverConverged: advancedMode ? advancedSolution.converged : null,
        solverWorstCaseCost: advancedMode ? advancedSolution.worstCaseCost : null,
        solverMs: advancedMode ? advancedSolution.solveMs : null
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

  terrainClearanceReport(includeMembers = true) {
    if (!this.cfg.enableGround) {
      return { nodes: Infinity, bars: Infinity, outerCables: Infinity, core: Infinity, coreCables: Infinity, minimum: Infinity };
    }
    const report = { nodes: Infinity, bars: Infinity, outerCables: Infinity, core: Infinity, coreCables: Infinity };
    const segmentClearance = (positionA, positionB, radius) => {
      let minimum = Infinity;
      const parameters = this.terrain.memberCollisionParameters(positionA, positionB);
      for (const t of parameters) {
        const x = positionA[0]*(1-t)+positionB[0]*t;
        const y = positionA[1]*(1-t)+positionB[1]*t;
        const z = positionA[2]*(1-t)+positionB[2]*t;
        minimum = Math.min(minimum, z-this.terrain.eval(x, y).h-radius);
      }
      return minimum;
    };

    for (const position of this.q) {
      report.nodes = Math.min(report.nodes,
        position[2]-this.terrain.eval(position[0], position[1]).h-this.cfg.nodeRadius);
    }
    for (const [i, j] of this.rover.bars) {
      report.bars = Math.min(report.bars, segmentClearance(this.q[i], this.q[j], 0.035));
    }
    if (includeMembers) {
      for (const [i, j] of this.rover.outerStrings) {
        report.outerCables = Math.min(report.outerCables, segmentClearance(this.q[i], this.q[j], 0.012));
      }
      report.core = this.corePosition[2]
        -this.terrain.eval(this.corePosition[0], this.corePosition[1]).h-this.rover.R_core;
      for (let node = 0; node < this.q.length; node++) {
        report.coreCables = Math.min(report.coreCables,
          segmentClearance(this.q[node], this.corePosition, 0.006));
      }
    }
    report.minimum = Math.min(...Object.values(report));
    return report;
  }

  minimumTerrainClearance(includeMembers = true) {
    return this.terrainClearanceReport(includeMembers).minimum;
  }

  enforcePathCorridor() {
    const halfWidth = Math.max(
      this.rover.R_outer+0.10,
      this.cfg.pathCorridorHalfWidth || 1.25
    );
    const skin = this.cfg.corridorSafetyMargin || 0.015;
    let minimumX = this.corePosition[0]-this.rover.R_core;
    let maximumX = this.corePosition[0]+this.rover.R_core;
    for (const position of this.q) {
      minimumX = Math.min(minimumX, position[0]-this.cfg.nodeRadius);
      maximumX = Math.max(maximumX, position[0]+this.cfg.nodeRadius);
    }

    let correction = 0;
    if (maximumX > halfWidth-skin) correction = halfWidth-skin-maximumX;
    if (minimumX+correction < -halfWidth+skin) {
      correction += -halfWidth+skin-(minimumX+correction);
    }
    if (Math.abs(correction) > 1e-12) {
      for (const position of this.q) position[0] += correction;
      this.corePosition[0] += correction;
      const centroidVelocityX = (
        this.v.reduce((sum, velocity) => sum+velocity[0], 0)+this.coreVelocity[0]
      )/(this.v.length+1);
      const movingOutward = correction < 0 ? centroidVelocityX > 0 : centroidVelocityX < 0;
      if (movingOutward) {
        for (const velocity of this.v) velocity[0] -= centroidVelocityX;
        this.coreVelocity[0] -= centroidVelocityX;
      }
    }

    const centreX = this.q.reduce((sum, position) => sum+position[0], 0)/this.q.length;
    return {
      lane: this.modelType === 'fixed' ? 'A' : 'B',
      worldOffset: this.modelType === 'fixed'
        ? -this.cfg.modelLaneOffset : this.cfg.modelLaneOffset,
      halfWidth,
      centreX,
      correction,
      boundaryHit: Math.abs(correction) > 1e-12
    };
  }

  enforceTerrainNonPenetration() {
    // Apply hard contact to every rendered physical component: nodes, bars,
    // outer strings, payload core, and all twelve core suspension cables.
    // Corrections are local to the colliding member; there is no whole-body
    // teleport or artificial COM translation.
    let maximumLocalCorrection = 0;
    const memberContactMap = new Map();
    const segments = [];
    const clearanceReport = {
      nodes: Infinity,
      bars: Infinity,
      outerCables: Infinity,
      core: Infinity,
      coreCables: Infinity
    };
    const addSegment = (kind, id, reportKey, positionA, positionB, velocityA, velocityB, radius) => {
      segments.push({
        key: `${kind}:${id}`, kind, id, reportKey,
        positionA, positionB, velocityA, velocityB, radius
      });
    };
    this.rover.bars.forEach(([i, j], index) =>
      addSegment('rod', `R${index+1}`, 'bars', this.q[i], this.q[j], this.v[i], this.v[j], 0.035));
    this.rover.outerStrings.forEach(([i, j], index) =>
      addSegment('outer-cable', `C${String(index+1).padStart(2, '0')}`, 'outerCables',
        this.q[i], this.q[j], this.v[i], this.v[j], 0.012));
    this.q.forEach((position, index) =>
      addSegment('core-cable', `PC${String(index+1).padStart(2, '0')}`, 'coreCables',
        position, this.corePosition, this.v[index], this.coreVelocity, 0.006));

    const projectPoint = (position, velocity, radius) => {
      const surface = this.terrain.eval(position[0], position[1]);
      const clearance = position[2]-surface.h-radius;
      const minimumZ = surface.h+radius+this.cfg.terrainClearanceEpsilon;
      if (position[2] >= minimumZ) return clearance;
      const correction = minimumZ-position[2];
      position[2] = minimumZ;
      maximumLocalCorrection = Math.max(maximumLocalCorrection, correction);
      const normalLength = Math.hypot(surface.dhdx, surface.dhdy, 1);
      const normal = [-surface.dhdx/normalLength, -surface.dhdy/normalLength, 1/normalLength];
      const inwardSpeed = velocity[0]*normal[0]+velocity[1]*normal[1]+velocity[2]*normal[2];
      if (inwardSpeed < 0) {
        velocity[0] -= inwardSpeed*normal[0];
        velocity[1] -= inwardSpeed*normal[1];
        velocity[2] -= inwardSpeed*normal[2];
      }
      return this.cfg.terrainClearanceEpsilon;
    };

    const projectSegment = segment => {
      let deepest = null;
      let minimumClearance = Infinity;
      const parameters = this.terrain.memberCollisionParameters(segment.positionA, segment.positionB);
      for (const t of parameters) {
        const x = segment.positionA[0]*(1-t)+segment.positionB[0]*t;
        const y = segment.positionA[1]*(1-t)+segment.positionB[1]*t;
        const z = segment.positionA[2]*(1-t)+segment.positionB[2]*t;
        const surface = this.terrain.eval(x, y);
        const clearance = z-surface.h-segment.radius;
        minimumClearance = Math.min(minimumClearance, clearance);
        const correction = MEMBER_COLLISION_SKIN+this.cfg.terrainClearanceEpsilon-clearance;
        if (correction > 0 && (!deepest || correction > deepest.correction)) {
          deepest = { x, y, surface, correction, t };
        }
      }
      if (!deepest) return minimumClearance;
      segment.positionA[2] += deepest.correction;
      segment.positionB[2] += deepest.correction;
      maximumLocalCorrection = Math.max(maximumLocalCorrection, deepest.correction);
      const normalLength = Math.hypot(deepest.surface.dhdx, deepest.surface.dhdy, 1);
      const normal = [
        -deepest.surface.dhdx/normalLength,
        -deepest.surface.dhdy/normalLength,
        1/normalLength
      ];
      const sampleVelocity = normal.reduce((sum, component, axis) => sum+component*(
        segment.velocityA[axis]*(1-deepest.t)+segment.velocityB[axis]*deepest.t), 0);
      if (sampleVelocity < 0) {
        for (let axis = 0; axis < 3; axis++) {
          segment.velocityA[axis] -= sampleVelocity*normal[axis];
          segment.velocityB[axis] -= sampleVelocity*normal[axis];
        }
      }
      const previous = memberContactMap.get(segment.key);
      if (!previous || deepest.correction > previous.correction) {
        memberContactMap.set(segment.key, {
          kind: segment.kind,
          id: segment.id,
          objectId: this.terrain.objectAt(deepest.x, deepest.y),
          position: [deepest.x, deepest.y, deepest.surface.h],
          normal,
          normalForce: this.cfg.kg*Math.pow(deepest.correction, 1.5),
          frictionForce: [0, 0, 0],
          correction: deepest.correction
        });
      }
      return minimumClearance+deepest.correction;
    };

    for (let node = 0; node < this.q.length; node++) {
      clearanceReport.nodes = Math.min(clearanceReport.nodes,
        projectPoint(this.q[node], this.v[node], this.cfg.nodeRadius));
    }
    clearanceReport.core = projectPoint(
      this.corePosition, this.coreVelocity, this.rover.R_core);
    // Each correction translates both endpoints upward. Later corrections
    // can only increase the clearance of an already-checked segment, so one
    // conservative pass is sufficient and avoids a second full terrain scan.
    for (const segment of segments) {
      clearanceReport[segment.reportKey] = Math.min(
        clearanceReport[segment.reportKey], projectSegment(segment));
    }
    clearanceReport.minimum = Math.min(...Object.values(clearanceReport));
    return {
      clearance: clearanceReport.minimum,
      clearanceReport,
      lift: maximumLocalCorrection,
      memberContacts: [...memberContactMap.values()]
    };
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
    // only by the independent fixed-rate controller clock below.
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
    const contactDetails = [];
    let actuationTraction = 0;
    let actuationRollTorque = 0;
    if (cfg.enableGround) {
      const contactCandidates = [];
      for (let i = 0; i < n; i++) {
        const surf = this.terrain.eval(q[i][0], q[i][1]);
        const zGround = surf.h;
        const signedPenetration = (zGround + cfg.nodeRadius) - q[i][2];
        const beta = cfg.contactSmoothBeta;
        const smoothPenetration = 0.5*(Math.sqrt(signedPenetration*signedPenetration + beta*beta) + signedPenetration);
        const contactBlend = 0.5*(signedPenetration/Math.sqrt(signedPenetration*signedPenetration + beta*beta) + 1);
        if (signedPenetration > -4*beta) {
          contactCandidates.push({ i, surf, zGround, smoothPenetration, contactBlend });
        }
      }
      const totalMass = n*this.dynamicNodeMass+cfg.coreMass;
      const supportedWeightPerContact = totalMass*Math.abs(cfg.gravity[2])/
        Math.max(1, contactCandidates.length);
      for (const candidate of contactCandidates) {
        const { i, surf, zGround, smoothPenetration, contactBlend } = candidate;
        contactNodes.push(i);
        const normalLength = Math.hypot(surf.dhdx, surf.dhdy, 1);
        const normal = [-surf.dhdx/normalLength, -surf.dhdy/normalLength, 1/normalLength];
        const normalVelocity = v[i][0]*normal[0]+v[i][1]*normal[1]+v[i][2]*normal[2];
        const springForce = cfg.kg*Math.pow(smoothPenetration, 1.5);
        const dampingForce = -cfg.cg*contactBlend*Math.min(0, normalVelocity);
        const normalForce = Math.max(0, springForce+dampingForce);
        // A hard constraint at zero penetration still carries the rover's
        // supported weight. Using only node self-weight made the Coulomb
        // envelope far too small and caused Model B to skate over rocks.
        const reportedNormalForce = Math.max(normalForce, supportedWeightPerContact);
        const frictionForce = [0, 0, 0];

        fNode[i][0] += normalForce*normal[0];
        fNode[i][1] += normalForce*normal[1];
        fNode[i][2] += normalForce*normal[2];

        const tangentX = v[i][0]-normalVelocity*normal[0];
        const tangentY = v[i][1]-normalVelocity*normal[1];
        const tangentZ = v[i][2]-normalVelocity*normal[2];
        const slip = Math.hypot(tangentX, tangentY, tangentZ);
        if (slip > 1e-5) {
          let mu = slip < 0.04 ? cfg.mu_g : 0.82*cfg.mu_g;
          let dragScale = 1.0;
          // Marsh zones reduce friction (boggy ground, soft traction)
          if (this.terrain.marshes && this.terrain.marshes.length > 0) {
            const marshInfo = this.terrain.marshAt(q[i][0], q[i][1]);
            if (marshInfo.inMarsh) mu *= marshInfo.frictionScale;
          }
          // Fine sand & dust beds: loose granular grip plus extra viscous drag
          if (this.terrain.sandPatches && this.terrain.sandPatches.length > 0) {
            const sandInfo = this.terrain.sandAt(q[i][0], q[i][1]);
            if (sandInfo.inSand) {
              mu *= sandInfo.frictionScale;
              dragScale = sandInfo.dragScale;
            }
          }
          const frictionLimit = mu*reportedNormalForce;
          const effectiveSupportedMass = totalMass/Math.max(1, contactCandidates.length);
          const desiredFriction =
            effectiveSupportedMass*slip/0.025+cfg.c_gt*dragScale*slip;
          const friction = Math.min(frictionLimit, desiredFriction);
          frictionForce[0] = -friction*tangentX/slip;
          frictionForce[1] = -friction*tangentY/slip;
          frictionForce[2] = -friction*tangentZ/slip;
          fNode[i][0] += frictionForce[0];
          fNode[i][1] += frictionForce[1];
          fNode[i][2] += frictionForce[2];
        }
        contactDetails.push({
          kind: 'node',
          id: `N${i+1}`,
          nodeIndex: i,
          objectId: this.terrain.objectAt(q[i][0], q[i][1]),
          position: [q[i][0], q[i][1], zGround],
          normal: normal.slice(),
          normalForce: reportedNormalForce,
          frictionForce
        });
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
        const learnedTargetSpeed = cfg.targetSpeed*(this.learningCommand?.speedScale || 1);
        const targetRollingRate = learnedTargetSpeed/Math.max(this.rover.R_outer, 0.1);
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
          cfg.rollTorqueGain*(this.learningCommand?.torqueScale || 1)*
            (targetRollingRate-rollingRate)*phaseScale,
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
        const rollingConstraintSpeed = this.terrain.course
          ? Math.max(0, rollingSurfaceSpeed)
          : rollingSurfaceSpeed;
        const rollingError = rollingConstraintSpeed-forwardSpeed;
        let requested = totalMass*cfg.rollingConstraintGain*rollingError;
        if (this.terrain.course && Math.abs(rollingSurfaceSpeed) > 0) {
          // Level 10 speed/grade assistance is coupled to measured shell
          // rotation. A stationary shell therefore receives no forward force:
          // the zero-net-force roll couple must start and sustain locomotion.
          const rotationAuthority = smoothStep01(
            Math.abs(rollingSurfaceSpeed)/Math.max(0.20*learnedTargetSpeed, 0.08)
          );
          const speedError = clampValue(
            learnedTargetSpeed-forwardSpeed, -learnedTargetSpeed, learnedTargetSpeed);
          const surface = this.terrain.eval(cx, cy);
          const uphillGrade = Math.max(0, surface.dhdx*direction[0]+surface.dhdy*direction[1]);
          requested += totalMass*cfg.courseSpeedGain*(this.learningCommand?.tractionScale || 1)*
            speedError*rotationAuthority;
          requested += totalMass*Math.abs(cfg.gravity[2])*cfg.courseGradeCompensationGain*
            uphillGrade*rotationAuthority;
        }
        actuationTraction = clampValue(requested, -coulombLimit, coulombLimit);
        const perContact = actuationTraction/contactNodes.length;
        for (const node of contactNodes) {
          fNode[node][0] += perContact*direction[0];
          fNode[node][1] += perContact*direction[1];
        }
      }
    }

    // Repulsive obstacle field adapted from the Drive paper reproduction.
    // Stones come from the spatial hash window around each node, which is
    // exactly the set that could pass the sensing-radius test below.
    if (this.modelType === 'adaptive' && cfg.obstacleAvoidance) {
      const sensingRadius = cfg.obstacleSensingRadius;
      const gamma = cfg.obstacleAvoidanceExponent;
      const queryReach = sensingRadius+this.terrain.maxSolidScatterRadius;
      for (let i = 0; i < n; i++) {
        for (const rock of this.terrain.rocksNear(q[i][0], q[i][1], false)) {
          const dx = q[i][0] - rock.x;
          const dy = q[i][1] - rock.y;
          const centerDistance = Math.sqrt(dx*dx + dy*dy);
          const rockRadius = Math.max(rock.rx || rock.r || 0.2, rock.ry || rock.r || 0.2);
          const surfaceDistance = Math.max(0.04, centerDistance-rockRadius);
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

    // Keep each complete shell inside its own lane before resolving the
    // terrain at the corrected x positions. The translation is lateral only
    // and applies equally to every component, preserving all rod lengths.
    const corridorProjection = this.enforcePathCorridor();
    if (corridorProjection.boundaryHit) this.metrics.corridorBoundaryHits++;

    // Final hard collision projection catches sub-millimetre roundoff.
    const terrainProjection = this.enforceTerrainNonPenetration();
    for (const component of ['nodes', 'bars', 'outerCables', 'core', 'coreCables']) {
      this.metrics.minimumTerrainClearances[component] = Math.min(
        this.metrics.minimumTerrainClearances[component],
        terrainProjection.clearanceReport[component]
      );
    }

    // Coulomb-bounded velocity solve at the finalized contact positions.
    // Matching centroid translation to measured shell rotation converts spin
    // into rolling without a drag force: a non-rotating shell receives no
    // forward velocity. A smaller lateral correction dissipates side-skate.
    // Both models need this rolling-grip solve — without it the baseline
    // cannot couple rotation into translation on rough ground (Level 14)
    // and slips straight backward; the models differ by controller quality,
    // not by contact physics.
    if (cfg.enableGround && contactNodes.length) {
      let projectedCx = 0, projectedCy = 0, projectedCz = 0;
      let projectedVx = 0, projectedVy = 0, projectedVz = 0;
      for (let node = 0; node < n; node++) {
        projectedCx += q[node][0]; projectedCy += q[node][1]; projectedCz += q[node][2];
        projectedVx += v[node][0]; projectedVy += v[node][1]; projectedVz += v[node][2];
      }
      projectedCx /= n; projectedCy /= n; projectedCz /= n;
      projectedVx /= n; projectedVy /= n; projectedVz /= n;
      const directionRaw = this.controlDiagnostics.desiredDirection || [0, 1, 0];
      const directionLength = Math.hypot(directionRaw[0], directionRaw[1]);
      if (directionLength > 1e-9) {
        const direction = [directionRaw[0]/directionLength, directionRaw[1]/directionLength];
        const lateral = [-direction[1], direction[0]];
        const projectedOmega = this.calcAngularVelocityVector(
          projectedCx, projectedCy, projectedCz,
          projectedVx, projectedVy, projectedVz
        );
        const rollAxis = [direction[1], -direction[0]];
        const rollingSurfaceSpeed = -(
          projectedOmega[0]*rollAxis[0]+projectedOmega[1]*rollAxis[1]
        )*this.rover.R_outer;
        const forwardSpeed = projectedVx*direction[0]+projectedVy*direction[1];
        const lateralSpeed = projectedVx*lateral[0]+projectedVy*lateral[1];
        const maximumDelta = 0.82*cfg.mu_g*Math.abs(cfg.gravity[2])*dt;
        const forwardDelta = clampValue(
          (cfg.adaptiveContactGrip || 0.72)*(rollingSurfaceSpeed-forwardSpeed),
          -maximumDelta,
          maximumDelta
        );
        const lateralDelta = clampValue(
          -0.55*(cfg.adaptiveContactGrip || 0.72)*lateralSpeed,
          -0.65*maximumDelta,
          0.65*maximumDelta
        );
        const deltaX = forwardDelta*direction[0]+lateralDelta*lateral[0];
        const deltaY = forwardDelta*direction[1]+lateralDelta*lateral[1];
        for (const velocity of v) {
          velocity[0] += deltaX;
          velocity[1] += deltaY;
        }
        this.coreVelocity[0] += deltaX;
        this.coreVelocity[1] += deltaY;
      }
    }

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
    this.metrics.maximumPathOffset = Math.max(this.metrics.maximumPathOffset, Math.abs(cx));
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
      // Incremental mean/variance (Welford): a 5000 s expedition pushes one
      // sample per physics step, so a growing sample array would cost both
      // memory and an O(n) pass every step.
      const previousMean = this.speedSampleStats.mean;
      this.speedSampleStats.count += 1;
      this.speedSampleStats.mean += (curVel-previousMean)/this.speedSampleStats.count;
      this.speedSampleStats.m2 += (curVel-previousMean)*(curVel-this.speedSampleStats.mean);
      const measuredDuration = this.terrain.course
        ? this.t-(this.measuredRunStartedAt ?? this.t)
        : this.t;
      this.metrics.measuredTime = Math.max(0, measuredDuration);
      this.metrics.avgVelocity = this.metrics.measuredDistance/Math.max(0.001, measuredDuration);
      if (this.speedSampleStats.count > 1) {
        this.metrics.speedVariance =
          this.speedSampleStats.m2/(this.speedSampleStats.count-1);
      }
    }
    const allCrestsReached = this.modelType !== 'adaptive'
      || !this.obstacleTracker
      || this.obstacleTracker.summary().allCheckpointsReached;
    // Scored goals exist in two shapes: the Level 10 A-vs-B course (which
    // carries its own obstacle checkpoint chain) and the Level 14 open 1km
    // expedition scored directly against targetGoalY.
    const scoredRun = Boolean(this.terrain.course) || cfg.experimentId === 14;
    const goalLineY = this.terrain.course ? cfg.courseGoalY : this.cfg.targetGoalY;
    if (scoredRun && cy >= goalLineY && allCrestsReached && !this.metrics.courseComplete) {
      this.metrics.courseComplete = true;
      this.measuredRunCompletedAt = this.t;
      this.metrics.completionTime = this.t;
    }
    const passedWaypoint = this.updateExpeditionWaypoints(cx, cy);
    this.metrics.expeditionWaypoints = {
      total: this.expeditionWaypoints.length,
      reached: this.nextWaypointIndex,
      lastPassed: passedWaypoint?.id || null
    };
    if (curVel > this.metrics.maxVelocity) this.metrics.maxVelocity = curVel;
    if (postOmega > this.metrics.maxAngularVelocity) this.metrics.maxAngularVelocity = postOmega;
    if (kineticEnergy > this.metrics.peakKineticEnergy) this.metrics.peakKineticEnergy = kineticEnergy;
    if (slipSpeed > this.metrics.maxSlipSpeed) this.metrics.maxSlipSpeed = slipSpeed;
    this.metrics.completedRolls = this.completedRolls;
    if (this.obstacleTracker) {
      const baseHeight = this.terrain.evalBase(cx, cy).h;
      this.metrics.obstacleSummary = this.obstacleTracker.update({ x: cx, y: cy, z: cz, baseHeight, time: this.t });
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

    const monitoring = this.monitor?.sample({
      time: this.t,
      q,
      centroid: [cx, cy, cz],
      velocity: [cvx, cvy, cvz],
      cableForces: outerCableForces,
      actuationOffsets: this.currentActuationOffsets,
      relaxedFlags: this.relaxedCableFlags,
      contacts: [...contactDetails, ...(terrainProjection.memberContacts || [])],
      constraintError,
      terrainClearance: terrainProjection.clearance,
      terrainClearanceComponents: terrainProjection.clearanceReport,
      distanceTraveled: this.metrics.distanceTraveled
    }) || null;
    if (monitoring) {
      this.metrics.goalReached = monitoring.goalReached;
      this.metrics.goalResult = monitoring.goalResult;
    }

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
      terrainClearanceComponents: terrainProjection.clearanceReport,
      terrainLiftCorrection: terrainProjection.lift,
      pathCorridor: { ...corridorProjection, centreX: cx },
      corePosition: this.corePosition.slice(),
      coreVelocity: this.coreVelocity.slice(),
      coreCableForces: this.coreCableForces.slice(),
      obstaclePhase: this.obstaclePhase,
      activeObstacleId: this.activeObstacleId,
      obstacleSummary: this.metrics.obstacleSummary,
      gaitState: this.locomotionState,
      actuationTraction,
      actuationRollTorque,
      monitoring,
      learning: this.routeLearner?.snapshot(this.learningCommand) || this.metrics.learning,
      contacts: monitoring?.contacts || contactDetails,
      cableTelemetry: monitoring?.cables || []
    };

    if (scoredRun && !this.runFinalized) {
      const deadline = this.cfg.missionDeadlineSeconds || cfg.T_end;
      if (this.metrics.courseComplete) this.finalizeRun('goal');
      else if (this.t >= deadline-1e-9) this.finalizeRun('timeout');
      this.currentDiag.learning = this.metrics.learning;
    }

    if (this.stepCount % 20 === 0) {
      // Rolling window: a 5000 s expedition would otherwise retain ~125k
      // samples per series. The HUD charts only ever read the last 1000.
      if (this.history.t.length >= this.cfg.maxHistorySamples) {
        for (const series of Object.values(this.history)) series.shift();
      }
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

// EMPIRICAL 14-EXPERIMENT BENCHMARK ENGINE
export class BenchmarkEngine {
  static runAllExperiments(baseConfig) {
    const results = [];

    for (let expId = 1; expId <= 14; expId++) {
      let terrainLvl = expId === 10 ? 7 : Math.min(14, expId);
      let actMode = 'roll_forward';
      let freq = baseConfig.gaitFrequency || 0.20;
      let deltaL = baseConfig.actuationDeltaL || 0.12;

      if (expId === 8) {
        freq = 0.25;
        deltaL = 0.03;
      } else if (expId === 9) {
        actMode = 'roll_forward';
      } else if (expId === 10) {
        deltaL = 0.04;
      }

      const cfg = new SimConfig(Object.assign({}, baseConfig, {
        experimentId: expId,
        terrainLevel: terrainLvl,
        abCourseEnabled: expId === 10,
        targetGoalY: expId === 10 ? 60 : (expId === 14 ? 800 : 25),
        targetDestination: [0, expId === 10 ? 60 : (expId === 14 ? 800 : 25)],
        T_end: expId === 10 ? LEVEL10_PERFORMANCE.timeLimit : (expId === 14 ? 5000 : 40),
        actuationMode: actMode,
        gaitFrequency: freq,
        actuationDeltaL: deltaL,
        gravity: baseConfig.gravity || [0, 0, -9.81],
        pretensionS: baseConfig.pretensionS || 40.0,
        kS: baseConfig.kS || 1200.0,
        enableDiagnosticsLog: false
      }));
      if (expId === 10) cfg.applyLevel10PerformanceProfile();
      else cfg.applyStandardPerformanceProfile();

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
