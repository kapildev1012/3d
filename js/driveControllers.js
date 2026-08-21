/**
 * Browser-native controller profiles adapted from the MATLAB controllers in
 * the linked Drive project. The original implementations depend on MATLAB,
 * YALMIP, Gurobi and (for NN_RollingDirection) an external .mat weight file.
 * These profiles preserve their observable control features for the current
 * six-bar topology: destination tracking, payload-height regulation,
 * receding-horizon prediction, bounded disturbances, input-change penalties,
 * and hard cable/rod actuator constraints.
 */

export const CONTROLLER_LABELS = {
  cpg: 'CPG Baseline',
  lqr: 'LQR Rolling Direction',
  lqr_payload: 'LQR + Payload Stabilization',
  ilqr: 'Iterative LQR',
  ilqr_minimax: 'iLQR Minimax Robust',
  ilqr_minimax_penalty: 'iLQR Minimax + Input Penalty',
  qp_mpc: 'QP-MPC Constrained',
  qp_mpc_payload: 'QP-MPC + Payload Stabilization',
  neural: 'Neural Geometry Policy'
};

const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

function normalized3(x, y, z = 0) {
  const length = Math.sqrt(x*x + y*y + z*z);
  if (length < 1e-9) return [0, 1, 0];
  return [x/length, y/length, z/length];
}

function directionForGait(cfg, centroid, velocity, obstacle, profile) {
  const [cx, cy, cz] = centroid;
  const mode = cfg.actuationMode === 'bounce_jump'
    ? 'roll_forward'
    : cfg.actuationMode;
  let dx = 0;
  let dy = 1;
  let dz = 0;

  if (mode === 'roll_backward') {
    dy = -1;
  } else if (mode === 'steer_left') {
    dx = -0.72; dy = 0.70;
  } else if (mode === 'steer_right') {
    dx = 0.72; dy = 0.70;
  } else if (mode === 'none') {
    return [0, 0, 0];
  } else {
    const target = cfg.targetDestination || [0, cfg.targetGoalY || 25];
    dx = target[0] - cx;
    dy = target[1] - cy;
  }

  if (profile.momentumBlend > 0) {
    const momentum = normalized3(velocity[0], velocity[1], 0);
    dx = (1-profile.momentumBlend)*dx + profile.momentumBlend*momentum[0];
    dy = (1-profile.momentumBlend)*dy + profile.momentumBlend*momentum[1];
  }

  if (profile.payloadWeight > 0) {
    const neutralZ = cfg.payloadTargetHeight || 0.55;
    dz += profile.payloadWeight * (neutralZ - cz);
  }

  if (obstacle?.detected) {
    const steerSign = obstacle.steerSign || (cx >= 0 ? -1 : 1);
    dx += profile.obstacleGain * steerSign * Math.max(0, 1.6-obstacle.distance);
    dz += profile.obstacleGain * Math.max(0, obstacle.height);
  }

  if (cfg.simpleRollingMode) return normalized3(dx, dy, 0);
  return normalized3(dx, dy, dz);
}

function getProfile(mode) {
  const profiles = {
    cpg: {
      trackingGain: 0.55, phaseGain: 0.50, speedGain: 0.32,
      cablePenalty: 0.10, inputPenalty: 0.05, payloadWeight: 0,
      momentumBlend: 0, robustGain: 0, obstacleGain: 0.10, horizonScale: 0.45
    },
    lqr: {
      trackingGain: 0.82, phaseGain: 0.12, speedGain: 0.55,
      cablePenalty: 0.50, inputPenalty: 0.18, payloadWeight: 0.10,
      momentumBlend: 0, robustGain: 0, obstacleGain: 0.25, horizonScale: 0.65
    },
    lqr_payload: {
      trackingGain: 0.78, phaseGain: 0.10, speedGain: 0.48,
      cablePenalty: 0.72, inputPenalty: 0.22, payloadWeight: 1.10,
      momentumBlend: 0, robustGain: 0.08, obstacleGain: 0.30, horizonScale: 0.70
    },
    ilqr: {
      trackingGain: 1.00, phaseGain: 0.28, speedGain: 0.62,
      cablePenalty: 0.30, inputPenalty: 0.35, payloadWeight: 0.55,
      momentumBlend: 0.25, robustGain: 0.08, obstacleGain: 0.38, horizonScale: 1.00
    },
    ilqr_minimax: {
      trackingGain: 0.92, phaseGain: 0.22, speedGain: 0.55,
      cablePenalty: 0.42, inputPenalty: 0.42, payloadWeight: 0.65,
      momentumBlend: 0.18, robustGain: 0.48, obstacleGain: 0.55, horizonScale: 1.05
    },
    ilqr_minimax_penalty: {
      trackingGain: 0.86, phaseGain: 0.18, speedGain: 0.50,
      cablePenalty: 0.58, inputPenalty: 0.78, payloadWeight: 0.72,
      momentumBlend: 0.16, robustGain: 0.58, obstacleGain: 0.62, horizonScale: 1.10
    },
    qp_mpc: {
      trackingGain: 0.90, phaseGain: 0.08, speedGain: 0.58,
      cablePenalty: 0.65, inputPenalty: 0.62, payloadWeight: 0.35,
      momentumBlend: 0.12, robustGain: 0.20, obstacleGain: 0.52, horizonScale: 1.25
    },
    qp_mpc_payload: {
      trackingGain: 0.86, phaseGain: 0.06, speedGain: 0.52,
      cablePenalty: 0.82, inputPenalty: 0.68, payloadWeight: 1.25,
      momentumBlend: 0.10, robustGain: 0.26, obstacleGain: 0.58, horizonScale: 1.30
    },
    neural: {
      trackingGain: 0.88, phaseGain: 0.16, speedGain: 0.52,
      cablePenalty: 0.36, inputPenalty: 0.28, payloadWeight: 0.30,
      momentumBlend: 0.15, robustGain: 0.12, obstacleGain: 0.42, horizonScale: 0.82
    }
  };
  return profiles[mode] || profiles.ilqr_minimax_penalty;
}

function neuralCablePolicy(front, side, vertical, phase, speedError, cableDeviation) {
  // Geometry-aligned fallback for the Drive NN controller. Its referenced
  // net2 .mat weights are not present in the shared folder, so this compact
  // tanh network operates on the same rotated-node/cable-state features.
  const h1 = Math.tanh(1.9*front - 1.15*vertical + 0.45*speedError);
  const h2 = Math.tanh(-0.75*side + 0.70*Math.sin(phase) - 0.55*cableDeviation);
  const h3 = Math.tanh(0.90*front*vertical + 0.35*Math.cos(phase));
  return Math.tanh(0.95*h1 + 0.42*h2 - 0.28*h3);
}

export function computeDriveControllerTargets(args) {
  const {
    cfg, rover, q, v, centroid, velocity, gaitPhase, obstacle,
    currentCableOffsets, currentRodOffsets, previousCableTargets,
    modelType, sensorNoise = [0, 0, 0]
  } = args;

  const requestedMode = modelType === 'fixed' ? 'cpg' : (cfg.controllerMode || 'ilqr_minimax_penalty');
  const profile = getProfile(requestedMode);
  const sensedCentroid = [
    centroid[0] + sensorNoise[0],
    centroid[1] + sensorNoise[1],
    centroid[2] + sensorNoise[2]
  ];
  const desiredDirection = directionForGait(cfg, sensedCentroid, velocity, obstacle, profile);
  const planarSpeed = velocity[0]*desiredDirection[0] + velocity[1]*desiredDirection[1];
  const targetSpeed = cfg.targetSpeed || 0.5;
  const speedError = targetSpeed - planarSpeed;
  const amplitude = cfg.actuationMode === 'none' ? 0 : clamp(
    cfg.actuationDeltaL * (0.55 + 0.55*Math.tanh(1.4*speedError)),
    0,
    cfg.actuationDeltaL
  );
  const disturbanceEstimate = profile.robustGain * (
    Math.abs(sensorNoise[0]) + Math.abs(sensorNoise[1]) +
    (obstacle?.detected ? Math.max(0, obstacle.height) : 0)
  );

  const cableTargets = new Array(rover.outerStrings.length).fill(0);
  let effort = 0;
  let cableDeviationCost = 0;
  let activeCableCount = 0;

  for (let s = 0; s < rover.outerStrings.length; s++) {
    const [i, j] = rover.outerStrings[s];
    const mx = (q[i][0] + q[j][0]) * 0.5 - centroid[0];
    const my = (q[i][1] + q[j][1]) * 0.5 - centroid[1];
    const mz = (q[i][2] + q[j][2]) * 0.5 - centroid[2];
    const scale = Math.max(rover.R_outer, 1e-6);
    const front = (mx*desiredDirection[0] + my*desiredDirection[1]) / scale;
    const side = (-mx*desiredDirection[1] + my*desiredDirection[0]) / scale;
    const vertical = mz / scale;
    const bodyPhase = Math.atan2(front, vertical || 1e-9);
    const cableDeviation = currentCableOffsets[s] / Math.max(rover.l0_outerStrings[s], 1e-6);

    let normalizedCommand;
    if (requestedMode === 'neural') {
      normalizedCommand = neuralCablePolicy(front, side, vertical, gaitPhase+bodyPhase, speedError, cableDeviation);
    } else {
      const rollingShape = Math.tanh(2.4*front) * (0.82 - 0.28*vertical);
      const phaseWave = Math.sin(gaitPhase + bodyPhase);
      const speedTerm = profile.speedGain * speedError * Math.tanh(2*front);
      const robustTerm = profile.robustGain * disturbanceEstimate * Math.sign(front || 1);
      normalizedCommand = Math.tanh(
        profile.trackingGain*rollingShape + profile.phaseGain*phaseWave +
        speedTerm - profile.cablePenalty*cableDeviation - robustTerm
      );
    }

    let target = amplitude * normalizedCommand;
    const previousTarget = previousCableTargets[s] || 0;
    const smoothing = clamp(profile.inputPenalty, 0, 0.92);
    target = previousTarget*smoothing + target*(1-smoothing);

    const base = rover.l0_outerStrings[s];
    const minRest = base * cfg.cableMinRatio;
    const maxRest = base * cfg.cableMaxRatio;
    target = clamp(target, base-maxRest, base-minRest);

    if (modelType === 'fixed') target *= 0.55;
    cableTargets[s] = target;
    const delta = target - previousTarget;
    effort += delta*delta;
    cableDeviationCost += target*target;
    if (Math.abs(target) > 0.02) activeCableCount++;
  }

  const rodTargets = new Array(rover.bars.length).fill(0);
  if (modelType === 'adaptive' && (cfg.actuatorMode === 'rods' || cfg.actuatorMode === 'both')) {
    for (let b = 0; b < rover.bars.length; b++) {
      const [i, j] = rover.bars[b];
      const mx = (q[i][0] + q[j][0]) * 0.5 - centroid[0];
      const my = (q[i][1] + q[j][1]) * 0.5 - centroid[1];
      const mz = (q[i][2] + q[j][2]) * 0.5 - centroid[2];
      const front = (mx*desiredDirection[0] + my*desiredDirection[1]) / Math.max(rover.R_outer, 1e-6);
      const vertical = mz / Math.max(rover.R_outer, 1e-6);
      const base = rover.l0_bars[b];
      const target = 0.22*cfg.actuationDeltaL*Math.tanh(1.7*front-0.65*vertical);
      rodTargets[b] = clamp(target, base*(cfg.rodMinRatio-1), base*(cfg.rodMaxRatio-1));
    }
  }

  const activeRodCount = rodTargets.reduce((count, targetValue) => count + (Math.abs(targetValue) > 0.002 ? 1 : 0), 0);
  if (cfg.actuatorMode === 'rods') {
    cableTargets.fill(0);
    activeCableCount = 0;
  }

  const target = cfg.targetDestination || [0, cfg.targetGoalY || 25];
  const destinationError = Math.hypot(target[0]-centroid[0], target[1]-centroid[1]);
  const controlCost = destinationError*0.08 + speedError*speedError*2.0 +
    cableDeviationCost*profile.cablePenalty + effort*(1+6*profile.inputPenalty) +
    disturbanceEstimate*disturbanceEstimate;

  const horizon = Math.max(2, Math.round(cfg.controlHorizon || 12));
  const horizonDt = Math.max(0.04, cfg.controllerDt || 0.08) * profile.horizonScale;
  const predictedPath = [];
  for (let k = 0; k <= horizon; k += Math.max(1, Math.floor(horizon/6))) {
    const tk = k*horizonDt;
    const predictedSpeed = clamp(planarSpeed + speedError*(1-Math.exp(-1.4*tk)), -2, 2);
    predictedPath.push([
      centroid[0] + desiredDirection[0]*predictedSpeed*tk,
      centroid[1] + desiredDirection[1]*predictedSpeed*tk,
      centroid[2] + desiredDirection[2]*0.25*Math.sin(Math.min(Math.PI, tk))
    ]);
  }

  return {
    cableTargets,
    rodTargets,
    diagnostics: {
      mode: requestedMode,
      modeLabel: CONTROLLER_LABELS[requestedMode],
      desiredDirection,
      predictedPath,
      controlCost: Number.isFinite(controlCost) ? controlCost : 0,
      activeCableCount,
      activeRodCount,
      disturbanceEstimate,
      horizon,
      neuralFallback: requestedMode === 'neural'
    }
  };
}

export function relaxedCableTension(ell, activeRestLength, baseRestLength, cfg) {
  const kS = cfg.kS;
  const standard = kS * Math.max(0, ell-activeRestLength);
  if (!cfg.stringRelaxation) return { tension: standard, relaxed: false };

  const z1 = baseRestLength * cfg.relaxStartRatio;
  const z2 = baseRestLength * cfg.relaxEndRatio;
  if (ell <= z1) return { tension: standard, relaxed: false };
  if (ell >= z2) {
    // The source paper's force plateaus at beta. On a closed tensegrity cage,
    // an unlimited plateau lets members separate indefinitely, so retain that
    // plateau region and add a high-extension safety envelope beyond it.
    const safetyStart = baseRestLength * (cfg.relaxSafetyRatio || 1.55);
    const safetyStiffness = kS * (cfg.relaxSafetyStiffnessRatio || 2.50);
    return {
      tension: cfg.relaxedTension + safetyStiffness*Math.max(0, ell-safetyStart),
      relaxed: true
    };
  }

  const s = clamp((ell-z1)/Math.max(z2-z1, 1e-6), 0, 1);
  const h00 = 2*s*s*s - 3*s*s + 1;
  const h10 = s*s*s - 2*s*s + s;
  const h01 = -2*s*s*s + 3*s*s;
  const f1 = kS * Math.max(0, z1-activeRestLength);
  const tension = h00*f1 + h10*(z2-z1)*kS + h01*cfg.relaxedTension;
  return { tension: Math.max(cfg.relaxedTension, tension), relaxed: true };
}
