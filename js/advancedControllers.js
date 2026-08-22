/**
 * ADVANCED CONTROLLERS — faithful browser ports of the Drive MATLAB suite
 * (drive_matlab/Controllers/*.m) operating on a reduced-order rolling model
 * identified from the live tensegrity state each control tick.
 *
 * Ported algorithm cores:
 *  - LQR_RollingDirection.m      → finite-horizon Riccati backward pass,
 *    discrete Euler system like the source, feedback u = −K(x−xref)+uff.
 *  - iLQR_RollingDirection.m     → nonlinear rollout, numerical dynamics
 *    Jacobians, quadratic cost expansion, regularized backward pass,
 *    backtracking line-search forward pass (DDP/iLQR standard form).
 *  - iLQRminimax_*_inputpenalty.m → alternating input descent / adversarial
 *    disturbance ascent with an input-change penalty term.
 *  - QP_MPC_RollingDirection.m   → receding-horizon optimization with hard
 *    box actuator constraints solved by projected gradient descent.
 *
 * Reduced-order model per tick (state x = [speedAlongGoal, heightDeviation]):
 *   s₁' = s₁ + dt·(thrust·tanh(u) − drag·s₁ − gradeLoss·d)
 *   s₂' = s₂ + dt·(−heightRelax·s₂ + rollCoupling·tanh(u))
 * u ∈ [-1,1] normalized cable command; d ≥ 0 measured disturbance level.
 */

export const ADVANCED_CONTROLLER_LABELS = {
  riccati_lqr: 'LQR · Riccati Backward Pass',
  ilqr_true: 'iLQR · Nonlinear Rollout',
  ilqr_minimax_true: 'Robust iLQR · Adversarial Disturbance',
  qp_mpc_proj: 'QP-MPC · Projected Gradient'
};

export const ADVANCED_MODES = new Set(Object.keys(ADVANCED_CONTROLLER_LABELS));

const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));

function makeModel(params) {
  const { thrust, drag, gradeLoss, heightRelax, rollCoupling, dtH } = params;
  return {
    params,
    step(speed, height, u, d) {
      const drive = Math.tanh(u);
      return [
        clamp(speed+dtH*(thrust*drive-drag*speed-gradeLoss*d), -2.5, 2.5),
        clamp(height+dtH*(-heightRelax*height+rollCoupling*drive), -0.6, 0.6)
      ];
    }
  };
}

function stageCost(state, u, previousU, weights, referenceSpeed) {
  const speedError = state[0]-referenceSpeed;
  return weights.qv*speedError*speedError + weights.qh*state[1]*state[1] +
    weights.r*u*u + weights.rd*(u-previousU)*(u-previousU);
}

function rollout(model, state0, inputs, dSeq, weights, referenceSpeed, previousCommand) {
  const states = [state0.slice()];
  const usedInputs = [];
  let previous = previousCommand;
  let cost = 0;
  for (let k = 0; k < inputs.length; k++) {
    const u = inputs[k];
    const d = Array.isArray(dSeq) ? dSeq[k] : dSeq;
    usedInputs.push(u);
    cost += stageCost(states[k], u, previous, weights, referenceSpeed);
    states.push(model.step(states[k][0], states[k][1], u, d));
    previous = u;
  }
  const terminalError = states[states.length-1][0]-referenceSpeed;
  cost += weights.qvT*terminalError*terminalError +
    weights.qhT*states[states.length-1][1]*states[states.length-1][1];
  return { states, inputs: usedInputs, cost };
}

function feedforwardCommand(params, referenceSpeed, disturbance) {
  return clamp(
    (params.drag*referenceSpeed+params.gradeLoss*disturbance)/params.thrust,
    -1, 1);
}

// ---------------------------------------------------------------------------
// 1. Finite-horizon Riccati LQR (LQR_RollingDirection.m port)
// ---------------------------------------------------------------------------

/** P ← Q + AᵀPA − AᵀPB(R+BᵀPB)⁻¹BᵀPA, K ← (R+BᵀPB)⁻¹BᵀPA. */
function riccatiBackwardPass(Adt, Bcol, qv, qh, r, horizon) {
  let P = [[qv, 0], [0, qh]];
  let K = [0, 0];
  for (let k = 0; k < horizon; k++) {
    // PA = P·A, PB = P·b
    const PA = [
      [P[0][0]*Adt[0][0]+P[0][1]*Adt[1][0], P[0][0]*Adt[0][1]+P[0][1]*Adt[1][1]],
      [P[1][0]*Adt[0][0]+P[1][1]*Adt[1][0], P[1][0]*Adt[0][1]+P[1][1]*Adt[1][1]]
    ];
    const PB = [P[0][0]*Bcol[0]+P[0][1]*Bcol[1], P[1][0]*Bcol[0]+P[1][1]*Bcol[1]];
    // AᵀPB equals BᵀPA here because P remains symmetric through the recursion.
    const AtPA = matMul2x2(matTranspose2x2(Adt), PA);
    const AtPB = [Bcol[0]*PA[0][0]+Bcol[1]*PA[1][0],
      Bcol[0]*PA[0][1]+Bcol[1]*PA[1][1]];
    const BtPA = [PB[0]*Adt[0][0]+PB[1]*Adt[0][1], PB[0]*Adt[1][0]+PB[1]*Adt[1][1]];
    const BtPB = PB[0]*Bcol[0]+PB[1]*Bcol[1];
    if (!Number.isFinite(BtPB+r) || BtPB+r <= 1e-12) throw new Error('Riccati: non-positive Quu');
    const invQuu = 1/(r+BtPB);
    K = [invQuu*BtPA[0], invQuu*BtPA[1]];
    P = [
      [AtPA[0][0]+qv-AtPB[0]*K[0], AtPA[0][1]-AtPB[0]*K[1]],
      [AtPA[1][0]-AtPB[1]*K[0], AtPA[1][1]+qh-AtPB[1]*K[1]]
    ];
    ensureFiniteMatrix(P, `Riccati P iteration ${k}`);
  }
  return { P, K };
}

function solveRiccatiLqr(ctx) {
  const { model, state0, disturbance, referenceSpeed, weights, horizon, previousCommand } = ctx;
  const p = model.params;
  // Continuous-time A,B discretized with forward Euler, matching the drive
  // controller's Aaug/Baug construction at reduced order.
  const Adt = [
    [1-p.dtH*p.drag, 0],
    [p.dtH*p.rollCoupling, 1-p.dtH*p.heightRelax]
  ];
  const Bcol = [p.dtH*p.thrust, p.dtH*p.rollCoupling];
  const { K } = riccatiBackwardPass(Adt, Bcol, weights.qv, weights.qh, weights.r, horizon);

  const uff = feedforwardCommand(p, referenceSpeed, disturbance);
  const closed = rolloutWithLaw(model, state0, disturbance, weights, referenceSpeed,
    previousCommand, K, uff, horizon);
  return {
    command: closed.inputs[0],
    cost: closed.cost,
    iterations: horizon,
    converged: Number.isFinite(closed.cost),
    worstCaseCost: closed.cost,
    gain: K,
    feedforward: uff
  };
}

function rolloutWithLaw(model, state0, disturbance, weights, referenceSpeed, previousCommand, K, uff, horizon) {
  const inputs = new Array(horizon).fill(0);
  const states = [state0.slice()];
  let previous = previousCommand;
  let cost = 0;
  for (let k = 0; k < horizon; k++) {
    const u = clamp(-(K[0]*(states[k][0]-referenceSpeed)+K[1]*states[k][1])+uff, -1, 1);
    inputs[k] = u;
    cost += stageCost(states[k], u, previous, weights, referenceSpeed);
    states.push(model.step(states[k][0], states[k][1], u, disturbance));
    previous = u;
  }
  const finalState = states[horizon];
  cost += weights.qvT*(finalState[0]-referenceSpeed)**2 + weights.qhT*finalState[1]**2;
  return { inputs, states, cost, finalState };
}

// ---------------------------------------------------------------------------
// 2./3. iLQR and robust minimax iLQR (iLQR / iLQRminimax ports)
// ---------------------------------------------------------------------------

function dynamicsJacobians(model, x, u, d) {
  const eps = 1e-4;
  const step = (sx, sy, su) => model.step(sx, sy, su, d);
  return {
    fx: [
      [(step(x[0]+eps, x[1], u)[0]-step(x[0]-eps, x[1], u)[0])/(2*eps),
        (step(x[0], x[1]+eps, u)[0]-step(x[0], x[1]-eps, u)[0])/(2*eps)],
      [(step(x[0]+eps, x[1], u)[1]-step(x[0]-eps, x[1], u)[1])/(2*eps),
        (step(x[0], x[1]+eps, u)[1]-step(x[0], x[1]-eps, u)[1])/(2*eps)]
    ],
    fu: [
      (model.step(x[0], x[1], u+eps, d)[0]-model.step(x[0], x[1], u-eps, d)[0])/(2*eps),
      (model.step(x[0], x[1], u+eps, d)[1]-model.step(x[0], x[1], u-eps, d)[1])/(2*eps)
    ]
  };
}

function solveIlqr(ctx, options = {}) {
  const { model, state0, disturbance, referenceSpeed, weights, horizon, previousCommand } = ctx;
  const maxOuter = options.maxOuter ?? 4;
  const adversarial = options.adversarial ?? false;
  const dMax = options.dMax ?? 0.5;

  let dLevel = disturbance;
  let dSeq = new Array(horizon).fill(dLevel);

  const uff = feedforwardCommand(model.params, referenceSpeed, disturbance);
  let inputs = new Array(horizon).fill(uff);
  let best = rollout(model, state0, inputs, dSeq, weights, referenceSpeed, previousCommand);
  let iterations = 0;

  for (let outer = 0; outer < maxOuter; outer++) {
    const traj = rollout(model, state0, inputs, dSeq, weights, referenceSpeed, previousCommand);

    // --- regularized backward pass ---
    // Terminal expansion of the quadratic terminal cost.
    let vx = [
      2*weights.qvT*(traj.states[horizon][0]-referenceSpeed),
      2*weights.qhT*traj.states[horizon][1]
    ];
    let vxx = [[2*weights.qvT, 0], [0, 2*weights.qhT]];
    const kSeq = new Array(horizon).fill(0);
    const KSeq = new Array(horizon).fill(null);
    let lambda = outer === 0 ? 0 : 1.0;
    let backwardOk = true;
    for (let k = horizon-1; k >= 0; k--) {
      const x = traj.states[k];
      const u = traj.inputs[k];
      const prevU = k === 0 ? previousCommand : traj.inputs[k-1];
      const { fx, fu } = dynamicsJacobians(model, x, u, dSeq[k]);
      const speedError = x[0]-referenceSpeed;
      const lx = [2*weights.qv*speedError, 2*weights.qh*x[1]];
      const lu = 2*(weights.r*u+weights.rd*(u-prevU));
      const lxx = [[2*weights.qv, 0], [0, 2*weights.qh]];
      const luu = 2*(weights.r+weights.rd);

      const fxtVxx = matMul2x2(matTranspose2x2(fx), vxx); // 2×2
      const fxt = matTranspose2x2(fx);
      const fxtVx = [fxt[0][0]*vx[0]+fxt[0][1]*vx[1], fxt[1][0]*vx[0]+fxt[1][1]*vx[1]];
      const futVxxFu = dot2(fu, matVec2(vxx, fu));
      const Qx = [lx[0]+fxtVx[0], lx[1]+fxtVx[1]];
      const Qu = lu+dot2(fu, vx);
      const Qxx = matAdd2x2(lxx, matMul2x2(fxtVxx, fx));
      const Qux = [fxtVxx[0][0]*fu[0]+fxtVxx[0][1]*fu[1], fxtVxx[1][0]*fu[0]+fxtVxx[1][1]*fu[1]];
      const QuuReg = luu+futVxxFu+lambda;
      if (!Number.isFinite(QuuReg) || QuuReg < 1e-9) { backwardOk = false; break; }
      const invQuu = 1/QuuReg;
      const kk = -invQuu*Qu;
      const KK = [-invQuu*Qux[0], -invQuu*Qux[1]];
      kSeq[k] = kk;
      KSeq[k] = KK;
      // Value-function update (completed-square form):
      //   V_x  = Q_x − Q_uxᵀ Quu⁻¹ Q_u
      //   V_xx = Q_xx − Q_uxᵀ Quu⁻¹ Q_ux
      vx = [Qx[0]-Qux[0]*invQuu*Qu, Qx[1]-Qux[1]*invQuu*Qu];
      vxx = [
        [Qxx[0][0]-Qux[0]*invQuu*Qux[0], Qxx[0][1]-Qux[0]*invQuu*Qux[1]],
        [Qxx[1][0]-Qux[1]*invQuu*Qux[0], Qxx[1][1]-Qux[1]*invQuu*Qux[1]]
      ];
      ensureFiniteMatrix(vxx, `iLQR Vxx at k=${k}`);
    }
    if (!backwardOk) break;

    // --- forward pass with backtracking line search ---
    let accepted = false;
    for (const alpha of [1, 0.5, 0.25, 0.1, 0.05]) {
      const candidateInputs = new Array(horizon).fill(0);
      let current = state0.slice();
      for (let k = 0; k < horizon; k++) {
        const dx = [current[0]-traj.states[k][0], current[1]-traj.states[k][1]];
        const du = alpha*kSeq[k]+dot2(KSeq[k] || [0, 0], dx);
        candidateInputs[k] = clamp(traj.inputs[k]+du, -1, 1);
        current = model.step(current[0], current[1], candidateInputs[k], dSeq[k]);
      }
      const candidate = rollout(model, state0, candidateInputs, dSeq, weights, referenceSpeed, previousCommand);
      iterations++;
      if (candidate.cost < best.cost-1e-7) {
        best = candidate;
        inputs = candidateInputs;
        accepted = true;
        break;
      }
    }
    if (!accepted && outer > 0) break;

    // --- adversarial disturbance ascent between input descents ---
    if (adversarial && accepted) {
      const epsD = 1e-3;
      const costPlus = rollout(model, state0, inputs,
        new Array(horizon).fill(dLevel+epsD), weights, referenceSpeed, previousCommand).cost;
      const lowerLevel = Math.max(0, dLevel-epsD);
      const costMinus = rollout(model, state0, inputs,
        new Array(horizon).fill(lowerLevel), weights, referenceSpeed, previousCommand).cost;
      const gradD = (costPlus-costMinus)/(dLevel+epsD-lowerLevel);
      dLevel = clamp(dLevel+0.35*gradD, Math.max(0, disturbance-dMax), disturbance+dMax);
      dSeq = new Array(horizon).fill(dLevel);
      best = rollout(model, state0, inputs, dSeq, weights, referenceSpeed, previousCommand);
    }
  }

  // Worst-case assessment: committed input under maximum credible disturbance.
  const worstCase = adversarial
    ? rollout(model, state0, inputs, new Array(horizon).fill(disturbance+dMax), weights, referenceSpeed, previousCommand).cost
    : best.cost;

  return {
    command: inputs[0],
    cost: best.cost,
    iterations,
    converged: iterations > 0 && Number.isFinite(best.cost),
    worstCaseCost: worstCase,
    inputSequence: inputs.slice(),
    disturbanceEstimate: dLevel
  };
}

// ---------------------------------------------------------------------------
// 4. Box-constrained projected-gradient MPC (QP_MPC port)
// ---------------------------------------------------------------------------

function solveProjectedMpc(ctx) {
  const { model, state0, disturbance, referenceSpeed, weights, horizon, previousCommand } = ctx;
  const costOf = seq => rollout(model, state0, seq, disturbance, weights, referenceSpeed, previousCommand).cost;
  const uff = feedforwardCommand(model.params, referenceSpeed, disturbance);
  let inputs = new Array(horizon).fill(uff);
  let best = costOf(inputs);
  const zeroInputCost = costOf(new Array(horizon).fill(previousCommand));
  let iterations = 0;
  for (let iter = 0; iter < 12; iter++) {
    const eps = 1e-3;
    const gradient = inputs.map((u, index) => {
      const plus = clamp(u+eps, -1, 1);
      const minus = clamp(u-eps, -1, 1);
      const denom = plus-minus;
      if (denom < 1e-12) return 0;
      const seqPlus = inputs.slice(); seqPlus[index] = plus;
      const seqMinus = inputs.slice(); seqMinus[index] = minus;
      return (costOf(seqPlus)-costOf(seqMinus))/denom;
    });
    let improved = false;
    for (const stepSize of [0.22, 0.08, 0.03]) {
      const next = inputs.map((u, k) => clamp(u-stepSize*gradient[k], -1, 1));
      const candidateCost = costOf(next);
      iterations++;
      if (candidateCost < best-1e-7) {
        best = candidateCost;
        inputs = next;
        improved = true;
        break;
      }
    }
    if (!improved) break;
  }
  return {
    command: inputs[0],
    cost: best,
    iterations,
    converged: Number.isFinite(best) && best <= zeroInputCost+1e-6,
    worstCaseCost: best,
    openLoopCost: zeroInputCost,
    inputSequence: inputs.slice()
  };
}

/** Public entry: solve one control tick for an advanced controller mode. */
export function solveAdvancedController(mode, context) {
  const started = performance.now();
  const safeContext = sanitizeContext(context);
  const model = makeModel(safeContext.params);
  const ctx = { ...safeContext, model };
  let solution;
  switch (mode) {
    case 'riccati_lqr': solution = solveRiccatiLqr(ctx); break;
    case 'ilqr_true': solution = solveIlqr(ctx, { maxOuter: 4 }); break;
    case 'ilqr_minimax_true': solution = solveIlqr(ctx, { maxOuter: 4, adversarial: true }); break;
    case 'qp_mpc_proj': solution = solveProjectedMpc(ctx); break;
    default: throw new Error(`Unknown advanced controller mode: ${mode}`);
  }
  solution.solveMs = performance.now()-started;
  solution.command = clamp(Number.isFinite(solution.command) ? solution.command : 0, -1, 1);
  return solution;
}

function sanitizeContext(context) {
  const state0 = context.state0?.map(value => Number.isFinite(value) ? value : 0) || [0, 0];
  return {
    ...context,
    state0,
    referenceSpeed: Number.isFinite(context.referenceSpeed) ? context.referenceSpeed : 0,
    disturbance: Number.isFinite(context.disturbance) ? clamp(context.disturbance, 0, 1) : 0,
    previousCommand: Number.isFinite(context.previousCommand) ? context.previousCommand : 0
  };
}

// --- tiny dense linear algebra helpers (2×2 / 2-vectors only) ---------------

function matTranspose2x2(a) {
  return [[a[0][0], a[1][0]], [a[0][1], a[1][1]]];
}
function matMul2x2(a, b) {
  return [
    [a[0][0]*b[0][0]+a[0][1]*b[1][0], a[0][0]*b[0][1]+a[0][1]*b[1][1]],
    [a[1][0]*b[0][0]+a[1][1]*b[1][0], a[1][0]*b[0][1]+a[1][1]*b[1][1]]
  ];
}
function matAdd2x2(a, b) {
  return [
    [a[0][0]+b[0][0], a[0][1]+b[0][1]],
    [a[1][0]+b[1][0], a[1][1]+b[1][1]]
  ];
}
function matVec2(a, v) {
  return [a[0][0]*v[0]+a[0][1]*v[1], a[1][0]*v[0]+a[1][1]*v[1]];
}
function dot2(a, b) {
  return a[0]*b[0]+a[1]*b[1];
}
function ensureFiniteMatrix(matrix, label) {
  for (const row of matrix) {
    for (const value of row) {
      if (!Number.isFinite(value)) throw new Error(`Non-finite value in ${label}`);
    }
  }
}
