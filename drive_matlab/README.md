# Drive Import · "Control of tensegrity" MATLAB Suite

Imported verbatim (no code removed or rewritten) from the shared Google Drive
folder "Control of tensegrity" on 2026-08-22.

## Contents

| File | Source role |
|---|---|
| `main.m` | 12-agent tiled formation demo, Table I parameters of the ECC paper |
| `six_bar_model.m` | 6-bar tensegrity topology: connectivity matrices C (24 cables), R (6 rods), node masses, cable stiffness/pretension |
| `tensegrityODE.m` | Stacked state-space wrapper (p, pDOT, RL, L) around pDDOT |
| `Dynamics_Generator.m` | Constraint-based dynamics: Lagrange-multiplier rod forces, smooth-max cable tension law `smax(x,b)=(sqrt(x^2+b^2)+x)/2`, Jacobians dpDDOT/d{p,pDOT,RL,L} |
| `RodConstraints.m` | Position/velocity/acceleration-level rigid-rod constraints G, GDOT, dGdp, dGDOTdp |
| `Controllers/LQR_RollingDirection.m` | Finite-horizon LQR, augmented system [A z;0 1], backward Riccati P-recursion |
| `Controllers/iLQR_RollingDirection.m` | Iterative LQR with line search |
| `Controllers/iLQRminimax_RollingDirection_2.m` | Robust iLQR against bounded adversarial disturbance w |
| `Controllers/iLQRminimax_RollingDirection_inputpenalty.m` | Minimax variant with input-change penalty |
| `Controllers/QP_MPC_RollingDirection.m` | Receding-horizon QP (YALMIP/Gurobi), trapezoidal discretization, box actuator constraints |
| `Controllers/*_centralPayload.m` | Payload-stabilized variants |
| `Controllers/NN_RollingDirection.m` | Learned neural policy over rotated node features (weights .mat not in share) |
| `Forces/*.m` | Floor contact w/ incline + viscous friction, gravity, general XYZ damping |

The PDF is a second copy of the ECC 2026 obstacle-avoidance paper (the root
folder already holds the original).

## Browser ports added to this project

| MATLAB source | JS implementation |
|---|---|
| `LQR_RollingDirection.m` | `js/advancedControllers.js` → `solveRiccatiLqr` (mode `riccati_lqr`) |
| `iLQR_RollingDirection.m` | `js/advancedControllers.js` → `solveIlqr` (mode `ilqr_true`) |
| `iLQRminimax_*_inputpenalty.m` | `js/advancedControllers.js` → adversarial ascent in `solveIlqr` (mode `ilqr_minimax_true`) |
| `QP_MPC_RollingDirection.m` | `js/advancedControllers.js` → `solveProjectedMpc` (mode `qp_mpc_proj`) |

The solvers run at the live controller rate against a reduced-order rolling
model (speed-along-goal + payload height) and modulate the support-face gait's
actuation effort. Select them in the UI under "Controller: Drive-ported
solvers". A second, heavier full-state reference stack lives under
`js/reference/` (see `tests/driveReferenceSuite.test.mjs`).

Tests: `tests/advancedControllers.test.mjs`.
