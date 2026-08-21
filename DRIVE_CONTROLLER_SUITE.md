# Additive Drive Controller and Mathematics Suite

This integration adds the controller and mathematics set from the shared Google Drive folder without editing the existing simulator source files. Open:

```text
http://127.0.0.1:3000/controller-suite.html
```

The wrapper loads the current `index.html`, installs the controller adapter, then starts the existing `app.js`. The original `index.html`, `js/app.js`, `js/simEngine.js`, and `js/visualizer.js` remain unchanged by this integration.

## Controller mapping

| Drive MATLAB source | Additive JavaScript implementation | Numerical method retained |
|---|---|---|
| `LQR_RollingDirection.m` | `LQRRollingDirectionController` | Finite-horizon discrete Riccati recursion, state/input quadratic penalties, actuator remapping and clipping |
| `LQR_RollingDirection_centralPayload.m` | `CentralPayloadLQRController` | LQR with additional payload height/velocity weighting |
| `iLQR_RollingDirection.m` | `IterativeLQRController` | Nonlinear rollout, dynamics linearization, backward gains, regularization and backtracking line search |
| `iLQRminimax_RollingDirection_2.m` | `MinimaxIterativeLQRController` | Coupled control/disturbance saddle-point Hessian, bounded worst-case disturbance and robust rollout |
| `iLQRminimax_RollingDirection_inputpenalty.m` | `InputPenaltyMinimaxController` | Minimax pass plus explicit change-in-input penalty |
| `QP_MPC_RollingDirection.m` | `QPMPCController` | Trapezoidal discretization, receding-horizon quadratic objective, hard length-rate constraints and projected solve |
| `QP_MPC_RollingDirection_centralPayload.m` | `CentralPayloadQPMPCController` | Constrained MPC with payload-state weighting |
| `NN_RollingDirection.m` | `NeuralRollingDirectionController` | Desired-direction coordinate rotation, node/cable features, dense-network inference contract and paired-actuator projection |

The shared folder does not contain the `net2` `.mat` weight file loaded by `NN_RollingDirection.m`. The neural controller therefore exposes an injectable `network` parameter and uses a deterministic geometry policy when no weights are supplied. Its diagnostics set `neuralFallback: true`; it is not presented as the missing trained network.

## Model, force, and paper mathematics

The following are implemented in `js/reference/tensegrityMath.js`:

- Exact 12-node, six-rod, 24-cable node order/connectivity from `six_bar_model.m`.
- Connectivity matrices, member coordinate differences, the weighted force-density Laplacian, geometry/rigidity matrix, and self-equilibrium residuals.
- Position- and velocity-level rigid-rod constraints, constraint Jacobians, and Lagrange-multiplier acceleration projection from `RodConstraints.m` and `Dynamics_Generator.m`.
- Tension-only smooth cable forces and their rest-length state, gravity, general XYZ damping, smooth inclined-floor contact, and contact-gated horizontal viscous friction.
- Paper power-law member force and potential (Eq. 10–14), collision avoidance (Eq. 17–18), Hermite string relaxation (Eq. 19–20), input saturation, damped agent dynamics, and the structural-stability sign/FDM checks.
- All Table I paper values in `PAPER_PARAMETERS`.

The paper prints a negative sign in Eq. 18 while defining its direction vector as pointing away from the obstacle. The Drive `main.m` notes that this makes the term attractive and uses the repulsive sign. The JavaScript function follows the executed `main.m` behavior by default and exposes `paperSign: true` for literal printed-equation reproduction.

## Files added

- `controller-suite.html` — additive simulator entry point.
- `js/installDriveControllerSuite.js` — runtime installer; adds the controller options and wraps the existing control hook.
- `js/reference/linearAlgebra.js` — dependency-free matrix, Riccati, finite-difference, trapezoidal, and eigenvalue helpers.
- `js/reference/tensegrityMath.js` — topology, force, constraint, dynamics, and paper equations.
- `js/reference/controllerAlgorithms.js` — all eight controller implementations.
- `js/reference/simulationControllerAdapter.js` — six-bar simulator state/input adapter.
- `tests/driveReferenceSuite.test.mjs` — topology, force-law, controller, and integration coverage.

## Direct API use

```js
import { attachDriveControllerSuite } from './js/reference/simulationControllerAdapter.js';

const suite = attachDriveControllerSuite(simulation);
const command = suite.solve('ilqr_minimax_penalty', centroid, velocity, obstacle);
```

The returned object matches the simulator control hook: `cableTargets`, `rodTargets`, `coreTargets`, and `diagnostics`.

## Source provenance

Reference folder: [Control of tensegrity](https://drive.google.com/drive/folders/1Lk2KT0lrpaJqzRadKmd85gHiOAdAJ6nO).

The implementation was adapted from the eight controller files in the `Controllers` subfolder, the four force files in `Forces`, `Dynamics_Generator.m`, `RodConstraints.m`, `six_bar_model.m`, `tensegrityODE.m`, the root `main.m`, and the six-page adaptive tensegrity control paper supplied in that folder.
