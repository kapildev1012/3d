# tensegrity_dae — Cartesian Lagrangian tensegrity DAE (MATLAB / Octave)

Implementation of the non-minimal-coordinate tensegrity dynamics in
`tensegrityDynamicspreprint (1).pdf`. It covers topology matrices, unilateral
cables with damping, the index-1 DAE, the Algorithm-1 geometric/energy
corrector, analytical linearisation, flexible bars, minimal realisation and
LQR. It runs on your existing 6-bar model in `../drive_matlab/six_bar_model.m`.
Nothing in `drive_matlab/`, `3D/` or the JS app was modified.

```matlab
cd tensegrity_dae
main_tensegrity_dae                          % 6-bar: linearise + LQR vs passive
main_tensegrity_dae('scenario','all')        % + drop test + paper T-bar
run('tests/run_tensegrity_dae_tests.m')      % 10 self-checks (~2 min)
```

## Layout (spec module → file)

| Spec | File | What it does |
|---|---|---|
| A  `tensegGenMat` | `structures/tensegGenMat.m` | `q = vec(N)`, `X_k, X̄_k, Y_k, P_k` (Eq. 1–5) plus stacked forms, `X_k'X_k`, `Y_k'Y_k`, constant `M` (Eq. 22), `M⁻¹`/Cholesky, `G` (Eq. 23), `A q = b` from pinned dofs / plane constraints |
| A  topologies | `structures/tensegStructures.m` | `'sixbar'` (reads your `six_bar_model.m`), `'tbar2d'` (paper Fig. 1a), `'prism3'` |
| A  prestress | `structures/tensegSelfStress.m`, `tensegFormFind.m` | force densities / rest lengths for equilibrium; dynamic relaxation onto the exact equilibrium |
| B  | `dynamics/tensegStrings.m`, `tensegControl.m` | unilateral σ (Eq. 25), slack logic, `Ŷ(q)`, `Ŷσ` (Eq. 30), axial dampers (Eq. 27–29); rest-length **or** force-density input |
| C  | `dynamics/tensegConstraints.m` | `R`, `R_q`, `ξ₂`, `Q` (Eq. 36–37, 44) |
| C  | `dynamics/tensegDaeDynamics.m` | `λ`, `q̈` (Eq. 34–35), `Ẇ_f = fᵀq̇` (Eq. 49), pinv fallback for redundant constraints, flexible toggle (Eq. 67–68) |
| D  | `dynamics/tensegEnergy.m`, `daeCorrector.m` | `E = T+V_g+V_s(+V_b)` with gradients; Algorithm 1 with `A_c`, `b_c` (Eq. 56–57), γ = 1e-10, max 10 iterations |
| D  | `dynamics/tensegIntegrate.m`, `tensegSimulate.m` | step-by-step Dormand–Prince 5(4) (adaptive) or RK4, corrector after **every** accepted step, zero-order-hold control loop, logging |
| E  | `controllers/tensegLinearizer.m` | exact `∂ξ/∂q` (column by column, p. 15), `∂ξ/∂q̇`, `∂ξ/∂σ`, `∂ξ/∂l_s`, `∂ξ/∂f`, `P_c`; assembles `A`, `B_σ`, `B_ls`, `B_f` (Eq. 59–63) |
| E  | `controllers/tensegLinearizeNumeric.m` | central-difference version (flexible bars, validation) |
| E  | `controllers/tensegReduce.m` | constraint-tangent coordinates `x = T z`, then the controllable subspace (orthogonal staircase) and a Hautus pruning step → minimal realisation |
| E  | `controllers/tensegLqrController.m` | ZOH-discretised LQR (doubling-algorithm DARE, no toolbox), `σ = clamp(σ₀ − Kz, ≥ 0)` |
| F  | `dynamics/tensegFlexTerms.m` | flexible bars: `M_q̈(q)`, velocity-coupling terms, `V_b`, Poisson radius |
| main | `main_tensegrity_dae.m` | scenarios and plots: nodal trajectories, σ(t), ‖b_k‖−l_bk per bar, \|E−E₀−W_f\| |
| — | `dynamics/tensegGroundForce.m` | optional penalty floor contact (external `f`) |

## Using it with your own structure

```matlab
topo = struct('N0',N0,'Cb',Cb,'Cs',Cs,'barMass',mb,'barRadius',rb, ...
              'stringStiffness',K,'stringRestLength',ls,'stringDamping',c, ...
              'fixedDofs',[node dim; ...],'pmNodes',idx,'pmMass',mp);
S   = tensegGenMat(topo, struct('gravity',[0;0;-9.806], 'flexibleBars',false));
u   = tensegControl(S,'restlength',ls);           % or tensegControl(S,'sigma',sigma)
out = tensegSimulate(S, S.q0, zeros(S.nq,1), u, fext, struct('T',5,'dtControl',0.01));
lin = tensegLinearizer(S, q0, dq0, u0, f0);        % A, Bsigma, Bls, Bf
red = tensegReduce(lin);                           % red.Ac, red.Bc, red.Tc, red.Tcpinv
```

`controller` in `tensegSimulate` can also be a handle `u = ctrl(t,q,dq)`. Use it
to plug in iLQR or QP-MPC: build them on `red.Ac`/`red.Bc` (or on the full
`lin.A`/`lin.Bsigma`), map the state with `z = red.Tcpinv*[q-q0; dq-dq0]`, and
return `tensegControl(S,'sigma',max(sigma0+dsigma,0))`. Your existing
`drive_matlab/Controllers/*` classes are built around the older
`(p, pDOT, RL, L)` / `Dynamics_Generator` interface. They are left unchanged and
not wired to this model.

## Verified behaviour (Octave 8.4)

* Analytical linearisation matches finite differences to ≤ 4e-8 (relative). Tested on all
  three structures, both input types, with damping, non-zero velocity and pinned nodes.
* The flexible-bar `M_q̈` and bias terms match a numerical Euler–Lagrange derivation to 7e-11.
* With Algorithm 1 at loose tolerance (1e-6), bar-length error stays ~1e-13 and energy
  error ~1e-10. Without it: 6-bar drop test 4e-5 J / 3e-7 m; T-bar 1e-8 J.
* 6-bar LQR: full state 72 → constraint tangent 60 → controllable 46. Over 3 s the shape
  error goes 7.9 mm → 0.10 mm; the passive ball stalls at 2.7 mm.

## Design notes and deviations from the spec

* **σ-input energy.** In force-density mode σ is held over each control interval, so
  the conservative cable potential is `½Σσ_k s_kᵀs_k`. The spec's `½Σσ²/K sᵀs`
  is the rest-length-mode value and is used there. Both have gradient `(Ŷσ)ᵀ`,
  as Eq. 56 requires.
* **Energy under control.** When the controller changes σ or l_s, the string energy
  jumps at fixed state. `tensegSimulate` books that jump as actuator work (`out.Wact`)
  and shifts `E₀`. Algorithm 1 is then exact inside each hold interval, and
  `E − E_init − W_f − W_act = 0` holds over the whole run (logged as `out.Eres`).
* **Velocity residual.** Algorithm 1 in the paper loops on ‖R‖ and the energy residual
  only. `daeCorrector` also loops while ‖R_q q̇‖ > γ (`checkVelocity`, default on).
* **Flexible bars.** The pages of the attached spec covering this were garbled, so this
  follows the paper's Section 3 kinetic energy `T = ½q̇ᵀMq̇ − ½Σ(I_b/l²)l̇²` and
  derives the Euler–Lagrange terms exactly. `ṙ = −νr l̇/l` integrates to
  `r = r₀(l/l₀)^(−ν)`, so the radius stays a function of q. In that model a
  bar's axial stretch carries no inertia, so `M_q̈` is singular unless the bar
  ends carry point masses. Use `tensegStructures('sixbar', struct('pointMassFraction',0.5))`
  or `topo.pmNodes/pmMass`; the code stops with an error message otherwise.
* **Damping in the linear model.** String damping `f_d` stays in the plant (`∂f_d/∂q`,
  `∂f_d/∂q̇` are in `A`). `B_f` is for external forces only.
* **Minimal realisation.** A free structure has 12 rigid-body states that internal
  forces cannot reach. Infinitesimal mechanisms do not stretch any cable to first order
  (`mᵀŶ = 0`), so σ cannot move them either. The staircase plus Hautus test removes both
  groups.
* **Your six_bar_model.m geometry** is rounded to 4 decimals and is not an exact
  equal-tension equilibrium. `tensegFormFind` settles it onto the regular tensegrity
  icosahedron: cables 0.408 → 0.404 m, bar/cable ratio 1.634, about 3.9 cm RMS shape
  change. The demos use that equilibrium.
* Not implemented: the Transformer discrepancy-learning pipeline from the
  handwritten notes (Module F, last bullet).
