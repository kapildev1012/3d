# Controlled 50 m A-vs-B Tensegrity Experiment

## Source classification

| Classification | Source | Used here |
|---|---|---|
| Paper-defined | `Adaptive_Tensegrity-Based_Control_for_Multi-Agent_Obstacle_Avoidance.pdf` in the supplied Drive | The string-relaxation force law and the distinction between attractive navigation and obstacle response. The paper model is a 2D tiled multi-agent formation; it is **not** presented as evidence for this 3D rover's rolling performance. |
| Drive-code-defined | `six_bar_model.m`, `RodConstraints.m`, force functions, and central-payload LQR/MPC controllers | Exact 6-bar, 12-node, 24-outer-cable topology; tension-only spring/damper mechanics; rigid-rod constraints; cable speed and length limits; explicit central payload concept. |
| GitHub/NTRT-defined | `kapildev1012/locomotion`, especially `bar_length_icosahedron` and `analysis/framework` | Fixed-rate physical control, support/contact recovery ideas, seeded obstacles, safety envelopes, progress monitoring, and telemetry structure. |
| Experiment-specific | This repository's `js/abExperiment.js` | Shared 0–70 m course, 10–60 m measured section, 12 obstacle placements, strict OVER-vs-AROUND classifier, retry cap, and A-vs-B reporting. These are experiment design choices, not claims copied from the paper. |

## Shared physics and robot parameters

| Parameter | Value | Classification |
|---|---:|---|
| Outer diameter | 1.00 m | Project requirement / Drive geometry scaled uniformly |
| Compression members | 6 rigid bars | Drive-code-defined |
| Outer nodes / outer cables | 12 / 24 | Drive-code-defined |
| Core suspension cables | 12 | Drive central-payload architecture; implemented explicitly here |
| Fixed physics step | 0.002 s (500 Hz) | Experiment-specific numerical setting |
| Controller period | 0.05 s (20 Hz) | Project implementation; fixed for A and B |
| Target reference speed | 0.20 m/s | User-specified; reference, not a forced velocity |
| Outer cable stiffness / damping | 1200 N/m / 32 N·s/m | Project-scaled Drive force model |
| Outer cable pretension | 40 N | Project setting |
| Core cable stiffness / damping | 1600 N/m / 18 N·s/m | Project central-payload setting |
| Core cable pretension | 50 N | Project central-payload setting |
| Cable rest-length rate limit | 0.10 m/s | Drive controller limit |
| Outer cable length envelope | 0.50–1.50 × nominal | Drive controller concept, scaled project envelope |
| Restitution | 0.02 | Experiment-specific non-bouncy contact setting |
| Ground friction | 0.85 | Project setting; shared by A and B |
| Gravity | 9.81 m/s² | Earth test condition |

The payload G trace is the magnitude of the summed **non-gravitational suspension force divided by payload weight**. It is not the previous synthetic, clipped shell-centroid proxy.

## Controlled course

| Item | Value |
|---|---:|
| Terrain extent | y=0–70 m |
| Unmeasured approach | y=0–10 m |
| Measured start / finish | y=10 m / y=60 m |
| Effective measured distance | 50 m |
| Obstacles | 12 total, 2–3 in each 10 m band |
| Shapes | Rounded rocks, low blocks, ramps, mounds |
| Heights | 0.08–0.26 m |
| Model B corridor half-width | 0.42 m around each obstacle centreline |
| Look-ahead distance | 1.65 m |
| Stall window / retry cap | 5 s / 2 retries per obstacle |

## A-vs-B control rule

- **Model A — baseline:** uses the same terrain, robot, contact law, timestep, and base support-face gait. It is allowed to choose a natural over-or-around path.
- **Model B — adaptive:** targets the centre footprint of the active obstacle. Cable adaptation is proportional to obstacle height and limited to 3, 4, or 6 selected outer cables. No all-cable maximum relaxation is allowed.
- Model B phases are `APPROACH → ALIGN → DEFORM_CLIMB → COM_OVER → DESCEND_RESTORE`. A five-second progress stall triggers a physical back-off/retry command; no state teleportation is used.
- A pass is classified `OVER` only after the COM crosses the obstacle's longitudinal extent while the rover entered its footprint and remained in the centre band. Crossing the obstacle station outside that evidence is `AROUND`; every Model B `AROUND` is a bypass violation.

## Reported metrics

Both models report measured path length, measured completion time, mean speed, speed variance, lateral travel, peak payload proper G, peak cable tension, deformation RMS, actuator work estimate, obstacle attempts/retries, and OVER/AROUND counts. Model B additionally reports bypass violations.

