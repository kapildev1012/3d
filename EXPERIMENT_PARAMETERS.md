# Controlled 50 m A-vs-B Tensegrity Experiment

## Source classification

| Classification | Source | Used here |
|---|---|---|
| Paper-defined | `Adaptive_Tensegrity-Based_Control_for_Multi-Agent_Obstacle_Avoidance.pdf` in the supplied Drive | The string-relaxation force law and the distinction between attractive navigation and obstacle response. The paper model is a 2D tiled multi-agent formation; it is **not** presented as evidence for this 3D rover's rolling performance. |
| Drive-code-defined | `six_bar_model.m`, `RodConstraints.m`, force functions, and central-payload LQR/MPC controllers | Exact 6-bar, 12-node, 24-outer-cable topology; tension-only spring/damper mechanics; rigid-rod constraints; cable speed and length limits; explicit central payload concept. |
| GitHub/NTRT-defined | `kapildev1012/locomotion`, especially `bar_length_icosahedron` and `analysis/framework` | Fixed-rate physical control, support/contact recovery ideas, seeded obstacles, safety envelopes, progress monitoring, and telemetry structure. |
| Experiment-specific | This repository's `js/abExperiment.js` | Shared 0–70 m course, 10–60 m measured section, 10 irregular obstacle formations, strict OVER-vs-AROUND classifier, retry cap, and A-vs-B reporting. These are experiment design choices, not claims copied from the paper. |

## Shared physics and robot parameters

| Parameter | Value | Classification |
|---|---:|---|
| Outer diameter | 1.00 m | Project requirement / Drive geometry scaled uniformly |
| Compression members | 6 rigid bars | Drive-code-defined |
| Outer nodes / outer cables | 12 / 24 | Drive-code-defined |
| Core suspension cables | 12 | Drive central-payload architecture; implemented explicitly here |
| Fixed physics step | 0.002 s (500 Hz) | Experiment-specific numerical setting |
| Controller period | 0.02 s (50 Hz) in Level 10 | Level 10 rolling-performance profile; fixed for A and B |
| Initial rolling-speed capability | 1.30 m/s in Levels 1–10 | Shared starting capability; the Level 10 route learner additionally applies bounded per-run speed scaling |
| Shared Mars terrain | Rough sand plus deterministic embedded stones and broken side ridges | Photo-inspired foundation used by every level; each level retains its distinct challenge layer |
| Outer cable stiffness / damping | 1200 N/m / 32 N·s/m | Project-scaled Drive force model |
| Outer cable pretension | 40 N | Project setting |
| Core cable stiffness / damping | 1600 N/m / 18 N·s/m | Project central-payload setting |
| Core cable pretension | 50 N | Project central-payload setting |
| Cable rest-length rate limit | 0.32 m/s in Level 10 | Level 10 performance setting within the bounded cable envelope |
| Outer cable length envelope | 0.50–1.50 × nominal | Drive controller concept, scaled project envelope |
| Restitution | 0.02 | Experiment-specific non-bouncy contact setting |
| Ground friction | 4.00 in Level 10 | High-traction obstacle-climb surface; shared by A and B |
| Gravity | 9.81 m/s² default · top-bar presets for Earth 9.81 / Mars 3.721 / Moon 1.625 m/s² plus a custom slider | UI quick-select is bi-directionally synced with the sidebar slider; the whole solver (integration, Hertz contact weight support, Coulomb/traction envelopes, roll couples, payload G normalization) reads `cfg.gravity` live on every step |

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
| Heights | 0.50 m = 0.50 × the 1.00 m rover diameter (all obstacles) |
| Minimum lateral radius | 1.50 m, retaining the original centres and obstacle types |
| Minimum longitudinal radius | 1.60 m (1.60 × rover diameter) for a physically traversable crest grade |
| Model B corridor half-width | 0.42 m around each obstacle centreline |
| Look-ahead distance | 2.40 m |
| Stall window / retry cap | 6 s / 2 retries per obstacle |
| Mission deadline | Reach Goal B within 120 s for a win; exceeding 120 s is a loss |
| Run-to-run learning | Persistent 2 m GPS segments with bounded gradient updates | Learns speed, roll torque, traction, alignment, and route waypoints from wins, losses, slip, grade, and wasted lateral travel |

The rover is not assigned a fixed arrival time. Each attempt ends in a win at Goal B or a loss at the 120 s deadline; the next attempt reuses the learned route and bounded controller updates.

## A-vs-B control rule

- **Model A — baseline:** uses the same terrain, robot, contact law, timestep, and base support-face gait. It is allowed to choose a natural over-or-around path.
- **Model B — adaptive:** targets the centre footprint of the active obstacle. Cable adaptation is proportional to obstacle height and limited to 3, 4, or 6 selected outer cables. No all-cable maximum relaxation is allowed.
- Model B phases are `APPROACH → ALIGN → DEFORM_CLIMB → COM_OVER → DESCEND_RESTORE`. A six-second progress stall triggers a physical back-off/retry command; no state teleportation is used.
- A pass is classified `OVER` only after the COM crosses the obstacle's longitudinal extent while the rover entered its footprint and remained in the centre band. Crossing the obstacle station outside that evidence is `AROUND`; every Model B `AROUND` is a bypass violation.

## Reported metrics

Both models report measured path length, measured completion time, mean speed, speed variance, lateral travel, peak payload proper G, peak cable tension, deformation RMS, actuator work estimate, obstacle attempts/retries, and OVER/AROUND counts. Model B additionally reports bypass violations.
