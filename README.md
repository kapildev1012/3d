# 6-Bar Tensegrity Rover Simulator

A browser-based 3D physics simulation of a six-strut, 24-cable tensegrity rover with a suspended payload core, terrain contact, support-face rolling control, obstacle traversal, and an A-vs-B adaptive-control course.

All ten levels share a photo-inspired Martian foundation with rough iron-rich sand, granular color variation, embedded dark stones, and broken rocky ridges. Level-specific craters, slopes, boulders, and mission obstacles are layered on top. Every level starts with the Level 10 target-speed capability of 1.30 m/s.

## Gravity quick-select

The top navigation bar carries a gravity selector synced bi-directionally with the sidebar slider:

- 🌍 Earth · 9.81 m/s²
- 🔴 Mars · 3.721 m/s²
- 🌙 Moon · 1.625 m/s²
- ⚙️ Custom (fine-tuned through the live sidebar slider)

Changing gravity retunes the running solver immediately on every terrain level: numerical integration, Hertz contact weight support, Coulomb friction and traction limits, roll-couple ceilings, and payload G normalization all read `cfg.gravity` per step, and the monitoring HUD/charts re-render from the live physics (the monitoring header shows the active `g` and rover weight at all times).

## Level 14 — 1 km² open-world expedition

Level 14 expands to a true 1000 m × 1000 m Martian field centred on the origin ((x, y) ∈ [−500 m, +500 m] on both axes) with multi-octave dune fields, crater rims and uplifts, jagged ridge lines, drop-off ledges, plateau inclines, marsh basins, and pale fine-sand/dust beds that locally reduce grip and add drag. The expedition crosses from the y = −450 m start line to the +450 m goal gate over a scored chain of solid crest checkpoints through extremely dense surface clutter: ~12,000 pebbles (5–15 cm), ~4,800 medium and sharp angular rocks (20–60 cm), ~1,200 clustered monolithic boulders (up to 4 m), a ~3,500-stone corridor gauntlet, plus an 18,000-fragment cosmetic micro-gravel layer — over 40,000 anchored elements in total. Every stone is seated onto the exact terrain heightfield with ground-normal orientation so nothing floats; only the sub-centimetre chips stay outside the physics array.

## Visual rendering

- **Zenith midday sun:** a visible blazing sun disc with a soft dust halo sits directly overhead along the light direction; near-vertical illumination casts intense, stark shadows almost directly beneath every rock and strut.
- **Sharp shadow frustum:** the directional light rides along with the rover inside a tight ortho frustum with crisp PCF filtering, keeping contact shadows sharp anywhere on the km² field.
- **Extreme surface texturing:** seamlessly tiling procedural albedo plus bump maps add millimetre regolith grain, while multi-octave vertex-colour variation paints broad albedo drift, wind-winnowed bright dust sheets, darker fine-grain basins, and slope-exposed rock.
- **Untouched natural ground:** no paths, roads or trails exist anywhere; distance fog converges into the dusty butterscotch sky for seamless atmospheric depth.
- **Dark basaltic scatter:** stones render as dark weathered rock against the bright dust via size-classed instanced meshes — tens of thousands of instances in a handful of draw calls.
- **Horizon structure:** Level 14 rings the map fringe with low, weathered ridge and cliff silhouettes that break the horizon exactly like the reference photography.

## Run

```bash
npm start            # node server.mjs → http://127.0.0.1:3100/  (PORT, HOST env respected; binds 0.0.0.0)
npm run build        # checks every module/import/asset, writes the static site to dist/
npm run preview      # serves dist/ with the same API
npm test             # full test suite (npm run verify = build + tests)
```

No dependencies to install (Node ≥ 18). Use `npm start` rather than a plain static server: it also provides the
benchmark API that stores records on disk.

## Workstation layout

- **Toolbar**: transport (Play/Pause · Step 0.1 s · Reset · speed up to 10×), mission clock with real-time factor,
  record-storage status, *Copy link*; second row: Level, Gait, Controller, Physics, Actuation, Gravity, Research preset.
- **Viewport (≈68 %)** with camera/overlay controls in its header and a docked Model A vs Model B telemetry strip
  underneath, so nothing covers the rover.
- **Inspector (≈32 %)**: Parameters (every slider has a typed numeric input), Gait, Experiment, Physics audit, Tools.
- **Dock**: Benchmark matrix (14 levels) · Live mission & structural telemetry · Analytical figures (Figs 1–13, category
  filters, dark on screen, *Paper export* = white IEEE PDF/SVG/600-dpi PNG) · Paper benchmark; *Show all* stacks them.
- **Shareable URLs**: `?level=6&controller=sgt_tipping&core=dae&act=pure&g=3.721&speed=2&cam=top` opens that exact setup.

## Benchmark records (never lost)

Every run is recorded continuously: a checkpoint every 1.0 s of simulated time, and a final record when the mission is
completed or failed (deadline, level time limit, 30 s stall) or the user resets, pauses (checkpoint), switches
level/controller/physics/preset, or closes the tab. Runs shorter than 0.5 s and 1 cm are ignored.

| Layer | Where | Survives |
| --- | --- | --- |
| localStorage `tensegrity_benchmarks_v2` + IndexedDB | browser | refresh, new tab, reboot |
| `data/benchmarks.json` via `/api/benchmarks` (atomic writes) | local server disk | clearing browser data, other browsers, reboot |
| `public/default_benchmarks.json` | seed (14 headless 20 s baseline runs, labelled SEED) | loaded only into an empty store |

All layers merge by run id (newest copy wins); *Clear saved records* writes a tombstone so no layer can resurrect cleared
runs. Export/Import JSON and CSV are in the matrix header; each level's history drawer exports that level alone. On
read-only hosting (Vercel) the API reports `persisted: false` and records live in the visitor's browser.

*Run all 14 levels* steps Levels 1→14 headless (10–60 s each), records every run and shows a downloadable report.
Regenerate the seed with `npm run seed`.

## Deploy

`npx vercel --prod` (uses `vercel.json`: `npm run build` → `dist/`, plus the `api/benchmarks.js` function).

## Real-time monitoring

The adaptive rover records and displays:

- Start A, Goal B, COM trajectory, distance-to-goal, configurable goal radius, and latched completion results.
- Horn/Kabsch-style rigid-aligned formation RMS so translation and rolling rotation do not count as deformation.
- Per-cable rest/current length, ΔL, strain, tension-only force, slack state, and overload state for all 24 cables.
- Active node/rod terrain contacts, object IDs, positions, normals, normal forces, friction forces, and contact durations.
- Rod residuals, intersections, terrain penetration, node separation, cable overload/strain, and collapse warnings.
- Six synchronized mission/structure charts plus the original controller and payload charts.
- A 120-second mission deadline with explicit win/loss outcomes rather than a fixed 90-second arrival rule.
- Persistent GPS-like 2 m route segments and bounded gradient updates after every attempt.
- Automatic new attempts begin only after both Model A and Model B reach their goals, then reuse learned speed, roll torque, traction, alignment, and route waypoints.

The central thresholds and logging settings are in `MONITORING_DEFAULTS` in `js/monitoringSystem.js` and can be overridden through `SimConfig({ monitoring: { ... } })`.

The dashboard exports `goal_tracking.csv`, `formation_metrics.csv`, `cable_metrics.csv`, `contact_events.csv`, `complete_simulation_log.csv`, and a complete JSON log. Raw export samples are retained at the physics timestep; graph smoothing does not alter exported values.

## Research mode (paper)

The simulator contains a second, physically audited mode used for the paper in `paper/`:

- **Lagrangian DAE core** (`js/lagrangianDae.js`): non-minimal Cartesian coordinates, consistent rigid-bar mass matrix, exact bar-length multipliers and the minimum-norm constraint + energy corrector (Algorithm 1).
- **Passive Hunt–Crossley contact** with its elastic potential in the energy, regularized Coulomb friction, no clamps or impulses.
- **Work ledger**: every force category and projection is booked; the residual ε = E − E₀ − ΣW and the aid share are shown in the *Physics audit* tab.
- **SGT gait** (`js/tippingController.js`): Sensitivity-Gradient Tipping, cable-only rolling from the pinned quasi-static KKT sensitivity of the centre of mass.

In the app, press **Research preset** (or choose Physics = *Lagrangian DAE* and Actuation = *Cable only*). The default game mode keeps the original engine with its locomotion aids, which the audit reports as non-physical work.

| Command | What it does |
| --- | --- |
| `npm run test:research` | DAE, corrector, ledger, SGT, schema and LaTeX tests |
| `npm run experiments` | full protocol: 6 methods, 10 scenarios, 7 ablations, 3 terrains (114 runs, resumable, `results/paper/runs/`) |
| `npm run paper:data` | `results/paper/tables.tex`, `numbers.tex`, figures (PDF/SVG/600-dpi PNG) |
| `node tools/train_guided_policy.mjs` | MPC-guided policy search training + held-out evaluation |
| `node tools/run_shield_study.mjs` | Tension-shield study, {SGT, policy} × {off, on} × 10 conditions |
| `npm run paper:pdf` | builds `paper/main.pdf` (needs LaTeX; IEEEtran is bundled) |

The *Comparative benchmark* panel runs the same protocol in the browser and has **Copy LaTeX benchmark table**. Every figure window has **Export for IEEE** (SVG, PDF, 600-dpi PNG at 3.5 in or 7.16 in).

## MPC + GNSS controller (satellite positioning)

*Controller → MPC + GNSS · Kalman-fused path tracking* (`js/gnssNavigation.js`) steers from a simulated GNSS receiver instead of the true centre of mass. The receiver gives 5 Hz fixes with 0.30 m noise, a slowly drifting 0.20 m bias, 0.2 s latency and 3 % dropouts. A constant-velocity Kalman filter fuses the delayed fixes and rejects outliers with a χ² gate. A steering MPC then plans the heading over a 3 s horizon (cross-track error and heading error, heading rate limited to ±0.6 rad/s) along the start → goal line.

The rolling effort comes from the existing QP-MPC in the default engine. In cable-only research mode, the MPC + GNSS heading drives the SGT tipping gait, because the support-face gait barely rolls without aids. *Gait → GNSS / Kalman* shows the estimate error, the filter σ and the cross-track error. Receiver and MPC settings can be overridden with `cfg.gps` and `cfg.gpsMpc`. The paper benchmark lists it as *MPC + GNSS*.

## MPC + GPS controller (MPC-guided policy search)

In this project **GPS = Guided Policy Search**. *Controller → MPC + GPS · guided policy search (learned policy)* (`js/guidedPolicySearch.js`) is a neural cable policy trained with Guided Policy Search (Levine & Koltun 2013). It uses the MPC-guided variant of Zhang et al. (ICRA 2016), which was applied to SUPERball by Zhang et al. (ICRA 2017).

- **Teacher (MPC):** a receding-horizon QP on the SGT gait's pinned quasi-static KKT sensitivity model, over 6 steps of 50 ms. It enforces box limits and spool-rate limits, and adds a proximity term λ‖ũ₁ − π_θ(o)‖² that pulls it toward the learner. With λ = 0 and no rate limit, its first step is exactly the SGT command.
- **Learner:** two small MLPs, one shared by the 24 outer cables and one by the 12 payload cables. They use local features in the frame of the rolling direction, so the policy does not depend on which face the rover rests on. At run time the policy replaces the KKT solve, while the settle → shift → tip → release cycle is kept.
- **Training** (`node tools/train_guided_policy.mjs`): iteration 0 imitates the teacher. Each later iteration rolls out the policy-regularized teacher, plus one episode that executes the policy itself. Every visited state is labelled with the unregularized teacher, and the policy is retrained on all data so far. The script writes `js/guidedPolicyWeights.js` and `results/guided_policy/{training,evaluation}.json`, which includes a held-out comparison against SGT and the teacher.

Shipped policy, trained for 8 iterations (114k labelled actuator samples) and evaluated on 4 held-out runs (seeds 101–104, Levels 1 and 3, straight and ±6 m offset goals, 20 s, cable-only DAE):

| | progress | tips | aborts | controller time |
| --- | --- | --- | --- | --- |
| SGT | 2.11 m | 11.0 | 0.25 | 0.53 ms |
| MPC teacher | 2.27 m | 10.8 | 0.25 | 0.63 ms |
| learned policy | 1.89 m | 11.5 | 0.50 | 0.73 ms |

The policy reaches 90 % of SGT's progress without solving the KKT model every tick. It still uses one sensitivity solve per tip cycle to choose a feasible edge (`cfg.guidedPolicy.edgeSelection = 'alignment'` drops that, at a cost in progress). It is not faster than SGT on this small structure.

## MPC + RL: parallel residual-MPPI, twin critic and LLM co-designer

Controller `residual_mppi` ("MPC + RL" in the controller menu, `residualMppi` in the benchmark) is the SGT
tipping gait plus a learned, bounded residual, always cable-only (assist share 0) and normally behind the
tension shield:

    u = clip( u_SGT + du ),   |du| <= 0.030 m per cable,   total stroke <= 0.120 m

Three tiers (all in `js/`, driver `tools/run_hybrid_training.mjs`):

1. **Zero-copy worker pool** (`parallelPool.js`, `parallelWorker.js`, `residualEnv.js`, `simSnapshot.js`).
   One headless 500 Hz DAE simulation per `worker_threads` worker (Node) or Web Worker (browser). After the
   bootstrap message every exchange is shared memory: `Atomics.wait/notify` command words, `Float64Array` views
   for 48-D observations, 24-D noise, 17-D telemetry and a checkpoint slot. A tick is 25 DAE sub-steps (20 Hz).
   `serializeState` / `deserializeState` snapshot the full dynamic state (DAE, energy ledger, SGT state
   machine, actuator-delay queue, RNGs, Level 1-14 crest-checkpoint tracker); restore is bit-exact, also into a
   fresh simulator through bytes (`tests/simSnapshot.test.mjs`). In the browser the page must be cross-origin
   isolated; `server.mjs` and `vercel.json` send COOP `same-origin` + COEP `credentialless`.
2. **Residual-MPPI + twin critic** (`residualMppi.js`, `mppiPlanner.js`). The actor maps the 48-D observation to
   6 *synergies* (one per strut: pull the four cables at one strut end, release the four at the other) plus a
   24-D fine residual, `du = 0.030 tanh(0.7 M' tanh z + 0.3 r)`; heads start at zero so an untrained actor is
   exactly the SGT command. Every 10 ticks (0.5 s) env 0 publishes a checkpoint, K structured noise sequences are
   rolled out in parallel from it (tension shield inside the loop), scored by the physics reward plus
   `gamma^H min(V1, V2)(s_H)` and combined with exponential weights. The weighted noise is executed, the actor is
   distilled from the executed actions and the twin critic is trained by TD(0) and the MPPI return.
3. **LLM MPC / reward co-designer** (`llmMpcOptimizer.js`, `llmSystemPrompt.js`). After each epoch the
   stress-matrix digest (v̄, RMSE, max|e|, t_rec, ∫uᵀu, CoT, t_solve, success, rod residual, relaxed cables,
   contact slip; nominal, push, gaussian, laplace, outlier, stiffness -20 %, friction -40 %, payload +30 %,
   delay 100 ms) and the Pareto history go to an LLM with a strict JSON schema (`MpcAndRewardUpdateSchema`).
   The reply updates the MPC cost weights, SGT gains, shield bounds and RL reward weights, is validated and
   clamped to a safe box (the model can only *shrink* the residual stroke), and a Pareto archive reverts to the best
   checkpoint when completion drops or an outer cable exceeds 180 N. Without a key, or if a call fails, a
   deterministic rule tuner implementing the same five diagnostic rules is used.

```
npm run hybrid                                    # full pipeline (all cores)
npm run hybrid:quick                              # 1 epoch smoke run
node tools/run_hybrid_training.mjs --epochs 8 --workers 9 --candidates 16 --matrix full --seeds 1,2,3
npm run experiments -- --train-hybrid --epochs 4  # same pipeline through the experiment runner
npm run experiments -- --only hybrid              # SGT vs SGT+shield vs residual-MPPI actor, paper protocol
```

LLM credentials come from the environment only (`ANTHROPIC_API_KEY`, `OPENAI_API_KEY` or `LLM_API_KEY`;
optional `LLM_PROVIDER`, `LLM_MODEL`, `LLM_API_URL`) and are never logged or stored. Only the numeric
digest above is sent. Outputs: `js/residualMppiWeights.js` (best checkpoint, loaded by the UI),
`results/hybrid/` (per-epoch JSON, summary, `comparative_benchmark.md`) and `public/hybrid/summary.json`
(shown in *Comparative benchmark > Hybrid training*, which can also run the parallel search live in Web Workers).

Mapping of the tunables to this code base: `q*`/`r*` are the stage-cost weights of the MPPI return,
`minCableTensionN` / `maxCableTensionN` are the tension-shield bounds (payload cables >= min, outer cables <=
max), `sgtGainKp` scales the normalized COM-sensitivity gradient before the stroke clip (1 = unit gain),
`sgtGainKd` is first-order command smoothing `u <- u_prev + (u - u_prev)/(1 + Kd)`.

## Tension shield (suspension-integrity safety filter)

**Finding.** While the shell rolls, gravity and the outer-cable actuation compress it (the centroid drops about 15 %). The node-to-payload distances shrink, and the 12 payload suspension cables lose their pretension. Without a filter, payload cables are slack about 35 % of the time, and in every sample at least one is slack. The "suspended" payload is loose in the cage.

**Filter** (`js/tensionShield.js`, *Gait → TENSION SHIELD*, URL `?shield=1`, `cfg.tensionShield = true`). Any tipping controller (SGT, the MPC teacher or the learned policy) proposes 36 rest-length offsets. The filter returns the closest command (weighted ‖u − u_nom‖, with outer cables weighted 10× so it corrects mainly with the payload cables) that satisfies discrete-time control-barrier conditions h(u) ≥ (1 − γ)·h(now) for:
- payload-cable tension ≥ 10 N;
- outer-cable tension ≤ 150 N;
- payload-to-rod clearance ≥ 0.08 m;
- the nominal command's predicted COM shift along the tipping edge (kept, so tipping effort is re-routed rather than cancelled).

The constraints are predicted with the same pinned quasi-static KKT sensitivities dq/du that SGT already factorizes, so no new model is added. The QP has 36 variables and is solved by FISTA on the dual, with the box exact in the primal.

**Study** (`node tools/run_shield_study.mjs` → `results/shield_study/`; 10 conditions = 3 seeds on Level 1, 2 on Level 3, push, impulse, payload +30 %, 100 ms delay, cable stiffness −20 %; 20 s cable-only runs):

| | SGT off → on | better in | MPC + GPS policy off → on | better in |
| --- | --- | --- | --- | --- |
| Progress | 1.49 → 2.20 m | 8/10 | 1.32 → 1.29 m | 4/10 |
| Payload-cable slack time | 35.1 → 2.7 % | 10/10 | 33.6 → 1.6 % | 10/10 |
| Mean payload-cable tension | 6.2 → 18.0 N | 10/10 | 6.9 → 17.5 N | 10/10 |
| Aborted tip cycles | 1.1 → 0.0 | 6/10 | 1.5 → 1.7 | 3/10 |
| Payload rattle RMS | 18.6 → 15.0 mm | 9/10 | 22.4 → 27.5 mm | 3/10 |
| Payload accel. 99th pct | 2.5 → 3.2 g | 4/10 | 2.0 → 1.7 g | 5/10 |
| Controller time | 0.64 → 3.6 ms | | 0.79 → 3.6 ms | |

- **What holds:** the suspension stays taut in every condition, for both controllers. With SGT, keeping the payload cables taut makes tipping more effective: it goes 48 % further and never aborts.
- **What does not:** the clearance and outer-load barriers hold only on the quasi-static prediction. In 3 of 20 filtered runs the payload came within 0.08 m of a rod, and peak outer tension exceeded 150 N during tumbles and impacts. Peak payload acceleration did not improve for SGT. These are dynamic events, and covering them needs a second-order (inertial) barrier.

## Materials and structural analysis (TsgFEM)

The workstation simulates the **steel build** by default (*Parameters → Struts & cables*, URL `?mat=steel|lightweight`): Ø25 × 1.5 mm S355 steel tube struts (0.739 kg each) and Ø2 mm 1960 MPa UHS steel wire rope in series with the compliant actuator spring (`js/materials.js`). Rope mass is lumped onto the end nodes and the payload. `SimConfig` and the paper experiments keep the original lightweight parameters. Each benchmark record stores its `material`.

*Tools → Structural analysis · TsgFEM* runs a JavaScript port (`js/tsgfem.js`, `js/structuralAnalysis.js`) of the **Tensegrity Finite Element Method** toolbox by Shuo Ma, Muhao Chen and Robert E. Skelton (JOSS 2022, [doi:10.21105/joss.03390](https://doi.org/10.21105/joss.03390), MPL-2.0). The original MATLAB functions and licence are in `third_party/TsgFEM/`. The analysis covers:

- the equilibrium matrix and its SVD (self-stress and mechanism modes);
- prestress design with grouped members;
- minimal-mass sizing (buckling and yield);
- the safety margins of the installed parts;
- tangent stiffness, consistent mass and modal frequencies.

The simulated icosahedral shape (d/h = 1/φ) has no self-stress state. The statics therefore use TsgFEM's form-found expanded octahedron (d/h = 0.5) with the same bars.

`npm run tsgfem:validate` runs the original toolbox in GNU Octave (or MATLAB) on the exported rover model and compares it with the port. `tests/tsgfem.test.mjs` runs that comparison automatically when Octave is installed.

## Verify

```bash
npm run build
```

## Solid 3D obstacles on every level (1–14)

Every level now has deterministic, physically collidable 3D obstacles, identical for Model A (Baseline) and Model B (Adaptive). The two lanes collide against and render the same list.

| Module | What it does |
|---|---|
| `js/obstacleField.js` | `buildObstacleFieldForLevel(levelId, roughnessRms, seed, course)` is a seeded generator. It produces `box` OBBs (crates, step ledges, low walls, narrow gate pillars), `rock` jagged basalt boulders (sphere-swept lobe clusters), `cylinder` fallen pipes and rubble rollers, and `ramp` inclined wedges. It keeps a 2.5 m clear spawn pad around the start. Every obstacle is anchored to the heightfield, and the obstacles are re-seated whenever the terrain is regenerated (the roughness slider calls `generateSurface()`). Each kind has an exact signed distance function and an analytic normal. |
| `js/obstacleContact.js` | Contact for 12 rod end-nodes, 3 quadrature points per strut (18) and the payload core. It runs after the constraint corrector in both Legacy SHAKE and the 500 Hz DAE. It projects along the SDF normal by translating the whole rod, so rod lengths are preserved exactly. Restitution is e = 0.02 (locked), with Coulomb friction impulses. Contact can never add energy, and its work is booked as `contact` in the Physics Audit ledger, so DAE \|ε\| stays at about 1e-9 J. |
| `js/obstacleRenderer.js` | Chamfered crates and concrete blocks with edge highlights, displaced flat-shaded basalt dodecahedrons, pipes and ramps. Shadows are on. Cyan checkpoint rings and `CP` tags sit on every obstacle that must be crested. |
| `js/obstacleTracker.js` | Over/around classification per model. It feeds `OBSTACLES: X over / Y around` and `BYPASS VIOL.` in the HUD cards and the dock tables. A bypass violation means Model B went around an obstacle it should have crested; gates, walls and tall offset rocks may legally be passed around. |

Set `cfg.obstacleField = false` to turn the field off (e.g. for legacy benchmark comparisons).

## LLM co-designer endpoints

`tools/run_hybrid_training.mjs` reads a local `.env` (git-ignored; see `.env.example`). With `VITE_LLM_API_URL`, `VITE_LLM_API_KEY` and `VITE_LLM_MODEL` it calls any OpenAI-compatible chat/completions endpoint (Gemini, Groq, or a local Ollama) with `response_format: json_object`. The reply is parsed, schema-validated and clamped. Without these, it uses `ANTHROPIC_API_KEY` or `OPENAI_API_KEY`, and otherwise falls back to the offline rule-based tuner. Keys are never printed or stored in results.

Controllers: **MPC + RL** (`residual_mppi`, trained with `npm run hybrid:rl`, MPC/reward fixed) and **MPC + RL + LLM** (`residual_mppi_llm`, trained with `npm run hybrid`, co-designed by the LLM).
