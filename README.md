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
| `npm run paper:pdf` | builds `paper/main.pdf` (needs LaTeX; IEEEtran is bundled) |

The *Comparative benchmark* panel runs the same protocol in the browser and has **Copy LaTeX benchmark table**. Every figure window has **Export for IEEE** (SVG, PDF, 600-dpi PNG at 3.5 in or 7.16 in).

## MPC + GPS controller

*Controller → MPC + GPS · Kalman-fused path tracking* (`js/gpsNavigation.js`) steers from a simulated GNSS receiver instead of the true centre of mass. The receiver gives 5 Hz fixes with 0.30 m noise, a slowly drifting 0.20 m bias, 0.2 s latency and 3 % dropouts. A constant-velocity Kalman filter fuses the delayed fixes and rejects outliers with a χ² gate. A steering MPC then plans the heading over a 3 s horizon (cross-track error and heading error, heading rate limited to ±0.6 rad/s) along the start → goal line.

The rolling effort comes from the existing QP-MPC in the default engine. In cable-only research mode, the MPC + GPS heading drives the SGT tipping gait, because the support-face gait barely rolls without aids. *Gait → GPS / Kalman* shows the estimate error, the filter σ and the cross-track error. Receiver and MPC settings can be overridden with `cfg.gps` and `cfg.gpsMpc`. The paper benchmark lists it as *MPC + GPS*.

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
