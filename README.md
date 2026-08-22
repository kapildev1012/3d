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
npm run dev
```

Open `http://127.0.0.1:3000/`.

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

## Verify

```bash
npm run build
```
