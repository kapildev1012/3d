# 6-Bar Tensegrity Rover Simulator

A browser-based 3D physics simulation of a six-strut, 24-cable tensegrity rover with a suspended payload core, terrain contact, support-face rolling control, obstacle traversal, and an A-vs-B adaptive-control course.

All ten levels share a photo-inspired Martian foundation with rough iron-rich sand, granular color variation, embedded dark stones, and broken rocky ridges. Level-specific craters, slopes, boulders, and mission obstacles are layered on top. Every level starts with the Level 10 target-speed capability of 1.30 m/s.

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
- Automatic new attempts that reuse learned speed, roll torque, traction, alignment, and route waypoints.

The central thresholds and logging settings are in `MONITORING_DEFAULTS` in `js/monitoringSystem.js` and can be overridden through `SimConfig({ monitoring: { ... } })`.

The dashboard exports `goal_tracking.csv`, `formation_metrics.csv`, `cable_metrics.csv`, `contact_events.csv`, `complete_simulation_log.csv`, and a complete JSON log. Raw export samples are retained at the physics timestep; graph smoothing does not alter exported values.

## Verify

```bash
npm run build
```
