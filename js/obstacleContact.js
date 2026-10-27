/**
 * OBSTACLECONTACT.JS - node + strut-segment SDF contact against the solid
 * obstacle field (js/obstacleField.js), run once per 500 Hz step in both the
 * Legacy SHAKE core and the DAE core.
 *
 * Contact points (31 per rover):
 *   - the 12 rod end-nodes (radius cfg.nodeRadius)
 *   - 3 interior quadrature points per rigid strut, t = 1/4, 1/2, 3/4 (18 points,
 *     radius STRUT_RADIUS), so a strut cannot knife through a box corner or a
 *     sharp rock between its end-nodes
 *   - the payload core sphere (radius rover.R_core)
 *
 * For each penetrating point (phi = sdf - r < 0):
 *   position  : project out along the analytic SDF normal by -phi. Every correction
 *               (end-node or strut point) translates both end-nodes of the rod by the
 *               same vector, so rod lengths (R(q) = 0) are preserved exactly; the step
 *               therefore runs after the DAE constraint corrector without disturbing it.
 *   velocity  : inelastic normal impulse with restitution e = 0.02 (locked) and a
 *               Coulomb-bounded tangential impulse |dv_t| <= mu |dv_n|.
 * The caller books the total energy change of this step into the work ledger
 * category 'contact' (Simulation.bookProjection), so E(t) = E0 + sum W(t)
 * stays closed and the unbooked drift |eps| stays ~0.
 */
import { sdf, obstaclesNear } from './obstacleField.js';

export const OBSTACLE_RESTITUTION = 0.02;          // locked
export const STRUT_RADIUS = 0.035;
export const STRUT_QUADRATURE = Object.freeze([0.25, 0.5, 0.75]);
export const OBSTACLE_FRICTION = 0.70;              // basalt / concrete on the shell end-caps

const normal = new Float64Array(3);

/**
 * Resolve obstacle contact for one simulation in place.
 * @param {object} sim Simulation (uses q, v, corePosition, coreVelocity, rover, cfg)
 * @param {Array} field obstacle list (lane-local coordinates)
 * @returns {object} step report: contacts, maxPenetration, minClearance, normalImpulse,
 *          frictionImpulse, touches: Map(id -> {top: bool, maxNormalZ})
 */
export function resolveObstacleContacts(sim, field, options = {}) {
  const report = { contacts: 0, maxPenetration: 0, minClearance: Infinity, normalImpulse: 0, frictionImpulse: 0, touches: new Map() };
  if (!field?.length) return report;
  const q = sim.q, v = sim.v, cfg = sim.cfg;
  const e = OBSTACLE_RESTITUTION;
  const mu = (options.friction ?? cfg.obstacleFriction ?? OBSTACLE_FRICTION) * (cfg.plantFrictionScale ?? 1);
  const nodeR = cfg.nodeRadius, coreR = sim.rover.R_core;

  // Broad phase: obstacles within reach of the rover's bounding sphere.
  let cx = 0, cy = 0;
  for (const p of q) { cx += p[0]; cy += p[1]; }
  cx /= q.length; cy /= q.length;
  const near = obstaclesNear(field, cx, cy, sim.rover.R_outer + nodeR + 0.15);
  if (!near.length) return report;

  const touch = (o, nz) => {
    const t = report.touches.get(o.id) || { top: false, maxNormalZ: -1 };
    t.maxNormalZ = Math.max(t.maxNormalZ, nz);
    if (nz > 0.55) t.top = true;
    report.touches.set(o.id, t);
  };

  // One contact point: position p (array view or computed), radius r, and an
  // apply(dx, dv) callback distributing the correction to the owning DOFs.
  const solvePoint = (px, py, pz, vx, vy, vz, r, apply) => {
    let moved = false, ox = 0, oy = 0, oz = 0;
    for (const o of near) {
      if (pz + oz - r > o.top + 0.02) continue;
      const phi = sdf(o, px + ox, py + oy, pz + oz, normal) - r;
      report.minClearance = Math.min(report.minClearance, phi);
      if (phi >= 0) continue;
      const nx = normal[0], ny = normal[1], nz = normal[2];
      report.contacts++;
      report.maxPenetration = Math.max(report.maxPenetration, -phi);
      touch(o, nz);
      ox -= phi * nx; oy -= phi * ny; oz -= phi * nz;
      moved = true;
      // Velocity: restitution e on the approaching normal component, Coulomb on the tangent.
      const vn = vx * nx + vy * ny + vz * nz;
      if (vn < 0) {
        const dvn = -(1 + e) * vn;
        const tx = vx - vn * nx, ty = vy - vn * ny, tz = vz - vn * nz;
        const vt = Math.hypot(tx, ty, tz);
        const scale = vt > 1e-12 ? Math.min(1, mu * dvn / vt) : 0;
        const dvx = dvn * nx - scale * tx, dvy = dvn * ny - scale * ty, dvz = dvn * nz - scale * tz;
        vx += dvx; vy += dvy; vz += dvz;
        report.normalImpulse += dvn;
        report.frictionImpulse += scale * vt;
        apply(null, [dvx, dvy, dvz]);
      }
    }
    if (moved) apply([ox, oy, oz], null);
  };

  // 12 rod end-nodes. The correction translates the node's whole rod (both end-nodes),
  // so rod lengths and rod-length rates are preserved exactly and the constraint
  // corrector / SHAKE never has to push the node back into the obstacle.
  const partner = sim.rodPartner || (sim.rodPartner = (() => {
    const map = new Array(q.length).fill(-1);
    for (const [i, j] of sim.rover.bars) { map[i] = j; map[j] = i; }
    return map;
  })());
  for (let i = 0; i < q.length; i++) {
    const p = q[i], w = v[i], j = partner[i];
    const pj = j >= 0 ? q[j] : null, wj = j >= 0 ? v[j] : null;
    solvePoint(p[0], p[1], p[2], w[0], w[1], w[2], nodeR, (dx, dv) => {
      if (dx) for (let k = 0; k < 3; k++) { p[k] += dx[k]; if (pj) pj[k] += dx[k]; }
      if (dv) for (let k = 0; k < 3; k++) { w[k] += dv[k]; if (wj) wj[k] += dv[k]; }
    });
  }
  // 18 strut quadrature points (translate both end-nodes: rod length unchanged)
  for (const [i, j] of sim.rover.bars) {
    for (const t of STRUT_QUADRATURE) {
      const a = q[i], b = q[j], va = v[i], vb = v[j];
      solvePoint(
        a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2]),
        va[0] + t * (vb[0] - va[0]), va[1] + t * (vb[1] - va[1]), va[2] + t * (vb[2] - va[2]),
        STRUT_RADIUS, (dx, dv) => {
          if (dx) for (let k = 0; k < 3; k++) { a[k] += dx[k]; b[k] += dx[k]; }
          if (dv) for (let k = 0; k < 3; k++) { va[k] += dv[k]; vb[k] += dv[k]; }
        });
    }
  }
  // payload core sphere
  const cp = sim.corePosition, cv = sim.coreVelocity;
  if (cp && cv) {
    solvePoint(cp[0], cp[1], cp[2], cv[0], cv[1], cv[2], coreR, (dx, dv) => {
      if (dx) { cp[0] += dx[0]; cp[1] += dx[1]; cp[2] += dx[2]; }
      if (dv) { cv[0] += dv[0]; cv[1] += dv[1]; cv[2] += dv[2]; }
    });
  }
  return report;
}

/** Signed clearance of every contact point (no state change); min over points. */
export function obstacleClearance(sim, field) {
  if (!field?.length) return Infinity;
  let min = Infinity;
  const n = new Float64Array(3);
  const check = (x, y, z, r) => { for (const o of obstaclesNear(field, x, y, r)) min = Math.min(min, sdf(o, x, y, z, n) - r); };
  for (const p of sim.q) check(p[0], p[1], p[2], sim.cfg.nodeRadius);
  for (const [i, j] of sim.rover.bars) for (const t of STRUT_QUADRATURE) {
    const a = sim.q[i], b = sim.q[j];
    check(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2]), STRUT_RADIUS);
  }
  if (sim.corePosition) check(...sim.corePosition, sim.rover.R_core);
  return min;
}
