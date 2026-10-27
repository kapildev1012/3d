/**
 * OBSTACLEFIELD.JS - deterministic solid 3D obstacles for every level (1-14).
 *
 * buildObstacleFieldForLevel(levelId, roughnessRms, seed, course) returns a list
 * of rigid, static obstacles in lane-local coordinates (x = lateral offset inside
 * a model's lane, y = along the course, z = up). The same list is used by Model A
 * (Baseline) and Model B (Adaptive): each lane renders and collides against an
 * identical copy, so the two models always face the same obstacles.
 *
 * Kinds (each with an exact signed distance function, see sdf()):
 *   box       oriented bounding box with a chamfer radius (crates, step ledges,
 *             low walls, narrow gate pillars)
 *   rock      jagged basalt boulder = union of sphere-swept lobes
 *   cylinder  fallen pipe / rubble roller lying on the ground
 *   ramp      inclined wedge rising along the course direction
 *
 * The layout depends only on (levelId, seed); obstacle elevations are anchored
 * to the terrain heightfield, so dragging the roughness slider (which calls
 * TerrainModel.generateSurface()) re-seats every obstacle on the new ground.
 * A clear spawn pad (default 2.5 m) is kept around the start point so the
 * PASSIVE_SETTLE phase never starts inside an obstacle.
 */

export const OBSTACLE_KINDS = Object.freeze(['box', 'rock', 'cylinder', 'ramp']);
export const SPAWN_PAD_RADIUS = 2.5;

/** Per-level obstacle profile: difficulty d in [0, 1] and template weights. */
export const LEVEL_OBSTACLE_PROFILES = Object.freeze({
  1: { d: 0.15, spacing: 3.2, mix: { crate: 2, rock: 3, roller: 2, ramp: 1, ledge: 1 } },
  2: { d: 0.30, spacing: 2.8, mix: { rock: 5, crate: 1, roller: 1 } },
  3: { d: 0.45, spacing: 2.8, mix: { rock: 5, crate: 1, gate: 1 } },
  4: { d: 0.55, spacing: 2.7, mix: { ledge: 3, ramp: 2, rock: 2, wall: 1 } },
  5: { d: 0.40, spacing: 3.0, mix: { rock: 3, roller: 2, crate: 1 } },
  6: { d: 0.35, spacing: 3.2, mix: { ramp: 2, rock: 2, roller: 2 } },
  7: { d: 0.60, spacing: 2.6, mix: { rock: 3, ledge: 1, ramp: 1, crate: 1, gate: 1 } },
  8: { d: 0.65, spacing: 2.4, mix: { rock: 4, crate: 2, roller: 1, wall: 1 } },
  9: { d: 0.40, spacing: 3.0, mix: { roller: 2, rock: 2, ramp: 1 } },
  10: { d: 0.50, spacing: 3.0, mix: { crate: 2, roller: 2, gate: 1, ledge: 1 } },
  11: { d: 0.75, spacing: 2.3, mix: { rock: 3, roller: 3, crate: 2, wall: 1 } },
  12: { d: 0.70, spacing: 2.5, mix: { rock: 3, roller: 2, ramp: 1, gate: 1 } },
  13: { d: 0.90, spacing: 2.3, mix: { rock: 3, crate: 2, ledge: 2, ramp: 1, roller: 1, gate: 1, wall: 1 } },
  14: { d: 0.60, spacing: 16.0, mix: { rock: 3, crate: 2, roller: 2, ramp: 1, gate: 1, ledge: 1 }, maxCount: 60 }
});

// ------------------------------------------------------------------ seeded RNG
export function obstacleRng(seed) {
  let a = (seed >>> 0) || 0x9e3779b9;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const pickWeighted = (rng, mix) => {
  const entries = Object.entries(mix);
  const total = entries.reduce((s, [, w]) => s + w, 0);
  let r = rng() * total;
  for (const [name, w] of entries) { r -= w; if (r <= 0) return name; }
  return entries[entries.length - 1][0];
};
const lerp = (a, b, t) => a + (b - a) * t;

/**
 * Generate the obstacle layout for one level.
 * @param {number} levelId 1..14
 * @param {number} roughnessRms terrain RMS [m] (sets how deep obstacles are bedded)
 * @param {number} seed run seed (layout is a pure function of levelId and seed)
 * @param {object} course { startY, goalY, heightAt(x, y), avoid: [{y, halfLength}], corridorHalfWidth, spawnPad }
 */
export function buildObstacleFieldForLevel(levelId, roughnessRms, seed, course = {}) {
  const profile = LEVEL_OBSTACLE_PROFILES[levelId] || LEVEL_OBSTACLE_PROFILES[1];
  const d = profile.d;
  const rng = obstacleRng(((seed ?? 1) * 2654435761 ^ (levelId * 0x51ed27)) >>> 0);
  const startY = course.startY ?? 0;
  const goalY = course.goalY ?? 25;
  const pad = course.spawnPad ?? SPAWN_PAD_RADIUS;
  const corridor = course.corridorHalfWidth ?? 1.25;
  const avoid = course.avoid || [];
  const heightAt = course.heightAt || (() => 0);
  const embed = 0.02 + 0.5 * Math.max(0, roughnessRms || 0);   // bedding depth below local ground

  // "Over" obstacles (on the rolling path) stay below what a 1 m cable-actuated shell can crest;
  // "around" obstacles (offset, pillars, walls) may be taller and are passed laterally.
  const lowH = 0.045 + 0.075 * d;        // 0.056 m (L1) ... 0.113 m (L13)
  const tallH = 0.20 + 0.30 * d;         // 0.245 m ... 0.47 m

  const field = [];
  const spacing = profile.spacing;
  let y = startY + pad + 0.8;
  const endY = goalY - 1.2;
  let side = rng() < 0.5 ? -1 : 1;
  while (y < endY && field.length < (profile.maxCount ?? 40)) {
    const blocked = avoid.some(a => Math.abs(y - a.y) < (a.halfLength ?? 1.5) + 0.6);
    if (blocked) { y += 0.4; continue; }
    const template = pickWeighted(rng, profile.mix);
    side = rng() < 0.7 ? -side : side;
    const id = n => `O${String(field.length + 1 + n).padStart(2, '0')}`;
    const yaw = (rng() - 0.5) * 0.9;
    switch (template) {
      case 'crate': {
        const onPath = rng() < 0.55;
        const hx = lerp(0.12, 0.22, rng()), hy = lerp(0.12, 0.22, rng());
        const h = onPath ? lowH * lerp(0.8, 1.0, rng()) : tallH * lerp(0.6, 1.0, rng());
        const x = onPath ? (rng() - 0.5) * 0.4 : side * lerp(0.42, 0.75, rng());
        field.push(box(id(0), 'crate', onPath ? 'cross' : 'bypass', x, y, yaw, hx, hy, h, 0.018));
        break;
      }
      case 'ledge': {                                   // step ledge spanning the lane: must be crested
        const hy = lerp(0.22, 0.45, rng());
        field.push(box(id(0), 'ledge', 'cross', (rng() - 0.5) * 0.2, y, (rng() - 0.5) * 0.12, corridor + 0.05, hy, lowH * lerp(0.7, 0.9, rng()), 0.02));
        break;
      }
      case 'wall': {                                    // low wall reaching in from one side
        const hx = lerp(0.35, 0.55, rng());
        field.push(box(id(0), 'wall', 'bypass', side * (corridor - hx + 0.05), y, (rng() - 0.5) * 0.25, hx, 0.07, tallH * lerp(0.45, 0.65, rng()), 0.012));
        break;
      }
      case 'gate': {                                    // two narrow pillars, 1.2+ m clear gap
        const half = lerp(0.70, 0.80, rng()), r = 0.09, h = tallH * lerp(0.9, 1.15, rng());
        const gx = (rng() - 0.5) * 0.15;
        field.push(box(id(0), 'pillar', 'gate', gx - half - r, y, 0, r, r, h, 0.01));
        field.push(box(id(1), 'pillar', 'gate', gx + half + r, y, 0, r, r, h, 0.01));
        break;
      }
      case 'roller': {                                  // fallen pipe across (part of) the lane
        const radius = 0.03 + 0.035 * d * lerp(0.8, 1.1, rng());
        const halfLength = lerp(0.45, 0.95, rng());
        const x = (rng() - 0.5) * 0.6;
        field.push({ id: id(0), kind: 'cylinder', template: 'roller', role: 'cross', x, y, yaw: (rng() - 0.5) * 0.7, radius, halfLength });
        break;
      }
      case 'ramp': {
        const hx = lerp(0.40, 0.65, rng()), ly = lerp(0.30, 0.50, rng());
        field.push({ id: id(0), kind: 'ramp', template: 'ramp', role: 'cross', x: (rng() - 0.5) * 0.4, y, yaw: (rng() - 0.5) * 0.3, hx, ly, height: lowH * lerp(1.0, 1.4, rng()) });
        break;
      }
      default: {                                        // jagged basalt boulder (sphere-swept cluster)
        const onPath = rng() < 0.6;
        const h = onPath ? lowH * lerp(0.9, 1.25, rng()) : tallH * lerp(0.5, 0.85, rng());
        const x = onPath ? (rng() - 0.5) * 0.5 : side * lerp(0.45, 0.8, rng());
        const lobes = [];
        const count = 3 + Math.floor(rng() * 3);
        const base = Math.max(0.05, Math.min(0.20, 0.9 * h));
        for (let k = 0; k < count; k++) {
          const a = k * 2.39996 + rng() * 0.6, rad = k === 0 ? 0 : base * lerp(0.45, 0.9, rng());
          lobes.push({ dx: rad * Math.cos(a), dy: rad * Math.sin(a), r: base * lerp(k === 0 ? 0.95 : 0.55, k === 0 ? 1.1 : 0.85, rng()), top: k === 0 ? h : h * lerp(0.55, 0.85, rng()) });
        }
        field.push({ id: id(0), kind: 'rock', template: 'boulder', role: onPath ? 'cross' : 'bypass', x, y, yaw, height: h, lobes, facetSeed: Math.floor(rng() * 1e9) });
      }
    }
    y += spacing * lerp(0.8, 1.25, rng());
  }
  // Spawn pad: drop anything whose footprint reaches into the pad around the start point.
  const kept = field.filter(o => Math.hypot(o.x, o.y - startY) - footprintRadius(o) > pad);
  anchorObstacleField(kept, heightAt, embed);
  kept.sort((a, b) => a.minY - b.minY);           // obstaclesNear() relies on this order
  kept.forEach((o, i) => { o.index = i; });
  return kept;
}

function box(id, template, role, x, y, yaw, hx, hy, height, chamfer) {
  return { id, kind: 'box', template, role, x, y, yaw, hx, hy, height, chamfer };
}

export function footprintRadius(o) {
  switch (o.kind) {
    case 'box': return Math.hypot(o.hx, o.hy);
    case 'cylinder': return o.halfLength + o.radius;
    case 'ramp': return Math.hypot(o.hx, o.ly);
    case 'rock': return Math.max(...o.lobes.map(l => Math.hypot(l.dx, l.dy) + l.r));
    default: return 0.5;
  }
}

/**
 * Seat every obstacle on the heightfield. Called on generation and whenever the
 * terrain is regenerated (roughness slider); only z-related fields change.
 * The base sits `embed` below the LOWEST ground under the footprint, so no
 * corner floats above a hollow; heights are measured from the local ground.
 */
export function anchorObstacleField(field, heightAt, embed = 0.03) {
  for (const o of field) {
    const c = Math.cos(o.yaw || 0), s = Math.sin(o.yaw || 0);
    const samples = [[0, 0]];
    const ex = o.kind === 'cylinder' ? o.halfLength : o.kind === 'rock' ? footprintRadius(o) * 0.7 : o.hx;
    const ey = o.kind === 'cylinder' ? o.radius : o.kind === 'ramp' ? o.ly : o.kind === 'rock' ? footprintRadius(o) * 0.7 : o.hy;
    for (const [u, v] of [[1, 1], [1, -1], [-1, 1], [-1, -1], [1, 0], [-1, 0], [0, 1], [0, -1]]) samples.push([u * ex, v * ey]);
    let minH = Infinity, centreH = 0;
    samples.forEach(([u, v], k) => {
      const h = heightAt(o.x + c * u - s * v, o.y + s * u + c * v);
      if (k === 0) centreH = h;
      minH = Math.min(minH, h);
    });
    o.groundZ = centreH;
    o.zBase = minH - embed;
    o.embed = centreH - o.zBase;                     // material below the centre ground level
    if (o.kind === 'box') {
      o.hz = 0.5 * (o.height + o.embed);
      o.cz = o.zBase + o.hz;
      o.top = o.zBase + 2 * o.hz;
    } else if (o.kind === 'cylinder') {
      o.cz = centreH + o.radius - Math.min(0.5 * o.radius, embed * 0.5);
      o.top = o.cz + o.radius;
    } else if (o.kind === 'ramp') {
      o.top = centreH + o.height;
    } else if (o.kind === 'rock') {
      for (const lobe of o.lobes) lobe.cz = centreH + lobe.top - lobe.r;
      o.top = centreH + o.height;
    }
    o.boundR = footprintRadius(o) + 0.02;
    o.minY = o.y - o.boundR; o.maxY = o.y + o.boundR;
  }
  return field;
}

// ------------------------------------------------------------------ signed distance functions
// sdf(o, p, out) returns phi (negative inside) and writes the outward unit normal into out[0..2].

function sdBox(o, px, py, pz, out) {
  const c = Math.cos(o.yaw), s = Math.sin(o.yaw);
  const dx = px - o.x, dy = py - o.y, dz = pz - o.cz;
  const lx = c * dx + s * dy, ly = -s * dx + c * dy, lz = dz;
  const r = o.chamfer || 0;
  const qx = Math.abs(lx) - (o.hx - r), qy = Math.abs(ly) - (o.hy - r), qz = Math.abs(lz) - (o.hz - r);
  const mx = Math.max(qx, 0), my = Math.max(qy, 0), mz = Math.max(qz, 0);
  const outside = Math.hypot(mx, my, mz);
  let nx, ny, nz;
  if (outside > 1e-12) {
    nx = Math.sign(lx) * mx / outside; ny = Math.sign(ly) * my / outside; nz = Math.sign(lz) * mz / outside;
  } else if (qx >= qy && qx >= qz) { nx = Math.sign(lx) || 1; ny = 0; nz = 0; }
  else if (qy >= qz) { nx = 0; ny = Math.sign(ly) || 1; nz = 0; }
  else { nx = 0; ny = 0; nz = Math.sign(lz) || 1; }
  out[0] = c * nx - s * ny; out[1] = s * nx + c * ny; out[2] = nz;
  return outside + Math.min(Math.max(qx, qy, qz), 0) - r;
}

function sdRock(o, px, py, pz, out) {
  let best = Infinity;
  const c = Math.cos(o.yaw), s = Math.sin(o.yaw);
  for (const lobe of o.lobes) {
    const lx = o.x + c * lobe.dx - s * lobe.dy, ly = o.y + s * lobe.dx + c * lobe.dy;
    const dx = px - lx, dy = py - ly, dz = pz - lobe.cz;
    const len = Math.hypot(dx, dy, dz);
    const phi = len - lobe.r;
    if (phi < best) {
      best = phi;
      if (len > 1e-12) { out[0] = dx / len; out[1] = dy / len; out[2] = dz / len; } else { out[0] = 0; out[1] = 0; out[2] = 1; }
    }
  }
  return best;
}

function sdCylinder(o, px, py, pz, out) {
  const ax = Math.cos(o.yaw), ay = Math.sin(o.yaw);
  const wx = px - o.x, wy = py - o.y, wz = pz - o.cz;
  const sAx = wx * ax + wy * ay;
  let rx = wx - sAx * ax, ry = wy - sAx * ay, rz = wz;
  const rho = Math.hypot(rx, ry, rz);
  if (rho > 1e-12) { rx /= rho; ry /= rho; rz /= rho; } else { rx = 0; ry = 0; rz = 1; }
  const d1 = rho - o.radius, d2 = Math.abs(sAx) - o.halfLength;
  const sg = Math.sign(sAx) || 1;
  if (d1 > 0 && d2 > 0) {
    const h = Math.hypot(d1, d2);
    out[0] = (d1 * rx + d2 * sg * ax) / h; out[1] = (d1 * ry + d2 * sg * ay) / h; out[2] = d1 * rz / h;
    return h;
  }
  if (d1 > d2) { out[0] = rx; out[1] = ry; out[2] = rz; } else { out[0] = sg * ax; out[1] = sg * ay; out[2] = 0; }
  return Math.max(d1, d2);
}

/**
 * Wedge = intersection of 6 half-spaces. Inside the solid (phi <= 0, the only
 * case the contact solver projects) max-of-planes is the exact signed distance;
 * outside it is a lower bound (exact on faces, conservative near edges).
 */
function sdRamp(o, px, py, pz, out) {
  const c = Math.cos(o.yaw), s = Math.sin(o.yaw);
  const dx = px - o.x, dy = py - o.y;
  const lx = c * dx + s * dy, ly = -s * dx + c * dy, lz = pz - o.zBase;
  const e = o.embed, H = o.height, L = o.ly;
  const k = Math.hypot(H, 2 * L);
  const planes = [
    [-lx - o.hx, -1, 0, 0], [lx - o.hx, 1, 0, 0],
    [-ly - L, 0, -1, 0], [ly - L, 0, 1, 0],
    [-lz, 0, 0, -1],
    [(-H * (ly + L) + 2 * L * (lz - e)) / k, 0, -H / k, 2 * L / k]
  ];
  let best = planes[0];
  for (const p of planes) if (p[0] > best[0]) best = p;
  out[0] = c * best[1] - s * best[2]; out[1] = s * best[1] + c * best[2]; out[2] = best[3];
  return best[0];
}

export function sdf(o, px, py, pz, out) {
  switch (o.kind) {
    case 'box': return sdBox(o, px, py, pz, out);
    case 'rock': return sdRock(o, px, py, pz, out);
    case 'cylinder': return sdCylinder(o, px, py, pz, out);
    case 'ramp': return sdRamp(o, px, py, pz, out);
    default: return Infinity;
  }
}

/** Obstacles whose y-band can reach [y - reach, y + reach] (field sorted by minY). */
export function obstaclesNear(field, x, y, reach) {
  const out = [];
  for (const o of field) {
    if (o.minY - reach > y) break;
    if (o.maxY + reach < y) continue;
    if (Math.hypot(x - o.x, y - o.y) <= o.boundR + reach) out.push(o);
  }
  return out;
}

/** Top-of-obstacle height at (x, y) or -Infinity (for HUD/clearance probes; not used for contact). */
export function obstacleTopAt(field, x, y) {
  let top = -Infinity;
  const n = [0, 0, 0];
  for (const o of obstaclesNear(field, x, y, 0)) {
    if (sdf(o, x, y, o.top - 1e-4, n) <= 0) top = Math.max(top, o.top);
  }
  return top;
}
