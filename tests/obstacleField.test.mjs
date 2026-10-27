import test from 'node:test';
import assert from 'node:assert/strict';
import { buildObstacleFieldForLevel, anchorObstacleField, sdf, SPAWN_PAD_RADIUS, footprintRadius } from '../js/obstacleField.js';
import { resolveObstacleContacts, obstacleClearance, OBSTACLE_RESTITUTION } from '../js/obstacleContact.js';
import { FieldObstacleTracker, mergeSummaries } from '../js/obstacleTracker.js';
import { SimConfig, Simulation, TerrainModel, SphericalRoverModel } from '../js/simEngine.js?v=20261027-ob1';
import { configureLevel } from '../js/levels.js';
import { buildConfig } from '../js/experimentProtocol.js';

console.info = () => {};
const flat = () => 0;

test('every level 1-14 gets a deterministic obstacle field with all four kinds across the levels', () => {
  const kinds = new Set();
  for (let level = 1; level <= 14; level++) {
    const cfg = new SimConfig({ seed: 3 }); configureLevel(cfg, level);
    const a = new TerrainModel(cfg).obstacleField, b = new TerrainModel(cfg).obstacleField;
    assert.ok(a.length > 0, `level ${level} has obstacles`);
    assert.deepEqual(a.map(o => [o.id, o.kind, o.x, o.y]), b.map(o => [o.id, o.kind, o.x, o.y]));
    a.forEach(o => kinds.add(o.kind));
  }
  assert.deepEqual([...kinds].sort(), ['box', 'cylinder', 'ramp', 'rock']);
});

test('spawn pad: no obstacle footprint within 2.5 m of the start point', () => {
  for (let level = 1; level <= 14; level++) {
    const startY = level === 10 ? 9 : level === 14 ? -451 : 0;
    const field = buildObstacleFieldForLevel(level, 0.06, 7, { startY, goalY: startY+40, heightAt: flat });
    for (const o of field) assert.ok(Math.hypot(o.x, o.y-startY)-footprintRadius(o) > SPAWN_PAD_RADIUS, `${level}/${o.id}`);
  }
});

test('obstacles are anchored to the heightfield and re-seated when the ground changes', () => {
  const field = buildObstacleFieldForLevel(4, 0.09, 2, { startY: 0, goalY: 25, heightAt: flat });
  const tops = field.map(o => o.top);
  anchorObstacleField(field, () => 0.30, 0.02);
  field.forEach((o, i) => assert.ok(Math.abs(o.top-tops[i]-0.30) < 0.05, `${o.id} moved with the ground`));
  // roughness slider path: TerrainModel keeps the layout and re-anchors
  const cfg = new SimConfig({ seed: 2 }); configureLevel(cfg, 7);
  const terrain = new TerrainModel(cfg);
  const layout = terrain.obstacleField.map(o => [o.id, o.x, o.y]);
  cfg.groundRMS = 0.16; terrain.generateSurface();
  assert.deepEqual(terrain.obstacleField.map(o => [o.id, o.x, o.y]), layout);
  for (const o of terrain.obstacleField) assert.ok(o.zBase < terrain.eval(o.x, o.y).h, `${o.id} base below ground`);
});

test('SDFs: sign inside/outside and the analytic normal equals the finite-difference gradient', () => {
  const shapes = anchorObstacleField([
    { id: 'b', kind: 'box', x: 0, y: 0, yaw: 0.5, hx: 0.3, hy: 0.2, height: 0.15, chamfer: 0.02 },
    { id: 'r', kind: 'rock', x: 0, y: 0, yaw: 0.2, height: 0.15, lobes: [{ dx: 0, dy: 0, r: 0.12, top: 0.15 }, { dx: 0.1, dy: 0, r: 0.07, top: 0.1 }] },
    { id: 'c', kind: 'cylinder', x: 0, y: 0, yaw: 0.3, radius: 0.07, halfLength: 0.5 },
    { id: 'w', kind: 'ramp', x: 0, y: 0, yaw: 0.1, hx: 0.5, ly: 0.4, height: 0.15 }
  ], flat, 0.03);
  const n = [0, 0, 0], m = [0, 0, 0], h = 1e-6;
  for (const o of shapes) {
    assert.ok(sdf(o, o.x, o.y, o.groundZ+0.02, n) < 0, `${o.kind} inside`);
    assert.ok(sdf(o, o.x, o.y, o.top+0.2, n) > 0, `${o.kind} outside`);
    for (const p of [[0.05, 0.02, o.top+0.05], [0.4, 0.1, 0.05], [-0.1, 0.45, 0.08], [0.02, -0.03, o.top-0.01]]) {
      sdf(o, ...p, n);
      const g = [0, 1, 2].map(k => { const a = p.slice(), b = p.slice(); a[k] += h; b[k] -= h; return (sdf(o, ...a, m)-sdf(o, ...b, m))/(2*h); });
      const dot = g[0]*n[0]+g[1]*n[1]+g[2]*n[2];
      assert.ok(Math.abs(Math.hypot(...n)-1) < 1e-9, `${o.kind} unit normal`);
      assert.ok(dot > 0.999, `${o.kind} normal matches gradient at ${p}: ${dot}`);
    }
  }
});

function dropOnto(shape, core = 'dae') {
  const cfg = buildConfig({ method: 'sgt', level: 1, seed: 1 });
  if (core === 'legacy') cfg.physicsCore = 'legacy';
  cfg.obstacleField = false;
  const terrain = new TerrainModel(cfg);
  terrain.obstacleField = anchorObstacleField([shape], (x, y) => terrain.eval(x, y).h, 0.03);
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), terrain, 'adaptive');
  for (const p of sim.q) p[2] += 0.18; sim.corePosition[2] += 0.18;
  sim.ledger.E0 = sim.ledgerEnergy(); sim.ledger.E = sim.ledger.E0;
  for (const k of Object.keys(sim.ledger.work)) sim.ledger.work[k] = 0;
  return sim;
}

for (const core of ['dae', 'legacy']) {
  test(`${core}: dropping the shell onto a crate - no penetration after any step, rods rigid, contact dissipative${core === 'dae' ? ', ledger closed' : ''}`, () => {
    const sim = dropOnto({ id: 'T1', kind: 'box', template: 'crate', role: 'cross', x: 0.05, y: 0.1, yaw: 0.4, hx: 0.25, hy: 0.2, height: 0.12, chamfer: 0.018 }, core);
    let maxRes = 0, minClear = Infinity, maxRod = 0;
    for (let k = 0; k < 700; k++) {
      sim.step();
      maxRes = Math.max(maxRes, Math.abs(sim.ledger.residual));
      if (k > 5) minClear = Math.min(minClear, obstacleClearance(sim, sim.terrain.obstacleField));
      sim.rover.bars.forEach(([i, j], b) => {
        const a = sim.q[i], c = sim.q[j];
        maxRod = Math.max(maxRod, Math.abs(Math.hypot(a[0]-c[0], a[1]-c[1], a[2]-c[2])-sim.rover.l0_bars[b]));
      });
    }
    assert.ok(sim.metrics.obstacleContacts > 0);
    assert.ok(minClear > -1e-6, `post-step clearance ${minClear}`);
    assert.ok(maxRod < 1e-6, `rod error ${maxRod}`);
    if (core === 'dae') assert.ok(maxRes < 1e-6, `|eps| ${maxRes}`);
    assert.ok(sim.metrics.obstacleSummary.records[0].topContact, 'shell rests on the crate top');
  });
}

test('contact impulse: restitution 0.02 locked, Coulomb bound, strut points stop a knifing rod', () => {
  assert.equal(OBSTACLE_RESTITUTION, 0.02);
  // a single horizontal rod falling edge-first onto a thin pillar between its end-nodes
  const pillar = anchorObstacleField([{ id: 'P', kind: 'box', x: 0, y: 0, yaw: 0, hx: 0.04, hy: 0.04, height: 0.4, chamfer: 0.005 }], flat, 0.02);
  const sim = {
    cfg: { nodeRadius: 0.05 }, rover: { bars: [[0, 1]], R_outer: 0.5, R_core: 0.05 },
    q: [[-0.4, 0, 0.37], [0.4, 0, 0.37]], v: [[0, 0, -1], [0, 0, -1]], corePosition: null, coreVelocity: null
  };
  const r = resolveObstacleContacts(sim, pillar);
  assert.ok(r.contacts > 0, 'mid-rod quadrature point detected the pillar');
  assert.ok(sim.q[0][2] >= 0.4+0.035-1e-9 && Math.abs(sim.q[0][2]-sim.q[1][2]) < 1e-12, 'rod lifted rigidly onto the pillar top');
  assert.ok(Math.abs(sim.v[0][2]-0.02) < 1e-12, `rebound speed ${sim.v[0][2]} = e * 1 m/s`);
});

test('tracker: over vs around and bypass violations (Model B only)', () => {
  const field = anchorObstacleField([
    { id: 'O01', kind: 'box', template: 'crate', role: 'cross', x: 0, y: 5, yaw: 0, hx: 0.2, hy: 0.2, height: 0.08, chamfer: 0.01 },
    { id: 'O02', kind: 'box', template: 'crate', role: 'cross', x: 0, y: 8, yaw: 0, hx: 0.2, hy: 0.2, height: 0.08, chamfer: 0.01 },
    { id: 'O03', kind: 'box', template: 'pillar', role: 'gate', x: 0.9, y: 11, yaw: 0, hx: 0.09, hy: 0.09, height: 0.4, chamfer: 0.01 }
  ], flat, 0.02);
  for (const [model, expectViolations] of [['adaptive', 1], ['fixed', 0]]) {
    const t = new FieldObstacleTracker(field, model);
    for (let y = 3; y <= 13; y += 0.05) {
      const x = y > 7 && y < 9 ? 0.7 : 0;            // swerve around O02 only
      t.update({ x, y, touches: Math.abs(y-5) < 0.1 ? new Map([['O01', { top: true, maxNormalZ: 0.9 }]]) : null });
    }
    const s = t.summary();
    assert.deepEqual([s.over, s.around, s.bypassViolations], [1, 2, expectViolations], model);
  }
  assert.equal(mergeSummaries({ over: 2, around: 1, retries: 0, total: 3, bypassViolations: 1 }, { over: 1, around: 2, total: 3, pending: 0, bypassViolations: 1 }).over, 3);
});
