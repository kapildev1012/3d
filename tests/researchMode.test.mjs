import test from 'node:test';
import assert from 'node:assert/strict';
import { SimConfig, SphericalRoverModel, TerrainModel, Simulation, assistEnergyShare } from '../js/simEngine.js';
import { validateObject, validateSettingsFile, EXPERIMENT_SCHEMA } from '../js/configSchema.js';
import { latexTable, meanStd } from '../js/experimentProtocol.js';

const quiet = fn => { const info = console.info; console.info = () => {}; try { return fn(); } finally { console.info = info; } };

test('SGT rolls the rover forward with cable actuation only (no assists)', () => {
  const cfg = new SimConfig({ terrainLevel: 1, experimentId: 1, seed: 3, enableDiagnosticsLog: false, monitoring: { enabled: false } });
  cfg.applyStandardPerformanceProfile();
  cfg.applyResearchProfile();
  const sim = quiet(() => new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive'));
  const y0 = sim.currentDiag.centroid[1];
  quiet(() => { for (let k = 0; k < Math.round(15/cfg.dt); k++) sim.step(); });
  const L = sim.currentDiag.energyLedger;
  assert.equal(L.work.assist, 0);
  assert.equal(L.work.virtual, 0);
  assert.equal(L.assistShare, 0);
  assert.ok(sim.sgt.tipCount >= 2, `tips ${sim.sgt.tipCount}`);
  assert.ok(sim.currentDiag.centroid[1]-y0 > 0.3, `progress ${sim.currentDiag.centroid[1]-y0}`);
  assert.ok(L.maxAbsResidual < 1e-8);
});

test('disturbance work is booked and the ledger stays closed', () => {
  const cfg = new SimConfig({ terrainLevel: 1, experimentId: 1, enableDiagnosticsLog: false, monitoring: { enabled: false },
    physicsCore: 'dae', pureCableActuation: true, disturbance: { type: 'step', time: 0.5, duration: 0.5, force: [10, 0, 0] } });
  cfg.applyStandardPerformanceProfile();
  const sim = quiet(() => new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive'));
  quiet(() => { for (let k = 0; k < 750; k++) sim.step(); });
  sim.injectImpulse([0.3, 0, 0]);
  quiet(() => { for (let k = 0; k < 50; k++) sim.step(); });
  assert.ok(Math.abs(sim.ledger.work.disturbance) > 1e-3);
  assert.ok(sim.ledger.maxAbsResidual < 1e-8, `residual ${sim.ledger.maxAbsResidual}`);
  // Pre-correction drift is measured separately from the enforced residual.
  assert.ok(Number.isFinite(sim.ledger.driftAbs) && sim.ledger.driftAbs > 0);
  assert.ok(sim.ledger.maxStepDrift < 0.5, `step drift ${sim.ledger.maxStepDrift}`);
  assert.ok(Number.isFinite(sim.currentDiag.energyLedger.maxStepDrift));
});

test('legacy remains the interactive default; research profile selects the DAE', () => {
  const cfg = new SimConfig({ enableDiagnosticsLog: false });
  assert.equal(cfg.physicsCore, 'legacy');
  cfg.applyResearchProfile();
  assert.equal(cfg.physicsCore, 'dae');
  assert.equal(cfg.pureCableActuation, true);
  assert.equal(cfg.controllerMode, 'sgt_tipping');
});

test('assist share is the aid fraction of the energy input', () => {
  assert.equal(assistEnergyShare({ assist: 0, virtual: 0 }, 10), 0);
  assert.equal(assistEnergyShare({ assist: 30, virtual: -5 }, 10), 0.75);
});

test('schema validation clamps, rounds and rejects with explicit messages', () => {
  const { values, issues } = validateObject(EXPERIMENT_SCHEMA, { actuatorDelay: '9', seed: 4.6, noiseModel: 'pink', sgtActiveCables: 'abc' });
  assert.equal(values.actuatorDelay, 0.5);
  assert.equal(values.seed, 5);
  assert.equal(values.noiseModel, 'none');
  assert.equal(values.sgtActiveCables, 24);
  assert.equal(issues.length, 4);
  const settings = validateSettingsFile({ monitoring: { goalThreshold: -1, typoKey: 3, enabled: 'yes' } });
  assert.equal(settings.settings.monitoring.goalThreshold, 0.01);
  assert.equal(settings.settings.monitoring.enabled, true);
  assert.ok(settings.issues.some(issue => issue.includes('typoKey')));
});

test('LaTeX table uses booktabs, mean ± std and bolds only strict winners', () => {
  const cell = values => meanStd(values);
  const rows = [
    { label: 'A', cells: { v: cell([1, 2, 3]), s: cell([0, 0]) } },
    { label: 'B', cells: { v: cell([0.5, 0.7]), s: cell([0, 0]) } },
    { label: 'Ref', reference: true, cells: { v: cell([9, 9]), s: cell([1, 1]) } }
  ];
  const tex = latexTable(rows, [{ key: 'v', label: 'v', digits: 2, better: 'max' }, { key: 's', label: 's', digits: 0, better: 'max' }]);
  assert.match(tex, /\\toprule/);
  assert.match(tex, /\\midrule/);
  assert.match(tex, /\\bottomrule/);
  assert.match(tex, /\\textbf\{2\.00\\,\$\\pm\$\\,1\.00\}/);
  assert.ok(!tex.includes('\\textbf{9'), 'reference rows are never bold');
  assert.ok(!tex.includes('\\textbf{0'), 'ties are not bold');
});
