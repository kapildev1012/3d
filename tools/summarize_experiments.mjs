#!/usr/bin/env node
/**
 * Aggregate results/paper/runs/*.json into
 *   results/paper/tables.tex      booktabs tables (mean ± std, best bold)
 *   results/paper/summary.json    aggregated metrics for every group
 *   results/paper/figure_data.json  mean ± 1 sigma time series for plotting
 */
import { readdirSync, readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { METHODS, SCENARIOS, ABLATIONS, meanStd, latexTable } from '../js/experimentProtocol.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const runDir = path.join(root, 'results', 'paper', 'runs');
const outDir = path.join(root, 'results', 'paper');
const runs = readdirSync(runDir).filter(f => f.endsWith('.json')).map(f => JSON.parse(readFileSync(path.join(runDir, f), 'utf8')));
console.log(`[summary] ${runs.length} runs`);

const pick = predicate => runs.filter(predicate);
const cells = (list, keys) => Object.fromEntries(keys.map(key => [key, meanStd(list.map(r => r.metrics[key]))]));
const KEYS = ['speed', 'progress', 'rmse', 'mae', 'recovery', 'effort', 'actuatorJ', 'cot', 'solveMsMean', 'solveMsMax',
  'success', 'assistShare', 'maxResidual', 'maxStepDrift', 'driftAbs', 'maxBarError', 'tips'];
const summary = {};

// ---------------------------------------------------------------- Table I
const nominal = pick(r => r.group === 'nominal');
const methodOrder = ['sgt', 'heuristic', 'lqr', 'ilqr', 'mpc', 'legacyAssisted'];
const rowsI = methodOrder.map(m => {
  const list = nominal.filter(r => r.spec.method === m);
  return list.length ? { key: m, label: (METHODS[m].proposed ? METHODS[m].label : METHODS[m].short)+(METHODS[m].reference ? '$^\\dagger$' : ''), reference: Boolean(METHODS[m].reference), n: list.length, cells: cells(list, KEYS) } : null;
}).filter(Boolean);
summary.nominal = rowsI;
const tableI = latexTable(rowsI, [
  { key: 'speed', label: 'Speed $\\bar v$\\,$\\uparrow$ (m/s)', digits: 3, better: 'max' },
  { key: 'rmse', label: 'RMSE\\,$\\downarrow$ (m)', digits: 3, better: null },
  { key: 'mae', label: 'max $|e|$\\,$\\downarrow$ (m)', digits: 3, better: null },
  { key: 'effort', label: 'Effort\\,$\\downarrow$ (m$^2$s)', digits: 3, better: 'min' },
  { key: 'cot', label: 'CoT\\,$\\downarrow$ (--)', digits: 1, better: 'min' },
  { key: 'solveMsMean', label: '$t_{\\mathrm{solve}}$\\,$\\downarrow$ (ms)', digits: 2, better: 'min' },
  { key: 'success', label: 'Success\\,$\\uparrow$ (\\%)', digits: 0, better: 'max', scale: 100, showStd: false },
  { key: 'assistShare', label: 'Aid\\,$\\downarrow$ (\\%)', digits: 0, better: 'min', scale: 100, showStd: false }
], {
  caption: 'Nominal cable-only locomotion (Level 1, 20\\,s, 5 seeds, mean\\,$\\pm$\\,std).',
  label: 'tab:nominal',
  wide: true,
  paper: true,
  note: 'All methods use the Lagrangian DAE core, passive contact and identical actuator limits. Effort: $\\int\\!\\vect u^{\\top}\\vect u\\,dt$. Success: $\\Delta y \\ge 1$\\,m in 20\\,s. Aid: share of the energy input supplied by non-physical aids. Bold: best admissible value; cross-track errors are not marked because they are only comparable between methods that travel a similar distance. $^\\dagger$Original engine with roll couple, traction, grip and corridor aids; not admissible, shown for reference.'
});

// ---------------------------------------------------------------- Table II physics
const sgtFull = nominal.filter(r => r.spec.method === 'sgt');
const ablation = key => pick(r => r.group === 'ablation' && r.spec.ablation === key);
const physicsRows = [
  { label: 'DAE + Alg.~1, energy row', list: sgtFull },
  { label: 'DAE + Alg.~1, constr.\\ only', list: ablation('noEnergyRow') },
  { label: 'Penalty + SHAKE (legacy)', list: ablation('legacyCore') },
  { label: 'Legacy + assists$^\\dagger$', list: nominal.filter(r => r.spec.method === 'legacyAssisted'), reference: true }
].filter(row => row.list.length).map(row => {
  // Numerical work: energy changed by projections and by the corrector [J]
  const numerical = row.list.filter(r => r.ledgerWork).map(r => Math.abs(r.ledgerWork.constraint || 0)+Math.abs(r.ledgerWork.corrector || 0));
  const aid = row.list.filter(r => r.ledgerWork).map(r => Math.abs(r.ledgerWork.assist || 0)+Math.abs(r.ledgerWork.virtual || 0));
  return { label: row.label, reference: Boolean(row.reference), n: row.list.length, cells: { ...cells(row.list, KEYS), assistShare: meanStd(row.list.map(r => r.metrics.assistShare)), numericalWork: meanStd(numerical), aidWork: meanStd(aid) } };
});
summary.physics = physicsRows;
const tableII = latexTable(physicsRows, [
  { key: 'maxResidual', label: 'max $|\\varepsilon|$ (J)', digits: 1, better: null, sci: true },
  { key: 'maxStepDrift', label: 'max $|\\delta_k|$\\,$\\downarrow$ (J)', digits: 1, better: null, sci: true },
  { key: 'driftAbs', label: '$\\sum_k|\\delta_k|$\\,$\\downarrow$ (J)', digits: 1, better: null, sci: true },
  { key: 'maxBarError', label: 'Bar error\\,$\\downarrow$ (m)', digits: 1, better: 'min', sci: true },
  { key: 'numericalWork', label: '$|W_{\\mathrm{num}}|$\\,$\\downarrow$ (J)', digits: 1, better: 'min', sci: true },
  { key: 'speed', label: 'Speed $\\bar v$ (m/s)', digits: 3, better: null },
  { key: 'assistShare', label: 'Aid\\,$\\downarrow$ (\\%)', digits: 0, better: null, scale: 100, showStd: false }
], {
  caption: 'Physical consistency of the simulator under the SGT gait (Level 1, 20\\,s).',
  label: 'tab:physics',
  wide: true,
  paper: true,
  firstHeader: 'Physics core',
  note: '$\\varepsilon=E-E_0-\\sum W$: energy that no booked work explains \\emph{after} correction (held at solver tolerance by the energy row). $\\delta_k$: the same quantity for step $k$ \\emph{before} correction, i.e.\\ the per-step bound on any unbooked energy source. $|W_{\\mathrm{num}}|$: energy changed by constraint projections and by the corrector (booked, but numerical). $^\\dagger$Original engine with its locomotion aids and support-face gait.'
});

// ---------------------------------------------------------------- Table III stress
const stressRows = [];
for (const scenario of ['nominal', 'impulse', 'push', 'gaussian', 'laplace', 'outlier', 'stiffness', 'friction', 'payload', 'delay']) {
  const of = method => scenario === 'nominal'
    ? nominal.filter(r => r.spec.method === method && r.spec.seed <= 3)
    : pick(r => r.group === 'stress' && r.spec.scenario === scenario && r.spec.method === method);
  const sgt = of('sgt'), heu = of('heuristic');
  if (!sgt.length) continue;
  stressRows.push({
    label: ({ nominal: 'Nominal', impulse: 'Lateral impulse 0.5\\,m/s', push: 'Push 10\\,N for 2\\,s', gaussian: 'Gaussian',
      laplace: 'Laplace', outlier: 'Outliers (2\\,\\%, $10\\times$)', stiffness: 'Cable stiffness $-20$\\,\\%',
      friction: 'Friction $-40$\\,\\%', payload: 'Payload mass $+30$\\,\\%', delay: 'Delay 100\\,ms' })[scenario] || SCENARIOS[scenario].label,
    scenario, n: sgt.length,
    cells: {
      sgtSpeed: meanStd(sgt.map(r => r.metrics.speed)),
      heuSpeed: meanStd(heu.map(r => r.metrics.speed)),
      sgtRmse: meanStd(sgt.map(r => r.metrics.rmse)),
      sgtSuccess: meanStd(sgt.map(r => r.metrics.success)),
      sgtRecovery: meanStd(sgt.map(r => r.metrics.recovery))
    }
  });
}
summary.stress = stressRows;
const stressGroups = [];
{
  const at = name => stressRows.findIndex(r => r.scenario === name);
  for (const [name, label] of [['nominal', 'Reference'], ['impulse', 'Disturbance'], ['gaussian', 'Sensor noise'], ['stiffness', 'Plant mismatch'], ['delay', 'Actuation']]) {
    const index = at(name);
    if (index >= 0 && !stressGroups.some(g => g.at === index)) stressGroups.push({ at: index, label });
  }
}
const tableIII = latexTable(stressRows, [
  { key: 'sgtSpeed', label: 'SGT $\\bar v$\\,$\\uparrow$ (m/s)', digits: 3, better: null },
  { key: 'heuSpeed', label: 'Heur.\\ $\\bar v$ (m/s)', digits: 3, better: null },
  { key: 'sgtRmse', label: 'SGT RMSE\\,$\\downarrow$ (m)', digits: 3, better: null },
  { key: 'sgtRecovery', label: '$t_{\\mathrm{rec}}$\\,$\\downarrow$ (s)', digits: 2, better: null },
  { key: 'sgtSuccess', label: 'Success\\,$\\uparrow$ (\\%)', digits: 0, better: null, scale: 100, showStd: false }
], {
  caption: 'Robustness of SGT (Level 1, 20\\,s, 3 seeds).',
  label: 'tab:stress',
  paper: true,
  firstHeader: 'Scenario',
  groups: stressGroups,
  note: 'Heur.: support-face heuristic under the same scenario. $t_{\\mathrm{rec}}$: time to regain the nominal heading after the disturbance.'
});

// ---------------------------------------------------------------- Table IV ablation
const ablationRows = Object.entries(ABLATIONS).map(([key, def]) => {
  const list = key === 'full' ? sgtFull.filter(r => r.spec.seed <= 3) : ablation(key);
  return list.length ? { label: def.label, n: list.length, cells: cells(list, KEYS) } : null;
}).filter(Boolean);
summary.ablation = ablationRows;
const tableIV = latexTable(ablationRows, [
  { key: 'speed', label: 'Speed $\\bar v$\\,$\\uparrow$ (m/s)', digits: 3, better: 'max' },
  { key: 'tips', label: 'Tips\\,$\\uparrow$', digits: 1, better: 'max' },
  { key: 'rmse', label: 'RMSE\\,$\\downarrow$ (m)', digits: 3, better: 'min' },
  { key: 'cot', label: 'CoT\\,$\\downarrow$ (--)', digits: 1, better: 'min' },
  { key: 'success', label: 'Success\\,$\\uparrow$ (\\%)', digits: 0, better: 'max', scale: 100, showStd: false }
], {
  caption: 'Ablation of SGT and of the physics core (Level 1, 20\\,s, 3 seeds).',
  label: 'tab:ablation',
  paper: true,
  firstHeader: 'Variant',
  groups: [{ at: 0, label: 'Proposed' }, { at: 1, label: 'Gait ingredients' }, { at: 5, label: 'Physics core' }]
});

// ---------------------------------------------------------------- Table V generalization
const genRows = [];
for (const level of [1, 4, 7]) {
  for (const method of ['sgt', 'heuristic']) {
    const list = level === 1 ? nominal.filter(r => r.spec.method === method && r.spec.seed <= 3)
      : pick(r => r.group === 'generalization' && r.spec.level === level && r.spec.method === method);
    if (list.length) genRows.push({ label: `Level ${level}: ${METHODS[method].short}`, n: list.length, cells: cells(list, KEYS) });
  }
}
summary.generalization = genRows;
const tableV = latexTable(genRows.map(r => ({ ...r, label: r.label.replace(/^Level \d+: /, '') })), [
  { key: 'speed', label: 'Speed $\\bar v$\\,$\\uparrow$ (m/s)', digits: 3, better: null },
  { key: 'rmse', label: 'RMSE\\,$\\downarrow$ (m)', digits: 3, better: null },
  { key: 'success', label: 'Success\\,$\\uparrow$ (\\%)', digits: 0, better: null, scale: 100, showStd: false },
  { key: 'maxResidual', label: 'max $|\\varepsilon|$ (J)', digits: 1, better: null, sci: true }
], {
  caption: 'Terrain generalization (3 seeds per cell).',
  label: 'tab:terrain',
  paper: true,
  firstHeader: 'Method',
  groups: [1, 4, 7].map((level, i) => ({ at: 2*i, label: `Level ${level}${level === 1 ? ' (rolling plain)' : level === 4 ? ' (rocky)' : ' (rugged relief)'}` }))
});

writeFileSync(path.join(outDir, 'tables.tex'), [tableI, tableII, tableIII, tableIV, tableV].join('\n\n')+'\n');
// One file per table, so the paper can place each one where it is discussed.
mkdirSync(path.join(outDir, 'tables'), { recursive: true });
for (const [name, table] of [['nominal', tableI], ['physics', tableII], ['stress', tableIII], ['ablation', tableIV], ['terrain', tableV]]) {
  writeFileSync(path.join(outDir, 'tables', `${name}.tex`), table+'\n');
}
writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, (k, v) => (k === 'runs' ? undefined : v), 2));

// ---------------------------------------------------------------- figure data
const band = (list, field) => {
  if (!list.length) return null;
  const length = Math.min(...list.map(r => r.series.t.length));
  const t = list[0].series.t.slice(0, length);
  const mean = [], std = [];
  for (let i = 0; i < length; i++) {
    const s = meanStd(list.map(r => (field === 'absE' ? Math.abs(r.series.e[i]) : r.series[field][i])));
    mean.push(s.mean); std.push(s.std);
  }
  return { t, mean, std };
};
const figureData = { nominal: {}, physics: {}, stress: summary.stress, ablation: summary.ablation, perRun: {} };
for (const m of methodOrder) {
  const list = nominal.filter(r => r.spec.method === m);
  if (!list.length) continue;
  figureData.nominal[m] = { label: METHODS[m].short, proposed: Boolean(METHODS[m].proposed), reference: Boolean(METHODS[m].reference),
    y: band(list, 'y'), absE: band(list, 'absE'), u: band(list, 'u'), residual: band(list, 'residual'), barError: band(list, 'barError') };
}
for (const [key, list] of [['dae', sgtFull], ['daeNoEnergy', ablation('noEnergyRow')], ['legacy', ablation('legacyCore')]]) {
  if (list.length) figureData.physics[key] = { residual: band(list, 'residual'), barError: band(list, 'barError') };
}
const sgtSeed1 = sgtFull.find(r => r.spec.seed === 1);
if (sgtSeed1) figureData.perRun.sgtSeed1 = sgtSeed1.series;
writeFileSync(path.join(outDir, 'figure_data.json'), JSON.stringify(figureData));
console.log('[summary] wrote tables.tex, summary.json, figure_data.json');
