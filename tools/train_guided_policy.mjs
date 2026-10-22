#!/usr/bin/env node
/**
 * MPC-guided policy search for the six-bar rover (cable-only, Lagrangian DAE).
 *
 *   node tools/train_guided_policy.mjs [--iterations 4] [--episodes 4] [--duration 15] [--workers 2]
 *
 * Iteration 0 rolls out the MPC teacher (λ = 0) and fits the policy by
 * regression on its actions. Every later iteration
 *   C-step: rolls out the teacher regularized toward the current policy
 *           (λ_i > 0), plus one episode that executes the policy itself
 *           (DAgger-style coverage of the learner's own state distribution);
 *           every visited state is labelled with the unregularized teacher;
 *   S-step: retrains the policy on the aggregated data set.
 * Finally the pure policy is evaluated on held-out seeds and headings against
 * SGT and the teacher. Writes js/guidedPolicyWeights.js and
 * results/guided_policy/{training,evaluation}.json.
 */
import { spawn } from 'node:child_process';
import { writeFileSync, mkdirSync, readFileSync, existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = fileURLToPath(import.meta.url);
const root = path.resolve(path.dirname(here), '..');
const arg = (name, fallback) => {
  const i = process.argv.indexOf(`--${name}`);
  return i > 0 ? process.argv[i+1] : fallback;
};

// ------------------------------------------------------------------ one episode (child process)
async function runEpisode(spec) {
  console.info = () => {};
  const { SimConfig, TerrainModel, SphericalRoverModel, Simulation } = await import('../js/simEngine.js');
  const { METHODS } = await import('../js/experimentProtocol.js');
  const { configureLevel } = await import('../js/levels.js');
  const { GuidedPolicy, actuatorFeatures } = await import('../js/guidedPolicySearch.js');
  const cfg = new SimConfig({});
  configureLevel(cfg, spec.level);
  METHODS.sgt.configure(cfg);
  cfg.seed = spec.seed;
  cfg.targetDestination = spec.target;
  cfg.targetGoalY = spec.target[1];
  const samples = { outerX: [], outerY: [], coreX: [], coreY: [] };
  if (spec.mode !== 'sgt') {
    cfg.controllerMode = 'guided_policy';
    const policy = spec.weights ? new GuidedPolicy(spec.weights) : null;
    cfg.guidedPolicy = {
      policy, usePolicy: Boolean(policy),
      teacher: spec.mode === 'policy' && !spec.label ? false : { lambda: spec.lambda || 0 },
      executePolicy: spec.mode === 'policy',
      edgeSelection: spec.edgeSelection || 'feasibility',
      // Record (features, unregularized-teacher label) for every visited state.
      onSample: spec.collect ? (ctx, label, pi) => {
        const features = pi?.features || actuatorFeatures(ctx);
        features.outer.forEach((x, k) => { samples.outerX.push(x); samples.outerY.push([label[k]]); });
        features.core.forEach((x, k) => { samples.coreX.push(x); samples.coreY.push([label[24+k]]); });
      } : null
    };
  }
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
  const start = sim.q.reduce((s, p) => [s[0]+p[0]/12, s[1]+p[1]/12], [0, 0]);
  const t0 = Date.now();
  while (sim.t < spec.duration) sim.step();
  const end = sim.q.reduce((s, p) => [s[0]+p[0]/12, s[1]+p[1]/12], [0, 0]);
  const goal = spec.target;
  const toGoal = [goal[0]-start[0], goal[1]-start[1]];
  const len = Math.hypot(...toGoal) || 1;
  const disp = [end[0]-start[0], end[1]-start[1]];
  const progress = (disp[0]*toGoal[0]+disp[1]*toGoal[1])/len;
  const crossTrack = Math.abs(disp[0]*toGoal[1]-disp[1]*toGoal[0])/len;
  return {
    spec: { ...spec, weights: undefined },
    metrics: { progress, crossTrack, tips: sim.sgt?.tipCount ?? 0, aborts: sim.sgt?.abortCount ?? 0,
      controllerMs: sim.controllerTiming.totalMs/Math.max(1, sim.controllerTiming.count), wallSeconds: (Date.now()-t0)/1000 },
    samples: spec.collect ? samples : null
  };
}

if (process.argv.includes('--episode')) {
  const spec = JSON.parse(readFileSync(arg('episode'), 'utf8'));
  const result = await runEpisode(spec);
  writeFileSync(arg('out'), JSON.stringify(result));
  process.exit(0);
}

// ------------------------------------------------------------------ driver
const tmp = path.join(root, 'results', 'guided_policy', 'tmp');
mkdirSync(tmp, { recursive: true });
let counter = 0;
function episode(spec) {
  const id = `${process.pid}_${counter++}`;
  const specFile = path.join(tmp, `spec_${id}.json`), outFile = path.join(tmp, `out_${id}.json`);
  writeFileSync(specFile, JSON.stringify(spec));
  return new Promise((resolve, reject) => {
    const child = spawn(process.execPath, [here, '--episode', specFile, '--out', outFile], { stdio: ['ignore', 'ignore', 'inherit'] });
    child.on('exit', code => (code === 0 ? resolve(JSON.parse(readFileSync(outFile, 'utf8'))) : reject(new Error(`episode exited ${code}`))));
  });
}
async function runAll(specs, workers) {
  const results = new Array(specs.length);
  let next = 0;
  await Promise.all(Array.from({ length: workers }, async () => {
    while (next < specs.length) { const i = next++; results[i] = await episode(specs[i]); }
  }));
  return results;
}

if (process.argv.includes('--evaluate-only')) {
  const { GUIDED_POLICY_WEIGHTS } = await import('../js/guidedPolicyWeights.js');
  const conds = [
    { level: 1, target: [0, 25], seed: 101 }, { level: 1, target: [6, 25], seed: 102 },
    { level: 3, target: [0, 25], seed: 103 }, { level: 3, target: [-6, 25], seed: 104 }
  ];
  const variants = (arg('variants', 'feasibility,alignment')).split(',');
  const specs = conds.flatMap(c => variants.map(v => ({ ...c, duration: 20, mode: 'policy', label: false, weights: GUIDED_POLICY_WEIGHTS, edgeSelection: v })));
  const rows = (await runAll(specs, Number(arg('workers', 2)))).map(r => ({ variant: r.spec.edgeSelection, level: r.spec.level, target: r.spec.target, ...r.metrics }));
  for (const r of rows) console.log(r.variant.padEnd(12), 'L', r.level, JSON.stringify(r.target).padEnd(8), 'Δ', r.progress.toFixed(2), 'xt', r.crossTrack.toFixed(2), 'tips', r.tips, 'aborts', r.aborts, 'ms', r.controllerMs.toFixed(2));
  for (const v of variants) {
    const sel = rows.filter(r => r.variant === v);
    console.log('mean', v, 'Δ', (sel.reduce((s, r) => s+r.progress, 0)/sel.length).toFixed(3), 'aborts', (sel.reduce((s, r) => s+r.aborts, 0)/sel.length).toFixed(2));
  }
  writeFileSync(path.join(root, 'results', 'guided_policy', 'evaluation_edge_selection.json'), JSON.stringify(rows, null, 1));
  process.exit(0);
}
const iterations = Number(arg('iterations', 4));
const perIteration = Number(arg('episodes', 4));
const duration = Number(arg('duration', 15));
const workers = Number(arg('workers', 2));
const daggerEpisodes = Number(arg('dagger', 1));
const hidden = Number(arg('hidden', 32));
const { GuidedPolicy, OUTER_FEATURES, CORE_FEATURES } = await import('../js/guidedPolicySearch.js');

const conditions = [
  { level: 1, target: [0, 25] }, { level: 1, target: [10, 25] }, { level: 1, target: [-10, 25] },
  { level: 3, target: [0, 25] }, { level: 3, target: [8, 25] }, { level: 3, target: [-8, 25] }
];
const lambdas = [0, 0.5, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0];
const data = { outerX: [], outerY: [], coreX: [], coreY: [] };
const policy = new GuidedPolicy({
  outer: { sizes: [OUTER_FEATURES, hidden, hidden, 1] }, core: { sizes: [CORE_FEATURES, Math.round(0.75*hidden), Math.round(0.75*hidden), 1] }
}, 7);
policy.trained = false;
const log = [];
for (let it = 0; it < iterations; it++) {
  const weights = it === 0 ? null : policy.toJSON();
  const specs = [];
  for (let e = 0; e < perIteration; e++) {
    const c = conditions[(it*perIteration+e) % conditions.length];
    const dagger = it > 0 && e >= perIteration-daggerEpisodes;
    specs.push({ ...c, seed: 1+it*10+e, duration, collect: true, weights,
      mode: dagger ? 'policy' : 'teacher', label: true, lambda: dagger ? 0 : lambdas[it] });
  }
  const started = Date.now();
  const results = await runAll(specs, workers);
  for (const r of results) for (const key of Object.keys(data)) data[key].push(...r.samples[key]);
  if (it === 0) { policy.outer.fitNormalization(data.outerX); policy.core.fitNormalization(data.coreX); }
  const lossOuter = policy.outer.train(data.outerX, data.outerY, { epochs: it === 0 ? 25 : 12, lr: 3e-3, seed: it+1 });
  const lossCore = policy.core.train(data.coreX, data.coreY, { epochs: it === 0 ? 25 : 12, lr: 3e-3, seed: it+11 });
  const entry = {
    iteration: it, lambda: lambdas[it], samples: data.outerY.length+data.coreY.length,
    lossOuter, lossCore, minutes: (Date.now()-started)/60000,
    episodes: results.map(r => ({ mode: r.spec.mode, level: r.spec.level, target: r.spec.target, ...r.metrics }))
  };
  log.push(entry);
  console.log(`[gps] it ${it} λ=${entry.lambda} samples ${entry.samples} mse outer ${lossOuter.toFixed(4)} core ${lossCore.toFixed(4)} · `+
    entry.episodes.map(e => `${e.mode}:Δ${e.progress.toFixed(2)}m`).join(' ')+` · ${entry.minutes.toFixed(1)} min`);
}

// held-out evaluation
const weights = policy.toJSON();
weights.meta = { trainedAt: new Date().toISOString(), iterations, episodesPerIteration: perIteration, duration,
  samples: data.outerY.length+data.coreY.length };
const evalConditions = [
  { level: 1, target: [0, 25], seed: 101 }, { level: 1, target: [6, 25], seed: 102 },
  { level: 3, target: [0, 25], seed: 103 }, { level: 3, target: [-6, 25], seed: 104 }
];
const evalSpecs = [];
for (const c of evalConditions) {
  evalSpecs.push({ ...c, duration: 20, mode: 'sgt' });
  evalSpecs.push({ ...c, duration: 20, mode: 'teacher', lambda: 0 });
  evalSpecs.push({ ...c, duration: 20, mode: 'policy', label: false, weights });
}
const evaluation = (await runAll(evalSpecs, workers)).map(r => ({ mode: r.spec.mode, level: r.spec.level, target: r.spec.target, seed: r.spec.seed, ...r.metrics }));
const summary = {};
for (const mode of ['sgt', 'teacher', 'policy']) {
  const rows = evaluation.filter(r => r.mode === mode);
  const mean = key => rows.reduce((s, r) => s+r[key], 0)/rows.length;
  summary[mode] = { progress: mean('progress'), crossTrack: mean('crossTrack'), tips: mean('tips'), aborts: mean('aborts'), controllerMs: mean('controllerMs') };
}
console.log('[gps] held-out (20 s, 4 conditions):', JSON.stringify(summary, (k, v) => (typeof v === 'number' ? +v.toFixed(3) : v)));
weights.meta.evaluation = summary;

writeFileSync(path.join(root, 'js', 'guidedPolicyWeights.js'),
  `/** Trained guided-policy-search weights (written by tools/train_guided_policy.mjs). */\nexport const GUIDED_POLICY_WEIGHTS = ${JSON.stringify(weights)};\n`);
writeFileSync(path.join(root, 'results', 'guided_policy', 'training.json'), JSON.stringify(log, null, 1));
writeFileSync(path.join(root, 'results', 'guided_policy', 'evaluation.json'), JSON.stringify({ evaluation, summary }, null, 1));
console.log('[gps] wrote js/guidedPolicyWeights.js');
