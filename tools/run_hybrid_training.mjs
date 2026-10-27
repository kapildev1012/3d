#!/usr/bin/env node
/**
 * Three-tier hybrid training pipeline (cable-only, tension shield on, assist share 0):
 *
 *   tier 1  zero-copy SharedArrayBuffer worker pool (js/parallelPool.js), one
 *           500 Hz DAE simulation per worker thread
 *   tier 2  morphology-aware residual-MPPI + twin value critic (js/mppiPlanner.js)
 *   tier 3  closed-loop LLM MPC / reward co-designer with Pareto rollback
 *           (js/llmMpcOptimizer.js)
 *
 *   node tools/run_hybrid_training.mjs [--epochs 4] [--workers N] [--candidates 8]
 *        [--train-episodes 2] [--train-duration 10] [--eval-duration 10]
 *        [--matrix quick|full] [--seeds 1] [--llm auto|anthropic|openai|offline]
 *        [--resume] [--out results/hybrid]
 *
 * Each epoch: (a) MPPI training episodes, (b) multi-seed evaluation of the
 * actor over the stress matrix, (c) LLM / rule update of MPC and reward
 * weights, with automatic rollback to the best Pareto checkpoint. The best
 * checkpoint is written to js/residualMppiWeights.js and the Comparative
 * benchmark table to results/hybrid/.
 */
import os from 'node:os';
import path from 'node:path';
import { writeFileSync, readFileSync, existsSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { SharedVecTensegrityEnv } from '../js/parallelPool.js';
import { MorphologyActorCritic, synergyMatrix } from '../js/residualMppi.js';
import { ResidualMppiTrainer, evaluateSpecs, aggregateDigests, stressMatrix, STRESS_SCENARIOS } from '../js/mppiPlanner.js';
import { LlmMpcCoDesigner, DEFAULT_MPC, DEFAULT_REWARD, toParams, promptRow, loadDotEnv } from '../js/llmMpcOptimizer.js';
import { SimConfig, SphericalRoverModel } from '../js/simEngine.js?v=20261021-ws1';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const opt = (name, fallback) => { const i = args.indexOf(`--${name}`); return i >= 0 && args[i+1] && !args[i+1].startsWith('--') ? args[i+1] : fallback; };
const flag = name => args.includes(`--${name}`);

const epochs = Number(opt('epochs', 4));
const workers = Math.max(2, Number(opt('workers', Math.min(os.cpus().length, 9))));
const K = Number(opt('candidates', 8));
const trainEpisodes = Number(opt('train-episodes', 2));
const trainDuration = Number(opt('train-duration', 10));
const evalDuration = Number(opt('eval-duration', 10));
const matrixKind = opt('matrix', 'quick');
const seeds = opt('seeds', '1').split(',').map(Number);
// --no-tuner: MPC + RL only. The MPC/reward parameters stay at their defaults (no LLM or
// rule-based co-designer, no rollback-driven parameter changes); writes the separate
// checkpoint js/residualMppiWeightsRl.js used by controllerMode 'residual_mppi'.
const noTuner = flag('no-tuner');
const outDir = path.resolve(root, opt('out', noTuner ? 'results/hybrid_rl' : 'results/hybrid'));
const publicDir = path.join(root, 'public', noTuner ? 'hybrid_rl' : 'hybrid');
const weightsFile = noTuner ? 'residualMppiWeightsRl.js' : 'residualMppiWeights.js';
const weightsExport = noTuner ? 'RESIDUAL_MPPI_WEIGHTS_RL' : 'RESIDUAL_MPPI_WEIGHTS';
mkdirSync(outDir, { recursive: true });
const log = (...a) => console.log('[hybrid]', ...a);

const matrix = matrixKind === 'full'
  ? stressMatrix({ seeds })
  : stressMatrix({ levels: [1, 4], scenarios: ['nominal', 'push', 'friction', 'delay'], seeds });
const trainSpecs = [
  { level: 1, scenario: 'nominal', seed: 11 }, { level: 1, scenario: 'push', seed: 12 }, { level: 4, scenario: 'nominal', seed: 13 },
  { level: 1, scenario: 'payload', seed: 14 }, { level: 1, scenario: 'friction', seed: 15 }, { level: 7, scenario: 'nominal', seed: 16 }
];

const cfg0 = new SimConfig({});
const rover = new SphericalRoverModel(cfg0);
const synergy = synergyMatrix(rover);
const model = new MorphologyActorCritic(synergy, 1);
await loadDotEnv(path.join(root, '.env'));            // VITE_LLM_API_URL / _KEY / _MODEL (never printed)
const designer = new LlmMpcCoDesigner({ provider: noTuner ? 'offline' : opt('llm', 'auto') });
log(`workers ${workers}, candidates ${K}, matrix ${matrixKind} (${matrix.length} runs), tuner: ${noTuner ? 'off (MPC + RL only)' : designer.provider}${designer.provider !== 'offline' ? ` (${designer.model})` : ''}`);

let mpc = { ...DEFAULT_MPC }, reward = { ...DEFAULT_REWARD };
let epochStart = 0;
const statePath = path.join(outDir, 'state.json');
if (flag('resume') && existsSync(statePath)) {
  const st = JSON.parse(readFileSync(statePath, 'utf8'));
  model.load(st.model); mpc = st.mpc; reward = st.reward; epochStart = st.nextEpoch;
  for (const e of st.history) designer.record({ epoch: e.epoch, mpc: e.mpc, reward: e.reward, digests: e.digests, extra: e.extra });
  log(`resumed at epoch ${epochStart}`);
}

const pool = new SharedVecTensegrityEnv({ numEnvs: workers, numCandidates: K, baseUrl: new URL('../js/parallelPool.js', import.meta.url).href });
await pool.start();
const t0 = Date.now();
const minutes = () => ((Date.now()-t0)/60000).toFixed(1);

const withParams = (m, r) => { model.params = toParams(m, r, model.params); };
const evaluate = async (variant = 'residual') => {
  const specs = matrix.map(s => ({ ...s, variant }));
  return aggregateDigests(await evaluateSpecs(pool, model, specs, evalDuration));
};

const history = [];
const snapshotModel = () => ({ actor: Array.from(model.actor.w), critic: model.critic.toJSON() });
const restoreModel = extra => { model.actor.w.set(extra.actor); model.critic.load(extra.critic); };

// Baselines on the same matrix (plain SGT and SGT under the 0.120 m clamp).
withParams(mpc, reward);
const baselines = {};
for (const variant of ['sgt', 'sgt120']) { baselines[variant] = await evaluate(variant); log(`baseline ${variant} done (${minutes()} min)`); }
writeFileSync(path.join(outDir, 'baselines.json'), JSON.stringify(baselines, null, 1));

if (epochStart === 0) {
  const digests = await evaluate('residual');
  const entry = designer.record({ epoch: 0, mpc, reward, digests, extra: snapshotModel() });
  history.push(entry);
  log(`epoch 0 (untrained actor = SGT-0.120): composite ${entry.composite.toFixed(2)}, completion ${entry.safety.completion.toFixed(0)}%`);
}

for (let epoch = Math.max(1, epochStart); epoch <= epochs; epoch++) {
  withParams(mpc, reward);
  const trainer = new ResidualMppiTrainer(pool, model, { horizon: Math.round(mpc.horizonSteps), seed: 100+epoch });
  trainer.criticSteps = epoch > 1 ? 1000 : 0;
  const trainInfo = [];
  for (let e = 0; e < trainEpisodes; e++) {
    const spec = trainSpecs[((epoch-1)*trainEpisodes+e) % trainSpecs.length];
    const r = await trainer.runEpisode(spec, trainDuration, {
      onPlan: ({ cycle, cycles, plan, run }) => { if (cycle % 5 === 0) log(`  train ep ${e} cycle ${cycle+1}/${cycles} ess ${plan.ess.toFixed(1)} best-vs-nominal ${plan.advantage.toFixed(3)} (${minutes()} min)`); }
    });
    trainInfo.push({ spec, progress: r.digest.progressM, ess: r.meanEss, gain: r.meanGainOverNominal, advantage: r.meanAdvantage, synergyShare: r.synergyShare, violating: r.violatingCandidates });
    log(`epoch ${epoch} train ${spec.scenario} L${spec.level}: progress ${r.digest.progressM.toFixed(2)} m, mean ESS ${r.meanEss.toFixed(1)}, best-candidate advantage ${r.meanAdvantage.toFixed(3)}/plan`);
  }
  const digests = await evaluate('residual');
  const entry = designer.record({ epoch, mpc, reward, digests, extra: snapshotModel() });
  entry.train = trainInfo;
  history.push(entry);
  const decision = designer.decide(entry);
  log(`epoch ${epoch}: composite ${entry.composite.toFixed(2)}, completion ${entry.safety.completion.toFixed(0)}%, max outer ${entry.safety.maxOuterTensionN.toFixed(0)} N, Pareto ${entry.onFront ? 'front' : 'dominated'} -> ${decision.rollback ? `ROLLBACK (${decision.reason}) to epoch ${decision.to.epoch}` : 'keep'}`);
  let context = { mpc, reward, digests };
  if (decision.rollback) {
    restoreModel(decision.to.extra);
    mpc = structuredClone(decision.to.mpc); reward = structuredClone(decision.to.reward);
    context = { mpc, reward, digests: decision.to.digests };
  }
  if (noTuner) {
    entry.proposal = { source: 'none', summary: 'tuner off: MPC/reward parameters held at defaults', mpc, reward, rolledBack: decision.rollback, reason: decision.reason };
  } else {
    const proposal = await designer.propose(context);
    log(`  tuner (${proposal.source}): ${proposal.diagnosticSummary.slice(0, 300)}`);
    entry.proposal = { source: proposal.source, summary: proposal.diagnosticSummary, mpc: proposal.updatedMpcConfig, reward: proposal.updatedRewardWeights, rolledBack: decision.rollback, reason: decision.reason };
    mpc = proposal.updatedMpcConfig; reward = proposal.updatedRewardWeights;
  }
  writeFileSync(path.join(outDir, `epoch_${String(epoch).padStart(2, '0')}.json`), JSON.stringify({ epoch, mpc: entry.mpc, reward: entry.reward, digests: entry.digests, composite: entry.composite, safety: entry.safety, train: trainInfo, proposal: entry.proposal }, null, 1));
  writeFileSync(statePath, JSON.stringify({ nextEpoch: epoch+1, mpc, reward, model: model.toJSON(),
    history: designer.history.map(e => ({ epoch: e.epoch, mpc: e.mpc, reward: e.reward, digests: e.digests, extra: e.extra })) }));
}

// ------------------------------------------------------------------ results
const best = designer.archive.best();
restoreModel(best.extra);
const bestParams = toParams(best.mpc, best.reward, model.params);
model.params = bestParams;
writeFileSync(path.join(root, 'js', weightsFile),
  `/** Trained residual-MPPI actor (${noTuner ? 'MPC + RL, fixed parameters' : 'MPC + RL + LLM co-designer'}; written by tools/run_hybrid_training.mjs). null = untrained: zero residual, i.e. plain SGT. */\n` +
  `export const ${weightsExport} = ${JSON.stringify({ actor: model.actor.toJSON(), params: bestParams,
    meta: { trainedAt: new Date().toISOString(), tuner: noTuner ? 'off' : designer.provider, epochs, bestEpoch: best.epoch, composite: best.composite, llmProvider: designer.provider, matrix: matrixKind, evalDuration } })};\n`);
const summary = {
  bestEpoch: best.epoch, tuner: noTuner ? 'off' : 'on', provider: noTuner ? 'none' : designer.provider, model: designer.provider === 'offline' ? null : designer.model,
  history: designer.history.map(e => ({ epoch: e.epoch, composite: e.composite, completion: e.safety.completion, maxOuterTensionN: e.safety.maxOuterTensionN, onFront: e.onFront })),
  front: designer.archive.entries.map(e => e.epoch), mpc: best.mpc, reward: best.reward,
  final: best.digests.map(promptRow), baselines: Object.fromEntries(Object.entries(baselines).map(([k, v]) => [k, v.map(promptRow)])),
  llmError: designer.lastError, wallMinutes: Number(minutes())
};
writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1));
writeFileSync(path.join(outDir, 'comparative_benchmark.md'), benchmarkMarkdown(best.digests, baselines));
// The UI (Comparative benchmark tab > Hybrid training) reads these two files.
mkdirSync(publicDir, { recursive: true });
writeFileSync(path.join(publicDir, 'summary.json'), JSON.stringify(summary));
writeFileSync(path.join(publicDir, 'comparative_benchmark.md'), benchmarkMarkdown(best.digests, baselines));
log(`best epoch ${best.epoch}; wrote js/${weightsFile} and ${path.relative(root, outDir)}/ (${minutes()} min)`);
await pool.terminate();

function benchmarkMarkdown(finalRows, bases) {
  const f = (x, d = 3) => (Number.isFinite(x) ? x.toFixed(d) : '–');
  const header = '| Level | Disturbance | Controller | v̄ (m/s) | RMSE (m) | max\\|e\\| (m) | t_rec (s) | ∫uᵀu | CoT | t_solve (ms) | Success (%) | rod res. (mm) | relaxed cables (%) | slip (m/s) |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n';
  let out = '# Comparative benchmark – residual-MPPI vs SGT (cable-only, tension shield on)\n\n'+header;
  const rows = [['SGT', bases.sgt], ['SGT, 0.120 m clamp', bases.sgt120], ['Residual-MPPI actor', finalRows]];
  for (const base of finalRows) {
    for (const [name, list] of rows) {
      const d = list.find(r => r.level === base.level && r.disturbance === base.disturbance);
      if (!d) continue;
      out += `| ${d.level} | ${d.disturbance} | ${name} | ${f(d.avgSpeedMs)} | ${f(d.rmseM)} | ${f(d.maxErrorM)} | ${f(d.recoveryTimeS, 1)} | ${f(d.controlEffortUtu, 4)} | ${f(d.costOfTransport, 1)} | ${f(d.solveTimeMs, 1)} | ${f(d.successRatePct, 0)} | ${f(d.rodResidualMm, 5)} | ${f(d.relaxedCablesPct, 1)} | ${f(d.contactSlipMs, 3)} |\n`;
    }
  }
  return out;
}
process.exit(0);
