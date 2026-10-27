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
 *   node tools/run_hybrid_training.mjs [--outer-rounds 3] [--inner-episodes 500]
 *        [--episode-duration 5] [--min-inner-steps 100000] [--workers N] [--candidates 8]
 *        [--eval-duration 10] [--matrix quick|full] [--seeds 1]
 *        [--llm auto|anthropic|openai|compatible|offline] [--no-tuner]
 *        [--checkpoint-every 25] [--resume] [--out results/hybrid]
 *   (--epochs / --train-episodes / --train-duration are accepted as aliases)
 *
 * Two nested loops:
 *
 *   INNER LOOP (fast learning, >= --min-inner-steps physics steps per round):
 *     --inner-episodes MPPI-guided episodes on the shared-memory worker pool,
 *     domain-randomised over levels 1-13 (solid obstacles on) and the stress
 *     scenarios. Every 0.5 s plan: K rollouts at 500 Hz DAE / 20 Hz control,
 *     then actor distillation + twin-critic TD updates. The MPC weights and the
 *     reward are FIXED inside a round. Progress is checkpointed every
 *     --checkpoint-every episodes so a long round can be resumed.
 *
 *   OUTER LOOP (meta-control, once per round = every 500-1000 episodes):
 *     stress-matrix evaluation of the actor, aggregated failure modes of the
 *     round's episodes (cable slack, rod residual, slip, overload, aborts,
 *     success), then the LLM (or the rule tuner) updates the MPC weights,
 *     gait timing and reward weights; Pareto archive + automatic rollback.
 *
 * The best checkpoint is written to js/residualMppiWeights.js (or
 * js/residualMppiWeightsRl.js with --no-tuner) and the Comparative benchmark
 * table to results/hybrid/.
 */
import os from 'node:os';
import path from 'node:path';
import { writeFileSync, readFileSync, existsSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { SharedVecTensegrityEnv } from '../js/parallelPool.js';
import { MorphologyActorCritic, synergyMatrix, DEFAULT_PARAMS } from '../js/residualMppi.js';
import { ResidualMppiTrainer, evaluateSpecs, aggregateDigests, stressMatrix, STRESS_SCENARIOS } from '../js/mppiPlanner.js';
import { LlmMpcCoDesigner, DEFAULT_MPC, DEFAULT_REWARD, toParams, promptRow, loadDotEnv, TERRAIN_LEVELS, compositeScore, safetyReport, SAFETY } from '../js/llmMpcOptimizer.js';
import { SimConfig, SphericalRoverModel } from '../js/simEngine.js?v=20261027-ob1';
import { multiTerrainSpec, freshInner, accumulateInner, innerSummary } from '../js/trainingLoop.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const opt = (name, fallback) => { const i = args.indexOf(`--${name}`); return i >= 0 && args[i+1] && !args[i+1].startsWith('--') ? args[i+1] : fallback; };
const flag = name => args.includes(`--${name}`);

const epochs = Number(opt('outer-rounds', opt('epochs', 3)));
const workers = Math.max(2, Number(opt('workers', Math.min(os.cpus().length, 9))));
const K = Number(opt('candidates', 8));
const trainEpisodes = Number(opt('inner-episodes', opt('train-episodes', 500)));
const trainDuration = Number(opt('episode-duration', opt('train-duration', 5)));
const minInnerSteps = Number(opt('min-inner-steps', 100000));
const checkpointEvery = Number(opt('checkpoint-every', 25));
const PHYSICS_HZ = 500;
const evalDuration = Number(opt('eval-duration', 10));
const matrixKind = opt('matrix', 'quick');
const seeds = opt('seeds', '1').split(',').map(Number);
// --no-tuner: MPC + RL only. The MPC/reward parameters stay at their defaults (no LLM or
// rule-based co-designer, no rollback-driven parameter changes); writes the separate
// checkpoint js/residualMppiWeightsRl.js used by controllerMode 'residual_mppi'.
const noTuner = flag('no-tuner');
const outDir = path.resolve(root, opt('out', noTuner ? 'results/hybrid_rl' : 'results/hybrid'));
// Only the default output folders publish to the UI and overwrite the deployed weights;
// any other --out (smoke tests, sweeps) keeps everything inside that folder.
const defaultOut = !args.includes('--out');
const publicDir = defaultOut ? path.join(root, 'public', noTuner ? 'hybrid_rl' : 'hybrid') : path.join(outDir, 'public');
const weightsFile = noTuner ? 'residualMppiWeightsRl.js' : 'residualMppiWeights.js';
const weightsExport = noTuner ? 'RESIDUAL_MPPI_WEIGHTS_RL' : 'RESIDUAL_MPPI_WEIGHTS';
mkdirSync(outDir, { recursive: true });
const log = (...a) => console.log('[hybrid]', ...a);

// Held-out evaluation on ALL 14 terrains ('quick': 4 disturbances, 'full': all 9).
const matrix = matrixKind === 'full'
  ? stressMatrix({ levels: TERRAIN_LEVELS, seeds })
  : stressMatrix({ levels: TERRAIN_LEVELS, scenarios: ['nominal', 'push', 'friction', 'delay'], seeds });
// Inner loop: every block of 14 episodes covers all 14 terrains (js/trainingLoop.js multiTerrainSpec),
// scenario and seed randomised; one shared terrain- and MPC-conditioned policy.

const cfg0 = new SimConfig({});
const rover = new SphericalRoverModel(cfg0);
const synergy = synergyMatrix(rover);
const model = new MorphologyActorCritic(synergy, 1);
await loadDotEnv(path.join(root, '.env'));            // VITE_LLM_API_URL / _KEY / _MODEL (never printed)
const designer = new LlmMpcCoDesigner({ provider: noTuner ? 'offline' : opt('llm', 'auto') });
log(`workers ${workers}, candidates ${K}, 14 terrains, eval matrix ${matrixKind} (${matrix.length} runs), tuner: ${noTuner ? 'off (MPC + RL only)' : designer.provider}${designer.provider !== 'offline' ? ` (${designer.model})` : ''}`);

// Per-terrain configuration tables: [14 x MPC/gait keys] and [14 x reward keys].
const perLevel = value => Object.fromEntries(TERRAIN_LEVELS.map(L => [L, structuredClone(value)]));
let mpcBy = perLevel(DEFAULT_MPC), rewardBy = perLevel(DEFAULT_REWARD);
const levelBest = {};                                 // per-terrain best { composite, completion, mpc, reward }
let epochStart = 0, episodeStart = 0, innerResume = null, innerByLevelResume = null;
const statePath = path.join(outDir, 'state.json');
if (flag('resume') && existsSync(statePath)) {
  const st = JSON.parse(readFileSync(statePath, 'utf8'));
  model.load(st.model);
  mpcBy = st.mpcBy || perLevel(st.mpc || DEFAULT_MPC); rewardBy = st.rewardBy || perLevel(st.reward || DEFAULT_REWARD);
  Object.assign(levelBest, st.levelBest || {});
  epochStart = st.nextEpoch; episodeStart = st.nextEpisode || 0; innerResume = st.inner || null; innerByLevelResume = st.innerByLevel || null;
  for (const e of st.history) designer.record({ epoch: e.epoch, mpc: e.mpc, reward: e.reward, digests: e.digests, extra: e.extra });
  log(`resumed at round ${epochStart}, episode ${episodeStart}`);
}

const pool = new SharedVecTensegrityEnv({ numEnvs: workers, numCandidates: K, baseUrl: new URL('../js/parallelPool.js', import.meta.url).href });
await pool.start();
const t0 = Date.now();
const minutes = () => ((Date.now()-t0)/60000).toFixed(1);

const paramsOf = L => toParams(mpcBy[L], rewardBy[L], DEFAULT_PARAMS);
const evaluate = async (variant = 'residual') => {
  const specs = matrix.map(s => ({ ...s, variant }));
  return aggregateDigests(await evaluateSpecs(pool, model, specs, evalDuration, spec => paramsOf(spec.level)));
};
const rowsOf = (digests, L) => digests.filter(d => d.level === L);

const history = [];
const snapshotModel = () => ({ actor: Array.from(model.actor.w), critic: model.critic.toJSON() });
const restoreModel = extra => { model.actor.w.set(extra.actor); model.critic.load(extra.critic); };

// Baselines on the same 14-terrain matrix (plain SGT and SGT under the 0.120 m clamp).
const baselines = {};
for (const variant of ['sgt', 'sgt120']) { baselines[variant] = await evaluate(variant); log(`baseline ${variant} done (${minutes()} min)`); }
writeFileSync(path.join(outDir, 'baselines.json'), JSON.stringify(baselines, null, 1));

const updateLevelBest = digests => {
  for (const L of TERRAIN_LEVELS) {
    const rows = rowsOf(digests, L);
    if (!rows.length) continue;
    const composite = compositeScore(rows), safety = safetyReport(rows);
    if (safety.overload) continue;
    if (!levelBest[L] || composite > levelBest[L].composite) levelBest[L] = { composite, completion: safety.completion, mpc: structuredClone(mpcBy[L]), reward: structuredClone(rewardBy[L]) };
  }
};

if (epochStart === 0) {
  const digests = await evaluate('residual');
  const entry = designer.record({ epoch: 0, mpc: mpcBy, reward: rewardBy, digests, extra: snapshotModel() });
  history.push(entry);
  updateLevelBest(digests);
  log(`round 0 (untrained actor = SGT-0.120, 14 terrains): composite ${entry.composite.toFixed(2)}, completion ${entry.safety.completion.toFixed(0)}%, max outer ${entry.safety.maxOuterTensionN.toFixed(0)} N`);
}

const trainer = new ResidualMppiTrainer(pool, model, { horizon: Math.round(mpcBy[1].horizonSteps), seed: 100+epochStart });
const saveState = (nextEpoch, nextEpisode, inner, innerByLevel) => writeFileSync(statePath, JSON.stringify({
  nextEpoch, nextEpisode, inner, innerByLevel, mpcBy, rewardBy, levelBest, model: model.toJSON(),
  history: designer.history.map(e => ({ epoch: e.epoch, mpc: e.mpc, reward: e.reward, digests: e.digests, extra: e.extra }))
}));

for (let epoch = Math.max(1, epochStart); epoch <= epochs; epoch++) {
  // ---------------- INNER LOOP: all 14 terrains, each with its own fixed configuration this round
  const inner = epoch === epochStart && innerResume ? innerResume : freshInner();
  const innerByLevel = epoch === epochStart && innerByLevelResume ? innerByLevelResume : perLevel(freshInner());
  const firstEpisode = epoch === epochStart ? episodeStart : 0;
  const roundStart = Date.now();
  for (let e = firstEpisode; e < trainEpisodes || inner.realSteps < minInnerSteps; e++) {
    const spec = multiTerrainSpec(epoch, e);
    model.params = paramsOf(spec.level);                   // terrain k runs with its own MPC / gait / reward
    trainer.opts.horizon = Math.round(mpcBy[spec.level].horizonSteps);
    const r = await trainer.runEpisode(spec, trainDuration);
    accumulateInner(inner, spec, r, { candidates: K, durationS: trainDuration });
    accumulateInner(innerByLevel[spec.level], spec, r, { candidates: K, durationS: trainDuration });
    if ((e+1) % checkpointEvery === 0 || e+1 === trainEpisodes) {
      const rate = (inner.realSteps+inner.rolloutSteps)/Math.max(1e-3, (Date.now()-roundStart)/1000);
      log(`round ${epoch} episode ${e+1}/${trainEpisodes}: real steps ${inner.realSteps.toLocaleString()}, rollout steps ${inner.rolloutSteps.toLocaleString()} (${Math.round(rate).toLocaleString()} steps/s), success ${(100*inner.success/inner.episodes).toFixed(0)}%, mean progress ${(inner.progress/inner.episodes).toFixed(2)} m, slack ${(inner.slack/inner.episodes).toFixed(1)}%, max rod ${inner.rodMax.toFixed(4)} mm, ESS ${(inner.ess/inner.episodes).toFixed(1)} (${minutes()} min)`);
      saveState(epoch, e+1, inner, innerByLevel);
    }
  }
  const innerStats = innerSummary(inner);

  // ---------------- OUTER LOOP: 14-terrain evaluation, global + per-terrain rollback, ONE batched meta-update
  const digests = await evaluate('residual');
  const entry = designer.record({ epoch, mpc: structuredClone(mpcBy), reward: structuredClone(rewardBy), digests, extra: snapshotModel() });
  entry.train = innerStats;
  history.push(entry);
  const decision = designer.decide(entry);
  log(`round ${epoch} (${inner.episodes} episodes, ${(inner.realSteps+inner.rolloutSteps).toLocaleString()} physics steps): composite ${entry.composite.toFixed(2)}, completion ${entry.safety.completion.toFixed(0)}%, max outer ${entry.safety.maxOuterTensionN.toFixed(0)} N, Pareto ${entry.onFront ? 'front' : 'dominated'} -> ${decision.rollback ? `ROLLBACK (${decision.reason}) to round ${decision.to.epoch}` : 'keep'}`);
  let evalDigests = digests;
  const reverted = [];
  if (decision.rollback) {
    restoreModel(decision.to.extra);
    mpcBy = structuredClone(decision.to.mpc); rewardBy = structuredClone(decision.to.reward);
    evalDigests = decision.to.digests;
  } else {
    // Per-terrain rollback: a terrain whose own score, completion or tension regressed gets its best configuration back.
    for (const L of TERRAIN_LEVELS) {
      const rows = rowsOf(digests, L), best = levelBest[L];
      if (!rows.length || !best) continue;
      const composite = compositeScore(rows), safety = safetyReport(rows);
      if (safety.overload || safety.completion < best.completion-SAFETY.completionDropPts || composite < best.composite-0.1*Math.abs(best.composite)) {
        mpcBy[L] = structuredClone(best.mpc); rewardBy[L] = structuredClone(best.reward); reverted.push(L);
      }
    }
    updateLevelBest(digests);
    if (reverted.length) log(`  per-terrain rollback: levels ${reverted.join(', ')} restored to their best configuration`);
  }
  let proposalLog = { source: 'none', insight: 'tuner off: per-terrain parameters held at defaults' };
  if (!noTuner) {
    const byLevel = Object.fromEntries(TERRAIN_LEVELS.map(L => [L, { mpc: mpcBy[L], reward: rewardBy[L], digests: rowsOf(evalDigests, L), inner: innerSummary(innerByLevel[L]) }]));
    const proposal = await designer.proposeAll(byLevel, innerStats);
    for (const L of TERRAIN_LEVELS) { mpcBy[L] = proposal.byLevel[L].updatedMpcConfig; rewardBy[L] = proposal.byLevel[L].updatedRewardWeights; }
    proposalLog = { source: proposal.source, insight: proposal.insight, rejected: proposal.rejected,
      perTerrain: Object.fromEntries(TERRAIN_LEVELS.map(L => [L, proposal.byLevel[L].diagnosticSummary])) };
    log(`  meta-controller (${proposal.source}, one call for 14 terrains): ${(proposal.insight || proposal.byLevel[1].diagnosticSummary).slice(0, 300)}`);
  }
  entry.proposal = { ...proposalLog, rolledBack: decision.rollback, reason: decision.reason, revertedLevels: reverted };
  writeFileSync(path.join(outDir, `epoch_${String(epoch).padStart(2, '0')}.json`), JSON.stringify({ epoch, mpcBy: entry.mpc, rewardBy: entry.reward, digests: entry.digests, composite: entry.composite, safety: entry.safety, inner: innerStats, innerByLevel: Object.fromEntries(TERRAIN_LEVELS.map(L => [L, innerSummary(innerByLevel[L])])), proposal: entry.proposal }, null, 1));
  saveState(epoch+1, 0, null, null);
}

// ------------------------------------------------------------------ results
const best = designer.archive.best();
restoreModel(best.extra);
const paramsByLevel = Object.fromEntries(TERRAIN_LEVELS.map(L => [L, toParams(best.mpc[L], best.reward[L], DEFAULT_PARAMS)]));
model.params = paramsByLevel[1];
writeFileSync(defaultOut ? path.join(root, 'js', weightsFile) : path.join(outDir, weightsFile),
  `/** Trained residual-MPPI actor (${noTuner ? 'MPC + RL, fixed parameters' : 'MPC + RL + LLM co-designer'}, 14-terrain conditioned; written by tools/run_hybrid_training.mjs). null = untrained: zero residual, i.e. plain SGT. */\n` +
  `export const ${weightsExport} = ${JSON.stringify({ actor: model.actor.toJSON(), params: paramsByLevel[1], paramsByLevel,
    meta: { trainedAt: new Date().toISOString(), tuner: noTuner ? 'off' : designer.provider, rounds: epochs, episodesPerRound: trainEpisodes, bestRound: best.epoch, composite: best.composite, llmProvider: designer.provider, matrix: matrixKind, evalDuration, terrains: 14 } })};\n`);
const summary = {
  bestEpoch: best.epoch, tuner: noTuner ? 'off' : 'on', provider: noTuner ? 'none' : designer.provider, model: designer.provider === 'offline' ? null : designer.model,
  terrains: 14,
  history: designer.history.map(e => ({ epoch: e.epoch, composite: e.composite, completion: e.safety.completion, maxOuterTensionN: e.safety.maxOuterTensionN, onFront: e.onFront })),
  front: designer.archive.entries.map(e => e.epoch), mpcByLevel: best.mpc, rewardByLevel: best.reward,
  final: best.digests.map(promptRow), baselines: Object.fromEntries(Object.entries(baselines).map(([k, v]) => [k, v.map(promptRow)])),
  llmError: designer.lastError, wallMinutes: Number(minutes())
};
writeFileSync(path.join(outDir, 'summary.json'), JSON.stringify(summary, null, 1));
writeFileSync(path.join(outDir, 'comparative_benchmark.md'), benchmarkMarkdown(best.digests, baselines));
// The UI (Comparative benchmark tab > Hybrid training) reads these two files.
mkdirSync(publicDir, { recursive: true });
writeFileSync(path.join(publicDir, 'summary.json'), JSON.stringify(summary));
writeFileSync(path.join(publicDir, 'comparative_benchmark.md'), benchmarkMarkdown(best.digests, baselines));
log(`best round ${best.epoch}; wrote ${defaultOut ? `js/${weightsFile}` : path.relative(root, path.join(outDir, weightsFile))} and ${path.relative(root, outDir)}/ (${minutes()} min)`);
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
