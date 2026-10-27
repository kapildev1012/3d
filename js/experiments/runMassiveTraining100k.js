/**
 * RUNMASSIVETRAINING100K.JS - long two-loop MPC + RL + LLM training in the browser.
 *
 *   (dev tools)  window.runMassiveTrainingLoop(...)  - exposed by js/app.js
 *   runMassiveTrainingLoop(null, 'You are an expert NMPC ... relaxed cables under 1% ...');
 *
 * Same schedule as tools/run_hybrid_training.mjs:
 *   inner loop  episodesPerBatch domain-randomised episodes (Levels 1-13, solid
 *               obstacles, stress scenarios) on the SharedArrayBuffer Web Worker
 *               pool: 500 Hz DAE, 20 Hz residual-MPPI, actor + twin-critic updates.
 *               MPC weights and reward are fixed inside a batch.
 *   outer loop  after every batch: held-out stress-matrix evaluation + the batch's
 *               aggregated failure modes -> LLM (OpenAI-compatible endpoint, or the
 *               offline rule tuner) -> validated, clamped update; Pareto archive and
 *               automatic rollback.
 * Every metric the LLM sees is measured; nothing is filled in.
 *
 * Checkpoints go to IndexedDB ('TensegrityRoverDB' / 'checkpoints'): the latest state
 * after every batch (key 'latest', used to resume) and a snapshot every 5,000 episodes.
 * The page must stay open and cross-origin isolated (server.mjs / vercel.json send
 * COOP/COEP). The API key is passed in memory only and never stored.
 */
import { SharedVecTensegrityEnv } from '../parallelPool.js';
import { MorphologyActorCritic, synergyMatrix, DEFAULT_PARAMS } from '../residualMppi.js';
import { ResidualMppiTrainer, evaluateSpecs, aggregateDigests, stressMatrix } from '../mppiPlanner.js';
import { LlmMpcCoDesigner, DEFAULT_MPC, DEFAULT_REWARD, toParams, TERRAIN_LEVELS, compositeScore, safetyReport, SAFETY } from '../llmMpcOptimizer.js';
import { LLM_SYSTEM_PROMPT } from '../llmSystemPrompt.js';
import { multiTerrainSpec, freshInner, accumulateInner, innerSummary, isValidIncidence } from '../trainingLoop.js';
import { SimConfig, SphericalRoverModel } from '../simEngine.js?v=20261027-ob1';

const DB = 'TensegrityRoverDB', STORE = 'checkpoints';

function openDb() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(DB, 1);
    request.onupgradeneeded = e => { const db = e.target.result; if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE, { keyPath: 'key' }); };
    request.onsuccess = e => resolve(e.target.result);
    request.onerror = () => reject(request.error);
  });
}
export async function saveCheckpoint(key, data) {
  try {
    const db = await openDb();
    await new Promise((resolve, reject) => { const tx = db.transaction(STORE, 'readwrite'); tx.objectStore(STORE).put({ key, savedAt: new Date().toISOString(), ...data }); tx.oncomplete = resolve; tx.onerror = () => reject(tx.error); });
    db.close();
    return true;
  } catch { return false; }
}
export async function loadCheckpoint(key = 'latest') {
  try {
    const db = await openDb();
    const value = await new Promise((resolve, reject) => { const r = db.transaction(STORE).objectStore(STORE).get(key); r.onsuccess = () => resolve(r.result || null); r.onerror = () => reject(r.error); });
    db.close();
    return value;
  } catch { return null; }
}

/**
 * @param {Float64Array|null} incidenceMatrix6x24 strut-end incidence; null or a placeholder -> the rover's own
 * @param {string} llmSystemPrompt task objective, prepended to the full co-designer prompt (schema + parameter map)
 * @param {number} totalTargetEpisodes
 * @param {number} episodesPerBatch inner-loop episodes between LLM meta-updates (500-1000)
 * @param {number} stepsPerEpisode 20 Hz control ticks per episode (200 = 10 s)
 * @param {object} options { workers, candidates, llm: { apiUrl, apiKey, model } | null, signal (AbortSignal),
 *                           resume (default true), evalDuration, onProgress(status) }
 */
export async function runMassiveTrainingLoop(incidenceMatrix6x24 = null, llmSystemPrompt = '', totalTargetEpisodes = 100000,
  episodesPerBatch = 500, stepsPerEpisode = 200, options = {}) {
  if (typeof crossOriginIsolated !== 'undefined' && !crossOriginIsolated) {
    throw new Error('SharedArrayBuffer needs a cross-origin-isolated page (serve with server.mjs, which sends COOP/COEP).');
  }
  const report = status => { options.onProgress?.(status); if (status.log) console.log(`[massive-training] ${status.log}`); };
  const rover = new SphericalRoverModel(new SimConfig({}));
  let incidence = synergyMatrix(rover);
  if (incidenceMatrix6x24 && isValidIncidence(incidenceMatrix6x24)) incidence = Float64Array.from(incidenceMatrix6x24);
  else if (incidenceMatrix6x24) report({ log: 'the supplied incidence matrix is not a strut-end incidence (each row needs four +1 and four -1 entries); using the rover topology instead' });

  const durationS = stepsPerEpisode / 20;
  const workers = Math.max(2, Math.min(options.workers ?? ((navigator.hardwareConcurrency || 4) - 1), 9));
  const K = options.candidates ?? 8;
  const model = new MorphologyActorCritic(incidence, 1);
  const llm = options.llm && options.llm.apiUrl ? options.llm : null;
  const designer = new LlmMpcCoDesigner({
    provider: llm ? 'compatible' : 'offline', endpoint: llm?.apiUrl, apiKey: llm?.apiKey || 'ollama', model: llm?.model || 'gemini-2.5-flash',
    systemPrompt: llmSystemPrompt ? `${llmSystemPrompt.trim()}\n\n${LLM_SYSTEM_PROMPT}` : LLM_SYSTEM_PROMPT
  });

  // All 14 terrains in every batch, each with its own MPC / gait / reward configuration.
  const perLevel = value => Object.fromEntries(TERRAIN_LEVELS.map(L => [L, structuredClone(value)]));
  let mpcBy = perLevel(DEFAULT_MPC), rewardBy = perLevel(DEFAULT_REWARD), completed = 0, batch = 0;
  const levelBest = {};
  const saved = options.resume === false ? null : await loadCheckpoint('latest');
  if (saved?.model) {
    model.load(saved.model); completed = saved.episodes; batch = saved.batch;
    mpcBy = saved.mpcBy || perLevel(saved.mpc || DEFAULT_MPC); rewardBy = saved.rewardBy || perLevel(saved.reward || DEFAULT_REWARD);
    Object.assign(levelBest, saved.levelBest || {});
    for (const e of saved.history || []) designer.record(e);
    report({ log: `resumed from IndexedDB at ${completed} episodes (batch ${batch})` });
  }

  const pool = new SharedVecTensegrityEnv({ numEnvs: workers, numCandidates: K });
  await pool.start();
  const trainer = new ResidualMppiTrainer(pool, model, { horizon: Math.round(mpcBy[1].horizonSteps), seed: 100 + batch });
  const paramsOf = L => toParams(mpcBy[L], rewardBy[L], DEFAULT_PARAMS);
  const matrix = stressMatrix({ levels: TERRAIN_LEVELS, scenarios: ['nominal', 'push'], seeds: [1] });
  const evaluate = async () => aggregateDigests(await evaluateSpecs(pool, model, matrix, options.evalDuration ?? 10, spec => paramsOf(spec.level)));
  const rowsOf = (digests, L) => digests.filter(d => d.level === L);
  const updateLevelBest = digests => {
    for (const L of TERRAIN_LEVELS) {
      const rows = rowsOf(digests, L);
      if (!rows.length || safetyReport(rows).overload) continue;
      const composite = compositeScore(rows);
      if (!levelBest[L] || composite > levelBest[L].composite) levelBest[L] = { composite, completion: safetyReport(rows).completion, mpc: structuredClone(mpcBy[L]), reward: structuredClone(rewardBy[L]) };
    }
  };
  const snapshot = () => ({ actor: Array.from(model.actor.w), critic: model.critic.toJSON() });
  const started = performance.now();
  const aborted = () => options.signal?.aborted;

  try {
    if (!designer.history.length) {
      const digests = await evaluate();
      designer.record({ epoch: 0, mpc: structuredClone(mpcBy), reward: structuredClone(rewardBy), digests, extra: snapshot() });
      updateLevelBest(digests);
      report({ log: 'baseline evaluation done (batch 0 = untrained actor)' });
    }
    while (completed < totalTargetEpisodes && !aborted()) {
      batch++;
      const inner = freshInner(), innerByLevel = perLevel(freshInner());
      const batchStart = performance.now();
      for (let ep = 0; ep < episodesPerBatch && completed < totalTargetEpisodes && !aborted(); ep++) {
        const spec = multiTerrainSpec(batch, ep);
        model.params = paramsOf(spec.level);
        trainer.opts.horizon = Math.round(mpcBy[spec.level].horizonSteps);
        const r = await trainer.runEpisode(spec, durationS);
        accumulateInner(inner, spec, r, { candidates: K, durationS });
        accumulateInner(innerByLevel[spec.level], spec, r, { candidates: K, durationS });
        completed++;
        const elapsed = (performance.now() - batchStart) / 1000;
        report({ batch, episode: ep + 1, episodesPerBatch, completed, totalTargetEpisodes,
          stepsPerSecond: (inner.realSteps + inner.rolloutSteps) / Math.max(1e-3, elapsed),
          etaBatchS: elapsed / (ep + 1) * (episodesPerBatch - ep - 1), inner: innerSummary(inner) });
      }
      if (!inner.episodes) break;
      const innerStats = innerSummary(inner);
      // ---- outer loop: evaluate, keep or roll back, meta-update
      const digests = await evaluate();
      const entry = designer.record({ epoch: batch, mpc: structuredClone(mpcBy), reward: structuredClone(rewardBy), digests, extra: snapshot() });
      const decision = designer.decide(entry);
      let evalDigests = digests;
      const reverted = [];
      if (decision.rollback) {
        model.actor.w.set(decision.to.extra.actor); model.critic.load(decision.to.extra.critic);
        mpcBy = structuredClone(decision.to.mpc); rewardBy = structuredClone(decision.to.reward);
        evalDigests = decision.to.digests;
      } else {
        for (const L of TERRAIN_LEVELS) {                    // per-terrain rollback
          const rows = rowsOf(digests, L), best = levelBest[L];
          if (!rows.length || !best) continue;
          const composite = compositeScore(rows), safety = safetyReport(rows);
          if (safety.overload || safety.completion < best.completion - SAFETY.completionDropPts || composite < best.composite - 0.1 * Math.abs(best.composite)) {
            mpcBy[L] = structuredClone(best.mpc); rewardBy[L] = structuredClone(best.reward); reverted.push(L);
          }
        }
        updateLevelBest(digests);
      }
      // ONE batched meta-update for all 14 terrains.
      const proposal = await designer.proposeAll(Object.fromEntries(TERRAIN_LEVELS.map(L => [L, { mpc: mpcBy[L], reward: rewardBy[L], digests: rowsOf(evalDigests, L), inner: innerSummary(innerByLevel[L]) }])), innerStats);
      for (const L of TERRAIN_LEVELS) { mpcBy[L] = proposal.byLevel[L].updatedMpcConfig; rewardBy[L] = proposal.byLevel[L].updatedRewardWeights; }
      const summaryText = proposal.insight || proposal.byLevel[1].diagnosticSummary;
      report({ log: `[Batch ${batch}] ${completed}/${totalTargetEpisodes} episodes (${((performance.now() - batchStart) / 1000).toFixed(0)} s) · 14 terrains · composite ${entry.composite.toFixed(2)} · completion ${entry.safety.completion.toFixed(0)}% · ${decision.rollback ? `rollback to batch ${decision.to.epoch} (${decision.reason})` : reverted.length ? `per-terrain rollback L${reverted.join(', L')}` : 'kept'} · ${proposal.source} (1 call, 14 terrains): ${summaryText.slice(0, 240)}`,
        batchDone: { batch, composite: entry.composite, completion: entry.safety.completion, rollback: decision.rollback, reverted, source: proposal.source, summary: summaryText, inner: innerStats } });
      const history = designer.history.map(e => ({ epoch: e.epoch, mpc: e.mpc, reward: e.reward, digests: e.digests, extra: e.extra }));
      await saveCheckpoint('latest', { episodes: completed, batch, model: model.toJSON(), mpcBy, rewardBy, levelBest, history });
      if (Math.floor(completed / 5000) > Math.floor((completed - inner.episodes) / 5000)) {
        await saveCheckpoint(`episodes-${completed}`, { episodes: completed, batch, model: model.toJSON(), mpcBy, rewardBy });
        report({ log: `[Checkpoint] saved model state at ${completed} episodes to IndexedDB` });
      }
    }
  } finally {
    await pool.terminate();
  }
  const best = designer.archive.best();
  const result = {
    episodes: completed, batches: batch, minutes: (performance.now() - started) / 60000, stopped: Boolean(aborted()),
    best: best ? { batch: best.epoch, composite: best.composite, mpc: best.mpc, reward: best.reward, actor: best.extra.actor } : null,
    llmError: designer.lastError
  };
  report({ log: `${result.stopped ? 'stopped' : 'finished'} after ${completed} episodes; best batch ${result.best?.batch}`, done: result });
  return result;
}

/**
 * Deployment checkpoint in the format of js/residualMppiWeights.js (download it and run
 * `node tools/install_checkpoint.mjs <file>` to make it the MPC + RL + LLM controller).
 */
export function deploymentCheckpoint(result) {
  if (!result?.best) return null;
  const paramsByLevel = Object.fromEntries(TERRAIN_LEVELS.map(L => [L, toParams(result.best.mpc[L], result.best.reward[L], DEFAULT_PARAMS)]));
  return { actor: { w: result.best.actor, mixSynergy: 0.7, mixFine: 0.3 }, params: paramsByLevel[1], paramsByLevel,
    meta: { trainedAt: new Date().toISOString(), source: 'browser runMassiveTrainingLoop', episodes: result.episodes, bestBatch: result.best.batch, composite: result.best.composite } };
}
