/**
 * LLMMPCOPTIMIZER.JS - closed-loop LLM MPC and reward co-designer.
 *
 * After every multi-seed evaluation batch over the stress matrix
 * (nominal, push, gaussian, laplace, outlier, stiffness -20 %, friction -40 %,
 * payload +30 %, delay 100 ms on Levels 1, 4, 7, 10) the Comparative-benchmark
 * digest (v_bar, RMSE, max|e|, t_rec, int u'u, CoT, t_solve, Success,
 * rodResidualMm, relaxedCablesPct, contactSlipMs) and the Pareto history are
 * sent to an LLM with a strict JSON-schema output that updates
 *
 *   (a) MPC cost weights   Q_vel, Q_rmse, Q_comMargin, Q_clearance, R_effort, R_smooth,
 *   (b) SGT gains and Tension-Shield bounds,
 *   (c) RL reward-shaping weights.
 *
 * Safety: every value is validated against the schema and clamped to a
 * physically safe box before use (the model can only *shrink* the residual
 * stroke); a Pareto archive keeps the non-dominated checkpoints and the
 * designer reverts to the best one when mission completion drops or an outer
 * cable overloads. With no API key (or when the call fails) the designer falls
 * back to a deterministic rule tuner implementing the same five diagnostic
 * rules, so the loop always runs.
 *
 * Credentials come from the environment only (ANTHROPIC_API_KEY, OPENAI_API_KEY
 * or LLM_API_KEY) and are never logged or written to disk.
 */
import { LLM_SYSTEM_PROMPT } from './llmSystemPrompt.js';
import { DEFAULT_PARAMS } from './residualMppi.js';

export const TUNABLE_MPC_KEYS = Object.freeze([
  'horizonSteps', 'qVelocityTrack', 'qLateralRmse', 'qComMargin', 'qClearance',
  'rControlEffort', 'rSmoothnessDeltaU', 'minCableTensionN', 'maxCableTensionN', 'sgtGainKp', 'sgtGainKd', 'maxResidualStrokeM',
  'sgtSettleSpeed', 'sgtSettleMinTime', 'sgtSettleMaxTime', 'sgtReleaseTime'
]);
export const REWARD_KEYS = Object.freeze(['velocity', 'rmse', 'effort', 'cot', 'residual', 'slack', 'slip', 'comMargin']);

/** Safe box for every tunable (spec bounds). */
export const BOUNDS = Object.freeze({
  horizonSteps: [6, 16], qVelocityTrack: [0.01, 50], qLateralRmse: [0.01, 50], qComMargin: [0.01, 50], qClearance: [0.01, 50],
  rControlEffort: [0.001, 50], rSmoothnessDeltaU: [0.001, 50], minCableTensionN: [5, 25], maxCableTensionN: [120, 200],
  sgtGainKp: [0.5, 2.5], sgtGainKd: [0, 2], maxResidualStrokeM: [0.005, 0.030],
  sgtSettleSpeed: [0.08, 0.50], sgtSettleMinTime: [0.03, 0.40], sgtSettleMaxTime: [0.25, 1.50], sgtReleaseTime: [0.10, 0.50],
  velocity: [0, 20], rmse: [0, 20], effort: [0, 20], cot: [0, 20], residual: [0, 20], slack: [0, 20], slip: [0, 20], comMargin: [0, 20]
});

/** Hard limits used by the rollback logic. */
export const SAFETY = Object.freeze({ maxOuterTensionN: 180, completionDropPts: 5, rodLimitMm: 0.05 });

export const DEFAULT_REWARD = Object.freeze({
  velocity: DEFAULT_PARAMS.wVelocity, rmse: DEFAULT_PARAMS.wRmse, effort: DEFAULT_PARAMS.wEffort, cot: DEFAULT_PARAMS.wCot,
  residual: DEFAULT_PARAMS.wResidual, slack: DEFAULT_PARAMS.wSlack, slip: DEFAULT_PARAMS.wSlip, comMargin: DEFAULT_PARAMS.wComMargin
});
export const DEFAULT_MPC = Object.freeze(Object.fromEntries(TUNABLE_MPC_KEYS.map(k => [k, DEFAULT_PARAMS[k]])));

/** JSON Schema for the structured LLM output (strict: no extra keys, every key required). */
export const MpcAndRewardUpdateSchema = Object.freeze({
  type: 'object',
  additionalProperties: false,
  required: ['diagnosticSummary', 'updatedMpcConfig', 'updatedRewardWeights'],
  properties: {
    diagnosticSummary: { type: 'string', description: 'Concise physical reasoning, before the numbers.' },
    updatedMpcConfig: {
      type: 'object', additionalProperties: false, required: [...TUNABLE_MPC_KEYS],
      properties: Object.fromEntries(TUNABLE_MPC_KEYS.map(k => [k, { type: k === 'horizonSteps' ? 'integer' : 'number' }]))
    },
    updatedRewardWeights: {
      type: 'object', additionalProperties: false, required: [...REWARD_KEYS],
      properties: Object.fromEntries(REWARD_KEYS.map(k => [k, { type: 'number' }]))
    }
  }
});

const clamp = (x, [lo, hi]) => Math.max(lo, Math.min(hi, x));
const finite = (x, fallback) => (Number.isFinite(x) ? x : fallback);

/** Validate + clamp a response into safe parameters. Throws on structural violations. */
export function validateUpdate(raw, current) {
  if (!raw || typeof raw !== 'object') throw new Error('response is not an object');
  if (typeof raw.diagnosticSummary !== 'string') throw new Error('diagnosticSummary missing');
  const mpc = {}, reward = {};
  for (const key of TUNABLE_MPC_KEYS) {
    const v = raw.updatedMpcConfig?.[key];
    if (typeof v !== 'number') throw new Error(`updatedMpcConfig.${key} missing or not a number`);
    mpc[key] = clamp(finite(v, current.mpc[key]), BOUNDS[key]);
  }
  mpc.horizonSteps = Math.round(mpc.horizonSteps);
  for (const key of REWARD_KEYS) {
    const v = raw.updatedRewardWeights?.[key];
    if (typeof v !== 'number') throw new Error(`updatedRewardWeights.${key} missing or not a number`);
    reward[key] = clamp(finite(v, current.reward[key]), BOUNDS[key]);
  }
  return { diagnosticSummary: raw.diagnosticSummary.slice(0, 2000), updatedMpcConfig: mpc, updatedRewardWeights: reward };
}

/** MPC + reward dictionaries -> the flat parameter object used by js/residualMppi.js. */
export function toParams(mpc, reward, base = DEFAULT_PARAMS) {
  return {
    ...base, ...mpc,
    wVelocity: reward.velocity, wRmse: reward.rmse, wEffort: reward.effort, wCot: reward.cot,
    wResidual: reward.residual, wSlack: reward.slack, wSlip: reward.slip, wComMargin: reward.comMargin
  };
}

// ------------------------------------------------------------------ scoring and Pareto
const num = (x, fallback = 0) => (Number.isFinite(x) ? x : fallback);

/** Digest row in the field names the system prompt uses (`tRecS`). */
export function promptRow(d) {
  return {
    level: d.level, disturbance: d.disturbance, seeds: d.seeds,
    avgSpeedMs: round(d.avgSpeedMs), rmseM: round(d.rmseM), maxErrorM: round(d.maxErrorM), tRecS: round(d.recoveryTimeS),
    controlEffortUtu: round(d.controlEffortUtu), costOfTransport: round(d.costOfTransport), solveTimeMs: round(d.solveTimeMs),
    successRatePct: round(d.successRatePct), rodResidualMm: round(d.rodResidualMm, 5), relaxedCablesPct: round(d.relaxedCablesPct),
    contactSlipMs: round(d.contactSlipMs), energyDriftJ: round(d.energyDriftJ, 6), progressM: round(d.progressM),
    maxOuterTensionN: round(d.maxOuterTensionN, 1)
  };
}
const round = (x, d = 4) => (Number.isFinite(x) ? Number(x.toFixed(d)) : null);

/** Scalar quality (higher is better): completion -> speed -> RMSE -> CoT -> slack -> rod residual. */
export function compositeScore(digests) {
  let total = 0;
  for (const d of digests) {
    total += 2.0*num(d.successRatePct)
      + 50.0*num(d.avgSpeedMs)
      - 25.0*num(d.rmseM)
      - 10.0*Math.min(num(d.costOfTransport, 50), 50)
      - 2.0*num(d.relaxedCablesPct)
      - 100.0*num(d.rodResidualMm);
  }
  return total/Math.max(1, digests.length);
}

/** Aggregate objectives (all "higher is better") used for Pareto dominance. */
export function objectives(digests) {
  const m = key => digests.reduce((s, d) => s+num(d[key]), 0)/Math.max(1, digests.length);
  return [m('successRatePct'), m('avgSpeedMs'), -m('rmseM'), -Math.min(m('costOfTransport'), 50), -m('relaxedCablesPct'), -m('rodResidualMm')];
}
const dominates = (a, b) => a.every((v, i) => v >= b[i]-1e-12) && a.some((v, i) => v > b[i]+1e-12);

export function safetyReport(digests) {
  const maxOuter = Math.max(0, ...digests.map(d => num(d.maxOuterTensionN)));
  return {
    overload: maxOuter > SAFETY.maxOuterTensionN,
    maxOuterTensionN: maxOuter,
    rodExceeded: digests.some(d => num(d.rodResidualMm) > SAFETY.rodLimitMm),
    completion: digests.reduce((s, d) => s+num(d.successRatePct), 0)/Math.max(1, digests.length)
  };
}

export class ParetoArchive {
  constructor() { this.entries = []; }
  /** Adds the entry; returns true when it is non-dominated (and prunes what it dominates). */
  add(entry) {
    if (this.entries.some(e => dominates(e.objectives, entry.objectives))) return false;
    this.entries = this.entries.filter(e => !dominates(entry.objectives, e.objectives));
    this.entries.push(entry);
    return true;
  }
  /** Best safe checkpoint by composite score (falls back to any). */
  best() {
    const safe = this.entries.filter(e => !e.safety.overload);
    const pool = safe.length ? safe : this.entries;
    return pool.reduce((a, b) => (b.composite > a.composite ? b : a), pool[0] || null);
  }
}

// ------------------------------------------------------------------ rule-based fallback tuner
/**
 * Deterministic tuner implementing the five diagnostic rules of the system
 * prompt. Used when no LLM is configured and as a fallback if a call fails.
 */
export function ruleBasedUpdate(current, digests) {
  const mpc = { ...current.mpc }, reward = { ...current.reward };
  const fired = [];
  const rows = digests;
  const where = (pred) => rows.filter(pred);
  const hard = where(d => [4, 7, 10].includes(d.level));
  const mul = (obj, key, f) => { obj[key] = clamp(obj[key]*f, BOUNDS[key]); };
  const add = (obj, key, v) => { obj[key] = clamp(obj[key]+v, BOUNDS[key]); };

  const slack = Math.max(0, ...rows.map(d => num(d.relaxedCablesPct)));
  if (slack > 2) {
    add(mpc, 'minCableTensionN', 2); mpc.minCableTensionN = Math.max(8, mpc.minCableTensionN);
    mul(mpc, 'rSmoothnessDeltaU', 1.25); mul(reward, 'slack', 1.2);
    fired.push(`slack up to ${slack.toFixed(1)}% of cables: raise payload-tension floor to ${mpc.minCableTensionN.toFixed(1)} N and smoothness penalty`);
  }
  const stalled = hard.filter(d => num(d.successRatePct) < 50);
  if (stalled.length) {
    mul(mpc, 'qComMargin', 1.2); mul(mpc, 'sgtGainKp', 1.15); mul(reward, 'comMargin', 1.15);
    fired.push(`${stalled.length} rocky-terrain cell(s) below 50% completion: stronger COM-margin drive and SGT gain`);
  }
  const slip = Math.max(0, ...where(d => d.disturbance === 'friction').map(d => num(d.contactSlipMs)));
  if (slip > 0.15) {
    mul(mpc, 'rSmoothnessDeltaU', 1.25); mul(mpc, 'qClearance', 1.15); mul(reward, 'slip', 1.25);
    fired.push(`slip ${slip.toFixed(2)} m/s under reduced friction: smoother commands and more clearance weight`);
  }
  const push = where(d => d.disturbance === 'push' || d.disturbance === 'impulse');
  const rmse = Math.max(0, ...push.map(d => num(d.rmseM))), trec = Math.max(0, ...push.map(d => num(d.recoveryTimeS)));
  if (rmse > 0.5 || trec > 5) {
    mul(mpc, 'qLateralRmse', 1.2); mul(reward, 'rmse', 1.15); mpc.maxCableTensionN = Math.min(mpc.maxCableTensionN, 180);
    fired.push(`disturbance response weak (RMSE ${rmse.toFixed(2)} m, recovery ${trec.toFixed(1)} s): more lateral-error weight`);
  }
  const rod = Math.max(0, ...rows.map(d => num(d.rodResidualMm))), drift = Math.max(0, ...rows.map(d => num(d.energyDriftJ)));
  if (rod > SAFETY.rodLimitMm || drift > 1.0) {
    mul(mpc, 'rControlEffort', 1.25); mul(mpc, 'rSmoothnessDeltaU', 1.25); mul(mpc, 'maxResidualStrokeM', 0.85);
    mul(reward, 'residual', 1.2); mul(reward, 'effort', 1.1);
    fired.push(`rod residual ${rod.toFixed(4)} mm / energy drift ${drift.toFixed(3)} J: damp bang-bang actuation and shrink the residual stroke`);
  }
  if (!fired.length) {
    mul(mpc, 'qVelocityTrack', 1.1); mul(reward, 'velocity', 1.1);
    // The gait waits in SETTLE/RELEASE most of a cycle: shorten the waits while everything is admissible.
    mul(mpc, 'sgtReleaseTime', 0.9); mul(mpc, 'sgtSettleMaxTime', 0.9); mul(mpc, 'sgtSettleSpeed', 1.1);
    fired.push('all admissibility checks pass: push forward speed and shorten the SGT settle/release waits');
  }
  return { diagnosticSummary: `[rule-based] ${fired.join('; ')}.`, updatedMpcConfig: roundHorizon(mpc), updatedRewardWeights: reward };
}
const roundHorizon = mpc => ({ ...mpc, horizonSteps: Math.round(mpc.horizonSteps) });

/** Parse a model reply that should be JSON (tolerates ```json fences and leading prose). */
export function parseJsonContent(text) {
  if (text && typeof text === 'object') return text;
  const raw = String(text ?? '').trim().replace(/^```(?:json)?\s*/i, '').replace(/```\s*$/, '');
  try { return JSON.parse(raw); }
  catch {
    const a = raw.indexOf('{'), b = raw.lastIndexOf('}');
    if (a >= 0 && b > a) return JSON.parse(raw.slice(a, b+1));
    throw new Error('model reply is not JSON');
  }
}

/**
 * Node only: read KEY=VALUE lines from a local .env (never committed, never printed)
 * into process.env without overriding variables that are already set.
 */
export async function loadDotEnv(file = '.env') {
  if (typeof process === 'undefined' || !process.versions?.node) return false;
  const { readFileSync, existsSync } = await import('node:fs');
  if (!existsSync(file)) return false;
  for (const line of readFileSync(file, 'utf8').split(/\r?\n/)) {
    const m = line.match(/^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$/);
    if (!m || line.trim().startsWith('#')) continue;
    const value = m[2].replace(/^(['"])(.*)\1$/, '$2');
    if (process.env[m[1]] === undefined) process.env[m[1]] = value;
  }
  return true;
}

// ------------------------------------------------------------------ the co-designer
export class LlmMpcCoDesigner {
  /**
   * @param {object} options provider ('anthropic' | 'openai' | 'offline' | 'auto'), model, endpoint,
   *        apiKey (defaults to the environment), systemPrompt, fetchImpl, timeoutMs
   */
  constructor(options = {}) {
    // Settings come from options, then Vite-style env (import.meta.env), then process.env (Node,
    // filled from a local .env by loadDotEnv()). VITE_LLM_* select any OpenAI-compatible endpoint:
    // Gemini (generativelanguage.googleapis.com/v1beta/openai), Groq, or a local Ollama server.
    const env = { ...((typeof process !== 'undefined' && process.env) || {}), ...(import.meta.env || {}) };
    this.systemPrompt = options.systemPrompt || LLM_SYSTEM_PROMPT;
    this.fetchImpl = options.fetchImpl || (typeof fetch === 'function' ? fetch.bind(globalThis) : null);
    this.timeoutMs = options.timeoutMs ?? 90000;
    let provider = options.provider || env.LLM_PROVIDER || 'auto';
    const compatibleUrl = env.VITE_LLM_API_URL;
    if (provider === 'auto' && compatibleUrl) provider = 'compatible';
    if (provider === 'compatible') {
      // OpenAI-compatible chat/completions with response_format json_object (Gemini, Groq, Ollama, ...).
      this.apiKey = options.apiKey ?? env.VITE_LLM_API_KEY ?? env.LLM_API_KEY ?? 'ollama';
      this.endpoint = options.endpoint || compatibleUrl || env.LLM_API_URL || 'http://localhost:11434/v1/chat/completions';
      this.model = options.model || env.VITE_LLM_MODEL || env.LLM_MODEL || 'gemini-2.5-flash';
      this.provider = this.fetchImpl ? 'compatible' : 'offline';
      this.archive = new ParetoArchive(); this.history = []; this.lastError = null;
      return;
    }
    this.apiKey = options.apiKey ?? (provider === 'openai' ? env.OPENAI_API_KEY : provider === 'anthropic' ? env.ANTHROPIC_API_KEY : null)
      ?? env.LLM_API_KEY ?? null;
    if (provider === 'auto') {
      if (env.ANTHROPIC_API_KEY) { provider = 'anthropic'; this.apiKey = options.apiKey ?? env.ANTHROPIC_API_KEY; }
      else if (env.OPENAI_API_KEY) { provider = 'openai'; this.apiKey = options.apiKey ?? env.OPENAI_API_KEY; }
      else provider = 'offline';
    }
    if (provider !== 'offline' && (!this.apiKey || !this.fetchImpl)) provider = 'offline';
    this.provider = provider;
    this.model = options.model || env.LLM_MODEL || (provider === 'openai' ? 'gpt-4o' : 'claude-sonnet-5-5');
    this.endpoint = options.endpoint || env.LLM_API_URL ||
      (provider === 'openai' ? 'https://api.openai.com/v1/chat/completions' : 'https://api.anthropic.com/v1/messages');
    this.archive = new ParetoArchive();
    this.history = [];
    this.lastError = null;
  }

  /** Score a batch and file it in the Pareto archive. `extra` carries the checkpoint payload (e.g. actor weights). */
  record({ epoch, mpc, reward, digests, extra = null }) {
    const entry = {
      epoch, mpc: structuredClone(mpc), reward: structuredClone(reward), digests,
      composite: compositeScore(digests), objectives: objectives(digests), safety: safetyReport(digests), extra
    };
    // Overloaded checkpoints are never rollback targets and must not displace safe ones.
    entry.onFront = entry.safety.overload ? false : this.archive.add(entry);
    this.history.push(entry);
    return entry;
  }

  /**
   * Decide whether to keep going from `entry` or revert: revert when mission
   * completion fell by more than SAFETY.completionDropPts below the best
   * checkpoint, or an outer cable exceeded SAFETY.maxOuterTensionN.
   */
  decide(entry) {
    const best = this.archive.best();
    if (entry.safety.overload) {
      const reason = `outer-cable overload ${entry.safety.maxOuterTensionN.toFixed(0)} N > ${SAFETY.maxOuterTensionN} N`;
      // No safe checkpoint yet (e.g. the very first evaluation overloads): nothing to roll back
      // to, so keep going and let the tuner push the parameters toward the safe set.
      if (!best) return { rollback: false, reason: `${reason}; no safe checkpoint to roll back to yet`, to: entry };
      return { rollback: true, reason, to: best };
    }
    const previousBest = this.history.filter(e => e !== entry && !e.safety.overload).reduce((m, e) => Math.max(m, e.safety.completion), -Infinity);
    if (best && Number.isFinite(previousBest) && entry.safety.completion < previousBest-SAFETY.completionDropPts) {
      return { rollback: true, reason: `mission completion fell ${entry.safety.completion.toFixed(1)}% < ${previousBest.toFixed(1)}%`, to: best };
    }
    if (best && best !== entry && entry.composite < best.composite-0.1*Math.abs(best.composite)) {
      return { rollback: true, reason: `composite score regressed by >10% (${entry.composite.toFixed(1)} vs ${best.composite.toFixed(1)})`, to: best };
    }
    return { rollback: false, reason: 'accepted', to: entry };
  }

  /** Ask the model (or the rule tuner) for the next parameters. Always returns a validated, clamped update. */
  async propose({ mpc, reward, digests, innerLoop = null }) {
    const current = { mpc, reward };
    const best = this.archive.best();
    const rows = digests.map(promptRow);
    let raw, source = this.provider;
    if (this.provider === 'offline') raw = ruleBasedUpdate(current, digests);
    else {
      try { raw = await this.callModel({ mpc, reward, rows, best, innerLoop }); }
      catch (error) {
        this.lastError = String(error.message || error).replace(/(key|token|bearer)\s*[:=]?\s*\S+/gi, '$1 [redacted]');
        raw = ruleBasedUpdate(current, digests); source = 'offline-fallback';
      }
    }
    let update;
    try { update = validateUpdate(raw, current); }
    catch (error) {
      this.lastError = `invalid model output: ${error.message}`;
      update = validateUpdate(ruleBasedUpdate(current, digests), current); source = 'offline-fallback';
    }
    return { ...update, source };
  }

  userPayload({ mpc, reward, rows, best, innerLoop = null }) {
    return JSON.stringify({
      // Aggregated failure modes of the inner learning loop since the last meta-update.
      innerLoopSummary: innerLoop,
      targetVelocityMs: 1.30, maxAllowedStrokeM: 0.120, maxResidualStrokeM: 0.030,
      currentMpcConfig: mpc, currentRewardWeights: reward,
      bestParetoCheckpoint: best ? { epoch: best.epoch, mpc: best.mpc, reward: best.reward, composite: round(best.composite, 3) } : null,
      latestBenchmarkTable: rows,
      paretoFrontEpochs: this.archive.entries.map(e => ({ epoch: e.epoch, composite: round(e.composite, 3), objectives: e.objectives.map(v => round(v, 4)) })),
      previousEpochsSummary: this.history.slice(-4).map(h => ({ epoch: h.epoch, composite: round(h.composite, 3), completion: round(h.safety.completion, 1),
        overload: h.safety.overload, mpc: h.mpc }))
    }, null, 1);
  }

  async callModel(context) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeoutMs);
    try {
      const payload = this.userPayload(context);
      if (this.provider === 'anthropic') {
        const response = await this.fetchImpl(this.endpoint, {
          method: 'POST', signal: controller.signal,
          headers: { 'content-type': 'application/json', 'x-api-key': this.apiKey, 'anthropic-version': '2023-06-01' },
          body: JSON.stringify({
            model: this.model, max_tokens: 2048, temperature: 0.2, system: this.systemPrompt,
            messages: [{ role: 'user', content: payload }],
            tools: [{ name: 'update_mpc_and_reward', description: 'Return the next MPC cost weights, SGT/shield parameters and RL reward weights.', input_schema: MpcAndRewardUpdateSchema }],
            tool_choice: { type: 'tool', name: 'update_mpc_and_reward' }
          })
        });
        if (!response.ok) throw new Error(`LLM HTTP ${response.status}`);
        const data = await response.json();
        const block = (data.content || []).find(b => b.type === 'tool_use');
        if (!block) throw new Error('no tool_use block in response');
        return block.input;
      }
      // 'compatible' endpoints (Gemini / Groq / Ollama) accept json_object but not always a strict
      // json_schema, so the schema goes into the system prompt and validateUpdate() enforces it.
      const compatible = this.provider === 'compatible';
      const response = await this.fetchImpl(this.endpoint, {
        method: 'POST', signal: controller.signal,
        headers: { 'content-type': 'application/json', authorization: `Bearer ${this.apiKey}` },
        body: JSON.stringify({
          model: this.model, temperature: 0.2,
          messages: [
            { role: 'system', content: compatible ? `${this.systemPrompt}\n\nReply with ONE JSON object only, matching this JSON Schema:\n${JSON.stringify(MpcAndRewardUpdateSchema)}` : this.systemPrompt },
            { role: 'user', content: payload }
          ],
          response_format: compatible ? { type: 'json_object' }
            : { type: 'json_schema', json_schema: { name: 'MpcAndRewardUpdateSchema', strict: true, schema: MpcAndRewardUpdateSchema } }
        })
      });
      if (!response.ok) throw new Error(`LLM HTTP ${response.status}`);
      const data = await response.json();
      return parseJsonContent(data.choices?.[0]?.message?.content);
    } finally { clearTimeout(timer); }
  }
}
