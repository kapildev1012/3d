import test from 'node:test';
import assert from 'node:assert/strict';
import {
  LlmMpcCoDesigner, MpcAndRewardUpdateSchema, validateUpdate, ruleBasedUpdate, compositeScore, ParetoArchive, safetyReport,
  DEFAULT_MPC, DEFAULT_REWARD, BOUNDS, TUNABLE_MPC_KEYS, REWARD_KEYS, toParams
} from '../js/llmMpcOptimizer.js';
import { LLM_SYSTEM_PROMPT } from '../js/llmSystemPrompt.js';

const row = (over = {}) => ({ level: 1, disturbance: 'nominal', seeds: 1, avgSpeedMs: 0.12, rmseM: 0.2, maxErrorM: 0.4, recoveryTimeS: NaN,
  controlEffortUtu: 0.02, costOfTransport: 4, solveTimeMs: 5, successRatePct: 100, rodResidualMm: 0.001, relaxedCablesPct: 1,
  contactSlipMs: 0.05, energyDriftJ: 0.01, progressM: 2, maxOuterTensionN: 90, ...over });
const current = { mpc: { ...DEFAULT_MPC }, reward: { ...DEFAULT_REWARD } };
const goodResponse = () => ({ diagnosticSummary: 'ok', updatedMpcConfig: { ...DEFAULT_MPC }, updatedRewardWeights: { ...DEFAULT_REWARD } });

test('system prompt carries the five diagnostic rules and the parameter map', () => {
  for (const needle of ['relaxedCablesPct', 'PASSIVE_SETTLE', 'contactSlipMs', 'tRecS', 'rodResidualMm', 'MpcAndRewardUpdateSchema', 'PARAMETER MAP']) assert.ok(LLM_SYSTEM_PROMPT.includes(needle), needle);
});

test('schema is strict and covers every tunable', () => {
  assert.equal(MpcAndRewardUpdateSchema.additionalProperties, false);
  assert.deepEqual(MpcAndRewardUpdateSchema.properties.updatedMpcConfig.required, [...TUNABLE_MPC_KEYS]);
  assert.deepEqual(MpcAndRewardUpdateSchema.properties.updatedRewardWeights.required, [...REWARD_KEYS]);
});

test('validateUpdate clamps to the safe box, rounds the horizon, rejects missing fields', () => {
  const wild = goodResponse();
  Object.assign(wild.updatedMpcConfig, { horizonSteps: 99.4, minCableTensionN: -4, maxCableTensionN: 900, maxResidualStrokeM: 0.2, sgtGainKp: 50 });
  wild.updatedRewardWeights.velocity = 1e6;
  const safe = validateUpdate(wild, current);
  assert.equal(safe.updatedMpcConfig.horizonSteps, 16);
  assert.equal(safe.updatedMpcConfig.minCableTensionN, BOUNDS.minCableTensionN[0]);
  assert.equal(safe.updatedMpcConfig.maxCableTensionN, 200);
  assert.equal(safe.updatedMpcConfig.maxResidualStrokeM, 0.030);          // the model may only shrink the residual stroke
  assert.equal(safe.updatedMpcConfig.sgtGainKp, 2.5);
  assert.equal(safe.updatedRewardWeights.velocity, 20);
  const broken = goodResponse(); delete broken.updatedMpcConfig.qClearance;
  assert.throws(() => validateUpdate(broken, current), /qClearance/);
  assert.throws(() => validateUpdate({ ...goodResponse(), diagnosticSummary: 3 }, current), /diagnosticSummary/);
});

test('rule tuner implements the five diagnostic rules', () => {
  const slack = ruleBasedUpdate(current, [row({ relaxedCablesPct: 9 })]);
  assert.ok(slack.updatedMpcConfig.minCableTensionN > current.mpc.minCableTensionN && slack.updatedMpcConfig.rSmoothnessDeltaU > current.mpc.rSmoothnessDeltaU);
  const stall = ruleBasedUpdate(current, [row({ level: 7, successRatePct: 0 })]);
  assert.ok(stall.updatedMpcConfig.qComMargin > current.mpc.qComMargin && stall.updatedMpcConfig.sgtGainKp > current.mpc.sgtGainKp);
  assert.ok(stall.updatedMpcConfig.sgtGainKp <= current.mpc.sgtGainKp*1.25+1e-12);
  const slip = ruleBasedUpdate(current, [row({ disturbance: 'friction', contactSlipMs: 0.4 })]);
  assert.ok(slip.updatedMpcConfig.qClearance > current.mpc.qClearance);
  const push = ruleBasedUpdate(current, [row({ disturbance: 'push', rmseM: 1.2 })]);
  assert.ok(push.updatedMpcConfig.qLateralRmse > current.mpc.qLateralRmse && push.updatedMpcConfig.maxCableTensionN <= 180);
  const rod = ruleBasedUpdate(current, [row({ rodResidualMm: 0.2 })]);
  assert.ok(rod.updatedMpcConfig.rControlEffort > current.mpc.rControlEffort && rod.updatedMpcConfig.maxResidualStrokeM < current.mpc.maxResidualStrokeM);
  const calm = ruleBasedUpdate(current, [row()]);
  assert.ok(calm.updatedMpcConfig.qVelocityTrack > current.mpc.qVelocityTrack);
});

test('composite score ranks completion > speed and penalizes slack, RMSE and rod residual', () => {
  const base = compositeScore([row()]);
  assert.ok(compositeScore([row({ successRatePct: 0 })]) < base);
  assert.ok(compositeScore([row({ avgSpeedMs: 0.3 })]) > base);
  assert.ok(compositeScore([row({ relaxedCablesPct: 20 })]) < base);
  assert.ok(compositeScore([row({ rodResidualMm: 0.5 })]) < base);
  assert.ok(Number.isFinite(compositeScore([row({ costOfTransport: NaN })])));
});

test('Pareto archive keeps only non-dominated checkpoints', () => {
  const designer = new LlmMpcCoDesigner({ provider: 'offline' });
  const a = designer.record({ epoch: 0, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row()] });
  const b = designer.record({ epoch: 1, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row({ avgSpeedMs: 0.2, rmseM: 0.1 })] });
  const c = designer.record({ epoch: 2, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row({ avgSpeedMs: 0.05, rmseM: 0.5 })] });
  assert.ok(b.onFront); assert.ok(!c.onFront);
  assert.deepEqual(designer.archive.entries.map(e => e.epoch), [1]);
  assert.equal(designer.archive.best().epoch, 1);
});

test('rollback: outer-cable overload and mission-completion drop revert to the best checkpoint', () => {
  const designer = new LlmMpcCoDesigner({ provider: 'offline' });
  designer.record({ epoch: 0, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row({ avgSpeedMs: 0.2 })] });
  const overload = designer.record({ epoch: 1, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row({ avgSpeedMs: 0.3, maxOuterTensionN: 195 })] });
  assert.equal(safetyReport([row({ maxOuterTensionN: 195 })]).overload, true);
  const d1 = designer.decide(overload);
  assert.ok(d1.rollback && /overload/.test(d1.reason) && d1.to.epoch === 0);
  const dropped = designer.record({ epoch: 2, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row({ successRatePct: 60 })] });
  const d2 = designer.decide(dropped);
  assert.ok(d2.rollback && /completion/.test(d2.reason));
  const fine = designer.record({ epoch: 3, mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [row({ avgSpeedMs: 0.25 })] });
  assert.equal(designer.decide(fine).rollback, false);
});

test('anthropic provider: forced tool call, key only in the header, output validated', async () => {
  let seen;
  const fetchImpl = async (url, init) => {
    seen = { url, init, body: JSON.parse(init.body) };
    const resp = goodResponse(); resp.updatedMpcConfig.qVelocityTrack = 6;
    return { ok: true, json: async () => ({ content: [{ type: 'tool_use', name: 'update_mpc_and_reward', input: resp }] }) };
  };
  const designer = new LlmMpcCoDesigner({ provider: 'anthropic', apiKey: 'sk-test-123', fetchImpl, model: 'claude-sonnet-5-5' });
  assert.equal(designer.provider, 'anthropic');
  const out = await designer.propose({ ...current, digests: [row()] });
  assert.equal(out.source, 'anthropic');
  assert.equal(out.updatedMpcConfig.qVelocityTrack, 6);
  assert.equal(seen.body.tool_choice.name, 'update_mpc_and_reward');
  assert.deepEqual(seen.body.tools[0].input_schema, JSON.parse(JSON.stringify(MpcAndRewardUpdateSchema)));
  assert.equal(seen.init.headers['x-api-key'], 'sk-test-123');
  assert.ok(!seen.init.body.includes('sk-test-123'));
  assert.ok(seen.body.system.includes('NMPC'));
  assert.ok(JSON.parse(seen.body.messages[0].content).latestBenchmarkTable[0].tRecS !== undefined);
});

test('openai provider uses strict json_schema output', async () => {
  let body;
  const fetchImpl = async (_, init) => { body = JSON.parse(init.body); return { ok: true, json: async () => ({ choices: [{ message: { content: JSON.stringify(goodResponse()) } }] }) }; };
  const designer = new LlmMpcCoDesigner({ provider: 'openai', apiKey: 'k', fetchImpl });
  const out = await designer.propose({ ...current, digests: [row()] });
  assert.equal(out.source, 'openai');
  assert.equal(body.response_format.json_schema.strict, true);
});

test('malformed model output or a failed call falls back to the rule tuner (never throws, never leaks the key)', async () => {
  const bad = new LlmMpcCoDesigner({ provider: 'anthropic', apiKey: 'sk-secret', fetchImpl: async () => ({ ok: true, json: async () => ({ content: [{ type: 'tool_use', input: { diagnosticSummary: 'x' } }] }) }) });
  const out1 = await bad.propose({ ...current, digests: [row({ relaxedCablesPct: 9 })] });
  assert.equal(out1.source, 'offline-fallback');
  assert.ok(out1.updatedMpcConfig.minCableTensionN > current.mpc.minCableTensionN);
  const down = new LlmMpcCoDesigner({ provider: 'anthropic', apiKey: 'sk-secret', fetchImpl: async () => { throw new Error('connect ECONNREFUSED key sk-secret'); } });
  const out2 = await down.propose({ ...current, digests: [row()] });
  assert.equal(out2.source, 'offline-fallback');
  assert.ok(!String(down.lastError).includes('sk-secret'));
});

test('no API key: offline provider, no network call', async () => {
  const saved = { ...process.env };
  delete process.env.ANTHROPIC_API_KEY; delete process.env.OPENAI_API_KEY; delete process.env.LLM_API_KEY; delete process.env.LLM_PROVIDER;
  try {
    const designer = new LlmMpcCoDesigner({ fetchImpl: () => { throw new Error('network must not be used'); } });
    assert.equal(designer.provider, 'offline');
    const out = await designer.propose({ ...current, digests: [row()] });
    assert.equal(out.source, 'offline');
  } finally { Object.assign(process.env, saved); }
});

test('toParams maps reward names onto the packed parameter names', () => {
  const p = toParams(DEFAULT_MPC, { ...DEFAULT_REWARD, slack: 7 });
  assert.equal(p.wSlack, 7);
  assert.equal(p.qVelocityTrack, DEFAULT_MPC.qVelocityTrack);
});

test('OpenAI-compatible endpoint (Gemini / Groq / Ollama): json_object request, fenced JSON parsed, output validated', async () => {
  const { LlmMpcCoDesigner, parseJsonContent, DEFAULT_MPC, DEFAULT_REWARD } = await import('../js/llmMpcOptimizer.js');
  assert.deepEqual(parseJsonContent('```json\n{"a":1}\n```'), { a: 1 });
  assert.deepEqual(parseJsonContent('Here you go: {"a":2} thanks'), { a: 2 });
  let sent;
  const reply = { diagnosticSummary: 'ok', updatedMpcConfig: { ...DEFAULT_MPC, maxResidualStrokeM: 0.5 }, updatedRewardWeights: DEFAULT_REWARD };
  const fetchImpl = async (url, init) => { sent = { url, init }; return { ok: true, json: async () => ({ choices: [{ message: { content: '```json\n'+JSON.stringify(reply)+'\n```' } }] }) }; };
  const d = new LlmMpcCoDesigner({ provider: 'compatible', endpoint: 'https://api.groq.com/openai/v1/chat/completions', apiKey: 'test-key', model: 'llama-3.3-70b-versatile', fetchImpl });
  assert.equal(d.provider, 'compatible');
  const out = await d.propose({ mpc: DEFAULT_MPC, reward: DEFAULT_REWARD, digests: [] });
  const body = JSON.parse(sent.init.body);
  assert.equal(body.response_format.type, 'json_object');
  assert.equal(body.model, 'llama-3.3-70b-versatile');
  assert.equal(sent.init.headers.authorization, 'Bearer test-key');
  assert.equal(out.source, 'compatible');
  assert.ok(out.updatedMpcConfig.maxResidualStrokeM <= 0.030);     // the model may never widen the residual stroke
});
