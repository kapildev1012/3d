/**
 * TRAININGLOOP.JS - shared bookkeeping of the two-loop hybrid training schedule,
 * used by tools/run_hybrid_training.mjs (Node) and
 * js/experiments/runMassiveTraining100k.js (browser).
 *
 *   inner loop: domain-randomised MPPI + RL episodes, MPC weights / reward fixed
 *   outer loop: every N episodes the LLM (or rule tuner) meta-controller sees
 *               innerSummary() plus a held-out stress-matrix evaluation
 */
export const TRAIN_LEVELS = Object.freeze([1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13]);
export const TRAIN_SCENARIOS = Object.freeze(['nominal', 'nominal', 'push', 'gaussian', 'laplace', 'outlier', 'stiffness', 'friction', 'payload', 'delay']);
export const PHYSICS_STEPS_PER_TICK = 25;            // 500 Hz physics / 20 Hz control

/** Deterministic domain-randomised episode spec for (round, episode). */
export function trainSpec(round, episode) {
  let h = (round * 7919 + episode * 104729 + 12345) >>> 0;
  const next = () => { h = (Math.imul(h ^ (h >>> 15), 2246822507) + 0x9e3779b9) >>> 0; return h; };
  return { level: TRAIN_LEVELS[next() % TRAIN_LEVELS.length], scenario: TRAIN_SCENARIOS[next() % TRAIN_SCENARIOS.length], seed: 1000 + (next() % 100000) };
}

export const freshInner = () => ({ episodes: 0, realSteps: 0, rolloutSteps: 0, success: 0, progress: 0, speed: 0, slack: 0, rodMax: 0, slip: 0,
  maxOuter: 0, overload: 0, aborts: 0, tips: 0, ess: 0, advantage: 0, ret: 0, byLevel: {}, byScenario: {} });

/**
 * Add one finished episode (ResidualMppiTrainer.runEpisode result) to the round's
 * statistics. Success = the same pace as the benchmark (>= 1 m per 20 s).
 */
export function accumulateInner(inner, spec, result, { candidates, durationS }) {
  const d = result.digest;
  const real = result.ticks * PHYSICS_STEPS_PER_TICK;
  const ok = d.progressM >= durationS / 20 ? 1 : 0;
  inner.episodes++; inner.realSteps += real; inner.rolloutSteps += real * Math.max(0, candidates - 1);
  inner.success += ok; inner.progress += d.progressM; inner.speed += d.avgSpeedMs;
  inner.slack += d.relaxedCablesPct || 0; inner.rodMax = Math.max(inner.rodMax, d.rodResidualMm || 0); inner.slip += d.contactSlipMs || 0;
  inner.maxOuter = Math.max(inner.maxOuter, d.maxOuterTensionN || 0); inner.overload += (d.maxOuterTensionN || 0) > 180 ? 1 : 0;
  inner.aborts += d.aborts || 0; inner.tips += d.tips || 0;
  inner.ess += result.meanEss || 0; inner.advantage += result.meanAdvantage || 0; inner.ret += result.meanReturn || 0;
  for (const [bucket, key] of [[inner.byLevel, `L${spec.level}`], [inner.byScenario, spec.scenario]]) {
    const b = bucket[key] || (bucket[key] = { n: 0, s: 0, p: 0 });
    b.n++; b.s += ok; b.p += d.progressM;
  }
  return inner;
}

/** Aggregated failure modes of a round (sent to the LLM meta-controller as innerLoopSummary). */
export function innerSummary(inner) {
  const n = Math.max(1, inner.episodes);
  const table = bucket => Object.fromEntries(Object.entries(bucket).map(([k, v]) => [k, { episodes: v.n, successRatePct: 100 * v.s / v.n, meanProgressM: v.p / v.n }]));
  return {
    episodes: inner.episodes, realPhysicsSteps: inner.realSteps, rolloutPhysicsSteps: inner.rolloutSteps,
    successRatePct: 100 * inner.success / n, meanProgressM: inner.progress / n, meanSpeedMs: inner.speed / n,
    meanRelaxedCablesPct: inner.slack / n, maxRodResidualMm: inner.rodMax, meanSlipMs: inner.slip / n,
    maxOuterTensionN: inner.maxOuter, overloadEpisodes: inner.overload, aborts: inner.aborts, tips: inner.tips,
    meanEss: inner.ess / n, meanAdvantagePerPlan: inner.advantage / n, meanReturnPerPlan: inner.ret / n,
    byLevel: table(inner.byLevel), byScenario: table(inner.byScenario)
  };
}

/**
 * True when `m` has the structure of the rover's strut-end incidence matrix
 * (6 x 24, each row four +1 and four -1 entries, zeros elsewhere). A uniform or
 * random placeholder makes all six synergies identical, so callers fall back to
 * synergyMatrix(rover).
 */
export function isValidIncidence(m) {
  if (!m || m.length !== 6 * 24) return false;
  for (let r = 0; r < 6; r++) {
    let plus = 0, minus = 0;
    for (let c = 0; c < 24; c++) {
      const v = m[r * 24 + c];
      if (v === 1) plus++; else if (v === -1) minus++; else if (v !== 0) return false;
    }
    if (plus !== 4 || minus !== 4) return false;
  }
  return true;
}
