export function bothModelsReachedGoals(metricsA, metricsB) {
  return [metricsA, metricsB].every(metrics =>
    metrics?.courseComplete === true && metrics?.runOutcome === 'win');
}

export function canStartNextTrainingAttempt({
  autoLearningEnabled,
  experimentId,
  metricsA,
  metricsB
}) {
  return autoLearningEnabled === true &&
    (experimentId === 10 || experimentId === 14) &&
    bothModelsReachedGoals(metricsA, metricsB);
}

export function currentTrainingAttemptNumber(runCount, metricsB) {
  const completedRuns = Math.max(0, Number.isFinite(runCount) ? Math.floor(runCount) : 0);
  return Math.max(1, completedRuns+(metricsB?.runTerminal === true ? 0 : 1));
}
