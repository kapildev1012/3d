import test from 'node:test';
import assert from 'node:assert/strict';
import {
  bothModelsReachedGoals,
  canStartNextTrainingAttempt,
  currentTrainingAttemptNumber
} from '../js/trainingCycle.js';

const running = { courseComplete: false, runOutcome: 'running' };
const timeout = { courseComplete: false, runOutcome: 'loss' };
const goal = { courseComplete: true, runOutcome: 'win' };

test('next training waits until both Model A and Model B reach their goals', () => {
  assert.equal(bothModelsReachedGoals(running, running), false);
  assert.equal(bothModelsReachedGoals(goal, running), false);
  assert.equal(bothModelsReachedGoals(running, goal), false);
  assert.equal(bothModelsReachedGoals(goal, timeout), false);
  assert.equal(bothModelsReachedGoals(timeout, goal), false);
  assert.equal(bothModelsReachedGoals(goal, goal), true);
});

test('automatic restart also requires enabled Level 10 or 14 training', () => {
  const base = {
    autoLearningEnabled: true,
    experimentId: 10,
    metricsA: goal,
    metricsB: goal
  };
  assert.equal(canStartNextTrainingAttempt(base), true);
  assert.equal(canStartNextTrainingAttempt({ ...base, experimentId: 14 }), true);
  assert.equal(canStartNextTrainingAttempt({ ...base, autoLearningEnabled: false }), false);
  assert.equal(canStartNextTrainingAttempt({ ...base, experimentId: 9 }), false);
  assert.equal(canStartNextTrainingAttempt({ ...base, metricsA: running }), false);
  assert.equal(canStartNextTrainingAttempt({ ...base, metricsB: timeout }), false);
});

test('attempt number advances only after the completed models are reset', () => {
  assert.equal(currentTrainingAttemptNumber(0, running), 1);
  assert.equal(currentTrainingAttemptNumber(20, { ...goal, runTerminal: true }), 20);
  assert.equal(currentTrainingAttemptNumber(20, { ...running, runTerminal: false }), 21);
});
