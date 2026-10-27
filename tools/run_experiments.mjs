#!/usr/bin/env node
/**
 * Paper experiment batch: nominal comparison, generalization, stress tests
 * and ablations, all through js/experimentProtocol.js.
 *
 *   node tools/run_experiments.mjs [--workers 2] [--duration 20] [--only nominal,stress]
 *   node tools/run_experiments.mjs --only hybrid        # SGT vs SGT+shield vs residual-MPPI actor
 *   node tools/run_experiments.mjs --train-hybrid ...   # worker pool + residual-MPPI + LLM co-designer
 *
 * Results are written incrementally to results/paper/runs/<id>.json, so an
 * interrupted batch resumes where it stopped. tools/summarize_experiments.mjs
 * turns them into LaTeX tables and figure data.
 */
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import { mkdirSync, existsSync, writeFileSync, readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import os from 'node:os';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '..');
const outDir = path.join(root, 'results', 'paper', 'runs');

export function jobList(duration = 20, { includeHybrid = false, includeLeague = false, leagueLevels = [1, 4], leagueScenarios = ['nominal', 'push', 'gaussian', 'friction', 'payload', 'delay'], leagueSeeds = [1] } = {}) {
  const jobs = [];
  const add = (group, spec) => {
    const id = [group, spec.method, spec.scenario || 'nominal', spec.ablation || 'full', `L${spec.level || 1}`, `s${spec.seed}`].join('__');
    jobs.push({ id, group, spec: { level: 1, scenario: 'nominal', ...spec }, duration });
  };
  for (const method of ['sgt', 'heuristic', 'lqr', 'ilqr', 'mpc', 'legacyAssisted']) {
    for (const seed of [1, 2, 3, 4, 5]) add('nominal', { method, seed });
  }
  for (const method of ['sgt', 'heuristic']) {
    for (const level of [4, 7]) for (const seed of [1, 2, 3]) add('generalization', { method, level, seed });
  }
  for (const scenario of ['impulse', 'push', 'gaussian', 'laplace', 'outlier', 'stiffness', 'friction', 'payload', 'delay']) {
    for (const method of ['sgt', 'heuristic']) for (const seed of [1, 2, 3]) add('stress', { method, scenario, seed });
  }
  for (const ablation of ['noPayload', 'noFeasibility', 'noGeometric', 'sparse8', 'noEnergyRow', 'legacyCore']) {
    for (const seed of [1, 2, 3]) add('ablation', { method: 'sgt', ablation, seed });
  }
  // Hybrid pipeline evaluation (only with --only hybrid / --include-hybrid): the deployed
  // residual-MPPI actor against SGT and SGT + shield on the same protocol.
  if (includeHybrid) {
    for (const method of ['sgt', 'sgtShield', 'residualMppi', 'residualMppiLlm']) {
      for (const scenario of ['nominal', 'push', 'friction', 'delay']) {
        for (const seed of [1, 2, 3]) add('hybrid', { method, scenario, seed });
      }
    }
  }
  // Controller league (--only league): every admissible cable-only controller on the
  // same levels/scenarios/seeds, so MPC + RL and MPC + RL + LLM are compared fairly.
  if (includeLeague) {
    const methods = ['sgt', 'heuristic', 'lqr', 'ilqr', 'mpc', 'mpcGnss', 'guidedPolicy', 'sgtShield', 'policyShield', 'residualMppi', 'residualMppiLlm'];
    for (const method of methods) for (const level of leagueLevels) for (const scenario of leagueScenarios) for (const seed of leagueSeeds) add('league', { method, scenario, level, seed });
  }
  return jobs;
}

if (isMainThread) {
  const args = process.argv.slice(2);
  const opt = name => { const i = args.indexOf(`--${name}`); return i >= 0 ? args[i+1] : null; };
  const workers = Number(opt('workers') || Math.max(1, Math.min(4, os.cpus().length)));
  const duration = Number(opt('duration') || 20);
  const only = opt('only') ? new Set(opt('only').split(',')) : null;
  // npm run experiments -- --train-hybrid [hybrid options]: run the three-tier training pipeline
  // (worker pool + residual-MPPI + LLM co-designer) instead of the paper batch.
  if (args.includes('--train-hybrid')) {
    const rest = args.filter(a => a !== '--train-hybrid');
    const { spawnSync } = await import('node:child_process');
    const run = spawnSync(process.execPath, [path.join(here, 'run_hybrid_training.mjs'), ...rest], { stdio: 'inherit' });
    process.exit(run.status ?? 1);
  }
  const includeHybrid = args.includes('--include-hybrid') || Boolean(only?.has('hybrid'));
  mkdirSync(outDir, { recursive: true });
  const includeLeague = Boolean(only?.has('league'));
  const list = name => opt(name)?.split(',');
  const jobs = jobList(duration, { includeHybrid, includeLeague,
    ...(list('levels') && { leagueLevels: list('levels').map(Number) }), ...(list('scenarios') && { leagueScenarios: list('scenarios') }), ...(list('seeds') && { leagueSeeds: list('seeds').map(Number) }) }).filter(job => (!only || only.has(job.group)) && !existsSync(path.join(outDir, `${job.id}.json`)));
  console.log(`[experiments] ${jobs.length} runs to do, ${workers} worker(s), ${duration} s each`);
  let next = 0, done = 0;
  const started = Date.now();
  const spawn = () => {
    if (next >= jobs.length) return null;
    const job = jobs[next++];
    return new Promise(resolve => {
      const worker = new Worker(fileURLToPath(import.meta.url), { workerData: job });
      worker.on('message', result => {
        writeFileSync(path.join(outDir, `${job.id}.json`), JSON.stringify(result));
        done++;
        const m = result.metrics;
        console.log(`[${done}/${jobs.length}] ${job.id}  dy=${m.progress.toFixed(2)} m  |eps|max=${m.maxResidual.toExponential(1)}  ${((Date.now()-started)/60000).toFixed(1)} min`);
      });
      worker.on('error', error => { console.error(`[error] ${job.id}: ${error.stack || error}`); });
      worker.on('exit', () => resolve(spawn()));
    });
  };
  const pool = [];
  for (let w = 0; w < workers; w++) pool.push(spawn());
  await Promise.all(pool.filter(Boolean));
  console.log(`[experiments] finished in ${((Date.now()-started)/60000).toFixed(1)} min`);
} else {
  const { runExperiment } = await import('../js/experimentProtocol.js');
  const originalInfo = console.info;
  console.info = () => {};
  const result = await runExperiment(workerData.spec, { duration: workerData.duration });
  console.info = originalInfo;
  parentPort.postMessage({ id: workerData.id, group: workerData.group, ...result });
}
