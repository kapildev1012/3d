#!/usr/bin/env node
/**
 * Tension-certified safety filter study: {SGT, learned policy} × {filter off, on}
 * over seeds, terrain and disturbance scenarios (cable-only DAE, 20 s runs).
 *
 *   node tools/run_shield_study.mjs [--workers 2] [--seconds 20]
 *
 * Writes results/shield_study/{runs.json,summary.json,summary.md}.
 */
import { spawn } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const arg = (name, fallback) => { const i = process.argv.indexOf(`--${name}`); return i > 0 ? process.argv[i+1] : fallback; };
const workers = Number(arg('workers', 2));
const seconds = arg('seconds', '20');
const out = path.join(root, 'results', 'shield_study');
mkdirSync(out, { recursive: true });

const conditions = [
  { id: 'L1-s1', scenario: 'nominal', level: 1, seed: 1 },
  { id: 'L1-s2', scenario: 'nominal', level: 1, seed: 2 },
  { id: 'L1-s3', scenario: 'nominal', level: 1, seed: 3 },
  { id: 'L3-s1', scenario: 'nominal', level: 3, seed: 1 },
  { id: 'L3-s2', scenario: 'nominal', level: 3, seed: 2 },
  { id: 'push', scenario: 'push', level: 1, seed: 1 },
  { id: 'impulse', scenario: 'impulse', level: 1, seed: 1 },
  { id: 'payload+30%', scenario: 'payload', level: 1, seed: 1 },
  { id: 'delay 100ms', scenario: 'delay', level: 1, seed: 1 },
  { id: 'k-20%', scenario: 'stiffness', level: 1, seed: 1 }
];
const controllers = [['sgt_tipping', 'SGT'], ['guided_policy', 'MPC + GPS (learned policy)']];
const jobs = [];
for (const c of conditions) for (const [mode] of controllers) for (const shield of ['off', 'on']) jobs.push({ ...c, mode, shield });

const run = job => new Promise((resolve, reject) => {
  const child = spawn(process.execPath, [path.join(root, 'tools', 'shield_metrics.mjs'), job.mode, job.shield, job.scenario, String(job.level), seconds, String(job.seed)],
    { stdio: ['ignore', 'pipe', 'inherit'] });
  let text = '';
  child.stdout.on('data', chunk => { text += chunk; });
  child.on('exit', code => (code === 0 ? resolve({ ...JSON.parse(text.trim().split('\n').pop()), condition: job.id }) : reject(new Error(`${job.id} ${job.mode} ${job.shield}: exit ${code}`))));
});
const results = new Array(jobs.length);
let next = 0, done = 0;
await Promise.all(Array.from({ length: workers }, async () => {
  while (next < jobs.length) {
    const i = next++;
    results[i] = await run(jobs[i]);
    console.log(`[shield] ${++done}/${jobs.length} ${jobs[i].id} ${jobs[i].mode} ${jobs[i].shield}: Δy ${results[i].progress.toFixed(2)} m, slack ${results[i].slackPct.toFixed(1)} %`);
  }
}));
writeFileSync(path.join(out, 'runs.json'), JSON.stringify(results, null, 1));

const metrics = [
  ['progress', 'Progress (m)', 'max'], ['slackPct', 'Payload-cable slack time (%)', 'min'],
  ['meanCoreTension', 'Mean payload-cable tension (N)', 'max'], ['minClearance', 'Min payload–rod clearance (m)', 'max'],
  ['rattleMm', 'Payload rattle RMS (mm)', 'min'], ['payloadAccelP99g', 'Payload accel. 99th pct (g)', 'min'],
  ['maxOuterTension', 'Max outer-cable tension (N)', 'min'], ['aborts', 'Aborted tip cycles', 'min'], ['controllerMs', 'Controller time (ms/tick)', 'min']
];
const stats = values => {
  const mean = values.reduce((s, v) => s+v, 0)/values.length;
  return { mean, std: Math.sqrt(values.reduce((s, v) => s+(v-mean)**2, 0)/Math.max(1, values.length-1)) };
};
const summary = {};
for (const [mode, label] of controllers) {
  summary[label] = {};
  for (const [key, name, better] of metrics) {
    const off = conditions.map(c => results.find(r => r.condition === c.id && r.mode === mode && r.shield === 'off')[key]);
    const on = conditions.map(c => results.find(r => r.condition === c.id && r.mode === mode && r.shield === 'on')[key]);
    const improved = on.filter((v, i) => (better === 'max' ? v > off[i] : v < off[i])).length;
    summary[label][key] = { name, off: stats(off), on: stats(on), improvedIn: improved, of: conditions.length };
  }
}
writeFileSync(path.join(out, 'summary.json'), JSON.stringify({ conditions, seconds: Number(seconds), summary }, null, 1));
const fmt = s => `${s.mean.toFixed(3)} ± ${s.std.toFixed(3)}`;
let md = `# Tension-certified safety filter study\n\n${conditions.length} conditions (${conditions.map(c => c.id).join(', ')}), ${seconds} s cable-only runs, mean ± std over conditions; "better in" counts paired conditions where the filter improved the metric.\n`;
for (const [, label] of controllers) {
  md += `\n## ${label}\n\n| Metric | Filter off | Filter on | Better in |\n|---|---|---|---|\n`;
  for (const [key] of metrics) { const m = summary[label][key]; md += `| ${m.name} | ${fmt(m.off)} | ${fmt(m.on)} | ${m.improvedIn}/${m.of} |\n`; }
}
writeFileSync(path.join(out, 'summary.md'), md);
console.log(md);
