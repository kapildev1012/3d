#!/usr/bin/env node
/**
 * Summarize the controller league (node tools/run_experiments.mjs --only league):
 * every admissible cable-only controller on the same levels / scenarios / seeds.
 * Writes results/league/league.md and results/league/league.json.
 *
 * Ranking: success rate first (advanced >= 1 m in 20 s), then mean progress.
 * RMSE is reported but not ranked on its own: a controller that barely moves
 * has a tiny cross-track error. CoT (energy per weight and metre) is only
 * meaningful for controllers that actually travel.
 */
import { readdirSync, readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const runsDir = path.join(root, 'results', 'paper', 'runs');
const outDir = path.join(root, 'results', 'league');
const LABEL = {
  sgt: 'SGT', heuristic: 'Heuristic', lqr: 'LQR', ilqr: 'iLQR', mpc: 'QP-MPC', mpcGnss: 'MPC + GNSS', guidedPolicy: 'MPC + GPS',
  sgtShield: 'SGT + shield', policyShield: 'MPC + GPS + shield', residualMppi: 'MPC + RL', residualMppiLlm: 'MPC + RL + LLM'
};

const runs = readdirSync(runsDir).filter(f => f.startsWith('league__')).map(f => JSON.parse(readFileSync(path.join(runsDir, f), 'utf8')));
if (!runs.length) { console.error('no league runs found'); process.exit(1); }
const mean = xs => { const v = xs.filter(Number.isFinite); return v.length ? v.reduce((a, b) => a + b, 0) / v.length : NaN; };
const byMethod = new Map();
for (const r of runs) {
  const key = r.spec.method;
  if (!byMethod.has(key)) byMethod.set(key, []);
  byMethod.get(key).push(r);
}
const rows = [...byMethod].map(([method, list]) => {
  const moving = list.filter(r => r.metrics.progress > 0.05);
  return {
    method, label: LABEL[method] || method, runs: list.length,
    successPct: 100 * mean(list.map(r => r.metrics.success)),
    progressM: mean(list.map(r => r.metrics.progress)),
    speedMs: mean(list.map(r => r.metrics.speed)),
    rmseM: mean(list.map(r => r.metrics.rmse)),
    cot: mean(moving.map(r => r.metrics.cot)),
    effort: mean(list.map(r => r.metrics.effort)),
    solveMs: mean(list.map(r => r.metrics.solveMsMean)),
    maxRodErrMm: 1000 * Math.max(...list.map(r => r.metrics.maxBarError || 0)),
    maxEpsJ: Math.max(...list.map(r => r.metrics.maxResidual || 0)),
    assistShare: Math.max(...list.map(r => r.metrics.assistShare || 0))
  };
}).sort((a, b) => b.successPct - a.successPct || b.progressM - a.progressM);

// Per-cell winners (level x scenario): who travelled furthest.
const cells = new Map();
for (const r of runs) {
  const key = `L${r.spec.level} ${r.spec.scenario}`;
  if (!cells.has(key)) cells.set(key, []);
  cells.get(key).push(r);
}
const cellRows = [...cells].sort().map(([cell, list]) => {
  const sorted = list.slice().sort((a, b) => b.metrics.progress - a.metrics.progress);
  const pick = m => list.find(r => r.spec.method === m)?.metrics.progress;
  const bestOther = sorted.find(r => !r.spec.method.startsWith('residual'));
  return { cell, winner: LABEL[sorted[0].spec.method], winnerProgress: sorted[0].metrics.progress,
    mpcRl: pick('residualMppi'), mpcRlLlm: pick('residualMppiLlm'), bestOther: LABEL[bestOther?.spec.method], bestOtherProgress: bestOther?.metrics.progress };
});

const f = (x, d = 3) => (Number.isFinite(x) ? x.toFixed(d) : '–');
let md = `# Controller league (cable-only, ${runs.length} runs, 20 s each, solid obstacles on)\n\n`;
md += '| Rank | Controller | Runs | Success (%) | Progress (m) | v̄ (m/s) | RMSE (m) | CoT | ∫uᵀu | t_solve (ms) | max rod err (mm) | max \\|ε\\| (J) | assist |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|\n';
rows.forEach((r, i) => {
  md += `| ${i + 1} | ${r.label} | ${r.runs} | ${f(r.successPct, 0)} | ${f(r.progressM, 2)} | ${f(r.speedMs)} | ${f(r.rmseM)} | ${f(r.cot, 1)} | ${f(r.effort, 3)} | ${f(r.solveMs, 2)} | ${f(r.maxRodErrMm, 4)} | ${r.maxEpsJ.toExponential(1)} | ${r.assistShare} |\n`;
});
md += '\nRanking: success rate (advanced ≥ 1 m), then mean progress. RMSE alone favours controllers that barely move.\n\n';
md += '## Per cell: furthest progress\n\n| Cell | Winner | Progress (m) | MPC + RL | MPC + RL + LLM | Best non-RL | its progress |\n|---|---|---|---|---|---|---|\n';
for (const c of cellRows) md += `| ${c.cell} | ${c.winner} | ${f(c.winnerProgress, 2)} | ${f(c.mpcRl, 2)} | ${f(c.mpcRlLlm, 2)} | ${c.bestOther} | ${f(c.bestOtherProgress, 2)} |\n`;
mkdirSync(outDir, { recursive: true });
writeFileSync(path.join(outDir, 'league.md'), md);
writeFileSync(path.join(outDir, 'league.json'), JSON.stringify({ rows, cells: cellRows }, null, 1));
console.log(md);
