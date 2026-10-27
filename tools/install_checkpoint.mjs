#!/usr/bin/env node
/**
 * Install a downloaded browser-training checkpoint as a deployed controller.
 *
 *   node tools/install_checkpoint.mjs residual-mppi-checkpoint.json          # MPC + RL + LLM
 *   node tools/install_checkpoint.mjs residual-mppi-checkpoint.json --rl     # MPC + RL
 *
 * The file is validated (actor weight count, finite numbers, parameter bounds)
 * before js/residualMppiWeights.js (or js/residualMppiWeightsRl.js) is replaced.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { MorphologyActorCritic, synergyMatrix } from '../js/residualMppi.js';
import { BOUNDS } from '../js/llmMpcOptimizer.js';
import { SimConfig, SphericalRoverModel } from '../js/simEngine.js?v=20261027-ob1';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const file = process.argv[2];
const rl = process.argv.includes('--rl');
if (!file) { console.error('usage: node tools/install_checkpoint.mjs <checkpoint.json> [--rl]'); process.exit(1); }
const ckpt = JSON.parse(readFileSync(file, 'utf8'));
const model = new MorphologyActorCritic(synergyMatrix(new SphericalRoverModel(new SimConfig({}))));
if (!Array.isArray(ckpt.actor?.w) || !ckpt.actor.w.every(Number.isFinite)) throw new Error('actor weights missing or not finite');
// model.load() accepts the current 73-D layout and migrates 48-D checkpoints; any other size throws.
const checkParams = (params, where) => {
  for (const [key, [lo, hi]] of Object.entries(BOUNDS)) {
    const v = params?.[key];
    if (v !== undefined && (!Number.isFinite(v) || v < lo || v > hi)) throw new Error(`${where} parameter ${key} = ${v} outside [${lo}, ${hi}]`);
  }
};
checkParams(ckpt.params, 'global');
for (const [level, params] of Object.entries(ckpt.paramsByLevel || {})) {
  if (!(Number(level) >= 1 && Number(level) <= 14)) throw new Error(`unknown level ${level}`);
  checkParams(params, `level ${level}`);
}
model.load(ckpt);
const target = rl ? 'residualMppiWeightsRl.js' : 'residualMppiWeights.js';
const name = rl ? 'RESIDUAL_MPPI_WEIGHTS_RL' : 'RESIDUAL_MPPI_WEIGHTS';
writeFileSync(path.join(root, 'js', target),
  `/** Installed from ${path.basename(file)} by tools/install_checkpoint.mjs. null = untrained. */\nexport const ${name} = ${JSON.stringify({ actor: model.actor.toJSON(), params: ckpt.params || {}, ...(ckpt.paramsByLevel && { paramsByLevel: ckpt.paramsByLevel }), meta: ckpt.meta || {} })};\n`);
console.log(`installed ${path.basename(file)} -> js/${target}`);
