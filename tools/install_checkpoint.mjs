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
if (!ckpt.actor?.w || ckpt.actor.w.length !== model.actor.w.length) throw new Error(`actor has ${ckpt.actor?.w?.length} weights, expected ${model.actor.w.length}`);
if (!ckpt.actor.w.every(Number.isFinite)) throw new Error('actor weights must be finite');
for (const [key, [lo, hi]] of Object.entries(BOUNDS)) {
  const v = ckpt.params?.[key];
  if (v !== undefined && (!Number.isFinite(v) || v < lo || v > hi)) throw new Error(`parameter ${key} = ${v} outside [${lo}, ${hi}]`);
}
model.load(ckpt);
const target = rl ? 'residualMppiWeightsRl.js' : 'residualMppiWeights.js';
const name = rl ? 'RESIDUAL_MPPI_WEIGHTS_RL' : 'RESIDUAL_MPPI_WEIGHTS';
writeFileSync(path.join(root, 'js', target),
  `/** Installed from ${path.basename(file)} by tools/install_checkpoint.mjs. null = untrained. */\nexport const ${name} = ${JSON.stringify({ actor: model.actor.toJSON(), params: ckpt.params || {}, meta: ckpt.meta || {} })};\n`);
console.log(`installed ${path.basename(file)} -> js/${target}`);
