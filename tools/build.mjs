#!/usr/bin/env node
/**
 * Production build (no bundler needed: the app is native ES modules).
 *
 *   1. syntax-check every JS module (node --check equivalent via vm);
 *   2. verify that every relative import in js/ and every local asset in
 *      index.html resolves to a file (catches broken paths before deploy);
 *   3. copy the static site into dist/ (index.html, css/, js/, config/,
 *      public/) — the API lives in api/ and lib/ and is deployed by Vercel
 *      as a serverless function, or served by server.mjs locally.
 */
import { readFileSync, readdirSync, statSync, mkdirSync, rmSync, cpSync, existsSync, writeFileSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const dist = path.join(root, 'dist');
const errors = [];

const walk = dir => readdirSync(dir).flatMap(name => {
  const full = path.join(dir, name);
  return statSync(full).isDirectory() ? walk(full) : [full];
});

// 1 + 2: modules
const modules = [...walk(path.join(root, 'js')), ...walk(path.join(root, 'lib')), ...walk(path.join(root, 'api')),
  path.join(root, 'server.mjs')].filter(file => /\.(m?js)$/.test(file));
for (const file of modules) {
  const check = spawnSync(process.execPath, ['--check', file], { encoding: 'utf8' });
  if (check.status !== 0) errors.push(`syntax: ${path.relative(root, file)}\n${check.stderr}`);
  const source = readFileSync(file, 'utf8');
  const importRe = /(?:import|export)\s[^'"]*?from\s*['"](\.{1,2}\/[^'"]+)['"]|import\(\s*['"](\.{1,2}\/[^'"]+)['"]\s*\)/g;
  for (const match of source.matchAll(importRe)) {
    const spec = (match[1] || match[2]).split('?')[0];
    const target = path.resolve(path.dirname(file), spec);
    if (!existsSync(target)) errors.push(`import: ${path.relative(root, file)} -> ${spec} (missing)`);
  }
}

// 2: index.html local assets
const html = readFileSync(path.join(root, 'index.html'), 'utf8');
for (const match of html.matchAll(/(?:src|href)="(\.\/[^"?#]+)/g)) {
  if (!existsSync(path.join(root, match[1]))) errors.push(`asset: index.html -> ${match[1]} (missing)`);
}

if (errors.length) {
  console.error(`[build] ${errors.length} problem(s):\n${errors.join('\n')}`);
  process.exit(1);
}

// 3: copy
rmSync(dist, { recursive: true, force: true });
mkdirSync(dist, { recursive: true });
for (const entry of ['index.html', 'css', 'js', 'config', 'public', 'controller-suite.html']) {
  const from = path.join(root, entry);
  if (existsSync(from)) cpSync(from, path.join(dist, entry), { recursive: true });
}
writeFileSync(path.join(dist, 'build.json'), JSON.stringify({ builtAt: new Date().toISOString(), modules: modules.length }, null, 1));
console.log(`[build] ok: ${modules.length} modules checked, imports and assets resolve; static site in dist/`);
