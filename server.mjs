#!/usr/bin/env node
/**
 * Local / self-hosted server for the rover workstation (no dependencies).
 *
 *   node server.mjs                 serve the project folder
 *   ROOT=dist node server.mjs       serve the production build
 *   PORT=3100 HOST=0.0.0.0          defaults
 *
 * Static files + benchmark persistence API:
 *   GET    /api/benchmarks          -> { state, persisted, storage }
 *   POST   /api/benchmarks          body: state -> union-merged into
 *                                   ./data/benchmarks.json (atomic write)
 *   DELETE /api/benchmarks          header X-Confirm: clear -> tombstone
 *   GET    /api/health
 */
import http from 'node:http';
import { promises as fs, createReadStream } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createFileStore } from './lib/benchmarkFileStore.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(here, process.env.ROOT || '.');
const PORT = Number(process.env.PORT) || 3100;
const HOST = process.env.HOST || '0.0.0.0';
const DATA_FILE = path.resolve(here, process.env.BENCHMARK_FILE || 'data/benchmarks.json');
const MAX_BODY = 16*1024*1024;

const store = createFileStore(DATA_FILE);

const MIME = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.svg': 'image/svg+xml',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.ico': 'image/x-icon', '.pdf': 'application/pdf',
  '.txt': 'text/plain; charset=utf-8', '.md': 'text/markdown; charset=utf-8', '.csv': 'text/csv; charset=utf-8',
  '.woff2': 'font/woff2', '.map': 'application/json'
};
// Never serve server-side state, VCS metadata or dependencies as static files.
const BLOCKED = [/^\/data(\/|$)/, /^\/node_modules(\/|$)/, /(^|\/)\.[^/]/, /^\/lib(\/|$)/, /^\/server\.mjs$/];

function send(res, status, body, headers = {}) {
  const payload = typeof body === 'string' || Buffer.isBuffer(body) ? body : JSON.stringify(body);
  res.writeHead(status, {
    'Content-Type': typeof body === 'string' && !headers['Content-Type'] ? 'text/plain; charset=utf-8' : 'application/json; charset=utf-8',
    'Cache-Control': 'no-store',
    ...headers
  });
  res.end(payload);
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', chunk => {
      size += chunk.length;
      if (size > MAX_BODY) { reject(Object.assign(new Error('payload too large'), { status: 413 })); req.destroy(); return; }
      chunks.push(chunk);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });
}

async function handleApi(req, res, pathname) {
  if (pathname === '/api/health') return send(res, 200, { ok: true, storage: store.readOnlyReason ? 'read-only' : 'disk' });
  if (pathname !== '/api/benchmarks') return send(res, 404, { error: 'not found' });
  try {
    if (req.method === 'GET') {
      const state = await store.readState();
      return send(res, 200, { state, persisted: true, storage: store.readOnlyReason ? 'read-only' : 'disk' });
    }
    if (req.method === 'POST') {
      const text = await readBody(req);
      let input;
      try { input = JSON.parse(text || '{}'); } catch { return send(res, 400, { error: 'invalid JSON' }); }
      const result = await store.mergeAndWrite(input?.state || input);
      return send(res, 200, { state: result.state, persisted: result.persisted, reason: result.reason || null });
    }
    if (req.method === 'DELETE') {
      if (req.headers['x-confirm'] !== 'clear') return send(res, 400, { error: 'missing X-Confirm: clear' });
      const result = await store.clear();
      return send(res, 200, { state: result.state, persisted: result.persisted, reason: result.reason || null });
    }
    return send(res, 405, { error: 'method not allowed' }, { Allow: 'GET, POST, DELETE' });
  } catch (error) {
    console.error('[api] benchmarks:', error);
    return send(res, error.status || 500, { error: error.message || 'server error' });
  }
}

async function handleStatic(req, res, pathname) {
  if (req.method !== 'GET' && req.method !== 'HEAD') return send(res, 405, 'method not allowed');
  let decoded;
  try { decoded = decodeURIComponent(pathname); } catch { return send(res, 400, 'bad path'); }
  if (decoded.includes('\0') || BLOCKED.some(rule => rule.test(decoded))) return send(res, 404, 'not found');
  let file = path.resolve(ROOT, `.${decoded}`);
  if (file !== ROOT && !file.startsWith(ROOT+path.sep)) return send(res, 404, 'not found');
  try {
    let stat = await fs.stat(file);
    if (stat.isDirectory()) {
      file = path.join(file, 'index.html');
      stat = await fs.stat(file);
    }
    const ext = path.extname(file).toLowerCase();
    res.writeHead(200, {
      'Content-Type': MIME[ext] || 'application/octet-stream',
      'Content-Length': stat.size,
      // Modules are cache-busted with ?v=...; HTML must always revalidate.
      'Cache-Control': ext === '.html' ? 'no-cache' : 'public, max-age=60',
      'X-Content-Type-Options': 'nosniff'
    });
    if (req.method === 'HEAD') return res.end();
    createReadStream(file).pipe(res);
  } catch {
    send(res, 404, 'not found');
  }
}

const server = http.createServer((req, res) => {
  const { pathname } = new URL(req.url, 'http://localhost');
  if (pathname.startsWith('/api/')) return handleApi(req, res, pathname);
  return handleStatic(req, res, pathname);
});

server.on('error', error => {
  if (error.code === 'EADDRINUSE') {
    console.error(`[server] port ${PORT} is already in use. Start with another port: PORT=3200 npm start`);
    process.exit(1);
  }
  throw error;
});

server.listen(PORT, HOST, () => {
  const shown = HOST === '0.0.0.0' ? '127.0.0.1' : HOST;
  console.log(`[server] 6-bar tensegrity workstation: http://${shown}:${PORT}/`);
  console.log(`[server] serving ${ROOT}`);
  console.log(`[server] benchmark records: ${DATA_FILE}`);
});
