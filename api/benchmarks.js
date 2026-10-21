/**
 * Serverless variant of /api/benchmarks (Vercel Node runtime).
 *
 * Serverless filesystems are read-only (EROFS) and /tmp is not shared
 * between instances, so by default this endpoint is stateless: it merges
 * what the browser sends and returns it with { persisted: false }. Records
 * then live in each visitor's browser (localStorage + IndexedDB) and can be
 * exported/imported as JSON. Set BENCHMARK_FILE to a writable, persistent
 * path to enable disk persistence on a self-hosted function runtime.
 */
import { createFileStore } from '../lib/benchmarkFileStore.mjs';
import { emptyState, mergeStates } from '../js/benchmarkCore.js';

const store = process.env.BENCHMARK_FILE ? createFileStore(process.env.BENCHMARK_FILE) : null;

async function readJson(req) {
  if (req.body && typeof req.body === 'object') return req.body;
  if (typeof req.body === 'string') return JSON.parse(req.body || '{}');
  const chunks = [];
  for await (const chunk of req) chunks.push(chunk);
  return JSON.parse(Buffer.concat(chunks).toString('utf8') || '{}');
}

export default async function handler(req, res) {
  res.setHeader('Cache-Control', 'no-store');
  try {
    if (req.method === 'GET') {
      const state = store ? await store.readState() : emptyState();
      return res.status(200).json({ state, persisted: Boolean(store), storage: store ? 'disk' : 'browser-only' });
    }
    if (req.method === 'POST') {
      let input;
      try { input = await readJson(req); } catch { return res.status(400).json({ error: 'invalid JSON' }); }
      if (store) {
        const result = await store.mergeAndWrite(input?.state || input);
        return res.status(200).json({ state: result.state, persisted: result.persisted, reason: result.reason || null });
      }
      return res.status(200).json({ state: mergeStates(input?.state || input), persisted: false, reason: 'browser-only deployment' });
    }
    if (req.method === 'DELETE') {
      if (req.headers['x-confirm'] !== 'clear') return res.status(400).json({ error: 'missing X-Confirm: clear' });
      if (store) {
        const result = await store.clear();
        return res.status(200).json({ state: result.state, persisted: result.persisted });
      }
      return res.status(200).json({ state: { ...emptyState(), clearedAt: new Date().toISOString() }, persisted: false });
    }
    res.setHeader('Allow', 'GET, POST, DELETE');
    return res.status(405).json({ error: 'method not allowed' });
  } catch (error) {
    return res.status(500).json({ error: error.message || 'server error' });
  }
}
