/**
 * Disk layer of the benchmark store (Node only).
 *
 *   readState()          -> normalized state from ./data/benchmarks.json
 *   mergeAndWrite(input) -> union-merge input into the file, atomically
 *   clear()              -> write a tombstone (clearedAt = now, no runs)
 *
 * Writes are atomic (temp file in the same directory + rename) and
 * serialized through one promise chain, so concurrent POSTs from several
 * tabs never interleave or truncate the file. A read-only filesystem
 * (EROFS / EACCES / EPERM, e.g. serverless deployments) never throws:
 * the call reports { persisted: false } and the browser layers keep the data.
 */
import { promises as fs } from 'node:fs';
import path from 'node:path';
import { emptyState, mergeStates, normalizeState } from '../js/benchmarkCore.js';

const READ_ONLY_CODES = new Set(['EROFS', 'EACCES', 'EPERM', 'ENOSPC']);

export function createFileStore(file) {
  let chain = Promise.resolve();
  let readOnlyReason = null;

  async function readState() {
    try {
      const text = await fs.readFile(file, 'utf8');
      return normalizeState(JSON.parse(text));
    } catch (error) {
      if (error.code === 'ENOENT') return emptyState();
      if (error instanceof SyntaxError) {
        // Corrupt file: keep a copy for inspection and start from empty
        // rather than failing every request.
        try { await fs.copyFile(file, `${file}.corrupt-${Date.now()}`); } catch { /* ignore */ }
        return emptyState();
      }
      throw error;
    }
  }

  async function writeAtomic(state) {
    if (readOnlyReason) return { persisted: false, reason: readOnlyReason };
    try {
      await fs.mkdir(path.dirname(file), { recursive: true });
      const temp = `${file}.${process.pid}.${Date.now()}.tmp`;
      await fs.writeFile(temp, JSON.stringify(state, null, 1));
      await fs.rename(temp, file);
      return { persisted: true };
    } catch (error) {
      if (READ_ONLY_CODES.has(error.code)) {
        readOnlyReason = `${error.code}: server storage is read-only`;
        return { persisted: false, reason: readOnlyReason };
      }
      throw error;
    }
  }

  const serialize = task => {
    const next = chain.then(task, task);
    chain = next.catch(() => {});
    return next;
  };

  return {
    file,
    readState,
    mergeAndWrite(input) {
      return serialize(async () => {
        const current = await readState();
        const merged = mergeStates(current, input);
        merged.updatedAt = new Date().toISOString();
        const result = await writeAtomic(merged);
        return { state: merged, ...result };
      });
    },
    clear() {
      return serialize(async () => {
        const state = { ...emptyState(), clearedAt: new Date().toISOString(), updatedAt: new Date().toISOString() };
        const result = await writeAtomic(state);
        return { state, ...result };
      });
    },
    get readOnlyReason() { return readOnlyReason; }
  };
}
