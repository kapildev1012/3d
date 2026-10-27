/**
 * BENCHMARKPERSISTENCE.JS - three storage layers for the 14-level matrix
 *
 *   Layer 1  localStorage 'tensegrity_benchmarks_v2'  synchronous, instant
 *            hydration on page load; mirrored to IndexedDB (no 5 MB cap).
 *   Layer 2  /api/benchmarks -> ./data/benchmarks.json on the local Node
 *            server (atomic writes); survives clearing browser data,
 *            switching browsers and reboots. On read-only (serverless)
 *            deployments the API answers persisted:false and the browser
 *            layers carry the records.
 *   Layer 3  public/default_benchmarks.json seed (loaded only into an empty
 *            store) plus JSON/CSV export and JSON import.
 *
 * All layers merge with benchmarkCore.mergeStates (union of run ids, newest
 * record wins, clear-tombstone respected), so hydration from any subset of
 * layers converges to the same state.
 */
import {
  emptyState, mergeStates, normalizeRun, normalizeState, migrateLegacyRecords, toCSV
} from './benchmarkCore.js?v=20261027-ob1';

export const STORAGE_KEY = 'tensegrity_benchmarks_v2';
const LEGACY_KEYS = ['tensegrity-experiment-records-v1', 'tensegrity_rover_benchmarks_v1'];
const IDB_NAME = 'tensegrity_benchmarks';
const IDB_STORE = 'kv';
const IDB_KEY = 'state';
const SERVER_DEBOUNCE_MS = 2500;
const LOCAL_TRIM_RUNS = 4000; // localStorage fallback size when the quota is hit

function openIdb() {
  return new Promise((resolve, reject) => {
    if (typeof indexedDB === 'undefined') { reject(new Error('IndexedDB unavailable')); return; }
    const request = indexedDB.open(IDB_NAME, 1);
    request.onupgradeneeded = () => request.result.createObjectStore(IDB_STORE);
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
    request.onblocked = () => reject(new Error('IndexedDB blocked'));
  });
}

function idbTransaction(db, mode, run) {
  return new Promise((resolve, reject) => {
    const tx = db.transaction(IDB_STORE, mode);
    const result = run(tx.objectStore(IDB_STORE));
    tx.oncomplete = () => resolve(result?.result);
    tx.onerror = () => reject(tx.error);
    tx.onabort = () => reject(tx.error);
  });
}

export class BenchmarkPersistence {
  constructor({ apiUrl = './api/benchmarks', seedUrl = './public/default_benchmarks.json', onChange = null } = {}) {
    this.apiUrl = apiUrl;
    this.seedUrl = seedUrl;
    this.onChange = onChange;
    this.state = emptyState();
    this.status = { local: 'pending', idb: 'pending', server: 'pending', serverReason: null, lastServerSync: null, seeded: false };
    this.serverTimer = null;
    this.idbTimer = null;
    this.dbPromise = null;
  }

  get storage() {
    try { return window.localStorage; } catch { return null; }
  }

  // ---------------------------------------------------------------- hydrate

  /** Synchronous: localStorage v2 + migrated v1 records. Call before first paint. */
  hydrateSync() {
    const storage = this.storage;
    const layers = [];
    try {
      const raw = storage?.getItem(STORAGE_KEY);
      if (raw) layers.push(JSON.parse(raw));
      this.status.local = storage ? 'ok' : 'unavailable';
    } catch {
      this.status.local = 'unavailable';
    }
    for (const key of LEGACY_KEYS) {
      try {
        const raw = storage?.getItem(key);
        if (!raw) continue;
        const parsed = JSON.parse(raw);
        // v1 records store: { entries, snapshots }; a v2-shaped mirror under
        // the spec's alternative key is merged as-is.
        layers.push(Array.isArray(parsed?.runs) ? parsed : { runs: migrateLegacyRecords(parsed) });
      } catch { /* unreadable legacy key: ignore */ }
    }
    this.state = mergeStates(...layers);
    this.saveLocal();
    return this.state;
  }

  /** Asynchronous: IndexedDB + server disk + (empty store only) seed. */
  async hydrateAsync() {
    const before = this.fingerprint();
    const [idbState, serverState] = await Promise.all([this.readIdb(), this.readServer()]);
    this.state = mergeStates(this.state, idbState, serverState);
    if (!this.state.runs.length && !this.state.clearedAt) {
      const seed = await this.readSeed();
      if (seed?.runs?.length) {
        this.state = mergeStates(this.state, seed);
        this.status.seeded = true;
      }
    }
    this.saveLocal();
    this.scheduleIdb(0);
    // Push the union back so the disk file also learns about browser-only runs.
    if (this.status.server === 'disk') this.scheduleServer(0);
    if (this.fingerprint() !== before) this.emit();
    return this.state;
  }

  async readIdb() {
    try {
      this.dbPromise ||= openIdb();
      const db = await this.dbPromise;
      const value = await idbTransaction(db, 'readonly', store => store.get(IDB_KEY));
      this.status.idb = 'ok';
      return value || null;
    } catch {
      this.status.idb = 'unavailable';
      return null;
    }
  }

  async readServer() {
    try {
      const response = await fetch(this.apiUrl, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const body = await response.json();
      this.status.server = body.storage === 'disk' && body.persisted !== false ? 'disk'
        : body.storage === 'read-only' ? 'read-only' : 'browser-only';
      this.status.lastServerSync = new Date().toISOString();
      return body.state || null;
    } catch {
      // Static hosting without the API (or offline): browser layers only.
      this.status.server = 'offline';
      return null;
    }
  }

  async readSeed() {
    try {
      const response = await fetch(this.seedUrl, { cache: 'no-store' });
      if (!response.ok) return null;
      const seed = normalizeState(await response.json());
      seed.runs.forEach(run => { run.source = 'seed'; });
      return seed;
    } catch {
      return null;
    }
  }

  // ---------------------------------------------------------------- write

  fingerprint() {
    const runs = this.state.runs;
    return `${runs.length}|${this.state.clearedAt}|${runs.length ? runs[runs.length-1].updatedAt : ''}|`+
      `${runs.reduce((sum, run) => sum+Date.parse(run.updatedAt) % 100000, 0)}`;
  }

  emit() {
    try { this.onChange?.(this.state, this.status); } catch (error) { console.error(error); }
  }

  /** Insert or update one run in every layer. flush=true pushes to disk now. */
  upsertRun(run, { flush = false } = {}) {
    const normalized = normalizeRun(run);
    if (!normalized) return false;
    const index = this.state.runs.findIndex(item => item.id === normalized.id);
    if (index >= 0) this.state.runs[index] = normalized;
    else this.state.runs.push(normalized);
    this.state.updatedAt = new Date().toISOString();
    this.saveLocal();
    this.scheduleIdb(flush ? 0 : 400);
    this.scheduleServer(flush ? 0 : SERVER_DEBOUNCE_MS);
    this.emit();
    return true;
  }

  saveLocal() {
    const storage = this.storage;
    if (!storage) { this.status.local = 'unavailable'; return; }
    try {
      storage.setItem(STORAGE_KEY, JSON.stringify(this.state));
      this.status.local = 'ok';
    } catch {
      // Quota: keep the newest runs here; IndexedDB and disk hold everything.
      try {
        const trimmed = { ...this.state, runs: this.state.runs.slice(-LOCAL_TRIM_RUNS) };
        storage.setItem(STORAGE_KEY, JSON.stringify(trimmed));
        this.status.local = 'trimmed';
      } catch {
        this.status.local = 'full';
      }
    }
  }

  scheduleIdb(delay) {
    clearTimeout(this.idbTimer);
    this.idbTimer = setTimeout(() => this.writeIdb(), delay);
  }

  async writeIdb() {
    try {
      this.dbPromise ||= openIdb();
      const db = await this.dbPromise;
      await idbTransaction(db, 'readwrite', store => store.put(this.state, IDB_KEY));
      this.status.idb = 'ok';
    } catch {
      this.status.idb = 'unavailable';
    }
  }

  scheduleServer(delay) {
    if (this.status.server === 'offline' && delay > 0) return; // retried on the next flush
    clearTimeout(this.serverTimer);
    this.serverTimer = setTimeout(() => this.writeServer(), delay);
  }

  async writeServer() {
    try {
      const response = await fetch(this.apiUrl, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ state: this.state })
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const body = await response.json();
      this.status.server = body.persisted ? 'disk' : (body.reason?.includes('read-only') ? 'read-only' : 'browser-only');
      this.status.serverReason = body.reason || null;
      this.status.lastServerSync = new Date().toISOString();
      if (body.state) {
        const before = this.fingerprint();
        this.state = mergeStates(this.state, body.state);
        if (this.fingerprint() !== before) { this.saveLocal(); this.scheduleIdb(200); this.emit(); }
      }
    } catch {
      this.status.server = 'offline';
    }
  }

  /** pagehide / beforeunload: synchronous local save + best-effort beacon to disk. */
  flushOnUnload() {
    this.saveLocal();
    clearTimeout(this.serverTimer);
    try {
      if (navigator.sendBeacon && this.status.server !== 'offline') {
        const blob = new Blob([JSON.stringify({ state: this.state })], { type: 'application/json' });
        navigator.sendBeacon(this.apiUrl, blob);
      }
    } catch { /* page is closing; local copy is already saved */ }
    this.writeIdb();
  }

  // ---------------------------------------------------------------- user actions

  async clearAll() {
    this.state = { ...emptyState(), clearedAt: new Date().toISOString(), updatedAt: new Date().toISOString() };
    this.saveLocal();
    for (const key of LEGACY_KEYS) { try { this.storage?.removeItem(key); } catch { /* ignore */ } }
    await this.writeIdb();
    try {
      await fetch(this.apiUrl, { method: 'DELETE', headers: { 'X-Confirm': 'clear' } });
    } catch { /* offline: the tombstone is pushed with the next sync */ }
    this.emit();
  }

  exportJSON() {
    return JSON.stringify({ ...this.state, exportedAt: new Date().toISOString() }, null, 2);
  }

  exportCSV() {
    return toCSV(this.state);
  }

  /** Merge an exported file. Returns the number of runs added or updated. */
  importJSON(text) {
    const incoming = normalizeState(JSON.parse(text));
    incoming.clearedAt = null; // an import never deletes local records
    // An explicit import must survive an earlier "Clear": runs older than the
    // local tombstone are re-stamped so the tombstone does not drop them.
    const cut = Date.parse(this.state.clearedAt || '') || 0;
    const now = new Date().toISOString();
    incoming.runs.forEach(run => {
      run.source = run.source === 'live' ? 'import' : run.source;
      if (Date.parse(run.updatedAt) <= cut) run.updatedAt = now;
    });
    const before = new Map(this.state.runs.map(run => [run.id, run.updatedAt]));
    this.state = mergeStates(this.state, incoming);
    const changed = this.state.runs.filter(run => before.get(run.id) !== run.updatedAt).length;
    this.saveLocal();
    this.scheduleIdb(0);
    this.scheduleServer(0);
    this.emit();
    return changed;
  }
}
