/**
 * BENCHMARKMATRIX.JS - the 14-level performance matrix
 *
 * render()      full rebuild from the persisted store (on any store change)
 * updateLive()  rewrites only the active level's cells from the running
 *               simulation (called ~4 Hz), so expanded history drawers,
 *               hover and focus are never destroyed by the live stream.
 */
import {
  summarizeLevel, displayStatus, formatRunSummary, primaryMetrics, LEVEL_COUNT
} from './benchmarkCore.js?v=20261021-ws1';
import { LEVELS } from './levels.js?v=20261021-ws1';

const esc = text => String(text ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmt = (value, digits) => (Number.isFinite(value) ? value.toFixed(digits) : '—');
const stamp = iso => {
  const date = new Date(iso);
  return Number.isFinite(date.getTime())
    ? date.toLocaleString(undefined, { day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit' })
    : '';
};
const STATUS_CLASS = {
  'ACTIVE LIVE': 'status-live', COMPLETED: 'status-completed', FAILED: 'status-failed',
  INCOMPLETE: 'status-incomplete', 'NOT RUN': 'status-notrun', PAUSED: 'status-incomplete'
};
const statusBadge = label => `<span class="status ${STATUS_CLASS[label] || ''}">${label}</span>`;
const REASONS = {
  goal: 'goal reached', deadline: 'mission deadline', 'time-limit': 'level time limit', stall: 'stalled',
  reset: 'reset', 'level-switch': 'level switched', 'controller-change': 'controller changed',
  'physics-change': 'physics changed', preset: 'preset applied', unload: 'page closed', rebuild: 'rover rebuilt',
  'parameter-change': 'parameters changed', 'auto-benchmark': 'auto-benchmark', 'seed-duration': 'seed run',
  'experiment-apply': 'experiment applied', 'mode-change': 'mode changed', legacy: 'imported (v1)', snapshot: 'v1 snapshot'
};

function recordCell(run, kind) {
  if (!run) return `<td class="rec-${kind}"><span class="rec muted">—</span></td>`;
  const tip = `${displayStatus(run)} · ${REASONS[run.reason] || run.reason || ''} · ${run.controllerShort || run.controller || ''}`+
    ` · ${run.physicsCore || ''} · ${new Date(run.startedAt).toLocaleString()} · ${run.id}`;
  const m = primaryMetrics(run) || {};
  const time = run.status === 'completed' && Number.isFinite(m.completionTime) ? m.completionTime : m.time;
  const compact = `${fmt(m.distance, 1)} m · ${fmt(time, 1)} s · ${fmt(m.avgSpeed, 2)} m/s`;
  return `<td class="rec-${kind}" title="${esc(formatRunSummary(run))} · ${esc(tip)}"><span class="rec">${compact}</span>`+
    `<span class="rec-sub">${displayStatus(run).toLowerCase()} · ${esc(run.controllerShort || '—')} · ${esc(stamp(run.startedAt))}${run.source === 'seed' ? ' · seed' : ''}</span></td>`;
}

function historyTable(summary) {
  const runs = summary.runs.slice().sort((a, b) => Date.parse(a.startedAt)-Date.parse(b.startedAt));
  const model = (m, cls) => (m
    ? `<td class="r ${cls}">${fmt(m.distance, 2)}</td><td class="r ${cls}">${fmt(m.avgSpeed, 3)}</td><td class="r ${cls}">${fmt(m.maxTension, 0)}</td>`
    : `<td class="r">—</td><td class="r">—</td><td class="r">—</td>`);
  const rows = runs.map((run, index) => {
    const b = run.B;
    return `<tr>
      <td>#${index+1}${run.source === 'seed' ? '<span class="tag-seed">SEED</span>' : ''}</td>
      <td>${esc(stamp(run.startedAt))}</td>
      <td>${statusBadge(run.status === 'active' ? 'INCOMPLETE' : displayStatus(run))}</td>
      <td title="${esc(run.reason || '')}">${esc(REASONS[run.reason] || run.reason || '—')}</td>
      <td>${esc(run.controllerShort || run.controller || '—')}</td>
      <td>${esc(run.gait || '—')}${run.gaitChanges ? ` (+${run.gaitChanges})` : ''}</td>
      <td>${esc(run.physicsCore || '—')}/${esc(run.actuation === 'cable-only' ? 'cable' : run.actuation === 'assisted' ? 'assist' : '—')}</td>
      <td class="r">${fmt(run.simTime, 1)}</td>
      ${model(run.A, 'a')}
      ${model(b, 'b')}
      <td class="r b">${fmt(b?.maxG, 2)}</td>
      <td class="r b">${fmt(b?.deformation, 3)}</td>
    </tr>`;
  }).join('');
  return `<div class="history">
    <div class="history-head">
      <span>${summary.count} run${summary.count === 1 ? '' : 's'} on Level ${summary.level} · ${summary.completed} completed</span>
      <button type="button" class="btn btn-xs" data-history-export="${summary.level}" data-format="csv">Export level CSV</button>
      <button type="button" class="btn btn-xs" data-history-export="${summary.level}" data-format="json">Export level JSON</button>
    </div>
    <div class="table-scroll"><table class="table history-table">
      <thead><tr><th>Run</th><th>Started</th><th>Status</th><th>Ended by</th><th>Controller</th><th>Gait</th><th>Core / act.</th><th class="r">t (s)</th>
        <th class="r a">A dist (m)</th><th class="r a">A v̄ (m/s)</th><th class="r a">A T max (N)</th>
        <th class="r b">B dist (m)</th><th class="r b">B v̄ (m/s)</th><th class="r b">B T max (N)</th><th class="r b">B max G</th><th class="r b">B deform (m)</th></tr></thead>
      <tbody>${rows || '<tr><td colspan="16" class="muted">No runs yet.</td></tr>'}</tbody>
    </table></div>
  </div>`;
}

export class BenchmarkMatrix {
  constructor({ body, total, getState, getLive, onSelectLevel, onExportLevel }) {
    this.body = body;
    this.total = total;
    this.getState = getState;
    this.getLive = getLive;
    this.onSelectLevel = onSelectLevel;
    this.onExportLevel = onExportLevel;
    this.expanded = new Set();
    this.body?.addEventListener('click', event => this.handleClick(event));
    this.body?.addEventListener('keydown', event => {
      const row = event.target.closest?.('tr.matrix-row');
      if (row && (event.key === 'Enter' || event.key === ' ') && event.target === row) {
        event.preventDefault();
        this.onSelectLevel?.(Number(row.dataset.level));
      }
    });
  }

  handleClick(event) {
    const exportButton = event.target.closest('[data-history-export]');
    if (exportButton) {
      this.onExportLevel?.(Number(exportButton.dataset.historyExport), exportButton.dataset.format);
      return;
    }
    const toggle = event.target.closest('[data-toggle-history]');
    if (toggle) {
      const level = Number(toggle.dataset.toggleHistory);
      if (this.expanded.has(level)) this.expanded.delete(level); else this.expanded.add(level);
      this.render();
      return;
    }
    if (event.target.closest('.history-row')) return;
    const row = event.target.closest('tr.matrix-row');
    if (row) this.onSelectLevel?.(Number(row.dataset.level));
  }

  render() {
    if (!this.body) return;
    const state = this.getState();
    const live = this.getLive();
    let html = '';
    for (let level = 1; level <= LEVEL_COUNT; level++) {
      const summary = summarizeLevel(state, level);
      const info = LEVELS[level-1];
      const isCurrent = live.level === level;
      const open = this.expanded.has(level);
      html += `<tr class="matrix-row${isCurrent ? ' is-current' : ''}" data-level="${level}" tabindex="0" aria-current="${isCurrent ? 'true' : 'false'}" title="Load Level ${level}">
        <td class="lvl"><span class="lvl-name"><span class="lvl-num">${level}</span>${esc(info.name)}</span><span class="lvl-sub">${esc(info.terrain)}</span></td>
        ${this.metricCells(level, summary, live)}
        <td class="r"><button type="button" class="runs-btn" data-toggle-history="${level}" aria-expanded="${open}" title="Show all runs of Level ${level}">${summary.count} ${open ? '▴' : '▾'}</button></td>
        ${recordCell(summary.best, 'best')}
        ${recordCell(summary.worst, 'worst')}
      </tr>`;
      if (open) html += `<tr class="history-row"><td colspan="12">${historyTable(summary)}</td></tr>`;
    }
    this.body.innerHTML = html;
    const count = state.runs.length;
    if (this.total) this.total.textContent = `${count} all-time run${count === 1 ? '' : 's'} recorded`;
  }

  /** Latest/active metrics + status for one level (8 cells). */
  metricCells(level, summary, live) {
    const isCurrent = live.level === level;
    let m, status, obstacles;
    if (isCurrent && live.metrics) {
      m = live.metrics;
      status = live.status;
      obstacles = live.obstacles;
    } else {
      const latest = summary.latest;
      m = primaryMetrics(latest);
      status = displayStatus(latest);
      obstacles = latest?.obstacles;
    }
    return `<td class="r" data-cell="distance">${fmt(m?.distance, 2)}</td>
      <td class="r" data-cell="time">${fmt(m?.time, 1)}</td>
      <td class="r" data-cell="speed">${fmt(m?.avgSpeed, 2)}</td>
      <td class="r" data-cell="g">${fmt(m?.maxG, 2)}</td>
      <td class="r" data-cell="tension">${fmt(m?.maxTension, 1)}</td>
      <td class="r" data-cell="deform">${fmt(m?.deformation, 3)}</td>
      <td class="r" data-cell="obstacles">${Number.isFinite(obstacles) ? obstacles : '—'}</td>
      <td data-cell="status">${statusBadge(status)}</td>`;
  }

  /** Cheap per-frame refresh of the active row only. */
  updateLive() {
    const live = this.getLive();
    const row = this.body?.querySelector(`tr.matrix-row[data-level="${live.level}"]`);
    if (!row || !row.classList.contains('is-current')) { this.render(); return; }
    const m = live.metrics || {};
    const set = (cell, text) => {
      const el = row.querySelector(`[data-cell="${cell}"]`);
      if (el && el.textContent !== text) el.textContent = text;
    };
    set('distance', fmt(m.distance, 2));
    set('time', fmt(m.time, 1));
    set('speed', fmt(m.avgSpeed, 2));
    set('g', fmt(m.maxG, 2));
    set('tension', fmt(m.maxTension, 1));
    set('deform', fmt(m.deformation, 3));
    set('obstacles', Number.isFinite(live.obstacles) ? String(live.obstacles) : '—');
    const statusCell = row.querySelector('[data-cell="status"]');
    const badge = statusBadge(live.status);
    if (statusCell && statusCell.dataset.badge !== live.status) {
      statusCell.innerHTML = badge;
      statusCell.dataset.badge = live.status;
    }
  }
}
