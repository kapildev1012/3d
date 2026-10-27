/**
 * BENCHMARKPANEL.JS - In-browser multi-seed comparative benchmark
 *
 * Runs js/experimentProtocol.js (the same code as tools/run_experiments.mjs)
 * for the selected methods x seeds, plots mean +/- 1 sigma bands and builds
 * a booktabs LaTeX table (best values bold, mean +/- std).
 */
import { METHODS, SCENARIOS, runExperiment, meanStd, latexTable } from './experimentProtocol.js?v=20261027-ob1';
import { matlabLineOptions, MATLAB_COLORS } from './matlabCharts.js?v=20261027-ob1';

const COLUMNS = [
  { key: 'speed', label: '$\\bar v$ (m/s)', digits: 3, better: 'max' },
  { key: 'rmse', label: 'RMSE (m)', digits: 3, better: 'min' },
  { key: 'mae', label: 'max |e| (m)', digits: 3, better: 'min' },
  { key: 'recovery', label: '$t_\\mathrm{rec}$ (s)', digits: 2, better: 'min' },
  { key: 'effort', label: '$\\int u^\\top u\\,dt$ (m$^2$s)', digits: 3, better: 'min' },
  { key: 'cot', label: 'CoT (--)', digits: 2, better: 'min' },
  { key: 'solveMsMean', label: '$t_\\mathrm{solve}$ (ms)', digits: 2, better: 'min' },
  { key: 'success', label: 'Success (\\%)', digits: 0, better: 'max', scale: 100, showStd: false },
  { key: 'assistShare', label: 'Assist (\\%)', digits: 0, better: 'min', scale: 100, showStd: false }
];

/** Average time series of several runs on the common 0.1 s grid. */
function bandOf(runs, field) {
  const length = Math.min(...runs.map(run => run.series.t.length));
  const t = runs[0].series.t.slice(0, length);
  const mean = [], lower = [], upper = [];
  for (let i = 0; i < length; i++) {
    const stats = meanStd(runs.map(run => run.series[field][i]));
    mean.push({ x: t[i], y: stats.mean });
    lower.push({ x: t[i], y: stats.mean-stats.std });
    upper.push({ x: t[i], y: stats.mean+stats.std });
  }
  return { mean, lower, upper };
}

/**
 * Yield until the browser has painted (falls back to a timer when the tab is
 * in the background). A bare setTimeout(0) between simulation chunks starves
 * requestAnimationFrame, so status text and charts would never be redrawn.
 */
function nextPaint() {
  return new Promise(resolve => {
    let done = false;
    const finish = () => { if (!done) { done = true; resolve(); } };
    if (typeof requestAnimationFrame === 'function') requestAnimationFrame(() => setTimeout(finish, 0));
    setTimeout(finish, 100);
  });
}

/** Size a chart to its container now (charts created inside a hidden tab start at 0×0). */
function fitChart(chart) {
  const box = chart?.canvas?.parentElement?.getBoundingClientRect();
  if (!box || box.width < 2 || box.height < 2) return;
  if (Math.abs(chart.width-box.width) > 1 || Math.abs(chart.height-box.height) > 1 || chart.width === 0) chart.resize();
}

export class BenchmarkPanel {
  constructor(Chart, registerCharts) {
    this.Chart = Chart;
    this.results = [];
    this.running = false;
    this.root = document.getElementById('benchmark-panel');
    if (!this.root) return;
    this.buildCharts();
    registerCharts?.(this.charts);
    this.root.querySelector('#bench-run')?.addEventListener('click', () => this.run());
    this.root.querySelector('#bench-stop')?.addEventListener('click', () => { this.stopRequested = true; });
    this.root.querySelector('#bench-copy-latex')?.addEventListener('click', () => this.copyLatex());
    this.root.querySelector('#bench-download')?.addEventListener('click', () => this.download());
  }

  buildCharts() {
    const make = (id, yLabel, legend = 'northwest') => {
      const canvas = document.getElementById(id);
      return canvas ? new this.Chart(canvas.getContext('2d'), {
        type: 'line', data: { datasets: [] }, options: matlabLineOptions({ yLabel, legend })
      }) : null;
    };
    this.charts = {
      'bench-progress': make('chart-bench-progress', 'Forward progress Δy (m)'),
      'bench-error': make('chart-bench-error', 'Cross-track error |e(t)| (m)'),
      'bench-effort': make('chart-bench-effort', 'Actuation ‖u(t)‖ (m)')
    };
    // Empty-state hint over each plot until the first run finishes.
    for (const chart of Object.values(this.charts)) {
      const axes = chart?.canvas?.parentElement;
      if (!axes) continue;
      axes.style.position ||= 'relative';
      const hint = document.createElement('div');
      hint.className = 'bench-empty-hint';
      hint.textContent = 'No benchmark runs yet: press “Run benchmark” above (mean ± 1σ over the selected seeds).';
      Object.assign(hint.style, {
        position: 'absolute', inset: '0', display: 'flex', alignItems: 'center', justifyContent: 'center',
        textAlign: 'center', padding: '0 24px', font: '12px var(--font-ui, system-ui)', color: 'var(--text-dim, #8b93a1)', pointerEvents: 'none'
      });
      axes.appendChild(hint);
    }
    // Re-fit whenever the panel becomes visible or changes size.
    if (typeof ResizeObserver === 'function') {
      const observer = new ResizeObserver(() => this.fitCharts());
      for (const chart of Object.values(this.charts)) if (chart?.canvas?.parentElement) observer.observe(chart.canvas.parentElement);
    }
  }

  fitCharts() {
    for (const chart of Object.values(this.charts || {})) {
      if (!chart) continue;
      fitChart(chart);
      if (chart.width > 0) chart.update('none');
    }
  }

  selection() {
    const methods = [...this.root.querySelectorAll('input[name="bench-method"]:checked')].map(input => input.value);
    const seeds = Math.max(1, Math.min(10, Math.round(Number(this.root.querySelector('#bench-seeds')?.value) || 3)));
    const duration = Math.max(5, Math.min(120, Number(this.root.querySelector('#bench-duration')?.value) || 20));
    const scenario = this.root.querySelector('#bench-scenario')?.value || 'nominal';
    const level = Number(this.root.querySelector('#bench-level')?.value) || 1;
    return { methods, seeds, duration, scenario, level };
  }

  setStatus(text) {
    const status = this.root.querySelector('#bench-status');
    if (status) status.textContent = text;
  }

  async run() {
    if (this.running) return;
    const { methods, seeds, duration, scenario, level } = this.selection();
    if (!methods.length) { this.setStatus('Select at least one method.'); return; }
    this.running = true;
    this.stopRequested = false;
    this.results = [];
    this.root.querySelector('#bench-run').disabled = true;
    const total = methods.length*seeds;
    let index = 0;
    const started = performance.now();
    try {
      for (const method of methods) {
        for (let seed = 1; seed <= seeds; seed++) {
          if (this.stopRequested) throw new Error('stopped');
          index++;
          const result = await runExperiment({ method, scenario, seed, level }, {
            duration,
            onChunk: async fraction => {
              this.setStatus(`Run ${index}/${total} · ${METHODS[method].short} · seed ${seed} · ${(100*fraction).toFixed(0)} %`);
              await nextPaint();
              if (this.stopRequested) throw new Error('stopped');
            }
          });
          this.results.push(result);
          this.render();
        }
      }
      this.setStatus(`Done: ${total} runs in ${((performance.now()-started)/1000).toFixed(0)} s (${SCENARIOS[scenario].label}, Level ${level}).`);
    } catch (error) {
      this.setStatus(error.message === 'stopped' ? `Stopped after ${this.results.length} runs.` : `Error: ${error.message}`);
      if (error.message !== 'stopped') throw error;
    } finally {
      this.running = false;
      this.root.querySelector('#bench-run').disabled = false;
    }
  }

  grouped() {
    const groups = new Map();
    for (const result of this.results) {
      const key = result.spec.method;
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key).push(result);
    }
    return groups;
  }

  render() {
    const groups = this.grouped();
    const methodKeys = [...groups.keys()];
    const styleOf = key => {
      const method = METHODS[key];
      const color = MATLAB_COLORS[Object.keys(METHODS).indexOf(key) % MATLAB_COLORS.length];
      return { color, dash: method.proposed ? [] : method.reference ? [2, 3] : [7, 4] };
    };
    const bandDatasets = field => methodKeys.flatMap(key => {
      const { mean, lower, upper } = bandOf(groups.get(key), field);
      const { color, dash } = styleOf(key);
      return [
        { label: '', hideInLegend: true, data: lower, borderWidth: 0, pointRadius: 0, fill: false, borderColor: 'transparent' },
        { label: '', hideInLegend: true, data: upper, borderWidth: 0, pointRadius: 0, fill: '-1',
          backgroundColor: `${color}2E`, borderColor: 'transparent', fillBetween: true, lowerData: lower },
        { label: `${METHODS[key].short} (n=${groups.get(key).length})`, data: mean, borderColor: color,
          backgroundColor: color, borderWidth: METHODS[key].proposed ? 2 : 1.5, borderDash: dash, pointRadius: 0, fill: false }
      ];
    });
    const update = (key, field) => {
      const chart = this.charts[key];
      if (!chart) return;
      chart.data.datasets = bandDatasets(field);
      fitChart(chart);
      chart.update('none');
      const hint = chart.canvas.parentElement?.querySelector('.bench-empty-hint');
      if (hint) hint.style.display = methodKeys.length > 0 ? 'none' : 'flex';
    };
    // |e| for the error figure
    for (const run of this.results) run.series.absE = run.series.absE || run.series.e.map(Math.abs);
    update('bench-progress', 'y');
    update('bench-error', 'absE');
    update('bench-effort', 'u');
    this.renderTable(groups);
  }

  tableRows(groups = this.grouped()) {
    return [...groups.entries()].map(([key, runs]) => {
      const cells = {};
      for (const column of COLUMNS) cells[column.key] = meanStd(runs.map(run => run.metrics[column.key]));
      return { label: METHODS[key].label, reference: Boolean(METHODS[key].reference), cells, n: runs.length };
    });
  }

  renderTable(groups) {
    const body = this.root.querySelector('#bench-table-body');
    if (!body) return;
    const rows = this.tableRows(groups);
    const fmt = (cell, column) => {
      if (!cell || !Number.isFinite(cell.mean)) return '—';
      const scale = column.scale ?? 1;
      const mean = (cell.mean*scale).toFixed(column.digits);
      return column.showStd === false || !(cell.std > 0) ? mean : `${mean} ± ${(cell.std*scale).toFixed(column.digits)}`;
    };
    body.innerHTML = rows.map(row => `<tr><td>${row.label}${row.reference ? ' †' : ''}</td><td>${row.n}</td>${
      COLUMNS.map(column => `<td>${fmt(row.cells[column.key], column)}</td>`).join('')}</tr>`).join('');
  }

  latex() {
    const { scenario, level, duration } = this.selection();
    return latexTable(this.tableRows(), COLUMNS, {
      caption: `Cable-only locomotion, ${SCENARIOS[scenario].label.toLowerCase()}, Level ${level}, ${duration}\\,s runs (mean\\,$\\pm$\\,std over seeds).`,
      label: `tab:bench_${scenario}_L${level}`,
      note: 'Best value per column in bold (non-admissible reference rows excluded). All methods share the Lagrangian DAE core, passive contact and identical actuator limits.'
    });
  }

  async copyLatex() {
    if (!this.results.length) { this.setStatus('Run the benchmark first.'); return; }
    const text = this.latex();
    try {
      await navigator.clipboard.writeText(text);
      this.setStatus('LaTeX table copied to the clipboard.');
    } catch {
      const area = this.root.querySelector('#bench-latex');
      if (area) { area.hidden = false; area.value = text; area.select(); }
      this.setStatus('Clipboard unavailable: the table is shown below, select and copy it.');
    }
  }

  download() {
    if (!this.results.length) return;
    const blob = new Blob([JSON.stringify(this.results, null, 2)], { type: 'application/json' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'benchmark_results.json';
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1500);
  }
}
