/**
 * PARAMIDPANEL.JS - Tools tab: "Parameter identification · particle filter".
 * Runs the particle filter of final_codes/MainFile.m (js/paramIdentification.js)
 * on the planar model of forward_tensegrity.m or on the 3D rover
 * (js/roverParamIdentification.js) in a module worker (js/paramIdWorker.js),
 * with a live convergence chart and a results table. Without worker support
 * it runs on the main thread, one load case per animation frame.
 */
import { particleFilter } from './paramIdentification.js?v=20261029-pf1';
import { buildIdentificationProblem, problemInfo } from './paramIdPanelCore.js?v=20261029-pf1';
import { convergenceSvg, SERIES_COLORS } from './paramIdPlot.js?v=20261029-pf1';

const MODES = {
  planar: {
    steps: 100, particles: 100, noiseMm: 0,
    note: 'forward_tensegrity.m: 6 nodes, 3 struts, 6 springs. Five factors (strut rest lengths, spring stiffness) from all 18 node coordinates per load case; MainFile.m settings. About 5 s.'
  },
  rover: {
    steps: 80, particles: 100, noiseMm: 0,
    note: 'Six-bar rover, 13 nodes, 6 bars, 36 cables. Four factors (outer and payload cable stiffness and pre-stretch) from 13 node positions under ±20 N loads; likelihood annealed from 3 mm to 0.1 mm. About 1 min.'
  }
};
const fmt = (v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : '—');

export class ParamIdPanel {
  constructor(root = document.getElementById('paramid-panel')) {
    this.root = root;
    if (!root) return;
    const $ = id => root.querySelector(`#${id}`);
    this.el = {
      mode: $('paramid-mode'), steps: $('paramid-steps'), particles: $('paramid-particles'), noise: $('paramid-noise'),
      seed: $('paramid-seed'), run: $('paramid-run'), stop: $('paramid-stop'), status: $('paramid-status'),
      note: $('paramid-note'), chart: $('paramid-chart'), body: $('paramid-body'), download: $('paramid-download')
    };
    this.el.mode?.addEventListener('change', () => this.applyMode());
    this.el.run?.addEventListener('click', () => this.run());
    this.el.stop?.addEventListener('click', () => { this.stopRequested = true; });
    this.el.download?.addEventListener('click', () => this.download());
    this.applyMode();
  }

  applyMode() {
    const mode = MODES[this.el.mode.value] || MODES.planar;
    this.el.steps.value = mode.steps;
    this.el.particles.value = mode.particles;
    this.el.noise.value = mode.noiseMm;
    this.el.note.textContent = mode.note;
  }

  readSettings() {
    const num = (el, lo, hi, fallback) => { const v = Number(el.value); return Number.isFinite(v) ? Math.min(hi, Math.max(lo, v)) : fallback; };
    return {
      mode: this.el.mode.value === 'rover' ? 'rover' : 'planar',
      steps: Math.round(num(this.el.steps, 5, 300, 80)),
      particles: Math.round(num(this.el.particles, 10, 500, 100)),
      noiseMm: num(this.el.noise, 0, 1, 0),
      seed: Math.round(num(this.el.seed, 0, 1e9, 46))
    };
  }

  async run() {
    if (this.running) return;
    const s = this.readSettings();
    this.running = true; this.stopRequested = false; this.result = null;
    this.el.run.disabled = true; this.el.stop.disabled = false; this.el.download.disabled = true;
    this.el.status.textContent = 'Generating the synthetic measurements…';
    this.el.chart.innerHTML = ''; this.el.body.innerHTML = '';
    let info = null, last = null, summary = null;
    const history = { mean: [], std: [] };
    const onProblem = p => { info = p; };
    const onStep = step => {
      last = step;
      history.mean.push(step.mean); history.std.push(step.std);
      this.el.status.textContent = `Load case ${step.k}/${step.steps} · ${step.solves.toLocaleString()} solves · N_eff ${step.neff.toFixed(1)} · ${(step.elapsedMs/1000).toFixed(1)} s`;
      this.el.chart.innerHTML = convergenceSvg({ mean: history.mean, std: history.std, trueTheta: info.trueTheta,
        lower: info.lower, upper: info.upper, labels: info.params.map(p => p.label), totalSteps: s.steps });
      this.renderTable(info, step.mean, step.std);
    };
    try {
      summary = await (typeof Worker === 'function' ? this.runInWorker(s, onProblem, onStep) : this.runHere(s, onProblem, onStep));
      this.result = { settings: s, problem: info, history, summary: summary || { stoppedAt: last?.k ?? 0, final: last?.mean, std: last?.std } };
      this.el.status.textContent = summary
        ? `Done: ${summary.steps} load cases, ${summary.forwardSolves.toLocaleString()} solves, ${(summary.elapsedMs/1000).toFixed(1)} s. Largest error ${Math.max(...summary.errorPct.map(Math.abs)).toFixed(2)} %.`
        : `Stopped at load case ${last?.k ?? 0}.`;
      this.el.download.disabled = !info;
    } catch (error) {
      console.error('[paramid]', error);
      this.el.status.textContent = `Identification failed: ${error.message}`;
    } finally {
      this.running = false;
      this.el.run.disabled = false; this.el.stop.disabled = true;
    }
  }

  /** Off the main thread; resolves with the summary, or null when stopped. */
  runInWorker(settings, onProblem, onStep) {
    return new Promise((resolve, reject) => {
      let worker;
      try {
        worker = new Worker(new URL('./paramIdWorker.js?v=20261029-pf1', import.meta.url), { type: 'module' });
        this.lastRunMode = 'worker';
      } catch (error) {
        this.runHere(settings, onProblem, onStep).then(resolve, reject);
        return;
      }
      const poll = setInterval(() => {
        if (!this.stopRequested) return;
        clearInterval(poll); worker.terminate(); resolve(null);
      }, 100);
      const finish = (fn, value) => { clearInterval(poll); worker.terminate(); fn(value); };
      worker.onmessage = ({ data }) => {
        if (data.type === 'problem') onProblem(data.info);
        else if (data.type === 'step') onStep(data.step);
        else if (data.type === 'done') finish(resolve, data.summary);
        else if (data.type === 'error') finish(reject, new Error(data.message));
      };
      worker.onerror = event => finish(reject, new Error(event.message || 'worker error'));
      worker.postMessage(settings);
    });
  }

  /** Main-thread fallback: one load case per animation frame. */
  async runHere(settings, onProblem, onStep) {
    this.lastRunMode = 'main thread';
    await new Promise(r => setTimeout(r, 20));
    const problem = buildIdentificationProblem(settings);
    onProblem(problemInfo(problem));
    const it = particleFilter(problem, { particles: settings.particles, ...problem.filterDefaults });
    let r;
    while (!(r = it.next()).done) {
      onStep(r.value);
      await new Promise(res => requestAnimationFrame(() => res()));
      if (this.stopRequested) return null;
    }
    return r.value;
  }

  renderTable(problem, mean, std) {
    const colors = SERIES_COLORS.dark;
    this.el.body.innerHTML = problem.params.map((p, i) => {
      const t = problem.trueTheta[i], err = 100*(mean[i]-t)/t;
      return `<tr><td><span class="pid-swatch" style="background:${colors[i % colors.length]}"></span>${p.label}</td>`
        +`<td class="num">${fmt(t, 2)}</td><td class="num">${fmt(mean[i])} ± ${fmt(2*std[i])}</td><td class="num">${fmt(err, 2)} %</td></tr>`;
    }).join('');
  }

  download() {
    if (!this.result) return;
    const blob = new Blob([JSON.stringify(this.result, null, 1)], { type: 'application/json;charset=utf-8' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `param_identification_${this.result.settings.mode}_seed${this.result.settings.seed}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }
}
