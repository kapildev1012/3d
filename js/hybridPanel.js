/**
 * HYBRIDPANEL.JS - "Hybrid training" block of the Comparative-benchmark tab.
 *
 *  - shows the last run of tools/run_hybrid_training.mjs (public/hybrid/summary.json):
 *    epoch history with Pareto/rollback status, the LLM-tuned parameters and the
 *    stress-matrix digest of the residual-MPPI actor vs SGT;
 *  - optionally runs residual-MPPI *search at test time* on a pool of Web Workers
 *    sharing SharedArrayBuffers (needs a cross-origin-isolated page; server.mjs and
 *    vercel.json send COOP/COEP).
 */
import { SharedVecTensegrityEnv } from './parallelPool.js?v=20261027-ob1';
import { MorphologyActorCritic, synergyMatrix } from './residualMppi.js?v=20261027-ob1';
import { ResidualMppiTrainer } from './mppiPlanner.js?v=20261027-ob1';
import { RESIDUAL_MPPI_WEIGHTS } from './residualMppiWeights.js?v=20261027-ob1';
import { SimConfig, SphericalRoverModel } from './simEngine.js?v=20261027-ob1';

const fmt = (x, d = 3) => (Number.isFinite(x) ? x.toFixed(d) : '–');

function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) { if (k === 'text') node.textContent = v; else node.setAttribute(k, v); }
  for (const child of [].concat(children)) if (child) node.appendChild(typeof child === 'string' ? document.createTextNode(child) : child);
  return node;
}

export class HybridPanel {
  constructor(root = document.getElementById('hybrid-panel')) {
    this.root = root;
    if (!root) return;
    this.status = root.querySelector('#hybrid-status');
    this.demoStatus = root.querySelector('#hybrid-demo-status');
    this.runButton = root.querySelector('#hybrid-demo-run');
    this.stopButton = root.querySelector('#hybrid-demo-stop');
    this.runButton?.addEventListener('click', () => this.runDemo());
    this.stopButton?.addEventListener('click', () => { this.stop = true; });
    this.renderEnvironment();
    this.load();
    this.bindLongTraining();
  }

  /** Long two-loop training (js/experiments/runMassiveTraining100k.js) from the panel. */
  bindLongTraining() {
    const $ = id => this.root.querySelector(`#${id}`);
    const start = $('mt-start'), stop = $('mt-stop'), download = $('mt-download'), status = $('mt-status'), logList = $('mt-log');
    if (!start) return;
    start.disabled = !this.canRun;
    const addLog = text => {
      const li = document.createElement('li');
      li.textContent = `${new Date().toLocaleTimeString()} · ${text}`;
      logList.prepend(li);
      while (logList.children.length > 60) logList.lastChild.remove();
    };
    start.addEventListener('click', async () => {
      if (this.longRun) return;
      const num = (id, fallback) => { const v = Number($(id).value); return Number.isFinite(v) && v > 0 ? v : fallback; };
      const url = $('mt-url').value.trim();
      const llm = url ? { apiUrl: url, apiKey: $('mt-key').value, model: $('mt-model').value.trim() || 'gemini-2.5-flash' } : null;
      this.longAbort = new AbortController();
      start.disabled = true; stop.disabled = false; download.disabled = true;
      status.textContent = 'Starting workers…';
      try {
        const { runMassiveTrainingLoop } = await import('./experiments/runMassiveTraining100k.js');
        this.longRun = runMassiveTrainingLoop(null, $('mt-prompt').value, num('mt-total', 100000), num('mt-batch', 500), num('mt-steps', 200), {
          workers: $('mt-workers').value ? num('mt-workers', 4) : undefined, llm, signal: this.longAbort.signal,
          onProgress: st => {
            if (st.log) addLog(st.log);
            if (st.inner) {
              const eta = st.etaBatchS > 3600 ? `${(st.etaBatchS/3600).toFixed(1)} h` : `${Math.round(st.etaBatchS/60)} min`;
              status.textContent = `Batch ${st.batch}: episode ${st.episode}/${st.episodesPerBatch} · total ${st.completed.toLocaleString()}/${st.totalTargetEpisodes.toLocaleString()} · `
                + `${Math.round(st.stepsPerSecond).toLocaleString()} physics steps/s · success ${st.inner.successRatePct.toFixed(0)}% · `
                + `slack ${st.inner.meanRelaxedCablesPct.toFixed(1)}% · rod ${st.inner.maxRodResidualMm.toFixed(4)} mm · next LLM update in ~${eta}`;
            }
          }
        });
        this.lastResult = await this.longRun;
        status.textContent = `${this.lastResult.stopped ? 'Stopped' : 'Finished'}: ${this.lastResult.episodes.toLocaleString()} episodes in ${this.lastResult.minutes.toFixed(1)} min; best batch ${this.lastResult.best?.batch ?? '–'}.`;
        download.disabled = !this.lastResult.best;
      } catch (error) {
        status.textContent = `Long training failed: ${error.message}`;
      } finally {
        this.longRun = null; start.disabled = !this.canRun; stop.disabled = true;
      }
    });
    stop.addEventListener('click', () => { this.longAbort?.abort(); status.textContent = 'Stopping after the current episode (state is checkpointed)…'; });
    download.addEventListener('click', async () => {
      const { deploymentCheckpoint } = await import('./experiments/runMassiveTraining100k.js');
      const ckpt = deploymentCheckpoint(this.lastResult);
      if (!ckpt) return;
      const blob = new Blob([JSON.stringify(ckpt)], { type: 'application/json' });
      const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(blob), download: 'residual-mppi-checkpoint.json' });
      a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000);
      addLog('checkpoint downloaded; install it with: node tools/install_checkpoint.mjs residual-mppi-checkpoint.json');
    });
  }

  renderEnvironment() {
    const isolated = typeof crossOriginIsolated !== 'undefined' && crossOriginIsolated && typeof SharedArrayBuffer !== 'undefined';
    const cores = navigator.hardwareConcurrency || 4;
    this.canRun = isolated;
    const note = this.root.querySelector('#hybrid-env');
    if (note) {
      note.textContent = isolated
        ? `SharedArrayBuffer ready · ${cores} logical cores · parallel MPPI pool available`
        : 'SharedArrayBuffer unavailable: this page is not cross-origin isolated (serve with COOP same-origin + COEP credentialless, e.g. via server.mjs). The learned actor still runs in the normal simulator.';
    }
    if (this.runButton) this.runButton.disabled = !isolated;
  }

  async load() {
    try {
      const response = await fetch('./public/hybrid/summary.json', { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      this.summary = await response.json();
      try {                                              // optional MPC + RL (tuner off) run
        const rl = await fetch('./public/hybrid_rl/summary.json', { cache: 'no-store' });
        if (rl.ok) this.rlSummary = await rl.json();
      } catch { /* not trained yet */ }
      this.renderSummary(this.summary);
      if (this.status) this.status.textContent = `Last training run: ${this.summary.history.length} evaluated epochs · tuner: ${this.summary.provider}${this.summary.model ? ` (${this.summary.model})` : ' (rule-based)'} · best epoch ${this.summary.bestEpoch} · ${this.summary.wallMinutes} min`;
    } catch {
      if (this.status) this.status.textContent = 'No hybrid training results yet. Run: npm run hybrid (writes public/hybrid/summary.json).';
    }
  }

  renderSummary(s) {
    const history = this.root.querySelector('#hybrid-history');
    if (history) {
      history.replaceChildren(...s.history.map(h => el('tr', {}, [
        el('td', { text: String(h.epoch) }), el('td', { text: fmt(h.composite, 2) }), el('td', { text: `${fmt(h.completion, 0)} %` }),
        el('td', { text: `${fmt(h.maxOuterTensionN, 0)} N` }), el('td', { text: h.onFront ? 'Pareto front' : 'dominated' }),
        el('td', { text: h.epoch === s.bestEpoch ? 'best' : '' })
      ])));
    }
    const params = this.root.querySelector('#hybrid-params');
    if (params) {
      if (s.mpcByLevel) {
        // 14-terrain checkpoint: one configuration per level (compact view of the main knobs).
        params.replaceChildren(...Object.entries(s.mpcByLevel).map(([L, m]) => el('tr', {}, [
          el('td', { text: `L${L}` }),
          el('td', { text: `qVel ${fmt(m.qVelocityTrack, 2)} · qCOM ${fmt(m.qComMargin, 2)} · Kp ${fmt(m.sgtGainKp, 2)} · settle ${fmt(m.sgtSettleSpeed, 2)} m/s · release ${fmt(m.sgtReleaseTime, 2)} s · T ${fmt(m.minCableTensionN, 0)}–${fmt(m.maxCableTensionN, 0)} N · H ${m.horizonSteps}` })
        ])));
      } else {
        const entries = [...Object.entries(s.mpc || {}), ...Object.entries(s.reward || {}).map(([k, v]) => [`reward.${k}`, v])];
        params.replaceChildren(...entries.map(([k, v]) => el('tr', {}, [el('td', { text: k }), el('td', { text: Number.isInteger(v) ? String(v) : fmt(v, 4) })])));
      }
    }
    const table = this.root.querySelector('#hybrid-table');
    if (table) {
      const rows = [];
      for (const base of s.final) {
        for (const [name, list] of [['SGT', s.baselines.sgt], ['SGT 0.120 m', s.baselines.sgt120], ['MPC + RL', this.rlSummary?.final || []], ['MPC + RL + LLM', s.final]]) {
          const d = list.find(r => r.level === base.level && r.disturbance === base.disturbance);
          if (!d) continue;
          rows.push(el('tr', name === 'MPC + RL + LLM' ? { class: 'hybrid-best' } : {}, [
            el('td', { text: String(d.level) }), el('td', { text: d.disturbance }), el('td', { text: name }),
            el('td', { text: fmt(d.avgSpeedMs) }), el('td', { text: fmt(d.rmseM) }), el('td', { text: fmt(d.maxErrorM) }),
            el('td', { text: fmt(d.tRecS, 1) }), el('td', { text: fmt(d.controlEffortUtu, 4) }), el('td', { text: fmt(d.costOfTransport, 1) }),
            el('td', { text: fmt(d.solveTimeMs, 1) }), el('td', { text: fmt(d.successRatePct, 0) }),
            el('td', { text: fmt(d.rodResidualMm, 5) }), el('td', { text: fmt(d.relaxedCablesPct, 1) }), el('td', { text: fmt(d.contactSlipMs, 3) })
          ]));
        }
      }
      table.replaceChildren(...rows);
    }
  }

  /** Residual-MPPI search at test time on a Web Worker pool (no learning updates). */
  async runDemo() {
    if (this.running || !this.canRun) return;
    this.running = true; this.stop = false;
    this.runButton.disabled = true; this.stopButton.disabled = false;
    const set = text => { if (this.demoStatus) this.demoStatus.textContent = text; };
    let pool;
    try {
      const level = Number(document.getElementById('bench-level')?.value) || 1;
      const scenario = document.getElementById('bench-scenario')?.value || 'nominal';
      const duration = Math.max(2, Math.min(20, Number(document.getElementById('hybrid-demo-duration')?.value) || 5));
      const workers = Math.max(2, Math.min(9, (navigator.hardwareConcurrency || 4)-1));
      const candidates = 8;
      set(`Starting ${workers} workers (1 real + ${workers-1} rollout)…`);
      pool = new SharedVecTensegrityEnv({ numEnvs: workers, numCandidates: candidates });
      await pool.start();
      const model = new MorphologyActorCritic(synergyMatrix(new SphericalRoverModel(new SimConfig({}))), 1);
      if (RESIDUAL_MPPI_WEIGHTS) model.load(RESIDUAL_MPPI_WEIGHTS);
      const trainer = new ResidualMppiTrainer(pool, model, { horizon: Math.round(model.params.horizonSteps || 10) });
      const started = performance.now();
      const result = await trainer.runEpisode({ level, scenario, seed: 1 }, duration, {
        train: false,
        onPlan: ({ cycle, cycles, plan }) => {
          set(`Planning ${cycle+1}/${cycles}: ESS ${plan.ess.toFixed(1)} of ${candidates}, best candidate vs prior ${plan.advantage >= 0 ? '+' : ''}${plan.advantage.toFixed(3)}, ${plan.violating} shield-violating`);
          if (this.stop) throw new Error('stopped');
        }
      });
      set(`Done in ${((performance.now()-started)/1000).toFixed(1)} s wall: advanced ${fmt(result.digest.progressM, 2)} m in ${duration} s of simulation, mean ESS ${fmt(result.meanEss, 1)}, ${result.violatingCandidates} shield-violating candidate rollouts, ${workers} workers.`);
    } catch (error) {
      set(error.message === 'stopped' ? 'Stopped.' : `Parallel pool failed: ${error.message}`);
    } finally {
      try { await pool?.terminate(); } catch { /* ignore */ }
      this.running = false; this.runButton.disabled = !this.canRun; this.stopButton.disabled = true;
    }
  }
}
