/**
 * AUTOBENCHMARK.JS - "Run all 14 levels"
 *
 * Loads Levels 1 → 14 in turn and steps both rovers as fast as the CPU
 * allows (time-sliced, ~30 ms of physics per slice, so the 3D view, the
 * live matrix row and the figures keep updating). Each level runs for the
 * chosen simulated duration or until its mission ends; the run is recorded
 * by the RunRecorder like any other run (source 'auto'), so it lands in the
 * matrix, the history drawer and every storage layer. At the end a report
 * (one row per level) is shown and can be downloaded as JSON or CSV.
 */
import { primaryMetrics, displayStatus } from './benchmarkCore.js?v=20261027-ob1';
import { LEVELS } from './levels.js?v=20261027-ob1';

const SLICE_MS = 30;

export class AutoBenchmark {
  constructor(app) {
    this.app = app;
    this.running = false;
    this.stopRequested = false;
    this.report = null;
  }

  async run({ duration = 20, levels = LEVELS.map(level => level.id), onProgress } = {}) {
    if (this.running) return null;
    const app = this.app;
    this.running = true;
    this.stopRequested = false;
    const wasPlaying = app.isPlaying;
    app.setPlaying(false, { record: false });
    app.recorder.commit('auto-benchmark');
    const results = [];
    const startedAt = new Date().toISOString();
    try {
      for (let index = 0; index < levels.length && !this.stopRequested; index++) {
        const level = levels[index];
        app.loadExperiment(level, { reason: 'auto-benchmark' });
        app.recorder.source = 'auto';
        const steps = Math.round(duration/app.cfg.dt);
        let k = 0;
        let finished = false;
        while (k < steps && !finished && !this.stopRequested) {
          const sliceStart = performance.now();
          while (k < steps && performance.now()-sliceStart < SLICE_MS) {
            if (!app.simA.metrics.runTerminal) app.simA.step();
            if (!app.simB.metrics.runTerminal) app.simB.step();
            k++;
            if (k % 25 === 0) {
              app.recorder.tick();
              if (app.recorder.awaitingReset) { finished = true; break; }
            }
          }
          onProgress?.({ level, index, total: levels.length, fraction: (index+k/steps)/levels.length, simTime: app.simB.t });
          await new Promise(resolve => setTimeout(resolve, 0));
        }
        app.recorder.tick();
        const run = app.recorder.activeRun ? app.recorder.finalize('auto-benchmark', 'terminated') : app.recorder.lastFinalized;
        results.push({ level, name: LEVELS[level-1].name, run: run || null });
        app.recorder.source = 'live';
      }
    } finally {
      app.recorder.source = 'live';
      this.running = false;
      if (wasPlaying && !this.stopRequested) app.setPlaying(true, { record: false });
    }
    this.report = { startedAt, finishedAt: new Date().toISOString(), duration, stopped: this.stopRequested, results };
    return this.report;
  }

  stop() { this.stopRequested = true; }

  static reportRows(report) {
    return (report?.results || []).map(({ level, name, run }) => {
      const b = primaryMetrics(run);
      const a = run?.A;
      return {
        level, name,
        status: run ? displayStatus(run) : 'NOT RUN',
        reason: run?.reason || '',
        simTime: run?.simTime ?? null,
        B_distance: b?.distance ?? null, B_avgSpeed: b?.avgSpeed ?? null, B_maxG: b?.maxG ?? null,
        B_maxTension: b?.maxTension ?? null, B_deformation: b?.deformation ?? null,
        A_distance: a?.distance ?? null, A_avgSpeed: a?.avgSpeed ?? null,
        runId: run?.id || ''
      };
    });
  }

  static reportCSV(report) {
    const rows = AutoBenchmark.reportRows(report);
    if (!rows.length) return '';
    const keys = Object.keys(rows[0]);
    const escape = value => (value === null || value === undefined ? '' : /[",\n]/.test(String(value)) ? `"${String(value).replace(/"/g, '""')}"` : String(value));
    return [keys.join(','), ...rows.map(row => keys.map(key => escape(row[key])).join(','))].join('\n')+'\n';
  }
}
