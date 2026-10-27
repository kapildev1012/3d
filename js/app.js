/**
 * APP.JS - Main Coordinator for 6-Bar Tensegrity Icosahedron Rover Simulator
 */

import { SimConfig, TerrainModel, SphericalRoverModel, Simulation, StructuralOptimizer } from './simEngine.js?v=20261027-ob1';
import { relaxedCableTension } from './driveControllers.js?v=20261027-ob1';
import { Visualizer } from './visualizer.js?v=20261027-ob1';
import { AdaptiveRouteLearner, LEVEL14_LEARNING_DEFAULTS } from './adaptiveLearning.js?v=20261027-ob1';
import {
  bothModelsReachedGoals,
  canStartNextTrainingAttempt,
  currentTrainingAttemptNumber
} from './trainingCycle.js?v=20261027-ob1';
import { BenchmarkPersistence } from './benchmarkPersistence.js?v=20261027-ob1';
import { RunRecorder } from './runRecorder.js?v=20261027-ob1';
import { BenchmarkMatrix } from './benchmarkMatrix.js?v=20261027-ob1';
import { AutoBenchmark } from './autoBenchmark.js?v=20261027-ob1';
import { runsForLevel, toCSV, emptyState } from './benchmarkCore.js?v=20261027-ob1';
import { LEVELS, configureLevel, levelName } from './levels.js?v=20261027-ob1';
import { renderFigures } from './figures.js?v=20261027-ob1';
import { MATERIALS, DEFAULT_APP_MATERIAL, applyMaterial, massBreakdown } from './materials.js?v=20261027-ob1';
import { roverStructuralModel } from './structuralAnalysis.js?v=20261027-ob1';
import {
  bindNumericInputs, syncNumericInputs, bindDockTabs, trackToolbarHeight, readUrlState, writeUrlState
} from './uiBindings.js?v=20261027-ob1';
import { EXPERIMENT_SCHEMA, validateObject, validateSettingsFile } from './configSchema.js?v=20261027-ob1';
import { SCENARIOS } from './experimentProtocol.js?v=20261027-ob1';
import { BenchmarkPanel } from './benchmarkPanel.js?v=20261027-ob1';
import { HybridPanel } from './hybridPanel.js?v=20261027-ob1';
import { installMatlabTheme, setChartTheme, matlabLineOptions, matlabLine, xy, exportChartPNG, exportChartCSV, exportChartIEEE } from './matlabCharts.js?v=20261027-ob1';

const Chart = window.Chart;

function movingAverage(values, windowSize) {
  const result = new Array(values.length);
  let sum = 0;
  for (let i = 0; i < values.length; i++) {
    const value = Number.isFinite(values[i]) ? values[i] : 0;
    sum += value;
    if (i >= windowSize) sum -= Number.isFinite(values[i-windowSize]) ? values[i-windowSize] : 0;
    result[i] = sum/Math.min(i+1, windowSize);
  }
  return result;
}

const formatVector = (vector, digits = 2) => vector?.length
  ? `(${vector.map(value => Number(value).toFixed(digits)).join(', ')}) m` : '—';

function downloadText(filename, contents, mimeType = 'text/csv;charset=utf-8') {
  const blob = new Blob([contents], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  document.body.appendChild(anchor);
  anchor.click();
  anchor.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

const ALLOWED_SPEEDS = [0.25, 0.5, 1, 2, 4, 10];

class App {
  constructor(centralSettings = {}) {
    this.simSpeed = 1.0;
    this.isPlaying = true;
    this.isGeometryCheckpointMode = false; // Start in active Locomotion Simulation Mode!
    this.showDebugLabels = false;
    this.selectedCableIndex = 0;
    this.autoLearningEnabled = true;
    this.learningRestartAt = null;
    this.learningStorageKey = 'tensegrity-route-learning-v1';
    this.lastPersistedLearningRevision = -1;
    this.centralSettings = centralSettings || {};

    // Persistent benchmark store: synchronous localStorage hydration first,
    // so the matrix is complete on the very first paint.
    this.suspendUrlSync = true; // applyUrlState() writes the URL once at the end
    this.persistence = new BenchmarkPersistence({
      onChange: () => { this.matrix?.render(); this.updateStorageStatus(); }
    });
    this.persistence.hydrateSync();

    renderFigures(document);
    this.renderLevelOptions();
    this.initSimulation();
    this.initVisualizer();
    this.initCharts();
    this.benchmarkPanel = new BenchmarkPanel(Chart, charts => { this.benchmarkCharts = charts; });
    this.hybridPanel = new HybridPanel();
    this.initMonitoringControls();

    this.recorder = new RunRecorder(this, this.persistence);
    this.autoBenchmark = new AutoBenchmark(this);
    this.matrix = new BenchmarkMatrix({
      body: document.getElementById('experiments-table-body'),
      total: document.getElementById('records-total'),
      getState: () => this.persistence.state,
      getLive: () => this.liveMatrixState(),
      onSelectLevel: level => this.selectLevel(level),
      onExportLevel: (level, format) => this.exportLevel(level, format)
    });

    this.bindEvents();
    bindNumericInputs();
    trackToolbarHeight();
    this.showDock = bindDockTabs({ onShow: () => this.resizeVisibleCharts() });

    // Graph refresh cadence in physics steps. A non-positive
    // chartUpdateSeconds means true real-time: redraw on every rendered frame.
    const chartUpdateSeconds = Number.isFinite(this.cfg.monitoring?.chartUpdateSeconds)
      ? this.cfg.monitoring.chartUpdateSeconds : 0;
    this.chartUpdateIntervalSteps = chartUpdateSeconds > 0
      ? Math.max(1, Math.round(chartUpdateSeconds/this.cfg.dt))
      : 1;

    // Closing or refreshing the tab commits the open run to every layer.
    const onLeave = () => {
      this.recorder.commit('unload');
      this.persistence.flushOnUnload();
    };
    window.addEventListener('pagehide', onLeave);
    window.addEventListener('beforeunload', onLeave);

    this.updateViewportHeader();
    this.matrix.render();
    this.updateStorageStatus();
    // Sync the top-bar preset dropdown, sidebar slider and readout with the
    // configured starting gravity (Earth by default).
    this.setGravity(Math.abs(this.cfg.gravity?.[2] ?? 9.81), { source: 'init' });
    // IndexedDB + disk file + seed (async); the matrix re-renders on merge.
    this.persistence.hydrateAsync().then(() => this.updateStorageStatus());
    requestAnimationFrame(t => this.animationLoop(t));
  }

  renderLevelOptions() {
    const select = document.getElementById('select-experiment');
    if (!select) return;
    select.innerHTML = LEVELS.map(level =>
      `<option value="${level.id}">L${level.id} · ${level.name}</option>`).join('');
    select.value = '10';
  }

  /** Live metrics of the active level for the matrix's ACTIVE LIVE row. */
  liveMatrixState() {
    const m = this.simB?.metrics;
    const active = this.recorder?.activeRun;
    let status = 'ACTIVE LIVE';
    if (!active) {
      const last = this.recorder?.lastFinalized;
      if (this.recorder?.awaitingReset && last?.level === this.cfg.experimentId) {
        status = last.status === 'completed' ? 'COMPLETED' : last.status === 'failed' ? 'FAILED' : 'INCOMPLETE';
      } else if (!(this.simB?.t > 0)) {
        status = 'NOT RUN';
      }
    } else if (!this.isPlaying) {
      status = 'PAUSED';
    }
    return {
      level: this.cfg.experimentId,
      status,
      obstacles: this.obstacleCount(),
      metrics: m && this.simB.t > 0 ? {
        distance: m.distanceTraveled, time: m.timeElapsed, avgSpeed: m.avgVelocity,
        maxG: m.payloadAccelMax, maxTension: m.maxCableTension, deformation: m.shapeDeformationMax
      } : null
    };
  }

  obstacleCount() {
    const terrain = this.terrain;
    if (!terrain) return null;
    const field = terrain.obstacleField?.length || 0;     // solid 3D obstacles (all levels)
    if (terrain.course) return terrain.course.obstacles.length+field;
    if (terrain.expeditionObstacles?.length) return terrain.expeditionObstacles.length+field;
    return (terrain.rocks || []).filter(rock => rock.kind === 'challenge').length+field;
  }

  /** Single entry point for play/pause; pause commits a durable checkpoint. */
  setPlaying(playing, { record = true } = {}) {
    this.isPlaying = Boolean(playing);
    const button = document.getElementById('btn-play-pause');
    if (button) button.textContent = this.isPlaying ? 'Pause' : 'Play';
    document.body.classList.toggle('is-paused', !this.isPlaying);
    if (!this.isPlaying && record) this.recorder?.checkpoint('pause');
    this.lastFrameTime = performance.now();
    this.accumulatedTime = 0;
  }

  /** Commit the open run, then restart both rovers from the start line. */
  resetRovers(reason = 'reset') {
    this.recorder?.commit(reason);
    this.simA.reset();
    this.simB.reset();
    this.accumulatedTime = 0;
    this.lastChartStep = 0;
    this.learningRestartAt = null;
  }

  selectLevel(level) {
    if (this.autoBenchmark?.running) { this.showNotice('Auto-benchmark running: stop it to choose a level.', 'warn'); return; }
    if (level === this.cfg.experimentId) { this.showDock?.('matrix'); return; }
    this.loadExperiment(level);
    this.syncPhysicsControls();
    this.showNotice(`Loaded ${levelName(level)}`, 'ok');
  }

  updateViewportHeader() {
    const level = LEVELS[this.cfg.experimentId-1];
    const title = document.getElementById('txt-active-experiment-title');
    if (title) title.textContent = levelName(this.cfg.experimentId);
    const sub = document.getElementById('txt-active-terrain');
    if (sub) sub.textContent = level ? level.terrain : '';
    const core = document.getElementById('chip-core');
    if (core) core.textContent = this.cfg.physicsCore === 'dae' ? 'Lagrangian DAE' : 'legacy SHAKE';
    const act = document.getElementById('chip-actuation');
    if (act) act.textContent = this.cfg.pureCableActuation ? 'cable-only' : 'assisted';
  }

  updateStorageStatus() {
    const chip = document.getElementById('storage-status');
    if (!chip || !this.persistence) return;
    const s = this.persistence.status;
    const server = { disk: 'disk', 'read-only': 'disk read-only', 'browser-only': 'browser only', offline: 'no server', pending: '…' }[s.server] || s.server;
    chip.textContent = `records: ${server}${s.idb === 'ok' ? ' · IndexedDB' : ''}${s.local === 'ok' ? ' · local' : ''}`;
    chip.title = `Local storage: ${s.local} · IndexedDB: ${s.idb} · server: ${s.server}${s.serverReason ? ` (${s.serverReason})` : ''}`+
      `${s.lastServerSync ? ` · last disk sync ${new Date(s.lastServerSync).toLocaleTimeString()}` : ''}${s.seeded ? ' · seeded from public/default_benchmarks.json' : ''}`;
    chip.classList.toggle('is-ok', s.server === 'disk');
    chip.classList.toggle('is-warn', s.server !== 'disk');
  }

  updateRunChip() {
    const chip = document.getElementById('chip-run');
    if (!chip) return;
    const run = this.recorder?.activeRun;
    const text = run ? `REC ${run.id.slice(-6)} · ${run.checkpoints} cp` : (this.recorder?.awaitingReset ? 'run committed' : 'run —');
    if (chip.textContent !== text) chip.textContent = text;
  }

  /** Current setup as shareable URL parameters. */
  syncUrl() {
    if (this.suspendUrlSync) return null;
    return writeUrlState({
      level: this.cfg.experimentId,
      gait: this.cfg.actuationMode,
      controller: this.cfg.controllerMode || 'natural_support_face',
      core: this.cfg.physicsCore,
      act: this.cfg.pureCableActuation ? 'pure' : 'assisted',
      g: Number(Math.abs(this.cfg.gravity?.[2] ?? 9.81).toFixed(3)),
      mat: this.cfg.material || DEFAULT_APP_MATERIAL,
      shield: this.cfg.tensionShield ? 1 : null,
      speed: this.simSpeed,
      cam: document.getElementById('select-camera')?.value || 'chase'
    });
  }

  /**
   * Tipping-gait controllers (SGT, MPC + GPS guided policy, cable-only MPC + GNSS) need
   * the research strokes and spool speed; the game defaults (0.12 m outer,
   * 0.065 m payload) are too short for any cable-only tip.
   */
  applyTippingGaitStrokes() {
    const mode = this.cfg.controllerMode;
    const tipping = mode === 'sgt_tipping' || mode === 'guided_policy' || mode === 'residual_mppi' || mode === 'residual_mppi_llm' || (mode === 'mpc_gnss' && this.cfg.pureCableActuation);
    if (!tipping) return;
    Object.assign(this.cfg, { actuationDeltaL: 0.15, coreActuationDeltaL: 0.15, cableLinearVelocity: 0.20, coreCableLinearVelocity: 0.20 });
    const slider = document.getElementById('slider-actuation');
    if (slider) slider.value = '0.15';
    const readout = document.getElementById('val-actuation');
    if (readout) readout.textContent = '0.150 m';
  }

  /** Switch strut/cable material (masses, inertia, series cable stiffness). */
  setMaterial(id, { rebuild = true } = {}) {
    if (!MATERIALS[id]) return;
    applyMaterial(this.cfg, id);
    this.updateMaterialReadout();
    this.tsgfemResult = null;
    if (rebuild) this.loadExperiment(this.cfg.experimentId, { reason: 'parameter-change' });
    this.syncUrl();
  }

  updateMaterialReadout() {
    const id = this.cfg.material || 'lightweight';
    const material = MATERIALS[id];
    const mass = massBreakdown(id);
    const byId = key => document.getElementById(key);
    if (byId('select-material')) byId('select-material').value = id;
    if (byId('val-material-mass')) byId('val-material-mass').textContent = `${mass.total.toFixed(2)} kg · ${(mass.total*9.81).toFixed(1)} N`;
    if (byId('material-spec')) {
      byId('material-spec').textContent = material.strut
        ? `${material.strutSpec}: ${material.strutMass.toFixed(3)} kg each, buckling ${(material.strut.bucklingLoad/1000).toFixed(1)} kN. `
          + `${material.cableSpec}: breaking ${(material.rope.breakingForce/1000).toFixed(2)} kN, effective k ${material.kS.toFixed(0)} N/m.`
        : `${material.strutSpec}; ${material.cableSpec.toLowerCase()}.`;
    }
    if (byId('slider-stiffness')) byId('slider-stiffness').value = String(Math.round(this.cfg.kS));
    if (byId('val-stiffness')) byId('val-stiffness').textContent = `${this.cfg.kS.toFixed(0)} N/m`;
  }

  /** TsgFEM statics + modal analysis of the current rover (Tools tab). */
  runStructuralAnalysis() {
    const byId = key => document.getElementById(key);
    const { js, input } = roverStructuralModel({ material: this.cfg.material || 'lightweight', config: this.cfg });
    this.tsgfemResult = { js, input };
    const set = (key, text) => { const el = byId(key); if (el) el.textContent = text; };
    const ff = js.formFinding, pre = js.prestress, mm = js.minimalMass, ins = js.installed;
    set('tsg-ratio', `${ff.modelledRatio.toFixed(3)} modelled → ${ff.equilibriumRatio.toFixed(2)} equilibrium`);
    set('tsg-modes', `${ff.modelledSelfStressModes} modelled · ${ff.equilibriumSelfStressModes} form-found`);
    set('tsg-forces', `${pre.barForce.toFixed(1)} N / ${pre.cableForce.toFixed(1)} N`);
    set('tsg-core', Math.abs(pre.coreForce) < 1e-6*Math.abs(pre.cableForce) ? '0 N (not in the self-stress state)' : `${pre.coreForce.toFixed(1)} N`);
    const safety = v => (v === null || !Number.isFinite(v) ? 'n/a' : `${v.toFixed(0)}×`);
    set('tsg-buckling', safety(ins.bucklingSafety));
    set('tsg-cable', safety(ins.cableSafety));
    set('tsg-minmass', `${mm.structureMass.toFixed(3)} kg (bar r ${(mm.barRadius*1000).toFixed(1)} mm, cable r ${(mm.cableRadius*1000).toFixed(2)} mm)`);
    const breakdown = massBreakdown(this.cfg.material || 'lightweight');
    set('tsg-installed', `${(breakdown.struts+breakdown.cables).toFixed(3)} kg struts + cables`);
    set('tsg-modal', js.modal.elasticHz.slice(0, 5).map(f => f.toFixed(2)).join(', ')+' Hz');
    set('tsg-highest', `${js.modal.highestHz.toFixed(0)} Hz (bar axial)`);
    set('tsgfem-note', ff.usedEquilibriumShape
      ? 'The simulated icosahedral shape (d/h = 1/φ) has no self-stress state, so statics use the TsgFEM form-found expanded octahedron (d/h = 0.5) with the same bars. Bar axial modes sit far above the shell modes, which supports treating struts as rigid in the simulator.'
      : 'The simulated shape is prestressable as modelled.');
    byId('tsgfem-results')?.classList.remove('hidden');
    if (byId('btn-export-tsgfem')) byId('btn-export-tsgfem').disabled = false;
    return js;
  }

  /** Apply ?level=&gait=&controller=&core=&act=&g=&speed=&cam=&preset= on startup. */
  applyUrlState(state = readUrlState()) {
    this.suspendUrlSync = true;
    try {
      if (state.preset === 'research') {
        this.loadExperiment(state.level || 1, { reason: 'preset' });
        this.cfg.applyResearchProfile();
        this.simA.reset(); this.simB.reset();
      } else if (state.level && state.level !== this.cfg.experimentId) {
        this.loadExperiment(state.level, { reason: 'url' });
      }
      const set = (id, value) => { const el = document.getElementById(id); if (el && value !== undefined && value !== null) el.value = value; };
      let rebuild = false;
      if (state.core === 'dae' || state.core === 'legacy') { rebuild ||= this.cfg.physicsCore !== state.core; this.cfg.physicsCore = state.core; }
      if (state.act === 'pure' || state.act === 'assisted') {
        const pure = state.act === 'pure';
        rebuild ||= this.cfg.pureCableActuation !== pure;
        this.cfg.pureCableActuation = pure;
      }
      if (state.controller && document.querySelector(`#select-controller option[value="${CSS.escape(state.controller)}"]`)) {
        rebuild ||= this.cfg.controllerMode !== state.controller;
        this.cfg.controllerMode = state.controller;
        this.applyTippingGaitStrokes();
      }
      if (state.mat && MATERIALS[state.mat] && state.mat !== this.cfg.material) {
        this.setMaterial(state.mat, { rebuild: false });
        rebuild = false;
        this.loadExperiment(this.cfg.experimentId, { reason: 'url' });
      }
      if (state.shield === '1' && !this.cfg.tensionShield) {
        this.cfg.tensionShield = true;
        rebuild = true;
        const button = document.getElementById('btn-toggle-shield');
        if (button) { button.textContent = 'TENSION SHIELD: ON'; button.setAttribute('aria-pressed', 'true'); button.classList.add('is-on'); }
      }
      if (rebuild) { this.simA.reset(); this.simB.reset(); }
      if (state.gait && document.querySelector(`#select-gait-mode option[value="${CSS.escape(state.gait)}"]`)) {
        this.cfg.actuationMode = state.gait; set('select-gait-mode', state.gait);
      }
      if (state.g !== null) this.setGravity(state.g, { source: 'url' });
      if (state.speed !== null && ALLOWED_SPEEDS.includes(state.speed)) { this.simSpeed = state.speed; set('select-speed', String(state.speed)); }
      if (state.cam && ['chase', 'orbit', 'top', 'side'].includes(state.cam)) { set('select-camera', state.cam); this.vis.setCameraMode(state.cam); }
      if (state.tab) this.showDock?.(state.tab);
      this.syncPhysicsControls();
    } finally {
      this.suspendUrlSync = false;
    }
    this.syncUrl();
  }

  exportLevel(level, format) {
    const state = { ...emptyState(), runs: runsForLevel(this.persistence.state, level) };
    if (format === 'json') downloadText(`level${level}_runs.json`, JSON.stringify(state, null, 2), 'application/json;charset=utf-8');
    else downloadText(`level${level}_runs.csv`, toCSV(state));
  }

  resizeVisibleCharts() {
    requestAnimationFrame(() => {
      for (const chart of Object.values(window.Chart?.instances || {})) {
        if (chart.canvas?.offsetParent) chart.resize();
      }
    });
  }

  initSimulation() {
    this.cfg = new SimConfig({
      experimentId: 10,
      terrainLevel: 10,
      actuationMode: 'roll_forward',
      abCourseEnabled: true,
      targetDestination: [0, 60],
      targetGoalY: 60,
      monitoring: { ...(this.centralSettings.monitoring || {}) }
    }).applyLevel10PerformanceProfile();
    // The workstation simulates the steel build by default; SimConfig and the
    // paper/CLI experiments keep the original lightweight parameters.
    applyMaterial(this.cfg, DEFAULT_APP_MATERIAL);
    this.rover = new SphericalRoverModel(this.cfg);
    this.terrain = new TerrainModel(this.cfg);
    this.routeLearner = new AdaptiveRouteLearner({
      deadlineSeconds: this.cfg.missionDeadlineSeconds,
      courseStartY: this.cfg.courseStartY,
      courseGoalY: this.cfg.courseGoalY,
      baseTargetSpeed: this.cfg.targetSpeed
    }, this.loadRouteLearning());

    // Model A: Fixed Tensegrity Baseline
    this.simA = new Simulation(this.cfg, this.rover, this.terrain, 'fixed');

    // Model B: Adaptive Tension & Deformation
    this.simB = new Simulation(this.cfg, this.rover, this.terrain, 'adaptive', this.routeLearner);
  }

  loadRouteLearning() {
    try {
      const raw = window.localStorage.getItem(this.learningStorageKey);
      return raw ? JSON.parse(raw) : null;
    } catch {
      return null;
    }
  }

  persistRouteLearning() {
    if (!this.routeLearner || this.routeLearner.revision === this.lastPersistedLearningRevision) return;
    try {
      window.localStorage.setItem(this.learningStorageKey, JSON.stringify(this.routeLearner.serialize()));
      this.lastPersistedLearningRevision = this.routeLearner.revision;
    } catch {
      // Learning still works in memory when storage is unavailable.
    }
  }

  initVisualizer() {
    const container = document.getElementById('canvas-container-main');
    this.vis = new Visualizer(container, this.rover, this.terrain, {
      showForceOverlay: true,
      showGeometryCheckpoint: this.isGeometryCheckpointMode,
      showDebugLabels: this.showDebugLabels,
      selectedCable: this.selectedCableIndex
    });
  }

  initCharts() {
    installMatlabTheme(Chart);
    setChartTheme('dark', Chart);
    const make = (id, type, datasets, options) => {
      const element = document.getElementById(id);
      if (!element) return null;
      return new Chart(element.getContext('2d'), { type, data: { datasets }, options });
    };
    const L = matlabLine;

    // --- Mission ------------------------------------------------------------
    this.chartG = make('chart-gforce', 'line', [
      L('Payload core |a| (1 s mean)', 0)
    ], matlabLineOptions({ yLabel: 'Proper acceleration (g)', beginAtZero: true }));

    this.chartV = make('chart-velocity', 'line', [
      L('Planar speed (m/s)', 0),
      L('Shell deformation RMS (m)', 1, { borderDash: [6, 4] })
    ], matlabLineOptions({ yLabel: 'Speed (m/s), RMS (m)', beginAtZero: true }));

    this.chartControl = make('chart-control', 'line', [
      L('Model A (baseline)', 0),
      L('Model B (adaptive)', 1)
    ], matlabLineOptions({ yLabel: 'max_k | ‖b_k‖ − l_k | (m)', yLog: true, legend: 'southeast' }));

    this.chartForce = make('chart-force-law', 'line', [
      L('Linear tension', 0),
      L('Hermite relaxing tension', 1)
    ], matlabLineOptions({ xLabel: 'Cable length / rest length (-)', yLabel: 'Tension (N)', beginAtZero: true, legend: 'northwest' }));
    this.updateForceLawChart();

    // --- Physics audit (Lagrangian DAE + work ledger) -------------------------
    this.chartLedger = make('chart-energy-ledger', 'line', [
      L('Cables (actuation + elastic)', 0),
      L('Ground contact', 1),
      L('Drag', 2),
      L('Assists (couple, traction, grip, corridor, virtual)', 3),
      L('Constraint projection / corrector', 6, { borderDash: [6, 4] })
    ], matlabLineOptions({ yLabel: 'Cumulative work W(t) (J)', legend: 'northwest' }));

    this.chartResidual = make('chart-energy-residual', 'line', [
      L('Model A', 0),
      L('Model B', 1)
    ], matlabLineOptions({ yLabel: '|E(t) − E₀ − ΣW(t)| (J)', yLog: true, legend: 'southeast' }));

    this.initMonitoringCharts();
    this.chartRegistry = {
      'goal-error': this.chartGoalError, 'formation-error': this.chartFormationError,
      'cable-delta': this.chartCableDelta, 'cable-force': this.chartCableForce,
      'cable-bars': this.chartCableBars, 'contact-count': this.chartContactCount,
      'com-path': this.chartComPath, gforce: this.chartG, velocity: this.chartV,
      control: this.chartControl, 'force-law': this.chartForce,
      'energy-ledger': this.chartLedger, 'energy-residual': this.chartResidual
    };
  }

  initMonitoringCharts() {
    const make = (id, type, datasets, options) => {
      const element = document.getElementById(id);
      if (!element) return null;
      return new Chart(element.getContext('2d'), { type, data: { datasets }, options });
    };
    const L = matlabLine;
    this.chartGoalError = make('chart-goal-error', 'line', [
      L('||Goal - COM(t)||', 0),
      L('Success threshold', 1, { borderDash: [6, 4], borderWidth: 1 })
    ], matlabLineOptions({ yLabel: 'Distance to goal (m)', beginAtZero: true }));
    this.chartFormationError = make('chart-formation-error', 'line', [
      L('Rigid-aligned RMS', 0),
      L('Warning threshold', 1, { borderDash: [6, 4], borderWidth: 1 })
    ], matlabLineOptions({ yLabel: 'Formation error (m)', beginAtZero: true }));
    this.chartCableDelta = make('chart-cable-delta', 'line', [
      L('Selected cable ΔL', 0),
      L('Zero', 7, { borderDash: [3, 3], borderWidth: 1, borderColor: '#808080', hideInLegend: true })
    ], matlabLineOptions({ yLabel: 'Length change (m)' }));
    this.chartCableForce = make('chart-cable-force', 'line', [
      L('Selected cable', 0),
      L('Mean of 24 cables', 1),
      L('Overload limit', 6, { borderDash: [6, 4], borderWidth: 1 })
    ], matlabLineOptions({ yLabel: 'Tension (N)', beginAtZero: true }));
    this.chartContactCount = make('chart-contact-count', 'line', [
      L('Active contacts', 0, { stepped: true })
    ], matlabLineOptions({ yLabel: 'Contacts (-)', beginAtZero: true, yTicks: { stepSize: 1, precision: 0 } }));
    this.chartComPath = make('chart-com-path', 'scatter', [
      { ...L('COM trajectory', 0), showLine: true },
      { ...L('Start A', 4), showLine: false, pointRadius: 5 },
      { ...L('Goal B', 1), showLine: false, pointRadius: 6, pointStyle: 'rectRot' },
      { ...L('Obstacles', 2), showLine: false, pointRadius: 4 }
    ], matlabLineOptions({ xLabel: 'x (m)', yLabel: 'y (m)', legend: 'southeast' }));
    const barOptions = matlabLineOptions({ xLabel: 'Cable', yLabel: 'Tension (N)', beginAtZero: true });
    barOptions.scales.x.type = 'category';
    barOptions.scales.x.ticks = { ...barOptions.scales.x.ticks, autoSkip: false, maxRotation: 90, maxTicksLimit: 30, font: { size: 9 } };
    this.chartCableBars = make('chart-cable-bars', 'bar', [
      { label: 'Tension', data: [], backgroundColor: [], borderWidth: 0, barPercentage: 0.9, categoryPercentage: 0.9 },
      { type: 'line', ...L('Overload limit', 6, { borderDash: [6, 4], borderWidth: 1 }) }
    ], barOptions);
  }

  initMonitoringControls() {
    const cableSelect = document.getElementById('select-monitor-cable');
    if (cableSelect) {
      cableSelect.innerHTML = this.rover.outerStrings.map((_, index) =>
        `<option value="${index}">C${String(index+1).padStart(2, '0')}</option>`).join('');
      cableSelect.value = String(this.selectedCableIndex);
    }
    const thresholdInput = document.getElementById('input-goal-threshold');
    if (thresholdInput) {
      const threshold = this.cfg.monitoring.goalThreshold;
      thresholdInput.value = String(threshold);
      const value = document.getElementById('val-goal-threshold');
      if (value) value.textContent = `${threshold.toFixed(2)} m`;
    }
  }

  updateForceLawChart() {
    if (!this.chartForce || !this.rover?.l0_outerStrings?.length) return;
    const base = this.rover.l0_outerStrings[0];
    const standard = [];
    const adaptive = [];
    for (let ratio = 0.90; ratio <= 1.60 + 1e-9; ratio += 0.01) {
      const ell = base * ratio;
      standard.push({ x: ratio, y: this.cfg.kS * Math.max(0, ell-base) });
      adaptive.push({ x: ratio, y: relaxedCableTension(ell, base, base, Object.assign({}, this.cfg, { stringRelaxation: true })).tension });
    }
    this.chartForce.data.datasets[0].data = standard;
    this.chartForce.data.datasets[1].data = adaptive;
    this.chartForce.update('none');
  }

  handleAdaptiveLearningCycle(now) {
    this.persistRouteLearning();
    const bothModelsAtGoal = canStartNextTrainingAttempt({
      autoLearningEnabled: this.autoLearningEnabled,
      experimentId: this.cfg.experimentId,
      metricsA: this.simA.metrics,
      metricsB: this.simB.metrics
    });
    if (!bothModelsAtGoal) {
      this.learningRestartAt = null;
      return;
    }
    if (this.learningRestartAt === null) this.learningRestartAt = now+2500;
    if (now < this.learningRestartAt) return;
    this.simA.reset();
    this.simB.reset();
    this.accumulatedTime = 0;
    this.lastChartStep = 0;
    this.learningRestartAt = null;
  }

  animationLoop(timestamp) {
    requestAnimationFrame((t) => this.animationLoop(t));

    if (!this.lastFrameTime) this.lastFrameTime = timestamp || performance.now();
    const now = timestamp || performance.now();
    const realDeltaSec = Math.min(0.05, (now - this.lastFrameTime) / 1000.0);
    this.lastFrameTime = now;

    if (this.isPlaying && !this.isGeometryCheckpointMode) {
      // Clock-driven fixed-step accumulator. Playback speed changes simulated
      // time, never the 1 ms physics integration step.
      const targetSimDelta = realDeltaSec * (this.simSpeed || 1.0);
      this.accumulatedTime = (this.accumulatedTime || 0) + targetSimDelta;

      let safetyGuard = 0;
      const frameStart = performance.now();
      // Budget ~24 ms of physics per frame; if the machine cannot keep up,
      // drop the backlog instead of letting it grow (spiral of death).
      while (this.accumulatedTime >= this.cfg.dt && safetyGuard < 400 && performance.now()-frameStart < 24) {
        if (this.simA.t < this.cfg.T_end && !this.simA.metrics.runTerminal) this.simA.step();
        if (this.simB.t < this.cfg.T_end && !this.simB.metrics.runTerminal) this.simB.step();
        this.accumulatedTime -= this.cfg.dt;
        safetyGuard++;
      }
      if (this.accumulatedTime > 4*this.cfg.dt) this.accumulatedTime = 0;
      const simulated = safetyGuard*this.cfg.dt;
      this.rtfFilter = 0.9*(this.rtfFilter ?? 1)+0.1*(realDeltaSec > 0 ? simulated/realDeltaSec : 1);
      if (now-(this.lastRtfPaint || 0) > 500) {
        this.lastRtfPaint = now;
        const rtf = document.getElementById('sim-rtf');
        if (rtf) rtf.textContent = `${this.rtfFilter.toFixed(2)}×`;
      }
    }

    this.handleAdaptiveLearningCycle(now);
    // Run recording: checkpoint every 1 s of simulated time, commit on outcome.
    this.recorder.tick();
    // Live matrix row + run chip at 4 Hz (full re-render only on store changes).
    if (now-(this.lastMatrixPaint || 0) > 250) {
      this.lastMatrixPaint = now;
      this.matrix.updateLive();
      this.updateRunChip();
    }

    // Render 3D Scene Dual
    this.vis.updateDual({
      simA: { q: this.simA.q, diag: this.simA.currentDiag },
      simB: { q: this.simB.q, diag: this.simB.currentDiag }
    });

    this.updateHUD();
    this.updateCharts();
  }

  static controllerShortLabel(mode) {
    const map = {
      natural_support_face: 'Natural', cpg: 'CPG',
      lqr: 'LQR', lqr_payload: 'LQR+P',
      ilqr: 'iLQR', ilqr_minimax: 'MM-iLQR', ilqr_minimax_penalty: 'MM-iLQR+P',
      qp_mpc: 'MPC', qp_mpc_payload: 'MPC+P', neural: 'NN',
      riccati_lqr: 'Ric-LQR', ilqr_true: 'iLQR-T',
      ilqr_minimax_true: 'MM-T', qp_mpc_proj: 'QP-MPC', mpc_gnss: 'MPC+GNSS', guided_policy: 'MPC+GPS', residual_mppi: 'MPC+RL', residual_mppi_llm: 'MPC+RL+LLM'
    };
    return map[mode] || mode || '—';
  }

  // Single entry point for every gravity change (top-bar preset, sidebar
  // slider). Writes cfg.gravity live, keeps the slider, readout, and preset
  // dropdown in sync, and refreshes telemetry immediately.
  setGravity(valueG, { source } = {}) {
    const val = Math.max(0, Math.min(20, valueG));
    this.cfg.gravity = [0.0, 0.0, -val];
    const slider = document.getElementById('slider-gravity');
    const readout = document.getElementById('val-gravity');
    if (slider && source !== 'slider') {
      slider.value = String(val);
    }
    let label = `${val.toFixed(2)} m/s²`;
    if (Math.abs(val-9.81) < 0.1) label = "9.81 m/s² (Earth)";
    else if (Math.abs(val-3.721) < 0.1) label = "3.721 m/s² (Mars)";
    else if (Math.abs(val-1.625) < 0.1) label = "1.625 m/s² (Moon)";
    else if (val === 0) label = "0.00 m/s² (Zero-G)";
    else label = `${label} (Custom)`;
    if (readout) readout.textContent = label;

    // Reflect the nearest matching preset in the top bar; anything else is
    // Custom.
    const presetSelect = document.getElementById('select-gravity-preset');
    if (presetSelect && source !== 'preset') {
      const match = Math.abs(val-9.81) < 0.1 ? 'earth'
        : Math.abs(val-3.721) < 0.1 ? 'mars'
        : Math.abs(val-1.625) < 0.1 ? 'moon' : 'custom';
      presetSelect.value = match;
    }

    // The monitoring charts re-sample every frame and pick up the new gravity
    // on their own; the URL reflects the setup.
    this.syncUrl();
  }

  updateHUD() {
    const timeElem = document.getElementById('sim-time');
    if (timeElem) {
      const attempt = currentTrainingAttemptNumber(this.routeLearner?.runCount, this.simB.metrics);
      const limit = this.terrain.course ? (this.cfg.missionDeadlineSeconds || this.cfg.T_end) : this.cfg.T_end;
      const attemptTime = Math.max(this.simA.t, this.simB.t);
      timeElem.textContent = `${attemptTime.toFixed(2)} s / ${limit} s${this.terrain.course ? ` · try ${attempt}` : ''}`;
    }

    const diagA = this.simA.currentDiag;
    const diagB = this.simB.currentDiag;
    const mB = this.simB.metrics;
    const mA = this.simA.metrics;

    // Model A HUD
    const vA = diagA.velocityVector;
    const speedA = vA ? Math.hypot(vA[0], vA[1]) : 0;
    const elAVel = document.getElementById('hud-a-vel'); if (elAVel) elAVel.textContent = `${speedA.toFixed(2)} m/s`;

    const elAOmega = document.getElementById('hud-a-omega'); if (elAOmega) elAOmega.textContent = `${(diagA.angularVelocity||0).toFixed(2)} rad/s`;
    const maxTA = diagA.outerCableForces ? Math.max(...diagA.outerCableForces) : 0;
    const elATension = document.getElementById('hud-a-tension'); if (elATension) elATension.textContent = `${maxTA.toFixed(1)} N`;
    const elADeform = document.getElementById('hud-a-deform'); if (elADeform) elADeform.textContent = `${(diagA.deformationRMS||0).toFixed(3)} m`;
    const elAGait = document.getElementById('hud-a-gait'); if (elAGait) elAGait.textContent = diagA.state || 'PASSIVE_SETTLE';

    // Model B HUD
    const vB = diagB.velocityVector;
    const speedB = vB ? Math.hypot(vB[0], vB[1]) : 0;
    const elBVel = document.getElementById('hud-b-vel'); if (elBVel) elBVel.textContent = `${speedB.toFixed(2)} m/s`;

    const elBOmega = document.getElementById('hud-b-omega'); if (elBOmega) elBOmega.textContent = `${(diagB.angularVelocity||0).toFixed(2)} rad/s`;
    const maxTB = diagB.outerCableForces ? Math.max(...diagB.outerCableForces) : 0;
    const elBTension = document.getElementById('hud-b-tension'); if (elBTension) elBTension.textContent = `${maxTB.toFixed(1)} N`;
    const elBDeform = document.getElementById('hud-b-deform'); if (elBDeform) elBDeform.textContent = `${(diagB.deformationRMS||0).toFixed(3)} m`;
    const elBGait = document.getElementById('hud-b-gait'); if (elBGait) elBGait.textContent = diagB.state || 'ROLLING';
    const setText = (id, value) => { const el = document.getElementById(id); if (el) el.textContent = value; };
    const controllerDisplay = `${diagB.controllerLabel || 'CPG Baseline'}${diagB.neuralFallback ? ' (geometry fallback)' : ''}`;
    setText('hud-b-controller', controllerDisplay);
    setText('hud-b-constraint', `${((diagB.constraintError || 0)*1000).toFixed(3)} mm`);
    setText('drive-controller-status', controllerDisplay);
    setText('drive-control-cost', (diagB.controlCost || 0).toFixed(3));
    setText('drive-constraint-error', `${((diagB.constraintError || 0)*1000).toFixed(3)} mm`);
    const terrainClearanceMm = (diagB.terrainClearance || 0)*1000;
    setText('drive-terrain-clearance', `${terrainClearanceMm.toFixed(3)} mm`);
    const face = (diagB.supportFace || []).map(node => `N${node+1}`).join(',');
    const edge = (diagB.targetEdge || []).map(node => `N${node+1}`).join(',');
    setText('drive-support-face', `${face || '—'} / ${edge || '—'}`);
    setText('drive-com-margin', Number.isFinite(diagB.comMargin) ? `${diagB.comMargin.toFixed(3)} m` : '—');
    setText('drive-slip-speed', `${(diagB.slipSpeed || 0).toFixed(3)} m/s`);
    setText('drive-rolling-ratio', (diagB.rollingRatio || 0).toFixed(2));
    setText('drive-kinetic-energy', `${(diagB.kineticEnergy || 0).toFixed(3)} J`);
    setText('drive-completed-rolls', String(diagB.completedRolls || 0));
    setText('drive-active-cables', `${diagB.activeCableCount || 0}C / ${this.rover.outerStrings.length} · ${diagB.activeRodCount || 0}R / ${this.rover.bars.length}`);
    setText('drive-relaxation', `${Math.round((diagB.relaxationFraction || 0)*100)}%`);
    const shield = this.simB?.sgt?.shieldInfo;
    setText('drive-shield', !this.cfg.tensionShield ? 'off'
      : shield ? `${shield.active} active · min payload T ${shield.minCoreTension.toFixed(1)} N · Δu ${(1000*shield.correction).toFixed(1)} mm${shield.shiftDropped ? ' · shift relaxed' : ''}`
        : 'on (tipping-gait controllers)');
    const gnss = this.simB?.controlDiagnostics?.gnss;
    setText('drive-gnss', gnss
      ? `est. err ${gnss.error.toFixed(2)} m · σ ${gnss.sigma.toFixed(2)} m · e ${gnss.crossTrack.toFixed(2)} m · ${gnss.fixes} fixes`
      : 'off (select MPC + GNSS)');
    const summaryA = mA.obstacleSummary || { over: 0, around: 0, retries: 0 };
    const summaryB = mB.obstacleSummary || { over: 0, around: 0, retries: 0, bypassViolations: 0 };
    setText('hud-a-obstacles', `${summaryA.over} over / ${summaryA.around} around`);
    setText('hud-b-obstacles', `${summaryB.over} over / ${summaryB.around} around`);
    setText('hud-b-bypass', `${summaryB.bypassViolations || 0}`);
    setText('course-current-obstacle', diagB.activeObstacleId || 'none');
    const expedition = mB.expeditionWaypoints?.total > 0 ? mB.expeditionWaypoints : null;
    setText('course-checkpoints', expedition
      ? `${expedition.reached} / ${expedition.total} waypoints`
      : `${summaryB.checkpointsReached || 0} / ${summaryB.total || 10}`);
    setText('course-phase', diagB.obstaclePhase || 'cruise');
    const courseY = diagB.centroid?.[1] || 0;
    const startLineY = this.cfg.courseStartY || 10;
    const goalLineY = this.cfg.courseGoalY || this.cfg.targetGoalY;
    const measuredLength = goalLineY-startLineY;
    setText('course-progress', courseY < startLineY
      ? `approach y=${courseY.toFixed(2)} / ${startLineY.toFixed(2)} m`
      : `${Math.max(0, Math.min(measuredLength, courseY-startLineY)).toFixed(1)} / ${measuredLength.toFixed(1)} m`);
    this.updateMonitoringHUD(diagB.monitoring);
    // Live gravity telemetry: reacts instantly to the top-bar preset or the
    // sidebar slider, on every terrain level.
    const gravityNow = Number.isFinite(diagB.monitoring?.gravity)
      ? diagB.monitoring.gravity : Math.abs(this.cfg.gravity?.[2] ?? 9.81);
    setText('monitor-gravity',
      `g = ${gravityNow.toFixed(3)} m/s² · rover weight ≈ ${(diagB.monitoring?.weightForce ?? 0).toFixed(1)} N`);
    // Tables and learner text at 4 Hz: rebuilding them every frame costs
    // layout time that the physics needs.
    const now = performance.now();
    if (now-(this.lastTablePaint || 0) > 250) {
      this.lastTablePaint = now;
      this.updateLearningHUD();
      this.updateABComparisonTable();
    }
  }

  updateMonitoringHUD(monitoring) {
    if (!monitoring) return;
    const setText = (id, value) => { const element = document.getElementById(id); if (element) element.textContent = value; };
    setText('monitor-start', formatVector(monitoring.start));
    setText('monitor-goal', formatVector(monitoring.goal));
    setText('monitor-com', formatVector(monitoring.com));
    setText('monitor-goal-error', `${monitoring.goalError.toFixed(3)} m`);
    setText('monitor-remaining', `${monitoring.remainingX.toFixed(2)} / ${monitoring.remainingY.toFixed(2)} m`);
    setText('monitor-formation', `${monitoring.formationError.toFixed(4)} m`);
    setText('monitor-stability', `${monitoring.stabilityLevel.toUpperCase()} · ${monitoring.status}`);
    setText('monitor-rod-error', `${(monitoring.maximumRodError*1000).toFixed(3)} mm`);
    setText('monitor-node-separation', `${monitoring.maximumNodeSeparation.toFixed(3)} m`);
    setText('monitor-clearance', `${(monitoring.terrainClearance*1000).toFixed(3)} mm`);
    setText('monitor-force-summary', `${monitoring.maximumCableForce.toFixed(1)} / ${monitoring.averageCableForce.toFixed(1)} N`);
    setText('monitor-run-peaks', `${(monitoring.peakCableForce ?? 0).toFixed(1)} N · ${(monitoring.peakCableStrainPercent ?? 0).toFixed(1)}%`);
    setText('monitor-max-strain', `${monitoring.maximumCableStrain.toFixed(2)}%`);
    setText('monitor-cable-counts', `${monitoring.slackCableCount} / ${monitoring.overloadedCableCount}`);
    setText('monitor-grounded', monitoring.grounded ? 'GROUNDED' : 'AIRBORNE');
    setText('monitor-contact-count', String(monitoring.activeContactCount));
    setText('monitor-contact-ids', monitoring.contacts.map(contact => contact.id).join(', ') || '—');
    setText('monitor-log-count', `${this.simB.monitor?.records.length || 0} raw samples`);

    const statusElement = document.getElementById('monitor-status');
    if (statusElement) {
      const missionOutcome = this.simB.metrics.runOutcome;
      statusElement.textContent = missionOutcome === 'win' ? 'MISSION WIN'
        : missionOutcome === 'loss' ? 'MISSION LOSS' : monitoring.status.toUpperCase();
      statusElement.classList.remove('status-stable', 'status-moderate', 'status-danger', 'status-goal');
      statusElement.classList.add(missionOutcome === 'win' ? 'status-goal'
        : missionOutcome === 'loss' ? 'status-danger'
        : monitoring.stabilityLevel === 'green' ? 'status-stable'
          : monitoring.stabilityLevel === 'yellow' ? 'status-moderate' : 'status-danger');
    }
    const goalResult = document.getElementById('monitor-goal-result');
    if (goalResult) {
      const metrics = this.simB.metrics;
      if (metrics.runOutcome === 'win') {
        const bothModelsAtGoal = bothModelsReachedGoals(this.simA.metrics, metrics);
        goalResult.textContent = bothModelsAtGoal
          ? `WIN · goal reached in ${metrics.completionTime.toFixed(2)} s · both models at goal · next training starts soon`
          : `WIN · goal reached in ${metrics.completionTime.toFixed(2)} s · waiting for Model A before next training`;
        goalResult.className = 'note note-win';
      } else if (metrics.runOutcome === 'loss') {
        const remaining = Math.max(0, this.cfg.courseGoalY-(this.simB.currentDiag.centroid?.[1] || 0));
        const deadline = this.cfg.missionDeadlineSeconds || this.cfg.T_end;
        goalResult.textContent = `LOSS · deadline (${deadline.toFixed(0)} s) exceeded · ${remaining.toFixed(2)} m remaining · auto training waits until both models reach their goals`;
        goalResult.className = 'note note-bad';
      } else {
        const deadline = this.cfg.missionDeadlineSeconds || this.cfg.T_end;
        goalResult.textContent = `Attempt ${this.routeLearner?.runCount+1 || 1} running · reach Goal B before ${deadline.toFixed(0)} s`;
        goalResult.className = 'note';
      }
    }
    const warningElement = document.getElementById('monitor-warnings');
    if (warningElement) {
      warningElement.textContent = monitoring.warnings.length
        ? `WARNING · ${monitoring.warnings.join(' · ')}` : 'No structural warnings';
      warningElement.className = monitoring.warnings.length ? 'note note-bad' : 'note note-ok';
    }

    const cable = monitoring.cables[this.selectedCableIndex] || monitoring.cables[0];
    if (cable) {
      setText('cable-inspector-nodes', `N${cable.nodeA+1} ↔ N${cable.nodeB+1}`);
      setText('cable-inspector-lengths', `${cable.restLength.toFixed(4)} / ${cable.currentLength.toFixed(4)} m`);
      setText('cable-inspector-delta', `${cable.deltaLength >= 0 ? '+' : ''}${cable.deltaLength.toFixed(5)} m`);
      setText('cable-inspector-strain', `${cable.strainPercent.toFixed(2)}%`);
      setText('cable-inspector-force', `${cable.force.toFixed(2)} N`);
      setText('cable-inspector-state', cable.state.toUpperCase());
      const stateElement = document.getElementById('cable-inspector-state');
      if (stateElement) stateElement.className = cable.state === 'overload' ? 'c-bad'
        : cable.state === 'slack' ? 'c-slack' : cable.state === 'moderate' || cable.state === 'high' ? 'c-warn' : 'c-ok';
    }
    const contactList = document.getElementById('monitor-contact-list');
    if (contactList) {
      const activeEvents = this.simB.monitor?.activeContactEvents;
      contactList.innerHTML = monitoring.contacts.length ? monitoring.contacts.map(contact => {
        const friction = Math.hypot(...(contact.frictionForce || [0, 0, 0]));
        const event = activeEvents?.get(`${contact.kind || 'node'}:${contact.id}`);
        const duration = event ? ` · ${(monitoring.time-event.startTime).toFixed(2)} s` : '';
        return `<div class="kv"><span>${contact.id} · ${contact.objectId}${duration}</span><span>${contact.normalForce.toFixed(1)} N · ${friction.toFixed(1)} N fr</span></div>`;
      }).join('') : 'No active contacts';
    }
  }

  updateLearningHUD() {
    const learning = this.simB.currentDiag.learning || this.routeLearner?.snapshot(this.simB.learningCommand);
    if (!learning) return;
    const setText = (id, value) => {
      const element = document.getElementById(id);
      if (element) element.textContent = value;
    };
    const command = learning.currentCommand || this.simB.learningCommand || {};
    const lastRun = learning.lastRun;
    setText('learning-attempt', String(currentTrainingAttemptNumber(learning.runCount, this.simB.metrics)));
    setText('learning-record', `${learning.wins} W / ${learning.losses} L`);
    setText('learning-best', learning.bestTime === null ? 'No completed win yet' : `${learning.bestTime.toFixed(2)} s`);
    setText('learning-segment', `${command.segmentLabel || 'S01'} · waypoint x=${(command.waypointX || 0).toFixed(2)} m`);
    setText('learning-scales', `${(command.speedScale || 1).toFixed(3)} speed · ${(command.torqueScale || 1).toFixed(3)} torque · ${(command.tractionScale || 1).toFixed(3)} grip`);
    setText('learning-last-result', lastRun
      ? `${lastRun.outcome.toUpperCase()} · ${lastRun.time.toFixed(2)} s · loss ${lastRun.loss.toFixed(3)}`
      : 'Waiting for first completed attempt');
    const gradient = lastRun?.gradient;
    setText('learning-gradient', gradient
      ? `Δ speed ${gradient.speed >= 0 ? '+' : ''}${gradient.speed.toFixed(4)} · Δ torque ${gradient.torque >= 0 ? '+' : ''}${gradient.torque.toFixed(4)}`
      : 'Gradient update after win/loss');
    const autoButton = document.getElementById('btn-toggle-auto-learning');
    if (autoButton) autoButton.textContent = `Auto training: ${this.autoLearningEnabled ? 'ON' : 'OFF'}`;
  }

  updateABComparisonTable() {
    const body = document.getElementById('ab-comparison-body');
    if (!body) return;
    const renderModel = (label, metrics, rowClass) => {
      const obstacles = metrics.obstacleSummary || { over: 0, around: 0, retries: 0, bypassViolations: 0 };
      const deadline = this.cfg.missionDeadlineSeconds || this.cfg.T_end;
      const completion = metrics.runOutcome === 'loss' ? `LOSS · ${deadline.toFixed(0)} s`
        : metrics.completionTime === null ? 'running' : `WIN · ${metrics.completionTime.toFixed(1)} s`;
      const f = (value, digits, unit = '') => (Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : '—');
      return `<tr class="${rowClass}">
        <td>${label}</td>
        <td>${f(metrics.measuredDistance, 2, ' m')}</td>
        <td>${completion}</td>
        <td>${f(metrics.avgVelocity, 3, ' m/s')}</td>
        <td>${f(metrics.speedVariance, 5)}</td>
        <td>${f(metrics.lateralTravel, 2, ' m')}</td>
        <td>${obstacles.over}</td>
        <td>${obstacles.around}</td>
        <td>${obstacles.retries}</td>
        <td class="${obstacles.bypassViolations ? 'c-bad' : ''}">${obstacles.bypassViolations || 0}</td>
        <td>${f(metrics.energyCost, 2, ' J')}</td>
        <td>${f(metrics.payloadAccelMax, 2, ' g')}</td>
      </tr>`;
    };
    body.innerHTML = renderModel('Model A · baseline', this.simA.metrics, 'row-a')+
      renderModel('Model B · adaptive', this.simB.metrics, 'row-b');
  }

  updateCharts() {
    if (this.simB.stepCount - (this.lastChartStep || 0) >= this.chartUpdateIntervalSteps) {
      this.lastChartStep = this.simB.stepCount;
      const hist = this.simB.history;
      // 20-second window (history samples arrive every 40 ms of sim time).
      const start = Math.max(0, hist.t.length-500);
      const t = hist.t.slice(start);
      const win = series => series.slice(start);
      const set = (chart, index, data) => { if (chart) chart.data.datasets[index].data = data; };

      set(this.chartG, 0, xy(t, movingAverage(win(hist.centroidAccel), 25)));
      this.chartG?.update('none');
      set(this.chartV, 0, xy(t, movingAverage(win(hist.planarSpeed), 25)));
      set(this.chartV, 1, xy(t, movingAverage(win(hist.deformation), 25)));
      this.chartV?.update('none');

      const histA = this.simA.history;
      const startA = Math.max(0, histA.t.length-500);
      const positive = value => Math.max(Math.abs(value), 1e-16);
      set(this.chartControl, 0, xy(histA.t.slice(startA), histA.constraintError.slice(startA), positive));
      set(this.chartControl, 1, xy(t, win(hist.constraintError), positive));
      this.chartControl?.update('none');

      if (this.chartLedger && hist.cableWork) {
        set(this.chartLedger, 0, xy(t, win(hist.cableWork)));
        set(this.chartLedger, 1, xy(t, win(hist.contactWork)));
        set(this.chartLedger, 2, xy(t, win(hist.dragWork || [])));
        set(this.chartLedger, 3, xy(t, win(hist.assistWork)));
        set(this.chartLedger, 4, xy(t, win(hist.constraintWork)));
        this.chartLedger.update('none');
      }
      if (this.chartResidual && hist.energyResidual) {
        set(this.chartResidual, 0, xy(histA.t.slice(startA), (histA.energyResidual || []).slice(startA), positive));
        set(this.chartResidual, 1, xy(t, win(hist.energyResidual), positive));
        this.chartResidual.update('none');
      }

      this.updateMonitoringCharts();
      this.updatePhysicsAudit();
    }
  }

  updateMonitoringCharts() {
    const monitor = this.simB.monitor;
    if (!monitor?.history?.t.length) return;
    const history = monitor.history;
    const windowSamples = Math.ceil(monitor.settings.chartWindowSeconds/monitor.settings.chartSamplePeriod);
    const start = Math.max(0, history.t.length-windowSamples);
    const t = history.t.slice(start);
    const constant = value => (t.length ? [{ x: t[0], y: value }, { x: t[t.length-1], y: value }] : []);
    const cableDelta = history.cableDelta.slice(start).map(values => values[this.selectedCableIndex] || 0);
    const cableForce = history.cableForce.slice(start).map(values => values[this.selectedCableIndex] || 0);
    const cableAverage = history.cableForce.slice(start).map(values =>
      values.reduce((sum, value) => sum+value, 0)/Math.max(1, values.length));
    const set = (chart, index, data) => { if (chart) chart.data.datasets[index].data = data; };

    set(this.chartGoalError, 0, xy(t, movingAverage(history.goalError.slice(start), 3)));
    set(this.chartGoalError, 1, constant(monitor.settings.goalThreshold));
    this.chartGoalError?.update('none');

    set(this.chartFormationError, 0, xy(t, movingAverage(history.formationError.slice(start), 3)));
    set(this.chartFormationError, 1, constant(monitor.settings.formationWarningThreshold));
    this.chartFormationError?.update('none');

    set(this.chartCableDelta, 0, xy(t, movingAverage(cableDelta, 3)));
    set(this.chartCableDelta, 1, constant(0));
    this.chartCableDelta?.update('none');

    set(this.chartCableForce, 0, xy(t, movingAverage(cableForce, 3)));
    set(this.chartCableForce, 1, xy(t, movingAverage(cableAverage, 3)));
    set(this.chartCableForce, 2, constant(monitor.settings.cableOverloadForce));
    this.chartCableForce?.update('none');

    set(this.chartContactCount, 0, xy(t, history.contactCount.slice(start)));
    this.chartContactCount?.update('none');

    const pathStart = Math.max(0, history.t.length-2500);
    set(this.chartComPath, 0, history.comX.slice(pathStart).map((x, index) => ({ x, y: history.comY[pathStart+index] })));
    set(this.chartComPath, 1, [{ x: monitor.start[0], y: monitor.start[1] }]);
    set(this.chartComPath, 2, [{ x: monitor.goal[0], y: monitor.goal[1] }]);
    const obstacles = [...(this.terrain.course?.obstacles
      || (this.terrain.rocks || []).filter(rock => rock.kind === 'challenge')), ...(this.terrain.obstacleField || [])];
    set(this.chartComPath, 3, obstacles.map(obstacle => ({ x: obstacle.x, y: obstacle.y })));
    this.chartComPath?.update('none');

    const cableId = `C${String(this.selectedCableIndex+1).padStart(2, '0')}`;
    const deltaLabel = document.getElementById('chart-cable-delta-label');
    if (deltaLabel) deltaLabel.textContent = `${cableId} · ΔL [m]`;
    const forceLabel = document.getElementById('chart-cable-force-label');
    if (forceLabel) forceLabel.textContent = `${cableId} · tension [N]`;

    // Instantaneous force distribution (MATLAB colours by cable state).
    const stateColors = {
      slack: '#7E2F8E', overload: '#A2142F', high: '#D95319',
      moderate: '#EDB120', nominal: '#0072BD'
    };
    const liveCables = monitor.latest?.cables || [];
    if (liveCables.length && this.chartCableBars) {
      this.chartCableBars.data.labels = liveCables.map(cable => cable.id);
      this.chartCableBars.data.datasets[0].data = liveCables.map(cable => cable.force);
      this.chartCableBars.data.datasets[0].backgroundColor =
        liveCables.map(cable => stateColors[cable.state] || '#0072BD');
      this.chartCableBars.data.datasets[1].data =
        new Array(liveCables.length).fill(monitor.settings.cableOverloadForce);
      this.chartCableBars.update('none');
    }
  }

  /** Physics-audit panel: work ledger, residual, constraint error, assist share. */
  updatePhysicsAudit() {
    const panel = document.getElementById('physics-audit-body');
    if (!panel) return;
    const fmt = (value, digits = 3) => Number.isFinite(value) ? value.toFixed(digits) : '—';
    const sci = value => Number.isFinite(value) ? value.toExponential(2) : '—';
    const row = (label, a, b, cls = '') =>
      `<tr><td class="audit-label">${label}</td><td class="${cls}">${a}</td><td class="${cls}">${b}</td></tr>`;
    const diag = sim => sim.currentDiag?.energyLedger;
    const A = diag(this.simA), B = diag(this.simB);
    if (!A || !B) return;
    const work = (ledger, key) => fmt(ledger.work[key], 2);
    const share = ledger => `${(100*ledger.assistShare).toFixed(1)} %`;
    panel.innerHTML = [
      row('Physics core', this.simA.currentDiag.physicsCore === 'dae' ? 'Lagrangian DAE' : 'Legacy SHAKE',
        this.simB.currentDiag.physicsCore === 'dae' ? 'Lagrangian DAE' : 'Legacy SHAKE'),
      row('Actuation', this.cfg.pureCableActuation ? 'cable only' : 'with assists',
        this.cfg.pureCableActuation ? 'cable only' : 'with assists'),
      row('E(t) [J]', fmt(A.E), fmt(B.E)),
      row('|ε| = |E − E₀ − ΣW| [J]', sci(Math.abs(A.residual)), sci(Math.abs(B.residual)), 'mono'),
      row('max |ε| [J]', sci(A.maxAbsResidual), sci(B.maxAbsResidual), 'mono'),
      // Before the corrector: what the integrator could not explain this step.
      row('max per-step drift [J]', sci(A.maxStepDrift), sci(B.maxStepDrift), 'mono'),
      row('Σ|drift| (corrected) [J]', fmt(A.driftAbs, 3), fmt(B.driftAbs, 3)),
      row('max bar error [m]', sci(this.simA.metrics.maxConstraintError), sci(this.simB.metrics.maxConstraintError), 'mono'),
      row('W cables [J]', work(A, 'cables'), work(B, 'cables')),
      row('W contact [J]', work(A, 'contact'), work(B, 'contact')),
      row('W drag [J]', work(A, 'drag'), work(B, 'drag')),
      row('W assists [J]', fmt(A.work.assist+A.work.virtual, 2), fmt(B.work.assist+B.work.virtual, 2)),
      row('W constraint proj. [J]', fmt(A.work.constraint+A.work.rodPenalty, 2), fmt(B.work.constraint+B.work.rodPenalty, 2)),
      row('Algorithm-1 ΔW [J]', sci(A.work.corrector), sci(B.work.corrector), 'mono'),
      row('Actuator energy [J]', fmt(this.simA.metrics.energyCost, 1), fmt(this.simB.metrics.energyCost, 1)),
      row('Assist share of input', share(A), share(B), 'strong')
    ].join('');
  }

  updateMetricsTable() {
    this.matrix?.render();
  }

  /**
   * Load a terrain level into physics, 3D view, inspector, matrix and URL.
   * The open run is committed first, so switching levels never loses data.
   */
  loadExperiment(expId, { reason = 'level-switch' } = {}) {
    this.recorder?.commit(reason);
    configureLevel(this.cfg, expId);
    this.applyTippingGaitStrokes();
    const gaitSelect = document.getElementById('select-gait-mode');
    if (gaitSelect) gaitSelect.value = this.cfg.actuationMode;
    const experimentSelect = document.getElementById('select-experiment');
    if (experimentSelect) experimentSelect.value = String(expId);
    const speedSlider = document.getElementById('slider-target-speed');
    if (speedSlider) speedSlider.value = String(this.cfg.targetSpeed);
    const speedValue = document.getElementById('val-target-speed');
    if (speedValue) speedValue.textContent = `${this.cfg.targetSpeed.toFixed(2)} m/s`;
    const roughnessSlider = document.getElementById('slider-roughness');
    if (roughnessSlider) roughnessSlider.value = String(this.cfg.groundRMS);
    const roughnessValue = document.getElementById('val-roughness');
    if (roughnessValue) roughnessValue.textContent = `${this.cfg.groundRMS.toFixed(3)} m`;
    const controlRate = document.getElementById('control-rate-value');
    if (controlRate) controlRate.textContent = `${Math.round(1/this.cfg.controllerDt)} Hz`;

    if (expId === 10 || expId === 14) {
      this.learningStorageKey = expId === 14 ? 'tensegrity-route-learning-v14' : 'tensegrity-route-learning-v1';
      this.routeLearner = new AdaptiveRouteLearner(expId === 14 ? {
        ...LEVEL14_LEARNING_DEFAULTS,
        courseStartY: this.cfg.courseStartY || 10,
        courseGoalY: this.cfg.targetGoalY,
        baseTargetSpeed: this.cfg.targetSpeed
      } : {
        deadlineSeconds: this.cfg.missionDeadlineSeconds,
        courseStartY: this.cfg.courseStartY || 10,
        courseGoalY: this.cfg.targetGoalY,
        baseTargetSpeed: this.cfg.targetSpeed,
        segmentLength: 2.0
      }, this.loadRouteLearning());
    } else {
      this.routeLearner = null;
    }

    this.rover = new SphericalRoverModel(this.cfg);
    this.terrain = new TerrainModel(this.cfg);
    this.simA = new Simulation(this.cfg, this.rover, this.terrain, 'fixed');
    this.simB = new Simulation(this.cfg, this.rover, this.terrain, 'adaptive', this.routeLearner);
    this.accumulatedTime = 0;
    this.lastChartStep = 0;

    this.vis.roverModel = this.rover;
    this.vis.terrainModel = this.terrain;
    this.vis.scene.clear();
    this.vis.initThree();
    this.vis.createTerrainMesh();
    this.vis.createRoverObjects();
    this.vis.createTrajectoryTrail();
    this.vis.createGeometryCheckpointOverlay();
    this.updateForceLawChart();
    syncNumericInputs();
    this.updateViewportHeader();
    this.matrix?.render();
    this.syncUrl();
  }

  showNotice(message, kind = 'warn') {
    showGlobalNotice(message, kind);
  }

  /** Read, validate and apply the Experiment tab, then reset both rovers. */
  applyExperimentPanel() {
    const read = id => document.getElementById(id)?.value;
    const raw = {
      seed: read('input-seed'), noiseModel: read('select-noise-model'), noiseSigma: read('input-noise-sigma'),
      actuatorDelay: read('input-delay'), plantStiffnessScale: read('input-stiffness-scale'),
      plantFrictionScale: read('input-friction-scale'), plantPayloadMassScale: read('input-payload-scale'),
      sgtStroke: read('input-sgt-stroke'), sgtCoreStroke: read('input-sgt-core'),
      sgtSpoolSpeed: read('input-sgt-rate'), sgtActiveCables: read('input-sgt-cables')
    };
    const { values, issues } = validateObject(EXPERIMENT_SCHEMA, raw);
    const field = { seed: 'input-seed', noiseSigma: 'input-noise-sigma', actuatorDelay: 'input-delay',
      plantStiffnessScale: 'input-stiffness-scale', plantFrictionScale: 'input-friction-scale',
      plantPayloadMassScale: 'input-payload-scale', sgtStroke: 'input-sgt-stroke', sgtCoreStroke: 'input-sgt-core',
      sgtSpoolSpeed: 'input-sgt-rate', sgtActiveCables: 'input-sgt-cables' };
    for (const [key, id] of Object.entries(field)) {
      const element = document.getElementById(id);
      if (!element) continue;
      const corrected = issues.some(issue => issue.startsWith(`${key}:`));
      element.classList.toggle('is-invalid', corrected);
      element.value = String(values[key]);
    }
    const status = document.getElementById('experiment-validation');
    if (status) status.textContent = issues.length ? issues.join(' · ') : 'All inputs valid.';

    const cfg = this.cfg;
    if (document.getElementById('chk-seed-lock')?.checked) cfg.seed = values.seed;
    else cfg.seed = Math.floor(Math.random()*1e6);
    // Reset stress hooks, then apply the scenario preset and the manual fields.
    Object.assign(cfg, { sensorNoise: { model: 'none' }, actuatorDelay: 0, disturbance: null,
      plantStiffnessScale: 1, plantFrictionScale: 1, plantPayloadMassScale: 1 });
    const scenario = read('select-scenario') || 'nominal';
    SCENARIOS[scenario]?.apply(cfg);
    if (values.noiseModel !== 'none') {
      cfg.sensorNoise = { model: values.noiseModel, sigmaQ: values.noiseSigma, sigmaV: 4*values.noiseSigma,
        outlierRate: 0.02, outlierScale: 10 };
    }
    if (values.actuatorDelay > 0) cfg.actuatorDelay = values.actuatorDelay;
    if (values.plantStiffnessScale !== 1) cfg.plantStiffnessScale = values.plantStiffnessScale;
    if (values.plantFrictionScale !== 1) cfg.plantFrictionScale = values.plantFrictionScale;
    if (values.plantPayloadMassScale !== 1) cfg.plantPayloadMassScale = values.plantPayloadMassScale;
    if (cfg.controllerMode === 'sgt_tipping') {
      cfg.actuationDeltaL = values.sgtStroke;
      cfg.coreActuationDeltaL = values.sgtCoreStroke;
      cfg.cableLinearVelocity = values.sgtSpoolSpeed;
      cfg.coreCableLinearVelocity = values.sgtSpoolSpeed;
      cfg.sgtOptions = { ...(cfg.sgtOptions || {}), activeCables: values.sgtActiveCables };
    }
    this.loadExperiment(cfg.experimentId, { reason: 'experiment-apply' });
    if (cfg.controllerMode === 'sgt_tipping') {
      cfg.actuationDeltaL = values.sgtStroke;
      cfg.coreActuationDeltaL = values.sgtCoreStroke;
      cfg.cableLinearVelocity = values.sgtSpoolSpeed;
      cfg.coreCableLinearVelocity = values.sgtSpoolSpeed;
    }
    this.simA.reset(false);
    this.simB.reset(false);
    this.syncPhysicsControls();
    this.showNotice(`Applied: ${SCENARIOS[scenario]?.label || scenario}, seed ${cfg.seed}`, 'warn');
  }

  syncPhysicsControls() {
    const core = document.getElementById('select-physics-core');
    if (core) core.value = this.cfg.physicsCore === 'dae' ? 'dae' : 'legacy';
    const pure = document.getElementById('select-actuation-physics');
    if (pure) pure.value = this.cfg.pureCableActuation ? 'pure' : 'assisted';
    const controller = document.getElementById('select-controller');
    if (controller && this.cfg.controllerMode) controller.value = this.cfg.controllerMode;
    const stroke = document.getElementById('slider-actuation');
    if (stroke) stroke.value = String(this.cfg.actuationDeltaL);
    const strokeValue = document.getElementById('val-actuation');
    if (strokeValue) strokeValue.textContent = `${this.cfg.actuationDeltaL.toFixed(3)} m`;
    const gait = document.getElementById('select-gait-mode');
    if (gait) gait.value = this.cfg.actuationMode;
    syncNumericInputs();
    this.updateViewportHeader();
    this.syncUrl();
  }

  bindEvents() {
    const byId = id => document.getElementById(id);
    const on = (id, type, handler) => byId(id)?.addEventListener(type, handler);

    // Geometry checkpoint vs locomotion
    on('btn-mode-toggle', 'click', () => {
      this.isGeometryCheckpointMode = !this.isGeometryCheckpointMode;
      byId('txt-mode-toggle').textContent = this.isGeometryCheckpointMode ? 'Mode: Geometry' : 'Mode: Locomotion';
      byId('geometry-checkpoint-banner')?.classList.toggle('hidden', !this.isGeometryCheckpointMode);
      this.vis.toggleGeometryCheckpoint(this.isGeometryCheckpointMode);
      if (this.isGeometryCheckpointMode) this.resetRovers('mode-change');
    });

    // Gait: the trajectory continues, so the run stays open (checkpointed).
    const gaitSelect = byId('select-gait-mode');
    const setGait = mode => {
      if (this.cfg.actuationMode === mode) return;
      this.cfg.actuationMode = mode;
      if (gaitSelect) gaitSelect.value = mode;
      this.recorder.checkpoint('gait-change');
      this.syncUrl();
    };
    gaitSelect?.addEventListener('change', event => setGait(event.target.value));

    // Controller defines the experiment: commit the run and restart cleanly.
    on('select-controller', 'change', event => {
      this.recorder.commit('controller-change');
      this.cfg.controllerMode = event.target.value;
      this.applyTippingGaitStrokes();
      this.simB.controlDiagnostics.mode = event.target.value;
      this.resetRovers('controller-change');
      this.syncUrl();
    });

    on('select-actuator-mode', 'change', event => { this.cfg.actuatorMode = event.target.value; });

    // Keyboard: W/A/S/D gait, Space play/pause
    window.addEventListener('keydown', event => {
      const tag = document.activeElement?.tagName?.toLowerCase();
      if (['input', 'select', 'textarea'].includes(tag) || event.metaKey || event.ctrlKey || event.altKey) return;
      const key = event.key.toLowerCase();
      const map = { w: 'roll_forward', arrowup: 'roll_forward', s: 'roll_backward', arrowdown: 'roll_backward',
        a: 'steer_left', arrowleft: 'steer_left', d: 'steer_right', arrowright: 'steer_right' };
      if (key === ' ' && tag !== 'button') {
        event.preventDefault();
        this.setPlaying(!this.isPlaying);
        return;
      }
      if (map[key] && !document.activeElement?.closest?.('[role="tablist"]')) {
        event.preventDefault();
        setGait(map[key]);
      }
    });

    // Terrain level
    on('select-experiment', 'change', event => {
      if (this.autoBenchmark.running) { event.target.value = String(this.cfg.experimentId); return; }
      this.loadExperiment(parseInt(event.target.value, 10));
      this.syncPhysicsControls();
    });

    // Physics core / actuation admissibility (both rovers share one config)
    on('select-physics-core', 'change', event => {
      this.recorder.commit('physics-change');
      this.cfg.physicsCore = event.target.value;
      this.resetRovers('physics-change');
      this.syncPhysicsControls();
    });
    on('select-actuation-physics', 'change', event => {
      this.recorder.commit('physics-change');
      this.cfg.pureCableActuation = event.target.value === 'pure';
      this.resetRovers('physics-change');
      this.syncPhysicsControls();
    });
    on('btn-research-preset', 'click', () => {
      this.loadExperiment(1, { reason: 'preset' });
      this.cfg.applyResearchProfile();
      this.resetRovers('preset');
      this.syncPhysicsControls();
      this.showNotice('Research preset: Lagrangian DAE · passive contact · cable-only · SGT gait · Level 1', 'ok');
    });

    on('btn-step', 'click', () => {
      this.setPlaying(false, { record: false });
      const steps = Math.round(0.1/this.cfg.dt);
      for (let k = 0; k < steps; k++) {
        if (this.simA.t < this.cfg.T_end && !this.simA.metrics.runTerminal) this.simA.step();
        if (this.simB.t < this.cfg.T_end && !this.simB.metrics.runTerminal) this.simB.step();
      }
      this.recorder.tick();
      this.lastChartStep = 0;
    });
    on('btn-inject-impulse', 'click', () => {
      this.simA.injectImpulse([0.5, 0, 0]);
      this.simB.injectImpulse([0.5, 0, 0]);
      this.showNotice('Lateral 0.5 m/s impulse injected (booked as disturbance work).', 'warn');
    });
    on('btn-apply-experiment', 'click', () => this.applyExperimentPanel());

    const controlsToggle = byId('btn-controls-toggle');
    const header = document.querySelector('.app-header');
    if (controlsToggle && header) {
      if (window.matchMedia('(max-width: 640px)').matches) {
        header.classList.add('controls-collapsed');
        controlsToggle.setAttribute('aria-expanded', 'false');
      }
      controlsToggle.addEventListener('click', () => {
        const collapsed = header.classList.toggle('controls-collapsed');
        controlsToggle.setAttribute('aria-expanded', String(!collapsed));
      });
    }

    // Inspector tabs (arrow keys move between tabs)
    const tabs = [...document.querySelectorAll('.sidebar-tab')];
    const showTab = name => {
      tabs.forEach(tab => {
        const active = tab.dataset.tab === name;
        tab.classList.toggle('is-active', active);
        tab.setAttribute('aria-selected', String(active));
        tab.tabIndex = active ? 0 : -1;
      });
      document.querySelectorAll('[data-tab-panel]').forEach(panel => {
        panel.hidden = panel.dataset.tabPanel !== name;
      });
      if (name === 'audit') this.updatePhysicsAudit();
    };
    tabs.forEach((tab, index) => {
      tab.addEventListener('click', () => showTab(tab.dataset.tab));
      tab.addEventListener('keydown', event => {
        if (event.key !== 'ArrowRight' && event.key !== 'ArrowLeft') return;
        const next = tabs[(index+(event.key === 'ArrowRight' ? 1 : tabs.length-1)) % tabs.length];
        next.focus();
        showTab(next.dataset.tab);
      });
    });
    showTab('params');

    // Figure category filter
    document.addEventListener('click', event => {
      const button = event.target.closest?.('.fig-filter-btn');
      if (!button) return;
      const group = button.dataset.figFilter;
      document.querySelectorAll('.fig-filter-btn').forEach(other => other.classList.toggle('is-active', other === button));
      document.querySelectorAll('#figure-grid .fig').forEach(figure => {
        figure.hidden = group !== 'all' && figure.dataset.figGroup !== group;
      });
      this.resizeVisibleCharts();
    });

    // Figure export: screen PNG (dark, as displayed) and CSV
    document.addEventListener('click', event => {
      const png = event.target.closest?.('[data-export-png]');
      const csv = event.target.closest?.('[data-export-csv]');
      const key = png?.dataset.exportPng || csv?.dataset.exportCsv;
      const chart = key ? (this.chartRegistry?.[key] || this.benchmarkCharts?.[key]) : null;
      if (!chart) return;
      const stamp = `L${this.cfg.experimentId}_${this.cfg.physicsCore}_${this.cfg.pureCableActuation ? 'cableonly' : 'assisted'}`;
      if (png) exportChartPNG(chart, `fig_${key}_${stamp}.png`, 4);
      else exportChartCSV(chart, `fig_${key}_${stamp}.csv`);
      event.target.closest('details')?.removeAttribute('open');
    });
    // Paper export: white background, IEEE sizes, MATLAB colours
    document.addEventListener('click', async event => {
      const item = event.target.closest?.('[data-ieee]');
      if (!item) return;
      const chart = this.chartRegistry?.[item.dataset.ieee] || this.benchmarkCharts?.[item.dataset.ieee];
      item.closest('details')?.removeAttribute('open');
      if (!chart) return;
      try {
        await exportChartIEEE(chart, item.dataset.format, item.dataset.width, `fig_${item.dataset.ieee}`);
      } catch (error) {
        this.showNotice(`Export failed: ${error.message}`, 'error');
      }
    });
    // Close open export menus when clicking elsewhere
    document.addEventListener('click', event => {
      document.querySelectorAll('details.fig-menu[open]').forEach(menu => {
        if (!menu.contains(event.target)) menu.removeAttribute('open');
      });
    });

    // Transport
    on('btn-play-pause', 'click', () => this.setPlaying(!this.isPlaying));
    on('btn-reset', 'click', () => {
      this.resetRovers('reset');
      this.setPlaying(true, { record: false });
    });
    on('select-speed', 'change', event => {
      const requested = parseFloat(event.target.value);
      this.simSpeed = ALLOWED_SPEEDS.includes(requested) ? requested : 1.0;
      this.accumulatedTime = 0;
      this.lastFrameTime = performance.now();
      this.syncUrl();
    });
    on('select-camera', 'change', event => {
      this.vis.setCameraMode(event.target.value);
      this.syncUrl();
    });
    on('btn-toggle-forces', 'click', event => {
      const onState = event.currentTarget.getAttribute('aria-pressed') !== 'true';
      this.vis.toggleForceOverlay(onState);
      event.currentTarget.setAttribute('aria-pressed', String(onState));
      event.currentTarget.classList.toggle('is-on', onState);
    });
    on('btn-toggle-debug', 'click', event => {
      this.showDebugLabels = !this.showDebugLabels;
      this.vis.toggleDebugLabels(this.showDebugLabels);
      event.currentTarget.setAttribute('aria-pressed', String(this.showDebugLabels));
      event.currentTarget.classList.toggle('is-on', this.showDebugLabels);
    });
    on('btn-copy-link', 'click', async () => {
      const url = this.syncUrl() || window.location.href;
      try {
        await navigator.clipboard.writeText(url);
        this.showNotice('Link to this exact setup copied.', 'ok');
      } catch {
        this.showNotice(url, 'warn');
      }
    });

    // Route learner
    on('btn-toggle-auto-learning', 'click', () => {
      this.autoLearningEnabled = !this.autoLearningEnabled;
      this.learningRestartAt = null;
      this.updateLearningHUD();
    });
    on('btn-clear-learning', 'click', () => {
      this.routeLearner?.resetLearning();
      this.persistRouteLearning();
      this.recorder.commit('reset');
      this.simA.reset(false);
      this.simB.reset(false);
      this.accumulatedTime = 0;
      this.learningRestartAt = null;
      this.updateLearningHUD();
    });

    // Monitoring
    on('select-monitor-cable', 'change', event => {
      this.selectedCableIndex = Number(event.target.value) || 0;
      this.vis.setSelectedCable(this.selectedCableIndex);
      this.updateMonitoringHUD(this.simB.currentDiag.monitoring);
      this.updateMonitoringCharts();
    });
    on('input-goal-threshold', 'input', event => {
      const threshold = Number(event.target.value);
      this.cfg.monitoring.goalThreshold = threshold;
      if (this.simB.monitor) this.simB.monitor.settings.goalThreshold = threshold;
      const value = byId('val-goal-threshold');
      if (value) value.textContent = `${threshold.toFixed(2)} m`;
    });
    document.querySelectorAll('[data-monitor-export]').forEach(button => {
      button.addEventListener('click', () => {
        if (!this.simB.monitor) return;
        downloadText(`${button.dataset.monitorExport}.csv`, this.simB.monitor.exportCsv(button.dataset.monitorExport));
      });
    });
    on('btn-export-monitor-json', 'click', () => {
      if (!this.simB.monitor) return;
      downloadText('complete_simulation_log.json', this.simB.monitor.exportJson(), 'application/json;charset=utf-8');
    });

    // ---------------------------------------------------------- benchmark records
    const day = () => new Date().toISOString().slice(0, 10);
    on('btn-export-json', 'click', () => downloadText(`tensegrity_benchmarks_${day()}.json`, this.persistence.exportJSON(), 'application/json;charset=utf-8'));
    on('btn-export-csv', 'click', () => downloadText(`tensegrity_benchmarks_${day()}.csv`, this.persistence.exportCSV()));
    on('input-import-json', 'change', async event => {
      const file = event.target.files?.[0];
      event.target.value = '';
      if (!file) return;
      try {
        const changed = this.persistence.importJSON(await file.text());
        this.showNotice(`Imported ${file.name}: ${changed} run${changed === 1 ? '' : 's'} added or updated.`, 'ok');
      } catch (error) {
        this.showNotice(`Import failed: ${error.message}`, 'error');
      }
    });
    const clearDialog = byId('dlg-clear');
    on('btn-clear-records', 'click', () => {
      const count = byId('dlg-clear-count');
      if (count) count.textContent = String(this.persistence.state.runs.length);
      if (clearDialog?.showModal) clearDialog.showModal();
      else if (window.confirm('Delete all saved benchmark records?')) this.clearRecords();
    });
    clearDialog?.addEventListener('close', () => {
      if (clearDialog.returnValue === 'confirm') this.clearRecords();
    });

    // Run all 14 levels
    const runAll = byId('btn-run-all');
    runAll?.addEventListener('click', async () => {
      if (this.autoBenchmark.running) return;
      const duration = Number(byId('select-autobench-duration')?.value) || 20;
      const strip = byId('autobench-status');
      const bar = byId('autobench-bar');
      const text = byId('autobench-text');
      strip?.classList.remove('hidden');
      runAll.disabled = true;
      const started = performance.now();
      const report = await this.autoBenchmark.run({
        duration,
        onProgress: ({ level, index, total, fraction, simTime }) => {
          if (bar) bar.style.width = `${(100*fraction).toFixed(1)}%`;
          const elapsed = (performance.now()-started)/1000;
          const eta = fraction > 0.02 ? elapsed*(1-fraction)/fraction : NaN;
          if (text) text.textContent = `Level ${level} (${index+1}/${total}) · t = ${simTime.toFixed(1)} / ${duration} s`+
            `${Number.isFinite(eta) ? ` · about ${Math.ceil(eta/60)} min left` : ''}`;
        }
      });
      runAll.disabled = false;
      if (text) text.textContent = report?.stopped ? 'Stopped' : `Done in ${((performance.now()-started)/60000).toFixed(1)} min`;
      if (report) this.showAutoBenchmarkReport(report);
    });
    on('btn-stop-run-all', 'click', () => this.autoBenchmark.stop());
    on('btn-report-json', 'click', () => {
      const report = this.autoBenchmark.report;
      if (report) downloadText(`auto_benchmark_${day()}.json`, JSON.stringify(report, null, 2), 'application/json;charset=utf-8');
    });
    on('btn-report-csv', 'click', () => {
      const report = this.autoBenchmark.report;
      if (report) downloadText(`auto_benchmark_${day()}.csv`, AutoBenchmark.reportCSV(report));
    });

    // ---------------------------------------------------------- parameters
    on('slider-gravity', 'input', event => this.setGravity(parseFloat(event.target.value), { source: 'slider' }));
    on('select-gravity-preset', 'change', event => {
      const presets = { earth: 9.81, mars: 3.721, moon: 1.625 };
      if (event.target.value === 'custom') {
        document.querySelector('.sidebar-tab[data-tab="params"]')?.click();
        byId('slider-gravity')?.focus();
        return;
      }
      this.setGravity(presets[event.target.value], { source: 'preset' });
    });

    // Pretension / stiffness rebuild the rover: readout live while dragging,
    // rebuild once on release (the open run is committed first).
    on('slider-pretension', 'input', event => {
      byId('val-pretension').textContent = `${parseFloat(event.target.value).toFixed(1)} N`;
    });
    on('slider-pretension', 'change', event => {
      this.cfg.pretensionS = parseFloat(event.target.value);
      this.loadExperiment(this.cfg.experimentId, { reason: 'parameter-change' });
    });
    on('slider-stiffness', 'input', event => {
      byId('val-stiffness').textContent = `${parseFloat(event.target.value).toFixed(0)} N/m`;
    });
    on('slider-stiffness', 'change', event => {
      this.cfg.kS = parseFloat(event.target.value);
      this.loadExperiment(this.cfg.experimentId, { reason: 'parameter-change' });
    });
    on('select-material', 'change', event => this.setMaterial(event.target.value));
    this.updateMaterialReadout();
    on('btn-run-tsgfem', 'click', () => {
      try { this.runStructuralAnalysis(); } catch (error) {
        console.error('[tsgfem]', error);
        byId('tsgfem-note').textContent = `Analysis failed: ${error.message}`;
      }
    });
    on('btn-export-tsgfem', 'click', () => {
      if (!this.tsgfemResult) return;
      downloadText(`rover_tsgfem_${this.cfg.material || 'model'}.json`, JSON.stringify(this.tsgfemResult.input, null, 1), 'application/json;charset=utf-8');
    });
    on('slider-restitution', 'input', event => {
      event.target.value = '0.02';
      this.cfg.restitution = 0.02;
      byId('val-restitution').textContent = '0.02 (locked)';
    });
    on('slider-actuation', 'input', event => {
      const value = parseFloat(event.target.value);
      this.cfg.actuationDeltaL = value;
      byId('val-actuation').textContent = `${value.toFixed(3)} m`;
    });
    on('slider-roughness', 'input', event => {
      byId('val-roughness').textContent = `${parseFloat(event.target.value).toFixed(2)} m`;
    });
    on('slider-roughness', 'change', event => {
      // Rebuild physics and rendering from the same seed so the mesh and the
      // contact solver always share one surface.
      const value = parseFloat(event.target.value);
      this.loadExperiment(this.cfg.experimentId, { reason: 'parameter-change' });
      this.cfg.groundRMS = value;
      this.terrain.generateSurface();
      this.rebuildScene();
      byId('slider-roughness').value = String(value);
      byId('val-roughness').textContent = `${value.toFixed(2)} m`;
      syncNumericInputs();
    });
    on('slider-target-speed', 'input', event => {
      event.target.value = '1.30';
      this.cfg.targetSpeed = 1.30;
      byId('val-target-speed').textContent = '1.30 m/s';
    });
    on('slider-incline', 'input', event => {
      this.cfg.inclineDegrees = parseFloat(event.target.value);
      byId('val-incline').textContent = `${this.cfg.inclineDegrees.toFixed(0)}°`;
    });
    on('slider-incline', 'change', () => {
      this.terrain.generateSurface();
      this.rebuildScene();
    });
    on('btn-toggle-shield', 'click', event => {
      this.cfg.tensionShield = !this.cfg.tensionShield;
      event.currentTarget.textContent = `TENSION SHIELD: ${this.cfg.tensionShield ? 'ON' : 'OFF'}`;
      event.currentTarget.setAttribute('aria-pressed', String(this.cfg.tensionShield));
      event.currentTarget.classList.toggle('is-on', this.cfg.tensionShield);
      this.recorder.commit('parameter-change');
      this.resetRovers('parameter-change');
    });
    on('btn-toggle-relaxation', 'click', event => {
      this.cfg.stringRelaxation = !this.cfg.stringRelaxation;
      event.currentTarget.textContent = `RELAXATION: ${this.cfg.stringRelaxation ? 'ON' : 'OFF'}`;
      event.currentTarget.setAttribute('aria-pressed', String(this.cfg.stringRelaxation));
      event.currentTarget.classList.toggle('is-on', this.cfg.stringRelaxation);
      this.updateForceLawChart();
    });

    // Structural optimizer
    on('btn-run-optimization', 'click', () => {
      byId('opt-results-box').classList.remove('hidden');
      const opt = StructuralOptimizer.runOptimization(this.cfg);
      byId('opt-pretension').textContent = `${opt.bestConfig.pretension.toFixed(0)} N`;
      byId('opt-stiffness').textContent = `${opt.bestConfig.stiffness.toFixed(0)} N/m`;
      byId('opt-score').textContent = `${Math.min(99.9, Math.max(70.0, opt.bestScore)).toFixed(1)} / 100`;
      this.cfg.pretensionS = opt.bestConfig.pretension;
      this.cfg.kS = opt.bestConfig.stiffness;
      this.cfg.actuationDeltaL = opt.bestConfig.actuation;
      byId('slider-pretension').value = opt.bestConfig.pretension;
      byId('val-pretension').textContent = `${opt.bestConfig.pretension.toFixed(1)} N`;
      byId('slider-stiffness').value = opt.bestConfig.stiffness;
      byId('val-stiffness').textContent = `${opt.bestConfig.stiffness.toFixed(0)} N/m`;
      byId('slider-actuation').value = opt.bestConfig.actuation;
      byId('val-actuation').textContent = `${opt.bestConfig.actuation.toFixed(3)} m`;
      syncNumericInputs();
    });
  }

  rebuildScene() {
    this.vis.scene.clear();
    this.vis.initThree();
    this.vis.createTerrainMesh();
    this.vis.createRoverObjects();
    this.vis.createTrajectoryTrail();
    this.vis.createGeometryCheckpointOverlay();
  }

  async clearRecords() {
    this.recorder.commit('reset');
    await this.persistence.clearAll();
    this.recorder.lastFinalized = null;
    this.matrix.render();
    this.showNotice('All saved benchmark records deleted from browser and disk.', 'ok');
  }

  showAutoBenchmarkReport(report) {
    const table = document.getElementById('report-table');
    if (table) {
      const rows = AutoBenchmark.reportRows(report);
      const f = (v, d) => (Number.isFinite(v) ? v.toFixed(d) : '—');
      table.innerHTML = `<thead><tr><th>Level</th><th>Status</th><th>Ended by</th><th class="r">t (s)</th>
        <th class="r">B dist (m)</th><th class="r">B v̄ (m/s)</th><th class="r">B max G</th><th class="r">B T max (N)</th><th class="r">B deform (m)</th>
        <th class="r">A dist (m)</th><th class="r">A v̄ (m/s)</th></tr></thead><tbody>${rows.map(r => `<tr>
        <td>L${r.level} · ${r.name}</td><td>${r.status}</td><td>${r.reason}</td><td class="r num">${f(r.simTime, 1)}</td>
        <td class="r num">${f(r.B_distance, 2)}</td><td class="r num">${f(r.B_avgSpeed, 3)}</td><td class="r num">${f(r.B_maxG, 2)}</td>
        <td class="r num">${f(r.B_maxTension, 1)}</td><td class="r num">${f(r.B_deformation, 3)}</td>
        <td class="r num">${f(r.A_distance, 2)}</td><td class="r num">${f(r.A_avgSpeed, 3)}</td></tr>`).join('')}</tbody>`;
    }
    const dialog = document.getElementById('dlg-report');
    if (dialog?.showModal) dialog.showModal();
  }
}

async function loadCentralSettings() {
  let payload = {};
  try {
    const response = await fetch(`./config/simulation.settings.json?t=${Date.now()}`, { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    payload = await response.json();
  } catch {
    // Missing or unreadable file: run entirely on built-in defaults.
    return { settings: {}, issues: [] };
  }
  const { settings, issues } = validateSettingsFile(payload);
  return { settings, issues };
}

function showGlobalNotice(message, kind = 'error') {
  let notice = document.getElementById('app-notice');
  if (!notice) {
    notice = document.createElement('div');
    notice.id = 'app-notice';
    notice.setAttribute('role', kind === 'error' ? 'alert' : 'status');
    document.body.appendChild(notice);
  }
  notice.className = `app-notice is-${kind}`;
  notice.textContent = message;
  clearTimeout(showGlobalNotice.timer);
  showGlobalNotice.timer = setTimeout(() => notice.remove(), kind === 'error' ? 12000 : 6000);
}

// Error boundary: any uncaught error pauses the simulation and is shown to
// the user instead of silently freezing the animation loop.
// Browser-level notices that are not program errors (e.g. Chart.js layout
// re-flows trigger "ResizeObserver loop ..."), and errors from other origins
// (extensions, CDN scripts) without a location, must not stop the simulation.
const BENIGN_ERRORS = [/ResizeObserver loop/i, /^Script error\.?$/i];
window.addEventListener('error', event => {
  const message = String(event.message || '');
  if (BENIGN_ERRORS.some(pattern => pattern.test(message))) {
    event.stopImmediatePropagation?.();
    return;
  }
  if (window.app) window.app.isPlaying = false;
  showGlobalNotice(`Simulation paused after an error: ${event.message}`, 'error');
});
window.addEventListener('unhandledrejection', event => {
  if (window.app) window.app.isPlaying = false;
  showGlobalNotice(`Simulation paused after an error: ${event.reason?.message || event.reason}`, 'error');
});

window.addEventListener('DOMContentLoaded', async () => {
  const { settings, issues } = await loadCentralSettings();
  try {
    // Read the shareable URL before the app writes its own state into it.
    const urlState = readUrlState();
    window.app = new App(settings);
    window.app.applyUrlState(urlState);
  } catch (error) {
    showGlobalNotice(`Startup failed: ${error.message}`, 'error');
    throw error;
  }
  if (issues.length) showGlobalNotice(`Settings file: ${issues.join(' · ')}`, 'warn');
});
