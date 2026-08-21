/**
 * APP.JS - Main Coordinator for 6-Bar Tensegrity Icosahedron Rover Simulator
 */

import { SimConfig, TerrainModel, SphericalRoverModel, Simulation, StructuralOptimizer, BenchmarkEngine } from './simEngine.js?v=20260821-modela1';
import { relaxedCableTension } from './driveControllers.js?v=20260821-forward1';
import { Visualizer } from './visualizer.js?v=20260821-modela1';
import { AdaptiveRouteLearner } from './adaptiveLearning.js?v=20260821-learning1';

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

const EXPERIMENT_NAMES = [
  "Level 1: Rough Mars Sand Flats → Forward Rolling",
  "Level 2: Mars Small-Rock Field",
  "Level 3: Mars Boulder Field Rolling",
  "Level 4: Rocky Ridge Climb & Roll Over",
  "Level 5: Eroded Mars Crater Escape",
  "Level 6: Sandy Mars Slope Uphill Roll (18°)",
  "Level 7: Irregular Mars Mountain Landscape",
  "Level 8: Rough-Sand Rolling Endurance",
  "Level 9: Rocky Non-Bounce Settling Recovery",
  "Level 10: Learned Mars Multi-Obstacle Mission"
];

const MARS_ROUGHNESS_BY_LEVEL = [0, 0.035, 0.05, 0.07, 0.09, 0.075, 0.06, 0.10, 0.10, 0.10, 0.06];

class App {
  constructor() {
    this.simSpeed = 1.0;
    this.isPlaying = true;
    this.isGeometryCheckpointMode = false; // Start in active Locomotion Simulation Mode!
    this.showDebugLabels = false;
    this.selectedCableIndex = 0;
    this.autoLearningEnabled = true;
    this.learningRestartAt = null;
    this.learningStorageKey = 'tensegrity-route-learning-v1';
    this.lastPersistedLearningRevision = -1;

    this.initSimulation();
    this.initVisualizer();
    this.initCharts();
    this.initMonitoringControls();
    this.bindEvents();

    // Start rendering immediately. The controlled A-vs-B table is populated
    // from the live solver; no synthetic or blocking startup benchmark.
    this.benchmarkData = [];
    this.updateMetricsTable();
    requestAnimationFrame(t => this.animationLoop(t));
  }

  initSimulation() {
    this.cfg = new SimConfig({
      experimentId: 10,
      terrainLevel: 10,
      actuationMode: 'roll_forward',
      abCourseEnabled: true,
      targetDestination: [0, 60],
      targetGoalY: 60
    }).applyLevel10PerformanceProfile();
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
    // 1. Core Acceleration G-Force Chart
    const ctxG = document.getElementById('chart-gforce').getContext('2d');
    this.chartG = new Chart(ctxG, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Payload Core G-Force [G]',
            data: [],
            borderColor: '#10b981',
            backgroundColor: 'rgba(16, 185, 129, 0.15)',
            fill: true,
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Time [s]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } },
          y: { title: { display: true, text: 'Payload Accel [G]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' }, min: 0 }
        },
        plugins: { legend: { labels: { color: '#f8fafc' } } }
      }
    });

    // 2. Velocity & Deformation Profile Chart
    const ctxV = document.getElementById('chart-velocity').getContext('2d');
    this.chartV = new Chart(ctxV, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Rolling Velocity — 1 s mean [m/s]',
            data: [],
            borderColor: '#06b6d4',
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.35
          },
          {
            label: 'Procrustes RMS — 1 s mean [m]',
            data: [],
            borderColor: '#f59e0b',
            borderWidth: 2,
            borderDash: [4, 4],
            pointRadius: 0,
            tension: 0.35
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Time [s]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } },
          y: { title: { display: true, text: 'Speed / RMS [fixed scale]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' }, min: 0, max: 1.40 }
        },
        plugins: { legend: { labels: { color: '#f8fafc' } } }
      }
    });

    const ctxControl = document.getElementById('chart-control').getContext('2d');
    this.chartControl = new Chart(ctxControl, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Control Cost', data: [], borderColor: '#8b5cf6', borderWidth: 2, pointRadius: 0, tension: 0.2, yAxisID: 'y' },
          { label: 'Rod Residual [mm]', data: [], borderColor: '#10b981', borderWidth: 2, pointRadius: 0, tension: 0.2, yAxisID: 'y' },
          { label: 'Relaxed Cables [%]', data: [], borderColor: '#e879f9', borderWidth: 2, pointRadius: 0, borderDash: [4, 4], tension: 0.2, yAxisID: 'yPercent' }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Time [s]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } },
          y: { position: 'left', beginAtZero: true, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } },
          yPercent: { position: 'right', min: 0, max: 100, grid: { drawOnChartArea: false }, ticks: { color: '#e879f9' } }
        },
        plugins: { legend: { labels: { color: '#f8fafc' } } }
      }
    });

    const ctxForce = document.getElementById('chart-force-law').getContext('2d');
    this.chartForce = new Chart(ctxForce, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Standard Linear Tension [N]', data: [], borderColor: '#06b6d4', borderWidth: 2, pointRadius: 0 },
          { label: 'Adaptive Relaxing Tension [N]', data: [], borderColor: '#e879f9', borderWidth: 2, pointRadius: 0 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Cable length / base rest length', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } },
          y: { title: { display: true, text: 'Tension [N]', color: '#94a3b8' }, beginAtZero: true, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } }
        },
        plugins: { legend: { labels: { color: '#f8fafc' } } }
      }
    });
    this.updateForceLawChart();
    this.initMonitoringCharts();
  }

  initMonitoringCharts() {
    const timeOptions = yTitle => ({
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      normalized: true,
      scales: {
        x: { title: { display: true, text: 'Simulation time [s]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8', maxTicksLimit: 8 } },
        y: { title: { display: true, text: yTitle, color: '#94a3b8' }, beginAtZero: true, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } }
      },
      plugins: { legend: { labels: { color: '#e2e8f0' } } }
    });
    const createLineChart = (id, datasets, yTitle) => new Chart(document.getElementById(id).getContext('2d'), {
      type: 'line',
      data: { labels: [], datasets },
      options: timeOptions(yTitle)
    });
    const line = (label, color, extras = {}) => ({
      label, data: [], borderColor: color, backgroundColor: `${color}22`,
      borderWidth: 2, pointRadius: 0, tension: 0.2, ...extras
    });
    this.chartGoalError = createLineChart('chart-goal-error', [
      line('Goal error [m]', '#22d3ee', { fill: true }),
      line('Success threshold', '#22c55e', { borderDash: [6, 4], borderWidth: 1 })
    ], 'Distance to Goal B [m]');
    this.chartFormationError = createLineChart('chart-formation-error', [
      line('Formation RMS [m]', '#a78bfa', { fill: true }),
      line('Warning threshold', '#f97316', { borderDash: [6, 4], borderWidth: 1 })
    ], 'Rigid-aligned RMS error [m]');
    this.chartCableDelta = createLineChart('chart-cable-delta', [
      line('Selected cable ΔL [m]', '#facc15'),
      line('Zero extension', '#64748b', { borderDash: [4, 4], borderWidth: 1 })
    ], 'Cable length change [m]');
    this.chartCableForce = createLineChart('chart-cable-force', [
      line('Selected cable [N]', '#fb7185'),
      line('All-cable average [N]', '#38bdf8'),
      line('Overload limit [N]', '#ef4444', { borderDash: [6, 4], borderWidth: 1 })
    ], 'Tension force [N]');
    this.chartContactCount = createLineChart('chart-contact-count', [
      line('Active contacts', '#f97316', { stepped: true, fill: true })
    ], 'Active node / rod contacts');
    this.chartComPath = new Chart(document.getElementById('chart-com-path').getContext('2d'), {
      type: 'scatter',
      data: { datasets: [
        { label: 'COM trajectory', data: [], showLine: true, borderColor: '#22d3ee', backgroundColor: '#22d3ee', borderWidth: 2, pointRadius: 0 },
        { label: 'Start A', data: [], backgroundColor: '#38bdf8', pointRadius: 6 },
        { label: 'Goal B', data: [], backgroundColor: '#22c55e', pointRadius: 7, pointStyle: 'rectRot' },
        { label: 'Obstacles', data: [], backgroundColor: '#f59e0b88', borderColor: '#f59e0b', pointRadius: 5 }
      ] },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        scales: {
          x: { title: { display: true, text: 'World X [m]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } },
          y: { title: { display: true, text: 'World Y [m]', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' } }
        },
        plugins: { legend: { labels: { color: '#e2e8f0' } } }
      }
    });
  }

  initMonitoringControls() {
    const cableSelect = document.getElementById('select-monitor-cable');
    if (cableSelect) {
      cableSelect.innerHTML = this.rover.outerStrings.map((_, index) =>
        `<option value="${index}">C${String(index+1).padStart(2, '0')}</option>`).join('');
      cableSelect.value = String(this.selectedCableIndex);
    }
  }

  updateForceLawChart() {
    if (!this.chartForce || !this.rover?.l0_outerStrings?.length) return;
    const base = this.rover.l0_outerStrings[0];
    const labels = [];
    const standard = [];
    const adaptive = [];
    for (let ratio = 0.90; ratio <= 1.60 + 1e-9; ratio += 0.02) {
      const ell = base * ratio;
      labels.push(ratio.toFixed(2));
      standard.push(this.cfg.kS * Math.max(0, ell-base));
      adaptive.push(relaxedCableTension(ell, base, base, Object.assign({}, this.cfg, { stringRelaxation: true })).tension);
    }
    this.chartForce.data.labels = labels;
    this.chartForce.data.datasets[0].data = standard;
    this.chartForce.data.datasets[1].data = adaptive;
    this.chartForce.update('none');
  }

  runFullBenchmark() {
    this.benchmarkData = BenchmarkEngine.runAllExperiments(this.cfg);
    this.updateMetricsTable();
  }

  handleAdaptiveLearningCycle(now) {
    this.persistRouteLearning();
    if (!this.autoLearningEnabled || this.cfg.experimentId !== 10 || !this.simB.metrics.runTerminal) {
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
      while (this.accumulatedTime >= this.cfg.dt && safetyGuard < 400) {
        if (this.simA.t < this.cfg.T_end && !this.simA.metrics.runTerminal) this.simA.step();
        if (this.simB.t < this.cfg.T_end && !this.simB.metrics.runTerminal) this.simB.step();
        this.accumulatedTime -= this.cfg.dt;
        safetyGuard++;
      }
    }

    this.handleAdaptiveLearningCycle(now);

    // Render 3D Scene Dual
    this.vis.updateDual({
      simA: { q: this.simA.q, diag: this.simA.currentDiag },
      simB: { q: this.simB.q, diag: this.simB.currentDiag }
    });

    this.updateHUD();
    this.updateCharts();
  }

  updateHUD() {
    const timeElem = document.getElementById('sim-time');
    if (timeElem) {
      const attempt = this.routeLearner?.runCount+1 || 1;
      const limit = this.terrain.course ? (this.cfg.missionDeadlineSeconds || this.cfg.T_end) : this.cfg.T_end;
      timeElem.textContent = `${this.simB.t.toFixed(2)} s / ${limit} s${this.terrain.course ? ` · try ${attempt}` : ''}`;
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
    const summaryA = mA.obstacleSummary || { over: 0, around: 0, retries: 0 };
    const summaryB = mB.obstacleSummary || { over: 0, around: 0, retries: 0, bypassViolations: 0 };
    setText('hud-a-obstacles', `${summaryA.over} over / ${summaryA.around} around`);
    setText('hud-b-obstacles', `${summaryB.over} over / ${summaryB.around} around`);
    setText('hud-b-bypass', `${summaryB.bypassViolations || 0}`);
    setText('course-current-obstacle', diagB.activeObstacleId || 'none');
    setText('course-checkpoints', `${summaryB.checkpointsReached || 0} / ${summaryB.total || 10}`);
    setText('course-phase', diagB.obstaclePhase || 'cruise');
    const courseY = diagB.centroid?.[1] || 0;
    setText('course-progress', courseY < 10
      ? `approach y=${courseY.toFixed(2)} / 10.00 m`
      : `${Math.max(0, Math.min(50, courseY-10)).toFixed(1)} / 50.0 m`);
    this.updateMonitoringHUD(diagB.monitoring);
    this.updateLearningHUD();
    this.updateABComparisonTable();
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
        goalResult.textContent = `WIN · goal reached in ${metrics.completionTime.toFixed(2)} s · learner keeps the faster route`;
        goalResult.className = 'mt-2 rounded bg-emerald-950/60 p-2 text-emerald-300 font-bold';
      } else if (metrics.runOutcome === 'loss') {
        const remaining = Math.max(0, this.cfg.courseGoalY-(this.simB.currentDiag.centroid?.[1] || 0));
        goalResult.textContent = `LOSS · 120 s exceeded · ${remaining.toFixed(2)} m remaining · gradient update saved for next try`;
        goalResult.className = 'mt-2 rounded bg-rose-950/60 p-2 text-rose-300 font-bold';
      } else {
        goalResult.textContent = `Attempt ${this.routeLearner?.runCount+1 || 1} running · reach Goal B before 120 s`;
        goalResult.className = 'mt-2 rounded bg-slate-900 p-2 text-slate-400';
      }
    }
    const warningElement = document.getElementById('monitor-warnings');
    if (warningElement) {
      warningElement.textContent = monitoring.warnings.length
        ? `WARNING · ${monitoring.warnings.join(' · ')}` : 'No structural warnings';
      warningElement.className = monitoring.warnings.length
        ? 'mt-2 rounded bg-rose-950/60 p-2 text-rose-300 font-bold'
        : 'mt-2 rounded bg-emerald-950/40 p-2 text-emerald-300';
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
      if (stateElement) stateElement.className = cable.state === 'overload' ? 'text-rose-400 font-bold'
        : cable.state === 'slack' ? 'text-fuchsia-400' : cable.state === 'moderate' || cable.state === 'high'
          ? 'text-amber-300' : 'text-emerald-300';
    }
    const contactList = document.getElementById('monitor-contact-list');
    if (contactList) {
      contactList.innerHTML = monitoring.contacts.length ? monitoring.contacts.map(contact => {
        const friction = Math.hypot(...(contact.frictionForce || [0, 0, 0]));
        return `<div class="flex justify-between gap-2"><span class="text-orange-300">${contact.id} · ${contact.objectId}</span><span>${contact.normalForce.toFixed(1)} N normal · ${friction.toFixed(1)} N friction</span></div>`;
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
    setText('learning-attempt', String(learning.runCount+1));
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
    const renderModel = (label, metrics, color) => {
      const obstacles = metrics.obstacleSummary || { over: 0, around: 0, retries: 0, bypassViolations: 0 };
      const completion = metrics.runOutcome === 'loss' ? 'LOSS · 120 s'
        : metrics.completionTime === null ? 'running' : `WIN · ${metrics.completionTime.toFixed(1)} s`;
      return `<tr class="border-b border-slate-800">
        <td class="py-2.5 px-3 font-bold ${color}">${label}</td>
        <td class="py-2.5 px-3">${metrics.measuredDistance.toFixed(2)} m</td>
        <td class="py-2.5 px-3">${completion}</td>
        <td class="py-2.5 px-3">${metrics.avgVelocity.toFixed(3)} m/s</td>
        <td class="py-2.5 px-3">${metrics.speedVariance.toFixed(5)}</td>
        <td class="py-2.5 px-3">${metrics.lateralTravel.toFixed(2)} m</td>
        <td class="py-2.5 px-3 text-emerald-400">${obstacles.over}</td>
        <td class="py-2.5 px-3 text-amber-400">${obstacles.around}</td>
        <td class="py-2.5 px-3">${obstacles.retries}</td>
        <td class="py-2.5 px-3 ${obstacles.bypassViolations ? 'text-rose-400 font-bold' : 'text-emerald-400'}">${obstacles.bypassViolations || 0}</td>
        <td class="py-2.5 px-3">${metrics.energyCost.toFixed(2)} J</td>
        <td class="py-2.5 px-3">${metrics.payloadAccelMax.toFixed(2)} G</td>
      </tr>`;
    };
    body.innerHTML = renderModel('Model A · baseline', this.simA.metrics, 'text-slate-300')+
      renderModel('Model B · adaptive OVER', this.simB.metrics, 'text-cyan-300');
  }

  updateCharts() {
    if (this.simB.stepCount - (this.lastChartStep || 0) >= 250) {
      this.lastChartStep = this.simB.stepCount;
      const hist = this.simB.history;
      // Show a calm 20-second window instead of redrawing the full, noisy
      // physics history. History samples arrive every 20 ms.
      const start = Math.max(0, hist.t.length-1000);
      const labels = hist.t.slice(start).map(t => t.toFixed(1));
      this.chartG.data.labels = labels;
      this.chartG.data.datasets[0].data = movingAverage(hist.centroidAccel, 50).slice(start);
      this.chartG.update('none');

      this.chartV.data.labels = labels;
      this.chartV.data.datasets[0].data = movingAverage(hist.planarSpeed, 50).slice(start);
      this.chartV.data.datasets[1].data = movingAverage(hist.deformation, 50).slice(start);
      this.chartV.update('none');

      this.chartControl.data.labels = labels;
      this.chartControl.data.datasets[0].data = movingAverage(hist.controlCost, 50).slice(start);
      this.chartControl.data.datasets[1].data = movingAverage(hist.constraintError, 50).slice(start).map(error => error*1000);
      this.chartControl.data.datasets[2].data = movingAverage(hist.relaxationFraction, 50).slice(start).map(fraction => fraction*100);
      this.chartControl.update('none');

      this.updateMonitoringCharts();

      this.updateMetricsTable();
    }
  }

  updateMonitoringCharts() {
    const monitor = this.simB.monitor;
    if (!monitor?.history?.t.length) return;
    const history = monitor.history;
    const windowSamples = Math.ceil(monitor.settings.chartWindowSeconds/monitor.settings.chartSamplePeriod);
    const start = Math.max(0, history.t.length-windowSamples);
    const labels = history.t.slice(start).map(time => time.toFixed(2));
    const thresholdLine = value => new Array(labels.length).fill(value);
    const cableDelta = history.cableDelta.slice(start).map(values => values[this.selectedCableIndex] || 0);
    const cableForce = history.cableForce.slice(start).map(values => values[this.selectedCableIndex] || 0);
    const cableAverage = history.cableForce.slice(start).map(values =>
      values.reduce((sum, value) => sum+value, 0)/Math.max(1, values.length));

    this.chartGoalError.data.labels = labels;
    this.chartGoalError.data.datasets[0].data = movingAverage(history.goalError.slice(start), 3);
    this.chartGoalError.data.datasets[1].data = thresholdLine(monitor.settings.goalThreshold);
    this.chartGoalError.update('none');

    this.chartFormationError.data.labels = labels;
    this.chartFormationError.data.datasets[0].data = movingAverage(history.formationError.slice(start), 3);
    this.chartFormationError.data.datasets[1].data = thresholdLine(monitor.settings.formationWarningThreshold);
    this.chartFormationError.update('none');

    this.chartCableDelta.data.labels = labels;
    this.chartCableDelta.data.datasets[0].data = movingAverage(cableDelta, 3);
    this.chartCableDelta.data.datasets[1].data = thresholdLine(0);
    this.chartCableDelta.update('none');

    this.chartCableForce.data.labels = labels;
    this.chartCableForce.data.datasets[0].data = movingAverage(cableForce, 3);
    this.chartCableForce.data.datasets[1].data = movingAverage(cableAverage, 3);
    this.chartCableForce.data.datasets[2].data = thresholdLine(monitor.settings.cableOverloadForce);
    this.chartCableForce.update('none');

    this.chartContactCount.data.labels = labels;
    this.chartContactCount.data.datasets[0].data = history.contactCount.slice(start);
    this.chartContactCount.update('none');

    const pathStart = Math.max(0, history.t.length-2500);
    this.chartComPath.data.datasets[0].data = history.comX.slice(pathStart).map((x, index) => ({
      x, y: history.comY[pathStart+index]
    }));
    this.chartComPath.data.datasets[1].data = [{ x: monitor.start[0], y: monitor.start[1] }];
    this.chartComPath.data.datasets[2].data = [{ x: monitor.goal[0], y: monitor.goal[1] }];
    this.chartComPath.data.datasets[3].data = (this.terrain.course?.obstacles || []).map(obstacle => ({ x: obstacle.x, y: obstacle.y }));
    this.chartComPath.update('none');

    const cableId = `C${String(this.selectedCableIndex+1).padStart(2, '0')}`;
    const deltaLabel = document.getElementById('chart-cable-delta-label');
    if (deltaLabel) deltaLabel.textContent = `${cableId} · ΔL [m]`;
    const forceLabel = document.getElementById('chart-cable-force-label');
    if (forceLabel) forceLabel.textContent = `${cableId} · tension [N]`;
  }

  updateMetricsTable() {
    const tableBody = document.getElementById('experiments-table-body');
    if (!tableBody) return;

    let html = '';
    const bData = this.benchmarkData || [];

    for (let i = 1; i <= 10; i++) {
      const isCurrent = (i === this.cfg.experimentId);
      const bRes = bData[i - 1] || {};

      const distance = isCurrent ? this.simB.metrics.distanceTraveled : bRes.distance;
      const time = isCurrent ? this.simB.metrics.timeElapsed : bRes.time;
      const avgSpeed = isCurrent ? this.simB.metrics.avgVelocity : bRes.avgVelocity;
      const maxG = isCurrent ? this.simB.metrics.payloadAccelMax : bRes.maxG;
      const maxTension = isCurrent ? this.simB.metrics.maxCableTension : bRes.maxTension;
      const deformation = isCurrent ? this.simB.metrics.shapeDeformationMax : bRes.deformation;
      const obstacles = isCurrent && this.terrain.course
        ? this.terrain.course.obstacles.length
        : bRes.obstacles;
      const number = (value, digits, unit = '') => Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : '—';

      const rowClass = isCurrent ? "bg-cyan-950/40 border-b border-cyan-800/80 font-bold" : "border-b border-slate-800 hover:bg-slate-800/40 transition";
      const badge = isCurrent ? '<span class="px-2 py-0.5 rounded bg-emerald-900/60 text-emerald-300 border border-emerald-700 text-[10px]">ACTIVE LIVE</span>' : `<span class="text-slate-500 text-[10px]">${bRes.status || 'NOT RUN'}</span>`;

      html += `<tr class="${rowClass}">
        <td class="py-2.5 px-3 font-semibold text-cyan-300">${EXPERIMENT_NAMES[i - 1]}</td>
        <td class="py-2.5 px-3 text-slate-200">${number(distance, 2, ' m')}</td>
        <td class="py-2.5 px-3 text-slate-300">${number(time, 1, ' s')}</td>
        <td class="py-2.5 px-3 text-emerald-400 font-bold">${number(avgSpeed, 2, ' m/s')}</td>
        <td class="py-2.5 px-3 text-amber-400">${number(maxG, 1, ' G')}</td>
        <td class="py-2.5 px-3 text-rose-400">${number(maxTension, 1, ' N')}</td>
        <td class="py-2.5 px-3 text-slate-300">${number(deformation, 3, ' m')}</td>
        <td class="py-2.5 px-3 text-cyan-400">${Number.isFinite(obstacles) ? obstacles : '—'}</td>
        <td class="py-2.5 px-3">${badge}</td>
      </tr>`;
    }

    tableBody.innerHTML = html;
  }

  loadExperiment(expId) {
    this.cfg.experimentId = expId;
    this.cfg.terrainLevel = Math.min(7, expId);
    this.cfg.groundRMS = MARS_ROUGHNESS_BY_LEVEL[expId] || 0.04;
    this.cfg.abCourseEnabled = expId === 10;
    this.cfg.targetGoalY = expId === 10 ? 60 : 25;
    this.cfg.targetDestination = [0, this.cfg.targetGoalY];
    if (expId === 10) this.cfg.applyLevel10PerformanceProfile();
    else this.cfg.applyStandardPerformanceProfile();

    if (expId === 2) this.cfg.actuationMode = 'roll_backward';
    else this.cfg.actuationMode = 'roll_forward';
    const gaitSelect = document.getElementById('select-gait-mode');
    if (gaitSelect) gaitSelect.value = this.cfg.actuationMode;

    document.getElementById('txt-active-experiment-title').textContent = EXPERIMENT_NAMES[expId - 1];
    const experimentSelect = document.getElementById('select-experiment');
    if (experimentSelect) experimentSelect.value = String(expId);
    const speedSlider = document.getElementById('slider-target-speed');
    if (speedSlider) speedSlider.value = String(this.cfg.targetSpeed);
    const speedValue = document.getElementById('val-target-speed');
    if (speedValue) speedValue.textContent = `${this.cfg.targetSpeed.toFixed(2)} m/s rolling target`;
    const roughnessSlider = document.getElementById('slider-roughness');
    if (roughnessSlider) roughnessSlider.value = String(this.cfg.groundRMS);
    const roughnessValue = document.getElementById('val-roughness');
    if (roughnessValue) roughnessValue.textContent = `${this.cfg.groundRMS.toFixed(3)} m`;
    const controlRate = document.getElementById('control-rate-value');
    if (controlRate) controlRate.textContent = `${Math.round(1/this.cfg.controllerDt)} Hz`;

    this.rover = new SphericalRoverModel(this.cfg);
    this.terrain = new TerrainModel(this.cfg);
    this.simA = new Simulation(this.cfg, this.rover, this.terrain, 'fixed');
    this.simB = new Simulation(
      this.cfg,
      this.rover,
      this.terrain,
      'adaptive',
      expId === 10 ? this.routeLearner : null
    );
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
  }

  bindEvents() {
    // Mode Switcher: Geometry Checkpoint vs Locomotion Simulation
    document.getElementById('btn-mode-toggle').addEventListener('click', () => {
      this.isGeometryCheckpointMode = !this.isGeometryCheckpointMode;
      const btnTxt = document.getElementById('txt-mode-toggle');
      const banner = document.getElementById('geometry-checkpoint-banner');

      if (this.isGeometryCheckpointMode) {
        btnTxt.textContent = "Mode: Geometry Checkpoint";
        banner.classList.remove('hidden');
        this.vis.toggleGeometryCheckpoint(true);
        this.simA.reset();
        this.simB.reset();
      } else {
        btnTxt.textContent = "Mode: Locomotion Simulation";
        banner.classList.add('hidden');
        this.vis.toggleGeometryCheckpoint(false);
      }
    });

    // Gait Mode Selector
    const gaitSelect = document.getElementById('select-gait-mode');
    if (gaitSelect) {
      gaitSelect.addEventListener('change', (e) => {
        this.cfg.actuationMode = e.target.value;
      });
    }

    const controllerSelect = document.getElementById('select-controller');
    controllerSelect?.addEventListener('change', (e) => {
      this.cfg.controllerMode = e.target.value;
      this.simB.controlDiagnostics.mode = e.target.value;
    });

    document.getElementById('select-actuator-mode')?.addEventListener('change', (e) => {
      this.cfg.actuatorMode = e.target.value;
    });

    // Interactive Keyboard Driving Controls (W/A/S/D / Arrow keys)
    window.addEventListener('keydown', (e) => {
      if (['input', 'select', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;

      const k = e.key.toLowerCase();
      let newMode = null;
      if (k === 'w' || k === 'arrowup') newMode = 'roll_forward';
      else if (k === 's' || k === 'arrowdown') newMode = 'roll_backward';
      else if (k === 'a' || k === 'arrowleft') newMode = 'steer_left';
      else if (k === 'd' || k === 'arrowright') newMode = 'steer_right';

      if (newMode) {
        e.preventDefault();
        this.cfg.actuationMode = newMode;
        if (gaitSelect) gaitSelect.value = newMode;
      }
    });

    // Experiment Selector
    document.getElementById('select-experiment').addEventListener('change', (e) => {
      this.loadExperiment(parseInt(e.target.value));
    });

    // Play/Pause
    document.getElementById('btn-play-pause').addEventListener('click', () => {
      this.isPlaying = !this.isPlaying;
      document.getElementById('btn-play-pause').textContent = this.isPlaying ? 'Pause' : 'Play';
    });

    // Reset
    document.getElementById('btn-reset').addEventListener('click', () => {
      this.simA.reset();
      this.simB.reset();
      this.accumulatedTime = 0;
      this.learningRestartAt = null;
      this.isPlaying = true;
      document.getElementById('btn-play-pause').textContent = 'Pause';
    });

    document.getElementById('btn-toggle-auto-learning')?.addEventListener('click', () => {
      this.autoLearningEnabled = !this.autoLearningEnabled;
      this.learningRestartAt = null;
      this.updateLearningHUD();
    });

    document.getElementById('btn-clear-learning')?.addEventListener('click', () => {
      this.routeLearner?.resetLearning();
      this.persistRouteLearning();
      this.simA.reset(false);
      this.simB.reset(false);
      this.accumulatedTime = 0;
      this.learningRestartAt = null;
      this.updateLearningHUD();
    });

    // Multi-rate realtime playback with an unchanged physics time step.
    document.getElementById('select-speed').addEventListener('change', (e) => {
      const requestedSpeed = parseFloat(e.target.value);
      const allowedSpeeds = [0.25, 0.5, 1, 2, 4];
      this.simSpeed = allowedSpeeds.includes(requestedSpeed) ? requestedSpeed : 1.0;
      this.accumulatedTime = 0;
      this.lastFrameTime = performance.now();
    });

    // Camera Mode
    document.getElementById('select-camera').addEventListener('change', (e) => {
      this.vis.setCameraMode(e.target.value);
    });

    // Toggle Force Overlay
    let forceOverlayOn = true;
    document.getElementById('btn-toggle-forces').addEventListener('click', () => {
      forceOverlayOn = !forceOverlayOn;
      this.vis.toggleForceOverlay(forceOverlayOn);
      document.getElementById('btn-toggle-forces').textContent = `Force Overlay: ${forceOverlayOn ? 'ON' : 'OFF'}`;
    });

    document.getElementById('btn-toggle-debug')?.addEventListener('click', (event) => {
      this.showDebugLabels = !this.showDebugLabels;
      this.vis.toggleDebugLabels(this.showDebugLabels);
      event.currentTarget.textContent = `IDs: ${this.showDebugLabels ? 'ON' : 'OFF'}`;
      event.currentTarget.classList.toggle('text-cyan-300', this.showDebugLabels);
    });

    document.getElementById('select-monitor-cable')?.addEventListener('change', (event) => {
      this.selectedCableIndex = Number(event.target.value) || 0;
      this.vis.setSelectedCable(this.selectedCableIndex);
      this.updateMonitoringHUD(this.simB.currentDiag.monitoring);
      this.updateMonitoringCharts();
    });

    document.getElementById('input-goal-threshold')?.addEventListener('input', (event) => {
      const threshold = Number(event.target.value);
      this.cfg.monitoring.goalThreshold = threshold;
      if (this.simB.monitor) this.simB.monitor.settings.goalThreshold = threshold;
      const value = document.getElementById('val-goal-threshold');
      if (value) value.textContent = `${threshold.toFixed(2)} m`;
    });

    document.querySelectorAll('[data-monitor-export]').forEach(button => {
      button.addEventListener('click', () => {
        const kind = button.dataset.monitorExport;
        if (!this.simB.monitor) return;
        downloadText(`${kind}.csv`, this.simB.monitor.exportCsv(kind));
      });
    });
    document.getElementById('btn-export-monitor-json')?.addEventListener('click', () => {
      if (!this.simB.monitor) return;
      downloadText('complete_simulation_log.json', this.simB.monitor.exportJson(), 'application/json;charset=utf-8');
    });

    // Gravity Slider Control
    document.getElementById('slider-gravity').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.gravity = [0.0, 0.0, -val];
      let label = `${val.toFixed(2)} m/s²`;
      if (Math.abs(val - 9.81) < 0.1) label = "9.81 m/s² (Earth)";
      else if (Math.abs(val - 3.71) < 0.1) label = "3.71 m/s² (Mars)";
      else if (Math.abs(val - 1.62) < 0.1) label = "1.62 m/s² (Moon)";
      else if (val === 0) label = "0.00 m/s² (Zero-G)";
      document.getElementById('val-gravity').textContent = label;

      this.updateMetricsTable();
    });

    // Parameter Sliders
    document.getElementById('slider-pretension').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.pretensionS = val;
      document.getElementById('val-pretension').textContent = `${val.toFixed(1)} N`;
      this.loadExperiment(this.cfg.experimentId);
      this.updateMetricsTable();
    });

    document.getElementById('slider-stiffness').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.kS = val;
      document.getElementById('val-stiffness').textContent = `${val.toFixed(0)} N/m`;
      this.loadExperiment(this.cfg.experimentId);
      this.updateMetricsTable();
    });

    document.getElementById('slider-restitution').addEventListener('input', (e) => {
      const lockedRestitution = 0.02;
      e.target.value = String(lockedRestitution);
      this.cfg.restitution = lockedRestitution;
      document.getElementById('val-restitution').textContent = '0.02 (non-bouncy lock)';
    });

    document.getElementById('slider-actuation').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.actuationDeltaL = val;
      document.getElementById('val-actuation').textContent = `${val.toFixed(3)} m`;
      this.updateMetricsTable();
    });

    document.getElementById('slider-roughness').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.groundRMS = val;
      document.getElementById('val-roughness').textContent = `${val.toFixed(2)} m`;
    });
    document.getElementById('slider-roughness').addEventListener('change', () => {
      // Rebuild both physics and rendering from the same terrain seed so the
      // roughness slider never leaves an old smooth mesh over new collisions.
      this.loadExperiment(this.cfg.experimentId);
    });

    document.getElementById('slider-target-speed').addEventListener('input', (e) => {
      const referenceSpeed = 1.30;
      e.target.value = String(referenceSpeed);
      this.cfg.targetSpeed = referenceSpeed;
      document.getElementById('val-target-speed').textContent = `${referenceSpeed.toFixed(2)} m/s rolling target`;
    });

    document.getElementById('slider-incline')?.addEventListener('input', (e) => {
      this.cfg.inclineDegrees = parseFloat(e.target.value);
      document.getElementById('val-incline').textContent = `${this.cfg.inclineDegrees.toFixed(0)}°`;
      this.terrain.generateSurface();
      this.vis.scene.clear();
      this.vis.initThree();
      this.vis.createTerrainMesh();
      this.vis.createRoverObjects();
      this.vis.createTrajectoryTrail();
      this.vis.createGeometryCheckpointOverlay();
    });

    document.getElementById('btn-toggle-relaxation')?.addEventListener('click', (e) => {
      this.cfg.stringRelaxation = !this.cfg.stringRelaxation;
      e.currentTarget.textContent = `RELAXATION: ${this.cfg.stringRelaxation ? 'ON' : 'OFF'}`;
      e.currentTarget.classList.toggle('text-violet-300', this.cfg.stringRelaxation);
      e.currentTarget.classList.toggle('text-slate-400', !this.cfg.stringRelaxation);
      this.updateForceLawChart();
    });

    // Structural Optimizer Runner
    document.getElementById('btn-run-optimization').addEventListener('click', () => {
      const resultsBox = document.getElementById('opt-results-box');
      resultsBox.classList.remove('hidden');

      const opt = StructuralOptimizer.runOptimization(this.cfg);

      document.getElementById('opt-pretension').textContent = `${opt.bestConfig.pretension.toFixed(0)} N`;
      document.getElementById('opt-stiffness').textContent = `${opt.bestConfig.stiffness.toFixed(0)} N/m`;
      document.getElementById('opt-score').textContent = `${Math.min(99.9, Math.max(70.0, opt.bestScore)).toFixed(1)} / 100`;

      // Apply best parameters to live simulation
      this.cfg.pretensionS = opt.bestConfig.pretension;
      this.cfg.kS = opt.bestConfig.stiffness;
      this.cfg.actuationDeltaL = opt.bestConfig.actuation;

      document.getElementById('slider-pretension').value = opt.bestConfig.pretension;
      document.getElementById('val-pretension').textContent = `${opt.bestConfig.pretension.toFixed(1)} N`;
      document.getElementById('slider-stiffness').value = opt.bestConfig.stiffness;
      document.getElementById('val-stiffness').textContent = `${opt.bestConfig.stiffness.toFixed(0)} N/m`;
      document.getElementById('slider-actuation').value = opt.bestConfig.actuation;
      document.getElementById('val-actuation').textContent = `${opt.bestConfig.actuation.toFixed(2)} m`;

      this.runFullBenchmark();
    });
  }
}

window.addEventListener('DOMContentLoaded', () => {
  window.app = new App();
});
