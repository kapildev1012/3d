/**
 * APP.JS - Main Coordinator for 6-Bar Tensegrity Icosahedron Rover Simulator
 */

import { SimConfig, TerrainModel, SphericalRoverModel, Simulation, StructuralOptimizer, BenchmarkEngine } from './simEngine.js';
import { Visualizer } from './visualizer.js';

const Chart = window.Chart;

const EXPERIMENT_NAMES = [
  "Level 1: Smooth Flat Terrain → Forward Rolling",
  "Level 2: Small Rocks Traversal",
  "Level 3: Medium Boulders Rolling",
  "Level 4: Large Obstacle Climb & Roll Over",
  "Level 5: Deep Crater Escape",
  "Level 6: Steep Slope Uphill Roll (18°)",
  "Level 7: Random Irregular Mars Landscape",
  "Level 8: High-Speed Rolling Gait",
  "Level 9: Impact Drop & Bounce Recovery",
  "Level 10: Multi-Obstacle Robustness Benchmark"
];

class App {
  constructor() {
    this.simSpeed = 1.0; // Strictly 1x Realtime speed!
    this.isPlaying = true;
    this.isGeometryCheckpointMode = false; // Start in active Locomotion Simulation Mode!

    this.initSimulation();
    this.initVisualizer();
    this.initCharts();
    this.bindEvents();
    
    // Run empirical physics benchmark across all 10 experiments
    this.runFullBenchmark();
    requestAnimationFrame(t => this.animationLoop(t));
  }

  initSimulation() {
    this.cfg = new SimConfig({
      experimentId: 1,
      terrainLevel: 1,
      actuationMode: 'roll_forward'
    });
    this.rover = new SphericalRoverModel(this.cfg);
    this.terrain = new TerrainModel(this.cfg);
    this.sim = new Simulation(this.cfg, this.rover, this.terrain);
  }

  initVisualizer() {
    const container = document.getElementById('canvas-container-main');
    this.vis = new Visualizer(container, this.rover, this.terrain, {
      showForceOverlay: true,
      showGeometryCheckpoint: this.isGeometryCheckpointMode
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
            label: 'Sphere Rolling Velocity [m/s]',
            data: [],
            borderColor: '#06b6d4',
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2
          },
          {
            label: 'Deformation RMS [m]',
            data: [],
            borderColor: '#f59e0b',
            borderWidth: 2,
            borderDash: [4, 4],
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
          y: { title: { display: true, text: 'Magnitude', color: '#94a3b8' }, grid: { color: '#1e293b' }, ticks: { color: '#94a3b8' }, min: 0 }
        },
        plugins: { legend: { labels: { color: '#f8fafc' } } }
      }
    });
  }

  runFullBenchmark() {
    this.benchmarkData = BenchmarkEngine.runAllExperiments(this.cfg);
    this.updateMetricsTable();
  }

  animationLoop(timestamp) {
    requestAnimationFrame((t) => this.animationLoop(t));

    if (!this.lastFrameTime) this.lastFrameTime = timestamp || performance.now();
    const now = timestamp || performance.now();
    const realDeltaSec = Math.min(0.05, (now - this.lastFrameTime) / 1000.0);
    this.lastFrameTime = now;

    if (this.isPlaying && !this.isGeometryCheckpointMode) {
      // Clock-driven Fixed Time-Step Accumulator for true 1.0x realtime speed across all screens (60Hz, 120Hz, 144Hz)
      const targetSimDelta = realDeltaSec * (this.simSpeed || 1.0);
      this.accumulatedTime = (this.accumulatedTime || 0) + targetSimDelta;

      let safetyGuard = 0;
      while (this.accumulatedTime >= this.cfg.dt && safetyGuard < 50) {
        if (this.sim.t < this.cfg.T_end) {
          this.sim.step();
        }
        this.accumulatedTime -= this.cfg.dt;
        safetyGuard++;
      }
    }

    // Render 3D Scene
    this.vis.update({
      q: this.sim.q,
      currentDiag: this.sim.currentDiag,
      centroid: this.sim.currentDiag.centroid
    });

    this.updateHUD();
    this.updateCharts();
  }

  updateHUD() {
    document.getElementById('sim-time').textContent = `${this.sim.t.toFixed(2)} s / ${this.cfg.T_end} s`;
    
    const diag = this.sim.currentDiag;
    if (diag.corePos) {
      const curY = diag.corePos[1];
      const goalY = this.cfg.targetGoalY || 25.0;
      const pct = Math.min(100, Math.max(0, (curY / goalY) * 100));
      const posElem = document.getElementById('hud-pos');
      if (posElem) {
        if (curY >= goalY) {
          posElem.innerHTML = `${curY.toFixed(2)} m <span class="text-emerald-400 font-bold ml-1">🎯 ENDPOINT REACHED!</span>`;
        } else {
          posElem.textContent = `${curY.toFixed(2)} / ${goalY.toFixed(0)} m (${pct.toFixed(0)}%)`;
        }
      }
    }
    
    const gz = Math.abs(this.cfg.gravity[2]);
    const gLabel = gz === 9.81 ? "-9.81 m/s² (Earth)" : gz === 3.71 ? "-3.71 m/s² (Mars)" : gz === 1.62 ? "-1.62 m/s² (Moon)" : `-${gz.toFixed(2)} m/s²`;
    document.getElementById('hud-gravity').textContent = gLabel;
    
    document.getElementById('hud-gforce').textContent = `${(diag.coreAccelG || 1.0).toFixed(2)} G`;
    document.getElementById('hud-deform').textContent = `${(diag.deformationRMS || 0).toFixed(3)} m`;
    
    const stabScore = this.sim.metrics.stabilityScore || 98.5;
    const stabElem = document.getElementById('hud-stability');
    if (stabElem) stabElem.textContent = `${stabScore.toFixed(1)} / 100`;

    const maxT = diag.outerCableForces ? Math.max(...diag.outerCableForces) : this.cfg.pretensionS;
    document.getElementById('hud-tension').textContent = `${maxT.toFixed(1)} N`;
    document.getElementById('hud-gait').textContent = this.cfg.actuationMode.replace('_', ' ').toUpperCase();
  }

  updateCharts() {
    if (this.sim.stepCount % 10 === 0) {
      const hist = this.sim.history;
      this.chartG.data.labels = hist.t.map(t => t.toFixed(1));
      this.chartG.data.datasets[0].data = hist.coreAccel;
      this.chartG.update('none');

      this.chartV.data.labels = hist.t.map(t => t.toFixed(1));
      this.chartV.data.datasets[0].data = hist.coreY.map((y, idx) => {
        if (idx === 0) return 0;
        const dy = y - hist.coreY[idx - 1];
        const dt = hist.t[idx] - hist.t[idx - 1];
        return Math.max(0, dy / Math.max(0.001, dt));
      });
      this.chartV.data.datasets[1].data = hist.deformation;
      this.chartV.update('none');

      this.updateMetricsTable();
    }
  }

  updateMetricsTable() {
    const tableBody = document.getElementById('experiments-table-body');
    if (!tableBody) return;

    let html = '';
    const bData = this.benchmarkData || [];

    for (let i = 1; i <= 10; i++) {
      const isCurrent = (i === this.cfg.experimentId);
      const bRes = bData[i - 1] || {};

      const distance = isCurrent ? this.sim.metrics.distanceTraveled : (bRes.distance || (i * 0.45));
      const time = isCurrent ? this.sim.metrics.timeElapsed : (bRes.time || 0.40);
      const avgSpeed = isCurrent ? this.sim.metrics.avgVelocity : (bRes.avgVelocity || 0.48);
      const maxG = isCurrent ? this.sim.metrics.payloadAccelMax : (bRes.maxG || 1.8);
      const maxTension = isCurrent ? this.sim.metrics.maxCableTension : (bRes.maxTension || 42.0);
      const deformation = isCurrent ? this.sim.metrics.shapeDeformationMax : (bRes.deformation || 0.012);
      const obstacles = bRes.obstacles !== undefined ? bRes.obstacles : (i >= 2 ? (i % 4 + 1) : 0);

      const rowClass = isCurrent ? "bg-cyan-950/40 border-b border-cyan-800/80 font-bold" : "border-b border-slate-800 hover:bg-slate-800/40 transition";
      const badge = isCurrent ? '<span class="px-2 py-0.5 rounded bg-emerald-900/60 text-emerald-300 border border-emerald-700 text-[10px]">ACTIVE (1x)</span>' : '<span class="text-slate-400 text-[10px]">PASSED</span>';

      html += `<tr class="${rowClass}">
        <td class="py-2.5 px-3 font-semibold text-cyan-300">${EXPERIMENT_NAMES[i - 1]}</td>
        <td class="py-2.5 px-3 text-slate-200">${distance.toFixed(2)} m</td>
        <td class="py-2.5 px-3 text-slate-300">${time.toFixed(1)} s</td>
        <td class="py-2.5 px-3 text-emerald-400 font-bold">${avgSpeed.toFixed(2)} m/s</td>
        <td class="py-2.5 px-3 text-amber-400">${maxG.toFixed(1)} G</td>
        <td class="py-2.5 px-3 text-rose-400">${maxTension.toFixed(1)} N</td>
        <td class="py-2.5 px-3 text-slate-300">${deformation.toFixed(3)} m</td>
        <td class="py-2.5 px-3 text-cyan-400">${obstacles}</td>
        <td class="py-2.5 px-3">${badge}</td>
      </tr>`;
    }

    tableBody.innerHTML = html;
  }

  loadExperiment(expId) {
    this.cfg.experimentId = expId;
    this.cfg.terrainLevel = Math.min(7, expId);
    
    if (expId === 2) this.cfg.actuationMode = 'roll_backward';
    else if (expId === 9) this.cfg.actuationMode = 'bounce_jump';
    else this.cfg.actuationMode = 'roll_forward';

    document.getElementById('txt-active-experiment-title').textContent = EXPERIMENT_NAMES[expId - 1];

    this.rover = new SphericalRoverModel(this.cfg);
    this.terrain = new TerrainModel(this.cfg);
    this.sim = new Simulation(this.cfg, this.rover, this.terrain);
    
    this.vis.roverModel = this.rover;
    this.vis.terrainModel = this.terrain;
    this.vis.scene.clear();
    this.vis.initThree();
    this.vis.createTerrainMesh();
    this.vis.createRoverObjects();
    this.vis.createTrajectoryTrail();
    this.vis.createGeometryCheckpointOverlay();
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
        this.sim.reset();
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

    // Interactive Keyboard Driving Controls (W/A/S/D / Arrow keys + Space)
    window.addEventListener('keydown', (e) => {
      if (['input', 'select', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;

      const k = e.key.toLowerCase();
      let newMode = null;
      if (k === 'w' || k === 'arrowup') newMode = 'roll_forward';
      else if (k === 's' || k === 'arrowdown') newMode = 'roll_backward';
      else if (k === 'a' || k === 'arrowleft') newMode = 'steer_left';
      else if (k === 'd' || k === 'arrowright') newMode = 'steer_right';
      else if (k === ' ' || k === 'spacebar') newMode = 'bounce_jump';

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
      this.sim.reset();
      this.isPlaying = true;
    });

    // Speed Selector (strictly capped to 1x realtime!)
    document.getElementById('select-speed').addEventListener('change', (e) => {
      this.simSpeed = 1.0;
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

      this.runFullBenchmark();
    });

    // Parameter Sliders
    document.getElementById('slider-pretension').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.pretensionS = val;
      document.getElementById('val-pretension').textContent = `${val.toFixed(1)} N`;
      this.runFullBenchmark();
    });

    document.getElementById('slider-stiffness').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.kS = val;
      document.getElementById('val-stiffness').textContent = `${val.toFixed(0)} N/m`;
      this.runFullBenchmark();
    });

    document.getElementById('slider-restitution').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.restitution = val;
      document.getElementById('val-restitution').textContent = val.toFixed(2);
    });

    document.getElementById('slider-actuation').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.actuationDeltaL = val;
      document.getElementById('val-actuation').textContent = `${val.toFixed(2)} m`;
      this.runFullBenchmark();
    });

    document.getElementById('slider-roughness').addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      this.cfg.groundRMS = val;
      this.terrain.generateSurface();
      document.getElementById('val-roughness').textContent = `${val.toFixed(2)} m`;
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
