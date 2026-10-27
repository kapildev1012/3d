import { DRIVE_CONTROLLER_LABELS } from './reference/controllerAlgorithms.js';
import { attachDriveControllerSuite } from './reference/simulationControllerAdapter.js';
import { Simulation, SimConfig, SphericalRoverModel, TerrainModel } from './simEngine.js?v=20261027-ob1';

// DRIVE_CONTROLLER_LABELS is frozen; keep the hybrid label locally instead of
// mutating it (the old assignment threw "object is not extensible" in strict
// module code and silently disabled this whole module).
const MASTER_HYBRID_LABEL = 'Master Hybrid · Multi-Terrain Adaptive Solver';
const labelFor = mode => (mode === 'master_hybrid' ? MASTER_HYBRID_LABEL : DRIVE_CONTROLLER_LABELS[mode] || mode);

window.addEventListener('tensegrity:drive-controller-suite-ready', () => {
    const selector = document.getElementById('select-controller');
    if (selector && !selector.querySelector(`option[value="master_hybrid"]`)) {
        const option = document.createElement('option');
        option.value = 'master_hybrid';
        option.textContent = MASTER_HYBRID_LABEL;
        selector.prepend(option);
    }
});

class MasterHybridAdapter {
    constructor(simulation) {
        this.simulation = simulation;
        this.suite = attachDriveControllerSuite(simulation);
        this.blendingWindow = 10;
        this.blendStep = 0;
        this.previousMode = 'qp_mpc_payload';
        this.currentMode = 'qp_mpc_payload';
        this.previousCommand = null;
        this.activeCommand = null;
    }

    solve(mode, centroid, velocity, obstacle) {
        if (mode !== 'master_hybrid') {
            return this.suite.solve(mode, centroid, velocity, obstacle);
        }

        const sim = this.simulation;
        const cfg = sim.cfg;
        let nextMode = 'qp_mpc_payload';

        // Mode switching uses only on-board measurements (no terrain IDs):
        // climbing = obstacle phase or a measured uphill grade / contact load,
        // slip = measured contact slip speed relative to the shell speed.
        const diag = sim.currentDiag || {};
        const isClimbing = sim.obstaclePhase === 'DEFORM_CLIMB' ||
                           cfg.inclineDegrees >= 8 ||
                           this.detectHighContactForce(sim);
        const speed = Math.hypot(...(diag.velocityVector || [0, 0, 0]));
        const isSlipTerrain = (diag.slipSpeed || 0) > Math.max(0.15, 0.6*speed);

        if (isSlipTerrain) {
            nextMode = 'ilqr_minimax'; // Robust adversarial
        } else if (isClimbing) {
            nextMode = 'ilqr_minimax_penalty';
        } else {
            nextMode = 'qp_mpc_payload'; // Fast flat terrain cruise
        }

        if (nextMode !== this.currentMode) {
            this.previousMode = this.currentMode;
            this.currentMode = nextMode;
            this.blendStep = this.blendingWindow;
            this.previousCommand = this.activeCommand;
        }

        // (Hermite relaxation is already always active for the adaptive model in
        // calcOuterCableForce; the old global cfg.stringRelaxation = true also
        // changed Model A through the shared config, so it was removed.)

        const result = this.suite.solve(this.currentMode, centroid, velocity, obstacle);
        this.activeCommand = result;

        if (this.blendStep > 0 && this.previousCommand) {
            const t = 1 - (this.blendStep / this.blendingWindow);
            const smoothT = t * t * (3 - 2 * t); // Cubic Hermite spline blending
            
            result.cableTargets = result.cableTargets.map((val, idx) => 
                this.previousCommand.cableTargets[idx] * (1 - smoothT) + val * smoothT
            );
            
            this.blendStep--;
        }

        result.diagnostics.modeLabel = `Hybrid (${this.currentMode}) [Blend: ${this.blendStep}]`;
        return result;
    }

    detectHighContactForce(sim) {
        const weight = (sim.dynamicNodeMass*sim.rover.nOuter+sim.cfg.coreMass)*Math.abs(sim.cfg.gravity[2]);
        return (sim.currentDiag?.contacts || []).some(contact => (contact.normalForce || 0) > 0.6*weight);
    }
}

// Override natural support controller in Simulation to inject master hybrid
const originalController = Simulation.prototype.updateNaturalRollingController;
Simulation.prototype.updateNaturalRollingController = function (...args) {
    const mode = this.cfg.controllerMode;
    if (mode === 'master_hybrid') {
        if (!this._hybridAdapter) this._hybridAdapter = new MasterHybridAdapter(this);
        return this._hybridAdapter.solve(mode, ...args);
    }
    return originalController.apply(this, args);
};

// Physics-audit batch runner (headless, chunked so the UI stays responsive).
// Every run uses the real Simulation API on the Lagrangian DAE core and
// reports measured progress, energy residual and assist share.
class BatchBenchmarkRunner {
    constructor(options = {}) {
        this.controllers = options.controllers || ['natural_support_face', 'sgt_tipping'];
        this.levels = options.levels || [1, 4, 7];
        this.modes = options.modes || ['assisted', 'pure'];
        this.duration = options.duration || 20;
        this.results = [];
        this.progressEl = document.getElementById('benchmark-progress');
        this.leaderEl = document.getElementById('benchmark-leader');
        this.tbody = document.getElementById('benchmark-table-body');
    }

    async runAll() {
        this.results = [];
        if (this.tbody) this.tbody.innerHTML = '';
        const jobs = [];
        for (const level of this.levels) for (const controller of this.controllers) for (const mode of this.modes) jobs.push({ level, controller, mode });
        for (let i = 0; i < jobs.length; i++) {
            const job = jobs[i];
            if (this.progressEl) this.progressEl.textContent = `${i+1}/${jobs.length} · L${job.level} · ${labelFor(job.controller)} · ${job.mode}`;
            const result = await this.runSingle(job);
            this.results.push(result);
            this.appendResultRow(result);
            this.updateLeaderboard();
        }
        if (this.progressEl) this.progressEl.textContent = `Completed ${jobs.length} runs`;
        this.exportCSV();
        this.exportJSON();
    }

    async runSingle({ level, controller, mode }) {
        const cfg = new SimConfig({ terrainLevel: level, experimentId: level, enableDiagnosticsLog: false,
            monitoring: { enabled: false } });
        cfg.applyStandardPerformanceProfile();
        if (controller === 'sgt_tipping') cfg.applyResearchProfile();
        Object.assign(cfg, { physicsCore: 'dae', controllerMode: controller, pureCableActuation: mode === 'pure' });
        const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
        const y0 = sim.currentDiag.centroid[1];
        const steps = Math.round(this.duration/cfg.dt);
        const chunk = 250;
        const started = performance.now();
        for (let k = 0; k < steps; k += chunk) {
            for (let j = 0; j < chunk && k+j < steps; j++) sim.step();
            await new Promise(resolve => setTimeout(resolve, 0));
        }
        const ledger = sim.currentDiag.energyLedger;
        const dy = sim.currentDiag.centroid[1]-y0;
        return {
            level, controller, mode, duration: this.duration,
            progressY: dy, speed: dy/this.duration,
            path: sim.metrics.distanceTraveled,
            maxResidual: ledger.maxAbsResidual,
            assistShare: ledger.assistShare,
            actuatorEnergy: sim.metrics.energyCost,
            maxBarError: sim.metrics.maxConstraintError,
            tips: sim.sgt?.tipCount ?? sim.completedRolls,
            wallSeconds: (performance.now()-started)/1000
        };
    }

    appendResultRow(res) {
        if (!this.tbody) return;
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${res.level}</td><td>${labelFor(res.controller)}</td><td>${res.mode === 'pure' ? 'cable-only' : 'assisted'}</td>`+
            `<td>${res.progressY.toFixed(2)}</td><td>${res.speed.toFixed(3)}</td><td class="mono">${res.maxResidual.toExponential(1)}</td>`+
            `<td>${(100*res.assistShare).toFixed(0)}%</td>`;
        this.tbody.appendChild(tr);
    }

    updateLeaderboard() {
        const pure = this.results.filter(r => r.mode === 'pure').sort((a, b) => b.speed-a.speed);
        if (pure.length && this.leaderEl) this.leaderEl.textContent = `${labelFor(pure[0].controller)} · L${pure[0].level} · ${pure[0].speed.toFixed(3)} m/s`;
    }

    download(name, contents, type) {
        const a = document.createElement('a');
        a.href = URL.createObjectURL(new Blob([contents], { type }));
        a.download = name;
        a.click();
        setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    }

    exportCSV() {
        const keys = ['level', 'controller', 'mode', 'duration', 'progressY', 'speed', 'path', 'maxResidual', 'assistShare', 'actuatorEnergy', 'maxBarError', 'tips', 'wallSeconds'];
        const rows = this.results.map(r => keys.map(key => r[key]).join(','));
        this.download('physics_audit_batch.csv', [keys.join(','), ...rows].join('\n'), 'text/csv');
    }

    exportJSON() {
        this.download('physics_audit_batch.json', JSON.stringify(this.results, null, 2), 'application/json');
    }
}

function bindBatchButton() {
    const btn = document.getElementById('btn-run-benchmark');
    if (!btn || btn.dataset.bound) return;
    btn.dataset.bound = '1';
    btn.addEventListener('click', async () => {
        btn.disabled = true;
        try { await new BatchBenchmarkRunner().runAll(); } finally { btn.disabled = false; }
    });
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', bindBatchButton);
else bindBatchButton();

// Expose to window for console/CLI callable entry point
window.BatchBenchmarkRunner = BatchBenchmarkRunner;
window.runMasterBenchmark = options => new BatchBenchmarkRunner(options).runAll();
