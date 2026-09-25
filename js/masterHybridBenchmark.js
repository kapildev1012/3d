import { DRIVE_CONTROLLER_LABELS } from './reference/controllerAlgorithms.js';
import { attachDriveControllerSuite } from './reference/simulationControllerAdapter.js';
import { Simulation, SimConfig } from './simEngine.js?v=20260821-level10';

DRIVE_CONTROLLER_LABELS['master_hybrid'] = 'Master Hybrid · Multi-Terrain Adaptive Solver';

window.addEventListener('tensegrity:drive-controller-suite-ready', () => {
    const selector = document.getElementById('select-controller');
    if (selector && !selector.querySelector(`option[value="master_hybrid"]`)) {
        const option = document.createElement('option');
        option.value = 'master_hybrid';
        option.textContent = DRIVE_CONTROLLER_LABELS['master_hybrid'];
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
        const level = cfg.terrainLevel;
        let nextMode = 'qp_mpc_payload';

        // Detect aggressive climb conditions
        const isClimbing = sim.gaitState === 'DEFORM_CLIMB' || 
                           cfg.inclineDegrees >= 8 || 
                           this.detectHighContactForce(sim);
                           
        // Detect slip conditions
        const isSlipTerrain = [9, 12, 13].includes(level);

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

        // Always enable Hermite relaxation for isolation
        cfg.stringRelaxation = true;

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
        for (const [nodeIdx, active] of Object.entries(sim.contactState || {})) {
            if (active && sim.normalForce && sim.normalForce[nodeIdx] > 25) {
                return true;
            }
        }
        return false;
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

// Headless Batch Benchmark Runner
class BatchBenchmarkRunner {
    constructor() {
        this.controllers = Object.keys(DRIVE_CONTROLLER_LABELS);
        this.terrains = Array.from({length: 14}, (_, i) => i + 1);
        this.results = [];
        this.progressEl = document.getElementById('benchmark-progress');
        this.leaderEl = document.getElementById('benchmark-leader');
        this.tbody = document.getElementById('benchmark-table-body');
    }

    async runAll() {
        this.results = [];
        this.tbody.innerHTML = '';
        
        let total = this.controllers.length * this.terrains.length;
        let count = 0;

        for (const terrain of this.terrains) {
            for (const controller of this.controllers) {
                this.progressEl.textContent = `Running Terrain ${terrain}, Controller ${controller} (${count}/${total})`;
                
                const result = await this.runSingleHeadless(controller, terrain);
                this.results.push(result);
                
                this.appendResultRow(result);
                this.updateLeaderboard();
                
                count++;
                await new Promise(r => setTimeout(r, 0)); // Yield to UI
            }
        }

        this.progressEl.textContent = 'Completed';
        this.exportCSV();
        this.exportJSON();
    }

    async runSingleHeadless(controller, level) {
        // Run in fast headless mode
        const cfg = new SimConfig({
            controllerMode: controller,
            terrainLevel: level,
            T_end: 120.0,
            showAnimation: false // Headless speedup
        });

        const sim = new Simulation(cfg);
        sim.initialize();

        const maxSteps = cfg.T_end / cfg.dt;
        let step = 0;
        let status = 'TIMEOUT';
        
        const startY = sim.rover.q0_outer[0][1];
        
        while (step < maxSteps) {
            sim.step();
            step++;
            
            // Check win/loss conditions
            const centroid = sim.computeCentroid(sim.rover.q_outer);
            if (centroid[1] >= 60.0) { // Goal Y
                status = 'COMPLETED';
                break;
            }
            if (sim.isOverloaded || sim.isCollapsed) {
                status = 'OVERLOAD';
                break;
            }
        }

        const finalCentroid = sim.computeCentroid(sim.rover.q_outer);
        const distance = finalCentroid[1] - startY;
        const time = step * cfg.dt;
        const avgSpeed = time > 0 ? distance / time : 0;
        
        // Compute CPS
        const cps = (avgSpeed / 1.30) + (distance / 50.0); // Simplified CPS score
        
        return {
            level,
            controller,
            status,
            distance: distance.toFixed(2),
            time: time.toFixed(2),
            avgSpeed: avgSpeed.toFixed(2),
            maxG: (sim.diagnostics?.maxPayloadG || 1.0).toFixed(2),
            maxTension: (sim.diagnostics?.maxTension || 0.0).toFixed(1),
            deformation: (sim.diagnostics?.maxDeformation || 0.0).toFixed(3),
            obstacles: level >= 10 ? 10 : 0,
            cps: cps.toFixed(2)
        };
    }

    appendResultRow(res) {
        const tr = document.createElement('tr');
        tr.innerHTML = `
            <td class="p-2 border-t border-slate-800">Level ${res.level}</td>
            <td class="p-2 border-t border-slate-800">${DRIVE_CONTROLLER_LABELS[res.controller] || res.controller}</td>
            <td class="p-2 border-t border-slate-800">
                <span class="px-2 py-1 rounded text-[10px] font-bold ${res.status === 'COMPLETED' ? 'bg-emerald-900/50 text-emerald-400' : 'bg-rose-900/50 text-rose-400'}">${res.status}</span>
            </td>
            <td class="p-2 border-t border-slate-800">${res.distance}</td>
            <td class="p-2 border-t border-slate-800">${res.time}</td>
            <td class="p-2 border-t border-slate-800">${res.avgSpeed}</td>
            <td class="p-2 border-t border-slate-800">${res.maxG}</td>
            <td class="p-2 border-t border-slate-800">${res.maxTension}</td>
            <td class="p-2 border-t border-slate-800">${res.deformation}</td>
            <td class="p-2 border-t border-slate-800">${res.obstacles}</td>
        `;
        this.tbody.appendChild(tr);
    }

    updateLeaderboard() {
        const sorted = [...this.results].sort((a, b) => parseFloat(b.cps) - parseFloat(a.cps));
        if (sorted.length > 0) {
            this.leaderEl.textContent = `${DRIVE_CONTROLLER_LABELS[sorted[0].controller]} (CPS: ${sorted[0].cps})`;
        }
    }

    exportCSV() {
        const headers = ['Level', 'Controller', 'Status', 'Distance', 'Time', 'AvgSpeed', 'MaxG', 'MaxTension', 'Deformation', 'CPS'];
        const rows = this.results.map(r => [r.level, r.controller, r.status, r.distance, r.time, r.avgSpeed, r.maxG, r.maxTension, r.deformation, r.cps].join(','));
        const csv = [headers.join(','), ...rows].join('\\n');
        
        const blob = new Blob([csv], { type: 'text/csv' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'master_benchmark_comparison.csv';
        a.click();
    }

    exportJSON() {
        const blob = new Blob([JSON.stringify(this.results, null, 2)], { type: 'application/json' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'master_benchmark_comparison.json';
        a.click();
    }
}

document.addEventListener('DOMContentLoaded', () => {
    const btn = document.getElementById('btn-run-benchmark');
    if (btn) {
        btn.addEventListener('click', () => {
            const runner = new BatchBenchmarkRunner();
            runner.runAll();
        });
    }
});

// Expose to window for console/CLI callable entry point
window.BatchBenchmarkRunner = BatchBenchmarkRunner;
window.runMasterBenchmark = () => new BatchBenchmarkRunner().runAll();
