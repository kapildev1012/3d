import { Simulation } from './simEngine.js?v=20260821-roll1';
import { DRIVE_CONTROLLER_LABELS } from './reference/controllerAlgorithms.js';
import { attachDriveControllerSuite } from './reference/simulationControllerAdapter.js';

const installationFlag = Symbol.for('tensegrity.drive-controller-suite');

if (!Simulation.prototype[installationFlag]) {
  const naturalController = Simulation.prototype.updateNaturalRollingController;

  Simulation.prototype.updateNaturalRollingController = function (...args) {
    const mode = this.cfg.controllerMode || 'natural_support_face';
    if (mode === 'natural_support_face' || !DRIVE_CONTROLLER_LABELS[mode]) {
      return naturalController.apply(this, args);
    }
    try {
      const [centroid, velocity, , obstacle] = args;
      return attachDriveControllerSuite(this).solve(mode, centroid, velocity, obstacle);
    } catch (error) {
      console.error(`[Drive controller suite] ${mode} failed; using support-face fallback`, error);
      const fallback = naturalController.apply(this, args);
      fallback.diagnostics.mode = mode;
      fallback.diagnostics.modeLabel = `${DRIVE_CONTROLLER_LABELS[mode]} · safe fallback`;
      return fallback;
    }
  };

  Object.defineProperty(Simulation.prototype, installationFlag, { value: true });
}

const selector = document.getElementById('select-controller');
if (selector) {
  for (const [value, label] of Object.entries(DRIVE_CONTROLLER_LABELS)) {
    if (selector.querySelector(`option[value="${value}"]`)) continue;
    const option = document.createElement('option');
    option.value = value;
    option.textContent = label;
    selector.append(option);
  }
}

window.dispatchEvent(new CustomEvent('tensegrity:drive-controller-suite-ready', {
  detail: { controllers: { ...DRIVE_CONTROLLER_LABELS } }
}));

