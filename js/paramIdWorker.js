/**
 * PARAMIDWORKER.JS - runs the particle-filter identification off the main thread
 * for the Tools tab (js/paramIdPanel.js). Message in: settings; messages out:
 * { type: 'problem' | 'step' | 'done' | 'error' }.
 */
import { buildIdentificationProblem, problemInfo } from './paramIdPanelCore.js?v=20261029-pf1';
import { particleFilter } from './paramIdentification.js?v=20261029-pf1';

self.onmessage = ({ data: settings }) => {
  try {
    const problem = buildIdentificationProblem(settings);
    self.postMessage({ type: 'problem', info: problemInfo(problem) });
    const it = particleFilter(problem, { particles: settings.particles, ...problem.filterDefaults });
    let r;
    while (!(r = it.next()).done) self.postMessage({ type: 'step', step: r.value });
    self.postMessage({ type: 'done', summary: r.value });
  } catch (error) {
    self.postMessage({ type: 'error', message: error.message });
  }
};
