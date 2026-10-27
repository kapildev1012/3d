/**
 * EXPERIMENTPROTOCOL.JS - Shared, deterministic evaluation protocol
 *
 * Used both by the in-browser benchmark and by tools/run_experiments.mjs so
 * the numbers in the paper and the numbers on screen come from the same code.
 *
 * A run = (method, scenario, seed, level, duration). Every method uses the
 * Lagrangian DAE core with Algorithm 1 and the passive contact model unless
 * stated otherwise; all cable-only methods share the same actuator limits
 * (outer stroke 0.15 m = 30 % of rest length, payload stroke 0.15 m, spool
 * speed 0.20 m/s), so differences come from the controller alone.
 *
 * Metrics (desired heading +y, reference = straight line x = x0):
 *   progress      Delta y [m] and mean speed [m/s]
 *   rmse, mae     cross-track error e(t) = x_c(t) - x0 [m]: RMS and max |e|
 *                 (the key 'mae' is the MAXIMUM absolute error, reported as max|e|)
 *   success       Delta y >= successDistance within the run
 *   recovery      time after a disturbance until the rover is 0.05 m ahead of
 *                 its pre-disturbance position [s] (NaN if never)
 *   effort        integral of u'u dt over all rest-length offsets [m^2 s]
 *   actuatorJ     sum tension x |rest-length change| [J]
 *   CoT           actuatorJ / (m g Delta y) [-]
 *   solveMs       mean / worst controller wall time per call [ms]
 *   maxResidual   max |E - E0 - sum W| [J];  maxBarError [m];  assistShare [-]
 *   maxStepDrift  max per-step unexplained energy before the corrector [J]
 *   driftAbs      sum of |per-step unexplained energy| over the run [J]
 */

import { SimConfig, TerrainModel, SphericalRoverModel, Simulation } from './simEngine.js?v=20261027-ob1';
import { residualTunedParams } from './residualDeploy.js?v=20261027-ob1';

export const METHODS = Object.freeze({
  sgt: {
    label: 'SGT (proposed)', short: 'SGT', proposed: true,
    configure: cfg => cfg.applyResearchProfile()
  },
  heuristic: {
    label: 'Support-face heuristic', short: 'Heuristic',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'natural_support_face'; }
  },
  lqr: {
    label: 'Riccati-LQR effort + support face', short: 'LQR',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'riccati_lqr'; }
  },
  ilqr: {
    label: 'iLQR effort + support face', short: 'iLQR',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'ilqr_true'; }
  },
  mpc: {
    label: 'QP-MPC effort + support face', short: 'QP-MPC',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'qp_mpc_proj'; }
  },
  mpcGnss: {
    label: 'MPC + GNSS (satellite positioning, Kalman-fused steering)', short: 'MPC+GNSS',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'mpc_gnss'; }
  },
  guidedPolicy: {
    label: 'MPC + GPS: MPC-guided policy search (learned policy)', short: 'MPC+GPS',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'guided_policy'; }
  },
  residualMppi: {
    label: 'MPC + RL: residual-MPPI actor on SGT (default MPC/reward, tension shield)', short: 'MPC+RL',
    configure: cfg => {
      cfg.applyResearchProfile(); cfg.controllerMode = 'residual_mppi';
      cfg.tensionShield = { coreTensionMin: 10, outerTensionMax: 150 };
    }
  },
  residualMppiLlm: {
    label: 'MPC + RL + LLM: residual-MPPI actor with LLM-co-designed MPC/reward (tension shield)', short: 'MPC+RL+LLM',
    configure: cfg => {
      cfg.applyResearchProfile(); cfg.controllerMode = 'residual_mppi_llm';
      const tuned = residualTunedParams('residual_mppi_llm', cfg.experimentId);
      cfg.tensionShield = { coreTensionMin: tuned.minCableTensionN ?? 10, outerTensionMax: Math.min(150, tuned.maxCableTensionN ?? 150) };
    }
  },
  sgtShield: {
    label: 'SGT + tension-certified safety filter', short: 'SGT+shield',
    configure: cfg => { cfg.applyResearchProfile(); cfg.tensionShield = true; }
  },
  policyShield: {
    label: 'MPC + GPS (guided policy search) + tension shield', short: 'MPC+GPS+shield',
    configure: cfg => { cfg.applyResearchProfile(); cfg.controllerMode = 'guided_policy'; cfg.tensionShield = true; }
  },
  legacyAssisted: {
    label: 'Legacy engine with external assists (not admissible)', short: 'Legacy+assist', reference: true,
    configure: cfg => {
      Object.assign(cfg, { physicsCore: 'legacy', contactModel: 'legacy', pureCableActuation: false,
        controllerMode: 'natural_support_face' });
    }
  }
});

export const SCENARIOS = Object.freeze({
  nominal: { label: 'Nominal', apply: () => {} },
  impulse: { label: 'Lateral impulse 0.5 m/s', impulse: { time: 6, dv: [0.5, 0, 0] }, apply: () => {} },
  push: { label: 'Step push 10 N, 2 s', apply: cfg => { cfg.disturbance = { type: 'step', time: 6, duration: 2, force: [10, 0, 0] }; } },
  gaussian: { label: 'Gaussian sensor noise', apply: cfg => { cfg.sensorNoise = { model: 'gaussian', sigmaQ: 0.005, sigmaV: 0.02 }; } },
  laplace: { label: 'Laplace sensor noise', apply: cfg => { cfg.sensorNoise = { model: 'laplace', sigmaQ: 0.005, sigmaV: 0.02 }; } },
  outlier: { label: 'Sensor outliers (2 %, 10x)', apply: cfg => { cfg.sensorNoise = { model: 'outlier', sigmaQ: 0.005, sigmaV: 0.02, outlierRate: 0.02, outlierScale: 10 }; } },
  stiffness: { label: 'Cable stiffness -20 %', apply: cfg => { cfg.plantStiffnessScale = 0.8; } },
  friction: { label: 'Friction -40 %', apply: cfg => { cfg.plantFrictionScale = 0.6; } },
  payload: { label: 'Payload mass +30 %', apply: cfg => { cfg.plantPayloadMassScale = 1.3; } },
  delay: { label: 'Actuator delay 100 ms', apply: cfg => { cfg.actuatorDelay = 0.10; } }
});

export const ABLATIONS = Object.freeze({
  full: { label: 'SGT (full)', apply: () => {} },
  noPayload: { label: 'w/o payload actuation', apply: cfg => { cfg.sgtOptions = { ...(cfg.sgtOptions || {}), coreStrokeRatio: 0 }; } },
  noFeasibility: { label: 'w/o feasibility edge choice', apply: cfg => { cfg.sgtOptions = { ...(cfg.sgtOptions || {}), edgeSelection: 'alignment' }; } },
  noGeometric: { label: 'w/o bar geometric stiffness', apply: cfg => { cfg.sgtOptions = { ...(cfg.sgtOptions || {}), ignoreBarCompression: true }; } },
  sparse8: { label: 'sparse: 8 cables', apply: cfg => { cfg.sgtOptions = { ...(cfg.sgtOptions || {}), activeCables: 8 }; } },
  noEnergyRow: { label: 'DAE w/o energy row', apply: cfg => { cfg.daeEnergyCorrection = false; } },
  legacyCore: { label: 'legacy SHAKE core', apply: cfg => { cfg.physicsCore = 'legacy'; cfg.contactModel = 'legacy'; } }
});

export function buildConfig({ method = 'sgt', scenario = 'nominal', ablation = null, seed = 42, level = 1, overrides = null } = {}) {
  const cfg = new SimConfig({
    terrainLevel: level, experimentId: level, seed,
    enableDiagnosticsLog: false, monitoring: { enabled: false }
  });
  cfg.applyStandardPerformanceProfile();
  METHODS[method].configure(cfg);
  SCENARIOS[scenario]?.apply(cfg);
  if (ablation) ABLATIONS[ablation].apply(cfg);
  if (overrides) Object.assign(cfg, overrides);         // e.g. { residualOnlinePlanning: true }
  return cfg;
}

/**
 * Run one experiment. onChunk(progress) is awaited every `chunkSteps` steps
 * (lets the browser stay responsive). Returns metrics and a down-sampled
 * time series (every 0.1 s) for plotting.
 */
export async function runExperiment(spec, { duration = 20, successDistance = 1.0, chunkSteps = 500, onChunk = null } = {}) {
  const cfg = buildConfig(spec);
  const sim = new Simulation(cfg, new SphericalRoverModel(cfg), new TerrainModel(cfg), 'adaptive');
  const scenario = SCENARIOS[spec.scenario || 'nominal'];
  const steps = Math.round(duration/cfg.dt);
  const sampleEvery = Math.max(1, Math.round(0.1/cfg.dt));
  const x0 = sim.currentDiag.centroid[0];
  const y0 = sim.currentDiag.centroid[1];
  const series = { t: [], x: [], y: [], e: [], u: [], residual: [], barError: [] };
  let sumE2 = 0, maxAbsE = 0, nE = 0, effort = 0;
  let impulseDone = false, yBeforeDisturbance = null, recovery = NaN;
  const disturbanceTime = scenario.impulse?.time ?? cfg.disturbance?.time ?? null;
  const disturbanceEnd = scenario.impulse ? disturbanceTime
    : (cfg.disturbance ? disturbanceTime+(cfg.disturbance.duration ?? 0) : null);

  for (let k = 0; k < steps; k++) {
    if (scenario.impulse && !impulseDone && sim.t >= scenario.impulse.time) {
      sim.injectImpulse(scenario.impulse.dv);
      impulseDone = true;
    }
    if (disturbanceTime !== null && yBeforeDisturbance === null && sim.t >= disturbanceTime) {
      yBeforeDisturbance = sim.currentDiag.centroid[1];
    }
    const diag = sim.step();
    const [cx, cy] = diag.centroid;
    const e = cx-x0;
    sumE2 += e*e; nE++; maxAbsE = Math.max(maxAbsE, Math.abs(e));
    let uu = 0;
    for (const u of sim.currentActuationOffsets) uu += u*u;
    for (const u of sim.currentCoreActuationOffsets) uu += u*u;
    effort += uu*cfg.dt;
    if (disturbanceEnd !== null && Number.isNaN(recovery) && sim.t >= disturbanceEnd &&
      yBeforeDisturbance !== null && cy >= yBeforeDisturbance+0.05) {
      recovery = sim.t-disturbanceTime;
    }
    if (k % sampleEvery === 0) {
      series.t.push(sim.t); series.x.push(cx-x0); series.y.push(cy-y0); series.e.push(e);
      series.u.push(Math.sqrt(uu));
      series.residual.push(diag.energyLedger ? Math.abs(diag.energyLedger.residual) : NaN);
      series.barError.push(diag.constraintError);
    }
    if (onChunk && (k+1) % chunkSteps === 0) await onChunk((k+1)/steps);
  }
  const diag = sim.currentDiag;
  // Progress along the level's travel direction (Level 2 rolls backward, toward -y).
  const dy = (cfg.actuationMode === 'roll_backward' ? -1 : 1)*(diag.centroid[1]-y0);
  const totalMass = sim.dynamicNodeMass*sim.rover.nOuter+sim.plantCoreMass;
  const g = Math.abs(cfg.gravity[2]);
  return {
    spec: { ...spec, duration },
    metrics: {
      progress: dy,
      speed: dy/duration,
      rmse: Math.sqrt(sumE2/Math.max(1, nE)),
      mae: maxAbsE,
      success: dy >= successDistance ? 1 : 0,
      recovery,
      effort,
      actuatorJ: sim.metrics.energyCost,
      cot: dy > 0.05 ? sim.metrics.energyCost/(totalMass*g*dy) : NaN,
      solveMsMean: sim.controllerTiming.count ? sim.controllerTiming.totalMs/sim.controllerTiming.count : NaN,
      solveMsMax: sim.controllerTiming.maxMs,
      controlHz: 1/cfg.controllerDt,
      maxResidual: diag.energyLedger?.maxAbsResidual ?? NaN,
      maxStepDrift: diag.energyLedger?.maxStepDrift ?? NaN,   // [J per step] before correction
      driftAbs: diag.energyLedger?.driftAbs ?? NaN,           // [J] total |drift| removed or left
      maxBarError: sim.metrics.maxConstraintError,
      assistShare: diag.energyLedger?.assistShare ?? NaN,
      tips: sim.sgt?.tipCount ?? sim.completedRolls
    },
    // Final work per ledger category [J] (cables, contact, corrector, ...),
    // so the numerical share of the energy budget can be reported.
    ledgerWork: sim.ledger?.work ? { ...sim.ledger.work } : null,
    series
  };
}

/** mean and sample standard deviation, ignoring non-finite values. */
export function meanStd(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) return { mean: NaN, std: NaN, n: 0 };
  const mean = finite.reduce((s, v) => s+v, 0)/finite.length;
  const variance = finite.length > 1
    ? finite.reduce((s, v) => s+(v-mean)**2, 0)/(finite.length-1) : 0;
  return { mean, std: Math.sqrt(variance), n: finite.length };
}

/** Aggregate runs by a key function into {key: {metric: {mean, std, n}}}. */
export function aggregate(runs, keyOf) {
  const groups = new Map();
  for (const run of runs) {
    const key = keyOf(run);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(run);
  }
  const out = {};
  for (const [key, list] of groups) {
    const metrics = {};
    for (const name of Object.keys(list[0].metrics)) metrics[name] = meanStd(list.map(r => r.metrics[name]));
    out[key] = { n: list.length, metrics, runs: list };
  }
  return out;
}

/**
 * booktabs LaTeX table. columns: [{ key, label, digits, better: 'max'|'min'|null, scale }]
 * Bold marks the best mean per column among rows flagged comparable.
 */
export function latexTable(rows, columns, { caption = '', label = '', note = '', wide = false, paper = false, groups = [], firstHeader = 'Method' } = {}) {
  if (paper) return latexTablePaper(rows, columns, { caption, label, note, wide, groups, firstHeader });
  // wide: two-column float (table*) for IEEE double-column layouts
  const fmt = (value, digits, sci = false) => {
    if (!Number.isFinite(value)) return '--';
    if (!sci) return value.toFixed(digits);
    const [mantissa, exponent] = value.toExponential(digits).split('e');
    return `$${mantissa}{\\times}10^{${Number(exponent)}}$`;
  };
  const best = {};
  for (const col of columns) {
    if (!col.better) continue;
    const candidates = rows.filter(r => !r.reference).map(r => r.cells[col.key]?.mean).filter(Number.isFinite);
    // No bold when every comparable row ties (nothing is "best").
    if (candidates.length > 1 && Math.max(...candidates)-Math.min(...candidates) > 1e-12) {
      best[col.key] = col.better === 'max' ? Math.max(...candidates) : Math.min(...candidates);
    }
  }
  const lines = [];
  const env = wide ? 'table*' : 'table';
  lines.push('% requires \\usepackage{booktabs,adjustbox}', `\\begin{${env}}[t]`, '\\centering', `\\caption{${caption}}`, `\\label{${label}}`,
    '\\setlength{\\tabcolsep}{2.5pt}', '\\footnotesize',
    // never wider than the column/page (requires \\usepackage{adjustbox})
    `\\begin{adjustbox}{max width=${wide ? '\\textwidth' : '\\columnwidth'}}`,
    `\\begin{tabular}{l${'c'.repeat(columns.length)}}`, '\\toprule',
    ['Method', ...columns.map(c => c.label)].join(' & ')+' \\\\', '\\midrule');
  for (const row of rows) {
    const cells = columns.map(col => {
      const cell = row.cells[col.key];
      if (!cell || !Number.isFinite(cell.mean)) return '--';
      const scale = col.scale ?? 1;
      let text = cell.std > 0 && col.showStd !== false && !col.sci
        ? `${fmt(cell.mean*scale, col.digits)}\\,$\\pm$\\,${fmt(cell.std*scale, col.digits)}`
        : fmt(cell.mean*scale, col.digits, col.sci);
      if (!row.reference && Number.isFinite(best[col.key]) && Math.abs(cell.mean-best[col.key]) <= 1e-12*Math.max(1, Math.abs(best[col.key]))) {
        text = `\\textbf{${text}}`;
      }
      return text;
    });
    // Escape bare % (a LaTeX comment would swallow the rest of the row)
    const label = String(row.label).replace(/(^|[^\\])%/g, '$1\\%');
    lines.push([label, ...cells].join(' & ')+' \\\\');
  }
  lines.push('\\bottomrule', '\\end{tabular}', '\\end{adjustbox}');
  if (note) lines.push(`\\\\[2pt]\\parbox{${wide ? '\\textwidth' : '\\columnwidth'}}{\\scriptsize ${note}}`);
  lines.push(`\\end{${env}}`);
  return lines.join('\n');
}


/**
 * Paper layout: two-line headers (metric + arrow / unit), standard deviation
 * in smaller type, optional group rows, best values in bold, table notes.
 */
function latexTablePaper(rows, columns, { caption, label, note, wide, groups, firstHeader }) {
  const fmt = (value, digits, sci = false) => {
    if (!Number.isFinite(value)) return '--';
    if (!sci) {
      const text = value.toFixed(digits);
      if (/^-0?\.?0*$/.test(text)) return text.slice(1);          // "-0.000" -> "0.000"
      return text.startsWith('-') ? `$-$${text.slice(1)}` : text;  // typeset minus sign
    }
    const [mantissa, exponent] = value.toExponential(digits).split('e');
    return `$${mantissa}{\\times}10^{${Number(exponent)}}$`;
  };
  const best = {};
  for (const col of columns) {
    if (!col.better) continue;
    const candidates = rows.filter(r => !r.reference).map(r => r.cells[col.key]?.mean).filter(Number.isFinite);
    if (candidates.length > 1 && Math.max(...candidates)-Math.min(...candidates) > 1e-12) {
      best[col.key] = col.better === 'max' ? Math.max(...candidates) : Math.min(...candidates);
    }
  }
  // split "name (unit) arrow" into "name arrow" / "(unit)"
  const head = col => {
    const m = /^(.*?)\s*\(([^()]*)\)(.*)$/.exec(col.label);
    return m ? [`${m[1]}${m[3]}`, `(${m[2]})`] : [col.label, ''];
  };
  const heads = columns.map(head);
  const env = wide ? 'table*' : 'table';
  const width = wide ? '\\textwidth' : '\\columnwidth';
  const lines = [`\\begin{${env}}[t]`, '\\centering', `\\caption{${caption}}`, `\\label{${label}}`,
    '\\setlength{\\tabcolsep}{3.2pt}', '\\renewcommand{\\arraystretch}{1.08}', '\\footnotesize',
    `\\begin{adjustbox}{max width=${width}}`,
    `\\begin{tabular}{@{}l${'c'.repeat(columns.length)}@{}}`, '\\toprule',
    [firstHeader, ...heads.map(h => h[0])].join(' & ')+' \\\\'];
  if (heads.some(h => h[1])) lines.push(['', ...heads.map(h => `{\\scriptsize ${h[1]}}`)].join(' & ')+' \\\\');
  lines.push('\\midrule');
  rows.forEach((row, index) => {
    const group = groups.find(g => g.at === index);
    if (group) {
      if (index > 0) lines.push('\\addlinespace[2pt]');
      lines.push(`\\multicolumn{${columns.length+1}}{@{}l}{\\textit{${group.label}}} \\\\`);
    }
    const cells = columns.map(col => {
      const cell = row.cells[col.key];
      if (!cell || !Number.isFinite(cell.mean)) return '--';
      const scale = col.scale ?? 1;
      let main = fmt(cell.mean*scale, col.digits, col.sci);
      const isBest = !row.reference && Number.isFinite(best[col.key]) && Math.abs(cell.mean-best[col.key]) <= 1e-12*Math.max(1, Math.abs(best[col.key]));
      if (isBest) main = `\\textbf{${main}}`;
      const std = cell.std > 0 && col.showStd !== false && !col.sci ? `\\,{\\scriptsize$\\pm$${fmt(cell.std*scale, col.digits)}}` : '';
      return main+std;
    });
    const name = String(row.label).replace(/(^|[^\\])%/g, '$1\\%');
    lines.push([(groups.length ? '\\quad ' : '')+name, ...cells].join(' & ')+' \\\\');
  });
  lines.push('\\bottomrule', '\\end{tabular}', '\\end{adjustbox}');
  if (note) lines.push(`\\par\\vspace{3pt}\\parbox{${width}}{\\scriptsize\\raggedright ${note}}`);
  lines.push(`\\end{${env}}`);
  return lines.join('\n');
}
