/**
 * CONFIGSCHEMA.JS - Runtime validation for simulation inputs and settings
 *
 * The app is plain ES modules (no TypeScript build), so inputs are validated
 * at runtime: every user-editable value and every key of
 * config/simulation.settings.json is checked against a declared type/range.
 * Out-of-range numbers are clamped; wrong types and unknown enum values fall
 * back to the default. Each correction is reported, never silently applied.
 */

export const EXPERIMENT_SCHEMA = Object.freeze({
  seed: { type: 'integer', min: 0, max: 999999, default: 42 },
  noiseModel: { type: 'enum', values: ['none', 'gaussian', 'laplace', 'outlier'], default: 'none' },
  noiseSigma: { type: 'number', min: 0, max: 0.05, default: 0.005, unit: 'm' },
  actuatorDelay: { type: 'number', min: 0, max: 0.5, default: 0, unit: 's' },
  plantStiffnessScale: { type: 'number', min: 0.5, max: 1.5, default: 1 },
  plantFrictionScale: { type: 'number', min: 0.2, max: 2, default: 1 },
  plantPayloadMassScale: { type: 'number', min: 0.5, max: 2, default: 1 },
  sgtStroke: { type: 'number', min: 0.05, max: 0.24, default: 0.15, unit: 'm' },
  sgtCoreStroke: { type: 'number', min: 0, max: 0.2, default: 0.15, unit: 'm' },
  sgtSpoolSpeed: { type: 'number', min: 0.02, max: 0.5, default: 0.2, unit: 'm/s' },
  sgtActiveCables: { type: 'integer', min: 1, max: 24, default: 24 }
});

export const MONITORING_SCHEMA = Object.freeze({
  enabled: { type: 'boolean', default: true },
  rawLogging: { type: 'boolean', default: true },
  maxRawSamples: { type: 'integer', min: 100, max: 1000000, default: 50000 },
  chartSamplePeriod: { type: 'number', min: 0.005, max: 5, default: 0.05 },
  chartWindowSeconds: { type: 'number', min: 1, max: 3600, default: 60 },
  chartUpdateSeconds: { type: 'number', min: 0, max: 10, default: 0 },
  goalThreshold: { type: 'number', min: 0.01, max: 10, default: 0.5 },
  contactForceThreshold: { type: 'number', min: 0, max: 1000, default: 0.25 },
  formationStableThreshold: { type: 'number', min: 0, max: 5, default: 0.08 },
  formationWarningThreshold: { type: 'number', min: 0, max: 5, default: 0.18 },
  formationCollapseThreshold: { type: 'number', min: 0, max: 5, default: 0.35 },
  rodLengthWarning: { type: 'number', min: 0, max: 1, default: 0.005 },
  penetrationWarning: { type: 'number', min: 0, max: 1, default: 0.002 },
  cableModerateStrainPercent: { type: 'number', min: 0, max: 500, default: 10 },
  cableHighStrainPercent: { type: 'number', min: 0, max: 500, default: 25 },
  cableMaximumStrainPercent: { type: 'number', min: 0, max: 500, default: 40 },
  cableOverloadForce: { type: 'number', min: 1, max: 100000, default: 720 },
  slackForceThreshold: { type: 'number', min: 0, max: 100, default: 0.25 },
  maximumNodeSeparationRatio: { type: 'number', min: 1, max: 20, default: 2.6 },
  collapseRadiusRatio: { type: 'number', min: 0, max: 1, default: 0.22 }
});

/** Validate one value; returns { value, issue } where issue is null if valid. */
export function validateValue(rule, raw, name) {
  if (rule.type === 'boolean') {
    if (typeof raw === 'boolean') return { value: raw, issue: null };
    return { value: rule.default, issue: `${name}: expected true/false, got ${JSON.stringify(raw)}; using ${rule.default}` };
  }
  if (rule.type === 'enum') {
    if (rule.values.includes(raw)) return { value: raw, issue: null };
    return { value: rule.default, issue: `${name}: "${raw}" is not one of ${rule.values.join(', ')}; using "${rule.default}"` };
  }
  const number = typeof raw === 'string' && raw.trim() !== '' ? Number(raw) : raw;
  if (typeof number !== 'number' || !Number.isFinite(number)) {
    return { value: rule.default, issue: `${name}: not a number (${JSON.stringify(raw)}); using ${rule.default}` };
  }
  let value = rule.type === 'integer' ? Math.round(number) : number;
  let issue = rule.type === 'integer' && value !== number ? `${name}: rounded to integer ${value}` : null;
  if (value < rule.min || value > rule.max) {
    const clamped = Math.min(rule.max, Math.max(rule.min, value));
    issue = `${name}: ${value}${rule.unit ? ' '+rule.unit : ''} outside [${rule.min}, ${rule.max}]; clamped to ${clamped}`;
    value = clamped;
  }
  return { value, issue };
}

/**
 * Validate an object against a schema.
 * @returns {{ values: object, issues: string[], unknown: string[] }}
 *   values contains every schema key (defaults filled in for missing ones
 *   only when fillDefaults is true).
 */
export function validateObject(schema, input = {}, { fillDefaults = true } = {}) {
  const values = {};
  const issues = [];
  const unknown = [];
  for (const [name, rule] of Object.entries(schema)) {
    if (input[name] === undefined) {
      if (fillDefaults) values[name] = rule.default;
      continue;
    }
    const result = validateValue(rule, input[name], name);
    values[name] = result.value;
    if (result.issue) issues.push(result.issue);
  }
  for (const name of Object.keys(input)) {
    if (!(name in schema) && !name.startsWith('$')) unknown.push(name);
  }
  return { values, issues, unknown };
}

/** Validate the central settings file payload ({ monitoring: {...} }). */
export function validateSettingsFile(payload) {
  if (!payload || typeof payload !== 'object') {
    return { settings: {}, issues: ['settings file is not a JSON object; using built-in defaults'] };
  }
  const result = validateObject(MONITORING_SCHEMA, payload.monitoring || {}, { fillDefaults: false });
  const issues = [...result.issues, ...result.unknown.map(name => `monitoring.${name}: unknown key ignored`)];
  return { settings: { monitoring: result.values }, issues };
}
