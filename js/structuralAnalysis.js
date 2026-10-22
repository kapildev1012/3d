/**
 * STRUCTURALANALYSIS.JS - TsgFEM structural analysis of the simulated rover
 *
 * Builds the rover geometry from the simulator (SphericalRoverModel), applies
 * a material preset (materials.js) and runs the TsgFEM port (tsgfem.js):
 * form-finding, self-stress, prestress design, minimal-mass sizing, safety
 * margins of the installed parts and modal analysis.
 *
 * roverStructuralModel() also returns `input`, the same model in TsgFEM's
 * MATLAB conventions (1-based indices), used by tools/tsgfem_validate.mjs to
 * cross-check the port against the original toolbox.
 */
import { SimConfig, SphericalRoverModel } from './simEngine.js?v=20261021-ws1';
import { MATERIAL_LIB, analyzeRover } from './tsgfem.js?v=20261021-ws1';
import { MATERIALS, applyMaterial } from './materials.js?v=20261021-ws1';

const HUB_MASS = 0.2, PAYLOAD_MASS = 1.6;

export function roverStructuralModel({ material = 'steel', config = null } = {}) {
  const cfg = config ? { ...config } : new SimConfig({});
  applyMaterial(cfg, material);
  const preset = MATERIALS[material] || MATERIALS.lightweight;
  const rover = new SphericalRoverModel(cfg instanceof SimConfig ? cfg : Object.assign(new SimConfig({}), cfg));
  const nodes = rover.q0_outer.map(p => [...p]);
  const core = [0, 1, 2].map(a => nodes.reduce((s, p) => s+p[a], 0)/nodes.length);

  const steel = preset.strut && preset.rope;
  const barMaterial = steel ? MATERIAL_LIB.bars.Steel_Q345 : MATERIAL_LIB.bars.Aluminum;
  const stringMaterial = steel ? MATERIAL_LIB.strings.UHS_steel_string : MATERIAL_LIB.strings.Steel_string;
  const barLength = 0.8505;
  const installed = steel
    ? { barArea: preset.strut.area, barMass: preset.strutMass, bucklingLoad: preset.strut.bucklingLoad,
      cableBreaking: preset.rope.breakingForce, cableMassPerMetre: preset.rope.massPerMetre }
    : { // lightweight: 0.3 kg solid-equivalent aluminium bar, massless cables
      barArea: preset.strutMass/(barMaterial.rho*barLength), barMass: preset.strutMass,
      bucklingLoad: Math.PI**2*barMaterial.E*Math.PI*preset.strutRadius**4/4/(barLength**2),
      cableBreaking: NaN, cableMassPerMetre: 0 };
  const design = {
    cablePretension: cfg.pretensionS ?? 40, corePretension: cfg.pretensionCore ?? 50,
    barMaterial, stringMaterial, installed,
    cableK: preset.kS, coreK: preset.kCore, hubMass: preset.nodeMass ?? HUB_MASS, payloadMass: preset.coreMass ?? PAYLOAD_MASS,
    c_b: 0.5, c_s: 0.3, thick: 0.0015
  };
  const js = analyzeRover({ nodes, core, bars: rover.bars, outerStrings: rover.outerStrings }, design);
  js.material = { id: preset.id, label: preset.label, strutSpec: preset.strutSpec, cableSpec: preset.cableSpec,
    barLib: barMaterial, stringLib: stringMaterial };

  const f = js.fem;
  const one = idx => idx.map(e => e+1);
  const input = {
    source: 'six-bar tensegrity rover (js/structuralAnalysis.js)',
    nodes: f.nodes,
    bars: f.groups.bars.map(e => f.pairs[e].map(n => n+1)),
    outerStrings: f.groups.outer.map(e => f.pairs[e].map(n => n+1)),
    coreCables: f.groups.core.map(e => f.pairs[e].map(n => n+1)),
    groups: { bars: one(f.groups.bars), outer: one(f.groups.outer), core: one(f.groups.core) },
    cablePretension: design.cablePretension,
    barMaterial, stringMaterial, c_b: design.c_b, c_s: design.c_s, thick: design.thick,
    memberE: f.E, memberA: f.A, memberL0: f.l0, memberMass: f.memberMass, nodeMass: f.nodeMass
  };
  return { input, js, design };
}
