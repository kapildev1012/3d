/**
 * MATERIALS.JS - strut and cable material presets
 *
 * Masses and stiffnesses are computed from geometry and material data, not
 * typed in, so changing a dimension updates everything consistently.
 *
 * Model conventions (both physics cores):
 *  - each strut is a rigid tube; its mass and transverse inertia enter the
 *    consistent bar mass matrix (DAE) or are split onto the end nodes (legacy);
 *  - each cable is a steel wire rope in series with a compliant spring (the
 *    actuated series-elastic element used on SUPERball-class rovers). The
 *    rope's axial stiffness EA/L is combined in series with the spring, and
 *    its mass is lumped half to each end (node or payload core).
 *  - a steel rope alone (~330 kN/m per cable) would make the shell
 *    effectively rigid and the 2 ms integrator unstable; the series spring
 *    is what makes cable-driven shape change possible.
 */

const STEEL_DENSITY = 7850;          // kg/m³
const OUTER_CABLE_LENGTH = 0.4923;   // m, mean rest length of the 24 outer cables
const CORE_CABLE_LENGTH = 0.4687;    // m, mean rest length of the 12 payload cables
const BAR_LENGTH = 0.8505;           // m
const OUTER_CABLES_PER_NODE = 4;
const HUB_MASS = 0.2;                // kg, end-cap hub + actuator per node
const PAYLOAD_MASS = 1.6;            // kg
const SPRING_K_OUTER = 1200;         // N/m series spring, outer cables
const SPRING_K_CORE = 1600;          // N/m series spring, payload cables

function tube({ outerDiameter, wall, E, yieldStress, density = STEEL_DENSITY, length = BAR_LENGTH }) {
  const ro = outerDiameter/2, ri = ro-wall;
  const area = Math.PI*(ro*ro-ri*ri);
  const inertia = Math.PI/4*(ro**4-ri**4);
  return {
    mass: density*area*length,
    // J = m/12 (3 r_eff² + l²) with r_eff² = ro² + ri² reproduces the tube's transverse inertia
    radiusEff: Math.sqrt(ro*ro+ri*ri),
    area, inertia,
    bucklingLoad: Math.PI**2*E*inertia/(length*length),
    yieldLoad: yieldStress*area
  };
}

function wireRope({ diameter, fillFactor, grade, spinningLoss, ropeModulus, layFactor = 1.08, density = STEEL_DENSITY }) {
  const metallicArea = fillFactor*Math.PI*diameter*diameter/4;
  return {
    metallicArea,
    massPerMetre: density*metallicArea*layFactor,
    breakingForce: metallicArea*grade*spinningLoss,
    axialStiffnessEA: ropeModulus*metallicArea
  };
}

const series = (k1, k2) => 1/(1/k1+1/k2);

export const MATERIALS = Object.freeze({
  steel: (() => {
    const strut = tube({ outerDiameter: 0.025, wall: 0.0015, E: 200e9, yieldStress: 355e6 });
    const rope = wireRope({ diameter: 0.002, fillFactor: 0.52, grade: 1960e6, spinningLoss: 0.83, ropeModulus: 100e9 });
    const outerRopeK = rope.axialStiffnessEA/OUTER_CABLE_LENGTH;
    const coreRopeK = rope.axialStiffnessEA/CORE_CABLE_LENGTH;
    const cableNodeShare = rope.massPerMetre*(OUTER_CABLES_PER_NODE*OUTER_CABLE_LENGTH/2+CORE_CABLE_LENGTH/2);
    const cableCoreShare = rope.massPerMetre*12*CORE_CABLE_LENGTH/2;
    return {
      id: 'steel',
      label: 'Steel struts + UHS steel cables',
      strutSpec: 'Steel tube Ø25 × 1.5 mm, S355 (E 200 GPa)',
      cableSpec: 'UHS steel wire rope Ø2 mm 7×19, 1960 MPa grade, in series with a compliant spring',
      strutMass: strut.mass,
      strutRadius: strut.radiusEff,
      nodeMass: HUB_MASS+cableNodeShare,
      coreMass: PAYLOAD_MASS+cableCoreShare,
      kS: series(SPRING_K_OUTER, outerRopeK),
      kCore: series(SPRING_K_CORE, coreRopeK),
      strut, rope,
      cableMass: rope.massPerMetre*(24*OUTER_CABLE_LENGTH+12*CORE_CABLE_LENGTH)
    };
  })(),
  lightweight: {
    id: 'lightweight',
    label: 'Lightweight (original: 0.3 kg bars, massless cables)',
    strutSpec: '0.3 kg rigid bar, Ø24 mm',
    cableSpec: 'Massless elastic cable',
    strutMass: 0.3, strutRadius: 0.012, nodeMass: HUB_MASS, coreMass: PAYLOAD_MASS,
    kS: SPRING_K_OUTER, kCore: SPRING_K_CORE, strut: null, rope: null, cableMass: 0
  }
});

export const DEFAULT_APP_MATERIAL = 'steel';

/** Apply a material preset to a SimConfig (in place). */
export function applyMaterial(cfg, id) {
  const material = MATERIALS[id] || MATERIALS.lightweight;
  Object.assign(cfg, {
    strutMass: material.strutMass, strutRadius: material.strutRadius,
    nodeMass: material.nodeMass, coreMass: material.coreMass,
    kS: material.kS, kCore: material.kCore, material: material.id
  });
  return cfg;
}

/** Totals for display: rover mass breakdown [kg]. */
export function massBreakdown(id) {
  const material = MATERIALS[id] || MATERIALS.lightweight;
  const struts = 6*material.strutMass;
  const cables = material.cableMass;
  const hubs = 12*HUB_MASS;
  const total = 12*material.nodeMass+struts+material.coreMass;
  return { struts, cables, hubs, payload: PAYLOAD_MASS, total };
}
