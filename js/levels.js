/**
 * LEVELS.JS - the 14 terrain levels as one definition used everywhere
 *
 * The level's terrain geometry (heightfield features, rocks, craters,
 * slopes, marshes, obstacle course) is generated deterministically by
 * TerrainModel.generateSurface() from (terrainLevel, groundRMS, seed);
 * contact parameters are per-level properties of that terrain (marsh
 * friction scaling, sand-patch drag, etc.). This module holds the names,
 * the baseline physics settings each level starts from, and
 * configureLevel(), which the browser app, the auto-benchmark and the
 * Node seed generator all use, so a level means the same thing in every view.
 */

export const LEVELS = Object.freeze([
  { id: 1, name: 'Rough Mars Sand Flats → Forward Rolling', short: 'Sand flats', roughness: 0.035,
    terrain: 'Low-amplitude sand ripples, moderate friction', group: 'basic' },
  { id: 2, name: 'Mars Small-Rock Field', short: 'Small rocks', roughness: 0.05,
    terrain: 'Small rock scatter (0.15 m crests) on the path', group: 'rocks' },
  { id: 3, name: 'Mars Boulder Field Rolling', short: 'Boulders', roughness: 0.07,
    terrain: 'Boulders 0.28 m high requiring compliant shell deformation', group: 'rocks' },
  { id: 4, name: 'Rocky Ridge Climb & Roll Over', short: 'Rocky ridge', roughness: 0.09,
    terrain: 'Stepped bedrock ridges 0.45–0.55 m', group: 'rocks' },
  { id: 5, name: 'Eroded Mars Crater Escape', short: 'Crater', roughness: 0.075,
    terrain: 'Concave crater basin, r = 2 m, 0.6 m deep', group: 'relief' },
  { id: 6, name: 'Sandy Mars Slope Uphill Roll (18°)', short: '18° slope', roughness: 0.06,
    terrain: 'Continuous 18° incline over 14 m', group: 'relief' },
  { id: 7, name: 'Irregular Mars Mountain Landscape', short: 'Mountains', roughness: 0.10,
    terrain: 'Multi-frequency rugged relief, rocks and a crater', group: 'relief' },
  { id: 8, name: 'Mixed-Size Boulder Scatter', short: 'Mixed boulders', roughness: 0.10,
    terrain: 'Randomized multi-scale obstacle gauntlet', group: 'rocks' },
  { id: 9, name: 'Mars Marsh Wetlands', short: 'Marsh', roughness: 0.06,
    terrain: 'Soft marsh basins with reduced traction', group: 'soft' },
  { id: 10, name: 'Learned Mars A-vs-B Mission', short: 'A-vs-B mission', roughness: 0.06,
    terrain: '50 m measured course, 10 rock formations, over vs bypass rules', group: 'mission' },
  { id: 11, name: 'Uneven Rubble Field', short: 'Rubble', roughness: 0.14,
    terrain: 'High-RMS sharp rubble testing cable tension peaks', group: 'rocks' },
  { id: 12, name: 'Mars Bog & Boulder Gauntlet', short: 'Bog + boulders', roughness: 0.11,
    terrain: 'Low-traction bog patches combined with boulders', group: 'soft' },
  { id: 13, name: 'Extreme Mars Composite', short: 'Extreme', roughness: 0.15,
    terrain: 'Slopes, craters, large boulders and rough RMS', group: 'relief' },
  { id: 14, name: '1km² Open-World Mars Expedition', short: '1 km² expedition', roughness: 0.18,
    terrain: '1000 m × 1000 m procedural sector, multi-waypoint route', group: 'mission' }
]);

export const LEVEL_COUNT = LEVELS.length;

export function levelName(id) {
  const level = LEVELS[id-1];
  return level ? `Level ${id}: ${level.name}` : `Level ${id}`;
}

/**
 * Apply a level's baseline to a SimConfig (in place) and return it.
 * Mirrors the historical behaviour of the app's loadExperiment():
 *  - Level 10 runs the A-vs-B course on the Level-7 relief with the
 *    Level-10 performance profile;
 *  - Level 14 spans the 1 km² field (start y = -450 m, goal y = +450 m);
 *  - every other level uses the standard performance profile.
 */
export function configureLevel(cfg, expId) {
  const level = LEVELS[expId-1] || LEVELS[0];
  cfg.experimentId = level.id;
  cfg.terrainLevel = level.id === 10 ? 7 : level.id;
  cfg.groundRMS = level.roughness;
  cfg.abCourseEnabled = level.id === 10;
  cfg.targetGoalY = level.id === 10 ? 60 : (level.id === 14 ? 450 : 25);
  cfg.targetDestination = [0, cfg.targetGoalY];
  if (level.id === 10) {
    cfg.applyLevel10PerformanceProfile();
  } else {
    cfg.applyStandardPerformanceProfile();
    if (level.id === 14) {
      cfg.T_end = 5000.0;
      cfg.courseStartY = -450.0;
      cfg.courseGoalY = 450.0;
      cfg.courseMaxY = 480.0;
      cfg.missionDeadlineSeconds = 5000.0;
    }
  }
  cfg.actuationMode = level.id === 2 ? 'roll_backward' : 'roll_forward';
  return cfg;
}
