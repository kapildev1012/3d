/**
 * OBSTACLETRACKER.JS - over/around classification for the solid obstacle field,
 * one tracker per model (Model A Baseline, Model B Adaptive).
 *
 * While the rover centroid is inside an obstacle's along-course window the
 * tracker records (a) whether any contact point touched the obstacle's upper
 * surface (contact normal z > 0.55, reported by obstacleContact.js) and
 * (b) the smallest lateral distance of the centroid from the obstacle centre.
 * When the centroid leaves the window ahead of the obstacle it is classified:
 *   over    : the shell rode on top of it, or the centroid passed over its footprint
 *   around  : it was passed laterally
 * Bypass violation: a 'cross' obstacle (low, on the rolling path: crates, ledges,
 * rollers, ramps, low boulders) passed around by Model B. Gate pillars, walls and
 * tall offset obstacles are legally passed around.
 * summary() has the same fields as abExperiment.js ObstaclePassTracker so the HUD
 * cards and Dock tables show both trackers through one code path (mergeSummaries).
 */
export class FieldObstacleTracker {
  constructor(field = [], modelType = 'fixed') {
    this.field = field;
    this.modelType = modelType;
    this.records = new Map(field.map(o => [o.id, {
      id: o.id, template: o.template, kind: o.kind, role: o.role,
      status: 'pending', topContact: false, footprintPass: false, contacted: false, minLateral: Infinity
    }]));
  }

  update({ x, y, touches = null }) {
    for (const o of this.field) {
      const rec = this.records.get(o.id);
      if (rec.status === 'over' || rec.status === 'around') continue;
      const t = touches?.get(o.id);
      if (t) { rec.contacted = true; if (t.top) rec.topContact = true; }
      const halfWidth = o.kind === 'box' || o.kind === 'ramp' ? o.hx : o.kind === 'cylinder' ? o.halfLength : o.boundR * 0.7;
      if (y >= o.minY - 0.6 && y <= o.maxY + 0.6) {
        if (rec.status === 'pending') rec.status = 'engaged';
        const lateral = Math.abs(x - o.x);
        rec.minLateral = Math.min(rec.minLateral, lateral);
        if (lateral <= halfWidth && Math.abs(y - o.y) <= o.boundR) rec.footprintPass = true;
      }
      if (y > o.maxY + 0.6) rec.status = rec.topContact || rec.footprintPass ? 'over' : 'around';
    }
    return this.summary();
  }

  summary() {
    const records = [...this.records.values()];
    const over = records.filter(r => r.status === 'over').length;
    const around = records.filter(r => r.status === 'around').length;
    const crossAround = records.filter(r => r.status === 'around' && r.role === 'cross').length;
    return {
      total: records.length, over, around, retries: 0,
      pending: records.filter(r => r.status === 'pending' || r.status === 'engaged').length,
      contacted: records.filter(r => r.contacted).length,
      bypassViolations: this.modelType === 'adaptive' ? crossAround : 0,
      records: records.map(r => ({ ...r }))
    };
  }
}

/** Combine the Level-10/14 course tracker summary with the solid-field summary. */
export function mergeSummaries(course, field) {
  if (!course) return field || null;
  if (!field) return course;
  return {
    ...course,
    total: (course.total || 0) + field.total,
    over: course.over + field.over,
    around: course.around + field.around,
    pending: (course.pending || 0) + field.pending,
    bypassViolations: (course.bypassViolations || 0) + field.bypassViolations,
    field
  };
}
