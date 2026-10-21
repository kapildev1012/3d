/**
 * FIGURES.JS - figure-card definitions (Figs 1–13 + paper benchmark plots)
 *
 * Cards are generated before the charts are created, so every figure has the
 * same toolbar: category tag, subtitle and an Export menu with
 *   - Paper export (white background, IEEE 3.5 in / 7.16 in, PDF/SVG/600 dpi PNG)
 *   - Screen PNG (dark, as displayed) and CSV of the plotted data.
 */

export const FIGURE_GROUPS = Object.freeze([
  { id: 'all', label: 'All' },
  { id: 'kinematics', label: 'Kinematics & Path' },
  { id: 'cables', label: 'Cable Tension & Force Law' },
  { id: 'payload', label: 'Payload & Formation' },
  { id: 'energy', label: 'Energy & Constraints' }
]);

export const FIGURES = Object.freeze([
  { n: 1, key: 'energy-ledger', canvas: 'chart-energy-ledger', group: 'energy', title: 'Work ledger: cumulative work by mechanism', sub: 'Model B · W(t) [J]' },
  { n: 2, key: 'energy-residual', canvas: 'chart-energy-residual', group: 'energy', title: 'Energy-balance residual (log)', sub: '|E − E₀ − ΣW| [J]' },
  { n: 3, key: 'control', canvas: 'chart-control', group: 'energy', title: 'Bar-length constraint violation (log)', sub: 'max |‖b‖ − l| [m]' },
  { n: 4, key: 'goal-error', canvas: 'chart-goal-error', group: 'kinematics', title: 'Goal tracking error', sub: '‖Goal − COM(t)‖ [m]' },
  { n: 5, key: 'com-path', canvas: 'chart-com-path', group: 'kinematics', title: 'Centre-of-mass path, top view', sub: 'x–y [m]' },
  { n: 6, key: 'velocity', canvas: 'chart-velocity', group: 'kinematics', title: 'Speed and shell deformation', sub: '1 s moving mean' },
  { n: 7, key: 'gforce', canvas: 'chart-gforce', group: 'payload', title: 'Payload core proper acceleration', sub: '[g]' },
  { n: 8, key: 'formation-error', canvas: 'chart-formation-error', group: 'payload', title: 'Formation error', sub: 'rigid-aligned RMS [m]' },
  { n: 9, key: 'contact-count', canvas: 'chart-contact-count', group: 'kinematics', title: 'Active terrain contacts', sub: 'nodes + rods' },
  { n: 10, key: 'cable-delta', canvas: 'chart-cable-delta', group: 'cables', title: 'Cable length change', sub: 'ΔL [m]', subId: 'chart-cable-delta-label' },
  { n: 11, key: 'cable-force', canvas: 'chart-cable-force', group: 'cables', title: 'Cable tension', sub: 'tension [N]', subId: 'chart-cable-force-label' },
  { n: 12, key: 'cable-bars', canvas: 'chart-cable-bars', group: 'cables', title: 'Tension distribution, all 24 cables', sub: 'instantaneous [N]' },
  { n: 13, key: 'force-law', canvas: 'chart-force-law', group: 'cables', title: 'Linear vs relaxing cable force law', sub: 'Hermite relaxation' }
]);

export const BENCH_FIGURES = Object.freeze([
  { key: 'bench-progress', canvas: 'chart-bench-progress', title: 'Benchmark: forward progress Δy(t)', sub: 'mean ± 1σ' },
  { key: 'bench-error', canvas: 'chart-bench-error', title: 'Benchmark: cross-track error |e(t)|', sub: 'mean ± 1σ' },
  { key: 'bench-effort', canvas: 'chart-bench-effort', title: 'Benchmark: actuation ‖u(t)‖', sub: 'mean ± 1σ' }
]);

const groupLabel = id => FIGURE_GROUPS.find(group => group.id === id)?.label || '';

function exportMenu(key) {
  const item = (format, width, label) =>
    `<button type="button" role="menuitem" data-ieee="${key}" data-format="${format}" data-width="${width}">${label}</button>`;
  return `<details class="fig-menu">
    <summary class="btn btn-xs" title="Export this figure">Export</summary>
    <div class="fig-menu-list" role="menu">
      <span class="fig-menu-head">Paper export · white BG · IEEE 3.5 in</span>
      ${item('pdf', 'single', 'PDF (vector)')}${item('svg', 'single', 'SVG (vector)')}${item('png', 'single', 'PNG · 600 dpi')}
      <span class="fig-menu-head">Paper export · white BG · IEEE 7.16 in</span>
      ${item('pdf', 'double', 'PDF (vector)')}${item('svg', 'double', 'SVG (vector)')}${item('png', 'double', 'PNG · 600 dpi')}
      <span class="fig-menu-head">Screen &amp; data</span>
      <button type="button" role="menuitem" data-export-png="${key}">Screen PNG (dark, 4×)</button>
      <button type="button" role="menuitem" data-export-csv="${key}">CSV of plotted data</button>
    </div>
  </details>`;
}

export function figureCard(figure) {
  const number = figure.n ? `Fig. ${figure.n}` : '';
  return `<figure class="fig" data-fig-group="${figure.group || 'bench'}">
    <figcaption class="fig-bar">
      <span class="fig-num">${number}</span>
      <span class="fig-title">${figure.title}</span>
      <span class="fig-sub"${figure.subId ? ` id="${figure.subId}"` : ''}>${figure.sub || ''}</span>
      ${figure.group ? `<span class="fig-tag">${groupLabel(figure.group)}</span>` : ''}
      ${exportMenu(figure.key)}
    </figcaption>
    <div class="fig-axes"><canvas id="${figure.canvas}"></canvas></div>
  </figure>`;
}

/** Fill the figure grid and filter bar; returns the number of cards. */
export function renderFigures(document) {
  const grid = document.getElementById('figure-grid');
  if (grid) grid.innerHTML = FIGURES.map(figureCard).join('');
  const filter = document.getElementById('figure-filter');
  if (filter) {
    filter.innerHTML = FIGURE_GROUPS.map((group, index) =>
      `<button type="button" class="seg-btn fig-filter-btn${index === 0 ? ' is-active' : ''}" data-fig-filter="${group.id}">${group.label}</button>`).join('');
  }
  const bench = document.getElementById('bench-figure-grid');
  if (bench) bench.innerHTML = BENCH_FIGURES.map(figureCard).join('');
  return FIGURES.length;
}
