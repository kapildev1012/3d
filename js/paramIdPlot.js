/**
 * PARAMIDPLOT.JS - convergence chart of the particle-filter identification as an
 * SVG string (used live in the Tools tab and for the files written by
 * tools/run_param_identification.mjs). Same content as the figure of
 * final_codes/MainFile.m: estimate per parameter, +-2 sigma band, true value dashed.
 */
export const SERIES_COLORS = {
  dark: ['#F59E0B', '#10B981', '#3B82F6', '#EF4444', '#A78BFA'],
  light: ['#B45309', '#047857', '#1D4ED8', '#B91C1C', '#6D28D9']
};
const THEMES = {
  dark: { bg: 'none', text: '#C2C8D4', muted: '#8A93A6', grid: '#1E2230', axis: '#2A3042' },
  light: { bg: '#FFFFFF', text: '#1F2937', muted: '#4B5563', grid: '#E5E7EB', axis: '#9CA3AF' }
};
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const niceStep = range => {
  const raw = range/5, mag = 10**Math.floor(Math.log10(raw)), f = raw/mag;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10)*mag;
};

/**
 * mean, std: per step, arrays of P values; trueTheta: P values; lower/upper: prior bounds
 * (set the y range like ylim([theta_min-0.05, theta_max+0.05]) in MainFile.m);
 * labels: P legend texts; totalSteps: x range (keeps the axis fixed while a run streams in).
 */
export function convergenceSvg({ mean, std, trueTheta, lower, upper, labels = [], totalSteps = mean.length, title = '' },
  { width = 360, height = 220, theme = 'dark', legend = false } = {}) {
  const T = THEMES[theme] || THEMES.dark, colors = SERIES_COLORS[theme] || SERIES_COLORS.dark;
  const P = trueTheta.length;
  const legendRows = legend ? Math.ceil(P/2) : 0;
  const m = { l: 40, r: 10, t: title ? 24 : 10, b: 28+legendRows*16 };
  const w = width-m.l-m.r, h = height-m.t-m.b;
  const y0 = Math.min(...lower)-0.05, y1 = Math.max(...upper)+0.05;
  const X = k => m.l+(totalSteps > 1 ? (k-1)/(totalSteps-1) : 0)*w;
  const Y = v => m.t+(1-(v-y0)/(y1-y0))*h;
  const out = [];
  out.push(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" width="100%" role="img" aria-label="${esc(title || 'Parameter estimates')}" font-family="Inter, system-ui, sans-serif">`);
  if (T.bg !== 'none') out.push(`<rect width="${width}" height="${height}" fill="${T.bg}"/>`);
  out.push(`<defs><clipPath id="pid-clip"><rect x="${m.l}" y="${m.t}" width="${w}" height="${h}"/></clipPath></defs>`);
  if (title) out.push(`<text x="${m.l}" y="15" font-size="12" font-weight="600" fill="${T.text}">${esc(title)}</text>`);
  // grid and axes
  const dy = niceStep(y1-y0);
  for (let v = Math.ceil(y0/dy)*dy; v <= y1+1e-9; v += dy) {
    out.push(`<line x1="${m.l}" x2="${m.l+w}" y1="${Y(v).toFixed(1)}" y2="${Y(v).toFixed(1)}" stroke="${T.grid}"/>`);
    out.push(`<text x="${m.l-6}" y="${(Y(v)+3.5).toFixed(1)}" font-size="10" text-anchor="end" fill="${T.muted}">${v.toFixed(dy < 0.1 ? 2 : 1)}</text>`);
  }
  const dx = Math.max(1, niceStep(Math.max(totalSteps-1, 1)));
  for (let k = 0; k <= totalSteps; k += dx) {
    if (k < 1 && totalSteps > 1) { out.push(`<text x="${X(1).toFixed(1)}" y="${m.t+h+14}" font-size="10" text-anchor="middle" fill="${T.muted}">1</text>`); continue; }
    out.push(`<text x="${X(k).toFixed(1)}" y="${m.t+h+14}" font-size="10" text-anchor="middle" fill="${T.muted}">${k}</text>`);
  }
  out.push(`<rect x="${m.l}" y="${m.t}" width="${w}" height="${h}" fill="none" stroke="${T.axis}"/>`);
  out.push(`<text x="${m.l+w/2}" y="${m.t+h+25}" font-size="10" text-anchor="middle" fill="${T.muted}">load case k</text>`);
  // bands, estimates, true values
  out.push('<g clip-path="url(#pid-clip)">');
  for (let p = 0; p < P; p++) {
    const c = colors[p % colors.length];
    if (mean.length > 1) {
      const up = mean.map((mu, k) => `${X(k+1).toFixed(1)},${Y(mu[p]+2*std[k][p]).toFixed(1)}`);
      const dn = mean.map((mu, k) => `${X(k+1).toFixed(1)},${Y(mu[p]-2*std[k][p]).toFixed(1)}`).reverse();
      out.push(`<polygon points="${[...up, ...dn].join(' ')}" fill="${c}" fill-opacity="0.18" stroke="none"/>`);
    }
    out.push(`<line x1="${m.l}" x2="${m.l+w}" y1="${Y(trueTheta[p]).toFixed(1)}" y2="${Y(trueTheta[p]).toFixed(1)}" stroke="${c}" stroke-dasharray="4 3" stroke-opacity="0.8"/>`);
    if (mean.length) out.push(`<polyline points="${mean.map((mu, k) => `${X(k+1).toFixed(1)},${Y(mu[p]).toFixed(1)}`).join(' ')}" fill="none" stroke="${c}" stroke-width="1.6"/>`);
  }
  out.push('</g>');
  if (legend) {
    labels.forEach((label, p) => {
      const col = p % 2, row = Math.floor(p/2), lx = m.l+col*(w/2), ly = height-legendRows*16+row*16+4;
      out.push(`<line x1="${lx}" x2="${lx+16}" y1="${ly}" y2="${ly}" stroke="${colors[p % colors.length]}" stroke-width="2"/>`);
      out.push(`<text x="${lx+22}" y="${ly+3.5}" font-size="10.5" fill="${T.text}">${esc(label)} (true ${trueTheta[p]})</text>`);
    });
  }
  out.push('</svg>');
  return out.join('');
}
