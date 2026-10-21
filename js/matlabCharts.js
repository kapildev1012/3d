/**
 * MATLABCHARTS.JS - MATLAB-style figure theme for Chart.js
 *
 * Reproduces the look of MATLAB R2014b+ default axes so on-screen plots can
 * go straight into a paper:
 *   - white axes on a light-grey "figure" background, black box ("box on")
 *   - default MATLAB line color order (lines colormap)
 *   - light solid grid ("grid on"), inward ticks, Helvetica 10 pt
 *   - legend drawn INSIDE the axes with a white box (northeast by default)
 *   - true numeric x axes (no repeated category labels)
 *   - export: PNG at 300 dpi-equivalent scale, and CSV of the plotted data
 */

export const MATLAB_COLORS = Object.freeze([
  '#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F'
]);

export const MATLAB_FONT = 'Helvetica, Arial, "Liberation Sans", sans-serif';

/*
 * Screen theme. The workstation shows figures dark (matches the UI); paper
 * export (chartToSVG / exportChartIEEE) always redraws them white with the
 * MATLAB colour order, independent of the screen theme.
 */
export const SCREEN_COLORS = Object.freeze([
  '#60A5FA', '#F59E0B', '#34D399', '#C084FC', '#F472B6', '#22D3EE', '#F87171', '#A3A3A3'
]);
const THEMES = {
  light: { background: '#ffffff', axis: '#262626', tick: '#262626', grid: 'rgba(38, 38, 38, 0.15)',
    legendBg: 'rgba(255,255,255,0.94)', legendText: '#262626', font: MATLAB_FONT, colors: MATLAB_COLORS },
  dark: { background: '#11131A', axis: '#3A4258', tick: '#8A93A6', grid: '#1E2230',
    legendBg: 'rgba(12, 14, 20, 0.92)', legendText: '#C2C8D4',
    font: "'JetBrains Mono', ui-monospace, Menlo, monospace", colors: SCREEN_COLORS }
};
let THEME = THEMES.light;
const AXIS_COLOR = '#262626';
const themed = key => () => THEME[key];

/** Switch every chart (existing and future) between 'dark' and 'light'. */
export function setChartTheme(name, Chart = globalThis.Chart) {
  THEME = THEMES[name] || THEMES.light;
  if (!Chart) return;
  Chart.defaults.font.family = THEME.font;
  Chart.defaults.color = THEME.tick;
  Chart.defaults.plugins.tooltip.backgroundColor = THEME.legendBg;
  Chart.defaults.plugins.tooltip.titleColor = THEME.legendText;
  Chart.defaults.plugins.tooltip.bodyColor = THEME.legendText;
  Chart.defaults.plugins.tooltip.borderColor = THEME.axis;
  for (const chart of Object.values(Chart.instances || {})) {
    recolorDatasets(chart);
    chart.update('none');
  }
}

/** Datasets created with matlabLine() carry colorIndex; map it to the theme palette. */
export function recolorDatasets(chart, palette = THEME.colors) {
  for (const dataset of chart.data.datasets) {
    if (!Number.isInteger(dataset.colorIndex)) continue;
    const color = palette[dataset.colorIndex % palette.length];
    if (!dataset.fixedBorderColor) dataset.borderColor = color;
    if (!Array.isArray(dataset.backgroundColor)) dataset.backgroundColor = color;
  }
}

/** Axes background + box, drawn around the chart area. */
const matlabAxesPlugin = {
  id: 'matlabAxes',
  beforeDraw(chart) {
    const { ctx, chartArea } = chart;
    if (!chartArea) return;
    ctx.save();
    ctx.fillStyle = THEME.background;
    ctx.fillRect(0, 0, chart.width, chart.height);
    ctx.restore();
  },
  afterDraw(chart) {
    const { ctx, chartArea } = chart;
    if (!chartArea) return;
    ctx.save();
    ctx.strokeStyle = THEME.axis;
    ctx.lineWidth = 0.75;
    ctx.strokeRect(chartArea.left+0.5, chartArea.top+0.5,
      chartArea.right-chartArea.left-1, chartArea.bottom-chartArea.top-1);
    ctx.restore();
  }
};

/** MATLAB-style legend inside the axes (location: northeast|northwest|southeast|southwest). */
const matlabLegendPlugin = {
  id: 'matlabLegend',
  afterDatasetsDraw(chart, _args, options) {
    if (!options || options.display === false) return;
    const items = chart.data.datasets
      .map((dataset, index) => ({ dataset, index }))
      .filter(({ dataset, index }) => dataset.label && !dataset.hideInLegend && chart.isDatasetVisible(index));
    if (!items.length) return;
    const { ctx, chartArea } = chart;
    const fontSize = options.fontSize || 11;
    ctx.save();
    ctx.font = `${fontSize}px ${THEME.font}`;
    const lineLength = 22, pad = 6, gap = 5, rowHeight = fontSize+4;
    const width = Math.max(...items.map(({ dataset }) => ctx.measureText(dataset.label).width))+lineLength+gap+2*pad;
    const height = items.length*rowHeight+2*pad-4;
    const location = options.location || 'northeast';
    const x = location.includes('west') ? chartArea.left+8 : chartArea.right-width-8;
    const y = location.includes('south') ? chartArea.bottom-height-8 : chartArea.top+8;
    ctx.fillStyle = THEME.legendBg;
    ctx.fillRect(x, y, width, height);
    ctx.strokeStyle = THEME.axis;
    ctx.lineWidth = 0.6;
    ctx.strokeRect(x+0.5, y+0.5, width-1, height-1);
    items.forEach(({ dataset }, row) => {
      const cy = y+pad+row*rowHeight+fontSize/2;
      const color = dataset.borderColor || dataset.backgroundColor || THEME.tick;
      if (dataset.type === 'bar' || chart.config.type === 'bar') {
        ctx.fillStyle = Array.isArray(dataset.backgroundColor) ? THEME.colors[0] : dataset.backgroundColor;
        ctx.fillRect(x+pad, cy-4, lineLength, 8);
      } else if (dataset.showLine === false || (chart.config.type === 'scatter' && !dataset.showLine)) {
        ctx.fillStyle = dataset.backgroundColor || color;
        ctx.beginPath(); ctx.arc(x+pad+lineLength/2, cy, 3.5, 0, 2*Math.PI); ctx.fill();
      } else {
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.min(2.5, dataset.borderWidth || 1.5);
        ctx.setLineDash(dataset.borderDash || []);
        ctx.beginPath(); ctx.moveTo(x+pad, cy); ctx.lineTo(x+pad+lineLength, cy); ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.fillStyle = THEME.legendText;
      ctx.textBaseline = 'middle';
      ctx.fillText(dataset.label, x+pad+lineLength+gap, cy);
    });
    ctx.restore();
  }
};

let installed = false;
export function installMatlabTheme(Chart) {
  if (installed || !Chart) return;
  installed = true;
  Chart.register(matlabAxesPlugin, matlabLegendPlugin);
  Chart.defaults.font.family = MATLAB_FONT;
  Chart.defaults.font.size = 11;
  Chart.defaults.color = AXIS_COLOR;
  Chart.defaults.animation = false;
  Chart.defaults.plugins.legend.display = false;
  Chart.defaults.plugins.tooltip.backgroundColor = 'rgba(255,255,255,0.96)';
  Chart.defaults.plugins.tooltip.titleColor = AXIS_COLOR;
  Chart.defaults.plugins.tooltip.bodyColor = AXIS_COLOR;
  Chart.defaults.plugins.tooltip.borderColor = AXIS_COLOR;
  Chart.defaults.plugins.tooltip.borderWidth = 0.6;
  Chart.defaults.elements.line.borderWidth = 1.5;
  Chart.defaults.elements.point.radius = 0;
}

function axis({ title, type = 'linear', min, max, beginAtZero, ticks = {}, grid = true, position, reverse } = {}) {
  return {
    type,
    position,
    reverse,
    min,
    max,
    beginAtZero,
    title: title ? { display: true, text: title, color: themed('tick'), font: { size: 11 } } : { display: false },
    grid: { display: grid, color: themed('grid'), drawTicks: true, tickLength: -5, tickColor: themed('axis'), drawOnChartArea: grid },
    border: { display: false },
    ticks: { color: themed('tick'), padding: 8, maxTicksLimit: 8, ...ticks }
  };
}

/**
 * Options for a MATLAB-style line plot with numeric x.
 * @param {object} o { title, xLabel, yLabel, yLog, yMin, yMax, xMin, xMax,
 *                     legend: 'northeast'|false, y2: { label, min, max } }
 */
export function matlabLineOptions(o = {}) {
  const scales = {
    x: axis({ title: o.xLabel || 'Time (s)', min: o.xMin, max: o.xMax, ticks: o.xTicks }),
    y: axis({ title: o.yLabel, type: o.yLog ? 'logarithmic' : 'linear', min: o.yMin, max: o.yMax,
      beginAtZero: o.beginAtZero, ticks: o.yLog ? { callback: logTick } : o.yTicks })
  };
  if (o.y2) {
    scales.y2 = axis({ title: o.y2.label, min: o.y2.min, max: o.y2.max, position: 'right', grid: false });
  }
  return {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    normalized: true,
    parsing: o.parsing ?? true,
    layout: { padding: { top: 4, right: 10, bottom: 2, left: 2 } },
    interaction: { mode: 'nearest', intersect: false, axis: 'x' },
    scales,
    plugins: {
      legend: { display: false },
      matlabLegend: o.legend === false ? { display: false } : { location: o.legend || 'northeast' },
      title: o.title ? { display: true, text: o.title, color: themed('tick'), font: { size: 12, weight: 'bold' }, padding: { bottom: 6 } } : { display: false },
      decimation: { enabled: true, algorithm: 'min-max' }
    }
  };
}

function logTick(value) {
  const exponent = Math.log10(value);
  if (Math.abs(exponent-Math.round(exponent)) > 1e-9) return '';
  const superscripts = { '-': '⁻', 0: '⁰', 1: '¹', 2: '²', 3: '³', 4: '⁴', 5: '⁵', 6: '⁶', 7: '⁷', 8: '⁸', 9: '⁹' };
  return '10'+String(Math.round(exponent)).split('').map(char => superscripts[char] ?? char).join('');
}

/** Dataset helper: MATLAB color order by index. */
export function matlabLine(label, colorIndex, extras = {}) {
  return {
    label,
    data: [],
    colorIndex,
    borderColor: THEME.colors[colorIndex % THEME.colors.length],
    backgroundColor: THEME.colors[colorIndex % THEME.colors.length],
    borderWidth: 1.5,
    pointRadius: 0,
    tension: 0,
    fill: false,
    ...extras
  };
}

/** Zip time and value arrays into {x, y} points (skipping non-finite values). */
export function xy(times, values, transform = null) {
  const out = new Array(Math.min(times.length, values.length));
  let k = 0;
  for (let i = 0; i < out.length; i++) {
    const y = transform ? transform(values[i]) : values[i];
    if (Number.isFinite(y) && Number.isFinite(times[i])) out[k++] = { x: times[i], y };
  }
  out.length = k;
  return out;
}

/** Export a chart as a high-resolution PNG (scale x device pixels). */
export function exportChartPNG(chart, filename, scale = 4) {
  const previous = chart.options.devicePixelRatio;
  chart.options.devicePixelRatio = scale;
  chart.resize();
  const url = chart.toBase64Image('image/png', 1);
  chart.options.devicePixelRatio = previous;
  chart.resize();
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  link.click();
}

/** Export every dataset of a chart as tidy CSV (series,x,y). */
export function exportChartCSV(chart, filename) {
  const rows = ['series,x,y'];
  const labels = chart.data.labels || [];
  chart.data.datasets.forEach(dataset => {
    const name = `"${String(dataset.label || 'series').replace(/"/g, '""')}"`;
    dataset.data.forEach((point, index) => {
      if (point && typeof point === 'object') rows.push(`${name},${point.x},${point.y}`);
      else rows.push(`${name},${labels[index] ?? index},${point}`);
    });
  });
  const blob = new Blob([rows.join('\n')], { type: 'text/csv;charset=utf-8' });
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = filename;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}

/* ------------------------------------------------------------------------- *
 * IEEE publication export: vector SVG re-drawn from the chart's data and
 * scales, sized for single (3.5 in) or double (7.16 in) column, Times font,
 * 8 pt labels, 1 pt box, inward ticks, light dashed grid.
 * ------------------------------------------------------------------------- */
const IEEE_FONT = '"Times New Roman", Times, "Nimbus Roman", serif';
const escapeXml = text => String(text).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&apos;' }[c]));

export function chartToSVG(chart, options = {}) {
  // Paper figures always use the MATLAB colour order on white, whatever the
  // screen theme is; colours are swapped only for the duration of the build.
  const saved = chart.data.datasets.map(d => [d.borderColor, d.backgroundColor]);
  recolorDatasets(chart, MATLAB_COLORS);
  try {
    return buildSVG(chart, options);
  } finally {
    chart.data.datasets.forEach((d, i) => { [d.borderColor, d.backgroundColor] = saved[i]; });
  }
}

function buildSVG(chart, { widthIn = 3.5, aspect = 0.72, font = IEEE_FONT, fontPt = 8 } = {}) {
  const W = widthIn*72, H = W*aspect;                 // points
  const m = { left: fontPt*5.2, right: fontPt*1.2, top: fontPt*1.2, bottom: fontPt*4.2 };
  const xs = chart.scales.x, ys = chart.scales.y;
  const isCategory = xs.type === 'category';
  const px0 = m.left, px1 = W-m.right, py0 = H-m.bottom, py1 = m.top;
  const yLog = ys.type === 'logarithmic';
  const yMin = yLog ? Math.log10(ys.min) : ys.min, yMax = yLog ? Math.log10(ys.max) : ys.max;
  const labels = chart.data.labels || [];
  const xMin = isCategory ? -0.5 : xs.min, xMax = isCategory ? labels.length-0.5 : xs.max;
  const X = x => px0+(x-xMin)/((xMax-xMin) || 1)*(px1-px0);
  const Y = y => py0-((yLog ? Math.log10(Math.max(y, 1e-300)) : y)-yMin)/((yMax-yMin) || 1)*(py0-py1);
  const out = [];
  out.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${widthIn}in" height="${(widthIn*aspect).toFixed(3)}in" viewBox="0 0 ${W.toFixed(2)} ${H.toFixed(2)}" font-family='${font}' font-size="${fontPt}">`);
  out.push(`<rect x="0" y="0" width="${W}" height="${H}" fill="#ffffff"/>`);
  out.push(`<defs><clipPath id="axes"><rect x="${px0}" y="${py1}" width="${px1-px0}" height="${py0-py1}"/></clipPath></defs>`);
  // grid + ticks
  const tick = 3;
  const xTicks = isCategory ? labels.map((label, i) => ({ value: i, label })) : xs.ticks.map(t => ({ value: t.value, label: t.label }));
  const yTicks = ys.ticks.map(t => ({ value: t.value, label: yLog ? logTick(t.value) : t.label, exponent: yLog ? Math.round(Math.log10(t.value)) : null }))
    .filter(t => !yLog || t.label);
  for (const t of xTicks) {
    const x = X(t.value);
    if (x < px0-0.01 || x > px1+0.01) continue;
    if (!isCategory) out.push(`<line x1="${x}" y1="${py1}" x2="${x}" y2="${py0}" stroke="#D9D9D9" stroke-opacity="0.6" stroke-width="0.4" stroke-dasharray="1.5 1.5"/>`);
    out.push(`<line x1="${x}" y1="${py0}" x2="${x}" y2="${py0-tick}" stroke="#111111" stroke-width="0.5"/>`);
    out.push(`<line x1="${x}" y1="${py1}" x2="${x}" y2="${py1+tick}" stroke="#111111" stroke-width="0.5"/>`);
    const text = Array.isArray(t.label) ? t.label.join(' ') : t.label;
    if (isCategory) out.push(`<text x="${x}" y="${py0+fontPt*0.6}" transform="rotate(-90 ${x} ${py0+fontPt*0.6})" text-anchor="end" dominant-baseline="middle" font-size="${fontPt-2}">${escapeXml(text)}</text>`);
    else out.push(`<text x="${x}" y="${py0+fontPt*1.3}" text-anchor="middle">${escapeXml(text)}</text>`);
  }
  for (const t of yTicks) {
    const y = Y(t.value);
    if (y < py1-0.01 || y > py0+0.01) continue;
    out.push(`<line x1="${px0}" y1="${y}" x2="${px1}" y2="${y}" stroke="#D9D9D9" stroke-opacity="0.6" stroke-width="0.4" stroke-dasharray="1.5 1.5"/>`);
    out.push(`<line x1="${px0}" y1="${y}" x2="${px0+tick}" y2="${y}" stroke="#111111" stroke-width="0.5"/>`);
    out.push(`<line x1="${px1}" y1="${y}" x2="${px1-tick}" y2="${y}" stroke="#111111" stroke-width="0.5"/>`);
    if (t.exponent !== null && t.exponent !== undefined) {
      // 10^n with a real superscript (font-independent, unlike U+207B)
      out.push(`<text x="${px0-fontPt*0.5}" y="${y}" text-anchor="end" dominant-baseline="middle">10<tspan dy="${-fontPt*0.4}" font-size="${(fontPt*0.72).toFixed(2)}">${t.exponent < 0 ? '&#8722;' : ''}${Math.abs(t.exponent)}</tspan></text>`);
    } else {
      out.push(`<text x="${px0-fontPt*0.5}" y="${y}" text-anchor="end" dominant-baseline="middle">${escapeXml(t.label)}</text>`);
    }
  }
  // data
  out.push(`<g clip-path="url(#axes)">`);
  chart.data.datasets.forEach((dataset, index) => {
    if (!chart.isDatasetVisible(index)) return;
    const color = Array.isArray(dataset.borderColor) ? MATLAB_COLORS[0] : (dataset.borderColor || MATLAB_COLORS[index % 7]);
    const dash = dataset.borderDash?.length ? ` stroke-dasharray="${dataset.borderDash.map(d => d*0.6).join(' ')}"` : '';
    const type = dataset.type || chart.config.type;
    const points = dataset.data.map((p, i) => (p && typeof p === 'object') ? [p.x, p.y] : [isCategory ? i : Number(labels[i]), p])
      .filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y) && (!yLog || y > 0));
    if (type === 'bar') {
      const width = (X(1)-X(0))*0.8;
      dataset.data.forEach((value, i) => {
        if (!Number.isFinite(value)) return;
        const fill = Array.isArray(dataset.backgroundColor) ? dataset.backgroundColor[i] : dataset.backgroundColor;
        out.push(`<rect x="${X(i)-width/2}" y="${Math.min(Y(value), Y(0))}" width="${width}" height="${Math.abs(Y(0)-Y(value))}" fill="${fill || MATLAB_COLORS[0]}"/>`);
      });
      return;
    }
    if (dataset.fillBetween && points.length) {
      // shaded band: this dataset is the upper bound, data2 the lower bound
      const lower = (dataset.lowerData || []).map(p => [p.x, p.y]);
      const path = [...points.map(([x, y]) => `${X(x)},${Y(y)}`), ...lower.slice().reverse().map(([x, y]) => `${X(x)},${Y(y)}`)].join(' ');
      out.push(`<polygon points="${path}" fill="${dataset.backgroundColor || color}" fill-opacity="0.18" stroke="none"/>`);
      return;
    }
    const showLine = type !== 'scatter' || dataset.showLine;
    if (showLine && points.length > 1) {
      const stepped = dataset.stepped;
      let d = '';
      points.forEach(([x, y], i) => {
        if (i === 0) d += `M${X(x).toFixed(2)},${Y(y).toFixed(2)}`;
        else if (stepped) d += `H${X(x).toFixed(2)}V${Y(y).toFixed(2)}`;
        else d += `L${X(x).toFixed(2)},${Y(y).toFixed(2)}`;
      });
      out.push(`<path d="${d}" fill="none" stroke="${color}" stroke-width="${Math.min(1.2, (dataset.borderWidth || 1.5)*0.6)}"${dash} stroke-linejoin="round"/>`);
    }
    if (!showLine || dataset.pointRadius > 0) {
      const r = Math.max(1.4, (dataset.pointRadius || 3)*0.45);
      for (const [x, y] of points) out.push(`<circle cx="${X(x)}" cy="${Y(y)}" r="${r}" fill="${dataset.backgroundColor || color}"/>`);
    }
  });
  out.push(`</g>`);
  // box
  out.push(`<rect x="${px0}" y="${py1}" width="${px1-px0}" height="${py0-py1}" fill="none" stroke="#111111" stroke-width="0.6"/>`);
  // axis labels
  const xTitle = xs.options.title?.display ? xs.options.title.text : '';
  const yTitle = ys.options.title?.display ? ys.options.title.text : '';
  if (xTitle) out.push(`<text x="${(px0+px1)/2}" y="${H-fontPt*0.6}" text-anchor="middle">${escapeXml(xTitle)}</text>`);
  if (yTitle) out.push(`<text x="${fontPt*0.9}" y="${(py0+py1)/2}" text-anchor="middle" transform="rotate(-90 ${fontPt*0.9} ${(py0+py1)/2})">${escapeXml(yTitle)}</text>`);
  // legend (northeast unless configured)
  const legendOpt = chart.options.plugins?.matlabLegend;
  const items = chart.data.datasets.filter((d, i) => d.label && !d.hideInLegend && !d.fillBetween && chart.isDatasetVisible(i));
  if (items.length && legendOpt?.display !== false) {
    const rowH = fontPt*1.25, lineL = fontPt*2;
    const width = Math.max(...items.map(d => d.label.length))*fontPt*0.48+lineL+fontPt*1.6;
    const height = items.length*rowH+fontPt*0.6;
    const loc = legendOpt?.location || 'northeast';
    const lx = loc.includes('west') ? px0+4 : px1-width-4;
    const ly = loc.includes('south') ? py0-height-4 : py1+4;
    out.push(`<rect x="${lx}" y="${ly}" width="${width}" height="${height}" fill="#ffffff" stroke="#111111" stroke-width="0.4"/>`);
    items.forEach((d, row) => {
      const cy = ly+fontPt*0.3+rowH*(row+0.5);
      const color = Array.isArray(d.borderColor) ? MATLAB_COLORS[0] : (d.borderColor || d.backgroundColor);
      const dash = d.borderDash?.length ? ` stroke-dasharray="${d.borderDash.map(v => v*0.6).join(' ')}"` : '';
      if ((d.type || chart.config.type) === 'bar') out.push(`<rect x="${lx+fontPt*0.5}" y="${cy-2}" width="${lineL}" height="4" fill="${Array.isArray(d.backgroundColor) ? MATLAB_COLORS[0] : d.backgroundColor}"/>`);
      else out.push(`<line x1="${lx+fontPt*0.5}" y1="${cy}" x2="${lx+fontPt*0.5+lineL}" y2="${cy}" stroke="${color}" stroke-width="0.9"${dash}/>`);
      out.push(`<text x="${lx+fontPt+lineL}" y="${cy}" dominant-baseline="middle">${escapeXml(d.label)}</text>`);
    });
  }
  out.push('</svg>');
  return out.join('\n');
}

function downloadBlob(blob, filename) {
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = filename;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 2000);
}

/**
 * format: 'svg' | 'png' (600 dpi) | 'pdf' (vector, via the print dialog).
 * width: 'single' (3.5 in) | 'double' (7.16 in).
 */
export async function exportChartIEEE(chart, format, width, basename) {
  const widthIn = width === 'double' ? 7.16 : 3.5;
  const aspect = width === 'double' ? 0.38 : 0.72;
  const svg = chartToSVG(chart, { widthIn, aspect });
  const name = `${basename}_${width === 'double' ? '7p16in' : '3p5in'}`;
  if (format === 'svg') {
    downloadBlob(new Blob([svg], { type: 'image/svg+xml' }), `${name}.svg`);
    return;
  }
  if (format === 'png') {
    const dpi = 600;
    const image = new Image();
    const url = URL.createObjectURL(new Blob([svg], { type: 'image/svg+xml' }));
    await new Promise((resolve, reject) => { image.onload = resolve; image.onerror = reject; image.src = url; });
    const canvas = document.createElement('canvas');
    canvas.width = Math.round(widthIn*dpi);
    canvas.height = Math.round(widthIn*aspect*dpi);
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
    URL.revokeObjectURL(url);
    canvas.toBlob(blob => downloadBlob(blob, `${name}_600dpi.png`), 'image/png');
    return;
  }
  if (format === 'pdf') {
    // Vector PDF through the browser's print engine ("Save as PDF").
    const win = window.open('', '_blank');
    if (!win) throw new Error('Pop-up blocked: allow pop-ups to export PDF.');
    win.document.write(`<!doctype html><html><head><title>${escapeXml(name)}</title><style>
      @page { size: ${widthIn}in ${(widthIn*aspect).toFixed(3)}in; margin: 0; }
      html, body { margin: 0; padding: 0; background: #fff; }
      svg { display: block; width: ${widthIn}in; height: ${(widthIn*aspect).toFixed(3)}in; }
    </style></head><body>${svg}<script>window.onload = () => { window.focus(); window.print(); };<\/script></body></html>`);
    win.document.close();
  }
}
