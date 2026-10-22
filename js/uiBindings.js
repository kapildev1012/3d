/**
 * UIBINDINGS.JS - workstation chrome that is independent of the physics
 *
 *  - numeric inputs next to every slider marked data-num (type or drag;
 *    both fire the slider's own input/change handlers);
 *  - dock tabs with a "Show all" option (remembered per browser);
 *  - --toolbar-h kept equal to the real toolbar height (sticky offsets);
 *  - URL query state: ?level=10&gait=roll_forward&controller=sgt_tipping
 *    &core=dae&act=pure&g=3.721&speed=2&cam=chase&tab=matrix
 */

const decimals = step => {
  const text = String(step);
  return text.includes('.') ? text.split('.')[1].length : 0;
};

export function bindNumericInputs(root = document) {
  root.querySelectorAll('input[type="range"][data-num]').forEach(range => {
    if (range.dataset.numBound) return;
    range.dataset.numBound = '1';
    const number = document.createElement('input');
    number.type = 'number';
    number.className = 'num-input';
    number.min = range.min;
    number.max = range.max;
    number.step = range.step || 'any';
    number.disabled = range.disabled;
    number.setAttribute('aria-label', `${range.labels?.[0]?.textContent || range.id} value`);
    const digits = decimals(range.step || 1);
    const show = () => { number.value = Number(range.value).toFixed(digits); number.disabled = range.disabled; };
    show();
    range.after(number);
    range.addEventListener('input', show);
    range.addEventListener('change', show);
    // Programmatic slider updates (presets, level loads) do not fire events.
    range.syncNumber = show;
    const commit = () => {
      const value = Number(number.value);
      if (!Number.isFinite(value)) { show(); return; }
      const clamped = Math.min(Number(range.max), Math.max(Number(range.min), value));
      range.value = String(clamped);
      range.dispatchEvent(new Event('input', { bubbles: true }));
      range.dispatchEvent(new Event('change', { bubbles: true }));
      show();
    };
    number.addEventListener('change', commit);
    number.addEventListener('keydown', event => { if (event.key === 'Enter') { event.preventDefault(); commit(); } });
  });
}

/** Refresh every numeric box from its slider (after code changed slider values). */
export function syncNumericInputs(root = document) {
  root.querySelectorAll('input[type="range"][data-num]').forEach(range => range.syncNumber?.());
}

export function bindDockTabs({ onShow } = {}) {
  const dock = document.querySelector('.dock');
  const tabs = [...document.querySelectorAll('.dock-tab')];
  const panels = [...document.querySelectorAll('[data-dock-panel]')];
  const showAll = document.getElementById('chk-dock-all');
  const show = name => {
    tabs.forEach(tab => {
      const active = tab.dataset.dock === name;
      tab.classList.toggle('is-active', active);
      tab.setAttribute('aria-selected', String(active));
      tab.tabIndex = active ? 0 : -1;
    });
    panels.forEach(panel => { panel.hidden = panel.dataset.dockPanel !== name; });
    try { localStorage.setItem('ws.dockTab', name); } catch { /* ignore */ }
    onShow?.(name);
  };
  tabs.forEach((tab, index) => {
    tab.addEventListener('click', () => show(tab.dataset.dock));
    tab.addEventListener('keydown', event => {
      if (event.key !== 'ArrowRight' && event.key !== 'ArrowLeft') return;
      const next = tabs[(index+(event.key === 'ArrowRight' ? 1 : tabs.length-1)) % tabs.length];
      next.focus();
      show(next.dataset.dock);
    });
  });
  showAll?.addEventListener('change', () => {
    dock.classList.toggle('show-all', showAll.checked);
    try { localStorage.setItem('ws.dockAll', showAll.checked ? '1' : '0'); } catch { /* ignore */ }
    onShow?.(showAll.checked ? 'all' : document.querySelector('.dock-tab.is-active')?.dataset.dock);
  });
  let initial = 'matrix';
  try {
    initial = localStorage.getItem('ws.dockTab') || initial;
    if (localStorage.getItem('ws.dockAll') === '1' && showAll) { showAll.checked = true; dock.classList.add('show-all'); }
  } catch { /* ignore */ }
  if (!tabs.some(tab => tab.dataset.dock === initial)) initial = 'matrix';
  show(initial);
  return show;
}

export function trackToolbarHeight() {
  const toolbar = document.getElementById('toolbar');
  if (!toolbar) return;
  const apply = () => document.documentElement.style.setProperty('--toolbar-h', `${toolbar.offsetHeight}px`);
  apply();
  if (typeof ResizeObserver !== 'undefined') new ResizeObserver(apply).observe(toolbar);
}

// ------------------------------------------------------------------ URL state

const URL_KEYS = ['level', 'gait', 'controller', 'core', 'act', 'g', 'speed', 'cam', 'preset', 'tab', 'mat', 'shield'];

export function readUrlState(search = window.location.search) {
  const params = new URLSearchParams(search);
  const state = {};
  for (const key of URL_KEYS) if (params.has(key)) state[key] = params.get(key);
  const level = Math.round(Number(state.level));
  state.level = level >= 1 && level <= 14 ? level : null;
  const g = Number(state.g);
  state.g = Number.isFinite(g) && g >= 0 && g <= 20 ? g : null;
  const speed = Number(state.speed);
  state.speed = Number.isFinite(speed) ? speed : null;
  return state;
}

export function writeUrlState(state) {
  const params = new URLSearchParams();
  for (const [key, value] of Object.entries(state)) {
    if (value !== null && value !== undefined && value !== '') params.set(key, String(value));
  }
  const url = `${window.location.pathname}?${params.toString()}${window.location.hash}`;
  if (url !== `${window.location.pathname}${window.location.search}${window.location.hash}`) {
    window.history.replaceState(null, '', url);
  }
  return `${window.location.origin}${url}`;
}
