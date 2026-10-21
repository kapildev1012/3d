#!/usr/bin/env python3
"""
Publication figures for the paper (IEEE sizes, MATLAB look).

    python3 tools/make_figures.py            # reads results/paper/figure_data.json

Writes results/paper/figures/<name>.{pdf,svg,png} (PNG at 600 dpi).
Style: MATLAB default axes (box on, inward ticks on all four sides, light
dashed grid), MATLAB `lines` colour order, Times-compatible serif at 8 pt.
Line styles: solid = proposed (SGT), dashed = baselines, dotted = reference,
so every figure stays legible in grayscale.
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATA = os.path.join(ROOT, "results", "paper", "figure_data.json")
OUT = os.path.join(ROOT, "results", "paper", "figures")
COLORS = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"]
SINGLE, DOUBLE = 3.5, 7.16
METHOD_ORDER = ["sgt", "heuristic", "lqr", "ilqr", "mpc", "legacyAssisted"]

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "TeX Gyre Termes", "Nimbus Roman", "Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8, "legend.fontsize": 7,
    "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.linewidth": 0.6, "axes.edgecolor": "#111111", "axes.prop_cycle": matplotlib.cycler(color=COLORS),
    "xtick.direction": "in", "ytick.direction": "in", "xtick.top": True, "ytick.right": True,
    "xtick.major.width": 0.5, "ytick.major.width": 0.5, "xtick.minor.visible": True, "ytick.minor.visible": True,
    "axes.grid": True, "grid.color": "#D9D9D9", "grid.alpha": 0.6, "grid.linestyle": "--", "grid.linewidth": 0.4,
    "lines.linewidth": 1.0, "legend.frameon": True, "legend.edgecolor": "#111111", "legend.fancybox": False,
    "legend.framealpha": 0.95, "savefig.dpi": 600, "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
    "pdf.fonttype": 42, "ps.fonttype": 42, "svg.fonttype": "none",
})


def style_of(key, meta):
    if meta.get("proposed"):
        return dict(linestyle="-", linewidth=1.3)
    if meta.get("reference"):
        return dict(linestyle=":", linewidth=1.1)
    return dict(linestyle="--", linewidth=0.9)


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    for ext in ("pdf", "svg", "png"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"))
    plt.close(fig)
    print(f"  wrote {name}.pdf/.svg/.png")


def band(ax, series, color, label, style, log=False):
    t, mean, std = series["t"], series["mean"], series["std"]
    lower = [max(m - s, 1e-16) if log else m - s for m, s in zip(mean, std)]
    upper = [m + s for m, s in zip(mean, std)]
    ax.fill_between(t, lower, upper, color=color, alpha=0.18, linewidth=0)
    ax.plot(t, [max(m, 1e-16) for m in mean] if log else mean, color=color, label=label, **style)


def fig_physics(data):
    phys = data.get("physics", {})
    if not phys:
        return
    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE, 2.0))
    labels = {"dae": "DAE + Alg. 1 (energy row)", "daeNoEnergy": "DAE + Alg. 1 (constraints only)", "legacy": "Penalty + SHAKE (legacy)"}
    styles = {"dae": dict(linestyle="-", linewidth=1.3), "daeNoEnergy": dict(linestyle="--", linewidth=0.9), "legacy": dict(linestyle=":", linewidth=1.1)}
    for i, key in enumerate(["dae", "daeNoEnergy", "legacy"]):
        if key not in phys:
            continue
        band(axes[0], phys[key]["residual"], COLORS[i], labels[key], styles[key], log=True)
        band(axes[1], phys[key]["barError"], COLORS[i], labels[key], styles[key], log=True)
    axes[0].set_yscale("log"); axes[1].set_yscale("log")
    axes[0].set_xlabel("Time (s)"); axes[1].set_xlabel("Time (s)")
    axes[0].set_ylabel(r"Energy residual $|\varepsilon(t)|$ (J)")
    axes[1].set_ylabel(r"Max. bar-length error (m)")
    axes[0].legend(loc="lower right")
    for ax, tag in zip(axes, "ab"):
        ax.set_title(f"({tag})", loc="left", fontsize=8)
    fig.tight_layout(w_pad=1.2)
    save(fig, "fig_physics_consistency")


def fig_nominal(data):
    nom = data.get("nominal", {})
    if not nom:
        return
    fig, axes = plt.subplots(1, 3, figsize=(DOUBLE, 2.0))
    for i, key in enumerate(METHOD_ORDER):
        if key not in nom:
            continue
        meta = nom[key]
        st = style_of(key, meta)
        label = meta["label"] + (" (ref.)" if meta.get("reference") else "")
        band(axes[0], meta["y"], COLORS[i], label, st)
        band(axes[1], meta["absE"], COLORS[i], label, st)
        band(axes[2], meta["u"], COLORS[i], label, st)
    axes[0].set_ylabel(r"Forward progress $\Delta y$ (m)")
    axes[1].set_ylabel(r"Cross-track error $|e(t)|$ (m)")
    axes[2].set_ylabel(r"Actuation $\|u(t)\|_2$ (m)")
    for ax, tag in zip(axes, "abc"):
        ax.set_xlabel("Time (s)")
        ax.set_title(f"({tag})", loc="left", fontsize=8)
    axes[0].legend(loc="upper left", ncol=1)
    fig.tight_layout(w_pad=1.0)
    save(fig, "fig_nominal_locomotion")


def fig_bars(rows, key_a, key_b, label_a, label_b, xlabel, name, width=SINGLE):
    """Horizontal grouped bars (mean +/- 1 sigma); labels read without rotation."""
    if not rows:
        return
    names = [r["label"] for r in rows][::-1]
    y = list(range(len(rows)))
    h = 0.38 if key_b else 0.6
    def vals(key):
        cells = [r["cells"][key] for r in rows][::-1]
        clean = lambda v: v if v == v else 0
        return [clean(c["mean"]) for c in cells], [clean(c["std"]) for c in cells]
    fig, ax = plt.subplots(figsize=(width, 0.2 * len(rows) + 0.55))
    m_a, s_a = vals(key_a)
    ax.barh([i + (h / 2 if key_b else 0) for i in y], m_a, h, xerr=s_a, color=COLORS[0], label=label_a,
            error_kw=dict(elinewidth=0.6, capsize=1.5), edgecolor="#111111", linewidth=0.4)
    if key_b:
        m_b, s_b = vals(key_b)
        ax.barh([i - h / 2 for i in y], m_b, h, xerr=s_b, color=COLORS[1], label=label_b, hatch="////",
                error_kw=dict(elinewidth=0.6, capsize=1.5), edgecolor="#111111", linewidth=0.4)
        ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, borderaxespad=0.3)
    ax.set_yticks(y)
    ax.set_yticklabels(names)
    ax.minorticks_off()
    ax.grid(axis="y", visible=False)
    ax.set_xlabel(xlabel)
    ax.axvline(0, color="#111111", linewidth=0.5)
    ax.set_xlim(left=min(0, min(m_a)))
    fig.tight_layout()
    save(fig, name)


def fig_path(data):
    run = data.get("perRun", {}).get("sgtSeed1")
    if not run:
        return
    fig, ax = plt.subplots(figsize=(SINGLE, 2.0))
    ax.plot(run["x"], run["y"], color=COLORS[0], linewidth=1.2, label="SGT centre of mass")
    ax.plot([0, 0], [min(run["y"]), max(run["y"])], color="#111111", linestyle=":", linewidth=0.9, label="Reference heading")
    ax.plot(run["x"][0], run["y"][0], "o", color=COLORS[4], markersize=4, label="Start")
    ax.plot(run["x"][-1], run["y"][-1], "s", color=COLORS[1], markersize=4, label="End (20 s)")
    ax.set_xlabel(r"Lateral position $x - x_0$ (m)")
    ax.set_ylabel(r"Forward position $y - y_0$ (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), borderaxespad=0)
    fig.tight_layout()
    save(fig, "fig_sgt_path")


def main():
    if not os.path.exists(DATA):
        sys.exit(f"missing {DATA}; run tools/summarize_experiments.mjs first")
    data = json.load(open(DATA))
    print("[figures] writing to", OUT)
    fig_physics(data)
    fig_nominal(data)
    fig_bars(data.get("stress", []), "sgtSpeed", "heuSpeed", "SGT (proposed)", "Support-face heuristic",
             r"Mean speed $\bar v$ (m/s)", "fig_robustness")
    fig_bars(data.get("ablation", []), "speed", None, "", None, r"Mean speed $\bar v$ (m/s)", "fig_ablation")
    fig_path(data)


if __name__ == "__main__":
    main()
