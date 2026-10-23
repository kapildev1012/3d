#!/usr/bin/env python3
"""Teaser figure (Fig. 1): one SGT tip cycle with the suspension-integrity
filter, rendered from results/paper/teaser_frames.json, and the payload-cable
tension with and without the filter. Writes results/paper/figures/fig_teaser.pdf/png."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "results", "paper", "figures")
COLORS = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"]
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "TeX Gyre Termes", "Nimbus Roman", "Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix", "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.linewidth": 0.6,
    "savefig.dpi": 600, "savefig.bbox": "tight", "savefig.pad_inches": 0.02, "pdf.fonttype": 42,
})

d = json.load(open(os.path.join(ROOT, "results", "paper", "teaser_frames.json")))
F, bars, outer = d["frames"], d["bars"], d["outer"]

# one complete cycle after the first two tips: settle -> shift -> tip -> release
start = next(i for i, f in enumerate(F) if f["tips"] == 2 and f["state"] == "SGT_SHIFT")
settle = start-1
tip_i = next(i for i in range(start, len(F)) if F[i]["tips"] == 3)
shift_i = (start+tip_i)//2
pre_tip = tip_i-1
after = min(len(F)-1, tip_i+6)
picks = [(settle, "settle"), (shift_i, "shift"), (pre_tip, "tip"), (after, "release")]

def draw(ax, f, title):
    q = np.array(f["q"]); c = np.array(f["core"])
    centre = q.mean(axis=0)
    # ground patch
    gx, gy = np.meshgrid(np.linspace(centre[0]-0.52, centre[0]+0.52, 2), np.linspace(centre[1]-0.52, centre[1]+0.52, 2))
    ax.plot_surface(gx, gy, np.zeros_like(gx), color="#E8E2D6", alpha=0.7, linewidth=0, shade=False)
    if f["face"]:
        tri = q[f["face"]].copy(); tri[:, 2] = 0.002
        ax.add_collection3d(Poly3DCollection([tri], facecolor=COLORS[4], alpha=0.35, edgecolor=COLORS[4], linewidth=0.6))
    umax = 0.15
    for s, (i, j) in enumerate(outer):
        u = f["cable"][s]
        col = COLORS[1] if u > 0.02*umax else ("#9A9A9A" if u > -0.02*umax else COLORS[5])
        ax.plot(*zip(q[i], q[j]), color=col, linewidth=0.9 if u > 0.02*umax else 0.5)
    for i in range(12):
        slack = f["coreT"][i] < 1.0
        ax.plot(*zip(q[i], c), color=COLORS[6] if slack else COLORS[0], linewidth=0.5, linestyle=":" if slack else "-")
    for i, j in bars:
        ax.plot(*zip(q[i], q[j]), color="#202020", linewidth=2.2, solid_capstyle="round")
    ax.scatter(*c, s=55, color=COLORS[2], edgecolor="#5a4500", linewidth=0.5, depthshade=False)
    ax.set_xlim(centre[0]-0.52, centre[0]+0.52); ax.set_ylim(centre[1]-0.52, centre[1]+0.52); ax.set_zlim(0.0, 0.98)
    ax.view_init(elev=14, azim=-8)
    ax.set_box_aspect((1, 1, 0.94), zoom=1.12)
    ax.set_axis_off()
    ax.text2D(0.5, 0.97, title, transform=ax.transAxes, ha="center", va="top", fontsize=8)

fig = plt.figure(figsize=(7.16, 1.75))
labels = "abcd"
for k, (idx, name) in enumerate(picks):
    ax = fig.add_axes([-0.012+k*0.17, -0.05, 0.19, 1.06], projection="3d")
    f = F[idx]
    draw(ax, f, f"({labels[k]}) {name}, $t$ = {f['t']:.1f} s")

ax = fig.add_axes([0.765, 0.24, 0.23, 0.62])
on, off = d["tensionOn"], d["tensionOff"]
ax.plot([p["t"] for p in off], [p["mean"] for p in off], color=COLORS[6], linewidth=0.8, label="SGT")
ax.plot([p["t"] for p in on], [p["mean"] for p in on], color=COLORS[0], linewidth=0.8, label="SGT + filter")
ax.axhline(10, color="#555555", linewidth=0.5, linestyle="--")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Mean payload\ncable tension (N)")
ax.set_xlim(0, 20); ax.set_ylim(0, 42)
ax.grid(True, color="#D9D9D9", linestyle="--", linewidth=0.4)
ax.legend(loc="upper center", ncol=2, frameon=True, fancybox=False, edgecolor="#111111", handlelength=1.2, columnspacing=0.8, borderpad=0.3)
ax.set_title("(e) suspension integrity", fontsize=8)

os.makedirs(OUT, exist_ok=True)
for ext in ("pdf", "png"):
    fig.savefig(os.path.join(OUT, f"fig_teaser.{ext}"))
print("picks", [(F[i]["t"], F[i]["state"]) for i, _ in picks])
