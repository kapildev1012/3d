# PARAMETERS.md

Every numerical value used by the implementation, with its provenance.
Source tags: **[T1]** = Table I of the paper, **[TXT]** = paper body text, **[EQ]** = derived
from a paper equation, **[FIG]** = digitised from a paper figure (graph-derived estimate),
**[STD]** = standard numerical/implementation choice (see `ASSUMPTIONS.md`).

The paper uses **non-dimensional, self-consistent units** throughout: length `[L]`, time `[T]`,
mass `[M]`. No SI unit is ever stated (Figs. 5–6 axes are labelled only "X"/"Y"). All angles in
the implementation are radians; no rotations/quaternions occur (planar point masses).

## 1. Simulation / integration

| Name | Meaning | Value | Unit | Source | Fixed? |
|---|---|---|---|---|---|
| `dt` | integration & control step | 0.05 | [T] | [T1] | fixed |
| `T_end` | simulation duration | 40 | [T] | [TXT] Figs. 5–6 last panel | fixed |
| `snapshots` | figure snapshot times | 0, 15, 30, 40 | [T] | [TXT] Figs. 5–6 captions | fixed |
| `integrator` | fixed-step scheme | `symplectic_euler` | – | [STD] A6 | option (`rk4`) |
| `seed` | RNG seed (noise) | 20260707 | – | [STD] A7 (unspecified in paper) | option |
| `d` | spatial dimension | 2 | – | [TXT] "`d = 2` dimensional" | fixed |

## 2. Agent dynamics — Eq. (9)

| Name | Meaning | Value | Unit | Source | Fixed? |
|---|---|---|---|---|---|
| `m` | agent mass | 1 | [M] | [T1] | fixed |
| `c` | linear damping coefficient | 1.5 | [M]/[T] | [T1] | fixed |
| `u_max` | input saturation magnitude | 10 | [M][L]/[T]² | [T1] | fixed |
| `u_nav` | navigation input `u_i^n` | [0, 3] | [M][L]/[T]² | [T1] + [TXT] | fixed |
| `v_inf` | resulting terminal speed `‖u_nav‖/c` | 2 | [L]/[T] | [EQ] derived | derived |
| `tau` | velocity time constant `m/c` | 0.6667 | [T] | [EQ] derived | derived |

## 3. Tensegrity member force law — Eqs. (10), (11), (21)

| Name | Meaning | Value | Unit | Source | Fixed? |
|---|---|---|---|---|---|
| `kS` | string gain | 0.0341 | [M][L]^(1−αS)/[T]² | [T1] (= Eq. (21), rounded) | derived from Eq. (21) |
| `alphaS` | string exponent | 2 | – | [T1] | fixed |
| `kB` | bar gain | −50 | [M][L]^(1−αB)/[T]² | [T1] | fixed |
| `alphaB` | bar exponent | −0.5 | – | [T1] | fixed |
| `lS` | nominal string (tile side) length | 15 | [L] | [T1] | fixed |
| `kB/kS` from Eq. (21) | `−lS^(αS−αB)·√2^(1−αB)` | −1465.53 | – | [EQ] | derived |
| `kB/kS` from Table I | `−50/0.0341` | −1466.28 | – | [T1] | consistency check: 5.1e-4 rel. err. |
| bar nominal length | `√2·lS` | 21.2132 | [L] | [EQ] | derived |
| string force at `lS` | `kS·lS^αS` | 7.6725 | [M][L]/[T]² | [EQ] | derived |
| bar force at `√2 lS` | `kB·(√2 lS)^αB` | −10.8547 | [M][L]/[T]² | [EQ] | derived |
| string `ω` at `lS` | `kS·lS^(αS−1)` | 0.5115 | [M]/[T]² | [EQ] | derived |
| bar `ω` at `√2 lS` | `kB·(√2 lS)^(αB−1)` | −0.5117 | [M]/[T]² | [EQ] | derived |

Lemma 2 check: `kS(αS−1) = 0.0341 ≥ 0` ✔, `kB(αB−1) = 75 ≥ 0` ✔.

## 4. String relaxation — Eqs. (19), (20)

| Name | Meaning | Value | Unit | Source | Fixed? |
|---|---|---|---|---|---|
| `z1` | start of relaxation spline | 15.5 | [L] | [T1] ("slightly greater than `lS`") | fixed |
| `z2` | end of relaxation spline | 50 | [L] | [T1] | fixed |
| `beta` | relaxed (constant) string force | 8 | [M][L]/[T]² | [T1] | fixed |
| `f(z1)` | spline start value `kS z1^αS` | 8.1922 | [M][L]/[T]² | [EQ] | derived |
| `f'(z1)` | spline start slope `αS kS z1^(αS−1)` | 1.0571 | [M]/[T]² | [EQ] | derived |
| `f'(z2)` | spline end slope | 0 | [M]/[T]² | [EQ] Eq. (20) has no `m1` term | derived |
| spline peak | max of `h` | 13.512 at `ℓ = 25.87` | – | [EQ] | derived (matches Fig. 3: ≈13.5 at ≈26) |
| `relaxation` | enable Eq. (19) | false (Fig. 5) / true (Fig. 6) | – | [TXT] | experiment switch |

## 5. Obstacle avoidance — Eqs. (17), (18)

| Name | Meaning | Value | Unit | Source | Fixed? |
|---|---|---|---|---|---|
| `ry` | sensing / avoidance radius | 8 | [L] | [T1] | fixed |
| `ka` | avoidance gain | 20 | [M][L]^(1+γ)/[T]² | [T1] | fixed |
| `gamma` | avoidance exponent | 0.4 | – | [T1] | fixed |
| `avoid_sign` | sign correction of Eq. (18) | +1 (repulsive) | – | [STD] A1 | option (−1 = literal) |
| `interagent_avoidance` | agents also act as obstacles | true | – | [TXT] "When obstacle `j` is another agent…" | option |
| `u^c` at `r=1` | `ka(r^−γ − ry^−γ)` | 11.29 | [M][L]/[T]² | [EQ] | derived |
| `u^c` at `r=4` | – | 2.79 | [M][L]/[T]² | [EQ] | derived |
| `u^c` at `r=ry` | – | 0 | – | [EQ] | derived |

## 6. Measurement noise

| Name | Meaning | Value | Unit | Source | Fixed? |
|---|---|---|---|---|---|
| `sigma` | std. dev. of Gaussian measurement noise | 0.5 | [L] | [T1] + [TXT] "imposed on all measurements" | fixed |
| `noise_mode` | what is corrupted | each measured relative-position **vector component**, independently per agent, per member/obstacle, per step | – | [STD] A5 | option |

## 7. Formation geometry

| Name | Meaning | Value | Source |
|---|---|---|---|
| `tiles` | square tiles [rows, cols] | [2, 3] | [TXT] "six interconnected squares (two rows and three columns)" |
| `n` | number of agents | 12 | [TXT] "tiled formation consisting of 12 agents" |
| node grid | 4 columns × 3 rows, spacing `lS` | x ∈ {−22.5,−7.5,7.5,22.5}, y ∈ {−15,0,15} | [FIG] Fig. 5(a)/6(a), digitised (±0.15) |
| `S` | number of string members | 24 (= 6 tiles × 4) | [EQ]/[FIG] §6.1 of PAPER_ANALYSIS |
| `B` | number of bar members | 12 (= 6 tiles × 2) | [EQ]/[FIG] |
| string multiplicity | interior edges 2, boundary edges 1 | – | [EQ] statics + [FIG] solid vs dashed lines |
| `q(0)` | initial positions | `= q*` (nominal grid) | [FIG] Fig. 5(a)/6(a) |
| `v(0)` | initial velocities | 0 | [STD] A8 (unspecified; figure shows undeformed grid at `t=0`) |

## 8. Obstacles (graph-derived, [FIG])

| # | centre x | centre y | radius | digitised radius (8 panels) |
|---|---|---|---|---|
| 1 | 0 | 90 | 12 | 11.79 – 12.19 (mean 12.07) |
| 2 | 30 | 50 | 6.5 | 6.62 – 6.70 (mean 6.65) |
| 3 | −30 | 30 | 6.5 | 6.59 – 6.63 (mean 6.61) |

Digitised radii are ~0.1 larger than the adopted values because the mask threshold includes the
anti-aliased rim (≈0.5 px ≈ 0.1 length units). Adopted values are the plausible round numbers
inside the measurement band; `results/data/obstacle_sensitivity.mat` quantifies the effect of
using the raw digitised values instead (validation test T13).

## 9. Plotting / figure-reproduction parameters ([FIG])

| Name | Value | Source |
|---|---|---|
| Fig. 5/6 axis limits | x ∈ [−50, 50], y ∈ [−20, 140] | digitised tick range |
| Fig. 3 axis limits | x ∈ [0, 60], y ∈ [0, 25] | read from figure |
| Fig. 4 axis limits | x ∈ [0, 60], y ∈ [0, 30] | read from figure |
| string colour/style | red, dashed | Fig. 2 caption + Figs. 5–6 |
| bar colour/style | blue, dashed | Fig. 2 caption + Figs. 5–6 |
| agent marker | black filled circle | Figs. 5–6 |
| obstacle | solid red disc | [TXT] "Solid red circles denote obstacles" |
| detection indicator | translucent red disc of radius `r_y` on the agent | [TXT] "translucent red circle … within its avoidance radius `r_y`" |

## 10. Explicitly `UNSPECIFIED_IN_PAPER`

| Item | Resolution |
|---|---|
| Random seed | chosen, documented (A7); all figures/validation are reproducible |
| Numerical integrator | `dt` is given, scheme is not → semi-implicit (symplectic) Euler at the given `dt`, plus an RK4 cross-check and a `dt`-refinement study (A6) |
| Saturation type (vector vs component) | vector-magnitude clamp (A3), component clamp available |
| Noise target (which measurements, vector vs scalar) | all relative-position vectors, per component (A5) |
| Obstacle positions/radii | digitised from Figs. 5–6 (§8) |
| Initial velocities | zero (A8) |
| Member multiplicity of shared tile edges | 2, forced by statics + figures (§6.1 of PAPER_ANALYSIS.md) |
| Sign of Eq. (18) | corrected to repulsive (A1) |
| Whether obstacle "closest point" uses the true or measured geometry | true geometry, then the *relative vector* is corrupted by noise (A5) |
