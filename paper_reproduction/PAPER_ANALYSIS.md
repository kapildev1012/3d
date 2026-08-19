# PAPER_ANALYSIS.md

Technical specification extracted from:

> B. Ingalls, Q. Nelson, L. R. Garcia Carrillo, M. Majji,
> **"Adaptive Tensegrity-Based Control for Multi-Agent Obstacle Avoidance"**,
> 2026 European Control Conference (ECC), July 7–10 2026, Reykjavík, Iceland, pp. 2442–2447.
> ISBN 978-3-907144-13-8. Public release #AFRL-2025-5288. Video: https://youtu.be/25G0i-iGIP4

All equation numbers below are the paper's own numbering. Everything in this file was read
from the PDF in the parent folder (text layer + page rasters for the equations, and pixel
digitisation of Figures 3–6 for quantities that appear only graphically).

---

## 1. Problem

A homogeneous multi-agent system (MAS) of `n` point-mass agents in `d = 2` dimensions must

1. hold a prescribed geometric formation using **only local relative-position measurements**,
2. translate under a higher-level navigation command, and
3. deform around convex obstacles that are sensed **only locally** (radius `r_y`), then reform,

without inter-agent communication of obstacle information and without any global path planner.

The formation is a **virtual tensegrity structure**: agents are nodes; each edge is either a
virtual *string* (tensile, attractive) or a virtual *bar* (compressive, repulsive). The paper's
novelty is (i) power-law member forces with no rest length (so the pre-stress/self-stress
property of tensegrity is preserved), and (ii) an **adaptive string relaxation** law that caps
tensile force once an edge is stretched beyond `z1`, which lets the formation open up around a
large obstacle and close again afterwards while keeping its topology.

---

## 2. Notation, graph theory and tensegrity preliminaries (Sec. II)

`q_i ∈ R^d` is the position of node `i`; `q = [q_1 ... q_n] ∈ R^{d×n}`.
`G = (V, E)` undirected, `N_i = {j : a_ij ≠ 0}`.

* **Eq. (1)** Graph Laplacian: `L = D(A) − A`, `D(A) = diag(Σ_j a_ij)`.
  `G` is connected iff `rank(L) = n − 1`.
* **Eq. (2)** Stability necessary condition: `vᵀ K v > 0` for every non-trivial motion `v`.
* **Eq. (3)** `K = K_E + K_G` (linear + geometric stiffness).
* Tension coefficient (force density) of an edge: `ω_ij = f_ij / ℓ_ij`, `ℓ_ij = ||q_i − q_j||`.
* **Eq. (4)** Force-density matrix (FDM) `D ∈ R^{n×n}`, a weighted Laplacian:

  ```
  [D]_ij = −ω_ij                 i ≠ j , (i,j) ∈ E
  [D]_ii =  Σ_{k∈N_i} ω_ik
  [D]_ij =  0                    otherwise
  ```
* **Eq. (5)** Self-equilibrium (form-finding): `D x = 0`, `D y = 0`, `D z = 0`.
* **Eq. (6)** Connectivity matrices `C_B ∈ R^{B×n}`, `C_S ∈ R^{S×n}` with entries
  `−1` at the member start node, `+1` at the end node, `0` otherwise;
  `C = [C_Bᵀ C_Sᵀ]ᵀ`; member coordinate differences `u = C x`, `v = C y`, `w = C z`.
* **Eq. (7)** Geometry matrix with `U = diag(u)`, `V = diag(v)`, `W = diag(w)`:
  `G = [U u  V v  W w  U v  U w  V w]`.
* **Lemma 1** (from Zhang & Ohsaki, ref. [21]) — a tensegrity in `d` dimensions is stable if
  1. `rank`-deficiency of `D` is at least `d + 1`,
  2. `D ⪰ 0`,
  3. `rank(G) = d(d+1)/2`.

## 3. Agent model (Sec. III)

* Agent `i` measures `q_i − q_j` for `j ∈ N_i` (neighbours) and, for obstacles inside the
  sensing radius `r_y`, the vector to the **closest point of the obstacle** `d_j`,
  `j ∈ N_i^c` (the local obstacle set).
* **Eq. (8)** Control input:  `u_i = u_i^t(q) + Σ_{j∈N_i^c} u_ij^c(q_i, d_j, r_y) + u_i^n`
* **Eq. (9)** Dynamics (point mass with linear drag and input saturation):

  ```
  m q̈_i = −c q̇_i + sat_{u_max}(u_i)
  ```

  `m` mass, `c > 0` damping, `u_i^t` edge (tensegrity) force, `u_ij^c` collision-avoidance term,
  `u_i^n` navigation term ("the magnitude of the resultant summation is clamped to ±u_max").
* Objective: `q_i(t) → q_i*` where `q*` is a nominal self-stress state, and — because obstacle
  information is purely local — the equilibrium itself must be allowed to *expand* so the
  formation can deform around obstacles while preserving formation geometry/topology.

## 4. Tensegrity-based formation control (Sec. IV-A)

* **Eq. (10)** Internal member force (power law, **no rest length**): `f_ij(z) = k_ij z^{α_ij}`
* **Eq. (11)** `{k_ij, α_ij} = {k_B, α_B}` on bar edges, `{k_S, α_S}` on string edges, `{0,0}` if `i=j`.
* **Eq. (12)** Collective potential `V(q) = Σ_{(i,j)∈E} ψ(||q_i − q_j||)`,
  **Eq. (13)** `ψ(z) = ∫_0^z f(s) ds`.
* **Eq. (14)** Edge force term (negative gradient of the potential):

  ```
  u_i^t = −∇_{q_i} V(q) = −Σ_{j∈N_i} f(||q_i−q_j||) n_ij = −Σ_{j∈N_i} k_ij ℓ_ij^{α_ij} n_ij
  n_ij = (q_i − q_j)/ℓ_ij
  ```

  Sign convention: `k > 0` ⇒ pull toward the neighbour (string), `k < 0` ⇒ push away (bar).
* **Eq. (15)/(16)** Hessian blocks and `K_E`; `K_G = D ⊗ I_d`:

  ```
  H_ij = −ω_ij (I_d + (α_ij−1) n*_ij n*_ijᵀ)                    i ≠ j
  H_ii =  Σ_k ω_ik (I_d + (α_ik−1) n*_ik n*_ikᵀ)
  K_E,ij = −ω_ij (α_ij−1) n*_ij n*_ijᵀ , K_E,ii = Σ_k ω_ik (α_ik−1) n*_ik n*_ikᵀ
  vᵀ K_E v = Σ_{i≠j} ω_ij ℓ_ij^{−2} (α_ij−1) ((q_i*−q_j*)ᵀ(v_i−v_j))²
  ```
* **Lemma 2** Structural stability condition `k_ij (α_ij − 1) ≥ 0 ∀(i,j) ∈ E`
  (i.e. `α>1 ⇒ k≥0`; `α=1 ⇒ k∈R`; `α<1 ⇒ k≤0`).
* **Remark 2** Consequently bars need `k_B < 0`, `α_B < 0`; strings need `k_S > 0`, `α_S ≥ 1`.

## 5. Expansion for obstacle avoidance (Sec. IV-B)

* **Eq. (17)** `p_ij = (q_i − d_j)/||q_i − d_j||`; if the "obstacle" is another agent, `p_ij = n_ij`.
* **Eq. (18)** Collision-avoidance term, `k_a > 0`, `γ > 0`:

  ```
  u_ij^c = −k_a ( ||q_i − d_j||^{−γ} − r_y^{−γ} ) p_ij
  ```

  ⚠ **Sign**: as printed this is *attractive* toward the obstacle inside the sensing radius
  (the bracket is positive for `||q_i−d_j|| < r_y` and `p_ij` points *from* the obstacle *to*
  the agent). The implementation uses the repulsive sign `+k_a(·)p_ij`, which is the only
  reading consistent with the term being a "collision avoidance" term, with the paper's own
  Figs. 5–6 (agents are pushed away and never penetrate the obstacles), and with `k_a > 0`.
  Both variants are available via `cfg.avoid_sign` and the literal one is demonstrated to fail
  (see `ASSUMPTIONS.md`, item A1 and `results/figures/fig12_*`). The magnitude vanishes
  C¹-smoothly at `||q_i−d_j|| = r_y`.
* **Eq. (19)** String relaxation (replaces Eq. (10) on string edges):

  ```
  f_ij(ℓ) =  k_ij ℓ^{α_ij}     ℓ ≤ z1
             h(ℓ)              z1 ≤ ℓ ≤ z2
             β                 ℓ ≥ z2
  ```
* **Eq. (20)** Cubic (Hermite) spline, `s = (ℓ − z1)/(z2 − z1)`:

  ```
  h(ℓ) = (2s³ − 3s² + 1) k_ij z1^{α_ij}
       + (s³ − 2s² + s)(z2 − z1) k_ij α_ij z1^{α_ij − 1}
       + (−2s³ + 3s²) β
  ```

  This is the standard cubic Hermite basis with `p0 = f(z1)`, `m0 = f'(z1)`, `p1 = β`, `m1 = 0`;
  hence `f` is C¹ at `z1` and has zero slope at `z2`, joining the constant branch `β` smoothly.
  Verified numerically: it reproduces Fig. 3 (peak `13.51` at `ℓ ≈ 25.9`, `f(z1) = 8.19`,
  `f(ℓ≥z2) = 8`).

## 6. Numerical experiment (Sec. V)

* `d = 2`. Base unit: one square tile of 4 agents — 4 perimeter **strings**, 2 diagonal **bars**
  (Fig. 2).
* Simulation formation: **six interconnected square tiles, 2 rows × 3 columns ⇒ 12 agents**
  on a 4 × 3 node grid with nominal spacing `l_S`.
* **Eq. (21)** Analytic self-stress condition for this formation:

  ```
  k_B / k_S = − l_S^{α_S − α_B} · √2^{−α_B + 1}
  ```

  Derivation (reproduced independently, and it fixes the member multiplicity — see below):
  at a *corner* node two orthogonal strings pull inward with resultant `√2 k_S l_S^{α_S}` and one
  diagonal bar pushes outward with `|k_B| (√2 l_S)^{α_B}`; equating gives Eq. (21).
  Check with Table I: RHS `= −15^{2.5}·√2^{1.5} = −1465.5` vs `k_B/k_S = −50/0.0341 = −1466.3`
  (0.06 % — the table's `k_S` is Eq. (21) rounded to 3 significant figures).
* Gaussian noise, `σ = 0.5`, "imposed on all measurements".
* Navigation input `u_i^n = [0, 3]` (constant, all agents) ⇒ motion in `+Y`.
* Fig. 5: snapshots `t = 0, 15, 30, 40 s` **without** string relaxation.
  Fig. 6: same **with** string relaxation.
* Reported result (qualitative): both behave the same for small deformations; **without**
  relaxation the formation "can get hung up while navigating around large obstacles";
  **with** relaxation it passes the large obstacle by "a large local deformation of the two
  center units while keeping the integrity of the edge units", with no obstacle information
  exchanged between agents.

### 6.1 Member multiplicity (derived, not stated)

The 12-agent tiled structure is the **union of the members of the six tiles**: 6×4 = **24 strings**
and 6×2 = **12 bars**. Edges shared by two tiles therefore carry **two coincident strings**
(multiplicity 2). This is not stated in words but is forced by two independent facts:

1. **Statics.** With multiplicity 1 the uniform grid is *not* an equilibrium: a mid-edge node
   would see a residual `√2|k_B|(√2 l_S)^{α_B} − k_S l_S^{α_S} = k_S l_S^{α_S} = 7.67` outward,
   contradicting the paper's claim that the equilibrium of this formation follows analytically
   from Eq. (21). With multiplicity = number of sharing tiles, **every** node balances exactly
   (verified to machine precision in `validate_results.m`, test T2).
2. **Figure 5(a)/6(a).** The shared (interior) string edges are drawn as *solid* red lines while
   the boundary string edges are *dashed* — i.e. two dashed lines with different dash phase are
   drawn on top of each other on interior edges.

### 6.2 Quantities that appear only in the figures (digitised)

Digitised from the 8 embedded raster panels of Figs. 5–6 (axis-frame calibration verified
against the known `t = 0` grid, residual < 0.15 length units; see `tools/` in the parent folder):

| Quantity | Digitised | Adopted |
|---|---|---|
| Obstacle 1 centre / radius | (0.10, 90.0) / 12.13 ± 0.03 | (0, 90) / 12 |
| Obstacle 2 centre / radius | (29.9, 50.0) / 6.64 ± 0.03 | (30, 50) / 6.5 |
| Obstacle 3 centre / radius | (−29.6, 30.0) / 6.62 ± 0.03 | (−30, 30) / 6.5 |
| Initial grid | x ∈ {−22.5,−7.5,7.5,22.5}, y ∈ {−15,0,15} | same (spacing `l_S`, centred at origin) |
| Sensing-radius halo | ≈ 7.3–7.5 (overlapping halos) | consistent with `r_y = 8` |
| Axes | x ∈ [−50,50], y ∈ [−20,140] | same |

Centroid `Y` of the 12 agents per panel (graph-derived estimates, ±0.5):

| t [s] | Fig. 5 (no relaxation) | Fig. 6 (relaxation) |
|---|---|---|
| 0  | 0.08  | −0.01 |
| 15 | 33.28 | 32.74 |
| 30 | 73.40 | 74.52 |
| 40 | 86.72 | 104.88 |

## 7. Algorithm (implementation sequence)

```
1  parameters            <- Table I  (paper_config.m)
2  geometry              <- 2x3 tiling, spacing l_S, centred; member list with multiplicity
3  gains                 <- Eq. (21) consistency check of (k_B, k_S, l_S, α_S, α_B)
4  tensegrity analysis   <- Eq. (4)-(7), Lemma 1, Eq. (15)/(16), Lemma 2
5  state                 <- q(0) = q*, v(0) = 0
6  loop k = 0 .. T/dt
     6a  member lengths from *noisy* relative measurements (σ)          Eq. (14)
     6b  string force: Eq. (10)  or Eq. (19)/(20) if relaxation enabled
     6c  bar force:    Eq. (10)
     6d  u_i^t = −Σ f n̂                                                 Eq. (14)
     6e  obstacle set N_i^c: closest surface point d_j, measured, gate ‖·‖ ≤ r_y
     6f  u_ij^c                                                          Eq. (17),(18)
     6g  u_i = u_i^t + Σ u_ij^c + u_i^n ; saturate to u_max              Eq. (8),(9)
     6h  integrate m q̈ = −c q̇ + sat(u)  (fixed step dt)                 Eq. (9)
     6i  record state, member lengths, relaxation flags, detections, diagnostics
7  metrics: centroid, obstacle clearance, min inter-agent distance, shape error
8  figures (Fig. 2,3,4,5,6 + diagnostics) and validation report
```

## 8. Results reproduced here

See `VALIDATION.md`. Summary: the deterministic parts of the paper (Eq. (21), the self-stress
equilibrium and all three Lemma-1 conditions, Fig. 3 and Fig. 4) are reproduced to machine or
digitisation precision; the qualitative result of Figs. 5–6 (hung up without relaxation, passes
with relaxation) is reproduced; the *rate of translation* in Figs. 5–6 is faster than Eq. (9)
with Table I permits (`v_∞ = u^n/c = 2` exactly), which is documented as an unresolved
inconsistency in the paper rather than fitted away.
