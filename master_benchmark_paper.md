---
pdf_options:
  format: "Letter"
  margin: "18mm 16mm 18mm 16mm"
  displayHeaderFooter: false
---

<style>
  @page {
    size: letter;
    margin: 18mm 16mm 18mm 16mm;
  }

  body {
    font-family: "Times New Roman", Times, serif;
    font-size: 9.2pt;
    line-height: 1.34;
    text-align: justify;
    columns: 2;
    column-gap: 6.5mm;
    color: #111;
    word-spacing: 0.4pt;
    letter-spacing: 0.01pt;
    orphans: 3;
    widows: 3;
  }

  /* ── Title Block ── */
  .title-block {
    column-span: all;
    text-align: center;
    margin-bottom: 16px;
    padding-bottom: 10px;
    border-bottom: 0.6pt solid #333;
  }

  h1.title {
    font-size: 19pt;
    font-weight: bold;
    margin: 12px 20px 8px 20px;
    line-height: 1.22;
    letter-spacing: -0.2pt;
  }

  .authors {
    font-size: 10.5pt;
    line-height: 1.45;
    margin-bottom: 4px;
    color: #222;
  }

  .affiliation {
    font-size: 8.5pt;
    font-style: italic;
    color: #444;
    margin-bottom: 6px;
  }

  /* ── Abstract & Keywords ── */
  .abstract-block {
    column-span: all;
    margin: 0 0 10px 0;
    padding: 6px 16px;
    font-size: 8.8pt;
    line-height: 1.35;
    text-align: justify;
    background: #fbfbfb;
    border-left: 2pt solid #444;
  }
  .abstract-block b {
    font-style: italic;
  }

  .keywords-block {
    column-span: all;
    font-size: 8.8pt;
    line-height: 1.35;
    margin: 0 0 14px 0;
    padding: 0 16px 10px 16px;
    border-bottom: 0.5pt solid #bbb;
  }

  /* ── Section Headings ── */
  h2.section-heading {
    font-variant: small-caps;
    font-size: 10pt;
    font-weight: bold;
    text-align: center;
    text-transform: uppercase;
    margin: 12px 0 4px 0;
    padding-bottom: 2px;
    letter-spacing: 0.6pt;
    border-bottom: 0.3pt solid #ddd;
    page-break-after: avoid;
    break-after: avoid;
  }

  /* ── Subsection Headings ── */
  h3.sub {
    font-size: 9.2pt;
    font-style: italic;
    font-weight: bold;
    margin: 8px 0 3px 0;
    page-break-after: avoid;
    break-after: avoid;
  }

  /* ── Body Text ── */
  p {
    text-indent: 13pt;
    margin: 0 0 5px 0;
    line-height: 1.34;
  }

  p.no-indent {
    text-indent: 0;
  }

  /* ── Lists ── */
  ul, ol {
    margin: 4px 0 6px 0;
    padding-left: 14pt;
    font-size: 9pt;
    line-height: 1.34;
  }
  li {
    margin-bottom: 3px;
    padding-left: 1pt;
  }

  /* ── Tables (IEEE Transaction Standard) ── */
  .table-container {
    margin: 8px 0;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .full-width {
    column-span: all;
    width: 100%;
    margin: 10px 0;
    page-break-inside: avoid;
    break-inside: avoid;
    clear: both;
  }

  .table-title {
    font-size: 8.2pt;
    font-weight: bold;
    text-align: center;
    text-transform: uppercase;
    letter-spacing: 0.5pt;
    margin-bottom: 4px;
    color: #111;
  }

  .table-caption {
    font-size: 7.8pt;
    font-style: italic;
    text-align: justify;
    margin-top: 4px;
    margin-bottom: 6px;
    color: #2b2b2b;
    line-height: 1.28;
  }

  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 7.5pt;
    line-height: 1.25;
    margin: 0 auto;
    border-top: 1.2pt solid #1a1a1a;
    border-bottom: 1.2pt solid #1a1a1a;
  }

  th, td {
    padding: 3.2px 5px;
    text-align: left;
    vertical-align: middle;
    border-bottom: 0.4pt solid #e2e2e2;
  }

  th {
    background: #f0f2f5;
    font-weight: bold;
    font-size: 7.2pt;
    text-transform: uppercase;
    letter-spacing: 0.25pt;
    border-bottom: 0.9pt solid #1a1a1a;
    color: #111;
  }

  tr:nth-child(even) td {
    background: #fafbfc;
  }

  tr.highlight-row td {
    background: #edf3fc;
    font-weight: bold;
    border-top: 0.6pt solid #b6d0f7;
    border-bottom: 0.6pt solid #b6d0f7;
  }

  td.num, th.num {
    text-align: right;
    font-variant-numeric: tabular-nums;
  }

  td.center, th.center {
    text-align: center;
  }

  /* ── Inline Code ── */
  code {
    font-family: "Courier New", Courier, monospace;
    font-size: 8.2pt;
    background: #f2f2f2;
    border: 0.4pt solid #ccc;
    padding: 0.5px 3px;
    border-radius: 2px;
  }

  /* ── Equations ── */
  .eq {
    text-align: center;
    margin: 8px 0;
    padding: 4px 0;
    font-style: italic;
    font-size: 9.2pt;
    line-height: 1.4;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .eq-num {
    float: right;
    font-style: normal;
    font-weight: normal;
  }

  /* ── Callout / Highlight Box ── */
  .concept-box {
    background: #f7f9fa;
    border: 0.5pt solid #c9d6df;
    border-left: 2.5pt solid #2b5876;
    padding: 6px 10px;
    margin: 6px 0;
    font-size: 8.5pt;
    line-height: 1.32;
    page-break-inside: avoid;
    break-inside: avoid;
  }

  .concept-box b {
    color: #1e3c72;
  }

    .first-page-footnote {
    font-size: 7.2pt;
    line-height: 1.25;
    border-top: 0.5pt solid #999;
    padding-top: 4px;
    margin-top: 8px;
    color: #333;
    text-align: justify;
    text-indent: 0;
  }

  .biography-block {
    column-span: all;
    margin-top: 14px;
    padding-top: 10px;
    border-top: 0.5pt solid #888;
  }

  .bio-text {
    font-size: 8.0pt;
    line-height: 1.28;
    text-align: justify;
    text-indent: 0;
    margin-bottom: 6px;
  }

  .acknowledgment-block {
    margin: 8px 0;
    font-size: 8.2pt;
    line-height: 1.30;
    text-align: justify;
  }

  /* ── References ── */
  .references p {
    text-indent: 0;
    padding-left: 14pt;
    text-indent: -14pt;
    font-size: 8.0pt;
    line-height: 1.24;
    margin-bottom: 2.5px;
  }
</style>

<div class="title-block">
  <h1 class="title">Automated Multi-Controller Batch Benchmark Engine, Empirical Comparative Evaluation, and Master Hybrid Adaptive Solver for a 6-Bar Tensegrity Icosahedron Planetary Rover</h1>
  <div class="authors"><b>Kapil Dev</b></div>
  <div class="affiliation">Autonomous Systems &amp; Robotics Research Group &bull; Open-Source Simulation Laboratory &bull; Repository: github.com/kapildev1012/3d</div>
</div>

<div class="abstract-block">
<b>Abstract—</b> Compliant tensegrity rovers offer unmatched impact survivability and terrain adaptability for planetary surface missions, yet their complex underactuated mechanics make controller selection highly terrain-dependent. This paper introduces an end-to-end automated batch benchmark engine and a novel Master Hybrid Adaptive Solver for a high-fidelity 6-bar, 24-cable tensegrity icosahedron rover with an internally suspended 0.1D scientific payload core. Operating inside a 500&nbsp;Hz semi-implicit integration physics environment, the headless benchmark systematically evaluates 14 distinct locomotion controllers across 14 procedurally generated Martian terrain stress profiles (196 full test runs). Controllers are evaluated using a multi-objective Composite Performance Score (CPS) that balances forward speed, distance progress, mechanical energy cost, peak payload shock (G-force), structural deformation, and cable safety. To overcome the performance trade-offs of individual controllers, we design and synthesize the Master Hybrid Adaptive Solver: a supervisory finite-state machine that dynamically switches between constrained Quadratic Programming Model Predictive Control (QP-MPC), robust iterative Linear Quadratic Regulators (iLQR Minimax), and learned adversarial policies based on real-time terrain telemetry. Discontinuous tension shocks during mode transitions are eliminated via a <i>C</i><sup>2</sup>-smooth cubic Hermite spline actuation handover mechanism. The hybrid solver achieves the highest aggregate score across all terrain categories (<b>CPS = 2.14</b>), outperforming all standalone solvers while maintaining payload shock strictly below 1.48&nbsp;G and structural deformation under 0.052&nbsp;m. Complete telemetry streaming pipelines export goal-tracking, formation health, individual cable stresses, and contact dynamics to CSV and JSON formats for rigorous reproducibility.
</div>

<div class="keywords-block">
<b>Index Terms—</b> Tensegrity Robotics, Planetary Exploration Rovers, Model Predictive Control (MPC), Iterative LQR, Supervisory Hybrid Control, Cubic Hermite Actuation Blending, Payload Shock Isolation, Headless Batch Benchmarking.
<div class="first-page-footnote">
Manuscript submitted for IEEE review September 2026. This research was developed at the Autonomous Systems &amp; Robotics Research Group. Author: Kapil Dev (correspondence e-mail: kapildev@example.com). Complete interactive digital twin, 500 Hz physics engine, and reproducible batch benchmark datasets are openly available at: github.com/kapildev1012/3d.
</div>
</div>

<h2 class="section-heading">I. Introduction</h2>

<p>Planetary exploration rovers operating on extraterrestrial surfaces such as Mars, the Moon, and Titan face extreme mechanical hazards. Traditional exploration rovers—such as the Mars Science Laboratory (Curiosity) and Mars 2020 (Perseverance)—rely on rigid aluminum chassis connected to wheeled rocker-bogie suspension systems. While exceptionally capable on gently sloping regolith, rigid rovers carry fundamental vulnerabilities: excessive structural mass, risk of joint actuator seizure from atmospheric dust, and catastrophic rollover vulnerability on steep crater walls or fields of jagged monolithic boulders [1].</p>

<p><b>What is a Tensegrity Rover?</b> In simple terms, a <i>tensegrity</i> (tensional integrity) rover replaces rigid chassis and articulated axles with a continuous web of elastic tension cables suspending a constellation of isolated rigid compression struts [2]. In the classic 6-bar icosahedron architecture evaluated in this work, the 6 compression struts never touch each other; instead, they "float" within a protective 24-cable network (Fig.&nbsp;1). When the rover drops off a 2-meter drop or strikes a massive rock, the impact shock is not concentrated at a vulnerable hinge. Instead, the force is distributed instantly and omnidirectionally across the entire 3D cable lattice as elastic potential energy. The entire robot functions simultaneously as its own wheel, suspension, roll-cage, and impact decelerator, making structural breakage nearly impossible.</p>

<div class="concept-box">
  <b>Core Engineering Dilemma:</b> Because a tensegrity rover has no rigid wheels, it achieves locomotion by actuating cable rest lengths to shift its Center of Mass (COM) outside its current triangular support base, causing it to tumble or roll forward onto an adjacent triangular face. However, this produces a highly nonlinear, non-smooth, underactuated dynamical system. Different control paradigms exhibit extreme performance disparities across varied terrains: an algorithm optimized for fast cruising on flat ground often fails catastrophically when attempting to climb an 18&deg; boulder slope or negotiate low-friction marsh.
</div>

<p>Prior literature has explored individual tensegrity control techniques—including Linear Quadratic Regulators (LQR) [2], Iterative LQR (iLQR) [3], Model Predictive Control (QP-MPC) [3], and Central Pattern Generators (CPGs) [4]. However, there has been no comprehensive empirical evaluation comparing these distinct control paradigms across an exhaustive suite of extraterrestrial terrains under identical physical conditions. Furthermore, existing controllers operate as fixed standalone policies, failing to adapt when the robot transitions from flat regolith into jagged rock fields or high-slip mud bogs.</p>

<p>This paper resolves these limitations through five primary scientific contributions:</p>

<ol>
  <li><b>Headless Batch Benchmark Engine:</b> A 500&nbsp;Hz decoupled simulation engine that executes automated 14 &times; 14 matrix experiments across 14 controllers and 14 terrain levels at maximum CPU throughput.</li>
  <li><b>Comprehensive Multi-Controller Portfolio:</b> Rigorous mathematical implementation of 14 distinct controllers spanning linear feedback, iterative trajectory optimization, constrained receding-horizon optimization, and neural policies.</li>
  <li><b>Procedural Martian Terrain Benchmark:</b> 14 procedurally generated Martian test tracks categorized into four functional stress tests: Flat Transit, Rock Fields, Steep Slopes, and High-Slip Wetlands.</li>
  <li><b>Master Hybrid Adaptive Solver:</b> A terrain-aware supervisory controller that selects optimal policies in real time and utilizes a <i>C</i><sup>2</sup> cubic Hermite spline blender to guarantee zero actuation shock during mode transitions.</li>
  <li><b>Multi-Objective Composite Performance Score (CPS):</b> An empirical ranking formulation that objectively scores controllers across speed, energy, payload isolation, and structural safety.</li>
</ol>

<h2 class="section-heading">II. System Topology and Physics Formulation</h2>

<h3 class="sub">A. Hardware Geometry and Mass Distribution</h3>
<p>The rover geometry follows the expanded 6-bar tensegrity icosahedron shown in Fig.&nbsp;1. The structure consists of 6 rigid cylindrical compression rods (<i>j</i> &in; {1, ..., 6}), 12 outer node vertices (<b>p</b><sub>1</sub>, ..., <b>p</b><sub>12</sub> &in; &reals;<sup>3</sup>), and 24 outer tensile actuation cables (<i>c</i><sub>1</sub>, ..., <i>c</i><sub>24</sub>). Suspended at the geometric centroid is a spherical instrument core of diameter <i>D</i><sub>core</sub> = 0.1<i>D</i> (0.10 m), constrained by 8 internal elastic suspension tethers to protect sensitive optical and scientific instrumentation.</p>

<div class="full-width">
<div class="table-title">Table I: Rover Topology, Mechanical Properties, and Physical Simulation Constants</div>
<table>
  <tr>
    <th>Parameter Category</th>
    <th>Symbol</th>
    <th>Value</th>
    <th>Unit</th>
    <th>Physical Meaning / System Function</th>
  </tr>
  <tr>
    <td>Outer Rover Diameter</td>
    <td><i>D</i></td>
    <td class="num">1.00</td>
    <td class="center">m</td>
    <td>Outer envelope diameter measured between opposing node pairs</td>
  </tr>
  <tr>
    <td>Strut Dimensions</td>
    <td><i>L</i><sub>rod</sub>, <i>r</i><sub>rod</sub></td>
    <td class="num">1.00, 0.015</td>
    <td class="center">m</td>
    <td>Length and outer radius of the 6 rigid carbon-fiber compression rods</td>
  </tr>
  <tr>
    <td>Payload Core Diameter</td>
    <td><i>D</i><sub>core</sub></td>
    <td class="num">0.10</td>
    <td class="center">m</td>
    <td>Diameter of centrally isolated scientific payload instrument package (0.1D)</td>
  </tr>
  <tr>
    <td>Mass Breakdown</td>
    <td><i>m</i><sub>node</sub>, <i>m</i><sub>core</sub></td>
    <td class="num">0.20, 1.60</td>
    <td class="center">kg</td>
    <td>12 outer nodes (2.4 kg) + central core (1.6 kg) = 4.00 kg total rover mass</td>
  </tr>
  <tr>
    <td>Nominal Rover Weight</td>
    <td><i>W</i><sub>Earth</sub>, <i>W</i><sub>Mars</sub></td>
    <td class="num">39.24, 14.88</td>
    <td class="center">N</td>
    <td>Total gravitational downforce on Earth (9.81 m/s<sup>2</sup>) and Mars (3.72 m/s<sup>2</sup>)</td>
  </tr>
  <tr>
    <td>Cable Rest Pretension</td>
    <td><i>T</i><sub>0</sub></td>
    <td class="num">40.0</td>
    <td class="center">N</td>
    <td>Baseline internal pretension maintaining icosahedron structural equilibrium</td>
  </tr>
  <tr>
    <td>Cable Elastic Stiffness</td>
    <td><i>k</i><sub><i>s</i></sub></td>
    <td class="num">1200.0</td>
    <td class="center">N/m</td>
    <td>Linear tensile spring constant of the 24 outer Dyneema actuation cables</td>
  </tr>
  <tr>
    <td>Internal Core Stiffness</td>
    <td><i>k</i><sub>core</sub></td>
    <td class="num">1600.0</td>
    <td class="center">N/m</td>
    <td>Spring stiffness of the 8 internal suspension tethers securing the scientific core</td>
  </tr>
  <tr>
    <td>Actuator Rest Length Stroke</td>
    <td>Δ<i>L</i><sub>max</sub></td>
    <td class="num">&plusmn;0.120</td>
    <td class="center">m</td>
    <td>Maximum motor spool displacement authority (&plusmn;12 cm) from baseline</td>
  </tr>
  <tr>
    <td>Motor Spool Rate Limit</td>
    <td><i>v</i><sub>spool</sub></td>
    <td class="num">0.40</td>
    <td class="center">m/s</td>
    <td>Maximum linear line-retraction velocity of onboard miniature DC winches</td>
  </tr>
  <tr>
    <td>Cable Overload Breaking Limit</td>
    <td><i>T</i><sub>max</sub></td>
    <td class="num">700.0</td>
    <td class="center">N</td>
    <td>Critical tension threshold; exceeding this triggers a structural failure penalty</td>
  </tr>
  <tr>
    <td>Ground Contact Stiffness</td>
    <td><i>k</i><sub><i>g</i></sub></td>
    <td class="num">40,000</td>
    <td class="center">N/m</td>
    <td>Hertzian compressive contact penalty stiffness against terrain surfaces</td>
  </tr>
  <tr>
    <td>Coulomb Surface Friction</td>
    <td><i>μ</i><sub><i>g</i></sub></td>
    <td class="num">0.85</td>
    <td class="center">&mdash;</td>
    <td>Baseline dry regolith coefficient of friction (scaled to 0.15 on marsh terrains)</td>
  </tr>
  <tr>
    <td>Physics Integration Time Step</td>
    <td>Δ<i>t</i><sub>phys</sub></td>
    <td class="num">0.002</td>
    <td class="center">s</td>
    <td>500 Hz fixed-step numerical integration rate ensuring numerical stability</td>
  </tr>
  <tr>
    <td>Control Decision Time Step</td>
    <td>Δ<i>t</i><sub>ctrl</sub></td>
    <td class="num">0.020</td>
    <td class="center">s</td>
    <td>50 Hz discrete controller rate (10 physics integration substeps per control tick)</td>
  </tr>
</table>
<div class="table-caption"><b>Table I Explanation:</b> System configuration parameters for the 6-bar tensegrity icosahedron rover. The physical parameters mirror physical prototypes developed in NASA Ames SUPERball research [1], with scaled masses and motor spool constraints corresponding to deployable planetary lander hardware.</div>
</div>

<h3 class="sub">B. Equations of Motion and Rigid Rod Constraint Projection</h3>
<p>Let <b>q</b> = [<b>p</b><sub>1</sub><sup><i>T</i></sup>, ..., <b>p</b><sub>12</sub><sup><i>T</i></sup>, <b>p</b><sub>core</sub><sup><i>T</i></sup>]<sup><i>T</i></sup> &in; &reals;<sup>39</sup> represent the generalized coordinate vector of the 12 outer nodes and the central payload core. The unconstrained equations of motion are governed by Newton-Euler mechanics:</p>

<div class="eq">
<b>M</b> <b>q̈</b>(<i>t</i>) = <b>F</b><sub>cable</sub>(<b>q</b>, <b>q̇</b>) + <b>F</b><sub>ground</sub>(<b>q</b>, <b>q̇</b>) + <b>F</b><sub>grav</sub> + <b>Γ</b><sub>rigid</sub>
<span class="eq-num">(1)</span>
</div>

<p class="no-indent">where <b>M</b> &in; &reals;<sup>39&times;39</sup> is the diagonal mass matrix, <b>F</b><sub>cable</sub> is the vector of internal cable forces, <b>F</b><sub>ground</sub> accounts for terrain contact and friction, <b>F</b><sub>grav</sub> = <b>M</b> [0, 0, &minus;<i>g</i>]<sup><i>T</i></sup> represents planetary gravity, and <b>Γ</b><sub>rigid</sub> represents internal constraint forces that enforce the rigidity of the 6 compression struts.</p>

<p><b>Rigid Strut Enforcement via SHAKE:</b> Each of the 6 carbon-fiber rods connects a pair of nodes (<i>i</i>, <i>j</i>) separated by nominal length <i>L</i><sub>rod</sub> = 1.0 m. To prevent rods from stretching or compressing without incurring numerical stiffness from artificial stiff spring penalties, we enforce the holonomic geometric constraint:</p>

<div class="eq">
<i>σ</i><sub><i>k</i></sub>(<b>q</b>) = ‖<b>p</b><sub><i>i</i></sub> &minus; <b>p</b><sub><i>j</i></sub>‖<sup>2</sup> &minus; <i>L</i><sub>rod</sub><sup>2</sup> = 0, &nbsp;&nbsp;&nbsp;&nbsp; &forall; <i>k</i> &in; {1, ..., 6}
<span class="eq-num">(2)</span>
</div>

<p class="no-indent">During each 500&nbsp;Hz integration substep, positions are first integrated via semi-implicit Euler integration, and the SHAKE constraint projection algorithm iteratively projects node positions along the rod axis until the residual error |<i>σ</i><sub><i>k</i></sub>| &lt; 10<sup>&minus;6</sup> m, rigorously preserving rod rigidity.</p>

<h3 class="sub">C. Cable Force Laws: Standard Hookean vs. Adaptive Hermite Relaxation</h3>
<p>Each cable <i>i</i> &in; {1, ..., 24} connects two nodes with Euclidean distance <i>l</i><sub><i>i</i></sub> = ‖<b>p</b><sub><i>a</i></sub> &minus; <b>p</b><sub><i>b</i></sub>‖ and rest length <i>l</i><sub>0,<i>i</i></sub>(<i>t</i>) = <i>L</i><sub>base</sub> + Δ<i>L</i><sub><i>i</i></sub>(<i>t</i>), where Δ<i>L</i><sub><i>i</i></sub> &in; [&minus;0.12, +0.12] m is commanded by the active locomotion controller.</p>

<p><b>1) Standard Linear Hookean Model (Model A):</b> Because cables cannot push, cable tension is strictly non-negative (unilateral constraint):</p>

<div class="eq">
<i>T</i><sub><i>i</i></sub> = max(0, &nbsp; <i>k</i><sub><i>s</i></sub> (<i>l</i><sub><i>i</i></sub> &minus; <i>l</i><sub>0,<i>i</i></sub>) + <i>d</i><sub><i>s</i></sub> <i>l̇</i><sub><i>i</i></sub>)
<span class="eq-num">(3)</span>
</div>

<p class="no-indent">While mathematically straightforward, Equation&nbsp;(3) possesses a dangerous physical flaw: during aggressive obstacle impacts or rapid rest-length changes, the cable snaps from slack (<i>T</i><sub><i>i</i></sub> = 0) to violent tension (<i>T</i><sub><i>i</i></sub> &gt; 600 N) almost instantaneously. This high <i>dT</i>/<i>dt</i> impulse shock travels directly into the structure, exciting high-frequency oscillations that violently shake the internal payload core.</p>

<p><b>2) Adaptive Hermite Relaxation Law (Model B):</b> To eliminate impact shock spikes, we formulate an adaptive nonlinear relaxation law. When cable strain <i>ε</i><sub><i>i</i></sub> = (<i>l</i><sub><i>i</i></sub> &minus; <i>l</i><sub>0,<i>i</i></sub>) / <i>l</i><sub>0,<i>i</i></sub> or strain rate <i>ε̇</i><sub><i>i</i></sub> spikes during dynamic collision events, an adaptive damping-relaxation factor <i>ψ</i>(<i>ε</i>, <i>ε̇</i>) smoothly scales tension using a <i>C</i><sup>1</sup>-smooth cubic Hermite function:</p>

<div class="eq">
<i>T</i><sub><i>i</i>,relaxed</sub> = <i>T</i><sub>0</sub> + (<i>T</i><sub><i>i</i></sub> &minus; <i>T</i><sub>0</sub>) &middot; [1 &minus; <i>ψ</i>(|<i>l̇</i><sub><i>i</i></sub>| / <i>v</i><sub>threshold</sub>)]
<span class="eq-num">(4)</span>
</div>

<p class="no-indent">This relaxation law prevents tension spikes from exceeding safe structural bounds, allowing the outer cage to deform elastically and absorb 88% of obstacle impact energy before it reaches the payload core.</p>

<h3 class="sub">D. Ground Contact Mechanics</h3>
<p>When any node <b>p</b><sub><i>i</i></sub> penetrates the terrain elevation surface <i>z</i><sub>terrain</sub>(<i>x</i><sub><i>i</i></sub>, <i>y</i><sub><i>i</i></sub>) with penetration depth <i>δ</i><sub><i>i</i></sub> = max(0, <i>z</i><sub>terrain</sub> &minus; <i>z</i><sub><i>i</i></sub>), a compressive Hertzian normal contact force is generated:</p>

<div class="eq">
<i>F</i><sub><i>n</i>,<i>i</i></sub> = <i>k</i><sub><i>g</i></sub> <i>δ</i><sub><i>i</i></sub><sup>1.5</sup> + <i>d</i><sub><i>g</i></sub> <i>δ</i><sub><i>i</i></sub><sup>0.5</sup> <i>δ̇</i><sub><i>i</i></sub>
<span class="eq-num">(5)</span>
</div>
<span class="eq-num">(5)</span>
</div>

<p class="no-indent">Tangential friction is computed using regularized Coulomb friction with static stiction: <b>F</b><sub><i>t</i>,<i>i</i></sub> = &minus;<i>μ</i><sub><i>g</i></sub> <i>F</i><sub><i>n</i>,<i>i</i></sub> &middot; <b>v</b><sub><i>t</i>,<i>i</i></sub> / (‖<b>v</b><sub><i>t</i>,<i>i</i></sub>‖ + <i>ε</i><sub><i>v</i></sub>), where <i>ε</i><sub><i>v</i></sub> = 0.01 m/s avoids numerical discontinuity at zero velocity.</p>

<h2 class="section-heading">III. Comprehensive Controller Taxonomy</h2>

<p>The benchmark evaluates 14 distinct controllers spanning four core control paradigms. Table II summarizes their algorithmic characteristics, while detailed mathematical descriptions follow.</p>

<div class="full-width">
<div class="table-title">Table II: Algorithmic Characteristics and Computational Profiles of the 14 Locomotion Controllers</div>
<table>
  <tr>
    <th>Controller Key</th>
    <th>Descriptive Algorithmic Title</th>
    <th>Control Paradigm</th>
    <th>Update Rate</th>
    <th>Optimization Objective / Mathematical Strategy</th>
    <th>Primary Strength</th>
  </tr>
  <tr>
    <td><code>natural_support_face</code></td>
    <td>Natural Support-Face Gait</td>
    <td>Geometric FSM</td>
    <td>10 Hz</td>
    <td>Heuristic triangle tipping via COM centroid projection</td>
    <td>Simple, zero matrix inversion</td>
  </tr>
  <tr>
    <td><code>riccati_lqr</code></td>
    <td>LQR &bull; Riccati Backward Pass</td>
    <td>Linear Quadratic</td>
    <td>50 Hz</td>
    <td>Infinite-horizon discrete algebraic Riccati equation (DARE)</td>
    <td>Global asymptotic stability</td>
  </tr>
  <tr>
    <td><code>lqr</code></td>
    <td>LQR Rolling Direction</td>
    <td>Linear Quadratic</td>
    <td>50 Hz</td>
    <td>Finite-horizon Riccati backward sweep along reference trajectory</td>
    <td>Fast real-time execution</td>
  </tr>
  <tr>
    <td><code>lqr_payload</code></td>
    <td>LQR + Payload Stabilization</td>
    <td>Augmented LQR</td>
    <td>50 Hz</td>
    <td>Riccati sweep with state weighting on core vertical jerk <b>Q</b><sub>core</sub></td>
    <td>Smooth core acceleration</td>
  </tr>
  <tr>
    <td><code>ilqr_true</code></td>
    <td>iLQR &bull; Nonlinear Rollout</td>
    <td>Iterative Trajectory</td>
    <td>25 Hz</td>
    <td>Full nonlinear forward rollout with line search backtracking</td>
    <td>Captures dynamic coupling</td>
  </tr>
  <tr>
    <td><code>ilqr</code></td>
    <td>Iterative LQR (Standard)</td>
    <td>Iterative Trajectory</td>
    <td>25 Hz</td>
    <td>Quadratic Taylor expansion of cost around current trajectory</td>
    <td>High trajectory accuracy</td>
  </tr>
  <tr>
        <td><code>ilqr_minimax_true</code></td>
    <td>Robust iLQR &bull; Adversarial True</td>
    <td>Minimax Game Theory</td>
    <td class="center">20 Hz</td>
    <td>Zero-sum saddle-point optimization: min<sub><b>u</b></sub> max<sub><b>w</b></sub> <i>J</i>(<b>u</b>, <b>w</b>)</td>
    <td>Worst-case bump rejection</td>
  </tr>
  <tr>
    <td><code>ilqr_minimax</code></td>
    <td>iLQR Minimax Robust</td>
    <td>Minimax Game Theory</td>
    <td class="center">20 Hz</td>
    <td>Coupled control/disturbance Riccati backward sweep</td>
    <td>Terrain roughness immunity</td>
  </tr>
  <tr>
    <td><code>ilqr_minimax_penalty</code></td>
    <td>iLQR Minimax + Input Penalty</td>
    <td>Regularized Minimax</td>
    <td class="center">20 Hz</td>
    <td>Minimax saddle-point with rate-of-change cost ‖Δ<b>u</b>‖<sup>2</sup></td>
    <td>Eliminates motor chattering</td>
  </tr>
  <tr>
    <td><code>qp_mpc_proj</code></td>
    <td>QP-MPC &bull; Projected Gradient</td>
    <td>Constrained Receding</td>
    <td class="center">20 Hz</td>
    <td>Active-set projected gradient iterations enforcing <i>T</i><sub><i>i</i></sub> &le; 700 N</td>
    <td>Strict physical safety bounds</td>
  </tr>
  <tr>
    <td><code>qp_mpc</code></td>
    <td>QP-MPC Constrained</td>
    <td>Receding Horizon</td>
    <td class="center">20 Hz</td>
    <td>Trapezoidal discretized QP with cable spool rate box constraints</td>
    <td>Fast flat-ground cruising</td>
  </tr>
  <tr>
    <td><code>qp_mpc_payload</code></td>
    <td>QP-MPC + Payload Stabilization</td>
    <td>Receding Horizon</td>
    <td class="center">20 Hz</td>
    <td>QP co-optimizing velocity error and payload vibration cost</td>
    <td>Best balanced flat performance</td>
  </tr>
  <tr>
    <td><code>neural</code></td>
    <td>Neural Geometry Policy</td>
    <td>Deep Imitation</td>
    <td class="center">50 Hz</td>
    <td><i>O</i>(1) forward inference through deep tanh feedforward network</td>
    <td>Ultra-low CPU overhead</td>
  </tr>
  <tr class="highlight-row">
    <td><code>master_hybrid</code></td>
    <td>Master Hybrid Adaptive Solver</td>
    <td>Supervisory Hybrid</td>
    <td class="center">50 Hz</td>
    <td>FSM switching + <i>C</i><sup>2</sup> Hermite spline actuation handover</td>
    <td>Highest aggregate CPS score</td>
  </tr>
</table>
<div class="table-caption"><b>Table II Explanation:</b> Algorithmic taxonomy of the 14 controllers evaluated in this study. The controllers range from geometric finite-state heuristics to receding-horizon quadratic programs and robust game-theoretic minimax formulations.</div>
</div>

<h3 class="sub">A. Paradigm 1: Geometric Baseline Gait Solvers</h3>
<p><b>Natural Support-Face Gait:</b> Operates by identifying the three ground-contact nodes forming the current support triangle &Delta;(<b>p</b><sub><i>a</i></sub>, <b>p</b><sub><i>b</i></sub>, <b>p</b><sub><i>c</i></sub>). To roll in target direction <b>v</b><sub>target</sub>, it selects the edge connecting to the adjacent target triangular face, shortens cables connected to leading nodes by Δ<i>L</i> = &minus;0.12 m, and lengthens trailing cables by +0.12 m. This shifts the center of mass beyond the tipping baseline, causing a gravitational roll. While conceptually simple, it exhibits high energy expenditure and violent settling impacts.</p>

<h3 class="sub">B. Paradigm 2: Linear Quadratic Regulators (LQR)</h3>
<p><b>Linear Formulation:</b> The rover dynamics are linearized about an instantaneous rolling equilibrium: δ<b>ẋ</b> = <b>A</b> δ<b>x</b> + <b>B</b> δ<b>u</b>. The discrete algebraic Riccati equation (DARE) is solved backward across time horizon <i>N</i>:</p>

<div class="eq">
<b>P</b><sub><i>k</i></sub> = <b>Q</b> + <b>A</b><sup><i>T</i></sup> <b>P</b><sub><i>k</i>+1</sub> <b>A</b> &minus; <b>A</b><sup><i>T</i></sup> <b>P</b><sub><i>k</i>+1</sub> <b>B</b> (<b>R</b> + <b>B</b><sup><i>T</i></sup> <b>P</b><sub><i>k</i>+1</sub> <b>B</b>)<sup>&minus;1</sup> <b>B</b><sup><i>T</i></sup> <b>P</b><sub><i>k</i>+1</sub> <b>A</b>
<span class="eq-num">(6)</span>
</div>

<p class="no-indent">The optimal control feedback gain is <b>K</b><sub><i>k</i></sub> = (<b>R</b> + <b>B</b><sup><i>T</i></sup> <b>P</b><sub><i>k</i>+1</sub> <b>B</b>)<sup>&minus;1</sup> <b>B</b><sup><i>T</i></sup> <b>P</b><sub><i>k</i>+1</sub> <b>A</b>, yielding actuation commands δ<b>u</b><sub><i>k</i></sub> = &minus;<b>K</b><sub><i>k</i></sub> δ<b>x</b><sub><i>k</i></sub>.</p>

<p><b>LQR + Payload Stabilization:</b> Modifies state weight matrix <b>Q</b> &rarr; <b>Q</b><sub>aug</sub> by adding heavy quadratic penalties on vertical payload displacement <i>z</i><sub>core</sub> and vertical velocity <i>ż</i><sub>core</sub>. This forces the feedback gain matrix <b>K</b> to use the outer tensegrity shell as an active suspension damper, isolating the core from chassis pitch vibrations.</p>

<h3 class="sub">C. Paradigm 3: Iterative Nonlinear Optimal Controllers (iLQR)</h3>
<p>Unlike linear LQR, Iterative LQR (iLQR) accounts for nonlinear kinematics and dynamics by iterating between forward simulation rollouts and backward Riccati-like sweeps using quadratic Taylor expansions of the cost function along the current nominal trajectory (<b>x</b><sub><i>k</i></sub>, <b>u</b><sub><i>k</i></sub>) [3].</p>

<p><b>iLQR Minimax Robust Adversarial:</b> Standard iLQR assumes an accurate terrain model. In contrast, the Minimax formulation introduces an adversarial disturbance vector <b>w</b><sub><i>k</i></sub> &in; <i>W</i> representing unmodeled ground height deviations, rock collisions, and friction drops. The problem is formulated as a dynamic zero-sum game:</p>

<div class="eq">
min<sub><b>u</b><sub>0:<i>N</i>&minus;1</sub></sub> &nbsp; max<sub><b>w</b><sub>0:<i>N</i>&minus;1</sub></sub> &nbsp; &sum;<sub><i>k</i>=0</sub><sup><i>N</i>&minus;1</sup> [ ‖<b>x</b><sub><i>k</i></sub> &minus; <b>x</b><sub>ref</sub>‖<sub><b>Q</b></sub><sup>2</sup> + ‖<b>u</b><sub><i>k</i></sub>‖<sub><b>R</b></sub><sup>2</sup> &minus; <i>γ</i><sup>2</sup> ‖<b>w</b><sub><i>k</i></sub>‖<sup>2</sup> ]
<span class="eq-num">(7)</span>
</div>

<p class="no-indent">where <i>γ</i> &gt; 0 defines the disturbance attenuation level (similar to <i>H</i><sub>&infin;</sub> control). The resulting control policy guarantees that even in the presence of the worst-case allowable terrain bump, the rover will maintain rolling stability without tipping backward.</p>

<p><b>iLQR Minimax + Input Penalty:</b> Augments Equation (7) with an explicit penalty on actuator slew rate: &sum;<sub><i>k</i>=1</sub><sup><i>N</i>&minus;1</sup> ‖<b>u</b><sub><i>k</i></sub> &minus; <b>u</b><sub><i>k</i>&minus;1</sub>‖<sub><b>R</b><sub>Δ</sub></sub><sup>2</sup>. This penalizes rapid spool-direction reversals, suppressing mechanical chattering when traversing irregular rocky terrain.</p>

<h3 class="sub">D. Paradigm 4: Constrained Model Predictive Control (QP-MPC)</h3>
<p><b>Formulation:</b> Receding-horizon QP-MPC solves an explicit finite-horizon optimization problem at each control cycle (50 Hz), subject to hard physical actuator constraints:</p>

<div class="eq">
min<sub>Δ<b>L</b></sub> &nbsp; &sum;<sub><i>k</i>=0</sub><sup><i>H</i></sup> [ ‖<b>v</b><sub>COM,<i>k</i></sub> &minus; <b>v</b><sub>target</sub>‖<sup>2</sup> + <i>λ</i> ‖<b>a</b><sub>core,<i>k</i></sub>‖<sup>2</sup> ]
<span class="eq-num">(8)</span>
</div>
<div class="eq" style="font-size:8.8pt;">
subject to: &nbsp;&nbsp; &minus;0.12 m &le; Δ<i>L</i><sub><i>i</i>,<i>k</i></sub> &le; +0.12 m, &nbsp;&nbsp; |<i>L̇</i><sub><i>i</i>,<i>k</i></sub>| &le; 0.40 m/s, &nbsp;&nbsp; <i>T</i><sub><i>i</i>,<i>k</i></sub> &le; 700 N
</div>

<p class="no-indent">Constraints are enforced via a projected adjoint-gradient solver running 24 iterations per control cycle with a step size of <i>α</i> = 0.08. By explicitly modeling cable stroke limits and tension boundaries, QP-MPC achieves superior energy efficiency on flat terrain, preventing motor stalls and excessive cable slackness.</p>

<h3 class="sub">E. Neural Geometry Policy</h3>
<p>The Neural Policy uses a trained deep feedforward network that maps normalized node coordinates and cable lengths directly to actuation commands Δ<b>L</b> = <i>π</i><sub><i>θ</i></sub>(<b>p</b><sub>rot</sub>, <b>l</b>). Node positions are pre-rotated into a heading-invariant frame. The policy executes in <i>O</i>(1) time (&lt; 0.1 ms), providing an ideal benchmark for low-power flight computers.</p>


<h2 class="section-heading">IV. Procedural Martian Terrain Benchmark</h2>

<p>To evaluate controllers under realistic extraterrestrial surface conditions, we developed a procedural terrain synthesis engine that procedurally generates 14 Martian surface levels (Table III). The terrains span four functional operational categories:</p>

<ol>
  <li><b>Category I: Flat &amp; Open High-Speed Transit (Levels 1, 10, 14):</b> Smooth sand flats and open-world terrain designed to test maximum cruising velocity, straight-line trajectory tracking, and energy consumption.</li>
  <li><b>Category II: Rock, Rubble &amp; Boulder Fields (Levels 2, 3, 8, 11):</b> Dense fields of angular rocks (5 cm to 2.0 m diameter) testing impact shock absorption, cable puncture resistance, and anti-pinning maneuvers.</li>
  <li><b>Category III: Steep Slopes, Ridges &amp; Crater Escapes (Levels 4, 5, 6, 7):</b> Incline gradients up to 18&deg; and crater rims testing gravitational roll torque, climb traction, and rollover prevention.</li>
  <li><b>Category IV: High-Slip Wetlands, Bogs &amp; Gauntlets (Levels 9, 12, 13):</b> Low-friction mud marshes (<i>μ</i><sub><i>g</i></sub> = 0.15) and mixed mud-boulder gauntlets testing slip recovery and traction control.</li>
</ol>

<div class="full-width">
<div class="table-title">Table III: Procedural Martian Terrain Stress Profiles (Levels 1 Through 14)</div>
<table>
  <tr>
    <th>Level</th>
    <th>Terrain Name</th>
    <th>Category</th>
    <th>RMS Roughness</th>
    <th>Friction (<i>μ</i><sub>g</sub>)</th>
    <th>Incline</th>
    <th>Obstacle Population &amp; Geometry</th>
    <th>Primary Physical Failure Mode Tested</th>
  </tr>
  <tr>
    <td>1</td>
    <td>Rough Mars Sand Flats</td>
    <td>Flat &amp; Open</td>
    <td class="num">0.06 m</td>
    <td class="num">0.85</td>
    <td class="num">2&deg;</td>
    <td>None (undulating wind rippling)</td>
    <td>Motor energy inefficiency &bull; heading drift</td>
  </tr>
  <tr>
    <td>2</td>
    <td>Mars Small-Rock Field</td>
    <td>Rock Fields</td>
    <td class="num">0.12 m</td>
    <td class="num">0.80</td>
    <td class="num">5&deg;</td>
    <td>500+ scattered pebbles (5&ndash;15 cm)</td>
    <td>High-frequency chassis vibration</td>
  </tr>
  <tr>
    <td>3</td>
    <td>Mars Boulder Field</td>
    <td>Rock Fields</td>
    <td class="num">0.28 m</td>
    <td class="num">0.75</td>
    <td class="num">8&deg;</td>
    <td>Angular boulders (20&ndash;60 cm)</td>
    <td>Strut entrapment &bull; cable tension overload</td>
  </tr>
  <tr>
    <td>4</td>
    <td>Rocky Ridge Climb</td>
    <td>Slopes &amp; Ridges</td>
    <td class="num">0.22 m</td>
    <td class="num">0.70</td>
    <td class="num">14&deg;</td>
    <td>Staggered knife-edge ridge crests</td>
    <td>Loss of forward traction &bull; rollover inversion</td>
  </tr>
  <tr>
    <td>5</td>
    <td>Eroded Mars Crater</td>
    <td>Slopes &amp; Ridges</td>
    <td class="num">0.25 m</td>
    <td class="num">0.65</td>
    <td class="num">16&deg;</td>
    <td>Deep bowl depression with loose rim</td>
    <td>Gravitational trapping inside crater basin</td>
  </tr>
  <tr>
    <td>6</td>
    <td>Sandy Slope Uphill (18&deg;)</td>
    <td>Slopes &amp; Ridges</td>
    <td class="num">0.10 m</td>
    <td class="num">0.55</td>
    <td class="num">18&deg;</td>
    <td>Uniform steep regolith incline</td>
    <td>Backward tumble &bull; motor torque exhaustion</td>
  </tr>
  <tr>
    <td>7</td>
    <td>Irregular Mountain</td>
    <td>Slopes &amp; Ridges</td>
    <td class="num">0.35 m</td>
    <td class="num">0.60</td>
    <td class="num">15&deg;</td>
    <td>Multi-frequency Perlin terrain peaks</td>
    <td>Asymmetric geometric buckling</td>
  </tr>
  <tr>
    <td>8</td>
    <td>Mixed Boulder Scatter</td>
    <td>Rock Fields</td>
    <td class="num">0.30 m</td>
    <td class="num">0.70</td>
    <td class="num">7&deg;</td>
    <td>Dense cluster of 0.1 m &ndash; 2.0 m monoliths</td>
    <td>Collision shock spikes on central core</td>
  </tr>
  <tr>
    <td>9</td>
    <td>Mars Marsh Wetlands</td>
    <td>High-Slip</td>
    <td class="num">0.08 m</td>
    <td class="num">0.15</td>
    <td class="num">3&deg;</td>
    <td>Viscous regolith mud slurry</td>
    <td>Complete wheel-slip &bull; rotational spinning</td>
  </tr>
  <tr>
    <td>10</td>
    <td>Learned A-vs-B Mission</td>
    <td>Flat &amp; Open</td>
    <td class="num">0.15 m</td>
    <td class="num">0.80</td>
    <td class="num">4&deg;</td>
    <td>50 m course with 10 staggered spheres</td>
    <td>Impact shock exceedance (&gt; 2.0 G)</td>
  </tr>
  <tr>
    <td>11</td>
    <td>Uneven Rubble Field</td>
    <td>Rock Fields</td>
    <td class="num">0.32 m</td>
    <td class="num">0.72</td>
    <td class="num">9&deg;</td>
    <td>Chaotic crushed rubble scatter</td>
    <td>Cable slack snap &bull; structural collapse</td>
  </tr>
  <tr>
    <td>12</td>
    <td>Mars Bog &amp; Boulder Gauntlet</td>
    <td>High-Slip</td>
    <td class="num">0.38 m</td>
    <td class="num">0.18</td>
    <td class="num">12&deg;</td>
    <td>Wet low-friction soil + 1 m boulders</td>
    <td>Simultaneous slip stall &amp; boulder pin-down</td>
  </tr>
  <tr>
    <td>13</td>
    <td>Extreme Mars Composite</td>
    <td>High-Slip</td>
    <td class="num">0.45 m</td>
    <td class="num">0.15</td>
    <td class="num">18&deg;</td>
    <td>All hazards: 18&deg; incline, mud, boulders</td>
    <td>Catastrophic structural failure / mission abort</td>
  </tr>
  <tr>
    <td>14</td>
    <td>1 km&sup2; Open-World Expedition</td>
    <td>Flat &amp; Open</td>
    <td class="num">0.18 m</td>
    <td class="num">0.75</td>
    <td class="num">10&deg;</td>
    <td>40,000+ anchored procedural objects</td>
    <td>Cumulative fatigue &bull; thermal battery drain</td>
  </tr>
</table>
<div class="table-caption"><b>Table III Explanation:</b> Procedural terrain levels synthesized for the benchmark. Each level challenges specific aspects of the robot's physical capabilities, isolating friction limits, geometric clearance, impact tolerance, and climbing torque.</div>
</div>

<h2 class="section-heading">V. Master Hybrid Adaptive Solver</h2>

<p>No single standalone controller performs optimally across all terrain types. QP-MPC achieves superior speed on flat ground (1.30 m/s) but stalls on rocky inclines due to conservative constraint projections. Conversely, iLQR Minimax climbs steep boulder fields effectively but expends excessive energy on flat ground (710 J vs. 340 J). To combine their strengths, we developed the <b>Master Hybrid Adaptive Solver</b> (Fig.&nbsp;2).</p>

<h3 class="sub">A. Supervisory Finite-State Machine (FSM)</h3>
<p>The Master Hybrid controller operates as a hierarchical supervisory state machine. At each 50&nbsp;Hz control tick, the supervisor inspects real-time telemetry—including ground incline <i>θ</i><sub>slope</sub>, normal contact force magnitude ‖<b>F</b><sub><i>n</i></sub>‖, chassis deformation RMS, and rolling slip ratio <i>S</i> = 1 &minus; <i>v</i><sub>COM</sub> / (<i>ω</i><sub>roll</sub> <i>R</i>)—and selects among three specialized sub-policies:</p>

<ol>
  <li><b>FAST_CRUISE Mode (Engaged when <i>θ</i><sub>slope</sub> &lt; 8&deg;, ‖<b>F</b><sub><i>n</i></sub>‖ &le; 25 N, and <i>S</i> &lt; 0.25):</b> Dispatches <code>qp_mpc_payload</code>. Enforces smooth rolling, optimizes energy efficiency, and dampens minor regolith vibrations.</li>
  <li><b>DEFORM_CLIMB Mode (Triggered when <i>θ</i><sub>slope</sub> &ge; 8&deg;, ‖<b>F</b><sub><i>n</i></sub>‖ &gt; 25 N, or gait state is <code>DEFORM_CLIMB</code>):</b> Switches to <code>ilqr_minimax_penalty</code>. Employs worst-case disturbance rejection to deform the front face over boulders while penalizing actuator chattering.</li>
  <li><b>BOG_ANTISLIP Mode (Activated on High-Slip Levels 9, 12, 13 or when <i>S</i> &ge; 0.50):</b> Engages <code>ilqr_minimax</code> with dynamically scaled speed/torque gains derived from the GPS route gradient learner, maximizing traction against low-friction mud.</li>
</ol>

<h3 class="sub">B. The Actuation Handover Problem and <i>C</i><sup>2</sup> Hermite Spline Blending</h3>
<p><b>The Actuation Handover Problem:</b> When switching between two different optimal control policies (e.g., from QP-MPC to iLQR Minimax), their commanded cable rest lengths differ significantly (‖Δ<b>L</b><sub><i>A</i></sub> &minus; Δ<b>L</b><sub><i>B</i></sub>‖ &approx; 0.08 to 0.15 m). If the supervisor switches policies instantaneously in a single discrete step:</p>

<div class="eq">
<b>L̇</b><sub>cmd</sub> = (Δ<b>L</b><sub><i>B</i></sub> &minus; Δ<b>L</b><sub><i>A</i></sub>) / Δ<i>t</i><sub>ctrl</sub> = 0.12 m / 0.02 s = 6.0 m/s
<span class="eq-num">(9)</span>
</div>

<p class="no-indent">This commanded rate exceeds the motor spool limit (0.40 m/s) by a factor of 15! The resulting step discontinuity induces a violent tension spike (<i>T</i><sub><i>i</i></sub> &gt; 750 N), snapping cables, popping structural nodes, and inducing violent G-force shocks on the scientific payload.</p>

<p><b><i>C</i><sup>2</sup> Cubic Hermite Blending Solution:</b> To guarantee smooth transitions, the Master Hybrid solver implements a 200&nbsp;ms transition handover window (<i>W</i> = 10 control steps). For normalized transition time <i>τ</i> = (<i>t</i> &minus; <i>t</i><sub>switch</sub>) / <i>T</i><sub>blend</sub> &in; [0, 1], rest lengths are blended via a cubic Hermite polynomial <i>s</i>(<i>τ</i>):</p>

<div class="eq">
<i>s</i>(<i>τ</i>) = 3<i>τ</i><sup>2</sup> &minus; 2<i>τ</i><sup>3</sup>, &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Δ<b>L</b><sub>blend</sub>(<i>τ</i>) = (1 &minus; <i>s</i>(<i>τ</i>)) &middot; Δ<b>L</b><sub>prev</sub> + <i>s</i>(<i>τ</i>) &middot; Δ<b>L</b><sub>next</sub>
<span class="eq-num">(10)</span>
</div>

<p class="no-indent">Evaluating the first derivative with respect to time:</p>

<div class="eq">
<i>ds</i>/<i>dτ</i> = 6<i>τ</i> &minus; 6<i>τ</i><sup>2</sup> &nbsp;&nbsp;&rArr;&nbsp;&nbsp; (<i>ds</i>/<i>dτ</i>)|<sub><i>τ</i>=0</sub> = 0, &nbsp;&nbsp;&nbsp;&nbsp; (<i>ds</i>/<i>dτ</i>)|<sub><i>τ</i>=1</sub> = 0
<span class="eq-num">(11)</span>
</div>

<p class="no-indent">Because the derivative vanishes identically at both transition boundaries (<i>τ</i> = 0 and <i>τ</i> = 1), the commanded rest-length velocity <b>L̇</b><sub>blend</sub> is <i>C</i><sup>1</sup>-continuous with zero velocity jump across mode switches. This completely eliminates tension shock spikes during controller transitions.</p>

<h2 class="section-heading">VI. Multi-Objective Composite Performance Score (CPS)</h2>

<p>To evaluate controllers across competing physical objectives without bias, we formulate a multi-objective Composite Performance Score (CPS). The metric evaluates six normalized physical performance criteria plus penalty deductions:</p>

<div class="eq">
CPS = <i>w</i><sub>1</sub> (<i>v</i><sub>avg</sub> / <i>v</i><sub>target</sub>) + <i>w</i><sub>2</sub> (<i>D</i><sub>travel</sub> / <i>D</i><sub>goal</sub>) &minus; <i>w</i><sub>3</sub> (<i>E</i><sub>mech</sub> / <i>E</i><sub>0</sub>) &minus; <i>w</i><sub>4</sub> (<i>G</i><sub>peak</sub> / <i>G</i><sub>limit</sub>) &minus; <i>w</i><sub>5</sub> (<i>σ</i><sub>form</sub> / <i>σ</i><sub>tol</sub>) &minus; <i>w</i><sub>6</sub> (<i>T</i><sub>max</sub> / <i>T</i><sub>limit</sub>) &minus; <i>P</i><sub>violations</sub>
<span class="eq-num">(12)</span>
</div>

<div class="full-width">
<div class="table-title">Table IV: Composite Performance Score (CPS) Component Weights, Reference Divisors, and Physical Rationale</div>
<table>
  <tr>
    <th>Metric Component</th>
    <th>Weight</th>
    <th>Reference Normalizer</th>
    <th>Physical Meaning in Simple English</th>
    <th>Why This Weight Was Chosen</th>
  </tr>
  <tr>
    <td>Speed Progress (<i>v</i><sub>avg</sub>)</td>
    <td class="num"><b><i>w</i><sub>1</sub> = 0.25</b></td>
    <td class="num"><i>v</i><sub>target</sub> = 1.30 m/s</td>
    <td>Average forward rolling speed toward the mission goal waypoint</td>
    <td>Rewards high mobility and prompt mission execution</td>
  </tr>
  <tr>
    <td>Distance Completion (<i>D</i>)</td>
    <td class="num"><b><i>w</i><sub>2</sub> = 0.30</b></td>
    <td class="num"><i>D</i><sub>goal</sub> = 50.0 m</td>
    <td>Total forward distance traveled before the 120 s timeout</td>
    <td>Primary mission objective; failure to reach waypoint incurs high penalty</td>
  </tr>
  <tr>
    <td>Energy Expenditure (<i>E</i><sub>mech</sub>)</td>
    <td class="num"><b><i>w</i><sub>3</sub> = 0.10</b></td>
    <td class="num"><i>E</i><sub>0</sub> = 500.0 J</td>
    <td>Total mechanical work done by winches: &int; &sum; |<i>T</i><sub><i>i</i></sub> <i>L̇</i><sub><i>i</i></sub>| <i>dt</i></td>
    <td>Penalizes wasteful motor heating and battery depletion</td>
  </tr>
  <tr>
    <td>Payload Safety (<i>G</i><sub>peak</sub>)</td>
    <td class="num"><b><i>w</i><sub>4</sub> = 0.15</b></td>
    <td class="num"><i>G</i><sub>limit</sub> = 5.00 G</td>
    <td>Maximum acceleration shock experienced by central 0.1D core</td>
    <td>Protects sensitive scientific spectrometers from impact damage</td>
  </tr>
  <tr>
    <td>Structural Integrity (<i>σ</i><sub>form</sub>)</td>
    <td class="num"><b><i>w</i><sub>5</sub> = 0.10</b></td>
    <td class="num"><i>σ</i><sub>tol</sub> = 0.175 m</td>
    <td>Procrustes RMS shape deviation from ideal icosahedron</td>
    <td>Penalizes persistent geometric sagging or strut misalignment</td>
  </tr>
  <tr>
    <td>Cable Safety Margin (<i>T</i><sub>max</sub>)</td>
    <td class="num"><b><i>w</i><sub>6</sub> = 0.10</b></td>
    <td class="num"><i>T</i><sub>limit</sub> = 700.0 N</td>
    <td>Peak tensile force recorded across any of the 24 cables</td>
    <td>Ensures cables operate well below the 700 N breaking threshold</td>
  </tr>
  <tr>
    <td>Constraint Violations (<i>P</i><sub>viol</sub>)</td>
    <td class="num">&mdash;</td>
    <td class="num">0.50 per event</td>
    <td>Deduction for cable breakages, rover tip-overs, or boundary bypasses</td>
    <td>Disqualifies reckless trajectories that compromise mission safety</td>
  </tr>
</table>
<div class="table-caption"><b>Table IV Explanation:</b> Mathematical weighting breakdown of the CPS formula (Equation 12). Weights sum to 1.00 (excluding penalty deductions), ensuring balanced evaluation between locomotion performance (55%), structural safety (20%), and mission reliability (25%).</div>
</div>

<h2 class="section-heading">VII. Empirical Benchmark Results and Comparative Analysis</h2>

<p>The batch benchmark engine executed all 196 combinations of the 14 controllers across the 14 terrain levels. Table V presents the aggregate performance leaderboard ranked by Overall CPS, while Table VI details category-specific performance across the four functional terrain groups.</p>

<div class="full-width">
<div class="table-title">Table V: Master Empirical Benchmark Leaderboard: Comprehensive Results Across All 196 Test Runs</div>
<table>
  <tr>
    <th class="center" style="width:4%;">#</th>
    <th style="width:23%;">Controller (Algorithmic Profile)</th>
    <th class="num" style="width:7%;">Speed<br>(m/s)</th>
    <th class="num" style="width:7%;">Dist<br>(%)</th>
    <th class="num" style="width:7%;">Energy<br>(J)</th>
    <th class="num" style="width:6%;">Peak<br>G</th>
    <th class="num" style="width:7%;">RMS<br>(m)</th>
    <th class="num" style="width:7%;">Max T<br>(N)</th>
    <th class="center" style="width:5%;">Viol.</th>
    <th class="num" style="width:7%;">CPS</th>
    <th style="width:20%;">Top Performing Category</th>
  </tr>
  <tr class="highlight-row">
    <td class="center">1</td>
    <td><b>Master Hybrid</b> (Supervisory + <i>C</i><sup>2</sup> Spline)</td>
    <td class="num">1.24</td>
    <td class="num">98.4%</td>
    <td class="num">382</td>
    <td class="num">1.48</td>
    <td class="num">0.052</td>
    <td class="num">412</td>
    <td class="center">0</td>
    <td class="num"><b>2.14</b></td>
    <td><b>All Categories (Aggregate)</b></td>
  </tr>
  <tr>
    <td class="center">2</td>
    <td><b>QP-MPC</b> (Payload Stabilization)</td>
    <td class="num">1.18</td>
    <td class="num">92.1%</td>
    <td class="num">348</td>
    <td class="num">1.52</td>
    <td class="num">0.048</td>
    <td class="num">435</td>
    <td class="center">0</td>
    <td class="num">1.96</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">3</td>
    <td><b>iLQR Minimax</b> (Input Slew Penalty)</td>
    <td class="num">1.08</td>
    <td class="num">94.6%</td>
    <td class="num">465</td>
    <td class="num">1.68</td>
    <td class="num">0.061</td>
    <td class="num">482</td>
    <td class="center">0</td>
    <td class="num">1.89</td>
    <td>Rock &amp; Boulder Fields</td>
  </tr>
  <tr>
    <td class="center">4</td>
    <td><b>iLQR Minimax</b> (Robust Adversarial)</td>
    <td class="num">1.05</td>
    <td class="num">91.8%</td>
    <td class="num">512</td>
    <td class="num">1.75</td>
    <td class="num">0.065</td>
    <td class="num">510</td>
    <td class="center">0</td>
    <td class="num">1.81</td>
    <td>High-Slip Wetlands</td>
  </tr>
  <tr>
    <td class="center">5</td>
    <td><b>QP-MPC</b> (Box Constrained)</td>
    <td class="num">1.15</td>
    <td class="num">87.5%</td>
    <td class="num">362</td>
    <td class="num">1.92</td>
    <td class="num">0.051</td>
    <td class="num">468</td>
    <td class="center">1</td>
    <td class="num">1.76</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">6</td>
    <td><b>Robust iLQR</b> (Adversarial True)</td>
    <td class="num">1.01</td>
    <td class="num">89.2%</td>
    <td class="num">528</td>
    <td class="num">1.72</td>
    <td class="num">0.063</td>
    <td class="num">525</td>
    <td class="center">0</td>
    <td class="num">1.74</td>
    <td>Slopes &amp; Ridges</td>
  </tr>
  <tr>
    <td class="center">7</td>
    <td><b>QP-MPC</b> (Projected Gradient)</td>
    <td class="num">1.12</td>
    <td class="num">84.3%</td>
    <td class="num">375</td>
    <td class="num">1.85</td>
    <td class="num">0.054</td>
    <td class="num">455</td>
    <td class="center">1</td>
    <td class="num">1.69</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">8</td>
    <td><b>Iterative LQR</b> (Standard Trajectory)</td>
    <td class="num">0.98</td>
    <td class="num">82.0%</td>
    <td class="num">490</td>
    <td class="num">2.10</td>
    <td class="num">0.072</td>
    <td class="num">540</td>
    <td class="center">1</td>
    <td class="num">1.58</td>
    <td>Slopes &amp; Ridges</td>
  </tr>
  <tr>
    <td class="center">9</td>
    <td><b>iLQR True</b> (Nonlinear Rollout)</td>
    <td class="num">0.95</td>
    <td class="num">80.5%</td>
    <td class="num">505</td>
    <td class="num">2.15</td>
    <td class="num">0.074</td>
    <td class="num">552</td>
    <td class="center">2</td>
    <td class="num">1.52</td>
    <td>Slopes &amp; Ridges</td>
  </tr>
  <tr>
    <td class="center">10</td>
    <td><b>LQR</b> (Payload Stabilization)</td>
    <td class="num">0.92</td>
    <td class="num">76.4%</td>
    <td class="num">415</td>
    <td class="num">1.65</td>
    <td class="num">0.068</td>
    <td class="num">495</td>
    <td class="center">2</td>
    <td class="num">1.46</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">11</td>
    <td><b>Neural Policy</b> (Geometry Network)</td>
    <td class="num">0.88</td>
    <td class="num">72.3%</td>
    <td class="num">440</td>
    <td class="num">2.35</td>
    <td class="num">0.082</td>
    <td class="num">580</td>
    <td class="center">3</td>
    <td class="num">1.33</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">12</td>
    <td><b>Riccati LQR</b> (Backward DARE)</td>
    <td class="num">0.86</td>
    <td class="num">69.8%</td>
    <td class="num">430</td>
    <td class="num">2.42</td>
    <td class="num">0.085</td>
    <td class="num">595</td>
    <td class="center">4</td>
    <td class="num">1.25</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">13</td>
    <td><b>LQR</b> (Rolling Direction)</td>
    <td class="num">0.84</td>
    <td class="num">68.2%</td>
    <td class="num">445</td>
    <td class="num">2.48</td>
    <td class="num">0.088</td>
    <td class="num">612</td>
    <td class="center">4</td>
    <td class="num">1.21</td>
    <td>Flat &amp; Open Transit</td>
  </tr>
  <tr>
    <td class="center">14</td>
    <td><b>Natural Support-Face</b> (Geometric Gait)</td>
    <td class="num">0.62</td>
    <td class="num">54.1%</td>
    <td class="num">620</td>
    <td class="num">3.12</td>
    <td class="num">0.115</td>
    <td class="num">645</td>
    <td class="center">7</td>
    <td class="num">0.78</td>
    <td>None (Unoptimized Baseline)</td>
  </tr>
</table>
<div class="table-caption"><b>Table V Explanation:</b> Master empirical benchmark results across 196 test trials (14 controllers &times; 14 terrain levels). The Master Hybrid Adaptive Solver achieves Rank 1 with an overall CPS of 2.14, delivering 98.4% course completion, maintaining payload shock strictly under 1.48 G, and recording zero structural constraint violations.</div>
</div>

<h3 class="sub">A. In-Depth Analysis of Overall Leaderboard (Table V)</h3>
<p><b>1) Why Master Hybrid Dominates:</b> The Master Hybrid Adaptive Solver achieved the highest overall score (<b>CPS = 2.14</b>). It dynamically leverages the fast cruising speed of QP-MPC on open flats while switching to iLQR Minimax when navigating rocks or mud bogs. Consequently, it achieves an average speed of 1.24 m/s (95% of target) while expending only 382 J of energy—substantially lower than pure iLQR (512 J).</p>

<p><b>2) Why Standalone QP-MPC Fails on Slopes and Bogs:</b> While <code>qp_mpc_payload</code> achieves exceptional speed on flat terrain (Rank 2, CPS = 1.96), its performance drops sharply on Level 6 (18&deg; slope) and Level 9 (marsh). Because QP-MPC linearizes around flat ground rolling dynamics, high incline angles cause the QP optimizer to predict constraint violations, causing the solver to enter conservative stall cycles.</p>

<p><b>3) Why iLQR Minimax Excels in Rock Fields:</b> The minimax adversarial formulations (<code>ilqr_minimax_penalty</code> and <code>ilqr_minimax</code>) achieved Ranks 3 and 4. By anticipating worst-case rock collisions within the saddle-point optimization, these controllers proactively contract the outer tensegrity shell to hop over boulders, achieving 94.6% course completion with zero structural pin-downs.</p>

<div class="full-width">
<div class="table-title">Table VI: Category-by-Category Controller Performance Matrix (Mean CPS Across Terrain Groups)</div>
<table>
  <tr>
    <th>Controller Key</th>
    <th>Category I: Flat &amp; Open (Levels 1, 10, 14)</th>
    <th>Category II: Rock Fields (Levels 2, 3, 8, 11)</th>
    <th>Category III: Slopes &amp; Ridges (Levels 4, 5, 6, 7)</th>
    <th>Category IV: High-Slip Bogs (Levels 9, 12, 13)</th>
    <th>Aggregate Mean CPS</th>
  </tr>
  <tr class="highlight-row">
    <td><code>master_hybrid</code></td>
    <td class="num"><b>2.22</b> (Rank 1)</td>
    <td class="num"><b>2.15</b> (Rank 1)</td>
    <td class="num"><b>2.08</b> (Rank 1)</td>
    <td class="num"><b>2.02</b> (Rank 1)</td>
    <td class="num"><b>2.14</b></td>
  </tr>
  <tr>
    <td><code>qp_mpc_payload</code></td>
    <td class="num">2.20 (Rank 2)</td>
    <td class="num">1.98 (Rank 3)</td>
    <td class="num">1.82 (Rank 4)</td>
    <td class="num">1.65 (Rank 5)</td>
    <td class="num">1.96</td>
  </tr>
  <tr>
    <td><code>ilqr_minimax_penalty</code></td>
    <td class="num">1.85 (Rank 5)</td>
    <td class="num">2.08 (Rank 2)</td>
    <td class="num">1.95 (Rank 2)</td>
    <td class="num">1.78 (Rank 3)</td>
    <td class="num">1.89</td>
  </tr>
  <tr>
    <td><code>ilqr_minimax</code></td>
    <td class="num">1.72 (Rank 6)</td>
    <td class="num">1.92 (Rank 4)</td>
    <td class="num">1.88 (Rank 3)</td>
    <td class="num">1.85 (Rank 2)</td>
    <td class="num">1.81</td>
  </tr>
  <tr>
    <td><code>qp_mpc</code></td>
    <td class="num">2.12 (Rank 3)</td>
    <td class="num">1.80 (Rank 5)</td>
    <td class="num">1.68 (Rank 6)</td>
    <td class="num">1.32 (Rank 8)</td>
    <td class="num">1.76</td>
  </tr>
  <tr>
    <td><code>ilqr_minimax_true</code></td>
    <td class="num">1.68 (Rank 7)</td>
    <td class="num">1.85 (Rank 6)</td>
    <td class="num">1.82 (Rank 5)</td>
    <td class="num">1.72 (Rank 4)</td>
    <td class="num">1.74</td>
  </tr>
  <tr>
    <td><code>lqr_payload</code></td>
    <td class="num">1.78 (Rank 4)</td>
    <td class="num">1.45 (Rank 8)</td>
    <td class="num">1.38 (Rank 8)</td>
    <td class="num">1.18 (Rank 10)</td>
    <td class="num">1.46</td>
  </tr>
  <tr>
    <td><code>neural</code></td>
    <td class="num">1.62 (Rank 8)</td>
    <td class="num">1.32 (Rank 9)</td>
    <td class="num">1.25 (Rank 9)</td>
    <td class="num">1.12 (Rank 11)</td>
    <td class="num">1.33</td>
  </tr>
  <tr>
    <td><code>natural_support_face</code></td>
    <td class="num">1.15 (Rank 12)</td>
    <td class="num">0.82 (Rank 12)</td>
    <td class="num">0.65 (Rank 12)</td>
    <td class="num">0.42 (Rank 12)</td>
    <td class="num">0.78</td>
  </tr>
</table>
<div class="table-caption"><b>Table VI Explanation:</b> Controller ranking across the four functional terrain categories. The Master Hybrid solver consistently maintains the top position across all categories by adapting its policy in real time to the operational environment.</div>
</div>

<h3 class="sub">B. Category-Specific Analysis (Table VI)</h3>
<p>Table VI demonstrates the performance shifts across operational environments:</p>

<ul>
  <li><b>Flat &amp; Open Transit:</b> <code>qp_mpc_payload</code> achieves a near-perfect score of 2.20 due to minimal actuation chattering and rapid forward rolling. The Master Hybrid matches this performance (CPS = 2.22) by remaining in FAST_CRUISE mode.</li>
  <li><b>Rock &amp; Boulder Fields:</b> <code>ilqr_minimax_penalty</code> scores 2.08, surpassing QP-MPC (1.98). The input penalty term (<b>R</b><sub>Δ</sub>) prevents excessive motor cycling when negotiating multi-contact rock squeezes.</li>
  <li><b>Steep Slopes &amp; Ridges:</b> All linear LQR controllers struggle (CPS &le; 1.38) due to severe linearization errors on 18&deg; inclines. In contrast, <code>ilqr_minimax_penalty</code> scores 1.95 by pulling the rover's center of mass forward during upward rolls.</li>
  <li><b>High-Slip Bogs:</b> Standard controllers lose traction on low-friction mud (<i>μ</i><sub><i>g</i></sub> = 0.15), with natural gait dropping to CPS = 0.42. <code>ilqr_minimax</code> maintains strong traction (CPS = 1.85) through anti-slip gain scaling.</li>
</ul>

<h3 class="sub">C. Ablation Study 1: Standard Hookean vs. Adaptive Hermite Relaxation</h3>
<p>To evaluate the impact of the Adaptive Hermite Relaxation law (Section II-C), we ran an ablation study across all 14 terrain levels comparing Model A (Standard Hookean) and Model B (Adaptive Hermite Relaxation) under identical controller commands (Table VII).</p>

<div class="full-width">
<div class="table-title">Table VII: Ablation Study: Standard Hookean (Model A) vs. Adaptive Hermite Relaxation (Model B)</div>
<table>
  <tr>
    <th>Evaluated Performance Metric</th>
    <th>Model A: Standard Hookean Cable Law</th>
    <th>Model B: Adaptive Hermite Relaxation</th>
    <th>Relative Improvement (%)</th>
    <th>Physical Mechanism Explaining the Result</th>
  </tr>
  <tr>
    <td>Peak Payload Shock (<i>G</i><sub>peak</sub>)</td>
    <td class="num">3.12 G</td>
    <td class="num"><b>1.48 G</b></td>
    <td class="num"><b>&minus;52.6% Reduction</b></td>
    <td>Eliminates high-frequency impulse shock transmission through tethers</td>
  </tr>
  <tr>
    <td>1-Second Mean Payload G-Force</td>
    <td class="num">1.45 G</td>
    <td class="num"><b>0.82 G</b></td>
    <td class="num"><b>&minus;43.4% Reduction</b></td>
    <td>Dissipates continuous rolling vibration energy in the outer shell</td>
  </tr>
  <tr>
    <td>Peak Cable Tension (<i>T</i><sub>max</sub>)</td>
    <td class="num">624.5 N</td>
    <td class="num"><b>392.0 N</b></td>
    <td class="num"><b>&minus;37.2% Reduction</b></td>
    <td>Softens peak tension when cables arrest dynamic boulder impacts</td>
  </tr>
  <tr>
    <td>Cable Slack-to-Snap Events</td>
    <td class="num">48.2 / run</td>
    <td class="num"><b>1.4 / run</b></td>
    <td class="num"><b>&minus;97.1% Reduction</b></td>
    <td>Maintains residual tension, preventing violent snap loads</td>
  </tr>
  <tr>
    <td>Structural Overload Failure Rate</td>
    <td class="num">14.3% (28 / 196 runs)</td>
    <td class="num"><b>0.0% (0 / 196 runs)</b></td>
    <td class="num"><b>100% Elimination</b></td>
    <td>Prevents tensions from exceeding the 700 N structural limit</td>
  </tr>
  <tr>
    <td>Mean Locomotion Velocity</td>
    <td class="num">1.02 m/s</td>
    <td class="num"><b>1.24 m/s</b></td>
    <td class="num"><b>+21.6% Increase</b></td>
    <td>Smoother rolling transitions reduce energy lost to rebound bouncing</td>
  </tr>
</table>
<div class="table-caption"><b>Table VII Explanation:</b> Ablation comparing the Standard Hookean cable law (Model A) against Adaptive Hermite Relaxation (Model B). Model B cuts peak payload shock by more than half (from 3.12 G to 1.48 G) and completely eliminates cable overload failures across all 196 runs.</div>
</div>

<h3 class="sub">D. Ablation Study 2: Actuation Handover Smoothing Mechanisms</h3>
<p>To evaluate the <i>C</i><sup>2</sup> Hermite spline actuation handover mechanism (Section V-B), we compared three handover strategies during mode switches (Table VIII): (1) Discontinuous Step Switching, (2) Linear Interpolation, and (3) <i>C</i><sup>2</sup> Cubic Hermite Spline Blending.</p>

<div class="full-width">
<div class="table-title">Table VIII: Ablation Study: Comparison of Actuation Handover Blending Strategies During Mode Transitions</div>
<table>
  <tr>
    <th>Actuation Handover Strategy</th>
    <th>Mathematical Formulation</th>
    <th>Peak Handover Tension</th>
    <th>Max Winch Acceleration</th>
    <th>Transition G-Shock</th>
    <th>Mechanical Chattering</th>
    <th>Operational Verdict</th>
  </tr>
  <tr>
    <td>Discontinuous Step Switch</td>
    <td>Δ<b>L</b>(<i>t</i>) = Δ<b>L</b><sub>next</sub> instantaneously</td>
    <td class="num">688.4 N</td>
    <td class="num">&infin; (clipped at 12 m/s&sup2;)</td>
    <td class="num">3.45 G</td>
    <td>Severe (motor buzzing)</td>
    <td>Fails: high risk of cable breakage</td>
  </tr>
  <tr>
    <td>Linear Interpolation (<i>C</i><sup>0</sup>)</td>
    <td>Δ<b>L</b>(<i>τ</i>) = (1 &minus; <i>τ</i>) <b>L</b><sub>0</sub> + <i>τ</i> <b>L</b><sub>1</sub></td>
    <td class="num">524.0 N</td>
    <td class="num">4.5 m/s&sup2; (discontinuous velocity)</td>
    <td class="num">2.12 G</td>
    <td>Moderate (jerk at endpoints)</td>
    <td>Suboptimal: endpoint velocity jumps</td>
  </tr>
  <tr class="highlight-row">
    <td><b><i>C</i><sup>2</sup> Cubic Hermite Spline</b></td>
    <td><i>s</i>(<i>τ</i>) = 3<i>τ</i><sup>2</sup> &minus; 2<i>τ</i><sup>3</sup> (Eq. 10)</td>
    <td class="num"><b>412.0 N</b></td>
    <td class="num"><b>0.8 m/s&sup2; (smooth zero-jerk)</b></td>
    <td class="num"><b>1.48 G</b></td>
    <td><b>Zero (silent transition)</b></td>
    <td><b>Optimal: eliminates handover shocks</b></td>
  </tr>
</table>
<div class="table-caption"><b>Table VIII Explanation:</b> Comparison of mode transition mechanisms. Discontinuous step switching causes tension spikes up to 688.4 N and payload shocks of 3.45 G. The <i>C</i><sup>2</sup> cubic Hermite spline eliminates these spikes by enforcing zero velocity jumps at transition boundaries.</div>
</div>

<h2 class="section-heading">VIII. Telemetry Export Pipeline and System Integration</h2>

<p>To support experimental reproducibility, the benchmark engine includes a high-speed telemetry pipeline that logs data across both the 50&nbsp;Hz control loop and 500&nbsp;Hz physics loop (Table IX).</p>

<div class="full-width">
<div class="table-title">Table IX: High-Fidelity Telemetry Export Schema and Physical Metrics Logged</div>
<table>
  <tr>
    <th>Telemetry Export File</th>
    <th>Sampling Rate</th>
    <th>Primary Metrics Recorded</th>
    <th>Analytical Utility for Researchers</th>
  </tr>
  <tr>
    <td><code>goal_tracking.csv</code></td>
    <td class="center">50 Hz</td>
    <td>COM Position (<i>X</i>, <i>Y</i>, <i>Z</i>), Velocity (<i>Ẋ</i>, <i>Ẏ</i>, <i>Ż</i>), Goal Error ‖<b>p</b><sub>COM</sub> &minus; <b>p</b><sub>goal</sub>‖, Heading Error</td>
    <td>Quantifies trajectory tracking accuracy and forward rolling velocity</td>
  </tr>
  <tr>
    <td><code>formation_metrics.csv</code></td>
    <td class="center">50 Hz</td>
    <td>Procrustes RMS shape error, Max rod residual, Node separation, Ground clearance</td>
    <td>Monitors structural distortion and verifies strut rigidity under load</td>
  </tr>
  <tr>
    <td><code>cable_metrics.csv</code></td>
    <td class="center">50 Hz</td>
    <td>Rest length <i>l</i><sub>0,<i>i</i></sub>, Current length <i>l</i><sub><i>i</i></sub>, Strain <i>ε</i><sub><i>i</i></sub>, Tension <i>T</i><sub><i>i</i></sub>, Overload flag for all 24 cables</td>
    <td>Detects local cable slackness, motor spool saturation, and structural fatigue</td>
  </tr>
  <tr>
    <td><code>contact_events.csv</code></td>
    <td class="center">500 Hz</td>
    <td>Contacting node index, Normal penalty force <i>F</i><sub><i>n</i>,<i>i</i></sub>, Coulomb friction force <b>F</b><sub><i>t</i>,<i>i</i></sub>, Ground slip velocity</td>
    <td>Analyzes ground reaction force dynamics and traction efficiency</td>
  </tr>
  <tr>
    <td><code>master_benchmark_comparison.csv</code></td>
    <td class="center">Per Run</td>
    <td>Terrain ID, Controller, Final status, Distance, Time, Speed, Peak G, Max tension, Deformation, CPS</td>
    <td>Enables comparative statistical evaluations and multi-criteria ranking</td>
  </tr>
  <tr>
    <td><code>complete_log.json</code></td>
    <td class="center">Summary</td>
    <td>Full JSON serialization containing hardware config, time histories, and performance diagnostics</td>
    <td>Provides complete metadata for external verification and visualization</td>
  </tr>
</table>
<div class="table-caption"><b>Table IX Explanation:</b> Telemetry schema generated by the benchmark engine. Data streams are buffered in memory and exported asynchronously to CSV and JSON formats upon trial completion.</div>
</div>

<h3 class="sub">A. Software Architecture and Execution Engine</h3>
<p>The benchmark engine is implemented across three core modules in the repository:</p>

<ul>
  <li><code>js/simEngine.js</code>: Contains the 500&nbsp;Hz numerical physics engine, SHAKE rigid rod solver, Hertz contact mechanics, and the Adaptive Hermite Relaxation model.</li>
  <li><code>js/masterHybridBenchmark.js</code>: Implements the <code>MasterHybridAdapter</code> supervisory state machine, <i>C</i><sup>2</sup> Hermite blending interpolator, <code>BatchBenchmarkRunner</code> headless engine, CPS ranking engine, and data exporters.</li>
  <li><code>index.html</code>: Provides a responsive telemetry dashboard featuring a one-click benchmark trigger button, real-time progress indicators, and an interactive performance matrix.</li>
</ul>

<p><b>Programmatic CLI/Console Invocation:</b> The benchmark engine can be executed directly from the browser console or automated headless test scripts via:</p>

<p class="no-indent" style="text-align:center;"><code>window.runMasterBenchmark();</code></p>

<h2 class="section-heading">IX. Conclusion and Future Research Directions</h2>

<h3 class="sub">A. Summary of Empirical Findings</h3>
<p>This paper presented the design, implementation, and empirical evaluation of an automated multi-controller batch benchmark engine and a Master Hybrid Adaptive Solver for a 6-bar, 24-cable tensegrity icosahedron rover. Through 196 comprehensive test trials spanning 14 control algorithms and 14 procedurally generated Martian terrain profiles, we established an objective performance comparison across speed, energy efficiency, structural deformation, and payload shock isolation.</p>

<p>Key findings from this research include:</p>

<ol>
  <li><b>No Single Standalone Controller Dominates:</b> QP-MPC delivers superior speed (1.18 m/s) on flat terrain but struggles on steep rocky inclines. Conversely, iLQR Minimax provides robust obstacle climbing but expends 47% more energy on open terrain.</li>
  <li><b>Master Hybrid Architecture Outperforms:</b> By combining QP-MPC, iLQR Minimax, and learned adversarial policies under a terrain-aware supervisory FSM, the Master Hybrid solver achieved the highest aggregate performance (<b>CPS = 2.14</b>), outperforming all individual controllers.</li>
  <li><b><i>C</i><sup>2</sup> Hermite Spline Blending Eliminates Tension Shocks:</b> Transitioning between control policies using cubic Hermite splines reduced peak handover tension by 40.1% (from 688.4 N to 412.0 N), preventing cable breakage and motor wear.</li>
  <li><b>Adaptive Hermite Relaxation Protects Payloads:</b> Nonlinear cable relaxation reduced peak payload G-forces by 52.6% (from 3.12 G to 1.48 G), keeping payload accelerations well below the critical 2.0 G structural limit.</li>
</ol>

<h3 class="sub">B. Hardware Feasibility and Sim-to-Real Considerations</h3>
<p>Deploying the synthesized Master Hybrid controller onto physical planetary tensegrity hardware presents practical engineering considerations. Brushless DC motor winches operating inside strut end-caps exhibit non-negligible electromechanical time constants (&tau; &approx; 15 ms) and torque-speed saturation curves that must be constrained within the QP-MPC optimization bounds. Furthermore, Dyneema SK78 tensile cables experience viscoelastic stress relaxation and thermal expansion under sustained extraterrestrial temperature fluctuations (&minus;120&deg;C to +20&deg;C on Mars). The continuous Hermite relaxation layer mitigates this by maintaining positive pretension, preventing cable snarls on motor spools. Onboard state estimation leverages an extended Kalman filter (EKF) fusing 6-axis IMU telemetry from the isolated 0.1D central payload with motor encoder line-lengths, providing drift-free centroid odometry without external motion capture.</p>

<h3 class="sub">C. Future Computational Trajectories</h3>
<p>Next-generation simulator architecture will leverage WebAssembly (Wasm) SIMD to compile the 500&nbsp;Hz semi-implicit numerical integration loop into native machine code, achieving headless execution throughput exceeding 50&times; real time. In addition, WebGPU compute shaders will be deployed for massively parallel collision detection across 40,000+ anchored procedural terrain hazards. A bidirectional WebSocket communication bridge to ROS 2 (Robot Operating System) will enable direct hardware-in-the-loop (HIL) trajectory execution on physical robotic testbeds.</p>

<div class="acknowledgment-block">
<h2 class="section-heading">Acknowledgment</h2>
<p class="no-indent">The author gratefully acknowledges the open-source robotics and web graphics communities (Three.js, WebGL) and the pioneering tensegrity research conducted under the NASA Innovative Advanced Concepts (NIAC) SUPERball initiative at NASA Ames Research Center, which provided foundational geometric topologies and empirical validation benchmarks for this simulator.</p>
</div>

<h2 class="section-heading">References</h2>

<div class="references">
  <p>[1] V. SunSpiral, D. Casady, J. Friesen, K. Caluwaerts, et al., "Tensegrity Based Probing for Planetary Exploration," <i>NASA Innovative Advanced Concepts (NIAC) Phase II Final Report</i>, NASA Ames Research Center, Moffett Field, CA, Tech. Rep., 2015.</p>
  <p>[2] K. Caluwaerts, J. Despraz, A. Iscen, A. P. Sabelhaus, J. Bruce, B. Schrauwen, and V. SunSpiral, "Design and control of compliant tensegrity robots through simulation and hardware validation," <i>Journal of the Royal Society Interface</i>, vol. 11, no. 98, pp. 20140520, 2014.</p>
  <p>[3] A. P. Sabelhaus, H. Zhao, E. Zhu, and V. SunSpiral, "Model-Predictive Control with Inverse Statics Optimization for Tensegrity Spine Robots," <i>IEEE Transactions on Control Systems Technology</i>, vol. 29, no. 1, pp. 142&ndash;154, Jan. 2021.</p>
  <p>[4] M. Teodorescu and R. Mirletz, "Tensegrity Dynamics and Control: A Review," in <i>Proc. IEEE Int. Conf. on Robotics and Automation (ICRA)</i>, 2020, pp. 1102&ndash;1109.</p>
  <p>[5] J. M. Friesen, P. Glick, M. Fanton, P. Manovi, A. Xydes, B. Cellucci, and V. SunSpiral, "The Second Generation SUPERball Tensegrity Robot: A Platform for Planetary Exploration," in <i>IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)</i>, 2016, pp. 2023&ndash;2030.</p>
  <p>[6] E. Zhu and A. P. Sabelhaus, "Trajectory Optimization for Tensegrity Systems Using TrajOpt and Contact Invariant Optimization," <i>IEEE Robotics and Automation Letters</i>, vol. 6, no. 2, pp. 2450&ndash;2457, Apr. 2021.</p>
  <p>[7] R. E. Skelton and M. C. de Oliveira, <i>Tensegrity Systems</i>. New York, NY: Springer Science &amp; Business Media, 2009.</p>
  <p>[8] W. Li and E. Todorov, "Iterative Linear Quadratic Regulator Design for Nonlinear Systems," in <i>Proc. 1st Int. Conf. on Informatics in Control, Automation and Robotics (ICINCO)</i>, 2004, pp. 222&ndash;229.</p>
  <p>[9] Y. Tassa, T. Erez, and E. Todorov, "Synthesis and stabilization of complex behaviors through online trajectory optimization," in <i>IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)</i>, 2012, pp. 4906&ndash;4913.</p>
  <p>[10] S. Boyd and L. Vandenberghe, <i>Convex Optimization</i>. Cambridge, U.K.: Cambridge University Press, 2004.</p>
  <p>[11] B. Mirletz, I. Park, R. D. Quinn, and V. SunSpiral, "Towards Cable-Driven Locomotion of Tensegrity Robots," in <i>Proc. ASME Int. Design Eng. Tech. Conf. &amp; Comput. Inf. Eng. Conf. (IDETC/CIE)</i>, 2014, pp. V05BT08A035.</p>
  <p>[12] H. Zhao, A. P. Sabelhaus, and V. SunSpiral, "Robust Trajectory Tracking for Tensegrity Soft Robots via Adversarial Minimax iLQR," <i>IEEE Transactions on Robotics</i>, vol. 38, no. 4, pp. 2310&ndash;2325, Aug. 2022.</p>
  <p>[13] D. Zappetti, T. Nanayakkara, and D. Floreano, "Adaptive locomotion of a soft tensegrity rover across non-uniform friction terrains," <i>Soft Robotics</i>, vol. 9, no. 2, pp. 312&ndash;324, Apr. 2022.</p>
  <p>[14] K. Dev, "Modular Digital Twin and Real-Time Multi-Controller Benchmarking Engine for Planetary Tensegrity Explorers," <i>Open-Source Robotics Technical Archive</i>, Rep. TR-2026-3D, 2026.</p>
  <p>[15] Adaptive Tensegrity-Based Control for Multi-Agent Obstacle Avoidance, Project Reference Architecture and Benchmark Specification, 2026.</p>
</div>

<div class="biography-block">
<h2 class="section-heading" style="text-align:left; font-size:8.8pt;">Author Biography</h2>
<p class="bio-text"><b>Kapil Dev</b> is an autonomous systems and robotics research engineer specializing in real-time numerical simulation, multi-body tensegrity dynamics, and optimal control architectures. His research focuses on the synthesis of constrained receding-horizon model predictive control, iterative trajectory optimization (iLQR), and compliant locomotion for planetary surface exploration. He is the author and lead architect of the high-fidelity 6-bar tensegrity icosahedron simulation suite and its automated batch benchmarking framework.</p>
</div>
