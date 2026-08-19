function R = paper_reference_data()
%PAPER_REFERENCE_DATA  Values read out of the paper, for validation.
%
%   R = PAPER_REFERENCE_DATA()
%
%   Two kinds of reference values, kept strictly apart:
%
%   R.reported  : numbers printed in the paper (Table I, equations, text).
%   R.digitised : GRAPH-DERIVED ESTIMATES obtained by pixel-digitising the
%                 embedded raster panels of Figs. 5 and 6 (and reading Figs. 3
%                 and 4).  Calibration: the axis frame of each panel was located
%                 and mapped onto the tick ranges x in [-50,50], y in [-20,140];
%                 the calibration was verified against the known t = 0 grid
%                 (residual < 0.15 length units).  Agent markers were found by
%                 morphological closing + disc template matching (12 markers per
%                 panel, match scores 0.73 - 1.00).
%                 Uncertainty: centres +/- 0.15, radii +/- 0.1, centroid +/- 0.5.
%
%   Nothing in this file is used to drive the simulation; it is only compared
%   against simulation output in VALIDATE_RESULTS.

% ----------------------------------------------------------------- reported
R.reported.table1 = struct( ...
    'dt', 0.05, 'm', 1, 'u_nav', [0;3], 'c', 1.5, 'lS', 15, 'sigma', 0.5, ...
    'ry', 8, 'ka', 20, 'u_max', 10, 'kS', 0.0341, 'kB', -50, ...
    'alphaS', 2, 'alphaB', -0.5, 'z1', 15.5, 'z2', 50, 'beta', 8, 'gamma', 0.4);
R.reported.n_agents      = 12;
R.reported.tiles         = [2 3];
R.reported.d             = 2;
R.reported.snapshots     = [0 15 30 40];
R.reported.eq21_ratio    = -15^(2 - (-0.5)) * sqrt(2)^(1 - (-0.5));   % Eq. (21)

% ---------------------------------------------------------------- digitised
% obstacles [x y R]: mean over the 8 panels of Figs. 5-6
R.digitised.obstacles_measured = [  0.13  90.06  12.07
                                   29.91  49.96   6.65
                                  -29.59  29.96   6.61 ];
R.digitised.obstacles_adopted  = [  0     90     12.0
                                   30     50      6.5
                                  -30     30      6.5 ];
R.digitised.obstacle_radius_tol = 0.25;

% initial formation (Fig. 5(a) / 6(a))
R.digitised.grid_x = [-22.5 -7.5 7.5 22.5];
R.digitised.grid_y = [-15 0 15];
R.digitised.grid_tol = 0.2;

% centroid Y of the 12 agents in each panel
R.digitised.snapshots   = [0 15 30 40];
R.digitised.centroidY_fig5 = [ 0.08 33.28 73.40 86.72];   % without relaxation
R.digitised.centroidY_fig6 = [-0.01 32.74 74.52 104.88];  % with relaxation
R.digitised.centroidY_tol  = 0.5;

% full 12-agent position sets, sorted by descending Y then ascending X
R.digitised.q_fig5 = cat(3, ...
  [ -7.50 15.15; 7.51 15.14; -22.42 15.09; 22.32 15.05; -7.44 0.14; 7.36 0.10; ...
   -22.39 0.09; 22.30 0.04; -22.48 -14.91; 7.37 -14.96; 22.28 -14.98; -7.53 -14.98].', ...
  [ -5.08 49.44; -18.70 49.19; 7.54 48.57; 17.95 45.32; -16.64 34.25; -4.13 33.65; ...
    8.79 33.02; 22.87 32.65; -19.09 19.05; 23.70 18.42; 8.93 17.93; -5.12 17.88].', ...
  [-24.86 89.69; 24.26 87.25; -12.71 79.38; -26.65 78.40; 10.97 77.85; 23.82 75.57; ...
   -10.54 72.56; 6.74 71.66; -27.40 64.91; 20.73 62.07; -11.55 61.16; 5.22 60.33].', ...
  [-21.32 107.67; -24.14 100.47; 18.55 97.73; -28.73 93.34; 18.38 90.68; -16.19 89.01; ...
    19.40 83.03; -15.87 82.46; 10.48 77.50; -17.01 75.71; 4.14 74.58; 3.38 68.41].');

R.digitised.q_fig6 = cat(3, ...
  [  7.46 15.08; -7.48 15.06; 22.36 15.01; -22.38 15.00; 7.42 0.04; -7.44 0.03; ...
   -22.40 -0.01; 22.38 -0.02; -22.39 -15.05; -7.49 -15.08; 7.35 -15.10; 22.27 -15.11].', ...
  [ -5.06 49.10; -18.51 48.77; 7.77 47.95; 18.41 44.61; -16.44 33.80; -4.12 33.21; ...
    8.81 32.50; 23.10 32.18; -19.17 18.33; 23.56 17.81; -5.32 17.36; 8.79 17.32].', ...
  [-26.60 90.88; 26.19 88.02; -14.71 81.27; 14.18 80.76; -27.47 79.06; 24.80 75.14; ...
   -11.00 74.13; 8.26 73.09; -27.56 65.68; -11.63 62.48; 20.98 62.10; 5.52 61.57].', ...
  [-26.38 122.30; -9.26 118.21; 29.15 116.90; 12.15 116.12; -29.00 109.15; -12.45 105.65; ...
    13.95 103.26; 30.07 103.13; -33.24 94.75; -18.53 92.97; 18.07 88.73; 31.11 87.42].');

% Figure 3 / Figure 4 read-outs (string / bar force magnitude curves)
R.digitised.fig3 = struct( ...
    'f_at_z1',        8.19, ...   % = kS*z1^alphaS
    'peak_value',     13.5, ...   % visual peak of the relaxing curve
    'peak_location',  26,   ...   % ell at the peak
    'plateau',        8,    ...   % = beta
    'tol_value',      0.4, 'tol_location', 1.5);
R.digitised.fig4 = struct( ...
    'bar_at_z2',      7.07, ...   % |kB|*z2^alphaB = 50/sqrt(50)
    'string_at_60',   8, ...
    'bar_at_60',      6.45, ...
    'tol',            0.3);

% Qualitative claims (Sec. V) that the reproduction must exhibit
R.claims = { ...
 'Without string relaxation the formation gets hung up on the large obstacle.'; ...
 'With string relaxation the formation deforms its two centre units, keeps the edge units and passes.'; ...
 'Both behave similarly for the small deformations caused by the two small obstacles.'; ...
 'No obstacle information is exchanged between agents.'};
end
