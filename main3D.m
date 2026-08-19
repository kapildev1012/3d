% MAIN3D  Run the 3-D Tensegrity Lander obstacle avoidance extension.
%
% This script builds the 3-D tensegrity lander and runs two simulations
% side-by-side across continuous uneven 3-D terrain with multiple obstacles:
%   Case A: No string relaxation (rigid tensegrity baseline)  -- Fig. 5 analog
%   Case B: Adaptive string relaxation (deforming & reforming) -- Fig. 6 analog
%
% The behaviour directly mirrors the ECC 2026 paper's 2-D comparison:
%   - Case A: formation approaches obstacle, gets stuck / deflected (rigid)
%   - Case B: formation DEFORMS around obstacle (strings relax, Eqs.19-20),
%             then REFORMS after passing it (formation error drops back to baseline)
%
% Following:
%   B. Ingalls, Q. Nelson, L. R. Garcia Carrillo, M. Majji,
%   "Adaptive Tensegrity-Based Control for Multi-Agent Obstacle Avoidance,"
%   2026 European Control Conference (ECC).

close all; clc;

addpath(fullfile(pwd, '3D'));
addpath(fullfile(pwd, 'paper_reproduction', 'utils'));

fprintf('=================================================================\n');
fprintf(' 3-D TENSEGRITY LANDER: ADAPTIVE OBSTACLE AVOIDANCE ON TERRAIN\n');
fprintf(' (3-D extension of ECC 2026 Fig. 5 vs Fig. 6 comparison)\n');
fprintf('=================================================================\n');

%% 1. Configuration
cfg = config3D();
fprintf('Terrain  : COMSOL Fourier random surface (RMS=%.2f m, %d modes)\n', ...
    cfg.groundRMS, (2*cfg.groundM+1)*(2*cfg.groundN+1)-1);
fprintf('Obstacles: %d spherical, radii [%.2f, %.2f] m at Y=[%.1f, %.1f] m\n', ...
    size(cfg.obstacles,1), cfg.obstacles(1,4), cfg.obstacles(2,4), ...
    cfg.obstacles(1,2), cfg.obstacles(2,2));
fprintf('Nav input: [%.2f, %.2f, %.2f] N  |  T_end = %.0f s\n', ...
    cfg.u_nav(1), cfg.u_nav(2), cfg.u_nav(3), cfg.T_end);
fprintf('Relaxation onset z1 = %.4f m (nominal l_S0 = %.4f m, ratio=%.2f)\n', ...
    cfg.z1, sqrt(3/8), cfg.z1/sqrt(3/8));

%% 2. Build lander & verify exact prestress equilibrium
L = buildLander3D(cfg);
nz0 = sampleNoise3D(L, cfg);
nz0.mem(:) = 0; nz0.mem_j(:) = 0; nz0.obs(:) = 0;
nz0.agt(:) = 0; nz0.process(:) = 0;
ut0 = internalForce3D(L.q0, L, cfg, nz0);
fprintf('Prestress self-stress residual: %.4e N (machine-precision Eq.21)\n\n', ...
    max(abs(ut0(:))));

%% 3. Case A: No String Relaxation (Fig. 5 analog — rigid baseline)
fprintf('--- Case A: No Relaxation (rigid baseline, ECC Fig. 5 analog) ---\n');
cfgA = cfg; cfgA.relaxation = false;
rng(cfg.seed, 'twister');
tA  = tic; outA = simulate3D(cfgA, L); wallA = toc(tA);
fprintf('  Finished %.2f s wall-time | Diverged: %d | Final Y: %.2f m\n\n', ...
    wallA, outA.diverged, outA.centroid(2,end));

%% 4. Case B: Adaptive String Relaxation (Fig. 6 analog — proposed method)
fprintf('--- Case B: Adaptive Relaxation (proposed method, ECC Fig. 6 analog) ---\n');
cfgB = cfg; cfgB.relaxation = true;
rng(cfg.seed, 'twister');
tB  = tic; outB = simulate3D(cfgB, L); wallB = toc(tB);
fprintf('  Finished %.2f s wall-time | Diverged: %d | Final Y: %.2f m\n\n', ...
    wallB, outB.diverged, outB.centroid(2,end));

%% 5. Deformation & Recovery Diagnostics (ECC paper comparison)
fprintf('=================================================================\n');
fprintf(' DEFORMATION  ->  REFORM  DIAGNOSTIC  (mirrors ECC Fig. 5 vs 6)\n');
fprintf('=================================================================\n');

metA = formationMetrics3D(outA, L, cfgA);
metB = formationMetrics3D(outB, L, cfgB);
defA = deformationAnalysis3D(outA, L, cfgA);
defB = deformationAnalysis3D(outB, L, cfgB);

% --- Print formation error at key Y positions (like a snapshot table) ---
y_checkpoints = [2, 8, 10, 12, 15, 20, 24, 26, 28, 32, 38];
fprintf('\nFormation Error (Procrustes RMS) at key Y-positions:\n');
fprintf('  %-8s  %-16s  %-16s\n', 'Y [m]', 'Case A (rigid)', 'Case B (adaptive)');
fprintf('  %-8s  %-16s  %-16s\n', '------', '--------------', '----------------');
for yc = y_checkpoints
    [~, idxA] = min(abs(outA.centroid(2,:) - yc));
    [~, idxB] = min(abs(outB.centroid(2,:) - yc));
    eA = defA.formation_error(idxA);
    eB = defB.formation_error(idxB);
    flag = '';
    if yc == cfg.obstacles(1,2), flag = '  <- Obstacle 1 centre'; end
    if yc == cfg.obstacles(2,2), flag = '  <- Obstacle 2 centre'; end
    if yc > cfg.obstacles(1,2) && yc < cfg.obstacles(2,2) - 2, flag = '  [between obs -- reform zone]'; end
    fprintf('  Y=%-6.1f  %.4f m        %.4f m       %s\n', yc, eA, eB, flag);
end

fprintf('\nSummary:\n');
fprintf('  Peak deformation  Case A: %.4f m | Case B: %.4f m\n', ...
    max(defA.formation_error), max(defB.formation_error));
fprintf('  Recovery pct (terrain-relative)  Case A: %.1f%% | Case B: %.1f%%\n', ...
    defA.recovery_pct, defB.recovery_pct);
fprintf('  Inter-obstacle recovery           Case A: %.1f%% | Case B: %.1f%%\n', ...
    defA.recovery_pct_obs1, defB.recovery_pct_obs1);
fprintf('  Min obstacle clearance Case A: %.4f m | Case B: %.4f m\n', ...
    min(metA.min_clearance), min(metB.min_clearance));
fprintf('  Peak relaxed strings  Case A: %d (rigid) | Case B: %d (adaptive)\n', ...
    max(metA.n_relaxed), max(metB.n_relaxed));
fprintf('=================================================================\n\n');

%% 6. Figures & Animation
fprintf('--- Generating Plots & Live 3-D Animation ---\n');
plotResults3D(outA, outB, L, cfg);
animate3D(outA, outB, L, cfg);

fprintf('\nSimulation complete.\n');
fprintf('MATLAB figures show:\n');
fprintf('  Fig 1: 3-D trajectory on solid-brown COMSOL random terrain\n');
fprintf('  Fig 2: Formation error [Deform->Reform], relaxed strings, clearance\n');
fprintf('  Fig 3: Member strain histories (Case B)\n');
fprintf('  Animation: Side-by-side Case A (rigid) vs Case B (deform & reform)\n');
