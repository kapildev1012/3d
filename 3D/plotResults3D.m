function plotResults3D(outA, outB, L, cfg)
%PLOTRESULTS3D  ECC-style comparison plots for the 3-D lander.
%
%   PLOTRESULTS3D(outA, outB, L, cfg)
%
%   Reproduces the ECC Fig. 5 vs Fig. 6 comparison in 3-D:
%     Fig 1: 3-D centroid trajectories on solid brown COMSOL random terrain
%     Fig 2: Formation error / relaxed strings / clearance (deform->reform)
%     Fig 3: Member strain histories (Case B only)

hasA = ~isempty(outA);
hasB = ~isempty(outB);

if hasA
    metA = formationMetrics3D(outA, L, cfg);
    defA = deformationAnalysis3D(outA, L, cfg);
end
if hasB
    metB = formationMetrics3D(outB, L, cfg);
    defB = deformationAnalysis3D(outB, L, cfg);
end

nO = size(cfg.obstacles, 1);

% ================================================================
% Figure 1: 3-D Trajectory on Solid Brown COMSOL Random Terrain
% ================================================================
fig1 = figure('Name','3-D Trajectory on COMSOL Random Terrain', ...
              'Color','w','Position',[50 50 860 660]);
ax1  = axes(fig1);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');

if cfg.enableGround
    y_end = cfg.obstacles(end,2) + 14;
    [Xg,Yg] = meshgrid(-4.5:0.25:4.5, 0:0.25:y_end);
    Zg = evalSurface(Xg, Yg, cfg);
    % Solid brown terrain -- fully opaque, Gouraud lit
    surf(ax1, Xg, Yg, Zg, ...
         'FaceColor',       [0.62 0.42 0.22], ...
         'EdgeColor',       [0.38 0.24 0.10], ...
         'EdgeAlpha',       0.10, ...
         'FaceAlpha',       1.00, ...
         'SpecularStrength', 0.10, ...
         'AmbientStrength',  0.55);
    lighting(ax1,'gouraud');
    light(ax1,'Position',[4 -6 10],'Style','infinite');
end

% Obstacles
[sx,sy,sz] = sphere(28);
for o = 1:nO
    ox=cfg.obstacles(o,1); oy=cfg.obstacles(o,2);
    oz=cfg.obstacles(o,3); R=cfg.obstacles(o,4);
    surf(ax1, ox+R*sx, oy+R*sy, oz+R*sz, ...
         'FaceColor',[0.95 0.10 0.10],'EdgeColor','none','FaceAlpha',0.70);
end

% Trajectories
if hasA
    plot3(ax1, metA.centroid(1,:), metA.centroid(2,:), metA.centroid(3,:), ...
          'b-','LineWidth',2.5,'DisplayName','Case A – No Relaxation (rigid)');
end
if hasB
    plot3(ax1, metB.centroid(1,:), metB.centroid(2,:), metB.centroid(3,:), ...
          'r-','LineWidth',2.5,'DisplayName','Case B – Adaptive (deforms & reforms)');
end

xlabel(ax1,'X [m]'); ylabel(ax1,'Y [m]'); zlabel(ax1,'Z [m]');
title(ax1, {'3-D Lander: Centroid Trajectory on COMSOL Random Terrain', ...
            'Brown = solid terrain | Red = obstacles | Blue/Red lines = Case A/B'}, ...
    'Interpreter','none');
legend(ax1,'Location','best');
view(ax1,38,24);

% ================================================================
% Figure 2: Deform -> Reform analysis (4 subplots, ECC style)
% ================================================================
fig2 = figure('Name','Deform -> Reform Analysis (ECC Fig.5 vs Fig.6)', ...
              'Color','w','Position',[110 110 860 760]);

% Obstacle Y centres for vertical markers
obs_Y = cfg.obstacles(:,2);

% Helper: shade obstacle encounter bands on an axes
shadeBands = @(ax_h, yB) deal(ax_h, yB);  % placeholder, done inline below

%-- Subplot 1: Formation Error (THE key plot showing deform->reform) --%
sp1 = subplot(4,1,1); hold on; grid on;

% Shade obstacle encounter zones (pink)
yl_pre = [0, max([hasA*max(defA.formation_error), hasB*max(defB.formation_error)])*1.15 + 0.01];
for o = 1:nO
    band_lo = obs_Y(o) - cfg.obstacles(o,4) - cfg.ry;
    band_hi = obs_Y(o) + cfg.obstacles(o,4) + cfg.ry;
    if hasB
        yB = metB.centroid(2,:);
        t_in  = metB.t(find(yB >= band_lo, 1,'first'));
        t_out = metB.t(find(yB >= band_hi, 1,'first'));
        if ~isempty(t_in) && ~isempty(t_out)
            fill([t_in t_out t_out t_in], [yl_pre(1) yl_pre(1) yl_pre(2) yl_pre(2)], ...
                 [1 0.75 0.75],'FaceAlpha',0.35,'EdgeColor','none','HandleVisibility','off');
        end
    end
end

if hasA
    plot(metA.t, defA.formation_error, 'b-','LineWidth',1.8, ...
         'DisplayName','Case A (No Relax) — rigid');
end
if hasB
    plot(metB.t, defB.formation_error, 'r-','LineWidth',1.8, ...
         'DisplayName','Case B (Adaptive) — deforms & reforms');
end
ylabel('Formation Error [m]','FontWeight','bold');
title({'Formation Error = Procrustes RMS node displacement', ...
       'Pink zones = obstacle encounter  |  Error drops back = REFORMED'}, ...
    'Interpreter','none');
legend('Location','northwest');
xlim([0, cfg.T_end]);

%-- Subplot 2: Relaxed strings (0 in free terrain = full reform) --%
subplot(4,1,2); hold on; grid on;
if hasA
    plot(metA.t, metA.n_relaxed,'b-','LineWidth',1.5, ...
         'DisplayName','Case A (0 always — rigid)');
end
if hasB
    plot(metB.t, metB.n_relaxed,'r-','LineWidth',1.5, ...
         'DisplayName','Case B (relaxes at obstacle, drops to 0 = reformed)');
end
ylabel('Relaxed Strings (#)','FontWeight','bold');
title('Active Relaxed Strings (Eqs.19-20)  —  0 = fully reformed');
ylim([-0.5, 25]); legend('Location','northwest');
xlim([0, cfg.T_end]);

%-- Subplot 3: Clearance --%
subplot(4,1,3); hold on; grid on;
if hasA
    plot(metA.t, metA.min_clearance,'b-','LineWidth',1.5,'DisplayName','Case A');
end
if hasB
    plot(metB.t, metB.min_clearance,'r-','LineWidth',1.5,'DisplayName','Case B');
end
yline(0,'k--','LineWidth',1.2,'DisplayName','Obstacle surface');
ylabel('Min Clearance [m]','FontWeight','bold');
title('Minimum Obstacle Clearance  (Case B maintains wider clearance)');
legend('Location','southwest');
xlim([0, cfg.T_end]);

%-- Subplot 4: Centroid Y vs time (shows forward progress) --%
subplot(4,1,4); hold on; grid on;
if hasA
    plot(metA.t, metA.centroid(2,:),'b-','LineWidth',1.5,'DisplayName','Case A');
end
if hasB
    plot(metB.t, metB.centroid(2,:),'r-','LineWidth',1.5,'DisplayName','Case B');
end
for o = 1:nO
    yline(cfg.obstacles(o,2),'k--','LineWidth',1.0, ...
          'DisplayName',sprintf('Obstacle %d',o),'HandleVisibility', 'on');
end
xlabel('Time [s]','FontWeight','bold');
ylabel('Centroid Y [m]','FontWeight','bold');
title('Forward Progress (Y) — lander passes both obstacles');
legend('Location','northwest');
xlim([0, cfg.T_end]);

% ================================================================
% Figure 3: Member Strain Histories (Case B — adaptive)
% ================================================================
if hasB
    figure('Name','Member Strains (Case B adaptive)','Color','w', ...
           'Position',[180 180 800 560]);

    subplot(2,1,1); hold on; grid on;
    plot(defB.strain_bars','b-','LineWidth',0.9);
    yline(0,'k--','LineWidth',0.9);
    ylabel('Bar Strain  (l-l0)/l0','FontWeight','bold');
    title('Case B: 6 Compressive Bar Strains (deform around obstacles)');
    xlim([1, size(defB.strain_bars,2)]);

    subplot(2,1,2); hold on; grid on;
    plot(defB.strain_strings','r-','LineWidth',0.7);
    yline(0,'k--','LineWidth',0.9);
    % Mark relaxation onset threshold
    z1_strain = cfg.z1 / sqrt(3/8) - 1;
    yline(z1_strain,'m--','LineWidth',1.4,'DisplayName', ...
          sprintf('Relaxation onset z_1 (strain=%.3f)',z1_strain));
    xlabel('Time Step','FontWeight','bold');
    ylabel('String Strain  (l-l0)/l0','FontWeight','bold');
    title({'Case B: 24 Adaptive String Strains', ...
           'Strings cross z_1 (magenta) -> relax -> recover below z_1 after obstacle'}, ...
        'Interpreter','none');
    legend('Location','northwest');
    xlim([1, size(defB.strain_strings,2)]);
end

end
