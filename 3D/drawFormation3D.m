function drawFormation3D(ax, q, L, cfg, obstacles, activeObstacle, traj, t, vel, panelTitle, relaxedMembers)
%DRAWFORMATION3D  Render the 3-D tensegrity lander on solid brown COMSOL random terrain.
%
%   DRAWFORMATION3D(ax, q, L, cfg, obstacles, activeObstacle, traj, t, vel, panelTitle, relaxedMembers)
%
%   Renders:
%     1. Solid brown rough terrain from COMSOL Fourier spectral synthesis
%     2. Red spherical obstacles
%     3. Tensegrity bars (blue thick), strings (red dashed / orange when relaxed)
%     4. Black filled nodes, orange halos when near obstacle
%     5. Green centroid trajectory

if nargin < 11 || isempty(relaxedMembers)
    relaxedMembers = false(size(L.members, 1), 1);
end

cla(ax); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

% ---------------------------------------------------------------
% 1.  Solid brown COMSOL random rough terrain
% ---------------------------------------------------------------
if cfg.enableGround
    cx  = mean(q(1,:));
    cy  = mean(q(2,:));
    spanX = 3.5;
    spanY = 5.0;
    res   = 0.20;                                    % grid resolution [m]
    [Xg, Yg] = meshgrid(cx-spanX : res : cx+spanX, ...
                         cy-spanY : res : cy+spanY);

    % COMSOL Fourier surface evaluation (vectorised, analytic)
    Zg = evalSurface(Xg, Yg, cfg);

    % Solid brown / desert earth tone -- full opacity, slight lighting
    surf(ax, Xg, Yg, Zg, ...
         'FaceColor',     [0.62 0.42 0.22], ...   % rich earth brown
         'EdgeColor',     [0.40 0.26 0.12], ...   % darker brown edges
         'EdgeAlpha',     0.20, ...               % subtle edge lines
         'FaceAlpha',     1.00, ...               % fully opaque (no transparency)
         'SpecularStrength', 0.15, ...
         'AmbientStrength',  0.60, ...
         'DiffuseStrength',  0.80);
    lighting(ax, 'gouraud');
    light(ax, 'Position', [2 -3 6], 'Style', 'infinite');
end

% ---------------------------------------------------------------
% 2.  3-D spherical obstacles (bright red)
% ---------------------------------------------------------------
nO = size(obstacles, 1);
[sx, sy, sz] = sphere(28);
for o = 1:nO
    ox = obstacles(o, 1);
    oy = obstacles(o, 2);
    oz = obstacles(o, 3);
    R  = obstacles(o, 4);
    surf(ax, ox + R*sx, oy + R*sy, oz + R*sz, ...
         'FaceColor', [0.95 0.12 0.12], 'EdgeColor', 'none', 'FaceAlpha', 0.78);
end

% ---------------------------------------------------------------
% 3.  Tensegrity members (bars & strings)
% ---------------------------------------------------------------
for mm = 1:size(L.members, 1)
    i = L.members(mm, 1);
    j = L.members(mm, 2);
    px = [q(1,i), q(1,j)];
    py = [q(2,i), q(2,j)];
    pz = [q(3,i), q(3,j)];

    if ~L.isString(mm)
        % Compressive bar -- thick solid blue
        plot3(ax, px, py, pz, 'b-', 'LineWidth', 3.0);
    else
        if relaxedMembers(mm)
            % Relaxed string -- bright gold/orange (Eq. 19-20 active)
            plot3(ax, px, py, pz, '-', 'Color', [1.0 0.55 0.05], 'LineWidth', 2.2);
        else
            % Tensioned string -- red dashed
            plot3(ax, px, py, pz, 'r--', 'LineWidth', 1.2);
        end
    end
end

% ---------------------------------------------------------------
% 4.  Nodes and obstacle detection halos
% ---------------------------------------------------------------
for i = 1:L.n
    if activeObstacle(i)
        % Orange ring when node is in obstacle sensing range
        plot3(ax, q(1,i), q(2,i), q(3,i), 'o', ...
              'Color', [1 0.38 0], 'MarkerSize', 13, 'LineWidth', 2.5);
    end
    plot3(ax, q(1,i), q(2,i), q(3,i), 'ko', ...
          'MarkerFaceColor', 'k', 'MarkerSize', 5.5);
end

% ---------------------------------------------------------------
% 5.  Centroid trajectory (green)
% ---------------------------------------------------------------
if size(traj, 2) >= 2
    plot3(ax, traj(1,:), traj(2,:), traj(3,:), 'g-', 'LineWidth', 2.2);
end

% ---------------------------------------------------------------
% 6.  Axis labels, view, dynamic info
% ---------------------------------------------------------------
xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');

centroid = mean(q, 2);
if cfg.followRobot
    span = 3.0;
    xlim(ax, [centroid(1) - span, centroid(1) + span]);
    ylim(ax, [centroid(2) - span, centroid(2) + span]);
    zlim(ax, [centroid(3) - span*0.7, centroid(3) + span*1.1]);
end
view(ax, 44, 28);

nRelaxed = sum(relaxedMembers);
infoLine = sprintf('t = %.2f s | Y = %.1f m | Speed = %.2f m/s | Relaxed Strings = %d', ...
                   t, centroid(2), norm(vel), nRelaxed);
title(ax, {panelTitle, infoLine}, 'Interpreter', 'none');

end
