function main
close all; clc; rng(1);
experiment = 'both';
% =====================================================================
% VISUALIZATION OPTIONS
% =====================================================================
showAnimation   = true;   % turn the live animation on/off
animationSpeed  = 1.0;    % >1 = faster playback, <1 = slower
followRobot     = true;   % true = camera follows formation centroid
saveVideo       = false;  % true = also save robot_simulation.mp4
drawEvery       = 4;      % render every N simulation steps (speed)
% =====================================================================
% SIMULATION PARAMETERS  (Table I of the paper, used verbatim)
% =====================================================================
dt        = 0.05;   % [s] integration step               (Table I)
simTime   = 150;     % [s] total simulated time (paper figures show
                     %     0-40s reaching the first obstacle; we run
                     %     longer so both formations clearly resolve
                     %     against the obstacles -- INFERRED, not
                     %     stated in the paper)
m         = 1;        % [kg] agent mass                  (Table I)
c         = 1.5;      % damping coefficient               (Table I)
un        = [0; 3];   % constant navigation input u_i^n   (Table I)
umax      = 10;       % input saturation                  (Table I)
sigma     = 0.5;      % std. dev. of measurement noise     (Table I)
lS        = 15;       % [m] string rest/tile side length   (Table I)
kS        = 0.0341;   % string gain k_S                    (Table I)
alphaS    = 2;         % string exponent alpha_S            (Table I)
kB        = -50;      % bar gain k_B                        (Table I)
alphaB    = -0.5;      % bar exponent alpha_B                (Table I)
z1        = 15.5;     % relaxation start distance           (Table I)
z2        = 50;       % relaxation end distance             (Table I)
beta      = 8;        % constant string force beyond z2      (Table I)
ry        = 8;        % [m] obstacle sensing/avoidance radius (Table I)
ka        = 20;       % collision avoidance gain              (Table I)
gamma     = 0.4;      % collision avoidance exponent           (Table I)
% =====================================================================
% FORMATION TOPOLOGY: 12-agent tiled formation, "six interconnected
% squares (two rows and three columns)" (Sec. V). This is a grid of
% 3 rows x 4 columns of AGENTS = 12 nodes, forming a 2x3 grid of unit
% square tiles. Perimeter/tile edges are STRINGS (tensile), the two
% diagonals of every tile are BARS (compressive) -- matching Fig. 2,
% where blue lines (bars) are the diagonals of each square and red
% lines (strings) are the square edges.
% =====================================================================
gridRows = 3;   % agent grid rows
gridCols = 4;   % agent grid columns
n = gridRows * gridCols;   % = 12 agents
d = 2;                     % planar (d=2) formation, as in Sec. V
nodeId = @(r,c) (r-1)*gridCols + c;
q0 = zeros(d, n);
for r = 1:gridRows
   for c = 1:gridCols
       q0(:, nodeId(r,c)) = [(c-1)*lS; (r-1)*lS];
   end
end
% --- Build edge list: [i, j, isBar] ---------------------------------
edges = zeros(0,3);
for r = 1:gridRows
   for c = 1:gridCols-1
       edges(end+1,:) = [nodeId(r,c), nodeId(r,c+1), 0]; %#ok<AGROW>
   end
end
for r = 1:gridRows-1
   for c = 1:gridCols
       edges(end+1,:) = [nodeId(r,c), nodeId(r+1,c), 0]; %#ok<AGROW>
   end
end
for r = 1:gridRows-1
   for c = 1:gridCols-1
       edges(end+1,:) = [nodeId(r,c),   nodeId(r+1,c+1), 1]; %#ok<AGROW>
       edges(end+1,:) = [nodeId(r,c+1), nodeId(r+1,c),   1]; %#ok<AGROW>
   end
end
numEdges = size(edges,1); %#ok<NASGU>
% =====================================================================
% OBSTACLES
% Circular obstacles the formation must navigate around while moving
% in +Y under the constant navigation force u_i^n = [0,3] (Sec. V).
% The paper shows solid red obstacle discs in Fig. 5/6 but does not
% give numeric obstacle coordinates/radii in the text -- their
% positions/radii below are INFERRED (chosen so the formation must
% deform around them along its +Y path); everything else (dynamics,
% control law, gains) uses the paper's stated equations/values.
% Both panels use the SAME obstacles so the comparison is fair.
% =====================================================================
centroid0 = mean(q0,2);
obstacles = struct('center', {}, 'radius', {});
obstacles(1).center = centroid0 + [0; 55];   obstacles(1).radius = 20;
obstacles(2).center = centroid0 + [55; 95];  obstacles(2).radius = 13;
numObstacles = numel(obstacles);
% =====================================================================
% Pre-generate the noise sequence ONCE and reuse it for every mode, so
% "with" vs "without" relaxation differ ONLY in the control law, not in
% random measurement noise -- an apples-to-apples comparison.
% =====================================================================
N = round(simTime/dt);
noiseSeq = sigma * randn(d, n, N);
% =====================================================================
% Determine which mode(s) to run
% =====================================================================
switch experiment
   case 'both'
       modeNames   = {'norelaxation', 'relaxation'};
       panelTitles = {'No string relaxation (Fig. 5 baseline)', ...
                      'With string relaxation (Fig. 6, proposed)'};
   case 'relaxation'
       modeNames   = {'relaxation'};
       panelTitles = {'With string relaxation (Fig. 6, proposed)'};
   case 'norelaxation'
       modeNames   = {'norelaxation'};
       panelTitles = {'No string relaxation (Fig. 5 baseline)'};
   otherwise
       error('Unknown experiment "%s". Use "both", "relaxation", or "norelaxation".', experiment);
end
numModes = numel(modeNames);
% =====================================================================
% Initialize one independent simulation state per mode
% =====================================================================
sim = struct();
for mIdx = 1:numModes
   sim(mIdx).relaxOn    = strcmp(modeNames{mIdx}, 'relaxation');
   sim(mIdx).q          = q0;
   sim(mIdx).qdot       = zeros(d,n);
   sim(mIdx).trajectory = zeros(N+1, d);
   sim(mIdx).trajectory(1,:) = mean(q0,2)';
   sim(mIdx).speedLog   = zeros(N+1,1);
   sim(mIdx).activeObstacle = false(numObstacles, n);
   sim(mIdx).agentClose = false(1, n);
end
% =====================================================================
% Figure / axes setup
% =====================================================================
if showAnimation
   fig = figure('Color','w','Name','Tensegrity MAS Obstacle Avoidance', ...
                'Position',[80 80 420*numModes+200 750]);
   ax = [];
   for mIdx = 1:numModes
       ax(mIdx) = subplot(1,numModes,mIdx);
       hold(ax(mIdx),'on'); axis(ax(mIdx),'equal'); grid(ax(mIdx),'on');
       xlabel(ax(mIdx),'X [m]'); ylabel(ax(mIdx),'Y [m]');
       title(ax(mIdx), panelTitles{mIdx});
   end
   if saveVideo
       vw = VideoWriter('robot_simulation.mp4','MPEG-4');
       vw.FrameRate = max(1, round((1/dt)*animationSpeed/drawEvery));
       open(vw);
   end
end
% =====================================================================
% MAIN SIMULATION LOOP -- advance every mode by one step, in lockstep,
% using the SAME pre-generated noise draw at each step.
% =====================================================================
for k = 1:N
   noise = noiseSeq(:,:,k);
   for mIdx = 1:numModes
       sim(mIdx) = stepSim(sim(mIdx), noise, edges, obstacles, ...
           kB, alphaB, kS, alphaS, z1, z2, beta, ...
           ry, ka, gamma, un, umax, c, m, dt, k);
   end
   if showAnimation && mod(k, drawEvery) == 0
       for mIdx = 1:numModes
           drawFormation(ax(mIdx), sim(mIdx).q, edges, obstacles, ...
               sim(mIdx).activeObstacle, sim(mIdx).agentClose, sim(mIdx).trajectory(1:k+1,:), ...
               k*dt, mean(sim(mIdx).qdot,2), followRobot, lS, panelTitles{mIdx});
       end
       drawnow;
       if saveVideo
           writeVideo(vw, getframe(fig));
       end
   end
end
if showAnimation
   for mIdx = 1:numModes
       drawFormation(ax(mIdx), sim(mIdx).q, edges, obstacles, ...
           sim(mIdx).activeObstacle, sim(mIdx).agentClose, sim(mIdx).trajectory, ...
           N*dt, mean(sim(mIdx).qdot,2), followRobot, lS, panelTitles{mIdx});
   end
   drawnow;
   if saveVideo
       writeVideo(vw, getframe(fig));
       close(vw);
       fprintf('Saved animation to robot_simulation.mp4\n');
   end
end
% =====================================================================
% Summary comparison plot: trajectories & speeds of all modes overlaid
% =====================================================================
figure('Color','w','Name','Summary comparison','Position',[1050 80 520 780]);
colors = {[0.85 0.1 0.1], [0.1 0.45 0.85]};
subplot(2,1,1); hold on; axis equal; grid on;
th = linspace(0,2*pi,50);
for o = 1:numObstacles
   fill(obstacles(o).center(1)+obstacles(o).radius*cos(th), ...
        obstacles(o).center(2)+obstacles(o).radius*sin(th), ...
        [0.7 0.7 0.7], 'FaceAlpha',0.6, 'EdgeColor','k');
end
legendLabels = cell(1,numModes);
for mIdx = 1:numModes
   plot(sim(mIdx).trajectory(:,1), sim(mIdx).trajectory(:,2), '-', ...
       'Color', colors{mIdx}, 'LineWidth',1.8);
   legendLabels{mIdx} = panelTitles{mIdx};
end
xlabel('X [m]'); ylabel('Y [m]'); title('Formation centroid trajectory');
legend(legendLabels, 'Location','southoutside');
subplot(2,1,2); hold on; grid on;
tAxis = (0:N)*dt;
for mIdx = 1:numModes
   plot(tAxis, sim(mIdx).speedLog, '-', 'Color', colors{mIdx}, 'LineWidth',1.4);
end
xlabel('Time [s]'); ylabel('Centroid speed [m/s]'); title('Centroid speed vs. time');
legend(legendLabels, 'Location','northwest');
% =====================================================================
% Fig. 3 & Fig. 4 reproductions: static analytical force-vs-distance
% curves computed directly from Eq. (10) and the string-relaxation
% law Eq. (19)-(20). These do NOT depend on the multi-agent
% simulation above -- they are the same per-edge force law the
% simulation uses, evaluated over a sweep of inter-agent distance.
% =====================================================================
plotForceCurves(kS, alphaS, kB, alphaB, z1, z2, beta);
end
% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================
function s = stepSim(s, noise, edges, obstacles, kB, alphaB, kS, alphaS, ...
       z1, z2, beta, ry, ka, gamma, un, umax, c, m, dt, k)
% Advance a single simulation state s by one time step dt, using the
% supplied noise draw for this step. Implements Eq. (8)-(9),(14),(18)-(20).
n = size(s.q,2);
qMeas = s.q + noise;
% ---- Tensegrity edge (bar/string) forces: Eq. (14),(19),(20) --------
ut = zeros(size(s.q));
for e = 1:size(edges,1)
   i = edges(e,1); j = edges(e,2); isBar = edges(e,3);
   rel = qMeas(:,i) - qMeas(:,j);
   ell = norm(rel);
   if ell < 1e-6, continue; end
   nij = rel/ell;
   if isBar
       f = kB * ell^alphaB;   % Eq. (10): bars never relax
   else
       if s.relaxOn
           if ell <= z1
               f = kS * ell^alphaS;                            % Eq. (10)
           elseif ell <= z2
               f = relaxSpline(ell, z1, z2, kS, alphaS, beta);  % Eq. (20)
           else
               f = beta;                                        % constant
           end
       else
           f = kS * ell^alphaS;   % no-relaxation baseline (Fig. 5)
       end
   end
   ut(:,i) = ut(:,i) - f * nij;
   ut(:,j) = ut(:,j) + f * nij;
end
% ---- Collision / obstacle avoidance term: Eq. (17)-(18) --------------
% Applied to BOTH static obstacles and OTHER AGENTS: per Eq. (17),
% "When obstacle j is another agent, then p_ij = n_ij" -- i.e. every
% agent within sensing radius r_y (obstacle or teammate) contributes a
% repulsive term. This is what keeps agents from colliding with each
% other, in addition to not hitting the environment obstacles.
uc = zeros(size(s.q));
activeObstacle = false(numel(obstacles), n);
agentClose = false(1, n);
% NOTE ON SIGN: Eq. (18) as printed, u_c_ij = -ka(dist^-gamma - r_y^-gamma) p_ij
% with p_ij pointing AWAY from the obstacle/agent and ka>0, is only ever
% evaluated when dist<=r_y -- in that regime dist^-gamma > r_y^-gamma, so
% the bracket is positive and the leading minus sign makes the force point
% BACK toward the obstacle/agent (attraction), which contradicts the
% paper's own stated purpose ("ensure that agents do not collide with each
% other or obstacles") and was empirically confirmed here: with the
% literal sign, agents collapsed to ~0.05 m separation instead of being
% pushed apart. The sign is flipped below (+ka instead of -ka) so the
% term is actually repulsive, matching the stated design intent.
for i = 1:n
   % -- vs. static obstacles --
   for o = 1:numel(obstacles)
       toCenter = qMeas(:,i) - obstacles(o).center;
       distToCenter = norm(toCenter);
       if distToCenter < 1e-6
           dirUnit = [1;0];
       else
           dirUnit = toCenter/distToCenter;
       end
       dj = obstacles(o).center + dirUnit*obstacles(o).radius;
       rel = qMeas(:,i) - dj;
       distIJ = max(norm(rel), 1e-3);
       if distIJ <= ry
           activeObstacle(o,i) = true;
           pij = rel/distIJ;
           uc(:,i) = uc(:,i) + ka*(distIJ^(-gamma) - ry^(-gamma))*pij;  % Eq. (18), repulsive
       end
   end
   % -- vs. every other agent (Eq. (17): p_ij = n_ij when j is an agent) --
   for j = 1:n
       if j == i, continue; end
       rel = qMeas(:,i) - qMeas(:,j);
       distIJ = max(norm(rel), 1e-3);
       if distIJ <= ry
           agentClose(i) = true;
           nij = rel/distIJ;
           uc(:,i) = uc(:,i) + ka*(distIJ^(-gamma) - ry^(-gamma))*nij;  % Eq. (18), repulsive
       end
   end
end
s.activeObstacle = activeObstacle;
s.agentClose = agentClose;
% ---- Total control input, Eq. (8), saturated per Eq. (9) -------------
U = ut + uc + un;
for i = 1:n
   mag = norm(U(:,i));
   if mag > umax
       U(:,i) = U(:,i) * (umax/mag);
   end
end
% ---- Agent dynamics, Eq. (9): m*qddot = -c*qdot + sat(u) -------------
qddot = (U - c*s.qdot) / m;
s.qdot = s.qdot + qddot*dt;
s.q    = s.q + s.qdot*dt;
s.trajectory(k+1,:) = mean(s.q,2)';
s.speedLog(k+1) = norm(mean(s.qdot,2));
end
function plotForceCurves(kS, alphaS, kB, alphaB, z1, z2, beta)
% Reproduces Fig. 3 (standard vs. relaxing string force law, Eq. (19)-(20))
% and Fig. 4 (magnitude of virtual bar vs. virtual string force, Eq. (10))
% from the paper. Purely analytical -- no simulation involved.
ellRange = linspace(0.5, 60, 1000);
% Standard string law, Eq. (10), evaluated over the full range
fStandard = kS .* ellRange.^alphaS;
% Relaxing string law, Eq. (19)-(20) piecewise
fRelax = zeros(size(ellRange));
for idx = 1:numel(ellRange)
   ell = ellRange(idx);
   if ell <= z1
       fRelax(idx) = kS * ell^alphaS;                          % Eq. (10)
   elseif ell <= z2
       fRelax(idx) = relaxSpline(ell, z1, z2, kS, alphaS, beta); % Eq. (20)
   else
       fRelax(idx) = beta;                                       % constant
   end
end
% Magnitude of the virtual bar force, Eq. (10) with bar parameters
% (kB is negative -> compressive; magnitude is what Fig. 4 plots)
fBarMag = abs(kB) .* ellRange.^alphaB;
% ---- Fig. 3: String force vs. inter-agent distance -------------------
figure('Color','w','Name','Fig. 3 - String force vs distance','Position',[80 850 550 430]);
hold on; grid on;
plot(ellRange, fStandard, 'r-', 'LineWidth',1.6);
plot(ellRange, fRelax,    'b-', 'LineWidth',1.6);
plot([z1 z1], [0 25], 'r--');
plot([z2 z2], [0 25], 'r--');
text(z1, 25.5, 'z_1', 'Color','r', 'HorizontalAlignment','center');
text(z2, 25.5, 'z_2', 'Color','r', 'HorizontalAlignment','center');
xlabel('Distance between agents (\ell_{ij})');
ylabel('Force from virtual string');
title(' String force vs. inter-agent distance for standard and relaxing string');
legend('Standard','Relaxing','Location','northeast');
xlim([0 60]); ylim([0 26]);
% ---- Fig. 4: Magnitude of force from virtual Bar and String ----------
figure('Color','w','Name','Fig. 4 - Bar and String force magnitude','Position',[650 850 550 430]);
hold on; grid on;
plot(ellRange, fBarMag,      'm-', 'LineWidth',1.6);
plot(ellRange, abs(fRelax),  'b-', 'LineWidth',1.6);
plot([z1 z1], [0 30], 'r--');
plot([z2 z2], [0 30], 'r--');
text(z1, 30.5, 'z_1', 'Color','r', 'HorizontalAlignment','center');
text(z2, 30.5, 'z_2', 'Color','r', 'HorizontalAlignment','center');
xlabel('Distance between agents (\ell_{ij})');
ylabel('Magnitude of force');
title(' Magnitude of force from virtual Bar and String');
legend('Virtual bar','Virtual string','Location','northeast');
xlim([0 60]); ylim([0 31]);
end
function f = relaxSpline(ell, z1, z2, kij, alphaij, beta)
% Cubic spline connecting the standard string law to the constant
% relaxed force beta, Eq. (20).
s = (ell - z1) / (z2 - z1);
f = (2*s^3 - 3*s^2 + 1) * kij * z1^alphaij ...
 + (s^3 - 2*s^2 + s) * (z2 - z1) * kij * z1^(alphaij-1) * alphaij ...
 + (-2*s^3 + 3*s^2) * beta;
end
function drawFormation(ax, q, edges, obstacles, activeObstacle, agentClose, traj, t, vel, followRobot, lS, panelTitle)
% Render the current formation state on the given axes: bars, strings,
% agents, obstacles, trajectory, camera follow, and an info readout.
cla(ax); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
n = size(q,2);
th = linspace(0,2*pi,40);
for o = 1:numel(obstacles)
   fill(ax, obstacles(o).center(1)+obstacles(o).radius*cos(th), ...
            obstacles(o).center(2)+obstacles(o).radius*sin(th), ...
            'r', 'FaceAlpha',0.85, 'EdgeColor','none');
end
for e = 1:size(edges,1)
   i = edges(e,1); j = edges(e,2); isBar = edges(e,3);
   if isBar
       plot(ax, [q(1,i) q(1,j)], [q(2,i) q(2,j)], 'b-', 'LineWidth',1.2);
   else
       plot(ax, [q(1,i) q(1,j)], [q(2,i) q(2,j)], 'r--', 'LineWidth',1.0);
   end
end
anyActive = any(activeObstacle,1);
for i = 1:n
   if anyActive(i)
       fill(ax, q(1,i)+2.5*cos(th), q(2,i)+2.5*sin(th), ...
            [1 0.5 0.5], 'FaceAlpha',0.5, 'EdgeColor','none');
   end
end
% orange halo: agent is actively repelling from a nearby teammate
% (Eq. (17)-(18) applied agent-to-agent, per user request)
for i = 1:n
   if agentClose(i)
       fill(ax, q(1,i)+2.0*cos(th), q(2,i)+2.0*sin(th), ...
            [1 0.65 0], 'FaceAlpha',0.6, 'EdgeColor','none');
   end
end
plot(ax, q(1,:), q(2,:), 'ko', 'MarkerFaceColor','k', 'MarkerSize',6);
plot(ax, traj(:,1), traj(:,2), 'g-', 'LineWidth',1.5);
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]');
centroid = mean(q,2);
if followRobot
   halfSpan = 4*lS;
   xlim(ax, [centroid(1)-halfSpan, centroid(1)+halfSpan]);
   ylim(ax, [centroid(2)-halfSpan, centroid(2)+1.5*halfSpan]);
end
infoLine = sprintf('Time: %.2f s | Centroid: [%.1f, %.1f] | Speed: %.2f m/s', ...
   t, centroid(1), centroid(2), norm(vel));
title(ax, {panelTitle, infoLine});
end