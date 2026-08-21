function result = level10_payload_core_gforce(varargin)
%LEVEL10_PAYLOAD_CORE_GFORCE  Level-10 central payload isolation experiment.
%
%   result = level10_payload_core_gforce()
%   result = level10_payload_core_gforce('Duration', 20, 'ShowFigure', false)
%
%   This experiment couples a 1.6 kg central payload to all 12 tensegrity
%   nodes with tension-only spring/damper suspension cables.  The payload
%   force is applied to the core and the equal/opposite reaction is applied
%   to the outer structure.  G-force is computed from specific force
%   (non-gravitational force / mass), which is what an accelerometer reads.
%
%   There is no empirical acceleration scaling, hard G clipping, or visual
%   moving-average filter in the reported measurements.
%
%   Name/value options:
%     Duration       simulation duration [s]                 (40)
%     Dt             fixed physics time step [s]             (0.002)
%     SettleDuration passive settling before Level 10 [s]    (2.0)
%     TargetSpeed    nominal forward speed [m/s]             (0.20)
%     CoreMass       central payload mass [kg]                (1.60)
%     CoreStiffness  each suspension cable stiffness [N/m]   (1600)
%     CoreDamping    each suspension cable damping [N s/m]   (18)
%     ShowFigure     leave the results figure visible         (true)
%     SaveResults    save MAT and PNG outputs                  (true)
%     OutputDir      output folder                             (results/...)
%     Verbose        print progress and the final report       (true)

rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, '3D'));
addpath(fullfile(rootDir, 'paper_reproduction', 'utils'));

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Duration', 40, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'Dt', 0.002, @(x) isnumeric(x) && isscalar(x) && x > 0 && x <= 0.005);
addParameter(p, 'SettleDuration', 2.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'TargetSpeed', 0.20, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'CoreMass', 1.60, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'CoreStiffness', 1600, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'CoreDamping', 18, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'ShowFigure', true, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'SaveResults', true, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'OutputDir', fullfile(rootDir, 'results', 'level10_payload_core'), ...
    @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(p, 'Verbose', true, @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
parse(p, varargin{:});
opt = p.Results;
opt.ShowFigure = logical(opt.ShowFigure);
opt.SaveResults = logical(opt.SaveResults);
opt.Verbose = logical(opt.Verbose);
opt.OutputDir = char(opt.OutputDir);

% -------------------------------------------------------------------------
% Level-10 outer tensegrity configuration: fixed 500 Hz physics, Earth
% gravity, non-bouncy contact, rough terrain, and ten staggered obstacles.
% -------------------------------------------------------------------------
cfg = config3D();
cfg.dt = opt.Dt;
cfg.T_end = opt.Duration;
cfg.n_steps = round(cfg.T_end / cfg.dt);
cfg.seed = 1010;
cfg.m = 0.20;                    % mass at each of 12 rod endpoints [kg]
cfg.c = 0.90;                    % global node drag [N s/m]
cfg.gravity = [0; 0; -9.81];     % acceleration [m/s^2] in this experiment
cfg.disturbanceForce = [0; 0; 0];
cfg.u_nav = zeros(3, 1);         % assigned after payload mass is defined
cfg.u_max = 50;
cfg.kS = 180;
cfg.kB = -cfg.kS * (3 * sqrt(6) / 8);
cfg.kg = 4500;
cfg.cg = 60;
cfg.mu_g = 0.85;
cfg.c_gt = 6.0;
cfg.groundRMS = 0.09;
cfg.ry = 0.45;
cfg.ka = 2.5;
cfg.gamma = 0.40;
cfg.sigma = 0;
cfg.processNoiseSigma = 0;
cfg.noise_mode = 'none';
cfg.showAnimation = false;
cfg.verbose = false;
cfg.obstacles = level10Obstacles();

L = buildLander3D(cfg);
assert(L.nB == 6 && L.n == 12 && L.nS == 24, ...
    'Level 10 requires 6 rods, 12 rod endpoints, and 24 outer cables.');

% Physical payload and suspension.  Pretension is encoded through a shorter
% cable rest length, so the cable law remains F = k(L-L0)+c*dL/dt.
payload.mass = opt.CoreMass;     % [kg]
payload.k = opt.CoreStiffness;   % [N/m] per suspension cable
payload.c = opt.CoreDamping;     % [N s/m] per suspension cable
payload.pretension = 25;         % [N] at the geometric centre
payload.maxTension = 400;        % [N] physical cable/load-cell limit
payload.allowableTravel = 0.075; % [m] radial payload travel limit
payload.gLimit = 1.50;           % [G] Level-10 acceptance threshold

% Coulomb-friction feed-forward plus viscous-drag feed-forward.  This is a
% force command (not a position/translation edit) and keeps the load-test
% course near the requested 0.20 m/s once rolling resistance is established.
supportedMassPerNode = cfg.m + payload.mass / L.n;
cfg.u_nav = [0; cfg.c * opt.TargetSpeed + ...
    cfg.mu_g * supportedMassPerNode * abs(cfg.gravity(3)); 0];

rng(cfg.seed, 'twister');
[q, v] = initialCageState(L, cfg);
cageCentroid0 = mean(q, 2);
payload.restLength = vecnorm(q - cageCentroid0, 2, 1).' ...
    - payload.pretension / payload.k;
payload.restLength = max(payload.restLength, 0.05);

% Start the payload at its static sag equilibrium to avoid a non-physical
% initialization impulse.  The whole rover still receives passive settling.
corePosition = staticCoreEquilibrium(q, payload, cfg.gravity);
coreVelocity = mean(v, 2);
coreRelative0 = corePosition - mean(q, 2);

N = cfg.n_steps;
t = (0:N) * cfg.dt;
cagePosition = zeros(3, N + 1);
corePositionHistory = zeros(3, N + 1);
cageSpecificForce = zeros(3, N + 1);
coreSpecificForce = zeros(3, N + 1);
suspensionTension = zeros(L.n, N + 1);
minimumTerrainClearance = zeros(1, N + 1);

nz = noiselessNoise(L, cfg);
activeCfg = cfg;
nextProgress = 0.1;

if opt.Verbose
    fprintf('\n=== LEVEL 10: CENTRAL PAYLOAD CORE G-FORCE ISOLATION ===\n');
    fprintf('Topology: rods=%d, rod endpoints=%d, outer cables=%d, suspension cables=%d\n', ...
        L.nB, L.n, L.nS, L.n);
    fprintf('Physics: %.0f Hz fixed step | payload %.2f kg | target %.2f m/s\n', ...
        1 / cfg.dt, payload.mass, opt.TargetSpeed);
    fprintf('Passive settling: %.2f s | Level-10 obstacles: %d\n', ...
        opt.SettleDuration, size(cfg.obstacles, 1));
end

for step = 1:N + 1
    timeNow = t(step);
    cagePosition(:, step) = mean(q, 2);
    corePositionHistory(:, step) = corePosition;

    % Navigation is disabled during passive settling.  Physics, gravity,
    % contact, suspension, and damping remain active.
    if timeNow < opt.SettleDuration
        activeCfg.u_nav = zeros(3, 1);
    else
        activeCfg.u_nav = cfg.u_nav;
    end

    [outerForce, diagnostics] = controlInput3D(q, v, L, activeCfg, nz);
    [coreCableForce, nodeReaction, cableTension] = suspensionForces( ...
        q, v, corePosition, coreVelocity, payload);
    coupledNodeForce = outerForce + nodeReaction;

    % Specific force excludes gravity.  It is the physically correct
    % accelerometer quantity and therefore reads approximately 1 G at rest.
    nodeSpecific = (coupledNodeForce - cfg.c * v ...
        + repmat(cfg.disturbanceForce, 1, L.n)) / cfg.m;
    cageSpecificForce(:, step) = mean(nodeSpecific, 2);
    coreSpecificForce(:, step) = coreCableForce / payload.mass;
    suspensionTension(:, step) = cableTension;
    minimumTerrainClearance(step) = min(q(3, :) - diagnostics.h_ground - cfg.nodeRadius);

    if step == N + 1
        break;
    end

    % Equal-and-opposite coupled dynamics.  Semi-implicit Euler is stable
    % for this stiffness at the enforced <=5 ms fixed physics step.
    nodeAcceleration = nodeSpecific + repmat(cfg.gravity, 1, L.n);
    coreAcceleration = coreSpecificForce(:, step) + cfg.gravity;
    v = v + cfg.dt * nodeAcceleration;
    q = q + cfg.dt * v;
    coreVelocity = coreVelocity + cfg.dt * coreAcceleration;
    corePosition = corePosition + cfg.dt * coreVelocity;

    if any(~isfinite(q(:))) || any(~isfinite(v(:))) ...
            || any(~isfinite(corePosition)) || any(~isfinite(coreVelocity))
        error('level10_payload_core_gforce:diverged', ...
            'The coupled simulation became non-finite at t = %.4f s.', timeNow);
    end

    if opt.Verbose && step / max(N, 1) >= nextProgress
        fprintf('  simulation %3.0f%% complete\n', 100 * nextProgress);
        nextProgress = nextProgress + 0.1;
    end
end

% -------------------------------------------------------------------------
% Proper acceleration and dynamic isolation metrics.
% -------------------------------------------------------------------------
g0 = 9.80665;
gravityReference = -cfg.gravity / g0;
cageProperG = vecnorm(cageSpecificForce, 2, 1) / g0;
coreProperG = vecnorm(coreSpecificForce, 2, 1) / g0;
cageShockG = vecnorm(cageSpecificForce / g0 - gravityReference, 2, 1);
coreShockG = vecnorm(coreSpecificForce / g0 - gravityReference, 2, 1);

metricMask = t >= opt.SettleDuration;
if ~any(metricMask)
    metricMask = true(size(t));
end

relativeMotion = corePositionHistory - cagePosition;
suspensionTravel = vecnorm(relativeMotion - coreRelative0, 2, 1);
cageShockRms = sqrt(mean(cageShockG(metricMask).^2));
coreShockRms = sqrt(mean(coreShockG(metricMask).^2));
cageShockPeak = max(cageShockG(metricMask));
coreShockPeak = max(coreShockG(metricMask));

result = struct();
result.level = 10;
result.name = 'Central Payload Core G-Force Isolation';
result.t = t;
result.cagePosition = cagePosition;
result.corePosition = corePositionHistory;
result.relativeMotion = relativeMotion;
result.cageSpecificForce = cageSpecificForce;
result.coreSpecificForce = coreSpecificForce;
result.cageProperG = cageProperG;
result.coreProperG = coreProperG;
result.cageShockG = cageShockG;
result.coreShockG = coreShockG;
result.suspensionTension = suspensionTension;
result.minimumTerrainClearance = minimumTerrainClearance;
result.metrics = struct( ...
    'peakCageProperG', max(cageProperG(metricMask)), ...
    'peakCoreProperG', max(coreProperG(metricMask)), ...
    'rmsCageShockG', cageShockRms, ...
    'rmsCoreShockG', coreShockRms, ...
    'peakCageShockG', cageShockPeak, ...
    'peakCoreShockG', coreShockPeak, ...
    'rmsIsolationPercent', isolationPercent(cageShockRms, coreShockRms), ...
    'peakIsolationPercent', isolationPercent(cageShockPeak, coreShockPeak), ...
    'maxPayloadTravelM', max(suspensionTravel(metricMask)), ...
    'maxSuspensionTensionN', max(suspensionTension(:, metricMask), [], 'all'), ...
    'minimumTerrainClearanceM', min(minimumTerrainClearance(metricMask)), ...
    'gLimit', payload.gLimit, ...
    'passed', false);
result.metrics.passed = result.metrics.peakCoreProperG <= payload.gLimit ...
    && result.metrics.maxPayloadTravelM <= payload.allowableTravel;
result.configuration = struct('simulation', cfg, 'payload', payload, 'options', opt);

figureHandle = [];
if opt.ShowFigure || opt.SaveResults
    figureHandle = plotIsolationResult(result, payload, opt.ShowFigure);
end

if opt.SaveResults
    if ~exist(opt.OutputDir, 'dir')
        mkdir(opt.OutputDir);
    end
    matFile = fullfile(opt.OutputDir, 'level10_payload_core_gforce.mat');
    pngFile = fullfile(opt.OutputDir, 'level10_payload_core_gforce.png');
    result.outputMatFile = matFile;
    result.outputPngFile = pngFile;
    save(matFile, 'result');
    exportgraphics(figureHandle, pngFile, 'Resolution', 180);
end

if ~opt.ShowFigure && ~isempty(figureHandle)
    close(figureHandle);
end

if opt.Verbose
    fprintf('\nLevel-10 payload-isolation result: %s\n', passText(result.metrics.passed));
    fprintf('  Peak proper acceleration: cage %.3f G | core %.3f G\n', ...
        result.metrics.peakCageProperG, result.metrics.peakCoreProperG);
    fprintf('  RMS dynamic shock:        cage %.3f G | core %.3f G\n', ...
        result.metrics.rmsCageShockG, result.metrics.rmsCoreShockG);
    fprintf('  Isolation: RMS %.1f%% | peak %.1f%%\n', ...
        result.metrics.rmsIsolationPercent, result.metrics.peakIsolationPercent);
    fprintf('  Payload travel: %.2f mm / %.2f mm allowed\n', ...
        1000 * result.metrics.maxPayloadTravelM, 1000 * payload.allowableTravel);
    fprintf('  Maximum suspension tension: %.1f N\n', ...
        result.metrics.maxSuspensionTensionN);
    if opt.SaveResults
        fprintf('  Saved: %s\n', result.outputMatFile);
        fprintf('         %s\n', result.outputPngFile);
    end
    fprintf('==========================================================\n\n');
end

end

function obstacles = level10Obstacles()
% Ten staggered spherical hazards along the +Y Level-10 course.
obstacles = [ ...
    -0.34, 1.30, 0.25, 0.20;
     0.30, 1.95, 0.28, 0.24;
    -0.12, 2.65, 0.31, 0.28;
     0.38, 3.35, 0.24, 0.22;
    -0.40, 4.05, 0.34, 0.30;
     0.08, 4.75, 0.27, 0.25;
     0.40, 5.45, 0.32, 0.29;
    -0.30, 6.15, 0.25, 0.23;
     0.18, 6.85, 0.35, 0.31;
    -0.05, 7.55, 0.29, 0.26];
end

function [q, v] = initialCageState(L, cfg)
q = L.q0;
v = zeros(3, L.n);
terrainHeight = evalSurface(q(1, :), q(2, :), cfg);
clearance = q(3, :) - terrainHeight - cfg.nodeRadius;
q(3, :) = q(3, :) + (0.020 - min(clearance));
end

function nz = noiselessNoise(L, cfg)
M = size(L.members, 1);
nO = max(size(cfg.obstacles, 1), 1);
nz.mode = 'relative';
nz.mem = zeros(3, M);
nz.mem_j = zeros(3, M);
nz.mem_scalar = zeros(M, 1);
nz.obs = zeros(3, L.n, nO);
nz.obs_scalar = zeros(L.n, nO);
nz.agt = zeros(3, L.n, L.n);
nz.agt_scalar = zeros(L.n, L.n);
nz.process = zeros(3, L.n);
end

function equilibrium = staticCoreEquilibrium(q, payload, gravity)
equilibrium = mean(q, 2);
zeroNodeVelocity = zeros(size(q));
zeroCoreVelocity = zeros(3, 1);

for iteration = 1:40
    force = suspensionForces(q, zeroNodeVelocity, equilibrium, ...
        zeroCoreVelocity, payload) + payload.mass * gravity;
    if norm(force) < 1e-8
        break;
    end

    jacobian = zeros(3, 3);
    epsilon = 1e-6;
    for axis = 1:3
        perturbed = equilibrium;
        perturbed(axis) = perturbed(axis) + epsilon;
        perturbedForce = suspensionForces(q, zeroNodeVelocity, perturbed, ...
            zeroCoreVelocity, payload) + payload.mass * gravity;
        jacobian(:, axis) = (perturbedForce - force) / epsilon;
    end

    if rcond(jacobian) < 1e-12
        break;
    end
    correction = -jacobian \ force;
    correctionNorm = norm(correction);
    if correctionNorm > 0.01
        correction = correction * (0.01 / correctionNorm);
    end
    equilibrium = equilibrium + correction;
end
end

function [coreForce, nodeReaction, tension] = suspensionForces( ...
        nodePosition, nodeVelocity, corePosition, coreVelocity, payload)
n = size(nodePosition, 2);
coreForce = zeros(3, 1);
nodeReaction = zeros(3, n);
tension = zeros(n, 1);

for cable = 1:n
    displacement = nodePosition(:, cable) - corePosition;
    lengthNow = norm(displacement);
    if lengthNow < 1e-12
        continue;
    end
    direction = displacement / lengthNow;
    lengthRate = dot(nodeVelocity(:, cable) - coreVelocity, direction);
    cableForce = payload.k * (lengthNow - payload.restLength(cable)) ...
        + payload.c * lengthRate;
    tension(cable) = min(payload.maxTension, max(0, cableForce));
    forceVector = tension(cable) * direction;
    coreForce = coreForce + forceVector;
    nodeReaction(:, cable) = -forceVector;
end
end

function percentage = isolationPercent(inputMetric, outputMetric)
if inputMetric <= eps
    percentage = NaN;
else
    percentage = 100 * (1 - outputMetric / inputMetric);
end
end

function fig = plotIsolationResult(result, payload, showFigure)
visibility = 'off';
if showFigure
    visibility = 'on';
end

fig = figure('Name', 'Level 10 - Central Payload Core G-Force Isolation', ...
    'Color', 'w', 'Visible', visibility, 'Position', [80, 80, 1280, 820]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Level 10 — Central Payload Core G-Force Isolation', ...
    'FontWeight', 'bold');

nexttile;
plot(result.t, result.cageProperG, 'Color', [0.35, 0.35, 0.40], ...
    'LineWidth', 1.0, 'DisplayName', 'Outer cage');
hold on;
plot(result.t, result.coreProperG, 'Color', [0.00, 0.60, 0.38], ...
    'LineWidth', 1.5, 'DisplayName', 'Suspended core');
yline(payload.gLimit, '--', '1.5 G limit', 'Color', [0.85, 0.25, 0.15]);
xlabel('Time [s]'); ylabel('Proper acceleration [G]');
title('Accelerometer Load'); grid on; legend('Location', 'best');

nexttile;
plot(result.t, result.cageShockG, 'Color', [0.35, 0.35, 0.40], ...
    'LineWidth', 1.0, 'DisplayName', 'Cage dynamic shock');
hold on;
plot(result.t, result.coreShockG, 'Color', [0.00, 0.45, 0.85], ...
    'LineWidth', 1.5, 'DisplayName', 'Core dynamic shock');
xlabel('Time [s]'); ylabel('Dynamic specific force [G]');
title(sprintf('RMS Isolation = %.1f%%', result.metrics.rmsIsolationPercent));
grid on; legend('Location', 'best');

nexttile;
relativeDelta = 1000 * (result.relativeMotion - result.relativeMotion(:, 1));
plot(result.t, relativeDelta(1, :), 'LineWidth', 1.1, 'DisplayName', 'X');
hold on;
plot(result.t, relativeDelta(2, :), 'LineWidth', 1.1, 'DisplayName', 'Y');
plot(result.t, relativeDelta(3, :), 'LineWidth', 1.1, 'DisplayName', 'Z');
yline(1000 * payload.allowableTravel, '--', 'travel limit');
yline(-1000 * payload.allowableTravel, '--', 'HandleVisibility', 'off');
xlabel('Time [s]'); ylabel('Core displacement relative to cage [mm]');
title(sprintf('Maximum Travel = %.1f mm', 1000 * result.metrics.maxPayloadTravelM));
grid on; legend('Location', 'best');

nexttile;
plot(result.t, max(result.suspensionTension, [], 1), ...
    'Color', [0.65, 0.15, 0.75], 'LineWidth', 1.4, ...
    'DisplayName', 'Maximum of 12 cables');
hold on;
plot(result.t, min(result.suspensionTension, [], 1), ...
    'Color', [0.95, 0.55, 0.10], 'LineWidth', 1.0, ...
    'DisplayName', 'Minimum of 12 cables');
yline(payload.maxTension, '--', 'load limit');
xlabel('Time [s]'); ylabel('Suspension tension [N]');
title(sprintf('Peak Tension = %.1f N', result.metrics.maxSuspensionTensionN));
grid on; legend('Location', 'best');
end

function text = passText(passed)
if passed
    text = 'PASS';
else
    text = 'FAIL';
end
end
