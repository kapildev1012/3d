function results = main_tensegrity_dae(varargin)
%MAIN_TENSEGRITY_DAE  Non-minimal Cartesian Lagrangian tensegrity DAE demo.
%
%   results = main_tensegrity_dae()                       % sixbar_lqr
%   results = main_tensegrity_dae('scenario', 'all')
%   results = main_tensegrity_dae('scenario', 'tbar2d', 'plot', false)
%
%   Options (name/value):
%     scenario  'sixbar_lqr' | 'sixbar_drop' | 'tbar2d' | 'all'
%     plot      true/false                         (default true)
%     T         simulation time override [s]
%     verbose   true/false                         (default true)
%
%   Scenarios
%   ---------
%   sixbar_lqr  : your 6-bar ball (drive_matlab/six_bar_model.m) in zero
%                 gravity. The model is prestressed (cable pretension from
%                 six_bar_model), settled onto its exact equilibrium,
%                 linearised analytically (Eq. 59-63), reduced to the
%                 constraint tangent space and its controllable subspace,
%                 and stabilised by a force-density LQR. The closed loop
%                 is compared with the passive structure from the same
%                 perturbed initial shape (light cable damping c = 0.5 N s/m
%                 in both runs: a few modes of the symmetric ball are almost
%                 uncontrollable by force densities and decay passively).
%   sixbar_drop : passive prestressed ball dropped onto a floor under
%                 gravity; corrector on vs off at loose tolerance.
%   tbar2d      : paper Fig. 1(a) 2-bar structure (pinned base, 90 %
%                 rest-length springs); corrector on vs off at loose tolerance.
%
%   Every run integrates x = [q; dq; Wf] with Dormand-Prince 5(4) and
%   applies Algorithm 1 (daeCorrector) after every accepted step, then plots
%   nodal trajectories, sigma(t), bar-length violations ||b_k|| - l_bk and
%   the energy error |E - E0 - Wf|.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'structures'), fullfile(here, 'dynamics'), fullfile(here, 'controllers'));
driveDir = fullfile(here, '..', 'drive_matlab');
if exist(driveDir, 'dir'), addpath(driveDir); end

opt = struct('scenario', 'sixbar_lqr', 'plot', true, 'T', [], 'verbose', true);
for a = 1:2:numel(varargin), opt.(varargin{a}) = varargin{a+1}; end

if strcmpi(opt.scenario, 'all')
    list = {'sixbar_lqr', 'sixbar_drop', 'tbar2d'};
else
    list = {opt.scenario};
end
results = struct();
for i = 1:numel(list)
    switch lower(list{i})
        case 'sixbar_lqr',  r = runSixbarLqr(opt);
        case 'sixbar_drop', r = runSixbarDrop(opt);
        case 'tbar2d',      r = runTbar(opt);
        otherwise, error('main_tensegrity_dae:scenario', 'Unknown scenario "%s".', list{i});
    end
    results.(list{i}) = r;
    if opt.plot, plotRuns(r); end
end
end

% =========================================================================
function r = runSixbarLqr(opt)
say(opt, '\n=== sixbar_lqr: analytical linearisation + LQR (zero gravity) ===\n');
T = pick(opt.T, 3);  dtc = 0.01;
[topo, info] = tensegStructures('sixbar', struct('stringDamping', 0.5));
S = tensegGenMat(topo, struct('gravity', [0;0;0]));

% 1) prestress from six_bar_model pretension, settle on exact equilibrium
st0 = tensegStrings(S, S.q0, zeros(S.nq,1), tensegControl(S));
ws = warning('off', 'tensegSelfStress:approximate');
[~, ls0] = tensegSelfStress(S, S.q0, info.pretension./st0.len);
warning(ws);
uRest = tensegControl(S, 'restlength', ls0);
[qEq, ff] = tensegFormFind(S, uRest, S.q0);
stEq = tensegStrings(S, qEq, zeros(S.nq,1), uRest);
sigma0 = stEq.sigma;
say(opt, ['  form finding: shape change %.2e m RMS (rounded coordinates in ' ...
    'six_bar_model.m), residual qdd %.1e, cable tension %.1f-%.1f N\n'], ...
    shapeError(qEq, S.q0), ff.qddNorm, min(sigma0.*stEq.len), max(sigma0.*stEq.len));

% 2) analytical linearisation about (qEq, 0, sigma0) with sigma as input
u0 = tensegControl(S, 'sigma', sigma0);
lin = tensegLinearizer(S, qEq, zeros(S.nq,1), u0, zeros(S.nq,1));
red = tensegReduce(lin);
say(opt, '  full state %d -> tangent %d -> controllable %d (invariance res %.1e)\n', ...
    2*S.nq, 2*red.nz, red.nc, red.invarianceResidual);
    if red.nPruned > 0, say(opt, '  (%d near-uncontrollable mechanism states split off by the Hautus test)\n', red.nPruned); end

% 3) LQR
[ctrl, design] = tensegLqrController(S, lin, red, struct('dt', dtc, ...
    'Qpos', 1e4, 'Qvel', 10, 'Rsig', 1e-2));
say(opt, '  LQR closed-loop spectral radius %.4f\n', design.spectralRadius);

% 4) perturbed initial shape (controllable directions, 1 cm scale)
randn('seed', 7);
zq = red.Tc(1:S.nq, :)*randn(red.nc, 1);
q0 = qEq + 0.01*zq/max(abs(zq));
[q0, dq0] = daeCorrector(S, q0, zeros(S.nq,1), 0, u0, 0, struct('energy', false));

sim = struct('T', T, 'dtControl', dtc, ...
    'integrator', struct('method', 'dopri5', 'rtol', 1e-10, 'atol', 1e-10));
say(opt, '  running passive ...\n');
outP = tensegSimulate(S, q0, dq0, u0, [], sim);
say(opt, '  running LQR ...\n');
outL = tensegSimulate(S, q0, dq0, ctrl, [], sim);
outP.shapeErr = shapeError(outP.q, qEq);
outL.shapeErr = shapeError(outL.q, qEq);
say(opt, '  shape RMS error at t=%.1fs: passive %.2e m | LQR %.2e m\n', T, outP.shapeErr(end), outL.shapeErr(end));
summarize(opt, 'passive', outP);  summarize(opt, 'LQR', outL);
r = struct('name', 'sixbar_lqr', 'S', S, 'qEq', qEq, 'sigma0', sigma0, 'lin', lin, ...
    'red', red, 'design', design, 'runs', {{outP, outL}}, 'labels', {{'passive', 'LQR'}});
end

% =========================================================================
function r = runSixbarDrop(opt)
say(opt, '\n=== sixbar_drop: prestressed ball dropped on a floor ===\n');
T = pick(opt.T, 1);
[topo, info] = tensegStructures('sixbar', struct('stringDamping', 1));
S0 = tensegGenMat(topo, struct('gravity', [0;0;0]));
st0 = tensegStrings(S0, S0.q0, zeros(S0.nq,1), tensegControl(S0));
ws = warning('off', 'tensegSelfStress:approximate');
[~, ls0] = tensegSelfStress(S0, S0.q0, info.pretension./st0.len);
warning(ws);
u = tensegControl(S0, 'restlength', ls0);
qEq = tensegFormFind(S0, u, S0.q0);
S = tensegGenMat(setfield(topo, 'N0', reshape(qEq, 3, [])), struct('gravity', info.gravity));
N = reshape(qEq, 3, []);  N(3,:) = N(3,:) - min(N(3,:)) + 0.2;   % 0.2 m above floor
q0 = N(:);
gp = struct('z0', 0, 'k', 2e4, 'c', 20, 'mu_v', 5);
fext = @(t, q, dq) tensegGroundForce(q, dq, gp);
loose = struct('method', 'dopri5', 'rtol', 1e-6, 'atol', 1e-6);
simOn  = struct('T', T, 'dtControl', 0.005, 'integrator', loose);
simOff = simOn;  simOff.corrector = struct('enabled', false);
say(opt, '  corrector OFF ...\n');  outOff = tensegSimulate(S, q0, zeros(S.nq,1), u, fext, simOff);
say(opt, '  corrector ON  ...\n');  outOn  = tensegSimulate(S, q0, zeros(S.nq,1), u, fext, simOn);
summarize(opt, 'no correction', outOff);  summarize(opt, 'Algorithm 1', outOn);
r = struct('name', 'sixbar_drop', 'S', S, 'runs', {{outOff, outOn}}, ...
    'labels', {{'no correction (tol 1e-6)', 'Algorithm 1 (tol 1e-6)'}});
end

% =========================================================================
function r = runTbar(opt)
say(opt, '\n=== tbar2d: paper Fig. 1(a), pinned base, 90%% springs ===\n');
T = pick(opt.T, 10);
topo = tensegStructures('tbar2d');
S = tensegGenMat(topo, struct('gravity', [0;0;0]));
u = tensegControl(S);                      % rest lengths from topo (90 %)
loose = struct('method', 'dopri5', 'rtol', 1e-6, 'atol', 1e-6);
simOn  = struct('T', T, 'dtControl', 0.02, 'integrator', loose);
simOff = simOn;  simOff.corrector = struct('enabled', false);
outOff = tensegSimulate(S, S.q0, zeros(S.nq,1), u, [], simOff);
outOn  = tensegSimulate(S, S.q0, zeros(S.nq,1), u, [], simOn);
summarize(opt, 'no correction', outOff);  summarize(opt, 'Algorithm 1', outOn);
r = struct('name', 'tbar2d', 'S', S, 'runs', {{outOff, outOn}}, ...
    'labels', {{'no correction (tol 1e-6)', 'Algorithm 1 (tol 1e-6)'}});
end

% =========================================================================
function plotRuns(r)
S = r.S;  nr = numel(r.runs);
cols = [0.85 0.2 0.1; 0.1 0.45 0.85; 0.2 0.6 0.2];
figure('Color', 'w', 'Name', ['Tensegrity DAE - ' r.name]);
o = r.runs{end};
subplot(2,2,1);  % nodal trajectories (z) of the last run
plot(o.t, o.q(3:3:end, :)');  grid on;
xlabel('t [s]');  ylabel('z_i [m]');  title(['Nodal heights - ' r.labels{end}]);
subplot(2,2,2);  % control input sigma(t)
plot(o.t, o.sigma');  grid on;
xlabel('t [s]');  ylabel('\sigma_k [N/m]');  title(['Force densities - ' r.labels{end}]);
subplot(2,2,3);  hold on;  grid on;  h = zeros(1,nr);
for i = 1:nr
    e = max(abs(r.runs{i}.barErr), [], 1);
    h(i) = semilogy(r.runs{i}.t, max(e, 1e-18), 'Color', cols(i,:), 'LineWidth', 1.3);
end
set(gca, 'YScale', 'log');
xlabel('t [s]');  ylabel('max_k | ||b_k|| - l_{bk} | [m]');  title('Bar-length violation');
legend(h, r.labels, 'Location', 'best');
subplot(2,2,4);  hold on;  grid on;
for i = 1:nr
    semilogy(r.runs{i}.t, max(abs(r.runs{i}.Eres), 1e-18), 'Color', cols(i,:), 'LineWidth', 1.3);
end
set(gca, 'YScale', 'log');
xlabel('t [s]');  ylabel('|E - E_0 - W_f| [J]');  title('Energy conservation error');

if isfield(o, 'shapeErr')
    figure('Color', 'w', 'Name', [r.name ' - shape regulation']);  hold on;  grid on;
    for i = 1:nr
        plot(r.runs{i}.t, r.runs{i}.shapeErr, 'Color', cols(i,:), 'LineWidth', 1.5);
    end
    xlabel('t [s]');  ylabel('shape RMS error [m]');  legend(r.labels);
    title('Deviation from equilibrium shape (rigid motion removed)');
end
figure('Color', 'w', 'Name', [r.name ' - final configuration']);
tensegPlotStructure(S, o.q(:,end));
title([r.name ' - final configuration (' r.labels{end} ')']);
end

function tensegPlotStructure(S, q)
N = reshape(q, 3, []);  hold on;  axis equal;  grid on;  view(3);
for k = 1:S.nb
    e = find(S.Cb(k,:));
    plot3(N(1,e), N(2,e), N(3,e), 'k-', 'LineWidth', 3);
end
for k = 1:S.ns
    e = find(S.Cs(k,:));
    plot3(N(1,e), N(2,e), N(3,e), 'r-', 'LineWidth', 1);
end
plot3(N(1,:), N(2,:), N(3,:), 'ko', 'MarkerFaceColor', 'w');
xlabel('x [m]');  ylabel('y [m]');  zlabel('z [m]');
end

% =========================================================================
function e = shapeError(Q, qRef)
% RMS nodal deviation after optimal rigid alignment (Kabsch)
P0 = reshape(qRef, 3, []);  P0c = P0 - repmat(mean(P0,2), 1, size(P0,2));
e = zeros(1, size(Q,2));
for j = 1:size(Q,2)
    P = reshape(Q(:,j), 3, []);  Pc = P - repmat(mean(P,2), 1, size(P,2));
    [U, ~, V] = svd(Pc*P0c');
    D = diag([1 1 sign(det(V*U'))]);
    Rm = V*D*U';
    e(j) = sqrt(mean(sum((Rm*Pc - P0c).^2, 1)));
end
end

function summarize(opt, label, o)
say(opt, '  %-14s max|bar err| %.2e m | max|E-E0-Wf| %.2e J | max ||R|| %.2e\n', ...
    label, max(abs(o.barErr(:))), max(abs(o.Eres)), max(o.Rnorm));
end

function say(opt, varargin)
if opt.verbose, fprintf(varargin{:}); end
end

function v = pick(v, d)
if isempty(v), v = d; end
end
