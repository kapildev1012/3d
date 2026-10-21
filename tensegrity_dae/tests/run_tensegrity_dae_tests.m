function run_tensegrity_dae_tests()
%RUN_TENSEGRITY_DAE_TESTS  Self-checks for the tensegrity DAE framework.
%
%   run_tensegrity_dae_tests()     (MATLAB or GNU Octave, ~1-2 min)
%
%   1. tensegGenMat: selectors, M symmetric positive definite, G = m_tot g
%   2. Analytical linearisation (Eq. 59-63) vs central differences,
%      3 structures x {rest-length, force-density} inputs, with damping,
%      non-zero velocity and boundary constraints
%   3. Flexible-bar EOM vs numerical Euler-Lagrange of T(q,dq)
%   4. Algorithm 1 keeps ||R|| and |E - E0 - Wf| at ~1e-10 where the
%      uncorrected integration drifts
%   5. Energy bookkeeping under a switching (zero-order-hold) controller
%   6. Stiff flexible bars reproduce the rigid-bar trajectory
%   7. Prestress / form finding gives an exact equilibrium
%   8. LQR on the minimal realisation shrinks the shape error

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root,'structures'), fullfile(root,'dynamics'), fullfile(root,'controllers'));
dd = fullfile(root, '..', 'drive_matlab');
if exist(dd, 'dir'), addpath(dd); end
ws = warning('off', 'tensegSelfStress:approximate');
cleanup = onCleanup(@() warning(ws));
randn('seed', 3);  rand('seed', 3);
nPass = 0;

%% 1 ------------------------------------------------------------ generator
[topo, info] = tensegStructures('sixbar');
S = tensegGenMat(topo, struct('gravity', info.gravity));
check(norm(S.M - S.M', 'fro') < 1e-12 && min(eig(S.M)) > 0, 'M symmetric positive definite');
mtot = sum(S.bar.m) + sum(S.pm.m);
Gs = reshape(S.G, 3, []);
check(norm(sum(Gs, 2) - mtot*info.gravity) < 1e-12, 'G sums to m_tot * g');
N = reshape(S.q0, 3, []);
k = 2;  e = find(S.Cb(k,:));  sgn = S.Cb(k, e);
bk = N(:, e(sgn > 0)) - N(:, e(sgn < 0));
check(norm(S.Xk{k}*S.q0 - bk) < 1e-14 && norm(S.Xbark{k}*S.q0 - mean(N(:,e),2)) < 1e-14, ...
    'X_k q = bar vector, Xbar_k q = bar midpoint');

%% 2 ------------------------------------------------------- linearisation
names = {'sixbar', 'tbar2d', 'prism3'};
worst = 0;
for i = 1:numel(names)
    topo = tensegStructures(names{i}, struct('stringDamping', 2));
    S = tensegGenMat(topo, struct('gravity', [0; 0; -9.8]));
    q = S.q0;  c = tensegConstraints(S, q, 0*q);
    dq = randn(S.nq,1);  dq = dq - pinv(c.Rq)*(c.Rq*dq);
    us = {tensegControl(S, 'restlength', 0.95*S.str.lenInit), ...
          tensegControl(S, 'sigma', 10 + rand(S.ns,1))};
    for j = 1:2
        fx = randn(S.nq,1);
        La = tensegLinearizer(S, q, dq, us{j}, fx);
        Ln = tensegLinearizeNumeric(S, q, dq, us{j}, fx);
        err = max([rel(La.A, Ln.A), rel(La.Bsigma, Ln.Bsigma), rel(La.Bf, Ln.Bf), ...
                   rel(La.Bls, Ln.Bls)]);
        worst = max(worst, err);
    end
end
check(worst < 1e-6, sprintf('analytic Jacobians match finite differences (worst %.1e)', worst));

%% 3 ----------------------------------------------------- flexible bars EL
topo = tensegStructures('prism3');
topo.pmNodes = (1:6)';  topo.pmMass = 0.05;  topo.barRadius = 0.05;  topo.barPoisson = 0.35;
Sf = tensegGenMat(topo, struct('gravity', [0;0;0], 'flexibleBars', true));
q = Sf.q0 + 0.02*randn(Sf.nq,1);  dq = randn(Sf.nq,1);  qdd = randn(Sf.nq,1);  h = 1e-6;
ft = tensegFlexTerms(Sf, q, dq);
Tf = @(a, b) getfield(tensegFlexTerms(Sf, a, b), 'T');
pf = @(a, b) getfield(tensegFlexTerms(Sf, a, b), 'p');
pn = zeros(Sf.nq,1);  gn = pn;
for i = 1:Sf.nq
    ei = zeros(Sf.nq,1);  ei(i) = h;
    pn(i) = (Tf(q, dq+ei) - Tf(q, dq-ei))/(2*h);
    gn(i) = (Tf(q+ei, dq) - Tf(q-ei, dq))/(2*h);
end
dpdt = (pf(q+h*dq, dq+h*qdd) - pf(q-h*dq, dq-h*qdd))/(2*h);
elErr = norm(dpdt - (ft.Mqq*qdd + ft.bias + ft.dTdq))/norm(dpdt);
check(norm(pn - ft.p)/norm(pn) < 1e-6 && norm(gn - ft.dTdq)/norm(gn) < 1e-5 && elErr < 1e-6, ...
    sprintf('flexible-bar Mqq, bias match Euler-Lagrange (err %.1e)', elErr));

%% 4 ---------------------------------------------------------- corrector
topo = tensegStructures('tbar2d');
S = tensegGenMat(topo, struct('gravity', [0;0;0]));
u = tensegControl(S);
loose = struct('method', 'dopri5', 'rtol', 1e-6, 'atol', 1e-6);
oOff = tensegSimulate(S, S.q0, 0*S.q0, u, [], struct('T', 3, 'dtControl', 0.05, ...
    'integrator', loose, 'corrector', struct('enabled', false)));
oOn  = tensegSimulate(S, S.q0, 0*S.q0, u, [], struct('T', 3, 'dtControl', 0.05, 'integrator', loose));
check(max(abs(oOn.Eres)) < 1e-9 && max(oOn.Rnorm) < 1e-9 && ...
      max(abs(oOff.Eres)) > 10*max(abs(oOn.Eres)), ...
    sprintf('Algorithm 1: |dE| %.1e (off: %.1e), ||R|| %.1e', max(abs(oOn.Eres)), ...
    max(abs(oOff.Eres)), max(oOn.Rnorm)));

%% 5 ------------------------------------------- switching controller energy
topo = tensegStructures('prism3', struct('stringDamping', 1));
S = tensegGenMat(topo, struct('gravity', [0;0;-9.8]));
ctrl = @(t, q, dq) tensegControl(S, 'sigma', 40 + 10*sin(7*t + (1:S.ns)'));
o = tensegSimulate(S, S.q0, 0*S.q0, ctrl, [], struct('T', 0.5, 'dtControl', 0.02));
check(max(abs(o.Eres)) < 1e-9 && abs(o.Wact(end)) > 1e-3, ...
    sprintf('energy balance with ZOH force-density control (|dE| %.1e, Wact %.2f J)', ...
    max(abs(o.Eres)), o.Wact(end)));

%% 6 ---------------------------------------------- stiff flexible ~ rigid
topo = tensegStructures('prism3');
topo.pmNodes = (1:6)';  topo.pmMass = 0.2;  topo.barRadius = 0.01;  topo.barStiffness = 1e7;
Sr = tensegGenMat(topo, struct('gravity', [0;0;0]));
Sf = tensegGenMat(topo, struct('gravity', [0;0;0], 'flexibleBars', true));
u = tensegControl(Sr, 'restlength', 0.9*Sr.str.lenInit);
sim = struct('T', 0.2, 'dtControl', 0.01);
oR = tensegSimulate(Sr, Sr.q0, 0*Sr.q0, u, [], sim);
oF = tensegSimulate(Sf, Sf.q0, 0*Sf.q0, u, [], sim);
dev = max(abs(oR.q(:) - oF.q(:)));
check(dev < 1e-3 && max(abs(oF.Eres)) < 1e-8, ...
    sprintf('stiff flexible bars track rigid bars (max dev %.1e m)', dev));

%% 7 ----------------------------------------------- prestress & form find
[topo, info] = tensegStructures('sixbar');
S = tensegGenMat(topo, struct('gravity', [0;0;0]));
st0 = tensegStrings(S, S.q0, 0*S.q0, tensegControl(S));
[~, ls0] = tensegSelfStress(S, S.q0, info.pretension./st0.len);
uR = tensegControl(S, 'restlength', ls0);
qEq = tensegFormFind(S, uR, S.q0);
stE = tensegStrings(S, qEq, 0*qEq, uR);
[sig, ~, ~, res] = tensegSelfStress(S, qEq, stE.sigma);
qdd = tensegDaeDynamics(S, qEq, 0*qEq, uR, []);
check(norm(qdd) < 1e-6 && res < 1e-8 && all(sig > 0), ...
    sprintf('form finding: |qdd| %.1e, self-stress residual %.1e', norm(qdd), res));

%% 8 ----------------------------------------------------------------- LQR
u0 = tensegControl(S, 'sigma', stE.sigma);
lin = tensegLinearizer(S, qEq, 0*qEq, u0, []);
red = tensegReduce(lin);
[ctrl, des] = tensegLqrController(S, lin, red, struct('dt', 0.01, 'Qpos', 1e4, 'Qvel', 10, 'Rsig', 1e-2));
zq = red.Tc(1:S.nq, :)*randn(red.nc, 1);
q0 = qEq + 0.01*zq/max(abs(zq));
[q0, dq0] = daeCorrector(S, q0, 0*q0, 0, u0, 0, struct('energy', false));
o = tensegSimulate(S, q0, dq0, ctrl, [], struct('T', 1, 'dtControl', 0.01, ...
    'integrator', struct('method', 'rk4', 'h', 2e-3)));
z0 = norm(red.Tcpinv*[q0 - qEq; dq0]);
z1 = norm(red.Tcpinv*[o.q(:,end) - qEq; o.dq(:,end)]);
check(des.spectralRadius < 1 && red.nc >= 40 && red.nc <= 48 && z1 < 0.2*z0, ...
    sprintf('LQR: rho %.4f, nc %d, |z| %.2e -> %.2e', des.spectralRadius, red.nc, z0, z1));

fprintf('\nAll %d tensegrity DAE tests passed.\n', nPass);

    function check(ok, msg)
        if ok
            nPass = nPass + 1;
            fprintf('  [PASS] %s\n', msg);
        else
            error('run_tensegrity_dae_tests:fail', 'FAILED: %s', msg);
        end
    end
end

function r = rel(a, b)
if isempty(b) || norm(b, 'fro') == 0, r = norm(a, 'fro'); return; end
r = norm(a - b, 'fro')/norm(b, 'fro');
end
