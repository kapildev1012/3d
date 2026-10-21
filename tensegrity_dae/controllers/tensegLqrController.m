function [ctrl, design] = tensegLqrController(S, lin, red, params)
%TENSEGLQRCONTROLLER  Discrete LQR on the minimal tensegrity realisation.
%
%   [ctrl, design] = tensegLqrController(S, lin, red, params)
%
%   lin : from tensegLinearizer about an equilibrium (q0, 0, sigma0)
%   red : from tensegReduce(lin)  (force-density input B_sigma)
%   params fields:
%     dt      control period [s]               (required)
%     Qpos    weight on controllable position-like states   (default 1e3)
%     Qvel    weight on velocity-like states                 (default 1)
%     Rsig    weight on dsigma                               (default 1)
%     sigmaMin, sigmaMax   actuator bounds (default 0, Inf)
%
%   The reduced model (Ac, Bc) is discretised exactly (zero-order hold,
%   matrix exponential) and the discrete algebraic Riccati equation is
%   solved by the structured doubling algorithm (no toolbox needed). The
%   returned handle
%       u = ctrl(t, q, dq)
%   computes  z = Tc^+ [q - q0; dq - dq0],  sigma = sigma0 - K z,
%   clamps sigma to [sigmaMin, sigmaMax] (sigma >= 0: strings cannot push)
%   and returns a 'sigma'-mode actuation struct for the DAE plant.
%
%   Weights: a state's "position-likeness" is measured by how much of the
%   corresponding basis vector lives in the dq_ block of Tc.

dt   = params.dt;
Qpos = getOpt(params, 'Qpos', 1e3);
Qvel = getOpt(params, 'Qvel', 1);
Rsig = getOpt(params, 'Rsig', 1);
smin = getOpt(params, 'sigmaMin', 0);
smax = getOpt(params, 'sigmaMax', Inf);

Ac = red.Ac;  Bc = red.Bc;
nc = size(Ac,1);  nu = size(Bc,2);
nq = S.nq;
posShare = sum(red.Tc(1:nq,:).^2, 1)./max(sum(red.Tc.^2,1), eps);
Qw = diag(Qpos*posShare + Qvel*(1 - posShare));
Rw = Rsig*eye(nu);

% exact ZOH discretisation
Mexp = expm([Ac, Bc; zeros(nu, nc+nu)]*dt);
Ad = Mexp(1:nc, 1:nc);  Bd = Mexp(1:nc, nc+1:end);
% DARE by the structure-preserving doubling algorithm (quadratic
% convergence, robust for lightly damped / marginal open-loop modes)
Ak = Ad;  Gk = Bd*(Rw\Bd');  Hk = Qw;  I = eye(nc);
for it = 1:100
    W  = I + Gk*Hk;
    WA = W\Ak;
    Gn = Gk + Ak*(W\Gk)*Ak';
    Hn = Hk + Ak'*Hk*WA;
    Ak = Ak*WA;
    Gk = (Gn + Gn')/2;
    dH = norm(Hn - Hk, 'fro');
    Hk = (Hn + Hn')/2;
    if dH <= 1e-12*max(1, norm(Hk, 'fro')), break; end
end
P = Hk;
Kd = (Rw + Bd'*P*Bd)\(Bd'*P*Ad);
clEig = eig(Ad - Bd*Kd);

q0 = lin.q0;  dq0 = lin.dq0;  sigma0 = lin.sigma0;
Tcp = red.Tcpinv;
ctrl = @(t, q, dq) lqrLaw(S, q, dq, q0, dq0, sigma0, Tcp, Kd, smin, smax);

design = struct('K', Kd, 'P', P, 'Ad', Ad, 'Bd', Bd, 'Q', Qw, 'R', Rw, ...
    'closedLoopEig', clEig, 'spectralRadius', max(abs(clEig)), 'iterations', it);
end

function u = lqrLaw(S, q, dq, q0, dq0, sigma0, Tcp, K, smin, smax)
z = Tcp*[q - q0; dq - dq0];
sigma = min(max(sigma0 - K*z, smin), smax);
u = tensegControl(S, 'sigma', sigma);
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
