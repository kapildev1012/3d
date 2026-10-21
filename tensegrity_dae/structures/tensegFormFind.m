function [qEq, info] = tensegFormFind(S, u, q, opts)
%TENSEGFORMFIND  Settle a structure onto its static equilibrium.
%
%   [qEq, info] = tensegFormFind(S, u, q, opts)
%
%   Dynamic relaxation: integrates the constrained DAE with artificial
%   mass-proportional damping f = -alpha*M*dq (bar lengths and boundary
%   conditions are kept exact by the Algorithm-1 corrector) until
%   ||dq|| and ||qdd|| fall below opts.tol.
%
%   opts: alpha (20), tol (1e-9), chunk (0.5 s), Tmax (30 s), fext (0)

if nargin < 4, opts = struct(); end
alpha = getOpt(opts, 'alpha', 20);
tol   = getOpt(opts, 'tol', 1e-9);
chunk = getOpt(opts, 'chunk', 0.5);
Tmax  = getOpt(opts, 'Tmax', 30);
fext0 = getOpt(opts, 'fext', zeros(S.nq,1));
if S.flexible, Mm = S.M0 + 1e-6*eye(S.nq); else, Mm = S.M; end
damp = @(t, qq, dq) fext0 - alpha*(Mm*dq);
sim = struct('T', chunk, 'dtControl', chunk, ...
    'integrator', struct('method','dopri5','rtol',1e-9,'atol',1e-11), ...
    'corrector', struct('enabled', true, 'energy', false));
dq = zeros(S.nq,1);
t = 0;
while t < Tmax
    out = tensegSimulate(S, q, dq, u, damp, sim);
    q = out.q(:,end);  dq = out.dq(:,end);
    t = t + chunk;
    qdd = tensegDaeDynamics(S, q, zeros(S.nq,1), u, fext0);
    if norm(dq) < tol && norm(qdd) < tol*1e3, break; end
end
qEq = q;
info = struct('time', t, 'dqNorm', norm(dq), 'qddNorm', norm(qdd), ...
    'shift', norm(qEq - S.q0));
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
