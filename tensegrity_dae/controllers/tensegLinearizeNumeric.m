function lin = tensegLinearizeNumeric(S, q0, dq0, u0, f0, h)
%TENSEGLINEARIZENUMERIC  Central-difference linearisation of the DAE plant.
%
%   lin = tensegLinearizeNumeric(S, q0, dq0, u0, f0, h)
%
%   Same outputs as tensegLinearizer (A, Bsigma, Bls, Bf, ...). Used for
%   flexible-bar models and to validate the analytical Jacobians.
%   For 'sigma' inputs Bls is empty, for 'restlength' inputs Bsigma is the
%   derivative w.r.t. a force-density command at the same sigma0.

if nargin < 5 || isempty(f0), f0 = zeros(S.nq,1); end
if nargin < 6 || isempty(h), h = 1e-6; end
nq = S.nq;  ns = S.ns;
[qdd0, aux] = tensegDaeDynamics(S, q0, dq0, u0, f0);
sigma0 = aux.sigma;
F = @(q, dq, u, f) tensegDaeDynamics(S, q, dq, u, f);

Jq = zeros(nq);  Jdq = zeros(nq);  Jf = zeros(nq);
for i = 1:nq
    e = zeros(nq,1);  e(i) = h;
    Jq(:,i)  = (F(q0+e, dq0, u0, f0) - F(q0-e, dq0, u0, f0))/(2*h);
    Jdq(:,i) = (F(q0, dq0+e, u0, f0) - F(q0, dq0-e, u0, f0))/(2*h);
    Jf(:,i)  = (F(q0, dq0, u0, f0+e) - F(q0, dq0, u0, f0-e))/(2*h);
end
% force-density input at fixed q: Yhat-linear, but keep sigma >= 0 clamp
% out of the way by differencing around sigma0 + h
Js = zeros(nq, ns);  Jl = zeros(nq, ns);
for k = 1:ns
    e = zeros(ns,1);  e(k) = h;
    sp = tensegControl(S, 'sigma', sigma0 + e + h);
    sm = tensegControl(S, 'sigma', sigma0 - e + h);
    Js(:,k) = (F(q0, dq0, sp, f0) - F(q0, dq0, sm, f0))/(2*h);
    if strcmp(u0.mode, 'restlength')
        Jl(:,k) = (F(q0, dq0, tensegControl(S,'restlength',u0.ls+e), f0) - ...
                   F(q0, dq0, tensegControl(S,'restlength',u0.ls-e), f0))/(2*h);
    end
end
Z = zeros(nq);
lin = struct();
lin.A = [Z, eye(nq); Jq, Jdq];
lin.Bsigma = [zeros(nq,ns); Js];
lin.Bls = [zeros(nq,ns); Jl];
lin.Bf = [Z; Jf];
lin.dxi_dq = Jq;  lin.dxi_ddq = Jdq;  lin.dxi_dsigma = Js;  lin.dxi_dls = Jl;  lin.dxi_df = Jf;
cs = tensegConstraints(S, q0, dq0);
lin.Rq = cs.Rq;  lin.Q = cs.Q;  lin.Pc = [];
lin.qdd0 = qdd0;  lin.lambda0 = aux.lambda;
lin.q0 = q0;  lin.dq0 = dq0;  lin.u0 = u0;  lin.f0 = f0;  lin.sigma0 = sigma0;
end
