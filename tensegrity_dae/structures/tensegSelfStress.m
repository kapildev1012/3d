function [sigma, ls, lambda, res] = tensegSelfStress(S, q, sigmaTarget, fext)
%TENSEGSELFSTRESS  Prestress equilibrium: force densities and rest lengths.
%
%   [sigma, ls, lambda, res] = tensegSelfStress(S, q, sigmaTarget, fext)
%
%   Finds force densities sigma (and constraint multipliers lambda) so that
%   q is a static equilibrium of the DAE (qdd = 0, dq = 0):
%       -Yhat(q) sigma + G + fext + Rq' lambda = 0,
%   choosing, among all solutions, the one closest to sigmaTarget
%   (least squares on sigma). Rest lengths realising sigma under the
%   unilateral law (Eq. 25) are  ls = ||s|| (1 - sigma/K).
%
%   sigmaTarget : ns x 1 or scalar (e.g. pretension./length)
%   fext        : 3n x 1 external force at equilibrium (default 0)
%   res         : equilibrium residual norm (should be ~1e-12)
%
%   A warning is issued if any sigma < 0 (no all-tension equilibrium exists
%   near sigmaTarget for this geometry/loading).

if nargin < 4 || isempty(fext), fext = zeros(S.nq,1); end
ns = S.ns;
sigmaTarget = sigmaTarget(:);
if numel(sigmaTarget) == 1, sigmaTarget = repmat(sigmaTarget, ns, 1); end

st = tensegStrings(S, q, zeros(S.nq,1), tensegControl(S, 'sigma', zeros(ns,1)));
cs = tensegConstraints(S, q, zeros(S.nq,1));
m = cs.m;
Aeq = [-st.Yhat, cs.Rq'];            % [sigma; lambda] unknowns
beq = -(S.G + fext);
% general solution y = yp + Nz
yp = pinv(Aeq)*beq;
[~, Sv, V] = svd(Aeq);
sv = diag(Sv);
r = sum(sv > max(size(Aeq))*eps(max([sv; 1]))*1e3);
N = V(:, r+1:end);
% min || sigma(y) - target ||
Ns = N(1:ns, :);
if isempty(N)
    y = yp;
else
    z = pinv(Ns)*(sigmaTarget - yp(1:ns));
    y = yp + N*z;
end
% If q admits no prestressed equilibrium near the target (e.g. coordinates
% rounded in the model file), fall back to the closest least-squares
% self-stress: add the next right singular vector of Aeq and scale it to
% the target. q is then only an approximate equilibrium; run
% tensegFormFind to settle onto the exact one.
if norm(y(1:ns) - sigmaTarget) > 0.5*norm(sigmaTarget)
    N2 = V(:, max(r,1):end);
    if r == size(V,2), N2 = V(:, end); end
    z = pinv(N2(1:ns,:))*(sigmaTarget - yp(1:ns));
    y = yp + N2*z;
    warning('tensegSelfStress:approximate', ['q is not an exact prestressed ' ...
        'equilibrium (residual %.3g); returning nearest self-stress - use ' ...
        'tensegFormFind to settle the geometry.'], norm(Aeq*y - beq));
end
sigma = y(1:ns);
lambda = y(ns+1:ns+m);
res = norm(Aeq*y - beq);
if any(sigma < -1e-9)
    warning('tensegSelfStress:negative', ...
        '%d string(s) need compression (sigma < 0) at this equilibrium.', sum(sigma < -1e-9));
end
K = S.str.K;
ls = st.len.*(1 - sigma./max(K, eps));
end
