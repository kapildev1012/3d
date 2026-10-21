function lin = tensegLinearizer(S, q0, dq0, u0, f0)
%TENSEGLINEARIZER  Analytical linearisation of qdd = xi(q,dq,sigma,f) (Sec. 2.5).
%
%   lin = tensegLinearizer(S, q0, dq0, u0, f0)
%
%   Linearises about (q0, dq0, u0, f0) into
%       d/dt [dq_; ddq_] = A [dq_; ddq_] + B_sigma dsigma + B_f df   (+ B_ls dls)
%
%   With P_c = I - Rq'(Rq M^-1 Rq')^-1 Rq M^-1 (Eq. 59-63):
%     dxi/df     =  M^-1 P_c
%     dxi/dsigma = -M^-1 P_c Yhat(q0)
%     dxi/dls    =  M^-1 P_c Yhat(q0) diag(K/||s||)      (rest-length input)
%     dxi/ddq    =  M^-1 P_c dfd/ddq - M^-1 Rq'(Rq M^-1 Rq')^-1 dxi2/ddq,
%                   dxi2/ddq = 4 dq0'X_k'X_k  (= 2 x bar rows of Rq(dq0))
%     dxi/dq     :  column-by-column chain rule through Rq(q), (Rq M^-1 Rq')^-1
%                   and xi1(q):  dxi/dq_i = M^-1 (dxi1_i + dRq_i' lambda + Rq' dlambda_i)
%   where dxi1/dq = -sum sigma_k Y_k'Y_k (+ material stiffness when u0 is a
%   rest-length input, since sigma then depends on q) + dfd/dq.
%
%   The string damping force fd is part of the plant (it enters A); f is
%   the external force input. Flexible-bar models are linearised
%   numerically (tensegLinearizeNumeric).
%
%   lin fields: A (6n x 6n), Bsigma (6n x ns), Bls (6n x ns), Bf (6n x 3n),
%   dxi_dq, dxi_ddq, dxi_dsigma, dxi_dls, dxi_df, Pc, Rq, Q, qdd0, lambda0,
%   q0, dq0, u0, f0, sigma0.

if nargin < 5 || isempty(f0), f0 = zeros(S.nq,1); end
if S.flexible
    lin = tensegLinearizeNumeric(S, q0, dq0, u0, f0);
    return;
end
nq = S.nq;  ns = S.ns;  nbc = S.nbc;  nb = S.nb;
Minv = S.Minv;

st = tensegStrings(S, q0, dq0, u0);
cs = tensegConstraints(S, q0, dq0);
Rq = cs.Rq;  m = cs.m;
sigma = st.sigma;

% ---------------------------------------------------- xi1 and its Jacobians
f   = f0(:) + st.fd;
xi1 = -st.fint + S.G + f;
[dfd_dq, dfd_ddq] = dampingJacobians(S, st);
sigRep = kron(sigma, ones(3,1));
Kgeo = full(S.Ys'*spdiags(sigRep, 0, 3*ns, 3*ns)*S.Ys);   % sum sigma_k Y_k'Y_k
dxi1_dq = -Kgeo + dfd_dq;
dsig_dls = zeros(ns,1);
if strcmp(u0.mode, 'restlength')
    lenS = max(st.len, eps);
    coef = S.str.K.*u0.ls(:)./lenS.^3.*st.taut;              % d sigma/d||s|| / ||s||
    dxi1_dq = dxi1_dq - st.Yhat*diag(coef)*st.Yhat';         % material stiffness
    dsig_dls = -S.str.K./lenS.*st.taut;
end
dxi1_ddq = dfd_ddq;

% ---------------------------------------------------- constrained solution
if m > 0
    MRt = Minv*Rq';
    Sc  = Rq*MRt;
    Sinv = robustInv(Sc);
    y = cs.xi2 + Rq*(Minv*xi1);
    lambda = -Sinv*y;
    Pc = eye(nq) - Rq'*Sinv*Rq*Minv;
else
    MRt = zeros(nq,0);  Sinv = zeros(0);  lambda = zeros(0,1);  Pc = eye(nq);
end
qdd0 = Minv*(xi1 + Rq'*lambda);

dxi_df  = Minv*Pc;
dxi_dsigma = -Minv*Pc*st.Yhat;
dxi_dls = Minv*Pc*(-st.Yhat*diag(dsig_dls));

% velocity Jacobian
if m > 0
    Bd = reshape(S.Xs*dq0, 3, nb);
    dxi2_ddq = [zeros(nbc,nq); 4*full(colBlock(Bd)'*S.Xs)];
    if nb == 0, dxi2_ddq = zeros(nbc,nq); end
    dxi_ddq = Minv*Pc*dxi1_ddq - MRt*Sinv*dxi2_ddq;
else
    dxi_ddq = Minv*dxi1_ddq;
end

% position Jacobian, column by column (paper, p. 15)
dxi_dq = zeros(nq,nq);
for i = 1:nq
    node = ceil(i/3);  dim = i - 3*(node-1);
    dxi1_i = dxi1_dq(:,i);
    if m > 0
        dRq = zeros(m, nq);
        if nb > 0
            dRq(nbc+1:end, :) = 2*full(diag(S.Cb(:,node))*S.Xdim{dim});
        end
        dS  = dRq*MRt + MRt'*dRq';
        dy  = dRq*(Minv*xi1) + Rq*(Minv*dxi1_i);
        dlam = Sinv*dS*(Sinv*y) - Sinv*dy;
        dxi_dq(:,i) = Minv*(dxi1_i + dRq'*lambda + Rq'*dlam);
    else
        dxi_dq(:,i) = Minv*dxi1_i;
    end
end

Z = zeros(nq);
lin = struct();
lin.A      = [Z, eye(nq); dxi_dq, dxi_ddq];
lin.Bsigma = [zeros(nq,ns); dxi_dsigma];
lin.Bls    = [zeros(nq,ns); dxi_dls];
lin.Bf     = [Z; dxi_df];
lin.dxi_dq = dxi_dq;  lin.dxi_ddq = dxi_ddq;
lin.dxi_dsigma = dxi_dsigma;  lin.dxi_dls = dxi_dls;  lin.dxi_df = dxi_df;
lin.Pc = Pc;  lin.Rq = Rq;  lin.Q = cs.Q;
lin.qdd0 = qdd0;  lin.lambda0 = lambda;
lin.q0 = q0;  lin.dq0 = dq0;  lin.u0 = u0;  lin.f0 = f0;  lin.sigma0 = sigma;
end

% ------------------------------------------------------------------ helpers
function [Jq, Jdq] = dampingJacobians(S, st)
nq = S.nq;
Jq = zeros(nq);  Jdq = zeros(nq);
c = S.str.c(:).*st.tautDamp;
for k = find(c ~= 0)'
    s = st.s(:,k);  w = st.ds(:,k);
    ss = s'*s;  ws = w'*s;
    dg_du = -c(k)*((s*w' + ws*eye(3))/ss - 2*ws*(s*s')/ss^2);
    dg_dw = -c(k)*(s*s')/ss;
    Y = S.Yk{k};
    Jq  = Jq  + full(Y'*dg_du*Y);
    Jdq = Jdq + full(Y'*dg_dw*Y);
end
end

function X = robustInv(A)
if rcond(A) > 1e-12, X = inv(A); else, X = pinv(A); end
end

function B = colBlock(V)
k = size(V,2);
B = sparse((1:3*k)', kron((1:k)', ones(3,1)), V(:), 3*k, k);
end
