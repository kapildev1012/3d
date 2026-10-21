function red = tensegReduce(lin, B, opts)
%TENSEGREDUCE  Constraint-nullspace projection + minimal (controllable) realisation.
%
%   red = tensegReduce(lin)            uses lin.Bsigma as input matrix
%   red = tensegReduce(lin, B, opts)
%
%   Step 1 - tangent space of the constraint manifold. With
%   N_c = null(Rq(q0)), admissible perturbations satisfy
%       dq_  = N_c z_q,
%       ddq_ = N_c z_v - Rq^+ Q(dq0) N_c z_q    (from Rq ddq_ + Q dq_ = 0)
%   i.e. x = T z, T = [N_c 0; -Rq^+ Q N_c  N_c]. Reduced model
%       z_dot = T^+ A T z + T^+ B du.
%   (At an equilibrium dq0 = 0, T = blkdiag(N_c, N_c).)
%
%   Step 2 - minimal realisation. Rigid-body modes of a free structure
%   cannot be reached by internal string forces, so the 2(3n-m)-state
%   tangent model is still not controllable. The controllable subspace
%   V = range[B, AB, A^2B, ...] is computed with the orthogonal staircase
%   algorithm and the model is projected:  Ac = V'Ar V,  Bc = V'Br.
%
%   Step 3 - weakly controllable modes (Hautus margin below
%   opts.hautusTol) are split off with an ordered Schur form (see code).
%
%   opts.tol        relative rank tolerance (default 1e-6)
%   opts.hautusTol  relative Hautus margin  (default 1e-3, 0 disables)
%
%   red fields: Nc, T, Tpinv, Ar, Br, invarianceResidual, V, Ac, Bc,
%   Tc = T*V (full-state basis of controllable coordinates), Tcpinv,
%   nz (tangent dim), nc (controllable dim).

if nargin < 2 || isempty(B), B = lin.Bsigma; end
if nargin < 3, opts = struct(); end
tol = 1e-6;
if isfield(opts, 'tol'), tol = opts.tol; end

Rq = lin.Rq;  nq = size(lin.A,1)/2;
if isempty(Rq)
    Nc = eye(nq);  RqP = zeros(nq,0);  Qm = zeros(0,nq);
else
    [~, Sv, V0] = svd(Rq);
    sv = diag(Sv);
    r = sum(sv > max(size(Rq))*eps(max(sv))*10);
    Nc = V0(:, r+1:end);
    RqP = pinv(Rq);  Qm = lin.Q;
end
nz = size(Nc, 2);
T = [Nc, zeros(nq,nz); -RqP*Qm*Nc, Nc];
Tpinv = pinv(T);
AT = lin.A*T;
Ar = Tpinv*AT;
Br = Tpinv*B;
invRes = norm(AT - T*Ar)/max(norm(AT), eps);

% controllable subspace: orthogonal staircase (Kalman) decomposition.
% Each step compresses the remaining input matrix with an SVD and peels off
% its range; only orthogonal transformations are used, so rank decisions
% are well conditioned (unlike powers A^k B).
nr = size(Ar,1);
Z = eye(nr);  Arem = Ar;  Brem = Br;  nc = 0;
scale = max([norm(Ar), norm(Br), 1]);
while nc < nr
    [U, ~, ~] = svd(Brem);
    sv = svd(Brem);
    r = sum(sv > tol*scale);
    if r == 0, break; end
    Arem = U'*Arem*U;
    Z(:, nc+1:end) = Z(:, nc+1:end)*U;
    nc = nc + r;
    Brem = Arem(r+1:end, 1:r);
    Arem = Arem(r+1:end, r+1:end);
    if isempty(Arem), break; end
end
Vc = Z(:, 1:nc);

% Step 3 - prune numerically uncontrollable modes (Hautus test). Infinitesimal
% mechanisms of a prestressed tensegrity do not change any string length to
% first order, so m'*Yhat = 0 and they cannot be moved by dsigma; with
% rounded/imperfect geometry they leak into the staircase with a tiny gain.
% Modes with  smin([Ac - lam I, Bc])/||Bc|| < hautusTol  are moved to the
% leading block of an ordered real Schur form and dropped; the kept block
% is an exact quotient system (its dynamics do not depend on the dropped
% coordinates).
hautusTol = 1e-3;
if isfield(opts, 'hautusTol'), hautusTol = opts.hautusTol; end
Ac = Vc'*Ar*Vc;  Bc = Vc'*Br;
nPruned = 0;  hautus = [];
if hautusTol > 0 && nc > 0
    [U, Ts] = schur(Ac, 'real');
    lam = ordeig(Ts);
    hautus = zeros(numel(lam), 1);
    nB = max(norm(Bc), eps);
    for j = 1:numel(lam)
        hautus(j) = min(svd([Ac - lam(j)*eye(nc), Bc]))/nB;
    end
    weak = hautus < hautusTol;
    if any(weak) && ~all(weak)
        [U, Ts] = ordschur(U, Ts, weak);
        nPruned = sum(weak);
        Vc = Vc*U(:, nPruned+1:end);
    end
end
red = struct();
red.nPruned = nPruned;  red.hautus = hautus;
red.Nc = Nc;  red.T = T;  red.Tpinv = Tpinv;
red.Ar = Ar;  red.Br = Br;  red.invarianceResidual = invRes;
red.V = Vc;  red.Ac = Vc'*Ar*Vc;  red.Bc = Vc'*Br;
red.Tc = T*Vc;  red.Tcpinv = Vc'*Tpinv;
red.nz = nz;  red.nc = size(Vc,2);
end
