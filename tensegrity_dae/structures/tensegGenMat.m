function S = tensegGenMat(topo, opts)
%TENSEGGENMAT  Topology parsing & constant matrix generation (Module A).
%
%   S = tensegGenMat(topo)
%   S = tensegGenMat(topo, opts)
%
%   Builds everything that is constant for a Cartesian (non-minimal)
%   Lagrangian tensegrity model with q = vec(N), N = [n_1 ... n_n] (3 x n):
%
%     * Kronecker selectors (Eq. 1-5)
%         X_k    = (theta_k' Cb) (x) I3        b_k    = X_k q   (bar vector)
%         Xbar_k = 1/2 (theta_k' |Cb|) (x) I3  bbar_k = Xbar_k q (bar CoM)
%         Y_k    = (psi_k' Cs) (x) I3          s_k    = Y_k q   (string vector)
%         P_k    = (phi_k' Lpm) (x) I3         p_k    = P_k q   (point mass)
%       plus the cached products X_k'X_k, Y_k'Y_k and stacked forms
%       Xs = Cb (x) I3, Ys = Cs (x) I3 used for vectorised evaluation.
%     * Constant mass matrix M (Eq. 22) and its inverse / Cholesky factor
%     * Constant gravity vector G (Eq. 23)
%     * Linear boundary constraints A q - b = 0 (LinConst)
%
%   topo fields (n nodes, nb bars, ns strings, npm point masses):
%     N0               3 x n   initial nodal coordinates            (required)
%     Cb               nb x n  bar connectivity, -1 start / +1 end  (required)
%     Cs               ns x n  string connectivity                  (required)
%     barMass          nb x 1  [kg]   (or barDensity [kg/m^3] -> rho*pi*r^2*l)
%     barRadius        nb x 1  [m]    (default 0.01)
%     pmNodes          npm x 1 node index of each point mass  (or topo.Lpm)
%     pmMass           npm x 1 [kg]
%     stringStiffness  ns x 1  K_k [N/m]  (or stringYoung & stringRadius:
%                              K_k = E pi r^2 / l_s, Eq. 25)
%     stringRestLength ns x 1  l_s [m]  (or stringRestFraction of initial length)
%     stringDamping    ns x 1  c_k [N s/m]   (default 0)
%     fixedDofs        k x 2   [node, dim] pairs pinned at their N0 value
%     planeAxis        1..3    fix this coordinate of every node (2-D models)
%     barStiffness     nb x 1  K_b [N/m]  flexible bars only (or barYoung)
%     barPoisson       nb x 1  nu  (default 0.3) flexible bars only
%     name             char
%
%   opts fields:
%     gravity          3 x 1   (default [0;0;-9.806])
%     flexibleBars     logical (default false)  -> Section 3, Eq. 64-68
%
%   All scalar parameters are expanded to vectors automatically.

if nargin < 2, opts = struct(); end
opts = setDefault(opts, 'gravity', [0; 0; -9.806]);
opts = setDefault(opts, 'flexibleBars', false);

%% ---------------------------------------------------------------- topology
N0 = topo.N0;
if size(N0,1) ~= 3, error('tensegGenMat:N0', 'topo.N0 must be 3 x n.'); end
n  = size(N0, 2);
nq = 3*n;
Cb = full(topo.Cb);  Cs = full(topo.Cs);
nb = size(Cb, 1);    ns = size(Cs, 1);
if nb > 0 && size(Cb,2) ~= n, error('tensegGenMat:Cb', 'Cb must be nb x n.'); end
if ns > 0 && size(Cs,2) ~= n, error('tensegGenMat:Cs', 'Cs must be ns x n.'); end

if isfield(topo, 'Lpm') && ~isempty(topo.Lpm)
    Lpm = full(topo.Lpm);
    pmMass = expandTo(getField(topo, 'pmMass', 0), size(Lpm,1));
elseif isfield(topo, 'pmNodes') && ~isempty(topo.pmNodes)
    pmNodes = topo.pmNodes(:);
    Lpm = zeros(numel(pmNodes), n);
    Lpm(sub2ind(size(Lpm), (1:numel(pmNodes))', pmNodes)) = 1;
    pmMass = expandTo(getField(topo, 'pmMass', 0), numel(pmNodes));
else
    Lpm = zeros(0, n);  pmMass = zeros(0,1);
end
npm = size(Lpm, 1);

q0 = N0(:);
I3 = speye(3);

%% --------------------------------------------- Kronecker selectors (Eq.1-5)
Xk = cell(nb,1);  Xbark = cell(nb,1);  XtX = cell(nb,1);
for k = 1:nb
    Xk{k}    = kron(sparse(Cb(k,:)), I3);
    Xbark{k} = 0.5*kron(sparse(abs(Cb(k,:))), I3);
    XtX{k}   = Xk{k}'*Xk{k};
end
Yk = cell(ns,1);  YtY = cell(ns,1);
for k = 1:ns
    Yk{k}  = kron(sparse(Cs(k,:)), I3);
    YtY{k} = Yk{k}'*Yk{k};
end
Pk = cell(npm,1);
for k = 1:npm
    Pk{k} = kron(sparse(Lpm(k,:)), I3);
end
Xs  = kron(sparse(Cb), I3);            % 3nb x 3n  stacked X_k
Xbs = 0.5*kron(sparse(abs(Cb)), I3);   % 3nb x 3n  stacked Xbar_k
Ys  = kron(sparse(Cs), I3);            % 3ns x 3n  stacked Y_k
% component selectors used by the analytic linearisation: Xdim{d} = Cb (x) e_d'
Xdim = cell(3,1);
for d = 1:3
    ed = sparse(1, d, 1, 1, 3);
    Xdim{d} = kron(sparse(Cb), ed);    % nb x 3n
end

%% ------------------------------------------------------- bar properties
lb = zeros(nb,1);
for k = 1:nb, lb(k) = norm(Xk{k}*q0); end
rb = expandTo(getField(topo, 'barRadius', 0.01), nb);
if isfield(topo, 'barMass')
    mb = expandTo(topo.barMass, nb);
elseif isfield(topo, 'barDensity')
    mb = expandTo(topo.barDensity, nb).*pi.*rb.^2.*lb;
else
    mb = zeros(nb,1);
end
Ib = mb/12.*(3*rb.^2 + lb.^2);         % transverse cylinder inertia

%% --------------------------------------------- mass matrix M (Eq. 22)
M0 = sparse(nq, nq);                   % translational + point-mass part
Mrot = sparse(nq, nq);                 % (I_b/l_b^2) X'X part
for k = 1:nb
    M0   = M0 + mb(k)*(Xbark{k}'*Xbark{k});
    Mrot = Mrot + Ib(k)/lb(k)^2*XtX{k};
end
for k = 1:npm
    M0 = M0 + pmMass(k)*(Pk{k}'*Pk{k});
end
M = full(M0 + Mrot);  M = (M + M')/2;
[Mchol, p] = chol(M);
if p ~= 0 && ~opts.flexibleBars
    error('tensegGenMat:mass', ['Mass matrix is not positive definite: every node ' ...
        'must belong to a bar with mass > 0 or carry a point mass.']);
end
Minv = [];
if p == 0
    Minv = Mchol\(Mchol'\eye(nq));  Minv = (Minv + Minv')/2;
end

%% --------------------------------------------- gravity vector G (Eq. 23)
g = opts.gravity(:);
G = zeros(nq,1);
for k = 1:nb,  G = G + mb(k)*(Xbark{k}'*g); end
for k = 1:npm, G = G + pmMass(k)*(Pk{k}'*g); end

%% --------------------------------- linear boundary constraints A q = b
rows = zeros(0,1);
if isfield(topo, 'fixedDofs') && ~isempty(topo.fixedDofs)
    fd = topo.fixedDofs;
    rows = [rows; 3*(fd(:,1)-1) + fd(:,2)];
end
if isfield(topo, 'planeAxis') && ~isempty(topo.planeAxis)
    rows = [rows; 3*((1:n)'-1) + topo.planeAxis];
end
rows = unique(rows);
nbc = numel(rows);
A = sparse(1:nbc, rows, 1, nbc, nq);
b = A*q0;

%% --------------------------------------------- string properties (Eq.25)
ls_init = zeros(ns,1);
for k = 1:ns, ls_init(k) = norm(Yk{k}*q0); end
if isfield(topo, 'stringRestLength')
    ls0 = expandTo(topo.stringRestLength, ns);
elseif isfield(topo, 'stringRestFraction')
    ls0 = expandTo(topo.stringRestFraction, ns).*ls_init;
else
    ls0 = ls_init;
end
if isfield(topo, 'stringStiffness')
    Ks = expandTo(topo.stringStiffness, ns);
elseif isfield(topo, 'stringYoung')
    Ks = expandTo(topo.stringYoung, ns).*pi.*expandTo(getField(topo,'stringRadius',1e-3),ns).^2./ls0;
else
    Ks = zeros(ns,1);
end
cs = expandTo(getField(topo, 'stringDamping', 0), ns);

%% --------------------------------------------- flexible-bar properties
if isfield(topo, 'barStiffness')
    Kb = expandTo(topo.barStiffness, nb);
elseif isfield(topo, 'barYoung')
    Kb = expandTo(topo.barYoung, nb).*pi.*rb.^2./lb;
else
    Kb = 1e6*ones(nb,1);
end
nu = expandTo(getField(topo, 'barPoisson', 0.3), nb);

%% --------------------------------------------- pack
S = struct();
S.name = getField(topo, 'name', 'tensegrity');
S.n = n;  S.nq = nq;  S.nb = nb;  S.ns = ns;  S.npm = npm;  S.nbc = nbc;
S.N0 = N0;  S.q0 = q0;
S.Cb = Cb;  S.Cs = Cs;  S.Lpm = Lpm;
S.Xk = Xk;  S.Xbark = Xbark;  S.Yk = Yk;  S.Pk = Pk;
S.XtX = XtX;  S.YtY = YtY;
S.Xs = Xs;  S.Xbs = Xbs;  S.Ys = Ys;  S.Xdim = Xdim;
S.bar = struct('m', mb, 'r', rb, 'l', lb, 'I', Ib, 'Kb', Kb, 'nu', nu);
S.pm  = struct('m', pmMass);
S.str = struct('K', Ks, 'ls0', ls0, 'c', cs, 'lenInit', ls_init);
S.M = M;  S.Minv = Minv;  S.Mchol = Mchol;  S.M0 = full(M0);
S.G = G;  S.g = g;
S.A = A;  S.b = b;  S.bcRows = rows;
S.flexible = logical(opts.flexibleBars);
end

% ---------------------------------------------------------------- helpers
function v = expandTo(v, n)
v = v(:);
if numel(v) == 1, v = repmat(v, n, 1); end
if numel(v) ~= n
    error('tensegGenMat:size', 'Parameter has %d entries, expected %d.', numel(v), n);
end
end

function v = getField(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function s = setDefault(s, f, d)
if ~isfield(s, f) || isempty(s.(f)), s.(f) = d; end
end
