function T = tensegrity_analysis(q, S, cfg)
%TENSEGRITY_ANALYSIS  Form-finding / stability quantities of Sec. II-B and IV-A.
%
%   T = TENSEGRITY_ANALYSIS(q, S, cfg) evaluates, at the configuration q:
%
%     Eq. (4)  force-density matrix  D  (weighted Laplacian, member multiplicity
%              included)  with  omega_ij = f_ij / ell_ij
%     Eq. (5)  self-equilibrium residuals  D*x, D*y (, D*z)
%     Eq. (6)  connectivity matrices C_B, C_S, C and member coordinate
%              differences u = C*x, v = C*y, w = C*z
%     Eq. (7)  geometry matrix G = [Uu Vv Ww Uv Uw Vw]
%     Lemma 1  rank deficiency of D >= d+1 ; D >= 0 ; rank(G) = d(d+1)/2
%     Eq. (15) stiffness matrix K = Hessian of V, blockwise
%     Eq. (16) K_E, with K_G = kron(D, I_d) ; Lemma 2 condition k(alpha-1) >= 0
%
%   Output fields
%     omega, ell, D, D_res, rank_D, rank_def_D, eig_D, psd_D
%     C, CB, CS, G, G2, rank_G, rank_G2
%     K, KE, KG, eig_K, eig_KE, K_num (finite-difference cross-check)
%     nodal_residual (d x n, = u_i^t at q, the self-stress check)
%     lemma1, lemma2  (logical structs)
%
%   Note: the Lemma-1/2 analysis is a statement about the SELF-STRESS state, so
%   it is evaluated with the noiseless standard force law (Eq. (10)); at
%   q = q* every member is shorter than z1, hence the relaxation branch of
%   Eq. (19) is inactive there anyway.

d = cfg.d;
n = S.n;
M = size(S.members, 1);

cfg0 = cfg;  cfg0.relaxation = false;      % pure power law, Eq. (10)

dq  = q(:, S.members(:,1)) - q(:, S.members(:,2));
ell = sqrt(sum(dq.^2, 1)).';
f   = edge_force(ell, S.k, S.alpha, S.type, cfg0);         % Eq. (10)
omega = f ./ ell;                                          % tension coefficient
nhat  = dq ./ ell.';                                       % d x M

% ------------------------------------------------- Eq. (4): FDM (weighted Lap.)
D = zeros(n, n);
for mm = 1:M
    i = S.members(mm,1);  j = S.members(mm,2);
    D(i,j) = D(i,j) - omega(mm);
    D(j,i) = D(j,i) - omega(mm);
    D(i,i) = D(i,i) + omega(mm);
    D(j,j) = D(j,j) + omega(mm);
end
D = (D + D.')/2;                                    % enforce exact symmetry

% ------------------------------------------------- Eq. (5): self-equilibrium
D_res = D * q.';                                    % n x d, should be ~ 0
eig_D = sort(eig(D));
tolD  = 1e-9 * max(1, max(abs(omega)));
rank_D     = sum(abs(eig_D) > tolD);
rank_def_D = n - rank_D;
psd_D      = min(eig_D) > -tolD;

% ------------------------------------- Eq. (6): connectivity, coordinate diffs
C  = [S.CB; S.CS];
xyz = zeros(n, 3);
xyz(:, 1:d) = q.';
uu = C * xyz(:,1);
vv = C * xyz(:,2);
ww = C * xyz(:,3);
U = diag(uu);  V = diag(vv);  W = diag(ww);

% ------------------------------------------------- Eq. (7): geometry matrix
G  = [U*uu, V*vv, W*ww, U*vv, U*ww, V*ww];
G2 = [U*uu, V*vv, U*vv];            % the d = 2 sub-block (w == 0)
rank_G  = rank(G);
rank_G2 = rank(G2);

% -------------------------------- Eqs. (15),(16): stiffness matrices at q
K  = zeros(d*n, d*n);
KE = zeros(d*n, d*n);
for mm = 1:M
    i = S.members(mm,1);  j = S.members(mm,2);
    nn = nhat(:,mm) * nhat(:,mm).';
    w  = omega(mm);
    al = S.alpha(mm);
    Hij  = -w * (eye(d) + (al-1)*nn);
    Eij  = -w * (al-1) * nn;
    ri = (i-1)*d + (1:d);
    rj = (j-1)*d + (1:d);
    K(ri, rj)  = K(ri, rj)  + Hij;
    K(rj, ri)  = K(rj, ri)  + Hij;
    K(ri, ri)  = K(ri, ri)  - Hij;
    K(rj, rj)  = K(rj, rj)  - Hij;
    KE(ri, rj) = KE(ri, rj) + Eij;
    KE(rj, ri) = KE(rj, ri) + Eij;
    KE(ri, ri) = KE(ri, ri) - Eij;
    KE(rj, rj) = KE(rj, rj) - Eij;
end
KG = kron(D, eye(d));                                  % Eq. (16) text
K  = (K + K.')/2;  KE = (KE + KE.')/2;

% -------------------------- finite-difference cross-check of K = grad^2 V(q)
K_num = zeros(d*n, d*n);
h = 1e-5;
for a = 1:d*n
    qp = q;  qm = q;
    qp(a) = qp(a) + h;
    qm(a) = qm(a) - h;
    gp = -reshape(internal_force(qp, S, cfg0, zero_noise(S, cfg0)), [], 1);  % grad V = -u^t
    gm = -reshape(internal_force(qm, S, cfg0, zero_noise(S, cfg0)), [], 1);
    K_num(:, a) = (gp - gm) / (2*h);
end
K_num = (K_num + K_num.')/2;

% ------------------------------------------------- nodal residual (self-stress)
nodal_residual = internal_force(q, S, cfg0, zero_noise(S, cfg0));   % Eq. (14)

% ------------------------------------------------------------------- lemmas
lemma1 = struct( ...
    'rank_deficiency', rank_def_D, ...
    'need_rank_deficiency', d+1, ...
    'cond1_rank', rank_def_D >= d+1, ...
    'cond2_psd', psd_D, ...
    'min_eig_D', min(eig_D), ...
    'cond3_rank_G', rank_G2 == d*(d+1)/2, ...
    'rank_G', rank_G, 'rank_G2', rank_G2, 'need_rank_G', d*(d+1)/2);
lemma1.all = lemma1.cond1_rank && lemma1.cond2_psd && lemma1.cond3_rank_G;

lemma2 = struct( ...
    'string', cfg.kS*(cfg.alphaS-1), ...
    'bar',    cfg.kB*(cfg.alphaB-1), ...
    'ok',     cfg.kS*(cfg.alphaS-1) >= 0 && cfg.kB*(cfg.alphaB-1) >= 0);

T = struct('ell', ell, 'f', f, 'omega', omega, 'D', D, 'D_res', D_res, ...
           'eig_D', eig_D, 'rank_D', rank_D, 'rank_def_D', rank_def_D, ...
           'psd_D', psd_D, 'C', C, 'CB', S.CB, 'CS', S.CS, ...
           'G', G, 'G2', G2, 'rank_G', rank_G, 'rank_G2', rank_G2, ...
           'K', K, 'KE', KE, 'KG', KG, 'K_num', K_num, ...
           'eig_K', sort(eig(K)), 'eig_KE', sort(eig(KE)), ...
           'nodal_residual', nodal_residual, ...
           'lemma1', lemma1, 'lemma2', lemma2);
end

% -------------------------------------------------------------------------
function nz = zero_noise(S, cfg)
c2 = cfg;  c2.noise_mode = 'none';
nz = sample_noise(S, c2);
end
