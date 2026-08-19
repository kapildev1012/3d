% smoke_test.m  (temporary) - core sanity check
addpath(genpath(fileparts(mfilename('fullpath'))));
cfg = paper_config();
S   = build_formation(cfg);
fprintf('n=%d  nS=%d  nB=%d  M=%d\n', S.n, S.nS, S.nB, size(S.members,1));
fprintf('q0 x = %s\n', mat2str(unique(S.q0(1,:))));
fprintf('q0 y = %s\n', mat2str(unique(S.q0(2,:))));
fprintf('multiplicity matrix A:\n'); disp(S.A);

T = tensegrity_analysis(S.q0, S, cfg);
fprintf('unique member lengths: %s\n', mat2str(unique(round(T.ell,6))'));
fprintf('omega string = %.6f  omega bar = %.6f\n', ...
        T.omega(find(S.type,1)), T.omega(find(~S.type,1)));
fprintf('max |nodal residual| (self-stress) = %.3e\n', max(abs(T.nodal_residual(:))));
fprintf('max |D*[x y]|                      = %.3e\n', max(abs(T.D_res(:))));
fprintf('rank(D) = %d   rank deficiency = %d (need >= %d)\n', ...
        T.rank_D, T.rank_def_D, cfg.d+1);
fprintf('eig(D) = %s\n', mat2str(round(T.eig_D',6)));
fprintf('min eig(D) = %.3e  -> PSD: %d\n', min(T.eig_D), T.psd_D);
fprintf('rank(G) full = %d, rank(G) d=2 block = %d (need %d)\n', ...
        T.rank_G, T.rank_G2, cfg.d*(cfg.d+1)/2);
fprintf('Lemma 1 all conditions: %d\n', T.lemma1.all);
fprintf('Lemma 2: kS(aS-1)=%.4f  kB(aB-1)=%.4f  ok=%d\n', ...
        T.lemma2.string, T.lemma2.bar, T.lemma2.ok);
fprintf('||K - K_num||_inf / ||K||_inf = %.3e\n', ...
        norm(T.K - T.K_num, inf)/norm(T.K, inf));
fprintf('min eig(K) = %.4e   min eig(KE) = %.4e\n', min(T.eig_K), min(T.eig_KE));
fprintf('Eq21: kB/kS table = %.4f   Eq21 = %.4f\n', cfg.kB/cfg.kS, ...
        -cfg.lS^(cfg.alphaS-cfg.alphaB)*sqrt(2)^(1-cfg.alphaB));

%% same analysis with the EXACT Eq.(21)-consistent kS (unrounded)
kS_exact = -cfg.kB / (cfg.lS^(cfg.alphaS-cfg.alphaB) * sqrt(2)^(1-cfg.alphaB));
fprintf('\n--- exact kS from Eq.(21) = %.10f (table: %.4f) ---\n', kS_exact, cfg.kS);
cfg2 = paper_config('kS', kS_exact);
S2   = build_formation(cfg2);
T2   = tensegrity_analysis(S2.q0, S2, cfg2);
fprintf('max |nodal residual| = %.3e\n', max(abs(T2.nodal_residual(:))));
fprintf('rank(D) = %d  rank deficiency = %d\n', T2.rank_D, T2.rank_def_D);
fprintf('eig(D) = %s\n', mat2str(round(T2.eig_D',8)));
fprintf('min eig(D) = %.3e  PSD: %d\n', min(T2.eig_D), T2.psd_D);
fprintf('rank(G2) = %d  Lemma1 all = %d\n', T2.rank_G2, T2.lemma1.all);
fprintf('min eig(K) = %.4e  min eig(KE) = %.4e\n', min(T2.eig_K), min(T2.eig_KE));
fprintf('null(D) basis vs [1 x y]: residual = %.3e\n', ...
        norm(null(T2.D)*(null(T2.D)\[ones(S2.n,1) S2.q0.']) - [ones(S2.n,1) S2.q0.'], inf));
