function res = run_experiment(cfg, label)
%RUN_EXPERIMENT  One reproducible experiment: build, analyse, simulate, measure.
%
%   res = RUN_EXPERIMENT(cfg)
%   res = RUN_EXPERIMENT(cfg, label)
%
%   Steps (see PAPER_ANALYSIS.md Sec. 7):
%     1  seed the RNG                                    (reproducibility, A7)
%     2  build the tiled tensegrity formation            (Sec. V)
%     3  tensegrity / stability analysis at q*            Eqs. (4)-(7),(15),(16)
%     4  integrate the closed loop                        Eqs. (8),(9)
%     5  compute metrics
%
%   res fields: cfg, S, T (analysis), out (histories), met (metrics), label.

if nargin < 2 || isempty(label), label = cfg.name; end

rng(cfg.seed, 'twister');                                    % step 1

S = build_formation(cfg);                                    % step 2
T = tensegrity_analysis(S.q0, S, cfg);                       % step 3

if cfg.verbose
    fprintf('[%s] n=%d agents, %d strings, %d bars | relaxation=%d | sigma=%.2f\n', ...
        label, S.n, S.nS, S.nB, cfg.relaxation, cfg.sigma);
    fprintf('        self-stress residual %.2e | rank def(D)=%d | rank(G)=%d | Lemma1=%d\n', ...
        max(abs(T.nodal_residual(:))), T.rank_def_D, T.rank_G2, T.lemma1.all);
end

t0  = tic;
out = simulate(cfg, S);                                      % step 4
wall = toc(t0);

met = formation_metrics(out, S, cfg);                        % step 5

if cfg.verbose
    fprintf('        %d steps in %.2f s | centroid Y(end)=%.2f | min clearance=%.2f | ', ...
        cfg.n_steps, wall, met.final_progress, min(met.min_clearance));
    fprintf('min agent dist=%.2f | sat %.1f%%\n', ...
        min(met.min_agent_dist), 100*out.sat_frac);
end

res = struct('cfg', cfg, 'S', S, 'T', T, 'out', out, 'met', met, ...
             'label', label, 'wall_time', wall);
end
