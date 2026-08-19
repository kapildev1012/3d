% smoke_diag.m (temporary) - isolate cause of obstacle penetration
addpath(genpath(fileparts(mfilename('fullpath'))));
modes = {'none','distance','relative'};
for relax = [false true]
  for a = 1:numel(modes)
    cfg = paper_config('relaxation', relax, 'T_end', 120, 'noise_mode', modes{a}, ...
        'snapshots', [0 20 40 60 80 100 120], 'verbose', false, ...
        'name', sprintf('relax=%d noise=%s', relax, modes{a}));
    res = run_experiment(cfg);
    m = res.met;
    fprintf('relax=%d noise=%-9s | minClear %7.2f | maxStr %5.2f | cleared %6.2f | Y(120) %6.1f | maxRelaxed %2d | sat %.1f%%\n', ...
        relax, modes{a}, min(m.min_clearance), max(m.ell_string_max), ...
        m.cleared_time, m.final_progress, max(m.n_relaxed), 100*res.out.sat_frac);
    fprintf('        centroid Y at %s = %s\n', mat2str(cfg.snapshots), mat2str(round(m.centroid_at,1)));
  end
end
