% smoke_stage.m (temporary) - compare configuration at matched centroid progress
addpath(genpath(fileparts(mfilename('fullpath'))));
R = paper_reference_data();

function show(res, target, qpaper, tag)
    m = res.met; out = res.out;
    k = find(m.progress_y >= target, 1);
    if isempty(k), fprintf('  %s: never reached Y=%.2f\n', tag, target); return; end
    q = out.q(:,:,k);
    q = q - mean(q,2) + [mean(qpaper(1,:)); mean(qpaper(2,:))];   % align centroid
    % greedy nearest-neighbour pairing paper->sim
    used = false(1,size(q,2)); err = zeros(1,size(qpaper,2));
    for a = 1:size(qpaper,2)
        d = vecnorm(q - qpaper(:,a), 2, 1); d(used) = inf;
        [err(a), j] = min(d); used(j) = true;
    end
    fprintf('  %s: t=%.2f (paper label differs) | RMS node err vs paper = %.2f | max %.2f\n', ...
        tag, out.t(k), sqrt(mean(err.^2)), max(err));
    fprintf('      sim  x-range [%.1f %.1f] y-range [%.1f %.1f] maxStr %.1f minAgentDist %.1f\n', ...
        min(q(1,:)), max(q(1,:)), min(q(2,:)), max(q(2,:)), ...
        m.ell_string_max(k), m.min_agent_dist(k));
    fprintf('      papr x-range [%.1f %.1f] y-range [%.1f %.1f]\n', ...
        min(qpaper(1,:)), max(qpaper(1,:)), min(qpaper(2,:)), max(qpaper(2,:)));
end

for relax = [false true]
    cfg = paper_config('relaxation', relax, 'T_end', 120, 'verbose', false);
    res = run_experiment(cfg);
    fprintf('=== relaxation = %d ===\n', relax);
    if relax, cy = R.digitised.centroidY_fig6; qp = R.digitised.q_fig6;
    else,     cy = R.digitised.centroidY_fig5; qp = R.digitised.q_fig5; end
    for a = 2:4
        show(res, cy(a), qp(:,:,a), sprintf('paper panel %d (centroid Y=%.2f)', a, cy(a)));
    end
    fprintf('  max string length over whole run: %.2f at t=%.2f\n', ...
        max(res.met.ell_string_max), res.out.t(find(res.met.ell_string_max == max(res.met.ell_string_max),1)));
    fprintf('  min agent distance over run: %.2f\n', min(res.met.min_agent_dist));
end
