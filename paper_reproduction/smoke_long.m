% smoke_long.m (temporary) - longer horizon, spatial-stage comparison
addpath(genpath(fileparts(mfilename('fullpath'))));
R = paper_reference_data();
for relax = [false true]
    cfg = paper_config('relaxation', relax, 'T_end', 120, ...
                       'snapshots', [0 20 40 60 80 100 120], ...
                       'name', sprintf('long relax=%d', relax));
    res = run_experiment(cfg);
    m = res.met;
    fprintf('   centroid Y at %s :\n      %s\n', mat2str(cfg.snapshots), ...
            mat2str(round(m.centroid_at,1)));
    fprintf('   final centroid Y %.2f (free flight %.2f)\n', ...
            m.final_progress, free_flight_solution(cfg.T_end,cfg));
    fprintf('   cleared_time (all agents above y=90) = %.2f ; centroid crossed 90 at %.2f\n', ...
            m.cleared_time, m.centroid_cross_time);
    fprintf('   max string len %.2f | max n relaxed %d | min clearance %.2f\n', ...
            max(m.ell_string_max), max(m.n_relaxed), min(m.min_clearance));
    % steady speed in the last 20 s
    k = res.out.t > cfg.T_end-20;
    fprintf('   mean dY/dt over last 20 s = %.3f (v_inf = %.3f)\n', ...
            mean(m.speed_y(k)), cfg.v_inf);
    % final positions
    qf = res.out.q(:,:,end);
    [~,ord] = sort(qf(2,:),'descend');
    fprintf('   final q (sorted by Y):\n');
    fprintf('      (%7.2f,%7.2f)\n', qf(:,ord));
    fprintf('\n');
end
