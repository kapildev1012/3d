% smoke_sim.m (temporary) - first closed-loop runs
addpath(genpath(fileparts(mfilename('fullpath'))));
R = paper_reference_data();

for relax = [false true]
    cfg = paper_config('relaxation', relax, 'name', sprintf('relax=%d', relax));
    res = run_experiment(cfg);
    m = res.met;
    fprintf('   centroid Y at t=[0 15 30 40] : %s\n', mat2str(round(m.centroid_at,2)));
    if relax
        fprintf('   paper (Fig 6)               : %s\n', mat2str(R.digitised.centroidY_fig6));
    else
        fprintf('   paper (Fig 5)               : %s\n', mat2str(R.digitised.centroidY_fig5));
    end
    tt = [0 15 30 40];
    fprintf('   free flight analytic        : %s\n', ...
        mat2str(round(arrayfun(@(t) free_flight_solution(t,cfg), tt),2)));
    fprintf('   max string length %.2f | n relaxed max %d | shape err end %.3f\n', ...
        max(m.ell_string_max), max(m.n_relaxed), m.shape_err(end));
    fprintf('   min clearance %.3f | min agent dist %.3f | cleared t %.2f\n\n', ...
        min(m.min_clearance), min(m.min_agent_dist), m.cleared_time);
end
