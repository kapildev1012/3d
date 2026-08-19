function met = formationMetrics3D(out, L, cfg)
%FORMATIONMETRICS3D  Scalar summary metrics for comparison.
%
%   met = FORMATIONMETRICS3D(out, L, cfg)

N = size(out.q, 3);
met.t = out.t;
met.centroid = out.centroid;
met.progress_y = out.centroid(2,:);

% Velocity of centroid
met.speed = zeros(1, N);
for s = 1:N
    met.speed(s) = norm(mean(out.v(:,:,s), 2));
end

def = deformationAnalysis3D(out, L, cfg);
met.formation_error = def.formation_error;
met.recovery_pct = def.recovery_pct;
met.recovery_pct_obs1 = def.recovery_pct_obs1;
met.max_strain = max(def.strain, [], 1);
met.min_clearance = min(out.clearance, [], 1);
met.n_relaxed = sum(out.relaxed, 1);

end
