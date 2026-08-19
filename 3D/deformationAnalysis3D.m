function def = deformationAnalysis3D(out, L, cfg)
%DEFORMATIONANALYSIS3D  Analyse member strains, deformation error, and recovery.
%
%   def = DEFORMATIONANALYSIS3D(out, L, cfg)
%
%   Computes:
%     def.strain           M x N  member strains relative to nominal l0
%     def.strain_bars      nBars x N
%     def.strain_strings   nStrings x N
%     def.formation_error  1 x N  Procrustes RMS node displacement
%     def.recovery_pct     overall end-of-run recovery
%     def.recovery_pct_obs1  recovery in free terrain between Obs 1 and Obs 2
%
%   Recovery Definition
%   -------------------
%   The lander sits on uneven terrain continuously, so the Procrustes error
%   has a non-zero "terrain baseline" err_terrain even in free terrain.
%   True recovery is how much of the ADDITIONAL deformation caused by the
%   obstacle (err_peak - err_terrain) has been eliminated after clearing.
%
%       recovery_pct = 100 * (1 - (err_after - err_terrain)
%                                  / (err_peak - err_terrain))
%
%   This equals 100% when the lander returns to its normal terrain-rolling
%   deformation level (no excess obstacle-induced deformation remaining).

N = size(out.q, 3);
M = size(L.members, 1);

% ---------------------------------------------------------------
% Member strains: epsilon = (l - l0) / l0
% ---------------------------------------------------------------
def.strain = zeros(M, N);
for s = 1:N
    def.strain(:, s) = (out.ell(:, s) - L.l0) ./ L.l0;
end

def.strain_bars    = def.strain(~L.isString, :);
def.strain_strings = def.strain( L.isString, :);

% ---------------------------------------------------------------
% Procrustes RMS formation error (shape-only, removes rigid motion)
% ---------------------------------------------------------------
def.formation_error = zeros(1, N);
P = L.q0 - mean(L.q0, 2);   % 3 x n, centred nominal shape

for s = 1:N
    Q  = out.q(:,:,s) - mean(out.q(:,:,s), 2);
    H  = P * Q.';
    [U, ~, V] = svd(H);
    R  = V * diag([1, 1, det(V * U.')]) * U.';
    def.formation_error(s) = sqrt(mean(sum((Q - R * P).^2, 1)));
end

% ---------------------------------------------------------------
% Compute terrain baseline and obstacle peaks from Y-position
% ---------------------------------------------------------------
y = out.centroid(2, :);

% Obstacle sensing zones (+/- 1.5x radius around each obstacle Y-centre)
nO = size(cfg.obstacles, 1);
in_obs = false(1, N);
for o = 1:nO
    oy = cfg.obstacles(o, 2);
    or_ = cfg.obstacles(o, 4) + cfg.ry;
    in_obs = in_obs | (abs(y - oy) <= or_);
end
free_terrain = ~in_obs;

% Terrain baseline: median formation error when NOT near any obstacle
if sum(free_terrain) >= 5
    err_terrain = median(def.formation_error(free_terrain));
else
    err_terrain = min(def.formation_error(1:min(20, N)));
end

% Peak formation error (during obstacle encounters)
err_peak = max(def.formation_error);

% ---------------------------------------------------------------
% Overall recovery (end of run — should be in free terrain)
% ---------------------------------------------------------------
% Use last 15% of simulation for final assessment
last_idx = max(1, round(0.85*N)) : N;
err_final = min(def.formation_error(last_idx));

if err_peak > err_terrain + 1e-4
    excess = max(err_peak - err_terrain, 1e-6);
    def.recovery_pct = max(0, min(100, ...
        100 * (1 - (err_final - err_terrain) / excess)));
else
    def.recovery_pct = 100;
end

% ---------------------------------------------------------------
% Inter-obstacle recovery (between Obs 1 and Obs 2)
% ---------------------------------------------------------------
if nO >= 2
    y_lo = cfg.obstacles(1,2) + cfg.obstacles(1,4) + 0.5;  % just past Obs 1
    y_hi = cfg.obstacles(2,2) - cfg.obstacles(2,4) - 0.5;  % just before Obs 2
else
    y_lo = 11.0; y_hi = 20.0;
end

between_idx = find(y >= y_lo & y <= y_hi);
if numel(between_idx) >= 5
    err_between = min(def.formation_error(between_idx));
    if err_peak > err_terrain + 1e-4
        excess = max(err_peak - err_terrain, 1e-6);
        def.recovery_pct_obs1 = max(0, min(100, ...
            100 * (1 - (err_between - err_terrain) / excess)));
    else
        def.recovery_pct_obs1 = 100;
    end
else
    def.recovery_pct_obs1 = def.recovery_pct;
end

end
