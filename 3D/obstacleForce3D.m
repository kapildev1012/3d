function [uc, detected, clearance, dmin_agent] = obstacleForce3D(q, L, cfg, nz)
%OBSTACLEFORCE3D  3-D spherical obstacle avoidance, Eqs. (17)-(18).
%
%   [uc, detected, clearance, dmin_agent] = OBSTACLEFORCE3D(q, L, cfg, nz)
%
%   For each node i and each spherical obstacle o with center c_o and
%   radius R_o:
%     d_j = c_o + R_o * (q_i - c_o) / ||q_i - c_o||   (closest surface point)
%     dist = ||q_i - d_j|| = ||q_i - c_o|| - R_o
%     p_ij = (q_i - d_j) / ||q_i - d_j||               Eq. (17)
%
%   If dist <= ry:
%     u_c_ij = +ka * (dist^(-gamma) - ry^(-gamma)) * p_ij   Eq. (18), repulsive
%
%   Also treats other nodes as point obstacles (Eq. 17: "When obstacle j
%   is another agent, then p_ij = n_ij").
%
%   Inputs:
%     q    3 x n   current node positions
%     L    struct  lander topology
%     cfg  struct  configuration (obstacles stored as [x y z R] rows)
%     nz   struct  noise
%
%   Outputs:
%     uc          3 x n   avoidance force per node
%     detected    n x 1   logical, node sensed at least one obstacle
%     clearance   n x 1   minimum distance to any obstacle surface
%     dmin_agent  scalar  minimum inter-node distance
%
%   See also INTERNALFORCE3D, CONTROLINPUT3D.

n  = L.n;
d  = 3;
nO = size(cfg.obstacles, 1);

uc          = zeros(d, n);
detected    = false(n, 1);
clearance   = inf(n, 1);
dmin_agent  = inf;

ry_term = cfg.ry^(-cfg.gamma);

% ========================================================
% Obstacle avoidance (spherical obstacles)
% ========================================================
for i = 1:n
    for o = 1:nO
        ctr = cfg.obstacles(o, 1:3).';
        Ro  = cfg.obstacles(o, 4);

        dv = q(:,i) - ctr;
        dn = norm(dv);

        if dn < 1e-12
            % Node is at obstacle centre; push outward in +Z
            dirUnit = [0; 0; 1];
            dn = 1e-12;
        else
            dirUnit = dv / dn;
        end

        % Signed clearance to surface (negative = penetration)
        r_true = dn - Ro;
        clearance(i) = min(clearance(i), r_true);

        % Vector from closest surface point to node
        if r_true >= 0
            vec_true = r_true * dirUnit;
        else
            % Penetrating: keep outward normal, use abs distance
            vec_true = abs(r_true) * dirUnit;
        end

        % Apply measurement noise
        if strcmp(nz.mode, 'relative')
            meas = vec_true + nz.obs(:, i, min(o, size(nz.obs,3)));
            r    = norm(meas);
            if r < 1e-9
                r = 1e-9;
                meas = r * dirUnit;
            end
            p = meas / r;
        else
            r = max(abs(r_true) + nz.obs_scalar(i, min(o, size(nz.obs_scalar,2))), 1e-9);
            p = dirUnit;
        end

        % Avoidance force (active only within sensing radius)
        if r <= cfg.ry
            detected(i) = true;
            uc(:,i) = uc(:,i) + cfg.avoid_sign * cfg.ka * ...
                      (r^(-cfg.gamma) - ry_term) * p;
        end
    end
end

% ========================================================
% Inter-node avoidance (nodes treat each other as point obstacles)
% ========================================================
if cfg.interagent_avoidance
    for i = 1:n
        for j = 1:n
            if j == i, continue; end

            dv = q(:,i) - q(:,j);
            dn = norm(dv);
            if i < j
                dmin_agent = min(dmin_agent, dn);
            end

            if dn < 1e-12
                error('obstacleForce3D:coincident', ...
                      'Nodes %d and %d coincide.', i, j);
            end

            if strcmp(nz.mode, 'relative')
                meas = dv + nz.agt(:, i, j);
                r    = norm(meas);
                if r < 1e-9
                    r = 1e-9;
                    meas = r * dv / dn;
                end
                p = meas / r;
            else
                r = max(dn + nz.agt_scalar(i, j), 1e-9);
                p = dv / dn;
            end

            if r <= cfg.ry
                uc(:,i) = uc(:,i) + cfg.avoid_sign * cfg.ka * ...
                          (r^(-cfg.gamma) - ry_term) * p;
            end
        end
    end
end

end
