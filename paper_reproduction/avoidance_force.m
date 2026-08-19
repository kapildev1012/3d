function [uc, detected, clearance, penetrating, dmin_agent] = avoidance_force(q, S, cfg, nz)
%AVOIDANCE_FORCE  Collision-avoidance term u_ij^c, Eqs. (17)-(18).
%
%   [uc, detected, clearance, penetrating, dmin_agent] = ...
%                                        AVOIDANCE_FORCE(q, S, cfg, nz)
%
%   Eq. (17)  p_ij = (q_i - d_j)/||q_i - d_j||     (d_j = closest obstacle point)
%   Eq. (18)  u_ij^c = -k_a ( ||q_i-d_j||^-gamma - r_y^-gamma ) p_ij
%
%   SIGN: the literal Eq. (18) attracts agents into obstacles (see
%   ASSUMPTIONS.md A1).  cfg.avoid_sign = +1 gives the repulsive form that is
%   consistent with k_a > 0, with the term's purpose and with Figs. 5-6;
%   cfg.avoid_sign = -1 reproduces the printed expression verbatim.
%
%   The term is active only for obstacles inside the sensing radius, i.e. for
%   j in N_i^c ("the agent can measure obstacles within the set radius r_y",
%   Sec. III).  Gating uses the NOISY measured distance, because that is all an
%   agent has.  Other agents inside r_y are treated as obstacles with
%   p_ij = n_ij ("When obstacle j is another agent, then p_ij = n_ij").
%
%   Outputs
%     uc          d x n   avoidance input per agent
%     detected    n x 1   logical, agent sensed at least one obstacle (for the
%                         translucent markers of Figs. 5-6)
%     clearance   n x 1   TRUE minimum distance to any obstacle surface
%                         (negative = penetration)
%     penetrating n x 1   logical
%     dmin_agent  scalar  true minimum inter-agent distance

n  = S.n;
d  = cfg.d;
nO = size(cfg.obstacles, 1);

uc          = zeros(d, n);
detected    = false(n, 1);
clearance   = inf(n, 1);
penetrating = false(n, 1);
dmin_agent  = inf;

ry_term = cfg.ry^(-cfg.gamma);

% -------------------------------------------------------------- obstacles
for i = 1:n
    for o = 1:nO
        ctr = cfg.obstacles(o, 1:cfg.d).';
        Ro  = cfg.obstacles(o, cfg.d+1);
        dv  = q(:,i) - ctr;
        dn  = norm(dv);
        if dn < 1e-12
            error('avoidance_force:atCentre', ...
                  'Agent %d is exactly at the centre of obstacle %d.', i, o);
        end
        nhat = dv / dn;
        r_true = dn - Ro;                       % signed clearance to surface
        clearance(i) = min(clearance(i), r_true);

        if r_true >= 0
            vec_true = r_true * nhat;           % q_i - d_j, Eq. (17) numerator
        else
            penetrating(i) = true;              % ASSUMPTIONS.md A10
            vec_true = -r_true * nhat;          % keep the OUTWARD normal
        end

        if strcmp(nz.mode, 'relative')
            meas = vec_true + nz.obs(:, i, o);
            r    = norm(meas);
            if r < 1e-9, r = 1e-9; meas = r*nhat; end
            p = meas / r;                        % Eq. (17)
        else
            r = max(abs(r_true) + nz.obs(i, o), 1e-9);
            p = nhat;
        end

        if r <= cfg.ry                           % j in N_i^c
            detected(i) = true;
            uc(:,i) = uc(:,i) + cfg.avoid_sign * cfg.ka * (r^(-cfg.gamma) - ry_term) * p;
        end
    end
end

% -------------------------------------------- other agents as obstacles
for i = 1:n
    for j = 1:n
        if j == i, continue; end
        dv = q(:,i) - q(:,j);
        dn = norm(dv);
        if i < j, dmin_agent = min(dmin_agent, dn); end
        if dn < 1e-12
            error('avoidance_force:coincident', 'Agents %d and %d coincide.', i, j);
        end
        if ~cfg.interagent_avoidance, continue; end
        if strcmp(nz.mode, 'relative')
            meas = dv + nz.agt(:, i, j);
            r    = norm(meas);
            if r < 1e-9, r = 1e-9; meas = r*dv/dn; end
            p = meas / r;
        else
            r = max(dn + nz.agt(i, j), 1e-9);
            p = dv / dn;
        end
        if r <= cfg.ry
            uc(:,i) = uc(:,i) + cfg.avoid_sign * cfg.ka * (r^(-cfg.gamma) - ry_term) * p;
        end
    end
end
end
