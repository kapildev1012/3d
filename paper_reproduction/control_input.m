function [u, dg] = control_input(q, S, cfg, nz)
%CONTROL_INPUT  Total agent input, Eq. (8), with the saturation of Eq. (9).
%
%   [u, dg] = CONTROL_INPUT(q, S, cfg, nz)
%
%   Eq. (8)   u_i = u_i^t(q) + sum_{j in N_i^c} u_ij^c(q_i,d_j,r_y) + u_i^n
%   Eq. (9)   the resultant is clamped to magnitude u_max  -> SATURATE
%
%   dg (diagnostics) fields: ut, uc, un, u_raw, ell_true, ell_meas, detected,
%   clearance, penetrating, dmin_agent, saturated.

[ut, ell_true, ell_meas] = internal_force(q, S, cfg, nz);                 % Eq. (14)
[uc, detected, clearance, penetrating, dmin_agent] = avoidance_force(q, S, cfg, nz); % Eq.(18)

un    = repmat(cfg.u_nav(:), 1, S.n);                                    % u_i^n
u_raw = ut + uc + un;                                                    % Eq. (8)

[u, saturated] = saturate(u_raw, cfg);                                   % Eq. (9)

if nargout > 1
    dg = struct('ut', ut, 'uc', uc, 'un', un, 'u_raw', u_raw, ...
                'ell_true', ell_true, 'ell_meas', ell_meas, ...
                'detected', detected, 'clearance', clearance, ...
                'penetrating', penetrating, 'dmin_agent', dmin_agent, ...
                'saturated', saturated);
end
end
