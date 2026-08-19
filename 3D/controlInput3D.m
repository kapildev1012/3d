function [u, dg] = controlInput3D(q, v, L, cfg, nz)
%CONTROLINPUT3D  Total agent input in 3-D, Eq. (8), with saturation (Eq. (9)).
%
%   [u, dg] = CONTROLINPUT3D(q, v, L, cfg, nz)
%
%   Eq. (8)   u_i = u_i^t(q) + sum_{j in N_i^c} u_ij^c(q_i,d_j,r_y) + u_i^n + ug_i
%   Eq. (9)   resultant clamped to magnitude u_max
%
%   ug_i is the new ground reaction force for the uneven surface.
%
%   Inputs:
%     q    3 x n   node positions
%     v    3 x n   node velocities
%     L    struct  lander topology
%     cfg  struct  configuration
%     nz   struct  noise realisation
%
%   Outputs:
%     u    3 x n   total saturated control force
%     dg   struct  diagnostics

[ut, ell_true, f_members] = internalForce3D(q, L, cfg, nz);
[uc, detected, clearance, dmin_agent] = obstacleForce3D(q, L, cfg, nz);
[ug, h_ground] = groundForce3D(q, v, cfg);

un = repmat(cfg.u_nav(:), 1, L.n);
u_raw = ut + uc + un + ug;

[u, saturated] = saturate(u_raw, cfg); % using existing saturate.m

if nargout > 1
    dg = struct('ut', ut, 'uc', uc, 'un', un, 'ug', ug, 'u_raw', u_raw, ...
                'ell_true', ell_true, 'f_members', f_members, ...
                'detected', detected, 'clearance', clearance, ...
                'dmin_agent', dmin_agent, 'saturated', saturated, ...
                'h_ground', h_ground);
end

end
