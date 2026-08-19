function a = dynamics3D(v, u, cfg, nz)
%DYNAMICS3D  Agent acceleration in 3-D, Eq. (9) + environment.
%
%   a = DYNAMICS3D(v, u, cfg, nz)
%
%   Point mass dynamics:
%     m*a = u - c*v + F_grav + F_dist + F_process
%
%   Inputs:
%     v    3 x n   velocities
%     u    3 x n   total control input (already saturated)
%     cfg  struct  configuration
%     nz   struct  noise (for process noise)
%
%   Outputs:
%     a    3 x n   accelerations

if strcmp(cfg.dynamicsModel, 'rigidBody')
    error('dynamics3D:rigidBody', ...
          'Rigid-body dynamics are not yet implemented. Requires mass properties, orientation, etc.');
end

n = size(v, 2);
a = zeros(3, n);

F_grav = repmat(cfg.gravity(:), 1, n);
F_dist = repmat(cfg.disturbanceForce(:), 1, n);

% Note: nz.process contains process noise (acceleration perturbation)
F_proc = nz.process * cfg.m;

% a = (u - c*v + F_grav + F_dist + F_proc) / m
a = (u - cfg.c * v + F_grav + F_dist + F_proc) / cfg.m;

end
