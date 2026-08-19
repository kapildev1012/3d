function out = simulate3D(cfg, L, q0, v0)
%SIMULATE3D  Time integration of the 3-D lander, Eq. (8)-(9).
%
%   out = SIMULATE3D(cfg, L)
%   out = SIMULATE3D(cfg, L, q0, v0)
%
%   Runs a symplectic Euler integration loop from t=0 to cfg.T_end.
%   Records full histories of state and diagnostics.
%
%   See also CONTROLINPUT3D, DYNAMICS3D.

if nargin < 3 || isempty(q0), q0 = L.q0; end
if nargin < 4 || isempty(v0), v0 = zeros(3, L.n); end

N = cfg.n_steps;
d = 3;
n = L.n;
M = size(L.members, 1);

out.t = (0:N) * cfg.dt;
out.q = zeros(d, n, N+1);
out.v = zeros(d, n, N+1);
out.u = zeros(d, n, N);
out.ell = zeros(M, N+1);
out.relaxed = false(M, N+1);
out.detected = false(n, N+1);
out.clearance = zeros(n, N+1);
out.dmin_agent = zeros(1, N+1);
out.f_members = zeros(M, N+1);
out.sat_any = false(n, N);
out.diverged = false;
out.h_ground = zeros(n, N+1);

q = q0;
v = v0;

for step = 1:N+1
    % 1. Sample Noise
    nz = sampleNoise3D(L, cfg);
    
    % 2. Control Input
    [u, dg] = controlInput3D(q, v, L, cfg, nz);
    
    % Record state
    out.q(:,:,step) = q;
    out.v(:,:,step) = v;
    out.ell(:,step) = dg.ell_true;
    out.f_members(:,step) = dg.f_members;
    out.detected(:,step) = dg.detected;
    out.clearance(:,step) = dg.clearance;
    out.dmin_agent(step) = dg.dmin_agent;
    out.h_ground(:,step) = dg.h_ground;
    
    % Relaxation flag for strings
    out.relaxed(:,step) = L.isString & (dg.ell_true > cfg.z1) & cfg.relaxation;
    
    if step == N+1, break; end
    
    % 3. Dynamics
    a = dynamics3D(v, u, cfg, nz);
    
    % Record control and saturation
    out.u(:,:,step) = u;
    out.sat_any(:,step) = dg.saturated(:);
    
    % 4. Step
    switch lower(cfg.integrator)
        case 'symplectic_euler'
            v = v + cfg.dt * a;
            q = q + cfg.dt * v;
        case 'rk4'
            % For simplicity, RK4 uses the same control input/noise across the step
            f = @(qq, vv) deal(vv, dynamics3D(vv, controlInput3D(qq, vv, L, cfg, nz), cfg, nz));
            [k1q, k1v] = f(q, v);
            [k2q, k2v] = f(q + cfg.dt/2*k1q, v + cfg.dt/2*k1v);
            [k3q, k3v] = f(q + cfg.dt/2*k2q, v + cfg.dt/2*k2v);
            [k4q, k4v] = f(q + cfg.dt  *k3q, v + cfg.dt  *k3v);
            q = q + cfg.dt/6*(k1q + 2*k2q + 2*k3q + k4q);
            v = v + cfg.dt/6*(k1v + 2*k2v + 2*k3v + k4v);
        otherwise
            error('simulate3D:integrator', 'Unknown integrator %s', cfg.integrator);
    end
    
    % Divergence check
    if any(~isfinite(q(:))) || any(~isfinite(v(:))) || max(abs(q(:))) > 1e4
        out.diverged = true;
        warning('simulate3D:diverged', 'Simulation diverged at step %d', step);
        % Truncate
        out.q = out.q(:,:,1:step);
        out.v = out.v(:,:,1:step);
        out.t = out.t(1:step);
        break;
    end
end

out.centroid = squeeze(mean(out.q, 2));

end
