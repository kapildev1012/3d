function out = simulate(cfg, S, q0, v0)
%SIMULATE  Fixed-step time integration of the closed-loop MAS, Eq. (8)-(9).
%
%   out = SIMULATE(cfg, S)            starts from q0 = S.q0, v0 = 0
%   out = SIMULATE(cfg, S, q0, v0)
%
%   Integrators (cfg.integrator, ASSUMPTIONS.md A6):
%     'symplectic_euler' (default)  one control update per step, dt = cfg.dt
%                                       v+ = v + dt*(-c*v + u)/m
%                                       q+ = q + dt*v+
%     'rk4'                         classical RK4; the measurement-noise
%                                   realisation is held fixed over the step
%                                   (a sampled-data controller), so with
%                                   cfg.noise_mode='none' this is a genuine
%                                   4th-order reference solution.
%
%   The RNG must be seeded by the caller (RUN_EXPERIMENT does it) so that runs
%   are reproducible.
%
%   Output struct (histories have one column per stored time):
%     t, q (d x n x N+1), v, u (d x n x N), a
%     ell (M x N+1) true member lengths, ell_meas_rms
%     relaxed (M x N+1) logical: string operating on the relaxation branch
%     detected (n x N+1), clearance (n x N+1), dmin_agent (1 x N+1)
%     centroid (d x N+1), sat_frac, n_penetration, diverged
%
%   See also CONTROL_INPUT, DYNAMICS, RUN_EXPERIMENT.

if nargin < 3 || isempty(q0), q0 = S.q0;              end
if nargin < 4 || isempty(v0), v0 = zeros(cfg.d, S.n); end

N  = cfg.n_steps;
d  = cfg.d;
n  = S.n;
M  = size(S.members, 1);

out.t        = (0:N) * cfg.dt;
out.q        = zeros(d, n, N+1);
out.v        = zeros(d, n, N+1);
out.u        = zeros(d, n, N);
out.a        = zeros(d, n, N);
out.ell      = zeros(M, N+1);
out.relaxed  = false(M, N+1);
out.detected = false(n, N+1);
out.clearance= zeros(n, N+1);
out.dmin_agent = zeros(1, N+1);
out.sat_any  = false(n, N);
out.n_penetration = 0;
out.diverged = false;
out.cfg      = cfg;

q = q0;
v = v0;

for step = 1:N+1
    nz          = sample_noise(S, cfg);
    [u, dg]     = control_input(q, S, cfg, nz);

    out.q(:,:,step)      = q;
    out.v(:,:,step)      = v;
    out.ell(:,step)      = dg.ell_true;
    out.relaxed(:,step)  = S.type & (dg.ell_true > cfg.z1) & cfg.relaxation;
    out.detected(:,step) = dg.detected;
    out.clearance(:,step)= dg.clearance;
    out.dmin_agent(step) = dg.dmin_agent;
    out.n_penetration    = out.n_penetration + sum(dg.penetrating);

    if step == N+1, break; end

    a = dynamics(v, u, cfg);                                  % Eq. (9)
    out.u(:,:,step)   = u;
    out.a(:,:,step)   = a;
    out.sat_any(:,step) = dg.saturated(:);

    switch lower(cfg.integrator)
        case 'symplectic_euler'
            v = v + cfg.dt * a;                               % semi-implicit
            q = q + cfg.dt * v;
        case 'rk4'
            % y = [q; v], noise held fixed over the step
            f = @(qq, vv) deal(vv, dynamics(vv, control_input(qq, S, cfg, nz), cfg));
            [k1q, k1v] = f(q, v);
            [k2q, k2v] = f(q + cfg.dt/2*k1q, v + cfg.dt/2*k1v);
            [k3q, k3v] = f(q + cfg.dt/2*k2q, v + cfg.dt/2*k2v);
            [k4q, k4v] = f(q + cfg.dt  *k3q, v + cfg.dt  *k3v);
            q = q + cfg.dt/6*(k1q + 2*k2q + 2*k3q + k4q);
            v = v + cfg.dt/6*(k1v + 2*k2v + 2*k3v + k4v);
        otherwise
            error('simulate:integrator', 'Unknown integrator "%s".', cfg.integrator);
    end

    if any(~isfinite(q(:))) || any(~isfinite(v(:)))
        out.diverged = true;
        out.diverged_step = step;
        warning('simulate:divergence', ...
            ['Non-finite state at step %d (t = %.3f). Saving partial history. ', ...
             'Likely cause: member length collapse or an unstable dt.'], step, step*cfg.dt);
        out.q = out.q(:,:,1:step);  out.v = out.v(:,:,1:step);
        out.t = out.t(1:step);      out.ell = out.ell(:,1:step);
        out.relaxed = out.relaxed(:,1:step);
        out.detected = out.detected(:,1:step);
        out.clearance = out.clearance(:,1:step);
        out.dmin_agent = out.dmin_agent(1:step);
        break
    end
end

out.centroid = squeeze(mean(out.q, 2));                   % d x (N+1)
out.sat_frac = mean(out.sat_any(:));
out.min_clearance = min(out.clearance(:));
out.max_ell  = max(out.ell(:));
end
