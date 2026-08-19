function m = formation_metrics(out, S, cfg)
%FORMATION_METRICS  Scalar diagnostics of a simulation run.
%
%   m = FORMATION_METRICS(out, S, cfg)
%
%   Fields
%     t                 1 x N time vector
%     centroid          d x N centroid of the 12 agents (what Figs. 5-6 show)
%     progress_y        1 x N centroid Y  (the paper's direction of travel)
%     speed_y           1 x N d/dt of progress_y
%     shape_err         1 x N RMS relative member-length error w.r.t. nominal
%     rigid_err         1 x N RMS node error after the best-fit rigid transform
%                             onto the nominal formation (translation+rotation)
%     ell_string_max    1 x N longest string
%     n_relaxed         1 x N number of strings on the relaxation branch
%     min_clearance     1 x N minimum distance of any agent to any obstacle
%                             surface (negative = penetration)
%     min_agent_dist    1 x N minimum inter-agent distance
%     n_detecting       1 x N number of agents sensing an obstacle
%     cleared_time      first time the whole formation is past the large
%                             obstacle (all agents above its centre) or NaN
%     centroid_at       centroid Y at cfg.snapshots
%     final_progress    centroid Y at the end of the run

d = cfg.d;
N = numel(out.t);
q = out.q;

m.t        = out.t;
m.centroid = out.centroid;
m.progress_y = out.centroid(2,:);
m.speed_y  = gradient(m.progress_y, cfg.dt);

nominal            = zeros(size(S.members,1),1);
nominal(S.type)    = cfg.lS;
nominal(~S.type)   = sqrt(2)*cfg.lS;

m.shape_err      = sqrt(mean(((out.ell - nominal)./nominal).^2, 1));
m.ell_string_max = max(out.ell(S.type,:), [], 1);
m.ell_string_min = min(out.ell(S.type,:), [], 1);
m.n_relaxed      = sum(out.relaxed, 1);
m.min_clearance  = min(out.clearance, [], 1);
m.min_agent_dist = out.dmin_agent;
m.n_detecting    = sum(out.detected, 1);

% ---- best-fit rigid transform error (Procrustes without scaling) ----------
m.rigid_err = zeros(1, N);
P = S.q0 - mean(S.q0, 2);
for s = 1:N
    Q  = q(:,:,s) - mean(q(:,:,s), 2);
    H  = P * Q.';
    [U_,~,V_] = svd(H);
    R  = V_ * diag([ones(1,d-1), det(V_*U_.')]) * U_.';
    m.rigid_err(s) = sqrt(mean(sum((Q - R*P).^2, 1)));
end

% ---- has the formation cleared the large obstacle? ------------------------
[~, iBig]  = max(cfg.obstacles(:,3));
yBig       = cfg.obstacles(iBig,2);
allAbove   = squeeze(all(q(2,:,:) > yBig, 2)).';
idx        = find(allAbove, 1);
if isempty(idx)
    m.cleared_time = NaN;
else
    m.cleared_time = out.t(idx);
end
m.centroid_cross_time = NaN;
idx2 = find(m.progress_y > yBig, 1);
if ~isempty(idx2), m.centroid_cross_time = out.t(idx2); end

% ---- values at the paper's snapshot times --------------------------------
m.snapshot_t = cfg.snapshots;
m.centroid_at = nan(1, numel(cfg.snapshots));
for a = 1:numel(cfg.snapshots)
    [~, k] = min(abs(out.t - cfg.snapshots(a)));
    if abs(out.t(k) - cfg.snapshots(a)) < cfg.dt/2
        m.centroid_at(a) = m.progress_y(k);
    end
end
m.final_progress = m.progress_y(end);
m.final_time     = out.t(end);
end
