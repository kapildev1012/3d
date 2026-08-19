function [ut, ell_true, ell_meas] = internal_force(q, S, cfg, nz)
%INTERNAL_FORCE  Tensegrity edge force term u_i^t, Eq. (14).
%
%   [ut, ell_true, ell_meas] = INTERNAL_FORCE(q, S, cfg, nz)
%
%   Implements Eq. (14):
%       u_i^t = -grad_{q_i} V(q) = - sum_{j in N_i} f(||q_i-q_j||) n_ij
%       n_ij  = (q_i - q_j) / ell_ij
%   with f from EDGE_FORCE (Eq. (10), or Eqs. (19)-(20) when relaxation is on).
%
%   The sum is evaluated MEMBER-wise, not neighbour-wise, so that string edges
%   shared by two tiles contribute twice (multiplicity 2, ASSUMPTIONS.md A2).
%
%   Each endpoint of a member uses its OWN noisy measurement of the relative
%   position (Sec. III + Sec. V noise statement, ASSUMPTIONS.md A5).
%
%   Outputs
%     ut        d x n  edge force per agent
%     ell_true  M x 1  true member lengths
%     ell_meas  M x 2  measured member lengths (column = endpoint)

M  = size(S.members, 1);
n  = S.n;
d  = cfg.d;
ut = zeros(d, n);

dq       = q(:, S.members(:,1)) - q(:, S.members(:,2));   % d x M, true
ell_true = sqrt(sum(dq.^2, 1)).';
ell_meas = zeros(M, 2);

if any(ell_true <= 1e-9)
    bad = find(ell_true <= 1e-9, 1);
    error('internal_force:collapse', ...
          ['Member %d (nodes %d-%d) has collapsed to length %.3e. ', ...
           'Simulation is invalid; no artificial clipping applied.'], ...
          bad, S.members(bad,1), S.members(bad,2), ell_true(bad));
end

for side = 1:2
    if side == 1
        base = dq;                       % as seen by node members(:,1)
    else
        base = -dq;                      % as seen by node members(:,2)
    end

    if strcmp(nz.mode, 'relative')
        meas = base + reshape(nz.mem(:,:,side), d, M);
        lm   = sqrt(sum(meas.^2, 1)).';
        dirv = meas ./ max(lm.', eps);
    else                                  % 'distance': exact direction
        lm   = max(ell_true + nz.mem(:,side), 1e-6);
        dirv = base ./ ell_true.';
    end
    ell_meas(:, side) = lm;

    f = edge_force(lm, S.k, S.alpha, S.type, cfg);        % Eqs. (10)/(19)
    contrib = -(f.' .* dirv);                             % Eq. (14): -f * n_ij

    nodes = S.members(:, side);
    for dim = 1:d
        ut(dim, :) = ut(dim, :) + accumarray(nodes, contrib(dim,:).', [n 1]).';
    end
end
end
