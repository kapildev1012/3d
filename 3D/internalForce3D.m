function [ut, ell_true, f_members] = internalForce3D(q, L, cfg, nz)
%INTERNALFORCE3D  Tensegrity member force term u_i^t, Eq. (14), in 3-D.
%
%   [ut, ell_true, f_members] = INTERNALFORCE3D(q, L, cfg, nz)
%
%   Implements Eq. (14):
%       u_i^t = - sum_{j in N_i} f(||q_i - q_j||) * n_ij
%       n_ij  = (q_i - q_j) / ||q_i - q_j||
%
%   with f from STRINGFORCE3D (with adaptive relaxation) or BARFORCE3D.
%
%   Each member contributes equal-and-opposite forces to its two endpoint
%   nodes.  Measurement noise is applied to the relative position vector
%   used for force computation.
%
%   Inputs:
%     q    3 x n   current node positions
%     L    struct  lander topology (from buildLander3D)
%     cfg  struct  configuration
%     nz   struct  noise (from sampleNoise3D)
%
%   Outputs:
%     ut        3 x n   total member force per node
%     ell_true  M x 1   true member lengths
%     f_members M x 1   scalar force per member
%
%   See also STRINGFORCE3D, BARFORCE3D, CONTROLINPUT3D.

M  = size(L.members, 1);   % 30
n  = L.n;                   % 12
d  = 3;

ut = zeros(d, n);

% True relative positions and lengths
dq       = q(:, L.members(:,1)) - q(:, L.members(:,2));   % 3 x M
ell_true = sqrt(sum(dq.^2, 1)).';                         % M x 1

% Check for collapsed members
if any(ell_true <= 1e-9)
    bad = find(ell_true <= 1e-9, 1);
    error('internalForce3D:collapse', ...
          'Member %d (nodes %d-%d) collapsed to length %.3e.', ...
          bad, L.members(bad,1), L.members(bad,2), ell_true(bad));
end

% Compute scalar force per member
f_members = zeros(M, 1);
for mm = 1:M
    if L.isString(mm)
        % Apply noisy measurement for force computation
        if strcmp(nz.mode, 'relative')
            meas = dq(:,mm) + nz.mem(:,mm);
            ell_meas = max(norm(meas), 1e-9);
        else
            ell_meas = max(ell_true(mm) + nz.mem_scalar(mm), 1e-6);
        end
        f_members(mm) = stringForce3D(ell_meas, cfg.kS, cfg.alphaS, ...
                                       cfg.z1, cfg.z2, cfg.beta, cfg.relaxation);
    else
        if strcmp(nz.mode, 'relative')
            meas = dq(:,mm) + nz.mem(:,mm);
            ell_meas = max(norm(meas), 1e-9);
        else
            ell_meas = max(ell_true(mm) + nz.mem_scalar(mm), 1e-6);
        end
        f_members(mm) = barForce3D(ell_meas, cfg.kB, cfg.alphaB);
    end
end

% Apply forces to nodes: Eq. (14)
% For each member (i,j): force on i is -f * n_ij, force on j is +f * n_ij
% where n_ij = (q_i - q_j) / ||q_i - q_j||
for mm = 1:M
    i = L.members(mm, 1);
    j = L.members(mm, 2);

    % Use noisy direction for each endpoint
    if strcmp(nz.mode, 'relative')
        % Node i's view
        meas_i = dq(:,mm) + nz.mem(:,mm);
        ell_i  = max(norm(meas_i), 1e-9);
        nij_i  = meas_i / ell_i;

        % Node j's view (opposite direction + independent noise)
        meas_j = -dq(:,mm) + nz.mem_j(:,mm);
        ell_j  = max(norm(meas_j), 1e-9);
        nji_j  = meas_j / ell_j;
    else
        nij = dq(:,mm) / ell_true(mm);
        nij_i = nij;
        nji_j = -nij;
    end

    % Force on node i: -f * n_ij (Eq. 14)
    ut(:,i) = ut(:,i) - f_members(mm) * nij_i;
    % Force on node j: -f * n_ji = +f * n_ij
    ut(:,j) = ut(:,j) - f_members(mm) * nji_j;
end

end
