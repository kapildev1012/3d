function nz = sampleNoise3D(L, cfg)
%SAMPLENOISE3D  Draw one set of 3-D measurement and process noise.
%
%   nz = SAMPLENOISE3D(L, cfg)
%
%   Generates measurement noise for:
%     - member relative positions (used by internalForce3D)
%     - obstacle distances (used by obstacleForce3D)
%     - inter-node distances (used by obstacleForce3D)
%   and process noise for the dynamics.
%
%   cfg.noise_mode:
%     'relative' : every measured relative-position vector gets an
%                  independent N(0, sigma^2 * I_3) perturbation
%     'distance' : only scalar distances are corrupted
%     'none'     : noiseless
%
%   Fields returned:
%     nz.mode         'relative' | 'distance'
%     nz.mem          3 x M      member noise (node i's view)
%     nz.mem_j        3 x M      member noise (node j's view, independent)
%     nz.mem_scalar   M x 1      scalar member noise (distance mode)
%     nz.obs          3 x n x nObs  obstacle noise
%     nz.obs_scalar   n x nObs   scalar obstacle noise
%     nz.agt          3 x n x n  inter-node noise
%     nz.agt_scalar   n x n      scalar inter-node noise
%     nz.process      3 x n      process noise (acceleration perturbation)
%
%   See also INTERNALFORCE3D, OBSTACLEFORCE3D.

d  = 3;
n  = L.n;
M  = size(L.members, 1);
nO = max(size(cfg.obstacles, 1), 1);

switch lower(cfg.noise_mode)
    case 'none'
        sig = 0;
        nz.mode = 'relative';
    case 'relative'
        sig = cfg.sigma;
        nz.mode = 'relative';
    case 'distance'
        sig = cfg.sigma;
        nz.mode = 'distance';
    otherwise
        error('sampleNoise3D:mode', 'Unknown noise_mode "%s".', cfg.noise_mode);
end

% Member noise
nz.mem       = sig * randn(d, M);       % node i's independent view
nz.mem_j     = sig * randn(d, M);       % node j's independent view
nz.mem_scalar = sig * randn(M, 1);      % for distance mode

% Obstacle noise
nz.obs        = sig * randn(d, n, nO);
nz.obs_scalar = sig * randn(n, nO);

% Inter-node noise
nz.agt        = sig * randn(d, n, n);
nz.agt_scalar = sig * randn(n, n);

% Process noise (acceleration perturbation)
nz.process = cfg.processNoiseSigma * randn(d, n);

end
