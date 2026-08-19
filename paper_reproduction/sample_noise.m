function nz = sample_noise(S, cfg)
%SAMPLE_NOISE  Draw one set of measurement-noise realisations.
%
%   nz = SAMPLE_NOISE(S, cfg) returns the noise used by INTERNAL_FORCE and
%   AVOIDANCE_FORCE for a single control update.
%
%   Paper: "Gaussian noise with a standard deviation of sigma = 0.5 was imposed
%   on all measurements" (Sec. V).  Agents measure RELATIVE POSITIONS
%   (Sec. III), so that is what is corrupted -- see ASSUMPTIONS.md item A5:
%
%     * cfg.noise_mode = 'relative' (default): every measured relative-position
%       vector gets an independent N(0, sigma^2 I_d) perturbation.  The two
%       endpoints of a member draw INDEPENDENT noise (each agent has its own
%       sensor), so action/reaction holds only in expectation.
%     * cfg.noise_mode = 'distance': only the measured scalar distance is
%       corrupted, directions are exact.
%     * cfg.noise_mode = 'none': noiseless (used for the deterministic tests).
%
%   Fields:
%     nz.mode  'relative' | 'distance'
%     nz.mem   d x M x 2 (or M x 2)  member measurement noise, per endpoint
%     nz.obs   d x n x nObs (or n x nObs)
%     nz.agt   d x n x n   (or n x n)   agent-as-obstacle measurement noise

d  = cfg.d;
n  = S.n;
M  = size(S.members, 1);
nO = size(cfg.obstacles, 1);

switch lower(cfg.noise_mode)
    case 'none'
        sig      = 0;
        nz.mode  = 'relative';
    case 'relative'
        sig      = cfg.sigma;
        nz.mode  = 'relative';
    case 'distance'
        sig      = cfg.sigma;
        nz.mode  = 'distance';
    otherwise
        error('sample_noise:mode', 'Unknown noise_mode "%s".', cfg.noise_mode);
end

if strcmp(nz.mode, 'relative')
    nz.mem = sig * randn(d, M, 2);
    nz.obs = sig * randn(d, n, max(nO,1));
    nz.agt = sig * randn(d, n, n);
else
    nz.mem = sig * randn(M, 2);
    nz.obs = sig * randn(n, max(nO,1));
    nz.agt = sig * randn(n, n);
end
end
