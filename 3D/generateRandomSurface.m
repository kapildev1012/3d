function surfData = generateRandomSurface(cfg)
%GENERATERANDOSUFSURFACE  Synthesises a 2-D random rough surface via Fourier spectral sum.
%
%   Following the COMSOL Multiphysics random surface generation methodology:
%   "How to Generate Random Surfaces in COMSOL Multiphysics"
%   https://www.comsol.com/blogs/how-to-generate-random-surfaces-in-comsol-multiphysics
%
%   The surface is constructed as a superposition of elementary cosine waves:
%
%       h(x, y) = sum_{m=-M..M, n=-N..N} A_{mn} * cos(kx_m*x + ky_n*y + phi_{mn})
%
%   where:
%     - kx_m = 2*pi*m*nu_x,  ky_n = 2*pi*n*nu_y  (discrete spatial frequencies)
%     - phi_{mn} ~ Uniform[-pi, pi]  (random phase)
%     - A_{mn} ~ (1 + (m/alpha)^2 + (n/beta)^2)^(-p/2)  (spectral envelope)
%
%   Amplitudes are then normalised to achieve the target RMS roughness cfg.groundRMS.
%
%   Inputs:
%     cfg  struct   configuration (from config3D)
%   Outputs:
%     surfData.modes   [kx, ky, A, phi] for each mode
%     surfData.sigma_h target RMS roughness

% Use a fixed reproducible seed for terrain (separate from agent noise seed)
rng(cfg.seed + 999, 'twister');

M = cfg.groundM;          % max harmonic in X
N = cfg.groundN;          % max harmonic in Y
nu_x = cfg.groundNuX;    % base spatial frequency X [1/m]
nu_y = cfg.groundNuY;    % base spatial frequency Y [1/m]
p    = cfg.groundPower;   % spectral roll-off
sigma_h = cfg.groundRMS;  % target RMS amplitude

% Pre-allocate
numModes = (2*M+1) * (2*N+1) - 1;  % exclude DC
modes = zeros(numModes, 4);  % [kx, ky, A_raw, phi]
idx = 0;

alpha_norm = M / 2.5;   % normalisation for elliptic spectral envelope
beta_norm  = N / 2.5;

for m = -M:M
    for n = -N:N
        if m == 0 && n == 0
            continue;   % skip DC component
        end
        kx  = 2 * pi * m * nu_x;
        ky  = 2 * pi * n * nu_y;
        % Spectral amplitude: power-law roll-off (COMSOL-style)
        k_sq = (m / alpha_norm)^2 + (n / beta_norm)^2;
        A_raw = 1.0 / ((1.0 + k_sq)^(p / 2));
        phi = (2 * rand() - 1) * pi;   % Uniform[-pi, pi]
        idx = idx + 1;
        modes(idx, :) = [kx, ky, A_raw, phi];
    end
end

modes = modes(1:idx, :);  % trim

% Normalise amplitudes so that RMS(h) == sigma_h
% Var[h] = sum(A_mn^2 / 2) => std = sqrt( sum(A^2)/2 )
raw_rms = sqrt(sum(modes(:, 3).^2) / 2);
if raw_rms > 1e-10
    modes(:, 3) = modes(:, 3) * (sigma_h / raw_rms);
end

surfData.modes   = modes;
surfData.nModes  = size(modes, 1);
surfData.sigma_h = sigma_h;
surfData.seed    = cfg.seed;

end
