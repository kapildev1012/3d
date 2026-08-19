function [h, dhdx, dhdy] = evalSurface(x, y, cfg)
%EVALSURFACE  Evaluate the COMSOL-based random surface height and exact gradients.
%
%   [h, dhdx, dhdy] = EVALSURFACE(x, y, cfg)
%
%   Uses analytical differentiation of the Fourier cosine series so that
%   exact surface normals can be computed for collision response.
%
%   The persistent cache means the random modes are only generated ONCE
%   per simulation run (keyed by seed), giving fast repeated evaluation.
%
%   Inputs:
%     x, y   arrays of any shape  -- query positions [m]
%     cfg    struct                -- configuration (from config3D)
%
%   Outputs:
%     h      [m]   surface height at (x,y)
%     dhdx   [m/m] dh/dx (for normal calculation)
%     dhdy   [m/m] dh/dy

persistent cachedSurf cachedSeed

if isempty(cachedSurf) || isempty(cachedSeed) || cachedSeed ~= cfg.seed
    cachedSurf = generateRandomSurface(cfg);
    cachedSeed = cfg.seed;
end

modes  = cachedSurf.modes;          % [kx, ky, A, phi]
nModes = cachedSurf.nModes;

orig_size = size(x);
xf = x(:)';
yf = y(:)';
nPts = numel(xf);

hf    = zeros(1, nPts);
dhdxf = zeros(1, nPts);
dhdyf = zeros(1, nPts);

for m = 1:nModes
    kx  = modes(m, 1);
    ky  = modes(m, 2);
    A   = modes(m, 3);
    phi = modes(m, 4);

    arg     = kx * xf + ky * yf + phi;
    cos_arg = cos(arg);
    sin_arg = sin(arg);

    hf    = hf    + A * cos_arg;
    dhdxf = dhdxf - A * kx * sin_arg;   % d/dx [A cos(kx*x + ky*y + phi)]
    dhdyf = dhdyf - A * ky * sin_arg;
end

h    = reshape(hf,    orig_size);
dhdx = reshape(dhdxf, orig_size);
dhdy = reshape(dhdyf, orig_size);

end
