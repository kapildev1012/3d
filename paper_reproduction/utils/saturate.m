function [us, flag] = saturate(u, cfg)
%SATURATE  Input saturation sat_{u_max}(.) of Eq. (9).
%
%   [us, flag] = SATURATE(u, cfg) with u of size d x n.
%
%   cfg.sat_mode = 'norm'      (default, ASSUMPTIONS.md A3)
%       "the magnitude of the resultant summation is clamped to +/- u_max"
%       -> us_i = u_i * min(1, u_max/||u_i||)   (direction preserved)
%   cfg.sat_mode = 'component'
%       -> us_i = max(-u_max, min(u_max, u_i))  element-wise
%
%   flag is a 1 x n logical marking the agents whose input was clamped.

switch lower(cfg.sat_mode)
    case 'norm'
        nrm  = sqrt(sum(u.^2, 1));
        flag = nrm > cfg.u_max;
        sc   = ones(1, size(u,2));
        sc(flag) = cfg.u_max ./ nrm(flag);
        us   = u .* sc;
    case 'component'
        us   = max(-cfg.u_max, min(cfg.u_max, u));
        flag = any(abs(u) > cfg.u_max, 1);
    otherwise
        error('saturate:mode', 'Unknown sat_mode "%s".', cfg.sat_mode);
end
end
