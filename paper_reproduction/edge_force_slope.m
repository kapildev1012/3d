function df = edge_force_slope(ell, k, alpha, isString, cfg)
%EDGE_FORCE_SLOPE  Analytic d f_ij / d ell of EDGE_FORCE.
%
%   Used for the C^1 continuity checks of Eqs. (19)-(20) (validate_results.m)
%   and for stiffness diagnostics.  Same argument convention as EDGE_FORCE.

df = k .* alpha .* ell.^(alpha - 1);                    % d/dl of Eq. (10)

if ~cfg.relaxation
    return
end

z1 = cfg.z1;  z2 = cfg.z2;  bta = cfg.beta;

mid = isString & (ell >  z1) & (ell <= z2);
top = isString & (ell >  z2);

if any(mid)
    s  = (ell(mid) - z1) / (z2 - z1);
    kk = k(mid);
    aa = alpha(mid);
    p0 = kk .* z1.^aa;
    m0 = kk .* aa .* z1.^(aa - 1);
    dhds = (6*s.^2 - 6*s) .* p0 ...
         + (3*s.^2 - 4*s + 1) * (z2 - z1) .* m0 ...
         + (-6*s.^2 + 6*s) * bta;                       % d h / d s
    df(mid) = dhds / (z2 - z1);                         % chain rule
end

df(top) = 0;
end
