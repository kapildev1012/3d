function f = edge_force(ell, k, alpha, isString, cfg)
%EDGE_FORCE  Internal member force f_ij, Eq. (10) with relaxation Eqs. (19)-(20).
%
%   f = EDGE_FORCE(ell, k, alpha, isString, cfg)
%
%   ell, k, alpha, isString are equal-size arrays (one entry per member).
%
%   Eq. (10)  f_ij(z) = k_ij * z^alpha_ij
%
%   Eq. (19)  for STRING members, when cfg.relaxation is true:
%                 k_ij*ell^alpha_ij      ell <= z1
%       f_ij =    h(ell)                 z1 <= ell <= z2
%                 beta                   ell >= z2
%
%   Eq. (20)  h(ell) = (2s^3-3s^2+1) k z1^alpha
%                    + (s^3-2s^2+s)(z2-z1) k alpha z1^(alpha-1)
%                    + (-2s^3+3s^2) beta ,      s = (ell-z1)/(z2-z1)
%
%   i.e. the cubic Hermite spline with p0 = f(z1), m0 = f'(z1), p1 = beta, m1 = 0.
%
%   Sign convention (Eq. (14)): f > 0 pulls the two nodes together (tension,
%   strings, k > 0);  f < 0 pushes them apart (compression, bars, k < 0).
%
%   See also EDGE_FORCE_SLOPE, INTERNAL_FORCE.

if any(ell <= 0)
    error('edge_force:degenerate', ...
          'Non-positive member length encountered (min = %.3e). Two agents coincide.', ...
          min(ell));
end

f = k .* ell.^alpha;                                    % Eq. (10)

if ~cfg.relaxation
    return
end

z1 = cfg.z1;  z2 = cfg.z2;  bta = cfg.beta;

mid = isString & (ell >  z1) & (ell <= z2);             % Eq. (19), middle branch
top = isString & (ell >  z2);                           % Eq. (19), constant branch

if any(mid)
    s  = (ell(mid) - z1) / (z2 - z1);
    kk = k(mid);
    aa = alpha(mid);
    p0 = kk .* z1.^aa;                                  % f(z1)
    m0 = kk .* aa .* z1.^(aa - 1);                      % f'(z1)
    f(mid) = (2*s.^3 - 3*s.^2 + 1) .* p0 ...
           + (s.^3 - 2*s.^2 + s) * (z2 - z1) .* m0 ...
           + (-2*s.^3 + 3*s.^2) * bta;                  % Eq. (20)
end

f(top) = bta;
end
