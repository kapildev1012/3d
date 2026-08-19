function f = stringForce3D(ell, kS, alphaS, z1, z2, beta, relaxOn)
%STRINGFORCE3D  Scalar string force with adaptive relaxation, Eqs. (10),(19)-(20).
%
%   f = STRINGFORCE3D(ell, kS, alphaS, z1, z2, beta, relaxOn)
%
%   Computes the scalar force magnitude for a virtual string member of
%   measured length ell.  Sign convention: f > 0 means tension (pulls
%   the two nodes together).
%
%   Without relaxation (relaxOn = false):
%     f = kS * ell^alphaS                                    Eq. (10)
%
%   With relaxation (relaxOn = true), piecewise:
%     ell <= z1  :  f = kS * ell^alphaS                      Eq. (10)
%     z1 < ell <= z2 :  f = h(ell)  (cubic Hermite spline)   Eq. (20)
%     ell > z2  :  f = beta                                  Eq. (19)
%
%   Eq. (20): h(ell) = (2s^3-3s^2+1) * kS*z1^alphaS
%                     + (s^3-2s^2+s) * (z2-z1) * kS*alphaS*z1^(alphaS-1)
%                     + (-2s^3+3s^2) * beta
%           where s = (ell - z1) / (z2 - z1)
%
%   ell can be a vector; all other arguments are scalar.
%
%   See also BARFORCE3D, INTERNALFORCE3D.

% Validate
assert(all(ell > 0), 'stringForce3D: member length must be positive (got %.3e).', min(ell));

% Standard string law, Eq. (10)
f = kS .* ell.^alphaS;

if ~relaxOn
    return
end

% Adaptive relaxation, Eq. (19)
mid = (ell > z1) & (ell <= z2);
top = (ell > z2);

if any(mid)
    s  = (ell(mid) - z1) / (z2 - z1);
    p0 = kS * z1^alphaS;                          % f(z1)
    m0 = kS * alphaS * z1^(alphaS - 1);           % f'(z1)
    f(mid) = (2*s.^3 - 3*s.^2 + 1) * p0 ...
           + (s.^3 - 2*s.^2 + s) * (z2 - z1) * m0 ...
           + (-2*s.^3 + 3*s.^2) * beta;           % Eq. (20)
end

f(top) = beta;

end
