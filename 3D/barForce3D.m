function f = barForce3D(ell, kB, alphaB)
%BARFORCE3D  Scalar bar (compressive) force, Eq. (10).
%
%   f = BARFORCE3D(ell, kB, alphaB)
%
%   Computes the scalar force for a virtual bar member of measured length
%   ell.  Bars are ALWAYS compressive: kB < 0, so f < 0 (pushes nodes
%   apart).  Bars are NEVER relaxed.
%
%   Eq. (10):  f = kB * ell^alphaB
%
%   ell can be a vector; kB and alphaB are scalar.
%
%   See also STRINGFORCE3D, INTERNALFORCE3D.

assert(all(ell > 0), 'barForce3D: member length must be positive (got %.3e).', min(ell));

f = kB .* ell.^alphaB;   % Eq. (10)

end
