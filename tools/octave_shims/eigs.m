function varargout = eigs(A, k, sigma, varargin)
% Octave shim used only to run the ORIGINAL final_codes/original/forward_tensegrity.m:
% Octave's eigs does not know MATLAB's 'smallestabs'. For these small dense
% matrices it returns the k eigenvalues of smallest magnitude, as MATLAB does.
if ischar(sigma) && strcmp(sigma, 'smallestabs')
  e = eig(full((A + A')/2));
  [~, i] = sort(abs(e));
  varargout{1} = e(i(1:k));
else
  [varargout{1:nargout}] = builtin('eigs', A, k, sigma, varargin{:});
end
end
