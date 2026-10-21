function u = tensegControl(S, mode, value)
%TENSEGCONTROL  Build the cable actuation input used by the DAE plant.
%
%   u = tensegControl(S, 'restlength', ls)   rest-length actuation l_s (ns x 1)
%   u = tensegControl(S, 'sigma', sigma)     direct force-density command (ns x 1)
%   u = tensegControl(S)                     rest lengths S.str.ls0 (passive)
%
%   In 'restlength' mode the force density follows the unilateral law (Eq. 25)
%       sigma_k = K_k (1 - l_sk/||s_k||) if ||s_k|| >= l_sk, else 0 (slack).
%   In 'sigma' mode sigma is used directly, clamped to sigma_k >= 0 (cables
%   cannot push); a cable with sigma_k = 0 counts as slack.

if nargin < 2 || isempty(mode), mode = 'restlength'; end
switch lower(mode)
    case 'restlength'
        if nargin < 3 || isempty(value), value = S.str.ls0; end
        u = struct('mode', 'restlength', 'ls', value(:), 'sigma', []);
    case 'sigma'
        u = struct('mode', 'sigma', 'ls', [], 'sigma', max(value(:), 0));
    otherwise
        error('tensegControl:mode', 'Unknown mode "%s" (use restlength|sigma).', mode);
end
end
