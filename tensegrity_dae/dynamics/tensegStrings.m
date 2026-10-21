function st = tensegStrings(S, q, dq, u)
%TENSEGSTRINGS  Unidirectional string mechanics and damping (Module B).
%
%   st = tensegStrings(S, q, dq, u)
%
%   Returns a struct with
%     s, ds    3 x ns   string vectors s_k = Y_k q and rates Y_k dq
%     len      ns x 1   ||s_k||
%     sigma    ns x 1   force densities (Eq. 25, unilateral)
%     taut     ns x 1   logical, sigma_k > 0 / ||s_k|| >= l_sk
%     Yhat     3n x ns  [Y_1'Y_1 q, ..., Y_ns'Y_ns q]
%     fint     3n x 1   Yhat*sigma = (sigma' (x) I_3n) Y' q   (Eq. 30)
%     fd       3n x 1   axial string damping force (Eq. 27-29)
%     Vs       scalar   string potential energy
%
%   Potential energy convention:
%     'restlength' mode: Vs = 1/2 sum K_k (||s_k|| - l_sk)^2 over taut strings
%                        (= 1/2 sum sigma_k^2/K_k s_k's_k, Eq. 46)
%     'sigma' mode     : sigma is held constant over a control interval, the
%                        force -sigma_k Y_k'Y_k q is then derived from
%                        Vs = 1/2 sum sigma_k s_k's_k.
%   In both modes dVs/dq = (Yhat*sigma)', as required by Eq. 56.

ns = S.ns;
if ns == 0
    st = struct('s', zeros(3,0), 'ds', zeros(3,0), 'len', zeros(0,1), ...
        'sigma', zeros(0,1), 'taut', false(0,1), 'tautDamp', false(0,1), ...
        'Yhat', zeros(S.nq,0), 'fint', zeros(S.nq,1), 'fd', zeros(S.nq,1), 'Vs', 0);
    return;
end
s   = reshape(S.Ys*q, 3, ns);
ds  = reshape(S.Ys*dq, 3, ns);
len = sqrt(sum(s.^2, 1))';
lenSafe = max(len, eps);

switch u.mode
    case 'restlength'
        ls = u.ls(:);
        taut  = (len - ls) >= 0;
        sigma = S.str.K.*(1 - ls./lenSafe).*taut;
        tautDamp = (len - ls) > 0;
        Vs = 0.5*sum(S.str.K.*((len - ls).*taut).^2);
    case 'sigma'
        sigma = max(u.sigma(:), 0);
        taut  = sigma > 0;
        tautDamp = taut;
        Vs = 0.5*sum(sigma.*len.^2);
    otherwise
        error('tensegStrings:mode', 'Unknown actuation mode "%s".', u.mode);
end

Sblk = colBlock(s);                    % 3ns x ns
Yhat = full(S.Ys'*Sblk);               % 3n x ns
fint = Yhat*sigma;

% axial damper, active only on taut strings (Eq. 27-29)
fd = zeros(S.nq,1);
c = S.str.c(:).*tautDamp;
if any(c)
    rate = sum(ds.*s, 1)'./lenSafe.^2;           % (ds's)/(s's)
    fdk  = -s.*repmat((c.*rate)', 3, 1);          % 3 x ns
    fd   = S.Ys'*fdk(:);
end

st = struct('s', s, 'ds', ds, 'len', len, 'sigma', sigma, 'taut', taut, ...
    'tautDamp', tautDamp, 'Yhat', Yhat, 'fint', fint, 'fd', fd, 'Vs', Vs);
end

function B = colBlock(V)
% 3 x k matrix -> sparse 3k x k block-column matrix with V(:,j) in block j
k = size(V,2);
B = sparse((1:3*k)', kron((1:k)', ones(3,1)), V(:), 3*k, k);
end
