function [q, dq, Wf, info] = daeCorrector(S, q, dq, Wf, u, E0, opts)
%DAECORRECTOR  Post-step geometric & energy violation corrector (Algorithm 1).
%
%   [q, dq, Wf, info] = daeCorrector(S, q, dq, Wf, u, E0, opts)
%
%   Given the uncorrected state (q^u, dq^u, Wf^u) after an integration step,
%   repeatedly solves the minimum-norm linear system (Eq. 56-57)
%
%       [ Rq          0        0 ] [dq_ ]   [ -R(q)              ]
%       [ Q           Rq       0 ] [ddq_] = [ -Rq dq             ]
%       [ dE/dq     dE/ddq    -1 ] [dWf ]   [ E0 - E(q,dq) + Wf  ]
%
%   with dE/dq = -G' + (Yhat sigma)', dE/ddq = dq'M, and applies
%   [dq_;ddq_;dWf] = Ac'(Ac Ac')^-1 bc until ||R||_2 <= gamma and
%   |E - E0 - Wf| <= gamma (or maxIter is reached).
%
%   opts fields (all optional):
%     gamma               1e-10
%     maxIter             10
%     constraints         true   include the geometric rows
%     energy              true   include the energy row
%     checkVelocity       true   also iterate while ||Rq dq||_2 > gamma
%                                (Algorithm 1 tests R and E only; the
%                                velocity rows are solved either way)
%
%   info: iters, Rnorm, Vnorm (||Rq dq||), Eres (E - E0 - Wf).

if nargin < 7, opts = struct(); end
gamma   = getOpt(opts, 'gamma', 1e-10);
maxIter = getOpt(opts, 'maxIter', 10);
useC    = getOpt(opts, 'constraints', true);
useE    = getOpt(opts, 'energy', true);
chkV    = getOpt(opts, 'checkVelocity', true);
nq = S.nq;

iters = 0;
while true
    cs = tensegConstraints(S, q, dq);
    Rn = norm(cs.R);
    Vn = norm(cs.Rq*dq);
    if useE
        [E, dEdq, dEddq] = tensegEnergy(S, q, dq, u);
        Eres = E - E0 - Wf;
    else
        Eres = 0;
    end
    violated = (useC && (Rn > gamma || (chkV && Vn > gamma))) || (useE && abs(Eres) > gamma);
    if ~violated || iters >= maxIter, break; end

    m = cs.m;
    if useC && useE
        Ac = [cs.Rq, zeros(m,nq), zeros(m,1);
              cs.Q,  cs.Rq,       zeros(m,1);
              dEdq,  dEddq,       -1];
        bc = [-cs.R; -cs.Rq*dq; -Eres];
    elseif useC
        Ac = [cs.Rq, zeros(m,nq), zeros(m,1);
              cs.Q,  cs.Rq,       zeros(m,1)];
        bc = [-cs.R; -cs.Rq*dq];
    else
        Ac = [dEdq, dEddq, -1];
        bc = -Eres;
    end
    AAt = Ac*Ac';
    if rcond(AAt) > 1e-14
        d = Ac'*(AAt\bc);
    else
        d = pinv(Ac)*bc;
    end
    q  = q  + d(1:nq);
    dq = dq + d(nq+1:2*nq);
    Wf = Wf + d(end);
    iters = iters + 1;
end
info = struct('iters', iters, 'Rnorm', Rn, 'Vnorm', Vn, 'Eres', Eres);
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
