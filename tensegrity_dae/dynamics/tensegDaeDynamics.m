function [qdd, aux] = tensegDaeDynamics(S, q, dq, u, fext)
%TENSEGDAEDYNAMICS  Index-1 DAE forward dynamics (Module C, Eq. 34-35 / 68).
%
%   [qdd, aux] = tensegDaeDynamics(S, q, dq, u, fext)
%
%   q, dq : 3n x 1 nodal positions / velocities (q = vec(N))
%   u     : actuation struct from tensegControl ('restlength' or 'sigma')
%   fext  : 3n x 1 external nodal force (excluding gravity, which is in G);
%           [] means zero.
%
%   Rigid bars:
%     xi1    = -Yhat(q) sigma + G + f,      f = fext + fd   (string damping)
%     lambda = -(Rq M^-1 Rq')^-1 (xi2 + Rq M^-1 xi1)
%     qdd    = M^-1 (xi1 + Rq' lambda)
%   Flexible bars (S.flexible): M is replaced by Mqq(q), the bar-length rows
%   leave R, and xi3 = xi1 - grad(Vb) - bias(q,dq) replaces xi1 (Eq. 67-68).
%   If Rq M^-1 Rq' is (near) singular - redundant constraints - its
%   Moore-Penrose pseudoinverse is used.
%
%   aux contains lambda, sigma, f (non-conservative force), Wf_dot = f'dq
%   (Eq. 49), and the constraint / string structs for reuse.

if nargin < 5 || isempty(fext), fext = zeros(S.nq,1); end
st = tensegStrings(S, q, dq, u);
cs = tensegConstraints(S, q, dq);
f  = fext(:) + st.fd;
xi1 = -st.fint + S.G + f;

if S.flexible
    ft = tensegFlexTerms(S, q, dq);
    xi = xi1 - ft.gradVb - ft.bias;               % xi3 without Rq'lambda
    Mm = ft.Mqq;
    if rcond(Mm) < 1e-13
        error('tensegDaeDynamics:singularMqq', ['M_qdd is singular: with flexible ' ...
            'bars the axial stretch of a bar carries no inertia unless its end ' ...
            'nodes have point masses (topo.pmNodes/pmMass).']);
    end
    MinvXi = Mm\xi;
    MinvRqT = Mm\cs.Rq';
else
    ft = [];
    xi = xi1;
    MinvXi = S.Minv*xi;
    MinvRqT = S.Minv*cs.Rq';
end

if cs.m > 0
    Sc = cs.Rq*MinvRqT;
    rhs = cs.xi2 + cs.Rq*MinvXi;
    lambda = -robustSolve(Sc, rhs);
    qdd = MinvXi + MinvRqT*lambda;
else
    lambda = zeros(0,1);
    qdd = MinvXi;
end

if nargout > 1
    aux = struct('lambda', lambda, 'sigma', st.sigma, 'taut', st.taut, ...
        'f', f, 'fd', st.fd, 'Wf_dot', f'*dq, 'xi1', xi1, ...
        'strings', st, 'constraints', cs, 'flex', ft);
end
end

function x = robustSolve(A, b)
if rcond(A) > 1e-12
    x = A\b;
else
    x = pinv(A)*b;
end
end
