function c = tensegConstraints(S, q, dq)
%TENSEGCONSTRAINTS  Holonomic constraints R(q), Jacobian R_q, xi_2 and Q.
%
%   c = tensegConstraints(S, q, dq)
%
%   Rigid bars (Eq. 36-37):
%     R   = [A q - b ; q'X_k'X_k q - l_bk^2]        (m x 1)
%     Rq  = [A ; 2 q'X_k'X_k]                         (m x 3n)
%     xi2 = [0 ; 2 dq'X_k'X_k dq]                     (-Rq*qdd = xi2)
%     Q   = [0 ; 2 dq'X_k'X_k] = d(Rq dq)/dq          (Eq. 44, used by Algorithm 1)
%   Flexible bars (Section 3): the bar-length rows are removed.
%
%   Also returns barLenErr = ||b_k|| - l_bk (nb x 1) for logging.

nbc = S.nbc;  nq = S.nq;  nb = S.nb;
Rlin = S.A*q - S.b;
B  = reshape(S.Xs*q, 3, nb);
barLen = sqrt(sum(B.^2,1))';
c.barLenErr = barLen - S.bar.l;
c.barLen = barLen;

if S.flexible || nb == 0
    c.R   = Rlin;
    c.Rq  = S.A;
    c.xi2 = zeros(nbc,1);
    c.Q   = sparse(nbc, nq);
else
    Bd = reshape(S.Xs*dq, 3, nb);
    c.R   = [Rlin; sum(B.^2,1)' - S.bar.l.^2];
    c.Rq  = [S.A; 2*(colBlock(B)'*S.Xs)];
    c.xi2 = [zeros(nbc,1); 2*sum(Bd.^2,1)'];
    c.Q   = [sparse(nbc,nq); 2*(colBlock(Bd)'*S.Xs)];
end
c.Rq = full(c.Rq);
c.Q  = full(c.Q);
c.m  = numel(c.R);
end

function Bk = colBlock(V)
k = size(V,2);
Bk = sparse((1:3*k)', kron((1:k)', ones(3,1)), V(:), 3*k, k);
end
