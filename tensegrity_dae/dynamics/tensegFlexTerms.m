function ft = tensegFlexTerms(S, q, dq)
%TENSEGFLEXTERMS  State-dependent inertia & elastic terms for flexible bars.
%
%   ft = tensegFlexTerms(S, q, dq)
%
%   Section 3 of the paper (Eq. 64-68). Bars may stretch axially; by the
%   Poisson effect r_dot = -nu r l_dot/l, which integrates exactly to
%   r_k = r_k0 (l_k/l_k0)^(-nu_k), so I_bk = m/12 (3 r_k^2 + l_k^2) is a
%   function of q. Kinetic energy of bar k (paper):
%       T_bk = 1/2 q'M_bk q - 1/2 (I_bk/l_k^2) l_dot_k^2
%            = 1/2 m |bbar_dot|^2 + 1/2 phi_k (|b_dot|^2 - l_dot_k^2),
%   with phi_k = I_bk/l_k^2, i.e. only the transverse (rotational) part of
%   b_dot carries rotary inertia. Euler-Lagrange then gives
%       Mqq(q) qdd + bias(q,dq) = generalized forces,
%       Mqq  = M0 + sum phi_k X_k'(I3 - bhat bhat')X_k            ("M_qdd")
%       bias = d/dt(dT/ddq)|_{qdd=0} - dT/dq                       ("M_qdot dq + M_q q")
%   Elastic bar energy (Eq. 66): Vb = 1/2 sum K_bk (l_k - l_k0)^2.
%
%   Outputs: Mqq, bias, T (total kinetic), p = dT/ddq (3n x 1),
%            dTdq (3n x 1), Vb, gradVb (3n x 1), r (current radii).

nq = S.nq;  nb = S.nb;
Mqq = S.M0;
bias = zeros(nq,1);
p = S.M0*dq;
T = 0.5*dq'*S.M0*dq;
dTdq = zeros(nq,1);
Vb = 0;  gradVb = zeros(nq,1);
r = zeros(nb,1);

for k = 1:nb
    X  = S.Xk{k};
    b  = X*q;   db = X*dq;
    l  = norm(b);
    bd = b'*db;                 % = l*l_dot
    ld = bd/l;
    m  = S.bar.m(k);  nu = S.bar.nu(k);
    l0 = S.bar.l(k);  r0 = S.bar.r(k);
    r2 = r0^2*(l/l0)^(-2*nu);
    r(k) = sqrt(r2);
    phi  = m/12*(1 + 3*r2/l^2);           % I_b/l^2
    dphi = -m/2*(nu + 1)*r2/l^3;          % d phi / d l

    Xtb  = X'*b;                           % = X'X q
    bperp_d = db - ld*b/l;                 % transverse part of b_dot
    w    = X'*bperp_d;                     % = d l_dot/dq * l
    uu   = db'*db - ld^2;

    T    = T + 0.5*phi*uu;
    p    = p + phi*w;
    dTk  = 0.5*dphi*uu*Xtb/l - phi*ld*w/l;
    dTdq = dTdq + dTk;

    % qdd-free part of d/dt(transverse b_dot)
    cvec = -((db'*db)*b/l^2 + bd*db/l^2 - 2*bd^2*b/l^4);
    bias = bias + dphi*ld*w + phi*(X'*cvec) - dTk;

    Mqq = Mqq + phi*(S.XtX{k} - (Xtb*Xtb')/l^2);

    % elastic bar (Eq. 66)
    Vb = Vb + 0.5*S.bar.Kb(k)*(l - l0)^2;
    gradVb = gradVb + S.bar.Kb(k)*(l - l0)*Xtb/l;
end
Mqq = full(Mqq);  Mqq = (Mqq + Mqq')/2;

ft = struct('Mqq', Mqq, 'bias', bias, 'T', T, 'p', p, 'dTdq', dTdq, ...
    'Vb', Vb, 'gradVb', gradVb, 'r', r);
end
