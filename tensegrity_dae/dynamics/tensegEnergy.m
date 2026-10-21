function [E, dEdq, dEddq, parts] = tensegEnergy(S, q, dq, u)
%TENSEGENERGY  Total mechanical energy and its gradients (Eq. 46, 52-55).
%
%   [E, dEdq, dEddq, parts] = tensegEnergy(S, q, dq, u)
%
%   E = T(dq) + Vg(q) + Vs(q) (+ Vb(q) for flexible bars)
%     T  = 1/2 dq'M dq                     (flexible: see tensegFlexTerms)
%     Vg = -G'q
%     Vs = string energy (see tensegStrings for the two actuation modes)
%   dEdq  (1 x 3n) = -G' + (Yhat sigma)'  [+ grad Vb' + dT/dq' if flexible]
%   dEddq (1 x 3n) = dq'M                 [= (dT/ddq)' if flexible]

st = tensegStrings(S, q, dq, u);
Vg = -S.G'*q;
if S.flexible
    ft = tensegFlexTerms(S, q, dq);
    T = ft.T;  Vb = ft.Vb;
    dEdq  = (-S.G + st.fint + ft.gradVb + ft.dTdq)';
    dEddq = ft.p';
else
    Mdq = S.M*dq;
    T = 0.5*dq'*Mdq;  Vb = 0;
    dEdq  = (-S.G + st.fint)';
    dEddq = Mdq';
end
E = T + Vg + st.Vs + Vb;
parts = struct('T', T, 'Vg', Vg, 'Vs', st.Vs, 'Vb', Vb);
end
