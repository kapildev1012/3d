function f = tensegGroundForce(q, dq, gp)
%TENSEGGROUNDFORCE  Penalty ground contact for nodes (external force f).
%
%   f = tensegGroundForce(q, dq, gp)
%
%   Flat floor at z = gp.z0. For every node below the floor:
%     normal     Fn = max(0, k*(z0 - z) - c*z_dot)
%     tangential Ft = -mu_v * Fn * v_xy      (viscous, Fn-scaled friction)
%   gp fields: z0 (0), k (2e4 N/m), c (50 N s/m), mu_v (5 s/m).
%   Its work enters Wf like any other non-conservative force, so the
%   energy balance E = E0 + Wf stays valid during impacts.

z0 = getOpt(gp, 'z0', 0);  k = getOpt(gp, 'k', 2e4);
c = getOpt(gp, 'c', 50);   mu = getOpt(gp, 'mu_v', 5);
N  = reshape(q, 3, []);  V = reshape(dq, 3, []);
pen = z0 - N(3,:);
Fn = max(0, k*pen - c*V(3,:)).*(pen > 0);
F = zeros(size(N));
F(3,:) = Fn;
F(1:2,:) = -mu*[Fn; Fn].*V(1:2,:);
f = F(:);
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
