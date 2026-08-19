function a = dynamics(v, u, cfg)
%DYNAMICS  Right-hand side of the agent dynamics, Eq. (9).
%
%   a = DYNAMICS(v, u, cfg) returns the acceleration of every agent,
%
%       m*qddot_i = -c*qdot_i + sat_{u_max}(u_i)      (Eq. (9))
%   =>  qddot_i   = ( -c*v_i + u_i ) / m
%
%   u must already be saturated (see CONTROL_INPUT / SATURATE).

a = (-cfg.c * v + u) / cfg.m;
end
