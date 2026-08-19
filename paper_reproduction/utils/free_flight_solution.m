function [y, v] = free_flight_solution(t, cfg)
%FREE_FLIGHT_SOLUTION  Exact solution of Eq. (9) with only the navigation input.
%
%   [y, v] = FREE_FLIGHT_SOLUTION(t, cfg)
%
%   With balanced internal forces (the formation starts in self-stress) and no
%   obstacle in range, Eq. (9) reduces for every agent to the scalar ODE
%
%       m*ydd = -c*yd + un ,   y(0) = 0 , yd(0) = 0
%
%   whose solution is
%
%       yd(t) = (un/c) (1 - exp(-c t/m))
%       y (t) = (un/c) t - (m*un/c^2) (1 - exp(-c t/m))
%
%   Terminal speed v_inf = un/c = 3/1.5 = 2 length units per time unit.
%   This is the analytic yardstick used in validation tests T8 and T14.

un = cfg.u_nav(2);
v  = (un/cfg.c) * (1 - exp(-cfg.c*t/cfg.m));
y  = (un/cfg.c) * t - (cfg.m*un/cfg.c^2) * (1 - exp(-cfg.c*t/cfg.m));
end
