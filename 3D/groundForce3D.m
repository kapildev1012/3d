function [ug, h_ground] = groundForce3D(q, v, cfg)
%GROUNDFORCE3D  Ground reaction and friction force on COMSOL random rough surface.
%
%   [ug, h_ground] = GROUNDFORCE3D(q, v, cfg)
%
%   Computes physical 3-D contact forces when lander nodes touch the random terrain:
%     1. Height and gradients from COMSOL-based Fourier surface synthesis
%     2. Outward surface normal: n_g = [-dh/dx; -dh/dy; 1] / norm
%     3. Normal penalty force: F_n = max(0, kg * delta - cg * v_n) * n_g
%     4. Tangential friction & lateral damping opposing tangential velocity

n = size(q, 2);
ug = zeros(3, n);
h_ground = zeros(1, n);

if ~cfg.enableGround
    return;
end

x = q(1, :);
y = q(2, :);
z = q(3, :);

% Evaluate random rough surface height and analytical gradients
[h, dhdx, dhdy] = evalSurface(x, y, cfg);
h_ground = h;

r_node = cfg.nodeRadius;

% Smooth approximation functions from FloorForceVertical_XIncline & FloorForceHorizontal_ViscousFriction
sMax  = @(x, Beta) (sqrt(x.^2 + Beta^2) + x) / 2;
sStep = @(x, Beta) 0.5 * (x ./ sqrt(x.^2 + Beta^2) + 1);

Beta = 1e-3;

for i = 1:n
    surface_z = h(i) + r_node;
    ztilde = surface_z - z(i); % penetration / proximity
    
    if ztilde > -0.05
        % Outward surface normal vector
        raw_norm = [-dhdx(i); -dhdy(i); 1.0];
        norm_len = norm(raw_norm);
        ng = raw_norm / norm_len;
        
        % Normal velocity
        vn = dot(v(:,i), ng);
        
        % Normal force using sMax (stiffness) + sStep (normal damping)
        fn_spring = sMax(ztilde * cfg.kg, Beta);
        fn_damp   = sStep(ztilde, Beta) * (-vn) * cfg.cg;
        fn_scalar = max(0, fn_spring + fn_damp);
        Fn = fn_scalar * ng;
        
        % Tangential velocity & smooth viscous friction (FloorForceHorizontal_ViscousFriction)
        vt = v(:,i) - vn * ng;
        vt_norm = norm(vt);
        
        if vt_norm > 1e-6
            tangent_dir = vt / vt_norm;
            friction_mag = sStep(ztilde, Beta) * cfg.c_gt * vt_norm + cfg.mu_g * fn_scalar * tanh(vt_norm / 0.05);
            Ft = -friction_mag * tangent_dir;
        else
            Ft = -sStep(ztilde, Beta) * cfg.c_gt * vt;
        end
        
        ug(:, i) = Fn + Ft;
    end
end

end
