function f = FloorForceVertical_XIncline(X,U,omega,args,desFunc)
%FLOORFORCE Vertical floor force with optional X-axis incline calculation
%   Computes vertical spring-damper reaction forces on nodes touching the ground.

switch desFunc
    case 'genF'
        baseFloor = args.baseFloor; % floor height Z-value
        stiffness = args.stiffness; % floor stiffness
        damping = args.damping;   % floor damping
        Beta = args.Beta;         % parameter for softMax/softStep
        incline = args.incline;   % incline in degrees in +X direction
        
        sMax = @(x,Beta) (sqrt(x.^2+Beta^2)+x)/2;
        sStep = @(x,Beta) 1/2*((x.^2+Beta^2).^(-1/2).*x+1);
        
        genF = zeros(size(omega.X0,1),1,class(X.p));
        verticalLift = tand(incline)*X.p(1:3:end);
        genF(3:3:end) = ...
            sMax((baseFloor + verticalLift - X.p(3:3:end))*stiffness,Beta) +...
            sStep((baseFloor + verticalLift - X.p(3:3:end)),Beta).*...
            (-X.pDOT(3:3:end))*damping;
        f = genF;
    case 'dgenFdp'
        baseFloor = args.baseFloor;
        stiffness = args.stiffness;
        damping = args.damping;
        Beta = args.Beta;
        incline = args.incline;
                
        verticalLift = tand(incline)*X.p(1:3:end);
        x_tilde = baseFloor + verticalLift - X.p(3:3:end);
        x = x_tilde*stiffness;
        
        dgenFdp = 1/2*((x.^2+(Beta)^2).^(-1/2).*x+1)*(-stiffness) +...
            1/2*(-(x_tilde.^2+(Beta)^2).^(-3/2).*(x_tilde.^2)+...
            (x_tilde.^2+(Beta)^2).^(-1/2))*damping.*(-X.pDOT(3:3:end))*(-1);
        dgenFdp = diag(kron(dgenFdp,[0 0 1]'));
        f = dgenFdp;
    case 'dgenFdpDOT'
        baseFloor = args.baseFloor;
        damping = args.damping;
        Beta = args.Beta;
        incline = args.incline;
        
        verticalLift = tand(incline)*X.p(1:3:end);
        x_tilde = baseFloor + verticalLift - X.p(3:3:end);
                
        dgenFdpDOT = -1/2*((x_tilde.^2+(Beta)^2).^(-1/2).*x_tilde+1)*damping;
        dgenFdpDOT = diag(kron(dgenFdpDOT,[0 0 1]'));
        f = dgenFdpDOT;
    case 'dgenFdRL'
        dgenFdRL = zeros(size(omega.X0,1),size(omega.C,1),class(X.p));
        f = dgenFdRL;
    case 'dgenFdL'
        dgenFdL = zeros(size(omega.X0,1),size(omega.R,1),class(X.p));
        f = dgenFdL;
end

end
