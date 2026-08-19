function f = FloorForceHorizontal_ViscousFriction(X,U,omega,args,desFunc)
%FLOORFORCE Horizontal viscous friction floor force calculation
%   Computes horizontal viscous friction forces on nodes contacting the ground.

switch desFunc
    case 'genF'
        baseFloor = args.baseFloor; % floor height Z-value
        damping = args.damping;   % floor damping
        Beta = args.Beta;         % parameter for softMax/softStep
        
        % sStep (smooth approximation to 0-1 step)
        sStep = @(x,Beta) 1/2*((x.^2+Beta^2).^(-1/2).*x+1);
        
        genF = zeros(size(omega.X0,1),1,class(X.p));
        ztilde = baseFloor - X.p(3:3:end); 
        
        % x-friction
        genF(1:3:end) = ...
            sStep(ztilde,Beta).*...
            (-X.pDOT(1:3:end))*damping;
        % y-friction
        genF(2:3:end) = ...
            sStep(ztilde,Beta).*...
            (-X.pDOT(2:3:end))*damping;
        
        f = genF;
    case 'dgenFdp'
        baseFloor = args.baseFloor;
        damping = args.damping;
        Beta = args.Beta;
                
        ztilde = baseFloor - X.p(3:3:end);
        
        dgenFxdp = 1/2*(-(ztilde.^2+(Beta)^2).^(-3/2).*(ztilde.^2)+...
            (ztilde.^2+(Beta)^2).^(-1/2))*damping.*(-X.pDOT(1:3:end))*(-1);
        dgenFdp = diag(kron(dgenFxdp,[1 0 0]'));     
        
        dgenFydp = 1/2*(-(ztilde.^2+(Beta)^2).^(-3/2).*(ztilde.^2)+...
            (ztilde.^2+(Beta)^2).^(-1/2))*damping.*(-X.pDOT(2:3:end))*(-1);
        dgenFdp = dgenFdp + diag(kron(dgenFydp,[0 1 0]'));
        
        f = dgenFdp;
    case 'dgenFdpDOT'
        baseFloor = args.baseFloor;
        damping = args.damping;
        Beta = args.Beta;
        
        ztilde = baseFloor - X.p(3:3:end);
                
        dgenFdpDOT = -1/2*((ztilde.^2+(Beta)^2).^(-1/2).*ztilde+1)*damping;
        dgenFdpDOT = diag(kron(dgenFdpDOT,[1 1 0]'));
        
        f = dgenFdpDOT;
    case 'dgenFdRL'
        dgenFdRL = zeros(size(X.p,1),size(X.RL,1),class(X.p));
        f = dgenFdRL;
    case 'dgenFdL'
        dgenFdL = zeros(size(X.p,1),size(X.L,1),class(X.p));
        f = dgenFdL;
end

end
