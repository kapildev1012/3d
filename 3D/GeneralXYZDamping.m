function f = GeneralXYZDamping(X,U,omega,args,desFunc)
%GENERALXYZDAMPING General viscous damping in 3D (X, Y, Z directions)

switch desFunc
    case 'genF'
        damping = args.damping;    
        genF = -damping*X.pDOT;
        f = genF;
    case 'dgenFdp'
        dgenFdp = zeros(size(omega.X0,1),size(omega.X0,1),class(X.p));
        f = dgenFdp;
    case 'dgenFdpDOT'
        damping = args.damping;
        dgenFdpDOT = diag(ones(size(omega.X0,1),1)*(-damping));
        f = dgenFdpDOT;
    case 'dgenFdRL'
        dgenFdRL = zeros(size(omega.X0,1),size(omega.C,1),class(X.p));
        f = dgenFdRL;
    case 'dgenFdL'
        dgenFdL = zeros(size(omega.X0,1),size(omega.R,1),class(X.p));
        f = dgenFdL;
end

end
