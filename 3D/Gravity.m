function f = Gravity(X,U,omega,args,desFunc)
%GRAVITY Gravity force vector calculation for 3D tensegrity nodes

switch desFunc
    case 'genF'
        gravity = args.gravity;
        genF = zeros(size(omega.X0),class(X.p));
        genF(3:3:end) = -omega.M.*gravity;
        f = genF;
    case 'dgenFdp'
        dgenFdp = zeros(size(omega.X0,1),size(omega.X0,1),class(X.p));
        f = dgenFdp;
    case 'dgenFdpDOT'
        dgenFdpDOT = zeros(size(omega.X0,1),size(omega.X0,1),class(X.p));
        f = dgenFdpDOT;
    case 'dgenFdRL'
        dgenFdRL = zeros(size(omega.X0,1),size(omega.C,1),class(X.p));
        f = dgenFdRL;
    case 'dgenFdL'
        dgenFdL = zeros(size(omega.X0,1),size(omega.R,1),class(X.p));
        f = dgenFdL;
end

end
