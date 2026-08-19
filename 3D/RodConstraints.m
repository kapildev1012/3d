function g = RodConstraints(X,U,hVars,omega,args,desFunc)
%RODCONSTRAINTS Matrix calculations for rigid rod length constraints.

switch desFunc
    case 'G' % position level: rod length error
        numRods = size(omega.R,1);  
        G = zeros(numRods,1,class(X.p));  
        for i = 1:numRods
            G(i) = (1/2)*(hVars.v{i}'*hVars.v{i}) - X.L(i)^2;
        end
        g = G;
    case 'GDOT' % velocity level
        numRods = size(omega.R,1);  
        GDOT = zeros(numRods,1,class(X.p));
        for i = 1:numRods
            GDOT(i) = X.p'*(hVars.RhatRhat{i})*X.pDOT;
        end
        g = GDOT;
    case 'dGdp' % acceleration/dynamics level jacobian
        numRods = size(omega.R,1);       
        dGdp = zeros(numRods,size(omega.X0,1),class(X.p));
        for i = 1:numRods
            dGdp(i,:) = X.p'*(hVars.RhatRhat{i});
        end
        g = dGdp;
    case 'dGDOTdp'
        numRods = size(omega.R,1);  
        dGDOTdp = zeros(numRods,size(omega.X0,1),class(X.p));
        for i = 1:numRods
            dGDOTdp(i,:) = X.pDOT'*(hVars.RhatRhat{i});
        end
        g = dGDOTdp;
    case 'dGdpDOT'
        g = [];
    case 'dGDOTdpDOT'
        g = [];
    case 'dGdRL'
        g = [];
    case 'dGDOTdRL'
        g = [];
    case 'dGdL'
        g = [];
    case 'dGDOTdL'
        g = [];
end

end
