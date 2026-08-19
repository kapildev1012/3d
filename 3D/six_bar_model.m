function [omega,X,constraints,generalForces] = six_bar_model()
%SIX_BAR_MODEL Defines full physical parameters and topology of 6-bar tensegrity model.

%% DEFINE ROD/CABLE PHYSICAL PARAMETERS
omega.rods.linear_velocity = 1*ones(6,1);
omega.rods.maxLength = 0.761*ones(6,1);
omega.rods.minLength = 0.4*ones(6,1);
omega.rods.constrained = [];

% mass matrix for node (12 nodes)
omega.M = 0.3/2*ones(12,1);

omega.cables.stiffness = 1.0*800*ones(24,1);
omega.cables.pretension = 1.0*30*ones(24,1);
omega.cables.linear_velocity = 0.1*ones(24,1);
omega.cables.unactuated = zeros(24,1);
omega.cables.maxLength = 0.6*ones(24,1);
omega.cables.minLength = 0.1*ones(24,1);

omega.cables.paired = []; 
omega.cables.passive = [];
omega.cables.similar = [];

%% DEFINE CABLE/ROD CONNECTIVITY MATRICES
omega.C = ...
   [0     1     0     0     0    -1     0     0     0     0     0     0;
    0     1     0     0    -1     0     0     0     0     0     0     0;
    0     1     0     0     0     0     0     0     0    -1     0     0;
    0     1     0     0     0     0     0     0     0     0     0    -1;
    1     0     0     0     0     0    -1     0     0     0     0     0;
    1     0     0     0     0     0     0    -1     0     0     0     0;
    1     0     0     0     0     0     0     0     0    -1     0     0;
    1     0     0     0     0     0     0     0     0     0     0    -1;
    0     0     1     0     0     0    -1     0     0     0     0     0;
    0     0     1     0     0     0     0    -1     0     0     0     0;
    0     0     1     0     0     0     0     0     0     0    -1     0;
    0     0     1     0     0     0     0     0    -1     0     0     0;
    0     0     0     1     0    -1     0     0     0     0     0     0;
    0     0     0     1    -1     0     0     0     0     0     0     0;
    0     0     0     1     0     0     0     0     0     0    -1     0;
    0     0     0     1     0     0     0     0    -1     0     0     0;
    0     0     0     0     0     1     0     0     0    -1     0     0;
    0     0     0     0     0     1     0     0    -1     0     0     0;
    0     0     0     0     1     0     0     0     0     0     0    -1;
    0     0     0     0     1     0     0     0     0     0    -1     0;
    0     0     0     0     0     0     1     0     0     0     0    -1;
    0     0     0     0     0     0     1     0     0     0    -1     0;
    0     0     0     0     0     0     0     1     0    -1     0     0;
    0     0     0     0     0     0     0     1    -1     0     0     0];
omega.C = sparse(omega.C);

omega.R = ...
  [-1     1     0     0     0     0     0     0     0     0     0     0;
    0     0     1    -1     0     0     0     0     0     0     0     0;
    0     0     0     0    -1     1     0     0     0     0     0     0;
    0     0     0     0     0     0     1    -1     0     0     0     0;
    0     0     0     0     0     0     0     0    -1     1     0     0;
    0     0     0     0     0     0     0     0     0     0     1    -1];
omega.R = sparse(omega.R);

%% DEFINE INITIAL NODAL POSITIONS/VELOCITIES
X.p = [
    0.1415
   -0.1884
   -0.3085
   -0.3785
   -0.0459
    0.0728
    0.3785
    0.0459
   -0.0728
   -0.1415
    0.1884
    0.3085
   -0.2290
    0.3049
   -0.0728
   -0.0924
   -0.2168
    0.3085
    0.0924
    0.2168
   -0.3085
    0.2290
   -0.3049
    0.0728
    0.2339
    0.0283
    0.3085
   -0.1495
   -0.3507
   -0.0728
    0.1495
    0.3507
    0.0728
   -0.2339
   -0.0283
   -0.3085
   ];

X.p(3:3:end) = X.p(3:3:end)-min(X.p(3:3:end))+0.1;
omega.X0 = X.p;

X.pDOT = zeros(36,1);

z = @(p,V) -kron(V'*V,eye(3))*p;
separationDist = @(p,V) sqrt(z(p,V)'*z(p,V)/2);

C = omega.C;
for cable = 1:size(C,1)
    X.RL(cable) = separationDist(X.p,C(cable,:));
end
X.RL = X.RL';

R = omega.R;
for rod = 1:size(R,1)
    X.L(rod) = separationDist(X.p,R(rod,:));
end
X.L = X.L';

omega.cableConstraintMatrix = eye(size(omega.C,1));
omega.rodConstraintMatrix = eye(size(omega.R,1));

omega.constraints{1}.filename = 'RodConstraints';
omega.constraints{1}.args = [];

baseFloor = min(X.p(3:3:end))-0.1;
omega.generalForces{1}.filename = 'FloorForceVertical_XIncline';
omega.generalForces{1}.args.baseFloor = baseFloor;
omega.generalForces{1}.args.stiffness = 5e4;
omega.generalForces{1}.args.damping = 1e-1;
omega.generalForces{1}.args.Beta = 1e-3;
omega.generalForces{1}.args.incline = 0;

omega.generalForces{2}.filename = 'Gravity';
omega.generalForces{2}.args.gravity = 9.81;

omega.generalForces{3}.filename = 'GeneralXYZDamping';
omega.generalForces{3}.args.damping = 5e-1;

omega.generalForces{4}.filename = 'FloorForceHorizontal_ViscousFriction';
omega.generalForces{4}.args.baseFloor = baseFloor;
omega.generalForces{4}.args.damping = 5e-1;
omega.generalForces{4}.args.Beta = 1e-3;

constraints = omega.constraints;
generalForces = omega.generalForces;

end
