function [topo, info] = tensegStructures(name, opts)
%TENSEGSTRUCTURES  Benchmark topologies for tensegGenMat.
%
%   [topo, info] = tensegStructures(name)
%   [topo, info] = tensegStructures(name, opts)
%
%   'sixbar'  : the 6-bar / 24-cable ball of drive_matlab/six_bar_model.m
%               (12 nodes). Node coordinates, cable/rod connectivity,
%               cable stiffness, pretension and node masses are read from
%               your six_bar_model.m. Each bar carries the mass of its two
%               end nodes (2 x omega.M) as a uniform cylinder.
%               opts.barRadius (0.01 m), opts.pointMassFraction (0): share
%               of each node mass kept as a point mass at the node instead
%               of in the bar (needed > 0 for flexible bars).
%   'tbar2d'  : planar 2-bar structure in the x-z plane after Fig. 1(a) of
%               the paper: bottom nodes pinned, bars 1->3 and 2->4, springs
%               1-4 (left), 2-3 (right), 4-3 (top); bar length 5 m, radius
%               0.05 m, density 500 kg/m^3, spring stiffness 100 N/m, left
%               and right springs at 90 % rest length, zero gravity.
%   'prism3'  : 3-bar tensegrity prism (9 strings), handy small 3-D test.
%
%   info carries suggested defaults (gravity, pretension, notes).

if nargin < 2, opts = struct(); end
info = struct('gravity', [0;0;-9.806], 'pretension', [], 'notes', '');

switch lower(name)
    case 'sixbar'
        here = fileparts(mfilename('fullpath'));
        driveDir = fullfile(here, '..', '..', 'drive_matlab');
        if exist(driveDir, 'dir'), addpath(driveDir); end
        [omega, X] = six_bar_model();
        N0 = reshape(X.p, 3, []);
        n  = size(N0, 2);
        Cb = full(omega.R);
        Cs = full(omega.C);
        nodeMass = omega.M(:);
        if numel(nodeMass) == 1, nodeMass = repmat(nodeMass, n, 1); end
        frac = getOpt(opts, 'pointMassFraction', 0);
        nb = size(Cb,1);
        mb = zeros(nb,1);
        for k = 1:nb
            ends = find(Cb(k,:) ~= 0);
            mb(k) = (1 - frac)*sum(nodeMass(ends));
        end
        topo = struct();
        topo.name = 'sixbar (drive_matlab/six_bar_model.m)';
        topo.N0 = N0;  topo.Cb = Cb;  topo.Cs = Cs;
        topo.barMass = mb;
        topo.barRadius = getOpt(opts, 'barRadius', 0.01);
        if frac > 0
            topo.pmNodes = (1:n)';
            topo.pmMass = frac*nodeMass;
        end
        topo.stringStiffness = omega.cables.stiffness(:);
        topo.stringDamping = getOpt(opts, 'stringDamping', 0);
        topo.barStiffness = getOpt(opts, 'barStiffness', 1e5);
        topo.barPoisson = getOpt(opts, 'barPoisson', 0.3);
        info.gravity = [0; 0; -9.81];
        info.pretension = omega.cables.pretension(:);
        info.cableMinLength = omega.cables.minLength(:);
        info.cableMaxLength = omega.cables.maxLength(:);
        info.notes = 'Rest lengths: use tensegSelfStress to prestress.';

    case 'tbar2d'
        Lb = 5;  w = Lb/sqrt(2);
        N0 = [0 w w 0;               % x
              0 0 0 0;               % y  (plane)
              0 0 w w];              % z
        Cb = [-1 0 1 0;               % bar 1->3
               0 -1 0 1];             % bar 2->4
        Cs = [-1 0 0 1;               % left  1-4
               0 -1 1 0;              % right 2-3
               0 0 1 -1];             % top   4-3
        topo = struct();
        topo.name = 'tbar2d (paper Fig. 1a)';
        topo.N0 = N0;  topo.Cb = Cb;  topo.Cs = Cs;
        topo.barRadius = 0.05;  topo.barDensity = 500;
        topo.stringStiffness = 100;
        topo.stringRestFraction = [0.9; 0.9; 1.0];
        topo.stringDamping = getOpt(opts, 'stringDamping', 0);
        topo.fixedDofs = [1 1; 1 3; 2 1; 2 3];
        topo.planeAxis = 2;
        topo.barStiffness = getOpt(opts, 'barStiffness', 1e6);
        info.gravity = [0; 0; 0];

    case 'prism3'
        R = 1;  H = 1.2;  tw = 5*pi/6;
        a = (0:2)*2*pi/3;
        bot = [R*cos(a); R*sin(a); zeros(1,3)];
        top = [R*cos(a+tw); R*sin(a+tw); H*ones(1,3)];
        N0 = [bot top];
        Cb = zeros(3,6);  Cs = zeros(9,6);
        for i = 1:3
            Cb(i, i) = -1;  Cb(i, 3+i) = 1;                       % bars
            j = mod(i,3) + 1;
            Cs(i, i) = -1;      Cs(i, j) = 1;                      % bottom ring
            Cs(3+i, 3+i) = -1;  Cs(3+i, 3+j) = 1;                  % top ring
            Cs(6+i, i) = -1;    Cs(6+i, 3+j) = 1;                  % verticals
        end
        topo = struct();
        topo.name = 'prism3';
        topo.N0 = N0;  topo.Cb = Cb;  topo.Cs = Cs;
        topo.barMass = 0.5;  topo.barRadius = 0.01;
        topo.stringStiffness = 500;
        topo.stringDamping = getOpt(opts, 'stringDamping', 0);
        topo.barStiffness = getOpt(opts, 'barStiffness', 1e5);
        info.gravity = [0; 0; 0];
    otherwise
        error('tensegStructures:name', 'Unknown structure "%s".', name);
end
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
