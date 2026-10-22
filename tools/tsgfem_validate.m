function tsgfem_validate(model_file, out_file)
% Run the ORIGINAL TsgFEM functions (third_party/TsgFEM/Function_library)
% on the rover model exported by tools/tsgfem_validate.mjs and write the
% results as JSON for comparison with the JavaScript port (js/tsgfem.js).
% Works in GNU Octave >= 7 and MATLAB (needs fsolve for hollow bars).
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'third_party', 'TsgFEM', 'Function_library'));
global Eb Es %#ok<GVMIS> used by TsgFEM's minimass_buckle_hollow
d = jsondecode(fileread(model_file));

N = d.nodes';                                   % 3 x nn
C_b = tenseg_ind2C(d.bars, N);
C_s = tenseg_ind2C([d.outerStrings; d.coreCables], N);
C = [C_b; C_s];
[ne, nn] = size(C);
[Ia, ~, ~, ~] = tenseg_boundary([], [], [], nn);
gr = {d.groups.bars'; d.groups.outer'; d.groups.core'};
Gp = tenseg_str_gp(gr, C);

% statics: equilibrium matrix, SVD, prestress design
[A_1a, A_1ag, ~, ~, l, l_gp] = tenseg_equilibrium_matrix1(N, C, Gp, Ia);
[~, ~, ~, V2] = tenseg_svd(A_1ag);
w0a = zeros(size(A_1a, 1), 1);
[~, ~, q, t] = tenseg_prestress_design(Gp, l, l_gp, A_1ag, V2, w0a, 2, d.cablePretension);

% minimal-mass design (hollow bars, given wall thickness)
Eb = d.barMaterial.E; Es = d.stringMaterial.E;
index_b = d.groups.bars'; index_s = [d.groups.outer', d.groups.core'];
t_mm = t; t_mm(index_s) = max(t_mm(index_s), 0);
[~, ~, ~, A_mm, ~, ~, ~, radius, ~, ~, ~, mass_mm] = tenseg_minimass(t_mm, l, Gp, d.stringMaterial.sigma, ...
  d.barMaterial.sigma, Eb, Es, index_b, index_s, d.c_b, d.c_s, d.barMaterial.rho, d.stringMaterial.rho, d.thick, 1);

% modal analysis of the installed rover
E = d.memberE; A = d.memberA; l0 = d.memberL0;
K_T = tenseg_stiff_matx2(C, N(:), E, A, l0);
M = tenseg_mass_matrix(d.memberMass, C, 0) + kron(diag(d.nodeMass), eye(3));
lam = sort(real(eig(Ia'*K_T*Ia, Ia'*M*Ia)));
freq = sqrt(max(lam, 0))/2/pi;

r.rank = rank(A_1a);
r.selfStressModesGrouped = size(V2, 2);
r.barForce = t(index_b(1));
r.cableForce = t(d.groups.outer(1));
r.qBar = q(index_b(1));
r.barRadius = radius(index_b(1));
r.cableRadius = radius(d.groups.outer(1));
r.structureMass = sum(mass_mm(~isnan(mass_mm)));   % zero-force strings: 0/0 rest length
r.frequencies = freq';
fid = fopen(out_file, 'w'); fprintf(fid, '%s', jsonencode(r)); fclose(fid);
end
