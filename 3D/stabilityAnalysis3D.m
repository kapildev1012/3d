function stab = stabilityAnalysis3D(out, L, cfg)
%STABILITYANALYSIS3D  Numerical stability and energy diagnostics.
%
%   stab = STABILITYANALYSIS3D(out, L, cfg)

N = size(out.q, 3);
n = L.n;

stab.kinetic_energy = zeros(1, N);
for s = 1:N
    v = out.v(:,:,s);
    stab.kinetic_energy(s) = 0.5 * cfg.m * sum(v(:).^2);
end

stab.max_disp = zeros(1, N);
for s = 1:N
    stab.max_disp(s) = max(sqrt(sum((out.q(:,:,s) - L.q0).^2, 1)));
end

% Force density matrix and eigenvalues at nominal configuration
M = size(L.members, 1);
omega = zeros(M, 1);
for mm = 1:M
    if L.isString(mm)
        f = stringForce3D(L.l0(mm), cfg.kS, cfg.alphaS, cfg.z1, cfg.z2, cfg.beta, false);
    else
        f = barForce3D(L.l0(mm), cfg.kB, cfg.alphaB);
    end
    omega(mm) = f / L.l0(mm);
end

D = zeros(n, n);
for mm = 1:M
    i = L.members(mm,1);
    j = L.members(mm,2);
    D(i,j) = D(i,j) - omega(mm);
    D(j,i) = D(j,i) - omega(mm);
    D(i,i) = D(i,i) + omega(mm);
    D(j,j) = D(j,j) + omega(mm);
end

stab.D = D;
stab.eig_D = sort(eig(D));

end
