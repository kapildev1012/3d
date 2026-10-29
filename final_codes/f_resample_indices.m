function indices = f_resample_indices(offsprings)
% Particle indices from offspring counts (moved out of MainFile.m unchanged).
    N = sum(offsprings);
    indices = zeros(1, N);
    ptr = 1;
    for i = 1:length(offsprings)
        count = offsprings(i);
        if count > 0
            indices(ptr : ptr + count - 1) = i;
            ptr = ptr + count;
        end
    end
end
