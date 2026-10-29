function offsprings = f_resample_residual(weights)
% Residual resampling: floor(N w) copies, the remainder drawn from the
% normalised residual weights (moved out of MainFile.m unchanged).
    np = length(weights);
    offsprings = floor(np * weights);
    nnp = np - sum(offsprings);
    if nnp > 0
        residual_w = weights - (offsprings / np);
        sum_res    = sum(residual_w);

        if sum_res > 0
            residual_w = residual_w / sum_res;
            cdf_w      = cumsum(residual_w);

            for k = 1:nnp
                threshold = rand;
                i = find(cdf_w >= threshold, 1, 'first');
                if isempty(i); i = np; end
                offsprings(i) = offsprings(i) + 1;
            end
        end
    end
end
