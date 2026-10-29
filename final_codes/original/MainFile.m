
clear; close all; clc;

%% 1. Synthetic Ground Truth Generation
rng(46); % Fixed random seed

exp_no = 100;
fixed_param = [];
var_param   = [0.8, 1.2, 0.7, 1.2, 0.8]; % True spring stiffnesses [k_f1 to k_f5] in N/mm
F_true      = [-0.01 + 0.02 * rand(exp_no, 2), zeros(exp_no, 1), 2 + randi(4, exp_no, 1)]';
param_input = [fixed_param, var_param];

fprintf('Generating synthetic measurement data...\n');
[output, Y_true] = forward_tensegrity_datasim(param_input, F_true, exp_no);

% Observation noise: tracking Nodes 4 and 6 coordinates
len_obs       = size(Y_true, 1);
sigma_obs     = 0.000; % Standard deviation of sensor noise [mm]
sigma_assumed = 0.001;  
R_cov         = (sigma_assumed^2) * eye(len_obs);

Y_noisy = Y_true + sigma_obs * randn(size(Y_true));

%% 2. Particle Filter Setup
fprintf('Particle filtering process...\n');
num_particles = 100;

theta_min = 0.7;
theta_max = 1.4;
num_param = numel(var_param);

% Initialize particles (Uniform distribution between theta_min and theta_max)
theta_particles = theta_min + (theta_max - theta_min) * rand(num_param, num_particles);
weights         = (1 / num_particles) * ones(1, num_particles); % Uniform weights

% Preallocation for tracking history & variance
theta_est_history  = zeros(num_param, exp_no);
theta_var_history  = zeros(num_param, exp_no); % Variance storage
eff_particles_hist = zeros(1, exp_no);

sigma_jitter     = 0.01; 
sigma_jitter_min = 0.005; 
degrade_rate     = 0.995; 
max_attempts     = 100;

%% 3. Sequential Filtering Loop
for k = 1:exp_no
    F_inp = F_true(:, k);
    Yk    = Y_noisy(:, k);
    
    % A. PREDICT / PROPAGATE & JITTER
    if k > 1
        % Add Gaussian jitter to prevent particle collapse
        theta_particles = theta_particles + sigma_jitter * randn(num_param, num_particles);
        
        % Physical Constraint Enforcement
        theta_particles = max(min(theta_particles, theta_max), theta_min);
    end
    
    % B. EVALUATE LIKELIHOOD
    logw = zeros(1, num_particles);
    for i = 1:num_particles
        theta_i = theta_particles(:, i);   
        output  = 0; 
        attempt = 0;
        
        % Solve forward model (retry with small perturbation if solver fails)
        while output == 0 && attempt < max_attempts
            attempt = attempt + 1;
            param_input = [fixed_param, theta_i'];
            [output, Y_pred] = forward_tensegrity(param_input, F_inp');
            
            if output == 0
                % Perturb working copy theta_i (Do NOT modify theta_particles directly during loop)
                theta_i = theta_particles(:, i) + sigma_jitter * randn(num_param, 1);
                theta_i = max(min(theta_i, theta_max), theta_min);
            end
        end
        
        % Save successful particle state back
        theta_particles(:, i) = theta_i;
        
        if output == 0
            % Solver failed across all attempts; assign negligible log likelihood
            logw(i) = -1e10;
        else
            residual = (Yk - reshape(Y_pred, len_obs, 1));
            logw(i)  = -0.5 * (residual' * (R_cov \ residual));
        end
    end
    
    % C. LOG-SUM-EXP NORMALIZATION (Prevents Numerical Underflow)
    maxLogW = max(logw);
    weights = exp(logw - maxLogW);
    sum_w   = sum(weights);
    
    if sum_w < 1e-18 || isnan(sum_w)
        weights = ones(1, num_particles) / num_particles;
    else
        weights = weights / sum_w;
    end
    
    % D. STATE ESTIMATION & VARIANCE CALCULATION
    % 1. Weighted Mean Estimation
    theta_mean = theta_particles * weights';
    theta_est_history(:, k) = theta_mean;
    
    % 2. Weighted Variance Calculation: Var(theta) = sum( w_i * (theta_i - mu)^2 )
    theta_diff = theta_particles - theta_mean; % [num_param x num_particles]
    theta_var_history(:, k) = (theta_diff.^2) * weights';
    
    % Handle potential NaN fallback
    if any(isnan(theta_est_history(:, k)))
        if k > 1
            theta_est_history(:, k) = theta_est_history(:, k-1);
            theta_var_history(:, k) = theta_var_history(:, k-1);
        else
            theta_est_history(:, k) = mean(theta_particles, 2);
            theta_var_history(:, k) = var(theta_particles, 0, 2);
        end
    end
    
    % E. EFFECTIVE PARTICLE SIZE & RESAMPLING
    N_eff = 1 / sum(weights.^2);
    eff_particles_hist(k) = N_eff;
    
    if N_eff < 0.5 * num_particles
        offsprings      = f_resample_residual(weights);
        idr             = f_resample_indices(offsprings);
        theta_particles = theta_particles(:, idr);
        weights         = (1 / num_particles) * ones(1, num_particles);
    end
    
    % Decay jitter rate
    sigma_jitter = max(sigma_jitter_min, sigma_jitter * degrade_rate);
    
    % Print step progress
    fprintf('Step %3d/%d | Est Theta: %s\n', k, exp_no, mat2str(theta_est_history(:, k)', 4));
end

fprintf('Estimation Complete.\n');


%% 4. Plotting Parameter Estimation History with 95% Confidence Bounds
fprintf('Generating Visualization Plot...\n');

steps = 1:exp_no;
theta_std_history = sqrt(theta_var_history); % Standard deviation (sigma)

% Single clean figure window
figure('Color', 'w', 'Position', [50, 50, 1150, 550]);
hold on; grid on; box on;

% Color palette for parameters
colors = lines(num_param);

for m = 1:num_param
    c = colors(m, :);
    
    % Compute 2-sigma bounds (approx. 95% confidence interval)
    upper_bound = theta_est_history(m, :) + 2 * theta_std_history(m, :);
    lower_bound = theta_est_history(m, :) - 2 * theta_std_history(m, :);
    
    % Shaded 95% Confidence Interval Band (2-sigma)
    fill([steps, fliplr(steps)], [upper_bound, fliplr(lower_bound)], c, ...
        'FaceAlpha', 0.45, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
    
    % Estimated Parameter Mean Trajectory
    plot(steps, theta_est_history(m, :), 'LineWidth', 2.0, 'Color', c, ...
        'DisplayName', sprintf('Est k_{f%d}', m));
    
    % True Ground Truth Line
    yline(var_param(m), '--', 'Color', c * 0.7, 'LineWidth', 1.4, ...
        'DisplayName', sprintf('True k_{f%d} = %.2f', m, var_param(m)));
end

% Formatting for Academic Publication
xlabel('Forcing Step / Dataset (k)', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('Stiffness Parameter $k_f$ (N/mm)', 'FontSize', 12, 'Interpreter', 'latex');
title('Parameter Estimation Convergence with 95\% Confidence Intervals ($2\sigma$ Bounds)', ...
      'FontSize', 13, 'Interpreter', 'latex');

legend('Location', 'northeastoutside', 'FontSize', 10, 'Interpreter', 'tex');
xlim([1, exp_no]);
ylim([theta_min - 0.05, theta_max + 0.05]);
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

%% =========================================================================
%  HELPER FUNCTIONS
%  =========================================================================

function indices = f_resample_indices(offsprings)
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

function offsprings = f_resample_residual(weights)
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