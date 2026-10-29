% MainFile.m - particle-filter identification of the planar tensegrity
% parameters ("Final Codes"). Runs in MATLAB and GNU Octave.
%
% Runnable copy of final_codes/original/MainFile.m; the algorithm is unchanged.
% Changes:
%  - the helper functions are in f_resample_indices.m and f_resample_residual.m
%    (Octave cannot call functions that are defined at the end of a script);
%  - yline is replaced by plot (yline does not exist in Octave);
%  - the parameters are labelled by what forward_tensegrity.m does with them:
%    del_s1..del_s3 scale the rest lengths of struts 1-3, del_f1 and del_f2 the
%    stiffness of spring types 1 and 2 (dimensionless factors, not N/mm);
%  - all 18 nodal coordinates are observed (the old comment said nodes 4 and 6);
%  - FINAL_EXP_NO and FINAL_PARTICLES (environment variables) shorten a run;
%  - the figure and the estimates are saved next to this file
%    (parameter_estimation.png / parameter_estimation.mat).

clear; close all; clc;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
headless = false;
if exist('OCTAVE_VERSION', 'builtin')
    tk = available_graphics_toolkits();
    if ~any(strcmp(tk, 'qt')) && any(strcmp(tk, 'gnuplot'))
        graphics_toolkit('gnuplot'); headless = true;   % Octave without a display
    end
end

%% 1. Synthetic Ground Truth Generation
rng(46); % Fixed random seed

exp_no = 100;
if ~isempty(getenv('FINAL_EXP_NO')), exp_no = str2double(getenv('FINAL_EXP_NO')); end
fixed_param = [];
var_param   = [0.8, 1.2, 0.7, 1.2, 0.8]; % True [del_s1 del_s2 del_s3 del_f1 del_f2]: strut rest-length and spring stiffness factors
F_true      = [-0.01 + 0.02 * rand(exp_no, 2), zeros(exp_no, 1), 2 + randi(4, exp_no, 1)]';
param_input = [fixed_param, var_param];
param_names = {'\delta_{s1} strut 1 length', '\delta_{s2} strut 2 length', '\delta_{s3} strut 3 length', ...
               '\delta_{f1} spring 1 stiffness', '\delta_{f2} spring 2 stiffness'};

fprintf('Generating synthetic measurement data...\n');
[output, Y_true] = forward_tensegrity_datasim(param_input, F_true, exp_no);

% Observation noise on all 18 nodal coordinates (6 nodes x, y, z)
len_obs       = size(Y_true, 1);
sigma_obs     = 0.000; % Standard deviation of sensor noise [mm]
sigma_assumed = 0.001;
R_cov         = (sigma_assumed^2) * eye(len_obs);

Y_noisy = Y_true + sigma_obs * randn(size(Y_true));

%% 2. Particle Filter Setup
fprintf('Particle filtering process...\n');
num_particles = 100;
if ~isempty(getenv('FINAL_PARTICLES')), num_particles = str2double(getenv('FINAL_PARTICLES')); end

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

if headless
    fig = figure('Color', 'w', 'Position', [50, 50, 1150, 550], 'Visible', 'off');
else
    fig = figure('Color', 'w', 'Position', [50, 50, 1150, 550]);
end
hold on; grid on; box on;

% Color palette for parameters
colors = lines(num_param);
h = [];   % legend entries (estimate + true line per parameter)

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
    h(end+1) = plot(steps, theta_est_history(m, :), 'LineWidth', 2.0, 'Color', c, ...
        'DisplayName', ['Est ' param_names{m}]);

    % True Ground Truth Line (plot instead of yline: also runs in Octave)
    h(end+1) = plot([1, exp_no], [var_param(m), var_param(m)], '--', 'Color', c * 0.7, 'LineWidth', 1.4, ...
        'DisplayName', sprintf('True %s = %.2f', param_names{m}, var_param(m)));
end

xlabel('Forcing step / dataset k', 'FontSize', 12);
ylabel('Parameter factor (-)', 'FontSize', 12);
title('Parameter estimation convergence with 95% confidence bands (\pm2\sigma)', 'FontSize', 13);

lg = legend(h, 'Location', 'northeastoutside');
set(lg, 'FontSize', 10);
xlim([1, exp_no]);
ylim([theta_min - 0.05, theta_max + 0.05]);
set(gca, 'FontSize', 11);

print(fig, '-dpng', '-r150', fullfile(here, 'parameter_estimation.png'));
save(fullfile(here, 'parameter_estimation.mat'), 'theta_est_history', 'theta_var_history', ...
     'eff_particles_hist', 'var_param', 'F_true');
fprintf('Saved parameter_estimation.png and parameter_estimation.mat in %s\n', here);
fprintf('Final estimate: %s (true %s)\n', mat2str(theta_est_history(:, end)', 4), mat2str(var_param, 4));
