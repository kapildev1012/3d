function cfg = config3D(varargin)
%CONFIG3D  Parameters for the 3-D tensegrity lander obstacle avoidance.
%
%   cfg = CONFIG3D()                        returns the default configuration
%   cfg = CONFIG3D('relaxation', true, ...) returns it with fields overridden
%
%   Replicates the ECC paper behaviour in 3-D (same deform -> reform cycle):
%     - Case A (relaxation=false): rigid lander gets pushed/slowed by obstacles
%     - Case B (relaxation=true ): lander deforms around obstacles, strings
%       relax (Eqs. 19-20), then REFORM after passing (formation error drops)
%
%   Key design choices that produce visible deform->reform:
%     1. Obstacles are LARGER than the lander footprint (~1.4 m radius vs
%        ~1 m wide lander), placed at the lander's mid-body Z height so they
%        hit FRONT nodes first.  This asymmetry = visible shape deformation.
%     2. Strong string restoring (kS=100) pulls nodes back quickly once the
%        avoidance force drops to zero past the obstacle.
%     3. Relaxation threshold z1 = 1.04*l_S0 is very tight so even a small
%        obstacle-induced stretch triggers relaxation (allows deformation).
%     4. Two obstacles well-spaced (Y=10, Y=26) with T_end=55 s so the
%        lander clearly passes BOTH and travels 10+ m of free terrain after.
%     5. No inter-agent avoidance: 12 nodes are one lander, not 12 agents.

% ----------------------------------------------------------- dimension
cfg.d = 3;

% ------------------------------------------------- integration / timing
cfg.dt         = 0.01;         % [s] time step
cfg.T_end      = 55;           % [s] total sim time -- lander travels ~42 m,
                                %     clearly past both obstacles + free terrain
cfg.integrator = 'symplectic_euler';
cfg.seed       = 42;

% -------------------------------------------------- agent dynamics Eq.(9)
cfg.m     = 1.0;               % [kg] node mass
cfg.c     = 0.9;               % damping (low -- allows structure to deform freely)
cfg.u_max = 18;                % input saturation
cfg.u_nav = [0; 3.8; 0];       % navigation input (+Y)

% ------------------------------------------ tensegrity member forces Eq.(10)
% Exact 3-D self-stress: kB = -kS * 3*sqrt(6)/8
cfg.kS     = 100.0;            % string gain  (strong restoring after obstacle)
cfg.alphaS = 2.0;
cfg.kB     = -cfg.kS * (3 * sqrt(6) / 8);  % exact prestress
cfg.alphaB = -0.5;

% --------------------------------- string relaxation Eqs.(19)-(20)
% Nominal string length l_S0 = sqrt(3/8) ~ 0.6124 m
cfg.relaxation = true;
cfg.z1    = 1.04 * sqrt(3/8);  % 0.638 m -- relaxation starts at TINY 4% stretch
                                %  (same design intent as z1=15.5 vs lS=15 in 2D)
cfg.z2    = 1.50 * sqrt(3/8);  % 0.918 m -- full relaxation
cfg.beta  = 22.0;              % [N] soft cap -- low so strings still pull back

% --------------------------------- obstacle avoidance Eqs.(17)-(18)
cfg.ry         = 1.10;         % [m] sensing radius -- LARGE relative to lander
                                %     so front nodes are hit first, rear nodes later
cfg.ka         = 12.0;         % avoidance gain
cfg.gamma      = 0.4;
cfg.avoid_sign = +1;

% IMPORTANT: inter-agent avoidance = between DISTINCT landers / UAVs.
% All 12 nodes here belong to ONE lander -- enabling it fights restoring forces.
cfg.interagent_avoidance = false;

% --------------------------------------------- obstacles [x y z R]
% Obstacles are:
%   - Larger than the lander footprint (R=1.3 m > lander width ~1 m)
%   - At Z height matching the lander mid-body (~1.5 m) so they hit
%     FRONT nodes in Y before the centroid reaches the obstacle centre
%   - Offset in X to create lateral asymmetry (one side hit harder)
%   - Well-spaced in Y (Y=10 and Y=26) with gap > 12 m for clear reform
cfg.obstacles = [ ...
     0.25,  10.0,  1.45,  1.30;   % Obstacle 1: right-of-centre at mid-body Z
    -0.25,  26.0,  1.50,  1.25];  % Obstacle 2: left-of-centre

% -------------------------------------------------- noise
cfg.sigma              = 0.01;
cfg.processNoiseSigma  = 0.0;
cfg.noise_mode         = 'relative';

% -------------------------------------------------- environment
cfg.environmentMode  = 'earth';
cfg.gravity          = [0; 0; -3.5];
cfg.disturbanceForce = [0; 0; 0];

% -------------------------------------------------- COMSOL random terrain
% https://www.comsol.com/blogs/how-to-generate-random-surfaces-in-comsol-multiphysics
cfg.enableGround  = true;
cfg.groundRMS     = 0.16;     % [m] RMS roughness (moderate -- terrain is uneven
                               %     but doesn't overwhelm obstacle deformation)
cfg.groundM       = 5;
cfg.groundN       = 7;
cfg.groundNuX     = 0.09;    % base spatial freq X [1/m]
cfg.groundNuY     = 0.06;    % base spatial freq Y [1/m]
cfg.groundPower   = 1.8;
cfg.kg       = 380;           % normal spring stiffness [N/m]
cfg.cg       = 28;            % normal damping [N*s/m]
cfg.mu_g     = 0.08;          % friction (low -- lets lander glide forward)
cfg.c_gt     = 2.5;           % tangential damping
cfg.nodeRadius = 0.05;

% Legacy sinusoidal params (kept for backward compat, not used for physics)
cfg.groundA1 = 0.22; cfg.groundW1 = 0.60;
cfg.groundA2 = 0.16; cfg.groundW2 = 0.80;
cfg.groundA3 = 0.12; cfg.groundW3 = 0.35;

% -------------------------------------------------- dynamics model
cfg.dynamicsModel = 'pointMass';

% -------------------------------------------------- scalability
cfg.numLanders = 1;

% -------------------------------------------------- saturation
cfg.sat_mode = 'norm';

% -------------------------------------------------- visualization
cfg.showAnimation = true;
cfg.saveVideo     = false;
cfg.drawEvery     = 5;
cfg.followRobot   = true;
cfg.videoFile     = fullfile(fileparts(mfilename('fullpath')), ...
                       '..', 'results', 'videos', 'lander3D.mp4');

% -------------------------------------------------- output
cfg.verbose = true;
cfg.outdir  = fullfile(fileparts(mfilename('fullpath')), '..', 'results');

% -------------------------------------------------------- name/value override
if mod(numel(varargin), 2) ~= 0
    error('config3D:args', 'Arguments must be name/value pairs.');
end
for a = 1:2:numel(varargin)
    fn = varargin{a};
    if ~isfield(cfg, fn)
        error('config3D:unknownField', 'Unknown field "%s".', fn);
    end
    cfg.(fn) = varargin{a+1};
end

% ------------------------------------------------- environment mode
switch lower(cfg.environmentMode)
    case 'space',  cfg.gravity = [0;0;0];
    case 'earth',  if all(cfg.gravity==0), cfg.gravity=[0;0;-3.5]; end
    case 'custom'  % user-supplied
    otherwise, error('config3D:env','Unknown environmentMode "%s".',cfg.environmentMode);
end

% ------------------------------------------------- derived
cfg.n_steps = round(cfg.T_end / cfg.dt);

% ------------------------------------------------- guards
assert(cfg.d == 3);
assert(cfg.kS > 0);         assert(cfg.kB < 0);
assert(cfg.alphaS >= 1);    assert(cfg.alphaB < 0);
assert(cfg.z2 > cfg.z1);   assert(cfg.m > 0);
assert(cfg.c >= 0);         assert(cfg.u_max > 0);
assert(numel(cfg.u_nav)==3);
assert(cfg.numLanders >= 1);

cfg.u_nav            = cfg.u_nav(:);
cfg.gravity          = cfg.gravity(:);
cfg.disturbanceForce = cfg.disturbanceForce(:);

end
