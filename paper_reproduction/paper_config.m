function cfg = paper_config(varargin)
%PAPER_CONFIG  All parameters of the ECC-2026 tensegrity MAS paper.
%
%   cfg = PAPER_CONFIG()                  returns the paper's nominal configuration
%   cfg = PAPER_CONFIG('relaxation',true) returns it with fields overridden
%
%   Reference:
%     B. Ingalls, Q. Nelson, L. R. Garcia Carrillo, M. Majji, "Adaptive
%     Tensegrity-Based Control for Multi-Agent Obstacle Avoidance",
%     2026 European Control Conference (ECC), pp. 2442-2447.
%
%   Every value is tagged with its provenance:
%     [T1]  Table I of the paper
%     [TXT] paper body text
%     [EQ]  derived from a paper equation
%     [FIG] digitised from a paper figure (graph-derived estimate)
%     [STD] implementation choice, see ASSUMPTIONS.md
%
%   See also BUILD_FORMATION, RUN_EXPERIMENT, MAIN.

% ---------------------------------------------------------------- dimension
cfg.d          = 2;            % [TXT] "d = 2 dimensional"

% ------------------------------------------------- integration / experiment
cfg.dt         = 0.05;         % [T1]  time step
cfg.T_end      = 40;           % [TXT] Figs. 5-6 final panel
cfg.snapshots  = [0 15 30 40]; % [TXT] Figs. 5-6 panel times
cfg.integrator = 'symplectic_euler';   % [STD] A6 ('symplectic_euler'|'rk4')
cfg.seed       = 20260707;     % [STD] A7 (unspecified in paper)

% -------------------------------------------------- agent dynamics, Eq. (9)
cfg.m          = 1;            % [T1]  mass
cfg.c          = 1.5;          % [T1]  damping
cfg.u_max      = 10;           % [T1]  input saturation magnitude
cfg.u_nav      = [0; 3];       % [T1]  navigation input u_i^n
cfg.sat_mode   = 'norm';       % [STD] A3 ('norm'|'component')

% ---------------------------------- tensegrity member force, Eqs. (10),(11)
cfg.lS         = 15;           % [T1]  nominal string (tile side) length
cfg.kS         = 0.0341;       % [T1]  string gain  (= Eq. (21), rounded)
cfg.alphaS     = 2;            % [T1]  string exponent
cfg.kB         = -50;          % [T1]  bar gain
cfg.alphaB     = -0.5;         % [T1]  bar exponent
% Table I lists kS to 3 significant figures.  The exact self-stress value that
% Eq. (21) prescribes for (kB, lS, alphaS, alphaB) is kS = 0.0341168841...;
% using the rounded 0.0341 leaves a residual nodal force of 7.6e-3 (0.099 % of
% the nominal string force), which breaks the *exact* Lemma-1 test but is
% dynamically negligible.  Default = Table I verbatim.
cfg.use_eq21_kS = false;       % [STD] true -> use the unrounded Eq. (21) value

% ------------------------------------- string relaxation, Eqs. (19) - (20)
cfg.relaxation = true;         % [TXT] false -> Fig. 5, true -> Fig. 6
cfg.z1         = 15.5;         % [T1]
cfg.z2         = 50;           % [T1]
cfg.beta       = 8;            % [T1]

% --------------------------------------- obstacle avoidance, Eqs. (17),(18)
cfg.ry         = 8;            % [T1]  sensing / avoidance radius
cfg.ka         = 20;           % [T1]  avoidance gain
cfg.gamma      = 0.4;          % [T1]  avoidance exponent
cfg.avoid_sign = +1;           % [STD] A1  +1 = repulsive (used), -1 = literal Eq. (18)
cfg.interagent_avoidance = true;       % [TXT] "when obstacle j is another agent"

% ------------------------------------------------------ measurement noise
cfg.sigma      = 0.5;          % [T1]  "imposed on all measurements"
cfg.noise_mode = 'relative';   % [STD] A5 ('relative'|'distance'|'none')

% ------------------------------------------------------ formation geometry
cfg.tiles      = [2 3];        % [TXT] six squares: 2 rows x 3 columns -> 12 agents

% -------------------------------------------- obstacles [x y R], [FIG] A9
cfg.obstacles  = [   0  90  12.0
                    30  50   6.5
                   -30  30   6.5 ];

% ------------------------------------------------------------- plotting
cfg.xlim       = [-50 50];     % [FIG] Figs. 5-6 axes
cfg.ylim       = [-20 140];    % [FIG]
cfg.col_string = [0.85 0.16 0.16];
cfg.col_bar    = [0.10 0.20 0.90];
cfg.col_obs    = [1.00 0.00 0.00];

% ------------------------------------------------------------- bookkeeping
cfg.name       = 'paper_nominal';
cfg.verbose    = true;
cfg.outdir     = fullfile(fileparts(mfilename('fullpath')), 'results');

% ------------------------------------------------------- name/value override
if mod(numel(varargin), 2) ~= 0
    error('paper_config:args', 'Arguments must be name/value pairs.');
end
for a = 1:2:numel(varargin)
    fn = varargin{a};
    if ~isfield(cfg, fn)
        error('paper_config:unknownField', 'Unknown configuration field "%s".', fn);
    end
    cfg.(fn) = varargin{a+1};
end

% ------------------------------------------------------------ derived values
% Eq. (21): kB/kS = -lS^(alphaS-alphaB) * sqrt(2)^(1-alphaB)
cfg.eq21_ratio = -cfg.lS^(cfg.alphaS - cfg.alphaB) * sqrt(2)^(1 - cfg.alphaB);
cfg.kS_eq21    = cfg.kB / cfg.eq21_ratio;
if cfg.use_eq21_kS
    cfg.kS = cfg.kS_eq21;
end
cfg.v_inf   = norm(cfg.u_nav) / cfg.c;   % [EQ] terminal free-flight speed
cfg.tau     = cfg.m / cfg.c;             % [EQ] velocity time constant
cfg.n_steps = round(cfg.T_end / cfg.dt);

% ---------------------------------------------------------- consistency guards
assert(cfg.kS > 0,        'Remark 2 requires kS > 0.');
assert(cfg.kB < 0,        'Remark 2 requires kB < 0.');
assert(cfg.alphaS >= 1,   'Remark 2 requires alphaS >= 1.');
assert(cfg.alphaB < 0,    'Remark 2 requires alphaB < 0.');
assert(cfg.kS*(cfg.alphaS-1) >= 0 && cfg.kB*(cfg.alphaB-1) >= 0, ...
       'Lemma 2 condition k(alpha-1) >= 0 violated.');
assert(cfg.z1 > cfg.lS,   'Table I / text: z1 must be slightly greater than lS.');
assert(cfg.z2 > cfg.z1,   'z2 must exceed z1.');
assert(abs(cfg.T_end/cfg.dt - round(cfg.T_end/cfg.dt)) < 1e-12, ...
       'T_end must be an integer multiple of dt.');
end
