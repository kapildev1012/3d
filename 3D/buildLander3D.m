function L = buildLander3D(cfg)
%BUILDLANDER3D  Build the 12-node, 6-bar, 24-string tensegrity lander.
%
%   L = BUILDLANDER3D(cfg) returns a struct describing the lander topology
%   and prestressed geometry.  The node coordinates, bar connectivity, and
%   string connectivity are taken verbatim from the user's existing lander
%   model (double-layer prism rotated about X and shifted upward in Z).
%
%   Output struct L:
%     L.n          12  (number of nodes)
%     L.nB          6  (number of bars)
%     L.nS         24  (number of strings)
%     L.q0         3 x 12  nominal node positions [m]
%     L.bars       6 x 2   bar node-index pairs
%     L.strings   24 x 2   string node-index pairs
%     L.members   30 x 2   all members (bars first, then strings)
%     L.isString  30 x 1   logical (true = string, false = bar)
%     L.k         30 x 1   gain per member
%     L.alpha     30 x 1   exponent per member
%     L.l0        30 x 1   nominal (rest) member lengths
%     L.inc       12 x 1 cell  members incident to each node
%     L.inc_other 12 x 1 cell  the other endpoint of each incident member
%
%   See also CONFIG3D, SIMULATE3D.

% ==================================================================
% 1. NODE COORDINATES  (user's existing lander model)
% ==================================================================
Lscale = 1;   % characteristic length

th1 = -atan(1/2);

N = [ ...
     Lscale/4    0           0;
     Lscale/4    0           Lscale;
    -Lscale/4    0           0;
    -Lscale/4    0           Lscale;
     0          -Lscale/2    3*Lscale/4;
     0           Lscale/2    3*Lscale/4;
     0          -Lscale/2    Lscale/4;
     0           Lscale/2    Lscale/4;
     Lscale/2   -Lscale/4    Lscale/2;
    -Lscale/2   -Lscale/4    Lscale/2;
     Lscale/2    Lscale/4    Lscale/2;
    -Lscale/2    Lscale/4    Lscale/2]';   % 3 x 12

% Rotate about X axis by th1
Rx = [1       0           0;
      0       cos(th1)   -sin(th1);
      0       sin(th1)    cos(th1)];
N = Rx * N;

% Shift upward in Z
N(3,:) = N(3,:) + 1;

% ==================================================================
% 2. BAR CONNECTIVITY  (6 bars)
% ==================================================================
C_b_in = [ ...
    1  2;
    3  4;
    5  6;
    7  8;
    9 10;
   11 12];

% ==================================================================
% 3. STRING CONNECTIVITY  (24 strings)
% ==================================================================
C_s_in = [ ...
     2  5;
     2  6;
     2  9;
     2 11;
     4  5;
     4  6;
     4 10;
     4 12;
     1  7;
     1  8;
     1  9;
     1 11;
     3  7;
     3  8;
     3 10;
     3 16;
     5  9;
     5 10;
     7  9;
     7 10;
     6 11;
     6 12;
     8 11;
     8 12];

% Fix: string 16 should connect nodes 3 and 12 (not 3 and 16)
C_s_in(16,:) = [3 12];

nn = size(N, 2);   % 12 nodes
nB = size(C_b_in, 1);   % 6 bars
nS = size(C_s_in, 1);   % 24 strings

% ==================================================================
% 4. COMBINED MEMBER LIST  (bars first, then strings)
% ==================================================================
members  = [C_b_in; C_s_in];        % 30 x 2
M        = nB + nS;                  % 30
isString = [false(nB,1); true(nS,1)];

% ==================================================================
% 5. NOMINAL MEMBER LENGTHS
% ==================================================================
l0 = zeros(M, 1);
for mm = 1:M
    i = members(mm, 1);
    j = members(mm, 2);
    l0(mm) = norm(N(:,i) - N(:,j));
end

% ==================================================================
% 6. PER-MEMBER GAINS AND EXPONENTS
% ==================================================================
k     = zeros(M, 1);
alpha = zeros(M, 1);
k(~isString)     = cfg.kB;       alpha(~isString)     = cfg.alphaB;
k(isString)      = cfg.kS;       alpha(isString)      = cfg.alphaS;

% ==================================================================
% 7. INCIDENCE LISTS  (which members touch each node)
% ==================================================================
inc       = cell(nn, 1);
inc_other = cell(nn, 1);
for i = 1:nn
    rows = find(members(:,1) == i | members(:,2) == i);
    oth  = zeros(numel(rows), 1);
    for a = 1:numel(rows)
        if members(rows(a), 1) == i
            oth(a) = members(rows(a), 2);
        else
            oth(a) = members(rows(a), 1);
        end
    end
    inc{i}       = rows;
    inc_other{i} = oth;
end

% ==================================================================
% 8. ASSEMBLE OUTPUT STRUCT
% ==================================================================
L = struct( ...
    'n',         nn, ...
    'nB',        nB, ...
    'nS',        nS, ...
    'q0',        N, ...            % 3 x 12
    'bars',      C_b_in, ...       % 6 x 2
    'strings',   C_s_in, ...       % 24 x 2
    'members',   members, ...      % 30 x 2
    'isString',  isString, ...     % 30 x 1
    'k',         k, ...            % 30 x 1
    'alpha',     alpha, ...        % 30 x 1
    'l0',        l0, ...           % 30 x 1
    'inc',       {inc}, ...
    'inc_other', {inc_other});

% ==================================================================
% 9. SANITY CHECKS
% ==================================================================
assert(L.n  == 12, 'Expected 12 nodes, got %d.', L.n);
assert(L.nB ==  6, 'Expected 6 bars, got %d.',   L.nB);
assert(L.nS == 24, 'Expected 24 strings, got %d.', L.nS);
assert(all(l0 > 0), 'All nominal member lengths must be positive.');
assert(all(members(:) >= 1 & members(:) <= nn), ...
    'Member indices must be in [1, %d].', nn);

if cfg.verbose
    fprintf('\n=== LANDER 3-D STRUCTURE ===\n');
    fprintf('  Nodes   : %d\n', nn);
    fprintf('  Bars    : %d\n', nB);
    fprintf('  Strings : %d\n', nS);
    fprintf('  Bar lengths   : [%.4f, %.4f] m\n', min(l0(1:nB)), max(l0(1:nB)));
    fprintf('  String lengths: [%.4f, %.4f] m\n', min(l0(nB+1:end)), max(l0(nB+1:end)));
    fprintf('  Centroid      : [%.4f, %.4f, %.4f] m\n', mean(N,2));
    fprintf('===========================\n\n');
end

end
