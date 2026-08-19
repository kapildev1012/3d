function S = build_formation(cfg)
%BUILD_FORMATION  Tiled virtual tensegrity structure of the paper's Sec. V.
%
%   S = BUILD_FORMATION(cfg) builds the "six interconnected squares (two rows
%   and three columns)" formation of 12 agents (Sec. V, Figs. 2, 5, 6).
%
%   Topology of one square tile (Fig. 2): 4 perimeter STRINGS + 2 diagonal BARS.
%   The tiled structure is the union of the members of all tiles, so a string
%   edge shared by two tiles appears TWICE (multiplicity 2).  See ASSUMPTIONS.md
%   item A2: this is what makes the uniform grid an exact self-stress state and
%   hence what makes Eq. (21) the equilibrium condition of the tiled formation.
%
%   Output struct S:
%     S.n         number of nodes (agents)
%     S.q0        d x n nominal (self-stress) node positions q*        [Eq. (5)]
%     S.members   M x 2 node index pairs (i,j), one row per member
%     S.type      M x 1 logical, true = string, false = bar
%     S.k         M x 1 gain      k_ij                                [Eq. (11)]
%     S.alpha     M x 1 exponent  alpha_ij                            [Eq. (11)]
%     S.nS,S.nB   number of string / bar members
%     S.CS,S.CB   connectivity matrices                               [Eq. (6)]
%     S.A         weighted adjacency (member multiplicity)
%     S.inc       n x 1 cell, inc{i} = row indices of members incident to i
%     S.inc_other n x 1 cell, the other endpoint for each of those members
%     S.grid      struct with nx, ny, and node index map
%
%   See also PAPER_CONFIG, TENSEGRITY_ANALYSIS, EDGE_FORCE.

ntr = cfg.tiles(1);            % tile rows
ntc = cfg.tiles(2);            % tile columns
nx  = ntc + 1;                 % nodes per row
ny  = ntr + 1;                 % nodes per column
n   = nx * ny;                 % 4 x 3 = 12 agents  [TXT: "12 agents"]

% ---- nominal node positions: uniform grid, spacing lS, centred at origin ----
% [FIG] Fig. 5(a)/6(a): x in {-22.5,-7.5,7.5,22.5}, y in {-15,0,15}
q0  = zeros(cfg.d, n);
idx = zeros(ny, nx);
for r = 1:ny
    for c = 1:nx
        k        = (r-1)*nx + c;
        idx(r,c) = k;
        q0(1,k)  = (c - (nx+1)/2) * cfg.lS;
        q0(2,k)  = (r - (ny+1)/2) * cfg.lS;
    end
end

% ---- member list: union over the six tiles --------------------------------
members = zeros(0,2);
isStr   = false(0,1);
for tr = 1:ntr
    for tc = 1:ntc
        bl = idx(tr  , tc  );      % bottom-left
        br = idx(tr  , tc+1);      % bottom-right
        tl = idx(tr+1, tc  );      % top-left
        tRt = idx(tr+1, tc+1);     % top-right
        % 4 perimeter strings (Fig. 2, red)
        members = [members; bl br; br tRt; tRt tl; tl bl];   %#ok<AGROW>
        isStr   = [isStr;   true;  true;   true;   true  ];  %#ok<AGROW>
        % 2 diagonal bars (Fig. 2, blue)
        members = [members; bl tRt; br tl];                  %#ok<AGROW>
        isStr   = [isStr;   false; false];                   %#ok<AGROW>
    end
end
M = size(members,1);

% ---- per-member gains, Eq. (11) ------------------------------------------
k     = zeros(M,1);
alpha = zeros(M,1);
k(isStr)      = cfg.kS;      alpha(isStr)      = cfg.alphaS;
k(~isStr)     = cfg.kB;      alpha(~isStr)     = cfg.alphaB;

% ---- connectivity matrices, Eq. (6): -1 start node, +1 end node ----------
bars    = find(~isStr);
strings = find(isStr);
CB = zeros(numel(bars), n);
CS = zeros(numel(strings), n);
for a = 1:numel(bars)
    CB(a, members(bars(a),1)) = -1;
    CB(a, members(bars(a),2)) = +1;
end
for a = 1:numel(strings)
    CS(a, members(strings(a),1)) = -1;
    CS(a, members(strings(a),2)) = +1;
end

% ---- weighted adjacency (multiplicity) and incidence lists ---------------
A = zeros(n,n);
for mm = 1:M
    i = members(mm,1); j = members(mm,2);
    A(i,j) = A(i,j) + 1;
    A(j,i) = A(j,i) + 1;
end
inc       = cell(n,1);
inc_other = cell(n,1);
for i = 1:n
    rows = find(members(:,1) == i | members(:,2) == i);
    oth  = zeros(numel(rows),1);
    for a = 1:numel(rows)
        if members(rows(a),1) == i
            oth(a) = members(rows(a),2);
        else
            oth(a) = members(rows(a),1);
        end
    end
    inc{i}       = rows;
    inc_other{i} = oth;
end

S = struct('n', n, 'q0', q0, 'members', members, 'type', isStr, ...
           'k', k, 'alpha', alpha, 'nS', numel(strings), 'nB', numel(bars), ...
           'CS', CS, 'CB', CB, 'A', A, 'inc', {inc}, 'inc_other', {inc_other}, ...
           'grid', struct('nx', nx, 'ny', ny, 'idx', idx), ...
           'strings', strings, 'bars', bars);

% ---- structural sanity ---------------------------------------------------
assert(S.nS == 4*ntr*ntc, 'Expected %d strings, got %d.', 4*ntr*ntc, S.nS);
assert(S.nB == 2*ntr*ntc, 'Expected %d bars, got %d.',    2*ntr*ntc, S.nB);
L = diag(sum(S.A,2)) - S.A;                                    % Eq. (1)
assert(rank(L) == n-1, 'Formation graph is not connected (rank(L) = %d).', rank(L));
end
