function out = tensegSimulate(S, q0, dq0, controller, fext, sim)
%TENSEGSIMULATE  Closed-loop DAE simulation with Algorithm-1 correction.
%
%   out = tensegSimulate(S, q0, dq0, controller, fext, sim)
%
%   controller : either a fixed actuation struct (tensegControl) or a
%                function handle  u = controller(t, q, dq)  evaluated at
%                every control step and held constant (zero-order hold).
%   fext       : [] | 3n x 1 vector | handle f = fext(t, q, dq)
%                (external nodal forces; gravity is already in S.G)
%   sim fields :
%     T            final time [s]                         (default 1)
%     dtControl    control / logging period [s]           (default 0.01)
%     integrator   struct for tensegIntegrate             (default dopri5,
%                  rtol = atol = 1e-10)
%     corrector    struct for daeCorrector plus .enabled  (default enabled)
%     verbose      print progress                         (default false)
%
%   State x = [q; dq; Wf] with Wf_dot = f'dq (Eq. 49).
%
%   Energy bookkeeping: the invariant used by the corrector is
%   E(q,dq;u) - E0 - Wf = 0. When the controller changes u, the string
%   potential jumps by dV = E(q,dq;u_new) - E(q,dq;u_old) at fixed state;
%   that actuator work is booked in out.Wact and E0 is shifted by dV, so the
%   corrector enforces exact energy balance inside every hold interval and
%   E - E_init - Wf - Wact = 0 holds over the whole run.
%
%   out fields (one row/column per control step, k = 1..K+1):
%     t, q (3n x K+1), dq, sigma (ns x K+1), u (cell), barErr (nb x K+1),
%     Rnorm, Vnorm, Eres (E - E_init - Wf - Wact), E, Wf, Wact,
%     corrIters (max iterations per interval), nSteps.

if nargin < 6, sim = struct(); end
T   = getOpt(sim, 'T', 1);
dtc = getOpt(sim, 'dtControl', 0.01);
integ = getOpt(sim, 'integrator', struct('method','dopri5','rtol',1e-10,'atol',1e-10));
corr  = getOpt(sim, 'corrector', struct());
corrOn = getOpt(corr, 'enabled', true);
verbose = getOpt(sim, 'verbose', false);

nq = S.nq;
K = round(T/dtc);
fextFun = makeFext(fext, nq);
isCtrlFun = isa(controller, 'function_handle');

q = q0(:);  dq = dq0(:);  Wf = 0;  Wact = 0;
if isCtrlFun, u = controller(0, q, dq); else, u = controller; end
Einit = tensegEnergy(S, q, dq, u);
E0 = Einit;

out.t = (0:K)*dtc;
out.q = zeros(nq, K+1);  out.dq = zeros(nq, K+1);
out.sigma = zeros(S.ns, K+1);  out.barErr = zeros(S.nb, K+1);
out.Rnorm = zeros(1,K+1);  out.Vnorm = zeros(1,K+1);
out.Eres = zeros(1,K+1);  out.E = zeros(1,K+1);
out.Wf = zeros(1,K+1);  out.Wact = zeros(1,K+1);
out.corrIters = zeros(1,K+1);  out.nSteps = zeros(1,K+1);
out.u = cell(1,K+1);
logStep(1, 0, 0);

maxIt = 0;
for k = 1:K
    t0 = (k-1)*dtc;
    if isCtrlFun && k > 1
        uNew = controller(t0, q, dq);
        Eold = tensegEnergy(S, q, dq, u);
        Enew = tensegEnergy(S, q, dq, uNew);
        Wact = Wact + (Enew - Eold);
        E0 = E0 + (Enew - Eold);
        u = uNew;
    end
    rhs = @(t, x) stateDeriv(S, t, x, u, fextFun);
    maxIt = 0;
    if corrOn
        post = @(t, x) correct(t, x);
    else
        post = [];
    end
    [x, st] = tensegIntegrate(rhs, t0, t0 + dtc, [q; dq; Wf], integ, post);
    q = x(1:nq);  dq = x(nq+1:2*nq);  Wf = x(end);
    logStep(k+1, maxIt, st.nSteps);
    if ~all(isfinite(q))
        warning('tensegSimulate:diverged', 'State diverged at t = %.4f s.', t0);
        out = truncate(out, k+1);
        return;
    end
    if verbose && mod(k, max(1, round(K/10))) == 0
        fprintf('  t = %6.3f s | ||R|| = %.2e | E res = %.2e\n', ...
            out.t(k+1), out.Rnorm(k+1), out.Eres(k+1));
    end
end

    function xc = correct(~, x)
        [qc, dqc, Wc, info] = daeCorrector(S, x(1:nq), x(nq+1:2*nq), x(end), u, E0, corr);
        maxIt = max(maxIt, info.iters);
        xc = [qc; dqc; Wc];
    end

    function logStep(i, iters, nsteps)
        cs = tensegConstraints(S, q, dq);
        stg = tensegStrings(S, q, dq, u);
        E = tensegEnergy(S, q, dq, u);
        out.q(:,i) = q;  out.dq(:,i) = dq;
        out.sigma(:,i) = stg.sigma;
        out.barErr(:,i) = cs.barLenErr;
        out.Rnorm(i) = norm(cs.R);  out.Vnorm(i) = norm(cs.Rq*dq);
        out.E(i) = E;  out.Wf(i) = Wf;  out.Wact(i) = Wact;
        out.Eres(i) = E - Einit - Wf - Wact;
        out.corrIters(i) = iters;  out.nSteps(i) = nsteps;
        out.u{i} = u;
    end
end

function dx = stateDeriv(S, t, x, u, fextFun)
nq = S.nq;
q = x(1:nq);  dq = x(nq+1:2*nq);
[qdd, aux] = tensegDaeDynamics(S, q, dq, u, fextFun(t, q, dq));
dx = [dq; qdd; aux.Wf_dot];
end

function f = makeFext(fext, nq)
if isempty(fext)
    f = @(t, q, dq) zeros(nq,1);
elseif isa(fext, 'function_handle')
    f = fext;
else
    v = fext(:);
    f = @(t, q, dq) v;
end
end

function out = truncate(out, k)
fn = fieldnames(out);
for i = 1:numel(fn)
    v = out.(fn{i});
    if iscell(v) || size(v,2) > 1, out.(fn{i}) = v(:, 1:k); end
end
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
