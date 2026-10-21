function [x, stats] = tensegIntegrate(fun, t0, t1, x, opts, postStep)
%TENSEGINTEGRATE  Step-by-step RK4 / Dormand-Prince integrator with a hook.
%
%   [x, stats] = tensegIntegrate(fun, t0, t1, x0, opts, postStep)
%
%   fun      @(t,x) -> dx/dt
%   postStep @(t,x) -> x   called after EVERY accepted step (used to run the
%            Algorithm-1 corrector, as in the paper); [] for none.
%   opts.method  'dopri5' (default, adaptive Dormand-Prince 5(4)) | 'rk4'
%   opts.h       rk4 step / dopri5 initial step   (default (t1-t0)/10)
%   opts.rtol, opts.atol   dopri5 tolerances      (default 1e-10, 1e-10)
%   opts.hmin              dopri5 minimum step    (default 1e-12)
%
%   stats: nSteps, nRejected, hLast.

if nargin < 6, postStep = []; end
method = getOpt(opts, 'method', 'dopri5');
h      = getOpt(opts, 'h', (t1 - t0)/10);
rtol   = getOpt(opts, 'rtol', 1e-10);
atol   = getOpt(opts, 'atol', 1e-10);
hmin   = getOpt(opts, 'hmin', 1e-12);
t = t0;
nSteps = 0;  nRej = 0;
span = t1 - t0;

switch lower(method)
    case 'rk4'
        nsub = max(1, ceil(span/h - 1e-9));
        h = span/nsub;
        for i = 1:nsub
            k1 = fun(t, x);
            k2 = fun(t + h/2, x + h/2*k1);
            k3 = fun(t + h/2, x + h/2*k2);
            k4 = fun(t + h,   x + h*k3);
            x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
            t = t0 + i*h;
            if ~isempty(postStep), x = postStep(t, x); end
            nSteps = nSteps + 1;
        end

    case 'dopri5'
        % Dormand-Prince 5(4) Butcher tableau
        c2=1/5; c3=3/10; c4=4/5; c5=8/9;
        a21=1/5;
        a31=3/40;       a32=9/40;
        a41=44/45;      a42=-56/15;      a43=32/9;
        a51=19372/6561; a52=-25360/2187; a53=64448/6561; a54=-212/729;
        a61=9017/3168;  a62=-355/33;     a63=46732/5247; a64=49/176;  a65=-5103/18656;
        b1=35/384; b3=500/1113; b4=125/192; b5=-2187/6784; b6=11/84;
        e1=71/57600; e3=-71/16695; e4=71/1920; e5=-17253/339200; e6=22/525; e7=-1/40;
        h = min(h, span);
        while t < t1 - 1e-14*max(1,abs(t1))
            h = min(h, t1 - t);
            k1 = fun(t, x);
            k2 = fun(t + c2*h, x + h*(a21*k1));
            k3 = fun(t + c3*h, x + h*(a31*k1 + a32*k2));
            k4 = fun(t + c4*h, x + h*(a41*k1 + a42*k2 + a43*k3));
            k5 = fun(t + c5*h, x + h*(a51*k1 + a52*k2 + a53*k3 + a54*k4));
            k6 = fun(t + h,    x + h*(a61*k1 + a62*k2 + a63*k3 + a64*k4 + a65*k5));
            xn = x + h*(b1*k1 + b3*k3 + b4*k4 + b5*k5 + b6*k6);
            k7 = fun(t + h, xn);
            errv = h*(e1*k1 + e3*k3 + e4*k4 + e5*k5 + e6*k6 + e7*k7);
            sc = atol + rtol*max(abs(x), abs(xn));
            err = sqrt(mean((errv./sc).^2));
            if err <= 1 || h <= hmin
                t = t + h;
                x = xn;
                if ~isempty(postStep), x = postStep(t, x); end
                nSteps = nSteps + 1;
                fac = 0.9*max(err, 1e-10)^(-1/5);
                h = h*min(5, max(0.2, fac));
            else
                nRej = nRej + 1;
                h = max(hmin, h*max(0.2, 0.9*err^(-1/5)));
            end
        end
    otherwise
        error('tensegIntegrate:method', 'Unknown method "%s".', method);
end
stats = struct('nSteps', nSteps, 'nRejected', nRej, 'hLast', h);
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
