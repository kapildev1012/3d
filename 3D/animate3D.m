function animate3D(outA, outB, L, cfg)
%ANIMATE3D  Live 3-D side-by-side animation comparing Case A and Case B.
%
%   ANIMATE3D(outA, outB, L, cfg)
%
%   Shows Case A (no relaxation) and Case B (adaptive relaxation) side-by-side
%   on the COMSOL random rough terrain with solid brown surface.

if ~cfg.showAnimation
    return;
end

hasA = ~isempty(outA);
hasB = ~isempty(outB);
numPanels = hasA + hasB;
if numPanels == 0, return; end

fig = figure('Color', 'w', ...
             'Name', '3-D Tensegrity Lander: Deform & Reform on Random Terrain', ...
             'Position', [60, 60, 580*numPanels, 620]);

ax     = gobjects(numPanels, 1);
titles = cell(numPanels, 1);
outs   = {};

panel = 1;
if hasA
    ax(panel) = subplot(1, numPanels, panel);
    titles{panel} = 'Case A: No String Relaxation (Rigid Baseline)';
    outs{end+1} = outA; %#ok<AGROW>
    panel = panel + 1;
end
if hasB
    ax(panel) = subplot(1, numPanels, panel);
    titles{panel} = 'Case B: Adaptive Relaxation (Deform & Reform)';
    outs{end+1} = outB; %#ok<AGROW>
end

% Determine animation length
Ns = cellfun(@(o) size(o.q, 3), outs);
N  = min(Ns);

if cfg.saveVideo
    try
        if ~isfolder(fileparts(cfg.videoFile))
            mkdir(fileparts(cfg.videoFile));
        end
        vw = VideoWriter(cfg.videoFile, 'MPEG-4');
        vw.FrameRate = 25;
        open(vw);
        doVideo = true;
    catch ME
        warning('animate3D:video', 'Could not open video writer: %s', ME.message);
        doVideo = false;
    end
else
    doVideo = false;
end

for step = 1:cfg.drawEvery:N
    t = (step - 1) * cfg.dt;

    for p = 1:numPanels
        out    = outs{p};
        q      = out.q(:, :, step);
        v      = out.v(:, :, step);
        active = out.detected(:, step);
        traj   = out.centroid(:, 1:step);
        rel    = out.relaxed(:, step);
        vel_c  = mean(v, 2);

        drawFormation3D(ax(p), q, L, cfg, cfg.obstacles, active, traj, t, vel_c, titles{p}, rel);
    end

    drawnow limitrate;

    if doVideo
        writeVideo(vw, getframe(fig));
    end
end

if doVideo
    close(vw);
    fprintf('  Video saved: %s\n', cfg.videoFile);
end

end
