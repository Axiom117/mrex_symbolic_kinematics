function play(poseSeq, scene, varargin)
%PLAY  Play back a solver pose sequence (pure renderer — no kinematics).
%   viz.play(POSESEQ, SCENE)
%   viz.play(POSESEQ, SCENE, 'Fps', FPS)
%   viz.play(POSESEQ, SCENE, 'ShowTrail', TF)
%   viz.play(POSESEQ, SCENE, 'ExtraFrames', EF)
%
%   POSESEQ : struct from solver.buildPoseSequence — per-frame mechanism poses
%   SCENE   : struct from viz.sceneSpec — static scene description
%   EF      : struct array {Label, Base, T} — virtual frames drawn per frame
%             at pose(Base)*T (e.g. the L3 tool frame attached to the FK
%             root); cleared/redrawn with the mechanism like the bodies.
%
%   The renderer never computes kinematics: every frame is redrawn from the
%   pose data in POSESEQ.Frames, already expressed in the sequence's anchor
%   frame ('world' / 'tool' → ground frames fixed at their calibrated
%   bindings, tool visibly traveling; tip trail overlay from POSESEQ.Trail).
%
%   Playback controls (buttons at bottom of figure):
%     ▶ Play / ⏸ Pause    — toggle playback
%     ⏮ Reset             — jump to first frame
%     Speed slider         — adjust fps (1–60)
%     Frame slider         — scrub to any frame
%
%   See also: solver.buildPoseSequence, viz.sceneSpec, viz.animate

    % ---- validate inputs ----
    assert(isstruct(poseSeq) && isstruct(scene), ...
        'viz:play:badInput', ...
        'POSESEQ and SCENE must be structs (see solver.buildPoseSequence / viz.sceneSpec).');

    % ---- parse name-value options ----
    opts = struct('Fps', 10, 'ShowTrail', true, 'ExtraFrames', struct([]));
    if ~isempty(varargin)
        assert(mod(numel(varargin), 2) == 0, ...
            'viz:play:badOptions', 'Name-value options must come in pairs.');
        for oi = 1:2:numel(varargin)
            oName = varargin{oi};
            oVal  = varargin{oi + 1};
            switch lower(oName)
                case 'fps'
                    assert(isnumeric(oVal) && isscalar(oVal) && oVal > 0, ...
                        'viz:play:badFps', 'Fps must be a positive scalar.');
                    opts.Fps = double(oVal);
                case 'showtrail'
                    assert(isscalar(oVal) && (islogical(oVal) || isnumeric(oVal)), ...
                        'viz:play:badShowTrail', 'ShowTrail must be a logical scalar.');
                    opts.ShowTrail = logical(oVal);
                case 'extraframes'
                    assert(isstruct(oVal), 'viz:play:badExtraFrames', ...
                        'ExtraFrames must be a struct array with fields Label, Base, T.');
                    for k = 1:numel(oVal)
                        assert(isfield(oVal(k), 'Label') && isfield(oVal(k), 'Base') ...
                            && isfield(oVal(k), 'T'), ...
                            'viz:play:badExtraFrames', ...
                            'ExtraFrames(%d) must have fields Label, Base, T.', k);
                    end
                    opts.ExtraFrames = oVal;
                otherwise
                    error('viz:play:badOption', 'Unknown option ''%s''.', oName);
            end
        end
    end

    nFrames = numel(poseSeq.Frames);
    assert(nFrames >= 1, 'viz:play:emptySequence', ...
        'PoseSequence contains no frames.');

    useWorldAnchor = any(strcmp(poseSeq.Anchor, {'world', 'tool'}));
    showTrail = opts.ShowTrail && useWorldAnchor;

    % ---- figure setup ----
    fig = figure('Name', sprintf('Animation: %s', poseSeq.MechName), 'Color', 'w', ...
        'NumberTitle', 'off');
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
    title(ax, sprintf('%s  —  frame 1 / %d', poseSeq.MechName, nFrames), ...
        'Interpreter', 'none');

    % world triad
    core.VizHelpers.triad(ax, eye(4), scene.L * 1.4, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % world-anchor mode: static ground-frame markers at their calibrated
    % world bindings (drawn once; untagged so renderFrame keeps them)
    if useWorldAnchor
        for g = 1:numel(poseSeq.GroundFrames)
            core.VizHelpers.triad(ax, poseSeq.Tbind{g}, scene.L * 1.2, 2.0, '-');
            text(ax, poseSeq.Tbind{g}(1, 4), poseSeq.Tbind{g}(2, 4), ...
                poseSeq.Tbind{g}(3, 4), ...
                ['  ' poseSeq.GroundFrames{g}], 'FontSize', 8, 'Color', [.35 .35 .35]);
        end
    end

    % ---- state shared between callbacks ----
    state = struct();
    state.seq        = poseSeq;
    state.scene      = scene;
    state.nFrames    = nFrames;
    state.varNames   = poseSeq.VarNames;
    state.trajectory = poseSeq.Q;
    state.ax         = ax;
    state.fig        = fig;
    state.currentFrame = 1;
    state.playing    = false;
    state.fps        = opts.Fps;
    state.timer      = [];
    state.useWorldAnchor = useWorldAnchor;
    state.showTrail      = showTrail;
    state.extraFrames    = opts.ExtraFrames;

    % ---- store state in figure for callbacks ----
    fig.UserData = state;

    % ---- draw first frame ----
    renderCurrentFrame(fig);

    % ---- playback controls ----
    btnW = 50; btnH = 22; margin = 8;
    yPos = 12;

    % Play/Pause button
    uicontrol('Style', 'pushbutton', 'String', '▶ Play', ...
        'Position', [margin, yPos, btnW+10, btnH], ...
        'Callback', @(src, ~) togglePlay(src, fig), ...
        'Parent', fig, 'Tag', 'btnPlay');

    % Reset button
    uicontrol('Style', 'pushbutton', 'String', '⏮ Reset', ...
        'Position', [margin + btnW + 30, yPos, btnW+10, btnH], ...
        'Callback', @(src, ~) resetAnimation(src, fig), ...
        'Parent', fig);

    % Speed label + slider
    uicontrol('Style', 'text', 'String', 'Speed:', ...
        'Position', [margin + 2*btnW + 70, yPos+2, 40, 18], ...
        'Parent', fig, 'BackgroundColor', 'w');
    uicontrol('Style', 'slider', 'Min', 1, 'Max', 60, 'Value', opts.Fps, ...
        'Position', [margin + 2*btnW + 110, yPos, 120, 20], ...
        'Callback', @(src, ~) setSpeed(src, fig), ...
        'Parent', fig, 'Tag', 'sliderSpeed');

    % Frame slider
    uicontrol('Style', 'text', 'String', 'Frame:', ...
        'Position', [margin + 2*btnW + 240, yPos+2, 40, 18], ...
        'Parent', fig, 'BackgroundColor', 'w');
    uicontrol('Style', 'slider', 'Min', 1, 'Max', nFrames, 'Value', 1, ...
        'Position', [margin + 2*btnW + 280, yPos, 200, 20], ...
        'SliderStep', [1/(nFrames-1) 10/(nFrames-1)], ...
        'Callback', @(src, ~) scrubFrame(src, fig), ...
        'Parent', fig, 'Tag', 'sliderFrame');

    rotate3d(ax, 'on');
    fprintf('Animation ready. %d frames at %d fps. Use ▶ Play to start.\n', ...
        nFrames, opts.Fps);
end

%% ---- callback helpers ----

function renderCurrentFrame(fig)
    state = fig.UserData;
    i = state.currentFrame;
    ax  = state.ax;
    seq = state.seq;
    scene = state.scene;

    % pose data for this frame (already anchored — no kinematics here)
    poses = seq.Frames{i};

    % per-frame joint values (for renderFrame's detail mode / labels)
    jvMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
    for j = 1:numel(state.varNames)
        jvMap(state.varNames{j}) = state.trajectory(i, j);
    end

    % render (bodies only, no triads/mates in animation mode)
    renderFrame(ax, poses, scene.Instances, scene.ConnectionInfo, scene.LibDir, ...
        scene.RepoRoot, scene.MechName, scene.NInst, scene.NConns, ...
        scene.L, jvMap, false, scene.GeomCache);

    % ---- virtual frames attached to sequence frames (e.g. the tool frame) ----
    if ~isempty(state.extraFrames)
        preKids = allchild(ax);
        for ef = 1:numel(state.extraFrames)
            E = state.extraFrames(ef);
            Tf = poses(E.Base) * E.T;
            core.VizHelpers.triad(ax, Tf, scene.L * 1.1, 2.0, '-');
            text(ax, Tf(1, 4), Tf(2, 4), Tf(3, 4), ['  ' E.Label], ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.55 0.10 0.55]);
        end
        postKids = allchild(ax);
        newKids = postKids(~ismember(postKids, preKids));
        for hk = 1:numel(newKids)
            set(newKids(hk), 'Tag', 'vizDynamic');
        end
    end

    if state.useWorldAnchor && state.showTrail
        drawTrail(ax, seq.Trail, i);
    end

    % update title
    title(ax, sprintf('%s  —  frame %d / %d', scene.MechName, i, state.nFrames), ...
        'Interpreter', 'none');

    % sync frame slider
    hSlider = findobj(state.fig, 'Tag', 'sliderFrame');
    if ~isempty(hSlider); set(hSlider, 'Value', i); end

    drawnow;
end

function togglePlay(~, fig)
    state = fig.UserData;
    if state.playing
        % pause
        state.playing = false;
        if ~isempty(state.timer)
            stop(state.timer);
            delete(state.timer);
            state.timer = [];
        end
        hBtn = findobj(fig, 'Tag', 'btnPlay');
        set(hBtn, 'String', '▶ Play');
    else
        % play
        state.playing = true;
        hBtn = findobj(fig, 'Tag', 'btnPlay');
        set(hBtn, 'String', '⏸ Pause');
        period = 1.0 / max(state.fps, 1);
        state.timer = timer('ExecutionMode', 'fixedRate', ...
            'Period', period, ...
            'TimerFcn', @(~, ~) advanceFrame(fig), ...
            'ErrorFcn', @(~, ~) disp('Timer error'));
        start(state.timer);
    end
    fig.UserData = state;
end

function advanceFrame(fig)
    state = fig.UserData;
    if ~state.playing; return; end
    state.currentFrame = state.currentFrame + 1;
    if state.currentFrame > state.nFrames
        state.currentFrame = 1;  % loop
    end
    fig.UserData = state;
    renderCurrentFrame(fig);
end

function resetAnimation(~, fig)
    state = fig.UserData;
    wasPlaying = state.playing;
    if wasPlaying
        togglePlay([], fig);  % pause first
        state = fig.UserData; % refresh after toggle
    end
    state.currentFrame = 1;
    fig.UserData = state;
    renderCurrentFrame(fig);
    if wasPlaying
        togglePlay([], fig);  % resume
    end
end

function setSpeed(src, fig)
    state = fig.UserData;
    newFps = round(get(src, 'Value'));
    state.fps = newFps;
    % restart timer with new period if playing
    if state.playing
        if ~isempty(state.timer)
            stop(state.timer);
            delete(state.timer);
        end
        period = 1.0 / max(newFps, 1);
        state.timer = timer('ExecutionMode', 'fixedRate', ...
            'Period', period, ...
            'TimerFcn', @(~, ~) advanceFrame(fig), ...
            'ErrorFcn', @(~, ~) disp('Timer error'));
        start(state.timer);
    end
    fig.UserData = state;
end

function scrubFrame(src, fig)
    state = fig.UserData;
    state.currentFrame = round(get(src, 'Value'));
    fig.UserData = state;
    renderCurrentFrame(fig);
end

function drawTrail(ax, trailPos, iFrame)
%DRAWTRAIL  Tip world path overlay: full path, traversed segment, marker.
    plot3(ax, trailPos(1, :), trailPos(2, :), trailPos(3, :), '--', ...
        'Color', [0.72 0.72 0.72], 'LineWidth', 1.0, 'Tag', 'vizDynamic');
    plot3(ax, trailPos(1, 1:iFrame), trailPos(2, 1:iFrame), ...
        trailPos(3, 1:iFrame), '-', 'Color', [0.85 0.45 0.10], ...
        'LineWidth', 2.0, 'Tag', 'vizDynamic');
    plot3(ax, trailPos(1, iFrame), trailPos(2, iFrame), trailPos(3, iFrame), ...
        'o', 'MarkerSize', 7, 'MarkerEdgeColor', [0.3 0.15 0.0], ...
        'MarkerFaceColor', [0.95 0.55 0.15], 'Tag', 'vizDynamic');
end
