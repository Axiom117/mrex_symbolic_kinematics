function animate(dslYaml, jointTrajectory, varNames, fps, varargin)
%ANIMATE  Animate a modular mechanism through a sequence of joint configurations.
%   viz.animate(DSLYAML, JOINTTRAJECTORY, VARNAMES) builds the mechanism from DSL
%   once, then renders one frame per row of JOINTTRAJECTORY, updating the figure
%   in-place.  VARNAMES is a cell array of canonical joint variable names (e.g.
%   {'joint_AB.q', 'joint_BD.q', ...}) matching the columns of JOINTTRAJECTORY.
%
%   viz.animate(DSLYAML, JOINTTRAJECTORY, VARNAMES, FPS) sets the playback speed
%   (default 10 fps).
%
%   viz.animate(..., 'Anchor', MODE) selects the display reference frame:
%     'root'  (default) — tool-rooted FK poses rendered as-is (root pinned).
%     'world' — re-express every pose in the world frame via
%               E(q) = T_bind * inv(T_tip<-ground_ref(q)), so that for L3
%               world-closed-loop mechanisms (e.g. m-rex-3t1r) the Manipulator
%               ground frames stay fixed at their calibrated world bindings
%               while the tool visibly travels.  Requires frames with
%               semantic_tag == 'ground'.  T_bind is taken from the zero-pose
%               FK — the same value as solver.ClosureSolver's auto-calibrated
%               world binding (see test_closure_solver_m_rex_3t1r check [5]).
%               Extra ground frames are reported as a branch-consistency
%               diagnostic (max gap printed at setup).
%
%   viz.animate(..., 'ShowTrail', TF) toggles the tip world-path overlay
%   (full path as a faint dashed line, traversed segment highlighted, current
%   tip marked).  Only meaningful in 'world' anchor mode; default true.
%
%   Playback controls (buttons at bottom of figure):
%     ▶ Play / ⏸ Pause    — toggle playback
%     ⏮ Reset             — jump to first frame
%     Speed slider         — adjust fps (1–60)
%     Frame slider         — scrub to any frame
%
%   Example — animate a closed-loop sweep of the parallelogram:
%       nFrames = 60;
%       theta = linspace(0, pi/3, nFrames);
%       Q = [theta; -theta; theta; -theta]';  % nFrames×4
%       varNames = {'joint_AB.q','joint_BD.q','joint_CD.q','joint_AC.q'};
%       viz.animate('../../specs/dsl/cases/single-closed-loop/robot_description.yaml', Q, varNames);
%
%   See also: +viz/mechanism, +ir/Expander.evaluateNumericDirect

    if nargin < 1 || isempty(dslYaml)
        error('viz:animate:usage', ...
            'Usage: viz.animate(dslYaml, jointTrajectory, varNames[, fps][, Name, Value...])');
    end
    if nargin < 2; jointTrajectory = []; end
    if nargin < 3; varNames = {}; end
    if nargin < 4 || isempty(fps); fps = 10; end

    % ---- parse name-value options ----
    opts = struct('Anchor', 'root', 'ShowTrail', true);
    if ~isempty(varargin)
        assert(mod(numel(varargin), 2) == 0, ...
            'viz:animate:badOptions', 'Name-value options must come in pairs.');
        for oi = 1:2:numel(varargin)
            oName = varargin{oi};
            oVal  = varargin{oi + 1};
            switch lower(oName)
                case 'anchor'
                    assert(ischar(oVal) || (isstring(oVal) && isscalar(oVal)), ...
                        'viz:animate:badAnchor', 'Anchor must be a char value.');
                    assert(any(strcmp(oVal, {'root', 'world'})), ...
                        'viz:animate:badAnchor', ...
                        'Anchor must be ''root'' or ''world'', got ''%s''.', char(oVal));
                    opts.Anchor = char(oVal);
                case 'showtrail'
                    assert(isscalar(oVal) && (islogical(oVal) || isnumeric(oVal)), ...
                        'viz:animate:badShowTrail', 'ShowTrail must be a logical scalar.');
                    opts.ShowTrail = logical(oVal);
                otherwise
                    error('viz:animate:badOption', 'Unknown option ''%s''.', oName);
            end
        end
    end

    % ---- path setup ----
    here = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(fileparts(here));

    % ---- build pipeline once (expensive) ----
    fprintf('Building symbolic pipeline (one-time) ... ');
    expander = ir.Expander(dslYaml);
    mechName = expander.MechName;
    inst     = expander.Instances;
    connInfo = expander.ConnectionInfo;
    libDir   = expander.LibDir;
    nInst    = numel(inst);
    nConns   = numel(connInfo);
    nJoints  = expander.JointVarMap.Count;
    jvKeys   = keys(expander.JointVarMap);
    fprintf('OK (%d instances, %d joints)\n', nInst, nJoints);

    % ---- pre-compile symbolic poses to numeric function handles (one-time cost) ----
    expander.compilePoseFunctions();

    % ---- validate trajectory ----
    if isempty(jointTrajectory) || isempty(varNames)
        error('viz:animate:noTrajectory', ...
            'jointTrajectory and varNames are required for animation.');
    end
    [nFrames, nCols] = size(jointTrajectory);
    assert(nCols == numel(varNames), ...
        'viz:animate:trajectoryMismatch', ...
        'jointTrajectory has %d columns but varNames has %d entries.', ...
        nCols, numel(varNames));
    for i = 1:numel(varNames)
        assert(isKey(expander.JointVarMap, varNames{i}), ...
            'viz:animate:varNotFound', ...
            'Joint variable "%s" not found in mechanism.', varNames{i});
    end

    % ---- characteristic scale (from zero pose) ----
    zeroPoses = expander.evaluateNumericDirect(containers.Map);
    maxr = 1; ks = keys(zeroPoses);
    for k = 1:numel(ks); P = zeroPoses(ks{k}); maxr = max(maxr, norm(P(1:3,4))); end
    L = max(4, 0.20 * maxr);

    % ---- pre-load all geometry (avoid per-frame disk I/O) ----
    geomCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    for i = 1:nInst
        for k = 1:numel(inst(i).bodies)
            gPath = inst(i).bodies{k}.geometry;
            if ~isempty(gPath) && ~isKey(geomCache, gPath)
                geomAbs = core.PathUtils.resolveGeometryPath(gPath, libDir, repoRoot);
                if ~isempty(geomAbs)
                    g = core.VizHelpers.importGeometry(geomAbs);
                    if ~isempty(g)
                        geomCache(gPath) = g;
                    end
                end
            end
        end
    end
    fprintf('  Geometry cache: %d unique STL files loaded\n', geomCache.Count);

    % ---- collect frame names actually needed for rendering (bodies + joints only) ----
    neededFrames = {};
    for i = 1:nInst
        for k = 1:numel(inst(i).bodies)
            neededFrames{end+1} = inst(i).bodies{k}.node; %#ok<AGROW>
        end
        for k = 1:numel(inst(i).joints)
            neededFrames{end+1} = inst(i).joints{k}.node; %#ok<AGROW>
        end
    end
    neededFrames = unique(neededFrames);
    fprintf('  Evaluating %d/%d frames per animation step\n', numel(neededFrames), numel(keys(expander.Poses)));

    % ---- world-anchor setup (L3 world-closed-loop rendering) ----
    % In 'world' mode every tool-rooted FK pose is re-expressed in the world
    % frame: E(q) = T_bind * inv(T_tip<-ground_ref(q)).  T_bind values come
    % from the zero-pose FK (identical to solver.ClosureSolver's auto-
    % calibrated world bindings, see test_closure_solver_m_rex_3t1r [5]).
    groundNodes = {};
    for i = 1:nInst
        for k = 1:numel(inst(i).frames)
            f = inst(i).frames{k};
            if strcmp(f.semantic_tag, 'ground')
                groundNodes{end+1} = f.node; %#ok<AGROW>
            end
        end
    end
    useWorldAnchor = strcmp(opts.Anchor, 'world');
    if useWorldAnchor && isempty(groundNodes)
        error('viz:animate:noGround', ...
            ['Anchor = ''world'' requires at least one frame with ' ...
             'semantic_tag == ''ground''; mechanism "%s" has none.'], mechName);
    end
    evalFrames = neededFrames;
    Tbind = {};
    trailPos = [];
    branchGapMax = 0;
    if useWorldAnchor
        evalFrames = unique([neededFrames(:).', groundNodes(:).']);
        Tbind = cell(1, numel(groundNodes));
        for g = 1:numel(groundNodes)
            Tbind{g} = zeroPoses(groundNodes{g});
        end
        % pre-compute the tip world path E(t) and the branch diagnostic
        fprintf('  World anchor: %d ground frame(s); pre-computing tip path ... ', ...
            numel(groundNodes));
        trailPos = zeros(3, nFrames);
        for k = 1:nFrames
            jvMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
            for j = 1:numel(varNames)
                jvMap(varNames{j}) = jointTrajectory(k, j);
            end
            gp = expander.evaluateNumericDirect(jvMap, groundNodes);
            E = Tbind{1} * core.RigidBodyMath.invT(gp(groundNodes{1}));
            trailPos(:, k) = E(1:3, 4);
            for g = 2:numel(groundNodes)
                Eg = Tbind{g} * core.RigidBodyMath.invT(gp(groundNodes{g}));
                branchGapMax = max(branchGapMax, norm(Eg - E, 'fro'));
            end
        end
        fprintf('OK\n  Tip path: %d points; max ground-branch gap = %.3e\n', ...
            nFrames, branchGapMax);
    end

    % ---- figure setup ----
    fig = figure('Name', sprintf('Animation: %s', mechName), 'Color', 'w', ...
        'NumberTitle', 'off');
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
    title(ax, sprintf('%s  —  frame 1 / %d', mechName, nFrames), 'Interpreter', 'none');

    % world triad
    core.VizHelpers.triad(ax, eye(4), L * 1.4, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % world-anchor mode: static ground-frame markers at their calibrated
    % world bindings (drawn once; untagged so renderFrame keeps them)
    if useWorldAnchor
        for g = 1:numel(groundNodes)
            core.VizHelpers.triad(ax, Tbind{g}, L * 1.2, 2.0, '-');
            text(ax, Tbind{g}(1, 4), Tbind{g}(2, 4), Tbind{g}(3, 4), ...
                ['  ' groundNodes{g}], 'FontSize', 8, 'Color', [.35 .35 .35]);
        end
    end

    % ---- state shared between callbacks ----
    state = struct();
    state.expander   = expander;
    state.inst       = inst;
    state.connInfo   = connInfo;
    state.libDir     = libDir;
    state.repoRoot   = repoRoot;
    state.mechName   = mechName;
    state.nInst      = nInst;
    state.nConns     = nConns;
    state.L          = L;
    state.varNames   = varNames;
    state.nFrames    = nFrames;
    state.trajectory = jointTrajectory;
    state.ax         = ax;
    state.fig        = fig;
    state.currentFrame = 1;
    state.playing    = false;
    state.fps        = fps;
    state.timer      = [];
    state.geomCache  = geomCache;
    state.neededFrames = neededFrames;
    state.evalFrames     = evalFrames;
    state.useWorldAnchor = useWorldAnchor;
    state.groundNodes    = groundNodes;
    state.Tbind          = Tbind;
    state.trailPos       = trailPos;
    state.showTrail      = opts.ShowTrail;

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
    uicontrol('Style', 'slider', 'Min', 1, 'Max', 60, 'Value', fps, ...
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
    fprintf('Animation ready. %d frames at %d fps. Use ▶ Play to start.\n', nFrames, fps);
end

%% ---- callback helpers ----

function renderCurrentFrame(fig)
    state = fig.UserData;
    i = state.currentFrame;
    ax  = state.ax;

    % build joint value map for this frame
    jvMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
    for j = 1:numel(state.varNames)
        jvMap(state.varNames{j}) = state.trajectory(i, j);
    end

    % evaluate numeric poses (only needed frames for speed)
    poses = state.expander.evaluateNumericDirect(jvMap, state.evalFrames);

    if state.useWorldAnchor
        % re-express all tool-rooted poses in the world frame:
        % E(q) = T_bind * inv(T_tip<-ground_ref(q))
        gref = state.groundNodes{1};
        E = state.Tbind{1} * core.RigidBodyMath.invT(poses(gref));
        ks = keys(poses);
        for k = 1:numel(ks)
            poses(ks{k}) = E * poses(ks{k});
        end
    end

    % render (bodies only, no triads/mates in animation mode)
    renderFrame(ax, poses, state.inst, state.connInfo, state.libDir, ...
        state.repoRoot, state.mechName, state.nInst, state.nConns, ...
        state.L, state.expander.JointValues, false, state.geomCache);

    if state.useWorldAnchor && state.showTrail
        drawTrail(ax, state.trailPos, i);
    end

    % update title
    title(ax, sprintf('%s  —  frame %d / %d', state.mechName, i, state.nFrames), ...
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
