function seq = buildPoseSequence(expander, jointTrajectory, varNames, varargin)
%BUILDPOSESEQUENCE  FK pose sequence for a joint trajectory (compute-layer product).
%   SEQ = solver.buildPoseSequence(EXPANDER, Q, VARNAMES)
%   SEQ = solver.buildPoseSequence(..., 'Anchor', MODE)
%   SEQ = solver.buildPoseSequence(..., 'Frames', FRAMENAMES)
%
%   Builds the per-frame pose data of a mechanism moving along a joint
%   trajectory — the "solver product" consumed by pure renderers (viz.play)
%   or external tools (export / planning / GUI display).  Poses are computed
%   with the numeric FK path of the expanded mechanism
%   (Expander.evaluateNumericDirect); call EXPANDER.compilePoseFunctions()
%   once beforehand for the fast path (sub-millisecond per frame).
%
%   Inputs:
%     EXPANDER : ir.Expander — expanded mechanism (symbolic pipeline)
%     Q        : nFrames × nJoints double — joint trajectory
%     VARNAMES : cell — canonical joint variable names (columns of Q)
%
%   Name-Value options:
%     'Anchor' : 'root'  (default) — tool-rooted FK poses as-is (root pinned)
%                'world' — re-express every pose in the internal world frame
%                  (world ≡ tip zero pose) via
%                  E(q) = T_bind * inv(T_root<-ground_ref(q)), so L3
%                  world-closed-loop mechanisms (mrex-3t1r_001) keep their
%                  ground frames fixed at the calibrated world bindings
%                  while the tool visibly travels.  T_bind is the zero-pose
%                  FK pose of each ground frame — the same auto-calibration
%                  value as solver.ClosureSolver.  Requires frames with
%                  semantic_tag == 'ground'.
%                'tool' — like 'world', then re-expressed in the tool world
%                  (world ≡ tool-frame zero pose, matching the L3 tool_frame
%                  semantics): every pose is additionally left-multiplied by
%                  inv(U), grounds sit at inv(U)·T_bind, and the tool frame
%                  zero pose is I₄ (the tip appears tilted by inv(U)).
%                  Requires 'ToolTransform' and ground frames.
%     'ToolTransform' : 4×4 double — U = T_root<-tool (e.g. from
%                solver.ClosureSolver.ToolTransform or the execution config's
%                tool_frame block).  Required for 'Anchor','tool'.
%     'Frames' : cell of frame names to include (default: all frames of the
%                expanded mechanism).  The FK root frame is always included
%                (it is the Trail source).
%
%   Output SEQ (struct):
%     .Frames       1×nFrames cell — each a containers.Map (frame → 4×4 double)
%     .FrameNames   cell — frame names present in every entry of .Frames
%     .RootFrame    char — FK root (task) frame name
%     .Anchor       char — 'root' | 'world'
%     .VarNames     cell (copy), .Q — trajectory (copy)
%     .Trail        3×nFrames double — root (tip) world path; zeros for 'root'
%     .Tbind        cell of 4×4 double — ground world bindings ('world' only)
%     .GroundFrames cell — ground frame names ('world' only; {} otherwise)
%     .BranchGapMax double — max ground-branch disagreement over frames
%                   ('world' only; 0 otherwise)
%     .ToolTransform 4×4 double — U when supplied ('tool' anchor); [] otherwise
%     .MechName     char
%
%   See also: viz.play, viz.animate, viz.sceneSpec, +solver/ClosureSolver

    % ---- validate inputs ----
    assert(isa(expander, 'ir.Expander'), ...
        'solver:buildPoseSequence:badInput', ...
        'EXPANDER must be an ir.Expander.');
    assert(isnumeric(jointTrajectory) && ~isempty(jointTrajectory), ...
        'solver:buildPoseSequence:badInput', ...
        'Q (jointTrajectory) must be a non-empty numeric matrix.');
    assert(iscell(varNames) && ~isempty(varNames), ...
        'solver:buildPoseSequence:badInput', ...
        'VARNAMES must be a non-empty cell array.');

    % ---- parse name-value options ----
    opts = struct('Anchor', 'root', 'Frames', {{}}, 'ToolTransform', []);
    if ~isempty(varargin)
        assert(mod(numel(varargin), 2) == 0, ...
            'solver:buildPoseSequence:badOptions', ...
            'Name-value options must come in pairs.');
        for oi = 1:2:numel(varargin)
            oName = varargin{oi};
            oVal  = varargin{oi + 1};
            switch lower(oName)
                case 'anchor'
                    assert(ischar(oVal) || (isstring(oVal) && isscalar(oVal)), ...
                        'solver:buildPoseSequence:badAnchor', ...
                        'Anchor must be a char value.');
                    assert(any(strcmp(oVal, {'root', 'world', 'tool'})), ...
                        'solver:buildPoseSequence:badAnchor', ...
                        'Anchor must be ''root'', ''world'' or ''tool'', got ''%s''.', ...
                        char(oVal));
                    opts.Anchor = char(oVal);
                case 'tooltransform'
                    assert(isnumeric(oVal) && isequal(size(oVal), [4 4]), ...
                        'solver:buildPoseSequence:missingToolTransform', ...
                        'ToolTransform must be a 4×4 double matrix.');
                    opts.ToolTransform = double(oVal);
                case 'frames'
                    opts.Frames = core.CommonUtils.asList(oVal);
                otherwise
                    error('solver:buildPoseSequence:badOption', ...
                        'Unknown option ''%s''.', oName);
            end
        end
    end
    if strcmp(opts.Anchor, 'tool')
        assert(~isempty(opts.ToolTransform), ...
            'solver:buildPoseSequence:missingToolTransform', ...
            ['Anchor = ''tool'' requires ''ToolTransform'' ' ...
             '(U = T_root<-tool, 4×4 double).']);
    end

    % ---- validate trajectory ----
    [nFrames, nCols] = size(jointTrajectory);
    assert(nCols == numel(varNames), ...
        'solver:buildPoseSequence:trajectoryMismatch', ...
        'jointTrajectory has %d columns but varNames has %d entries.', ...
        nCols, numel(varNames));
    for i = 1:numel(varNames)
        assert(isKey(expander.JointVarMap, varNames{i}), ...
            'solver:buildPoseSequence:varNotFound', ...
            'Joint variable "%s" not found in mechanism.', varNames{i});
    end

    % ---- FK root frame (Trail source; always included) ----
    rootFrame = expander.EdgeGraph_.RootNode;
    assert(~isempty(rootFrame), ...
        'solver:buildPoseSequence:noRoot', ...
        'Expander has no registered FK root (semantic_tag: root).');

    % ---- frame set ----
    if isempty(opts.Frames)
        frameNames = keys(expander.Poses);
    else
        frameNames = opts.Frames(:).';
    end
    frameNames = unique([frameNames(:).', {rootFrame}]);

    % ---- ground frames (world anchor) ----
    groundNodes = {};
    for i = 1:numel(expander.Instances)
        for k = 1:numel(expander.Instances(i).frames)
            f = expander.Instances(i).frames{k};
            if isfield(f, 'semantic_tag') && strcmp(f.semantic_tag, 'ground')
                groundNodes{end+1} = f.node; %#ok<AGROW>
            end
        end
    end
    useWorldAnchor = any(strcmp(opts.Anchor, {'world', 'tool'}));
    useToolAnchor  = strcmp(opts.Anchor, 'tool');
    assert(~useWorldAnchor || ~isempty(groundNodes), ...
        'solver:buildPoseSequence:noGround', ...
        ['Anchor = ''world'' requires at least one frame with ' ...
         'semantic_tag == ''ground''; mechanism "%s" has none.'], expander.MechName);

    % ---- world bindings: T_bind = zero-pose FK pose of each ground frame ----
    % (identical to solver.ClosureSolver's auto-calibrated world_binding)
    Tbind = {};
    if useWorldAnchor
        zeroPoses = expander.evaluateNumericDirect(containers.Map);
        Tbind = cell(1, numel(groundNodes));
        for g = 1:numel(groundNodes)
            assert(isKey(zeroPoses, groundNodes{g}), ...
                'solver:buildPoseSequence:groundNotFound', ...
                'Ground frame "%s" not in the expanded Poses map.', groundNodes{g});
            Tbind{g} = zeroPoses(groundNodes{g});
        end
    end
    Uinv = [];
    if useToolAnchor
        Uinv = core.RigidBodyMath.invT(opts.ToolTransform);
    end

    % ---- per-frame FK (+ world anchoring) ----
    Frames = cell(1, nFrames);
    trailPos = zeros(3, nFrames);
    branchGapMax = 0;
    if useWorldAnchor
        fprintf('  Pose sequence: %d frames, %s anchor (%d ground frame(s)), computing ... ', ...
            nFrames, opts.Anchor, numel(groundNodes));
    end
    for k = 1:nFrames
        jvMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
        for j = 1:numel(varNames)
            jvMap(varNames{j}) = jointTrajectory(k, j);
        end
        poses = expander.evaluateNumericDirect(jvMap, frameNames);

        if useWorldAnchor
            % capture pre-conjugation ground poses (anchoring + branch diagnostic)
            gPoses = cell(1, numel(groundNodes));
            for g = 1:numel(groundNodes)
                gPoses{g} = poses(groundNodes{g});
            end
            Ew = Tbind{1} * core.RigidBodyMath.invT(gPoses{1});
            if useToolAnchor
                E = Uinv * Ew;   % re-express in the tool world (world ≡ tool zero)
            else
                E = Ew;
            end
            ks = keys(poses);
            for kk = 1:numel(ks)
                poses(ks{kk}) = E * poses(ks{kk});
            end
            trailPos(:, k) = E(1:3, 4);
            for g = 2:numel(groundNodes)
                Egw = Tbind{g} * core.RigidBodyMath.invT(gPoses{g});
                branchGapMax = max(branchGapMax, norm(Egw - Ew, 'fro'));
            end
        end
        Frames{k} = poses;
    end
    if useWorldAnchor
        fprintf('OK\n  Tip path: %d points; max ground-branch gap = %.3e\n', ...
            nFrames, branchGapMax);
    end
    if useToolAnchor
        % re-express the bindings in the display (tool-world) coordinates
        for g = 1:numel(Tbind)
            Tbind{g} = Uinv * Tbind{g};
        end
    end

    % ---- assemble sequence ----
    seq = struct( ...
        'Frames',       {Frames}, ...
        'FrameNames',   {frameNames}, ...
        'RootFrame',    rootFrame, ...
        'Anchor',       opts.Anchor, ...
        'VarNames',     {varNames}, ...
        'Q',            jointTrajectory, ...
        'Trail',        trailPos, ...
        'Tbind',        {Tbind}, ...
        'GroundFrames', {groundNodes}, ...
        'BranchGapMax', branchGapMax, ...
        'ToolTransform', {opts.ToolTransform}, ...
        'MechName',     expander.MechName);
end
