classdef ExecutionConfig
%EXECUTIONCONFIG  L3 execution configuration: load, validate, and partition variables.
%   A value class that loads an L3 execution-config YAML file, validates it
%   against the JSON Schema and the symbolRegistry, and provides methods to
%   partition variables into known/unknown sets for FK or IK solving.
%
%   For closed_loop mode, closure_cuts may be:
%     - Explicitly declared in the YAML, in either of two forms:
%         * frame-pair cut  : {near, far}            — L2 internal loops
%         * world-anchored  : {near, anchor: world}  — L3 world系 loops (M-REx),
%           where the far side is the near frame's world_binding calibration.
%     - Auto-derived from EdgeGraph's closed_mate edges when omitted
%       (for L2 internal loops where DSL already declares closed: true).
%   Likewise, world_binding may be:
%     - Explicitly declared in the YAML (required when using anchor: world cuts).
%       Each entry's T is optional; empty T means "auto-calibrate at zero pose"
%       (resolved by the solver at construction time).
%     - Omitted for L2 internal loops (ground handled by IR root nodes).
%
%   Tool frame (optional, L3): a tool_frame block declares the user-facing
%   task/input frame as a fixed transform relative to endFrame.  When present,
%   the world is re-anchored to the tool-frame zero pose ("initial orientation
%   parallel to the world axes"), and all user-facing target poses / pose
%   reports are expressed in the tool frame:
%       - input (left-multiplied world-frame increment):
%             ΔT_internal = U · Δ_tool · U⁻¹
%       - output:  D(q) = U⁻¹ · T_rootWorld(q) · U
%   with U = T_root←tool = T_root←endFrame · T_endFrame←tool (requires a
%   joint-free root→endFrame chain; enforced by solver.ClosureSolver).
%   The resolved transform is exposed as cfg.ToolFrame:
%       .Present — logical, true when the YAML declares tool_frame
%       .T       — 4×4 double = T_endFrame←tool (angles normalized to rad)
%   When absent, behavior is unchanged (world ≡ endFrame zero pose).
%
%   Usage:
%       e = ir.Expander(dslYaml);
%       cfg = ir.ExecutionConfig('execution-config.yaml', e.SymbolRegistry, e.EdgeGraph_);
%       % cfg.Mode          — 'open_loop' or 'closed_loop'
%       % cfg.EndFrame      — target frame ref (e.g. 'pipette.tip_origin')
%       % cfg.ClosureCuts   — explicit (frame-pair / world-anchored) or auto cuts
%       % cfg.WorldBindings — ground↔world calibrations (T may be [] = auto)
%       % cfg.ToolFrame     — resolved tool-frame transform (Present=false when absent)
%       % cfg.getSolvingDirection()   — 'FK' or 'IK'
%       % [knownV, unknownV] = cfg.partitionVariables();
%
%   See also: +ir/Expander, +solver/KinematicModel, +ir/EdgeGraph

    % ---- public read-only properties ----
    properties (SetAccess = private)
        Mode          (1,:) char            % 'open_loop' or 'closed_loop'
        EndFrame      (1,:) char            % target frame ref (instance-qualified)
        KnownList     (:,1) cell            % cell array of known variable refs
        UnknownList   (:,1) cell            % cell array of unknown variable refs
        WorldBindings (:,1) struct          % struct array: ground, T (4×4 double | [] = auto-calibrate); empty for open_loop
        ActuatedJoints (:,1) cell           % cell array of actuated joint refs; empty for closed_loop
        ClosureCuts   (:,1) struct          % struct array: near, far, anchor, components; empty for open_loop
        ClosureSource (1,:) char            % 'explicit' (from YAML), 'auto' (from closed_mate edges), or '' (open_loop)
        ToolFrame     (1,1) struct          % struct: Present (logical), T (4×4 double = T_endFrame←tool; angles in rad)
        Tolerances    (1,1) struct          % struct: translation_mm, rotation_rad
        ConfigPath    (1,:) char            % absolute path to the loaded config file
    end

    % ---- private properties ----
    properties (Access = private)
        SymbolRegistry_ (:,1) struct        % copy of the symbolRegistry for validation
    end

    % ---- public methods ----
    methods

        %% Constructor: load execution-config, validate schema + cross-refs.
        %   obj = ExecutionConfig(CONFIGYAML, SYMBOLREGISTRY)
        %   obj = ExecutionConfig(CONFIGYAML, SYMBOLREGISTRY, EDGEGRAPH)
        %     CONFIGYAML      : path to L3 execution-config YAML file
        %     SYMBOLREGISTRY  : struct array from Expander.SymbolRegistry
        %     EDGEGRAPH       : (optional) ir.EdgeGraph — needed to auto-derive
        %                       closure_cuts from closed_mate edges when omitted
        %                       in the YAML (L2 internal loops).
        function obj = ExecutionConfig(configYaml, symbolRegistry, edgeGraph)
            arguments
                configYaml      (1,:) char
                symbolRegistry  (:,1) struct
                edgeGraph       = []  % optional; only needed for auto-derivation
            end

            % ---- resolve path ----
            here = fileparts(fileparts(mfilename('fullpath')));
            configYaml = core.PathUtils.resolve(configYaml, here);
            assert(exist(configYaml, 'file') > 0, ...
                'ir:ExecutionConfig:fileNotFound', ...
                'Execution config file not found: %s', configYaml);
            obj.ConfigPath = configYaml;

            % ---- load YAML ----
            cfg = core.readYaml(configYaml);

            % ---- cross-validate required fields ----
            assert(isfield(cfg, 'mode'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "mode" field.');
            assert(isfield(cfg, 'endFrame'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "endFrame" field.');
            assert(isfield(cfg, 'known'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "known" field.');
            assert(isfield(cfg, 'unknown'), ...
                'ir:ExecutionConfig:missingField', ...
                'execution-config must contain "unknown" field.');

            % ---- store mode and endFrame ----
            obj.Mode = cfg.mode;
            obj.EndFrame = cfg.endFrame;

            % ---- validate mode ----
            validModes = {'open_loop', 'closed_loop'};
            assert(ismember(obj.Mode, validModes), ...
                'ir:ExecutionConfig:invalidMode', ...
                'mode must be "open_loop" or "closed_loop", got "%s".', obj.Mode);

            % ---- store known/unknown lists ----
            obj.KnownList = core.CommonUtils.asList(cfg.known);
            obj.UnknownList = core.CommonUtils.asList(cfg.unknown);

            % ---- store symbolRegistry for validation ----
            obj.SymbolRegistry_ = symbolRegistry;

            % ---- validate mode-conditional fields ----
            switch obj.Mode
                case 'open_loop'
                    % actuated_joints required, closure_cuts + world_binding forbidden
                    assert(isfield(cfg, 'actuated_joints'), ...
                        'ir:ExecutionConfig:missingField', ...
                        'open_loop mode requires "actuated_joints" field.');
                    obj.ActuatedJoints = core.CommonUtils.asList(cfg.actuated_joints);
                    obj.WorldBindings = struct('ground', {}, 'T', {});
                    obj.ClosureCuts = struct('near', {}, 'far', {}, ...
                        'anchor', {}, 'components', {});

                    assert(~isfield(cfg, 'closure_cuts'), ...
                        'ir:ExecutionConfig:fieldConflict', ...
                        'open_loop mode forbids "closure_cuts".');
                    assert(~isfield(cfg, 'world_binding'), ...
                        'ir:ExecutionConfig:fieldConflict', ...
                        'open_loop mode forbids "world_binding".');

                case 'closed_loop'
                    obj.ActuatedJoints = {};
                    obj.ClosureSource = '';

                    assert(~isfield(cfg, 'actuated_joints'), ...
                        'ir:ExecutionConfig:fieldConflict', ...
                        'closed_loop mode forbids "actuated_joints".');

                    % --- closure_cuts: explicit YAML, auto-derived, or error ---
                    if isfield(cfg, 'closure_cuts')
                        % explicit cuts from YAML — two mutually exclusive forms:
                        %   frame-pair cut  : {near, far}            (L2)
                        %   world-anchored  : {near, anchor: world}  (L3)
                        cutsRaw = core.CommonUtils.asList(cfg.closure_cuts);
                        obj.ClosureCuts = struct('near', {}, 'far', {}, ...
                            'anchor', {}, 'components', {});
                        for i = 1:numel(cutsRaw)
                            cut = cutsRaw{i};
                            assert(isfield(cut, 'near') && ~isempty(cut.near), ...
                                'ir:ExecutionConfig:invalidCut', ...
                                'closure_cuts(%d) must contain "near".', i);
                            hasFar    = isfield(cut, 'far')    && ~isempty(cut.far);
                            hasAnchor = isfield(cut, 'anchor') && ~isempty(cut.anchor);
                            assert(xor(hasFar, hasAnchor), ...
                                'ir:ExecutionConfig:invalidCut', ...
                                ['closure_cuts(%d): exactly one of "far" (frame-pair) ' ...
                                 'or "anchor" (world-anchored) must be present.'], i);
                            if hasAnchor
                                anchorVal = cut.anchor;
                                assert(strcmp(anchorVal, 'world'), ...
                                    'ir:ExecutionConfig:invalidCut', ...
                                    'closure_cuts(%d): unsupported anchor "%s" (only "world").', ...
                                    i, anchorVal);
                                farRef = '';
                            else
                                anchorVal = '';
                                farRef = cut.far;
                            end
                            comps = core.CommonUtils.field(cut, 'components', ...
                                {'tx','ty','tz','rx','ry','rz'});
                            obj.ClosureCuts(end+1) = struct( ... %#ok<AGROW>
                                'near', cut.near, ...
                                'far', farRef, ...
                                'anchor', anchorVal, ...
                                'components', {comps});
                        end
                        obj.ClosureSource = 'explicit';
                    elseif ~isempty(edgeGraph)
                        % auto-derive from closed_mate edges (L2 internal loops)
                        allMates = edgeGraph.findMates();
                        closedMask = strcmp({allMates.kind}, 'closed_mate');
                        closedMates = allMates(closedMask);

                        assert(~isempty(closedMates), ...
                            'ir:ExecutionConfig:noClosureCuts', ...
                            ['closed_loop mode: no closure_cuts in YAML and ' ...
                             'no closed_mate edges in EdgeGraph. ' ...
                             'Either declare closure_cuts explicitly (L3 loop) ' ...
                             'or ensure DSL contains closed: true connections (L2 loop).']);

                        obj.ClosureCuts = struct('near', {}, 'far', {}, ...
                            'anchor', {}, 'components', {});
                        for i = 1:numel(closedMates)
                            % closed_mate edges are directional: socket → plug.
                            % The socket side (from) is on the spanning tree;
                            % the plug side (to) is the chord end.
                            % Both are reachable via FK (graph is connected through
                            % bidirectional mate edges).
                            obj.ClosureCuts(end+1) = struct( ... %#ok<AGROW>
                                'near', closedMates(i).from, ...   % socket (tree side)
                                'far',  closedMates(i).to, ...     % plug  (chord end)
                                'anchor', '', ...
                                'components', {{'tx','ty','tz','rx','ry','rz'}});
                        end
                        obj.ClosureSource = 'auto';
                    else
                        error('ir:ExecutionConfig:noClosureCuts', ...
                            ['closed_loop mode: closure_cuts not declared in YAML ' ...
                             'and no EdgeGraph provided for auto-derivation. ' ...
                             'Pass EdgeGraph as the third argument, or declare ' ...
                             'closure_cuts explicitly.']);
                    end

                    % --- world_binding: explicit YAML, or empty (L2 internal loops) ---
                    if isfield(cfg, 'world_binding')
                        wbRaw = core.CommonUtils.asList(cfg.world_binding);
                        obj.WorldBindings = struct('ground', {}, 'T', {});
                        for i = 1:numel(wbRaw)
                            wb = wbRaw{i};
                            assert(isfield(wb, 'ground') && ~isempty(wb.ground), ...
                                'ir:ExecutionConfig:invalidWorldBinding', ...
                                'world_binding entry %d must contain "ground".', i);
                            if isfield(wb, 'T') && ~isempty(wb.T)
                                % explicit calibration: normalize nested
                                % sequence to a 4×4 double matrix
                                Tbind = localAsMatrix4x4(wb.T, i);
                            else
                                % T omitted: auto-calibrate at zero pose
                                % (resolved by solver.ClosureSolver)
                                Tbind = [];
                            end
                            obj.WorldBindings(end+1) = struct( ... %#ok<AGROW>
                                'ground', wb.ground, ...
                                'T', Tbind);
                        end
                    else
                        % L2 internal loop: ground handled by IR root nodes,
                        % no explicit world_binding needed.
                        obj.WorldBindings = struct('ground', {}, 'T', {});
                    end

                    % --- cross-check: world-anchored cuts need a declared binding ---
                    if any(strcmp({obj.ClosureCuts.anchor}, 'world'))
                        assert(~isempty(obj.WorldBindings), ...
                            'ir:ExecutionConfig:anchorNotBound', ...
                            ['closure_cuts declare anchor "world" but ' ...
                             'world_binding is missing or empty.']);
                        grounds = {obj.WorldBindings.ground};
                        for i = 1:numel(obj.ClosureCuts)
                            if strcmp(obj.ClosureCuts(i).anchor, 'world')
                                assert(ismember(obj.ClosureCuts(i).near, grounds), ...
                                    'ir:ExecutionConfig:anchorNotBound', ...
                                    ['closure_cuts(%d): anchor "world" requires near ' ...
                                     'frame "%s" to appear in world_binding.'], ...
                                    i, obj.ClosureCuts(i).near);
                            end
                        end
                    end
            end

            % ---- tool_frame (optional, L3 task/input frame) ----
            obj.ToolFrame = struct('Present', false, 'T', eye(4));
            if isfield(cfg, 'tool_frame')
                obj.ToolFrame = localParseToolFrame(cfg.tool_frame);
            end

            % ---- tolerances (optional, with defaults) ----
            obj.Tolerances = struct( ...
                'translation_mm', 0.001, ...
                'rotation_rad', 0.001);
            if isfield(cfg, 'tolerances')
                tol = cfg.tolerances;
                if isfield(tol, 'translation_mm')
                    obj.Tolerances.translation_mm = tol.translation_mm;
                end
                if isfield(tol, 'rotation_rad')
                    obj.Tolerances.rotation_rad = tol.rotation_rad;
                end
            end

            % ---- cross-validate against symbolRegistry ----
            obj.validateAgainstRegistry();
        end

        %% partitionVariables  Split registry into known and unknown sets.
        %   [knownVars, unknownVars] = obj.partitionVariables()
        %     knownVars   : struct array — subset of symbolRegistry marked as known
        %     unknownVars : struct array — subset of symbolRegistry marked as unknown
        function [knownVars, unknownVars] = partitionVariables(obj)
            knownVars = obj.lookupByNameList(obj.KnownList);
            unknownVars = obj.lookupByNameList(obj.UnknownList);
        end

        %% getSolvingDirection  Determine FK or IK based on known/unknown partition.
        %   dir = obj.getSolvingDirection()
        %     returns 'FK' if endFrame is in unknown (joint vals known → compute pose)
        %     returns 'IK' if endFrame is in known (target pose known → solve joint vals)
        function dir = getSolvingDirection(obj)
            if ismember(obj.EndFrame, obj.UnknownList)
                dir = 'FK';
            elseif ismember(obj.EndFrame, obj.KnownList)
                dir = 'IK';
            else
                error('ir:ExecutionConfig:endFrameNotPartitioned', ...
                    'endFrame "%s" must appear in either known or unknown list.', ...
                    obj.EndFrame);
            end
        end

        %% getKnownJointVars  Return known joint variables with their sym handles.
        %   kjv = obj.getKnownJointVars()
        %     returns struct array: name, symHandle — for joint-type known vars
        function kjv = getKnownJointVars(obj)
            [knownVars, ~] = obj.partitionVariables();
            jointMask = strcmp({knownVars.type}, 'joint');
            kjv = knownVars(jointMask);
        end

        %% getUnknownJointVars  Return unknown joint variables with their sym handles.
        %   ujv = obj.getUnknownJointVars()
        %     returns struct array: name, symHandle — for joint-type unknown vars
        function ujv = getUnknownJointVars(obj)
            [~, unknownVars] = obj.partitionVariables();
            jointMask = strcmp({unknownVars.type}, 'joint');
            ujv = unknownVars(jointMask);
        end

    end

    % ---- private methods ----
    methods (Access = private)

        %% validateAgainstRegistry  Cross-validate all refs against symbolRegistry.
        function validateAgainstRegistry(obj)
            regNames = {obj.SymbolRegistry_.name};

            % -- endFrame must exist in registry --
            assert(ismember(obj.EndFrame, regNames), ...
                'ir:ExecutionConfig:refNotFound', ...
                'endFrame "%s" not found in symbolRegistry.', obj.EndFrame);

            % -- all known/unknown refs must exist --
            allRefs = [obj.KnownList(:); obj.UnknownList(:)];
            for i = 1:numel(allRefs)
                ref = allRefs{i};
                assert(ismember(ref, regNames), ...
                    'ir:ExecutionConfig:refNotFound', ...
                    'Variable ref "%s" (in known/unknown) not found in symbolRegistry.', ref);
            end

            % -- known + unknown must cover all joint variables exactly --
            jointInReg = regNames(strcmp({obj.SymbolRegistry_.type}, 'joint'));
            partitionedJoints = intersect([obj.KnownList(:); obj.UnknownList(:)], jointInReg);
            missingJoints = setdiff(jointInReg, partitionedJoints);
            extraJoints = setdiff(partitionedJoints, jointInReg);
            assert(isempty(missingJoints), ...
                'ir:ExecutionConfig:incompletePartition', ...
                'Joint variable(s) not covered by known/unknown: %s', ...
                strjoin(missingJoints, ', '));
            assert(isempty(extraJoints), ...
                'ir:ExecutionConfig:extraPartition', ...
                'Non-joint ref(s) in known/unknown: %s', ...
                strjoin(extraJoints, ', '));

            % -- no overlap between known and unknown --
            overlap = intersect(obj.KnownList, obj.UnknownList);
            assert(isempty(overlap), ...
                'ir:ExecutionConfig:overlap', ...
                'Variable(s) appear in both known and unknown: %s', ...
                strjoin(overlap, ', '));

            % -- open_loop: all actuated_joints must be joint-type in registry --
            if ~isempty(obj.ActuatedJoints)
                for i = 1:numel(obj.ActuatedJoints)
                    ref = obj.ActuatedJoints{i};
                    assert(ismember(ref, regNames), ...
                        'ir:ExecutionConfig:refNotFound', ...
                        'actuated_joint "%s" not found in symbolRegistry.', ref);
                    idx = find(strcmp(regNames, ref), 1);
                    assert(strcmp(obj.SymbolRegistry_(idx).type, 'joint'), ...
                        'ir:ExecutionConfig:typeMismatch', ...
                        'actuated_joint "%s" is type "%s", expected "joint".', ...
                        ref, obj.SymbolRegistry_(idx).type);
                end
            end

            % -- closed_loop: closure_cuts near/far are IR-level frame node names,
            %    not necessarily in symbolRegistry (port frames are not
            %    registered as observable symbols).  Validation of their
            %    existence in the propagated Poses map is deferred to
            %    solver.ClosureSolver construction.
        end

        %% lookupByNameList  Find registry entries matching a list of names.
        function entries = lookupByNameList(obj, nameList)
            entries = struct('name', {}, 'type', {}, 'symHandle', {}, ...
                'scope', {}, 'module_type', {}, 'instance', {});
            regNames = {obj.SymbolRegistry_.name};
            for i = 1:numel(nameList)
                idx = find(strcmp(regNames, nameList{i}), 1);
                if ~isempty(idx)
                    entries(end+1) = obj.SymbolRegistry_(idx); %#ok<AGROW>
                end
            end
        end

    end

end

%% ---- local functions (not methods) ----

function M = localAsMatrix4x4(raw, entryIdx)
%LOCALASMATRIX4X4  Normalize a YAML-parsed nested sequence to a 4×4 double.
%   RAW      : cell of 4 rows; each row a cell of 4 numeric scalars
%   ENTRYIDX : 1-based world_binding entry index (for error messages)
    assert(iscell(raw) && numel(raw) == 4, ...
        'ir:ExecutionConfig:invalidWorldBinding', ...
        'world_binding entry %d: "T" must be a 4×4 matrix (4 rows).', entryIdx);
    M = zeros(4, 4);
    for r = 1:4
        row = raw{r};
        assert(iscell(row) && numel(row) == 4, ...
            'ir:ExecutionConfig:invalidWorldBinding', ...
            'world_binding entry %d: "T" row %d must contain 4 values.', entryIdx, r);
        for c = 1:4
            v = row{c};
            assert(isnumeric(v) && isscalar(v), ...
                'ir:ExecutionConfig:invalidWorldBinding', ...
                'world_binding entry %d: "T" element (%d,%d) must be numeric.', ...
                entryIdx, r, c);
            M(r, c) = v;
        end
    end
end

function tf = localParseToolFrame(raw)
%LOCALPARSETOOLFRAME  Parse the optional L3 tool_frame block.
%   RAW : struct from the YAML reader — fields per
%         specs/schema/execution-config.schema.yaml $defs/toolFrame.
%   Returns struct: Present = true, T = T_endFrame←tool (4×4 double).
%   Angle tokens: numeric = rad (internal convention, §8); strings may
%   carry an explicit unit suffix ("45deg" / "0.7854rad").
    assert(isstruct(raw) && isfield(raw, 'transform') && isstruct(raw.transform), ...
        'ir:ExecutionConfig:invalidToolFrame', ...
        'tool_frame must contain a "transform" object.');

    tr = raw.transform;

    % ---- translation (mm) ----
    t = zeros(3, 1);
    if isfield(tr, 'translation') && ~isempty(tr.translation)
        t = core.CommonUtils.evalVec(tr.translation, struct());
    end

    % ---- rotation (align / rpy / axis_angle), delegating to RigidBodyMath ----
    R = eye(3);
    if isfield(tr, 'rotation') && ~isempty(tr.rotation)
        rot = localNormalizeRotationAngles(tr.rotation);
        R = core.RigidBodyMath.rot(rot, struct());
    end

    tf = struct('Present', true, 'T', [R, t; 0 0 0 1]);
end

function rot = localNormalizeRotationAngles(rot)
%LOCALNORMALIZEROTATIONANGLES  Convert angle tokens to numeric rad in-place.
%   After normalization RigidBodyMath.rot evaluates the struct with empty
%   params (pure numeric evaluation, no symbolic expressions at L3).
    if isfield(rot, 'rpy') && ~isempty(rot.rpy)
        r = rot.rpy;
        if ~iscell(r); r = num2cell(r); end
        for k = 1:numel(r)
            r{k} = localEvalAngleToken(r{k});
        end
        rot.rpy = r;
    elseif isfield(rot, 'axis_angle') && ~isempty(rot.axis_angle)
        aa = rot.axis_angle;
        aa.q = localEvalAngleToken(aa.q);
        rot.axis_angle = aa;
    end
    % align carries no angle tokens
end

function v = localEvalAngleToken(tok)
%LOCALEVALANGLETOKEN  One angle token → numeric rad.
%   numeric : interpreted as rad (internal convention, §8)
%   "45deg" : degrees with explicit unit label → rad
%   "0.7854rad" : rad with explicit unit label
    if iscell(tok) && isscalar(tok); tok = tok{1}; end
    if isnumeric(tok); v = double(tok); return; end
    s = strtrim(char(tok));
    if endsWith(lower(s), 'deg')
        v = core.CommonUtils.evalScalar(s(1:end-3), struct()) * pi / 180;
    elseif endsWith(lower(s), 'rad')
        v = core.CommonUtils.evalScalar(s(1:end-3), struct());
    else
        v = core.CommonUtils.evalScalar(s, struct());
    end
end
