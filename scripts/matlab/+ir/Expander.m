classdef Expander < handle
%EXPANDER  IR expander: DSL + module library → symbolic IR graph (pure symbolic pipeline).
%   A handle-class that encapsulates the full DSL→IR expansion pipeline:
%   load mechanism DSL, resolve module library, inject geometric parameters,
%   expand every instance's internal frame graph into the shared EdgeGraph
%   with symbolic joint variables, process port-to-port connections (mate
%   edges), register ground nodes, and propagate global poses via FK.
%
%   Usage:
%       % symbolic expansion only (Poses are sym):
%       e = ir.Expander('../../specs/dsl/examples/open-chain-2r/robot_description.yaml');
%
%       % numeric evaluation for viz or solver (configYaml is required):
%       posesNum = e.evaluateNumeric('../../specs/dsl/examples/open-chain-2r/joint_config.yaml');
%
%       % access expanded data:
%       % e.MechName, e.Instances, e.ConnectionInfo, e.Poses, e.LibDir,
%       % e.JointVarMap, e.EdgeGraph_
%
%   See also: +ir/EdgeGraph, +viz/mechanism, +core/PosePropagator, +solver/ClosureSolver

    % ---- public read-only properties ----
    properties (SetAccess = private)
        MechName       (1,:) char            % mechanism name from DSL
        Instances      (:,1) struct          % struct array: name, type, md, bodies, frames, joints
        ConnectionInfo (:,1) struct          % struct array: socketNode, plugNode, closed, label
        Poses                              % containers.Map: frame name → 4×4 sym homogeneous transform
        LibDir         (1,:) char            % absolute path to the module library directory
        JointVarMap                       % containers.Map: canonical var name (e.g. 'joint1.q') → symbolic handle
        JointValues                       % containers.Map: canonical var name → numeric value (populated by evaluateNumeric)
        SymbolRegistry (:,1) struct          % struct array: name, type (geometric/joint/task), symHandle, scope, module_type, instance
        EdgeGraph_                          % ir.EdgeGraph handle (public read for downstream consumers)
    end

    % ---- private properties ----
    properties (Access = private)
        DefCache_                           % containers.Map: module_type → parsed module def
        SymbolRegistry_ (:,1) struct = struct('name', {}, 'type', {}, 'symHandle', {}, 'scope', {}, 'module_type', {}, 'instance', {})
        PoseFuncs_                          % containers.Map: frame name → function_handle (populated by compilePoseFunctions)
        PoseFuncVars_                       % sym array: joint variables in the order expected by compiled functions
    end

    % ---- public methods ----
    methods

        %% Constructor: run the full DSL→IR expansion pipeline (pure symbolic).
        %   obj = Expander(DSLYAML)
        %     DSLYAML : path to mechanism-assembly DSL file
        function obj = Expander(dslYaml)
            if nargin < 1 || isempty(dslYaml)
                error('ir:Expander:usage', ...
                    'Usage: ir.Expander(dslYaml)');
            end

            % ---- resolve paths ----
            % 'here' = scripts/matlab/ (two levels up from +ir/)
            here = fileparts(fileparts(mfilename('fullpath')));

            dslYaml = core.PathUtils.resolve(dslYaml, here);
            assert(exist(dslYaml, 'file') > 0, 'DSL file not found: %s', dslYaml);

            % ---- load & validate DSL ----
            dsl = core.readYaml(dslYaml);
            assert(isfield(dsl, 'mechanism'), 'Not a mechanism assembly: %s', dslYaml);

            ver = core.CommonUtils.field(dsl, 'dsl_version', 0);
            assert(isequal(ver, 0), 'Unsupported dsl_version %s (expected 0).', num2str(ver));

            obj.MechName = dsl.mechanism;
            dslDir = fileparts(dslYaml);

            libRel = core.CommonUtils.field(dsl, 'module_library', '../../modules/');
            obj.LibDir = core.PathUtils.resolve(libRel, dslDir);
            assert(exist(obj.LibDir, 'dir') > 0, 'Module library not found: %s', libRel);

            % ---- load parameter configs ----
            % geometric parameters: <libDir>/config/dimensions.yaml (keyed by module_type)
            dimCfg = struct();
            dimCfgPath = fullfile(obj.LibDir, 'config', 'dimensions.yaml');
            if exist(dimCfgPath, 'file'); dimCfg = core.readYaml(dimCfgPath); end

            % --- initialize internal state ---
            obj.EdgeGraph_ = ir.EdgeGraph();
            obj.DefCache_ = containers.Map('KeyType', 'char', 'ValueType', 'any');
            obj.JointVarMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            obj.Instances = struct('name', {}, 'type', {}, 'md', {}, ...
                'bodies', {}, 'frames', {}, 'joints', {});

            % ---- expand instances ----
            assert(isfield(dsl, 'instances') && isstruct(dsl.instances), ...
                'Mechanism %s declares no instances.', obj.MechName);
            instNames = fieldnames(dsl.instances);
            nInst = numel(instNames);

            for i = 1:nInst
                iname = instNames{i};
                itype = dsl.instances.(iname).type;
                obj.Instances(i) = obj.localExpandInstance( ...
                    iname, itype, dimCfg);
            end

            % ---- process connections (mate edges) ----
            conns = core.CommonUtils.asList(core.CommonUtils.field(dsl, 'connections', {}));
            obj.ConnectionInfo = struct('socketNode', {}, 'plugNode', {}, ...
                'closed', {}, 'label', {});

            for c = 1:numel(conns)
                cn = conns{c};
                assert(isfield(cn, 'ports') && numel(cn.ports) == 2, ...
                    'Connection %d must list exactly two ports.', c);
                refA = cn.ports{1}; refB = cn.ports{2};
                [ia, pa] = obj.localParsePort(refA);
                [ib, pb] = obj.localParsePort(refB);
                fa = obj.localLookupFrame(ia, pa);
                fb = obj.localLookupFrame(ib, pb);
                assert(~isempty(fa), 'Connection %d: unknown port "%s".', c, refA);
                assert(~isempty(fb), 'Connection %d: unknown port "%s".', c, refB);

                % -- polarity check: one socket + one plug --
                polA = fa.polarity; polB = fb.polarity;
                if strcmp(polA, 'socket') && strcmp(polB, 'plug')
                    sk = fa; pl = fb;
                elseif strcmp(polA, 'plug') && strcmp(polB, 'socket')
                    sk = fb; pl = fa;
                else
                    error('ir:Expander:polarity', ...
                        ['Connection %d [%s ~ %s]: expected one socket + one plug, ' ...
                         'got "%s" + "%s".'], c, refA, refB, ...
                        core.CommonUtils.tern(isempty(polA), 'none', polA), ...
                        core.CommonUtils.tern(isempty(polB), 'none', polB));
                end

                % -- connection parameters: roll and symmetry --
                roll = core.CommonUtils.field(cn, 'roll', 0);
                nSym = core.CommonUtils.field(sk, 'symmetry', 4);
                isClosed = isequal(core.CommonUtils.field(cn, 'closed', false), true);

                if ~isClosed
                    obj.EdgeGraph_.addMateBidirectional(sk.node, pl.node, roll, nSym);
                else
                    obj.EdgeGraph_.addMateUnidirectional(sk.node, pl.node, roll, nSym);
                end

                obj.ConnectionInfo(end+1) = struct( ...
                    'socketNode', sk.node, 'plugNode', pl.node, ...
                    'closed', isClosed, ...
                    'label', sprintf('%s~%s', sk.node, pl.node)); %#ok<AGROW>
            end

            % ---- root nodes & FK propagation ----
            if ~obj.EdgeGraph_.hasRootNodes()
                obj.EdgeGraph_.addRoot(obj.Instances(1).bodies{1}.node);
            end
            obj.Poses = obj.EdgeGraph_.propagate();

            % ---- fill task frame symHandles from propagated Poses ----
            % NOTE: sym() wrapping normalizes numeric poses to constant sym
            % (e.g. the FK root seed frame I₄), keeping the registry contract
            % "task symHandle is sym" (specs/ir/symbol-registry.md §2).
            for i = 1:numel(obj.SymbolRegistry_)
                if strcmp(obj.SymbolRegistry_(i).type, 'task')
                    frameName = obj.SymbolRegistry_(i).name;
                    if isKey(obj.Poses, frameName)
                        obj.SymbolRegistry_(i).symHandle = sym(obj.Poses(frameName));
                    end
                end
            end

            % ---- publish SymbolRegistry ----
            obj.SymbolRegistry = obj.SymbolRegistry_;
        end

        %% evaluateNumeric  Substitute joint values from config and return numeric Poses.
        %   posesNum = obj.evaluateNumeric(CONFIGYAML)
        %     CONFIGYAML : path to per-instance joint variable config (required).
        %     posesNum   : containers.Map — frame name → 4×4 double transform,
        %                  suitable for viz rendering or numeric IK.
        %
        %   The config file is keyed by instance name → {variable: value},
        function posesNum = evaluateNumeric(obj, configYaml)
            if nargin < 2 || isempty(configYaml)
                error('ir:Expander:evaluateNumeric', ...
                    'Usage: obj.evaluateNumeric(configYaml) — configYaml is required.');
            end

            % -- extract all symbolic joint variables --
            jvKeys = keys(obj.JointVarMap);
            valsCell = values(obj.JointVarMap); % cell array: {sym('joint1.q'), sym('joint2.q'), ...}
            subsVars = [valsCell{:}]; % sym array: [sym('joint1.q'), sym('joint2.q'), ...]
            subsVals = zeros(1, numel(jvKeys)); % seed with zeros

            % overlay config values when a config file is provided
            if ~isempty(configYaml)
                here = fileparts(fileparts(mfilename('fullpath')));
                configYaml = core.PathUtils.resolve(configYaml, here);
                if exist(configYaml, 'file')
                    jointCfg = core.readYaml(configYaml);
                    instNames = fieldnames(jointCfg);
                    % loop over all instances in the config
                    for ii = 1:numel(instNames)
                        iname = instNames{ii};
                        ov = jointCfg.(iname);
                        if ~isstruct(ov); continue; end
                        varNames = fieldnames(ov);
                        % loop over all variable names in the instance config and update subsVals if the canonical name is found in JointVarMap
                        for jj = 1:numel(varNames)
                            canonicalName = [iname '.' varNames{jj}];
                            if isKey(obj.JointVarMap, canonicalName)
                                % find index in subsVars and update value
                                idx = find(subsVars == obj.JointVarMap(canonicalName), 1);
                                if ~isempty(idx)
                                    subsVals(idx) = ov.(varNames{jj});
                                end
                            end
                        end
                    end
                end
            end

            % store numeric joint values for downstream consumers (e.g. viz labels)
            obj.JointValues = containers.Map('KeyType', 'char', 'ValueType', 'double');
            for k = 1:numel(jvKeys)
                obj.JointValues(jvKeys{k}) = subsVals(k);
            end

            % substitute into every entry of the symbolic Poses map
            posesNum = containers.Map('KeyType', 'char', 'ValueType', 'any');
            ks = keys(obj.Poses);
            % loop over all nodes in the symbolic Poses map and evaluate numeric transforms
            for k = 1:numel(ks)
                nodeName = ks{k};
                T_sym = obj.Poses(nodeName);
                if isempty(subsVars)
                    T_num = double(T_sym);
                else
                    % substitute symbolic joint variables with numeric values and evaluate to double
                    T_num = double(subs(T_sym, subsVars, subsVals));
                end
                posesNum(nodeName) = T_num;
            end
        end

        %% evaluateNumericDirect  Substitute joint values from a map and return numeric Poses.
        %   posesNum = obj.evaluateNumericDirect(JOINTVALUEMAP)
        %   posesNum = obj.evaluateNumericDirect(JOINTVALUEMAP, FRAMENAMES)
        %     JOINTVALUEMAP : containers.Map — canonical name → numeric value
        %     FRAMENAMES    : cell array — subset of frame names to evaluate
        %                     (default: all frames in Poses map)
        %     posesNum      : containers.Map — frame name → 4×4 double transform.
        function posesNum = evaluateNumericDirect(obj, jointValueMap, frameNames)
            arguments
                obj
                jointValueMap containers.Map = containers.Map('KeyType','char','ValueType','double')
                frameNames     cell          = {}
            end

            % -- extract all symbolic joint variables --
            jvKeys = keys(obj.JointVarMap);
            valsCell = values(obj.JointVarMap);
            subsVars = [valsCell{:}];
            subsVals = zeros(1, numel(jvKeys));

            % overlay values from the input map
            mapKeys = keys(jointValueMap);
            for k = 1:numel(mapKeys)
                canonicalName = mapKeys{k};
                if isKey(obj.JointVarMap, canonicalName)
                    idx = find(subsVars == obj.JointVarMap(canonicalName), 1);
                    if ~isempty(idx)
                        subsVals(idx) = jointValueMap(canonicalName);
                    end
                end
            end

            % store numeric joint values for downstream consumers (e.g. viz labels)
            obj.JointValues = containers.Map('KeyType', 'char', 'ValueType', 'double');
            for k = 1:numel(jvKeys)
                obj.JointValues(jvKeys{k}) = subsVals(k);
            end

            % substitute into entries of the symbolic Poses map
            posesNum = containers.Map('KeyType', 'char', 'ValueType', 'any');
            if isempty(frameNames)
                ks = keys(obj.Poses);
            else
                ks = frameNames;
            end

            % use pre-compiled function handles when available (animation fast path)
            if ~isempty(obj.PoseFuncs_) && obj.PoseFuncs_.Count > 0
                for k = 1:numel(ks)
                    nodeName = ks{k};
                    if isKey(obj.PoseFuncs_, nodeName)
                        fh = obj.PoseFuncs_(nodeName);
                        T_num = fh(subsVals);
                    else
                        T_sym = obj.Poses(nodeName);
                        if isa(T_sym, 'sym')
                            T_num = double(subs(T_sym, subsVars, subsVals));
                        else
                            T_num = T_sym;  % constant pose, no subs needed
                        end
                    end
                    posesNum(nodeName) = T_num;
                end
            else
                % fallback: subs-based evaluation
                for k = 1:numel(ks)
                    nodeName = ks{k};
                    T_sym = obj.Poses(nodeName);
                    if isempty(subsVars)
                        T_num = double(T_sym);
                    else
                        T_num = double(subs(T_sym, subsVars, subsVals));
                    end
                    posesNum(nodeName) = T_num;
                end
            end
        end

        %% compilePoseFunctions  Pre-compile symbolic poses to numeric function handles.
        %   obj.compilePoseFunctions() converts every entry in the symbolic
        %   Poses map to a matlabFunction handle.  This is a one-time cost
        %   (~seconds) that accelerates subsequent evaluateNumericDirect calls
        %   by 100-1000× (sub-millisecond per frame).
        %
        %   Must be called after the symbolic pipeline is built (i.e. after
        %   the Expander constructor completes).
        function compilePoseFunctions(obj)
            jvKeys = keys(obj.JointVarMap);
            if isempty(jvKeys)
                % no joint variables — nothing to compile; subs on constants is fast
                obj.PoseFuncs_ = containers.Map('KeyType','char','ValueType','any');
                obj.PoseFuncVars_ = sym([]);
                return;
            end

            % build ordered sym array for matlabFunction input
            valsCell = values(obj.JointVarMap);
            obj.PoseFuncVars_ = [valsCell{:}];

            obj.PoseFuncs_ = containers.Map('KeyType', 'char', 'ValueType', 'any');
            ks = keys(obj.Poses);
            nTotal = numel(ks);
            fprintf('  Compiling %d pose functions ... ', nTotal);
            tStart = tic;
            for k = 1:numel(ks)
                nodeName = ks{k};
                T_sym = obj.Poses(nodeName);
                if isa(T_sym, 'sym')
                    fh = matlabFunction(T_sym, 'Vars', {obj.PoseFuncVars_});
                    obj.PoseFuncs_(nodeName) = fh;
                end
                % constant poses (double) are evaluated instantly — no handle needed
            end
            elapsed = toc(tStart);
            fprintf('done (%.1fs, %d functions)\n', elapsed, obj.PoseFuncs_.Count);
        end

    end

    % ---- private methods (migrated from +viz/mechanism.m local functions) ----
    methods (Access = private)

        %% expand a single instance: load module def, inject params, expand bodies/frames/edges
        function inst_i = localExpandInstance(obj, iname, itype, dimCfg)
            if isKey(obj.DefCache_, itype)
                md = obj.DefCache_(itype);
            else
                % fp: full path to module YAML file (e.g. <libDir>/pipette.yaml)
                fp = fullfile(obj.LibDir, [itype '.yaml']);
                assert(exist(fp, 'file') > 0, ...
                    'Instance "%s": module type "%s" not found in %s.', ...
                    iname, itype, obj.LibDir);
                
                % md: parsed module definition (struct) from YAML
                md = core.readYaml(fp);
                obj.DefCache_(itype) = md;
            end

            % inject geometric parameters (cubeLength, tipDistance, ...)
            params = struct();
            if isfield(dimCfg, itype)
                params = dimCfg.(itype);
            end

            % instance name prefix for all node names: "inst1.body"
            pre = [iname '.'];

            % ---- expand bodies ----
            bodies = core.CommonUtils.asList(core.CommonUtils.field(md, 'bodies', {}));
            bList = cell(1, numel(bodies));
            for k = 1:numel(bodies)
                b = bodies{k};
                bList{k} = struct('node', [pre b.name], 'name', b.name, ...
                    'geometry', core.CommonUtils.field(b, 'geometry', ''));

                % ---- collect observable body centers into SymbolRegistry ----
                if core.CommonUtils.field(b, 'observable', false)
                    obj.SymbolRegistry_(end+1) = struct( ...
                        'name', [pre b.name], ...
                        'type', 'task', ...
                        'symHandle', sym([]), ...  % placeholder; filled after FK propagation
                        'scope', 'instance', ...
                        'module_type', itype, ...
                        'instance', iname);
                end
            end

            % ---- expand frames ----
            frames = core.CommonUtils.asList(core.CommonUtils.field(md, 'frames', {}));
            fList = cell(1, numel(frames));
            for k = 1:numel(frames)
                f = frames{k};
                node = [pre f.name];
                fList{k} = struct('node', node, 'name', f.name, ...
                    'exposed', isfield(f, 'exposed') && isequal(f.exposed, true), ...
                    'polarity', core.CommonUtils.field(f, 'polarity', ''), ...
                    'semantic_tag', core.CommonUtils.field(f, 'semantic_tag', ''), ...
                    'symmetry', core.CommonUtils.field(f, 'symmetry', 4));

                % auto-register root frames for FK propagation.
                % semantic_tag='root' (e.g. ToolPipette.connector_side) marks
                % a frame as the propagation seed (pose = eye(4)).
                if strcmp(core.CommonUtils.field(f, 'semantic_tag', ''), 'root')
                    obj.EdgeGraph_.addRoot(node);
                end

                % ---- collect task frames into SymbolRegistry ----
                if isfield(f, 'observable') && isequal(f.observable, true) && ...
                   ~isempty(core.CommonUtils.field(f, 'semantic_tag', ''))
                    obj.SymbolRegistry_(end+1) = struct( ...
                        'name', node, ...
                        'type', 'task', ...
                        'symHandle', sym([]), ...  % placeholder; filled after FK propagation
                        'scope', 'instance', ...
                        'module_type', itype, ...
                        'instance', iname);
                end
            end

            % ---- expand fixed transforms ----
            fts = core.CommonUtils.asList(core.CommonUtils.field(md, 'fixed_transforms', {}));
            for k = 1:numel(fts)
                t = fts{k};
                % evaluate translation vector from struct (x, y, z) or list [x, y, z]
                tr = core.CommonUtils.evalVec(t.translation, params);
                % compute rotation matrix from rotation struct (rpy, axis_angle, or align)
                R = core.RigidBodyMath.rot(t.rotation, params);
                % synthesize 4x4 homogeneous transform and add to EdgeGraph
                T = core.RigidBodyMath.T(R, tr);
                obj.EdgeGraph_.addFixedTransform([pre t.from_frame], [pre t.to_frame], T);
            end

            % ---- expand joints ----
            jts = core.CommonUtils.asList(core.CommonUtils.field(md, 'joints', {}));
            jList = cell(1, numel(jts));
            for k = 1:numel(jts)
                j = jts{k};
                ax = core.CommonUtils.evalVec(j.axis, params);
                kind = core.CommonUtils.field(j, 'kind', 'revolute');
                % create a symbolic variable for this joint (e.g. sym('joint1.q'))
                symName = [pre j.variable];
                % ensure the symbolic variable name is valid for MATLAB (e.g. replace '.' with '_')
                validName = matlab.lang.makeValidName(symName);
                % joint variables are real by definition (angles / displacements);
                % the 'real' assumption keeps symbolic expansions conjugate-free
                % (RigidBodyMath.invT uses ctranspose R', which would otherwise
                % sprinkle conj() of the joint symbols into every pose)
                val = sym(validName, 'real');
                % register in JointVarMap for forward lookup from canonical name
                obj.JointVarMap(symName) = val;
                obj.EdgeGraph_.addJoint([pre j.from_frame], [pre j.to_frame], ax, val, kind);
                % unit vector normalization to avoid singularities in FK propagation; ax(:) converts ax to column vector
                jList{k} = struct('node', [pre j.from_frame], ...
                    'axis', ax(:) / max(norm(ax), eps), ...
                    'var', j.variable, 'val', val, 'kind', kind);

                % ---- collect joint variables into SymbolRegistry ----
                if core.CommonUtils.field(j, 'observable', true)
                    obj.SymbolRegistry_(end+1) = struct( ...
                        'name', symName, ...
                        'type', 'joint', ...
                        'symHandle', val, ...
                        'scope', 'instance', ...
                        'module_type', itype, ...
                        'instance', iname);
                end
            end

            % ---- assemble instance struct ----
            inst_i = struct('name', iname, 'type', itype, 'md', md, ...
                'bodies', {bList}, 'frames', {fList}, 'joints', {jList});
        end

        %% split an "instance.port" reference into its two parts
        function [instName, portName] = localParsePort(~, ref)
            d = strfind(ref, '.');
            assert(~isempty(d), ...
                'Malformed port reference "%s" (expected instance.port).', ref);
            instName = ref(1:d(1)-1);
            portName = ref(d(1)+1:end);
        end

        %% find the recorded frame struct for instance/port; [] if absent
        function f = localLookupFrame(obj, instName, portName)
            f = [];
            for i = 1:numel(obj.Instances)
                % skip instances that don't match the requested name
                if ~strcmp(obj.Instances(i).name, instName); continue; end

                for k = 1:numel(obj.Instances(i).frames)
                    if strcmp(obj.Instances(i).frames{k}.name, portName)
                        f = obj.Instances(i).frames{k};
                        return;
                    end
                end
            end
        end

    end

end
