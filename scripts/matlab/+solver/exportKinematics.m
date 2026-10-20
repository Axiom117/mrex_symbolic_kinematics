function out = exportKinematics(dslYaml, execYaml, varargin)
%EXPORTKINEMATICS  Compile a mechanism case into a self-contained kinematic package.
%   OUT = solver.exportKinematics(DSLYAML, EXECYAML)
%   OUT = solver.exportKinematics(..., 'OutputDir', DIR)
%   OUT = solver.exportKinematics(..., 'KnownValues', MAP)
%   OUT = solver.exportKinematics(..., 'Frames', CELL)
%
%   Runs the full symbolic pipeline (ir.Expander -> ir.ExecutionConfig ->
%   solver.ClosureSolver for closed_loop cases) and exports the compiled
%   numeric kernels of the mechanism as a self-contained model package:
%   plain MATLAB functions that depend only on base MATLAB — no Symbolic
%   Math Toolbox, no Expander, no repository code.  Consumers (PathPlanner
%   real-time IK, M-REx Generative batch evaluation, ...) call the generated
%   functions directly.  See scripts/matlab/MODEL_EXPORT.md for the
%   compiler -> package -> runtime plan.
%
%   Inputs:
%     DSLYAML  : path to the mechanism-assembly DSL file (robot_description.yaml)
%     EXECYAML : path to the L3 execution-config file (execution-config.yaml)
%
%   Name-Value options:
%     'OutputDir'   : target directory of the model package.  Default:
%                     <caseDir>/generated — the "generated" folder next to
%                     the case's DSL file (in-place, per-case export).
%     'KnownValues' : containers.Map — numeric values for known joint
%                     variables (canonical name -> numeric value); missing
%                     entries default to 0.  Same convention as
%                     solver.ClosureSolver.
%     'Frames'      : cell of frame node names for the fk_all kernel
%                     (default: every frame of the expanded mechanism).
%     'Limits'      : struct with lower/upper nq-vectors (input order =
%                     metadata.inputs.names; mm / rad); stored in metadata
%                     as inputs.lower / inputs.upper for the runtime.
%
%   Generated package (all files overwritten on each call):
%     README.md                   package documentation (generated)
%     kinematic_metadata.json     machine-readable contract (see below)
%     closure_residual.m          [closed_loop] r = closure_residual(q)
%     closure_residual_jacobian.m [closed_loop] J = closure_residual_jacobian(q)
%                                 (analytic d r / d q compiled from sym/jacobian)
%     fk_tool.m                   D = fk_tool(q) — task-frame pose
%     fk_all_matrix.m             raw all-frame kernel: (4N)×4 double
%     fk_all.m                    struct wrapper: poses = fk_all(q)
%
%   Input vector Q (n×1 double, same order for every kernel; the order is
%   recorded in kinematic_metadata.json under "inputs.names"):
%     closed_loop : unknown joint variables (ClosureSolver.UnknownNames order)
%     open_loop   : actuated joints (ExecutionConfig.ActuatedJoints order)
%   Units: translations in mm, rotations in rad.
%
%   Generic task-space metadata (package_version 0.2): solver.TaskSpaceAnalyzer
%   derives, at the zero pose, the task-space contract consumed by downstream
%   tooling: inputs.types ('prismatic'/'revolute'), inputs.axes (task-frame
%   motion-axis labels — 'x'/'y'/'z' for axis-aligned prismatic joints,
%   'u#'/'r#' direction clusters otherwise) and task_space {mobility ('3t1r',
%   ...), rotation {mode 'none'/'axis'/'full', axis}, dims, probe}.
%   mrex_generative's mrex.ConfigGenerator consumes task_space to derive the
%   sweep axes / rotation axis without hand-written per-mobility policy rules
%   (the policy's families become a fallback / explicit override).  If the
%   zero pose is not classifiable (rotation.mode = 'full'), consumers fall
%   back to their policy rules.
%
%   Output OUT (struct):
%     .OutputDir : char — package directory actually written
%     .Files     : cell — absolute paths of all files written
%     .Metadata  : struct — metadata serialized to kinematic_metadata.json
%
%   Example:
%     out = solver.exportKinematics( ...
%         'specs/dsl/cases/mrex-3t1r_001/robot_description.yaml', ...
%         'specs/dsl/cases/mrex-3t1r_001/execution-config.yaml');
%     % -> writes specs/dsl/cases/mrex-3t1r_001/generated/{README.md, ...}
%
%   See also: +solver/ClosureSolver, +ir/Expander, +ir/ExecutionConfig

    % ---- parse name-value options (manual parsing: `arguments` blocks and
    %      varargin cannot be combined — repository convention) ----
    knownValues = containers.Map('KeyType', 'char', 'ValueType', 'any');
    outputDir = '';
    frameSel = {};
    limitBounds = [];
    if ~isempty(varargin)
        assert(mod(numel(varargin), 2) == 0, ...
            'solver:exportKinematics:badOptions', ...
            'Name-value options must come in pairs.');
        for oi = 1:2:numel(varargin)
            oName = varargin{oi};
            oVal  = varargin{oi + 1};
            assert(ischar(oName) || (isstring(oName) && isscalar(oName)), ...
                'solver:exportKinematics:badOptions', ...
                'Option names must be char/string.');
            switch lower(char(oName))
                case 'outputdir'
                    assert(ischar(oVal) || (isstring(oVal) && isscalar(oVal)), ...
                        'solver:exportKinematics:badOption', ...
                        'OutputDir must be a char/string path.');
                    outputDir = char(oVal);
                case 'knownvalues'
                    assert(isa(oVal, 'containers.Map'), ...
                        'solver:exportKinematics:badOption', ...
                        'KnownValues must be a containers.Map.');
                    knownValues = oVal;
                case 'frames'
                    frameSel = core.CommonUtils.asList(oVal);
                case 'limits'
                    assert(isstruct(oVal) && isfield(oVal, 'lower') && ...
                        isfield(oVal, 'upper'), ...
                        'solver:exportKinematics:badOption', ...
                        'Limits must be a struct with lower/upper vectors.');
                    limitBounds = struct( ...
                        'lower', double(oVal.lower(:).'), ...
                        'upper', double(oVal.upper(:).'));
                otherwise
                    error('solver:exportKinematics:badOption', ...
                        'Unknown option ''%s''.', char(oName));
            end
        end
    end

    % ---- resolve input paths ----
    here = fileparts(fileparts(mfilename('fullpath')));   % scripts/matlab
    dslAbs  = localResolveFile(dslYaml,  here, 'DSL file');
    execAbs = localResolveFile(execYaml, here, 'Execution config');

    % ---- resolve output directory (default: <caseDir>/generated) ----
    if isempty(outputDir)
        outputDir = fullfile(fileparts(dslAbs), 'generated');
    elseif ~core.PathUtils.isAbsolutePath(outputDir)
        outputDir = fullfile(pwd, outputDir);
    end
    if exist(outputDir, 'dir') ~= 7
        mkdir(outputDir);
    end

    % ---- build the symbolic pipeline ----
    e = ir.Expander(dslAbs);
    cfg = ir.ExecutionConfig(execAbs, e.SymbolRegistry, e.EdgeGraph_);
    isClosed = strcmp(cfg.Mode, 'closed_loop');

    if isClosed
        cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap, knownValues);
        inNames = cs.UnknownNames;
        inSyms  = cs.UnknownSyms;              % 1×n sym row
        inputKind = 'unknown_joints';
        poseOf = @(name) cs.poseSym(name);     % known values already substituted
    else
        % open_loop FK export: inputs = actuated joints (declared order);
        % every other joint variable is substituted by its known value
        % (default 0) — the open_loop counterpart of the closed-loop
        % known-value substitution.
        inNames = cfg.ActuatedJoints(:).';
        inSyms = sym(zeros(1, numel(inNames)));
        for i = 1:numel(inNames)
            assert(isKey(e.JointVarMap, inNames{i}), ...
                'solver:exportKinematics:badActuated', ...
                'Actuated joint "%s" not found in JointVarMap.', inNames{i});
            inSyms(i) = e.JointVarMap(inNames{i});
        end
        inputKind = 'actuated_joints';

        kjv = cfg.getKnownJointVars();
        badKeys = setdiff(keys(knownValues), {kjv.name});
        assert(isempty(badKeys), ...
            'solver:exportKinematics:badKnownValue', ...
            'KnownValues contains non-known-joint key(s): %s', ...
            strjoin(badKeys, ', '));

        subst = localJointSubstitution(e, inNames, knownValues);
        poseOf = @(name) localSubstPose(e.Poses(name), subst);
    end
    nIn = numel(inNames);
    assert(nIn >= 1, ...
        'solver:exportKinematics:noInputs', ...
        'Mechanism "%s" has no %s to export.', e.MechName, inputKind);
    if ~isempty(limitBounds)
        assert(numel(limitBounds.lower) == nIn && ...
            numel(limitBounds.upper) == nIn, ...
            'solver:exportKinematics:badLimits', ...
            'Limits must have %d entries (metadata.inputs.names order).', nIn);
        assert(all(limitBounds.lower <= limitBounds.upper), ...
            'solver:exportKinematics:badLimits', ...
            'Every lower limit must be <= upper.');
    end

    fprintf('  [export] %s | %s | %d inputs -> %s\n', ...
        e.MechName, cfg.Mode, nIn, outputDir);

    % ---- task-frame pose kernel (fk_tool) ----
    hasWorld = isClosed && any(strcmp({cs.ClosureCuts.anchor}, 'world'));
    if hasWorld
        wIdx = find(strcmp({cs.ClosureCuts.anchor}, 'world'), 1);
        refGround = cs.ClosureCuts(wIdx).near;
        T_near = poseOf(refGround);
        Tbind  = cs.ClosureCuts(wIdx).T_bind;
        U      = cs.ToolTransform;
        % D(q) = U⁻¹ · T_bind · T_near(q)⁻¹ · U  (same formula as evalToolPose)
        Dsym = core.RigidBodyMath.invT(U) * ...
            (Tbind * core.RigidBodyMath.invT(T_near)) * U;
        taskFraming = 'world_tool';
    else
        Dsym = poseOf(cfg.EndFrame);
        refGround = '';
        taskFraming = 'root';
        if isClosed && cfg.ToolFrame.Present
            warning('solver:exportKinematics:toolFrameNotApplied', ...
                ['tool_frame is declared but no world-anchored cut exists ' ...
                 'to anchor it; fk_tool keeps the root-framed endFrame pose.']);
        end
    end
    if ~isa(Dsym, 'sym'); Dsym = sym(Dsym); end

    % ---- all-frame FK stack (fk_all) ----
    if isempty(frameSel)
        frameNames = sort(keys(e.Poses)).';
    else
        frameNames = frameSel(:).';
        for k = 1:numel(frameNames)
            assert(isKey(e.Poses, frameNames{k}), ...
                'solver:exportKinematics:frameNotFound', ...
                'Frame "%s" not found in propagated poses.', frameNames{k});
        end
    end
    nFrames = numel(frameNames);
    frameFields = matlab.lang.makeUniqueStrings( ...
        matlab.lang.makeValidName(frameNames));
    Tcell = cell(1, nFrames);
    for k = 1:nFrames
        Tk = poseOf(frameNames{k});
        if ~isa(Tk, 'sym'); Tk = sym(Tk); end
        Tcell{k} = Tk;
    end
    Tstack = vertcat(Tcell{:});   % (4N)×4 sym

    % ---- guard: exported expressions depend only on the input vector ----
    usedNames = [arrayfun(@char, symvar(Dsym), 'UniformOutput', false), ...
                 arrayfun(@char, symvar(Tstack), 'UniformOutput', false)];
    inRendered = cellfun(@char, num2cell(inSyms), 'UniformOutput', false);
    stray = setdiff(unique(usedNames), inRendered);
    assert(isempty(stray), ...
        'solver:exportKinematics:unresolvedVars', ...
        'Exported expressions still contain non-input variables: %s.', ...
        strjoin(stray, ', '));

    % ---- compile kernels (matlabFunction → standalone numeric .m files) ----
    files = {};
    if isClosed
        t0 = tic;
        matlabFunction(cs.ResidualSym, 'Vars', {inSyms.'}, ...
            'File', fullfile(outputDir, 'closure_residual.m'));
        files{end+1} = fullfile(outputDir, 'closure_residual.m'); %#ok<AGROW>

        Jsym = jacobian(cs.ResidualSym, inSyms);
        matlabFunction(Jsym, 'Vars', {inSyms.'}, ...
            'File', fullfile(outputDir, 'closure_residual_jacobian.m'));
        files{end+1} = fullfile(outputDir, 'closure_residual_jacobian.m'); %#ok<AGROW>
        fprintf(['  [export] closure_residual.m (%d components) + ' ...
            'jacobian (%d×%d) compiled (%.1fs)\n'], ...
            cs.NumComponents, cs.NumComponents, nIn, toc(t0));
    end

    t0 = tic;
    matlabFunction(Dsym, 'Vars', {inSyms.'}, ...
        'File', fullfile(outputDir, 'fk_tool.m'));
    files{end+1} = fullfile(outputDir, 'fk_tool.m'); %#ok<AGROW>
    matlabFunction(Tstack, 'Vars', {inSyms.'}, ...
        'File', fullfile(outputDir, 'fk_all_matrix.m'));
    files{end+1} = fullfile(outputDir, 'fk_all_matrix.m'); %#ok<AGROW>
    localWriteFkAllWrapper(fullfile(outputDir, 'fk_all.m'), frameFields, nIn);
    files{end+1} = fullfile(outputDir, 'fk_all.m'); %#ok<AGROW>
    fprintf('  [export] fk_tool.m + fk_all.m (%d frames) compiled (%.1fs)\n', ...
        nFrames, toc(t0));

    % ---- de-ambiguate `--` (codegen double negation) in compiled kernels ----
    % matlabFunction prints a nested negation -(-(u)) as the token `--`
    % (origin: the ry residual component asin(-R(3,1)); its derivative
    % -(-dR31/dq)/sqrt(1-R31²) renders as `--(dR31dq)`).  MATLAB parses the
    % token as two unary minuses, so the compiled value is correct, but the
    % look-alike decrement operator makes the Code Analyzer flag it ("`--x`
    % operation does not decrement the value of `x`") — a red diagnostic in
    % editors, and merely space-separating to `- -` does not silence it.
    % Rewrite as double-negation elimination — see
    % localDeambiguateDoubleNegation.
    kernelFiles = {'fk_tool.m', 'fk_all_matrix.m'};
    if isClosed
        kernelFiles = [{'closure_residual.m', 'closure_residual_jacobian.m'}, ...
            kernelFiles];
    end
    for ki = 1:numel(kernelFiles)
        localDeambiguateDoubleNegation(fullfile(outputDir, kernelFiles{ki}));
    end

    % ---- task-space analysis（泛化元数据：inputs.types/axes + task_space）----
    % 纯数值瞬时分析（solver.TaskSpaceAnalyzer）：零位探针处提取 mobility /
    % 旋转轴 / 输入类型与轴向标签 —— 下游（mrex_generative 配置生成）据此
    % 自动派生扫描网格与限位规则，不再依赖逐机构手工规则。
    t0 = tic;
    probe = struct();
    probe.names = inNames(:).';
    probe.probeQ = zeros(nIn, 1);
    if isClosed
        probe.constraintFcn = localLoopConstraintFcn(cs);
        poseAtZero = @(node) localEvalSym(cs.poseSym(node), ...
            cs.UnknownSyms, zeros(1, nIn));
    else
        probe.constraintFcn = [];
        poseAtZero = @(node) localEvalSym(poseOf(node), ...
            inSyms, zeros(1, nIn));
    end
    if hasWorld
        % 世界锚定（L3）：任务位姿 = 工具共轭变换（同 fk_tool 导出）
        probe.poseFcn = @(q) cs.evalToolPose(q);
        probe.toolU = cs.ToolTransform;
    else
        % 开环 / L2 内部闭环：任务位姿 = root 锚定 endFrame 位姿（同 fk_tool 导出）
        probe.poseFcn = @(q) double(subs(Dsym, inSyms, q(:).'));
        probe.toolU = eye(4);
    end
    [probe.kinds, probe.axesRoot, noRecord] = ...
        localInputMotionInfo(e, inNames, poseAtZero);
    if ~isempty(noRecord)
        warning('solver:exportKinematics:noJointRecord', ...
            ['No joint record for input(s): %s — types fall back to name ' ...
             'heuristics; their axis labels are left empty.'], ...
            strjoin(noRecord, ', '));
    end
    ts = solver.TaskSpaceAnalyzer.analyze(probe);
    fprintf(['  [export] task_space: mobility %s | rotation %s | ' ...
        'manifold %d | axes [%s] (%.1fs)\n'], ...
        ts.mobility, localRotationSummary(ts.rotation), ...
        ts.probe.manifold_dim, strjoin(ts.axes, ' '), toc(t0));

    % ---- runtime contract: actuated/passive split + rigid tool attachment ----
    [inputRoles, actIdx, passIdx] = localInputRoles(e, inNames, isClosed);
    [C_end, Cconstant] = localConstantPose(poseOf(cfg.EndFrame));

    % ---- metadata (machine-readable package contract) ----
    meta = struct();
    meta.package_format = 'smk-kinematic-package';
    meta.package_version = 0.2;
    gen = struct();
    gen.entry = 'solver.exportKinematics';
    gen.version = '0.2.0';
    gen.matlab_release = version('-release');
    gen.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    meta.generator = gen;

    meta.mechanism = e.MechName;
    meta.mode = cfg.Mode;
    meta.direction = cfg.getSolvingDirection();
    meta.end_frame = cfg.EndFrame;
    meta.units = struct('length', 'mm', 'angle', 'rad');
    meta.sources = struct( ...
        'dsl_file', dslAbs, ...
        'execution_config', execAbs, ...
        'module_library', e.LibDir);

    inputsMeta = struct();
    inputsMeta.kind = inputKind;
    inputsMeta.names = inNames;   % canonical names — column order of q
    inputsMeta.types = ts.types;  % per-input joint kind: prismatic / revolute
    inputsMeta.axes = ts.axes;    % per-input motion-axis label (task frame)
    inputsMeta.count = nIn;
    % Runtime contract: which entries are actuator commands vs passive joints.
    % closed_loop classifies Manipulator-module joints as actuated (M-REx
    % dx/dy/dz) and every other joint as passive; open_loop inputs are all
    % actuated by definition.
    inputsMeta.roles = inputRoles;
    inputsMeta.actuated_indices = actIdx;
    inputsMeta.passive_indices = passIdx;
    inputsMeta.actuated = inNames(actIdx);
    inputsMeta.passive = inNames(passIdx);
    inputsMeta.actuated_count = numel(actIdx);
    inputsMeta.passive_count = numel(passIdx);
    inputsMeta.lower = [];
    inputsMeta.upper = [];
    if ~isempty(limitBounds)
        inputsMeta.lower = limitBounds.lower;
        inputsMeta.upper = limitBounds.upper;
    end
    meta.inputs = inputsMeta;

    kjv = cfg.getKnownJointVars();
    kjCell = cell(1, numel(kjv));
    for i = 1:numel(kjv)
        v = 0;
        if isKey(knownValues, kjv(i).name)
            v = knownValues(kjv(i).name);
        end
        kjCell{i} = struct('name', kjv(i).name, 'value', v);
    end
    meta.known_joint_values = kjCell;

    if isClosed
        closureMeta = struct();
        closureMeta.source = cfg.ClosureSource;
        closureMeta.num_cuts = cs.NumCuts;
        closureMeta.num_components = cs.NumComponents;
        cutsCell = cell(1, cs.NumCuts);
        for i = 1:cs.NumCuts
            c = cs.ClosureCuts(i);
            ci = struct();
            ci.index = i;
            ci.near = c.near;
            ci.far = c.far;
            ci.anchor = c.anchor;
            ci.components = c.components;
            ci.T_mate = localRows(c.T_mate);
            if isfield(c, 'T_bind') && ~isempty(c.T_bind)
                ci.T_bind = localRows(c.T_bind);
            else
                ci.T_bind = [];
            end
            cutsCell{i} = ci;
        end
        closureMeta.cuts = cutsCell;
        meta.closure = closureMeta;
    end

    meta.tolerances = struct( ...
        'translation_mm', cfg.Tolerances.translation_mm, ...
        'rotation_rad', cfg.Tolerances.rotation_rad);

    tfMeta = struct();
    tfMeta.framing = taskFraming;
    tfMeta.end_frame = cfg.EndFrame;
    tfMeta.reference_ground = refGround;
    tfMeta.tool_conjugated = strcmp(taskFraming, 'world_tool');
    if tfMeta.tool_conjugated
        tfMeta.U = localRows(U);
    else
        tfMeta.U = localRows(eye(4));
    end
    % U = C * U_rel: keep the rigid root->endFrame attachment separate so a
    % runtime can replace U_rel (T_endFrame<-tool) per call without re-export.
    tfMeta.C = localRows(C_end);
    tfMeta.C_constant = Cconstant;
    tfMeta.tool_declared = cfg.ToolFrame.Present;
    tfMeta.U_rel = localRows(cfg.ToolFrame.T);
    meta.task_frame = tfMeta;
    runtimeSupports = {'fkPackage', 'fk'};
    if hasWorld
        runtimeSupports = [runtimeSupports, ...
            {'ikPackage', 'ik', 'ikTrajectory', ...
             'fkDrivers', 'ikDrivers'}]; %#ok<AGROW>
    elseif ~isClosed
        runtimeSupports = [runtimeSupports, {'fkDrivers'}]; %#ok<AGROW>
    end
    meta.runtime = struct( ...
        'api', 'solver.KinematicsRuntime', ...
        'api_version', '0.1.0', ...
        'units', 'mm/rad', ...
        'pose_convention', 'ZYX', ...
        'tool_frame_runtime', Cconstant, ...
        'package', matlab.lang.makeValidName(e.MechName), ...
        'supports', {runtimeSupports});

    % ---- task space（泛化元数据；来源 solver.TaskSpaceAnalyzer）----
    % mobility  / 旋转轴（mode = axis 时为任务系单位向量，与下游扫描的 phi
    % 轴同约定） / 维数 / 探针诊断（零位 q、物理环约束秩、流形维数）。
    meta.task_space = struct();
    meta.task_space.mobility = ts.mobility;
    meta.task_space.rotation = ts.rotation;
    meta.task_space.dims = ts.dims;
    meta.task_space.probe = ts.probe;

    stats = struct();
    stats.instances = numel(e.Instances);
    stats.edges = e.EdgeGraph_.numEdges;
    stats.joint_variables = e.JointVarMap.Count;
    stats.frames = nFrames;
    meta.mechanism_stats = stats;

    framesCell = cell(1, nFrames);
    for k = 1:nFrames
        fi = struct();
        fi.index = k;
        fi.node = frameNames{k};
        fi.field = frameFields{k};
        framesCell{k} = fi;
    end
    meta.frames = framesCell;

    fnMeta = struct();
    if isClosed
        fnMeta.closure_residual = struct('file', 'closure_residual.m', ...
            'input', 'q', 'output_size', [cs.NumComponents 1]);
        fnMeta.closure_residual_jacobian = struct('file', ...
            'closure_residual_jacobian.m', 'input', 'q', ...
            'output_size', [cs.NumComponents nIn]);
    end
    fnMeta.fk_tool = struct('file', 'fk_tool.m', 'input', 'q', ...
        'output_size', [4 4]);
    fnMeta.fk_all = struct('file', 'fk_all.m', 'input', 'q', ...
        'output_shape', '4N x 4 raw kernel (fk_all_matrix.m); struct of 4x4 poses');
    meta.functions = fnMeta;

    % ---- write package files: kinematic_metadata.json + README.md ----
    t0 = tic;
    jsonPath = fullfile(outputDir, 'kinematic_metadata.json');
    fid = fopen(jsonPath, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'solver:exportKinematics:writeFailed', ...
        'Cannot write %s.', jsonPath);
    fwrite(fid, jsonencode(meta, 'PrettyPrint', true), 'char');
    fclose(fid);
    files{end+1} = jsonPath; %#ok<AGROW>

    if isClosed
        zeroNote = sprintf('闭环残差 ‖r(0)‖ = %.2e（全部 %d 个残差分量；零位标定参考）', ...
            norm(cs.evalResidual(zeros(nIn, 1))), cs.NumComponents);
    else
        zeroNote = '零位 = 全部驱动关节取 0（开环 FK 无残差约束）';
    end
    readmePath = fullfile(outputDir, 'README.md');
    localWriteReadme(readmePath, e, cfg, ...
        meta, inNames, nIn, outputDir, dslAbs, execAbs, ...
        isClosed, taskFraming, zeroNote);
    files{end+1} = readmePath; %#ok<AGROW>
    fprintf('  [export] kinematic_metadata.json + README.md written (%.1fs)\n', ...
        toc(t0));

    % ---- summary ----
    out = struct();
    out.OutputDir = outputDir;
    out.Files = files;
    out.Metadata = meta;
end

%% ---- local functions (not methods) ----

function p = localResolveFile(p, here, label)
%LOCALRESOLVEFILE  Resolve PATH relative to the caller's `here` dir / CWD.
    p = core.PathUtils.resolve(p, here);
    assert(exist(p, 'file') > 0, ...
        'solver:exportKinematics:fileNotFound', ...
        '%s not found: %s', label, p);
    if ~core.PathUtils.isAbsolutePath(p)
        p = fullfile(pwd, p);
    end
end

function subst = localJointSubstitution(expander, inputNames, knownValues)
%LOCALJOINTSUBSTITUTION  Fixed substitutions for non-input joint variables.
%   Solves the open_loop counterpart of ClosureSolver's known-value
%   substitution: every joint variable that is not an export input is
%   replaced by its known value (default 0).
    jvKeys = keys(expander.JointVarMap);
    if isempty(jvKeys)
        subst = struct('syms', sym([]), 'vals', []);
        return;
    end
    valsCell = values(expander.JointVarMap);
    allSyms = [valsCell{:}];
    mask = ~ismember(jvKeys, inputNames);   % n×1 logical
    subst = struct('syms', reshape(allSyms(mask), 1, []), ...
                   'vals', zeros(1, nnz(mask)));
    % NOTE: iterate by explicit index — `for k = find(mask).'` is NOT safe
    % for an all-false mask: a 0×1 empty runs ONE iteration with k = []
    % (for iterates over columns), and `jvKeys{[]}` expands to an empty
    % comma-separated list (isKey then sees a single argument).
    idx = 1;
    kList = find(mask);
    for ii = 1:numel(kList)
        k = kList(ii);
        if isKey(knownValues, jvKeys{k})
            subst.vals(idx) = knownValues(jvKeys{k});
        end
        idx = idx + 1;
    end
end

function T = localSubstPose(T, subst)
%LOCALSUBSTPOSE  Apply fixed substitutions and normalize to sym.
    if isa(T, 'sym')
        if ~isempty(subst.syms)
            T = subs(T, subst.syms, subst.vals);
        end
    else
        T = sym(T);
    end
end

function c = localRows(T)
%LOCALROWS  Row-major cell encoding of a matrix for jsonencode.
%   num2cell(T,2) yields one cell per row, so the JSON nested arrays
%   round-trip through jsondecode with exact row/column order.
    c = num2cell(T, 2);
end

function localWriteFkAllWrapper(filePath, frameFields, nIn)
%LOCALWRITEFKALLWRAPPER  Write the hand-templated fk_all struct wrapper.
    namesLit = strjoin(cellfun(@(s) sprintf('''%s''', s), ...
        frameFields(:).', 'UniformOutput', false), '; ');
    txt = sprintf([ ...
        'function poses = fk_all(q)\n' ...
        '%%FK_ALL  All-frame FK poses (generated kinematic package - do not edit).\n' ...
        '%%   POSES = FK_ALL(Q) evaluates the forward kinematics of all frames\n' ...
        '%%   at input vector Q (%d x 1; input order: see kinematic_metadata.json).\n' ...
        '%%   POSES : struct - sanitized frame field name -> 4x4 double pose\n' ...
        '%%   (FK-root-anchored).\n' ...
        'raw = fk_all_matrix(q);\n' ...
        'names = {%s};\n' ...
        'poses = struct();\n' ...
        'for k = 1:numel(names)\n' ...
        '    poses.(names{k}) = raw(4*(k-1) + (1:4), :);\n' ...
        'end\n' ...
        'end\n'], nIn, namesLit);
    fid = fopen(filePath, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'solver:exportKinematics:writeFailed', ...
        'Cannot write %s.', filePath);
    fwrite(fid, txt, 'char');
    fclose(fid);
end

function localDeambiguateDoubleNegation(filePath)
%LOCALDEAMBIGUATEDOUBLENEGATION  Eliminate codegen `--` (double negation).
%   matlabFunction prints nested negation -(-(u)) as the token `--(u)`
%   (e.g. `mt2 = [--(t4.*5.0e-1-et97.*et98).*... ]`).  MATLAB parses that
%   as two unary minuses — the value is correct — but the token collides
%   with the decrement operator, so the Code Analyzer reports "`--x`
%   operation does not decrement the value of `x`" (shown as a red error on
%   opened files; space-separating to `- -` does NOT silence it).  Apply the
%   identity -(-u) = +u textually: `--` -> `+`, which is valid in every
%   context (prefix `--u` = u; binary `a--b` = a+b; exponent `2^--u` = 2^u;
%   after `.*`/`./` etc.).  A preceding `+` is folded into the rewrite
%   first so no `++` (the symmetric increment look-alike) can be produced.
%   No-op when the token is absent.
    txt = fileread(filePath);
    if ~contains(txt, '--')
        return;
    end
    txt = strrep(txt, '+--', '+');
    txt = strrep(txt, '--', '+');
    fid = fopen(filePath, 'w');
    assert(fid > 0, 'solver:exportKinematics:writeFailed', ...
        'Cannot rewrite %s.', filePath);
    fwrite(fid, txt, 'char');
    fclose(fid);
end

function localWriteReadme(filePath, e, cfg, meta, inNames, nIn, outputDir, ...
    dslAbs, execAbs, isClosed, taskFraming, zeroNote)
%LOCALWRITEREADME  Write the generated package README (UTF-8, Chinese prose).
    L = {};
    L{end+1} = sprintf('# %s — 自包含运动学模型包（generated）', e.MechName);
    L{end+1} = '';
    L{end+1} = sprintf('> ⚠️ 本目录由 `solver.exportKinematics` 自动生成（%s，MATLAB %s）。请勿手工编辑——', ...
        meta.generator.generated_at, meta.generator.matlab_release);
    L{end+1} = sprintf('> 重新生成（覆盖本目录内容）：`solver.exportKinematics(''%s'', ''%s'')`。', ...
        dslAbs, execAbs);
    L{end+1} = '>';
    L{end+1} = '> 流水线规划（符号编译器 → 模型包 → 运行引擎）见 `scripts/matlab/MODEL_EXPORT.md`。';
    L{end+1} = '';
    L{end+1} = '## 输入约定';
    L{end+1} = '';
    L{end+1} = sprintf('全部内核函数接收同一输入向量 `q`（%d×1 double，列向量；长度单位 mm、角度单位 rad）。', nIn);
    L{end+1} = sprintf('输入类型：`%s`（模式 `%s`，求解方向 `%s`）。', meta.inputs.kind, cfg.Mode, meta.direction);
    L{end+1} = '';
    L{end+1} = '| # | 变量 |';
    L{end+1} = '|---|------|';
    for i = 1:nIn
        L{end+1} = sprintf('| %d | `%s` |', i, inNames{i}); %#ok<AGROW>
    end
    L{end+1} = '';
    L{end+1} = zeroNote;
    L{end+1} = '';
    L{end+1} = sprintf('输入类型：%s；运动轴标签（任务系）：`%s`。', ...
        strjoin(cellfun(@(t) sprintf('`%s`', t), meta.inputs.types, ...
        'UniformOutput', false), ' / '), ...
        strjoin(meta.inputs.axes, ' '));
    if isfield(meta.inputs, 'actuated') && ~isempty(meta.inputs.actuated)
        L{end+1} = sprintf('驱动/被动分区：actuated = [%s]；passive = [%s]。', ...
            strjoin(meta.inputs.actuated, ', '), ...
            strjoin(meta.inputs.passive, ', '));
    end
    if isfield(meta.inputs, 'lower') && ~isempty(meta.inputs.lower)
        L{end+1} = sprintf('硬限位（q 顺序）：lower = [%s]；upper = [%s]。', ...
            num2str(meta.inputs.lower, ' %.6g'), ...
            num2str(meta.inputs.upper, ' %.6g'));
    end
    L{end+1} = '';
    L{end+1} = '## 任务空间（瞬时分析）';
    L{end+1} = '';
    L{end+1} = sprintf(['- mobility = `%s`：任务旋量 %d 维（平移 %d ⊕ 旋转 %d）；' ...
        '闭环流形 %d 维（零位探针）。'], ...
        meta.task_space.mobility, meta.task_space.dims.task, ...
        meta.task_space.dims.translation, meta.task_space.dims.rotation, ...
        meta.task_space.probe.manifold_dim);
    switch meta.task_space.rotation.mode
        case 'axis'
            L{end+1} = sprintf(['- 旋转自由度：绕任务系轴 `[%.4f %.4f %.4f]` 的单参数旋转' ...
                '（phi 扫描轴；下游工作空间评估同约定）。'], ...
                meta.task_space.rotation.axis);
        case 'none'
            L{end+1} = '- 旋转自由度：无（纯平移任务空间）。';
        otherwise
            L{end+1} = '- 旋转自由度：多轴（mode = full；下游须显式指定扫描策略）。';
    end
    L{end+1} = '';
    L{end+1} = '> `task_space` / `inputs.types` / `inputs.axes` 由 `solver.TaskSpaceAnalyzer` 自动提取，';
    L{end+1} = '> 供 mrex_generative 等下游免手工规则派生扫描配置；字段语义见 `scripts/matlab/MODEL_EXPORT.md`。';
    L{end+1} = '';
    L{end+1} = '## 文件';
    L{end+1} = '';
    L{end+1} = '| 文件 | 签名 | 语义 |';
    L{end+1} = '|------|------|------|';
    L{end+1} = '| `kinematic_metadata.json` | — | 机器可读契约：输入顺序、闭环切口与世界标定、工具变换、容差、frame 索引 |';
    if isClosed
        L{end+1} = sprintf('| `closure_residual.m` | `r = closure_residual(q)` | 闭环残差（%d×1；`‖r‖≈0` ⟺ 闭环成立） |', meta.closure.num_components);
        L{end+1} = sprintf('| `closure_residual_jacobian.m` | `J = closure_residual_jacobian(q)` | 解析雅可比 ∂r/∂q（%d×%d），供高斯-牛顿 / 信赖域迭代 |', meta.closure.num_components, nIn);
    end
    if strcmp(taskFraming, 'world_tool')
        L{end+1} = sprintf('| `fk_tool.m` | `D = fk_tool(q)` | 任务（工具）位姿 4×4：世界 ≡ 工具系零位，含工具变换共轭；参考 ground = `%s` |', meta.task_frame.reference_ground);
    else
        L{end+1} = sprintf('| `fk_tool.m` | `D = fk_tool(q)` | 任务位姿 4×4（endFrame = `%s`，root 锚定） |', meta.end_frame);
    end
    L{end+1} = sprintf('| `fk_all.m` / `fk_all_matrix.m` | `poses = fk_all(q)` | 全体 %d 个 frame 位姿（struct；`fk_all_matrix` 为 (4N)×4 原始内核） |', meta.mechanism_stats.frames);
    L{end+1} = '| `README.md` | — | 本文件 |';
    L{end+1} = '';
    L{end+1} = '## 用法示例';
    L{end+1} = '';
    L{end+1} = '```matlab';
    L{end+1} = sprintf('addpath(''%s'');   %% 将模型包目录加入 MATLAB 搜索路径', outputDir);
    L{end+1} = sprintf('q = zeros(%d, 1);', nIn);
    if isClosed
        L{end+1} = 'r = closure_residual(q);              % 闭环残差（mm / rad）';
        L{end+1} = 'J = closure_residual_jacobian(q);      % 解析雅可比 ∂r/∂q';
    end
    L{end+1} = 'D = fk_tool(q);                        % 任务位姿 4×4';
    L{end+1} = 'poses = fk_all(q);                      % 全体 frame 位姿 struct';
    L{end+1} = '```';
    L{end+1} = '';
    L{end+1} = '## 来源快照';
    L{end+1} = '';
    L{end+1} = sprintf('- DSL：`%s`', dslAbs);
    L{end+1} = sprintf('- execution-config：`%s`', execAbs);
    L{end+1} = sprintf('- 模块库：`%s`', e.LibDir);
    L{end+1} = sprintf('- 机制统计：%d instances / %d edges / %d joint vars / %d frames；mode = `%s`', ...
        meta.mechanism_stats.instances, meta.mechanism_stats.edges, ...
        meta.mechanism_stats.joint_variables, meta.mechanism_stats.frames, cfg.Mode);

    fid = fopen(filePath, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'solver:exportKinematics:writeFailed', ...
        'Cannot write %s.', filePath);
    fwrite(fid, strjoin(L, newline), 'char');
    fwrite(fid, newline, 'char');
    fclose(fid);
end

function T = localEvalSym(T, syms, vals)
%LOCALEVALSYM  subs 求值（常量位姿直接转 double）。
    if isa(T, 'sym')
        T = double(subs(T, syms, vals));
    else
        T = double(T);
    end
end

function [roles, actIdx, passIdx] = localInputRoles(e, inNames, isClosed)
%LOCALINPUTROLES  Runtime actuated/passive split for the exported input vector.
%   open_loop: every input is an actuated joint by definition.
%   closed_loop: joints belonging to Manipulator modules are actuators
%   (M-REx dx/dy/dz); all other joints are passive/internal.  This mirrors
%   the physical architecture and keeps the package self-describing without
%   adding a new execution-config field.
    n = numel(inNames);
    roles = repmat({'passive'}, 1, n);
    if ~isClosed
        roles = repmat({'actuated'}, 1, n);
    else
        for i = 1:n
            if localInputIsManipulator(e, inNames{i})
                roles{i} = 'actuated';
            end
        end
    end
    actIdx = find(strcmp(roles, 'actuated'));
    passIdx = find(strcmp(roles, 'passive'));
end

function tf = localInputIsManipulator(e, varName)
%LOCALINPUTISMANIPULATOR  True when VARNAME belongs to a Manipulator instance.
    tf = false;
    for i = 1:numel(e.Instances)
        inst = e.Instances(i);
        for k = 1:numel(inst.joints)
            if strcmp([inst.name '.' inst.joints{k}.var], varName)
                tf = strcmpi(inst.type, 'Manipulator');
                return;
            end
        end
    end
end

function [T, isConstant] = localConstantPose(T)
%LOCALCONSTANTPOSE  Numeric 4×4 value when T has no free symbols.
    if isa(T, 'sym')
        isConstant = isempty(symvar(T));
        if isConstant
            T = double(T);
        else
            T = eye(4);
        end
    else
        T = double(T);
        isConstant = true;
    end
end

function [kinds, axesRoot, missing] = localInputMotionInfo(e, inNames, poseAtZero)
%LOCALINPUTMOTIONINFO  逐输入的关节类型与装配零位运动轴（root 系方向）。
%   poseAtZero : @(nodeName) → 4×4 double — 该 frame 零位构型下的 root 位姿。
%   关节记录来自 Expander 实例表（kind / 局部轴），运动轴经该关节
%   from_frame 的零位姿态旋转到 root 系。无记录时类型回退到名字启发式
%   （*.dx/*.dy/*.dz → prismatic），轴向留空（标签 ''）。
    kinds = cell(1, numel(inNames));
    axesRoot = zeros(numel(inNames), 3);
    missing = {};
    for i = 1:numel(inNames)
        [kind, node, axLocal] = localJointRecord(e, inNames{i});
        if isempty(kind)
            if any(endsWith(inNames{i}, {'.dx', '.dy', '.dz'}))
                kind = 'prismatic';
            else
                kind = 'revolute';
            end
            missing{end + 1} = inNames{i}; %#ok<AGROW>
        else
            T = poseAtZero(node);
            d = T(1:3, 1:3) * axLocal(:);
            nd = norm(d);
            if nd > 1e-9
                axesRoot(i, :) = (d / nd).';
            end
        end
        kinds{i} = kind;
    end
end

function [kind, node, axLocal] = localJointRecord(e, varName)
%LOCALJOINTRECORD  按规范名（instance.variable）查实例关节记录。
    kind = ''; node = ''; axLocal = [];
    for i = 1:numel(e.Instances)
        inst = e.Instances(i);
        for k = 1:numel(inst.joints)
            if strcmp([inst.name '.' inst.joints{k}.var], varName)
                kind = inst.joints{k}.kind;
                node = inst.joints{k}.node;
                axLocal = inst.joints{k}.axis;
                return;
            end
        end
    end
end

function fcn = localLoopConstraintFcn(cs)
%LOCALLOOPCONSTRAINTFCN  物理环约束误差向量（供 TaskSpaceAnalyzer 的流形分析）。
%   world 锚定切口：导出残差把两个 ground 各自钉在零位标定上（12 分量对 8 未知
%   量过定，是"双 ground 钉零位"形式，不能用于流形分析）；物理约束只含
%   ground-ground 相对关系，故按相对化形式构造：
%       X_j = invT(T_1) · T_j · invT(invT(G_1) · G_j),  j = 2..nWorld
%   frame-pair 切口（L2）：X = invT(T_near · M) · T_far，按 components 掩码。
%   单个 world 切口（无环）时返回空约束（流形 = 全关节空间）。
    cuts = cs.ClosureCuts;
    isWorld = strcmp({cuts.anchor}, 'world');
    wIdx = find(isWorld);
    pIdx = find(~isWorld);
    allComps = {'tx', 'ty', 'tz', 'rx', 'ry', 'rz'};
    fcn = @evalConstraint;

    function r = evalConstraint(q)
        r = zeros(0, 1);
        if ~isempty(wIdx)
            T1 = evalFrame(cuts(wIdx(1)).near, q);
            G1i = core.RigidBodyMath.invT(cuts(wIdx(1)).T_bind);
            for j = wIdx(2:end)
                Tj = evalFrame(cuts(j).near, q);
                X = core.RigidBodyMath.invT(T1) * Tj * ...
                    core.RigidBodyMath.invT(G1i * cuts(j).T_bind);
                r = [r; localPoseErrVec(X)]; %#ok<AGROW>
            end
        end
        for j = pIdx
            Tn = evalFrame(cuts(j).near, q);
            Tf = evalFrame(cuts(j).far, q);
            X = core.RigidBodyMath.invT(Tn * cuts(j).T_mate) * Tf;
            v = localPoseErrVec(X);
            if isfield(cuts(j), 'components') && ~isempty(cuts(j).components)
                v = v(ismember(allComps, cuts(j).components));
            end
            r = [r; v]; %#ok<AGROW>
        end
    end

    function T = evalFrame(name, q)
        S = cs.poseSym(name);
        if isa(S, 'sym')
            T = double(subs(S, cs.UnknownSyms, q(:).'));
        else
            T = double(S);
        end
    end
end

function v = localPoseErrVec(X)
%LOCALPOSEERRVEC  4×4 相对位姿 → 6 维误差向量（平移 + 0.5·vee(R−Rᵀ)，X = I 时为零）。
    R = X(1:3, 1:3);
    v = [X(1:3, 4); 0.5 * [R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)]];
end

function s = localRotationSummary(rot)
%LOCALROTATIONSUMMARY  旋转分类的单行摘要（导出日志用）。
    switch rot.mode
        case 'axis'
            s = sprintf('axis [%.3f %.3f %.3f]', rot.axis);
        otherwise
            s = rot.mode;
    end
end
