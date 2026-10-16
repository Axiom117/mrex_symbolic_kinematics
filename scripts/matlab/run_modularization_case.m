function result = run_modularization_case(caseDir, outPath)
%RUN_MODULARIZATION_CASE  Batch driver: run one stage-C case through the symbolic engine.
%   RESULT = RUN_MODULARIZATION_CASE(CASEDIR)
%   RESULT = RUN_MODULARIZATION_CASE(CASEDIR, OUTPATH)
%
%   File protocol (stage C -> IV -> stage C):
%     reads  <caseDir>/robot_description.yaml (required, engine-readable YAML)
%            <caseDir>/joint_config.yaml     (optional; zero pose when absent)
%            <caseDir>/execution-config.yaml (optional; loaded + validated only —
%                                             closure solving is engine stage A.4b)
%     writes OUTPATH (default <caseDir>/result.json)
%
%   result.json v0 fields:
%     protocol, ok, engine_stage, mechanism, error (on failure)
%     counts  {instances, connections, edges, frames, unplaced}
%     mates   {count, max_position_gap_mm, max_z_misalignment, unplaced}
%     frames  [{name, pose16}]  all placed frames, sorted by name (row-major 4x4)
%     joints  [{name, value}]   canonical joint variables, sorted by name
%     ground_offsets [{ground, pose16}]  Manipulator ground frames (world_binding seeds)
%     execution_config {present, loaded, mode, direction, closure_cuts, closure_source}
%     dof     {value, note}     DOF analysis arrives with engine stage A.5.3
%     closure {note}            closure constraint construction is engine stage A.4b
%
%   ok = every frame placed and every mate closed within tolerance (1e-6 mm).
%   The driver never throws on case data errors: the message lands in
%   result.error and result.json is still written, so stage C can report it.
%
%   Usage (batch):
%     matlab -batch "addpath(genpath('<engine>/scripts/matlab')); run_modularization_case('<caseDir>');"
%
%   See also: +ir/Expander, +ir/ExecutionConfig, +viz/mechanism

    arguments
        caseDir (1,:) char
        outPath (1,:) char = ''
    end

    if isempty(outPath); outPath = fullfile(caseDir, 'result.json'); end
    caseDir = core.PathUtils.resolve(caseDir, pwd);

    result = struct();
    result.protocol = 'mrex_modularization/case-result@0';
    result.engine_stage = 'A.3.3';
    result.ok = false;

    try
        tol = 1e-6;   % mm / dimensionless alignment tolerance

        dslFile = fullfile(caseDir, 'robot_description.yaml');
        assert(exist(dslFile, 'file') > 0, ...
            'run_modularization_case: no robot_description.yaml in %s', caseDir);
        jointFile = fullfile(caseDir, 'joint_config.yaml');
        execFile  = fullfile(caseDir, 'execution-config.yaml');

        % ---- symbolic expansion + numeric evaluation (zero pose by default) ----
        e = ir.Expander(dslFile);
        poses = e.evaluateNumeric(jointFile);
        result.mechanism = e.MechName;

        % ---- counts + unplaced frames (bodies + frames of every instance) ----
        expected = {};
        for i = 1:numel(e.Instances)
            for k = 1:numel(e.Instances(i).bodies)
                expected{end+1} = e.Instances(i).bodies{k}.node; %#ok<AGROW>
            end
            for k = 1:numel(e.Instances(i).frames)
                expected{end+1} = e.Instances(i).frames{k}.node; %#ok<AGROW>
            end
        end
        frameNames = sort(keys(poses));
        unplaced = expected(~cellfun(@(n) isKey(poses, n), expected));

        result.counts = struct( ...
            'instances', numel(e.Instances), ...
            'connections', numel(e.ConnectionInfo), ...
            'edges', e.EdgeGraph_.numEdges, ...
            'frames', numel(frameNames), ...
            'unplaced', numel(unplaced));
        result.unplaced_frames = unplaced(:);

        % ---- mate diagnostics: socket/plug origin gap + relative orientation ----
        maxGap = 0; maxMis = 0; unplacedMates = 0;
        for c = 1:numel(e.ConnectionInfo)
            ci = e.ConnectionInfo(c);
            if ~isKey(poses, ci.socketNode) || ~isKey(poses, ci.plugNode)
                unplacedMates = unplacedMates + 1; continue;
            end
            Ps = poses(ci.socketNode); Pp = poses(ci.plugNode);
            gap = norm(Ps(1:3,4) - Pp(1:3,4));
            mis = abs(dot(Ps(1:3,3), Pp(1:3,3)) + 1);   % 0 when +Z anti-parallel
            maxGap = max(maxGap, gap);
            maxMis = max(maxMis, mis);
        end
        result.mates = struct( ...
            'count', numel(e.ConnectionInfo), ...
            'max_position_gap_mm', maxGap, ...
            'max_z_misalignment', maxMis, ...
            'unplaced', unplacedMates);

        % ---- all placed frames (row-major 16 numbers) ----
        frames = struct('name', {}, 'pose16', {});
        for i = 1:numel(frameNames)
            frames(end+1) = struct('name', frameNames{i}, ...
                'pose16', localPose16(poses(frameNames{i}))); %#ok<AGROW>
        end
        result.frames = frames;

        % ---- joint variables (canonical name -> value actually evaluated) ----
        jvNames = sort(keys(e.JointVarMap));
        joints = struct('name', {}, 'value', {});
        for i = 1:numel(jvNames)
            joints(end+1) = struct('name', jvNames{i}, ...
                'value', e.JointValues(jvNames{i})); %#ok<AGROW>
        end
        result.joints = joints;

        % ---- Manipulator ground frames = world_binding seeds ----
        ground = struct('ground', {}, 'pose16', {});
        for i = 1:numel(e.Instances)
            if ~strcmp(e.Instances(i).type, 'Manipulator'); continue; end
            node = [e.Instances(i).name '.ground'];
            if isKey(poses, node)
                ground(end+1) = struct('ground', node, ...
                    'pose16', localPose16(poses(node))); %#ok<AGROW>
            end
        end
        result.ground_offsets = ground;

        % ---- execution config: load + validate when present (never fatal) ----
        result.execution_config = struct('present', false, 'loaded', false, ...
            'mode', '', 'direction', '', 'closure_cuts', 0, 'closure_source', '');
        if exist(execFile, 'file')
            result.execution_config.present = true;
            try
                cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
                result.execution_config.loaded = true;
                result.execution_config.mode = cfg.Mode;
                result.execution_config.direction = cfg.getSolvingDirection();
                result.execution_config.closure_cuts = numel(cfg.ClosureCuts);
                result.execution_config.closure_source = cfg.ClosureSource;
            catch ME
                result.execution_config.loaded = false;
                result.execution_config.error = ME.message;
            end
        end

        % ---- not-yet-available analyses are reported honestly ----
        result.dof = struct('value', [], ...
            'note', 'pending engine stage A.5.3 (DOF analysis)');
        result.closure = struct( ...
            'note', 'pending engine stage A.4b (closure constraint construction)');

        result.ok = result.counts.unplaced == 0 && result.mates.unplaced == 0 && ...
            result.mates.max_position_gap_mm <= tol && result.mates.max_z_misalignment <= tol;
    catch ME
        result.ok = false;
        result.error = ME.message;
    end

    % ---- persist + report ----
    text = jsonencode(result, 'PrettyPrint', true);
    fid = fopen(outPath, 'w', 'n', 'UTF-8');
    assert(fid > 0, 'run_modularization_case: cannot write %s', outPath);
    fwrite(fid, text, 'char'); fclose(fid);

    fprintf('\n=== run_modularization_case: %s ===\n', outPath);
    if isfield(result, 'error')
        fprintf(2, '[ERROR] %s\n', result.error);
    else
        fprintf('mechanism %s: %d instance(s), %d connection(s), %d frame(s) placed\n', ...
            result.mechanism, result.counts.instances, result.counts.connections, ...
            result.counts.frames);
        fprintf('unplaced: %d frame(s), %d mate(s)\n', ...
            result.counts.unplaced, result.mates.unplaced);
        fprintf('mates: max gap %.3e mm, max z misalignment %.3e\n', ...
            result.mates.max_position_gap_mm, result.mates.max_z_misalignment);
        fprintf('ok = %d\n', result.ok);
    end
end

% ------------------------------------------------------------------------
function v = localPose16(T)
%LOCALPOSE16  Row-major 16-number form of a 4x4 homogeneous transform.
    v = reshape(T.', 1, 16);
end
