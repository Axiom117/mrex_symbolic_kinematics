function test_export_kinematics_m_rex_3t1r()
%TEST_EXPORT_KINEMATICS_M_REX_3T1R  Model-package export test (mrex-3t1r_001).
%   Runs solver.exportKinematics on the mrex-3t1r_001 case (closed_loop, L3
%   world-anchored cuts, tool_frame declared) and verifies the exported
%   self-contained package against the live solver pipeline:
%     [1] package written to <case>/generated (default OutputDir)
%     [2] kinematic_metadata.json: input order / cuts / bindings / tool U
%     [3] closure_residual.m matches ClosureSolver.evalResidual; r(0) ≈ 0
%     [4] closure_residual_jacobian.m: 12×8, finite, matches central FD
%     [5] fk_tool.m matches ClosureSolver.evalToolPose; fk_tool(0) = I₄
%     [6] fk_all.m: all 96 frames, spot-check vs Expander numeric FK
%     [7] README.md documents the package contract
%     [8] task-space metadata: inputs.types/axes + task_space (mobility 3t1r,
%         rotation axis [1,0,1]/√2, probe ranks) — the generic downstream contract
%
%   Writes (regenerated on every run): specs/dsl/cases/mrex-3t1r_001/generated/
%   (gitignored generated artifacts).
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_export_kinematics_m_rex_3t1r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    caseDir = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex-3t1r_001');
    dslFile = fullfile(caseDir, 'robot_description.yaml');
    execFile = fullfile(caseDir, 'execution-config.yaml');
    exportedDir = fullfile(caseDir, 'generated');

    fprintf('=== test_export_kinematics_m_rex_3t1r: export package checks ===\n');

    % ---- [1] export into the case's generated/ folder ----
    t0 = tic;
    out = solver.exportKinematics(dslFile, execFile);
    assert(strcmp(out.OutputDir, exportedDir), ...
        'Default OutputDir should be <case>/generated, got: %s', out.OutputDir);
    expectedFiles = {'README.md', 'kinematic_metadata.json', ...
        'closure_residual.m', 'closure_residual_jacobian.m', ...
        'fk_tool.m', 'fk_all.m', 'fk_all_matrix.m'};
    for i = 1:numel(expectedFiles)
        assert(exist(fullfile(exportedDir, expectedFiles{i}), 'file') > 0, ...
            'Missing exported file: %s', expectedFiles{i});
    end
    assert(numel(out.Files) == numel(expectedFiles), 'file count');
    % matlabFunction de-ambiguation guard: compiled kernels must not carry
    % the ambiguous `--` token (Code Analyzer decrement look-alike; see
    % localDeambiguateDoubleNegation in solver.exportKinematics).
    for i = 1:numel(out.Files)
        [~, ~, ext] = fileparts(out.Files{i});
        if strcmp(ext, '.m')
            assert(~contains(fileread(out.Files{i}), '--'), ...
                'Exported kernel contains ambiguous `--` token: %s', ...
                out.Files{i});
        end
    end
    fprintf('  [1] package exported: %d files (%.1fs) -> %s\n', ...
        numel(out.Files), toc(t0), exportedDir);

    % ---- live pipeline for cross-checks ----
    e = ir.Expander(dslFile);
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);

    % ---- [2] metadata round-trip ----
    meta = jsondecode(fileread(fullfile(exportedDir, 'kinematic_metadata.json')));
    assert(strcmp(meta.mechanism, e.MechName), 'metadata mechanism');
    assert(strcmp(meta.mode, 'closed_loop'), 'metadata mode');
    assert(strcmp(meta.direction, 'IK'), 'metadata direction');
    assert(strcmp(meta.inputs.kind, 'unknown_joints'), 'metadata input kind');
    assert(meta.inputs.count == 8, 'metadata input count');
    assert(isequal(meta.inputs.names(:).', cs.UnknownNames(:).'), ...
        'metadata input names/order');
    assert(meta.closure.num_cuts == 2, 'metadata cut count');
    assert(meta.closure.num_components == 12, 'metadata component count');
    assert(strcmp(meta.closure.cuts(1).anchor, 'world'), 'metadata cut1 anchor');
    dev = norm(meta.closure.cuts(1).T_bind - cs.ClosureCuts(1).T_bind, 'fro');
    assert(dev < 1e-12, 'T_bind cut1 round-trip dev %.3e', dev);
    dev2 = norm(meta.closure.cuts(2).T_bind - cs.ClosureCuts(2).T_bind, 'fro');
    assert(dev2 < 1e-12, 'T_bind cut2 round-trip dev %.3e', dev2);
    assert(strcmp(meta.task_frame.framing, 'world_tool'), 'metadata task framing');
    devU = norm(meta.task_frame.U - cs.ToolTransform, 'fro');
    assert(devU < 1e-12, 'tool U round-trip dev %.3e', devU);
    assert(numel(meta.frames) == numel(keys(e.Poses)), 'frame table size');
    assert(strcmp(meta.frames(1).field, ...
        matlab.lang.makeValidName(meta.frames(1).node)), 'frame field mapping');
    fprintf('  [2] metadata: inputs/cuts/bindings/tool U/frames round-trip OK\n');

    % ---- load generated kernels ----
    addpath(exportedDir);
    cleanupObj = onCleanup(@() rmpath(exportedDir)); %#ok<NASGU>

    qTest = [0.3; -0.2; 0.1; -0.25; 0.15; -0.1; 0.05; -0.05];
    qZero = zeros(8, 1);

    % ---- [3] residual kernel ----
    rGen = closure_residual(qTest);
    rCs  = cs.evalResidual(qTest);
    assert(all(isfinite(rGen)), 'residual not finite');
    dev = norm(rGen - rCs);
    assert(dev < 1e-9, 'residual dev vs solver %.3e', dev);
    r0 = closure_residual(qZero);
    assert(norm(r0) < 1e-12, 'r(0) = %.3e (expected < 1e-12)', norm(r0));
    fprintf('  [3] closure_residual matches solver (dev %.1e), r(0) = %.1e\n', ...
        dev, norm(r0));

    % ---- [4] jacobian kernel: finite + central FD of the residual kernel ----
    J = closure_residual_jacobian(qTest);
    assert(isequal(size(J), [12 8]), 'J size');
    assert(all(isfinite(J(:))), 'J not finite');
    h = 1e-6;
    fd = zeros(12, 8);
    for k = 1:8
        qp = qTest; qp(k) = qp(k) + h;
        qm = qTest; qm(k) = qm(k) - h;
        fd(:, k) = (closure_residual(qp) - closure_residual(qm)) / (2 * h);
    end
    devJ = max(abs(J(:) - fd(:)));
    assert(devJ < 1e-6, 'J vs FD dev %.3e', devJ);
    fprintf('  [4] closure_residual_jacobian: 12x8 finite, matches FD (%.1e)\n', ...
        devJ);

    % ---- [5] fk_tool kernel ----
    D = fk_tool(qTest);
    Dref = cs.evalToolPose(qTest);
    devD = norm(D - Dref, 'fro');
    assert(devD < 1e-9, 'fk_tool dev vs evalToolPose %.3e', devD);
    devD0 = norm(fk_tool(qZero) - eye(4), 'fro');
    assert(devD0 < 1e-12, 'fk_tool(0) dev from I4 %.3e', devD0);
    fprintf('  [5] fk_tool matches evalToolPose (%.1e); fk_tool(0) = I4 (%.1e)\n', ...
        devD, devD0);

    % ---- [6] fk_all kernel ----
    poses = fk_all(qTest);
    assert(numel(fieldnames(poses)) == numel(keys(e.Poses)), ...
        'fk_all field count');
    qMap = containers.Map(cs.UnknownNames, num2cell(qTest));
    spotNodes = {'manipulator_L.ground', 'manipulator_R.ground', ...
        'joint_1.linkB', 'frame_4.frame_hyper_cube', 'pipette.tip_origin'};
    ref = e.evaluateNumericDirect(qMap, spotNodes);
    worst = 0;
    for i = 1:numel(spotNodes)
        fld = matlab.lang.makeValidName(spotNodes{i});
        dv = norm(poses.(fld) - ref(spotNodes{i}), 'fro');
        worst = max(worst, dv);
    end
    assert(worst < 1e-9, 'fk_all spot dev %.3e', worst);
    dvRoot = norm(poses.pipette_tip_origin - eye(4), 'fro');
    assert(dvRoot < 1e-12, 'fk_all root pose not I4 (dev %.3e)', dvRoot);
    fprintf('  [6] fk_all: %d frames, spot checks OK (worst %.1e)\n', ...
        numel(fieldnames(poses)), worst);

    % ---- [7] README ----
    txt = fileread(fullfile(exportedDir, 'README.md'));
    assert(contains(txt, 'closure_residual') && ...
        contains(txt, 'kinematic_metadata') && ...
        contains(txt, 'solver.exportKinematics'), 'README content');
    fprintf('  [7] README.md documents the package contract\n');

    % ---- [8] task-space metadata（泛化字段：inputs.types/axes + task_space）----
    assert(numel(meta.inputs.types) == meta.inputs.count, 'types length');
    types = reshape(cellstr(meta.inputs.types), 1, []);
    assert(all(strcmp(types(1:6), 'prismatic')) && ...
        all(strcmp(types(7:8), 'revolute')), 'input types');
    assert(isequal(reshape(cellstr(meta.inputs.axes), 1, []), ...
        {'x', 'y', 'z', 'x', 'y', 'z', 'r1', 'r1'}), 'input axis labels');
    assert(strcmp(meta.task_space.mobility, '3t1r'), 'mobility');
    assert(strcmp(meta.task_space.rotation.mode, 'axis'), 'rotation mode');
    aRef = [1 0 1] / sqrt(2);
    devA = norm(meta.task_space.rotation.axis(:).' - aRef);
    assert(meta.task_space.rotation.axis(1) > 0 && devA < 1e-9, ...
        'rotation axis dev %.3e', devA);
    assert(meta.task_space.dims.task == 4 && ...
        meta.task_space.dims.translation == 3 && ...
        meta.task_space.dims.rotation == 1, 'task dims');
    assert(meta.task_space.probe.constraint_rank == 4 && ...
        meta.task_space.probe.manifold_dim == 4, 'probe ranks');
    fprintf(['  [8] task_space: %s, axis [%.3f %.3f %.3f], manifold %d; ' ...
        'axes [%s]\n'], meta.task_space.mobility, ...
        meta.task_space.rotation.axis, meta.task_space.probe.manifold_dim, ...
        strjoin(cellstr(meta.inputs.axes), ' '));
    fprintf('=== ALL CHECKS PASSED ===\n');
end
