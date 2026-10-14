function test_export_kinematics_single_closed_loop()
%TEST_EXPORT_KINEMATICS_SINGLE_CLOSED_LOOP  Model-package export test (L2 loop).
%   Runs solver.exportKinematics on the single-closed-loop case (closed_loop,
%   L2 frame-pair cut auto-derived from the DSL `closed: true` connection)
%   and verifies the exported package:
%     [1] package written to <case>/generated (default OutputDir)
%     [2] metadata: auto closure source, cut/frame-pair fields, root-framed task
%     [3] closure_residual.m matches ClosureSolver.evalResidual; r(0) ≈ 0
%     [4] closure_residual_jacobian.m: finite, matches central FD
%     [5] fk_tool.m = endFrame pose, matches Expander numeric FK
%     [6] fk_all.m: all frames, spot-check vs Expander numeric FK
%
%   Writes (regenerated on every run):
%   specs/dsl/cases/single-closed-loop/generated/ (gitignored artifacts).
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_export_kinematics_single_closed_loop.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    caseDir = fullfile(rootDir, 'specs', 'dsl', 'cases', 'single-closed-loop');
    dslFile = fullfile(caseDir, 'robot_description.yaml');
    execFile = fullfile(caseDir, 'execution-config.yaml');
    exportedDir = fullfile(caseDir, 'generated');

    fprintf('=== test_export_kinematics_single_closed_loop: export package checks ===\n');

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
    assert(strcmp(meta.closure.source, 'auto'), 'metadata closure source');
    assert(strcmp(meta.inputs.kind, 'unknown_joints'), 'metadata input kind');
    assert(meta.inputs.count == 4, 'metadata input count');
    assert(isequal(meta.inputs.names(:).', cs.UnknownNames(:).'), ...
        'metadata input names/order');
    assert(meta.closure.num_cuts == numel(cs.ClosureCuts), 'metadata cut count');
    assert(meta.closure.num_components == cs.NumComponents, 'metadata component count');
    % L2 frame-pair cut: far present, no world anchor, no T_bind
    c1 = meta.closure.cuts(1);
    assert(isempty(c1.anchor) && ~isempty(c1.far), 'L2 cut fields');
    assert(isempty(c1.T_bind), 'L2 cut should carry no T_bind');
    devM = norm(c1.T_mate - cs.ClosureCuts(1).T_mate, 'fro');
    assert(devM < 1e-12, 'T_mate round-trip dev %.3e', devM);
    assert(strcmp(meta.task_frame.framing, 'root'), 'metadata task framing');
    assert(strcmp(meta.task_frame.end_frame, cfg.EndFrame), 'metadata end frame');
    fprintf('  [2] metadata: auto cut (%d comps), frame-pair fields, root framing OK\n', ...
        meta.closure.num_components);

    % ---- load generated kernels ----
    addpath(exportedDir);
    cleanupObj = onCleanup(@() rmpath(exportedDir)); %#ok<NASGU>

    % closed configuration of the parallelogram: AB=+0.3, BD=-0.3, CD=+0.3, AC=-0.3
    qTest = [0.3; -0.3; 0.3; -0.3];
    qZero = zeros(4, 1);

    % ---- [3] residual kernel ----
    rGen = closure_residual(qTest);
    rCs  = cs.evalResidual(qTest);
    assert(all(isfinite(rGen)), 'residual not finite');
    dev = norm(rGen - rCs);
    assert(dev < 1e-9, 'residual dev vs solver %.3e', dev);
    rClosed = closure_residual(qTest);
    assert(norm(rClosed) < 1e-9, ...
        'closed parallelogram config should satisfy r ≈ 0 (|r| = %.3e)', norm(rClosed));
    r0 = closure_residual(qZero);
    assert(norm(r0) < 1e-12, 'r(0) = %.3e (expected < 1e-12)', norm(r0));
    fprintf('  [3] closure_residual matches solver (dev %.1e), r(closed cfg) = %.1e\n', ...
        dev, norm(rClosed));

    % ---- [4] jacobian kernel ----
    J = closure_residual_jacobian(qTest);
    assert(isequal(size(J), [meta.closure.num_components 4]), 'J size');
    assert(all(isfinite(J(:))), 'J not finite');
    h = 1e-6;
    fd = zeros(size(J));
    for k = 1:4
        qp = qTest; qp(k) = qp(k) + h;
        qm = qTest; qm(k) = qm(k) - h;
        fd(:, k) = (closure_residual(qp) - closure_residual(qm)) / (2 * h);
    end
    devJ = max(abs(J(:) - fd(:)));
    assert(devJ < 1e-6, 'J vs FD dev %.3e', devJ);
    fprintf('  [4] closure_residual_jacobian: %dx%d finite, matches FD (%.1e)\n', ...
        size(J, 1), size(J, 2), devJ);

    % ---- [5] fk_tool kernel = endFrame pose (root framing) ----
    D = fk_tool(qTest);
    qMap = containers.Map(cs.UnknownNames, num2cell(qTest));
    refEnd = e.evaluateNumericDirect(qMap, {cfg.EndFrame});
    devD = norm(D - refEnd(cfg.EndFrame), 'fro');
    assert(devD < 1e-9, 'fk_tool dev vs Expander FK %.3e', devD);
    fprintf('  [5] fk_tool = endFrame FK (dev %.1e)\n', devD);

    % ---- [6] fk_all kernel ----
    poses = fk_all(qTest);
    assert(numel(fieldnames(poses)) == numel(keys(e.Poses)), ...
        'fk_all field count');
    spotNodes = {'frame_link_A2.frame_hyper_cube', 'joint_CD.linkB', ...
        'frame_link_D2.frame_hyper_cube', 'frame_link_C1.faceZMinus'};
    ref = e.evaluateNumericDirect(qMap, spotNodes);
    worst = 0;
    for i = 1:numel(spotNodes)
        fld = matlab.lang.makeValidName(spotNodes{i});
        dv = norm(poses.(fld) - ref(spotNodes{i}), 'fro');
        worst = max(worst, dv);
    end
    assert(worst < 1e-9, 'fk_all spot dev %.3e', worst);
    fprintf('  [6] fk_all: %d frames, spot checks OK (worst %.1e)\n', ...
        numel(fieldnames(poses)), worst);

    fprintf('=== ALL CHECKS PASSED ===\n');
end
