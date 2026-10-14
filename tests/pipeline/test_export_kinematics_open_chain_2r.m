function test_export_kinematics_open_chain_2r()
%TEST_EXPORT_KINEMATICS_OPEN_CHAIN_2R  Model-package export test (open loop).
%   Runs solver.exportKinematics on the open-chain-2r case (open_loop mode:
%   FK-only package, inputs = actuated joints) and verifies:
%     [1] package written to <case>/generated; no residual/jacobian files
%     [2] metadata: open_loop / FK / actuated_joints input order, no closure
%     [3] fk_tool.m = endFrame FK, matches Expander numeric FK at test values
%     [4] fk_all.m: all frames, spot-check vs Expander numeric FK
%
%   Writes (regenerated on every run):
%   specs/dsl/cases/open-chain-2r/generated/ (gitignored artifacts).
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_export_kinematics_open_chain_2r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    caseDir = fullfile(rootDir, 'specs', 'dsl', 'cases', 'open-chain-2r');
    dslFile = fullfile(caseDir, 'robot_description.yaml');
    execFile = fullfile(caseDir, 'execution-config.yaml');
    exportedDir = fullfile(caseDir, 'generated');

    fprintf('=== test_export_kinematics_open_chain_2r: export package checks ===\n');

    % ---- [1] export into the case's generated/ folder ----
    t0 = tic;
    out = solver.exportKinematics(dslFile, execFile);
    assert(strcmp(out.OutputDir, exportedDir), ...
        'Default OutputDir should be <case>/generated, got: %s', out.OutputDir);
    expectedFiles = {'README.md', 'kinematic_metadata.json', ...
        'fk_tool.m', 'fk_all.m', 'fk_all_matrix.m'};
    for i = 1:numel(expectedFiles)
        assert(exist(fullfile(exportedDir, expectedFiles{i}), 'file') > 0, ...
            'Missing exported file: %s', expectedFiles{i});
    end
    % open_loop: FK-only package — closed-loop kernels must not exist
    assert(exist(fullfile(exportedDir, 'closure_residual.m'), 'file') == 0, ...
        'open_loop export must not write closure_residual.m');
    assert(exist(fullfile(exportedDir, 'closure_residual_jacobian.m'), 'file') == 0, ...
        'open_loop export must not write closure_residual_jacobian.m');
    fprintf('  [1] FK-only package exported: %d files (%.1fs) -> %s\n', ...
        numel(out.Files), toc(t0), exportedDir);

    % ---- live pipeline for cross-checks ----
    e = ir.Expander(dslFile);
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);

    % ---- [2] metadata ----
    meta = jsondecode(fileread(fullfile(exportedDir, 'kinematic_metadata.json')));
    assert(strcmp(meta.mechanism, e.MechName), 'metadata mechanism');
    assert(strcmp(meta.mode, 'open_loop'), 'metadata mode');
    assert(strcmp(meta.direction, 'FK'), 'metadata direction');
    assert(strcmp(meta.inputs.kind, 'actuated_joints'), 'metadata input kind');
    assert(meta.inputs.count == 2, 'metadata input count');
    assert(isequal(meta.inputs.names(:).', cfg.ActuatedJoints(:).'), ...
        'metadata input names/order');
    assert(~isfield(meta, 'closure'), 'open_loop metadata must not have closure');
    assert(strcmp(meta.task_frame.framing, 'root'), 'metadata task framing');
    assert(strcmp(meta.task_frame.end_frame, cfg.EndFrame), 'metadata end frame');
    fprintf('  [2] metadata: open_loop/FK, inputs = {%s}, no closure block\n', ...
        strjoin(meta.inputs.names, ', '));

    % ---- load generated kernels ----
    addpath(exportedDir);
    cleanupObj = onCleanup(@() rmpath(exportedDir)); %#ok<NASGU>

    qTest = [0.35; -0.45];

    % ---- [3] fk_tool kernel = endFrame FK ----
    D = fk_tool(qTest);
    qMap = containers.Map(cfg.ActuatedJoints, num2cell(qTest));
    refEnd = e.evaluateNumericDirect(qMap, {cfg.EndFrame});
    devD = norm(D - refEnd(cfg.EndFrame), 'fro');
    assert(devD < 1e-9, 'fk_tool dev vs Expander FK %.3e', devD);
    % FK sanity: rotation part orthonormal, planarity of a 2R arm not assumed
    R = D(1:3, 1:3);
    assert(abs(det(R) - 1) < 1e-9, 'fk_tool rotation det');
    assert(norm(R * R' - eye(3), 'fro') < 1e-9, 'fk_tool rotation orthonormality');
    fprintf('  [3] fk_tool = endFrame FK (dev %.1e)\n', devD);

    % ---- [4] fk_all kernel ----
    poses = fk_all(qTest);
    assert(numel(fieldnames(poses)) == numel(keys(e.Poses)), ...
        'fk_all field count');
    spotNodes = {'joint1.linkB', 'joint2.linkB', ...
        'frame1.frame_hyper_cube', 'frame2.frame_hyper_cube'};
    ref = e.evaluateNumericDirect(qMap, spotNodes);
    worst = 0;
    for i = 1:numel(spotNodes)
        fld = matlab.lang.makeValidName(spotNodes{i});
        dv = norm(poses.(fld) - ref(spotNodes{i}), 'fro');
        worst = max(worst, dv);
    end
    assert(worst < 1e-9, 'fk_all spot dev %.3e', worst);
    % fk_all entry for the end frame equals fk_tool
    dvEnd = norm(poses.frame2_frame_hyper_cube - D, 'fro');
    assert(dvEnd < 1e-12, 'fk_all endFrame dev vs fk_tool %.3e', dvEnd);
    fprintf('  [4] fk_all: %d frames, spot checks OK (worst %.1e)\n', ...
        numel(fieldnames(poses)), worst);

    fprintf('=== ALL CHECKS PASSED ===\n');
end
