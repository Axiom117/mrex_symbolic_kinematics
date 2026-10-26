function test_kinematics_runtime_m_rex_3t2r_v00()
%TEST_KINEMATICS_RUNTIME_M_REX_3T2R_V00  P0 runtime contract smoke test.
%   Exercises solver.KinematicsRuntime on the v00 3T2R package:
%     [1] metadata roles / U0 / C
%     [2] fkPackage / fk with the default tool frame
%     [3] target-aware ikPackage (nonzero target)
%     [4] driver-only fkDrivers (passive-joint solve)
%     [5] runtime tool frame Urel (translation + rotation)
%     [6] solver.exportDeployment -> namespaced smk.KinematicsRuntime
%
%   Requires: Symbolic Math Toolbox only when the package must be generated.
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_kinematics_runtime_m_rex_3t2r_v00.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    caseDir = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex_3t2r_v00');
    dslFile = fullfile(caseDir, 'robot_description.yaml');
    execFile = fullfile(caseDir, 'execution-config.yaml');
    pkgDir = fullfile(caseDir, 'generated');

    fprintf('=== test_kinematics_runtime_m_rex_3t2r_v00 ===\n');

    % ---- [0] ensure the package is current ----
    metaFile = fullfile(pkgDir, 'kinematic_metadata.json');
    needsExport = exist(metaFile, 'file') ~= 2;
    if ~needsExport
        md0 = jsondecode(fileread(metaFile));
        needsExport = ~isfield(md0.inputs, 'actuated_indices') || ...
            ~isfield(md0, 'runtime');
    end
    if needsExport
        solver.exportKinematics(dslFile, execFile);
    end

    rt = solver.KinematicsRuntime(pkgDir);

    % ---- [1] metadata roles / task frame ----
    assert(isequal(rt.ActuatedIndices, 1:6), 'actuated indices');
    assert(isequal(rt.PassiveIndices, 7:10), 'passive indices');
    assert(norm(rt.C - eye(4), 'fro') < 1e-12, 'C should be I for tip-rooted v00');
    Uref = localTO(eye(3), [0; 0; -1]);
    assert(norm(rt.U0 - Uref, 'fro') < 1e-9, ...
        'U0 should be tip-aligned with a -1 mm Z offset');
    fprintf('  [1] roles 6+4, U0=Trans(0,0,-1 mm), C=I\n');

    % ---- [2] package FK / default tool FK ----
    qZero = zeros(rt.NQ, 1);
    D0 = rt.fkPackage(qZero);
    assert(norm(D0 - eye(4), 'fro') < 1e-9, 'fkPackage(0) != I4');
    T0 = rt.fk(qZero);
    assert(norm(T0 - Uref, 'fro') < 1e-9, 'fk(0) != U0');
    fprintf('  [2] fkPackage(0)=I4, fk(0)=U0 (dev %.1e)\n', ...
        norm(T0 - Uref, 'fro'));

    % ---- [3] target-aware ikPackage ----
    Dtarget = eye(4);
    Dtarget(1, 4) = 2;   % package task frame +X 2 mm
    [q, ok, info] = rt.ikPackage(Dtarget);
    assert(ok, 'ikPackage did not converge: maxAbs %.3e', info.maxAbs);
    devD = norm(rt.fkPackage(q) - Dtarget, 'fro');
    assert(devD < 1e-6, 'fkPackage(ikPackage(D)) dev %.3e', devD);
    fprintf('  [3] ikPackage(+2 mm X): ok, maxAbs=%.1e, FK dev=%.1e\n', ...
        info.maxAbs, devD);

    % ---- [4] driver-only FK (passive joints solved from loop closure) ----
    % Use a fresh runtime so the passive-joint solve starts from q=0 rather
    % than the IK result already stored in rt.LastQ.
    d = q(rt.ActuatedIndices);
    rtFresh = solver.KinematicsRuntime(pkgDir);
    [Td, qd, okd, infod] = rtFresh.fkDrivers(d);
    assert(okd, 'fkDrivers did not converge: loop %.3e', infod.passive_norm);
    assert(norm(qd - q) < 1e-6, 'fkDrivers passive q dev %.3e', norm(qd - q));
    assert(norm(Td - rt.fk(q)) < 1e-6, ...
        'fkDrivers pose dev %.3e', norm(Td - rt.fk(q)));
    fprintf(['  [4] fkDrivers (from q=0): ok, loop=%.1e, passive-q dev=%.1e, ' ...
        'pose dev=%.1e\n'], infod.passive_norm, norm(qd - q), ...
        norm(Td - rt.fk(q)));

    % ---- [5] runtime tool frame (translation + rotation) ----
    Urel = localTO(localRy(30 * pi / 180), [1; 2; 3]);
    Ttarget = rt.fk(q, Urel);
    assert(norm(rt.fk(qZero, Urel) - Urel, 'fro') < 1e-9, ...
        'fk(0,Urel) should equal Urel');
    [q2, ok2, info2] = rt.ik(Ttarget, Urel);
    assert(ok2, 'ik(Urel) did not converge: maxAbs %.3e', info2.maxAbs);
    devT = norm(rt.fk(q2, Urel) - Ttarget, 'fro');
    assert(devT < 1e-6, 'runtime-tool FK/IK round trip dev %.3e', devT);
    [d2, qint2, ok3, info3, q3] = rt.ikDrivers(Ttarget, Urel);
    assert(ok3, 'ikDrivers did not converge: maxAbs %.3e', info3.maxAbs);
    assert(isequal(d2, q3(rt.ActuatedIndices)), 'driver split mismatch');
    assert(isequal(qint2, q3(rt.PassiveIndices)), 'passive split mismatch');
    fprintf(['  [5] Urel round trip: ok, maxAbs=%.1e, pose dev=%.1e; ' ...
        'ikDrivers split OK\n'], info2.maxAbs, devT);

    % ---- [6] namespaced deployment / self-contained runtime ----
    dep = solver.exportDeployment(pkgDir);
    assert(isfield(dep.Manifest, 'kernel_package'), 'manifest kernel_package');
    assert(strcmp(dep.Manifest.wrapper_ik, ...
        ['IK_' dep.Manifest.kernel_package]), 'manifest IK wrapper name');
    assert(strcmp(dep.Manifest.wrapper_fk, ...
        ['FK_' dep.Manifest.kernel_package]), 'manifest FK wrapper name');
    pkgKernelDir = fullfile(pkgDir, ['+' dep.Manifest.kernel_package]);
    assert(exist(fullfile(pkgKernelDir, 'fk_tool.m'), 'file') == 2, ...
        'namespaced fk_tool.m missing');
    assert(exist(fullfile(pkgDir, '+smk', 'KinematicsRuntime.m'), 'file') == 2, ...
        'deployed runtime class missing');
    addpath(pkgDir);
    rtDep = smk.KinematicsRuntime(pkgDir);
    assert(strcmp(rtDep.KernelPackage, dep.Manifest.kernel_package), ...
        'deployed runtime did not select namespaced kernels');
    devDep = norm(rtDep.fkPackage(qZero) - eye(4), 'fro');
    assert(devDep < 1e-9, 'deployed runtime fkPackage(0) dev %.3e', devDep);
    fprintf(['  [6] deployment: +%s kernels, smk.KinematicsRuntime, ' ...
        'fkPackage(0) dev=%.1e\n'], rtDep.KernelPackage, devDep);

    % ---- [7] batch trajectory IK ----
    nTraj = 4;
    Dtraj = repmat(eye(4), 1, 1, nTraj);
    Ttraj = repmat(eye(4), 1, 1, nTraj);
    for k = 1:nTraj
        Dtraj(1, 4, k) = k - 1;                 % package +X 0..3 mm
        Ttraj(:, :, k) = rt.U0 * Dtraj(:, :, k);
    end
    [qTraj, dTraj, okTraj, infoTraj] = rt.ikTrajectory( ...
        Ttraj, rt.Urel0, zeros(rt.NQ, 1));
    assert(all(okTraj), 'ikTrajectory failed: %s', mat2str(okTraj.'));
    assert(isequal(size(qTraj), [nTraj rt.NQ]), 'qTraj size');
    assert(isequal(size(dTraj), [nTraj numel(rt.ActuatedIndices)]), 'dTraj size');
    devTraj = 0;
    for k = 1:nTraj
        devTraj = max(devTraj, ...
            norm(rt.fkPackage(qTraj(k, :).') - Dtraj(:, :, k), 'fro'));
    end
    assert(devTraj < 1e-6, 'trajectory FK dev %.3e', devTraj);
    assert(all([infoTraj.feasible]), 'trajectory feasibility flags');
    fprintf(['  [7] ikTrajectory: %d frames, maxAbs=%.1e, maxJump=%.2f, ' ...
        'FK dev=%.1e\n'], nTraj, max([infoTraj.maxAbs]), ...
        max([infoTraj.branchJump]), devTraj);

    % ---- [8] runtime hard limits ----
    lower = [-20; -20; -30; -20; -20; -30; ...
             -pi; -pi; -pi; -pi];
    upper = -lower;
    rt.setLimits(lower, upper);
    [~, ~, okLim, infoLim] = rt.ikTrajectory( ...
        Ttraj, rt.Urel0, zeros(rt.NQ, 1));
    assert(all(okLim), 'ikTrajectory with limits failed');
    assert(all([infoLim.feasible]), 'limit feasibility flags');
    assert(max([infoLim.utilization]) <= 1, 'utilization exceeds 1');
    rt.clearLimits();
    fprintf(['  [8] limits: ok, maxUtil=%.2f, all feasible\n'], ...
        max([infoLim.utilization]));

    % ---- [9] exporter Limits option ----
    tmpDir = tempname;
    mkdir(tmpDir);
    outL = solver.exportKinematics(dslFile, execFile, ...
        'OutputDir', tmpDir, ...
        'Limits', struct('lower', lower, 'upper', upper));
    mdL = jsondecode(fileread(fullfile(outL.OutputDir, ...
        'kinematic_metadata.json')));
    assert(isequal(mdL.inputs.lower(:), lower), 'metadata lower limits');
    assert(isequal(mdL.inputs.upper(:), upper), 'metadata upper limits');
    rmdir(tmpDir, 's');
    fprintf('  [9] exportKinematics Limits metadata OK\n');

    % ---- [10] generated top-level wrappers ----
    ikName = ['IK_' dep.Manifest.kernel_package];
    fkName = ['FK_' dep.Manifest.kernel_package];
    assert(exist(fullfile(pkgDir, [ikName '.m']), 'file') == 2, ...
        'IK wrapper missing');
    assert(exist(fullfile(pkgDir, [fkName '.m']), 'file') == 2, ...
        'FK wrapper missing');
    Ttest = Ttraj(:, :, 2);
    [qw, okw] = feval(ikName, Ttest, rt.Urel0);
    Tw = feval(fkName, qw, rt.Urel0);
    assert(okw, 'IK wrapper did not converge');
    assert(norm(Tw - Ttest, 'fro') < 1e-6, ...
        'wrapper FK/IK dev %.3e', norm(Tw - Ttest, 'fro'));
    fprintf(['  [10] wrappers: %s / %s, round-trip dev=%.1e\n'], ...
        ikName, fkName, norm(Tw - Ttest, 'fro'));

    fprintf('=== ALL CHECKS PASSED ===\n');
end

function R = localRy(a)
    R = [cos(a), 0, sin(a); 0, 1, 0; -sin(a), 0, cos(a)];
end

function T = localTO(R, p)
    T = [R, p(:); 0 0 0 1];
end
