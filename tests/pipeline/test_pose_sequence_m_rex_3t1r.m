function test_pose_sequence_m_rex_3t1r()
%TEST_POSE_SEQUENCE_M_REX_3T1R  Pose-sequence product contract checks (A.5.2.0).
%   Verifies solver.buildPoseSequence / viz.sceneSpec on the mrex-3t1r_001
%   pipeline — the compute-layer product consumed by the pure renderer
%   (viz.play):
%     [1] root anchor: frame count, full frame set (all poses + root),
%         per-frame poses exactly equal to direct evaluateNumericDirect,
%         root pose pinned at I₄
%     [2] world anchor: T_bind = zero-pose FK ground pose; conjugation
%         contract seqW(X,k) = E_k · seqR(X,k) with
%         E_k = T_bind · inv(T_root<-ground(q_k)); Trail = root world path;
%         zero-frame branch gap ≈ 0; root frame pose = E_k
%     [3] error behavior: column/varNames mismatch, unknown variable,
%         bad anchor value
%     [4] sceneSpec: static snapshot fields + characteristic scale
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_pose_sequence_m_rex_3t1r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    dslFile = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex-3t1r_001', ...
        'robot_description.yaml');

    fprintf('=== test_pose_sequence_m_rex_3t1r: pose-sequence contract ===\n');

    % ---- pipeline + trajectory ----
    e = ir.Expander(dslFile);
    e.compilePoseFunctions();

    names = {'manipulator_L.dx', 'manipulator_L.dy', 'manipulator_L.dz', ...
             'manipulator_R.dx', 'manipulator_R.dy', 'manipulator_R.dz', ...
             'joint_1.q', 'joint_2.q'};
    Q = [ 0,   0,   0,   0,   0,    0,  0,     0;      % zero frame
          1,   2,   3,  -1,   0.5, -2,  0.02, -0.02;   % arbitrary frames
          5,   0,  -3,   2,  -2,    1, -0.05,  0.05];

    % ---- [1] root anchor: shapes + exactness ----
    seqR = solver.buildPoseSequence(e, Q, names);
    assert(strcmp(seqR.Anchor, 'root'), 'root fixture anchor');
    assert(numel(seqR.Frames) == 3, 'root fixture frame count');

    allNames = keys(e.Poses);
    expectedNames = sort(unique([allNames(:).', {seqR.RootFrame}]));
    assert(isequal(seqR.FrameNames, expectedNames), ...
        'FrameNames should cover all poses + the FK root');

    devDirect = 0; devRoot = 0;
    for k = 1:3
        jvMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
        for j = 1:numel(names); jvMap(names{j}) = Q(k, j); end
        posesDir = e.evaluateNumericDirect(jvMap);
        ks = keys(posesDir);
        for kk = 1:numel(ks)
            devDirect = max(devDirect, ...
                norm(seqR.Frames{k}(ks{kk}) - posesDir(ks{kk}), 'fro'));
        end
        devRoot = max(devRoot, ...
            norm(seqR.Frames{k}(seqR.RootFrame) - eye(4), 'fro'));
    end
    assert(devDirect < 1e-15, ...
        'sequence poses must equal direct evaluation (dev %.3e)', devDirect);
    assert(devRoot < 1e-12, 'root pose should be pinned at I4 (dev %.3e)', devRoot);
    fprintf(['  [1] root anchor: %d frames, %d frames/frame; direct-eval dev = %.2e, ' ...
        'root pinned (dev = %.2e)\n'], numel(seqR.Frames), numel(seqR.FrameNames), ...
        devDirect, devRoot);

    % ---- [2] world anchor: binding, conjugation, trail, branch gap ----
    seqW = solver.buildPoseSequence(e, Q, names, 'Anchor', 'world');
    assert(strcmp(seqW.Anchor, 'world'), 'world fixture anchor');
    assert(isequal(seqW.FrameNames, seqR.FrameNames), 'frame sets must match');
    assert(numel(seqW.Tbind) == 2 && numel(seqW.GroundFrames) == 2, ...
        'expected 2 ground bindings for m-rex');

    % T_bind = zero-pose FK ground pose (independent evaluation)
    zeroPoses = e.evaluateNumericDirect(containers.Map);
    devB = 0;
    for g = 1:numel(seqW.GroundFrames)
        devB = max(devB, norm(seqW.Tbind{g} - zeroPoses(seqW.GroundFrames{g}), 'fro'));
    end
    assert(devB < 1e-15, 'T_bind vs zero-pose FK dev %.3e', devB);

    % conjugation contract: seqW(X,k) = E_k · seqR(X,k), Trail = E_k(1:3,4)
    g1 = seqW.GroundFrames{1};
    devC = 0; devTrail = 0;
    for k = 1:3
        E = seqW.Tbind{1} * core.RigidBodyMath.invT(seqR.Frames{k}(g1));
        for kk = 1:numel(seqW.FrameNames)
            X = seqW.FrameNames{kk};
            devC = max(devC, norm(seqW.Frames{k}(X) - E * seqR.Frames{k}(X), 'fro'));
        end
        devTrail = max(devTrail, norm(seqW.Trail(:, k) - E(1:3, 4)));
    end
    assert(devC < 1e-12, 'world conjugation contract dev %.3e', devC);
    assert(devTrail < 1e-12, 'Trail dev %.3e', devTrail);

    % root frame pose in world anchor = E_k (tool world pose)
    E0 = seqW.Tbind{1} * core.RigidBodyMath.invT(seqR.Frames{1}(g1));
    devRootW = norm(seqW.Frames{1}(seqW.RootFrame) - E0, 'fro');
    assert(devRootW < 1e-12, 'world root pose should equal E (dev %.3e)', devRootW);

    % zero-frame branch gap ≈ 0 (closure holds at q = 0)
    g2 = seqW.GroundFrames{2};
    Eg0 = seqW.Tbind{2} * core.RigidBodyMath.invT(seqR.Frames{1}(g2));
    gap0 = norm(Eg0 - E0, 'fro');
    assert(gap0 < 1e-9, 'zero-frame branch gap %.3e', gap0);
    fprintf(['  [2] world anchor: T_bind dev = %.2e, conjugation dev = %.2e, ' ...
        'trail dev = %.2e, zero-frame gap = %.2e\n'], devB, devC, devTrail, gap0);

    % ---- [3] error behavior ----
    localExpectError(@() solver.buildPoseSequence(e, Q, names(1:7)), ...
        'solver:buildPoseSequence:trajectoryMismatch', 'column/varNames mismatch');
    localExpectError(@() solver.buildPoseSequence(e, Q, ...
        [names(1:7), {'joint_9.q'}]), ...
        'solver:buildPoseSequence:varNotFound', 'unknown variable');
    localExpectError(@() solver.buildPoseSequence(e, Q, names, 'Anchor', 'foo'), ...
        'solver:buildPoseSequence:badAnchor', 'bad anchor value');
    fprintf('  [3] error behavior: mismatch / unknown var / bad anchor all rejected\n');

    % ---- [4] sceneSpec: static snapshot ----
    scene = viz.sceneSpec(e);
    assert(strcmp(scene.MechName, seqR.MechName), 'scene mech name');
    assert(scene.NInst == numel(e.Instances) && scene.NConns > 0, ...
        'scene instance/connection counts');
    assert(scene.L > 0, 'characteristic scale must be positive');
    assert(scene.GeomCache.Count >= 1, 'geometry cache should be non-empty');
    fprintf(['  [4] sceneSpec: %d instances, %d connections, L = %.1f mm, ' ...
        '%d cached geometries\n'], scene.NInst, scene.NConns, scene.L, ...
        scene.GeomCache.Count);

    fprintf('\n=== test_pose_sequence_m_rex_3t1r: all checks passed ===\n');
end

%% ---- local helpers ----

function localExpectError(fcn, expectedId, label)
%LOCALEXPECTERROR  Assert that FCN raises an error with the expected id.
    try
        fcn();
        error('poseSeqCheck:noError', '[%s] no error raised (expected "%s").', ...
            label, expectedId);
    catch ME
        if strcmp(ME.identifier, 'poseSeqCheck:noError')
            fprintf(2, '  [FAIL] %s: no error raised\n', label);
            rethrow(ME);
        end
        if ~strcmp(ME.identifier, expectedId)
            fprintf(2, '  [FAIL] %s: expected "%s", got "%s": %s\n', ...
                label, expectedId, ME.identifier, ME.message);
            error('poseSeqCheck:wrongError', '%s', label);
        end
        fprintf('  [3] %s -> %s OK\n', label, expectedId);
    end
end
