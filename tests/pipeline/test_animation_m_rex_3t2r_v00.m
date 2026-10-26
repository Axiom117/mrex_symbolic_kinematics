function test_animation_m_rex_3t2r_v00()
%TEST_ANIMATION_M_REX_3T2R_V00  Layered pipeline + tool-frame animation (mrex_3t2r_v00).
%   Same end-to-end flow as test_animation_m_rex_3t2r.m, applied to the
%   stage-C base assembly mrex_3t2r_v00.  The tool frame ROLLS on a sphere —
%   the tip rotation sweeps the exact two-parameter family
%       R = Rz(-q1) · Rx(-q2)
%   — while the tip position follows a small 3D path (XY circle + z bob,
%   covering all three translation freedoms).  The joint trajectory is built
%   in closed form (no IK, no fmincon):
%       joint_3.q = -joint_1.q;  joint_4.q =  joint_2.q   (v00 closure)
%       dL = R_bind' * (p_target - p_swingL(q1, q2))
%       dR = R_bind' * (p_target - p_swingR(q3, q4))
%   where R_bind = rot(P_L) = rot(P_R) is the constant rotation shared by the
%   zero-pose ground bindings, and p_swingL/R are the tip world positions with
%   the corresponding roll deflected and both Manipulators at zero (evaluated
%   via the compiled pose functions).
%
%   NOTE — v00 differs from mrex_3t2r_001 in the platform-side Joint 2
%   mounting (roll 1 instead of none), so the mirrored relation q3 = -q2,
%   q4 = -q1 does NOT close this assembly.  The closed form here was verified
%   numerically (branch rotation dev < 1e-15 over the demo envelope).
%
%   Checks:
%     [1] symbolic pipeline + bindings (rot(P_L) == rot(P_R))
%     [2] closed form built: rotation-family exactness on BOTH branches +
%         travel envelope (10/10/15 mm) + tool-axis sweep on the unit sphere
%     [3] solver config: v00 execution config resolves U = Trans(0,0,-1 mm)
%     [4] compute layer: solver.buildPoseSequence('Anchor','tool') — tip path
%         = U⁻¹·D_k, zero pose shows the +1 mm Z tool offset, grounds static,
%         branch gap ~1e-14
%     [5] solver cross-check: ClosureSolver residual (setTipTarget) and
%         evalToolPose (tool-frame reporting) at sampled frames
%     [6] render layer: viz.sceneSpec + viz.play with the tool triad attached
%         to the root (ExtraFrames)
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_animation_m_rex_3t2r_v00.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    % ---- demo trajectory parameters (tool frame rolling on a sphere) ----
    rollA1Deg = 9;     % q1 (Rz roll) lissajous amplitude    [deg]
    rollA2Deg = 8;     % q2 (Rx roll) lissajous amplitude    [deg]
    RC        = 3;     % XY circle radius                    [mm]
    zAmp      = 1.2;   % z bob amplitude (3rd translation)   [mm]
                       % (9/8 deg + r=3 + z bob keeps the manip
                       %  travel within the ±10/±10/±15 mm envelope)
    nFrames   = 60;    % frames per loop
    fps       = 30;    % playback speed

    fprintf('=== test_animation_m_rex_3t2r_v00 (stage-C base + tool-frame sphere roll) ===\n');
    fprintf(['Demo: roll q1,q2 = ±%d, ±%d deg (lissajous; q3=-q1, q4=q2) + ' ...
        'XY circle r=%.1f mm + z bob ±%.1f mm, %d frames @ %d fps\n'], ...
        rollA1Deg, rollA2Deg, RC, zAmp, nFrames, fps);

    dslFile  = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex_3t2r_v00', ...
        'robot_description.yaml');
    execFile = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex_3t2r_v00', ...
        'execution-config.yaml');

    % ---- [1] symbolic pipeline + zero-pose bindings ----
    e = ir.Expander(dslFile);
    e.compilePoseFunctions();
    names = {'manipulator_1.dx', 'manipulator_1.dy', 'manipulator_1.dz', ...
             'manipulator_2.dx', 'manipulator_2.dy', 'manipulator_2.dz', ...
             'joint_1.q', 'joint_2.q', 'joint_3.q', 'joint_4.q'};

    zeroMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
    gp = e.evaluateNumericDirect(zeroMap, ...
        {'manipulator_1.ground', 'manipulator_2.ground'});
    P_L = gp('manipulator_1.ground');
    P_R = gp('manipulator_2.ground');
    R_bind = P_L(1:3, 1:3);
    rotDev = norm(R_bind - P_R(1:3, 1:3));
    assert(rotDev < 1e-12, 'ground binding rotations differ (%.3e).', rotDev);
    fprintf('  [1] pipeline OK; rot(P_L) == rot(P_R) (dev %.1e)\n', rotDev);

    % ---- [2] closed-form demo trajectory (Q + tip targets D_k) ----
    D = cell(1, nFrames);
    Q = zeros(nFrames, numel(names));
    maxFamDev = 0;   % exactness of R_tip == Rz(-q1)·Rx(-q2), both branches
    axTilt    = 0;   % tool-axis sweep angle on the unit sphere
    for k = 1:nFrames
        t = (k - 1) / (nFrames - 1);
        q1 = deg2rad(rollA1Deg) * sin(2 * pi * t);
        q2 = deg2rad(rollA2Deg) * sin(4 * pi * t);
        q3 = -q1;  q4 = q2;                     % v00 closure (same-order, one flip)
        p = [RC * cos(2 * pi * t); RC * sin(2 * pi * t); zAmp * sin(4 * pi * t)];

        % swing tip positions with the roll deflected, Manipulators at zero
        mL = containers.Map('KeyType', 'char', 'ValueType', 'double');
        mL('joint_1.q') = q1;  mL('joint_2.q') = q2;
        gL = e.evaluateNumericDirect(mL, {'manipulator_1.ground'});
        T_swL = P_L * core.RigidBodyMath.invT(gL('manipulator_1.ground'));

        mR = containers.Map('KeyType', 'char', 'ValueType', 'double');
        mR('joint_3.q') = q3;  mR('joint_4.q') = q4;
        gR = e.evaluateNumericDirect(mR, {'manipulator_2.ground'});
        T_swR = P_R * core.RigidBodyMath.invT(gR('manipulator_2.ground'));

        % constant-rotation compensation of the swing displacement
        dL = R_bind' * (p - T_swL(1:3, 4));
        dR = R_bind' * (p - T_swR(1:3, 4));

        R_tip = localRz(-q1) * localRx(-q2);
        D{k} = [R_tip, p; 0 0 0 1];
        Q(k, :) = [dL(:).', dR(:).', q1, q2, q3, q4];

        % both branches must reproduce the family rotation exactly
        maxFamDev = max(maxFamDev, ...
            max(norm(T_swL(1:3, 1:3) - R_tip, 'fro'), ...
                norm(T_swR(1:3, 1:3) - R_tip, 'fro')));
        % axis direction of the tip frame on the unit sphere
        axTilt = max(axTilt, acosd(min(1, abs(R_tip(3, 3)))));
    end

    % manipulator travel envelope (real stroke ±10 mm in X/Y, ±15 mm in Z)
    trav = abs(Q(:, 1:6));
    mX = max([trav(:, 1); trav(:, 4)]);
    mY = max([trav(:, 2); trav(:, 5)]);
    mZ = max([trav(:, 3); trav(:, 6)]);
    assert(mX <= 10 && mY <= 10 && mZ <= 15, ...
        'manipulator travel envelope exceeded (%.2f/%.2f/%.2f).', mX, mY, mZ);
    assert(maxFamDev < 1e-9, 'rotation-family exactness deviates (%.3e).', maxFamDev);
    fprintf(['  [2] closed form built: family dev = %.1e (both branches); ' ...
        'max travel |dx|=%.2f |dy|=%.2f |dz|=%.2f mm (limits 10/10/15); ' ...
        'tool-axis sweep <= %.1f deg\n'], maxFamDev, mX, mY, mZ, axTilt);

    % ---- [3] solver config: case tool frame resolves to U = Trans(0,0,-1) ----
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
    assert(numel(cs.UnknownSyms) == 10, ...
        'expected 10 unknowns (6 manip + 4 hinge), got %d.', numel(cs.UnknownSyms));
    U = cs.ToolTransform;
    Uref = localTO(eye(3), [0; 0; -1]);
    devU = norm(U - Uref, 'fro');
    assert(devU < 1e-12, ...
        'case tool frame should resolve to Trans(0,0,-1 mm), dev %.3e', devU);
    Uinv = core.RigidBodyMath.invT(U);
    fprintf('  [3] tool frame resolved: U = Trans(0,0,-1 mm) (dev %.2e)\n', devU);

    % ---- [4] compute layer: tool-world pose sequence ----
    seq = solver.buildPoseSequence(e, Q, names, 'Anchor', 'tool', ...
        'ToolTransform', U);
    assert(strcmp(seq.Anchor, 'tool') && numel(seq.Frames) == nFrames, ...
        'tool-anchored sequence shape mismatch');

    devTip = 0; devTrail = 0; devG = 0;
    g1 = seq.GroundFrames{1};
    for k = 1:nFrames
        Ek = Uinv * D{k};
        devTip = max(devTip, ...
            norm(seq.Frames{k}(seq.RootFrame) - Ek, 'fro'));
        devTrail = max(devTrail, ...
            norm(seq.Trail(:, k) - Ek(1:3, 4)));
        devG = max(devG, norm(seq.Frames{k}(g1) - seq.Frames{1}(g1), 'fro'));
    end
    % zero configuration through the same pipeline (frame 1 of the demo is
    % NOT the exact zero pose — position offset (RC,0,0) and z=0):
    % tip in tool world = U⁻¹ ⇒ the +1 mm Z tool offset
    seq0 = solver.buildPoseSequence(e, zeros(1, numel(names)), names, ...
        'Anchor', 'tool', 'ToolTransform', U);
    devZero = norm(seq0.Frames{1}(seq0.RootFrame) ...
        - localTO(eye(3), [0; 0; 1]), 'fro');
    assert(devTip < 1e-9, 'tool-world tip path deviates from U⁻¹·D_k (%.3e)', devTip);
    assert(devTrail < 1e-9, 'tool-world trail deviates (%.3e)', devTrail);
    assert(devZero < 1e-9, ...
        'zero pose should show the +1 mm Z offset (dev %.3e)', devZero);
    assert(devG < 1e-9, 'ground frame wanders in the tool world (dev %.3e)', devG);
    assert(seq.BranchGapMax < 1e-9, 'ground-branch gap %.3e', seq.BranchGapMax);
    fprintf(['  [4] tool-world sequence: tip dev = %.2e, trail dev = %.2e, ' ...
        'zero-pose offset dev = %.2e, ground drift = %.2e, branch gap = %.2e\n'], ...
        devTip, devTrail, devZero, devG, seq.BranchGapMax);

    % ---- [5] solver cross-check: residual + tool-frame reporting ----
    samples = unique(round(linspace(1, nFrames, 5)));
    maxR = 0; maxTool = 0;
    for k = samples
        cs.setTipTarget(D{k});
        qm = containers.Map('KeyType', 'char', 'ValueType', 'double');
        for j = 1:numel(names); qm(names{j}) = Q(k, j); end
        r = cs.evalResidual(qm);
        assert(all(isfinite(r)), 'non-finite residual component at frame %d.', k);
        maxR = max(maxR, norm(r));
        % tool-frame report: D_tool(q) = U⁻¹ · D_k · U
        maxTool = max(maxTool, norm(cs.evalToolPose(qm) - Uinv * D{k} * U, 'fro'));
    end
    assert(maxR < 1e-9, 'closure residual %.3e exceeds 1e-9.', maxR);
    assert(maxTool < 1e-9, 'evalToolPose deviates from U⁻¹·D_k·U (%.3e)', maxTool);
    fprintf(['  [5] solver (frames %s): max |r| = %.2e; ' ...
        'evalToolPose vs U⁻¹·D_k·U dev = %.2e\n'], mat2str(samples), maxR, maxTool);

    % ---- [6] render layer: pure playback with the tool triad attached ----
    scene = viz.sceneSpec(e);
    extra = struct('Label', 'tool (world-aligned)', 'Base', seq.RootFrame, 'T', U);
    fprintf('  [6] launching tool-world animation (close figure to continue) ...\n');
    viz.play(seq, scene, 'Fps', fps, 'ExtraFrames', extra);

    fprintf('=== test_animation_m_rex_3t2r_v00: all checks passed ===\n');
end

function R = localRx(a)
%LOCALRX  Rx(a) — rotation about +X.
    R = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
end

function R = localRz(a)
%LOCALRZ  Rz(a) — rotation about +Z.
    R = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
end

function T = localTO(R, p)
    T = [R, p(:); 0 0 0 1];
end
