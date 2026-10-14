function test_animation_m_rex_3t1r()
%TEST_ANIMATION_M_REX_3T1R  Layered pipeline + tool-frame animation (m-rex-3t1r).
%   Demonstrates the current end-to-end flow (A.5.2.0) — the compute layer
%   produces a controlled pose sequence (solver.buildPoseSequence), the render
%   layer is a pure consumer (viz.sceneSpec + viz.play) — together with the
%   L3 tool frame (world ≡ tool-frame zero; the case config declares
%   rpy [0, 45deg, 0], so the pipette appears tilted −45° in the tool world).
%
%   Demo trajectory (DEFINED HERE — the execution-config YAML only declares
%   L3 execution semantics, not the motion):
%     - phi (tip X-rotation): -phiMaxDeg .. +phiMaxDeg
%     - position: XY-plane circle of radius RC (one full turn), z = 0
%     - theta (Ry) = psi (Rz) = 0  (locked) — the RCM (x, y, z, phi) task mode
%
%   The joint trajectory is built in closed form (no IK, no fmincon):
%     joint_1.q = phi;   joint_2.q = -phi
%     dL = R_bind' * (p_target - p_swing_L(phi))
%     dR = R_bind' * (p_target - p_swing_R(-phi))
%   where R_bind = rot(P_L) = rot(P_R) is the constant rotation shared by the
%   zero-pose ground bindings, and p_swing_L/R are the tip world positions with
%   the corresponding hinge deflected and both Manipulators at zero (evaluated
%   via the compiled pose functions).
%
%   Checks:
%     [1] symbolic pipeline + bindings (rot(P_L) == rot(P_R))
%     [2] closed-form demo trajectory built; travel envelope 20/20/30 mm
%     [3] solver config: case tool_frame resolves to U = Ry(45°)
%     [4] compute layer: solver.buildPoseSequence('Anchor','tool') — tool-world
%         display: tip path = U⁻¹·D_k, zero pose shows the −45° pipette tilt,
%         grounds static, branch gap ~1e-14
%     [5] solver cross-check: ClosureSolver residual (setTipTarget) and
%         evalToolPose (tool-frame reporting) at sampled frames
%     [6] render layer: viz.sceneSpec + viz.play (pure data consumption) with
%         the tool triad attached to the root (ExtraFrames)
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_animation_m_rex_3t1r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    % ---- demo trajectory parameters (the prescribed demo motion) ----
    phiMaxDeg = 28;    % phi sweep: -phiMaxDeg .. +phiMaxDeg [deg]
                       % (28 deg keeps manip travel within the 20/20/30 envelope)
    RC        = 5;     % circle radius                         [mm]
    nFrames   = 60;    % frames per loop
    fps       = 30;    % playback speed

    fprintf('=== test_animation_m_rex_3t1r (A.5.2.0 layered + tool frame) ===\n');
    fprintf('Demo trajectory: phi -%d..+%d deg + XY circle r=%d mm, %d frames @ %d fps\n', ...
        phiMaxDeg, phiMaxDeg, RC, nFrames, fps);

    dslFile  = fullfile(rootDir, 'specs', 'dsl', 'cases', 'm-rex-3t1r', ...
        'robot_description.yaml');
    execFile = fullfile(rootDir, 'specs', 'dsl', 'cases', 'm-rex-3t1r', ...
        'execution-config.yaml');

    % ---- [1] symbolic pipeline + zero-pose bindings ----
    e = ir.Expander(dslFile);
    e.compilePoseFunctions();
    names = {'manipulator_L.dx', 'manipulator_L.dy', 'manipulator_L.dz', ...
             'manipulator_R.dx', 'manipulator_R.dy', 'manipulator_R.dz', ...
             'joint_1.q', 'joint_2.q'};

    zeroMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
    gp = e.evaluateNumericDirect(zeroMap, ...
        {'manipulator_L.ground', 'manipulator_R.ground'});
    P_L = gp('manipulator_L.ground');
    P_R = gp('manipulator_R.ground');
    R_bind = P_L(1:3, 1:3);
    rotDev = norm(R_bind - P_R(1:3, 1:3));
    assert(rotDev < 1e-12, 'ground binding rotations differ (%.3e).', rotDev);
    fprintf('  [1] pipeline OK; rot(P_L) == rot(P_R) (dev %.1e)\n', rotDev);

    % ---- [2] closed-form demo trajectory (Q + tip targets D_k) ----
    D = cell(1, nFrames);
    Q = zeros(nFrames, numel(names));
    for k = 1:nFrames
        t = (k - 1) / (nFrames - 1);
        phi = deg2rad(-phiMaxDeg + 2 * phiMaxDeg * t);
        a = 2 * pi * t;
        p = [RC * cos(a); RC * sin(a); 0];

        % tip world pose with hinge deflected, Manipulators at zero
        mL = containers.Map('KeyType', 'char', 'ValueType', 'double');
        mL('joint_1.q') = phi;
        gL = e.evaluateNumericDirect(mL, {'manipulator_L.ground'});
        T_swL = P_L * core.RigidBodyMath.invT(gL('manipulator_L.ground'));

        mR = containers.Map('KeyType', 'char', 'ValueType', 'double');
        mR('joint_2.q') = -phi;
        gR = e.evaluateNumericDirect(mR, {'manipulator_R.ground'});
        T_swR = P_R * core.RigidBodyMath.invT(gR('manipulator_R.ground'));

        % constant-rotation compensation of the swing displacement
        dL = R_bind' * (p - T_swL(1:3, 4));
        dR = R_bind' * (p - T_swR(1:3, 4));

        Q(k, :) = [dL(:).', dR(:).', phi, -phi];
        D{k} = [localRx(phi), p; 0 0 0 1];
    end

    % manipulator travel envelope (real stroke 20/20/30 mm)
    trav = abs(Q(:, 1:6));
    mX = max([trav(:, 1); trav(:, 4)]);
    mY = max([trav(:, 2); trav(:, 5)]);
    mZ = max([trav(:, 3); trav(:, 6)]);
    assert(mX <= 20 && mY <= 20 && mZ <= 30, ...
        'manipulator travel envelope exceeded (%.2f/%.2f/%.2f).', mX, mY, mZ);
    fprintf(['  [2] closed form built: max travel |dx|=%.2f |dy|=%.2f |dz|=%.2f mm ' ...
        '(limits 20/20/30)\n'], mX, mY, mZ);

    % ---- [3] solver config: case tool frame resolves to U = Ry(45°) ----
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
    assert(numel(cs.UnknownSyms) == 8, ...
        'expected 8 unknowns (6 manip + 2 hinge), got %d.', numel(cs.UnknownSyms));
    U = cs.ToolTransform;
    devU = norm(U - localTO(core.RigidBodyMath.roty(pi/4), [0; 0; 0]), 'fro');
    assert(devU < 1e-12, 'case tool frame should resolve to Ry(45°), dev %.3e', devU);
    Uinv = core.RigidBodyMath.invT(U);
    fprintf('  [3] tool frame resolved: U = Ry(45°) (dev %.2e)\n', devU);

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
    % NOT zero — the sweep starts at -phiMaxDeg): tip in tool world = U⁻¹
    seq0 = solver.buildPoseSequence(e, zeros(1, numel(names)), names, ...
        'Anchor', 'tool', 'ToolTransform', U);
    devZero = norm(seq0.Frames{1}(seq0.RootFrame) ...
        - localTO(core.RigidBodyMath.roty(-pi/4), [0; 0; 0]), 'fro');
    assert(devTip < 1e-9, 'tool-world tip path deviates from U⁻¹·D_k (%.3e)', devTip);
    assert(devTrail < 1e-9, 'tool-world trail deviates (%.3e)', devTrail);
    assert(devZero < 1e-9, 'zero pose should show the −45° tilt (dev %.3e)', devZero);
    assert(devG < 1e-9, 'ground frame wanders in the tool world (dev %.3e)', devG);
    assert(seq.BranchGapMax < 1e-9, 'ground-branch gap %.3e', seq.BranchGapMax);
    fprintf(['  [4] tool-world sequence: tip dev = %.2e, trail dev = %.2e, ' ...
        'zero-pose tilt dev = %.2e, ground drift = %.2e, branch gap = %.2e\n'], ...
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

    fprintf('=== test_animation_m_rex_3t1r: all checks passed ===\n');
end

function R = localRx(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function T = localTO(R, p)
    T = [R, p(:); 0 0 0 1];
end
