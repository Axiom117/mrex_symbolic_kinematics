function test_animation_m_rex_3t2r()
%TEST_ANIMATION_M_REX_3T2R  Layered pipeline + tool-frame animation (mrex_3t2r_001).
%   Demonstrates the 3T2R (3T ⊕ 2R) task space under PURE RCM: the tip stays
%   fixed at the zero-pose point (p ≡ 0 — no translation, no circle) and only
%   rotates.  The demo is a single-axis tour that sweeps each rotation axis to
%   its own stroke limit and back (0 → +q1 → 0 → +q2 → 0 → −q1 → 0 → −q2 → 0),
%   one axis at a time; the tip rotation stays in the exact two-parameter
%   family R = Ry(-q1)·Rx(q2).  The joint trajectory is built in closed form
%   (no IK, no fmincon):
%     joint_3.q = -joint_2.q;  joint_4.q = -joint_1.q      (mirror closure)
%     dL = R_bind' * (p_target - p_swingL(q1, q2))
%     dR = R_bind' * (p_target - p_swingR(q3, q4))
%   where R_bind = rot(P_L) = rot(P_R) is the constant rotation shared by the
%   zero-pose ground bindings, and p_swingL/R are the tip world positions with
%   the corresponding roll deflected and both Manipulators at zero (evaluated
%   via the compiled pose functions).
%
%   Per-axis ranges — calibrated here by bisection on the compiled pose
%   functions, coupling ignored (one axis at a time from the zero pose), with
%   the real Manipulator stroke |dx| <= 10, |dy| <= 10, |dz| <= 15 mm:
%     q1 (Ry roll): [-18.420, +20.520] deg (dx binds on the negative side, dz
%                                          on the positive side; the dy drive
%                                          travel is identically zero)
%     q2 (Rx roll): [-15.945, +15.945] deg (dy binds on both sides)
%   Each bound is asserted tight (0.1 deg beyond it already exceeds the
%   stroke); the demo tour sweeps q1 −18.4..+20.5 and q2 ±15.9 deg.
%
%   Checks:
%     [1] symbolic pipeline + bindings (rot(P_L) == rot(P_R))
%     [2] pure-RCM per-axis sweep limits (bisection, tightness asserted) +
%         closed-form single-axis tour: rotation-family exactness + travel
%         envelope ±10/±10/±15 mm + tool-axis sweep on the unit sphere
%     [3] solver config: case tool_frame resolves to U = Ry(45°)
%     [4] compute layer: solver.buildPoseSequence('Anchor','tool') — tip path
%         = U⁻¹·D_k, zero pose shows the −45° pipette tilt, grounds static,
%         branch gap ~1e-14
%     [5] solver cross-check: ClosureSolver residual (setTipTarget) and
%         evalToolPose (tool-frame reporting) at sampled frames
%     [6] render layer: viz.sceneSpec + viz.play with the tool triad attached
%         to the root (ExtraFrames); the tip trail is switched off (the tip is
%         a fixed RCM point, no path to draw)
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_animation_m_rex_3t2r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    % ---- demo trajectory parameters (pure-RCM single-axis tour) ----
    travelLim = [10 10 15];  % real Manipulator stroke |dx| |dy| |dz| [mm]
    nFrames   = 120;   % frames per loop (15 per tour segment)
    fps       = 30;    % playback speed

    fprintf('=== test_animation_m_rex_3t2r (layered + tool-frame pure RCM) ===\n');

    dslFile  = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex_3t2r_001', ...
        'robot_description.yaml');
    execFile = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex_3t2r_001', ...
        'execution-config.yaml');

    % ---- [1] symbolic pipeline + zero-pose bindings ----
    e = ir.Expander(dslFile);
    e.compilePoseFunctions();
    names = {'manipulator_L.dx', 'manipulator_L.dy', 'manipulator_L.dz', ...
             'manipulator_R.dx', 'manipulator_R.dy', 'manipulator_R.dz', ...
             'joint_1.q', 'joint_2.q', 'joint_3.q', 'joint_4.q'};

    zeroMap = containers.Map('KeyType', 'char', 'ValueType', 'double');
    gp = e.evaluateNumericDirect(zeroMap, ...
        {'manipulator_L.ground', 'manipulator_R.ground'});
    P_L = gp('manipulator_L.ground');
    P_R = gp('manipulator_R.ground');
    R_bind = P_L(1:3, 1:3);
    rotDev = norm(R_bind - P_R(1:3, 1:3));
    assert(rotDev < 1e-12, 'ground binding rotations differ (%.3e).', rotDev);
    fprintf('  [1] pipeline OK; rot(P_L) == rot(P_R) (dev %.1e)\n', rotDev);

    % ---- [2a] per-axis pure-RCM sweep limits (coupling ignored) ----
    travelQ1 = @(deg) localRcmTravel(deg, 0, e, P_L, P_R, R_bind);
    travelQ2 = @(deg) localRcmTravel(0, deg, e, P_L, P_R, R_bind);
    [q1Lo, q1Hi, q1LoB, q1HiB] = localSweepLimit(travelQ1, travelLim);
    [q2Lo, q2Hi, q2LoB, q2HiB] = localSweepLimit(travelQ2, travelLim);
    % demo amplitudes on a 0.1 deg grid, kept strictly inside each side
    q1Pos = floor(q1Hi * 10) / 10;   q1Neg = ceil(q1Lo * 10) / 10;
    q2Pos = floor(q2Hi * 10) / 10;   q2Neg = ceil(q2Lo * 10) / 10;
    fprintf(['  [2a] pure RCM per-axis limits (stroke %.0f/%.0f/%.0f mm, ' ...
        'coupling ignored):\n'], travelLim);
    fprintf('       q1 (Ry roll): [%.3f, +%.3f] deg  (binds %s low / %s high)\n', ...
        q1Lo, q1Hi, q1LoB, q1HiB);
    fprintf('       q2 (Rx roll): [%.3f, +%.3f] deg  (binds %s low / %s high)\n', ...
        q2Lo, q2Hi, q2LoB, q2HiB);
    fprintf(['Demo trajectory: single-axis RCM tour q1 [%g, +%g], q2 [%g, +%g] ' ...
        'deg (tip fixed), %d frames @ %d fps\n'], ...
        q1Neg, q1Pos, q2Neg, q2Pos, nFrames, fps);

    % ---- [2b] closed-form demo trajectory (Q + tip targets D_k) ----
    % One axis at a time: 0 → +q1 → 0 → +q2 → 0 → −q1 → 0 → −q2 → 0, so every
    % segment is a single-DOF RCM rotation and the tour starts/ends at zero.
    wp = deg2rad([0 0; q1Pos 0; 0 0; 0 q2Pos; 0 0; ...
                  q1Neg 0; 0 0; 0 q2Neg; 0 0]);
    nSeg = size(wp, 1) - 1;
    D = cell(1, nFrames);
    Q = zeros(nFrames, numel(names));
    maxFamDev = 0;   % exactness of R_tip == Ry(-q1)·Rx(q2)
    axTilt    = 0;   % tool-axis sweep angle on the unit sphere
    p = [0; 0; 0];   % RCM point = zero-pose tip position (held fixed)
    for k = 1:nFrames
        s = (k - 1) / (nFrames - 1) * nSeg;     % tour position [segment]
        i = min(floor(s) + 1, nSeg);
        u = s - (i - 1);
        q1 = wp(i, 1) + u * (wp(i + 1, 1) - wp(i, 1));
        q2 = wp(i, 2) + u * (wp(i + 1, 2) - wp(i, 2));
        q3 = -q2;  q4 = -q1;                    % mirror closure

        % swing tip positions with the roll deflected, Manipulators at zero
        mL = containers.Map('KeyType', 'char', 'ValueType', 'double');
        mL('joint_1.q') = q1;  mL('joint_2.q') = q2;
        gL = e.evaluateNumericDirect(mL, {'manipulator_L.ground'});
        T_swL = P_L * core.RigidBodyMath.invT(gL('manipulator_L.ground'));

        mR = containers.Map('KeyType', 'char', 'ValueType', 'double');
        mR('joint_3.q') = q3;  mR('joint_4.q') = q4;
        gR = e.evaluateNumericDirect(mR, {'manipulator_R.ground'});
        T_swR = P_R * core.RigidBodyMath.invT(gR('manipulator_R.ground'));

        % constant-rotation compensation of the swing displacement
        dL = R_bind' * (p - T_swL(1:3, 4));
        dR = R_bind' * (p - T_swR(1:3, 4));

        R_tip = localRyRx(q1, q2);
        D{k} = [R_tip, p; 0 0 0 1];
        Q(k, :) = [dL(:).', dR(:).', q1, q2, q3, q4];

        % left-branch FK must reproduce the family rotation exactly
        maxFamDev = max(maxFamDev, norm(T_swL(1:3, 1:3) - R_tip, 'fro'));
        % axis direction of the tip frame on the unit sphere
        axTilt = max(axTilt, acosd(min(1, abs(R_tip(3, 3)))));
    end

    % manipulator travel envelope (real stroke |dx|<=10, |dy|<=10, |dz|<=15 mm)
    trav = abs(Q(:, 1:6));
    mX = max([trav(:, 1); trav(:, 4)]);
    mY = max([trav(:, 2); trav(:, 5)]);
    mZ = max([trav(:, 3); trav(:, 6)]);
    tol = 1e-9;
    assert(mX <= travelLim(1) + tol && mY <= travelLim(2) + tol ...
        && mZ <= travelLim(3) + tol, ...
        'manipulator travel envelope exceeded (%.2f/%.2f/%.2f).', mX, mY, mZ);
    assert(maxFamDev < 1e-9, 'rotation-family exactness deviates (%.3e).', maxFamDev);
    fprintf(['  [2b] closed form built: family dev = %.1e; max travel ' ...
        '|dx|=%.2f |dy|=%.2f |dz|=%.2f mm (limits 10/10/15); ' ...
        'tool-axis sweep <= %.1f deg\n'], maxFamDev, mX, mY, mZ, axTilt);

    % ---- [3] solver config: case tool frame resolves to U = Ry(45°) ----
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
    assert(numel(cs.UnknownSyms) == 10, ...
        'expected 10 unknowns (6 manip + 4 hinge), got %d.', numel(cs.UnknownSyms));
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
    % frame 1 of the tour is the exact zero pose (q1 = q2 = 0, p = 0), so the
    % tool world shows the −45° pipette tilt directly
    devZero = norm(seq.Frames{1}(seq.RootFrame) ...
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
    viz.play(seq, scene, 'Fps', fps, 'ExtraFrames', extra, 'ShowTrail', false);

    fprintf('=== test_animation_m_rex_3t2r: all checks passed ===\n');
end

function R = localRyRx(a, b)
%LOCALRYRX  Ry(-a) * Rx(b): the exact two-parameter tip rotation family.
    Ra = [cos(a), 0, -sin(a); 0, 1, 0; sin(a), 0, cos(a)];   % Ry(-a)
    Rb = [1, 0, 0; 0, cos(b), -sin(b); 0, sin(b), cos(b)];   % Rx(b)
    R = Ra * Rb;
end

function T = localTO(R, p)
    T = [R, p(:); 0 0 0 1];
end

function v = localRcmTravel(deg1, deg2, e, P_L, P_R, R_bind)
%LOCALRCMTRAVEL  Drive travel [max|dx| max|dy| max|dz|] for a pure RCM pose.
%   Single-axis sweeps pass one angle and zero for the other; the mirror
%   closure (joint_3.q = -q2, joint_4.q = -q1) keeps the tip in the
%   Ry(-q1)·Rx(q2) family.
    q1 = deg2rad(deg1);  q2 = deg2rad(deg2);
    mL = containers.Map('KeyType', 'char', 'ValueType', 'double');
    mL('joint_1.q') = q1;  mL('joint_2.q') = q2;
    gL = e.evaluateNumericDirect(mL, {'manipulator_L.ground'});
    T_L = P_L * core.RigidBodyMath.invT(gL('manipulator_L.ground'));
    mR = containers.Map('KeyType', 'char', 'ValueType', 'double');
    mR('joint_3.q') = -q2;  mR('joint_4.q') = -q1;
    gR = e.evaluateNumericDirect(mR, {'manipulator_R.ground'});
    T_R = P_R * core.RigidBodyMath.invT(gR('manipulator_R.ground'));
    d = -R_bind.' * [T_L(1:3, 4), T_R(1:3, 4)];
    v = max(abs(d), [], 2).';
end

function [loDeg, hiDeg, loBind, hiBind] = localSweepLimit(travelAt, lim)
%LOCALSWEEPLIMIT  Stroke-safe single-axis rotation range [LODEG, HIDEG] [deg].
%   travelAt(q) -> [max|dx| max|dy| max|dz|] over both Manipulators for a
%   pure-RCM rotation of q about one axis from the zero pose (coupling is not
%   considered: one axis at a time).
%   LOBIND / HIBIND name the drive axis that binds on each side ('dx'..'dz',
%   'none' when that side stays inside LIM up to 60 deg).  Each returned bound
%   is asserted tight: 0.1 deg further already exceeds the stroke.
    [loDeg, loBind] = localSignLimit(travelAt, lim, -1);
    [hiDeg, hiBind] = localSignLimit(travelAt, lim, +1);
end

function [deg, bind] = localSignLimit(travelAt, lim, sgn)
    hiMax = 60;                        % beyond this the sweep is out of scope
    f = @(d) max(travelAt(sgn * d) - lim);
    if f(hiMax) < 0
        deg = sgn * hiMax;  bind = 'none';
        return;
    end
    lo = 0; hi = hiMax;
    for it = 1:60
        mid = (lo + hi) / 2;
        if f(mid) < 0, lo = mid; else, hi = mid; end
    end
    deg = sgn * lo;
    xyz = 'xyz';
    [~, a] = max(travelAt(deg) - lim);
    bind = ['d' xyz(a)];
    assert(f(lo + 0.1) > 0, 'sweep limit %.3f deg is not tight.', lo);
end
