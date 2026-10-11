function test_animation_m_rex_3t1r()
%TEST_ANIMATION_M_REX_3T1R  A.5.1: world-anchored animation of the 3T1R M-REx.
%   Demo trajectory (DEFINED HERE — in the parameter block below; the
%   execution-config YAML only declares L3 execution semantics, not the motion):
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
%   via the compiled pose functions).  Achieved accuracy: |E - D| ~ 1e-14.
%
%   Checks:
%     [1] symbolic pipeline + bindings (rot(P_L) == rot(P_R))
%     [2] closed-form trajectory reaches the targets per frame: ||E-D|| < 1e-9
%     [3] manipulator travel within the real stroke envelope (20/20/30 mm)
%     [4] sampled solver.ClosureSolver residual (execution-config.yaml) < 1e-9
%     [5] launch world-anchored animation with tip trail (viz.animate)
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
    fps       = 15;    % playback speed

    fprintf('=== test_animation_m_rex_3t1r (A.5.1) ===\n');
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

    % ---- [2] closed-form demo trajectory ----
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

    % per-frame closure via direct FK: E_L = E_R = D
    maxDev = 0;
    for k = 1:nFrames
        m = containers.Map('KeyType', 'char', 'ValueType', 'double');
        for j = 1:numel(names); m(names{j}) = Q(k, j); end
        pg = e.evaluateNumericDirect(m, ...
            {'manipulator_L.ground', 'manipulator_R.ground'});
        E_L = P_L * core.RigidBodyMath.invT(pg('manipulator_L.ground'));
        E_R = P_R * core.RigidBodyMath.invT(pg('manipulator_R.ground'));
        maxDev = max(maxDev, ...
            max(norm(E_L - D{k}, 'fro'), norm(E_R - D{k}, 'fro')));
    end
    fprintf('  [2] closed form (%d frames): max ||E - D|| = %.2e\n', nFrames, maxDev);
    assert(maxDev < 1e-9, 'closed-form trajectory deviation %.3e exceeds 1e-9.', maxDev);

    % ---- [3] manipulator travel envelope (real stroke 20/20/30 mm) ----
    trav = abs(Q(:, 1:6));
    mX = max([trav(:, 1); trav(:, 4)]);
    mY = max([trav(:, 2); trav(:, 5)]);
    mZ = max([trav(:, 3); trav(:, 6)]);
    fprintf('  [3] max travel: |dx|=%.2f  |dy|=%.2f  |dz|=%.2f mm (limits 20/20/30)\n', ...
        mX, mY, mZ);
    assert(mX <= 20 && mY <= 20 && mZ <= 30, ...
        'manipulator travel envelope exceeded (%.2f/%.2f/%.2f).', mX, mY, mZ);

    % ---- [4] solver-side closure residual on sampled frames ----
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
    assert(numel(cs.UnknownSyms) == 8, ...
        'expected 8 unknowns (6 manip + 2 hinge), got %d.', numel(cs.UnknownSyms));
    samples = unique(round(linspace(1, nFrames, 5)));
    maxR = 0;
    for k = samples
        cs.setTipTarget(D{k});
        qm = containers.Map('KeyType', 'char', 'ValueType', 'double');
        for j = 1:numel(names); qm(names{j}) = Q(k, j); end
        r = cs.evalResidual(qm);
        assert(all(isfinite(r)), 'non-finite residual component at frame %d.', k);
        maxR = max(maxR, norm(r));
    end
    fprintf('  [4] ClosureSolver residual (frames %s): max |r| = %.2e\n', ...
        mat2str(samples), maxR);
    assert(maxR < 1e-9, 'closure residual %.3e exceeds 1e-9.', maxR);

    % ---- [5] world-anchored animation ----
    fprintf('  [5] launching world-anchored animation (close figure to continue) ...\n');
    viz.animate(dslFile, Q, names, fps, 'Anchor', 'world');

    fprintf('=== test_animation_m_rex_3t1r: all checks passed ===\n');
end

function R = localRx(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end
