function test_closure_solver_m_rex_3t1r()
%TEST_CLOSURE_SOLVER_M_REX_3T1R  L3 world-closed-loop solver test (mrex-3t1r_001).
%   Builds the real mrex-3t1r_001 symbolic pipeline and verifies the L3
%   world-anchored support of solver.ClosureSolver.  Operates on the 3T1R
%   operational config (8 unknowns = 6 manipulator + 2 internal joints, all
%   12 residual components); the former 3T baseline partition is exercised
%   through a tmp fixture (see [9] / [11b]) so its coverage is preserved
%   while the case converges onto a single execution config:
%     [1] Expander builds (instances/edges/joints)
%     [2] ExecutionConfig: explicit cuts, 2 bindings, IK direction
%     [3] ClosureSolver: 8 unknowns, 12 components, cut struct fields
%     [4] verifyZeroPose (ΔT = I₄) — |r| < 1e-12
%     [5] T_bind matches independent Expander zero-pose FK evaluation
%     [6] perturbation dx_L = 1 mm → cut1 translation error ≈ 1 mm,
%         cut2 unaffected; cut1 rotation rows unresponsive
%     [7] residual depends only on the 8 unknown variables
%     [8] setTipTarget(ΔT): rebuilt residual matches hand formula at q=0;
%         reset to I₄ restores zero residual; printResidualReport smoke
%     [9] KNOWNVALUES (tmp 3T baseline fixture): bad key → error;
%         custom joint_1.q → closure broken
%    [10] explicit world_binding T: consistent accepted, perturbed rejected
%    [11] constraint Jacobians at zero pose: 3T1R 12×8 (full column rank);
%         baseline fixture 6×6 square (rank 6, ≈ I₆)
%    [12] fmincon IK for non-zero tip targets + world-rooted FK
%         cross-verification (T_W←tip = P_i · inv(T_tip←g_i(q)) ≈ ΔT);
%         Rx rotation solvable (q1 = φ, q2 = −φ), Ry rejected
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_closure_solver_m_rex_3t1r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    dslFile  = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex-3t1r_001', ...
        'robot_description.yaml');
    execFile = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex-3t1r_001', ...
        'execution-config.yaml');

    tmpDir = tempname(); mkdir(tmpDir);
    cleanupObj = onCleanup(@() rmdir(tmpDir, 's')); %#ok<NASGU>

    fprintf('=== test_closure_solver_m_rex_3t1r: L3 world-anchored solver checks ===\n');

    % ---- [1] symbolic pipeline ----
    t0 = tic;
    e = ir.Expander(dslFile);
    fprintf('  [1] Expander: %d instances, %d edges, %d joint vars (%.1fs)\n', ...
        numel(e.Instances), e.EdgeGraph_.numEdges, e.JointVarMap.Count, toc(t0));

    % ---- [2] execution config ----
    cfg = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    assert(strcmp(cfg.Mode, 'closed_loop'), 'mode');
    assert(strcmp(cfg.ClosureSource, 'explicit'), 'closure source');
    assert(numel(cfg.ClosureCuts) == 2, 'cut count');
    assert(numel(cfg.WorldBindings) == 2, 'binding count');
    assert(strcmp(cfg.getSolvingDirection(), 'IK'), 'direction');
    fprintf('  [2] ExecutionConfig: %d explicit cuts, %d bindings, direction IK\n', ...
        numel(cfg.ClosureCuts), numel(cfg.WorldBindings));

    % ---- [3] ClosureSolver construction ----
    t0 = tic;
    cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
    fprintf(['  [3] ClosureSolver built (%.1fs): %d cuts, %d unknowns, ' ...
        '%d components, TipTarget=I\n'], ...
        toc(t0), cs.NumCuts, numel(cs.UnknownSyms), cs.NumComponents);
    assert(numel(cs.UnknownSyms) == 8, 'unknown count (6 manip + 2 joints)');
    assert(strcmp(cs.UnknownNames{1}, 'manipulator_L.dx'), 'unknown[1]');
    assert(strcmp(cs.UnknownNames{7}, 'joint_1.q'), 'unknown[7]');
    assert(strcmp(cs.UnknownNames{8}, 'joint_2.q'), 'unknown[8]');
    assert(cs.NumComponents == 12, 'component count (2 cuts x 6)');
    assert(isequal(cs.TipTarget, eye(4)), 'TipTarget default');
    assert(strcmp(cs.ClosureCuts(1).near, 'manipulator_L.ground'), 'cut1 near');
    assert(strcmp(cs.ClosureCuts(1).anchor, 'world'), 'cut1 anchor');
    assert(isempty(cs.ClosureCuts(1).far), 'cut1 far empty');
    assert(isequal(cs.ClosureCuts(1).T_mate, eye(4)), 'cut1 T_mate = I');
    assert(isa(cs.ClosureCuts(1).T_bind, 'double') && ...
        isequal(size(cs.ClosureCuts(1).T_bind), [4 4]), 'cut1 T_bind shape');

    % ---- [4] zero-pose verification ----
    t0 = tic;
    cs.verifyZeroPose();
    fprintf('  [4] verifyZeroPose OK (numeric function built in %.1fs)\n', toc(t0));

    % ---- [5] T_bind vs independent Expander zero-pose FK ----
    qZero = containers.Map();
    kv = keys(e.JointVarMap);
    for k = 1:numel(kv); qZero(kv{k}) = 0; end
    poses0 = e.evaluateNumericDirect(qZero, ...
        {'manipulator_L.ground', 'manipulator_R.ground'});
    for i = 1:2
        dev = norm(cs.ClosureCuts(i).T_bind - poses0(cs.ClosureCuts(i).near), 'fro');
        assert(dev < 1e-12, sprintf('binding %d deviation %.3e', i, dev));
    end
    fprintf('  [5] T_bind matches Expander zero-pose FK (both grounds)\n');

    % ---- [6] perturbation: dx_L = 1 mm ----
    rp = cs.evalResidual(containers.Map({'manipulator_L.dx'}, {1}));
    tErr1 = norm(rp(1:3));
    rot1  = norm(rp(4:6));
    rErr2 = norm(rp(7:12));
    fprintf(['  [6] dx_L=1mm -> cut1 |t_err| = %.9f mm, cut1 |rot| = %.2e, ' ...
        'cut2 |r| = %.2e\n'], tErr1, rot1, rErr2);
    assert(abs(tErr1 - 1) < 1e-6, 'perturbation magnitude');
    assert(rot1 < 1e-9, 'cut1 rotation rows should not respond to prismatic drives');
    assert(rErr2 < 1e-9, 'cut2 should be unaffected');

    % ---- [7] residual depends only on unknown vars ----
    resNames = arrayfun(@char, symvar(cs.ResidualSym), 'UniformOutput', false);
    uNames = cellfun(@char, num2cell(cs.UnknownSyms), 'UniformOutput', false);
    assert(numel(resNames) == 8, 'residual var count');
    assert(all(ismember(resNames, uNames)), 'residual vars subset');
    fprintf('  [7] residual symvars: %s\n', strjoin(resNames, ', '));

    % ---- [8] setTipTarget rebuilds residual correctly ----
    D = eye(4); D(1,4) = 1;   % 1 mm along +X
    cs.setTipTarget(D);
    assert(isequal(cs.TipTarget, D), 'TipTarget stored');
    rz = cs.evalResidual(zeros(8,1));
    ok = true;
    for i = 1:2
        P = cs.ClosureCuts(i).T_bind;
        Tres = P \ (D \ P);               % expected T_res at q = 0
        exp6 = poseError6(Tres);
        seg = 6*(i-1) + (1:6);
        dev = norm(rz(seg) - exp6);
        fprintf('  [8] cut%d: ΔT=I+1mm·x -> |r| = %.6e, vs hand formula dev = %.2e\n', ...
            i, norm(rz(seg)), dev);
        ok = ok && dev < 1e-9;
    end
    assert(ok, 'hand formula mismatch');
    cs.setTipTarget(eye(4));
    r0 = cs.evalResidual(zeros(8,1));
    assert(norm(r0) < 1e-12, 'reset to I should restore zero residual');
    fprintf('  [8] setTipTarget round-trip OK (reset -> |r| = %.2e)\n', norm(r0));

    % printResidualReport smoke — exercises the "(world binding)" far label
    fprintf('  [8] printResidualReport smoke:\n');
    cs.printResidualReport(zeros(8,1));

    % ---- [9] known values map (former 3T baseline, now a tmp fixture) ----
    % The 3T1R operational config declares no known joint variables, so the
    % KNOWNVALUES feature is exercised against a fixture that carries the
    % A.4b baseline partition (joint_1.q / joint_2.q known, [tx,ty,tz] cuts).
    fixFile = fullfile(tmpDir, 'exec_baseline_3t.yaml');
    fid = fopen(fixFile, 'w');
    assert(fid > 0, 'cannot open %s', fixFile);
    fwrite(fid, localBaselineFixtureYaml());
    fclose(fid);
    cfgF = ir.ExecutionConfig(fixFile, e.SymbolRegistry, e.EdgeGraph_);

    checkError(@() solver.ClosureSolver(e.EdgeGraph_, cfgF, e.JointVarMap, ...
        containers.Map({'foo.bar'}, {1})), ...
        'solver:ClosureSolver:badKnownValue', 'bad known-value key (fixture)');
    cs2 = solver.ClosureSolver(e.EdgeGraph_, cfgF, e.JointVarMap, ...
        containers.Map({'joint_1.q'}, {0.05}));
    assert(numel(cs2.UnknownSyms) == 6, 'fixture unknown count');
    rk = cs2.evalResidual(zeros(6,1));
    fprintf('  [9] fixture joint_1.q=0.05 -> |r| = %.6e (closure broken as expected)\n', ...
        norm(rk));
    assert(norm(rk) > 1e-3, 'known value should break closure');

    % ---- [10] explicit binding: consistent accepted / perturbed rejected ----
    TbL = cs.ClosureCuts(1).T_bind;
    txt = fileread(execFile);
    writeExplicitConfig(fullfile(tmpDir, 'exec_T.yaml'), txt, ...
        'manipulator_L.ground', TbL);
    TbLp = TbL; TbLp(1,4) = TbLp(1,4) + 0.01;
    writeExplicitConfig(fullfile(tmpDir, 'exec_Tp.yaml'), txt, ...
        'manipulator_L.ground', TbLp);

    csT = solver.ClosureSolver(e.EdgeGraph_, ...
        ir.ExecutionConfig(fullfile(tmpDir, 'exec_T.yaml'), e.SymbolRegistry, e.EdgeGraph_), ...
        e.JointVarMap); %#ok<NASGU>
    rhsDev = norm(csT.ClosureCuts(1).T_bind - TbL, 'fro');
    assert(rhsDev < 1e-9, 'roundtrip explicit T');
    fprintf('  [10] explicit consistent T accepted (roundtrip dev = %.2e)\n', rhsDev);
    checkError(@() solver.ClosureSolver(e.EdgeGraph_, ...
        ir.ExecutionConfig(fullfile(tmpDir, 'exec_Tp.yaml'), e.SymbolRegistry, e.EdgeGraph_), ...
        e.JointVarMap), ...
        'solver:ClosureSolver:bindingMismatch', 'perturbed T rejected');

    % ---- [11] constraint Jacobians at zero pose ----
    % [11a] 3T1R operational system: 12 components x 8 unknowns.
    % cs is back at ΔT = I₄ (reset at the end of [8]); reuse its numeric handle.
    fh = cs.toNumericFunction();
    Jz = localJacobian(fh, zeros(8,1));
    sz = svd(Jz);
    rankJ = sum(sz > 1e-9 * sz(1));
    fprintf('  [11a] 3T1R constraint Jacobian at zero pose (∂r/∂x, 12x8):\n');
    disp(Jz);
    fprintf('        singular values: [%s]\n', strjoin( ...
        arrayfun(@(v) sprintf('%.6f', v), sz(:).', 'UniformOutput', false), ', '));
    fprintf('        rank = %d / 8 (full column rank), cond = %.4f\n', ...
        rankJ, sz(1)/sz(end));
    assert(rankJ == 8, 'expected full column rank 8 for the 8-unknown system');
    assert(sz(1)/sz(end) < 1e4, 'ill-conditioned 3T1R constraint Jacobian');

    % [11b] former 3T baseline (A.4b regression): square 6x6 system, J = I₆.
    csF = solver.ClosureSolver(e.EdgeGraph_, cfgF, e.JointVarMap);
    JzF = localJacobian(csF.toNumericFunction(), zeros(6,1));
    szF = svd(JzF);
    rankF = sum(szF > 1e-9 * szF(1));
    devI = norm(JzF - eye(6), 'fro');
    fprintf(['  [11b] baseline fixture Jacobian (6x6): rank = %d / 6, ' ...
        'cond = %.4f, ||J - I₆|| = %.2e\n'], rankF, szF(1)/szF(end), devI);
    assert(rankF == 6, 'expected full-rank 6x6 baseline at zero pose');
    assert(szF(1)/szF(end) < 1e4, 'ill-conditioned baseline Jacobian');
    assert(devI < 1e-6, 'baseline Jacobian should match I₆ (A.4b metric)');

    % ---- [12] fmincon IK + world-rooted FK cross-verification ----
    P_L = cs.ClosureCuts(1).T_bind;
    P_R = cs.ClosureCuts(2).T_bind;

    % Central finite differences are required here: with default forward
    % differences the SQP stalls at |r| ~ 1e-4..1e-6 on this system (mixed
    % mm/rad residual scale + revolute unknowns); central FD converges to
    % ~1e-13 deterministically (probed on MATLAB R2026a, 2026-10-11).
    ikOpts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', 'Display', 'off', ...
        'FiniteDifferenceType', 'central', ...
        'OptimalityTolerance', 1e-12, 'StepTolerance', 1e-12, ...
        'MaxFunctionEvaluations', 4000, 'MaxIterations', 400);

    % zero-target sanity: solver starts exactly at the optimum
    cs.setTipTarget(eye(4));
    [x0s, f0s, ef0] = cs.solveIK(zeros(8,1), ikOpts);
    rn0 = norm(cs.evalResidual(x0s));
    fprintf('  [12] zero-target sanity: |r| = %.2e, fval = %.2e, norm(x) = %.2e, exitflag = %d\n', ...
        rn0, f0s, norm(x0s), ef0);
    assert(rn0 < 1e-10 && norm(x0s) < 1e-6, 'zero-target IK should stay at zero');

    % candidate tip targets: translations and an Rx rotation (expected
    % solvable — the released internal joints give the full 3T1R task
    % manifold {3 translations ⊕ rotation about X}) and a small Ry rotation
    % (expected NOT closable — rotation about Y is outside the manifold).
    mkT = @(v) [eye(3), v(:); 0 0 0 1];
    ca = deg2rad(2);
    DrotRy = [cos(ca) 0 sin(ca); 0 1 0; -sin(ca) 0 cos(ca)];
    cx = deg2rad(10);
    DrotX = [1 0 0; 0 cos(cx) -sin(cx); 0 sin(cx) cos(cx)];
    cands = {
        'trans +X 10mm', mkT([10; 0; 0])
        'trans -X 5mm',  mkT([-5; 0; 0])
        'trans +Y 5mm',  mkT([0; 5; 0])
        'trans -Y 5mm',  mkT([0; -5; 0])
        'trans +Z 5mm',  mkT([0; 0; 5])
        'trans -Z 5mm',  mkT([0; 0; -5])
        'rot Rx 10deg',  [DrotX, [0;0;0]; 0 0 0 1]
        'rot Ry 2deg',   [DrotRy, [0;0;0]; 0 0 0 1]
    };

    rng(7);  % reproducibility for multi-start
    bestSolved = false; bestLabel = ''; bestT = []; bestX = [];
    passFlag = false(size(cands, 1), 1);
    xByRow = cell(size(cands, 1), 1);
    for c = 1:size(cands, 1)
        label = cands{c,1}; T = cands{c,2};
        cs.setTipTarget(T);
        [x, fval, exitflag, rNorm, nTries] = localSolveMultistart(cs, ikOpts, 4);

        % world-rooted FK estimates of the tip pose from both branches
        qMapC = containers.Map();
        for i = 1:numel(cs.UnknownNames)
            qMapC(cs.UnknownNames{i}) = x(i);
        end
        E_Lc = P_L * core.RigidBodyMath.invT( ...
            localEvalFrame(e, qMapC, 'manipulator_L.ground'));
        E_Rc = P_R * core.RigidBodyMath.invT( ...
            localEvalFrame(e, qMapC, 'manipulator_R.ground'));
        errLc = norm(E_Lc - T, 'fro');
        errRc = norm(E_Rc - T, 'fro');
        fprintf(['  [12] %-15s |r| = %.3e  fval = %.3e  ' ...
            'errL = %.3e  errR = %.3e  tries = %d  exitflag = %d\n'], ...
            label, rNorm, fval, errLc, errRc, nTries, exitflag);

        isNonZero = norm(T - eye(4), 'fro') > 1e-9;
        passFlag(c) = isNonZero && rNorm < 1e-6 && errLc < 1e-6 && errRc < 1e-6;
        xByRow{c} = x;
        if passFlag(c) && ~bestSolved
            bestSolved = true; bestLabel = label; bestT = T; bestX = x;
        end
    end
    assert(bestSolved, ...
        'no non-zero tip target passed full-residual + world-rooted checks');
    ryRow = find(strcmp(cands(:,1), 'rot Ry 2deg'));
    assert(~passFlag(ryRow), ...
        'Ry rotation unexpectedly closed — task manifold assumes Rx only');
    rxRow = find(strcmp(cands(:,1), 'rot Rx 10deg'));
    assert(passFlag(rxRow), ...
        'Rx rotation target not solved — released 1R joint capability broken');
    fprintf('  [12] passed targets: %s\n', strjoin(cands(passFlag, 1).', ', '));
    fprintf('  [12] verified target: %s\n', bestLabel);

    % the Rx solution should follow the 1:1 hinge map q1 = φ, q2 = −φ
    xr = xByRow{rxRow};
    wrap2pi = @(a) mod(a + pi, 2*pi) - pi;
    j1r = wrap2pi(xr(7)); j2r = wrap2pi(xr(8));
    devJ = max(abs(j1r - cx), abs(j2r + cx));
    fprintf(['  [12] Rx solution hinges: q1 = % .6f rad, q2 = % .6f rad ' ...
        '(dev from ±φ: %.2e)\n'], j1r, j2r, devJ);
    assert(devJ < 1e-4, 'Rx hinge map q1 = φ, q2 = −φ not reproduced by solver');

    % ---- detailed world-rooted verification for the verified target ----
    D = bestT;
    cs.setTipTarget(D);
    qMap = containers.Map();
    for i = 1:numel(cs.UnknownNames)
        qMap(cs.UnknownNames{i}) = bestX(i);
    end
    T_nL = localEvalFrame(e, qMap, 'manipulator_L.ground');  % T_tip←g_L
    T_nR = localEvalFrame(e, qMap, 'manipulator_R.ground');  % T_tip←g_R
    E_L = P_L * core.RigidBodyMath.invT(T_nL);   % T_W←tip from L branch
    E_R = P_R * core.RigidBodyMath.invT(T_nR);   % T_W←tip from R branch
    errL  = norm(E_L - D, 'fro');
    errR  = norm(E_R - D, 'fro');
    errLR = norm(E_L - E_R, 'fro');
    fprintf(['  [12] world-rooted FK: ||E_L-ΔT|| = %.3e, ||E_R-ΔT|| = %.3e, ' ...
        '||E_L-E_R|| = %.3e\n'], errL, errR, errLR);
    assert(errL < 1e-6 && errR < 1e-6, 'world-rooted tip pose does not match target');
    assert(errLR < 1e-6, 'branches disagree on tip pose');

    % full 6-DOF closure residuals at the solution (hand-computed per branch)
    for i = 1:2
        if i == 1; T_ni = T_nL; P_i = P_L; else; T_ni = T_nR; P_i = P_R; end
        T_res_i = core.RigidBodyMath.invT(T_ni) * (D \ P_i);
        e6 = poseError6(T_res_i);
        fprintf('  [12] cut%d full residual [tx ty tz | rx ry rz]: [%s]\n', i, ...
            strjoin(arrayfun(@(v) sprintf('% .3e', v), e6(:).', ...
            'UniformOutput', false), ' '));
        assert(norm(e6) < 1e-6, 'full closure residual too large');
    end

    fprintf('\n=== test_closure_solver_m_rex_3t1r: all checks passed ===\n');
end

%% ---- local helpers ----

function T = localEvalFrame(e, qMap, frameName)
%LOCALEVALFRAME  Numeric FK for one frame via the Expander's symbolic poses.
    poses = e.evaluateNumericDirect(qMap, {frameName});
    T = poses(frameName);
end

function e6 = poseError6(T)
%POSEERROR6  Z-Y-X pose error decomposition (mirrors ClosureSolver local fn).
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    ry = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
    rx = atan2(R(3,2), R(3,3));
    rz = atan2(R(2,1), R(1,1));
    e6 = [p; rx; ry; rz];
end

function J = localJacobian(fh, x0)
%LOCALJACOBIAN  Central-difference Jacobian of a vector-valued handle.
    x0 = x0(:);
    n = numel(x0);
    J = zeros(numel(fh(x0)), n);
    h = 1e-6;
    for i = 1:n
        xp = x0; xp(i) = xp(i) + h;
        xm = x0; xm(i) = xm(i) - h;
        J(:,i) = (fh(xp) - fh(xm)) / (2*h);
    end
end

function [xBest, fvalBest, exitflagBest, rnBest, nTries] = localSolveMultistart(cs, opts, nRestarts)
%LOCALSOLVEMULTISTART  solveIK from zero init, then random restarts if needed.
    nx = numel(cs.UnknownSyms);
    [xBest, fvalBest, exitflagBest] = cs.solveIK(zeros(nx,1), opts);
    rnBest = norm(cs.evalResidual(xBest));
    nTries = 1;
    while rnBest > 1e-6 && nTries <= nRestarts
        x0 = 25 * (rand(nx,1) - 0.5);
        [x, fval, ef] = cs.solveIK(x0, opts);
        rn = norm(cs.evalResidual(x));
        nTries = nTries + 1;
        if rn < rnBest
            xBest = x; fvalBest = fval; exitflagBest = ef; rnBest = rn;
        end
    end
end

function writeExplicitConfig(outPath, baseText, groundRef, T)
%WRITEEXPLICITCONFIG  Inject an explicit "T:" matrix under a world_binding entry.
    rowStrs = arrayfun(@(r) sprintf('      - [%s]', strjoin( ...
        arrayfun(@(v) sprintf('%.12g', v), T(r,:), 'UniformOutput', false), ', ')), ...
        1:4, 'UniformOutput', false);
    block = ['    T:' newline strjoin(rowStrs(:).', newline)];
    oldLine = ['  - ground: ' groundRef];
    assert(contains(baseText, oldLine), 'anchor line not found in config text');
    newText = strrep(baseText, oldLine, [oldLine newline block]);
    fid = fopen(outPath, 'w');
    assert(fid > 0, 'cannot open %s', outPath);
    fwrite(fid, newText);
    fclose(fid);
end

function yml = localBaselineFixtureYaml()
%LOCALBASELINEFIXTUREYAML  Former mrex-3t1r_001 execution-config.yaml (A.4b).
%   Reference partition of the 3T baseline: internal joints known (default 0,
%   KNOWNVALUES-overridable) and components [tx,ty,tz] per cut.  Written to a
%   tmp file so KNOWNVALUES and the square 6x6 baseline stay covered while
%   the case converges onto the single 3T1R operational config.
    yml = strjoin({ ...
        'mode: closed_loop'
        'endFrame: pipette.tip_origin'
        ''
        'world_binding:'
        '  - ground: manipulator_L.ground'
        '  - ground: manipulator_R.ground'
        ''
        'closure_cuts:'
        '  - near: manipulator_L.ground'
        '    anchor: world'
        '    components: [tx, ty, tz]'
        '  - near: manipulator_R.ground'
        '    anchor: world'
        '    components: [tx, ty, tz]'
        ''
        'known:'
        '  - pipette.tip_origin'
        '  - joint_1.q'
        '  - joint_2.q'
        ''
        'unknown:'
        '  - manipulator_L.dx'
        '  - manipulator_L.dy'
        '  - manipulator_L.dz'
        '  - manipulator_R.dx'
        '  - manipulator_R.dy'
        '  - manipulator_R.dz'
        ''
        'tolerances:'
        '  translation_mm: 0.001'
        '  rotation_rad: 0.001'
        }, newline);
end

function checkError(fcn, expectedId, label)
    try
        fcn();
        error('w3check:noError', '[%s] no error raised (expected "%s").', label, expectedId);
    catch ME
        if strcmp(ME.identifier, 'w3check:noError')
            fprintf(2, '  [FAIL] %s: no error raised\n', label);
            rethrow(ME);
        end
        if ~strcmp(ME.identifier, expectedId)
            fprintf(2, '  [FAIL] %s: expected "%s", got "%s": %s\n', ...
                label, expectedId, ME.identifier, ME.message);
            error('w3check:wrongError', '%s', label);
        end
        fprintf('  [PASS] %-46s -> %s\n', label, expectedId);
    end
end
