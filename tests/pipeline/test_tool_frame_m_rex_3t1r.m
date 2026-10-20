function test_tool_frame_m_rex_3t1r()
%TEST_TOOL_FRAME_M_REX_3T1R  L3 tool-frame (world-aligned task frame) checks.
%   Builds the mrex-3t1r_001 pipeline and verifies the tool_frame feature
%   (L3 execution layer, 2026-10-14).  The case config now declares the
%   tilted reference (rpy [0, "45deg", 0] → U = Ry(45°), world ≡ tool-frame
%   zero pose, tip initial attitude −45° in world):
%     [1] case config declares tool_frame: ToolFrame.Present = true,
%         T = Ry(45°)
%     [2] back-compat fixture (case copy with the block stripped): U = I₄,
%         setToolTarget ≡ setTipTarget (residual equality)
%     [3] spelling/negative fixtures (stripped copy + appended block):
%         numeric-rad spelling agrees with the case's "45deg"; translation
%         composes with rotation into T = [R t; 0 1]; missing transform is
%         rejected
%     [4] ClosureSolver resolves ToolTransform = Ry(45°) from the case config;
%         a translation-bearing tool_frame resolves U = [R t; 0 1], keeps
%         D(0) = I₄ and puts the derived tip origin at −Rᵀt
%     [5] zero pose: evalToolPose(0) ≈ I₄ (world-aligned tool frame); derived
%         tip initial attitude U⁻¹ = Ry(−45°) matches the design example
%         ("pipette frame 0,0,0,0,-45°,0"); cross-checked against an
%         independent KinematicModel world-rooted FK path
%     [6] algebraic equivalence: setToolTarget(D) ≡ setTipTarget(U·D·U⁻¹)
%         (residual equality at a probe configuration)
%     [7] IK end-to-end with tool targets: (a) +5 mm world-x translation;
%         (b) world-axis rotation achievable by the 3T1R manifold (about
%         (cos45°,0,sin45°), θ = 5°) with L/R branch agreement; (c) a pure
%         world-z rotation is rejected (internal axis U·ẑ ∦ x̂ — outside the
%         mechanism manifold), residual stays bounded away from zero
%
%   Requires: Symbolic Math Toolbox
%
%   Usage:
%       addpath(genpath('../../scripts/matlab'));
%       run('test_tool_frame_m_rex_3t1r.m');

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(fullfile(rootDir, 'scripts', 'matlab')));

    dslFile  = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex-3t1r_001', ...
        'robot_description.yaml');
    execFile = fullfile(rootDir, 'specs', 'dsl', 'cases', 'mrex-3t1r_001', ...
        'execution-config.yaml');

    tmpDir = tempname(); mkdir(tmpDir);
    cleanupObj = onCleanup(@() rmdir(tmpDir, 's')); %#ok<NASGU>

    fprintf('=== test_tool_frame_m_rex_3t1r: L3 tool-frame checks ===\n');

    % ---- [1] case config declares the tilted tool frame ----
    e = ir.Expander(dslFile);
    cfgBase = ir.ExecutionConfig(execFile, e.SymbolRegistry, e.EdgeGraph_);
    assert(cfgBase.ToolFrame.Present, ...
        'case config should declare tool_frame (tilted reference usage)');
    devCase = norm(cfgBase.ToolFrame.T - localTO(core.RigidBodyMath.roty(pi/4), [0; 0; 0]), 'fro');
    assert(devCase < 1e-12, 'case tool_frame should be Ry(45°), dev = %.2e', devCase);
    fprintf('  [1] case config: tool_frame declared, T = Ry(45°) (dev = %.2e)\n', devCase);

    % ---- [2] back-compat fixture: strip the case tool_frame → U = I₄ ----
    noToolFile = fullfile(tmpDir, 'exec-tool-stripped.yaml');
    strippedTxt = localStripToolFrame(fileread(execFile));
    localWriteText(noToolFile, strippedTxt);
    cfgNo = ir.ExecutionConfig(noToolFile, e.SymbolRegistry, e.EdgeGraph_);
    assert(~cfgNo.ToolFrame.Present, 'stripped fixture: ToolFrame should be absent');
    assert(norm(cfgNo.ToolFrame.T - eye(4), 'fro') < 1e-15, ...
        'stripped fixture: ToolFrame.T should default to I4');

    csBase = solver.ClosureSolver(e.EdgeGraph_, cfgNo, e.JointVarMap);
    nx = numel(csBase.UnknownSyms);
    assert(norm(csBase.ToolTransform - eye(4), 'fro') < 1e-15, ...
        'stripped fixture: ToolTransform should be I4 (no tool_frame)');

    qProbe = 0.01 * (1:nx).';
    Dprobe = localTO(core.RigidBodyMath.rotz(pi/12), [2; -1; 3]);
    csBase.setTipTarget(Dprobe);
    rTip = csBase.evalResidual(qProbe);
    csBase.setToolTarget(Dprobe);
    rTool = csBase.evalResidual(qProbe);
    dEq = norm(rTip - rTool);
    assert(dEq < 1e-15, ...
        'stripped fixture: setToolTarget must equal setTipTarget when U = I4 (|Δr| = %.2e)', dEq);
    fprintf(['  [2] stripped (no tool_frame): ToolTransform = I4, ' ...
        'setToolTarget ≡ setTipTarget (|Δr| = %.2e)\n'], dEq);

    % ---- [3] spelling variant (numeric rad) + negative case ----
    radFile = fullfile(tmpDir, 'exec-tool-rad.yaml');
    localWriteToolFrameFixture(strippedTxt, radFile, ...
        localToolFrameBlockRpy(sprintf('%.17g', pi/4)));
    cfgRad = ir.ExecutionConfig(radFile, e.SymbolRegistry, e.EdgeGraph_);
    assert(cfgRad.ToolFrame.Present, 'rad fixture: ToolFrame should be present');
    devSpell = norm(cfgRad.ToolFrame.T - cfgBase.ToolFrame.T, 'fro');
    assert(devSpell < 1e-15, 'numeric-rad spelling and the case "45deg" must agree');

    % translation (mm) composes with rotation: T = [R t; 0 1]
    tRel = [4; -2; 7];
    trFile = fullfile(tmpDir, 'exec-tool-translation.yaml');
    localWriteToolFrameFixture(strippedTxt, trFile, strjoin({ ...
        'tool_frame:'
        '  transform:'
        '    translation: [4, -2, 7]'
        '    rotation: { rpy: [0, 45deg, 0] }'
        }, newline));
    cfgTr = ir.ExecutionConfig(trFile, e.SymbolRegistry, e.EdgeGraph_);
    devTr = norm(cfgTr.ToolFrame.T - ...
        localTO(core.RigidBodyMath.roty(pi/4), tRel), 'fro');
    assert(devTr < 1e-12, ...
        'translation must enter T_endFrame<-tool (dev = %.2e)', devTr);

    badFile = fullfile(tmpDir, 'exec-tool-bad.yaml');
    localWriteToolFrameFixture(strippedTxt, badFile, strjoin({ ...
        'tool_frame:'
        '  translation: [1, 2, 3]'
        }, newline));
    localExpectError(@() ir.ExecutionConfig(badFile, e.SymbolRegistry, e.EdgeGraph_), ...
        'ir:ExecutionConfig:invalidToolFrame', 'tool_frame without transform');
    fprintf(['  [3] spellings: numeric rad ≡ case "45deg" (|Δ| = %.2e); ' ...
        'translation composed (dev = %.2e); missing transform rejected\n'], ...
        devSpell, devTr);

    % ---- [4] solver resolves ToolTransform = Ry(45°) from the case config ----
    cs = solver.ClosureSolver(e.EdgeGraph_, cfgBase, e.JointVarMap);
    U = cs.ToolTransform;
    Uinv = core.RigidBodyMath.invT(U);
    devU = norm(U - localTO(core.RigidBodyMath.roty(pi/4), [0; 0; 0]), 'fro');
    assert(devU < 1e-12, 'ToolTransform should resolve to Ry(45°), dev = %.2e', devU);

    % translation-bearing tool_frame: U = [R t; 0 1]; the tool origin is the
    % rotation center, so D(0) = I4 still holds and the derived tip origin in
    % the tool world is invT(U) = [Rᵀ, −Rᵀt]
    csTr = solver.ClosureSolver(e.EdgeGraph_, cfgTr, e.JointVarMap);
    UTr = csTr.ToolTransform;
    devUtr = norm(UTr - localTO(core.RigidBodyMath.roty(pi/4), tRel), 'fro');
    assert(devUtr < 1e-12, 'translation ToolTransform deviates (%.2e)', devUtr);

    nxTr = numel(csTr.UnknownSyms);
    D0tr = csTr.evalToolPose(zeros(nxTr, 1));
    assert(norm(D0tr - eye(4), 'fro') < 1e-12, ...
        'translation tool_frame: zero pose should still be I4');

    UinvTr = core.RigidBodyMath.invT(UTr);
    devTipTr = norm(UinvTr - localTO(core.RigidBodyMath.roty(-pi/4), ...
        -core.RigidBodyMath.roty(pi/4).' * tRel), 'fro');
    assert(devTipTr < 1e-12, 'derived tip origin should be −Rᵀt (dev = %.2e)', devTipTr);

    qProbeTr = 0.01 * (1:nxTr).';
    DtrProbe = localTO(core.RigidBodyMath.rotz(pi/12), [2; -1; 3]);
    csTr.setTipTarget(UTr * DtrProbe * UinvTr);
    rTrA = csTr.evalResidual(qProbeTr);
    csTr.setToolTarget(DtrProbe);
    rTrB = csTr.evalResidual(qProbeTr);
    dEqTr = norm(rTrA - rTrB);
    assert(dEqTr < 1e-15, ...
        'translation tool_frame: setToolTarget does not match the conjugated target (|dr| = %.2e)', ...
        dEqTr);

    fprintf(['  [4] ToolTransform = Ry(45°) (dev = %.2e); translation ' ...
        'U = [R t] (dev = %.2e), D(0) = I4, tip origin = −Rᵀt (dev = %.2e)\n'], ...
        devU, devUtr, devTipTr);

    % ---- [5] zero-pose semantics (world-aligned tool frame) ----
    kmL = solver.KinematicModel(e.EdgeGraph_, 'manipulator_L.ground', e.JointVarMap);
    kmR = solver.KinematicModel(e.EdgeGraph_, 'manipulator_R.ground', e.JointVarMap);
    cutL = cs.ClosureCuts(strcmp({cs.ClosureCuts.near}, 'manipulator_L.ground'));
    cutR = cs.ClosureCuts(strcmp({cs.ClosureCuts.near}, 'manipulator_R.ground'));

    D0 = cs.evalToolPose(zeros(nx, 1));
    dev0 = norm(D0 - eye(4), 'fro');
    assert(dev0 < 1e-12, 'zero pose: tool frame should be world-aligned (dev = %.2e)', dev0);

    % derived tip initial attitude in world = U⁻¹ = Ry(−45°) — matches the
    % design example ("pipette frame 0,0,0,0,-45°,0" with tool = tip·Ry(45°))
    devTip = norm(Uinv - localTO(core.RigidBodyMath.roty(-pi/4), [0; 0; 0]), 'fro');
    assert(devTip < 1e-12, 'derived tip attitude should be Ry(−45°), dev = %.2e', devTip);

    % cross-check evalToolPose(0) against an independent KinematicModel path
    qMap0 = localQMap(cs, zeros(nx, 1));
    DL0 = Uinv * (cutL.T_bind * core.RigidBodyMath.invT(kmL.eval(qMap0))) * U;
    devL0 = norm(DL0 - D0, 'fro');
    assert(devL0 < 1e-9, 'evalToolPose disagrees with KinematicModel path (dev = %.2e)', devL0);
    fprintf(['  [5] zero pose: |D(0) − I4| = %.2e; tip attitude U⁻¹ = Ry(−45°) ' ...
        '(dev = %.2e); km cross-check dev = %.2e\n'], dev0, devTip, devL0);

    % ---- [6] algebraic equivalence with conjugated tip targets ----
    Dprobe2 = localTO(core.RigidBodyMath.rotz(pi/12), [2; -1; 3]);
    cs.setTipTarget(U * Dprobe2 * Uinv);
    rA = cs.evalResidual(qProbe);
    cs.setToolTarget(Dprobe2);
    rB = cs.evalResidual(qProbe);
    dEq2 = norm(rA - rB);
    assert(dEq2 < 1e-15, 'setToolTarget must equal conjugated setTipTarget (|Δr| = %.2e)', dEq2);
    fprintf('  [6] setToolTarget(D) ≡ setTipTarget(U·D·U⁻¹): |Δr| = %.2e\n', dEq2);

    % ---- [7] IK end-to-end with tool-frame targets ----
    % Central finite differences required for tight convergence on this
    % system (see test_closure_solver_m_rex_3t1r [12] for the probe).
    ikOpts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', 'Display', 'off', ...
        'FiniteDifferenceType', 'central', ...
        'OptimalityTolerance', 1e-12, 'StepTolerance', 1e-12, ...
        'MaxFunctionEvaluations', 4000, 'MaxIterations', 400);

    % (a) translation: +5 mm along world x
    Dt = localTO(eye(3), [5; 0; 0]);
    cs.setToolTarget(Dt);
    [xa, ~, efa, rna, nta] = localSolveMultistart(cs, ikOpts, 4);
    Da = cs.evalToolPose(xa);
    ea = norm(Da - Dt, 'fro');
    fprintf(['  [7a] +5mm world-x tool target: |r| = %.2e, |D−D_target| = %.2e, ' ...
        'tries = %d, exitflag = %d\n'], rna, ea, nta, efa);
    assert(rna < 1e-6 && ea < 1e-6, 'translation tool target not reached');

    % (b) world-axis rotation achievable by the 3T1R manifold: internal
    %     Rx(θ) ↔ world axis a = U⁻¹·x̂ = (cos45°, 0, sin45°)
    th = deg2rad(5);
    aAxis = Uinv(1:3, 1:3) * [1; 0; 0];
    Dt2 = localTO(core.RigidBodyMath.axang(aAxis, th), [0; 0; 0]);
    cs.setToolTarget(Dt2);
    [xb, ~, efb, rnb, ntb] = localSolveMultistart(cs, ikOpts, 4);
    Db = cs.evalToolPose(xb);
    eb = norm(Db - Dt2, 'fro');

    % L/R world-rooted branch agreement at the solution
    qMapB = localQMap(cs, xb);
    DLb = Uinv * (cutL.T_bind * core.RigidBodyMath.invT(kmL.eval(qMapB))) * U;
    DRb = Uinv * (cutR.T_bind * core.RigidBodyMath.invT(kmR.eval(qMapB))) * U;
    eLRb = norm(DLb - DRb, 'fro');
    fprintf(['  [7b] world-axis (cos45°,0,sin45°) rotation 5°: |r| = %.2e, ' ...
        '|D−D_target| = %.2e, L/R gap = %.2e, tries = %d, exitflag = %d\n'], ...
        rnb, eb, eLRb, ntb, efb);
    assert(rnb < 1e-6 && eb < 1e-6, 'manifold rotation tool target not reached');
    assert(eLRb < 1e-9, 'L/R branches disagree on tool pose (gap = %.2e)', eLRb);

    % (c) pure world-z rotation lies outside the mechanism manifold
    %     (internal axis U·ẑ ∦ x̂): residual must stay bounded away from 0
    Dt3 = localTO(core.RigidBodyMath.rotz(deg2rad(5)), [0; 0; 0]);
    cs.setToolTarget(Dt3);
    xc = cs.solveIK(zeros(nx, 1), ikOpts);
    rnc = norm(cs.evalResidual(xc));
    fprintf('  [7c] pure world-z rotation correctly rejected: |r| = %.2e (> 1e-4)\n', rnc);
    assert(rnc > 1e-4, 'world-z rotation unexpectedly achievable (|r| = %.2e)', rnc);

    fprintf('\n=== test_tool_frame_m_rex_3t1r: all checks passed ===\n');
end

%% ---- local helpers ----

function T = localTO(R, p)
%LOCALTO  Homogeneous transform from rotation R and position p.
    T = [R, p(:); 0 0 0 1];
end

function qMap = localQMap(cs, x)
%LOCALQMAP  containers.Map from solver unknown names to values.
    qMap = containers.Map();
    for i = 1:numel(cs.UnknownNames)
        qMap(cs.UnknownNames{i}) = x(i);
    end
end

function localWriteText(outFile, txt)
%LOCALWRITETEXT  Write text to a file.
    fid = fopen(outFile, 'w');
    assert(fid > 0, 'cannot open %s for writing', outFile);
    fwrite(fid, txt);
    fclose(fid);
end

function localWriteToolFrameFixture(baseText, outFile, blockText)
%LOCALWRITETOOLFRAMEFIXTURE  Append a raw tool_frame block to a base config text.
    fid = fopen(outFile, 'w');
    assert(fid > 0, 'cannot open %s for writing', outFile);
    fwrite(fid, [baseText newline newline blockText newline]);
    fclose(fid);
end

function txtOut = localStripToolFrame(txt)
%LOCALSTRIPTOOLFRAME  Remove the top-level tool_frame block (and its children).
%   The case config declares a tool_frame; stripping it yields the back-compat
%   fixture that exercises the "no tool_frame" path (U = I₄).
    lines = regexp(txt, '\r\n|\r|\n', 'split');
    keep = true(1, numel(lines));
    skip = false;
    for k = 1:numel(lines)
        ln = lines{k};
        if ~skip && ~isempty(regexp(ln, '^tool_frame:\s*$', 'once'))
            skip = true;
            keep(k) = false;
            continue;
        end
        if skip
            if isempty(ln) || isspace(ln(1))
                keep(k) = false;        % blank or indented child of the block
            else
                skip = false;           % next top-level key — keep it
            end
        end
    end
    assert(any(~keep), 'localStripToolFrame: no tool_frame block found');
    txtOut = strjoin(lines(keep), newline);
end

function block = localToolFrameBlockRpy(angleToken)
%LOCALTOOLFRAMEBLOCKRPY  tool_frame block with rpy [0, angleToken, 0].
    block = strjoin({
        'tool_frame:'
        '  transform:'
        sprintf('    rotation: { rpy: [0, %s, 0] }', angleToken)
        }, newline);
end

function [xBest, fvalBest, exitflagBest, rnBest, nTries] = localSolveMultistart(cs, opts, nRestarts)
%LOCALSOLVEMULTISTART  solveIK from zero init, then random restarts if needed.
    nx = numel(cs.UnknownSyms);
    [xBest, fvalBest, exitflagBest] = cs.solveIK(zeros(nx, 1), opts);
    rnBest = norm(cs.evalResidual(xBest));
    nTries = 1;
    while rnBest > 1e-6 && nTries <= nRestarts
        x0 = 25 * (rand(nx, 1) - 0.5);
        [x, fval, ef] = cs.solveIK(x0, opts);
        rn = norm(cs.evalResidual(x));
        nTries = nTries + 1;
        if rn < rnBest
            xBest = x; fvalBest = fval; exitflagBest = ef; rnBest = rn;
        end
    end
end

function localExpectError(fcn, expectedId, label)
%LOCALEXPECTERROR  Assert that FCN raises an error with the expected id.
    try
        fcn();
        error('toolFrameCheck:noError', '[%s] no error raised (expected "%s").', ...
            label, expectedId);
    catch ME
        if strcmp(ME.identifier, 'toolFrameCheck:noError')
            fprintf(2, '  [FAIL] %s: no error raised\n', label);
            rethrow(ME);
        end
        if ~strcmp(ME.identifier, expectedId)
            fprintf(2, '  [FAIL] %s: expected "%s", got "%s": %s\n', ...
                label, expectedId, ME.identifier, ME.message);
            error('toolFrameCheck:wrongError', '%s', label);
        end
        fprintf('  [3] %s -> %s OK\n', label, expectedId);
    end
end
