%% compute_ground_truth_single_closed_loop.m
% FK→IK round-trip validation for the parallelogram single-closed-loop mechanism.
%
% Pipeline:
%   1. Define reference joint configuration (known to satisfy closure)
%   2. FK: compute end-frame pose from reference joint values via Expander symbolic poses
%   3. IK: solve for joint values from the target pose via ClosureSolver.solveIKWithTarget
%   4. Compare solved joint values to reference values
%   5. Verify closure + pose residuals at solution
%
% This is a cross-validation that the FK and IK paths are inverse-consistent:
%   IK(FK(q_ref)) ≈ q_ref   (up to the 1-DOF manifold symmetry)
%
% Parallelogram kinematics (verified):
%   AB = CD  (one diagonal),  BD = AC  (other diagonal),  AB = -BD
%   Reference: [AB=+θ, BD=-θ, CD=+θ, AC=-θ]
%
% Requires: Symbolic Math Toolbox, Optimization Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('compute_ground_truth_single_closed_loop.m');

fprintf('=== FK→IK Round-Trip Validation: single-closed-loop ===\n\n');

%% Paths
dslFile  = '../../specs/dsl/cases/single-closed-loop/robot_description.yaml';
execFile = '../../specs/dsl/cases/single-closed-loop/execution-config.yaml';

%% 1. Build symbolic pipeline
fprintf('1. Building symbolic pipeline ... ');
eSym = ir.Expander(dslFile);
fprintf('OK (%d instances, %d joints)\n', numel(eSym.Instances), eSym.JointVarMap.Count);

%% 2. Build ExecutionConfig and ClosureSolver
fprintf('2. Building ExecutionConfig + ClosureSolver ... ');
cfg = ir.ExecutionConfig(execFile, eSym.SymbolRegistry, eSym.EdgeGraph_);
cs = solver.ClosureSolver(eSym.EdgeGraph_, cfg, eSym.JointVarMap);
fprintf('OK (%d cuts, %d unknown vars)\n', cs.NumCuts, numel(cs.UnknownNames));

%% 3. Define reference joint configuration
%   Parallelogram closed: AB=+30°, BD=-30°, CD=+30°, AC=-30°
fprintf('\n3. Reference joint configuration:\n');
thetaRef = pi/6;
qRef = containers.Map();
qRef('joint_AB.q') =  thetaRef;
qRef('joint_BD.q') = -thetaRef;
qRef('joint_CD.q') =  thetaRef;
qRef('joint_AC.q') = -thetaRef;

for i = 1:numel(cs.UnknownNames)
    if isKey(qRef, cs.UnknownNames{i})
        fprintf('   %-20s = %+.6f rad (%+.1f°)\n', ...
            cs.UnknownNames{i}, qRef(cs.UnknownNames{i}), ...
            rad2deg(qRef(cs.UnknownNames{i})));
    end
end

%% 4. Verify closure at reference config
fprintf('\n4. Verifying closure at reference config ... ');
rRef = cs.evalResidual(qRef);
rNormRef = norm(rRef);
fprintf('|r| = %.3e\n', rNormRef);
assert(rNormRef < 1e-10, ...
    'Reference configuration does not satisfy closure (|r|=%.3e).', rNormRef);
fprintf('   PASS: reference config is a valid closed configuration.\n');

%% 5. FK: compute end-frame pose from reference joint values
endFrame = 'frame_link_D2.frame_hyper_cube';
fprintf('\n5. FK: computing T_target = FK(q_ref) for endFrame="%s" ...\n', endFrame);

% FK along the spanning tree from the Expander's symbolic poses
posesRef = eSym.evaluateNumericDirect(qRef, {endFrame});
T_target = posesRef(endFrame);  % 4×4 double

fprintf('   T_target (world → endFrame):\n');
fprintf('   Position: [%+.4f, %+.4f, %+.4f] mm\n', T_target(1,4), T_target(2,4), T_target(3,4));
fprintf('   Rotation matrix:\n');
for ri = 1:3
    fprintf('     [%+.6f  %+.6f  %+.6f]\n', T_target(ri,1), T_target(ri,2), T_target(ri,3));
end

% Sanity: T_target should not be identity (non-zero config)
assert(norm(T_target - eye(4)) > 1e-6, ...
    'T_target is identity — reference config may be zero-pose. Expected non-zero FK output.');
fprintf('   PASS: T_target is non-identity (non-zero FK output).\n');

%% 6. IK: solve for joint values from T_target
fprintf('\n6. IK: solving for joint values from T_target ...\n');

% Initial guess: zero pose (far from reference to test convergence robustness)
x0 = zeros(numel(cs.UnknownSyms), 1);
[x_solved, fval, exitflag, report] = cs.solveIKWithTarget(T_target, endFrame, x0);

fprintf('\n   --- IK Results ---\n');
fprintf('   Exit flag: %d, Final objective ‖r‖² = %.3e\n', exitflag, fval);
fprintf('   Solved joint values:\n');
for i = 1:numel(cs.UnknownNames)
    fprintf('     %-20s = %+.6f rad (%+.1f°)\n', ...
        cs.UnknownNames{i}, x_solved(i), rad2deg(x_solved(i)));
end

fprintf('\n   --- Residual Breakdown ---\n');
fprintf('   Closure residual norm:  %.3e\n', report.closureNorm);
fprintf('   Pose residual norm:     %.3e\n', report.poseNorm);

%% 7. Validate: closure residual at solution
fprintf('\n7. Validation: closure residual at solution ... ');
assert(report.closureNorm < 1e-4, ...
    'Closure residual at IK solution = %.3e (expected < 1e-4).', report.closureNorm);
fprintf('PASS (%.3e < 1e-4)\n', report.closureNorm);

%% 8. Validate: pose residual at solution
fprintf('8. Validation: pose residual at solution ... ');
assert(report.poseNorm < 1e-4, ...
    'Pose residual at IK solution = %.3e (expected < 1e-4).', report.poseNorm);
fprintf('PASS (%.3e < 1e-4)\n', report.poseNorm);

%% 9. FK consistency: re-compute end-frame pose from solved joint values
fprintf('\n9. FK consistency: T_FK(q_solved) vs T_target ... ');
qSolved = containers.Map();
for i = 1:numel(cs.UnknownNames)
    qSolved(cs.UnknownNames{i}) = x_solved(i);
end
posesSolved = eSym.evaluateNumericDirect(qSolved, {endFrame});
T_solved = posesSolved(endFrame);
T_diff = T_target \ T_solved;  % should be ≈ I
poseErr = norm(T_diff(1:3,4)) + norm(T_diff(1:3,1:3) - eye(3), 'fro');
fprintf('pose error = %.3e\n', poseErr);
assert(poseErr < 1e-4, ...
    'FK(q_solved) differs from T_target (pose error = %.3e).', poseErr);
fprintf('   PASS: FK(q_solved) ≈ T_target.\n');

%% 10. Compare solved joint values to reference
fprintf('\n10. Joint value recovery:\n');
qRefVec = zeros(numel(cs.UnknownSyms), 1);
for i = 1:numel(cs.UnknownNames)
    if isKey(qRef, cs.UnknownNames{i})
        qRefVec(i) = qRef(cs.UnknownNames{i});
    end
end

dqDirect = norm(x_solved - qRefVec);
fprintf('   ‖q_solved - q_ref‖ (direct)   = %.3e\n', dqDirect);

% Also check the sign-flipped equivalent (valid due to 1-DOF symmetry):
%   [-θ, +θ, -θ, +θ] is also a valid closed config producing the same T_target
%   since flipping all signs preserves the parallelogram geometry.
dqFlipped = norm(x_solved + qRefVec);
fprintf('   ‖q_solved + q_ref‖ (flipped)  = %.3e\n', dqFlipped);

recoveryErr = min(dqDirect, dqFlipped);
if dqDirect < 1e-6
    fprintf('   ✓ Direct match: q_solved ≈ q_ref\n');
elseif dqFlipped < 1e-6
    fprintf('   ✓ Sign-flipped match: q_solved ≈ -q_ref (valid equivalent)\n');
else
    fprintf('   ⚠ Recovery error = %.3e (expected < 1e-6)\n', recoveryErr);
end
assert(recoveryErr < 1e-5, ...
    'Joint recovery error = %.3e (expected < 1e-5). FK→IK round-trip failed.', recoveryErr);
fprintf('   PASS: joint values recovered within tolerance.\n');

%% Summary
fprintf('\n=== FK→IK Round-Trip Validation PASSED ===\n');
fprintf('   Reference:  [%+.4f, %+.4f, %+.4f, %+.4f]\n', qRefVec);
fprintf('   Solved:     [%+.4f, %+.4f, %+.4f, %+.4f]\n', x_solved);
fprintf('   ‖r_closure‖ = %.3e,  ‖r_pose‖ = %.3e,  Δq ≤ %.3e\n', ...
    report.closureNorm, report.poseNorm, recoveryErr);
