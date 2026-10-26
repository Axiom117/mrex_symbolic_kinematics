classdef ClosureSolver < handle
%CLOSURESOLVER  Closed-loop constraint construction and numerical solving.
%   Builds symbolic loop-closure residuals from an expanded kinematic graph,
%   converts them to numeric function handles, and bridges to fmincon for
%   inverse kinematics solving.
%
%   Residual construction (two cut forms):
%     Frame-pair cuts (L2): the mate transform M (Rz(roll*2π/sym)*Rx(π))
%     encodes the physical connection.  At closure, T_far should equal
%     T_near * M.  The residual is defined as:
%         T_res = inv(T_near * M) * T_far   →   should be I at closure.
%     World-anchored cuts (L3, e.g. M-REx): T_mate = I₄ and the far side is
%     the constant calibrated binding.  At closure, T_near should equal
%     ΔT⁻¹ · T_bind (ΔT = tip task target, default I₄):
%         T_res = inv(T_near) * (ΔT⁻¹ · T_bind)   →   should be I at closure.
%     Both inverses use the closed-form SE(3) inverse (core.RigidBodyMath.invT):
%     division-free and dramatically smaller expansions than a literal `\`
%     matrix inverse, whose ~1e+170 coefficients overflow to Inf/NaN when the
%     numeric kernels are evaluated in double precision (A.5.4 export).
%     Both residuals are decomposed into a 6×1 pose error vector
%     [tx,ty,tz,rx,ry,rz] (Z-Y-X Euler angles for rotation).
%
%   Known joint values (e.g. passive internal joints) are substituted
%   numerically into all propagated poses before the residual is built
%   (default 0; override via the KNOWNVALUES map argument):
%       cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap, knownMap);
%
%   Tool frame (L3, optional): when the execution config declares a
%   tool_frame, the world is re-anchored to the tool-frame zero pose and
%   task targets / pose reports are expressed in the tool frame:
%       cs.setToolTarget(deltaTool)   % ΔT_tip = U · Δ_tool · U⁻¹
%       D = cs.evalToolPose(q)        % D(q) = U⁻¹ · T_rootWorld(q) · U
%   where U = T_root←tool (constant; requires a rigid root→endFrame chain).
%   With no tool_frame declared, U = I₄ and setToolTarget ≡ setTipTarget.
%
%   Usage:
%       e   = ir.Expander(dslYaml);
%       cfg = ir.ExecutionConfig(execYaml, e.SymbolRegistry, e.EdgeGraph_);
%       cs  = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
%
%       % Zero-pose verification:
%       cs.verifyZeroPose();
%
%       % Evaluate at specific joint values:
%       r = cs.evalResidual([q_AB; q_BD; q_CD; q_AC]);
%
%       % Symbolic (known-substituted) frame pose — for model export:
%       T_sym = cs.poseSym('manipulator_L.ground');
%
%       % L3 world-anchored IK: move the tip by ΔT, then solve:
%       cs.setTipTarget(deltaT);
%       [x_opt, fval] = cs.solveIK();
%
%       % Tool-frame targets (world ≡ tool-frame zero pose):
%       cs.setToolTarget(deltaTool);       % world-axis increment, tool frame
%       D = cs.evalToolPose(x_opt);        % achieved tool pose in world
%
%   See also: +ir/EdgeGraph, +ir/ExecutionConfig, +solver/exportKinematics

    % ---- public read-only properties ----
    properties (SetAccess = private)
        ClosureCuts   (:,1) struct          % struct array: near, far, anchor, components, T_mate, T_bind (world cuts)
        UnknownSyms   (1,:) sym             % sym array: unknown joint variable handles
        UnknownNames  (1,:) cell            % cell array: canonical names of unknown variables
        ResidualSym   (:,1) sym             % sym column vector: full residual expression
        Tolerances    (1,1) struct          % struct: translation_mm, rotation_rad
        NumCuts       (1,1) double          % number of closure cuts
        NumComponents (1,1) double          % total number of residual components
        TipTarget     (4,4) double = eye(4) % ΔT — tip task target for world-anchored cuts (default I₄)
        ToolTransform (4,4) double = eye(4) % U = T_root←tool when tool_frame declared (else I₄)
    end

    % ---- private properties ----
    properties (Access = private)
        Poses_                             % containers.Map: frame name → 4×4 sym
        JointVarMap_                       % containers.Map: canonical name → sym handle
        EdgeGraph_                         % ir.EdgeGraph handle
        ExecConfig_                        % ir.ExecutionConfig
        ResidualFunc_                      % cached numeric function handle
    end

    % ---- public methods ----
    methods

        %% Constructor: propagate poses and build symbolic residual.
        %   obj = ClosureSolver(EDGEGRAPH, EXECCONFIG, JOINTVARMAP)
        %   obj = ClosureSolver(EDGEGRAPH, EXECCONFIG, JOINTVARMAP, KNOWNVALUES)
        %     KNOWNVALUES : (optional) containers.Map — numeric values for
        %                   known joint variables (e.g. 'joint_1.q' → 0.1).
        %                   Missing entries default to 0.
        function obj = ClosureSolver(edgeGraph, execConfig, jointVarMap, knownValues)
            arguments
                edgeGraph    (1,1) ir.EdgeGraph
                execConfig   (1,1) ir.ExecutionConfig
                jointVarMap  containers.Map
                knownValues  containers.Map = containers.Map('KeyType','char','ValueType','any')
            end

            obj.EdgeGraph_ = edgeGraph;
            obj.ExecConfig_ = execConfig;
            obj.JointVarMap_ = jointVarMap;
            obj.Tolerances = execConfig.Tolerances;
            obj.ResidualFunc_ = [];

            % ---- validate closure mode ----
            rawCuts = execConfig.ClosureCuts;
            assert(~isempty(rawCuts), ...
                'solver:ClosureSolver:noCuts', ...
                'ExecutionConfig must contain at least one ClosureCut.');
            assert(strcmp(execConfig.Mode, 'closed_loop'), ...
                'solver:ClosureSolver:wrongMode', ...
                'ExecutionConfig mode must be "closed_loop", got "%s".', ...
                execConfig.Mode);

            % ---- propagate symbolic poses through the edge graph ----
            obj.Poses_ = edgeGraph.propagate();

            % ---- resolve tool-frame transform U = T_root←tool (constant) ----
            obj.ToolTransform = obj.resolveToolTransform();

            % ---- enrich ClosureCuts (T_mate / world-binding calibration) ----
            obj.ClosureCuts = obj.enrichClosureCuts(rawCuts);
            obj.NumCuts = numel(obj.ClosureCuts);

            % ---- substitute known joint values into all propagated poses ----
            % (default 0; must run AFTER enrichment so that world-binding
            % calibration always reflects the true all-zero pose)
            obj.substituteKnownValues(knownValues);

            % ---- collect unknown joint variable sym handles ----
            ujv = execConfig.getUnknownJointVars();
            nUnknown = numel(ujv);
            obj.UnknownSyms = sym(zeros(1, nUnknown));
            obj.UnknownNames = cell(1, nUnknown);
            for i = 1:nUnknown
                obj.UnknownSyms(i) = ujv(i).symHandle;
                obj.UnknownNames{i} = ujv(i).name;
            end

            % ---- build and cache full symbolic residual ----
            obj.ResidualSym = obj.buildFullResidual();
            obj.NumComponents = numel(obj.ResidualSym);

            % ---- guard: residual must depend only on unknown variables ----
            % NOTE: sym names are sanitized by matlab.lang.makeValidName in the
            % Expander (dots → underscores), so compare against the rendered
            % names of the unknown sym handles, not the canonical dotted names.
            resNames = arrayfun(@char, symvar(obj.ResidualSym), 'UniformOutput', false);
            unknownSymsNames = cellfun(@char, num2cell(obj.UnknownSyms), ...
                'UniformOutput', false);
            extraVars = setdiff(resNames, unknownSymsNames);
            assert(isempty(extraVars), ...
                'solver:ClosureSolver:unsubstitutedVars', ...
                ['Residual still contains non-unknown variables: %s. ' ...
                 'Add their values to the known list / KNOWNVALUES map.'], ...
                strjoin(extraVars, ', '));
        end

        %% buildResidualForCut  Symbolic 6-DOF residual for one closure cut.
        %   res = obj.buildResidualForCut(CUTINDEX)
        %     CUTINDEX : index into obj.ClosureCuts
        %     res      : N×1 sym — residual components for this cut
        function res = buildResidualForCut(obj, cutIndex)
            cut = obj.ClosureCuts(cutIndex);

            % validate frame existence
            assert(isKey(obj.Poses_, cut.near), ...
                'solver:ClosureSolver:frameNotFound', ...
                'Near frame "%s" not in propagated poses.', cut.near);

            if strcmp(core.CommonUtils.field(cut, 'anchor', ''), 'world')
                % --- world-anchored cut (L3) ---
                % Near (ground) frame must equal ΔT⁻¹ · T_bind, where T_bind
                % is the calibrated zero-pose binding (T_mate = I₄).
                assert(isfield(cut, 'T_bind') && ~isempty(cut.T_bind), ...
                    'solver:ClosureSolver:missingBinding', ...
                    'World-anchored cut %d: T_bind not resolved.', cutIndex);
                T_near = obj.Poses_(cut.near);              % 4×4 sym
                T_far  = sym(obj.TipTarget \ cut.T_bind);   % constant target
                % Closed-form SE(3) inverse: T_err = inv(T_near) * T_far.
                % A literal `\` inverse expands via determinants (Cramer) into
                % ~1e+170-coefficient rational trees; squaring those in the
                % generated double-precision kernels overflows to Inf → NaN.
                T_err  = core.RigidBodyMath.invT(T_near) * T_far;   % I at closure
            else
                % --- frame-pair cut (L2, existing behavior) ---
                assert(isKey(obj.Poses_, cut.far), ...
                    'solver:ClosureSolver:frameNotFound', ...
                    'Far frame "%s" not in propagated poses.', cut.far);

                T_near = obj.Poses_(cut.near);  % 4×4 sym — socket frame FK pose
                T_far  = obj.Poses_(cut.far);   % 4×4 sym — plug frame FK pose
                M      = cut.T_mate;            % 4×4 double — mate transform (near→far)

                % At closure: T_far = T_near * M
                % Residual in far frame: inv(T_near * M) * T_far → I at closure
                % (closed-form SE(3) inverse — division-free expansion)
                T_expected = T_near * M;
                T_err = core.RigidBodyMath.invT(T_expected) * T_far;  % 4×4 sym
            end

            % decompose to 6-DOF pose error
            resFull = localSymbolicPoseErrorFromRelative(T_err);  % 6×1 sym

            % filter by components mask
            if isfield(cut, 'components') && ~isempty(cut.components)
                comps = cut.components;
                allComps = {'tx','ty','tz','rx','ry','rz'};
                mask = ismember(allComps, comps);
                res = resFull(mask);
            else
                res = resFull;
            end
        end

        %% buildFullResidual  Concatenate residuals from all closure cuts.
        %   res = obj.buildFullResidual()
        function res = buildFullResidual(obj)
            nCuts = numel(obj.ClosureCuts);
            resPieces = cell(nCuts, 1);
            for i = 1:nCuts
                resPieces{i} = obj.buildResidualForCut(i);
            end
            res = vertcat(resPieces{:});
        end

        %% poseSym  Symbolic frame pose with known values already substituted.
        %   T = obj.poseSym(FRAMENAME)
        %     FRAMENAME : char — fully-qualified frame name present in the
        %                 propagated pose graph (e.g. 'manipulator_L.ground').
        %     T         : 4×4 sym (double for constant poses) — the same FK
        %                 pose the residual is built from: known joint values
        %                 are substituted, unknown joint variables remain
        %                 symbolic.
        %   Exposed for downstream consumers that compile or analyse the
        %   symbolic model (e.g. solver.exportKinematics model packages).
        function T = poseSym(obj, frameName)
            arguments
                obj
                frameName (1,:) char
            end
            assert(isKey(obj.Poses_, frameName), ...
                'solver:ClosureSolver:frameNotFound', ...
                'Frame "%s" not found in propagated poses.', frameName);
            T = obj.Poses_(frameName);
        end

        %% toNumericFunction  Convert symbolic residual to fmincon-ready handle.
        %   fh = obj.toNumericFunction()
        %     fh : @(x) function handle — x is a column vector of unknown
        %          joint values in the order of obj.UnknownSyms.
        function fh = toNumericFunction(obj)
            if ~isempty(obj.ResidualFunc_)
                fh = obj.ResidualFunc_;
                return;
            end
            fh = matlabFunction(obj.ResidualSym, 'Vars', {obj.UnknownSyms.'});
            obj.ResidualFunc_ = fh;
        end

        %% verifyZeroPose  Assert residual ≈ 0 at zero joint configuration.
        function verifyZeroPose(obj)
            zeroVals = zeros(numel(obj.UnknownSyms), 1);
            r = obj.evalResidual(zeroVals);
            rNorm = norm(r);
            assert(rNorm < 1e-12, ...
                'solver:ClosureSolver:zeroPoseFailed', ...
                ['Zero-pose residual norm = %.3e (expected < 1e-12).\n' ...
                 'Residual components: %s'], ...
                rNorm, mat2str(r, 4));
            fprintf('  Zero-pose verification: |r| = %.2e  OK\n', rNorm);
        end

        %% setTipTarget  Set the tip task target ΔT and rebuild the residual.
        %   obj.setTipTarget(DELTAT)
        %     DELTAT : 4×4 double — desired tip pose relative to the calibrated
        %              zero pose (default eye(4) = pure closure).
        %   Affects world-anchored cuts only: their far constant becomes
        %   ΔT⁻¹ · T_bind.  Rebuilds ResidualSym and invalidates the cached
        %   numeric function handle.
        function setTipTarget(obj, deltaT)
            arguments
                obj
                deltaT (4,4) double
            end

            if ~any(strcmp({obj.ClosureCuts.anchor}, 'world'))
                warning('solver:ClosureSolver:noWorldCuts', ...
                    'setTipTarget has no effect: no world-anchored cuts present.');
            end

            obj.TipTarget = deltaT;
            obj.ResidualSym = obj.buildFullResidual();
            obj.NumComponents = numel(obj.ResidualSym);
            obj.ResidualFunc_ = [];   % invalidate cached numeric handle
        end

        %% setToolTarget  Set the task target expressed in the tool frame (L3).
        %   obj.setToolTarget(DELTAT)
        %     DELTAT : 4×4 double — desired tool-frame pose relative to the
        %              tool zero pose, expressed in world axes (left-multiplied
        %              world-frame increment; world ≡ tool-frame zero pose).
        %   Internally maps to the tip target via the constant conjugation
        %       ΔT_tip = U · DELTAT · U⁻¹,   U = obj.ToolTransform
        %   and delegates to setTipTarget (world-anchored cuts only).
        %   When no tool_frame is declared, U = I₄ and this is identical to
        %   setTipTarget.
        function setToolTarget(obj, deltaT)
            arguments
                obj
                deltaT (4,4) double
            end
            U = obj.ToolTransform;
            obj.setTipTarget(U * deltaT * core.RigidBodyMath.invT(U));
        end

        %% evalResidual  Evaluate numeric residual at given joint values.
        %   r = obj.evalResidual(JOINTVALS)
        %     JOINTVALS : numeric vector (order of obj.UnknownSyms) or
        %                 containers.Map (canonical name → numeric value)
        function r = evalResidual(obj, jointVals)
            vals = obj.resolveJointVals_(jointVals);
            fh = obj.toNumericFunction();
            r = fh(vals);
        end

        %% evalToolPose  Tool-frame pose in the world at given joint values.
        %   D = obj.evalToolPose(JOINTVALS)
        %     JOINTVALS : numeric vector (order of obj.UnknownSyms) or
        %                 containers.Map (canonical name → numeric value).
        %     D         : 4×4 double — tool-frame pose in the world,
        %                 D(q) = U⁻¹ · T_rootWorld(q) · U.
        %   The FK root pose in the world is recovered from the first
        %   world-anchored cut (world-rooted FK):
        %       T_rootWorld(q) = T_bind_i · inv(T_near_i(q))
        %   All cuts agree when the closure residual is ≈ 0.  Requires at
        %   least one world-anchored cut (L3).
        function D = evalToolPose(obj, jointVals)
            vals = obj.resolveJointVals_(jointVals);

            wIdx = find(strcmp({obj.ClosureCuts.anchor}, 'world'), 1);
            assert(~isempty(wIdx), ...
                'solver:ClosureSolver:evalToolPoseNoWorldCuts', ...
                'evalToolPose requires at least one world-anchored cut (L3).');

            T_near = obj.Poses_(obj.ClosureCuts(wIdx).near);   % 4×4 sym (knowns substituted)
            if isa(T_near, 'sym')
                T_near_num = double(subs(T_near, obj.UnknownSyms, vals.'));
            else
                T_near_num = double(T_near);
            end
            T_root_world = obj.ClosureCuts(wIdx).T_bind * core.RigidBodyMath.invT(T_near_num);

            U = obj.ToolTransform;   % U = T_root←tool
            D = core.RigidBodyMath.invT(U) * T_root_world * U;
        end

        %% solveIK  Solve IK via fmincon minimizing squared residual norm.
        %   [X_OPT, FVAL, EXITFLAG] = obj.solveIK()
        %   [X_OPT, FVAL, EXITFLAG] = obj.solveIK(X0, OPTIONS)
        %     OPTIONS : fmincon options (optimoptions object or struct);
        %               default: SQP with tight tolerances, iter-detailed.
        function [x_opt, fval, exitflag] = solveIK(obj, x0, options)
            arguments
                obj
                x0      (:,1) double = zeros(numel(obj.UnknownSyms), 1)
                options = struct()
            end

            if isstruct(options) && isempty(fieldnames(options))
                options = optimoptions('fmincon', ...
                    'Algorithm', 'sqp', ...
                    'Display', 'iter-detailed', ...
                    'OptimalityTolerance', 1e-12, ...
                    'StepTolerance', 1e-12, ...
                    'MaxFunctionEvaluations', 5000, ...
                    'MaxIterations', 1000);
            end

            fh = obj.toNumericFunction();

            % objective: sum of squared residuals
            objective = @(x) sum(fh(x(:)).^2);

            [x_opt, fval, exitflag] = fmincon(objective, x0, ...
                [], [], [], [], [], [], [], options);
        end

        %% solveIKWithTarget  Solve IK with combined closure + end-frame pose target.
        %   [X_OPT, FVAL, EXITFLAG, REPORT] = obj.solveIKWithTarget(TARGETPOSE, ENDFRAME)
        %   [X_OPT, ...] = obj.solveIKWithTarget(TARGETPOSE, ENDFRAME, X0, OPTIONS)
        %
        %   Builds a combined residual = [closure_residual; pose_error] and
        %   minimises ‖r‖² via fmincon SQP.  The pose error is the 6-DOF
        %   difference between TARGETPOSE and the FK pose of ENDFRAME,
        %   decomposed as [tx,ty,tz,rx,ry,rz] (Z-Y-X Euler).
        %
        %   Inputs:
        %     TARGETPOSE : 4×4 double — target pose for ENDFRAME in world frame
        %     ENDFRAME   : char — fully-qualified frame name (e.g.
        %                  'frame_link_D2.frame_hyper_cube').  Must exist in
        %                  the propagated Poses map.
        %     X0         : initial guess (default: zeros)
        %     OPTIONS    : fmincon options struct (default: SQP, tight tolerances)
        %
        %   Outputs:
        %     X_OPT      : optimal joint values
        %     FVAL       : final objective value ‖r‖²
        %     EXITFLAG   : fmincon exit flag
        %     REPORT     : struct with fields:
        %       .closureResidual  — closure-only residual at solution
        %       .poseResidual     — pose-only residual at solution
        %       .combinedResidual — full combined residual at solution
        %       .closureNorm      — ‖closure residual‖
        %       .poseNorm         — ‖pose residual‖
        function [x_opt, fval, exitflag, report] = solveIKWithTarget(obj, targetPose, endFrame, x0, options)
            arguments
                obj
                targetPose (4,4) double
                endFrame   (1,:) char
                x0         (:,1) double = zeros(numel(obj.UnknownSyms), 1)
                options    = struct()
            end

            % ---- validate endFrame ----
            assert(isKey(obj.Poses_, endFrame), ...
                'solver:ClosureSolver:endFrameNotFound', ...
                'End frame "%s" not found in propagated poses.', endFrame);

            % ---- build symbolic end-frame pose error ----
            T_end_sym = obj.Poses_(endFrame);       % 4×4 sym
            T_rel = targetPose \ T_end_sym;          % 4×4 sym (mixed: double \ sym)
            poseErrSym = localSymbolicPoseErrorFromRelative(T_rel);  % 6×1 sym

            % ---- build combined residual: closure + pose ----
            combinedSym = [obj.ResidualSym; poseErrSym];

            % ---- convert to numeric function handle ----
            fhCombined = matlabFunction(combinedSym, 'Vars', {obj.UnknownSyms.'});

            % ---- set up fmincon options ----
            if isstruct(options) && isempty(fieldnames(options))
                options = optimoptions('fmincon', ...
                    'Algorithm', 'sqp', ...
                    'Display', 'iter-detailed', ...
                    'OptimalityTolerance', 1e-12, ...
                    'StepTolerance', 1e-12, ...
                    'MaxFunctionEvaluations', 5000, ...
                    'MaxIterations', 1000);
            end

            % ---- solve ----
            objective = @(x) sum(fhCombined(x(:)).^2);
            [x_opt, fval, exitflag] = fmincon(objective, x0, ...
                [], [], [], [], [], [], [], options);

            % ---- build diagnostic report ----
            if nargout >= 4
                rFull = fhCombined(x_opt(:));
                nClosure = numel(obj.ResidualSym);
                rClosure = rFull(1:nClosure);
                rPose    = rFull(nClosure+1:end);

                report = struct();
                report.closureResidual = rClosure;
                report.poseResidual    = rPose;
                report.combinedResidual = rFull;
                report.closureNorm     = norm(rClosure);
                report.poseNorm        = norm(rPose);
            end
        end

        %% printResidualReport  Print a human-readable residual breakdown.
        function printResidualReport(obj, jointVals)
            r = obj.evalResidual(jointVals);
            rNorm = norm(r);

            if isa(jointVals, 'containers.Map')
                fprintf('\n=== Closure Residual Report ===\n');
                fprintf('Joint values:\n');
                for i = 1:numel(obj.UnknownNames)
                    if isKey(jointVals, obj.UnknownNames{i})
                        fprintf('  %-25s = % 12.6f\n', ...
                            obj.UnknownNames{i}, jointVals(obj.UnknownNames{i}));
                    end
                end
            else
                fprintf('\n=== Closure Residual Report ===\n');
                fprintf('Joint values (order: %s):\n', strjoin(obj.UnknownNames, ', '));
                fprintf('  [%s]\n', strjoin(arrayfun(@(v) sprintf('% 12.6f', v), ...
                    jointVals(:), 'UniformOutput', false), ', '));
            end

            fprintf('\nResidual components:\n');
            compIdx = 1;
            for i = 1:obj.NumCuts
                cut = obj.ClosureCuts(i);
                comps = cut.components;
                if strcmp(core.CommonUtils.field(cut, 'anchor', ''), 'world')
                    farLabel = '(world binding)';
                else
                    farLabel = cut.far;
                end
                fprintf('  Cut %d (%s ←→ %s):\n', i, cut.near, farLabel);
                for j = 1:numel(comps)
                    fprintf('    %-4s = % 12.6e\n', comps{j}, r(compIdx));
                    compIdx = compIdx + 1;
                end
            end
            fprintf('  |r| = %.3e\n', rNorm);
            fprintf('  Translation tolerance: %.1e mm\n', obj.Tolerances.translation_mm);
            fprintf('  Rotation tolerance:    %.1e rad\n', obj.Tolerances.rotation_rad);

            % Collect component labels for this run
            compLabels = {};
            for i = 1:obj.NumCuts
                compLabels = [compLabels, obj.ClosureCuts(i).components]; %#ok<AGROW>
            end
            tMask = contains(compLabels, 't');
            rMask = contains(compLabels, 'r');
            if any(tMask), tErr = norm(r(tMask)); else tErr = 0; end
            if any(rMask), rErr = norm(r(rMask)); else rErr = 0; end
            fprintf('  |t_err| = %.3e mm, |r_err| = %.3e rad\n', tErr, rErr);
        end

    end

    % ---- private methods ----
    methods (Access = private)

        %% resolveJointVals_  Normalize joint values to a numeric column vector.
        %   Accepts a numeric vector or a containers.Map (canonical name → value).
        function vals = resolveJointVals_(obj, jointVals)
            if isa(jointVals, 'containers.Map')
                vals = zeros(numel(obj.UnknownSyms), 1);
                for i = 1:numel(obj.UnknownNames)
                    if isKey(jointVals, obj.UnknownNames{i})
                        vals(i) = jointVals(obj.UnknownNames{i});
                    end
                end
            else
                vals = jointVals;
            end
            assert(numel(vals) == numel(obj.UnknownSyms), ...
                'solver:ClosureSolver:valCount', ...
                'Expected %d joint values, got %d.', ...
                numel(obj.UnknownSyms), numel(vals));
            vals = vals(:);
        end

        %% resolveToolTransform  U = T_root←tool when cfg declares tool_frame.
        %   U = T_root←endFrame · T_endFrame←tool.  Requires a joint-free
        %   (rigid) root→endFrame chain so that U is configuration-independent;
        %   otherwise the world≡tool-frame-zero mapping breaks and an error
        %   is raised.
        function U = resolveToolTransform(obj)
            U = eye(4);
            tf = obj.ExecConfig_.ToolFrame;
            if ~tf.Present
                return;
            end

            endFrame = obj.ExecConfig_.EndFrame;
            assert(isKey(obj.Poses_, endFrame), ...
                'solver:ClosureSolver:endFrameNotFound', ...
                'End frame "%s" not found in propagated poses.', endFrame);
            T_re = obj.Poses_(endFrame);

            if isa(T_re, 'sym')
                vars = symvar(T_re);
                assert(isempty(vars), ...
                    'solver:ClosureSolver:flexibleToolFrame', ...
                    ['tool_frame requires a joint-free (rigid) chain from ' ...
                     'the FK root to endFrame "%s"; found variable(s): %s. ' ...
                     'The world≡tool-frame-zero mapping only holds for a ' ...
                     'constant tool attachment.'], ...
                    endFrame, strjoin(cellfun(@char, num2cell(vars), ...
                    'UniformOutput', false), ', '));
                T_re = double(T_re);
            end

            U = T_re * tf.T;
        end

        %% enrichClosureCuts  Resolve mate transforms / world-binding calibrations.
        function cuts = enrichClosureCuts(obj, rawCuts)
            cuts = rawCuts;
            allMates = obj.EdgeGraph_.findMates();
            closedMask = strcmp({allMates.kind}, 'closed_mate');
            closedMates = allMates(closedMask);

            % world-binding declarations from ExecutionConfig (ground → T | [])
            wbGrounds = {obj.ExecConfig_.WorldBindings.ground};
            wbPoses   = {obj.ExecConfig_.WorldBindings.T};

            for i = 1:numel(cuts)
                if strcmp(core.CommonUtils.field(cuts(i), 'anchor', ''), 'world')
                    % --- world-anchored cut (L3): T_mate = I₄, constant far side ---
                    idx = find(strcmp(wbGrounds, cuts(i).near), 1);
                    assert(~isempty(idx), ...
                        'solver:ClosureSolver:anchorNotBound', ...
                        'World-anchored cut %d: near "%s" has no world_binding.', ...
                        i, cuts(i).near);

                    % calibration: zero-pose FK pose of the near frame
                    T_calib = obj.calibrateBinding(cuts(i).near);
                    T_declared = wbPoses{idx};
                    if ~isempty(T_declared)
                        % explicit calibration in YAML — verify consistency
                        dev = norm(T_declared - T_calib, 'fro');
                        assert(dev < 1e-6, ...
                            'solver:ClosureSolver:bindingMismatch', ...
                            ['World-anchored cut %d: declared world_binding T for ' ...
                             '"%s" deviates from the zero-pose FK calibration ' ...
                             'by %.3e (tolerance 1e-6). Regenerate the calibration ' ...
                             'or omit T for auto-calibration.'], ...
                            i, cuts(i).near, dev);
                        T_bind = T_declared;
                    else
                        T_bind = T_calib;
                    end

                    cuts(i).T_mate = eye(4);
                    cuts(i).T_bind = T_bind;
                else
                    % --- frame-pair cut (L2): match closed_mate edge ---
                    found = false;
                    for j = 1:numel(closedMates)
                        if strcmp(closedMates(j).from, cuts(i).near) && ...
                           strcmp(closedMates(j).to, cuts(i).far)
                            cuts(i).T_mate = closedMates(j).T;
                            found = true;
                            break;
                        end
                    end
                    assert(found, ...
                        'solver:ClosureSolver:mateNotFound', ...
                        ['No closed_mate edge matches cut %d (near="%s", far="%s"). ' ...
                         'Ensure EdgeGraph has not been modified after ExecutionConfig construction.'], ...
                        i, cuts(i).near, cuts(i).far);
                end
            end
        end

        %% calibrateBinding  Zero-pose FK pose of a frame (all joints = 0).
        function T0 = calibrateBinding(obj, frameName)
            assert(isKey(obj.Poses_, frameName), ...
                'solver:ClosureSolver:frameNotFound', ...
                'World-anchored cut frame "%s" not in propagated poses.', frameName);
            T_sym = obj.Poses_(frameName);
            if obj.JointVarMap_.Count == 0 || ~isa(T_sym, 'sym')
                T0 = double(T_sym);
                return;
            end
            valsCell = values(obj.JointVarMap_);
            allSyms = [valsCell{:}];
            T0 = double(subs(T_sym, allSyms, zeros(1, numel(allSyms))));
        end

        %% substituteKnownValues  Substitute known joint values into all poses.
        function substituteKnownValues(obj, knownValues)
            kjv = obj.ExecConfig_.getKnownJointVars();
            nKnown = numel(kjv);
            if nKnown == 0
                return;
            end

            % validate provided keys against known joint names
            knownNames = cell(1, nKnown);
            for i = 1:nKnown
                knownNames{i} = kjv(i).name;
            end
            if knownValues.Count > 0
                badKeys = setdiff(keys(knownValues), knownNames);
                assert(isempty(badKeys), ...
                    'solver:ClosureSolver:badKnownValue', ...
                    'KNOWNVALUES contains non-known-joint key(s): %s', ...
                    strjoin(badKeys, ', '));
            end

            % resolve values (missing entries default to 0)
            knownSyms = sym(zeros(1, nKnown));
            knownVals = zeros(1, nKnown);
            for i = 1:nKnown
                knownSyms(i) = kjv(i).symHandle;
                if knownValues.Count > 0 && isKey(knownValues, kjv(i).name)
                    knownVals(i) = knownValues(kjv(i).name);
                end
            end

            % substitute into every symbolic pose
            poseKeys = keys(obj.Poses_);
            for k = 1:numel(poseKeys)
                T = obj.Poses_(poseKeys{k});
                if isa(T, 'sym')
                    obj.Poses_(poseKeys{k}) = subs(T, knownSyms, knownVals);
                end
            end
        end

    end

end

%% ---- local functions (not methods) ----

function err = localSymbolicPoseErrorFromRelative(T_rel)
%LOCALSYMBOLICPOSEERRORFROMRELATIVE  6-DOF pose error from relative transform.
%   T_rel is a 4×4 homogeneous transform (sym or double) that should be
%   identity when the poses match.  Decomposes to [tx,ty,tz,rx,ry,rz] where
%   translation is in length units (mm) and rotation is Z-Y-X Euler angles (rad).
%
%   Uses atan2 from Symbolic Math Toolbox, which accepts sym inputs.

    % position error
    p_err = T_rel(1:3, 4);

    % rotation error: extract Z-Y-X Euler angles from R_rel
    R_rel = T_rel(1:3, 1:3);

    % Z-Y-X decomposition: R = Rz(rz) * Ry(ry) * Rx(rx)
    %   R(3,1) = -sin(ry)  →  ry = asin(-R(3,1))          (range [-π/2, π/2])
    %   R(3,2) =  cos(ry)*sin(rx)  →  rx = atan2(R(3,2), R(3,3))
    %   R(2,1) =  sin(rz)*cos(ry)  →  rz = atan2(R(2,1), R(1,1))
    %
    % NOTE (A.5.1): ry deliberately uses asin instead of the algebraically
    % equivalent atan2(-R31, sqrt(R11^2+R21^2)).  The atan2 form forces a
    % symbolic sqrt of a sum of squares whose expansion introduces conj()
    % terms and enormous rational coefficients; for mechanisms with heavily
    % expanded symbolic poses (e.g. mrex-3t1r_001 with full 6-component cuts)
    % the generated numeric code then cancels/overflows to NaN.  asin is
    % division- and sqrt-free and has the identical branch: both return the
    % ZYX ry ∈ [-π/2, π/2].
    ry = asin(-R_rel(3,1));
    rx = atan2(R_rel(3,2), R_rel(3,3));
    rz = atan2(R_rel(2,1), R_rel(1,1));

    err = [p_err; rx; ry; rz];
end
