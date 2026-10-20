classdef KinematicsRuntime < handle
%KINEMATICSRUNTIME  Generic numeric IK/FK runtime for an exported model package.
%   RT = solver.KinematicsRuntime(PKGDIR)
%
%   Loads the self-contained package written by solver.exportKinematics
%   (kinematic_metadata.json + closure_residual*.m / fk_tool.m / fk_all*.m)
%   and exposes the runtime surface consumed by PathPlanner / mrex_generative:
%
%       D  = rt.fkPackage(q)                 % package task pose (D(0) = I)
%       T  = rt.fk(q, Urel)                  % fixed-world tool pose
%       [q, ok, info] = rt.ikPackage(D, q0)  % target expressed as package D
%       [q, ok, info] = rt.ik(T, Urel, q0)   % target expressed as world->tool
%       [d, qint, ok, info, q] = rt.ikDrivers(T, Urel, q0)
%       [T, q, ok, info] = rt.fkDrivers(d, Urel, qint)%       [qTraj, dTraj, ok, info] = rt.ikTrajectory(Ttargets, Urel, q0, opts)
%       rt.setLimits(lower, upper);  rt.clearLimits();%
%   Inputs / outputs use the package's native units: translations in mm,
%   rotations in rad.  The tool frame Urel = T_tip<-tool (4x4, mm / unit
%   rotation) is a runtime argument; it does NOT require re-exporting:
%
%       tip pose in the fixed world : W(q) = U0 * D(q) * U0^-1
%       tool pose in the fixed world: T(q) = W(q) * C * Urel
%       U0 = metadata.task_frame.U (T_root<-tool at the zero pose)
%       C  = metadata.task_frame.C (T_root<-endFrame; I for tip-rooted cases)
%
%   IK injects the target through the world-anchored cuts.  The exported
%   closure_residual.m is compiled at the zero target; for a target D the
%   residual is rebuilt as vec(E0 * K_c) with
%       K_c = T_bind_c^-1 * W^-1 * T_bind_c,  W = U0 * D * U0^-1
%   and solved with Levenberg-Marquardt using the exported analytic Jacobian.
%
%   fkDrivers(d) solves the passive joints from the target-independent
%   ground-ground loop constraint, then evaluates FK.  This is what lets a
%   consumer compute the current pose from the six actuator displacements
%   alone (when no passive-joint encoders are available).
%
%   Assumptions (checked at construction): world-anchored cuts with
%   T_mate = I4; one package per MATLAB path/process unless a namespaced
%   +<mechanism> package (written by solver.exportDeployment) is present.
%
%   See also: solver.exportKinematics, solver.exportDeployment
    properties (SetAccess = private)
        PackageDir (1,:) char
        Mechanism (1,:) char
        Names (1,:) cell
        NQ (1,1) double
        Types (1,:) cell
        AngleMask (:,1) logical
        U0 (4,4) double = eye(4)
        U0i (4,4) double = eye(4)
        C (4,4) double = eye(4)
        Ci (4,4) double = eye(4)
        Urel0 (4,4) double = eye(4)
        Bind cell = {}
        NearIdx (1,:) double = []
        TolT (1,1) double = 1e-3
        TolR (1,1) double = 1e-3
        ActuatedIndices (1,:) double = []
        PassiveIndices (1,:) double = []
        Lower (:,1) double = []
        Upper (:,1) double = []
        LastQ (:,1) double = []
        HasClosure (1,1) logical = false
        ToolFrameExact (1,1) logical = true
        KernelPackage (1,:) char = ''
        TaskSpace struct = struct()
    end

    properties
        MaxIter (1,1) double = 60
        Verbose (1,1) logical = false
    end

    properties (Access = private)
        ResidualFcn
        JacobianFcn
        FkToolFcn
        FkAllFcn
    end

    methods
        function obj = KinematicsRuntime(pkgDir)
            arguments
                pkgDir (1,:) char
            end
            pkgDir = char(pkgDir);
            assert(exist(pkgDir, 'dir') == 7, ...
                'KinematicsRuntime: package directory not found: %s', pkgDir);
            metaFile = fullfile(pkgDir, 'kinematic_metadata.json');
            assert(exist(metaFile, 'file') == 2, ...
                'KinematicsRuntime: %s not found.', metaFile);
            md = jsondecode(fileread(metaFile));

            obj.PackageDir = pkgDir;
            obj.Mechanism = char(md.mechanism);
            obj.Names = cellstr(md.inputs.names);
            obj.Names = obj.Names(:).';
            obj.NQ = numel(obj.Names);
            assert(isfield(md.inputs, 'count') && md.inputs.count == obj.NQ, ...
                'KinematicsRuntime: inputs.count does not match inputs.names.');

            % ---- input kinds / units ----
            if isfield(md.inputs, 'types') && ~isempty(md.inputs.types)
                obj.Types = reshape(cellstr(md.inputs.types), 1, []);
                assert(numel(obj.Types) == obj.NQ, ...
                    'KinematicsRuntime: inputs.types length mismatch.');
                obj.AngleMask = strcmpi(obj.Types(:), 'revolute');
            else
                isLen = endsWith(obj.Names, {'.dx', '.dy', '.dz'});
                obj.AngleMask = ~isLen(:);
                obj.Types = repmat({'revolute'}, 1, obj.NQ);
                obj.Types(isLen) = {'prismatic'};
            end

            % ---- actuated / passive split ----
            [obj.ActuatedIndices, obj.PassiveIndices] = ...
                localInputRoles(md, obj.Names, obj.Types);

            % ---- optional hard limits ----
            if isfield(md.inputs, 'lower') && ~isempty(md.inputs.lower)
                obj.Lower = double(md.inputs.lower(:));
                obj.Upper = double(md.inputs.upper(:));
                assert(numel(obj.Lower) == obj.NQ && ...
                    numel(obj.Upper) == obj.NQ, ...
                    'KinematicsRuntime: inputs.lower/upper length mismatch.');
            end

            % ---- task frame U0 / C / default Urel ----
            if isfield(md, 'task_frame')
                if isfield(md.task_frame, 'U') && ~isempty(md.task_frame.U)
                    obj.U0 = double(md.task_frame.U);
                end
                if isfield(md.task_frame, 'C') && ~isempty(md.task_frame.C)
                    obj.C = double(md.task_frame.C);
                    obj.ToolFrameExact = ~(isfield(md.task_frame, ...
                        'C_constant') && isequal(md.task_frame.C_constant, false));
                end
                if isfield(md.task_frame, 'U_rel') && ~isempty(md.task_frame.U_rel)
                    obj.Urel0 = double(md.task_frame.U_rel);
                else
                    obj.Urel0 = localRigidInv(obj.C) * obj.U0;
                end
            end
            obj.U0i = localRigidInv(obj.U0);
            obj.Ci = localRigidInv(obj.C);

            % ---- closure cuts (target IK supports world-anchored cuts only) ----
            if isfield(md, 'closure') && isfield(md.closure, 'cuts')
                cuts = md.closure.cuts;
                if ~isstruct(cuts)
                    cuts = [cuts{:}];
                end
                nc = numel(cuts);
                isWorld = strcmp({cuts.anchor}, 'world') & ...
                    cellfun(@isempty, {cuts.far});
                if all(isWorld)
                    obj.Bind = cell(1, nc);
                    obj.NearIdx = zeros(1, nc);
                    nodes = arrayfun(@(f) char(f.node), md.frames, ...
                        'UniformOutput', false);
                    for c = 1:nc
                        assert(norm(double(cuts(c).T_mate) - eye(4), 'fro') < 1e-12, ...
                            'KinematicsRuntime: cut %d has non-identity T_mate.', c);
                        obj.Bind{c} = double(cuts(c).T_bind);
                        k = find(strcmp(nodes, cuts(c).near), 1);
                        assert(~isempty(k), ...
                            'KinematicsRuntime: frame %s not in metadata.', ...
                            cuts(c).near);
                        obj.NearIdx(c) = double(md.frames(k).index);
                    end
                    obj.HasClosure = nc > 0;
                else
                    % L2 frame-pair cuts / mixed layouts: FK remains valid,
                    % target IK and driver-only passive solve are not enabled.
                    warning('KinematicsRuntime:unsupportedClosure', ...
                        ['Package %s uses non-world closure cuts; ' ...
                         'fk/fkPackage are enabled, ik/fkDrivers are not.'], ...
                        obj.Mechanism);
                end
            end

            if isfield(md, 'tolerances')
                obj.TolT = double(md.tolerances.translation_mm);
                obj.TolR = double(md.tolerances.rotation_rad);
            end
            if isfield(md, 'task_space')
                obj.TaskSpace = md.task_space;
            end

            % ---- resolve numeric kernels (namespaced package preferred) ----
            addpath(pkgDir);
            obj.KernelPackage = localKernelPackage(pkgDir, obj.Mechanism, md);
            obj.ResidualFcn = localKernelHandle(obj.KernelPackage, ...
                'closure_residual', obj.HasClosure);
            obj.JacobianFcn = localKernelHandle(obj.KernelPackage, ...
                'closure_residual_jacobian', obj.HasClosure);
            obj.FkToolFcn = localKernelHandle(obj.KernelPackage, ...
                'fk_tool', true);
            obj.FkAllFcn = localKernelHandle(obj.KernelPackage, ...
                'fk_all_matrix', true);

            obj.LastQ = zeros(obj.NQ, 1);
            obj.selfCheck(md);
        end

        function D = fkPackage(obj, q)
            %FKPACKAGE  Package task pose D(q); D(0) = I for world_tool packages.
            q = obj.checkQ(q);
            D = double(obj.FkToolFcn(q));
        end

        function T = fk(obj, q, Urel)
            %FK  Tool pose in the fixed world for runtime tool frame Urel.
            if nargin < 3 || isempty(Urel)
                Urel = obj.Urel0;
            end
            Urel = obj.checkT(Urel, 'Urel');
            D = obj.fkPackage(q);
            W = obj.U0 * D * obj.U0i;
            T = W * obj.C * Urel;
        end

        function [q, ok, info] = ikPackage(obj, D, q0)
            %IKPACKAGE  Solve q for a package task pose D (target in tool-zero frame).
            assert(obj.HasClosure, ...
                'KinematicsRuntime: ikPackage requires closure cuts.');
            D = obj.checkT(D, 'D');
            if nargin < 3
                q0 = [];
            end
            starts = {obj.LastQ};
            if ~isempty(q0)
                starts{end+1} = double(q0(:));
            end
            starts{end+1} = zeros(obj.NQ, 1);
            [q, ok, info] = obj.solveStarts(starts, D);
        end

        function [q, ok, info] = ik(obj, T, Urel, q0)
            %IK  Solve q for a fixed-world tool pose T.
            if nargin < 3 || isempty(Urel)
                Urel = obj.Urel0;
            end
            if nargin < 4
                q0 = [];
            end
            T = obj.checkT(T, 'T');
            Urel = obj.checkT(Urel, 'Urel');
            W = T * localRigidInv(Urel) * obj.Ci;
            D = obj.U0i * W * obj.U0;
            [q, ok, info] = obj.ikPackage(D, q0);
        end

        function [d, qint, ok, info, q] = ikDrivers(obj, T, Urel, q0)
            %IKDRIVERS  IK and split q into actuator / passive-joint groups.
            if nargin < 3 || isempty(Urel)
                Urel = obj.Urel0;
            end
            if nargin < 4
                q0 = [];
            end
            [q, ok, info] = obj.ik(T, Urel, q0);
            d = q(obj.ActuatedIndices);
            qint = q(obj.PassiveIndices);
        end

        function [T, q, ok, info] = fkDrivers(obj, d, Urel, qint)
            %FKDRIVERS  FK from actuator displacements, solving passive joints.
            if nargin < 3 || isempty(Urel)
                Urel = obj.Urel0;
            end
            if nargin < 4
                qint = [];
            end
            d = double(d(:));
            assert(numel(d) == numel(obj.ActuatedIndices), ...
                'KinematicsRuntime: expected %d actuator values, got %d.', ...
                numel(obj.ActuatedIndices), numel(d));
            Urel = obj.checkT(Urel, 'Urel');

            q = obj.LastQ;
            q(obj.ActuatedIndices) = d;
            if ~isempty(qint)
                qint = double(qint(:));
                assert(numel(qint) == numel(obj.PassiveIndices), ...
                    'KinematicsRuntime: expected %d passive values, got %d.', ...
                    numel(obj.PassiveIndices), numel(qint));
                q(obj.PassiveIndices) = qint;
                ok = true;
                info = struct('passive_iterations', 0, ...
                    'passive_norm', NaN);
            elseif isempty(obj.PassiveIndices)
                ok = true;
                info = struct('passive_iterations', 0, ...
                    'passive_norm', NaN);
            else
                assert(obj.HasClosure && numel(obj.Bind) >= 2, ...
                    ['KinematicsRuntime: fkDrivers needs at least two ' ...
                     'world-anchored cuts to solve %d passive joint(s).'], ...
                    numel(obj.PassiveIndices));
                [q, ok, info] = obj.solvePassive(q);
            end
            T = obj.fk(q, Urel);
            if ok
                obj.LastQ = q;
            end
        end

        function util = utilization(obj, q)
            %UTILIZATION  Worst |q| / max(|lower|,|upper|); NaN without limits.
            q = obj.checkQ(q);
            if isempty(obj.Lower)
                util = NaN;
                return;
            end
            L = max(abs(obj.Lower), abs(obj.Upper));
            L(L == 0) = inf;
            util = max(abs(q) ./ L);
        end

        function setLimits(obj, lower, upper)
            %SETLIMITS  Override the package's hard input limits (mm / rad).
            %   setLimits(rt, LOWER, UPPER) with nq-vector bounds; [] clears.
            if isempty(lower) && isempty(upper)
                obj.Lower = [];
                obj.Upper = [];
                return;
            end
            lower = double(lower(:));
            upper = double(upper(:));
            assert(numel(lower) == obj.NQ && numel(upper) == obj.NQ, ...
                'KinematicsRuntime: limits must have %d entries.', obj.NQ);
            assert(all(lower <= upper), ...
                'KinematicsRuntime: every lower limit must be <= upper.');
            obj.Lower = lower;
            obj.Upper = upper;
        end

        function clearLimits(obj)
            %CLEARLIMITS  Remove runtime hard limits.
            obj.Lower = [];
            obj.Upper = [];
        end

        function [qTraj, dTraj, ok, info] = ikTrajectory(obj, Ttargets, Urel, q0, opts)
            %IKTRAJECTORY  Batch IK over a target pose sequence (P1).
            %   [qTraj, dTraj, ok, info] = rt.ikTrajectory(Ttargets, Urel, q0, opts)
            %
            %   Ttargets : 4x4xN | 1xN cell | Nx6 [x y z rx ry rz] (mm / rad)
            %   Urel     : 4x4 | 4x4xN | 1xN cell | Nx6 (tool frame per frame)
            %   q0       : initial guess (default: LastQ)
            %   opts     : StopOnFailure (false), MaxIter (rt.MaxIter)
            %
            %   qTraj : N x nq full joint trajectory
            %   dTraj : N x numel(ActuatedIndices) actuator trajectory
            %   ok    : N x 1 convergence flags
            %   info  : per-frame residual/iterations/utilization/branchJump
            if nargin < 3
                Urel = [];
            end
            if nargin < 4
                q0 = [];
            end
            if nargin < 5 || isempty(opts)
                opts = struct();
            end
            [N, getT] = localTargetSequence(Ttargets);
            [getU, ~] = localToolSequence(Urel, N, obj.Urel0);
            if isempty(q0)
                qPrev = obj.LastQ;
            else
                qPrev = obj.checkQ(q0);
            end
            stopOnFailure = isfield(opts, 'StopOnFailure') && ...
                isequal(opts.StopOnFailure, true);
            oldMax = obj.MaxIter;
            if isfield(opts, 'MaxIter') && ~isempty(opts.MaxIter)
                obj.MaxIter = double(opts.MaxIter);
            end
            cleanupObj = onCleanup(@() obj.restoreMaxIter(oldMax)); %#ok<NASGU>

            qTraj = nan(N, obj.NQ);
            dTraj = nan(N, numel(obj.ActuatedIndices));
            ok = false(N, 1);
            info = repmat(struct('maxAbs', NaN, 'translation', NaN, ...
                'rotation', NaN, 'iterations', 0, 'utilization', NaN, ...
                'limitActive', false, 'branchJump', NaN, ...
                'feasible', false), 1, N);
            for k = 1:N
                Tk = getT(k);
                Uk = getU(k);
                [q, okK, infoK] = obj.ik(Tk, Uk, qPrev);
                qTraj(k, :) = q.';
                dTraj(k, :) = q(obj.ActuatedIndices).';
                ok(k) = okK;
                info(k).maxAbs = infoK.maxAbs;
                info(k).translation = infoK.translation;
                info(k).rotation = infoK.rotation;
                info(k).iterations = infoK.iterations;
                info(k).utilization = obj.utilization(q);
                info(k).limitActive = localLimitActive(q, obj.Lower, obj.Upper);
                info(k).branchJump = norm(q - qPrev);
                info(k).feasible = okK && ...
                    (isnan(info(k).utilization) || info(k).utilization <= 1);
                if okK
                    qPrev = q;
                elseif stopOnFailure
                    break;
                end
            end
        end
    end

    methods (Access = private)
        function restoreMaxIter(obj, val)
            obj.MaxIter = val;
        end

        function q = checkQ(obj, q)
            assert(isnumeric(q) && numel(q) == obj.NQ, ...
                'KinematicsRuntime: expected %d q entries, got %d.', ...
                obj.NQ, numel(q));
            q = double(q(:));
        end

        function T = checkT(obj, T, label) %#ok<INUSL>
            assert(isnumeric(T) && isequal(size(T), [4 4]), ...
                'KinematicsRuntime: %s must be a 4x4 matrix.', label);
            T = double(T);
        end

        function [q, ok, info] = solveStarts(obj, starts, D)
            bestQ = starts{1}(:);
            bestInfo = [];
            bestErr = inf;
            bestOk = false;
            for s = 1:numel(starts)
                qs = starts{s}(:);
                if numel(qs) ~= obj.NQ
                    continue;
                end
                [qs, okS, infoS] = obj.solveFrom(qs, D);
                if infoS.maxAbs < bestErr
                    bestQ = qs;
                    bestInfo = infoS;
                    bestErr = infoS.maxAbs;
                    bestOk = okS;
                end
                if okS
                    break;
                end
            end
            q = bestQ;
            ok = bestOk;
            info = bestInfo;
            if isempty(info)
                r = obj.targetResidualNoJac(q, D);
                nc = numel(obj.Bind);
                r6 = reshape(r, 6, nc);
                info = struct('residual', r, 'maxAbs', max(abs(r)), ...
                    'translation', max(abs(r6(1:3, :)), [], 'all'), ...
                    'rotation', max(abs(r6(4:6, :)), [], 'all'), ...
                    'iterations', 0);
            end
            if ok
                obj.LastQ = q;
            end
        end

        function [q, ok, info] = solveFrom(obj, q, D)
            q = double(q(:));
            lambda = 1e-3;
            [r, J] = obj.targetResidual(q, D);
            f = r' * r;
            dq = zeros(obj.NQ, 1);
            itCount = 0;
            for it = 1:obj.MaxIter
                if f < 1e-24
                    break;
                end
                H = J' * J;
                g = J' * r;
                improved = false;
                for tries = 1:12 %#ok<NASGU>
                    A = H + lambda * diag(diag(H) + 1e-12);
                    dq = -(A \ g);
                    qn = q + dq;
                    if ~isempty(obj.Lower)
                        qn = max(min(qn, obj.Upper), obj.Lower);
                    end
                    rn = obj.targetResidualNoJac(qn, D);
                    fn = rn' * rn;
                    if fn < f
                        q = qn;
                        f = fn;
                        lambda = max(lambda / 5, 1e-12);
                        improved = true;
                        break;
                    end
                    lambda = lambda * 5;
                end
                itCount = it;
                if ~improved || norm(dq) < 1e-13
                    break;
                end
                [r, J] = obj.targetResidual(q, D);
            end
            r = obj.targetResidualNoJac(q, D);
            nc = numel(obj.Bind);
            r6 = reshape(r, 6, nc);
            info = struct();
            info.residual = r;
            info.maxAbs = max(abs(r));
            info.translation = max(abs(r6(1:3, :)), [], 'all');
            info.rotation = max(abs(r6(4:6, :)), [], 'all');
            info.iterations = itCount;
            ok = all(abs(r6(1:3, :)) < obj.TolT, 'all') && ...
                 all(abs(r6(4:6, :)) < obj.TolR, 'all');
        end

        function [r, J] = targetResidual(obj, q, D)
            r0 = double(obj.ResidualFcn(q));
            J0 = double(obj.JacobianFcn(q));
            nc = numel(obj.Bind);
            assert(numel(r0) == 6 * nc, ...
                'KinematicsRuntime: residual length %d != 6*numel(cuts)=%d.', ...
                numel(r0), 6 * nc);
            W = obj.U0 * D * obj.U0i;
            Wi = localRigidInv(W);
            r = zeros(6 * nc, 1);
            J = zeros(6 * nc, obj.NQ);
            for c = 1:nc
                rows = 6 * (c - 1) + (1:6);
                K = localRigidInv(obj.Bind{c}) * Wi * obj.Bind{c};
                [r(rows), Jr] = obj.mapResidual(r0(rows), K);
                J(rows, :) = Jr * J0(rows, :);
            end
        end

        function r = targetResidualNoJac(obj, q, D)
            r0 = double(obj.ResidualFcn(q));
            nc = numel(obj.Bind);
            W = obj.U0 * D * obj.U0i;
            Wi = localRigidInv(W);
            r = zeros(6 * nc, 1);
            for c = 1:nc
                rows = 6 * (c - 1) + (1:6);
                K = localRigidInv(obj.Bind{c}) * Wi * obj.Bind{c};
                r(rows) = obj.mapResidual(r0(rows), K);
            end
        end

        function [v, Jr] = mapResidual(~, v0, K)
            v = localT2Vec(localVec2T(v0) * K);
            if nargout < 2
                return;
            end
            if norm(K - eye(4), 'fro') < 1e-14
                Jr = eye(6);
                return;
            end
            h = 1e-7;
            Jr = zeros(6);
            for k = 1:6
                d = zeros(6, 1);
                d(k) = h;
                Jr(:, k) = (localT2Vec(localVec2T(v0 + d) * K) - ...
                            localT2Vec(localVec2T(v0 - d) * K)) / (2 * h);
            end
        end

        function [q, ok, info] = solvePassive(obj, q)
            pIdx = obj.PassiveIndices;
            lambda = 1e-3;
            r = obj.loopResidual(q);
            f = r' * r;
            itCount = 0;
            dp = zeros(numel(pIdx), 1);
            for it = 1:obj.MaxIter
                if f < 1e-24
                    break;
                end
                J = obj.loopJacobian(q, pIdx);
                H = J' * J;
                g = J' * r;
                improved = false;
                for tries = 1:12 %#ok<NASGU>
                    A = H + lambda * diag(diag(H) + 1e-12);
                    dp = -(A \ g);
                    qn = q;
                    qn(pIdx) = q(pIdx) + dp;
                    rn = obj.loopResidual(qn);
                    fn = rn' * rn;
                    if fn < f
                        q = qn;
                        f = fn;
                        lambda = max(lambda / 5, 1e-12);
                        improved = true;
                        break;
                    end
                    lambda = lambda * 5;
                end
                itCount = it;
                if ~improved || norm(dp) < 1e-13
                    break;
                end
                r = obj.loopResidual(q);
            end
            r = obj.loopResidual(q);
            info = struct('passive_iterations', itCount, ...
                'passive_norm', norm(r));
            ok = norm(r) < 1e-6;
        end

        function r = loopResidual(obj, q)
            if ~obj.HasClosure || numel(obj.Bind) < 2
                r = zeros(0, 1);
                return;
            end
            raw = double(obj.FkAllFcn(q));
            G1 = raw(4 * (obj.NearIdx(1) - 1) + (1:4), :);
            E1 = localRigidInv(obj.Bind{1});
            r = zeros(6 * (numel(obj.Bind) - 1), 1);
            for c = 2:numel(obj.Bind)
                Gc = raw(4 * (obj.NearIdx(c) - 1) + (1:4), :);
                X = localRigidInv(G1) * Gc * ...
                    localRigidInv(E1 * obj.Bind{c});
                rows = 6 * (c - 2) + (1:6);
                r(rows) = localT2Vec(X);
            end
        end

        function J = loopJacobian(obj, q, pIdx)
            h = 1e-6;
            r0 = obj.loopResidual(q);
            J = zeros(numel(r0), numel(pIdx));
            for k = 1:numel(pIdx)
                qp = q;
                qm = q;
                qp(pIdx(k)) = qp(pIdx(k)) + h;
                qm(pIdx(k)) = qm(pIdx(k)) - h;
                J(:, k) = (obj.loopResidual(qp) - obj.loopResidual(qm)) / (2 * h);
            end
        end

        function selfCheck(obj, md)
            q0 = zeros(obj.NQ, 1);
            isTool = isfield(md, 'task_frame') && ...
                isfield(md.task_frame, 'tool_conjugated') && ...
                isequal(md.task_frame.tool_conjugated, true);
            if isTool
                D = double(obj.FkToolFcn(q0));
                assert(norm(D - eye(4), 'fro') < 1e-8, ...
                    ['KinematicsRuntime: fk_tool(0) differs from I4 by %.3e ' ...
                     '(package contract violation).'], norm(D - eye(4), 'fro'));
            end
            if obj.HasClosure
                r = double(obj.ResidualFcn(q0));
                assert(norm(r) < 1e-8, ...
                    ['KinematicsRuntime: closure_residual(0) norm %.3e ' ...
                     '(package contract violation).'], norm(r));
            end
        end
    end
end

%% ---- local functions ----

function [actIdx, passIdx] = localInputRoles(md, names, types)
%LOCALINPUTROLES  Actuated/passive index split from metadata (with fallback).
    n = numel(names);
    if isfield(md.inputs, 'actuated_indices') && ...
            isfield(md.inputs, 'passive_indices')
        actIdx = double(md.inputs.actuated_indices(:).');
        passIdx = double(md.inputs.passive_indices(:).');
        % An all-passive closed-loop package has empty actuated_indices but
        % a non-empty passive_indices list; trust the metadata in that case.
        if ~isempty(actIdx) || ~isempty(passIdx)
            assert(all(actIdx >= 1 & actIdx <= n) && ...
                all(passIdx >= 1 & passIdx <= n), ...
                'KinematicsRuntime: actuated/passive indices out of range.');
            return;
        end
    end
    if isfield(md, 'mode') && strcmp(md.mode, 'open_loop')
        actIdx = 1:n;
    else
        actIdx = find(endsWith(names, {'.dx', '.dy', '.dz'}));
        if isempty(actIdx)
            actIdx = find(strcmpi(types, 'prismatic'));
        end
    end
    passIdx = setdiff(1:n, actIdx);
end

function pkg = localKernelPackage(pkgDir, mechanism, md)
%LOCALKERNELPACKAGE  Prefer an exported +<mechanism> package when present.
    pkg = '';
    cand = matlab.lang.makeValidName(mechanism);
    if isfield(md, 'runtime') && isfield(md.runtime, 'package') && ...
            ~isempty(md.runtime.package)
        cand = char(md.runtime.package);
    end
    manifest = fullfile(pkgDir, 'runtime_manifest.json');
    if exist(manifest, 'file') == 2
        try
            rm = jsondecode(fileread(manifest));
            if isfield(rm, 'kernel_package') && ~isempty(rm.kernel_package)
                cand = char(rm.kernel_package);
            end
        catch
            % malformed manifest: fall back to metadata / mechanism name
        end
    end
    if exist(fullfile(pkgDir, ['+' cand]), 'dir') == 7
        pkg = cand;
    end
end

function fh = localKernelHandle(pkg, base, required)
%LOCALKERNELHANDLE  Resolve a package function or the generic package function.
    if ~isempty(pkg)
        fname = [pkg '.' base];
    else
        fname = base;
    end
    w = which(fname);
    if isempty(w) && required
        error('KinematicsRuntime:kernelNotFound', ...
            'Required kernel "%s" not found (package dir on path?).', fname);
    end
    if isempty(w)
        fh = [];
    else
        fh = str2func(fname);
    end
end

function Ti = localRigidInv(T)
%LOCALRIGIDINV  Closed-form SE(3) inverse [R' -R't; 0 1].
    Ti = eye(4);
    Ti(1:3, 1:3) = T(1:3, 1:3)';
    Ti(1:3, 4) = -T(1:3, 1:3)' * T(1:3, 4);
end

function T = localVec2T(v)
%LOCALVEC2T  6-vector [t; rx; ry; rz] -> T = [Rz(rz)Ry(ry)Rx(rx), t].
    a = v(4); b = v(5); c = v(6);
    Rx = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
    Ry = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
    Rz = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
    T = eye(4);
    T(1:3, 1:3) = Rz * Ry * Rx;
    T(1:3, 4) = v(1:3);
end

function v = localT2Vec(T)
%LOCALT2VEC  T -> [t; rx; ry; rz] using the package's Z-Y-X convention.
    R = T(1:3, 1:3);
    v = [T(1:3, 4); ...
         atan2(R(3, 2), R(3, 3)); ...
         asin(max(-1, min(1, -R(3, 1)))); ...
         atan2(R(2, 1), R(1, 1))];
end

function [N, getT] = localTargetSequence(targets)
%LOCALTARGETSEQUENCE  Normalize 4x4xN / cell / Nx6 target pose input.
    if iscell(targets)
        N = numel(targets);
        getT = @(k) localAsT(targets{k});
    elseif isnumeric(targets) && ndims(targets) == 3 && ...
            size(targets, 1) == 4 && size(targets, 2) == 4
        N = size(targets, 3);
        getT = @(k) double(targets(:, :, k));
    elseif isnumeric(targets) && ismatrix(targets) && size(targets, 2) == 6
        N = size(targets, 1);
        getT = @(k) localPoseRowToT(targets(k, :));
    elseif isnumeric(targets) && isequal(size(targets), [4 4])
        N = 1;
        getT = @(k) double(targets);
    else
        error('KinematicsRuntime:badTargets', ...
            'Ttargets must be 4x4xN, a cell of 4x4, or Nx6 pose rows.');
    end
    assert(N >= 1, 'KinematicsRuntime:badTargets', ...
        'Ttargets must contain at least one pose.');
end

function [getU, N] = localToolSequence(Urel, N, defaultU)
%LOCALTOOLSEQUENCE  Normalize constant / per-frame tool frame input.
    if isempty(Urel)
        U = defaultU;
        getU = @(k) U;
    elseif isnumeric(Urel) && isequal(size(Urel), [4 4])
        U = double(Urel);
        getU = @(k) U;
    elseif iscell(Urel)
        assert(numel(Urel) == N, ...
            'KinematicsRuntime: Urel cell length must match Ttargets.');
        getU = @(k) localAsT(Urel{k});
    elseif isnumeric(Urel) && ndims(Urel) == 3 && ...
            size(Urel, 1) == 4 && size(Urel, 2) == 4
        assert(size(Urel, 3) == N, ...
            'KinematicsRuntime: Urel page count must match Ttargets.');
        getU = @(k) double(Urel(:, :, k));
    elseif isnumeric(Urel) && ismatrix(Urel) && size(Urel, 2) == 6
        assert(size(Urel, 1) == N, ...
            'KinematicsRuntime: Urel row count must match Ttargets.');
        getU = @(k) localPoseRowToT(Urel(k, :));
    else
        error('KinematicsRuntime:badToolFrame', ...
            'Urel must be 4x4, 4x4xN, a cell of 4x4, or Nx6 pose rows.');
    end
end

function T = localAsT(T)
%LOCALAST  Validate one 4x4 homogeneous transform.
    assert(isnumeric(T) && isequal(size(T), [4 4]), ...
        'KinematicsRuntime: expected a 4x4 homogeneous transform.');
    T = double(T);
end

function T = localPoseRowToT(v)
%LOCALPOSEROWTOT  [x y z rx ry rz] -> Rz(rz)Ry(ry)Rx(rx), p.
    assert(isnumeric(v) && numel(v) == 6, ...
        'KinematicsRuntime: pose rows must have 6 entries.');
    a = v(4); b = v(5); c = v(6);
    Rx = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
    Ry = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
    Rz = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
    T = eye(4);
    T(1:3, 1:3) = Rz * Ry * Rx;
    T(1:3, 4) = v(1:3);
end

function tf = localLimitActive(q, lower, upper)
%LOCALLIMITACTIVE  True when any finite-width bound is touched.
    tf = false;
    if isempty(lower)
        return;
    end
    mask = upper > lower;
    if ~any(mask)
        return;
    end
    tol = 1e-9;
    tf = any(q(mask) <= lower(mask) + tol) || ...
         any(q(mask) >= upper(mask) - tol);
end
