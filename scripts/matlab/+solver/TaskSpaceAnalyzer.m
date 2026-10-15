classdef TaskSpaceAnalyzer
%TASKSPACEANALYZER  编译后机构的瞬时任务空间分析（导出元数据泛化）。
%   TS = solver.TaskSpaceAnalyzer.analyze(PROBE)
%
%   对 solver.exportKinematics 编译出的机制做纯数值的「瞬时」任务空间分析，
%   产出可不依赖任何手工规则消费的泛化元数据（mobility / 旋转轴 / 输入标签）：
%
%     mobility  —— "NtMr"：任务旋量空间 S ⊆ se(3) 的一阶分解，t = 纯平动
%                  子空间维数（{ (0, v) ∈ S }），r = 角速度投影的秩（旋转自由度
%                  数；r ≤ 1 时角速度投影恰为一条直线 → 固定旋转轴）。
%                  例：mrex-3t1r_001 → "3t1r"（3 平移 ⊕ 1 旋转）。
%     rotation  —— {mode: 'none' | 'axis' | 'full'}；mode = 'axis' 时给出
%                  单位向量 axis（任务/工具系、"D 恒等"帧——即 fk_tool 在零位
%                  所处帧；与 WorkspaceEvaluator.pose2T 的 phi 轴向同约定）。
%     dims      —— .task（= rank(S)）、.translation（t）、.rotation（r）。
%     probe     —— 分析点诊断：.q（探针关节值，列向量）、.constraint_rank
%                  （物理环约束 Jacobian 的秩）、.manifold_dim（约束流形维数）。
%     types/axes—— 输入标签（见下）；probe.kinds / probe.axesRoot 缺省时为空。
%
%   方法（均在探针 q0 处一阶展开；中央差分，步长 1e-6）：
%     1. 任务旋量 Ξ_i = ∂D/∂q_i · D(q0)⁻¹（左平凡化），D = probe.poseFcn(q)；
%     2. 物理约束流形：Jc = ∂C/∂q（C = probe.constraintFcn(q)；开环时为空 →
%        流形 = 全关节空间），右奇异向量按相对容差 relTol 截断得切空间基 B；
%        S = Ξ · B。
%     3. 秩全部采用相对容差（σ > relTol·σmax），缺省 relTol = 1e-6——中央差分的
%        数值噪声通常在 1e-13 量级，结构小奇异值（如 m-rex 的 0.09）可稳定区分。
%     4. 旋转轴 a 由 Aw = S 的角速度行块的主左奇异向量给出，再经 R(D(q0))ᵀ
%        拉回「D 恒等」帧，并规范化符号（最靠前非零分量为正）。
%
%   输入标签（inputs.types / inputs.axes 的来源；见 CONFIG 生成端
%   mrex_generative 的 ConfigGenerator）：
%     types      = probe.kinds —— 'prismatic' / 'revolute'（逐输入）。
%     axes       = 逐输入的运动轴标签，在任务（D）系中判定：
%                  - prismatic 且与 ±x/±y/±z 对齐（容差 1e-3）→ 'x'/'y'/'z'；
%                  - 其余平行方向按出现次序聚类 → 'u1', 'u2', …（棱柱）、
%                    'r1', 'r2', …（旋转）；同一方向（含反向）同标签；
%                  - 无可判定方向（probe.axesRoot 缺失/零向量）→ ''。
%                  例：mrex-3t1r_001 → ["x" "y" "z" "x" "y" "z" "r1" "r1"]。
%
%   PROBE 结构（由 exportKinematics 组装）字段：
%     .names          cellstr, n×1 — 输入名（次序 = q 次序；仅信息用）
%     .kinds          cellstr | {} — 逐输入关节类型（可缺省）
%     .axesRoot       n×3 double | [] — 逐输入关节运动轴方向（root 系、
%                     q0 处；可缺省）
%     .poseFcn        @(q) → 4×4 double — 任务位姿 D(q)（fk_tool 语义）
%     .constraintFcn  @(q) → m×1 double — 物理环约束误差向量；开环时 []，
%                     此时流形 = 全关节空间（约束不裁剪）
%     .toolU          4×4 double | [] — 任务帧共轭 U（缺省 I₄；标签与轴拉回用）
%     .probeQ         n×1 double | [] — 探针位形（缺省全零，即零位）
%     .relTol         double | [] — 秩相对容差（缺省 1e-6）
%
%   注：mrex-3t1r_001 的实测标定（2026-10-15，MATLAB R2026a）：物理环约束
%   rank = 4（流形 4 维；导出残差 closure_residual 的 12 分量 Jacobian 满列秩
%   8 是"双 ground 钉零位"的过定形式，不能用于流形分析）；S 4 维、r = 1、
%   轴 = [1 0 1]/√2（任务系）、t = 3；零位与已解可行点结果一致。
%
%   See also: solver.exportKinematics, solver.ClosureSolver

    methods (Static)
        function ts = analyze(probe)
            arguments
                probe struct
            end

            names = probe.names(:).';
            n = numel(names);

            q0 = zeros(n, 1);
            if isfield(probe, 'probeQ') && ~isempty(probe.probeQ)
                q0 = double(probe.probeQ(:));
                assert(numel(q0) == n, ...
                    'TaskSpaceAnalyzer: probeQ must have %d entries.', n);
            end
            relTol = 1e-6;
            if isfield(probe, 'relTol') && ~isempty(probe.relTol)
                relTol = double(probe.relTol);
            end
            U = eye(4);
            if isfield(probe, 'toolU') && ~isempty(probe.toolU)
                U = double(probe.toolU);
            end
            Ui = core.RigidBodyMath.invT(U);

            % ---- 任务旋量（左平凡化，q0 处）----
            h = 1e-6;
            D0 = double(probe.poseFcn(q0));
            D0i = core.RigidBodyMath.invT(D0);
            Xi = zeros(6, n);
            for i = 1:n
                qp = q0; qp(i) = qp(i) + h;
                qm = q0; qm(i) = qm(i) - h;
                X = (double(probe.poseFcn(qp)) - double(probe.poseFcn(qm))) ...
                    / (2 * h) * D0i;
                Xi(:, i) = [X(3, 2); X(1, 3); X(2, 1); X(1:3, 4)];
            end

            % ---- 物理约束流形（切空间基 B）----
            hasC = isfield(probe, 'constraintFcn') && ~isempty(probe.constraintFcn);
            if hasC
                C0 = double(probe.constraintFcn(q0));
                hasC = ~isempty(C0);
            end
            if ~hasC
                B = eye(n);
                constraintRank = 0;
            else
                m = numel(C0);
                Jc = zeros(m, n);
                for i = 1:n
                    qp = q0; qp(i) = qp(i) + h;
                    qm = q0; qm(i) = qm(i) - h;
                    Jc(:, i) = (double(probe.constraintFcn(qp)) ...
                        - double(probe.constraintFcn(qm))) / (2 * h);
                end
                [B, constraintRank] = solver.TaskSpaceAnalyzer.nullBasis(Jc, relTol);
            end
            manifoldDim = size(B, 2);

            % ---- 任务旋量空间 S = Ξ · B ----
            S = Xi * B;
            d = solver.TaskSpaceAnalyzer.rankRel(S, relTol);
            if isempty(S)
                Aw = zeros(3, 0);
                Av = zeros(3, 0);
            else
                Aw = S(1:3, :);
                Av = S(4:6, :);
            end
            r = solver.TaskSpaceAnalyzer.rankRel(Aw, relTol);

            % 纯平动子空间：ker(ω) 在 Av 下的像的秩
            kerW = solver.TaskSpaceAnalyzer.nullBasis(Aw, relTol);
            if isempty(kerW)
                t = 0;
            else
                t = solver.TaskSpaceAnalyzer.rankRel(Av * kerW, relTol);
            end

            % ---- 分类 ----
            if r == 0
                rotation = struct('mode', 'none');
            elseif r == 1
                [Uw, ~, ~] = svd(Aw);
                a = Uw(:, 1);
                a0 = D0(1:3, 1:3).' * a;          % 拉回 D 恒等帧
                a0 = a0 / norm(a0);
                k = find(abs(a0) > 1e-9, 1);
                if ~isempty(k) && a0(k) < 0
                    a0 = -a0;                      % 符号规范化
                end
                rotation = struct('mode', 'axis', 'axis', a0(:).');
            else
                rotation = struct('mode', 'full');
            end

            ts = struct();
            ts.mobility = sprintf('%dt%dr', t, r);
            ts.rotation = rotation;
            ts.dims = struct('task', d, 'translation', t, 'rotation', r);
            ts.probe = struct('q', q0, ...
                'constraint_rank', constraintRank, 'manifold_dim', manifoldDim);

            % ---- 输入标签（types / axes）----
            [ts.types, ts.axes] = solver.TaskSpaceAnalyzer.labels( ...
                names, probe, Ui);
        end
    end

    methods (Static, Access = private)
        function [B, k] = nullBasis(M, relTol)
            %NULLBASIS  按相对容差截断的零空间基（列）。
            if isempty(M)
                B = eye(size(M, 2));
                k = 0;
                return;
            end
            [~, S, V] = svd(M);
            sv = zeros(size(M, 2), 1);
            diagS = diag(S);
            sv(1:numel(diagS)) = diagS;
            if isempty(sv) || sv(1) == 0
                k = 0;
            else
                k = sum(sv > relTol * sv(1));
            end
            B = V(:, k + 1:end);
        end

        function r = rankRel(M, relTol)
            %RANKREL  相对容差秩。
            if isempty(M)
                r = 0;
                return;
            end
            sv = svd(M);
            if isempty(sv) || sv(1) == 0
                r = 0;
            else
                r = sum(sv > relTol * sv(1));
            end
        end

        function [types, axesLab] = labels(names, probe, Ui)
            %LABELS  逐输入类型与任务系轴向标签（见类文档）。
            n = numel(names);
            types = {};
            axesLab = {};
            if ~isfield(probe, 'kinds') || isempty(probe.kinds) ...
                    || ~isfield(probe, 'axesRoot') || isempty(probe.axesRoot)
                return;
            end
            kinds = cellstr(probe.kinds);
            kinds = kinds(:).';
            axesRoot = double(probe.axesRoot);
            assert(numel(kinds) == n && size(axesRoot, 1) == n, ...
                'TaskSpaceAnalyzer: kinds/axesRoot must have %d entries.', n);

            types = kinds;
            axesLab = cell(1, n);

            dirsD = (Ui(1:3, 1:3) * axesRoot.').';   % n×3，任务系
            tol = 1e-3;
            % 棱柱（'u#'）与旋转（'r#'）分别聚类，互不共用标签空间
            prisDirs = zeros(0, 3); prisLabs = {};
            revDirs = zeros(0, 3);  revLabs = {};
            nPris = 0;
            nRev = 0;
            for i = 1:n
                d = dirsD(i, :);
                nd = norm(d);
                if nd < 1e-9
                    axesLab{i} = '';                 % 不可判定
                    continue;
                end
                d = d / nd;
                if strcmpi(kinds{i}, 'prismatic')
                    % 与 ±x/±y/±z 对齐 → 'x'/'y'/'z'
                    [mx, j] = max(abs(d));
                    if mx > 1 - tol
                        axesLab{i} = char('x' + (j - 1));
                        continue;
                    end
                    % 其余方向按平行聚类 → 'u#'
                    [lab, prisDirs, prisLabs, nPris] = ...
                        solver.TaskSpaceAnalyzer.clusterDir( ...
                        d, prisDirs, prisLabs, nPris, 'u');
                    axesLab{i} = lab; %#ok<AGROW>
                else
                    [lab, revDirs, revLabs, nRev] = ...
                        solver.TaskSpaceAnalyzer.clusterDir( ...
                        d, revDirs, revLabs, nRev, 'r');
                    axesLab{i} = lab; %#ok<AGROW>
                end
            end
        end

        function [lab, dirs, labs, count] = clusterDir(d, dirs, labs, count, prefix)
            %CLUSTERDIR  方向 ±d 与已有聚类（±）平行则复用标签，否则新建。
            for j = 1:size(dirs, 1)
                if min(norm(dirs(j, :) - d), norm(dirs(j, :) + d)) < 1e-3
                    lab = labs{j};
                    return;
                end
            end
            count = count + 1;
            lab = sprintf('%s%d', prefix, count);
            dirs(end + 1, :) = d; %#ok<AGROW>
            labs{end + 1} = lab; %#ok<AGROW>
        end
    end
end
