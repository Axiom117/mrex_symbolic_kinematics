# scripts/matlab — Class & API Reference

> 本文档是 `scripts/matlab/` 下关键类与方法的百科全书式参考手册。
> 关于各层之间的架构关系、数据流、设计理念，请参见 [ARCHITECTURE.md](./ARCHITECTURE.md)。

---

## 目录

| 包 | 类/函数 | 类型 | 职责 |
|----|---------|------|------|
| `+ir/` | [EdgeGraph](#edgegraph) | handle 类 | 位姿图 IR 累积器 |
| `+ir/` | [Expander](#expander) | handle 类 | DSL→IR 编排器（核心管线） |
| `+ir/` | [ExecutionConfig](#executionconfig) | value 类 | L3 执行配置与变量分区 |
| `+solver/` | [KinematicModel](#kinematicmodel) | handle 类 | 符号 FK 薄封装 + 问题构造 |
| `+solver/` | [ClosureSolver](#closuresolver) | handle 类 | 闭环残差构造 + fmincon 求解 |
| `+solver/` | [buildPoseSequence](#buildposesequence) | 函数 | FK 位姿序列构建（Q → PoseFrames） |
| `+solver/` | [exportKinematics](#exportkinematics) | 函数 | 自包含模型包导出（编译器 → 模型包） |
| `+viz/` | [mechanism](#mechanism) | 函数 | 机构装配与可视化 |
| `+viz/` | [module](#module) | 函数 | 单模块可视化校验 |
| `+viz/` | [animate](#animate) | 函数 | 轨迹动画兼容包装（buildPoseSequence + play） |
| `+viz/` | [play](#play) | 函数 | 位姿序列纯播放器（渲染层） |
| `+viz/` | [sceneSpec](#scenespec) | 函数 | 静态场景快照提取 |
| `+viz/` | [allModules](#allmodules) | 函数 | 批量模块校验 |

---

## +ir/ 中间表示层

### EdgeGraph

**类型**: `handle` 类  
**文件**: `+ir/EdgeGraph.m`  
**职责**: 有向位姿图容器。累积 `fixed` / `joint` / `mate` / `closed_mate` 四种边，管理 root nodes，桥接 `PosePropagator.propagatePoses()` 执行 FK 传播。

#### Public Properties

| 属性 | 类型 | 访问 | 说明 |
|------|------|------|------|
| `Edges` | `struct array` | 只读 | 字段: `from` (char), `to` (char), `T` (4×4), `kind` (char) |
| `RootNode` | `char` | 只读 | FK 传播种子节点名（单根；空字符串表示未注册） |

#### Edge Kinds

| kind | 方向 | 说明 |
|------|------|------|
| `'fixed'` | 双向 | 刚体固定变换（如 frame→body center） |
| `'joint'` | 双向 | 关节变换（revolute / prismatic，T 可含 `sym`） |
| `'mate'` | 双向 | 端口对接（socket↔plug），参与 FK 传播（生成树边） |
| `'closed_mate'` | 单向 | 闭环弦边，**不参与 FK 传播**，仅用于诊断残差 |

#### Public Methods

| 方法 | 签名 | 说明 |
|------|------|------|
| `addFixedTransform` | `(from, to, T)` | 插入 **双向** fixed 边对（含逆变换） |
| `addJoint` | `(from, to, axis, value, kind)` | 插入 **双向** joint 边对；`kind` 为 `'revolute'`（默认）或 `'prismatic'` |
| `addMateBidirectional` | `(socket, plug, roll, symmetry)` | 插入 **双向** mate 边对；T = Rz(roll×2π/sym) × Rx(π) |
| `addMateUnidirectional` | `(socket, plug, roll, symmetry)` | 插入 **单向** closed_mate 边（闭环弦边），不参与 FK |
| `addRoot` | `(node)` | 注册传播根节点（种子位姿 = eye(4)）；**单根**——重复注册不同节点报错 `ir:EdgeGraph:multipleRoots` |
| `propagate` | `() → containers.Map` | 执行 FK 传播，返回 `frame_name → 4×4 T` 的 map |
| `exportEdges` | `() → struct array` | 导出 FK 就绪的边数组（过滤 `closed_mate`，剥离 `kind` 字段），供 `PosePropagator` 消费 |
| `findMates` | `() → struct array` | 返回所有 mate / closed_mate 边（诊断用） |
| `countByKind` | `() → struct` | 按 kind 统计边数，返回 `struct('fixed',n,'joint',n,'mate',n,'closed_mate',n)` |
| `numEdges` | `() → double` | 总边数 |
| `numRootNodes` | `() → double` | 根节点数 |
| `hasRootNodes` | `() → logical` | 是否有注册的根节点 |

#### 设计要点

- **为什么是 handle class**: MATLAB 值语义会在多函数调用中强制传入传出累加器，handle class 支持原地修改，调用点干净。
- **双向边自动插入**: `addFixedTransform`、`addJoint`、`addMateBidirectional` 均自动计算并插入反向边（`core.RigidBodyMath.invT`），确保图可沿任意方向遍历。
- **mixed double/sym 支持**: `T` 矩阵可以是 `double` 或 `sym`，`core.RigidBodyMath.invT` 根据类型自动选择 `eye(4)` 或 `sym(eye(4))`，`PosePropagator` 对两种类型均透明。
- **closed_mate 的过滤**: `exportEdges()` 排除 `kind='closed_mate'` 的边，确保闭环弦边不参与 FK 传播（否则会形成环导致传播异常）。

#### 使用示例

```matlab
g = ir.EdgeGraph();
g.addRoot('world');
g.addFixedTransform('world', 'base', eye(4));
g.addJoint('base', 'link1', [0;0;1], sym('q1'), 'revolute');
g.addMateBidirectional('link1.plug', 'tool.socket', 0, 4);
poses = g.propagate();  % containers.Map: frame → 4×4 sym
```

#### See Also

`+core/PosePropagator`, `+ir/Expander`, `+viz/mechanism`, `+viz/module`

---

### Expander

**类型**: `handle` 类  
**文件**: `+ir/Expander.m`  
**职责**: DSL→IR 编排器。构造函数即完整管线：加载 DSL + 模块库 → 实例展开 → 连接处理 → FK 传播 → 输出符号位姿与符号注册表。

#### Public Properties

| 属性 | 类型 | 说明 |
|------|------|------|
| `MechName` | `char` | 机构名称（来自 DSL `mechanism` 字段） |
| `Instances` | `struct array` | 字段: `name`, `type`, `md`, `bodies`, `frames`, `joints` |
| `ConnectionInfo` | `struct array` | 字段: `socketNode`, `plugNode`, `closed`, `label` |
| `Poses` | `containers.Map` | `frame_name → 4×4 sym` 齐次变换（FK 传播结果） |
| `LibDir` | `char` | 模块库目录的绝对路径 |
| `JointVarMap` | `containers.Map` | canonical name（如 `'joint1.q'`）→ `sym` handle |
| `JointValues` | `containers.Map` | canonical name → `double`（最近一次数值代入的快照） |
| `SymbolRegistry` | `struct array` | 字段: `name`, `type`（`'joint'`/`'task'`/`'geometric'`）, `symHandle`, `scope`, `module_type`, `instance` |
| `EdgeGraph_` | `ir.EdgeGraph` | 内部位姿图句柄（供 `solver.KinematicModel` 等下游消费） |

#### Public Methods

| 方法 | 签名 | 说明 |
|------|------|------|
| `Expander` | `(dslYaml)` | **构造函数**，运行完整 DSL→IR 符号展开管线 |
| `evaluateNumeric` | `(configYaml) → containers.Map` | 加载 joint_config.yaml，代入数值，返回 `frame → 4×4 double` map |
| `evaluateNumericDirect` | `(jointValueMap, frameNames?) → containers.Map` | 直接从 `containers.Map` 代入数值（按需评估指定 frame 子集） |
| `compilePoseFunctions` | `()` | 预编译所有符号位姿为 `matlabFunction` 句柄，加速后续数值代入（~100-1000×） |

#### 构造函数内部管线

1. **路径解析**: 解析 DSL YAML 路径、模块库路径
2. **加载与校验**: `core.readYaml()` 加载 DSL，校验 `dsl_version`
3. **参数配置**: 加载 `<libDir>/config/dimensions.yaml`（几何参数，keyed by `module_type`）
4. **实例展开** (`localExpandInstance`，每个实例):
   - 加载模块 YAML → 注入几何参数
   - 展开 bodies / frames / fixed_transforms / joints
   - joint 变量**始终**创建为 `sym(name, 'real')` 实符号（A.5.4 修复：无 `real` 假设时 `invT` 的 `R'` 会在符号展开中引入 `conj(q)`，导致表达式膨胀且数值脆弱）
   - 名前缀（如 `inst1.body`）→ 写入 `EdgeGraph_`
   - 注册 `JointVarMap`、`SymbolRegistry`
5. **连接处理**: 极性校验（socket/plug） → `addMateBidirectional`（生成树）或 `addMateUnidirectional`（弦边）
6. **Root fallback + FK 传播**: `EdgeGraph_.propagate()` → `Poses` map（纯符号）
7. **填充 task frame 的 symHandle**: 从 `Poses` map 回填 `SymbolRegistry` 中 `type='task'` 的条目

#### 设计要点

- **纯符号管线 (A.4.0)**: 关节变量**始终**为 `sym`，`Poses` 始终为符号位姿 map。数值代入推迟到 `evaluateNumeric` 调用时。
- **为什么是 handle class**: 内部持有 `EdgeGraph_` 和 `DefCache_` 两个可变状态，handle 语义避免多步展开中反复传入传出。
- **模块定义缓存** (`DefCache_`): `containers.Map` 按 `module_type` 缓存已解析的模块 YAML，避免重复 IO。
- **PoseFuncs_ 加速**: `compilePoseFunctions()` 将符号位姿转为 `matlabFunction` 句柄，`evaluateNumericDirect` 在动画循环中可达到亚毫秒级/帧。

#### 使用示例

```matlab
% 符号展开
e = ir.Expander('../../specs/dsl/cases/open-chain-2r/robot_description.yaml');

% 数值代入
posesNum = e.evaluateNumeric('../../specs/dsl/cases/open-chain-2r/joint_config.yaml');

% 下游消费
km = solver.KinematicModel(e.EdgeGraph_, 'pipette.tip_origin');
cfg = ir.ExecutionConfig('exec-config.yaml', e.SymbolRegistry, e.EdgeGraph_);
```

#### See Also

`+ir/EdgeGraph`, `+ir/ExecutionConfig`, `+solver/KinematicModel`, `+viz/mechanism`

---

### ExecutionConfig

**类型**: `value` 类  
**文件**: `+ir/ExecutionConfig.m`  
**职责**: 加载并校验 L3 执行配置 YAML，交叉验证 `SymbolRegistry`，提供变量分区（known/unknown）与求解方向判定（FK/IK）。对 `closed_loop` 模式支持显式声明或从 `EdgeGraph` 的 `closed_mate` 边自动推导 `closure_cuts`。

#### Public Properties

| 属性 | 类型 | 说明 |
|------|------|------|
| `Mode` | `char` | `'open_loop'` 或 `'closed_loop'` |
| `EndFrame` | `char` | 目标 frame ref（instance-qualified，如 `'pipette.tip_origin'`） |
| `KnownList` | `cell array` | known 变量 ref 列表 |
| `UnknownList` | `cell array` | unknown 变量 ref 列表 |
| `WorldBindings` | `struct array` | 字段: `ground`, `T`（4×4 double）；open_loop 下为空 |
| `ActuatedJoints` | `cell array` | 驱动关节 ref 列表；closed_loop 下为空 |
| `ClosureCuts` | `struct array` | 字段: `near`, `far`, `components`；open_loop 下为空 |
| `ClosureSource` | `char` | `'explicit'`（YAML 声明）、`'auto'`（从 closed_mate 推导）或 `''`（open_loop） |
| `ToolFrame` | `struct` | 字段: `Present`（logical，YAML 是否声明 `tool_frame`）, `T`（4×4 double = T_endFrame←tool；角已归一化为 rad）。缺省 `Present=false`、`T=eye(4)` |
| `Tolerances` | `struct` | 字段: `translation_mm`, `rotation_rad`（默认均为 0.001） |
| `ConfigPath` | `char` | 配置文件绝对路径 |

#### Public Methods

| 方法 | 签名 | 说明 |
|------|------|------|
| `ExecutionConfig` | `(configYaml, symbolRegistry, edgeGraph?)` | 构造函数：加载 YAML → 校验 Schema → 交叉验证 SymbolRegistry |
| `partitionVariables` | `() → [knownVars, unknownVars]` | 将 SymbolRegistry 按 known/unknown 分区，返回两个 struct array |
| `getSolvingDirection` | `() → char` | 返回 `'FK'`（endFrame 在 unknown）或 `'IK'`（endFrame 在 known） |
| `getKnownJointVars` | `() → struct array` | 返回 known 中的 joint 类型变量（含 `name`, `symHandle`） |
| `getUnknownJointVars` | `() → struct array` | 返回 unknown 中的 joint 类型变量（含 `name`, `symHandle`） |

#### 构造函数校验逻辑

1. **Schema 校验**: `mode`, `endFrame`, `known`, `unknown` 必须存在
2. **Mode-conditional 校验**:
   - `open_loop`: 必须有 `actuated_joints`，禁止 `closure_cuts` 和 `world_binding`
   - `closed_loop`: 禁止 `actuated_joints`；`closure_cuts` 可显式声明或自动推导
3. **Registry 交叉校验**: 所有 ref 必须在 `SymbolRegistry` 中存在；joint 变量必须完整覆盖（不重不漏）
4. **closure_cuts 自动推导**: 若 YAML 未声明，从 `EdgeGraph.findMates()` 中提取 `kind='closed_mate'` 的边
5. **tool_frame 解析（可选）**: `tool_frame.transform` → `T_endFrame←tool`（4×4 double）。
   旋转支持 `align` / `rpy` / `axis_angle`（与模块定义同构）；角分量数值 = rad，
   或带显式单位的字符串（`"45deg"` / `"0.7854rad"`）。缺 `transform` 时报
   `ir:ExecutionConfig:invalidToolFrame`。语义（世界系重锚定）见
   `specs/dsl/connection-semantics.md` §6.5

#### 设计要点

- **value class**: 配置是不可变数据，构造后即冻结，value 语义更安全（避免意外共享修改）。
- **closure_cuts 的两种来源**: 显式声明（L3 world 系闭环，如 M-REx；`{near, far}` 帧对与 `{near, anchor: world}` world 锚定两种形式互斥）vs 自动推导（L2 内部闭环，DSL 已声明 `closed: true`）。
- **world_binding 的两种来源**: 显式声明（L3，`anchor: world` 切口的 near 必须命中其中某条 ground；条目 `T` 可省略 → 由求解器零位自动标定）vs 空（L2，ground 由 IR root nodes 处理）。

#### 使用示例

```matlab
e = ir.Expander(dslYaml);
cfg = ir.ExecutionConfig('exec-config.yaml', e.SymbolRegistry, e.EdgeGraph_);
dir = cfg.getSolvingDirection();        % 'FK' or 'IK'
[knownV, unknownV] = cfg.partitionVariables();
```

#### See Also

`+ir/Expander`, `+solver/KinematicModel`, `+solver/ClosureSolver`

---

## +solver/ 求解层

### KinematicModel

**类型**: `handle` 类  
**文件**: `+solver/KinematicModel.m`  
**职责**: 符号 FK 薄封装。从 `EdgeGraph` 的符号位姿 map 中抽取末端 frame 的 `TSym`，拆解为 `PosExpr` / `RotExpr`，提供数值求值和问题构造（FK/IK 位姿残差）。

#### Public Properties

| 属性 | 类型 | 说明 |
|------|------|------|
| `TSym` | `4×4 sym` | world→endFrame 齐次变换（符号表达式） |
| `PosExpr` | `3×1 sym` | 位置分量 `[x; y; z]` |
| `RotExpr` | `3×3 sym` | 旋转矩阵分量 |
| `JointVars` | `sym array` | `TSym` 中出现的所有关节符号变量（按 `symvar` 字母序） |
| `EndFrame` | `char` | 目标 frame 名称 |

#### Public Methods

| 方法 | 签名 | 说明 |
|------|------|------|
| `KinematicModel` | `(edgeGraph, endFrame, jointVarMap?)` | 构造函数：调用 `edgeGraph.propagate()` → 抽取 `endFrame` 的 `TSym` |
| `eval` | `(vals) → 4×4 double` | 代入数值求末端位姿；`vals` 可以是数值向量或 `containers.Map` |
| `evalPos` | `(vals) → 3×1 double` | 仅求位置分量 |
| `evalRot` | `(vals) → 3×3 double` | 仅求旋转分量 |
| `formulatePoseProblem` | `(execConfig, targetPose?) → struct` | 构造 FK/IK 求解问题，返回 `prob.Type`, `prob.eval(vals)`, `prob.JointVarNames` 等 |

#### `formulatePoseProblem` 返回值

| 字段 | 类型 | 说明 |
|------|------|------|
| `Type` | `char` | `'FK'`（末端位姿求值）或 `'IK'`（位姿误差残差） |
| `eval` | `function_handle` | FK: `@(vals) → 4×4 pose`；IK: `@(vals) → 6×1 residual` |
| `JointVarNames` | `cell array` | 关节变量 canonical name 列表 |
| `JointVarOrder` | `sym array` | 关节变量在 `eval()` 中的顺序 |
| `TargetPose` | `4×4 double` | IK 模式下的目标位姿 |

#### 设计要点

- **薄封装**: 不重复 FK 传播逻辑，直接读取 `EdgeGraph.propagate()` 的符号结果。
- **两种 eval 输入格式**: 数值向量（按 `JointVars` 顺序）或 `containers.Map`（按 canonical name），后者需传入 `JointVarMap`。
- **位姿误差定义**: IK 模式下使用 6-DOF 误差 `[tx,ty,tz,rx,ry,rz]`（位置差 mm + Z-Y-X Euler 角差 rad），由 `T_des \ T_cur` 分解得到。
- **不包含闭环约束**: 此方法仅处理末端位姿误差；闭环残差构造使用 `ClosureSolver.buildFullResidual()`。

#### 使用示例

```matlab
km = solver.KinematicModel(e.EdgeGraph_, 'pipette.tip_origin', e.JointVarMap);
T = km.eval([0.5236; -0.7854]);             % 数值位姿
prob = km.formulatePoseProblem(cfg);         % FK 或 IK 问题
```

#### See Also

`+ir/EdgeGraph`, `+ir/ExecutionConfig`, `+solver/ClosureSolver`

---

### ClosureSolver

**类型**: `handle` 类  
**文件**: `+solver/ClosureSolver.m`  
**职责**: 闭环约束构造与数值求解。支持两类 closure cut——L2 双 frame 切口（`(T_near * M) \ T_far`）与 L3 world 锚定切口（`anchor: world`，far 为标定常量）——构建 6-DOF 符号残差，转为数值函数句柄后桥接 `fmincon` 求解 IK。

#### Public Properties

| 属性 | 类型 | 说明 |
|------|------|------|
| `ClosureCuts` | `struct array` | 字段: `near`, `far`, `anchor`, `components`, `T_mate`, `T_bind`（world 锚定切口的标定常量，4×4 double） |
| `UnknownSyms` | `sym array` | unknown 关节变量的符号句柄 |
| `UnknownNames` | `cell array` | unknown 关节变量的 canonical name |
| `ResidualSym` | `sym column vector` | 完整符号残差表达式（所有 cut 拼接） |
| `Tolerances` | `struct` | 字段: `translation_mm`, `rotation_rad` |
| `NumCuts` | `double` | 闭环切割数 |
| `NumComponents` | `double` | 残差总维度 |
| `TipTarget` | `4×4 double` | ΔT——world 锚定切口的 tip 任务目标（默认 `eye(4)`，见 `setTipTarget`） |
| `ToolTransform` | `4×4 double` | U = T_root←tool（声明 `tool_frame` 时为 `T_root←endFrame·T_endFrame←tool`，否则 `I₄`） |

#### Public Methods

| 方法 | 签名 | 说明 |
|------|------|------|
| `ClosureSolver` | `(edgeGraph, execConfig, jointVarMap, knownValues?)` | 构造函数：传播符号位姿 → 标定/enrich closure cuts → known 关节代换（默认 0）→ 构建符号残差 |
| `buildResidualForCut` | `(cutIndex) → sym vector` | 为单个 closure cut 构建 6-DOF 符号残差（按 components 过滤） |
| `buildFullResidual` | `() → sym vector` | 拼接所有 cut 的残差 |
| `poseSym` | `(frameName) → 4×4 sym` | 「known 值已代换」的符号 frame 位姿（与残差同源；供 `solver.exportKinematics` 等下游编译/分析） |
| `toNumericFunction` | `() → function_handle` | 将符号残差转为 `@(x)` 数值函数句柄（缓存） |
| `verifyZeroPose` | `()` | 零位姿验证：断言 `‖residual‖ < 1e-12` |
| `setTipTarget` | `(deltaT) → void` | 设置 tip 任务目标 ΔT 并重建符号残差（仅作用于 world 锚定切口） |
| `setToolTarget` | `(deltaT) → void` | 设置**工具坐标系**任务目标（世界系左乘增量）；内部共轭 `ΔT_tip = U·Δ_tool·U⁻¹` 后委托 `setTipTarget`（未声明 tool_frame 时与 `setTipTarget` 完全一致） |
| `evalToolPose` | `(jointVals) → 4×4 double` | 给定关节值求工具坐标系位姿 `D(q) = U⁻¹·T_rootWorld(q)·U`（world-rooted FK，需 world 锚定切口） |
| `evalResidual` | `(jointVals) → vector` | 在给定关节值处求数值残差 |
| `solveIK` | `(x0?, options?) → [x_opt, fval, exitflag]` | 通过 `fmincon` SQP 最小化 `‖residual‖²` 求解 IK（options 可为 fmincon options 对象或 struct） |
| `solveIKWithTarget` | `(targetPose, endFrame, x0?, options?) → [x_opt, fval, exitflag, report]` | 组合残差 = 闭环残差 + 末端位姿误差，联合求解 |

#### 残差构造原理

对每个 closure cut：

**L2 双 frame 切口**（`near`=socket, `far`=plug）：

```
T_expected = T_near(q) * M               % M = Rz(roll*2π/sym) * Rx(π)
T_err      = inv(T_expected) * T_far(q)  % 应为 I 当闭环
```

**L3 world 锚定切口**（`anchor: world`，`T_mate = I₄`、far 为标定常量 `T_bind`）：

```
T_err = inv(T_near(q)) * (ΔT⁻¹ · T_bind) % 应为 I 当闭环
```

两处 `inv` 均经 `core.RigidBodyMath.invT` 以**闭式 SE(3) 逆**计算（无除法、展开小巧）；
等价于符号 `\`，但避免行列式（Cramer）展开的 ~1e+170 巨系数在 double 求值时溢出为
Inf/NaN（A.5.4：修复前解析雅可比在转轴关节姿态分量上有 4 个 NaN）。

- `T_bind`：零位构型标定位姿（`world_binding.T` 显式提供，或由 `calibrateBinding` 自动计算；
  显式值偏离自动标定 > 1e-6 时报 `bindingMismatch`）
- `residual = poseError(T_err)` → 6-DOF `[tx,ty,tz,rx,ry,rz]`，按 cut 的 `components` 掩码过滤；
  所有 cut 的残差纵向拼接为完整残差向量。

#### 设计要点

- **符号→数值编译**: `toNumericFunction()` 使用 `matlabFunction` 一次性编译，后续 `evalResidual` 直接调用数值句柄（无需重复 `subs`）。
- **闭式逆（A.5.4）**: 切口残差的矩阵逆一律经 `core.RigidBodyMath.invT`（SE(3) 闭式逆 `[R′, −R′t]`），替代符号 `\`（行列式展开）——残差内核提速 ~8.6×（507 → 59 µs），且解析雅可比的生成代码数值稳定（无 Inf/NaN）。
- **SQP 求解器**: 默认使用 `fmincon` 的 `sqp` 算法，tight tolerances（`OptimalityTolerance=1e-12`）。
- **零位姿验证**: 构造函数后应调用 `verifyZeroPose()` 确认符号推导无误。
- **组合求解**: `solveIKWithTarget` 将闭环约束 + 末端位姿目标同时优化，适用于同时满足闭环精度和 task-space 目标的场景。
- **L3 world 锚定切口**: `anchor: world` 的 cut 将 ground frame 锚定到 `world_binding` 标定位姿；`setTipTarget(ΔT)` 把 tip 任务目标注入常量侧（ΔT = 相对零位标定的位移），用于 3T/R 任务求解。
- **工具坐标系（tool_frame）**: 声明后“世界系 ≡ 工具坐标系零位姿态”，用户面目标位姿经 `setToolTarget` 共轭到 tip 侧、位姿报告经 `evalToolPose` 共轭回工具系；要求 root→endFrame 链无关节（否则报 `flexibleToolFrame`）。详见 `specs/dsl/connection-semantics.md` §6.5。
- **known 关节代换**: 构造时把 known 关节值（默认 0）`subs` 进全部符号位姿，残差只含 unknown 变量（构造末尾有守卫断言）。

#### 使用示例

```matlab
cs = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
cs.verifyZeroPose();                              % 零位姿验证
r = cs.evalResidual([0.5; -0.3; 0.1; -0.2]);     % 数值残差
[x_opt, fval] = cs.solveIK();                     % fmincon 求解

% L3 world 锚定闭环（M-REx）：设置 tip 目标 ΔT 后求解
csL3 = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap, ...
    containers.Map({'joint_1.q'}, {0.0}));       % known 关节值（缺省 0）
D = eye(4); D(1,4) = 10;                          % tip 沿 +X 平移 10 mm
csL3.setTipTarget(D);
[x_opt, fval] = csL3.solveIK();

% 工具坐标系目标（execution-config 声明 tool_frame 时）：世界系左乘增量
csL3.setToolTarget(D);                            % ΔT_tip = U · D · U⁻¹
[x_opt, fval] = csL3.solveIK();
Dach = csL3.evalToolPose(x_opt);                  % 工具系实际位姿（≈ D）
```

#### See Also

`+ir/EdgeGraph`, `+ir/ExecutionConfig`, `+solver/KinematicModel`

---

### buildPoseSequence

**类型**: 函数  
**文件**: `+solver/buildPoseSequence.m`  
**签名**: `seq = solver.buildPoseSequence(expander, Q, varNames, Name, Value...)`  
**职责**: FK 位姿序列构建器（A.5.2.0）——把关节轨迹 $Q$ 展开为逐帧全体 frame 位姿（`PoseSequence`）；「求解器 → 位姿数据 → 渲染」数据流中的计算层产物，渲染层（`viz.play`）只消费该数据、不做运动学计算。

#### 名称-值选项

| 选项 | 取值 | 说明 |
|------|------|------|
| `'Anchor'` | `'root'`（默认）/ `'world'` / `'tool'` | `'world'` 经 $E(q)=T_{\text{bind}}\cdot T_{g}^{-1}(q)$ 重新表达在内部世界系（ground 静止于标定值；要求 `semantic_tag: ground`）；`'tool'` 在其上再左乘 $U^{-1}$——世界 ≡ 工具系零位姿态（工具系零位 = $I_4$、tip 可见 $U^{-1}$ 倾角，与 L3 `tool_frame` 语义一致） |
| `'ToolTransform'` | `4×4 double` | $U = T_{\text{root}\leftarrow\text{tool}}$（如 `ClosureSolver.ToolTransform`）；`'Anchor','tool'` 必填 |
| `'Frames'` | cell | 纳入序列的 frame 子集（默认全部；FK root 恒含，作为 Trail 来源） |

#### 返回值字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `Frames` | `1×nFrames cell` | 每帧 `containers.Map`（frame → 4×4 double） |
| `FrameNames` / `RootFrame` / `Anchor` / `VarNames` / `Q` / `MechName` | — | 元数据 |
| `Trail` | `3×nFrames` | root（tip）世界路径（`'root'` 模式为零阵） |
| `Tbind` / `GroundFrames` / `BranchGapMax` | — | `'world'` / `'tool'` 模式的绑定、ground 列表与分支一致性诊断（`'tool'` 下 Tbind 为显示坐标 $U^{-1}T_{\text{bind}}$） |
| `ToolTransform` | `4×4 double` | 传入的 $U$（`'tool'` 模式；否则为 `[]`） |

#### 使用示例

```matlab
e = ir.Expander(dslFile);  e.compilePoseFunctions();
seq = solver.buildPoseSequence(e, Q, varNames, 'Anchor', 'world');
scene = viz.sceneSpec(e);
viz.play(seq, scene, 'Fps', 15);
```

#### See Also

`viz.play`, `viz.animate`, `+ir/Expander.evaluateNumericDirect`

---

### exportKinematics

**类型**: 函数  
**文件**: `+solver/exportKinematics.m`  
**签名**: `out = solver.exportKinematics(dslYaml, execYaml, Name, Value...)`  
**职责**: 模型导出器（A.5.4 / E1）——运行完整符号管线并把编译后的数值内核导出为**自包含模型包**（默认就地写入 `<caseDir>/generated/`）。生成的 `.m` 仅依赖 base MATLAB，供 PathPlanner / M-REx Generative 等外部消费端直接调用；三段式流水线规划（编译器 → 模型包 → 运行引擎）见 [MODEL_EXPORT.md](./MODEL_EXPORT.md)。

#### 名称-值选项

| 选项 | 取值 | 说明 |
|------|------|------|
| `'OutputDir'` | char | 模型包目录；默认 `<caseDir>/generated`（<caseDir> = DSL 文件所在目录） |
| `'KnownValues'` | `containers.Map` | known 关节的数值（canonical 名 → 值；缺省 0，同 `ClosureSolver`） |
| `'Frames'` | cell | `fk_all` 的 frame 子集（默认全部） |

#### 产物文件

| 文件 | 适用模式 | 签名 | 说明 |
|------|----------|------|------|
| `README.md` / `kinematic_metadata.json` | 全部 | — | 包文档 + 机器可读契约（输入顺序 / 切口 / 标定 / 容差 / frame 表） |
| `closure_residual.m` | closed_loop | `r = closure_residual(q)` | 闭环残差（mm / rad） |
| `closure_residual_jacobian.m` | closed_loop | `J = closure_residual_jacobian(q)` | 解析雅可比 ∂r/∂q |
| `fk_tool.m` | 全部 | `D = fk_tool(q)` | 任务位姿：L3 world 模式 = `U⁻¹·T_bind·T_near(q)⁻¹·U`（≡ `evalToolPose`）；其余 = root 锚定 endFrame 位姿 |
| `fk_all.m` / `fk_all_matrix.m` | 全部 | `poses = fk_all(q)` | 全体 frame 位姿 struct / (4N)×4 原始内核 |

**输入向量 q**：`closed_loop` = `UnknownNames` 顺序；`open_loop` = `ActuatedJoints` 顺序（均在 metadata `inputs.names`）；n×1 double 列向量；mm / rad。

#### 输出

| 字段 | 类型 | 说明 |
|------|------|------|
| `OutputDir` | char | 实际写入的包目录 |
| `Files` | cell | 写入的全部文件绝对路径 |
| `Metadata` | struct | 已序列化到 JSON 的元数据 |

#### 使用示例

```matlab
% 就地导出（写入 specs/dsl/cases/mrex-3t1r_001/generated/）
out = solver.exportKinematics( ...
    'specs/dsl/cases/mrex-3t1r_001/robot_description.yaml', ...
    'specs/dsl/cases/mrex-3t1r_001/execution-config.yaml');

% 消费端（无需 Symbolic Toolbox）
addpath(out.OutputDir);
r = closure_residual(q);  J = closure_residual_jacobian(q);
D = fk_tool(q);           poses = fk_all(q);
```

#### See Also

`+solver/ClosureSolver`, `+ir/Expander`, `+ir/ExecutionConfig`, `+solver/buildPoseSequence`

---

## +viz/ 可视化层

### mechanism

**类型**: 函数  
**文件**: `+viz/mechanism.m`  
**签名**: `result = viz.mechanism(dslYaml, configYaml?)`  
**职责**: 机构装配与可视化。委托 `ir.Expander` 完成 DSL→IR 符号展开，调用 `evaluateNumeric()` 代入数值后渲染。

#### 渲染内容

- 每个 body 的 STEP 几何（按类型着色）
- 每个 frame/port 的 RGB 坐标 triad（X=红, Y=绿, Z=蓝）
- 每个 mate 的诊断线段（socket↔plug，零长表示对齐）
- 关节轴高亮线段

#### 管线 (A.4.0)

```
DSL → ir.Expander (sym) → evaluateNumeric(config) → render
```

#### 参数来源

- 关节变量值: `configYaml`（keyed by instance name → variable）
- 几何参数: `<module_library>/config/dimensions.yaml`（keyed by `module_type`）

#### 输入

| 参数 | 类型 | 说明 |
|------|------|------|
| `dslYaml` | `char` | 机构装配 DSL YAML 路径 |
| `configYaml` | `char` | 关节变量配置文件路径（可选；未提供时代入 0） |

#### 输出

| 字段 | 类型 | 说明 |
|------|------|------|
| `mechanism` | `char` | 机构名称 |
| `poses` | `containers.Map` | `frame → 4×4 double` 数值位姿 |
| `unplaced` | `cell array` | 未能放置的 frame 列表 |

#### 使用示例

```matlab
viz.mechanism('../../specs/dsl/cases/open-chain-2r/robot_description.yaml', ...
              '../../specs/dsl/cases/open-chain-2r/joint_config.yaml');
```

#### See Also

`+ir/Expander`, `+viz/animate`

---

### module

**类型**: 函数  
**文件**: `+viz/module.m`  
**签名**: `result = viz.module(moduleYaml, configYaml?)`  
**职责**: 单模块可视化校验。直接操作 `ir.EdgeGraph`（不经过 Expander），构建模块内部 frame 图并渲染。

#### 渲染内容

- body STEP 几何（可用时）
- 每个 frame/port 的 RGB 坐标 triad
- exposed port: 实心点 + 粗体标签
- 内部 frame: 虚线 triad + 细标签
- 待提取旋转（pending SLX）: 洋红色标注

#### 管线

```
Module YAML → core.readYaml → 构建 EdgeGraph → propagate → render
```

与 `mechanism.m` 的区别：**跳过 Expander**，直接调用 `EdgeGraph` 方法，关节变量使用数值（从 config 读取或默认 0）。

#### 输入

| 参数 | 类型 | 说明 |
|------|------|------|
| `moduleYaml` | `char` | 模块定义 YAML 路径 |
| `configYaml` | `char` | 参数/关节变量配置文件路径（可选） |

#### 使用示例

```matlab
r = viz.module('../../specs/modules/joint.yaml', 'module_viz_config.yaml');
disp(r.frames.linkB);  % 4×4 double 位姿
```

#### See Also

`+ir/EdgeGraph`, `+viz/allModules`

---

### play

**类型**: 函数  
**文件**: `+viz/play.m`  
**签名**: `viz.play(poseSeq, scene, Name, Value...)`  
**职责**: 位姿序列纯播放器（A.5.2.0）。只消费 `(PoseSequence, SceneSpec)` 纯数据——**不做任何运动学计算**；逐帧读取 `poseSeq.Frames` 渲染机构几何，world 模式下叠加 `poseSeq.Trail` 轨迹与静态 ground 标记。

#### 名称-值选项

| 选项 | 取值 | 说明 |
|------|------|------|
| `'Fps'` | `double`（默认 10） | 播放速度 |
| `'ShowTrail'` | `logical`（默认 `true`） | tip 轨迹叠加（`'world'` / `'tool'` 序列生效） |
| `'ExtraFrames'` | struct array `{Label, Base, T}` | 逐帧挂载绘制的虚拟坐标系：位姿 = `pose(Base)·T`（如把 L3 工具系 triad 挂到 FK root 上；`'tool'` 世界里与世界轴重合） |

#### 播放控件

与 `viz.animate` 相同（▶ Play / ⏮ Reset / Speed / Frame）。

#### See Also

`solver.buildPoseSequence`, `viz.sceneSpec`, `viz.animate`

---

### sceneSpec

**类型**: 函数  
**文件**: `+viz/sceneSpec.m`  
**签名**: `scene = viz.sceneSpec(expander)`  
**职责**: 从 `Expander` 提取配置无关的静态场景快照，供 `viz.play` 渲染使用。

#### 返回值字段

| 字段 | 说明 |
|------|------|
| `MechName` / `Instances` / `ConnectionInfo` / `LibDir` / `RepoRoot` / `NInst` / `NConns` / `L` | 静态场景数据（含特征尺度） |
| `GeomCache` | `containers.Map`：几何路径 → 预加载几何（避免逐帧磁盘 IO） |

#### See Also

`viz.play`, `viz.animate`, `+ir/Expander`

---

### animate

**类型**: 函数  
**文件**: `+viz/animate.m`  
**签名**: `viz.animate(dslYaml, jointTrajectory, varNames, fps?, Name, Value...)`  
**职责**: 轨迹动画**兼容包装**（A.5.2.0 起）——内部 ＝ `ir.Expander` → `solver.buildPoseSequence` → `viz.sceneSpec` → `viz.play`；外部签名与选项保持不变。分层 API 见 `viz.play` / `solver.buildPoseSequence`。

#### 播放控件

| 控件 | 功能 |
|------|------|
| ▶ Play / ⏸ Pause | 切换播放/暂停 |
| ⏮ Reset | 跳回第一帧 |
| Speed 滑块 | 调整 fps（1–60） |
| Frame 滑块 | 拖拽到任意帧 |

#### 名称-值选项

| 选项 | 取值 | 说明 |
|------|------|------|
| `'Anchor'` | `'root'`（默认）或 `'world'` | 显示参照系。`'world'` 把 tool-rooted FK 位姿经 $E(q)=T_{\text{bind}}\cdot T_{\text{tip}\leftarrow g_{\text{ref}}}^{-1}(q)$ 共轭到世界系，使 L3 世界系闭环机构（如 mrex-3t1r_001）的两台 Manipulator ground frame 静止于标定位姿、工具端真实移动。要求机构存在 `semantic_tag: 'ground'` 的 frame；$T_{\text{bind}}$ 取零位 FK（与 `solver.ClosureSolver` 自动标定一致）；多余 ground frame 打印分支一致性诊断 |
| `'ShowTrail'` | `logical`，默认 `true` | 叠加 tip 世界轨迹：完整路径淡虚线 + 已走过段高亮 + 当前 tip 标记（仅 `'world'` 模式生效） |

#### 性能优化（位于计算层与渲染层）

1. **预编译**: `expander.compilePoseFunctions()` 将符号位姿转为 `matlabFunction` 句柄（一次性，~秒级）
2. **序列预计算**: 位姿在播放前一次性算好（`PoseSequence`），渲染循环无运动学开销
3. **几何预加载**: `viz.sceneSpec` 导入一次 STEP 几何 → `geomCache`，避免逐帧磁盘 IO

#### 输入

| 参数 | 类型 | 说明 |
|------|------|------|
| `dslYaml` | `char` | 机构装配 DSL YAML 路径 |
| `jointTrajectory` | `nFrames × nVars double` | 关节轨迹矩阵 |
| `varNames` | `cell array` | 列对应的 canonical 变量名 |
| `fps` | `double` | 播放速度（默认 10；可省略或传 `[]`） |
| `Name, Value...` | 名称-值对 | 可选，位于 `fps` 之后；见「名称-值选项」 |

#### 使用示例

```matlab
theta = linspace(0, pi/3, 60)';
Q = [theta, -theta, theta, -theta];
varNames = {'joint_AB.q','joint_BD.q','joint_CD.q','joint_AC.q'};
viz.animate('robot_description.yaml', Q, varNames, 15);
```

```matlab
% world 锚定动画（L3 世界系闭环机构，如 mrex-3t1r_001）：
% Q 由 tests/pipeline/test_animation_m_rex_3t1r.m 顶部参数区闭式构造生成
viz.animate(dslFile, Q, varNames, 15, 'Anchor', 'world');
```

#### See Also

`solver.buildPoseSequence`, `viz.play`, `viz.sceneSpec`, `+ir/Expander`

---

### allModules

**类型**: 函数  
**文件**: `+viz/allModules.m`  
**签名**: `viz.allModules(configYaml?)`  
**职责**: 批量校验 `specs/modules/` 下所有模块 YAML，每个模块打开一个 figure 并打印文本报告。

#### 管线

```
遍历 specs/modules/*.yaml → viz.module(f, configYaml) × N
```

失败时打印错误信息但不中断后续模块。

#### 使用示例

```matlab
viz.allModules();                                  % 使用默认 config
viz.allModules('custom_config.yaml');              % 使用自定义 config
```

#### See Also

`+viz/module`

---

## 附录：关键类型速查

### EdgeGraph.Edges 的 `kind` 枚举

| kind | FK 传播 | 方向 | 用途 |
|------|---------|------|------|
| `'fixed'` | ✅ | 双向 | 刚体固定变换 |
| `'joint'` | ✅ | 双向 | revolute / prismatic 关节 |
| `'mate'` | ✅ | 双向 | socket↔plug 对接（生成树边） |
| `'closed_mate'` | ❌ | 单向 | 闭环弦边（仅诊断残差） |

### SymbolRegistry 的 `type` 枚举

| type | symHandle | 来源 |
|------|-----------|------|
| `'joint'` | `sym` 标量 | 关节的 `variable` 字段（`observable: true`） |
| `'task'` | `4×4 sym` 矩阵 | body/frame 的 `observable: true`（位姿） |
| `'geometric'` | （保留） | 几何参数（当前未使用） |

### 求解方向判定

| 条件 | `getSolvingDirection()` | 含义 |
|------|------------------------|------|
| `endFrame ∈ unknown` | `'FK'` | 已知 joint values → 求末端位姿 |
| `endFrame ∈ known` | `'IK'` | 已知目标位姿 → 求 joint values |

---

> **文档维护**: 当新增类或为现有类添加公开方法/属性时，请同步更新本文档。
