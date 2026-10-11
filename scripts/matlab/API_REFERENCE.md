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
| `+viz/` | [mechanism](#mechanism) | 函数 | 机构装配与可视化 |
| `+viz/` | [module](#module) | 函数 | 单模块可视化校验 |
| `+viz/` | [animate](#animate) | 函数 | 关节轨迹动画（world 锚定 + tip 轨迹） |
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
   - joint 变量**始终**创建为 `sym` 对象
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

#### Public Methods

| 方法 | 签名 | 说明 |
|------|------|------|
| `ClosureSolver` | `(edgeGraph, execConfig, jointVarMap, knownValues?)` | 构造函数：传播符号位姿 → 标定/enrich closure cuts → known 关节代换（默认 0）→ 构建符号残差 |
| `buildResidualForCut` | `(cutIndex) → sym vector` | 为单个 closure cut 构建 6-DOF 符号残差（按 components 过滤） |
| `buildFullResidual` | `() → sym vector` | 拼接所有 cut 的残差 |
| `toNumericFunction` | `() → function_handle` | 将符号残差转为 `@(x)` 数值函数句柄（缓存） |
| `verifyZeroPose` | `()` | 零位姿验证：断言 `‖residual‖ < 1e-12` |
| `setTipTarget` | `(deltaT) → void` | 设置 tip 任务目标 ΔT 并重建符号残差（仅作用于 world 锚定切口） |
| `evalResidual` | `(jointVals) → vector` | 在给定关节值处求数值残差 |
| `solveIK` | `(x0?, options?) → [x_opt, fval, exitflag]` | 通过 `fmincon` SQP 最小化 `‖residual‖²` 求解 IK（options 可为 fmincon options 对象或 struct） |
| `solveIKWithTarget` | `(targetPose, endFrame, x0?, options?) → [x_opt, fval, exitflag, report]` | 组合残差 = 闭环残差 + 末端位姿误差，联合求解 |

#### 残差构造原理

对每个 closure cut：

**L2 双 frame 切口**（`near`=socket, `far`=plug）：

```
T_expected = T_near(q) * M           % M = Rz(roll*2π/sym) * Rx(π)
T_err      = T_expected \ T_far(q)   % 应为 I 当闭环
```

**L3 world 锚定切口**（`anchor: world`，`T_mate = I₄`、far 为标定常量 `T_bind`）：

```
T_err = T_near(q) \ (ΔT⁻¹ · T_bind)  % 应为 I 当闭环
```

- `T_bind`：零位构型标定位姿（`world_binding.T` 显式提供，或由 `calibrateBinding` 自动计算；
  显式值偏离自动标定 > 1e-6 时报 `bindingMismatch`）
- `residual = poseError(T_err)` → 6-DOF `[tx,ty,tz,rx,ry,rz]`，按 cut 的 `components` 掩码过滤；
  所有 cut 的残差纵向拼接为完整残差向量。

#### 设计要点

- **符号→数值编译**: `toNumericFunction()` 使用 `matlabFunction` 一次性编译，后续 `evalResidual` 直接调用数值句柄（无需重复 `subs`）。
- **SQP 求解器**: 默认使用 `fmincon` 的 `sqp` 算法，tight tolerances（`OptimalityTolerance=1e-12`）。
- **零位姿验证**: 构造函数后应调用 `verifyZeroPose()` 确认符号推导无误。
- **组合求解**: `solveIKWithTarget` 将闭环约束 + 末端位姿目标同时优化，适用于同时满足闭环精度和 task-space 目标的场景。
- **L3 world 锚定切口**: `anchor: world` 的 cut 将 ground frame 锚定到 `world_binding` 标定位姿；`setTipTarget(ΔT)` 把 tip 任务目标注入常量侧（ΔT = 相对零位标定的位移），用于 3T/R 任务求解。
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
```

#### See Also

`+ir/EdgeGraph`, `+ir/ExecutionConfig`, `+solver/KinematicModel`

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
| `unplacedFrames` | `cell array` | 未能放置的 frame 列表 |

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

### animate

**类型**: 函数  
**文件**: `+viz/animate.m`  
**签名**: `viz.animate(dslYaml, jointTrajectory, varNames, fps?, Name, Value...)`  
**职责**: 通过关节轨迹序列动画展示机构运动。构建一次 `Expander`，预编译位姿函数，逐帧代入数值渲染；可选 world 锚定渲染（L3 世界系闭环）与 tip 世界轨迹叠加。

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
| `'Anchor'` | `'root'`（默认）或 `'world'` | 显示参照系。`'world'` 把 tool-rooted FK 位姿经 $E(q)=T_{\text{bind}}\cdot T_{\text{tip}\leftarrow g_{\text{ref}}}^{-1}(q)$ 共轭到世界系，使 L3 世界系闭环机构（如 m-rex-3t1r）的两台 Manipulator ground frame 静止于标定位姿、工具端真实移动。要求机构存在 `semantic_tag: 'ground'` 的 frame；$T_{\text{bind}}$ 取零位 FK（与 `solver.ClosureSolver` 自动标定一致）；多余 ground frame 打印分支一致性诊断 |
| `'ShowTrail'` | `logical`，默认 `true` | 叠加 tip 世界轨迹：完整路径淡虚线 + 已走过段高亮 + 当前 tip 标记（仅 `'world'` 模式生效） |

#### 性能优化

1. **预编译**: `expander.compilePoseFunctions()` 将符号位姿转为 `matlabFunction` 句柄（一次性，~秒级）
2. **按需评估**: 仅评估 body + joint frame（非全部 frame），`evaluateNumericDirect` 利用预编译句柄达到亚毫秒/帧
3. **几何预加载**: 导入一次 STEP 几何 → `geomCache`，避免逐帧磁盘 IO

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
% world 锚定动画（L3 世界系闭环机构，如 m-rex-3t1r）：
% Q 由 tests/pipeline/test_animation_m_rex_3t1r.m 顶部参数区闭式构造生成
viz.animate(dslFile, Q, varNames, 15, 'Anchor', 'world');
```

#### See Also

`+viz/mechanism`, `+ir/Expander.evaluateNumericDirect`, `+ir/Expander.compilePoseFunctions`

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
