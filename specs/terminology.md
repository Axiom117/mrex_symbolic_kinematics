# 术语表（阶段 A.0）

> 后续 DSL、模块 schema 与解释器共享的统一术语。中文释义 + 英文规范名（解释器内部 IR 用名）。
> 约定细节见 `modeling-conventions.md`，机器可读常量见 `conventions.yaml`。

> 说明：层级名（`Element`/`Module`/`Mechanism`/`Execution`）保留 UpperCamelCase；凡直接进入 DSL / IR 的元件、结构与数据字段名，按 `modeling-conventions.md §4` 统一使用 lowerCamelCase。

---

## 层级架构（建模层次）

自底向上四层（详见 `modeling-conventions.md` §2）。

| 英文规范名 | 层 | 中文 | 含义 |
|------|------|------|------|
| `Element` | L0 | 元件 | 建模原语（`Body`/`Frame`/`Port`/`FixedTransform`/`Joint`/`Constraint`） |
| `Module` | L1 | 模块 | 封装若干元件的参数化模板（= 类） |
| `Mechanism` | L2 | 机构 | 多个模块实例按端口连接组成的机构本体 |
| `Execution` | L3 | 执行层 | 世界系 + 驱动源，闭合成可求解系统 |
| `externalDriver` | L3 | 外部驱动 | 提供多自由度驱动的驱动源，用于闭环构型；由 `Manipulator` 关节模块物化（见 `modeling-conventions.md §2.4、§5.6`） |
| `actuatedJoint` | L3 | 驱动关节 | 在执行层被指派为可驱动（`actuated`）的关节，用于开环串联 |
| `wrench` | L3 | 力旋量 | 外部驱动提供的多自由度广义力/位移输入 |
| `closureCriteria` | L3 | 闭环判据 | 回路切口处的相对位姿误差条件，生成求解残差 |

---

---

## 核心运动学原语

| 英文规范名 | 中文 | 含义 |
|------|------|------|
| `body` | 刚体 | 不可形变的实体，自带中心 `frame`，模块内部基本单元 |
| `frame` | 坐标系 | 一个局部右手参考系；位姿由 `fixedTransform` 给出 |
| `port` | 端口 | `exposed=true` 的 `frame`，用于跨模块连接；非独立元件 |
| `joint` | 关节 | 引入自由度的运动副，本阶段以 1-DOF revolute 为主 |
| `fixedTransform` | 固定变换 | 两坐标系间无自由度的刚性位姿关系 |
| `jointTransform` | 关节变换 | 由关节变量参数化的位姿关系 |
| `constraint` | 约束 | 闭环相容性的代数条件（残差候选项） |
| `symbolRegistry` | 符号表 | 尺寸参数、任务变量、关节变量的集中登记 |
| `observable` | 可观测标记 | frame 或 joint 上的布尔字段；真则该量被注册到变量注册表供后续分区求解；独立于 `exposed`（见 `modeling-conventions.md` §3.6） |
| `variableRegistry` | 变量注册表 | IR 展开时收集所有 `observable` frame 和 joint variable 的扁平列表，以实例限定名索引（如 `mc1.dx`） |
| `known` | 已知量 | 变量注册表中被分区为已知输入的变量（运行时提供数值，代入方程） |
| `unknown` | 未知量 | 变量注册表中被分区为待求输出的变量（由求解器计算）；`known`/`unknown` 分区决定 FK/IK 方向 |

## 结构与装配

| 英文规范名 | 中文 | 含义 |
|------|------|------|
| `moduleType` | 模块类 | 模块的类型定义（字段、端口、内部拓扑）；即 L1 `Module` |
| `Manipulator` | 驱动接口模块 | 把外部驱动器（如 microsupport）物化为 L1 关节模块；3-DOF 笛卡尔平移（三个 `prismatic` 串联，变量 `dx`/`dy`/`dz`），并把机构接地到 `world`（见 `§5.6`） |
| `instance` | 实例 | 模块类的一次具体化，带实例名与参数 |
| `connection` | 连接 | 两端口间的 mate 关系（见连接类型） |
| `world` | 世界系 | 机构图唯一根坐标系；缺省与 FK root 零位姿态重合，声明 `tool_frame` 后锚定到工具坐标系零位姿态 |
| `endFrame` | 末端系 | FK/IK 输出的任务坐标系（图内物理帧） |
| `toolFrame` | 工具坐标系 | L3 可选输入/输出参考系（`execution-config.yaml` 的 `tool_frame` 块，定义相对 `endFrame` 的固定变换）。声明后“世界系 ≡ 工具坐标系零位姿态”，目标位姿按左乘世界系增量表达（$\Delta T = U\cdot\Delta_{\text{tool}}\cdot U^{-1}$，$U = T_{\text{root}\leftarrow\text{tool}}$）。虚拟接口帧，不是实体坐标系；与 `root` 标记的 tip 参考帧区分（见 `dsl/connection-semantics.md` §6.5） |

## 端口语义标签（可选）

| 名 | 含义 |
|------|------|
| `connector` | 结构连接面 |
| `joint_side` | 关节侧连接面 |
| `tool` | 工具/功能参考点 |
| `dock` | 适配/对接面 |
| `ground` | 世界侧接地系（`Manipulator.ground`），无 `polarity`，由 L3 绑定到 `world`；非机械 mate。不触发 FK root 自动注册 |
| `root` | FK 传播起点（`ToolPipette.connector_side`），展开时自动调用 `addRoot()`，种子位姿 $T = I_4$ |

## 端口机械对接（mate）

| 英文规范名 | 中文 | 含义 |
|------|------|------|
| `polarity` | 端口极性 | 机械对接公母；`socket`（凹面钢片，`Frame` 六面与 `Manipulator.dock`）/ `plug`（凸面磁铁，其余机械面）。仅 `socket↔plug` 合法。任务/工具参考系与 `ground` 接地系不带 polarity（见 `modeling-conventions.md §9.5`） |
| `socket` | 母口 | 凹陷接收面，由 `Frame` 六面与 `Manipulator.dock` 提供 |
| `plug` | 公口 | 凸出对插面，`Pin`/`Joint`/`Adaptor`/工具连接面等 |
| `symmetry` | 旋转对称阶 | 端口绕 `+Z` 对插法向的旋转对称阶；缺省 4（方钢片 C4），约束 `roll` 合法离散取值（见 `§9.6`） |
| `roll` | 装配滚转 | 连接级离散参数，实际角 = `roll · 360/symmetry` 度；仅离散，连续转动须用 `Joint` 模块（见 `§10.3`） |
| `mateTransform` | 对插变换 | 解释器装配时施加的标准变换 `Rz(roll·360/symmetry)·Rx(π)`，使两端口面对面贴合（见 `§10.2`） |

## 连接类型

连接的含义唯一（见 `modeling-conventions.md §10`）：

| 名 | 含义 |
|------|------|
| `coincident` | 两端口经标准 mate 变换后面对面贴合（唯一合法连接类型）；所有 port `+Z` 统一朝外，解释器按极性自动施加 `mateTransform` |

> 原 `normal_aligned`（绕法向连续旋转）须通过插入 `Joint` 模块实现；
> 原 `adaptor`（坐标适配）须通过插入 `Adaptor` 模块实现。两者均不再是连接类型。
> 装配时的离散朝向选择由连接级 `roll` 表达（见 `§10.3`），非连接类型。

## 参数作用域

| 名 | 含义 |
|------|------|
| `module-class` | 属于模块类型定义 |
| `instance` | 属于单个实例 |
| `mechanism-config` | 属于整个机构配置 |
