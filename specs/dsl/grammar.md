# 机构描述语言（DSL）v0 语法规范（阶段 A.2.1）

> 本文档定义 L2 机构装配语言的完整语法规则，使任意机构结构有唯一合法写法。
> 权威建模约定见 `../modeling-conventions.md`（§9 端口、§10 连接、§11 参数作用域），
> 机器可读常量见 `../conventions.yaml`，模块字段定义见 `../schema/module-definition.schema.yaml`。
> 连接的几何语义（mate 变换、roll、closed）见 `connection-semantics.md`。
> 静态结构校验的形式化表达见 `../schema/mechanism-assembly.schema.yaml`。
>
> **状态**：A.2 v0。凡与本文档冲突的写法一律视为非法。

---

## 1. 定位与边界

### 1.1 DSL 是什么

DSL 是**机构结构说明书**：用文本描述一台具体机构由哪些模块实例组成、这些实例如何按端口连接。
它对应四层架构（`modeling-conventions.md` §2）中的 **L2 机构层**——只描述「本体长什么样、怎么拼」。

### 1.2 DSL 不是什么

DSL **不涉及求解，也不描述执行语义**。以下内容一律归 **L3 执行配置**：

| 不在 DSL 中 | 归属 |
|------|------|
| `world` 根与接地绑定 | L3 `world_binding` |
| `endFrame`（FK/IK 输出目标） | L3 变量分区 |
| `actuated`（驱动指派） | L3 `actuated_joints` |
| `known` / `unknown` 变量分区 | L3 partition |
| 标定偏移、求解边界、初值、收敛阈值 | L3 execution-config |
| 闭环回路的切口分量选择 | L3 `closure_cuts` |

> 设计依据：同一套机构本体（L2）可接入不同执行层（L3）——闭环执行得 IK 残差，开环执行得 FK 表达式（`modeling-conventions.md` §2.4）。把执行语义写进 DSL 会破坏复用性。因此 DSL 严格保持为**纯拓扑**：只有实例与连接。

---

## 2. 文件格式

- DSL 机构文件采用 **YAML**，与 A.1 模块定义（`../modules/*.yaml`）保持同一工具链，可由
  `mechanism-assembly.schema.yaml` 做 JSON Schema 静态校验。
- 文件扩展名 `.yaml`。建议在文件首行标注 schema：
  `# yaml-language-server: $schema=../schema/mechanism-assembly.schema.yaml`
- 一个文件描述一台机构。
- 注释使用 YAML 原生 `#`。推荐在文件头部注释中放置拓扑图（Mermaid 或 ASCII）与符号变量表。

---

## 3. 顶层结构

一份 DSL 机构文件是一个 YAML 映射，含以下顶层键：

| 键 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `dsl_version` | 整数 | 是 | DSL 语法版本号；v0 恒为 `0` |
| `mechanism` | 标识符 | 是 | 机构名，`^[a-zA-Z][a-zA-Z0-9_]*$` |
| `description` | 字符串 | 否 | 人类可读描述 |
| `module_library` | 路径 | 否 | 模块库目录，缺省 `../modules/` |
| `instances` | 映射 | 是 | 模块实例声明（§4）；至少一个 |
| `connections` | 列表 | 否 | 端口连接声明（§5）；单实例机构可为空 |

顶层不允许出现上述以外的键（`additionalProperties: false`）。

### 3.1 顶层骨架

```yaml
# yaml-language-server: $schema=../schema/mechanism-assembly.schema.yaml
dsl_version: 0
mechanism: open_chain_2r
description: 两段串联转动副加工具末端的开环链。
module_library: ../modules/

instances:
  # …见 §4
connections:
  # …见 §5
```

---

## 4. 实例声明（`instances`）

### 4.1 语法

`instances` 是一个**映射**，键为实例名，值为实例定义：

```yaml
instances:
  <instance_name>:
    type: <ModuleType>
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| （键）`instance_name` | 标识符 | 是 | 实例名，机构内唯一（映射键天然去重） |
| `type` | 模块类型名 | 是 | 引用的模块 `module_type`，必须在模块库中存在 |

v0 实例定义只允许 `type` 一个字段（`additionalProperties: false`）。

### 4.2 实例名规则

- 采用 **lowerCamelCase 或 snake_case**（`modeling-conventions.md` §4）。
- 正则：`^[a-z][a-zA-Z0-9_]*$`。
- **不得与 L0 元件关键字重名**：`body`、`frame`、`fixedTransform`、`joint`、`constraint`、`port`。
- 机构内唯一（YAML 映射键唯一性保证；重复键为非法 YAML）。

### 4.3 类型引用规则

- `type` 的取值必须等于模块库中某个模块文件的 `module_type` 字段（如 `Frame`、`Joint`、
  `Adaptor_45`、`Manipulator`、`Pin`、`ToolPipette`）。
- 采用 **UpperCamelCase**（模块类型名规则）；同族变体允许 `_<后缀>` 区分型号（如 `Adaptor_45`、`Adaptor_90`）。
- 「该 `module_type` 是否存在」属于跨文件校验，Schema 无法覆盖，由解释器加载模块库后校验（§7）。

### 4.4 实例参数规则（v0）

- 模块类参数（如 `Frame.cubeLength`、`ToolPipette.tipDistance`）在 **L1 模块定义**
  中**声明**（`name`/`unit`/`description`），但**取值**由独立配置文件
  `specs/modules/config/dimensions.yaml` 注入——该文件按 `module_type` 映射参数名到具体数值。
  解释器在加载模块库时读取此配置，将参数值注入模块类。
- **v0 禁止 variant**：同一模块类型的所有实例**参数完全相同**（与 Simulink 库块建模一致）。
  因此 DSL 实例声明**不写 `parameters` 块**；所有同类型实例统一继承 `config/dimensions.yaml` 中的取值。
- **v0 禁止跨实例引用**：参数值中不得出现对其他实例名或其参数的引用。
  跨实例依赖留待后续版本。
- 未在 `config/dimensions.yaml` 中配置的模块参数，其取值策略由 A.3 解释器与 L3 配置决定。
- 角度类参数若出现，必须显式标注单位（`modeling-conventions.md` §7），如 `"30 deg"`。

### 4.5 实例声明示例

```yaml
instances:
  frame0:
    type: Frame
  joint1:
    type: Joint          # 无实例参数
  pipette:
    type: ToolPipette
```

---

## 5. 连接声明（`connections`）

### 5.1 语法

`connections` 是一个**列表**，每个元素声明一条端口连接：

```yaml
connections:
  - ports: [<instance>.<port>, <instance>.<port>]
    roll: <int>       # 可选，缺省 0
    closed: <bool>    # 可选，缺省 false
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `ports` | 2 元素列表 | 是 | 连接的两个端口引用（§5.2）；**顺序无关**（§5.3） |
| `roll` | 整数 | 否 | 离散装配滚转索引，缺省 0（§5.4） |
| `closed` | 布尔 | 否 | 是否为闭环补边，缺省 false（§5.5） |

连接元素不允许出现上述以外的键。

### 5.2 端口引用格式

- 端口引用为字符串 `"<instance_name>.<port_name>"`，单个 `.` 分隔。
- `instance_name` 必须是 `instances` 中已声明的实例。
- `port_name` 必须是该实例对应模块中 `exposed: true` 的端口。
- 正则（Schema 层）：`^[a-z][a-zA-Z0-9_]*\.(face[XYZ][+-]|[a-zA-Z][a-zA-Z0-9_]*)$`。
- 「端口是否存在且 exposed」属跨文件校验，由解释器校验（§7）。

### 5.3 顺序无关（方向由极性推断）

- `ports` 列表的两个端口**顺序无关**：`[a.p, b.q]` 与 `[b.q, a.p]` 等价，指代同一连接。
- 连接不声明 socket/plug 谁是父谁是子。解释器读取两端口的 `polarity`，自动确定
  **socket 为父、plug 为子**，再施加标准 mate 变换（`connection-semantics.md` §2）。
- 因此用户无需记忆极性方向；只需保证两端极性互补（socket↔plug），否则解释器判非法。

### 5.4 roll 字段

- `roll` 为整数索引，实际装配滚转角 = `roll × 360 / symmetry` 度（`connection-semantics.md` §3）。
- 合法取值 `0 .. symmetry-1`（`symmetry` 由端口定义给出，缺省 4）。
- 缺省 0。
- 只表达**离散装配朝向**；运行时连续绕法向转动须改用 revolute `Joint` 模块（§5.6）。
- **传播性**：`roll` 会永久旋转下游所有模块的朝向。若仅需旋转 Joint 模块的关节轴方向而不影响下游，
  必须在 Joint 两端**成对使用 `roll`**（`roll: 1` / `roll: -1`）。详见 `connection-semantics.md` §3.5。

### 5.5 closed 字段

- `closed: true` 标记闭环补边（chord），缺省 `false`（树边）。
- 语义与 L2/L3 闭环归属详见 `connection-semantics.md` §6。

### 5.6 连接限制

连接的唯一含义是「两端口面对面对插贴合」（`modeling-conventions.md` §10）。以下需求**必须插入模块**：

| 需求 | 正确做法 |
|------|------|
| 坐标适配（额外平移/旋转） | 插入 `Adaptor_45` 等 Adaptor 模块 |
| 运行时连续绕法向转动 | 插入 revolute `Joint` 模块 |

### 5.7 闭环归属：L2 vs L3

详见 `connection-semantics.md` §6.4。简表：

| 回路类型 | 声明位置 |
|------|------|
| L2 机构内闭环（四杆环等） | DSL `closed: true` |
| L3 世界系闭环（M-REx 主构型） | L3 execution-config `closure_cuts` |

### 5.8 连接声明示例

```yaml
connections:
  # 基本连接（无 roll，缺省朝向）
  - ports: [frame0.faceXPlus, joint1.linkA]
  - ports: [joint1.linkB, frame1.faceXMinus]
  # Joint 两端 roll 配对：旋转关节轴，下游朝向不变（§5.4 配对规则）
  - ports: [frame1.faceZPlus, joint2.linkA]
    roll: 1
  - ports: [joint2.linkB, frame2.faceZMinus]
    roll: -1
  # 闭环补边
  - ports: [joint_C.linkB, frame_A.faceZPlus]
    closed: true
```

---

## 6. 合法性约束（prose 汇总）

以下约束共同保证「同一机构结构无两种等价但解释不同的写法」。分为 Schema 可校验与解释器须校验两类
（形式化分界见 `validation-checklist.md`）。

**Schema 可静态校验（`mechanism-assembly.schema.yaml`）**：
- 顶层键完整且无多余键；`dsl_version == 0`。
- `mechanism`、实例名、端口引用符合正则。
- `instances` 至少一个；每个实例含 `type`。
- 每条连接 `ports` 恰为 2 元素；`roll` 为非负整数；`closed` 为布尔。

**解释器须校验（跨文件/跨数组，Schema 无法覆盖）**：
- 每个实例 `type` 在模块库中存在。
- 实例名不与 L0 关键字冲突。
- 每个端口引用的实例已声明、端口在模块中 `exposed: true`。
- 连接两端**极性互补**（仅 `socket↔plug` 合法；`socket↔socket`、`plug↔plug`、
  任一端无极性者非法）。
- 每个端口**至多被占用一次**（无重复占用）。
- `roll` 落在 `0 .. symmetry-1`。
- 无悬空/无效端口引用。
- `closed` 连接确实闭合一个 L2 回路（不悬空）。

---

## 7. 唯一性与过关标准

- **写法唯一性**：同一机构结构不存在两种等价但语法不同的合法写法。一条连接由其无序端口对唯一标识，重复声明同一端口对为非法（端口单次占用）。
- **可校验性**：`mechanism-assembly.schema.yaml` 对非法连接、重复占用端口、悬空端口、缺失字段能明确报错。
- **案例过关**：`cases/` 下案例均通过 Schema 校验。

> 实现细节（DSL→IR 展开、命名空间前缀、mate 变换施加）见 `specs/ir/dsl-to-ir-mapping.md`。
