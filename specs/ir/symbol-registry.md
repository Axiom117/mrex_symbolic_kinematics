# IR 符号变量注册表规范（阶段 A.3.3）

> 本文档定义 IR（中间表示）中 `symbolRegistry` 的数据结构、变量类型分类和收集规则。
> 权威代码来源：`scripts/matlab/+ir/Expander.m`（`localExpandInstance` 展开时收集）。
>
> **状态**：A.3.3 v0。以已验证的 MATLAB 代码为准反推，非从零设计。

---

## 1. 概述

`symbolRegistry` 是 IR 展开阶段收集的**扁平符号变量清单**。它汇总机构中所有标记为 `observable: true` 的 frame 和 joint variable，以实例限定名（instance-qualified name）作为唯一标识。下游消费者（`ExecutionConfig`、`ClosureSolver`、A.5 求解器）通过 `symbolRegistry` 了解「机构中有哪些量可以参与求解」，而不需要遍历整个 IR 图。

### 1.1 变量来源

| 来源 | observability 条件 | IR 中的位置 |
|------|------|------|
| 模块 frame | `frame.observable == true` | `Expander.Instances(i).frames{k}.observable` |
| 模块 joint | `joint.observable == true`（默认所有 joint 为 observable） | `Expander.Instances(i).joints{k}` |

### 1.2 命名约定

所有条目名称为**实例限定名**：`instanceName.elementName`

- 示例：`joint1.q`（Joint 模块实例 `joint1` 的关节变量 `q`）
- 示例：`pipette.tip_origin`（ToolPipette 实例 `pipette` 的任务参考系 `tip_origin`）
- 示例：`frame0.frame_hyper_cube`（Frame 模块实例 `frame0` 的 body 中心坐标系 `frame_hyper_cube`）

---

## 2. 数据结构

`symbolRegistry` 是一个 struct 数组，每个元素含以下字段：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | char | 是 | 实例限定名，IR 中唯一标识（如 `joint1.q`） |
| `type` | char | 是 | 变量类型枚举：`'geometric'` / `'joint'` / `'task'` |
| `symHandle` | sym | 是 | MATLAB symbolic 句柄。geometric 类型为常数 sym（如 `sym(50)`）；joint 类型为自由符号（如 `sym('joint1_q')`）；task 类型为 frame 的 4×4 位姿 sym 表达式 |
| `scope` | char | 是 | 作用域：`'module-class'` / `'instance'` / `'mechanism'` |
| `module_type` | char | 否 | 来源模块类型（如 `Joint`、`ToolPipette`），`task` 类型 frame 所属模块 |
| `instance` | char | 是 | 来源实例名（如 `joint1`、`pipette`） |

---

## 3. 变量类型

### 3.1 `geometric` — 几何参数

模块类的尺寸参数，在模块定义中以符号表达式给出（如 `cubeLength/2`），由 `dimensions.yaml` 注入数值后在 IR 展开时求值为常数。

- **性质**：常数（数值包装为 `sym`），求解时不作为未知量
- **注册时机**：模块展开时检测到参数表达式中的符号（如 `cubeLength`）→ 求值后注册为 `sym(50)`（数值）
- **示例**：`frame0.cubeLength`（假设 cubeLength 被注册为 observable geometric 参数）

> **A.3.3 v0 简化**：当前 geometric 参数在 `dimensions.yaml` 和 `joint_config.yaml` 中分别注入，尚未通过 `observable` flag 系统化收集。v0 的 `symbolRegistry` 主要收集 `joint` 和 `task` 两类。

### 3.2 `joint` — 关节变量

模块实例的关节自由度变量。每个 joint 在 IR 展开时创建为独立的 `sym` 变量。

- **性质**：自由符号变量（求解时的已知量或未知量）
- **注册时机**：`localExpandInstance` 展开每个 joint 时（`Expander.m` L295-315）
- **注册条件**：`joint.observable != false`（默认 true）
- **示例**：`joint1.q`（revolute 角度）、`manipulator_L.dx`（prismatic 位移）

### 3.3 `task` — 任务参考系

标记为 `observable: true` 的 frame，通常是求解目标坐标系（FK 输出 / IK 目标）。

- **性质**：frame 的 4×4 符号位姿表达式 $T_{\text{frame}}(q)$，依赖于所有上游 joint 变量
- **注册时机**：`localExpandInstance` 展开 frames 时检测 `observable: true`
- **注册条件**：`frame.observable == true` 且 `frame.semantic_tag` 为 `'tool'` 或类似任务语义
- **示例**：`pipette.tip_origin`（ToolPipette 工具尖端）、`frame2.frame_hyper_cube`（末端 Frame 中心）

---

## 4. 收集规则（Expander 中实现）

### 4.1 收集时机

在 `localExpandInstance` 内部，展开 joints 和 frames 的循环中逐条收集。

### 4.2 joint 变量收集

```matlab
% 在 localExpandInstance 的 joints 展开循环中（L295-315 之后）
if core.CommonUtils.field(j, 'observable', true)
    entry = struct('name', symName, ...
                   'type', 'joint', ...
                   'symHandle', val, ...
                   'scope', 'instance', ...
                   'module_type', itype, ...
                   'instance', iname);
    obj.SymbolRegistry_(end+1) = entry;
end
```

### 4.3 task frame 收集

```matlab
% 在 localExpandInstance 的 frames 展开循环中（L245-270 之后）
if isfield(f, 'observable') && isequal(f.observable, true) && ...
   ~isempty(core.CommonUtils.field(f, 'semantic_tag', ''))
    % tasked frame — register for later TSym extraction
    entry = struct('name', node, ...
                   'type', 'task', ...
                   'symHandle', sym([]), ...  % placeholder; filled after FK propagation
                   'scope', 'instance', ...
                   'module_type', itype, ...
                   'instance', iname);
    obj.SymbolRegistry_(end+1) = entry;
end
```

### 4.4 FK 传播后填充

task frame 的 `symHandle` 在 FK 传播完成后从 `Poses` map 中填充：

```matlab
% 在 Expander 构造函数末尾，propagate() 之后
for i = 1:numel(obj.SymbolRegistry_)
    if strcmp(obj.SymbolRegistry_(i).type, 'task')
        frameName = obj.SymbolRegistry_(i).name;
        if isKey(obj.Poses, frameName)
            obj.SymbolRegistry_(i).symHandle = obj.Poses(frameName);
        end
    end
end
```

---

## 5. 使用场景

### 5.1 ExecutionConfig 交叉校验

`ExecutionConfig` 构造函数接收 `symbolRegistry` 并校验：
- `endFrame` 在 registry 中存在且 type=`'task'`
- `actuated_joints` 中的每个 ref 在 registry 中存在且 type=`'joint'`
- `known`/`unknown` 中的每个 ref 在 registry 中存在
- `closure_cuts` 中的每个 frame ref 在 registry 中存在

### 5.2 求解变量分区

`ExecutionConfig.partitionVariables()` 使用 `symbolRegistry` 将变量按 `known`/`unknown` 列表分组，返回 `knownVars` 和 `unknownVars` 两个 struct 数组。

### 5.3 求解器接口

A.5 求解器通过 `symbolRegistry` 确定：
- 哪些变量需要 `subs`（代入已知值）
- 哪些变量保留为 `fmincon` 优化变量（未知量）
- 哪个 frame 的位姿作为 FK 输出 / IK 目标（由 `endFrame` 定位）

---

## 6. 与 JointVarMap 的关系

| 维度 | JointVarMap | SymbolRegistry |
|------|------|------|
| 内容 | 仅 joint 变量 | joint + task + geometric |
| 键 | canonical name → sym handle | struct 数组（name + type + symHandle + metadata） |
| 用途 | FK 数值求值（subs） | 变量发现 + 分区 + 校验 |
| 冗余 | 无 | 包含 joint 的冗余副本（为 Schema 统一性） |

两个容器在 Expander 中同步构建，保持一致性但不互相替代：`JointVarMap` 服务于高效的 `subs` 查找，`SymbolRegistry` 服务于结构化的变量发现与配置校验。

---

## 7. 过关标准

- `open-chain-2r` 的 `symbolRegistry` 包含 `joint1.q`（type=joint）、`joint2.q`（type=joint），以及至少一个 task frame
- `symbolRegistry` 中无重复 `name`（唯一性）
- 所有 `joint` 条目的 `symHandle` 非空且在 `JointVarMap` 中存在对应键
- 所有 `task` 条目的 `symHandle` 在 FK 传播后非空且为 4×4 sym
