# 文档精简分析

> 本文档从两个维度分析项目文档的精简空间：
> - **跨文档重叠**（§1–§6）：多份文档之间的内容重复
> - **单文档冗余**（§7）：每份文档自身表达臃肿之处
>
> 与 `docs/README.md`（文档地图）配合使用：地图告诉你文档间的关系，本文档告诉你哪里可以砍。
>
> **状态**：v1，已覆盖全部主要文档的跨文档 + 单文档分析。

---

## 1. `modeling-conventions.md` ↔ `conventions.yaml`

**重叠本质**：同一套建模约定（4 层架构、元件类型、坐标系/单位、端口极性、连接规则、参数作用域）分别在 prose（~600 行）和 YAML（~100 行）中表达。**这是有意为之的双轨制**，不应合并，但需显式同步机制。

**精简策略**：在双方文件头部显式标注对应关系与同步日期。`conventions.yaml` 中每项加 `see: modeling-conventions.md §X` 注释；`modeling-conventions.md` 中每节加 `→ conventions.yaml <key>` 标注。优先级：**中**。

---

## 2. `modeling-conventions.md` §5（核心模块库一览）↔ `specs/modules/*.yaml`

**重叠本质**：`modeling-conventions.md` §5 用 prose 摘要了 6 个模块的端口名、偏移值、拓扑链、关节变量，而这些信息在模块 YAML 中是**权威数据**。prose 摘要不仅冗余，而且模块 YAML 更新后容易忘记同步 prose。

**精简策略**：`modeling-conventions.md` §5 退化为「模块清单 + 指路」表格（模块名/类别/DOF/一句话说明 + 链接到 YAML），删除端口偏移值、拓扑链等已在 YAML 中定义的细节。优先级：**高**。

---

## 3. `grammar.md` ↔ `mechanism-assembly.schema.yaml`

**重叠本质**：`grammar.md` 用 prose + 表格定义 DSL 的合法字段、类型、正则、必填/可选规则；`mechanism-assembly.schema.yaml` 用 JSON Schema 表达相同约束。两者字段列表高度重复。grammar.md 也有 Schema 无法覆盖的信息（如 §1.2「DSL 不是什么」的设计依据），但字段表部分纯属抄写。

**精简策略**：`grammar.md` 保留定位说明（§1）、设计依据（§1.2）、prose 独有的语义说明（如 §5.3 顺序无关的原因、§5.6 连接不能做的事）；字段表改为引用 Schema 的 `required`/`properties` 而不逐字抄写。优先级：**中**。

---

## 4. `connection-semantics.md`（DSL 层）↔ `port-attachment.md`（IR 层）

**重叠本质**：同一标准 mate 变换公式 $T = R_z(\theta) \cdot R_x(\pi), t=0$ 在两份文档中各自独立定义。极性门控规则（仅 `socket↔plug`）在两份文档中均有完整描述。`port-attachment.md` §1 已声明与 `connection-semantics.md` 的互补关系，但并未真正引用后者的公式定义——而是各自复制了一份。

**精简策略**：`port-attachment.md` 应**只写「IR 中如何表示」**：边类型区分（addMateBidirectional/addMateUnidirectional）、exportEdges 过滤、诊断用途。公式和极性规则直接引用 `connection-semantics.md` 对应节号，不再重复展开。优先级：**中**。

---

## 5. `ARCHITECTURE.md` 运动学概念节 ↔ IR 四份文档

**重叠本质**：`ARCHITECTURE.md` 的「运动学核心概念 —— DSL 可视化运作原理」节（约 150 行）展开讲解了 mate 变换公式、生成树/弦边策略、`kind` 过滤机制、FK 传播算法细节、mate gap/Zdot 诊断。这些内容在 `edge-types.md`（边类型）、`port-attachment.md`（mate 边表示）、`dsl-to-ir-mapping.md`（映射管线）中也有详细展开，且 IR spec 是权威来源。

**精简策略**：`ARCHITECTURE.md` 的该节应精简为「概念概述 + 指路到 IR spec」——每个概念给 2-3 句概述后直接链接到对应的 IR 文档节号。删除已在 IR spec 中详细展开的公式、代码片段、诊断公式。保留 `ARCHITECTURE.md` 独有的内容：设计决策（为什么 exportEdges 剥离元数据、为什么 PosePropagator 保持独立、为什么 ClosureSolver 独立于 Expander）。优先级：**高**。

---

## 6. 四份 IR 文档的概述节

**重叠本质**：`node-types.md`、`edge-types.md`、`dsl-to-ir-mapping.md`、`port-attachment.md` 每份的开头都有类似的「本文档定义…」「权威代码来源…」「状态：A.3.1 v0」模板，以及关于 IR 是什么的简介。`dsl-to-ir-mapping.md` §1 的总体管线图与 `ARCHITECTURE.md` 的 Expander 内部管线图也有部分重叠。

**精简策略**：各 IR 文档保留状态行与代码来源声明；删除重复的「IR 是什么」简介段落（改为一句引用 `node-types.md` §1 或文档地图）。`dsl-to-ir-mapping.md` §1 的管线图与 `ARCHITECTURE.md` 的管线图应统一为一个权威版本（建议放在 `dsl-to-ir-mapping.md`，`ARCHITECTURE.md` 引用）。优先级：**低**。

---

## 优先级汇总

| 优先级 | 编号 | 涉及文件 | 精简方向 |
|------|------|------|------|
| 🔴 高 | #2 | `modeling-conventions.md` §5 ↔ 模块 YAML | prose 摘要退化为清单，数值只存 YAML |
| 🔴 高 | #5 | `ARCHITECTURE.md` 概念节 ↔ IR spec | 概念节精简为概述 + 指路，删除重复公式/代码 |
| 🟡 中 | #1 | `modeling-conventions.md` ↔ `conventions.yaml` | 双向显式标注对应关系 + 同步日期 |
| 🟡 中 | #3 | `grammar.md` ↔ `mechanism-assembly.schema.yaml` | 字段表改为引用 Schema |
| 🟡 中 | #4 | `connection-semantics.md` ↔ `port-attachment.md` | IR 层只写表示，公式/规则引用 DSL 层 |
| 🟢 低 | #6 | 四份 IR 文档概述节 | 删除重复简介，统一管线图位置 |

---

## 7. 单文档冗余精简

> 以下逐文档列出**自身内容臃肿**之处（与跨文档重叠 #1–#6 正交）。精简目标：
> 不丢失信息，仅压缩表达——用表格替代段落、用引用替代抄写、用图示替代叙事。
> 按 `docs/README.md` 层级顺序排列。

### L1 · `docs/project-overview.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| 「什么是 DSL」节 | 约 30 行，用叙事体解释 DSL 概念，面向完全不了解 DSL 的读者 | 压缩为 5 行定义 + 1 个 Mermaid 分层图（DSL → IR → Solver 关系），细节指路 `grammar.md` |
| 「什么是 IR」节 | 约 25 行，同上问题 | 压缩为 3 行定义 + 「详见 `specs/ir/`」指路 |
| A.0–A.7 各阶段任务描述 | 每阶段约 30–80 行 prose，产出表格、过关标准、具体任务混杂在一起 | 每阶段统一为「目标 1 行 + 产出表格 + 过关标准 3 条」三段式，删除叙事性过渡句 |
| A.2.5 DSL 装配可视化 | 约 70 行，描述 `visualize_mechanism.m` 的功能细节，大量重复 `ARCHITECTURE.md` | 精简为 10 行摘要 + 指路 `ARCHITECTURE.md` 和 `scripts/matlab/README.md` |
| 「阶段退出条件」 | 7 条退出条件 + 4 条排查顺序，部分条件在阶段正文中已隐含 | 保留退出条件清单，删除排查顺序（属于操作手册，不属于规划文档） |
| 全文 | 大量「详见 X.md」交叉引用在后续阶段中反复出现 | 每份被引用文档只在首次出现时写完整路径，后续用缩略名 |

### L2 · `specs/modeling-conventions.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §3 元件类型表 + 逐项展开 | 先用一个大表列出 5 种元件，然后 §3.1–§3.5 逐项用属性表 + prose 重述一遍。表格已经包含了 80% 的信息 | 保留 §3 总表（含属性列），删除 §3.1–§3.5 的属性表（与总表重复）；每项只留 2-3 句关键语义说明 |
| §3.6 可观测变量管线 | Mermaid 流程图 + 长段落解释 + 与 Simulink 对应表，三遍讲同一件事 | 保留流程图 + 对应表，删除中间的长段落。管线行为从图中已可读 |
| §9.4 轴对齐规范化规则 | 4 步规则用 prose 列表写了一遍，又在 §5.1 Frame 模块端口表中用 `alignA/alignB` 列再写一遍 | §9.4 保留规则（这是权威出处），§5.1 端口表删除 `alignA/alignB` 列，改为「见 §9.4」 |
| 全文 | 「类比 Simulink/Simscape/URDF/OOP 类」的类比标注出现约 15 次 | 保留 §2 层级架构表中的类比列（集中一处），其余各处删除类比标注 |

### L4 · `specs/dsl/grammar.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §1.2「DSL 不是什么」 | 7 行表格，每行附带「原因」列解释设计依据 | 保留表格，删除「原因」列（设计依据已隐含在 §1.1 定位中） |
| §1.3「DSL 只做三件事」 | 3 条 bullet 重述 §1.1–§1.2 已表达的内容 | 删除整节，信息已在 §1.1–§1.2 中 |
| §5.2–§5.8 连接细节 | 7 个子节，部分内容（极性门控、roll 传播性）属于 `connection-semantics.md` | 语法层只保留字段定义（§5.1–§5.4），语义细节（§5.5–§5.8）移入 `connection-semantics.md` 或精简为引用 |
| §8「实现者须知」 | 4 条 bullet 重复 `dsl-to-ir-mapping.md` 和 `ARCHITECTURE.md` 中的信息 | 删除整节，改为「实现细节见 `specs/ir/dsl-to-ir-mapping.md`」 |

### L4 · `specs/dsl/connection-semantics.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §6.2「closed 标记的解释器行为」 | 约 100 行，包含残差公式推导、A.4 约束构造、可视化行为、代码片段。是全文最臃肿的单节 | 拆为三部分：(1) 核心语义 5 行（`closed` = chord，不参与 FK）；(2) 残差公式 + 变量表（保留，约 15 行）；(3) 下游行为指路到 A.4/A.5 对应文档。删除代码片段 |
| §6.4「L2 闭环 vs L3 世界系闭环」 | 约 80 行，含大量 prose 对比 | 用一张对比表格（已在 §6.4.2）+ 一张 Mermaid 拓扑图替代 prose。§6.4.3 可视化表现并入 ARCHITECTURE.md |
| §7「与手工推导对照」 | 验证用备注，不属于连接语义规范 | 移入 `tests/README.md` 或 DSL 案例的 README |

### L4 · `specs/dsl/case-conventions.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| 「Mermaid 语法要点」节 | 嵌入完整 YAML 模板示例（约 25 行），实际使用时直接参考案例即可 | 模板保留结构骨架（10 行），删除 `subgraph` 内的注释性节点，改为文字说明「参考 open-chain-2r 案例」 |

### L5 · `specs/ir/node-types.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §3.4「Root 自动注册」+ 工具端生长范式 | 约 40 行 prose 解释 root node 注册逻辑和 tool-rooted growth 概念 | 保留注册规则（5 行）和代码片段；工具端生长范式改为 1 个 Mermaid 图 + 3 句说明 |
| 底部「代码对照表」 | 17 行的逐行对照表，信息已在上方正文中表达 | 保留对照表（作为速查），但精简为单列表（规范条目 → 代码行号），删除第三列（已在正文） |

### L5 · `specs/ir/edge-types.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §2–§5 每种边类型 | 每种边都有「插入规则代码片段 + 变换公式 + 用途说明」三段，总计约 120 行 | 保留公式（数学定义是权威），删除代码片段（代码是源头，文档不需要抄写）；用途说明压缩为 1 行 |
| 底部「代码对照表」 | 19 行，同 node-types.md 问题 | 同 node-types.md 策略 |

### L5 · `specs/ir/dsl-to-ir-mapping.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §1 总体管线 Mermaid 图 + §9 完整数据流 Mermaid 图 | 两个图高度相似（都是一条 DSL→IR 的流水线），§9 只是比 §1 多了细节 | 合并为一个图（放在 §1），增加细节标注；§9 删除图，改为文字摘要 |
| §2–§8 每步的代码片段 | 每步都嵌入了来自 `Expander.m` 的 MATLAB 代码（约 80 行总计） | 代码片段改为行号引用（如 `Expander.m L52-56`），仅保留关键逻辑的伪代码（约 20 行总计）。文档读者不需要逐行读 MATLAB |
| 底部「代码对照表」 | 26 行，同上 | 同 node-types.md 策略 |

### L5 · `specs/ir/port-attachment.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| §2「标准 mate 变换（IR 实现）」 | 公式 + 代码片段 + 矩阵形式 + 分量分解，四遍讲同一件事 | 公式引用 `connection-semantics.md`，只保留「IR 实现差异」：addMateBidirectional 双向 vs addMateUnidirectional 单向 |
| §7「诊断：mate gap 与 Zdot」 | 代码片段 + 公式 + 解读 | 保留公式和理想值表；删除代码片段（已在 ARCHITECTURE.md 中） |

### L7 · `scripts/matlab/ARCHITECTURE.md`

| 位置 | 问题 | 策略 |
|------|------|------|
| 「运动学核心概念」节（约 150 行） | 见跨文档 #5。此外内部还有 3 个 Mermaid 图（总览、Expander 内部管线、数值化流程） | 保留 1 个总览图；管线细节图移入 `dsl-to-ir-mapping.md`（权威出处） |
| 「关键设计决策」§3 Mate 约定 | mate 公式 + 解释，已在 `connection-semantics.md` 中定义 | 删除公式，只保留「为什么这样设计」的决策理由 |
| 「关键设计决策」§4 多根支持 | 3 行说明，信息量低 | 已移除：EdgeGraph 改为单根语义，该节直接删除 |
| 「关键设计决策」§5「为什么 Expander 独立于 +viz」 | 对比表 + 4 段 prose，讲同一件事 | 保留对比表，删除 prose |

### L8 · 参考文档

| 文件 | 问题 | 策略 |
|------|------|------|
| `docs/reference/pathplanner-architecture.md` | 旧系统文档，内容详尽但与当前项目无直接技术依赖 | **不精简**。标注为「历史参考，只读」，在文档地图中降低其权重 |
| `docs/reference/urdf_module_reference/m-rex-urdf-analysis.md` | 详细分析文档，作为对照参考有价值 | **暂不精简**。待模块 YAML 全部冻结后再评估是否需要保留全文 |
| `docs/survey/robot-description-language-survey.md` | 调研报告，背景材料 | **不精简**。调研类文档保持完整以备回溯 |

### 其他

| 文件 | 问题 | 策略 |
|------|------|------|
| `/README.md`（根目录） | 描述不存在的目录结构（`cases/`、`architecture/`、`design/`），与实际情况严重脱节 | 重写为 10 行电梯 pitch：项目名 + 一句话定位 + 指向 `docs/README.md` 的链接 |
| `specs/README.md` | 同样描述不存在的子目录（`solver-contracts/` 等） | 重写为 5 行索引：指向 `docs/README.md`（文档地图）+ 本目录内容一句话概述 |

---

## 精简工作量估算

| 文档 | 当前估算行数 | 目标行数 | 主要手段 |
|------|------|------|------|
| `project-overview.md` | ~500 | ~300 | 阶段描述表格化、删除叙事过渡句、DSL/IR 概念压缩 |
| `modeling-conventions.md` | ~600 | ~350 | §3 删逐项属性表、§5 退化为清单、删类比标注 |
| `grammar.md` | ~200 | ~120 | §1.2 删原因列、§1.3 删除、§8 删除、§5 精简 |
| `connection-semantics.md` | ~300 | ~150 | §6.2 拆分 + 指路、§6.4 表格化、§7 移出 |
| `node-types.md` | ~150 | ~100 | 范式 prose 图示化、代码对照表精简 |
| `edge-types.md` | ~150 | ~90 | 删代码片段、用途说明压缩 |
| `dsl-to-ir-mapping.md` | ~200 | ~120 | 删重复 Mermaid 图、代码片段改行号引用 |
| `port-attachment.md` | ~120 | ~70 | 公式引用 DSL 层、删代码片段 |
| `ARCHITECTURE.md` | ~400 | ~250 | 概念节精简为概述 + 指路、删重复公式 |
| `/README.md` | ~20 | ~10 | 重写 |
| `specs/README.md` | ~8 | ~5 | 重写 |
| **合计** | **~2650** | **~1565** | **缩减约 40%** |
