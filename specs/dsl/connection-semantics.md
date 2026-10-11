# 连接语义（DSL 层）（阶段 A.2.1）

> 本文档定义端口连接在 DSL 层的精确语义：标准 mate 变换在 YAML 中如何表达、`roll` 参数、
> `closed` 标记的解释器行为，以及连接不能承载的语义。
> 权威几何约定见 `../modeling-conventions.md` §9（端口）、§10（连接）；机器可读常量见 `../conventions.yaml`。
> 语法规则见 `grammar.md`。
>
> **状态**：A.2 v0。mate 变换公式为 A.0 冻结项，本文档不重新定义，仅说明其 DSL 层表达。

---

## 1. 连接的唯一含义

一条 DSL 连接的含义**唯一**：把两个端口**面对面对插贴合**。连接本身**不携带任意空间变换、
不引入运行时自由度**。所有连接产生的几何都由下面的标准 mate 变换给出——用户在 DSL 中只声明
「哪两个端口相连」，几何由解释器统一施加。

---

## 2. 标准 mate 变换（冻结）

### 2.1 公式

设连接的 `socket` 端为父、`plug` 端为子，二者局部 `+Z` 均朝模块外。子端口坐标系相对父端口坐标系的位姿：

$$T_{\text{plug} \leftarrow \text{socket}} = R_z\!\left(\text{roll} \cdot \tfrac{360^\circ}{\text{symmetry}}\right) \cdot R_x(\pi), \qquad t = 0$$

- **`Rx(π)`（冻结）**：绕端口 `+X` 翻转 180°。把 `+Z` 翻成反平行（两法向面对面）、`+Y` 翻转、`+X` 不变。
  这实现「面对面对插」而非「坐标系数值相等」。
- **`Rz(...)`**：绕公共对插法向的离散滚转（§3）。
- **平移为零**：两对接面在原点相触；任何法向间隙属于模块几何，不属于连接。

### 2.2 DSL 层如何触发

DSL 连接**顺序无关**（`grammar.md` §5.3）：

```yaml
connections:
  - ports: [frame0.faceXPlus, joint1.linkA]
```

解释器处理流程：

1. 读取两端口的 `polarity`（来自各自模块定义）。
2. 校验极性互补：仅 `socket↔plug` 合法（§4）。
3. **自动定向**：把 `socket` 端（如 `frame0.faceXPlus`）取为父，`plug` 端（如 `joint1.linkA`）取为子。
4. 施加 §2.1 的 mate 变换，在 IR 中插入一条桥接边，把两端口 frame 面对面重合。

因此 `[frame0.faceXPlus, joint1.linkA]` 与 `[joint1.linkA, frame0.faceXPlus]` 完全等价——父/子由极性决定，
不由书写顺序决定。

### 2.3 socket 提供者

当前模块库中，`socket` 端口仅由两类模块提供（`modeling-conventions.md` §9.5）：

| socket 提供者 | 端口 |
|------|------|
| `Frame` | 六面 `faceX±`、`faceY±`、`faceZ±` |
| `Manipulator` | `dock` |

其余一切机械对接面（`Pin.sideA/B`、`Joint.linkA/B`、`Adaptor.attachment_point/pin_connector`、
`ToolPipette.connector_side`）均为 `plug`。故合法连接必有一端来自上表。

---

## 3. roll 参数

### 3.1 语义

- `roll` 是**连接级**整数字段（缺省 0），归属连接而非端口。
- 实际滚转角 = `roll × 360 / symmetry` 度，绕公共对插法向（`+Z`）。
- `symmetry` 为端口的旋转对称阶（缺省 4，方钢片 C4）；合法 `roll` 取值 `0 .. symmetry-1`。
- `symmetry = 4` 时，`roll ∈ {0, 1, 2, 3}` 对应 `0° / 90° / 180° / 270°`。

### 3.2 DSL 表达

```yaml
connections:
  - ports: [frame1.faceXPlus, joint2.linkA]
    roll: 1        # 装配时绕对插法向额外转 90°（symmetry=4）
```

### 3.3 roll 的可观测性

- 对**结构面**（`Frame`/`Pin`）：`roll` 在对称群内无差别，纯装配记号，不改变可求解几何。
- 对 **`Joint` 端口**：端口背后藏有打破对称的关节轴，同一个合法 `roll` 会**同步转动关节轴**——
  这正是无需「扭 90° 垫片模块」即可改变关节轴朝向的机理。

### 3.4 roll 只离散

`roll` 永远落在对称群内，机械上必然装得上。**真正需要运行时连续绕法向转动**的场合，
不用 `roll`，而是插入一个 revolute `Joint` 模块（轴取公共法向）——那是真自由度（§5）。

### 3.5 roll 的传播性与 Joint 配对规则

**关键行为**：`roll` 作用于一条连接上，会**永久旋转连接下游所有模块的朝向**。
这是因为 `roll` 通过 mate 变换 $R_z(\text{roll} \cdot 360^\circ / \text{symmetry})$ 旋转了子端口所属
刚体的整体系，该刚体上所有其他端口（包括出向端口）的坐标系也随之旋转。
后续连接均以此旋转后的坐标系为参考，因此 `roll` 的影响**沿连接链向下游传播，永不自动撤销**。

**Joint 模块的配对规则**：Joint 模块拥有两个端口（`linkA` 和 `linkB`），分别连接上游和下游模块。
若仅在一侧设置 `roll`（如 `roll: 1`），则不仅旋转了关节轴方向，还会使**下游所有模块的朝向永久偏移**。
为了**仅旋转关节轴而不影响下游**，必须在 Joint 两端**成对使用 `roll`**：

```yaml
# 正确：roll 成对出现，关节轴旋转 90° 对齐 X 轴，下游不受影响
connections:
  - ports: [frame_upstream.faceZPlus, joint.linkA]
    roll: 1          # 旋转关节体，对齐关节轴
  - ports: [joint.linkB, frame_downstream.faceZMinus]
    roll: -1         # 反向旋转，恢复下游朝向
```

**原理**：第一条连接（`roll: 1`）将 Joint 刚体绕对插法向旋转 90°，关节轴随之转动；
第二条连接（`roll: -1`）在 Joint 的出向端口处以等量反向旋转抵消，
使得下游 `frame_downstream` 的最终朝向与未设置 `roll` 时一致。
**净效果**：仅关节轴方向改变，下游模块朝向保持不变。

**何时需要配对**：
- 当 Joint 的默认旋转轴方向与机构运动平面不平行时（如默认轴沿 Y，而平行四边形变形沿 XZ 平面），
  需要在 Joint 两端使用 `roll: 1` / `roll: -1` 配对将轴旋转到正确方向。
- 若希望 `roll` 的旋转效果有意传播到下游（如整条支链需要统一旋转朝向），则仅在入口侧设置 `roll`，
  不在出口侧设反向 `roll`。

**对非 Joint 模块**（Frame、Pin 等）：`roll` 的传播行为同样存在，但由于这些模块的端口在对称群内
不可区分（`symmetry` 对称性），`roll` 的实际几何效果可能不可观测（§3.3）。

---

## 4. 极性门控

- 连接仅允许 `socket↔plug`。`socket↔socket`、`plug↔plug`、以及任一端无极性（任务系如
  `ToolPipette.tip_origin`、接地系如 `Manipulator.ground`）均由解释器直接判非法。
- 极性只是**逻辑门控**，决定连接是否被允许，不产生几何；连接几何仍由 §2 的 mate 变换给出。
- 每个端口**至多被占用一次**；重复占用为非法。

---

## 5. 连接不能承载的语义

连接不携带任意变换或自由度。以下需求**必须插入模块**，不得赋予连接新语义：

### 5.1 坐标适配 → 插入 `Adaptor`

若两端口面对插后朝向/位置不满足需求，需要额外的固定平移或旋转，则在两端口间插入 `Adaptor` 模块，
由其内部 `fixedTransform` 承载所需重定向，两侧各连接一次：

```yaml
# 错误：试图让连接自带一个 45° 旋转 —— DSL 无此语法，也不允许
# 正确：插入 Adaptor
connections:
  - ports: [manip.dock, adaptor.attachment_point]
  - ports: [adaptor.pin_connector, frameBase.faceZPlus]
```

### 5.2 连续绕法向旋转 → 插入 revolute `Joint`

若需运行时绕对插法向连续转动（真自由度），插入 revolute `Joint` 模块，关节轴取公共法向 `+Z`。
该自由度由 `Joint` 元件显式描述，与 §3 的离散 `roll`（装配朝向）语义分离。

---

## 6. closed 标记的解释器行为

### 6.1 语义

- `closed: true` 声明该连接是**闭环补边（chord）**：它在 L2 机构图中闭合了一个回路。
- 缺省 `false`（树边）。

### 6.2 解释器如何处理

被 `closed: true` 标记的连接，在 IR 中**照常施加 §2 的 mate 变换**，但**不参与 FK 位姿传播**——它作为单向诊断边（`kind: closed_mate`，见 `port-attachment.md`）存在于图中。A.4 阶段在此切开回路，生成 `Constraint`。

**残差公式**（与 L3 世界系闭环共用）：

$$T_{\text{residual}} = T_{\text{far}}^{-1} \cdot T_{\text{near}}$$

| 符号 | 含义 |
|------|------|
| $T_{\text{near}}$ | `F_near` 在 `world` 下的位姿（沿树边侧传播） |
| $T_{\text{far}}$ | `F_far` 在 `world` 下的位姿（沿补边侧传播） |
| $T_{\text{residual}}$ | `F_near` 相对 `F_far` 的位姿；环路闭合时 $= I$ |

6 个候选残差分量 $[t_x, t_y, t_z, r_x, r_y, r_z]$（平移 mm，姿态 Z-Y-X 欧拉角 rad）。具体约束哪些分量子集由 L3 `constrained_components` 决定（见 A.4 设计）。

> `closed` 在下游管线（IR 展开 → 校验 → FK → 回路识别 → 约束构造 → 求解）的完整行为表见 `port-attachment.md`。可视化表现见 `ARCHITECTURE.md`。

### 6.3 多闭环

- 每个 `closed: true` 连接对应**一个独立回路切口**。
- N 个独立回路 → N 个 `closed: true` 连接 → N 个 `Constraint`。
- 对「M 条支链连接同两个刚体」的并联结构，独立回路数 = `M − 1`，故其中 `M − 1` 条支链的闭合连接
  标 `closed: true`（选哪条支链作补边不影响物理等价性，仅影响切口位置）。

### 6.4 L2 闭环 vs L3 世界系闭环

`closed` **只标记在 L2 机构图内部即闭合的回路**。若回路仅在 L3 绑定 `world` 与外部驱动后才闭合（M-REx 主构型），则不写 `closed`——由 L3 execution-config 的 `closure_cuts` 声明。

| | L2 内部闭环（四杆环） | L3 世界系闭环（M-REx） |
|---|---|---|
| 闭合位置 | 两个模块端口之间 | 多个 `ground` frame 各带静态标定偏移绑到同一 `world` 参考系 |
| DSL `closed` | **是** | **否**（机构本体是开环链） |
| 切口声明位置 | DSL 连接级 `closed: true` | L3 execution-config `closure_cuts` |
| 残差公式 | $T_{\text{residual}} = T_{\text{far}}^{-1} \cdot T_{\text{near}}$ | 同左 |
| 可视化表现 | 补边 mate 线（橙色虚线） | 末端（`tip_origin`）在 `world` 原点，各 `ground` frame 在标定偏移位置（详见 `ARCHITECTURE.md`） |

**L3 切口声明（`anchor: world`，A.4b 落地）**：L3 世界系闭环的 `closure_cuts` 以
`anchor: world` 声明——`near` = 被绑定的 `ground` frame；far 侧不引用图内 frame，而是取
该 ground 的 `world_binding` 标定位姿 $T_{\text{bind}}$（残差中等效 mate 变换 $M = I_4$、
far 为常量）。求解器实现（`solver.ClosureSolver`）的残差为：

$$T_{\text{residual},i} = T_{\text{near},i}(q)^{-1} \cdot (\Delta T^{-1} \cdot T_{\text{bind},i})$$

- $T_{\text{bind},i}$：零位构型标定值——`world_binding.T` 显式给出，或省略后由求解器
  在全部关节零位下自动计算（显式值偏离自动标定 > 1e-6 时报错）。
- $\Delta T$：tip 任务目标（`setTipTarget`，默认 $I_4$ —— 纯闭环/零位验证）。
- 实例：`cases/m-rex-3t1r/execution-config.yaml`；端到端验证：
  `tests/pipeline/test_closure_solver_m_rex_3t1r.m`。

---

## 7. 验证参照

手工推导验证项见各 DSL 案例 README 与 `tests/` 目录。
