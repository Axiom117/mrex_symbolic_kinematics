# SLX 模块库拆解摘要

本目录中 `mrex_hypercube_modules.slx` 的关键结构如下：

- `simulink/blockdiagram.xml`：库级元信息
- `simulink/systems/system_root.xml`：顶层模块清单
- `simulink/systems/system_*.xml`：各模块内部定义

这意味着该 `slx` 可以作为符号化模块语言的参考源。当前能够稳定提取的信息包括：

- 模块名称
- 外部物理端口名称
- 内部块类型
- 几何资源文件名
- 刚体变换的平移和轴对齐规则
- 关节类型
- 直接写在 XML 中的符号参数名

当前不应把它当作完整真相来源的信息包括：

- Mask 初始化脚本和回调的全部语义
- Simulink GUI 中隐含但未直接体现在 XML 中的建模意图
- STEP 几何本身的高层工程语义

## 顶层库模块

从 `system_root.xml` 可读到 5 个顶层库模块：

| 模块名 | 子系统文件 | 外部端口 | 主要几何资源 | 主要内部结构 |
|------|------|------|------|------|
| `Frame` | `system_1.xml` | `faceXPlus`, `faceXMinus`, `faceYPlus`, `faceYMinus`, `faceZPlus`, `faceZMinus` | `assets/frame_hyper_cube.STEP` | 1 个主体 + 6 个面参考框架 |
| `Pin` | `system_15.xml` | `sideA`, `sideB` | `assets/connector_dowel_pin.step` | 1 个主体 + 2 个连接侧 |
| `Joint` | `system_21.xml` | `linkA`, `linkB` | `assets/linkage_hinge_joint.STEP` | 2 个主体 + 1 个转动副 |
| `ToolPipette` | `system_32.xml` | `connector_side`, `tip_origin` | `assets/tool.STEP` | 1 个主体 + 连接端 + 针尖原点 |
| `Adaptor`（DSL: `Adaptor_45`） | `system_39.xml` | `attachment_point`, `dock` | `assets/adaptor_45.STEP` | 1 个主体 + 2 级对齐框架 |

## 各模块可抽取定义

### 1. Frame

`Frame` 是最适合直接映射到 DSL 原语的模块。其主体是一个立方体实体，6 个端口都通过刚体变换从主体派生。

可直接抽取的字段：

- 几何资源：`assets/frame_hyper_cube.STEP`
- 参数：`cubeLength`
- 端口：`faceXPlus`, `faceXMinus`, `faceYPlus`, `faceYMinus`, `faceZPlus`, `faceZMinus`

各端口参考框架可从内部刚体变换直接读取：

| 端口 | 平移偏移 | 轴对齐 A | 轴对齐 B |
|------|------|------|------|
| `faceXPlus` | `[cubeLength/2, 0, 0] mm` | `+X -> +Y` | `+Y -> +Z` |
| `faceXMinus` | `[-cubeLength/2, 0, 0] mm` | `+X -> +Y` | `+Y -> +Z` |
| `faceYPlus` | `[0, cubeLength/2, 0] mm` | `+X -> +Y` | `+Y -> +Z` |
| `faceYMinus` | `[0, -cubeLength/2, 0] mm` | `+X -> +Y` | `+Y -> +Z` |
| `faceZPlus` | `[0, 0, cubeLength/2] mm` | `+Y -> -Z` | `+Z -> +X` |
| `faceZMinus` | `[0, 0, -cubeLength/2] mm` | `+Z -> +X` | `+Y -> +Z` |

内部拓扑可归纳为：

`body -> 6 个参考框架 -> 6 个外部面端口`

这说明 `Frame` 在 DSL 中应该被建模为：

- 一个刚体模块
- 一组命名连接面
- 每个连接面带有局部参考坐标系

### 2. Pin

`Pin` 由一个实体和两个连接侧组成。

可直接抽取的字段：

- 几何资源：`assets/connector_dowel_pin.step`
- 端口：`sideA`, `sideB`
- 两个内部刚体变换均以零平移为基准，只负责重新定向参考系

内部拓扑可归纳为：

`body -> transform(sideA) -> sideA`

`body -> transform(sideB) -> sideB`

这说明 `Pin` 更像一个双端连接件，而不是带自由度的运动副。

### 3. Joint

`Joint` 是最关键的运动学模块。其内部明确包含一个 Simscape `Revolute Joint`。

可直接抽取的字段：

- 几何资源：两个 `assets/linkage_hinge_joint.STEP`
- 端口：`linkA`, `linkB`
- 关节类型：`Revolute Joint`
- 两侧变换的平移偏移：`[0, -5, 0] mm`
- 两侧变换的轴对齐：`+X -> +Y`, `+Y -> +Z`

从连线关系可归纳为：

- `linkA` 连接到左侧实体参考框架
- `linkB` 连接到右侧实体参考框架
- 两侧实体分别通过刚体变换接入中间的 `Revolute Joint`

这说明 `Joint` 在 DSL 中不应只被视作一个形状块，而应拆成：

- 两个可连接的刚体侧
- 一个内部 1-DOF 转动副
- 两个外部参考框架到关节轴的固定位姿映射

### 4. ToolPipette

`ToolPipette` 是一个单主体工具模块，包含一个连接侧和一个工具尖端参考点。

可直接抽取的字段：

- 几何资源：`assets/tool.STEP`
- 参数：`tipDistance`
- 端口：`connector_side`, `tip_origin`
- `connector_side` 偏移：`[0, 5, 25] mm`
- `tip_origin` 偏移：`[0, 0, -tipDistance] mm`

内部拓扑可归纳为：

`body -> connector transform -> connector_side`

`body -> tip transform -> tip_origin`

这说明该模块适合在 DSL 中分成：

- 结构连接参考面
- 功能参考点或工具坐标原点

### 5. Adaptor（DSL: `Adaptor_45`）

`Adaptor`（本项目 L1 模块 `Adaptor_45`）由一个主体和两个不同用途的外部参考点组成。

可直接抽取的字段：

- 几何资源：`assets/adaptor_45.STEP`
- 端口：`attachment_point`, `dock`
- 内部存在三个定向用刚体变换：`Rigid Transform`、`align_axis`、`rectify`

从连线关系可归纳为：

- 主体直接分出一条支路到 `dock`
- 另一条支路经过 `rectify -> align_axis` 后到 `attachment_point`

这说明 `Adaptor_45` 在 DSL 中更像一个坐标适配模块，而不是纯连接块。

## 对符号化语言最有价值的抽象

从当前 `slx` 可以提炼出一套稳定的模块描述字段：

```yaml
module:
  name: Frame
  category: rigid_body | connector | joint | tool | adaptor
  geometry: assets/frame_hyper_cube.STEP
  parameters:
    - cubeLength
  ports:
    - name: faceXPlus
      role: connector
      transform:
        translation: [cubeLength/2, 0, 0]
        units: mm
        alignA: [+X, +Y]
        alignB: [+Y, +Z]
  internals:
    - type: solid
    - type: rigid_transform
  topology:
    - [body, faceXPlus_frame, faceXPlus]
```

如果后续要从拓扑编码自动生成运动学模型，建议 DSL 至少保留以下层次：

- 模块类别：刚体模块、连接件、关节模块、工具模块、适配器模块
- 外部端口：名称、端口角色、局部参考系
- 内部运动学原语：`rigid_transform`、`revolute_joint`、`prismatic_joint`
- 参数表达式：允许 `cubeLength`、`tipDistance` 这类未求值符号直接留在模型中
- 模块内部拓扑：用有向链或小图显式表示端口如何连接到内部参考框架和关节

## 当前结论

`mrex_hypercube_modules.slx` 已经足够作为第一版符号化模块语言的结构参考，因为它明确暴露了：

- 哪些对象是模块
- 每个模块有哪些外部连接口
- 外部连接口相对主体的局部坐标变换
- 哪些模块内部含有关节
- 哪些几何尺寸已经以符号参数形式存在

它还不够直接替代 DSL 设计的部分，在于模块语义仍偏 Simscape 实现视角。因此后续建模时，最好把 `slx` 抽取结果再归一化成“模块定义表 + 端口框架表 + 内部运动学图”三层文本表示。