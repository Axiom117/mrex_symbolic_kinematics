　# scripts/matlab — 模块定义可视化校验

> 用 MATLAB 读取 `specs/modules/*.yaml` 模块定义，注入参数后构建内部坐标系图并可视化，
> 用于**验证每个模块的文本定义是否能成功生成对应的模块描述**（端口/本体坐标系 + body 几何）。

## 文件

| 文件 | 作用 |
|------|------|
| `+core/readYaml.m` | 极简 YAML 子集解析器（已对照 PyYAML 验证，逐文件一致） |
| `+core/PosePropagator.m` | FK 传播引擎 + 关节变换矩阵（全部 static，纯计算层） |
| `+core/RigidBodyMath.m` | 3D 刚体变换与旋转数学原语 |
| `+core/VizHelpers.m` | 可视化辅助（triad、STEP 几何导入、颜色映射） |
| `+core/CommonUtils.m` | 通用工具（字段读取、YAML 列表展开、表达式求值） |
| `+core/PathUtils.m` | 路径解析工具 |
| `+ir/EdgeGraph.m` | 位姿图 IR 累积器（handle class）：管理边的增删、元数据、ground nodes，桥接 PosePropagator |
| `+viz/module.m` | 解析单个模块 → 求解坐标系位姿 → 画三元轴 + 优先加载 body STEP 几何 |
| `+viz/allModules.m` | 批量跑完 `specs/modules/` 下全部模块 |
| `+viz/mechanism.m` | 解析机构装配 DSL → 多实例拼接 → 全局 FK → 可视化 |
| `ARCHITECTURE.md` | 三层架构设计文档（调用层 → 累积层 → 计算层） |
| `config/` | 参数注入配置（`mechanism_viz_config.yaml` / `module_viz_config.yaml`） |

## 用法

在 MATLAB 中 `cd` 到本目录后：

```matlab
% 单个模块（带参数注入）
viz.module('../../specs/modules/frame.yaml', 'module_viz_config.yaml');

% 不注入参数：仅当模块平移无符号量时可行，否则会报未解析符号错误
viz.module('../../specs/modules/pin.yaml');

% 一次性校验全部模块
viz.allModules();

% 取回数值结果（每个 frame 的 4x4 全局位姿）做无图检查
r = viz.module('../../specs/modules/joint.yaml', 'module_viz_config.yaml');
disp(r.frames.linkB);
```

## 显示约定

- **三元坐标轴**：X=红、Y=绿、Z=蓝。轴长按机构特征尺寸自适应。
- **body**：优先加载 `bodies[].geometry` 指向的 STEP 几何，按 body pose 变换后以半透明蓝色面片显示。
- **几何缺失/失败**：若 `geometry` 缺失、路径无效、导入失败，或 MATLAB 当前不可用 STEP 导入 API，则不画 body 实体，仅保留 body triad、label 与各 frame/port。
- **port**（`exposed: true`）：实线三元轴 + 实心点 + 粗体标签。
- **内部 frame**（`exposed: false`，如 Adaptor_45 的 `rectify`/`align_axis`）：虚线三元轴 + 细标签。
- **pending 旋转**（待 SLX 提取）：洋红色标注 `(pending R)`，旋转按单位阵占位。
- `world` 坐标系画在原点，根 body 与之重合。

每个模块同时在命令行打印一份**坐标系报告**（名称、全局位置、+Z 朝向、是否 pending），即使不看图也能校验。

## 旋转语义（与 `specs/modeling-conventions.md` 一致）

- `align{a,b}`：规则 `s -> d` 表示子坐标系的 `s` 轴对应父坐标系的 `d` 轴；
  `R = DST * SRC'`，第三轴由右手定则补出（§9.4）。
- `rpy = [Rx,Ry,Rz]`：Z-Y-X 内旋，`R = Rz*Ry*Rx`（§8）。
- `axis_angle`：Rodrigues(omega, q)（§8 权威表示）。
- `pending`：单位阵占位，仅作图，提示该旋转数值未冻结。

## 参数注入

`module_viz_config.yaml` 按 `module_type` 分组，提供：

- 模块类几何参数（`cubeLength`、`tipDistance`），把符号平移（`cubeLength/2`、`-tipDistance`）解析为数值；
- 可选关节变量值（`Joint.q`），用于摆姿态展示自由度，缺省取 0（零位）。

未提供的符号会在求值时报明确错误，提示补进配置——这正是「定义能否成功生成描述」的校验点之一。

## geometry 路径解析

- `bodies[].geometry` 优先按仓库根相对路径解析，特别是 `assets/...` 会直接映射到仓库根下的 `assets/`。
- 绝对路径会直接使用。
- 其他相对路径会先相对模块 YAML 所在目录解析，再回退到仓库根解析。
- 文件名大小写按宽松模式匹配，因此 YAML 中 `.STEP` 与仓库里 `.step` 不一致时仍可找到同名文件。

## warning 行为

- 缺失文件：发出 `viz:module:geometryMissing` warning。
- 缺少可用导入 API：发出 `viz:module:geometryImportUnavailable` warning。
- 文件存在但导入/网格化失败：发出 `viz:module:geometryImportFailed` warning。
- 上述 warning 都不会中断整次可视化；frame/port 报告仍会继续输出。

## 局限

- 解析器只覆盖本项目模块/配置所用的 YAML 子集，非通用 YAML。
- STEP 显示依赖 MATLAB 本地可用的几何导入能力；当前实现会在可用时导入并网格化，缺失时退化为“仅 frame/port”。
- 关节默认按零位摆放（可经 config 指定角度）。
