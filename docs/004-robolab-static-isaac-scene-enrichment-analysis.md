# RoboLab 静态 Isaac 场景丰富化分析

状态：已实施，Workstation2 验证通过

日期：2026-07-31

## 结论

1. Workstation2 上的
   `/home/lenovo/swy/assets/robolab/sss225-robolab-assets`
   与 ModelScope 当前
   `sss225/robolab-assets@377fb4959532d2ee6055d3a874f25c4b327e2894`
   的 manifest 逐文件一致，可以判定为“当前 ModelScope 仓库完整”。
2. 当前需求只做静态场景丰富，不实现 RoboLab Task、predicate、reset、success、
   randomization、Isaac Lab environment registry 或模型。未来可选的 Task 第六层和
   IsaacLab 3.x 只记录边界，不形成当前需求。
3. RoboLab 的桌面、家具、道具、背景和场景初始布局属于现有 **Workcell**；
   NERO + Hand 2 或其他具身体仍属于 Asset / Binding / Assembly，Session 继续做唯一
   composition root。
4. 已在不升级五层 schema 的前提下实现 Workcell v1 typed compatibility leaf、
   source-neutral `ResolvedIsaacWorkcellPlan`、共享 materializer，以及 Workstation2
   ModelScope ensure/manifest verifier。
5. `base_empty`、`banana_bowl`、`workdesk` 三个场景已仅通过配置接入；旧的
   primitive-only 简单场景继续走同一个 Plan，因此当前不需要升级 Workcell v2。
6. ModelScope 保持唯一云端资产来源；不建立项目自有 Git LFS 镜像。资产只在
   Workstation2 恢复到 gitignored、带 commit 的项目相对目录，Mac 控制端不下载也不做
   Isaac 资产验证。
7. 实现选择了完整的 staging/resume/receipt 路径、USD dependency 和物理 census、
   三场景验证及一个非 NERO 回归，整体落在原“正式接入”估算上沿。后续同类静态布局
   通常只增加 profile、Workcell、Session 和验证证据。
8. 最终验证见
   `docs/validation/2026-07-31-robolab-static-workcell.md`。

这里的“静态布局”指确定的场景选择和初始摆放，不表示把所有物体强制改成 fixed。
RoboLab USD 中已经 author 的 dynamic / kinematic / static、碰撞和材质应默认保留；

2026-08-03 抓放 pilot 增加一条例外：banana-bowl 任务 profile 显式把 `table` 和
`bowl` 固定为 fixture，只保留 `banana` 为动态任务对象。该覆盖进入 resolved plan 和
run manifest，不修改 source-locked USD，也不改变其他 RoboLab 场景的默认保留策略。
本项目只是不实现 episode reset 和任务判定。

## 1. 本轮范围

包含：

- RoboLab scene USD、桌面、货架、箱筐、工具、食物等道具；
- HDR / EXR 背景和灯光选择；
- 场景到 Workcell world/table frame 的整体变换；
- NERO + Hand 2 和其他 Assembly 的 mount；
- ground、lighting、physics scene、collision 的冲突策略；
- 资产来源锁定、完整性和加载验证。

不包含：

- TaskSpec、自然语言任务、predicate、reward 或 success/failure；
- observation/action 配置、策略服务、Isaac Lab environment registration；
- episode reset、随机化、向量化环境和 benchmark/dashboard；
- 把 NERO + Hand 2 robot compiler 泛化成任意机器人 compiler；
- Task 第六层、IsaacLab 3.x training/evaluation adapter 和模型接入；本文只在
  “未来事实”中记录其兼容边界；
- Task、IsaacLab、模型或真实硬件相关的代码、配置和运行修改。

## 2. Workstation2 资产核验

### 2.1 核验结果

上游：

- [ModelScope 数据集文件页](https://modelscope.cn/datasets/sss225/robolab-assets/files)
- branch：`master`
- commit：`377fb4959532d2ee6055d3a874f25c4b327e2894`
- commit message：`Upload RoboLab Isaac assets (batch 52/52)`
- commit time：2026-07-23 17:21:21 +08:00

Workstation2：

- SSH alias / hostname：`lenovo-piper2` / `Workstation2`
- 本地目录：
  `/home/lenovo/swy/assets/robolab/sss225-robolab-assets`
- 普通文件：3,285
- 子目录：663（不含资产根目录）
- symlink：0
- 逻辑总大小：6,814,007,813 bytes（约 6.814 GB / 6.345 GiB）
- `du -sh`：6.4G

逐项对照 ModelScope manifest：

| 检查 | 结果 |
|---|---:|
| 上游 blob | 3,285 |
| 本地存在 | 3,285 / 3,285 |
| size mismatch | 0 |
| SHA-256 mismatch | 0 |
| 上游 tree | 663 |
| 本地存在 | 663 / 663 |
| zero-byte file | 0 |
| Git LFS pointer | 0 |
| `.part` / `.partial` / `.incomplete` / `.tmp` / `.download` | 0 |

因此本轮完整性判定为：

> **PASS — 与 2026-07-31 检查时 ModelScope `master@377fb495` 完整一致。**

ModelScope tree API 共返回 3,948 个条目，即 3,285 blobs + 663 trees。API 单页最多
返回 3,000 条，本轮读取了两页，并对每个 blob 的 `Path`、`Size` 和 `Sha256` 做了
本地逐项校验：

- [manifest API 第 1 页](https://modelscope.cn/api/v1/datasets/sss225/robolab-assets/repo/tree?Revision=master&Recursive=True&PageNumber=1&PageSize=3000)
- [manifest API 第 2 页](https://modelscope.cn/api/v1/datasets/sss225/robolab-assets/repo/tree?Revision=master&Recursive=True&PageNumber=2&PageSize=3000)

`~/.cache/modelscope/.lock` 下有 3,285 个同数据集的零字节 lock，时间覆盖本次下载过程；
没有下载进程、打开的 lock 或临时下载文件。这些是下载遗留锁，不是未完成下载的证据。

### 2.2 资产分区

| 分区 | 文件数 | 逻辑大小 |
|---|---:|---:|
| `assets/backgrounds` | 971 | 863,835,922 B |
| `assets/fixtures` | 58 | 181,231,844 B |
| `assets/materials` | 42 | 54,737,865 B |
| `assets/objects` | 1,948 | 4,736,608,988 B |
| `assets/robots` | 75 | 57,994,817 B |
| `assets/scenes` | 189 | 919,565,651 B |

关键索引均存在：

- `assets/objects/object_catalog.json`：312 条；
- `assets/scenes/_metadata/scene_metadata.json`：67 条；
- `assets/scenes/_metadata/scene_statistics.json`：官方统计 61 个整理后的场景、
  79 个 unique objects、平均 4.25 个 object；
- `assets/scenes/_metadata/scene_table.csv`；
- `assets/backgrounds/backgrounds.json`。

`assets/scenes/` 顶层有 96 个 `.usda` 和一个 `white_void.usd`。它们不应直接解读为
“97 个同等成熟的场景”：官方 README 的整理集合是 61 个，其余包含 keypoint、
测试或辅助场景。首批应从官方统计的 61 个场景选择。

### 2.3 完整性的边界

“ModelScope 仓库完整”不等于“所有 RoboLab 可选外部资源完整”：

- `assets/backgrounds/README.md` 明确要求 outdoor HDR 从 Google Drive 另下约 2 GB；
- 当前 ModelScope `outdoors/` 有 336 张 PNG 预览和 336 个 TXT 描述，但没有 HDR；
- `default/` 有 4 HDR + 1 EXR，`indoors/` 有 95 HDR，足够覆盖首批室内丰富化；
- outdoor HDR 若未来需要，应作为独立 source-lock 和独立完整性 Gate，不应混入本次
  PASS。

此外，RoboLab framework 的 Apache-2.0 不能自动覆盖所有第三方物体资产。当前树中
已有 YCB、HOPE、Handal、HOT3D 等各自 LICENSE；正式纳入场景 catalog 时仍需保留
subtree/file 级 provenance 和许可信息。

## 3. 实施前“一张桌子”基线

现有
`configs/workcells/isaac_nero_dual_hand2_simulation_nominal_v1.yaml`
只有：

- ground plane；
- `1.20 m × 1.20 m × 0.08 m` fixed box 桌面；
- nominal table top、左右 NERO mount 和若干 camera frame。

`src/wujihand/runtime/isaac_dual_scene.py` 的实际 materialization 也只：

- 创建默认 ground；
- 接受 fixed plane / box；
- 给 box 固定棕色；
- 硬编码一个强度为 1000 的 DomeLight；
- 随后加载双 NERO + 双 Hand 2。

fixed-hand 和 rotation-ball Isaac 入口还各自 author ground/table/light。也就是说，
实施前的问题不是“五层无法容纳复杂场景”，而是缺少共享的复杂 Workcell
materializer。

## 4. 结合五层架构重新归属

| 层 | 本需求的变化 | 判断 |
|---|---|---|
| Asset Manifest | 不放 RoboLab 桌子、背景和道具 | 不变 |
| Backend Binding | 不为环境伪造 control group / joint binding | 不变 |
| Assembly Spec | 继续只描述 NERO + Hand 2 或其他 embodiment forest | 不变 |
| Workcell | 引用 RoboLab scene/background 表示，保留 frame、mount 和 primitive overlay | 唯一需要扩展 |
| Session | 选择 Assembly + rich Workcell，并把 root 放到 mount | schema 首版不变 |

### 4.1 为什么不能塞进 Asset / Binding

现有 `AssetManifest` 只允许：

- `robot_arm`
- `robot_hand`
- `virtual_mechanism`

并要求非空 control group；`BackendBinding` 同样要求 group bindings。桌面、货架、
背景和普通道具不具备这些语义。把数百个环境物体包装成可控 Asset 会制造假的 DoF 和
控制合同，也会破坏五层原本的产品身份边界。

### 4.2 为什么属于 Workcell

`WorkcellSpec` 的既有定义就是“robot instances 之外的 world geometry 和 semantic
placement slots”。RoboLab 的完整 scene USD 是该 Workcell 在 Isaac backend 下的一种
表示，不是一个新的机器人产品，也不是任务。

所以本轮不增加第六层：

```text
Asset Manifest -> Backend Binding -> Assembly
                                      |
RoboLab static USD -> Workcell -------+-> Session -> Isaac scene
```

场景 USD 应作为一个整体引用；不要把 USD 内每个 bowl、banana、shelf 的 pose 再抄一份
到 YAML。USD 是布局的单一事实源，Workcell 只声明 source identity、整体变换、策略和
语义 mount。

### 4.3 实施前友好度

概念层面：**高**。

- robot assembly 与 world geometry 已分离；
- Session 已能替换 Workcell 而不改 Assembly；
- resolved Session 会把 Workcell 和兼容 profile 的内容 hash 纳入快照；
- 多 root Assembly 已支持双 NERO。

实现层面：**中低**。

- Workcell v1 的 entity 是 `plane | box | sphere | frustum` 封闭集合；
- 没有通用 USD layer/reference、background、light 和 collision policy；
- 三类 Isaac runner 的环境 authoring 尚未共用；
- 双 NERO + Hand 2 robot materializer 本身仍是专用拓扑；
- Session placement 只有 `root -> mount_id`，没有 per-session base offset。

因此可以承诺“同一静态环境管线可服务多具身体”，暂不能承诺“任意具身体自动注入任意
RoboLab 场景”。

## 5. 已采用的接入路线

### 5.1 第一步：Workcell v1 compatibility leaf

首个场景不改五层 schema。复用 Workcell 已有的 `compatibility_profile`，新增 typed
`isaac_static_usd_workcell.v1` leaf，最少描述：

- source id、ModelScope revision 和 scene USD path；
- source/default prim、target prim；
- scene frame 和 world transform；
- composition policy，首选 USD reference；
- ground policy：`preserve | project | none`；
- lighting/background policy：`preserve | project | selected_hdr`；
- authored physics/collision policy，首版默认 `preserve`；
- expected scene/closure digest；
- camera policy 和可选 collider census 约束。

对应 Workcell v1 继续声明：

- world/tabletop/scene anchor；
- 与 embodiment 无关的物理 mount，例如
  `table_near_left`、`table_near_right`，避免写成 `nero_*`；
- camera eye/target frame；
- 必要时的小型 primitive overlay；
- 不再重复 author 与 RoboLab scene 相同的 table/ground。

每个 rich Workcell 使用一个新的 Session 变体，保留当前 qualification Session
不变。这样 session hash 仍明确代表“哪套机器人 + 哪个静态布局”。

### 5.2 第二步：共享 `IsaacWorkcellMaterializer`

实现一个 embodiment-neutral 的 Isaac workcell materializer：

1. 在 `/World/Environment/<import_id>` 引用 upstream USD；
2. 检查 default prim、引用闭包、`metersPerUnit`、up axis；
3. 应用 scene root 的单一刚体变换，不改 upstream 文件；
4. 依据 policy 处理已有 `PhysicsScene`、GroundPlane、DomeLight、camera；
5. 保留 authored rigid body、collision、material；
6. 返回 environment/collider 路径和加载报告；
7. 之后由现有 NERO、fixed-hand 或 rotation-ball runner 注入各自 Assembly。

RoboLab scene README 显示很多场景已经包含 `PhysicsScene`、`PhysicsMaterial`、
`GroundPlane`、table，有些还包含 `world_camera`。当前默认 ground 和硬编码 DomeLight
不能无条件叠加，否则会出现双 ground、重复 physics scene 或曝光不一致。

5.2 必须先形成与配置版本、资产托管方无关的纯数据
`ResolvedIsaacWorkcellPlan`。ModelScope/RoboLab 专用的下载和 manifest 解析停在
`SourceResolver -> ResolvedContentRef`；Plan 只保存已闭合的 content identity、
通用 USD import/primitive operation、相对 environment namespace、transform、policy
和 logical entity inventory。未来其他 USD 库、Nucleus checkpoint 或 scene skill
生成物，只要能解析成确定性的 Isaac stage operation，都可进入同一个 Plan。

### 5.3 何时升级 Workcell v2

先用 compatibility leaf 跑通一个简单场景和一个拥挤场景。满足以下任一条件后，再把
稳定字段提升到 Workcell v2：

- 已维护三个以上 rich scene；
- fixed-hand、rotation-ball、NERO 三类入口都开始复用；
- 一个 Workcell 需要多层 USD composition；
- backend representation 需要稳定 schema 和统一 resolver。

Workcell v2 可以把 leaf 提升为 typed `representations.isaac.imports`，但仍属于
Workcell，不新增 `SceneSpec`。v2 应同时支持：

- primitive-only：保留现有简单 ground/table 场景；
- imports-only：引用完整 RoboLab 或其他来源的 USD；
- hybrid：USD scene 加少量 primitive overlay、semantic frame 和 mount。

loader 长期保留 Workcell v1，并让 v1/v2 归一到同一个 Plan；不强制迁移旧 Session，
也不改变旧 v1 snapshot/hash。不要一开始实现跨 MuJoCo/Isaac 的万能 scene compiler。

### 5.4 静态布局管线

目标管线应是：

```text
ModelScope pinned snapshot
  -> curated scene/background catalog
  -> Workcell semantic frames + Isaac representation
  -> resolved Session
  -> IsaacWorkcellMaterializer
  -> embodiment-specific robot injection
  -> stage/collision/render smoke
```

它可以做到“新增布局主要写配置”，但它是 **scene-layout pipeline**，不是 RoboLab
task-creation pipeline。

## 6. ModelScope 资产恢复与项目相对路径

最终决定：

- ModelScope `sss225/robolab-assets` 是唯一云端 canonical source；
- 不把完整资产提交到当前仓库，也不建立项目自有 Git LFS mirror；
- committed 配置只保存 dataset id、精确 commit、manifest identity/digest 和相对
  artifact path；
- 资产下载和 Isaac 验证只发生在 Workstation2；Mac 控制端不下载、不 settle、不启动
  Isaac 资产 smoke。

### 6.1 Workstation2 canonical path

固定使用 gitignored、带完整 commit 的项目相对目录：

```text
third_party/src/modelscope/sss225/robolab-assets/
  377fb4959532d2ee6055d3a874f25c4b327e2894/
```

该 version directory 直接包含上游 `README.md`、`.gitattributes` 和 `assets/`。
`third_party/sources.lock.yaml` 的 `local_runtime_path` 指向该精确相对路径；Workcell
只引用 source id/revision/scene path，不出现 Workstation2 绝对路径。

不使用外部 symlink。当前
`/home/lenovo/swy/assets/robolab/sss225-robolab-assets`
已经完成校验。本次部署以它为 seed，在同一文件系统优先 hardlink、必要时 copy 到
上述 version directory，并完成全 manifest 复验；没有建立 symlink。若以后需要
alias，也只能指向项目根内的 version directory；source lock 仍固定真实 version
path，不引用 `current`。

### 6.2 Workstation2 ensure 流程

所有获取动作由显式 Workstation2 preflight/ensure 入口完成，纯 spec/resolver import
和 Mac 测试不得触发网络：

1. 从项目根解析 pinned project-relative target；
2. 检查 version directory、关键 metadata 和本地 verification receipt；
3. target 缺失或未完成时，从 ModelScope 下载精确 commit 到同一文件系统中的 staging
   directory；
4. 按上游 manifest 校验 3,285 个 blob 的 path、size、SHA-256，以及 663 个 tree；
5. 校验通过后原子 rename 到 canonical target；失败的 staging 不得冒充可用快照；
6. verification receipt 写在资产目录外的 gitignored sibling 目录，避免污染上游树；
7. 后续运行只检查 receipt、manifest identity 和当前 scene dependency closure；不在
   每次启动时重算整个 6.8 GB tree。

Mac 只编辑/解析配置和发起 SSH 协作。本文记录的完整性、Isaac 6.0.1 stage smoke、
settling、碰撞和截图结论，全部以 Workstation2 结果为准。

## 7. NERO + Hand 2 与其他具身体

### 7.1 NERO 首要路径

首批不要直接导入最拥挤场景。建议顺序：

1. `base_empty.usda`：验证 USD closure、table/ground/physics 冲突和 NERO mount；
2. `banana_bowl.usda` 或 `colored_blocks.usda`：验证少量动态物体、材质和 settling；
3. `workdesk.usda` 或 `tools_sorting.usda`：验证拥挤布局、碰撞数量和渲染负载。

RoboLab 场景中的 table 尺寸和默认 robot side 未必与当前
`1.20 m × 1.20 m` 双 NERO nominal table 一致。每个候选场景都要重新确认：

- 两个 base mount 是否有实体支撑且不互撞；
- q7 初态是否与桌面、货架和道具无穿透；
- 左右 Hand 2 工作空间是否覆盖目标区域；
- 场景整体 transform 是否保持桌面高度和 up axis；
- scene 中是否已有 physics scene、ground、light 和 camera。

不要同时保留当前 primitive table 和 RoboLab scene 自带 table。首版应让 imported
scene 拥有 table/fixture/object，Workcell 只拥有语义 frame、mount 和政策。

### 7.2 其他具身体

可复用部分：

- ModelScope source lock 和 scene catalog；
- Workcell USD/background representation；
- Isaac workcell materializer；
- collision/material/lighting/load report；
- 使用物理语义命名的 mount。

不可直接复用部分：

- 当前 `DualNeroHand2IsaacScene` 的双 NERO + Hand 2 拓扑；
- NERO 专用 q27 route、初态和 drive gain；
- 当前 NERO 命名 mount；
- 不同底座的 base-frame offset、reachability 和 clearance。

其他 Assembly 可以通过新的 Session 选择同一个兼容 mount；如果 mount 或桌面尺寸不同，
应产生新的 Workcell revision，而不是修改 RoboLab USD 或在 runner 中增加机器人特例。

## 8. 代码量估算

以下均含对应 unit/contract test，不含 6.8 GB 资产，也不含 Task、IsaacLab 或模型：

| 范围 | 代码量 | 每场景配置 |
|---|---:|---:|
| 直接在 NERO runner 硬接一个 USD，仅做演示 | 100–250 LOC | 0–20 行 |
| Workcell v1 typed leaf + 通用 materializer + NERO 集成 | 450–900 LOC | 40–100 行 |
| Workstation2 ModelScope ensure + manifest verifier | 另加 150–300 LOC | 0 行 |
| 正式 Workcell v2、resolver/source-lock 和多场景 catalog | 900–1,600 LOC | 25–80 行 |
| 再迁移 fixed-hand/rotation-ball，覆盖现有 Isaac 入口 | 1,300–2,200 LOC | 25–80 行 |

分析阶段以 **600–1,200 LOC** 作为静态场景、NERO 优先且包含 Workstation2
资产恢复入口的 MVP 预算；
以 **1,300–2,200 LOC** 作为正式共享丰富 Workcell 的预算。
可选 Task 第六层、IsaacLab 3.x adapter 和模型均不在这两个预算中。

新增一个已知依赖闭包的布局，正常情况下不增加 Python，只增加：

- 一个 scene/Workcell profile；
- 一个小型 Session 变体；
- 一次 Isaac 6.0.1 stage/collision/render smoke 的证据。

## 9. 分阶段 Gate

Gate A～D 已全部通过。最终命令结果、场景 census、report digest 和兼容警告统一记录在
`docs/validation/2026-07-31-robolab-static-workcell.md`。

### Gate A：资产与候选场景

- 固定 ModelScope commit 和 manifest；
- 在 Workstation2 从项目根解析 canonical version directory；
- 目录缺失或校验未完成时，下载精确 commit 到 staging，校验通过后原子提升；
- 目录存在时先检查 receipt、manifest identity 和候选 scene dependency closure；
- 选 `base_empty`、一个简单动态场景、一个拥挤场景；
- 检查引用闭包和各 subtree license；
- outdoor HDR 不阻塞首批室内场景；
- Mac 不执行下载、完整性核验或 Isaac smoke。

### Gate B：Isaac Sim 6.0.1 原样打开

- 无 unresolved reference / missing material；
- default prim、meters、up axis 正确；
- scene collider 和 rigid body 可枚举；
- 不注入机器人，运行有限帧后动态物体不爆飞。

RoboLab 当前官方支持 Isaac Sim 5.0 / 5.1，项目目标机是 6.0.1。官方也明确不同版本的
PhysX 会影响 contact/settling，因此不能仅凭 USD 能打开就推断动力学兼容。

### Gate C：NERO + Hand 2

- environment 先加载，双 NERO + Hand 2 后注入；
- 没有重复 PhysicsScene / ground / light；
- 初态无穿透、无自碰撞异常；
- 双 base 有支撑，双手可覆盖选定工作区域；
- 运行有限帧并保存 screenshot、collider census 和加载时间。

### Gate D：管线复用

- 第二、第三个场景只改配置；
- 至少一个非 NERO Isaac 入口复用 materializer；
- 再决定 Workcell v2，而非预先冻结 schema。

## 10. 未来事实（当前无需求）

本节只记录兼容方向，不进入本期 schema、实现、依赖、验收或工作量。

### 10.1 可选 Task 第六层

未来可把 RoboLab 风格的任务组合建模为五层之后的可选第六层：

```text
TaskDefinition ----\
                    -> ResolvedTask
ResolvedSession ---/
```

它是 Session 的下游组合，不是修改现有五层，也不是让 `TaskDefinition` 继承或硬编码
某个特定 Session。`TaskDefinition` 应保持 robot-agnostic，只引用 Workcell scene
identity、logical entity role、instruction、predicate 和 success/failure；resolver
再将它与 `ResolvedSession` 的实际实体、能力和具身体绑定为 `ResolvedTask`。

本期不新增 Task schema、skill runner、predicate、reset、registry 或自动化入口。

### 10.2 IsaacLab 3.x

项目仿真目标固定为 Isaac Sim 6.0.1，因此未来若引入 IsaacLab，应选择兼容的 3.x
版本。官方 `v3.0.0-beta2.patch1` 明确增加 Isaac Sim 6.0.1 支持；同时 IsaacLab 3.0
是一次架构重构且当前仍为 beta，不能把 2.x 组件视为直接兼容。

RoboLab 当前 `pyproject.toml` 只提供：

- IsaacLab 2.2.0 + Isaac Sim 5.0.0；
- IsaacLab 2.3.2.post1 + Isaac Sim 5.1.0。

所以未来应单独提出 training/evaluation 集成计划，在独立可选环境中适配 IsaacLab
3.x；当前 direct Isaac teleop、五层 core 和静态 Workcell 不依赖 IsaacLab，也不因
未来训练预埋其 package/config 类型。可能的边界只记录为：

```text
ResolvedTask + ResolvedIsaacWorkcellPlan
  -> future IsaacLab 3.x execution adapter
  -> training / evaluation environment
```

模型同样不在本期，也不形成预留接口需求。

## 11. 剩余边界

1. Isaac 6.0.1 对部分上游 OmniPBR 参数有非阻塞 MDL warning；当前三个场景均已加载、
   仿真和截图通过，但尚未做逐材质迁移。
2. 本轮只做有界 smoke，没有建立实时帧率、显存或长时间漂移基准。
3. 第三方 object/background 的 license 和 provenance 仍需随未来 scene catalog 扩展
   逐项保留。
4. pinned commit 升级必须显式修改 source lock 并重新验证，不跟随 rolling
   `master`。

## 12. 本地依据

- `docs/components/five-layer-session-composition.md`
- `docs/decisions/0003-five-layer-session-composition.md`
- `configs/workcells/isaac_nero_dual_hand2_simulation_nominal_v1.yaml`
- `src/wujihand/specs/asset.py`
- `src/wujihand/specs/backend_binding.py`
- `src/wujihand/specs/workcell.py`
- `src/wujihand/runtime/config_repository.py`
- `src/wujihand/runtime/source_lock.py`
- `src/wujihand/runtime/modelscope_dataset.py`
- `src/wujihand/runtime/isaac_workcell_plan.py`
- `src/wujihand/runtime/isaac_workcell.py`
- `src/wujihand/runtime/session_resolver.py`
- `src/wujihand/runtime/isaac_dual_scene.py`
- `src/wujihand/integrity.py`
- `tools/ensure_modelscope_assets.py`
- `tools/validate_isaac_workcell.py`
- `docs/components/nero-isaac-dual-twin.md`
- `docs/validation/2026-07-28-isaac-6.0.1-python-3.12-compatibility.md`

外部依据：

- [NVlabs/RoboLab](https://github.com/NVlabs/RoboLab)
- [RoboLab scene catalog](https://github.com/NVlabs/RoboLab/blob/main/assets/scenes/README.md)
- [RoboLab background catalog](https://github.com/NVlabs/RoboLab/blob/main/assets/backgrounds/README.md)
- [RoboLab task concepts](https://github.com/NVlabs/RoboLab/blob/main/docs/task.md)
- [RoboLab dependency matrix](https://github.com/NVlabs/RoboLab/blob/main/pyproject.toml)
- [ModelScope sss225/robolab-assets](https://modelscope.cn/datasets/sss225/robolab-assets/files)
- [IsaacLab releases](https://github.com/isaac-sim/IsaacLab/releases)
- [IsaacLab installation](https://isaac-sim.github.io/IsaacLab/develop/source/setup/installation/index.html)
