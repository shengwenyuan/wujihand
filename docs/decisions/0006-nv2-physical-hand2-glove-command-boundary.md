# ADR-0006：NV-2 物理 Hand 2 与 Glove 命令边界

- 状态：已接受（两项架构决策）；self-collision policy 待资格确认
- NV-2 Gate：`PARTIAL`
- 日期：2026-07-28
- 影响范围：Hand 2 Asset/Isaac Binding、NERO—Hand 2 Assembly、Session v1、
  Glove input/retargeting adapter、NV-2 Isaac runner

## 背景

NV-2 最初只需要双 NERO 数字孪生，曾考虑把 Hand 2 作为不可控的渲染 child。Wuji
Glove 现已可进入本版本范围，虚拟左右 Hand 2 需要接收对应侧的 q20 命令。继续使用
visual-only Hand 2 或 `inactive` finger group 会直接与该需求冲突。

同时，官方 Hand 2 USD 是完整物理 articulation：包含 rigid body、collision、drive、
world-fixed root joint 和 20 个主动关节。若将它直接引用到 NERO 法兰旁而不处理 root，
Hand 2 仍固定到 world；若把 Glove 的 21 DoF 人手角直接当成 q20，又会混淆两种不同
布局。

本 ADR 固定产品与架构边界，不批准任何真实 NERO 或 Hand 2 运动。

## 决策一：使用完整物理 Hand 2 USD

左右 Hand 2 Isaac Backend Binding 直接引用固定
`wuji-description v2026.6.27` 的官方完整物理 USD，不制作 visual-only 派生表示：

- Asset Manifest 保持后端无关的 Hand 2 身份、side 和 canonical q20 layout；
- Backend Binding 拥有固定 USD、source revision、frame map 和 20-joint map；
- Assembly 只表达 `NERO link7 → Hand 2 hand_base` attachment；
- Workcell 拥有 world、桌面和 NERO mount；
- Session 选择本次运行使用的四个 control group。

来源 USD 中的 rigid body、collision、drive 和 20 个 revolute joint 都保留。绑定名称
使用 `physical`，不能用“twin”暗示同一物理 USD 已被剥除 physics。

当前左右 attachment transform 均为
`position=[0.023, 0, -0.0235] m`、`Ry(+90°)`。该变换抵消固定 J7 origin 横向
偏置，将 Hand 2 基座耦合面放到 mesh-derived `link6 +X` 端面中心，并保持既有手部
工作朝向；它是固定资产间的 simulation-nominal 接口映射，不表示存在直角转接结构。
经 prim 隔离确认，待旋转圆柱属于 NERO `link6`；其 visual/collision/mass 表示对齐
仍由 NERO Backend Binding 负责，不改写 J7。

## 决策二：Session v1 中四组全部显式 commanded

NV-2 沿用 Session v1，不增加 `inactive` 或 Session v2。完整 Session 必须恰好路由：

| side | NERO group | Hand 2 group |
|---|---|---|
| left | `arm_joints / agilex_nero_q7_v1 / q7` | `finger_joints / wuji_hand2_left_firmware_v1 / q20` |
| right | `arm_joints / agilex_nero_q7_v1 / q7` | `finger_joints / wuji_hand2_right_firmware_v1 / q20` |

四组共 54 logical DoF，全部是显式 commanded。runner 不得省略 Hand route、暗中注入
永久全零 q20，或通过专用旁路绕开 Session resolver。

Glove 设备和官方 retarget runtime 不进入五层 schema。它们保留在 adapter 边界：

```text
Wuji Glove hand_skeleton
  -> input adapter
  -> CanonicalHandObservation
       named MediaPipe 21 landmarks
       float (21, 3), metres
       side/frame/confidence/time/calibration provenance
  -> side-specific retarget adapter
       RetargetSession.for_hand(HandModel.WujiHand2, side=...)
  -> HandIntent
       side-specific Hand 2 q20 radians + explicit layout
  -> GloveHand2SimulationController
  -> JointCommandSupervisor
  -> compose_q27_hand_target
  -> corresponding same-side Session q20 group
```

SDK device、subscription、frame 和 retarget session 对象不得越过 adapter。核心
domain/ports 不 import `wuji_sdk`。

`hand_joint_angles` 是人手 21 DoF 模型产物，不是 Hand 2 q20，禁止直接透传。只有完整、
side 匹配、fresh、finite 且达到置信度要求的 named 21 点米制观测才可进入 retarget。
来源、标定或 frame epoch 改变，以及长时间中断后，必须 reset retarget session。

置信度策略固定为：`<0.90` 拒绝，`[0.90, 0.95)` 只允许以 `DEGRADED` 状态经过
supervisor，`>=0.95` 才是 success。missing、stale、错误 side/layout、NaN 或 solve
failure 不得创建新的 input-derived `HandIntent`。已有最后有效命令可在 supervisor
的 `0.25 s` freshness 窗内 hold；超时后 supervisor 输出渐进 return-to-rest 安全
命令。该安全输出不是由 invalid input 派生的新 q20 intent。

## q27 是物理实现后果，不是新增配置层

四个 logical command group 不应在 Isaac 中形成四棵独立 articulation。每侧 NERO 和
Hand 2 必须组成一棵物理 articulation：

```text
left  q27 = left NERO q7  + left Hand 2 q20
right q27 = right NERO q7 + right Hand 2 q20
```

simulation adapter 在 PhysX 初始化前：

1. 从 NERO Binding profile 对齐 `link6` visual/collision/mass，不改 joint frame；
2. 从已解析 Assembly 读取带固定平移与 `Ry(+90°)` 的 attachment transform；
3. 将 Hand 2 base 放置到 NERO 法兰；
4. 禁用 Hand 2 USD 的 world-fixed `root_joint`；
5. 移除该 prim 的 `ArticulationRootAPI`；
6. author `link7 → hand_base` FixedJoint；
7. 按 canonical joint name 和 USD joint path 验证 q7/q20 分区。

所有修改只存在于 live stage overlay，不修改固定来源 USD。stage 最终必须恰好有两个
articulation root，每个 27 DoF；逻辑总数仍为 54。

## Self-collision 资格边界

NERO 派生 articulation 的 self-collision 为关闭，而来源 Hand 2 articulation 的策略
不同。合并为一棵 q27 后，PhysX articulation root 只能采用一个 self-collision
设置。

当前 NV-2 smoke 使用：

```text
merged q27 self-collision = false
external collision shapes/contact = retained
```

这表示 Hand 2 finger/palm 内部 self-collision 未资格化。是否将该值冻结为 NV-2 最终
策略仍待项目负责人确认：

- 若保持 `false`，需在文档中明确内部 self-collision 不属于当前能力；
- 若改为 `true`，需先为 NERO 增加明确 collision filtering，再重跑结构、contact、
  GUI 和稳定性 Gate。

在确认前，inward-port tabletop v15 的 90/90 证明当前 nominal 定义下双侧
q7/q20、五指/组合手型、隔离、reset/recovery、limits、有界静置收敛，以及
`link6` 圆柱—小臂轴、基座端面中心/平行度、attachment anchor、mount/q7 准备位
和手工作姿态；右侧 live Glove
另有现场证据。它仍不证明 Hand 2 internal self-collision 或 deliberate
contact/unknown penetration，因此不关闭完整 NV-2 Gate。

## Nominal 与 measured Workcell

NV-2 允许使用名称和 assumption 都明确为 `simulation_nominal` 的 Workcell 完成功能
联调。该 revision 可支持：

- 五层闭合与 source/hash 复现；
- q27 拓扑、q7/q20 movement 和左右隔离；
- nominal 初始状态与基础外部 contact 检查。

当前 Workcell 将两底座放在同一近侧桌沿 `x=±0.32 m, y=-0.52 m`，yaw 均为
`-90°`，使接电侧朝桌内；Session runtime 唯一引用 typed tabletop qualification
profile，保存左右 q7 `[∓10°, +45°, 0°, +45°, +90°, 0°, 0°]`、Isaac-only
drive gain 和几何阈值。接电侧朝桌内由项目负责人实物确认；端口轴
`base local -X` 仍只是固定 mesh 的表示约定。该 profile 不是新的配置层，不修改
共享 Asset 的通用 q7 home，也不是硬件控制器参数。

它不能支持真实装配、clearance、可达空间、定位精度或真机安全结论。NERO J7
轴/零位/符号、法兰孔位 clocking、桌面尺寸和底座 mount 的实测值必须形成新的
measured Binding/Assembly/Workcell revision，不得静默覆盖 nominal 文件。

## 未采用方案

| 方案 | 原因 |
|---|---|
| visual-only Hand 2 派生 USD | 无法满足 Glove 驱动虚拟 Hand 2 q20 的版本需求 |
| Session v2 + Hand group `inactive` | 与四组显式命令目标冲突，并无必要扩大 schema |
| Session 中省略 Hand route | 破坏 resolver 对每个 Asset control group 恰好路由一次的合同 |
| 固定发送 neutral q20 | 隐藏实际 command ownership，无法表达 Glove 输入或失败语义 |
| 将 `hand_joint_angles` 21 DoF 截断/重排为 q20 | 人手模型与 Hand 2 机器人布局语义不同 |
| 在 domain/Session 保存 Wuji SDK 对象 | 破坏固定依赖方向和无硬件测试能力 |
| 把 Assembly `Ry(+90°)` 解释为实体直角转接件 | 当前只证明它是两个固定资产的接口坐标映射 |
| 把端口轴或 link6 Binding 对齐称为真机事实 | 尚无设备 J7/link6 只读回读、孔位接口图或现场测量 |

## 后果

### 正向

- Glove 手部遥操作可在 NV-2 内完成，不必推迟到臂位姿遥操作阶段。
- 两侧 Hand 2 保持真实 joint、drive、rigid body 和外部 collision 行为。
- 五层职责、Session v1 和现有 resolver 不需要破坏性升级。
- canonical observation、HandIntent 和 SDK adapter 可独立测试与回放。
- q7/q20 的逻辑 command ownership 与 Isaac q27 物理执行拓扑同时明确。

### 代价与限制

- live stage 必须在 PhysX 初始化前完成严格的 articulation 重组。
- runtime DOF 顺序不能假定为 q7 后接 q20，必须按 name/path 分区。
- Glove live Gate 依赖实际设备连接、side/serial 和 calibration provenance。
- nominal 场景不能替代 measured Workcell。
- self-collision policy 未确认前，NV-2 只能保持 PARTIAL。

## 验证责任

至少保留以下证据：

- Session 恰好 4 route、54 logical DoF，source/artifact/session hash 可复现；
- stage 恰好 2 个 articulation root，每个 q27；
- Hand world root 已禁用，FixedJoint body targets 正确；
- 左右 q7/q20 partition 完整、互斥且 side/layout 正确；
- q7/q20 response、hold、isolation、finite 和 limits；
- composition-level stale、missing、low-confidence、wrong-side、wrong-layout、NaN
  和 solve failure 不产生新的 input-derived `HandIntent`，并只经 supervisor
  hold/return-to-rest 输出安全命令；
- 至少一侧实际 Glove live
  `hand_skeleton → canonical → retarget → Isaac` smoke；
- self-collision policy 对应的 contact/GUI 资格证据。

inward-port tabletop v15 报告的 90 项检查全部通过：保留 scripted physical
v2 的双侧 q7、五指/组合手型、隔离、finite/limits、命令后 topology reset、回到
批准初态及 post-reset recovery，并增加分侧 q7 初态、`link6` 圆柱—小臂轴、
Hand 2 基座端面中心/平行度和 attachment anchor、
`link4 → link5` 小臂近水平、桌内方向、掌面向下和接电侧朝桌内检查；
controller/supervisor/composer 的无硬件测试也已覆盖 composition-level invalid
fail-closed。报告只证明 fixed external collider 保留和 bounded rest settling，
且明确 `deliberate_unknown_penetration_probe=false`。右侧实际 Glove live 已完成，
最近一次 2400 帧接收 2399 帧、拒绝 0 帧；稳定 identity、正式 calibration revision
和脱敏 replay 尚未冻结。接口近景已完成，但 deliberate contact/异常穿透与最终
self-collision 验证责任仍未闭合。历史 v6/v11/v12 只保留为旧接口定义证据。

## 重验触发器

以下任一变化必须重跑相关 contract 和 Isaac Gate：

- Hand 2 source revision、USD 或 q20 joint/layout；
- NERO 派生 USD、`link7` frame 或 q7 profile；
- Assembly attachment transform、Workcell revision 或 Session route；
- Hand world root / FixedJoint / q27 materialization 方式；
- Wuji SDK 或 retarget configuration/version；
- canonical 21-point frame、side、confidence 或 calibration policy；
- merged q27 self-collision policy。

## 官方依据

- Wuji Glove，《EMF 与手部追踪》，“手部追踪产物”“HandSkeleton”
  “HandJointAngles”“输出率调节”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking)。
- Wuji Glove，《手型标定》，“何时需要标定”“SDK 用户前提”“URDF 查找顺序”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/calibration)。
- Wuji SDK，《手部重定向（Retargeting）》，“快速开始”“输入格式”“实时遥操作示例”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting)。
- Wuji Retargeting，《API 介绍》，“输入格式”“输出格式”“不同输入模式的配置”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-retargeting/latest/api)。
- Wuji Technology，《Wuji Description 集成指南》，“3.2.3 Isaac Sim USD”与
  “3.3.2 Wuji Hand 2（Beta 1）整机结构件”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-description/latest/integration.md)。
- 固定复现来源：
  [`wuji-description v2026.6.27`](https://github.com/wuji-technology/wuji-description/releases/tag/v2026.6.27)。
