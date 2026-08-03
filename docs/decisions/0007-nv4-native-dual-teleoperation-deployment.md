# ADR-0007：NV-4 原生双侧遥操作与 Deployment 边界

- 状态：已接受
- 日期：2026-07-29
- 修订：2026-07-30，Workstation2 calibration 收敛为单一无版本入口
- 影响范围：NV-4 默认运行入口、DeploymentSpec、双 Tracker/双 Glove 编排、
  relative SE(3) mapping、仿真故障恢复与 Glove confidence policy
- 上位计划：
  [002：NV-4 原生双臂双手 Isaac 主线迭代计划](../002-nv4-native-dual-arm-dual-hand-simulation-mainline-plan.md)

## 背景

当前五层 Session 已经解析双 NERO、双物理 Hand 2、四条显式 command route 和两棵
q27 articulation；但日常 runner 仍以 scripted、单侧 Glove live、右 Tracker live
等互斥分支运行。继续增加 `side`、translation/rotation、单/双输入和数值 CLI 组合，
会让运行事实脱离 Session、重复控制逻辑，并扩大无法系统验证的模式树。

NV-4 需要同时保持：

- `Asset → Binding → Assembly → Workcell → Session` 五层事实所有权；
- Tracker→Arm 与 Glove→Hand 两类 canonical 链解耦；
- 默认双侧主线和左右隔离诊断均可复验；
- 两枚 Tracker 的设备完整性与共同坐标资格分开判定；
- 当前 GUI persistent/reference rebuild 行为不因单侧输入或 IK 故障回归；
- Wuji 官方 EMF confidence 说明不被静默扩写成所有 skeleton 帧的统一拒绝规则。

## 决策一：默认主线固定为双 Tracker + 双 Glove

NV-4 默认 `native_dual_live` 要求两枚 Tracker 和两只 Glove 全部就绪，分别形成：

```text
Tracker left  -> left ArmIntent
Tracker right -> right ArmIntent
Glove left    -> left HandIntent
Glove right   -> right HandIntent
```

四条链先独立完成输入校验、映射/retarget、监督和故障决定，再在同一个 simulation
tick 组合为左右 q27 target。默认入口不提供 `--side`、tracker-only、glove-only、
translation-only、rotation-only 或数值型 live mode 分支。

保留两个 committed diagnostic DeploymentSpec：

- `left_single_live`：左 Tracker + 左 Glove live，右臂显式 hold、右手显式 rest/hold；
- `right_single_live`：右 Tracker + 右 Glove live，左臂显式 hold、左手显式 rest/hold。

默认和两个诊断 Deployment 引用同一个专用双侧 live Session 以及相同的
side-neutral application/adapter 组件。该 live Session 与既有 qualification
Session 复用同一 Assembly、Workcell、实例 Binding、root placement 和四 route
拓扑；二者只在第五层 runtime role、transport contract 与 compatibility profile
上分工。这样不必把 `simulation` Session 偷换成 teleop consumer，也不复制前四层。
非活动侧仍覆盖 Session 声明的 route，不以遗漏 route、隐藏全零 command 或 runner
特判实现。两个诊断 spec 只用于隔离诊断和 HIL 回归，不与默认双侧主线并列为产品
模式。

## 决策二：DeploymentSpec 是运行根，不是第六个资产层

多进程 live run 从 middleware-neutral `DeploymentSpec v1` 启动。每个 DeploymentSpec
恰好引用一个 ResolvedSession，并拥有：

- process graph 与 managed producer lifecycle；
- 本机 Tracker/Glove identity binding 和 stream endpoint；
- tracking setup、calibration artifact 与 transport epoch；
- report/artifact 目的地。

它不得拥有或复制 Asset、Binding、Assembly、Workcell 数值，也不得重新定义 Session
拥有的 control layout、transport contract、mapping/retarget/IK/supervisor/freshness
policy。设备地址和完整 serial 只进入忽略的 local binding；提交的 spec 使用 logical
role 和非敏感引用。`session_hash`、`deployment_hash`、`local_binding_hash` 分别记录。

本决策明确修订
[ADR-0003 第 4 节](0003-five-layer-session-composition.md)：单进程仿真仍可直接从
Session composition 启动；多进程 live run 则由 DeploymentSpec 拥有进程实例和本机
binding，Session 继续作为五层场景、传输与控制合同的唯一组合根。前四层事实所有权
不变。

实现时新增专用 live Session，而不修改已经承担 scripted qualification 的
`isaac_nero_dual_hand2_physical_simulation_nominal_v1`。这是第五层职责的显式拆分，
不是新增资产层：Deployment 仍恰好引用一个 ResolvedSession，两个 Session 的共同
Assembly/Workcell/Binding 引用由 contract test 锁定。

## 决策三：共同 tracking universe 是资格 Gate

目标拓扑是两台 SteamVR Base Station 2.0 作为共享光学参考，两枚 VIVE Tracker
(3.0) 由一个 OpenVR runtime owner、一个活动 `TrackingUniverseStanding` 和同一个
`tracking_setup_revision` 输出到规范 `vive_tracking`。Base Station 不建立
“左 Base 专属左 Tracker、右 Base 专属右 Tracker”的应用控制链。

`2 Tracker + 2 Glove` 已配置只关闭设备数量待决项，不证明坐标已经一致。NV-4B 必须
同时验证 runtime owner、universe、setup revision、frame、serial/role 和交叉运动
方向。无法证明时暂停双侧 Gate，不以 tracker-specific 轴补丁支持两套隐式
tracking world。

共同 universe 通过后，`vive_tracking` delta axes → `workcell_world` delta axes
mapping 全局共享；左右 Tracker-to-handle 外参、Tracker anchor、当前 link7 anchor、
reference epoch、Lula solver 和 supervisor 仍完全独立。两只 Glove 只输出
side-specific skeleton/q20，不要求共享空间原点，但各自 identity、frame、layout 和
calibration revision 必须可追溯。

规格依据：

- [Valve OpenVR Tracker 与 Tracking Reference 运行时语义，“设备类别”“追踪状态”](https://github.com/ValveSoftware/openvr/blob/0924064316de3effbcd1acf1e309182a2deb1c05/docs/Driver_API_Documentation.md)
- [Valve SteamVR Tracking 原理与定位器职责，“定位器与被追踪物”](https://partner.steamgames.com/vrlicensing)
- [Valve Index Base Station / SteamVR Base Station 2.0，“系统配置”](https://store.steampowered.com/app/1059570/Valve_Index_Base_Station/)
- [HTC VIVE Tracker (3.0) Developer Guidelines v1.0，“Optics”“Coordinate system”](https://developer.vive.com/documents/824/HTC_Vive_Tracker_3.0_Developer_Guidelines_v1.0_01182021.pdf)

## 决策四：Workstation2 只保留一个 canonical mapping

Workstation2 只保留 simulation-only
`configs/calibrations/vive_tracker_workcell_workstation2.yaml`。它拥有唯一
`tracker_to_workcell` proper `3×3` 旋转、translation/rotation scale 和限幅；
runner、默认双侧及左右单侧 Deployment 全部引用同一入口。

平移 scale 为 `1.0`，X/Y/Z mapping clamp 分别为 `±0.4 m`。逐轴立方包络从
reference 到角点最大位移约 `sqrt(3) × 0.4 ≈ 0.693 m`。2026-07-30
inward-port tabletop 人工观察要求 X/Y 与 roll/pitch 同时反向，因此 calibration
使用一个 `Rz(180°)` 等效 frame 修正；Z 与 yaw 保持不变。它只限制 relative
mapping 输出，不声明机械臂完整包络可达、工作台安全或真机标定有效。

超过 mapping clamp 继续返回 component-wise clamped target；包络内的不可达目标交给
现有 IK 路径。孤立 IK 失败保持最后有效目标；连续 5 次失败只撤销对应侧 reference，
下一帧合格 Tracker 输入按当前 link7 自动重建。GUI 不退出，articulation 不回初态。

XYZ-only、RPY-only 与 XYZ+RPY relative SE(3) 是三个独立资格项。当前前两类分离测试
不能证明组合路径；NV-4A/C 必须执行真人复合轨迹并记录 clamp、target step/rate、
IK failure、limit margin 和 reference rebuild。

## 决策五：仿真故障按侧隔离

NV-4 使用以下默认故障矩阵：

| 故障 | 行为 |
|---|---|
| 单侧 Tracker 暂时 `calibrating/out_of_range` | 对应臂 hold，不刷新 freshness |
| 单侧 Tracker stale/断开或 IK 连败 | 只撤销对应臂 reference；恢复后自动重建 |
| 单侧 Glove stale/retarget 失败 | 只处理对应 Hand 2 hold/return-to-rest |
| managed producer restart | 为所属 stream 建立新 transport epoch，拒绝旧 epoch 数据 |
| 未管理的 sequence 回退 | fail closed，不自动接纳 |
| SteamVR universe/setup/global mapping 改变 | 双臂共同暂停并使旧 calibration/reference 失效 |
| Session/Isaac/backend invariant 失败 | 终止本次 control run |

该策略只适用于纯仿真资格与诊断。真机双臂仍使用后续 ADR 冻结的 coupled
deadman/disarm，不从仿真 side-local policy 外推。

## 决策六：修订 Glove confidence policy

Wuji 官方《EMF 与手部追踪》在 `EmfPose.confidence` 字段说明低于 `0.9` 可认为
不可靠；该说明没有为 `HandSkeleton` 的 21 个 landmark 定义统一硬拒绝阈值：
[官方页面](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking/)。

NV-4 保留已经由左右 live 证据使用的行为：

- missing、stale、错误 side/frame/layout、NaN 或 retarget failure 继续拒绝；
- 完整、finite 且最低 landmark confidence 不低于配置 floor 的帧可进入 retarget；
- 默认 floor 为 `0.0`，最低 confidence `<0.60` 产生 `DEGRADED` intent；
- 最低 confidence `>=0.60` 产生 `SUCCESS` intent；该标签阈值由 2026-08-03
  抓放 pilot 校准，旧运行使用的 `0.90` 作为历史 provenance 保留；
- `DEGRADED` intent 仍必须经过同一 JointCommandSupervisor，不等同于绕过监督。

本节明确 supersede
[ADR-0006](0006-nv2-physical-hand2-glove-command-boundary.md) 中
“`<0.90` 一律拒绝、`[0.90,0.95)` degraded、`>=0.95` success”的阈值段落。
ADR-0006 的 21×3→q20、side/layout、freshness、finite、retarget reset 和四 route
边界继续有效。

## 未采用方案

| 方案 | 原因 |
|---|---|
| 默认 runner 继续增加 side/mode/numeric flags | 形成隐式运行事实和组合爆炸 |
| 为左右单侧复制 runner | 重复 application/Isaac 生命周期，难以保证恢复行为一致 |
| 省略非活动侧 Session route | 破坏四组显式 commanded 与单一 Session 合同 |
| 把设备 serial、端口和 process graph 写入 Workcell/Session | 污染五层资产与场景事实 |
| 把两台 Base 分配给左右 Tracker | 与共享 tracking reference 拓扑和单一 frame Gate 冲突 |
| 为坐标不一致增加每 Tracker 临时轴补丁 | 掩盖 tracking universe/setup 资格失败 |
| 并行保留多个 Workstation2 mapping 文件 | 日常入口和 provenance 分裂，容易让不同启动模式引用不同坐标语义 |
| 只测试 XYZ-only 与 RPY-only | 不能证明组合 SE(3) 的 IK、限位和恢复行为 |
| 把 EMF `0.9` 说明扩写为 skeleton 统一拒绝线 | 数据产品和字段语义不同，且与现有 live 证据冲突 |

## 验证责任

- 无 backend fast tests：DeploymentSpec strict fields、引用闭合、local binding 分离、
  stable hash、默认双侧与左右单侧诊断模板。
- mapping tests：单一文件约束、proper rotation、1:1、逐轴 `±0.4 m`、方向、
  component clamp、组合 translation+rotation 和 provenance。
- application tests：左右独立 reference/mapper/solver/supervisor、单侧故障隔离、
  连续 5 次 IK 失败、current-pose rebuild 和旧 epoch 拒绝。
- Glove tests：左右 identity、双连接、低 confidence degraded、错误/缺失/NaN 拒绝、
  同侧 hold/rest 与对侧隔离。
- HIL：共同 OpenVR runtime/universe/setup revision、双 Tracker 交叉轨迹与遮挡、
  左右 XYZ-only/RPY-only/组合 SE(3)、单侧 arm+hand、默认四流和 GUI persistent。
- 每次运行记录 Session、DeploymentSpec、local binding、tracking setup、calibration、
  source 和 artifact hash；正式报告不得泄漏完整设备 serial。

## 后果与重验触发器

主 runner 的日常认知面和 CLI 分支可以显著下降，左右控制复用相同组件，单侧诊断仍有
明确入口；代价是必须新增严格 DeploymentSpec/runtime composition，并为共同 tracking
universe、双设备生命周期和四流故障建立独立资格证据。

下列变化必须重新对齐本 ADR 并重跑受影响 Gate：五层/DeploymentSpec 事实所有权、
默认设备完整性、单侧诊断语义、tracking universe/frame、canonical mapping 数值或 scope、
IK rebuild 次数、仿真 fault coupling、Glove confidence 阈值、四流 command ownership。
