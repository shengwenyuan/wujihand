# 014：Dual NERO T 型架 Isaac + ROS2 Record 并列场景开发计划

- 状态：正式初版；STEP→USD candidate 已完成，场景 mount、startup、ROS2 record 与 GUI
  qualification 待实施
- 日期：2026-08-11
- 目标平台：Workstation2 / Isaac Sim 6.0.1 / ROS2 Jazzy
- 机器人组合：双 NERO + 双 Wuji Hand 2 Beta1 v2026.8.3 + 双腕 D405 仿真组件
- 场景关系：T-frame 与现有 tabletop / RoboLab 入口并列，互不覆盖
- 真机边界：本计划只控制仿真资产；不授权 NERO 或 Hand2 真机发现、使能或运动

上游文档：

- [013：Wuji SDK + Description v2026.8.3 / Hand2 Beta1 同版链阶段一计划](013-wuji-description-v2026-8-3-hand2-beta1-phase1-upgrade-plan.md)
- [008：ROS2—Isaac 三目 q54 Mini 数据集开发计划](008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)
- [010：确定性 ROS2—Isaac—GUI 端到端 Qualification 计划](010-deterministic-ros-isaac-gui-e2e-qualification-plan.md)
- [ADR-0005：NERO 模型来源与临时仿真限位](decisions/0005-nero-model-source-and-provisional-limits.md)
- [ADR-0009：ROS2 全因果录制边界](decisions/0009-ros2-full-causal-recording-boundary.md)

## 0. 结论

最新版 ROS2 双臂双手 record 链可以接入 T-frame 场景，且不需要复制控制器、ROS source、
调度器、q54 合成器、录制器或数据导出实现。正确路线是复用已验收的
`SDK 2026.8.3 + Description v2026.8.3` Assembly 与 record 执行链，只新增 T-frame 的
Workcell、startup/geometry qualification、Session、Deployment、record-chain policy 和独立
Tracker→Workcell calibration。

当前适配不是纯配置替换。主 Isaac runner、独立 20 Hz preview 和
`DualNeroHand2IsaacScene` 仍从 `NeroDualTabletopQualificationProfile` 同时读取初始 q7、Isaac
drive 与桌台几何事实；必须先把 scene-neutral startup 从 tabletop geometry 中最小拆出。完成该
拆分后，record、q54、D405 和 preview 的主体实现均可原样复用，整体属于中等适配，不是新建一套
遥操作或采集栈。

场景组合固定为：

```text
T-frame wrapper USD
  -> visual.usdc + collision.usdc
  -> T-frame Workcell（静态设施、ground、mount、camera frames）

已验收双 NERO + 双 Hand2 8.3 + 双腕 D405 Assembly
  + T-frame Workcell
  + T-frame startup / geometry qualification
  -> T-frame Session
  -> T-frame ROS2 Deployment
  -> 现有 120/60/20 record 链
```

operator GUI 改动只发生在独立 20 Hz preview 进程：先把纯色背景由
`[0.12, 0.12, 0.12]` 覆盖为 `[0.30, 0.30, 0.30]`；只有背景调整后仍有明显阴影遮挡时，才在该
进程关闭阴影和环境遮蔽，同时保留柔和环境光。不得修改共享 Workcell 灯光、主 Isaac 进程、
D405 capture 或离线三视角 renderer。

## 1. 已冻结的上游与资产基线

### 1.1 Hand2 与 record 上游

项目负责人已确认 `wuji-sdk==2026.8.3 + wuji-description v2026.8.3` 的 Hand2 Beta1 同版链
完成验收。本计划直接消费以下已验收事实，不再重复迁移 Hand2：

- 左右 Hand2 Description revision 均为 `beta1_description_v2026_8_3`；
- Description root 为 `l_wrist` / `r_wrist`，并保留已验收的左右 root orientation
  compensation；
- SDK process、具名用户左右校准 URDF、Glove serial 与 matched-chain preflight 继续
  fail closed；
- 双 NERO→Hand2 attachment 与双腕 D405 Assembly 继续使用已验收版本；
- q54 顺序继续是 left arm q7 + left hand q20 + right arm q7 + right hand q20；
- Hand2 self-collision 继续关闭，已知少量极端姿态指间穿模不由 T-frame 需求修正；
- Hand2 仍为 Beta1；SDK、Description、用户校准 URDF 或官方模型任一变化都会使本计划的上游
  qualification 失效。

这表示新版数字孪生与输入/record 软件链可复用，不表示 Hand2 真机、NERO 真机或 Sim→Real 已
验证。

### 1.2 STEP 与三份 USD candidate

原始受限 STEP：

```text
/home/lenovo/下载/dual_nero_Tframe.STEP
size_bytes = 136148726
sha256     = 3cf8f2388649de17470e67db920077c8b6cd778cf80c2536d42f3ccacd0a7dc5
```

canonical ignored source 位于：

```text
third_party/src/restricted/dual-nero-tframe/
  sha256-3cf8f2388649de17470e67db920077c8b6cd778cf80c2536d42f3ccacd0a7dc5/
    dual_nero_Tframe.STEP
```

Workstation2 已生成 candidate：

```text
artifacts/derived/isaac/6.0.1/dual-nero-tframe/v1/
  candidate-20260811-v1-a/
```

三份运行相关 USD 为：

| 文件 | 本次 build SHA-256 | 角色 |
|---|---|---|
| `tframe.usda` | `0bc8d7446b026ccdb819a3ce124e516b208e6c2f8416232ba87572e503eea6af` | `/TFrame` wrapper，使用相对引用组合 visual/collision |
| `tframe_visual.usdc` | `ed6eade87adc2d42a4b45d2c75a64df1c67b5c07276add84966f71b6c844cda8` | 双面 visual mesh |
| `tframe_collision.usdc` | `7008af6c8f9155a6c6863a08fa86f86adbfd8c842d3fd0aa6fb5fa894dc65525` | 静态碰撞代理 |

已通过的 candidate 事实：

- Z-up、`metersPerUnit=1.0`、default prim `/TFrame`；
- 只保留 7 个承力 `M-BIMA` occurrence，不含两台 STEP 内置 NERO、电源、线缆和紧固件；
- 42 个静态 collider：34 个 convex hull + 8 个 static triangle mesh；
- 0 个 rigid body，未把整架压成单一凸包；
- wrapper 无绝对资产路径，OpenUSD 25.11 / Isaac Sim 6.0.1 可打开；
- inventory identity 为
  `6640a895f8306afa0ca29635bbe54abae31f43055f55fb22e3868dc1402ce359`；
- semantic identity 为
  `2a639aad11d8e22bf3395d9573fcd69dbd8d2ab2a4d9aec09a4e6d17ff2232d9`；
- 两次独立构建的 inventory/semantic identity 相同；六个 canonical USD/manifest/preview 输出字节
  相同；cleaned STEP 因 writer header/timestamp 不要求字节相同。

candidate 当前状态仍是 `generated_unqualified_simulation_only`。以下四项均未完成，不能绕过：

- STEP witness 的 left/right 语义未确认；
- STEP 中 NERO witness local origin/axes 到 pinned `base_link` 的关系未确认；
- Workcell mount 尚未接受；
- 没有任何真机授权。

### 1.3 Git 与派生资产边界

136 MB 原 STEP 不进入 GitHub，也不使用 Git LFS。Git 只管理 source pointer、selection、recipe、
导入工具、配置、测试与报告摘要；受限原 CAD 和完整派生几何保留在 ignored source/artifact 目录。

正式 Workcell 接入前必须显式提升本次 candidate：

1. source-lock 记录 raw STEP 身份、权限状态与 canonical local path；
2. generated source-lock 记录 wrapper、visual、collision 的本次精确 SHA；
3. 同时记录 raw、inventory、selection、generator、recipe、toolchain 和 semantic identity；
4. 运行时验证 wrapper 及两个相对依赖均存在且 hash 匹配；
5. 另一次语义等价构建不得在同一 generated revision 下静默替换。

## 2. ROS2 Record 链迁移评估

### 2.1 可直接复用

以下实现应保持一份，不建立 T-frame 副本：

- VIVE Tracker 与 Wuji Glove ROS2 source nodes；
- 四条 source→route control binding；
- `DualTeleoperationCycle`、IK、retarget、supervisor 与 stale/hold/fail-closed policy；
- 双侧 q27 执行与 q54 合成；
- 120 Hz physics / 60 Hz control / 20 Hz operator-preview 调度；
- rosbag2 recorder、run lifecycle、manifest、receipt、checksum 与 orderly shutdown；
- q54 causal alignment、离线 renderer、LeRobot 导出与质量分析；
- 已验收的双 NERO + 双 Hand2 8.3 + 双腕 D405 Assembly 和 Bindings；
- `isaac_nero_hand2_triview_q54_mini_dataset_v1` 的 joint/camera/timing 合同；
- `wuji_hand2_record_chain` 的 SDK/Description/Assembly/Deployment 精确身份 preflight 机制。

record-chain policy 已把 Deployment 与 Assembly 做成显式 ConfigRef，因此 T-frame 只需新增第二份
policy，指向 T-frame Deployment 与同一个已验收 Assembly；不需要复制 preflight 代码。

### 2.2 必须新增

- T-frame generated static USD profile；
- T-frame Workcell 与 ground、mount、camera semantic frames；
- T-frame scene-neutral startup profile；
- T-frame geometry qualification profile；
- T-frame qualification / record Session；
- T-frame ROS2 Deployment 与独立 report root；
- T-frame record-chain qualification policy；
- Tracker→T-frame calibration，不继承 tabletop provenance；
- T-frame scene fixture、resolver、Isaac smoke、record 与回归证据；
- operator-preview-only visual policy/readback receipt。

### 2.3 必须最小重构

当前硬耦合集中在四处：

1. `NeroDualTabletopQualificationProfile` 同时拥有 initial q7、Isaac drive 和 tabletop geometry；
2. `DualNeroHand2IsaacScene` 构造参数固定为上述类型；
3. 主 ROS runner 与 preview runner 都固定调用 tabletop loader；
4. 当前 control profile 的 `base_qualification` 精确指向 tabletop profile。

目标拆分：

```text
NeroDualSimulationStartupProfile
  initial q7 by exact instance/group/layout
  Isaac-only drive gains
  reset / settle policy

TabletopGeometryQualificationProfile
  table / palm / forearm / port direction gates

TFrameGeometryQualificationProfile
  mount / hanging-L / frame clearance / ground gates
```

旧 tabletop profile 可由严格兼容 adapter 继续解析，旧 Session/hash/default 不变。T-frame Session
不得 fallback 到 tabletop geometry，也不得复制 freshness、IK、retarget、supervisor 或调度数值。

### 2.4 适配量判断

主要工作量落在配置组合、startup 类型拆分、mount/姿态 qualification 和场景回归；record、ROS2
source、q54、D405 与数据产物实现本身不需要大规模改写。只要 mount/frame 资格闭合，这是一项中等
适配；若 STEP witness 无法与 pinned `base_link` 对齐，阻塞点是机械/坐标事实，不应通过增加代码
绕过。

## 3. T-frame 场景组合

### 3.1 五层归属

```text
Asset / Binding
  NERO、Hand2 8.3、D405 与 generated T-frame USD 的来源和 backend 表示

Assembly
  双 NERO roots
  NERO link7 -> Hand2 hand_base
  Hand2 -> D405 wrist-rig passive attachments

Workcell
  T-frame、ground、mount、camera semantic frames、共享数据渲染灯光

Session
  Assembly + Workcell + placements + startup/geometry + dataset profile

Deployment
  Glove/Tracker sources、进程图、唯一 execution owner、record/report root
```

T-frame 不进入 robot Assembly。若把它设为 Assembly root，会破坏 arm-root placement、双侧 runtime
解析、D405 passive instance 与 Session root-placement contract。

### 3.2 Workcell 内容

T-frame Workcell 至少定义：

- `world`；
- `tframe_origin`；
- `tframe_ground_contact`；
- `tframe_shoulder_center`；
- `nero_left_mount` / `nero_right_mount`；
- operator/scene/top/right-interface camera eye/target logical frames；
- project-owned ground plane；
- source-locked `tframe.usda` reference。

最终 stage 必须满足：

- 只有两台 Session NERO，不存在 STEP duplicate NERO prim；
- T-frame 只有静态 collider，没有 dynamic rigid body；
- 无 table/table_top 或 tabletop geometry qualification 引用；
- wrapper 的 visual/collision 相对依赖闭合；
- 两台 NERO root placement 精确落在已接受 mount；
- 初始姿态下 T-frame、地面、NERO、Hand2、D405 无穿透。

### 3.3 mount 资格边界

STEP witness 中点坐标系给出的 `±0.104 m` 只保留为 inspection candidate。升级为 Workcell mount
必须闭合：

```text
供应方/安装说明的物理左右关系
  <-> STEP NAUO37 / NAUO38 witness
  <-> pinned NERO mesh 安装盘面与 clocking
  <-> pinned URDF/USD base_link
  <-> Workcell nero_left_mount / nero_right_mount
```

任何 mount 值都必须标记为 CAD-derived、source-derived、人工确认或 simulation assumption。没有
证据时不得靠截图、视觉居中或试转关节冻结。

### 3.4 hanging-L 初始姿态

“双臂自然下垂并成 L 型”通过 pinned NERO URDF 的 FK/受限搜索冻结，不手填截图角度。至少验证：

- 上臂主轴总体朝 world down；
- 肘部弯折接近直角；
- 前臂朝明确的目标工作区；
- 左右掌面与手指方向明确；
- 两侧由各自 mount frame 求解，不预设关节向量简单镜像；
- 关节相对 pinned simulation limits 保留余量；
- T-frame、地面、双臂双手和 D405 零初始穿透；
- 10 次 reset/settle 后 q7 feedback、漂移和碰撞状态可重复。

该 q7、drive 和 mount 只属于当前 pinned NERO 资产、当前 T-frame revision 与 Isaac Sim 6.0.1 的
simulation nominal。它们不是 NERO 真机安全姿态或真机限位。

## 4. D405、q54 与录制合同

### 4.1 保持不变的合同

T-frame record Session 继续引用同一 q54/dataset profile，不复制或改写：

- q54 名称、顺序、单位和 action semantics；
- 120/60/20 调度与 30 fps dataset selection；
- observation/action 因果相位；
- rosbag allowlist、QoS、run/episode lifecycle、receipt 和 checksum；
- D405 logical camera IDs、profile、内参、分辨率、投影、frame truth 与 payload whitelist；
- D405 wrist-rig attachment、camera optical frame 和离线 renderer 路径；
- LeRobot schema、fps 与 action semantics。

T-frame Session 的 manifest 必须额外记录新 Workcell path/hash、generated asset revision、mount
revision、startup profile 和 scene qualification identity。

### 4.2 “D405 输出不变”的精确定义

dataset 模式的 D405 RGB 由离线三视角 renderer 生成，不把在线 wrist-camera 图像 topic 混入原始
rosbag。T-frame 场景与 tabletop 几何不同，因此相机看到的像素内容按设计会不同；不能要求跨
Workcell 图像字节相同。

本计划中的“不变”是：

- operator-preview 背景/阴影设置不进入主 Isaac、D405 capture 或离线 renderer；
- 对同一 T-frame scene state，preview 开/关或 preview 视觉策略变化不能改变 q54、camera profile、
  frame selection、projection、frame truth、payload checksum 与导出结构；
- 不新增或删除 D405 数据字段，不改变 30 fps 选择和 q54 对齐；
- 如果只切 preview 策略，主录制的 canonical semantic digest 与离线 D405 payload digest 必须保持
  一致。

### 4.3 首轮资格状态

T-frame 新入口先固定为：

```text
qualification_only = true
dataset_eligible   = false
```

完成 scene、mount、时序、D405、动作覆盖与数据质量 Gate 后，再单独提升为可采集入口。场景能启动、
q54 能写盘或 GUI 好看都不足以证明数据可用于训练。

## 5. Operator-preview 视觉优化

### 5.1 隔离原则

当前 Workcell 背景实际为 `[0.12, 0.12, 0.12]`，视觉接近黑色。调整只作用于独立的
`run_isaac_dataset_live_preview.py` 进程；主 120/60 Hz Isaac consumer 仍 headless，Workcell 的
selected HDR、背景和离线数据渲染灯光保持原值。

preview override 应在场景 materialize 完成后应用，因为 Workcell materialization 会先写入共享
背景值。override 使用进程内 Kit/RTX settings，不写回 USD、不修改 Workcell profile，也不进入
MCAP 或 dataset manifest 的数据渲染身份。

### 5.2 两步顺序

第一步：只改背景。

```text
operator_preview.background_color_rgb = [0.30, 0.30, 0.30]
operator_preview.shadows              = unchanged
operator_preview.ambient_occlusion    = unchanged
```

先确认黑色 Hand2、NERO 和 T-frame 轮廓可辨，并跑完整时序/像素 Gate。纯色背景不增加场景几何，
预计性能风险低。

第二步：只在第一步仍有明显手臂阴影遮挡时启用。

```text
operator_preview.background_color_rgb = [0.30, 0.30, 0.30]
operator_preview.shadows              = disabled
operator_preview.ambient_occlusion    = disabled
operator_preview.soft_environment     = preserved
```

关闭项必须使用 Isaac Sim 6.0.1 中可读回、可测试的稳定设置。若设置无法可靠 readback，或关闭后
Hand2 失去立体层次，则不合入第二步；背景优化可以独立交付。

### 5.3 receipt 与验收

preview receipt 增加或等价记录：

- visual policy identity/hash；
- requested/read-back background color；
- requested/read-back shadow/AO state；
- environment light 保留状态；
- renderer/minimal shading mode；
- effective render rate、missed periods、render mean/max；
- viewport baseline/motion/return 像素 Gate。

硬门：

| 指标 | 要求 |
|---|---:|
| physics | 120 Hz，保持现有合同 |
| control | 60 Hz，0 missed period |
| operator GUI | 20 Hz ±5%，0 missed period |
| 单帧 render | `< 50 ms` |
| q54 closure | 保持现有阈值与顺序 |
| recording | topic/schema/lifecycle/checksum 无变化 |
| D405 | profile、frame truth、离线 payload digest 不受 preview 策略影响 |
| authority | preview 无 control authority、无本地 physics、不得进入 MCAP |

## 6. 实施工作包与 Gate

### T0：冻结已验收上游

- 固定 Hand2 8.3 matched-chain、record Assembly、dataset profile 与当前 runner revision；
- 保存旧 tabletop/RoboLab resolved snapshot、默认 Deployment 与 smoke oracle；
- T-frame 不改旧入口的默认选择和事实。

Gate：上游身份可复现；旧入口 oracle 可重复。

### T1：提升 T-frame generated asset

- 提交 import tool、selection、recipe 和 contract tests；
- 增加 restricted raw pointer 与 generated artifact source-lock；
- 校验 wrapper/visual/collision 三文件 closure；
- 保留 candidate semantic/inventory identity 与单次 build hash；
- 普通 Mac/CI 缺少 restricted source 时只跳过资产重建，不影响旧场景测试。

Gate：错误 source、缺少相对依赖、hash 漂移、绝对路径或 duplicate NERO 全部 fail closed。

### T2：mount 与 Workcell

- 完成 witness→pinned mesh→`base_link` 对齐；
- 确认 left/right、clocking、安装盘面和 ground relation；
- 建立 T-frame Workcell、mount、ground 与 camera frames；
- 新 profile 复用当前数据渲染 HDR 身份和值，不修改旧 Workcell profile。

Gate：mount 可追溯，T-frame 静态，stage 无 table 或 duplicate NERO。

### T3：scene-neutral startup 与 scripted scene

- 拆分 startup 与 tabletop geometry；
- 保持旧 tabletop compatibility 和全部回归；
- 求解并冻结 hanging-L q7；
- 新增 T-frame scripted qualification Session；
- 完成 10 次 reset/settle、关节余量、双 q27 隔离与零穿透测试。

Gate：T-frame 不读取 tabletop geometry；旧 scene hash/行为不变。

### T4：ROS2 record 并列入口

- 新增 T-frame dataset Session、ROS2 Deployment 与 record-chain policy；
- 复用同一 Assembly、q54/dataset profile、四 routes 和 execution owner；
- 新建 Tracker→T-frame calibration；
- report root、run manifest 和 qualification identity 与旧场景隔离。

Gate：不得复制 runner/source/recorder；preflight 在设备连接和 Isaac 启动前闭合全部版本与场景身份。

### T5：device-free record qualification

- 运行固定 A→B→A 或等价 scene fixture；
- 验证完整 q54、scene truth、preview topic、rosbag、checksum 和有序关闭；
- 验证 D405 offline-render inputs、camera frames 与 scene manifest 完整；
- 对旧场景和 T-frame 各跑一次 resolver/record 回归。

Gate：120/60/20 全部通过，0 schedule miss；T-frame 失败不影响旧入口。

### T6：operator-preview 背景

- 在独立 preview 进程应用 `[0.30, 0.30, 0.30]`；
- 增加 process-local readback 与 receipt；
- 对 preview override 开/关做主录制与 D405 不变性比较；
- 完成 GUI 像素、运动、回归和性能 Gate。

Gate：背景改善可辨识度，且 control/GUI/render/q54/record/D405 全部满足第 5.3 节。

### T7：可选阴影/AO 关闭

- 只在 T6 后仍有明确遮挡证据时实施；
- 只改 preview 进程，保留环境光；
- 记录设置 readback、前后截图和性能差异；
- 对主录制/D405 再跑一次不变性比较。

Gate：轮廓更清晰且不丢失手部立体感；任何不确定 readback、黑手退化或时序失败都撤回 T7，保留
T6。

### T8：live qualification 与数据资格提升

- 先短时双 Glove + 双 Tracker live qualification；
- 检查双臂抖动、双手动作范围、对指、抓取意图、stale/clamp/hold/reject 与操作者可视性；
- 完成 replay、q54、D405、checksum、时间对齐和数据质量报告；
- 单独评审是否从 `dataset_eligible=false` 提升。

Gate：质量不足时保留 qualification-only，不用增加 episode 数量掩盖场景、控制或动作覆盖问题。

## 7. 验证矩阵

| 维度 | 旧 tabletop / RoboLab | 新 T-frame | 必须证明 |
|---|---|---|---|
| Assembly | 已验收 Hand2 8.3 record Assembly | 同一 Assembly | NERO/Hand2/D405 未复制或漂移 |
| Workcell | 旧配置不变 | T-frame + ground | 两入口独立解析与失败 |
| robot placement | tabletop mounts | accepted shoulder mounts | left/right/base_link 证据闭合 |
| startup | tabletop q7/geometry | hanging-L/startup + T-frame geometry | startup 与几何已解耦 |
| q54 | 固定 54 joints | 相同 profile/order | exact identity 与 closure |
| ROS routes | 四 routes | 同四 routes | source/route/control facts 相同 |
| recording | 现有 lifecycle/schema | 同一实现 | manifest 只新增场景身份 |
| D405 | 现有 profile/renderer | 同 profile/renderer | preview 不改变 D405 合同/同场景 payload |
| GUI | 当前外部 preview | 中灰背景；可选无 shadow/AO | 20 Hz ±5%、0 miss、render <50 ms |
| data status | 历史身份不变 | 初始 qualification-only | 未通过质量 Gate 不提升 |
| restricted CAD | 不依赖 | 缺源时 fail closed | 不影响普通 CI/旧场景 |
| 真机 | 不授权 | 不授权 | 无硬件访问 |

## 8. 预期文件影响

名称在实施时按最新代码 rebase，但职责边界固定。

新增候选：

- `configs/profiles/isaac_dual_nero_tframe_static_usd_v1.yaml`；
- `configs/workcells/isaac_nero_dual_hand2_tframe_v1.yaml`；
- `configs/profiles/isaac_nero_dual_simulation_startup_v1.yaml`；
- `configs/profiles/isaac_nero_dual_tframe_geometry_qualification_v1.yaml`；
- `configs/sessions/isaac_nero_dual_hand2_tframe_triview_q54_v2026_8_3_v1.yaml`；
- `configs/deployments/isaac_nero_hand2_ros_dual_tframe_triview_q54_v2026_8_3_v1.yaml`；
- `configs/qualifications/isaac_nero_hand2_tframe_record_chain_v2026_8_3_v1.yaml`；
- T-frame Tracker calibration 与 scene/record validation report；
- operator-preview visual policy；若实现为 preview 全局隔离默认，则不扩张 Workcell/Session schema。

最小代码触点：

- tabletop qualification loader：拆出通用 startup；
- `DualNeroHand2IsaacScene`：消费 scene-neutral startup；
- 主 ROS runner：按 resolved Session 加载 startup/geometry；
- 独立 preview runner：消费同一 startup，并应用 process-local visual override；
- launch/resolver：只有显式传递 preview policy 时才需小幅扩展；
- 对应 unit/contract/Isaac qualification tests。

不应修改：

- pinned NERO URDF/USD 关节树、origin、axis 或 limits；
- 已验收 NERO→Hand2 8.3 attachment；
- Glove/Tracker source、IK、retarget、supervisor、q27/q54 或 recorder 实现；
- D405 Asset/Binding/profile、camera contract 或离线 renderer；
- 旧 tabletop/RoboLab Workcell 灯光、mount、q7、Session、Deployment 和历史 artifact；
- Pi0.5 训练链。

## 9. 风险与停止边界

| 风险 | 判断 | 处理 |
|---|---|---|
| witness 与 `base_link` 不闭合 | 高风险点 | 停在 mount qualification，不猜 transform |
| left/right 或 clocking 未确认 | 高风险点 | 不创建可运行 placement |
| T-frame collider 过度保守 | 中 | 做局部 clearance/contact 验证，不改机器人 collider 掩盖 |
| startup 仍夹带 tabletop geometry | 中 | T-frame strict test 禁止 fallback/table 引用 |
| Tracker mapping 继承旧 provenance | 中 | 新 calibration 与新 report identity |
| preview 设置泄漏到主渲染 | 中 | 进程隔离 + A/B digest test |
| shadow/AO setting 无稳定 readback | 中 | 不实施 T7，只保留中灰背景 |
| record 可运行但动作覆盖差 | 高数据风险 | 保持 qualification-only，先修质量再扩 episode |
| 受限 CAD/派生资产误提交 | 中 | ignore + pre-commit/contract 检查 |
| 把仿真 q7/mount 称为真机事实 | 高安全风险 | 明确 simulation-only；真机另行 readback/授权 |

真实 NERO 运动前仍必须逐台确认序列号、固件、零位、符号、限位、安装朝向和供应方安全说明，并由
现场人员明确授权。本计划的 Isaac 验收不能替代这些步骤。

## 10. Definition of Done

只有同时满足以下条件，本计划才完成：

1. raw STEP 不进入 Git/Git LFS；source/permission/generated identity 可审计；
2. `tframe.usda`、visual、collision 三文件被显式提升并完整校验；
3. 最终 stage 不含 STEP duplicate NERO、电气杂件或 dynamic T-frame body；
4. left/right、mount、clocking、ground 和 `base_link` 对齐拥有可追溯证据；
5. T-frame Workcell、Session、Deployment 与旧入口并列且独立失败；
6. startup 从 tabletop geometry 解耦，旧 tabletop 行为和默认入口不变；
7. hanging-L q7 通过 FK、碰撞、关节余量与 10 次 reset/settle Gate；
8. 同一已验收 Hand2 8.3 Assembly、q54 profile、ROS routes 和 record 实现被复用；
9. T-frame Tracker mapping 独立，不继承 tabletop provenance；
10. device-free record 通过 120/60/20、q54、scene truth、rosbag、checksum、replay 与 orderly
    shutdown；
11. operator-preview 背景为 `[0.30, 0.30, 0.30]`，control 60 Hz/GUI 20 Hz/单帧 render 门通过；
12. 阴影/AO 只有在必要且可 readback、可回归时关闭，否则明确保持默认；
13. preview visual policy 不改变共享 Workcell 灯光、q54、record 或 D405 同场景输出；
14. T-frame 首轮始终 `qualification_only=true`、`dataset_eligible=false`；提升另有质量证据；
15. 所有 q7、drive、mount 和碰撞结论标记为当前固定来源下的 simulation-only；
16. 没有访问、使能或运动 NERO/Hand2 真机。
