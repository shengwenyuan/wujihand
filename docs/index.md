# 正式文档索引

本目录保存可版本化、可复现的项目正式文档。被 Git 忽略的 `plans/` 是本地动态记录，
不属于本索引或长期事实源。

## 当前边界

截至 2026-08-11，已验证主线是：真实 Wuji Glove + VIVE Tracker 输入，经 ROS2 与
Isaac Sim 6.0.1 驱动仿真 NERO + Wuji Hand 2，并完成遥操作采集、回放和数据导出。
这里的 HIL 指输入设备接入仿真。左右 Wuji Hand 2 Beta1 真机均已到货，但尚未在本仓库
完成真机资格验证或形成 Sim→Real 结论。历史文档中的“物理 Hand 2”若未明确写真机，指
保留刚体、碰撞和驱动的 physics-enabled 仿真资产。

默认双 NERO、ROS2、D405 与数据集业务入口仍固定为 `wuji-sdk==2026.7.21 +
wuji-description v2026.6.27`。并列的 `SDK 2026.8.3 + Description v2026.8.3` Hand2 Beta1
同版链已完成验收，可作为新场景的固定上游；这不自动切换历史默认入口。最新版双 NERO + 双
Hand2 8.3 + Glove/Tracker + ROS2 record 链已具备显式版本身份、fail-closed preflight、q54、
operator preview、rosbag、checksum 和离线验证能力，T-frame 将通过新的 Workcell/Session/
Deployment 复用该链。T-frame 首轮仍为 `qualification_only=true`、`dataset_eligible=false`，数据
资格需在新场景质量 Gate 后单独提升。Hand2 self-collision 继续关闭，少量指间穿模是已知非阻塞
限制；上述验收均未访问 NERO 或 Hand2 真机。

官方仍将 Wuji Hand 2 定义为 Beta 阶段产品；`v2026.7.23` 更改了 Hand 2 模型坐标约定、根节点、
命名、目录和碰撞语义。不能把滚动 `latest` 文档直接套入既有配置、数据集或验证结果。涉及升级时
应先读取
[Wuji Hand 2 发布记录](https://docs.wuji.tech/docs/zh/wuji-hand/latest/release-notes/)和
[Wuji Description 发布记录](https://docs.wuji.tech/docs/zh/wuji-description/latest/release-notes/)，
再建立独立迁移与回归 Gate。当前阶段一方案见
[013：Wuji SDK + Description v2026.8.3 / Hand2 Beta1 同版链阶段一计划](013-wuji-description-v2026-8-3-hand2-beta1-phase1-upgrade-plan.md)。
Hand2 Beta1 真机阶段的架构、安全 Gate、官方工具路径和未来真实遥操作采集预埋见
[015：Wuji Hand2 Beta1 真机 Bring-up、仿真解耦与遥操作采集预埋计划](015-wuji-hand2-beta1-real-hardware-bringup-and-teleoperation-data-plan.md)；
该计划从只读资格验证开始，不授权当前仿真入口访问或控制真机。

## 当前主要入口

- [项目章程与全局架构](000-project-charter-and-architecture.md)
- [五层 Session 组合](components/five-layer-session-composition.md)
- [NERO 双实例 + Hand 2 physics-enabled Isaac 仿真](components/nero-isaac-dual-twin.md)
- [ROS 2 Jazzy 双侧仿真遥操作](components/ros2-jazzy-dual-teleoperation.md)
- [ROS2—Isaac 三目 q54 mini 数据集](components/ros2-isaac-triview-q54-mini-dataset.md)
- [NERO + Hand 2 + Wuji Glove 仿真全流程](guides/nero-hand2-glove-simulation-flow.md)
- [最新确定性 ROS2—Isaac—GUI Qualification](validation/2026-08-06-deterministic-ros-isaac-gui-qualification.md)

## 全局基线与版本化计划

这些文件记录章程、路线、阶段计划与当时冻结的边界；它们不自动代表当前完成状态。

- [000：项目章程、执行预演与全局架构](000-project-charter-and-architecture.md)
- [001：NERO—VIVE 单/双臂数字孪生与真机遥操作版本计划](001-nero-vive-dual-arm-teleoperation-version-plan.md)
- [002：NV-4 原生双臂双手 Isaac 主线迭代计划](002-nv4-native-dual-arm-dual-hand-simulation-mainline-plan.md)
- [003：遥操作质量分析、ROS2 Jazzy 与 Sim→Real 计划](003-teleoperation-quality-analysis-ros2-sim2real-plan.md)
- [004：RoboLab 静态 Isaac 场景丰富化分析](004-robolab-static-isaac-scene-enrichment-analysis.md)
- [005：NV-5.1 ROS2—Isaac 60 Hz 控制迷你版本计划](005-ros2-isaac-60hz-control-feature-plan.md)
- [006：Isaac 60 Hz 到 LeRobot v3 的因果对齐与导出计划](006-isaac-60hz-lerobot-causal-alignment-plan.md)
- [007：双 NERO—Hand 2—D405 140°纯仿真集成计划](007-isaac-dual-wrist-d405-simulation-plan.md)
- [008：ROS2—Isaac 三目 q54 Mini 数据集开发计划](008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)
- [009：脚踏板、自动 Reset 与连续多 Episode 采集计划](009-pedal-auto-reset-multi-episode-mini-feature-plan.md)
- [010：确定性 ROS2—Isaac—GUI 端到端 Qualification 计划](010-deterministic-ros-isaac-gui-e2e-qualification-plan.md)
- [011：Mini 数据集完整性门禁与质量分级更新计划](011-mini-dataset-integrity-vs-quality-gate-update-plan.md)
- [012：PI05 q54 / H25 Mini 数据集过拟合开发计划](012-pi05-q54-h25-mini-overfit-development-plan.md)
- [013：Wuji SDK + Description v2026.8.3 / Hand2 Beta1 同版链阶段一计划](013-wuji-description-v2026-8-3-hand2-beta1-phase1-upgrade-plan.md)
- [014：Dual NERO T 型架 Isaac + ROS2 Record 并列场景开发计划](014-dual-nero-t-frame-isaac-ros2-record-scene-development-plan.md)
- [015：Wuji Hand2 Beta1 真机 Bring-up、仿真解耦与遥操作采集预埋计划](015-wuji-hand2-beta1-real-hardware-bringup-and-teleoperation-data-plan.md)

## Components

- [五层 Session 组合](components/five-layer-session-composition.md)
- [MediaPipe—Wuji Hand 2—Isaac 控制链路](components/mediapipe-isaac-control.md)
- [MuJoCo FR3 v2—Wuji Hand 2 桌面环境](components/mujoco-fr3-hand2-table.md)
- [NERO 双实例 + Hand 2 physics-enabled Isaac 仿真](components/nero-isaac-dual-twin.md)
- [ROS2—Isaac 三目 q54 mini 数据集](components/ros2-isaac-triview-q54-mini-dataset.md)
- [ROS 2 Jazzy 双侧仿真遥操作](components/ros2-jazzy-dual-teleoperation.md)
- [ROS2 遥操作离线质量分析器](components/teleoperation-quality-analyzer.md)
- [VIVE OpenVR 输入组件](components/vive-openvr-input.md)

## Guides

- [MediaPipe 控制 Hand 2 转向抓球指南](guides/mediapipe-hand2-rotation-ball.md)
- [MuJoCo FR3—Wuji Hand 2 桌面运行指南](guides/mujoco-fr3-hand2-table.md)
- [NERO + Hand 2 + Wuji Glove 仿真全流程](guides/nero-hand2-glove-simulation-flow.md)
- [VIVE Tracker NV-1 资格验证指南](guides/vive-tracker-qualification.md)
- [Workstation2 双 Wuji Glove 直连网络配置](guides/workstation2-dual-wuji-glove-network.md)

## Decisions

- [ADR-0001：Hand 2 固定法兰采用 D6 三轴转向](decisions/0001-hand2-fixed-flange-d6-rotation.md)
- [ADR-0002：MuJoCo 采用 FR3 v2 + Hand 2 的运行时组合](decisions/0002-mujoco-fr3v2-hand2-composition.md)
- [ADR-0003：以五层配置构建仿真资产与运行 Session](decisions/0003-five-layer-session-composition.md)
- [ADR-0004：NV-1 作为 VIVE 输入组件资格验证](decisions/0004-vive-input-component-qualification.md)
- [ADR-0005：NERO 模型来源与临时仿真限位](decisions/0005-nero-model-source-and-provisional-limits.md)
- [ADR-0006：NV-2 physics-enabled Hand 2 与 Glove 命令边界](decisions/0006-nv2-physical-hand2-glove-command-boundary.md)
- [ADR-0007：NV-4 原生双侧遥操作与 Deployment 边界](decisions/0007-nv4-native-dual-teleoperation-deployment.md)
- [ADR-0008：ROS 2 Jazzy 双侧遥操作边界](decisions/0008-ros2-jazzy-dual-teleoperation-boundary.md)
- [ADR-0009：ROS2 全因果链录制与离线分析边界](decisions/0009-ros2-full-causal-recording-boundary.md)
- [ADR-0010：D405 140°纯仿真腕部组件边界](decisions/0010-d405-140-simulation-wrist-rig-boundary.md)
- [ADR-0011：Mini 数据集因果事实与派生边界](decisions/0011-mini-dataset-causal-artifact-boundary.md)

## Validation

验证文档是带日期的历史证据。其通过结论只适用于文中锁定的代码、配置、资产和环境。

### 2026-08-06

- [确定性 ROS2—Isaac—GUI Qualification](validation/2026-08-06-deterministic-ros-isaac-gui-qualification.md)

### 2026-08-05

- [D405 双腕相机录制、Analyzer 与 60 Hz 性能验证](validation/2026-08-05-d405-recording-analyzer-performance.md)
- [双侧合成 D405 ROS2 录制资格验证](validation/2026-08-05-d405-ros-recording.md)
- [双侧 D405 static inspector 与 pause 生命周期](validation/2026-08-05-d405-static-inspector.md)
- [双侧 D405 wrist rig shared materializer / C2](validation/2026-08-05-d405-wrist-rig-c2.md)
- [双侧 D405 collision、Camera prim 与 S4 视觉验收](validation/2026-08-05-d405-wrist-rig-c3-camera.md)
- [ROS2—Isaac 三目 q54 mini 数据集无设备验收](validation/2026-08-05-ros2-isaac-triview-q54-mini-dataset.md)

### 2026-08-04

- [D405 140°纯仿真 Camera API 验证](validation/2026-08-04-d405-camera-api-spike.md)
- [D405 双腕资产生成验证](validation/2026-08-04-d405-wrist-rig-assets.md)
- [NERO—Hand 2 self-collision qualification](validation/2026-08-04-nero-hand2-self-collision.md)
- [NV-5.1 ROS2—Isaac 60 Hz 实现与 Workstation2 验证](validation/2026-08-04-nv51-ros2-isaac-60hz-local.md)

### 2026-08-03

- [ROS2 banana grasp pilot -02 审计](validation/2026-08-03-ros2-banana-grasp-pilot-02.md)
- [ROS2 banana grasp pilot -05 质量分析](validation/2026-08-03-ros2-banana-grasp-pilot-05-quality-analysis.md)
- [ROS2 banana grasp pilot -05 有序关闭复验](validation/2026-08-03-ros2-banana-grasp-pilot-05.md)
- [ROS2 全因果录制 Workstation2 无设备验证](validation/2026-08-03-ros2-full-recording-workstation2-device-free.md)

### 2026-07-31

- [NV-5 ROS 2 Jazzy Workstation2 HIL 验证](validation/2026-07-31-nv5-ros2-jazzy-hil.md)
- [RoboLab banana-in-bowl ROS Deployment 验证](validation/2026-07-31-robolab-banana-bowl-ros.md)
- [RoboLab 静态 Workcell 部署验证](validation/2026-07-31-robolab-static-workcell.md)
- [ROS2 全因果链录制离线验证](validation/2026-07-31-ros2-full-recording-offline.md)

### 2026-07-30

- [NV-5 ROS 2 Jazzy 离线验证](validation/2026-07-30-nv5-ros2-jazzy-offline.md)

### 2026-07-29

- [NV-4 Deployment 与 canonical mapping 基础验证](validation/2026-07-29-nv4-deployment-foundation.md)
- [NV-4 原生双侧实现与 Workstation2 预检](validation/2026-07-29-nv4-native-dual-preflight.md)

### 2026-07-28

- [Isaac Sim 6.0.1 / Python 3.12 起始兼容性验证](validation/2026-07-28-isaac-6.0.1-python-3.12-compatibility.md)
- [`lenovo-piper2` 目标机基线](validation/2026-07-28-lenovo-piper2-baseline.md)
- [NV-1 VIVE 最小跟踪验证](validation/2026-07-28-nv1-vive-minimal-tracking.md)
- [NV-2 NERO 双实例、physics-enabled Hand 2 与 Glove 链路阶段验证](validation/2026-07-28-nv2-nero-dual-hand2-twin.md)

### 2026-07-23

- [五层架构与既有仿真链路验证](validation/2026-07-23-five-layer-architecture.md)

### 2026-07-14

- [MuJoCo FR3 v2—Hand 2 长边侧置四棱台验证](validation/2026-07-14-mujoco-fr3-hand2-table-layout.md)

### 2026-07-13

- [Hand 2 固定法兰转向抓球验证](validation/2026-07-13-hand2-rotation-ball.md)
- [MediaPipe—Isaac 垂直切片验证](validation/2026-07-13-mediapipe-isaac-vertical-slice.md)
- [MuJoCo FR3 v2—Hand 2 桌面验证](validation/2026-07-13-mujoco-fr3-hand2-table.md)

## 维护规则

- `docs/` 是正式、版本化的长期事实源；`plans/` 不进入索引。
- `components/` 写已实现能力，`guides/` 写操作流程，`decisions/` 写重大取舍，
  `validation/` 写带版本边界的历史证据。
- 新增或删除正式文档时同步更新本索引，并检查每个 `docs/**/*.md` 文件至少被列出一次。
- 版本化计划说明目标和当时决策，不用计划状态替代组件文档或最新验证。
