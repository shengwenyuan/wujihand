# 正式文档索引

本目录保存可版本化、可复现的项目正式文档。动态执行计划不放在这里，而放入被 Git 忽略的 `plans/`。

## 当前文档

- [000：项目章程、执行预演与全局架构](000-project-charter-and-architecture.md)
- [001：NERO—VIVE 单/双臂数字孪生与真机遥操作版本计划](001-nero-vive-dual-arm-teleoperation-version-plan.md)
- [002：NV-4 原生双臂双手 Isaac 主线迭代计划](002-nv4-native-dual-arm-dual-hand-simulation-mainline-plan.md)
- [五层 Session 组合](components/five-layer-session-composition.md)
- [NERO 双实例 + 物理 Hand 2 Isaac 数字孪生（NV-2 部分完成）](components/nero-isaac-dual-twin.md)
- [VIVE OpenVR 输入组件](components/vive-openvr-input.md)
- [MediaPipe—Wuji Hand 2—Isaac 控制链路](components/mediapipe-isaac-control.md)
- [MuJoCo FR3 v2—Wuji Hand 2 桌面环境](components/mujoco-fr3-hand2-table.md)
- [ADR-0001：Hand 2 固定法兰采用 D6 三轴转向](decisions/0001-hand2-fixed-flange-d6-rotation.md)
- [ADR-0002：MuJoCo 采用 FR3 v2 + Hand 2 的运行时组合](decisions/0002-mujoco-fr3v2-hand2-composition.md)
- [ADR-0003：以五层配置构建仿真资产与运行 Session](decisions/0003-five-layer-session-composition.md)
- [ADR-0004：NV-1 作为 VIVE 输入组件资格验证](decisions/0004-vive-input-component-qualification.md)
- [ADR-0005：NERO 模型来源与临时仿真限位](decisions/0005-nero-model-source-and-provisional-limits.md)
- [ADR-0006：NV-2 物理 Hand 2 与 Glove 命令边界](decisions/0006-nv2-physical-hand2-glove-command-boundary.md)
- [ADR-0007：NV-4 原生双侧遥操作与 Deployment 边界](decisions/0007-nv4-native-dual-teleoperation-deployment.md)
- [NERO + Hand 2 + Wuji Glove 仿真全流程透明说明](guides/nero-hand2-glove-simulation-flow.md)
- [MediaPipe 控制 Hand 2 转向抓球指南](guides/mediapipe-hand2-rotation-ball.md)
- [VIVE Tracker NV-1 资格验证指南](guides/vive-tracker-qualification.md)
- [Workstation2 双 Wuji Glove 直连网络配置](guides/workstation2-dual-wuji-glove-network.md)
- [MuJoCo FR3—Wuji Hand 2 桌面运行指南](guides/mujoco-fr3-hand2-table.md)
- [2026-07-28 Isaac Sim 6.0.1 / Python 3.12 起始兼容性验证](validation/2026-07-28-isaac-6.0.1-python-3.12-compatibility.md)
- [2026-07-28 lenovo-piper2 目标机基线](validation/2026-07-28-lenovo-piper2-baseline.md)
- [2026-07-28 NV-1 VIVE 最小跟踪验证](validation/2026-07-28-nv1-vive-minimal-tracking.md)
- [2026-07-28 NV-2 NERO 双实例、物理 Hand 2 与 Glove 链路阶段验证](validation/2026-07-28-nv2-nero-dual-hand2-twin.md)
- [2026-07-29 NV-4 Deployment 与 canonical mapping 基础验证](validation/2026-07-29-nv4-deployment-foundation.md)
- [2026-07-29 NV-4 原生双侧实现与 Workstation2 预检](validation/2026-07-29-nv4-native-dual-preflight.md)
- [2026-07-23 五层架构与既有仿真链路验证](validation/2026-07-23-five-layer-architecture.md)
- [2026-07-13 垂直切片验证报告](validation/2026-07-13-mediapipe-isaac-vertical-slice.md)
- [2026-07-13 Hand 2 固定法兰转向抓球最终验证](validation/2026-07-13-hand2-rotation-ball.md)
- [2026-07-13 MuJoCo FR3 v2—Hand 2 桌面初版验证（历史）](validation/2026-07-13-mujoco-fr3-hand2-table.md)
- [2026-07-14 MuJoCo FR3 v2—Hand 2 长边侧置四棱台验证](validation/2026-07-14-mujoco-fr3-hand2-table-layout.md)

## 后续维护位置

- `architecture/`：稳定的系统结构、数据流和部署视图。
- `components/`：已实现功能、真实代码入口、配置入口和限制；未实现的能力不得先写成“已支持”。
- `guides/`：可执行的安装、运行、采集、回放和排障流程。
- `reference/`：项目数据 schema、坐标、关节布局、配置字段等长期参考。
- `validation/`：验证矩阵、基准结果和验收证据。
- `decisions/`：影响边界、依赖、数据事实源或安全策略的 ADR。

每个功能合入时，至少同步更新对应 component 文档中的代码入口和验证入口。
