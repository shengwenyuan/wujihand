# 硬件代际、数据边界与约束

## 目录

- [先判定产品](#先判定产品)
- [Wuji Hand 2 Beta 1](#wuji-hand-2-beta-1)
- [Wuji Hand 第一代](#wuji-hand-第一代)
- [Wuji Glove](#wuji-glove)
- [禁止混淆](#禁止混淆)
- [仿真与 teleop 的硬件核对顺序](#仿真与-teleop-的硬件核对顺序)

## 先判定产品

本页只保存代际路由与兼容性警告。回答当前参数前，必须通过官方 `wuji-docs` MCP 打开目标产品页、使用约束和发布记录；不要把本地快照当成当前事实。

| 线索 | 产品 | 文档根 | 典型软件路径 |
|---|---|---|---|
| Hand 2、Beta 1、Ethernet、MIT、可反驱 | Wuji Hand 2 | `wuji-hand/latest/` | `wuji-sdk`、Studio、Retargeting、Description |
| Hand v1、USB、位控、自锁、wujihandpy | Wuji Hand 第一代 | `wuji-hand/v1/` | `wuji-sdk.WujiHand` 或 legacy `wujihandpy`、ROS2、HMI、Upgrader |
| Glove、EMF、IMU、触觉、手部追踪 | Wuji Glove | `wuji-glove/latest/` | `wuji-sdk`、Studio、Retargeting |

绝不能把 `https://docs.wuji.tech/docs/zh/wuji-hand/latest/` 理解成第一代的“最新页”；它当前是 Hand 2 文档。第一代使用固定 `/v1/` 路径。

## Wuji Hand 2 Beta 1

权威入口：

- [产品介绍](https://docs.wuji.tech/docs/zh/wuji-hand/latest/overview/)
- [使用约束](https://docs.wuji.tech/docs/zh/wuji-hand/latest/usage-constraints/)
- [SDK 接口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)
- [发布记录](https://docs.wuji.tech/docs/zh/wuji-hand/latest/release-notes/)

> **Beta Gate：** 官方产品页明确说明手册对应 Beta 1 阶段样机，参数与表现不代表最终产品状态。接口、供电、软体、散热、可靠性、负载、零位和力矩透明性仍可能变化。发布记录还包含 Beta 2-only 硬件、固件和触觉能力；必须按实物硬件阶段、序列号和固件分流，不能把 `latest` 下的所有能力合成一台设备。

### 当前识别特征

- 20 个主动自由度，每指 4 个；以太网通信。
- 可反驱旋转直驱关节，MIT 力位混合控制。
- 产品页列出 1000 Hz × 20 轴、12 V DC；供电和持续负载还受 Beta 1 使用约束限制。
- 实时命令发布仍是完整 20 个 `JointCommand` 帧；配置和控制动作在 2026.7.1 资源模型中可通过 length-20 mask 选择关节。产品专属页与发布记录对此存在版本差异，精确能力以目标 `wuji-sdk` tag 为准。
- 全局关节顺序是 thumb 0–3、index 4–7、middle 8–11、ring 12–15、pinky 16–19；精确标签与正方向应回查 SDK/产品页。

### 软件与版本边界

2026-07-01 发布记录说明设备接口重构为统一资源模型，是破坏性变更：实时反馈走订阅流，实时命令走 publish。旧示例、旧字段或旧固件不能直接套用；需要一起核对：

```text
Hand 2 固件 + wuji-sdk 版本 + 产品 sdk-reference + 调用代码
```

新设计应优先从通用 [Wuji SDK](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/) 和产品专属 `sdk-reference/` 组合取证。Studio 用于设备管理、数据可视化和固件升级。

### Description 模型版本边界

| 范围 | 路径与标识 | 结论 |
|---|---|---|
| 本仓库固定 `v2026.6.27` | `hand2_beta/body/`、根 `{l,r}_base_link`、`wujihand.usd` | 历史仿真资产；现有配置、q20 映射、数据集和验证只对该 pin 成立 |
| 官方 `v2026.7.23+` | `hand2/hand2_beta1/body/`、根 `{l,r}_wrist`、`wujihand2.usd` | 新坐标约定和命名基线；碰撞、sites、ROS 包与机械资产也已改变 |

`v2026.7.23` 不是可机械替换的目录改名。即使两边都是 20 DoF，也必须重新核对 joint order/axis、root/attachment、retargeting link map、碰撞过滤、drive gains、指尖 site 和历史数据解释。官方说明当前 Hand 2 USD 的 kp/kv 仍沿用上一平台标定，待 Hand 2 硬件系统辨识后更新，因此不能把任一版本描述成已由真机辨识的高保真动力学模型。

### 仿真与力觉约束

- 当前官方模型入口是 `hand2/hand2_beta1/`；本仓库为复现既有结果继续固定历史 `hand2_beta/`。
- 官方使用约束仍说明未提供 Hand 2 带软体的仿真模型，不要把第一代 `hand/body-with-soft/` 误当作 Hand 2 资产。
- Beta 1 不提供指尖触觉；发布记录中的 Beta 2 触觉能力要求匹配硬件、固件和 SDK，不能外推到 Beta 1。
- 关节电流到外部力矩/接触力的链路尚未收敛，不能把电流读数直接作为接触力判据。
- Beta 1 的散热、耐用性、零位精度和负载数据有明确限制；设计碰撞、长时间满载或力控制方案前必须读 `usage-constraints/`。

### 相关入口

- 网络真机 teleop：[Wuji Retargeting](https://docs.wuji.tech/docs/zh/wuji-retargeting/latest/)
- 模型：[Wuji Description](https://docs.wuji.tech/docs/zh/wuji-description/latest/)
- 通用 SDK：[Wuji SDK](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/)
- GUI：[Wuji Studio](https://docs.wuji.tech/docs/zh/wuji-studio/latest/)

当前 Hand 2 产品入口没有单列已验证的 Hand 2 ROS2 指南。独立 `wujihandros2` 文档主要实证第一代 USB 设备；若官网其他页面声称 Hand 2 支持，先核对仓库 tag、发布说明和实际 transport，再作结论。

## Wuji Hand 第一代

权威入口：

- [产品介绍](https://docs.wuji.tech/docs/zh/wuji-hand/v1/overview/)
- [用户须知](https://docs.wuji.tech/docs/zh/wuji-hand/v1/user-notice/)
- [发布记录](https://docs.wuji.tech/docs/zh/wuji-hand/v1/release-notes/)

### 当前识别特征

- 20 个主动自由度，USB 2.0 通信。
- 自锁旋转直驱，不具备反驱能力；主控制模式为位置控制。
- 产品页列出 1000 Hz × 20 轴和 12–20 V DC。

### 两套 SDK 路径并存

- 通用 `wuji-sdk.WujiHand`：2026-06-15 后加入，资源式 API 基本镜像 Hand 2，适合统一跨代应用；以当前 Wuji SDK 页面和 release 为准。
- legacy `wujihandpy` / C++ 内核：第一代 USB 直连、实时控制和既有项目的权威入口。

不要混用两套类名、连接模型和异常类型。看到旧代码时先识别 import：

```text
from wuji_sdk ...       -> 通用 calendar-version SDK
import wujihandpy ...   -> 第一代 semver SDK
```

### 第一代专用工具

- [Wuji Hand ROS2](https://docs.wuji.tech/docs/zh/wujihandros2/latest/)：driver、topic/service、多手与 RViz；当前文档主要实证第一代。
- [Wuji Hand HMI](https://docs.wuji.tech/docs/zh/wuji-hand-hmi/latest/)：监控、标定和调试。
- [Wuji Hand Upgrader](https://docs.wuji.tech/docs/zh/wuji-hand-upgrader/latest/)：固件升级。
- [Wuji Hand Teleop](https://github.com/wuji-technology/wuji-hand-teleop)：第一代 ROS2 双手/机械臂整栈；不是 Hand 2 的默认方案。

### 旧触觉配件不是 Wuji Glove

第一代产品页里的“触觉感知手套”是装在机械手上的选配压力感知配件，历史接口包括 USB CDC、`/dev/ttyACM*` 和 24×32 压力帧。它不同于当前独立 Wuji Glove。查询中出现“触觉手套”时必须让上下文决定是哪一个，不能合并数据格式或 SDK。

## Wuji Glove

权威入口：

- [产品介绍](https://docs.wuji.tech/docs/zh/wuji-glove/latest/introduction/)
- [使用前准备](https://docs.wuji.tech/docs/zh/wuji-glove/latest/getting-started/)
- [SDK 数据参考](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/)
- [坐标系](https://docs.wuji.tech/docs/zh/wuji-glove/latest/coordinate-frames/)
- [发布记录](https://docs.wuji.tech/docs/zh/wuji-glove/latest/release-notes/)

### 数据层次

Wuji Glove 的 SDK 文档把数据分为原始数据、后处理产物和全局资源。常用子页包括：

```text
sdk-data-reference/
├── tactile/                 触觉帧、区域、二值/残差、点云
├── hand-tracking/           EMF、指尖姿态、21 DoF、人手骨架
├── imu/                     IMU 数据
├── coordinate-transforms/   tf_static / tf
├── device-operations/       参数、持久化、重启、日志
└── calibration/             IK 标定、profile、URDF 查找与用户隔离
```

### 关键形状与兼容性

- 当前触觉布局是 24×31，共 744 个槽位、526 个有效触点；发布记录显示 2026-06-18 从 24×32 变更，历史录制不能按新布局解释。
- 5 个 EMF 指尖模块和手背 IMU 产生手部追踪数据；精确频率、有效字段和降频行为必须以各数据子页为准。
- `hand_joint_angles` 是人手/手套模型的 21 DoF；机器人手输出是 20 DoF。必须经过 profile/retargeting，不能直接透传。
- 首次使用、换穿戴者或切换目标手型时做 IK 标定。
- 驱动 Hand 2 时明确使用 `wujihand2` profile/URDF；默认或静默 fallback 可能仍使用第一代模型并降低精度。
- 原始 EMF/IMU 可通过 `offline-pipeline/` 离线复算，适合把后处理从采集路径移出。

通用 SDK 的当前 `TactileGloveFrame` 页面也可能在第一代 Hand 类型下展示 744/24×31，这不能反向证明第一代机械手旧触觉配件已经改版；旧产品页和 legacy API 仍是 768/24×32。若使用当前 `wuji_sdk.WujiHand` 的配件适配层，固定 SDK 版本并实测帧长，或向官方确认。

### 网络与坐标

当前文档给出的默认设备地址是左 `192.168.1.100`、右 `192.168.1.101`、`/24`；发现与数据端口也在产品文档中定义。部署前核对现场配置，不要把默认值硬编码为协议事实。

坐标与机器人集成必须同时阅读 `coordinate-frames/` 和 SDK 的 `tf`/`tf_static` 数据定义。不要只凭可视化中的轴向猜坐标约定。

## 禁止混淆

| 不可混淆的对象 | 原因 |
|---|---|
| Hand 2 `latest` 与 Hand v1 | 产品、transport、控制模式、软件栈不同 |
| Hand v1 的旧触觉配件与 Wuji Glove | 硬件角色、网络/USB、数据布局和 SDK 不同 |
| Glove 21 DoF 与机器人手 20 DoF | 人手估计与执行器命令空间不同 |
| 当前 24×31 与历史 24×32 触觉帧 | 不兼容布局变更 |
| `wuji-sdk` 与 `wujihandpy` | 版本体系、API 和目标范围不同 |
| Hand 2 骨骼模型与第一代软体模型 | 当前 Hand 2 不提供带软体模型 |
| Hand 2 Beta 1 与 Beta 2 | 接口、供电、触觉、固件和物理表现可能不同 |
| `v2026.6.27 hand2_beta` 与 `v2026.7.23+ hand2/hand2_beta1` | 根节点、命名、坐标、碰撞和文件布局不兼容 |
| `latest` 当前事实与固定版本事实 | 滚动文档会改变 |

## 仿真与 teleop 的硬件核对顺序

仿真：

1. 识别产品代际和左右手。
2. 查产品参数、关节范围与坐标定义。
3. Hand 2 额外查使用约束。
4. 在 Wuji Description 区分第一代 `hand/`、本仓库历史 pin `hand2_beta/`、当前 `hand2/hand2_beta1/` 与 `glove/`。
5. 查 MuJoCo / Isaac / ROS2 对应集成页和模型 tag/submodule commit。

Teleop：

1. 查 Wuji Glove 连接、标定、profile 和实际 URDF 来源。
2. 查 Retargeting 的 21×3 输入、20 关节输出、配置和调参。
3. 按目标手代际选择网络 `wuji-sdk` 或第一代 USB/ROS2 输出。
4. 检查左右手、关节名/顺序、单位和限位。
5. 真机前检查供电、使能、急停/停止路径及产品安全约束。
