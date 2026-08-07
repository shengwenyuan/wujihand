---
name: apply-dexjoco-teleop-patterns
description: Reconstruct and apply the arm-plus-hand teleoperation patterns documented by DexJoCo without depending on DexJoCo's embodiment or runtime. Use when Codex needs to explain, design, review, or implement teleoperation involving a glove or hand tracker, HTC Vive/SteamVR/OpenVR tracker input, arm end-effector mapping, hand retargeting, single-arm or bimanual control, demonstration capture, or a shared pipeline targeting real hardware, an Isaac asset, and a MuJoCo asset. Also use when auditing DexJoCo teleoperation claims against its paper, tutorial, detailed note, or source code. Do not use it as the primary guide for DexJoCo policy training, benchmark task construction, or current Wuji product/API facts.
---

# Apply DexJoCo Teleop Patterns

把 DexJoCo 当作一份有源码证据的设计参考，而不是待复用的机器人、checkpoint、UDP 协议或运行时。保留其“腕部 6DoF 与手指重定向解耦”的核心经验，把设备、时间、坐标、安全和执行差异隔离到明确边界。

## 先固定问题

在解释或设计前明确：

```text
目标：解释 / 复核 / 架构设计 / 实现 / 排障
侧别：left / right / both
主端：glove 型号、tracker 型号、SteamVR/OpenVR 版本
手臂意图：absolute EE pose / relative EE motion / twist / arm joints
手部意图：landmarks / human joints / robot joints
从端：real device / Isaac asset / MuJoCo asset
控制模式：position / velocity / torque / impedance / simulator-specific
运行要求：频率、延迟、同步、录制、安全、单臂或双臂
版本事实：模型、SDK、固件、资产、标定和配置版本
```

不要仅凭相同 DoF 数、数组长度或文件名认定布局兼容。若请求没有指定从端，把三种从端作为独立 adapter 分支说明，不替用户静默选择一种控制语义。

## 按需读取参考

- 处理任何 DexJoCo teleop 事实时，读取 [references/dexjoco-system.md](references/dexjoco-system.md)。
- 设计、实现或评审可迁移系统时，再读取 [references/transfer-blueprint.md](references/transfer-blueprint.md)。
- 核对原文、源码、版本、许可或上游变化时，用 `rg --files` 和 `rg` 临时搜索当前 workspace，再读取实际找到的资料。不要在 skill 中保存开发机绝对路径或假设固定 workspace 布局。
- 涉及 Wuji Glove、Wuji Hand、Wuji SDK、模型或实时真机接口时，同时使用仓库中的 `lookup-wuji-docs` skill 查询当前官方页面。Wuji Hand 2 必须按 beta 产品处理，并在复用任何 joint/link、retargeting、collision 或 backend mapping 前通过 `wuji-description v2026.7.23` revision Gate。涉及 NERO/Songling、Orbbec、RealSense 或 SteamVR/VIVE 时，使用 `use-wujihand-robotics-mcps` 选择对应只读检索通道。不要把本 skill 中的历史或架构描述当作当前产品事实。

## 区分证据与建议

给关键结论标明来源层级：

```text
[论文] 方法级声明
[教程] 物理装配或软件设置
[代码] 固定 commit 下的实际行为
[笔记] 二次整理，需由论文或代码复核
[迁移建议] 面向本项目的新设计
[未验证] 仍需设备、资产或版本实测
```

当论文、教程、注释和代码不一致时，保留差异。复现固定版本时以目标 commit 的代码决定“实际做了什么”，但不要声称代码自动解决了方法或安全层面的缺口。

## 重建两条独立主链

始终先拆开，再组合：

```text
tracker -> valid 6DoF pose -> frame/extrinsic calibration
        -> clutch-relative motion -> ArmIntent

glove -> raw hand data -> canonical hand observation
      -> embodiment-specific retargeting -> HandIntent

ArmIntent + HandIntent -> temporal pairing -> supervision
                       -> backend command -> feedback
```

保留 DexJoCo 的可迁移点：

- 让 tracker 负责全局腕部/手臂运动，让 glove 负责局部手型；不要先强行拟合整个人体骨架。
- 用 clutch/reference pose 把操作者的相对运动映射到当前从端姿态，避免绝对 tracking world 直接驱动机器人。
- 把人手观测与目标手关节布局之间的差异交给独立 retargeter。
- 在数据层区分相对主端输入与绝对从端目标。

不要继承 DexJoCo 的隐式假设：

- 不把固定安装姿态当作 tracker-to-EE 外参。
- 不用无时间戳裸 UDP 数组作为项目 canonical contract。
- 不无限保持最后一帧而没有 freshness/watchdog。
- 不把 Allegro 16 维 checkpoint、关节顺序或限位用于其他手。
- 不把 MuJoCo mocap-body + operational-space controller 当作真机或 Isaac 的通用执行语义。

## 显式建立坐标与标定链

至少维护以下独立标定产物：

```text
tracker device identity and side
tracker-to-wearer/tool rigid mount extrinsic
tracking-world to command-frame alignment
clutch/reference pose and workspace scale
glove user calibration and canonical-hand transform
retargeter target model, fingertip/link map and joint layout
backend asset/device frame and joint mapping
```

用统一变换记号写出每一步的 parent/child frame。明确单位、右手系/左手系、四元数顺序、旋转作用方向和左右手镜像。把 DexJoCo 的

```text
delta = inverse(tracker_0) @ tracker_t
target_EE = EE_0 @ scaled(delta)
```

视为“已对齐且外参为单位阵”的特例；新系统应通过已标定的坐标共轭或等价 frame graph 显式映射。

## 先定义 canonical contract

不要沿用 23/46 维 flat action 作为跨后端接口。至少定义：

- 带 schema、sequence、source/receive monotonic time、clock domain、validity、frame、side、source ID 的 envelope。
- 带 tracking validity/quality 的 `TrackerPoseSample`。
- 带单位、点布局、置信度和 calibration ref 的 `CanonicalHandObservation`。
- 带 command frame、pose representation 和 mapping ref 的 `ArmIntent`。
- 带 `JointLayout`、单位、solver 状态和来源时间的 `HandIntent`。
- 区分原 intent、监督决定、实际 sent command 和 backend feedback。

让 real、Isaac、MuJoCo adapter 只负责把这些语义转换成各自 API、joint order、控制模式和时钟；不得让 OpenVR、glove SDK、Isaac 或 MuJoCo 类型进入 application/domain。

## 使用有门控的运行状态机

至少区分：

```text
OFFLINE -> READY -> ARMED -> ACTIVE -> HOLD
                              |         |
                              v         v
                            STOP <---- FAULT
```

只在以下条件同时满足后进入 `ACTIVE`：

- tracker 与 glove 数据新鲜、有效且 side/device identity 正确；
- 标定、frame graph、retarget config 和 backend layout 匹配；
- 从端反馈可用，当前姿态可作为 reference；
- supervisor、watchdog 和 stop path 已就绪；
- 操作者主动完成 arm/clutch。

为每个 backend 单独定义 hold、disable、stop、disconnect 和 recovery 语义。仿真中“保持上个目标”不自动等价于真机安全状态。

## 分别设计三个执行 adapter

- **Real device**：核对 Cartesian/joint 接口、控制模式、局部 watchdog、命令 deadline、限位/限速、使能/失能、急停、故障和反馈。先 shadow，再低速限幅，最后闭环。
- **Isaac asset**：锁定 USD/URDF 来源和 Isaac 版本，核对 world/root frame、articulation joint order、drive/controller 类型、physics step 与 feedback 时间。
- **MuJoCo asset**：锁定 MJCF 和 actuator 定义，核对 site/body/mocap frame、`qpos`/`ctrl` 顺序、Cartesian controller、timestep 与 substeps。

比较三后端时比较 canonical intent、映射和安全决定；不要要求三种动力学轨迹逐样本相同。

## 按风险递增验证

按以下顺序推进：

1. 用录制包或合成数据验证解码、side、shape、dtype、单位和 freshness。
2. 用静止、单轴平移、单轴旋转验证 tracker frame、外参与 clutch。
3. 用逐指、pinch、边界姿态验证 hand observation、retargeter 和 joint layout。
4. 用同一 canonical trace 分别做 headless Isaac/MuJoCo adapter contract test。
5. 覆盖 stale、丢包、乱序、invalid pose、错误 side、错误标定、NaN、超限和 backend fault。
6. 在仿真中完成耦合 arm + hand 与双臂验证。
7. 真机先只读/shadow，再人工可中止的限速运行。

报告真实执行过的检查、观测指标和未验证项。不要把脚本能启动、模型能加载或 GUI 能移动写成完整 teleop 验收。

## 交付时给出这些结果

根据请求输出：

- 当前问题与 DexJoCo 模式的对应关系。
- 可直接迁移、必须替换、仍待验证的三栏结论。
- arm 与 hand 两条数据流、合并点和状态机。
- 坐标/frame graph、标定产物和 joint layouts。
- real / Isaac / MuJoCo adapter 的职责与差异。
- freshness、supervision、stop/recovery 和 recorder 设计。
- 配置与 source/model/calibration provenance。
- 分阶段验证矩阵。

若在本仓库落代码，继续遵守 `docs/000-project-charter-and-architecture.md` 的 canonical contracts、ports/adapters、记录和验证边界；本 skill 只提供参考，不宣布任何 adapter 已实现。
