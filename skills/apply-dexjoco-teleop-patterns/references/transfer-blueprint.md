# 从 DexJoCo 迁移到 real / Isaac / MuJoCo 的 teleop 蓝图

## 目录

- [目标与非目标](#目标与非目标)
- [推荐分层](#推荐分层)
- [Canonical contracts](#canonical-contracts)
- [Arm 映射与坐标标定](#arm-映射与坐标标定)
- [Hand observation 与 retargeting](#hand-observation-与-retargeting)
- [运行状态机与时序](#运行状态机与时序)
- [三类 backend adapter](#三类-backend-adapter)
- [监督、停止与恢复](#监督停止与恢复)
- [记录与可复现性](#记录与可复现性)
- [实现顺序](#实现顺序)
- [验证矩阵](#验证矩阵)

## 目标与非目标

目标是让同一主端 glove + tracker 组合产生 backend-neutral intent，再分别驱动：

1. 真实 arm + hand；
2. Isaac 中的 arm + hand asset；
3. MuJoCo 中的 arm + hand asset。

共享的是输入语义、标定、intent、监督和记录，不是某一执行 API 或一个“通吃所有设备”的 N-DoF 数组。

不要：

- 直接 import 或调用 DexJoCo runtime；
- 复用 Allegro checkpoint 驱动不同手；
- 强迫三个 backend 使用相同低层 control mode；
- 把 simulator hold-last-target 当作真机安全策略；
- 让设备 SDK、OpenVR、Isaac 或 MuJoCo 类型跨过 adapter。

## 推荐分层

```text
Tracker SDK / OpenVR                  Glove SDK / stream
        |                                     |
TrackerInputAdapter                    HandInputAdapter
        |                                     |
TrackerPoseSample               CanonicalHandObservation
        |                                     |
ArmCalibrationMapper                    RetargetPort
        |                                     |
ArmIntent -------------------------- HandIntent
                 \                  /
                  TemporalPairer / TeleopSession
                              |
                        CompositeIntent
                              |
                          Supervisor
                              |
                 SafetyDecision + SentCommand
                              |
                         ExecutionPort
                    /            |            \
               RealAdapter   IsaacAdapter   MuJoCoAdapter
                    \            |            /
                           BackendFeedback
```

在本仓库对应：

```text
domain/       envelope、frames、layouts、intent、decision、feedback
ports/        input、retarget、execution、state、recorder 协议
application/  calibration、teleop orchestration、supervision、recording
adapters/     tracker/glove、retargeter、real/Isaac/MuJoCo、storage
runtime/      session config 与 dependency composition
```

## Canonical contracts

### 公共 envelope

每个样本至少携带：

```text
schema_version
session_id / episode_id
sequence
source_id
side
source_timestamp
receive_monotonic_timestamp
clock_domain
frame_id
valid_until or max_age policy
calibration/config provenance
```

设备无可靠 source time 时也必须记录 receive monotonic time，并把 source time 标为 unavailable；不要伪造一个看似精确的设备时间。

### `TrackerPoseSample`

至少表达：

```text
T_parent_tracker
parent_frame / child_frame
translation unit
rotation convention
tracking validity
tracking quality or reason
device serial / logical side binding
```

### `CanonicalHandObservation`

至少表达：

```text
points shape and semantic layout
meters or another explicit unit
canonical frame
left/right
per-point confidence/validity when available
glove user/device calibration ref
normalization transform ref
```

不要假设所有 glove 都自然输出 MediaPipe 21 点。让 input adapter 把设备特有骨架映射到项目 canonical schema；映射不充分时保留 missing mask，不用零值假装观测。

### `ArmIntent`

至少表达：

```text
side
target pose or motion
command frame
absolute/relative representation
translation and rotation units
mapping/calibration ref
source sample ref and age
```

推荐 application 层最终形成绝对 Cartesian target，同时保留产生它的 relative master motion。若某 arm backend 只接受 joints，让 adapter/独立 IK port 完成转换并报告求解状态。

### `HandIntent`

至少表达：

```text
side
target joint positions and optional velocity
JointLayout ID
units
retargeter model/config hash
solver status, residual or confidence
source sample ref and age
```

相同 16、20 或其他长度的向量必须通过 `JointLayout` 显式映射。layout 包含 joint names、order、side、正方向、limits、asset/device provenance。

### `CompositeIntent` 与执行事实

Composite intent 引用同一 control tick 采用的 arm/hand source 样本及各自 age。继续分开记录：

```text
raw sample
canonical observation
arm/hand intent
safety decision
sent backend command
backend feedback
```

不要用 intent 代替实际 sent command，也不要用 sent command 代替反馈。

## Arm 映射与坐标标定

### 变换约定

先固定一种记号，例如 `T^A_B` 表示 B frame 在 A frame 中的 pose。给矩阵乘法、主动/被动旋转、四元数顺序写 unit test。

设：

```text
T^V_T(t): tracker T 在 VR/tracking world V 中的 pose
T^B_E(0): arm EE E 在 robot/backend base B 中的 reference pose
C:        tracker reference coordinates 到 EE command coordinates 的已标定对齐
```

DexJoCo 特例：

```text
Delta_T = inverse(T^V_T(0)) @ T^V_T(t)
T^B_E(target) = T^B_E(0) @ ScaleTranslation(Delta_T)
```

一般化时用经验证的 frame graph 或共轭：

```text
Delta_E = C @ ScaleTranslation(Delta_T) @ inverse(C)
T^B_E(target) = T^B_E(0) @ Delta_E
```

`C` 的精确定义取决于项目变换约定。不要复制公式后省略 frame test。

### 分开保存四类 arm calibration

1. **device binding**：tracker serial ↔ logical left/right。
2. **mount extrinsic**：tracker frame ↔ glove/wrist/tool frame。
3. **world/command alignment**：tracking frame ↔ robot base 或 EE reference coordinates。
4. **workspace mapping**：translation scale、轴向 gain、rotation mapping、workspace clamp。

clutch reference 是 session 状态，不应覆盖永久外参。每次重新 clutch 都形成新的 reference event 并进入 recorder。

### 校准动作

至少做：

- 静止噪声与漂移；
- X/Y/Z 单轴正负平移；
- roll/pitch/yaw 单轴正负旋转；
- 左右手 side 和镜像；
- reference pose 重复性；
- workspace 边缘与 scale；
- tracker 暂时遮挡后的恢复。

用 marker 或已知机器人姿态验证，不靠 GUI “看起来差不多”。

## Hand observation 与 retargeting

### 保留两级 calibration

区分：

1. glove/佩戴者本身的传感器或手型 calibration；
2. 人手 canonical observation 到目标 robot hand 的 embodiment retargeting。

更换穿戴者可能只要求第 1 或两者都重做；更换目标 hand/asset 一定要求重新核对第 2、joint layout 和目标模型。

### RetargetPort 输入输出

RetargetPort 只消费 canonical observation 和 target embodiment profile，不消费 glove SDK object 或 backend object。输出 `HandIntent`，并报告：

```text
success / degraded / failure
residual or quality
clipped joints
model/config version
processing latency
```

可以借鉴 GeoRT 的：

- fingertip workspace calibration；
- target URDF/MJCF/kinematics 驱动的映射；
- direction、coverage、smooth sensitivity、pinch 等几何目标；
- 小模型低延迟部署。

不要默认继承：

- 四指/16 DoF；
- 独立 fingertip MLP；
- 无 collision/contact/history；
- 默认 GPU；
- 论文写出但固定代码未启用的 loss。

若目标 Wuji hand/glove 使用官方 retargeting，先用 `lookup-wuji-docs` 固定当前 SDK、模型、输入 shape、单位、side 和 joint output contract，再包装为 RetargetPort。

## 运行状态机与时序

### 推荐状态

```text
OFFLINE: dependency/source/backend 未就绪
READY:   sources 可读，配置和标定已加载，未允许控制
ARMED:   操作者请求控制，等待 reference 与全部 readiness gate
ACTIVE:  实时产生并监督 command
HOLD:    暂停更新；执行配置化 hold 行为
STOP:    执行受控停止/失能，需显式重新 arm
FAULT:   设备或系统故障，需清错和重新验证
```

### `ACTIVE` 门槛

同一门槛检查：

- tracker 和 hand source 新鲜；
- side/device identity 与 session config 一致；
- tracking validity、hand confidence 和 retarget status 合格；
- reference pose 与 backend feedback 同步到容差内；
- backend online，无 fault；
- supervisor/watchdog/stop path 活跃；
- recorder 可降级但不会阻塞控制。

### 多速率与 latest-value

允许各 source 异步采集，但必须：

- 有界队列或 drain-to-latest；
- 记录被 drain/drop 的样本数；
- 每个 control tick 记录采用的 sample ref 和 age；
- 超过 max age 时执行 reject/hold/stop；
- 处理乱序、重复和 source clock reset；
- 不在 device receiver thread 做阻塞 backend command。

需要严格 arm-hand 同步时定义 pairing tolerance；否则明确采用“各自最新且均未过期”，并记录 skew。

## 三类 backend adapter

### Real device

核对并配置：

```text
arm Cartesian/joint API and controller mode
hand joint API and control mode
command rate and deadline
local watchdog
workspace, joint, velocity, acceleration and effort/current limits
enable/disable/stop/emergency-stop
feedback timestamp and online/fault state
disconnect/reconnect semantics
```

网络 supervisor 不能替代设备附近的本地停止路径。先执行：

```text
read-only feedback -> shadow command -> disabled/limited dry run
-> low-speed single-axis -> coupled arm+hand -> full task
```

### Isaac asset

固定：

```text
Isaac/Isaac Lab version
USD/URDF source and commit
root/world frame
articulation joint names/order/axes
drive/controller type
physics_dt, control_decimation and simulation clock
feedback extraction and reset semantics
```

先做 headless 逐关节与 EE pose replay。不要把 USD 能加载视为 controller 或 teleop 已适配。

### MuJoCo asset

固定：

```text
MuJoCo version
MJCF source and commit
body/site/mocap names and frames
qpos/qvel/actuator ctrl mappings
actuator type and ranges
Cartesian controller or IK implementation
timestep, substeps and feedback timing
```

DexJoCo 的 mocap target + operational-space torque 是一个可选 MuJoCo adapter 实现，不应放进 application contract。

### 能力协商

让 backend profile 声明能力，例如：

```text
supports_cartesian_pose
supports_joint_position
supports_impedance
supports_atomic_arm_hand_command
supports_feedback_timestamp
supports_emergency_stop
```

不支持所需语义时在 composition/arming 阶段失败，不用运行时 `hasattr` 或静默降级猜测。

## 监督、停止与恢复

Supervisor 至少处理：

| 类别 | 示例 | 可能决定 |
|---|---|---|
| 输入健康 | stale、invalid pose、低置信度、side 跳变 | reject / hold / stop |
| 变换健康 | calibration 不匹配、NaN、非正交 rotation | reject / fault |
| retargeting | solver 失败、残差高、layout 错误 | hold / clamp / stop |
| 运动约束 | workspace、joint、速度、加速度、突跳 | clamp / reject |
| backend | feedback stale、fault、command error | hold / stop / fault |
| 双臂 | EE 相对距离、交叉碰撞、角色错位 | clamp / stop |

为 arm 和 hand 定义独立决定，再定义 composite 优先级。常见原则是任一关键链失效都不继续更新组合 command，但精确 hold/stop 动作必须按 backend 和任务风险配置。

恢复时不要直接继续旧 reference。通常需要：

1. source 和 backend 重新健康；
2. 退出旧 `ACTIVE`；
3. 重新取得当前 feedback；
4. 操作者重新 clutch/reference；
5. 重新 arm。

## 记录与可复现性

一次 session 至少保存：

```text
raw tracker and glove streams
canonical tracker/hand observations
calibration and frame graph refs
arm/hand intents
pairing/skew and freshness metrics
safety decisions and reasons
actual sent commands
backend feedback and faults
clutch/arm/hold/stop/reset events
config snapshot, source/model/SDK/asset versions
drop counts, latency and jitter summaries
```

保持多速率原始流。离线生成训练视图时再定义插值、hold、tolerance 和 missing mask。不要像固定 30 Hz 名义时间轴那样丢失真实接收时间。

## 实现顺序

1. 固定 canonical frames、layouts、envelopes 和 session config。
2. 用录制包实现 tracker/glove input adapter contract tests。
3. 完成 arm frame calibration 与 hand retargeting 的离线 golden tests。
4. 实现 supervisor 和 deterministic replay。
5. 先完成一个仿真 backend 的 headless vertical slice。
6. 用同一 trace 接第二个仿真 backend，验证 adapter 边界。
7. 接 live inputs，在仿真测 latency/jitter/drop/stale。
8. 接 real feedback adapter，先 shadow。
9. 验证本地 stop/watchdog 后才发送受限 command。
10. 最后启用耦合 arm + hand、双臂和数据采集。

不要同时调 input、frame、retargeter 和三个 backend；每次只打开一个新不确定边界。

## 验证矩阵

| 层 | 必测 |
|---|---|
| 解码 | shape、dtype、endianness、schema version、side、source ID |
| 时间 | sequence、duplicate、out-of-order、age、clock reset、drop |
| frame | 静止、单轴平移/旋转、quaternion order、左右镜像 |
| hand | fingertip/landmark layout、逐指方向、pinch、limits、solver failure |
| pairing | arm-hand skew、双侧同步、missing side |
| supervisor | accept/clamp/reject/hold/stop/fault 表驱动测试 |
| Isaac | asset mapping、headless replay、reset、feedback clock |
| MuJoCo | site/body/actuator mapping、controller、timestep/substeps |
| real | shadow diff、watchdog、disconnect、fault、stop/re-arm |
| recording | non-blocking、drop count、异常终止 manifest、replay trace |

跨 backend 对同一 trace 比较：

- canonical intent 是否完全相同；
- adapter joint/frame 映射是否可追踪；
- supervision decision 是否符合 backend profile；
- command-feedback 指标是否各自可解释。

不要要求真实设备、Isaac 和 MuJoCo 的反馈数值完全相同；动力学、controller 和 contact model 本来就不同。
