# MediaPipe—Wuji Hand 2—Isaac 控制链路

状态：原功能于 2026-07-13 完成验证；2026-07-23 接入五层 Session 组合。

固定手掌 q20 基线、固定 XYZ 三轴转向、动态球抓取及人工 MediaPipe 操作均已验证。人工操作能够抓起球，但流畅度和稳定性受 MediaPipe 手部姿态估计限制；本需求以功能可用收口，不继续扩展 wrist XYZ 或视觉滤波。

## 范围与边界

- 产品：**Wuji Hand 2 Beta 1 right**，20 个主动手指关节。
- 资产：默认 Session 固定 [`wuji-description v2026.6.27`](https://github.com/wuji-technology/wuji-description/releases/tag/v2026.6.27)；resolver 校验 Binding commit/path，runner 校验实际 USD SHA-256。
- 输入：MediaPipe 右手 21×3 landmarks，经 Wuji SDK 2026.7.2 重定向为 q20，同时估计掌面姿态。
- 仿真：法兰 XYZ 固定，增加 D6 `rotX/rotY/rotZ` 安装机构。该 3R 是项目的 Isaac overlay，不是 Hand 2 产品自带腕部关节。
- 任务：物理 task home 为 pitch `+60°`；球半径 `30 mm`，球心 `[0.130, 0.025, 0.410] m`。

## 核心入口

| 职责 | 文件 |
|---|---|
| MediaPipe、Wuji retarget、v1/v2 发布 | `tools/run_mediapipe_hand2_teleop.py` |
| q20 v1 producer / consumer Session | `configs/sessions/mediapipe_hand2_q20_udp_v1.yaml`、`configs/sessions/isaac_hand2_teleop_v1.yaml` |
| q20+pose v2 producer / consumer Session | `configs/sessions/mediapipe_hand2_hand_command_udp_v1.yaml`、`configs/sessions/isaac_hand2_right_rotation_ball_teleop_v1.yaml` |
| rotation-ball Isaac composition root | `tools/run_isaac_hand2_rotation_ball.py` |
| 隔离重复资格验证 | `tools/run_isaac_hand2_rotation_ball_trials.py` |
| Hand 2 产品身份与 Isaac 表示 | `configs/assets/wuji_hand2_beta1_right_v1.yaml`、`configs/bindings/isaac/wuji_hand2_beta1_right_v2026_6_27_v1.yaml` |
| D6 + Hand 2 装配 | `configs/assemblies/hand2_right_rotation_mount_v1.yaml` |
| rotation-ball Workcell / typed 兼容叶 | `configs/workcells/isaac_hand2_rotation_ball_v1.yaml`、`configs/base/hand2_rotation_ball_v1.yaml` |
| 五层解析与既有 runner 桥 | `src/wujihand/runtime/session_resolver.py`、`src/wujihand/runtime/session_compat.py` |
| 掌面估计与 neutral 标定 | `src/wujihand/adapters/input/mediapipe_palm_orientation.py`、`application/calibration/palm_orientation.py` |
| 姿态 contract 与安全监督 | `src/wujihand/domain/pose.py`、`application/supervision/pose_supervisor.py` |
| 原子 v2 命令与 UDP | `src/wujihand/ports/hand_command.py`、`adapters/transport/udp_hand_command.py` |
| D6、动态球与抓取判据 | `src/wujihand/adapters/simulation/hand2_rotation_mount.py`、`hand2_ball_scene.py`、`hand2_grasp.py` |
| 确定性抓取脚本 | `src/wujihand/runtime/rotation_ball_script.py` |

五层 Session 现在会在相机或 Isaac backend 初始化前解析。MediaPipe producer 与 Isaac
consumer 仍是两个进程：q20 v1 和 q20+pose v2 各有一对 Session，并通过明确的
transport contract 与 control layout 校验配对，不会因数组长度相近而互换。
为保持现有启动与配对语义，当前 producer Session 仍声明目标 Isaac backend、Binding
和 Workcell；这是 compatibility choice，不代表输入生产端已经抽象为可直接复用到
MuJoCo/ROS 的 backend-neutral target contract。

fixed-hand 基线的桌面、手安装位姿和相机已经直接归
`configs/workcells/isaac_hand2_table_v1.yaml` 所有。rotation-ball 的完整法兰、桌面、
球、D6 限位和资格参数暂时仍由已验证的
`configs/base/hand2_rotation_ball_v1.yaml` 单点持有，Workcell 通过 compatibility
profile 引用它；这避免迁移期复制物理事实。

runner 接受可选 `--session`，省略时仍按 `--command-source` 或 MediaPipe publish 参数
选择既有默认链路。Isaac 保留 `--asset`、`--profile`、`--scene-profile` 显式
override；它们及其文件内容 hash 会进入 `session_hash`，并继续接受相应的 layout、
关节、实际 USD hash 和模型检查。默认 Session 路径仍锚定 Binding/source lock；
fixed-hand 若同时覆盖 `--asset` 与 `--profile`，只证明两份实验输入彼此内容自洽，
不再提供 pinned provenance 结论。Isaac 报告写入 `session`/`session_hash`，
MediaPipe 启动日志打印相同信息。

## 数据流

```text
D435 RGB -> MediaPipe right landmarks
  ├─> Wuji RetargetSession -> q20
  └─> palm frame -> neutral calibration -> relative wxyz quaternion
      -> HandCommand v2(q20 + quaternion + quality + calibration_id)
      -> loopback UDP -> JointCommandSupervisor + PoseSupervisor
      -> finger20 targets + D6 wrist3 targets
      -> PhysX contacts/ball pose -> BallLiftEvaluator
```

domain/application 层不依赖 MediaPipe、Wuji SDK 或 Isaac 对象，只处理 NumPy 数值、显式 frame、单调时间戳、质量和 calibration token。

## 姿态与 clutch

掌面 frame 使用 wrist、index MCP、middle MCP、pinky MCP 四点构造正交坐标系。稳定 neutral 保存 `R0`，后续输出 `R0.T @ R`。因此操作者以舒适姿态完成 neutral 时，Isaac 已处于物理 `+60°` task home；到脚本验证的物理 `76°` 预抓取只需约 `+16°` 相对 pitch。

按 `c` 会请求 clutch/recenter：当前稳定手姿成为新的 identity，生成新的 `calibration_id`，并连续发送 3 个 identity 帧。Isaac 只在“新 calibration epoch + identity”时重新 arm，从而保持当前仿真腕姿并以真人当前姿态继续相对控制。

PoseSupervisor 在输入老化到 `250 ms` 时进入 `DEGRADED` 并保持最后安全姿态，`500 ms` 时进入 `DISARMED`。恢复必须重新稳定标定或 clutch，不能用旧 calibration epoch 自动恢复。

## 固定法兰 D6

```text
world fixed joint
  -> flange_anchor
     -> D6 wrist_rotation_joint (transXYZ locked, rotXY limited, rotZ periodic)
        -> Hand 2 r_base_link
           -> finger20
```

roll/pitch 相对 task home 限制为 `±89°`，yaw 目标做周期 unwrap。运行时按 DOF name + USD path 验证 wrist3/finger20，不依赖 23 维数组顺序。Hand 2 `r_base_link` 的 principal-axes quaternion 通过单位四元数平方根对称分配到 D6 两侧 joint frame，使驱动轴与可见手掌轴一致。

## 抓取事实与安全门

球体始终为 `DynamicSphere`；实现不使用 attachment、teleport、kinematic target 或逐帧 world-pose 搬运。一次通过要求：

- 球底离桌 `≥20 mm`；
- ball/table 接触消失；
- thumb 与至少两个对侧手指组接触；
- ball-in-palm 相对滑移 `≤5 mm`；
- 条件连续保持 `≥1 s`；
- 独立整手/table contact view 全程无接触。

`wujihand.q20.v1` 继续服务固定手掌兼容链路。rotation-ball 使用严格的 `wujihand.hand_command.v2` 原子携带 q20 和姿态；v1/v2 相互拒绝，v2 仅绑定 loopback，拒绝非法 schema、非有限数、错误 shape、非单位四元数、陈旧/未来时间戳和非单调 sequence。

## 已知限制

- 人工抓取已成功，但 MediaPipe 遮挡、landmark 抖动和大角度姿态跳变使操作不够顺畅。
- 固定 XYZ 只覆盖球位于手指闭合与法兰旋转共同可达区域的任务，不是通用桌面 pick。
- Hand 2 Beta 1 官方当前只有刚性骨骼仿真模型；本结果不代表软体指腹、触觉或真机抓力。
- pinned USD 或 Isaac 版本变化后，必须重新执行结构、严格物理和人工方向检查。
- 无 GPU/Isaac 或无 D435/兼容相机时，只能完成五层无 backend contract 回归，不能据此
  声称 authored-stage、PhysX 抓取或真人遥操作已重验。

五层职责见 [五层 Session 组合](five-layer-session-composition.md)，执行步骤见
[MediaPipe 控制 Hand 2 转向抓球指南](../guides/mediapipe-hand2-rotation-ball.md)，
本轮架构回归见
[2026-07-23 五层架构与既有仿真链路验证](../validation/2026-07-23-five-layer-architecture.md)，
原功能证据见
[2026-07-13 Hand 2 固定法兰转向抓球最终验证](../validation/2026-07-13-hand2-rotation-ball.md)。
