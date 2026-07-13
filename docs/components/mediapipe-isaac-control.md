# MediaPipe—Wuji Hand 2—Isaac 控制链路

状态：2026-07-13，需求开发结束。

固定手掌 q20 基线、固定 XYZ 三轴转向、动态球抓取及人工 MediaPipe 操作均已验证。人工操作能够抓起球，但流畅度和稳定性受 MediaPipe 手部姿态估计限制；本需求以功能可用收口，不继续扩展 wrist XYZ 或视觉滤波。

## 范围与边界

- 产品：**Wuji Hand 2 Beta 1 right**，20 个主动手指关节。
- 资产：固定 [`wuji-description v2026.6.27`](https://github.com/wuji-technology/wuji-description/releases/tag/v2026.6.27)；运行时校验 commit、USD 路径和 SHA-256。
- 输入：MediaPipe 右手 21×3 landmarks，经 Wuji SDK 2026.7.2 重定向为 q20，同时估计掌面姿态。
- 仿真：法兰 XYZ 固定，增加 D6 `rotX/rotY/rotZ` 安装机构。该 3R 是项目的 Isaac overlay，不是 Hand 2 产品自带腕部关节。
- 任务：物理 task home 为 pitch `+60°`；球半径 `30 mm`，球心 `[0.130, 0.025, 0.410] m`。

## 核心入口

| 职责 | 文件 |
|---|---|
| MediaPipe、Wuji retarget、v1/v2 发布 | `tools/run_mediapipe_hand2_teleop.py` |
| rotation-ball Isaac composition root | `tools/run_isaac_hand2_rotation_ball.py` |
| 隔离重复资格验证 | `tools/run_isaac_hand2_rotation_ball_trials.py` |
| 场景和验收参数 | `configs/base/hand2_rotation_ball_v1.yaml` |
| 掌面估计与 neutral 标定 | `src/wujihand/adapters/input/mediapipe_palm_orientation.py`、`application/calibration/palm_orientation.py` |
| 姿态 contract 与安全监督 | `src/wujihand/domain/pose.py`、`application/supervision/pose_supervisor.py` |
| 原子 v2 命令与 UDP | `src/wujihand/ports/hand_command.py`、`adapters/transport/udp_hand_command.py` |
| D6、动态球与抓取判据 | `src/wujihand/adapters/simulation/hand2_rotation_mount.py`、`hand2_ball_scene.py`、`hand2_grasp.py` |
| 确定性抓取脚本 | `src/wujihand/runtime/rotation_ball_script.py` |

离线可达性网格、未被运行时读取的 session YAML 和逐帧命令副本已从最终实现移除。

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

执行步骤见 `docs/guides/mediapipe-hand2-rotation-ball.md`，最终证据见 `docs/validation/2026-07-13-hand2-rotation-ball.md`。
