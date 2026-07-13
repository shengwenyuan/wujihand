# MediaPipe—Wuji Hand 2—Isaac 控制链路

状态：2026-07-13 已完成真人右手端到端验收，并在清理后重新通过脚本运动、loopback UDP 断流恢复和 Isaac Lab smoke。当前能力是固定手掌、20 个手指关节的实时镜像控制，不包含腕部、机械臂或物体抓取任务。

## 正式入口

| 职责 | 入口 |
|---|---|
| D435、MediaPipe、Wuji retarget 与 UDP 发布 | `tools/run_mediapipe_hand2_teleop.py` |
| 原生 Isaac Sim 5.1 场景、监督与 UDP 接收 | `tools/run_isaac_hand2_teleop.py` |
| Hand 2 派生 profile | `configs/profiles/hand2_right_v2026_6_27.yaml` |
| profile 加载和 firmware/backend 重排 | `src/wujihand/adapters/simulation/hand2_model.py` |
| loopback q20 协议 | `src/wujihand/adapters/transport/udp_joint_command.py` |
| 安全监督 | `src/wujihand/application/supervision/joint_supervisor.py` |
| 关节布局与静息姿态 | `src/wujihand/domain/hand2.py` |
| 上游版本、commit 与资产 hash | `third_party/sources.lock.yaml` |

`tools/` 只保留两个真实运行入口。无摄像头 q20 发送器和 Isaac Lab smoke 属于集成验证，分别位于 `tests/integration/send_test_q20_udp.py`、`tests/integration/isaaclab_smoke.py`。

## MediaPipe 到 Isaac 的运行链

```text
D435 RGB8 640x480 @ 30 Hz
  -> MediaPipe HandLandmarker VIDEO
  -> Right hand_world_landmarks, float32 (21, 3), metres
  -> Wuji RetargetSession(WujiHand2, Right).step()
  -> firmware-order q20, radians
  -> UdpJointCommandSender
  -> 127.0.0.1 JSON datagram: wujihand.q20.v1
  -> UdpJointCommandReceiver.receive_latest()
  -> JointCommandSupervisor @ 60 Hz
  -> Hand2ModelProfile.firmware_to_backend()
  -> Articulation.set_joint_position_targets()
  -> Isaac physics @ 120 Hz
```

具体调用顺序：

1. `run_mediapipe_hand2_teleop.py:main()` 从 D435 读取原始 RGB8；normalized landmarks 只用于预览，控制使用 `hand_world_landmarks[0]`。
2. 只接受 MediaPipe 分类为 `Right` 的单手。`RetargetSession.for_hand(HandModel.WujiHand2, Handedness.Right)` 将 `(21, 3)` 世界关键点映射为 20 维关节角；shape 或有限性异常会立即终止，不发送坏数据。
3. `UdpJointCommandSender.send()` 再按 `HAND2_RIGHT_LAYOUT` 校验 q20，并发送固定 schema/layout、session UUID、递增 sequence、`monotonic_ns` 和 q20。协议只允许 `127.0.0.1`，最大 4096 bytes，不使用 pickle。
4. Isaac 每个 60 Hz command tick 调用 `receive_latest()`，排空当前数据报并按 `host_time_ns` 只保留最新包；空轮询期间暂存最后一个已验证目标。
5. `JointCommandSupervisor.step()` 是唯一控制出口：拒绝错误 shape、NaN/Inf、未来时间戳和陈旧输入；先 clamp 到 profile limits，再按官方速度上限的 20% 限制单 tick 变化。
6. `Hand2ModelProfile.firmware_to_backend()` 按 Isaac 运行时的 `hand.dof_names` 显式重排。当前实测 firmware indices 为 `[4, 8, 16, 12, 0, 5, 9, 17, 13, 1, 6, 10, 18, 14, 2, 7, 11, 19, 15, 3]`，随后才写入 articulation。
7. 反馈通过 `backend_to_firmware()` 逆映射，以 canonical q20 顺序做限位、误差和验证记录。

MediaPipe 连续约半秒无右手时会清空 retargeter 的 warm-start/filter state；它不会伪造 stop 包。最后一条输入年龄超过 250 ms 时，Isaac supervisor 进入 `DEGRADED`，按同一速度限制逐步回到全零静息姿态。`age == 250 ms` 仍有效，`age > 250 ms` 才触发陈旧策略。

## Isaac 初始化与资源挂载链

```text
third_party/sources.lock.yaml
  -> restored wuji-description v2026.6.27 checkout
  -> hand2_beta/body/usd/right/wujihand.usd
  -> add_reference_to_stage(..., "/World/Hand2")
  -> discover /World/Hand2/root_joint ArticulationRootAPI
  -> Articulation("hand2_right")

configs/profiles/hand2_right_v2026_6_27.yaml
  -> load_hand2_model_profile()
  -> verify product/right/20 DOF/domain layout/rest
  -> verify runtime USD DOF names and limits
  -> build explicit firmware <-> backend mapping
```

初始化顺序：

1. CLI 在启动 Isaac 前确认 USD 和 profile 文件存在；`SimulationApp` 必须先创建，随后才导入 `pxr` 与 `isaacsim` API。
2. `World` 使用 metre 单位、120 Hz physics、30 Hz rendering 和 NumPy backend；代码挂载默认 ground、`/World/Table` 固定方块及 `/World/DomeLight`。
3. `add_reference_to_stage()` 将官方 right-hand 顶层 USD 挂到 `/World/Hand2`。该 USD 的 Physics/Sensor/Robot variants 继续组合 `configuration/wujihand_physics.usd`、`wujihand_base.usd`、`wujihand_sensor.usd` 和 `wujihand_robot.usd` 等同目录资源。
4. 代码扫描且只接受一个位于 `/World/Hand2` 下的 `ArticulationRootAPI`；当前根为 `/World/Hand2/root_joint`。`world.reset()` 后将固定手掌置于桌面上方、掌心朝世界 `+Z`，再设置观察相机。
5. 运行时读取派生 profile，要求它与代码中固定的 firmware layout/rest 完全一致；随后读取实际 `hand.dof_names` 和 USD limits。任一关节缺失、重复、顺序集合不同或 limits 不匹配都会 fail closed，不进入控制循环。
6. 最后创建 supervisor，并在 UDP 模式绑定 loopback receiver。原生控制场景不依赖 Isaac Lab；Isaac Lab 2.3.2 只作为未来任务系统的独立安装基线。

上游 checkout、MediaPipe 模型与运行产物均被 Git 忽略。新环境必须先按 `third_party/sources.lock.yaml` 恢复固定 tag/commit/hash；本机 Isaac 安装路径属于本地环境，不写入来源锁。

## 运行命令

先启动 Isaac 接收端：

```bash
export ISAAC_SIM_ROOT=/path/to/isaac-sim-standalone-5.1.0-linux-x86_64
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_teleop.py \
  --gui --command-source udp --udp-port 49152 --frames 36000
```

再启动 D435 右手 worker：

```bash
.venv/bin/python tools/run_mediapipe_hand2_teleop.py \
  --publish-udp-port 49152
```

正常 live 模式只在退出时打印短摘要，不保存逐帧日志或导出 USD。若要执行可复现的脚本资格验证，显式指定输出目录：

```bash
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_teleop.py \
  --command-source scripted --frames 600 \
  --validation-output-dir artifacts/runs/isaac_hand2_scripted
```

验证模式只写 `validation.json`、`commands.json` 和 `hand2_table.png`。无摄像头 UDP 轨迹和 Isaac Lab smoke 的命令见验证报告。

## 已知边界

- 当前 composition root 仍是两个 CLI；尚未拆出通用 runtime orchestration、MediaPipe input adapter 或 Wuji retarget adapter。
- 当前只控制固定基座 Hand 2 右手的 20 个手指关节；几何物抓取、reward/task state、HDF5、LeRobot、腕部/机械臂均未实现。
- receiver 以单调 `host_time_ns` 选择最新包；sequence 用于观测和记录，当前不承担跨 session 排序。
- 官方 Hand 2 USD 在 Sim 5.1 仍报告 5 个 fingertip `visuals` unresolved-reference 警告及 wrist collision fabric 警告；加载、20 DOF 驱动和当前视觉验证可用，但接触任务前必须重新资格验证。
