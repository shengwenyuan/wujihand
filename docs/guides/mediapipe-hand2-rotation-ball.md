# MediaPipe 控制 Hand 2 转向抓球指南

适用范围：Wuji Hand 2 Beta 1 right，固定 XYZ、物理 `+60°` task home、D6 三轴转向和 30 mm 动态球。

## 1. 自动检查

```bash
.venv/bin/ruff check .
.venv/bin/mypy src/wujihand
.venv/bin/pytest -q
.venv/bin/pytest -m requires_socket
```

其中 fast suite 会在不启动 Isaac 或相机的情况下解析全部 7 个正式 Session，并校验
五层引用、固定 `wuji-description v2026.6.27` Binding、v1/v2 transport 配对、
compatibility leaf 对照和架构依赖方向。它不能替代下面的 authored-stage、PhysX 或
真人设备回归。

Isaac USD overlay 快速检查：

```bash
export ISAAC_SIM_ROOT=/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  "$ISAAC_SIM_ROOT/python.sh" \
  tests/integration/isaac_hand2_rotation_ball_smoke.py --author-only
```

期望只有 `/World/Hand2Mount/world_fixed_joint` 一个 articulation root，旧 root 为 disabled。

## 2. 严格脚本抓取

```bash
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_rotation_ball.py \
  --session configs/sessions/isaac_hand2_right_rotation_ball_qualification_v1.yaml \
  --command-source scripted \
  --frames 1200 \
  --require-grasp-success \
  --validation-output-dir artifacts/runs/isaac_hand2_rotation_ball_final
```

退出码为 0 且 `validation.json` 中 `structural_passed`、`movement_observed`、`grasp_passed` 均为 `true` 才算通过。输出目录只保留核心 `validation.json` 和可选截图；完整释放/回 home 可用 1560 frames。

重复性 gate：

```bash
.venv/bin/python tools/run_isaac_hand2_rotation_ball_trials.py \
  --isaac-python "$ISAAC_SIM_ROOT/python.sh" \
  --trials 10 --required-successes 8 --frames 1200 \
  --output-dir artifacts/runs/isaac_hand2_rotation_ball_trials
```

## 3. 真人 MediaPipe

终端 A 启动 Isaac：

```bash
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_rotation_ball.py \
  --session configs/sessions/isaac_hand2_right_rotation_ball_teleop_v1.yaml \
  --command-source udp --udp-port 49152 --gui --frames 36000 \
  --validation-output-dir artifacts/runs/isaac_hand2_rotation_ball_live
```

终端 B 启动 D435/MediaPipe/Wuji worker：

```bash
.venv/bin/python tools/run_mediapipe_hand2_teleop.py \
  --session configs/sessions/mediapipe_hand2_hand_command_udp_v1.yaml \
  --publish-hand-command-port 49152
```

`--publish-hand-command-port` 发送 q20+rotation v2；legacy `--publish-udp-port` 仅发送 q20 v1，两者互斥。

`--session` 都是可选的：Isaac rotation-ball 按 `--command-source` 自动选择
qualification 或 teleop-consumer Session；MediaPipe 按 publish 参数自动选择 q20 v1
或 q20+pose v2 producer Session。因此旧命令继续可用。显式 Session 的 runtime role
或 transport contract 与命令模式不一致时会在设备/backend 启动前拒绝。

Isaac 的旧 `--asset`、`--profile`、`--scene-profile` 仍可作为有意 override 使用，并
会连同文件内容 hash 进入 `session_hash`；各 runner 仍执行对应的实际 USD SHA-256、
layout 和场景检查。fixed-hand 同时提供 `--asset` 与 `--profile` 时，二者可有意脱离
Session Binding/source lock，只保留内容一致性和模型 gate，不能据此声称 pinned
provenance；正式运行优先通过默认或显式 `--session` 选择锁定组合。每份 Isaac
`validation.json` 记录
`session`/`session_hash`，MediaPipe 启动日志也打印这两个字段，现场验收应一并留存。

## 4. Neutral 与 clutch

1. 裸右手以舒适、小角度姿态放入 D435 视野，保持约 15 个稳定帧。
2. `pose=tracking` 后，先做小幅 yaw/pitch/roll 检查方向和桌面净空。
3. 对准球体后缓慢收拢五指；避免遮挡掌根和 MCP landmarks。
4. 需要换一个舒适姿态继续转动时，停稳后按 `c`，保持新姿态至少 3 个有效帧。

clutch 不会把 Isaac 腕部弹回 60° home。它把“当前真人手姿”重新定义为 identity，并以“当前 Isaac 腕姿”为新 anchor；之后只叠加新的相对转动。每次 clutch 使用新的 `calibration_id`，identity 连发 3 帧，避免 UDP newest-only 读取漏掉重新 arm 帧。

快捷键：`c` clutch，`space` 打印 q20/姿态，`r` 重置 retarget 与姿态标定，`q`/Esc 退出。

## 5. `pose=disabled` 排查

`pose=disabled`（有时误写为 `diable`）表示尚未建立可安全跟踪的 calibration epoch，常见原因：

- 右手未稳定出现满 15 帧，spread 超过 `8°`，质量低于 `0.50`，或有效帧间隔超过 `100 ms`；
- 手丢失超过 `500 ms`，旧 calibration 已作废；
- 新 epoch 的首包不是 identity，Isaac 会 fail closed 拒绝 arm；
- 仍在使用 q20 v1 的 `--publish-udp-port`，没有发送腕姿；
- 两个进程端口不一致，或 v2 包被 schema/时间戳/sequence 校验拒绝。

恢复顺序：手放稳并减少遮挡 → 按 `r` 重新标定，或停稳按 `c` → 等待 `pose=tracking` → 再小角度操作。不要通过放宽 quaternion、时间戳或 epoch 校验绕过 disabled。

输入老化到 `250 ms` 时 Isaac 进入 degraded 并保持最后安全姿态，`500 ms` 后 disarm；恢复必须建立新 calibration epoch。

## 6. 人工验收

- 确认法兰 XYZ 不漂移、初始手掌物理 pitch 为 `60°`、球初始落在桌面。
- 分别小幅操作 yaw/pitch/roll，方向正确且无桌面碰撞。
- 至少完成一次“转向对准 → 五指收拢 → 球离桌并保持 → 释放”。
- 检查 `validation.json` 的 UDP accepted/rejected、监督状态、固定法兰误差和抓取判据。

当前人工测试已成功抓起球；操作不够顺畅属于 MediaPipe landmark/姿态稳定性限制，不影响本需求的功能验收结论。

以上真人抓球结论来自原 2026-07-13 验证。2026-07-23 五层改造后已重新执行 Isaac
fixed/rotation 物理回归、合成 q20 UDP 接收回归，以及 D435I + MediaPipe 的 30 帧
启动/采集/推理 smoke；相机画面中没有真人手，因此本轮没有重新验收真人 landmark、
live retarget 或真人闭环控制。精确结果见
[2026-07-23 五层架构与既有仿真链路验证](../validation/2026-07-23-five-layer-architecture.md)。
今后的 fast suite 通过仍只证明配置兼容，不能替代 GPU、设备或真人操作验收。
