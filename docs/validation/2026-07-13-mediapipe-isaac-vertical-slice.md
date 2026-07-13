# 2026-07-13 MediaPipe—Isaac 垂直切片验证

## 结论

固定手掌、右手到右 Wuji Hand 2 的 20 手指关节实时控制已通过人工端到端验收。清理后的代码又通过 profile/URDF 契约、supervisor、严格 UDP、原生 Isaac Sim 5.1 脚本运动、UDP 断流回静息和 Isaac Lab 2.3.2 CUDA smoke。

该结论只覆盖无物体的手指镜像控制。它不等于已经实现几何物抓取、腕部/机械臂控制或训练数据录制。

## 固定版本

| 项目 | 版本/标识 |
|---|---|
| Isaac Sim | 5.1.0 standalone |
| Isaac Lab | v2.3.2 / `37ddf626871758333d6ed89cf64ad702aef127d0` |
| Isaac Lab core package | 0.54.2 |
| GPU / driver | RTX 4090 / 580.159.03 |
| Python | 3.11.13 |
| Wuji Description | v2026.6.27 / `aee64892ebcf8e3237bedc30231bb09476cbc71d` |
| Wuji SDK | 2026.7.2 |
| MediaPipe | 0.10.35 |

精确上游 URL、commit、路径和 SHA-256 见 `third_party/sources.lock.yaml`。

## 人工端到端验收

操作者于 2026-07-13 使用已连接的 D435、裸右手和 Isaac GUI 完成此前约定的全部动作测试，并确认效果完全正确：

- 相机画面、MediaPipe `Right` handedness 与 21 点跟踪正常。
- 人右手到 Hand 2 右手的各手指方向、逐指动作、张手和握拳一致。
- MediaPipe q20 经 loopback UDP 到 Isaac articulation 的实时联动正确。
- 手离开画面或遮挡后的安全回静息行为正确。
- 操作者主观确认当前交互效果可接受；本轮没有采集可引用的数值端到端延迟，因此不填写虚构指标。

## 清理后自动验证

| 检查 | 结果 |
|---|---|
| Ruff format/check | 通过 |
| mypy strict（`src`） | 通过，无问题 |
| 默认 pytest | 13 passed，1 socket test deselected |
| loopback socket pytest | 1 passed，13 deselected |
| 原生 Isaac scripted | 600 frames，20 DOF，limits/profile 匹配，反馈有限且在容差内，观察到运动，最终命令 0 rad，最终最大误差 0.16611 rad |
| 原生 Isaac UDP loss recovery | 3600 frames，接收 240、拒绝 0，tracking 254 ticks，断流后 degraded 268 ticks，最终命令 0 rad，反馈在限位容差内 |
| Isaac Lab CUDA smoke | `cuda:0`，ground/light/dynamic cube、reset 和 8 physics frames 通过 |

原生 Isaac 验证产物位于被 Git 忽略的：

- `artifacts/runs/isaac_hand2_scripted_clean/`
- `artifacts/runs/isaac_hand2_udp_final/`

每个新验证目录只包含 `validation.json`、`commands.json` 和 `hand2_table.png`；不再默认导出约 66 MB 的可重建 USDA。

## 关键命令

默认快速测试：

```bash
.venv/bin/ruff check src tests tools
.venv/bin/mypy src
.venv/bin/pytest
```

socket 集成：

```bash
.venv/bin/pytest -m requires_socket
```

原生 Isaac 脚本运动：

```bash
export ISAAC_SIM_ROOT=/path/to/isaac-sim-standalone-5.1.0-linux-x86_64
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_teleop.py \
  --command-source scripted --frames 600 \
  --validation-output-dir artifacts/runs/isaac_hand2_scripted_clean
```

UDP 断流回归需要先启动较长的 Isaac 接收端，再在其 scene 初始化完成后运行有限发送器：

```bash
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_teleop.py \
  --command-source udp --udp-port 49154 --frames 3600 \
  --require-udp-loss-recovery \
  --validation-output-dir artifacts/runs/isaac_hand2_udp_final

.venv/bin/python tests/integration/send_test_q20_udp.py \
  --port 49154 --seconds 4 --hz 60
```

Isaac Lab smoke：

```bash
env -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV TERM=xterm \
  third_party/src/IsaacLab/isaaclab.sh \
  -p tests/integration/isaaclab_smoke.py \
  --headless --device cuda:0 --frames 8
```

## 上游资产警告与环境提示

官方 Hand 2 right USD 在 Sim 5.1 中可加载和驱动，但仍有 5 个 fingertip `visuals` 引用未解析警告，以及一个 wrist collision fabric 警告。当前无物体视觉和关节运动验证通过；接触抓取、数据生成或资产升级时必须重新验收物理行为。

Isaac 启动日志还提示 CPU powersave、PCIe 当前 Gen1 x4 和 IOMMU。它们没有阻止本轮正确性测试，但正式实时性能基准前应复查；本报告不把本轮主观体验替代为延迟/抖动性能基准。
