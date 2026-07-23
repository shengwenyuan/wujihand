# 2026-07-23 五层架构与既有仿真链路验证

## 结论

`Asset Manifest → Backend Binding → Assembly Spec → Workcell → Session`
五层配置已经在锁定的真实资产、MuJoCo 和 Isaac Sim 运行环境中完成阶段性验证。
七个现有 Session 均可解析并校验实际资产；固定手 Isaac、转腕抓球 Isaac、
q20 UDP 遥操作接收端以及 MuJoCo FR3 v2—Hand 2 场景的既有能力均保留。

本轮没有引入 UR5、新的单双臂 Session、ROS 或真机控制实现。MediaPipe 本轮只完成
真实 D435I 的启动、采集和推理 smoke；由于画面中没有真人手，本报告**不声明**
live human retarget 或真人实时控制已经重新验收。

## 固定范围

| 项目 | 值 |
|---|---|
| 架构范围 | Asset Manifest、Backend Binding、Assembly Spec、Workcell、Session |
| GPU / driver | NVIDIA RTX 4090 / `580.159.03` |
| Isaac Sim | standalone `5.1.0` |
| MuJoCo | `3.10.0`，使用实际 Menagerie 与 Wuji Description 资产树 |
| Hand 2 模型 | Wuji Hand 2 Beta 1 right / `wuji-description v2026.6.27` |
| 相机 | Intel RealSense D435I |

五层解析产出的 `session_hash` 是规范化配置输入的确定性标识，适合发现配置、来源、
override 类型或 override 文件内容变化。它本身不证明后端成功加载资产、物理过程正确、
设备在线或一次执行已经通过；这些结论必须由实际资产校验和下述运行时 Gate 分别给出。

## 配置、真实资产与自动回归

| 检查 | 结果 |
|---|---|
| worktree 默认快速全仓测试 | `210 passed, 12 skipped, 7 deselected` |
| loopback UDP socket suite | `7 passed, 222 deselected` |
| Ruff | 全仓通过 |
| mypy | `45` 个 source file 通过 |
| 实际资产测试集 | `222 passed, 7 deselected` |
| 七个 Session 的 `verify_artifacts=True` | `7/7` 通过 |
| MuJoCo 真实资产契约与集成 | 已包含在实际资产测试集并通过 |

快速门禁使用主项目的 Python 3.11 环境执行；当前 worktree 内未恢复固定第三方
checkout，因此 12 项资产相关用例按设计 skip。随后把同一源代码和配置同步到含真实
资产的隔离副本执行实际资产测试，避免把环境 skip 当成通过。

实际资产测试在包含真实 `mujoco_menagerie` 和 `wuji-description` 的隔离副本中执行，
不是仅用占位 fixture 验证路径存在。`verify_artifacts=True` 对七个现有 Session 的
锁定来源和内容执行验证；该结果与 Session 解析/golden hash 测试共同覆盖配置输入，
但仍不替代 GPU 运行结果。

### 执行命令与证据位置

最终 worktree 门禁：

```bash
cd /home/yanziwei/.codex/worktrees/c5db/wujihand
/home/yanziwei/swy/wujihand/.venv/bin/python -m pytest -q
/home/yanziwei/swy/wujihand/.venv/bin/python -m pytest -q -m requires_socket
/home/yanziwei/swy/wujihand/.venv/bin/ruff check .
/home/yanziwei/swy/wujihand/.venv/bin/mypy src
git diff --check
```

最终代码、配置和测试另同步到
`/tmp/wujihand-five-layer-final2.iHeuKp`，并从主 checkout 复制锁定的
`third_party/src/mujoco_menagerie` 与 `third_party/src/wuji-description`。该目录是
临时验证副本，不是长期 artifact；执行命令为：

```bash
cd /tmp/wujihand-five-layer-final2.iHeuKp
/home/yanziwei/swy/wujihand/.venv/bin/python -m pytest -q
env PYTHONPATH=src /home/yanziwei/swy/wujihand/.venv/bin/python -c \
  'from pathlib import Path; from wujihand.runtime import SessionResolver; root=Path.cwd(); resolver=SessionResolver(root); sessions=sorted((root / "configs/sessions").glob("*.yaml")); [resolver.resolve(path, verify_artifacts=True) for path in sessions]; print(f"verified_sessions={len(sessions)}")'
```

Isaac/设备运行使用另一份含相同 `src/`、`configs/`、`tools/` 内容的临时副本
`/tmp/wujihand-five-layer-final.sCg5Fr`；提交前用目录对照确认这三部分与 worktree
无差异。Isaac 解释器为：

```text
/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh
```

对应入口：

```bash
cd /tmp/wujihand-five-layer-final.sCg5Fr

/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh \
  tools/run_isaac_hand2_teleop.py \
  --session configs/sessions/isaac_hand2_fixed_preview_v1.yaml \
  --command-source scripted --frames 600 \
  --validation-output-dir artifacts/isaac-fixed-final

/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh \
  tools/run_isaac_hand2_rotation_ball.py \
  --session configs/sessions/isaac_hand2_right_rotation_ball_qualification_v1.yaml \
  --command-source scripted --frames 1200 --require-grasp-success \
  --validation-output-dir artifacts/isaac-rotation-final

/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh \
  tools/run_isaac_hand2_teleop.py \
  --session configs/sessions/isaac_hand2_teleop_v1.yaml \
  --command-source udp --udp-port 49166 --frames 3600 \
  --validation-output-dir artifacts/isaac-udp-final

/home/yanziwei/swy/wujihand/.venv/bin/python \
  tools/run_mediapipe_hand2_teleop.py --frames 30 --headless
```

UDP consumer 完成场景订阅后，由一次性 bounded local sender 以 `60 Hz` 发送 `1500`
个规范 q20 包；该临时 sender 没有写入仓库，因此不把它描述成长期可复现工具。
consumer 的逐帧 command/feedback 与 packet sequence 已写入
`artifacts/isaac-udp-final/commands.json`，最终统计写入同目录
`validation.json`。其他 GPU 证据分别位于
`artifacts/isaac-fixed-final/validation.json` 和
`artifacts/isaac-rotation-final/validation.json`。这些 `/tmp` 路径可能被系统清理，
本正式报告保留了验收数值；后续若要求长期逐帧复现，应把 bounded sender 提升为受测
工具并将运行 artifact 归档到约定存储。

## Isaac 固定手场景

固定手 preview Session 在 Isaac Sim 中执行 `600` physics frames，进程正常退出。

| Gate / 指标 | 结果 |
|---|---:|
| Session hash | `246899c5dce7b8c7c1de75e963e6c7b4aede0e780c755b1929c33d17d48c6472` |
| 实际 Hand 2 USD SHA-256 | `3cb3dcb18b07621a52a47a8daa98ab82794e3c77d36275d068b3b5b0516e5f00` |
| runtime DoF | `20` |
| movement | `true` |
| limits / finite | `true / true` |
| feedback within limits | `true` |

该运行同时证明固定手兼容入口消费了五层 Session、加载了锁定 USD，并在实际
articulation 上观察到有限且受限位约束的运动。

## Isaac 转腕抓球场景

转腕抓球 qualification Session 以 `--require-grasp-success` 执行 `1200` physics
frames，严格结构、运动和抓取 Gate 通过。

| Gate / 指标 | 结果 |
|---|---:|
| Session hash | `24f6edeb140870a7e91bd3f4fa7967ffb8f3ecea1f79ba5a6fa8c5601b3ec063` |
| runtime DoF | `23`，wrist3 + finger20 |
| 首次抓取通过 | `8.875 s` |
| 最佳连续保持 | `2.125 s` |
| 最大固定法兰平移误差 | 约 `3.408e-08 m` |
| Hand 2—table contact | `0` 接触帧 |
| structural / movement / grasp | `true / true / true` |

本结果验证当前 wrist + Hand 2 Assembly、场景 Workcell 和 qualification Session
仍能由既有 Isaac runner 执行；它不扩展为任意新 Assembly 的通用 Isaac 编译能力。

## q20 UDP 遥操作接收链路

固定手 teleop Session 在 Isaac Sim 中执行 `3600` frames；有限发送器在场景初始化后
发送 `1500` 个 q20 数据包。

| Gate / 指标 | 结果 |
|---|---:|
| Session hash | `50dec3f6f3bf6a5f4c47057e2580b276c5c61d76349ab5fd998680a001e3f8aa` |
| accepted / rejected packets | `1500 / 0` |
| tracking ticks | `1511` |
| movement | `true` |
| feedback within limits | `true` |

该结果验证的是规范 q20 UDP producer/consumer 契约与 Isaac 接收、监督和执行链路，
不等同于真实视觉输入的跟踪质量或真人遥操作体验验收。

## RealSense 与 MediaPipe smoke

真实 D435I 已完成 `30` 帧 headless 启动、采集和 MediaPipe 推理 smoke。观测吞吐约
`20.6 FPS`，单帧推理约 `10 ms`；相机和模型均能正常启动、采集与执行推理。

本次镜头中没有真人手，统计为 `valid_right=0`、`missing=30`。因此只确认设备/模型
启动与数据流 smoke，不声称以下能力在本轮被重新验证：

- 真人右手 21 点连续追踪；
- 21×3 到 q20 的实际 live retarget 效果；
- MediaPipe producer 到 Isaac 的真人闭环控制；
- 遮挡、掉手、延迟、抖动或主观操控体验。

## 结论边界

- 五层架构的配置解析、来源锁定、实际资产检查和既有 runner 兼容链路已得到自动与
  GPU 运行证据；worktree 默认快速全仓测试为
  `210 passed, 12 skipped, 7 deselected`。
- 当前专用 runner 对已知拓扑 fail closed；本报告不把兼容桥描述为任意资产、任意
  单双臂组合的通用 backend compiler。
- Session hash 只标识规范化输入。资产内容、Isaac/MuJoCo 加载、物理抓取和设备数据流
  分别依赖各自的实际运行 Gate，不能由 hash 推导。
- 本轮没有重新进行真人手入镜的 live MediaPipe retarget/control 验收；此前能力是否
  满足新的体验或性能目标，仍需单独人工复验。
