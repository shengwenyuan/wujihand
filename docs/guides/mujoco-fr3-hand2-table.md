# MuJoCo FR3—Wuji Hand 2 桌面运行指南

## 1. 安装

在仓库根目录使用项目 Python 3.11 环境：

共享项目环境建议保留全部既有能力：

```bash
uv sync --all-extras
```

若是只运行本组件的独立最小环境，可用 `uv sync --extra mujoco`；`uv sync` 会使环境与所选 extras 对齐，因此不要在需要 perception/retarget 的共享环境里漏掉它们。

固定版本是 `mujoco==3.10.0`。如果下载因临时 DNS/网络错误失败，直接重试；只有 `df -h` 明确显示空间不足时才先处理磁盘。

## 2. 恢复模型

模型固定信息以 `third_party/sources.lock.yaml` 为准。FR3 v2 的最小恢复方式：

```bash
git clone --filter=blob:none --no-checkout \
  https://github.com/google-deepmind/mujoco_menagerie.git \
  third_party/src/mujoco_menagerie
git -C third_party/src/mujoco_menagerie sparse-checkout init --cone
git -C third_party/src/mujoco_menagerie sparse-checkout set franka_fr3_v2
git -C third_party/src/mujoco_menagerie checkout --detach \
  71f066ad0be9cd271f7ed58c030243ef157af9f4
```

Wuji source checkout 必须包含：

```text
hand2_beta/body/mjcf
hand2_beta/body/meshes/right
hand2_beta/body/urdf
hand2_beta/body/usd/right
```

从零恢复的完整命令：

```bash
git clone --filter=blob:none --no-checkout \
  https://github.com/wuji-technology/wuji-description.git \
  third_party/src/wuji-description
git -C third_party/src/wuji-description sparse-checkout init --cone
git -C third_party/src/wuji-description sparse-checkout set \
  hand2_beta/body/urdf \
  hand2_beta/body/meshes/right \
  hand2_beta/body/mjcf \
  hand2_beta/body/usd/right
git -C third_party/src/wuji-description checkout --detach \
  aee64892ebcf8e3237bedc30231bb09476cbc71d
```

runner 会在编译前计算两份 MJCF 和两棵 mesh 目录清单的 SHA-256；文件缺失、被修改或 hash 不一致时直接退出。

### 2.1 当前布局

默认 profile 使用 `1.60 × 1.00 m` 桌面，以及 `y_max` 长边外侧的凸四棱台。台座顶高 `0.47 m`，FR3 base 固定在 `[0, 0.82, 0.47] m` 并朝桌心；编译后 joint2 应为 `z=0.803 m`，比 `0.75 m` 桌面高 `0.053 m`。runner 的 `scene_geometry` 回显桌面、台座和基座配置，并输出编译态 joint2 实测值；adapter 启动时另行核对 compiled base pose、基座 footprint 和 joint2 净高。

默认 camera headlight 已关闭；单盏 `observation_light` 只改善 GUI/EGL 观察，未参与接触或控制计算。overview 相机从台座一侧取景，默认画面应同时包含完整桌面、四棱台、FR3 和 Hand 2。

## 3. Headless smoke

```bash
.venv/bin/python tools/run_mujoco_fr3_hand2_table.py --duration-s 5
```

默认在 FR3 home + Hand 2 q20 rest 运行 5 秒并向 stdout 输出 JSON，包含实际 MuJoCo 版本、资产 hash、27-DoF 映射、初末状态、接触数和固定 attachment 漂移。保存报告：

```bash
.venv/bin/python tools/run_mujoco_fr3_hand2_table.py \
  --duration-s 10 \
  --report artifacts/runs/mujoco_fr3_hand2/validation.json
```

## 4. GUI

有桌面显示会话时：

```bash
.venv/bin/python tools/run_mujoco_fr3_hand2_table.py --gui --duration-s 30
```

viewer 关闭后会提前结束。GUI 只用于观察；状态、映射和 hash 检查与 headless 使用同一 adapter。

## 5. 设置关节目标

FR3 目标采用 profile 顺序 `fr3v2_joint1..7`。例如在安全 home 附近移动 joint1：

```bash
.venv/bin/python tools/run_mujoco_fr3_hand2_table.py \
  --duration-s 3 \
  --arm-target 0.05 0 0 -1.57079 0 1.57079 -0.7853
```

`--hand-target` 接受 canonical Hand 2 right q20；不提供时保持 rest。任何 shape、NaN/Inf 或限位错误都会拒绝执行。程序 API 使用：

```python
from pathlib import Path

from wujihand.adapters.simulation import MujocoFr3Hand2
from wujihand.runtime import load_mujoco_table_scene_config

root = Path.cwd()
config = load_mujoco_table_scene_config(
    root / "configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml"
)
environment = MujocoFr3Hand2.from_config(config, project_root=root)
environment.set_joint_targets(arm_q7, hand_q20)
state = environment.step()  # one 100 Hz control tick = five 2 ms physics steps
```

## 6. 离屏图像

无 X11 环境时通常使用 EGL：

```bash
MUJOCO_GL=egl .venv/bin/python tools/run_mujoco_fr3_hand2_table.py \
  --duration-s 1 \
  --render-ppm artifacts/runs/mujoco_fr3_hand2/overview.ppm
```

PPM 是无额外图像依赖的 RGB 输出。若 EGL 初始化失败，先确认显卡驱动/EGL 可用；也可去掉 `MUJOCO_GL=egl` 在有显示会话中渲染。

## 7. 验证

```bash
.venv/bin/python -m pytest \
  tests/unit/test_mujoco_table_config.py \
  tests/contract/test_mujoco_fr3_hand2_model.py \
  tests/integration/test_mujoco_fr3_hand2_smoke.py
.venv/bin/python -m ruff check src tests tools
.venv/bin/python -m mypy src
```

测试覆盖严格配置、四棱台凸网格、观察灯、joint2 净高、27-DoF/actuator 契约、固定法兰、确定性 reset、10 秒 home hold、arm/hand 小阶跃、平滑 Hand 2 桌面接触和 headless runner。

## 8. 当前限制

- 不要把 identity attachment 直接用于加工转接件；先用官方 STEP 和 FR3 法兰图纸求得真实 transform、质量和惯量。
- 当前桌面、四棱台和 `53 mm` joint2 净高是仿真布局，不是台座承载、抗倾覆、孔位或真实工作站的人机工程结论。
- GUI/仿真位置执行器不是 FR3 真机控制或安全验证。
- Hand 2 proximal 段的上游碰撞 bit 有意不全；需要完整接触覆盖时另建、版本化派生 collision profile，不能静默修改官方模型。
- 本入口没有 IK、末端位姿遥操作、Rokoko/Wuji Glove、采集或策略接口。
