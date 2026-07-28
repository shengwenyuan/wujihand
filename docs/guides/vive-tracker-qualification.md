# VIVE Tracker NV-1 资格验证指南

本指南在 `lenovo-piper2` 上只读验证一枚 Tracker。全过程不启动 Isaac、ROS 2 或
NERO，也不接受五层 `--session`；输出是 NV-1 input component 的 HIL 证据。

## 前置条件

1. Steam 与 SteamVR 已完成安装和更新，SteamVR runtime 正在运行。
2. HMD、Base Station、Tracker 与 dongle 已供电、配对并在 SteamVR 中可见。
3. 已完成 standing/room setup，并记录 tracking origin、Base Station 布局和 Tracker
   安装方向。
4. 目标机项目环境已安装 tracking extra：

```bash
cd /home/lenovo/swy/wujihand
source /home/lenovo/swy/venvs/wujihand-py312-nv1/bin/activate
python -m pip install -e '.[tracking]'
```

后续命令沿用这个独立 Python 3.12 venv。
每个 capture 必须使用新的输出目录；工具不会覆盖已有的五类 capture artifact。

## 1. 枚举并冻结身份

```bash
cd /home/lenovo/swy/wujihand
mkdir -p artifacts/nv1-vive
python tools/qualify_vive_tracker.py inventory \
  | tee artifacts/nv1-vive/inventory.json
```

从 JSON 中选择 `device_class=generic_tracker` 且 `connected=true` 的设备。后续
`--serial` 必须使用 inventory 返回的完整 serial；不要使用 USB ID 或 OpenVR device
index 代替。先把 serial、model、manufacturer、物理标签和拟分配 logical role 人工
核对一致，再开始采集。

如果 inventory 中没有目标 Tracker，先在 SteamVR UI 处理供电、配对或 runtime
问题，不创建临时 serial 映射。

## 2. 通用 capture

以下示例使用占位 serial `LHR-XXXXXXXX`。每个场景只替换 `--scenario` 和
`--output-dir`；同一枚 Tracker 保持 stream、role、tracking frame 不变。

```bash
python tools/qualify_vive_tracker.py capture \
  --serial LHR-XXXXXXXX \
  --stream-id vive.right \
  --logical-role operator_right \
  --tracking-frame vive_tracking \
  --scenario stationary \
  --duration-s 30 \
  --poll-hz 90 \
  --output-dir artifacts/nv1-vive/stationary-01
```

参数边界：

| 参数 | 含义与范围 |
|---|---|
| `--serial` | inventory 返回的稳定 Tracker serial；必填 |
| `--stream-id` | canonical stream identity；必填 |
| `--logical-role` | 可变的业务角色，不替代 serial；必填 |
| `--scenario` | 本次人工动作标签；必填 |
| `--duration-s` | `0.1`–`900` 秒，默认 `10` |
| `--poll-hz` | `1`–`500` Hz，默认 `90` |
| `--output-dir` | 本次独占 artifact 目录；必填 |
| `--tracking-frame` | 默认 `vive_tracking` |
| `--clutch-button-id` | 可选 OpenVR button bit，范围 `0`–`63` |
| `--clutch-input-id` | 默认 `tracker_clutch` |

capture 完成后在 stdout 打印 summary，并写入：

| 文件 | 内容 |
|---|---|
| `manifest.json` | 请求参数、选中设备、时间范围、CPU 统计和文件映射 |
| `raw_openvr.jsonl` | 每次 poll 对应的 JSON-safe OpenVR 原始记录 |
| `samples.jsonl` | canonical `TrackedRigidBodySample` |
| `events.jsonl` | canonical `ClutchEvent` |
| `summary.json` | valid ratio、rate、timestamp、spread、drift、dropout、重捕获和 CPU 指标 |

退出码 `0` 表示至少观察到一帧有效 pose；`3` 表示本次有完整证据但没有任何有效
pose，应检查 `summary.json` 和原始状态；运行、文件或严格 JSON 错误返回 `1`，
参数解析错误返回 `2`。这不是最终 NV-1 Gate 判定，方向、按钮和失联语义仍需人工
核对。

## 3. 人工场景

按以下顺序采集，每个场景使用独立目录并在实验记录中注明操作者、Tracker 固定方式和
动作时间点。

### 静止

刚性固定 Tracker，避免手持和桌面振动，采集至少 30 秒：

```bash
python tools/qualify_vive_tracker.py capture \
  --serial LHR-XXXXXXXX \
  --stream-id vive.right \
  --logical-role operator_right \
  --scenario stationary \
  --duration-s 30 \
  --poll-hz 90 \
  --output-dir artifacts/nv1-vive/stationary-01
```

检查 `timestamp_violation_count=0`、sample rate、valid ratio、position/orientation
RMS/peak 与首末有效 pose 的 position/orientation drift。少于两个有效 pose 时 drift
为 `null`。当前版本先记录基线，不在本指南中预设未验证阈值。

### 平移三轴

标记起点，分别沿已记录的 SteamVR standing frame X、Y、Z 正方向缓慢移动，建议使用
量尺给出约 `0.10 m` 的已知位移。分别使用：

```text
translate-x-positive
translate-y-positive
translate-z-positive
```

三个 scenario 标签和三个独立输出目录。检查 `samples.jsonl` 的 position delta：
单位必须为米，主变化轴和符号必须与现场 frame 记录一致，返回起点后应接近初始位置。

### 绕三轴转动

保持 Tracker 中心位置尽量不变，分别绕已标记 X、Y、Z 轴做可辨认的正向转动，再回到
初始姿态。使用：

```text
rotate-x-positive
rotate-y-positive
rotate-z-positive
```

检查 canonical quaternion 顺序为 active `wxyz`、连续帧无无意义符号翻转，并把三个
物理正方向与 standing frame 的观察结果写入验证记录。

### 遮挡、失联与重捕获

使用同一次 `occlusion-reacquisition` capture：

1. 先保持数秒有效跟踪。
2. 部分遮挡并记录开始/结束时刻。
3. 完全遮挡直到 pose 明确失效。
4. 恢复视线并保持静止，等待重新捕获。

无效区间的 sample 必须显式为非 `running` 状态，且
`position_m=null`、`quat_wxyz=null`、`quality=null`；恢复后才重新出现有效 pose。
检查 dropout count/duration 和 reacquisition duration。若要单独保存“全程不可见”
样例，退出码 `3` 是预期结果之一，但仍需确认 artifact 完整。

### 按钮 / clutch 候选

先确认目标硬件输入对应的 OpenVR button bit，再在 capture 中添加
`--clutch-button-id`。下面的 `2` 仅为命令格式示例，不代表 Tracker 的默认映射：

```bash
python tools/qualify_vive_tracker.py capture \
  --serial LHR-XXXXXXXX \
  --stream-id vive.right \
  --logical-role operator_right \
  --scenario clutch-button \
  --duration-s 15 \
  --poll-hz 90 \
  --clutch-button-id 2 \
  --clutch-input-id tracker_clutch \
  --output-dir artifacts/nv1-vive/clutch-button-01
```

执行至少两次完整按下/释放。检查 `events.jsonl` 中 press/release 成对、sequence 单调，
event 的 serial/stream/role/clock domain 与 pose stream 一致；press edge 应记录
`epoch_request=true`。

## 4. 离线回放

回放只读取 `samples.jsonl` 和 `events.jsonl`，重新执行严格解码与纯 metrics：

```bash
python tools/qualify_vive_tracker.py replay \
  --input-dir artifacts/nv1-vive/stationary-01 \
  --summary-output artifacts/nv1-vive/stationary-01-replay-summary.json
```

`--summary-output` 可省略，此时 summary 只打印到 stdout。目标文件已存在时工具会拒绝
覆盖。replay 的 metrics 和 event count 应与 capture summary 一致；replay scenario
固定为 `null`，也不包含 capture-only CPU 统计。

## 5. NV-1 Gate 记录

完成全部场景后再判定 NV-1：

- 重启 SteamVR 和采集进程后，同一物理 Tracker 仍返回同一 serial，并映射到同一
  stream/role 配置；
- valid pose finite，position 单位、active `wxyz`、tracking frame 和 host monotonic
  时间域均有证据；
- 平移与转动三轴的人工方向核对通过；
- 静止 spread/drift、采样率、CPU、timestamp、dropout 和 reacquisition 指标均已
  归档；
- 丢失期间没有复用旧 pose，重新捕获被显式记录；
- clutch 候选的 press/release 能与 pose stream 对齐；
- inventory、所有原始/canonical artifact、人工动作记录和最终摘要均保存并计算
  checksum。

任一项未完成时保持 NV-1 为 `IN_PROGRESS`，不把软件单测或 USB 枚举等同于硬件 Gate
通过。
