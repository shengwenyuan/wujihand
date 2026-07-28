# 2026-07-28 NV-1 VIVE 最小跟踪验证

状态：`IN_PROGRESS`
软件部署与离线 contract 已通过；真实 OpenVR HIL 等待 HMD 接入。

## 边界

本验证遵循 ADR-0004：NV-1 是只读 VIVE input component qualification，不创建或修改
Asset Manifest、Backend Binding、Assembly、Workcell 或 Simulation Session，不启动
Isaac、ROS 2、NERO、UDP command 或机械臂控制。后续真实 NERO Isaac consumer 仍以
五层 Session 为目标组合根。

## 部署

| 项目 | 实际值 |
|---|---|
| 主机 | `lenovo@Workstation2` / SSH alias `lenovo-piper2` |
| project | `/home/lenovo/swy/wujihand` |
| NV-1 venv | `/home/lenovo/swy/venvs/wujihand-py312-nv1` |
| Python / NumPy | `3.12.3` / `2.2.6` |
| OpenVR binding | `openvr 2.12.1401` |
| SteamVR | App 250820，BuildID `23791826`，runtime `2.16.7` |
| VR Server | `v1781734990` |
| artifact 根目录 | `/home/lenovo/swy/wujihand/artifacts/nv1-vive/` |

已部署：

- canonical `TrackedRigidBodySample`、`ClutchEvent` 与 tracking port；
- serial-addressed `OpenVrTrackerAdapter`；
- strict sample/event JSONL codec；
- rate、valid ratio、timestamp、stationary spread、short-term drift、dropout、
  reacquisition 与 capture CPU metrics；
- `inventory`、bounded `capture`、strict `replay` 诊断 CLI。

## 软件验证

目标机 Python 3.12 全仓结果为：

```text
298 passed, 2 skipped, 7 deselected
PYTEST_EXIT=0
```

两个 skip 都是未安装的可选 MuJoCo runtime；恢复的上游 Hand 2 asset contract 已通过。
Ruff、`mypy src` 及 qualification CLI/test 的 strict mypy 均退出 `0`。

离线测试覆盖 domain/port 不变量、OpenVR 3×4 matrix 转换、serial 重解析、device loss、
quaternion hemisphere、button edge、strict JSONL、metrics、CLI 产物和依赖方向。架构守卫
禁止 specs/domain/ports/application/compat 依赖 OpenVR 或 ROS SDK。

## SteamVR 与 USB 现场状态

当前 USB：

| USB ID | 名称 | serial |
|---|---|---|
| `28de:2101` | Watchman Dongle | `3C15A0301B` |
| `28de:2300` | Valve LHR | `LHR-24B6E288` |

四个对应 hidraw 节点均给 `lenovo` 读写 ACL。SteamVR lighthouse driver 已打开
`LHR-24B6E288` 的 IMU、Optical 和 VrController HID，并把其几何识别为与
`HTC Tracker 3` 相符；该日志仍不能替代 OpenVR inventory 的正式 device class/role
绑定。

目标机禁用了非特权 user namespace。没有改动该系统安全策略；本次继承已运行 Steam
客户端的 runtime library path，直接启动官方 `vrmonitor.sh`，`vrmonitor` 与
`vrserver` 已进入运行态。

## 当前阻塞证据

安装 SteamVR 前，CLI 正确报告：

```text
error: InitError_Init_PathRegistryNotFound
```

SteamVR 安装并启动后，路径注册问题消失；由于当前没有 HMD USB/显示设备枚举，
OpenVR background application 被 vrserver 明确拒绝：

```text
error: InitError_Init_HmdNotFound
```

因此尚不能把 USB serial 分配为 canonical logical role，也尚未产生真实
`TrackedRigidBodySample`。本状态不是 NV-1 Gate 通过。

## 证据 SHA-256

| 证据 | SHA-256 |
|---|---|
| `python312-full-tests.log` | `1cdf0c117d33d18a352ef7d7d471425675216d30368b18817874e016c9c6652d` |
| `python312-static-checks.log` | `6859cf91d99362dc6c277b4fd27a5d9055bd3021ee0d45c9eee8e8f103b995a6` |
| `runtime-versions.log` | `d189eb4a4f7f3b1166488851a2f18cb3f8d991530f5b23b9e18079981db3bb39` |
| `inventory-before-steamvr-v2.log` | `1a6a21e8e2ef6830c616207f003131ebd436fc51b0fd49313a74cda87632c822` |
| `inventory-initial.stderr` | `15c62ccd4402488410d776450e948ad754271725369b17ac11eab1d9b4183a66` |
| `vrmonitor-launch.log` | `7465ef396b3fcccd29d9f14fc8c8fc47a55f0970228c55fe4f6405d08893070f` |
| `vrmonitor-launch-with-runtime.log` | `7025088d7e79e30945f64201ebc5d2b86d437e6631670c0cff22a82b5596d0d5` |

## Gate 剩余项

1. 接入 HMD 的电源、USB 与显示链路，使官方 OpenVR background application 启动。
2. 保存正式 inventory；按 serial、device class、model、物理标签确认 logical role。
3. 完成 standing/tracking origin 与 Base Station 布局记录。
4. 采集静止、三轴平移、三轴旋转、遮挡/失联/重捕获和按钮场景。
5. 人工确认单位、frame、active `wxyz`、方向与 clutch edge。
6. 保存 raw/canonical artifact、summary、replay 和 checksum 后再判定 NV-1 Gate。
