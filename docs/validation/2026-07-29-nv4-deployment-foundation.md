# 2026-07-29 NV-4 Deployment 与 mapping v3 基础验证

> 这是 NV-4 第一批基础切片的阶段快照；其“runner 尚未接入”等描述保留为当时事实。
> 后续实现与目标机预检见
> [2026-07-29 NV-4 原生双侧实现与 Workstation2 预检](2026-07-29-nv4-native-dual-preflight.md)。

| 字段 | 值 |
|---|---|
| 状态 | `PARTIAL / FOUNDATION_PASS` |
| 范围 | NV-4A 文档/回归基础与早期 NV-4B 纯数据 DeploymentSpec 切片 |
| 未覆盖 | Isaac runner 集成、双 OpenVR producer、双 Glove 同时连接、真人组合 SE(3)、四流 GUI/HIL |
| 本机环境 | macOS，项目 `.venv` Python 3.11.15；无 Isaac/SteamVR 硬件执行 |
| 目标机探测 | `lenovo-piper2` → `lenovo@Workstation2`，只读 SSH 成功 |

## 结论

本轮形成了一个不依赖 Isaac、SteamVR 或 Wuji SDK 的可验证基础：

- mapping v2 文件保持 1:1、逐轴 `±0.08 m` 不变；
- 新增 simulation-only mapping v3，沿用 v2 proper 轴旋转与 rotation policy，
  translation scale 为 `1.0`、X/Y/Z 各 `±0.4 m`，最大角点位移约 `0.693 m`；
- mapper 的同一 sample 同时产生 translation 与 rotation target 已有单元测试，
  但真人 XYZ+RPY 复合轨迹仍未执行；
- 新增 strict `DeploymentSpec v1`、host-local `LocalDeviceBinding v1` 和 resolver；
- 默认双侧、左单侧、右单侧三个 DeploymentSpec 均引用同一个双 q27 Session，并
  恰好覆盖四个 Session route；
- 单侧 spec 通过显式 arm-hold/hand-rest fixture 表达非活动侧，没有新增 runner
  `--side` 分支；
- resolved snapshot 不保存完整本机 device identity/endpoint，只保存 hash 和
  calibration ID。

该结果只证明配置、引用、映射和纯 application 数学边界闭合。三个 DeploymentSpec
均明确携带 `qualification_status: pending`；当前主 runner 尚未解析它们，也没有完成
双 Tracker/双 Glove live。

## 架构检查

`DeploymentSpec` 位于五层之外，但恰好引用一个 ResolvedSession：

```text
DeploymentSpec
  -> one ResolvedSession
  -> tracking setup + mapping calibration reference
  -> managed/in-process component graph
  -> live/fixture source bindings
  -> host-local binding ID
  -> report root
```

Asset、Binding、Assembly、Workcell、control layout 和 placement 仍只由五层拥有。
Deployment resolver 验证每个 `instance_id/group_id` 与 Session 完全一致，并依据
resolved Asset kind 限制允许的 source kind。完整 serial/IP 只允许出现在被忽略的
`configs/local/`；提交的 example 只含占位符。

重大边界已写入
[ADR-0007](../decisions/0007-nv4-native-dual-teleoperation-deployment.md)。

## 数据规格核对

本轮按项目要求通过只读 MCP 核对规格，未执行资料中的任何示例命令：

- Valve《SteamVR Tracking 原理与定位器职责》“定位器与被追踪物”：
  Base Station 是共享定位参考，而不是应用层按左右专属绑定 Tracker。
  [source_url](https://partner.steamgames.com/vrlicensing)
- Valve《OpenVR Tracker 与 Tracking Reference 运行时语义》“设备类别”“追踪状态”：
  Generic Tracker、Tracking Reference 与 `Running_OK`/calibrating 状态必须区分。
  [source_url](https://github.com/ValveSoftware/openvr/blob/0924064316de3effbcd1acf1e309182a2deb1c05/docs/Driver_API_Documentation.md)
- HTC《VIVE Tracker (3.0) Developer Guidelines v1.0》“Optics”“Coordinate system”：
  本计划只使用 Tracker 3.0 的视场/设备坐标边界，不与其他 Tracker 代际混用。
  [source_url](https://developer.vive.com/documents/824/HTC_Vive_Tracker_3.0_Developer_Guidelines_v1.0_01182021.pdf)
- Wuji《EMF 与手部追踪》“EMF 位姿”“手部追踪产物”：
  `0.9` 说明属于 `EmfPose.confidence`；`HandSkeleton` 单独定义 21 个带 confidence
  的关键点，没有给出统一硬拒绝线。
  [source_url](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking/)

因此“设备数量已配置”与“共同 tracking universe 已通过”继续分开；Glove
confidence 决策由 ADR-0007 显式覆盖 ADR-0006 的旧阈值段落。

## 自动验证

### 定向回归

```text
.venv/bin/python -m pytest \
  tests/unit/test_deployment_spec.py \
  tests/contract/test_nv4_deployments.py \
  tests/unit/test_tracker_workcell_mapping.py \
  tests/unit/test_tracker_arm_teleoperation.py -q
```

结果：`41 passed`。

### 全套无硬件回归

```text
.venv/bin/python -m pytest -q
```

结果：`565 passed, 4 skipped, 11 deselected`。

- 2 个 MuJoCo 相关测试因本机未安装可选 `mujoco` 跳过；
- 2 个固定 Hand 2 model-profile case 因未恢复上游 `wuji-description` 跳过；
- socket/hardware marker 按默认项目配置未执行。

### 静态与架构检查

```text
.venv/bin/ruff check src tests tools
.venv/bin/mypy src
```

结果：

- Ruff：`All checks passed`
- mypy：`Success: no issues found in 75 source files`

## 目标机只读探测

按 `lenovo-piper2-access` skill 执行有界只读 SSH：

```text
hostname -> Workstation2
id -un   -> lenovo
pwd      -> /home/lenovo
```

`/home/lenovo/swy/wujihand` 当前是 detached HEAD，且存在大量此前 NV-2/NV-3 同步形成的
未提交和未跟踪内容。本轮没有覆盖、清理、reset 或同步该目录，也没有启动 Isaac、
SteamVR、Glove SDK 或机器人控制。

## 剩余 Gate

1. 将 native-dual compatibility leaf 和 DeploymentSpec composition 接入主 runner。
2. 由一个 managed OpenVR runtime owner 输出左右两条 serial-addressed stream。
3. 在 Workstation2 创建真实但本地保存的 device binding，验证同一 runtime/universe/
   setup revision；通过后产生新的 qualified tracking setup revision。
4. 用固定 Lula 依次执行左右 XYZ-only、RPY-only、XYZ+RPY 真人复合轨迹。
5. 同一 SDK manager 内验证双 Glove 同时连接、retarget、关闭与单侧丢流。
6. 进入默认四流 tick、side-local fault、单侧 diagnostic 和 GUI persistent HIL。
