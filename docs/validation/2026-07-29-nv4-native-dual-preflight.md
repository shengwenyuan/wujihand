# 2026-07-29 NV-4 原生双侧实现与 Workstation2 预检

| 字段 | 值 |
|---|---|
| 状态 | `SOFTWARE_PREHIL_PASS / HIL_REQUIRED` |
| 范围 | NV-4B～E 非人工实现、隔离部署、资产闭包、设备枚举和有界连接预检 |
| 未覆盖 | 人工 Tracker 运动、Glove 手势、Isaac GUI 四流、遮挡/故障和稳定性 Gate |
| 本机 | macOS 项目环境，Python 3.11 |
| 目标机 | `lenovo@Workstation2`，Ubuntu 24.04、Isaac Sim 6.0.1、Python 3.12 |
| 隔离部署 | `/home/lenovo/swy/wujihand_nv4` |

## 结论

NV-4 已具备进入真人 HIL 的软件主线，但当前设备状态不足以安全启动默认四流 GUI：

- 新增专用 `isaac_nero_dual_hand2_native_teleop_v1` Session。它复用既有
  qualification Session 的同一 Assembly、Workcell、四个实例 Binding、root
  placement 和四 route，只在第五层声明 live runtime role、composite transport
  contract 与 native-dual profile。
- 默认双侧、左单侧和右单侧 Deployment 均可编译成同一种四 route runtime plan；
  单侧诊断的非活动侧由显式 arm-hold/hand-rest fixture 接管。
- 一个受管 OpenVR owner 可按稳定 serial 产生左右 canonical stream；生命周期、
  transport epoch/revision、受管重启和迟到旧包拒绝已经进入合同与测试。
- 左右 NERO 复用 side-neutral arm controller，但各自拥有 readiness/reference、
  mapper、Lula FK/IK、q7 supervisor 和故障状态。
- 两只 Glove 复用同一 SDK manager，拥有独立 identity、subscription、retarget 和
  q20 supervisor；start 顺序固定为左→右，close 顺序固定为右→左。
- 默认 runner 已实现一个 tick 内先计算四路 decision、再提交左右两个 q27 target；
  省略 `--session` 时进入 native-dual Deployment 主线。

Workstation2 的有界预检同时确认：

- 两只 Glove 均在线，按左右 identity 显式连接、订阅和关闭成功；
- SteamVR/OpenVR 当前只枚举一枚已知 Tracker，且状态为 `disconnected`；第二枚
  Tracker 尚未取得可绑定的稳定 identity；
- 两只 Glove 在 5 秒有界预检内没有产生 `hand_skeleton` 帧；
- 因此没有启动 Isaac GUI，也没有发送任何真实 NERO、CAN 或 Hand 2 硬件命令。

这属于预期的人机介入边界，不是以 fallback 继续运行的理由。

## 五层和部署边界

```text
DeploymentSpec
  -> exactly one native-dual live ResolvedSession
       -> Asset Manifest
       -> Backend Binding
       -> Assembly
       -> Workcell
       -> Session runtime contract/profile/routes
  -> process lifecycle + host-local devices/endpoints
  -> tracking setup + mapping reference
  -> report root
```

实现没有把 Tracker/Glove identity、UDP 端口、OpenVR 解释器或 SDK 连接对象写入五层。
本机真实绑定只存在于 Git 忽略的
`configs/local/workstation2_nv4_v1.yaml`。resolved report 保存脱敏摘要和独立
`local_binding_hash`，不保存完整设备 identity/endpoint。

qualification 与 live 使用两个 Session 是有意的第五层职责拆分：

- qualification Session 保持 `runtime_role: simulation` 且无 transport contract；
- live Session 使用 `runtime_role: teleop_consumer` 和
  `wujihand.native_dual_teleoperation.v1`；
- contract test 锁定两者引用相同的 Assembly、Workcell 和实例 Binding。

因此没有把 qualification 行为静默改成 live，也没有增加第六个资产层。

## 已实现的代码边界

| 边界 | 实现 |
|---|---|
| Deployment/local binding strict schema | `src/wujihand/specs/deployment.py` |
| Native dual policy | `src/wujihand/specs/native_dual_teleoperation.py` |
| 跨层解析与脱敏快照 | `src/wujihand/runtime/deployment_resolver.py` |
| 四 route 运行 plan | `src/wujihand/runtime/native_dual_plan.py` |
| OpenVR process lifecycle | `src/wujihand/runtime/process_supervisor.py` |
| 单 owner 双 Tracker producer | `tools/stream_vive_trackers_udp.py` |
| Side-neutral arm application | `src/wujihand/application/teleoperation/tracker_arm_simulation.py` |
| Lula FK/IK adapter | `src/wujihand/adapters/simulation/lula_arm_kinematics.py` |
| 双 Glove composition | `src/wujihand/application/teleoperation/glove_hand2_set.py` |
| 默认四流运行入口 | `tools/run_isaac_nero_hand2_dual_twin.py` |

这些边界保持 `domain/ports → application → adapters → runtime/tools` 的依赖方向；
application controller 不 import Isaac、OpenVR 或 Wuji SDK。

## 配置闭包

Workstation2 隔离副本已恢复并验证固定派生资产：

```text
artifacts/derived/isaac/6.0.1/agilex_nero/nero_description/nero_description.usda
```

右单侧 Deployment 在 `verify_artifacts=True` 下解析成功：

```text
deployment  = isaac_nero_hand2_right_single_live_v1
session     = isaac_nero_dual_hand2_native_teleop_v1
mapping     = vive_tracker_workcell_workstation2_v3
qualified   = false
```

默认双侧 Deployment 在 backend 初始化前按设计失败：

```text
ValueError: local binding is missing key 'tracker_left'
```

这证明缺设备时不会猜测 side、复用右 Tracker 或静默退化为单侧。若要做单侧诊断，
必须显式选择 committed `right_single_live` 或 `left_single_live` Deployment。

## 自动验证

### 本机

```text
.venv/bin/python -m pytest -q
606 passed, 4 skipped, 11 deselected

.venv/bin/ruff check src tests tools
All checks passed

.venv/bin/mypy src
Success: no issues found in 82 source files
```

### Workstation2 隔离副本

隔离副本复用了目标机原有、固定来源的 `third_party` 与 derived Isaac artifact，
但没有覆盖、清理或 reset `/home/lenovo/swy/wujihand`。

```text
python -m pytest -q
608 passed, 2 skipped, 11 deselected

ruff check src tests tools
All checks passed

mypy src
Success
```

本机与目标机 skip 数差异来自可选第三方资产/环境可用性，不影响上述 NV-4
contract、application 和 runtime tests。

## 设备与进程预检

### SteamVR / Tracker

- SteamVR `vrserver` 与 `vrmonitor` 正在运行；
- USB 枚举到两个 Valve Watchman dongle；
- OpenVR inventory 枚举到两台 tracking reference 和一枚 Tracker 3.0；
- 当前 tracking reference 与已知 Tracker 均未形成可用 connected/Running pose；
- 第二枚 Tracker 未出现在 OpenVR inventory，也没有写入本机 binding。

对已知右 Tracker 执行 1 秒、只发 loopback UDP 的 producer 预检，结果为：

```text
error: configured Trackers are disconnected
```

该工具只读取 OpenVR pose，不加载 Isaac、不计算 IK、不连接机器人。

### Wuji Glove

SDK scan 枚举到一只左 Glove 和一只右 Glove。使用一个 manager、两个唯一 device
name 进行的有界预检完成：

1. 左右按 identity 显式连接；
2. 核对 SDK 返回 side；
3. 分别订阅 `hand_skeleton`；
4. 逆序取消并断开。

连接路径通过，但 5 秒内两侧均没有 skeleton observation。SDK 另有设备日志符号文件
下载 `HTTP 404` 警告；它影响设备日志解码，不足以解释或替代 skeleton 数据资格。
下一次测试需要佩戴/唤醒/标定后的真实手势输入来区分设备状态与数据链问题。

## 数据规格依据

本轮通过只读 MCP 核对并使用以下事实，没有执行资料中的示例命令：

- Valve《OpenVR Tracker 与 Tracking Reference 运行时语义》，“设备类别”“追踪状态”：
  `TrackingResult_Running_OK` 且 pose valid 才能作为可动作输入；
  calibrating/out-of-range 不能伪装为 running。
  [source_url](https://github.com/ValveSoftware/openvr/blob/0924064316de3effbcd1acf1e309182a2deb1c05/docs/Driver_API_Documentation.md)
- Wuji《快速开始》，“回调订阅”：SDK manager 会扫描并连接可用设备，应用通过回调
  获取订阅数据。
  [source_url](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/quick-start/)
- Wuji《设备发现与连接》，“设备发现”“多设备管理”：scan 结果提供 SN/address/type；
  同一 manager 可以用唯一 device name 管理多设备，并可按 SN 显式连接。
  [source_url](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/device-connection/)

## 当前停止点与下一 Gate

继续到 GUI 前需要项目负责人进行以下硬件操作：

1. 给两枚 Tracker 上电，并在同一个 SteamVR runtime 中完成配对；
2. 确认两枚 Tracker 与两台 Base 均可见，且 Tracker pose 进入稳定 Running；
3. 佩戴、唤醒并完成两只 Glove 当前所需标定，使 `hand_skeleton` 连续输出。

满足后先重跑无运动 inventory/连接预检，补齐 `tracker_left` 本机 binding；再按
`right_single_live → left_single_live → native_dual_live` 顺序进入 GUI，依次执行
XYZ-only、RPY-only、XYZ+RPY、双手和四流 HIL。

在上述 HIL 通过前：

- `tracking_setup.qualification_status` 保持 `pending`；
- 不删除历史 NV-2 qualification 路径；
- 不冻结 IK failure/rebuild 阈值；
- 不宣称共同 Standing universe、双 Tracker 坐标一致或双手控制通过；
- 不执行 NV-4F 的 runner 轻量化删除。
