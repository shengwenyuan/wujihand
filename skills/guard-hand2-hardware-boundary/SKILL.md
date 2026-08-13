---
name: guard-hand2-hardware-boundary
description: Design, implement, review, document, or test Wuji Hand2 Beta real-hardware bring-up, wuji-sdk adapters, diagnostics/control, ROS2 integration, real-hand teleoperation, or real-device recording in the wujihand repository. Use whenever work may mix Hand2 hardware with Isaac/MuJoCo simulation, simulation ticks, dataset recorders, SDK/firmware writes, or when deciding package, directory, configuration, event-time, safety, communication-quality, and command-ownership boundaries. Enforce a same-repository strong boundary, an independently extractable internal hardware package, event-time rather than simulation-tick semantics, staged integration, the project 85% response-rate floor, and fail-closed direct safety Gates.
---

# Guard Hand2 Hardware Boundary

把真机和仿真保持在同一仓库，以共享稳定契约；把真机 SDK、状态机、配置和证据保持为可独立拆包的
依赖岛。当前设备 bring-up 不依赖 Isaac、仿真 tick 或数据集 recorder。只在后续真实遥操作阶段接入
共同 ROS2/canonical 流程，再单独决定如何复用采集基础设施。

## 先判定工作阶段

把请求明确归入且只推进一个阶段：

```text
HARDWARE_BRINGUP
  官方工具、连接、身份、只读状态、诊断、单关节、全手脚本

REAL_TELEOP_INTEGRATION
  ROS2 输入、canonical observation、retarget、supervisor、真机 executor

REAL_DATA_COLLECTION
  真实 NERO/Hand2/D405 因果事件、episode、质量门禁和数据发布

SIM_REAL_ANALYSIS
  固定 trace 对照、模型误差分析；不得成为真机控制依赖
```

处理 `HARDWARE_BRINGUP` 时，禁止顺手修改 Isaac scene、simulation tick、仿真 recorder、dataset
schema 或 NERO 控制。若发现后续接口需求，只在类型/事件契约中预留并记录，不实现上层链路。

## 读取当前事实

1. 读取仓库 `docs/015-wuji-hand2-beta1-real-hardware-bringup-and-teleoperation-data-plan.md`。
2. 涉及 SDK/Description 版本链时读取
   `docs/013-wuji-description-v2026-8-3-hand2-beta1-phase1-upgrade-plan.md`。
3. 涉及全局分层时读取 `docs/000-project-charter-and-architecture.md`。
4. 同时使用仓库 `lookup-wuji-docs` skill，通过官方 `wuji-docs` MCP 搜索并读取 Hand2 用户须知、
   使用约束、SDK reference、SDK release notes 及对应 Studio/CLI 页面。
5. 把 `latest` 当作滚动入口；在报告中锁定查询日期、SDK/Studio/CLI/firmware readback 和设备身份。

不要从本 skill 推断当前设备固件、在线状态或可运动性。文档事实、仓库实现、live readback 和人工
观察必须分别记录。

## 建立可拆出的硬件 package

优先采用以下边界：

```text
packages/wujihand-hand2-hardware/
  pyproject.toml
  src/wujihand_hand2_hardware/
    __init__.py
    api.py                 # 小而稳定的公开 facade
    types.py               # identity/state/diagnostic/command/event 值对象
    sdk_client.py          # SdkManager/connect/disconnect
    mapping.py             # nid/label 与 SDK 20 关节协议
    readonly.py            # joint_states/joint_diagnostics/comm_diag/logs
    safety.py              # 硬件状态机、watchdog、fault Gate
    executor.py            # 唯一 SDK command owner
    journal.py             # bring-up 事件与 receipt，不依赖 dataset
  tests/
    unit/
    contract/

src/wujihand/adapters/hardware/
  wuji_hand2_package.py    # 后续主仓库 integration shim；只做类型/port 映射

ros2/wujihand_hand2_hardware/     # 后续 REAL_TELEOP_INTEGRATION 才创建
  package.xml
  wujihand_hand2_hardware/
    executor_node.py
```

硬件 package 只依赖 Python 基础库、`wuji_sdk` 和其自身值对象。禁止依赖：

- `omni`、Isaac、USD、MuJoCo、simulation clock；
- `rclpy`、ROS generated message；
- `wujihand.runtime`、`wujihand.dataset`、仿真 Session/Deployment resolver；
- D405、NERO、Tracker 或 Glove adapter；
- 用户工作站绝对路径和仓库全局单例。

让 package 通过 facade 接收显式 serial/address/side、安全 profile 和 clock abstraction，返回具名状态、
诊断与事件。不要让它解析主仓库的五层仿真配置。把 ROS、canonical q20 和 Deployment 转换留在薄
integration shim；这样可在需要时把 package 连同 contract tests 单独发布或迁出。

不要为了可拆包复制核心算法。SDK package 只表达设备协议和硬件安全；retarget、canonical
observation 与 application supervisor 仍由主仓库保持唯一实现。

## 使用事件时间，不复用仿真 tick

真机不存在稳定的 `control_tick == physics_step == render/record tick`。禁止给真机样本伪造：

- simulation tick、physics substep、simulation time；
- 同 tick 的 command/state/contact ground truth；
- 仿真 contact、rigid-body truth、reset/settle 语义；
- 固定整数频率下必然一一对应的输入、命令和反馈。

对每个真机事件至少记录：

```text
event_id
event_kind
side + device_serial
host_monotonic_ns
host_wall_time（仅用于人类定位）
device_timestamp_us（若该资源提供）
device_sequence（若该资源提供）
command_correlation_id（命令相关事件）
safety_state
payload_schema_revision
```

使用独立事件表达因果链：

```text
INPUT_OBSERVED
DECISION_PRODUCED
COMMAND_ATTEMPTED
COMMAND_ACCEPTED / COMMAND_FAILED
HARDWARE_STATE_OBSERVED
DIAGNOSTIC_OBSERVED
SAFETY_TRANSITION
CONNECTED / DISCONNECTED
```

不要把 `send()` 成功解释为物理关节已到达。用 `command_correlation_id`、设备 sequence/timestamp 和
host monotonic receipt 关联 attempted command 与后续 readback；允许多条状态落在两个命令之间，
也允许一个命令暂时没有可归因的状态。

设备时间域不明确时，保留原始时间戳和 host receipt，不擅自跨左右手拟合统一设备时钟。后续数据
采集可离线建立对齐模型，但不得回写或篡改原始事件。

## 复用 ROS2 流程，而不是仿真执行语义

进入 `REAL_TELEOP_INTEGRATION` 后复用：

- Glove source lifecycle、namespace、identity 和设备 ownership；
- `HandObservationEnvelope` 与 ROS↔canonical converter；
- bounded latest-only inbox、QoS、sequence、producer epoch 和 freshness 语义；
- canonical observation、SDK retarget、q20 layout 和 `JointCommandSupervisor`；
- ROS launch/Deployment 的显式组合模式；
- safety/event 消息的结构原则。

禁止复用：

- Isaac executor、articulation、drive、scene reset 和 physics callback；
- `SimulationStateFrame`、simulation camera truth 或 TeleoperationTickTrace 的 tick 等式；
- 仿真 `kp/kv`、effort、collision/contact 或 joint feedback 作为真机参数/安全事实；
- 通过 `--real`、自动发现或 fallback 把现有仿真入口改成真机入口。

让 ROS hardware executor 成为唯一真机 command owner。它订阅 canonical observation 或由主仓库
产生的 supervised decision；禁止开放一个绕过 retarget/supervisor/safety 的公共 raw-q20 写入 topic。
CLI、Studio 和 recorder 只观察；需要维护写入时，先停掉 executor 并显式转移 ownership。

为真机定义独立 ROS 事件消息或后端无关的因果 envelope。可以复用仿真事件字段名称和 checksum
模式，但不要为了“统一”强制填充不存在的 tick/contact/frame truth。

## 分离三类证据与采集

### 当前 bring-up

只写硬件 journal、identity/diagnostics snapshot、SDK/device logs、command receipt 和人工观察。使用
package 自带的轻量 JSONL/JSON writer；不要启动 rosbag、episode lifecycle 或仿真 dataset writer。

### 后续真实遥操作

通过 ROS2 发布 input、decision、attempted command、result/readback、diagnostic 和 safety event。
先证明事件完整性、watchdog 和唯一 command owner，再讨论 episode。

### 更后续真实数据采集

在独立需求中评估哪些现有 recorder/checksum/integrity 组件真正 backend-neutral。允许复用文件组织、
manifest、checksum 和质量门禁框架；禁止复用 simulation truth schema、tick 对齐假设和 Isaac source。
默认写 `qualification_only=true`、`dataset_eligible=false`，直到真实任务质量 Gate 通过。

## 按风险推进硬件调试

严格按以下顺序实施：

1. 用 CLI/Studio 完成只读发现、身份、版本和日志检查。
2. 初始化内部 hardware package，建立 fake SDK 与 import/dependency Gate。
3. 用 SDK 显式 SN + side + address 读取状态、诊断和通信，不 enable、不写参数。
4. 冻结 `nid`/label/side mapping；变长在线关节帧按 `nid` 处理。
5. 建立 disconnected/read_only/armed/enabled/faulted/estopped 状态机和 watchdog。
6. 在人工许可下单手、单关节、小位移、低风险测试。
7. 完成单手 20 关节脚本，再分别验收左右手。
8. 当前 bring-up 收口后，另立真实 ROS2/Glove teleop 需求。
9. 真实 teleop 稳定后，另立真实双 NERO/双 Hand2 数据采集需求。

每个阶段均保持前一阶段的独立入口可运行。不要让 Glove、ROS2、Isaac 或 dataset 变成 SDK 只读探针
和受控台架脚本的启动前提。

## 执行真机安全 Gate

从官方 `wuji-docs` 页面实时确认 Beta 阶段、供电、环境、固件与 API 约束。至少执行：

- 显式 serial/address/side/firmware/hardware revision/online joint count；
- 唯一 command owner；Studio 共享 Hand2 会话可能读写，不能默认安全；
- 未 arm 时禁止 `enable` 和 command send；
- stale、断连、少关节、未知错误、active limit、非有限值、异常电压/温度时 fail closed；
- `joint_diagnostics` 任一关节 response rate 低于当前项目下限时 fail closed；
- 先停止、disable，再按官方流程断电；
- 禁止自动 clear fault、set/clear origin、写 MIT 参数/effort limit、改 IP、reboot 或升级固件；
- 禁止把 Beta1 电流作为接触力/力矩，禁止假设指尖触觉存在；
- 禁止从 Isaac drive 参数推导真机 MIT 参数。

涉及任何运动、参数写入、origin、fault clear 或固件动作时，先列出精确目标、当前 readback、停止路径、
操作者动作和验收标准；需要用户或操作者许可时停在动作前。

### 保持通信 Gate 单一

对已验证的右手 Beta1、firmware `2.2.3`、SDK `2026.8.3` 和当前专用网络拓扑，使用 `85%`（含）作为
项目 `joint_diagnostics` response-rate 下限：

- `<85%` 阻断；`85–99%` 通过并记录；
- timeout、transport、`sdk_dropped`、CRC/UART、`comm_diag` offline/no-request 等计数或短窗只写 receipt，
  不再各自叠加 Gate；
- 状态流或诊断流 stale、q20 不完整、fault/limit、非有限值和温度条件仍独立阻断；
- 不再引入 `STRICT/OBSERVE` 双模式，不要求绝对 `100%`，也不为每类计数维护重复的 baseline/delta
  failure helper；
- 保留原始计数、窗口和上下文，使低响应与安全错误相关时仍可复盘。

明确把 `85%` 写成项目对当前固定栈的风险接受，不得称为 Wuji 官方合格线。官方文档只定义诊断字段
和短暂陈旧值语义，没有发布该阈值。hardware/firmware/SDK、网络拓扑、命令频率或设备 side 变化后，
重新打开通信资格验证；不得把右手结论继承给左手。

## 审查目录与修改范围

接受当前 bring-up 修改：

```text
packages/wujihand-hand2-hardware/**
hardware/**（物理装配或本地模板，若仓库规则允许）
configs/hardware/**
configs/qualifications/*hand2*hardware*
configs/examples/*hand2*hardware*
tools/*hand2*hardware*
tests/**/test_*hand2*hardware*
docs/015-*
docs/real-hardware/**
docs/validation/*hand2*hardware*
```

对这些路径默认发出越界警告，除非请求已进入相应后续阶段：

```text
src/wujihand/adapters/simulation/**
src/wujihand/runtime/isaac_*
configs/assets/**、configs/bindings/isaac/**、configs/workcells/**
ros2/**                         # bring-up 阶段不需要；teleop 阶段才允许
src/wujihand/dataset/**
pi/**
```

若工作区存在并行仿真/T-frame/Pi 修改，保持不动，不把它们混入硬件提交。为硬件需求建立小而可撤回的
commit：package foundation、read-only qualification、bounded motion、ROS2 integration 分开提交。

## 交付时报告

按当前阶段给出：

- 复用的稳定契约与明确未复用的仿真语义；
- hardware package 公开 facade、依赖图和可拆包检查；
- 设备/SDK/firmware/config 身份；
- host/device 时间、sequence 与 command correlation 规则；
- command ownership、安全状态机、watchdog 和停止路径；
- 生效的 response-rate 下限、非阻断通信观测和适用身份范围；
- 实际执行的 read-only/write/motion 范围；
- hardware journal/qualification receipt 与未验证项；
- 是否仍保持默认仿真入口零真机访问；
- 后续真实 teleop 或数据采集需求，只作边界说明，不提前实现。
