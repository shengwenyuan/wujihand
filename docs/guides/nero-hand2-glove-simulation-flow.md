# NERO + Hand 2 + Wuji Glove 仿真全流程透明说明

状态：**CURRENT / 2026-07-29**。预计阅读时间：5 分钟。

本文回答四个问题：Isaac 里加载了什么、Glove 发来什么、信号如何变成 Hand 2
动作，以及 `confidence`、`supervisor`、`qualification` 到底会不会阻止动作。

## 先看结论

- 左右各是一棵 `NERO q7 + Hand 2 q20 = q27` 的 Isaac articulation；全场共有
  2 棵 q27、4 条逻辑控制 route、54 个逻辑 DoF。
- 一次 Glove live 只拥有所选侧的 Hand 2 q20。两台 NERO q7 和另一只 Hand 2
  的命令保持不变，也没有 ROS、CAN 或真机命令输出。
- 当前置信度策略很宽松：取 21 个 landmark 的最低置信度；`<0.90` 只标记
  `DEGRADED`，仍然控制仿真手；`>=0.90` 标记 `SUCCESS`。硬拒绝下限是 `0.0`。
- `qualification` 是可记录的仿真测试 Gate，不是手套标定，也不是每帧动作滤波。
  Glove live 启动前最多等待 60 个仿真帧，随后在限定帧数内运行，并在结束时检查
  目标手响应、双臂/另一只手隔离和反馈有限值等条件。
- 2026-07-29 的左右手独立实测均为 `passed=true`：各运行 9000 帧、接收 8999
  帧、空轮询 1 次、拒绝 0 帧。

## 一条信号如何到达 Isaac

```text
Wuji SDK hand_skeleton
  21 个命名 landmark：position(m) + confidence + seq/device timestamp
        │
        ▼
WujiGloveHandSkeletonAdapter
  选择设备、核对左右侧、丢弃积压只取最新帧、规范化时钟/顺序/名称
        │
        ▼
CanonicalHandObservation
  固定 MediaPipe 21 点语义 + side/frame/calibration/transform provenance
        │
        ▼
WujiHand2RetargetAdapter
  RetargetSession.for_hand(WujiHand2, side) → 同侧 q20(rad) HandIntent
        │
        ▼
JointCommandSupervisor
  freshness 0.25 s + 关节限位 clamp + 按最大速度限制单步变化
        │
        ▼
compose_q27_hand_target
  只替换所选侧 q27 中的 q20，原 q7 和另一侧 q27 保持不变
        │
        ▼
Isaac position targets → physics/render step → q27 feedback → JSON Gate/report
```

常用名词：

| 名词 | 本项目中的准确含义 |
|---|---|
| q7 | 一台 NERO 的 7 个机械臂关节 |
| q20 | 一只 Hand 2 的 20 个执行关节 |
| q27 | 同侧 NERO q7 与 Hand 2 q20 合并后的 Isaac articulation |
| retarget | 把人手 21 个空间点转换为机器人手 q20；不是直接复制人手关节角 |
| intent | 带来源、侧别、布局和置信度的 q20 计算结果；还不代表已执行 |
| supervisor | 把 intent 变成受限位置命令，并处理缺帧、过期和异常值 |
| qualification | 对场景、拓扑、响应和隔离性做的有界仿真验收 |

## 当前仿真资产设置

五层配置仍是唯一的场景事实入口。Glove 是运行时输入，不会在启动时改写五层文件。

| 层 | 当前内容 | 不归这一层管理的内容 |
|---|---|---|
| Asset | 一种 NERO 身份、左右 Hand 2 身份；canonical q7/q20 布局 | USD 路径、世界坐标、Glove SDK |
| Binding | NERO 固定来源 URDF 的 Isaac 6.0.1 派生 USD；左右 Hand 2 完整物理 USD；joint/prim 映射 | 桌面位置、手臂间关系 |
| Assembly | `link7 → hand_base` 固定连接，左右均为 `[0.023, 0, -0.0235] m + Ry(+90°)` | 世界中的桌子和底座位置 |
| Workcell | `1.20 × 1.20 × 0.08 m` 固定桌；桌面 `z=0.80 m`；底座在 `x=±0.32, y=-0.52 m`，yaw `-90°`，接电侧朝桌内 | 关节控制权和输入设备 |
| Session | 组合以上四层，声明左右 q7/q20 四条 route，并引用 tabletop qualification profile | Glove serial/address 和实时帧 |

当前 Session hash 是
`46f543efdaa3eff26227ed73150902c20027b90afdc19cb35e9d814098800601`。
NERO 固定在 `agilexrobotics/agx_arm_urdf@f6642ce0…`，Hand 2 固定在
`wuji-description v2026.6.27@aee64892…`；完整文件 hash 由
`third_party/sources.lock.yaml` 管理。
两侧 NERO 的仿真准备位分别为：

```text
left  q7 = [-10°, +45°, 0°, +45°, +90°, 0°, 0°]
right q7 = [+10°, +45°, 0°, +45°, +90°, 0°, 0°]
```

当前 q7 Isaac drive gain 是 `stiffness=6500`、`damping=220.79402165819616`；
它们只是 Session qualification 数值，不是 NERO 真机控制器参数。Hand 2 q20
顺序为 thumb → index → middle → ring → pinky，每指 4 个量：拇指为
CMC flex/abd、MCP、IP，其余手指为 MCP flex/abd、PIP、DIP。PhysX 中的 q20
索引不连续，运行时必须同时按 joint name 和 USD path 找到分区，不能假设 q20
就是 q27 的最后 20 个连续索引。

Hand 2 原有的 world-fixed root 在 live stage 中被禁用，再用 FixedJoint 接到同侧
NERO；上游 USD 文件保持只读。合并 articulation 当前关闭内部 self-collision，
但桌面、地面等外部 collider 仍然存在。NERO `link6` 的表示对齐只旋转
visual/collision/质量属性，不修改 J7 joint frame 或运动学。

## Glove 原始信号与规范化

运行器通过 Wuji SDK 订阅 `hand_skeleton`。设备选择优先级是显式 serial、显式
address、最后才是 handedness；当前左右实测均使用 handedness 选择，并再次读取
设备报告侧别进行核对。

一帧输入必须满足：

- header `seq` 和设备 `timestamp_us` 严格递增；
- 左手 `frame_id=l_wrist`，右手 `frame_id=r_wrist`；
- 21 个 MediaPipe 语义名称各出现一次，不能缺失、重复或出现未知名称；
- 每个 position 是有限的 3 维米制向量，confidence 是 `[0,1]` 内有限值；
- source、calibration、transform 或 frame epoch 发生变化时，必须先 reset
  retargeter。

轮询是非阻塞的。如果 SDK 队列里已有多帧，adapter 会把队列读到空并只返回最新
一帧；这会主动丢弃过时积压，避免“逐帧补历史”造成越来越高的控制延迟。

设备时间属于 `wuji_glove_device_clock`，不能直接和主机 monotonic clock 相减。
因此 canonical observation 保存设备时间，但 `source_time_ns=None`，freshness 使用
主机接收时间。报告里的 input/retarget/render 耗时只覆盖主机调用，**不包含传感器
采集和设备到主机的真实端到端延迟**。

## 置信度：现在到底筛掉什么

当前 runner 使用 retarget adapter 的默认策略；这些阈值目前不是五层 YAML 参数。

| 条件 | 当前动作 |
|---|---|
| 任一 confidence 非有限或不在 `[0,1]` | canonical 构造失败，拒绝该帧 |
| 点缺失、position 非法、side/frame/order 不符 | 拒绝该帧，不生成新 intent |
| 21 点完整有限，最低 confidence `<0.90` 且 `>=0.0` | 生成 `DEGRADED` intent，**仍用于控制** |
| 21 点完整有限，最低 confidence `>=0.90` | 生成 `SUCCESS` intent并用于控制 |
| SDK retarget 输出不是有限 `(20,)` q20 | 拒绝该帧，不生成新 intent |

这里取的是 21 点的 **minimum**，不是均值；只要一个点低于 `0.90`，整帧就记为
`DEGRADED`。项目代码目前不会因为 `DEGRADED` 缩小动作、降低速度或回到 rest；
supervisor 接收到的仍是完整 q20。该标签目前用于 provenance、计数和后续调参，
不等于 supervisor 的 `DEGRADED` 安全状态。

## 缺帧、拒绝、限位与回 rest

对“没有新帧”和“新帧被拒绝”，controller 都不会伪造新的 input-derived intent：

1. 距最后有效帧不超过 `0.25 s` 时，继续使用最后一个有效 q20；
2. 超过 `0.25 s` 后，把目标切换到 Hand 2 rest；
3. 无论跟踪还是回 rest，都先按 Hand 2 profile 做 position clamp，再按
   `profile max_velocity × 1.0 × dt` 限制单步变化；
4. 最终 q20 被写入所选侧 q27 的手部分区，q7 原值不变。

因此回 rest 是渐进的，不是瞬间跳回。`position_clamped` 和 `rate_limited` 会分别
计入报告。注意：retarget SDK 自身可能有内部状态；项目侧明确存在的是 latest-frame、
freshness、clamp 和 slew-rate limit，并没有额外按 confidence 做均值滤波。

## qualification：启动时和结束时分别做什么

### 1. 启动前快速 readiness

Glove live 不执行耗时的完整 scripted 手型测试。它先让两棵 q27 在当前准备位运行：

- 每个窗口 15 个仿真帧；
- 至少 2 窗，最多 4 窗，即最多 60 帧；
- 比较两侧全部 q27 的窗口间最大关节差，阈值为 `0.03 rad`；
- 达到阈值可提前继续；最多 4 窗后即使未严格收敛也继续，并在报告中标记。

这一步只检查 Isaac 初态是否基本稳定。它不读取 Glove、不做用户标定，也不会发出
真机命令。完成后才依次出现：

```text
GLOVE LIVE CONNECTING  # 开 SDK connection / hand_skeleton subscription
GLOVE LIVE ARMED       # 已连接，supervisor 在 rest，等待首帧
GLOVE LIVE ACTIVE      # 第一份 canonical skeleton 已成功变成 q20 intent
```

### 2. live 运行结束 Gate

`--glove-frames N` 决定有界循环次数。每帧尝试取输入、应用命令、推进 physics/render
并读回反馈。结束时十项检查必须全部为 true：

- `accepted + empty + rejected == N`，且至少收到一帧有效 skeleton；
- supervised q20 和目标手反馈都至少变化 `0.01 rad`；
- 两侧 q7 命令保持，另一只手 q20 命令保持；
- 所选臂反馈漂移不超过 `0.05 rad`，另一侧完整 q27 漂移不超过 `0.03 rad`；
- readiness 不超过 4 窗，所有记录的反馈均为有限值。

无 `--glove-live` 时才走完整 scripted qualification：60 帧/窗、最多 8 窗、
要求 `0.005 rad` 严格收敛，并执行逐指、组合手型、隔离、reset 和 recovery。
二者用途不同，live 报告会明确写
`run_mode=glove_live_only`、`scripted_qualification_executed=false`。

正常跑完 N 帧后写 JSON、关闭 subscription/SDK connection 并退出 GUI；这不是崩溃。
提前中断仍会进入 connection 清理，但可能来不及生成最终报告。目标 physics 设置是
120 Hz；带 GUI 的真实循环速度由 render 决定，不能把 `physics_hz=120` 当作实测
显示或输入频率。

## 2026-07-29 左右手实测

两次测试均使用 Isaac Sim `6.0.1.0`、builtin calibration
`wuji_sdk.default_user.builtin.sdk_2026.7.21` 和 handedness selector。

| 指标 | left | right |
|---|---:|---:|
| 最终 Gate | `passed=true` | `passed=true` |
| simulation / accepted / empty / rejected | `9000 / 8999 / 1 / 0` | `9000 / 8999 / 1 / 0` |
| `DEGRADED` intent | `8962`（99.6%） | `7178`（79.8%） |
| accepted 最低 confidence：min / mean / max | `0.463 / 0.734 / 0.918` | `0.469 / 0.800 / 0.943` |
| readiness | `4` 窗，`0.981 s`，最终 `0.02165 rad` | `4` 窗，`0.999 s`，最终 `0.02165 rad` |
| connect → armed | `1257 ms` | `843 ms` |
| armed → 首个 intent | `19.5 ms` | `19.8 ms` |
| 实测 GUI loop | `67.41 Hz` | `67.07 Hz` |
| 主机 p95：input / retarget / sim+render | `0.352 / 0.366 / 17.91 ms` | `0.355 / 0.341 / 17.73 ms` |
| 目标手最大反馈变化 | `1.605 rad` | `1.762 rad` |
| 所选臂最大反馈漂移（Gate `<=0.05`） | `0.0325 rad` | `0.0380 rad` |
| 另一侧最大反馈漂移（Gate `<=0.03`） | `0.00218 rad` | `0.00198 rad` |
| position clamp / rate limit 次数 | `0 / 5` | `0 / 4` |

Workstation2 报告与 SHA-256：

```text
/home/lenovo/swy/wujihand/artifacts/validation/input-smoke/glove-left-hand2/live-left-fast-v1.json
e4162ddb131848f012cb2fab1eb443dd5051627df9fb8e8e45f485d1cb961a73

/home/lenovo/swy/wujihand/artifacts/validation/input-smoke/glove-right-hand2/live-right-fast-v1.json
347234f8b879ad6867c6dabf591c41b45966a97f4f2aec6c126cedaa034b5f57
```

两份报告的 Session hash 都是
`4b9f97fb1c946f92918bcb7f8ecffde68f859a6b511dc24914afb694f827578d`，并记录旧 q7
gain `6000 / 212.132`。报告生成后，项目继续修改了法兰、`link6` 表示、Assembly
mount 和 gain；当前 Session 已是前述 `abf48dd4…` 与 `6500 / 220.794`。Glove
input、retarget、supervisor 和快速 readiness 策略未因此改变，所以这两份报告证明
左右控制链和隔离逻辑均已打通；它们不替代对当前 `abf48dd4…` 资产快照的再次
端到端 qualification。

## 事实边界与入口

- 当前 Glove 以 handedness 而不是 serial 固定身份，builtin calibration 也不是
  正式的逐设备标定 revision。
- 当前只控制 Isaac 中的 Hand 2。真实 Hand 2、NERO、ROS 和 CAN 都不在此路径。
- NERO q7 限位来自固定 URDF 的临时仿真 profile。机身二维码 NERO 7F 页面与
  [《NERO用户手册》“1 机械臂简介 > 1.2 性能参数”](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/air57k7k3nhgeuxb)
  存在 J2/负载修订冲突，项目没有把两套参数拼接使用；详见
  [ADR-0005](../decisions/0005-nero-model-source-and-provisional-limits.md)。
- 真实 NERO 运动前仍需设备 joint readback、原厂安全说明和人工确认；本次双手通过
  不能外推为真机安全证据。

| 想核对的内容 | 唯一主入口 |
|---|---|
| 五层组合与当前 hash | `configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml` |
| 固定的上游版本与文件 hash | `third_party/sources.lock.yaml` |
| q7 准备位、几何 Gate、drive gain | `configs/profiles/isaac_nero_dual_tabletop_qualification_v1.yaml` |
| Hand 2 q20 顺序/限位/速度 | `configs/profiles/hand2_{left,right}_v2026_6_27.yaml` |
| Glove 连接、latest-frame、canonical 化 | `src/wujihand/adapters/input/wuji_glove.py` |
| 21 点到 q20、confidence 状态 | `src/wujihand/adapters/retargeting/wuji_hand2.py` |
| 缺帧与 rejected frame 语义 | `src/wujihand/application/teleoperation/glove_hand2.py` |
| clamp、rate limit、stale → rest | `src/wujihand/application/supervision/joint_supervisor.py` |
| 快速与完整 readiness 策略 | `src/wujihand/application/qualification/live_readiness.py` |
| Isaac composition、live Gate、JSON | `tools/run_isaac_nero_hand2_dual_twin.py` |
