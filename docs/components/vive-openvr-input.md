# VIVE OpenVR 输入组件

状态：软件实现与离线 contract 测试已完成；目标机 HIL 尚未通过。

本组件把 OpenVR Tracker 观测规范化为设备无关的 tracking contract，供 NV-1 枚举、
采集、记录和回放。它是只读输入组件，不启动 Isaac、ROS 2 或 NERO。

## 五层边界

NV-1 采用 ADR-0004 的组件资格验证路线：

- Tracker 不是 Asset Manifest 或 Backend Binding。
- NV-1 不创建 Assembly、Workcell 或虚假的 Simulation Session，也不改变现有
  Session hash。
- domain/ports 只认识 canonical tracking 值；OpenVR SDK 类型限制在
  `adapters/input`。
- NV-3 出现真实 NERO Isaac consumer 后，目标组合根仍是五层 Session；producer、
  transport、recorder 与目标 Session 再由 `DeploymentSpec` 编排。

## 已实现能力

### Domain

`TrackedRigidBodySample` 固定 schema v1：

- position 单位为米；
- orientation 为 active scalar-first `wxyz` 单位四元数；
- host timestamp 属于 `host_monotonic`；
- stream、device serial、logical role、tracking frame 和 sequence 均显式记录；
- tracking state 为 `uninitialized`、`calibrating`、`running`、`out_of_range`、
  `rotation_only` 或 `lost`。

只有 `running` 样本允许携带 pose 和 quality。其余状态必须
`pose_valid=false`，且 position、quaternion、quality 均为 `null`，因此失联时不会把
上一帧姿态伪装成新数据。

`ClutchEvent` 记录 press/release edge、input identity、sequence 和相同 host monotonic
时间域；按下沿可请求新的 clutch epoch。

### Port

`TrackingInputPort` 提供四个同步边界，其中 `poll()` 返回单次、非阻塞的规范化观测：

- `inventory()`：返回 serial、device class、model、manufacturer 和 connected；
- `start()`：解析配置的稳定 serial；
- `poll()`：原子返回一条 pose/state sample 及同次轮询观察到的 clutch edge；
- `close()`：释放 runtime，adapter 实现支持重复关闭。

`TrackerInventoryItem` 不暴露 OpenVR 临时 device index；`TrackingPoll` 校验 sample 与
event 的 stream、serial、role 和 clock domain 一致。

### OpenVR adapter

`OpenVrTrackerAdapter` 延迟导入固定的 `openvr==2.12.1401`：

- inventory 可在不指定 serial 时使用；
- `start()` 只接受唯一的 OpenVR `generic_tracker`；
- 每次 poll 都重新用 serial 解析当前 device index，不跨轮询持久化 index；
- 使用 standing tracking universe 读取 device-to-absolute 3×4 transform；
- 校验矩阵 finite、正交且右手，将平移和旋转转换为米与 active `wxyz`；
- 连续有效帧做 quaternion hemisphere 对齐；失联后清除该连续性状态；
- 显式映射 OpenVR tracking result，非法矩阵或不可用 pose 输出无姿态的状态样本；
- 可选 0–63 button bit 生成 clutch press/release edge；
- `last_raw_record` 只暴露隔离的 JSON-safe 原始观测副本。

当前 adapter 对 `Running_OK` 的规范化 quality 记为 `1.0`；这表示该帧满足当前
OpenVR 有效条件，不应解释为测量误差概率。

### Storage

tracking JSONL codec 对 sample/event 使用确定性 JSON 编码并执行严格解码：

- 字段集合必须与 schema 完全一致；
- 拒绝重复字段、NaN/Infinity、空记录和截断的末行；
- 读入时重建 domain 对象，再次执行单位、状态和不变量校验；
- 支持 sample 与 clutch event 的批量写入和全文件验证读取。

### Qualification metrics

`compute_tracking_metrics()` 对单一 stream/serial/frame 的采集顺序计算：

- sample count、valid count、valid ratio 与正时间间隔上的 sample rate；
- 相等或倒退 host timestamp 的 violation count；
- 有效 pose 的 position RMS/peak 与 SO(3) orientation RMS/peak；
- 首个与末个有效 pose 之间的 position drift 和最短 SO(3) orientation drift；少于
  两个有效 pose 时 drift 为 `null`；
- 连续无效区间的 dropout 数量、observed duration 和闭合区间的 reacquisition
  duration。

stationary spread 只有在对应输入片段确实保持静止时才代表静止抖动；函数不会替调用方
推断场景。

## 已验证范围

离线 unit/contract 测试覆盖：

- tracking domain 不变量及 pose/event identity；
- OpenVR 3×4 矩阵转换、state 映射、serial 重解析、device loss、quaternion
  hemisphere 和 button edge；
- 严格 JSON/JSONL round-trip 与畸形输入拒绝；
- rate、timestamp、stationary spread、short-term drift、dropout 和 reacquisition
  指标；
- domain、ports、application 对外部 SDK 的依赖方向。

尚未完成的目标机证据包括：SteamVR/OpenVR runtime inventory、真实 serial 绑定、
6-DoF 三轴方向、静止指标、遮挡/丢失/重捕获以及真实 Tracker 按钮事件。因此当前状态
不等价于 NV-1 Gate 通过。
