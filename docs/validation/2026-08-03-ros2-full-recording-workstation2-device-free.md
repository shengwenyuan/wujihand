# 2026-08-03 ROS2 全因果录制 Workstation2 无设备验证

## 结论

在 Tracker/AR、双手套和远端设备均未连接的前提下，Workstation2 已完成 Jazzy
接口构建、离线测试、录制启动拓扑、Isaac 限帧运行、20-topic MCAP 录制闭环和
robolab 动态物体回读验证。未启动 VIVE/OpenVR、Wuji Glove source 或真机链路。

当前可以停在真实控制设备接入前；尚不能据此评价真实遥操作的延迟、跟踪精度、
手部重定向误差或任务质量。

## 隔离范围

- 保留既有 `/home/lenovo/swy/wujihand_nv4` 不动；
- 验证副本：`/home/lenovo/swy/wujihand_recording_20260803`；
- colcon build/install/log 与合成录制产物均放在 `/tmp`；
- 合成 ROS 流量使用独立 domain 157、158、159；
- 合成输入只发布空的接口实例，用于发现、序列化和闭环验证，不代表有效控制数据。

## 构建与测试

```text
colcon build: 2 packages passed
colcon test: 4 passed, 0 failed
core unit: 552 passed
contract: 123 passed
integration: 11 deselected (requires_socket)
MuJoCo: 2 skipped (optional dependency absent)
```

Jazzy 已生成并可 introspect 新的 `RunRecordingStatus`、`SceneRigidBodyState` 和
`TeleoperationTickTrace` 消息。

`record:=true` 静态展开结果为 5 个执行进程、20 个 recorder topic、5 个输入发现
Gate；输入激活要求 consumer 与 recorder 两个 subscriber 均就绪。

## 无设备运行结果

基础双臂双手 deployment 的 Isaac consumer 在无输入保持模式下完成 2 帧并正常退出：

```text
left/right arm: waiting_for_tracker_sample
left/right hand: missing_input_return_to_rest
state: consumer_completed
```

单 topic 合成 recorder 验证生成 Jazzy MCAP、metadata、receipt 和 checksum；65 条消息，
receipt 为 `complete`。

完整 20-topic 合成闭环随后同时运行真实 Isaac consumer 与 rosbag wrapper：

- 基础桌面场景：20/20 topic 建档，2 个 tick，MCAP closure 为 `complete`；
- robolab banana+bowl 场景：20/20 topic 均有消息，987 条消息，2 个 tick，closure 为
  `complete`；
- robolab 的 `scene/rigid_body_state` 共 6 条，对应每 tick 的 table、bowl、banana；
- MCAP 可由 `rosbag2_py` 顺序读取并反序列化全部 987 条消息。

反序列化后的关键维度为：

```text
raw glove: 21 names + 21 valid + 63 xyz + 21 confidence
per-side tick: q7 arm command + q20 hand intent + q20 hand command
atomic scene target/feedback: q27 applied + q27 post-feedback
dynamic prims: table, bowl, banana
```

因此原始手部 21 点观测和重定向/安全处理后的 q20 必须且已经分层保留；二者不能互相
替代。最终 q20 同时进入原子 q27，便于事后对齐控制输入、施加目标与仿真反馈。

## 已知限制

- NVIDIA PCI 设备可见，但 `nvidia` 内核模块未加载，`nvidia-smi` 不可用；本次 Isaac
  依靠 CPU compatibility mode 通过，不代表实时性能或 GUI 合格；
- 两次完整合成录制在停止边界各报告 1 条 transport-layer lost message。20 个主题仍均
  成功落盘，receipt 结构闭合；正式采集时仍须用 source sequence/tick 连续性统计丢包，
  并考虑把 rosbag lost-message 计数写入 recorder 原始元数据；
- 合成输入不具备合法 tracking/glove 语义，不能验证 reference gate、IK 跟踪、q21/21点
  到 q20 的真实误差或端到端时延；
- 基础桌面场景没有受管动态物体，因此该 deployment 的 scene topic 建档但消息数为 0；
  robolab 场景已覆盖动态物体录制路径。

## 下一人工阶段

继续前先恢复并确认 NVIDIA 驱动。随后需要操作者连接并配置 AR/Tracker 与双手套，先做
短时输入-only/静止段，再做低速双臂双手 pilot；只有这一步开始后，才生成可用于质量
指标、统计图和 sim-to-real 比较的真实遥操作数据。
