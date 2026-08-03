# ADR-0009：ROS2 全因果链录制与离线分析边界

- 状态：已接受；Workstation2 `-05` 已完成有序关闭复验
- 日期：2026-07-31
- 前置决策：[ADR-0008：ROS2 Jazzy 双侧遥操作边界](0008-ros2-jazzy-dual-teleoperation-boundary.md)
- 实施计划：[003：遥操作质量分析计划](../003-teleoperation-quality-analysis-ros2-sim2real-plan.md)

## 背景

原 `record=true` 只录两路 Tracker、Tracker lifecycle 和两路 Glove 输入，无法把
Glove 21×3 landmarks、Tracker SE(3)、q20/q7 计算结果、最终 q27 apply 与 Isaac
post-step 状态闭合到同一控制 tick。

## 决策

1. `wujihand_interfaces_v2` 新增 `TeleoperationTickTrace`、
   `SceneRigidBodyState` 和 `RunRecordingStatus`。
2. 原始 Tracker/Glove typed topic 保存传感器数值；tick trace 只通过
   `source_id + producer_instance + transport_epoch + sequence` 关联所选输入，并保存：
   arm mapping、IK q7、active hand q20 intent、q7/q20 safety decision、atomic q27 target、
   pre-apply/post-step q27 和各阶段原始 monotonic timestamp。
3. route command 继续是只读观测 topic；`JointState` feedback 改为 world step 后发布，
   精确关联以 tick trace 为准。
4. `record=true` 使用 run-unique 被动 rosbag2 MCAP sidecar。consumer 在首个 control
   tick 前确认完整 allowlist 已发现 recorder subscription；source input 同时要求
   consumer + recorder 两个 subscription，满足后才 activate。consumer 或 recorder
   任一退出都会结束该 run。
5. 每个 run 生成 `manifest.json`、`recorder.json`、raw rosbag2、`receipt.json` 和
   `checksums.sha256`。manifest/recorder identity、receipt、非空 metadata/MCAP、零
   recorder exit 缺一即 `incomplete`；finalize 后 receipt 不再允许迟到进程改写。
6. `recorder_process_id` 暂保持 `null`：sidecar 不属于 Deployment execution graph，
   没有 command publisher、execution lease 或控制所有权。
7. 生产路径不计算 RMSE、coverage、percentile、统计检验或图片。后续离线分析包只读
   immutable run artifact。
8. trace/scene 发布异常只把录制标为 incomplete，不改变已经运行的控制决策。
9. `receipt.state=complete` 只证明产物正常闭合，不证明 topic/tick 无缺失；sequence、
   双侧逐 tick、raw↔trace join 和 drop 检查仍由离线 validator 执行。
10. rosbag 子进程使用独立 process group，由 recorder wrapper 独占关闭权。结束时
    consumer 先发布并等待 terminal status ack，再原子写 consumer receipt；wrapper
    观察 receipt 后只发送一次 SIGINT 给 rosbag，最后才 finalize/checksum。

## 当前能力边界

- 已记录 Workcell materialization、固定 fixture manifest pose 和 dynamic rigid-body
  pose/可用 velocity；banana-bowl 的 table/bowl 固定，banana 动态；
- raw contact、link7/palm/fingertip pose 和 Task truth 当前 capability 为 false；
- 已完成一次左侧抓放 pilot；因 raw contact 缺失，只能用物体运动证明 task outcome，
  不能归因具体 link/point/force；
- `banana-grasp-pilot-20260803-05` 已验证 Ctrl+C 由 consumer 锁存为 `stop_signal=2`，
  terminal status 获得 recorder ack，consumer receipt、MCAP、checksum 最终均为
  `complete`；
- 尚未形成正式数据集。

## 后续计划：踏板终止 episode

- 使用独立有线双踏板：一侧标记 `success`，另一侧标记 `abort`；通过稳定设备路径读取，
  不复用键盘焦点，也不进入手臂/手部遥操作输入。
- 独立 recording supervisor 对踏板输入做独占读取、去抖和单次锁存，并记录
  `run_id`、递增序号、主机 monotonic timestamp 与 outcome；重复触发必须幂等。
- 锁存时刻作为数据集 episode 的逻辑截止点；控制端进入 hold，随后沿现有 terminal
  ack → receipt → rosbag SIGINT → checksum 顺序有序闭合。
- 未来真机沿用该 episode 语义，但踏板终止不得替代硬件急停、安全 PLC 或驱动器停机。
- 手势和语音不作为主终止方式，避免污染动作数据或产生误触发。本项当前仅为计划，
  尚未实现。

## 验证责任

- 普通 Python：schema round-trip、q20/q27 保真、run closure/incomplete、topic/QoS 投影；
- Workstation2：`colcon build/test`、20-topic allowlist、recorder-ready、MCAP 回放；
- Isaac pilot：左右 arm/hand source→intent→command→q27→post-step 逐 tick 抽查，以及
  record on/off 扰动比较。
