# 2026-08-03 ROS2 banana grasp pilot -05 有序关闭复验

- 结论：**PASS**
- Run：`banana-grasp-pilot-20260803-05`
- 主机：Workstation2
- 入口：`isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2`

## 验证结果

- Ctrl+C 被 consumer 锁存为 `stop_signal=2`，未绕过 Python 关闭路径；
- MCAP 同时包含 `started` 与 terminal 两条 recording status，terminal ack 计数为 `1`；
- consumer receipt 为 `consumer_completed`，recorder 观察到 terminal receipt；
- consumer、recorder 与 rosbag 均为零退出，最终 `receipt.state=complete`；
- 单个 MCAP、metadata、manifest、recorder、receipt 的 checksum 全部通过；
- launch 日志没有 `ROS RECORDING WARNING` 或进程异常退出。

本次确认 ADR-0009 的有序关闭竞态已经修复。该 PASS 只覆盖录制闭合与产物完整性，
不替代 topic/sequence 离线质量分析，也不宣称 60 Hz 控制目标已经达成。
