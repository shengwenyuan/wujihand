# NV-5 ROS 2 Jazzy Workstation2 HIL 验证

- 日期：2026-07-31
- 主机：Workstation2 / Ubuntu 24.04
- ROS：Jazzy / Fast DDS / `ROS_DOMAIN_ID=57`
- Isaac：6.0.1 / Python 3.12
- 范围：双 Tracker、双 Wuji Glove、Isaac 双 NERO + 双 Hand 2；不连接真机

## 结果

NV-5 ROS 主链路功能 HIL 通过：

- 双 Glove 专用网口均为 link up，左右设备可达；
- SteamVR 同时枚举两枚 Tracker，OpenVR source 可持续发布两侧 canonical pose；
- 双臂 only 场景完成 XYZ、RPY 人工验证，用户确认方向与行为正确；
- 双臂双手场景完成左右手分别动作、同时动作与双臂动作，用户确认正确；
- VIVE source deactivate/activate 不退出 GUI，旧 reference 被撤销后可重新进入 tracking；
- Glove source deactivate 后双手进入受监督的无输入路径；
- Glove source activate 后使用新 transport epoch，左右 retarget state 独立重置；
- 修复后 Glove source deactivate/activate 不再退出 GUI，两个 LifecycleNode 均保持
  `active [3]`，Isaac consumer 持续运行。

最终 GUI 由用户保持开启并确认状态正确。四条 command topic 在同一 GUI 会话持续
发布；并行冷启动采样稳定后约为 `54～60 Hz`。该数据只证明链路持续性，不作为性能
阈值。

## 发现并修复的问题

Glove producer epoch 改变时，旧实现会在同一 control tick 内：

```text
invalidate epoch -> supervisor.hold(now_ns=t)
cycle.step        -> supervisor.step(now_ns=t)
```

这违反 `JointCommandSupervisor` 的严格单调时间合同。Isaac 的 fast-shutdown 又会把
未打印的异常表现为退出码 `0`，因此最初看起来像 GUI 正常关闭。

修复后的应用层语义为：

```text
invalidate epoch -> 清空该侧 transport/intent 状态并登记 pending hold
next cycle tick   -> 原子消费一次 hold
following tick    -> 接受新 epoch 的首个合法 observation
```

该修改位于 transport-neutral Glove application controller；ROS composition root
只转发 epoch 事件。没有增加 ROS 专用控制逻辑或临时恢复分支。Isaac 入口同时保留
异常堆栈并以非零状态退出，避免后续故障被 fast-shutdown 掩盖。

## 自动回归

- `653 passed, 4 skipped, 11 deselected`
- Ruff：通过
- `mypy src`：93 个 source files 通过
- `git diff --check`：通过
- Glove controller、双侧 controller set 与 shared cycle 定向测试：
  `24 passed`

## 仍待完成

- 未在本次最终会话重复物理单侧 Tracker 遮挡或单侧 Glove 断网；side-local
  fault/epoch 行为已有自动测试，真实单侧故障仍保留为 R7 的最后现场检查。
- 尚未形成延迟、jitter、drop/overwrite、CPU 和 real-time factor 的
  p50/p95/p99 基线，也未批准进入 NV-6 的性能阈值。
