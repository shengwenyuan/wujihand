# 2026-07-31 ROS2 全因果链录制离线验证

## 结论

录制契约、ROS projection、20-topic/QoS allowlist、run closure 和 incomplete 行为已通过
普通 Python 离线验证。未启动 ROS、Isaac、OpenVR、Wuji SDK 或远端设备。

## 已验证

- Glove 21×3 raw topic 可通过 producer/epoch/sequence 关联到 active q20 intent；
- Tracker raw topic 可关联到 mapping、IK q7 和最终 q7 decision；
- 每侧保存 atomic q27 target、pre-apply/post-step q27 和阶段原始时间；
- manifest/recorder identity、receipt、非空 MCAP/metadata 缺一即保持 incomplete；
- finalize 后 receipt 不可被迟到 consumer 改写，重复 finalize 幂等；
- 新 QoS projection 与完整录制 allowlist 一致；
- recorder/consumer 边界不引入质量计算或绘图依赖。

## 结果

```text
pytest -q
673 passed, 4 skipped, 11 deselected
```

录制契约定向测试、mypy 定向检查与 ruff 检查均通过。

## 尚未验证

- Workstation2 `colcon build/test` 和新 IDL 生成；
- rosbag2 subscriber-ready Gate、正常/异常 shutdown 与 MCAP 实际回放；
- Isaac dynamic rigid-body state 的目标机读取；
- record on/off 的控制周期扰动；
- 双 Tracker/双 Glove 短 pilot。
