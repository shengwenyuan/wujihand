# NV-5 ROS 2 Jazzy 离线验证

- 日期：2026-07-30
- 主机：Workstation2 / Ubuntu 24.04
- ROS：Jazzy / Fast DDS
- Isaac：6.0.1 / Python 3.12
- 硬件状态：双 Tracker、双 Glove 均下电

## 已通过

- 本地全量回归：`650 passed, 4 skipped, 11 deselected`；
- Ruff 全部通过，mypy 检查 `109` 个 source files 无问题；
- 两个 ROS package 完成 `colcon build --symlink-install`；
- ROS package 测试：`3 tests, 0 failures`；
- 五个 custom message 生成 Python/C/C++ typesupport；
- VIVE source Lifecycle configure 成功，状态为 `inactive`；
- Glove source Lifecycle configure 成功，左右 observation topic 均存在；
- native 共享场景抽取后完整 NV-2 headless qualification 全部通过；
- ROS consumer 在无输入条件下运行 60 frames，GUI 生命周期不依赖输入；
- 四 route fixture 回放运行 1200 simulation frames；
- 左右 Tracker 各建立一次 reference，并各获得 90 次 `ik_accepted`；
- 四个 input inbox 各接收 103 个合法样本，contract/identity 拒绝数均为 0；
- 双手 fixture 进入 retarget/supervision；
- run report 不包含设备明文 identity。

## 未完成

设备下电期间未执行 source activate、真实数据周期/抖动统计和 Workstation2 HIL。
这些项目保留为 NV-5 R7 Gate，重新上电前不尝试设备连接。
