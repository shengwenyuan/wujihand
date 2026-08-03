# 2026-08-03 ROS2 banana grasp pilot -02 审计

## 运行与动作

- Run：`banana-grasp-pilot-20260803-02`
- 时长：`103.54 s`，共 `116,951` 条消息，20/20 topic 非空
- 操作：仅左手/左臂；适应后抓起香蕉并放到碗上。右侧信号按操作者说明不用于本轮判断。

## 数据闭环

- 左 Tracker：`120.0 Hz`，sequence 连续，位姿有效。
- 左 Glove q21：`115.4 Hz`，21/21 landmark 全部有效；动作幅度明显。
- 左 q20：最大关节范围约 `0.32～1.37 rad`；q7 最大关节范围约 `0.71～1.54 rad`。
- q7/q20 写入 applied q27 的最大组合误差为 `0`。
- 香蕉约在第 `49.9 s` 开始移动，最高抬升约 `17.6 cm`，最终位移约 `31.1 cm`。

因此 q21→q20→q27→Isaac 物体运动链完整，动作描述与数据一致。

## 1–4 定位

1. **关闭竞态复现。** rosbag 在 Ctrl+C 后约 `0.13 s` 完成关闭并先行 finalize，consumer
   约 `0.40 s` 后才退出；产物因此仍为 `consumer_receipt_missing / incomplete`，terminal
   status 也未进入 bag。
2. **控制低频根因复现。** 原始输入约 `115～120 Hz`，Isaac tick `48.8 Hz`，每路新样本
   仅约 `9.3～9.5 Hz`，sample age P95 约 `87 ms`。单独按
   [NV-5.1 60 Hz feature 计划](../005-ros2-isaac-60hz-control-feature-plan.md)处理。
3. **不应加入低置信度硬拒绝。** 戴手套左侧的最小 landmark confidence P05 约 `0.626`，
   但低于 `0.50` 的片段出现在有效抓取期间。硬拒绝会触发 stale/hold，反而破坏抓取。
   保留 admission floor `0.0`，只把 success/degraded 标签阈值校准为 `0.60`。
4. **任务 fixture 语义错误。** 碗在香蕉开始运动前已移动约 `4.2 cm`；桌子虽只移动约
   `6 µm`，但二者均被建模成 dynamic rigid body。banana-bowl profile 改为桌子和碗固定、
   香蕉为唯一动态任务对象；固定 pose 写入 manifest，逐 tick 只录动态对象。

## 本轮修改边界

- 修复 recorder/consumer 有序关闭；
- 校准 Glove 质量标签，不改变 q20 admission；
- 固定桌子和碗；
- 控制频率只写 feature 计划，本轮不实现；
- raw contact 仍为 capability false，不能从本 bag 归因具体接触 link/point/force。

本地测试通过后仍需在 Workstation2 做一次短 pilot，确认 receipt complete、terminal status
存在、table/bowl 固定和 banana 动态，再开始正式三轮采集。
