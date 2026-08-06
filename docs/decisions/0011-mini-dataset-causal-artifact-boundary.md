# ADR-0011：Mini 数据集因果事实与派生边界

- 状态：已接受（仿真 mini 数据集范围）
- 日期：2026-08-04
- 上游：[ADR-0009](0009-ros2-full-causal-recording-boundary.md)
- 计划：[008](../008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)

## 决策

1. 当前 `run_id == episode_id`，一次 Isaac 启动、一次 MCAP 和一次有序关闭构成一个
   episode；脚踏板、自动 reset 和常驻多 episode session 不属于本版。
2. 在线主线只发布不可变事实，不计算数据集指标、图、success、训练 resize 或模型动作。
3. 每个控制 tick 保存左右 raw Tracker、raw Glove q21、arm q7、hand q20、最终 applied
   q54、pre/post feedback、qdot、scene 和 link truth。q21 与 q20 不互相替代。
4. q54 顺序固定为左 q7、左 q20、右 q7、右 q20。NERO 限位来自固定 URDF 的临时仿真
   profile；它不是两台真机的安全边界，未来真机运动前仍须逐设备只读回读。
5. 原始控制事实为 60 Hz。策略行只从 episode 首个完整控制 tick 开始，按相对偶数 tick
   精确选择为 30 Hz；禁止 nearest、插值、填充和复制图像。
6. 三路 RGB 必须表达所选 tick 的动作前状态。视觉 artifact 可以独立于 MCAP，但每帧必须
   通过 episode、control index、simulation time、state digest 和 checksum 唯一追溯。
7. accepted/rejected/incomplete 是数据管理状态，不是任务 success/failure。删除默认采用可恢复
   quarantine；永久 purge 必须是独立显式操作。
8. Pi0.5 只影响未来 processor 兼容信息，不改变原始 q54、RGB、时间和 provenance 合同。

## 结果

- 007 若额外录制 depth，不会使 depth 进入本数据集 policy whitelist。
- 当前仿真 D405 140°投影与未来实体 D405/D435i 标定严格分离。
- 任何缺失、错序、半 tick、未知 schema 或无法追溯的派生 artifact 都必须 fail closed。
