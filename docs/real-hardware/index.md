# 真机开发文档入口

本目录只记录真实 Wuji Hand2 / NERO 的设备发现、资格验证、受控运动和后续真实遥操作事实。
它与 `docs/validation/` 下的 Isaac / ROS2 仿真验收同仓库保存，但结论互不继承；仿真 tick、drive
参数、contact truth 和 dataset 资格不能作为真机依据。

总体架构与分阶段 Gate 仍以
[015：Wuji Hand2 Beta1 真机 Bring-up、仿真解耦与遥操作采集预埋计划](../015-wuji-hand2-beta1-real-hardware-bringup-and-teleoperation-data-plan.md)
为准。本目录保存实施后的状态和证据摘要。

## 当前状态

截至 2026-08-14：

| 范围 | 状态 | 结论 |
|---|---|---|
| 右手 H0 | 通过 | 网络、唯一身份、side、硬件/固件版本和 SDK 8.3 发现闭合 |
| 公共 H1 | 通过 | 独立 `wujihand-hand2-hardware` package 和只读资格验证路径已建立；未接入仿真主线 |
| 右手 H2 | **通信项关闭；长期热平衡未关闭** | 当前固定栈接受 `85%`（含）response 下限；约 497 s 的 `+5 °C` 长时温升事实仍保留 |
| 右手 H3 | **限定范围通过** | 五个 S1 轴按 thumb→pinky 串行完成，自动检查通过，操作者确认看到了预期效果；S2/S3/S4 与全手动作未覆盖 |
| 右手 H4 | **官方 example 功能证据通过；H4-A 全 20 轴待验收** | 官方 SDK 8.3 右 Glove→校准 URDF→右 Hand2 效果正常；0.5.2 保留 one-hot mapping 审计并放宽最小可检测位移 |
| 左手 | 未开始 | 不能继承右手结论 |

右手详细结果见
[Hand2 Beta1 右手 H0–H2 只读资格验证](wuji-hand2-right-h0-h2-validation.md)。
H3 限定豁免、实现、验证与 live 交接见
[Hand2 Beta1 右手 H3 有界运动与 live 记录](wuji-hand2-right-h3-bounded-motion.md)。
H4 分段、开发边界与 live Gate 见
[Hand2 Beta1 右手 H4 全关节台架计划](wuji-hand2-right-h4-full-joint-bench.md)。

## 长期边界

- Hand2 Beta1 的热、零位、软皮和长期可靠性仍是官方明确的 Beta 限制；固件、SDK、Studio 或硬件
  revision 变化后，既有真机 receipt 失效。
- `85%` 是当前右手固定栈的项目 response-rate 下限，不是官方指标；低于它才阻断，其他通信计数
  只保留证据。该策略不能外推到左手或不同软件/网络组合。
- 真机 package 默认无命令能力；进入 H3 后也只能由单一 executor 写设备。
- 原始 serial、设备日志和逐帧证据留在本地 ignored artifact；正式文档只保存必要的脱敏身份、hash、
  汇总和 Gate 结论。
- H3/H4 package 只做独立台架动作；官方 Glove teleoperation example 只作为厂商路径功能证据，
  不等于项目 ROS2 真机 teleop。NERO、Isaac、相机和数据采集仍属于后续独立需求。
- 人工观察只补充动作方向与可见性，不替代温度、fault、limit、stale、q20 完整性和 `<85%` Gate。
