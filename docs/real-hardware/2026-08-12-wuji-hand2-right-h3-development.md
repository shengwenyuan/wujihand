# Wuji Hand2 Beta1 右手 H3 运动映射开发与 live 记录

日期：2026-08-12 至 2026-08-13
范围：右手 `WH2KA01260803016`、SDK/Studio `2026.8.3`、固件 `2.2.3`、硬件 `0.2.0`
状态：**右手五指 S1 串行 live 与人工观察均通过；豁免范围外仍未验证**

## 官方边界与限定豁免

2026-08-13 查询的官方文档仍将 Hand2 定义为 Beta1；散热、零位、负载与可靠性尚未最终验证。
SDK 8.3 的 `joint_command` 每帧要求恰好 20 个 position/velocity/effort 元素，`enable/disable` 支持
20 位关节 mask，`emergency_stop` 作用于整手。依据：
[Hand2 文档入口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/)、
[用户须知](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/)、
[SDK 接口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)。

右手 H2 尚未严格闭合。首次 live 使用
`H2-WAIVER-20260812-RIGHT-SINGLE-JOINT`；根据首次结果和操作者明确授权，后续静态实现扩大为
`H2-WAIVER-20260812-RIGHT-S1-SEQUENCE`。新豁免只允许右手五个 S1 屈曲轴按固定顺序、每次仅一个
关节的小幅空载往返；不允许多关节同时运动，也不包含 S2/S3/S4、闭合抓取、负载、左手、Glove、
ROS2、Isaac、NERO 或数据采集。Studio 可通过 SDK bridge 与其他程序同时连接，但本项目 live 仍要求
Studio 完全退出，保持唯一 command owner。

## 首次 `index_S1` live 结果

证据：
`artifacts/validation/wuji-hand2-right-h3-index-s1-20260812-01`。

- 30 s strict preflight 通过：20 轴 `Ready`、通信窗口 100%、无新增 timeout/fault/limit；MCU 最高
  `44.782 °C`，已不受此前高温阻塞。
- 从显示 preview 到操作者输入 token 间隔约 `17.50 s`，这期间已连接的 1 kHz state/diagnostics
  subscription 没有被消费。
- 随后仅发送 49 帧 accepted command；使能后约 `0.29 s` 即 fail-closed。计划 `+0.03 rad` 尚未走完，
  最大 command 只到基线 `+0.008403 rad`。
- q4/NID6 readback 仅在 `0.293838–0.293862 rad` 之间波动，跨度约 `0.000024 rad`，没有形成可验收
  的物理位移；操作者“没有感觉到角度变化”与 receipt 一致。
- 触发项为 `sdk_dropped +32958`，executor 随即执行 whole-hand disable；最终
  `automatic_checks_passed=false`。

官方说明 `sdk_dropped` 可能来自应用订阅消费端或 SDK 内部流处理。结合本次 17.50 s 未消费窗口，
判断直接原因是 arm token 等待位置不合理造成订阅积压。该次结果不能用于判断完整 `+0.03 rad` 是否
足够可见，也不能记为关节映射通过。

## package 0.3.0 sequence 与 0.4.0 收口

独立 `packages/wujihand-hand2-hardware` package 已改为固定
`right-s1-flexion-v1` sequence：

| 步骤 | SDK/q20 | NID | Description 8.3 joint | 默认相对位移 |
|---|---|---:|---|---:|
| 1 | `thumb_S1/q0` | 1 | `r_thumb_cmc_flex` | `+0.12 rad` |
| 2 | `index_S1/q4` | 6 | `r_index_finger_mcp_flex` | `+0.12 rad` |
| 3 | `middle_S1/q8` | 11 | `r_middle_finger_mcp_flex` | `+0.12 rad` |
| 4 | `ring_S1/q12` | 16 | `r_ring_finger_mcp_flex` | `+0.12 rad` |
| 5 | `pinky_S1/q16` | 21 | `r_pinky_mcp_flex` | `+0.12 rad` |

`0.12 rad` 约为 `6.9°`，硬上限为 `0.15 rad`。执行时只使用一次空行确认，且确认发生在 SDK 连接和
subscription 建立之前；确认后才运行至少 4 s warm-up、30 s preflight 和静态 baseline。preflight
通过后，程序持续消费状态流，并在 3 s guarded hold 后自动开始，不再等待第二次输入。

五个步骤严格串行。每步重新测量 q20 baseline、检查 Description limit 加 `0.1 rad` margin、发送完整
q20 零 velocity/effort feed-forward、仅使能当前一个 S1 轴，以 1.5 s 外移、0.5 s 保持、1.5 s 返回，
随后 disable 并检查 readback，再进入下一指。任一失败立即终止余下步骤并 whole-hand disable；disable
失败才升级为 SDK whole-hand emergency stop。

0.4.0 收口后，通信只保留一个 Gate：`joint_diagnostics` 任一关节 response 低于 `85%`。`85–99%`、
timeout/transport/`comm_diag` counter 变化继续写入 receipt，但不单独阻断。状态/诊断 stale、缺失关节、
fault/limit、非有限值、非目标轴异常位移、目标未形成位移、回程误差、MCU 达到项目临时上限
`58 °C` 或异常升温仍是直接安全 Gate。`58 °C` 与 `85%` 都是项目门槛，不是 Wuji 官方额定值。

静态验收结果：Ruff、strict mypy、`36 passed`；覆盖确认前零连接/零写入、五个 mask 的顺序与唯一性、
热设备拒绝、`85%` 边界通过、`84%` 阻断、state watchdog、发送异常、Ctrl-C、disable 失败后 emergency stop、
实际 SDK 8.3 adapter 签名和非交互拒绝。`lenovo-piper2` 离线构建
`wujihand_hand2_hardware-0.3.0` sdist/wheel 通过；0.4.0 收口后本地 sdist/wheel 再次构建通过，精确同步
硬件 package 后 `lenovo-piper2` 复跑 `36 passed`。本次 0.4.0 验证未建立 SDK 会话、未 enable、未发送
任何设备命令，也未启动 Studio/Isaac。

## 2026-08-13 H3-06 最终 live 结果

证据目录：
`artifacts/validation/wuji-hand2-right-h3-s1-sequence-20260813-06`。

- 自动 sequence 完成拇指、食指、中指、无名指、小指五个 S1 步骤，共 `2105` 个 command frame；
  每步仅使能一个 mask，结束后 20 关节均为 `Ready` 并断开连接。
- 五步目标轴最大有符号位移约为 `0.033–0.089 rad`；非目标轴最大位移均小于 `0.009 rad`；回程误差
  约为 `0.014–0.047 rad`，满足本轮 bring-up 的放宽验收范围。
- 运行中出现孤立 `85%/87%` response 窗口，但没有对应 timeout 增量、E2E loss、SDK drop、fault、
  limit 或非有限值；最高 MCU 温度为 `56.553 °C`。
- 操作者于运行后确认“看到了效果”，接受五指按序、单轴可见运动。本地 receipt 的
  `operator_observation` 字段保持运行结束时的 `pending` 原值，本节作为不可回写 artifact 之外的正式
  人工验收记录。

结论：右手 `right-s1-flexion-v1` 在限定豁免内通过 H3。该结论不覆盖 S2/S3/S4、多关节同步、闭合抓取、
负载、左手、Glove/ROS2 遥操作或数据采集。

## 85% 通信基线决策

此前的 `100%` 绝对 Gate 会把本机重复出现、且不伴随其他错误的孤立窗口判成失败。自 2026-08-13 起，
本项目对当前右手 Beta1 + firmware `2.2.3` + SDK `2026.8.3` + 当前网络拓扑采用 `85%`（含）作为
response-rate 下限。低于 `85%` 才阻断；其余通信计数只保留证据。

该决策是基于当前真机证据和操作者风险接受的项目策略，Wuji 官方文档没有给出 `85%` 合格线。硬件、
固件、SDK、网络拓扑或命令频率变化后必须重新评估，不能外推到左手或其他设备。
