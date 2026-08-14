# Wuji Hand2 Beta1 右手 H4 全关节台架计划

- 目标设备：右手 `WH2KA01260803016`，`192.168.1.111:7447`
- 固定栈：hardware `0.2.0`、firmware `2.2.3`、Wuji SDK / Studio `2026.8.3`
- 当前状态：H4-A live 部分完成，0.5.2 全 20 轴复跑待验收；官方 SDK example 功能链已由操作者确认正常
- 范围：独立真机台架 package 不引入 Glove/ROS2；官方 example 仅作为 package 外部功能证据

## 官方边界

2026-08-13 通过官方 `wuji-docs` MCP 复核：Hand2 仍是 Beta 1，具有 20 个主动自由度；散热、零位、
负载、软体和长期可靠性尚未最终收敛。SDK 8.3 的命令帧必须包含恰好 20 个 `JointCommand`，反馈帧
可能只含在线关节，必须按 `nid` 还原 q20；`enable/disable` 可以使用 20 位 mask，`emergency_stop`
始终作用于整手。依据：

- [产品介绍](https://docs.wuji.tech/docs/zh/wuji-hand/latest/overview/)
- [使用约束](https://docs.wuji.tech/docs/zh/wuji-hand/latest/usage-constraints/)
- [SDK 接口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)
- [Hand2 发布记录](https://docs.wuji.tech/docs/zh/wuji-hand/latest/release-notes/)

`85%`（含）仍只是当前右手固定栈的项目 response-rate 下限，不是 Wuji 官方指标。低于它阻断；
timeout、transport 和 `comm_diag` 计数保留证据，不叠加第二套通信 Gate。fault、active limit、非有限值、
q20 缺失、反馈 stale 和温度条件始终独立阻断。

当前 H3/H4 温度策略为：MCU 绝对温度达到 `65 °C` 或相对本次启动基线升温达到 `+5 °C` 时阻断。
0.5.1 根据正常持续上电证据把原 `58 °C` 绝对线放宽为 `65 °C`；温度仍逐帧记录。两个值都是当前
右手空载 bring-up 的项目门槛，不是 Wuji 官方额定温度，也不授权长时间满载测试。

## H4 分段

### H4-A：20 关节逐轴隔离

本轮实现固定 profile `right-q20-isolation-v1`。一次人工回车在 SDK 连接前确认整段测试，随后完成新鲜
warm-up、30 秒只读 preflight 和静态 baseline。顺序为 thumb、index、middle、ring、pinky，每指依次
S1、S2、S3、S4，共 20 步：

1. sequence 开始前读取 MIT 参数和 effort limit，每步重新读取 q20 baseline，不写入任何参数；
2. 发送完整 q20 position command，velocity/effort feed-forward 为零；
3. 仅使能当前关节的一位 mask，其余 19 轴保持 baseline；
4. 默认沿正方向低速移动 `0.12 rad`、短暂停留、返回本步 baseline、disable，再做 readback；
5. 目标若接近 Description 8.3 limit 加 `0.10 rad` margin，则在 enable 前拒绝该步；
6. 任一步失败就取消余下步骤并 whole-hand disable；disable 失败才调用 whole-hand emergency stop。

这里的正方向是待验收对象，不预先声称 S2/S3/S4 的真实机械方向已闭合。H4-live 时，操作者必须逐步
确认界面所报 q/index/NID 与实际运动关节一致、方向符合官方运动方向图、没有串指或异常干涉。自动
readback 只能证明 commanded/observed q 和隔离性，不能替代这项人工观察。

H4-A 自动通过要求：20 个 one-hot mask 顺序正确、每帧 q20 完整、目标轴形成非零可检测位移、
非目标轴变化和回程误差在当前 bring-up 放宽门槛内、最终全部 `Ready`，且直接安全 Gate 全程未触发。
0.5.2 将 H4-A 最小检测量设为命令 delta 的 `1%`（默认 `0.0012 rad`）；H3 仍为 `5%`。该阈值只用于
排除完全无响应，不代表跟踪性能合格；每轴方向和隔离仍必须由操作者观察。通过后才冻结 S2/S3/S4
的方向事实。

### H4-B：官方路径功能证据

不再在本 package 内开发第二套多关节功能手型 executor。张开、分指、逐指屈伸和对指使用固定版本的
官方 SDK teleoperation example 验证；它是厂商路径功能证据，不是本项目 ROS2 真机 teleop 实现。
运行官方 example 时它是唯一 command owner；结束并释放连接后，H4-A 才能重新接管设备。

2026-08-14，操作者已运行官方 SDK 8.3 example 并确认右手遥操作效果正常。SDK 日志
`/home/lenovo/.wuji/logs/sdk_2026-08-14.log` 记录了：

- `18:38:20` 发现右 Hand2 `WH2KA01260803016` 与右 Glove `WG1KA06260623528`；
- 加载 Studio 校准用户目录中的 `models/right_hand.urdf`，订阅右 Glove `hand_skeleton`；
- 对 Hand2 执行 `SET effort_limit` 与 `SET mit_params`，随后 Hand2 bridge 上线；
- `18:40:54` bridge 退出，操作者报告整个遥操作效果正常。

同一窗口存在 `Unknown wire expr`、Glove `upgrade_status` 和退出时 `Unknown interest` 日志；它们没有
伴随操作者可见的控制失败，本轮作为非阻断 SDK/bridge 观测保留，不解释为 Hand2 动作故障。

该日志没有逐帧 q20、joint diagnostics 或温度，因此不能替代 H4-A receipt；官方 example 写入过
控制参数，下一次 H4-A 必须重新只读记录实际 MIT/effort readback，不能假设仍是历史值。

## 本轮开发与离线验收

只允许修改 `packages/wujihand-hand2-hardware/**` 和本目录真机文档：

- 在现有 sequence executor 中增加固定 H4-A profile，不复制第二套控制器；
- 保留 H3 profile 与 CLI 调用方式；profile 和 scope-id 不匹配时在确认、连接和写入前拒绝；
- fake SDK 覆盖 20 步顺序、one-hot mask、q20 command、限位预拒绝和 fail-closed；
- 执行 Ruff、strict mypy、pytest、wheel/sdist 构建和 import 边界检查；
- 在 `lenovo-piper2` 只做精确 package 同步及离线复验，不建立 SDK 会话。

静态验收完成后停在 H4-live 前。届时再给出唯一执行语句、上电/净空/Studio 退出动作、逐步观察清单
和停止标准；未获得操作者在场确认前，不连接右手、不 enable、不发送命令。

### H4-A-dev 结果

`wujihand-hand2-hardware` 已升级到 `0.5.0`，在原 sequence executor 内加入
`right-q20-isolation-v1` 和固定 scope `H4-RIGHT-Q20-ISOLATION-V1`；H3 原 profile 与
`--waiver-id` 调用仍可用。fake SDK 已证明 20 个 NID 按协议顺序执行、20 个 mask 均为 one-hot、
所有 command 均完整包含 q20、每步返回 baseline 并 disable，且接近 guarded Description limit 时在
command stream 打开前拒绝。

本地验收：Ruff 通过、strict mypy 源码通过、`40 passed`，0.5.0 wheel/sdist 构建通过。
`lenovo-piper2` 只精确同步了该 hardware package；Linux Python 3.11 隔离环境固定安装
`wuji-sdk==2026.8.3` 后，Ruff、strict mypy、同一组 `40 passed` 和 0.5.0 wheel/sdist 构建再次通过。
本轮未发现/连接 Hand2，未启动 Studio/Isaac，未 enable、未发送命令，也未修改仿真代码、配置或入口。

结论：H4-A-dev 通过并已部署；S2/S3/S4 正方向、真实 baseline、当前温度和 20 步可见隔离性仍只能
由下一次 H4-live 得出，当前不写作 H4-A 或 H4 通过。

### 0.5.2 简化调整

官方 SDK example 已证明当前固定栈可完成右 Glove→校准 URDF→右 Hand2 的整手功能动作，因此不再
复制 H4-B 控制逻辑。H4-A 保持原 one-hot executor，只把 H4 的最小 observed excursion 从 `5%`
降为 `1%`；温度、通信、fault、limit、stale、q20 完整性、非目标轴和回程 Gate 均保持不变。

## 收口判定

- H4-A-dev：**已通过**；0.5.2 只做 profile 级小幅放宽；
- H4-A-live：20 个关节逐轴的自动 receipt 与人工方向/隔离观察都通过；
- H4-B 官方路径功能证据：**已通过粗粒度人工验收**，不新增 package 控制代码；
- H4：待 H4-A-live 全 20 轴通过后，右手单手设备调试阶段收口；
- 左手仍必须从 H0 独立开始，不能继承右手任何 mapping 或通信结论。
