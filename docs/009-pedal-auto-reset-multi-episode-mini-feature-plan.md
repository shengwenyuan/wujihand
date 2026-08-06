# 009：脚踏板结束、自动 Reset 与连续多 Episode 采集小计划

- 状态：未来功能；不阻塞 008 的当前单 episode 采集
- 日期：2026-08-04
- 当前前提：尚无脚踏板，不在本轮或 008 第一阶段实施
- 上游数据合同：[008：三目 q54 Mini 数据集计划](008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)
- 原始录制边界：[ADR-0009](decisions/0009-ros2-full-causal-recording-boundary.md)
- 目标：Isaac 和输入 source 长驻，操作者踩踏板结束当前 episode，系统有序闭合独立
  MCAP、自动 reset 场景并准备下一 episode

## 1. 语义修正

脚踏板只表示操作者请求结束当前录制，不表示：

- task success；
- reward；
- 物体已经抓住；
- 香蕉已经放入碗；
- 模型或遥操作达到质量阈值；
- 急停或安全停机。

因此本功能采用：

    finish_requested

而不是 success。若未来使用双踏板，第二踏板可以表达：

    reject_requested

它只表示当前 episode 在闭合后直接进入 rejected quarantine，仍不表示 task failure。

ADR-0009 中早期设想的 success/abort 双踏板语义，在本功能真正实施前应通过新 ADR 修订为
finish/reject 或其他最终确认语义，不能直接沿用旧文字。

## 2. 目标工作流

    collection session start
      -> load Isaac scene once
      -> activate Tracker/Glove sources once
      -> reset and settle
      -> establish references
      -> start episode recorder
      -> recorder-ready
      -> episode ready / recording
      -> operator teleoperation
      -> pedal finish_requested
      -> complete current control tick
      -> hold
      -> close episode MCAP/receipt/checksum
      -> quick structural validation
      -> reset robot and scene
      -> settle and rearm references
      -> start next episode recorder
      -> next episode ready

Isaac、ROS source 和 collection supervisor 在多条 episode 之间保持运行；每条 episode 仍有
独立 run_id、episode_id、rosbag2 metadata、MCAP、receipt 和 checksum。

## 3. 非目标

- 不训练模型；
- 不计算 success；
- 不自动判断抓取；
- 不连接 NERO/Hand 2 真机；
- 不把脚踏板当急停、deadman 或 PLC；
- 不把多个 episode 写入一个无法独立闭合的 MCAP；
- 不在 reset 期间录制 policy rows；
- 不在每条结束后同步等待离线三目渲染；
- 不自动永久删除 rejected episode；
- 不取消 008 的 exact-tick、q54、相机和 LeRobot 合同。

## 4. 当前与目标架构差距

当前实现：

- launch 创建一个 run_id；
- recorder 与 consumer 都按 run 生命周期启动；
- consumer 或 recorder 退出会关闭整个 launch；
- trace publisher 仅在进程启动时决定是否创建；
- Ctrl+C 结束整个 run；
- 没有 episode supervisor；
- 没有场景 reset/rearm；
- 没有动态启动下一 recorder；
- 没有脚踏板输入。

目标实现需要把身份分为：

    collection_id
      ├── episode_id_0001 / run_id_0001
      ├── episode_id_0002 / run_id_0002
      └── episode_id_0003 / run_id_0003

collection session 不是 episode，也不拥有跨 episode 的 policy timeline。

## 5. 状态机

建议唯一 supervisor 状态：

    SESSION_STARTING
      -> SCENE_LOADING
      -> RESETTING
      -> SETTLING
      -> WAITING_INPUTS
      -> WAITING_REFERENCE
      -> STARTING_RECORDER
      -> WAITING_RECORDER_READY
      -> COUNTDOWN
      -> RECORDING
      -> STOP_REQUESTED
      -> DRAINING_TICK
      -> HOLDING
      -> FINALIZING_RECORDER
      -> QUICK_VALIDATING
      -> RESETTING

异常进入：

    QUARANTINING_EPISODE
      -> SESSION_PAUSED

或：

    SESSION_FAILED

任何状态不得隐式跳过 recorder-ready、scene settle 或 reference Gate。

## 6. 脚踏板输入合同

### 6.1 硬件边界

具体脚踏板型号、USB/HID/evdev 接口和设备路径在采购后冻结。实现必须使用：

- 稳定设备身份或 udev symlink；
- 独占 reader；
- 版本化 local binding；
- 明确 vendor/product/serial pseudonym；
- 无 shell key injection；
- 不依赖 GUI 键盘焦点。

脚踏板配置放入 local runtime binding 或独立 hardware profile，不通过临时启动参数散落。

### 6.2 原始事件

建议 PedalEvent.v1 至少保存：

- collection_id；
- source_id；
- producer_instance；
- transport_epoch；
- sequence；
- raw device event；
- pressed/released；
- host monotonic time；
- debounce decision；
- semantic event，finish_requested 或 reject_requested；
- active episode_id；
- supervisor state；
- accepted/ignored；
- reason。

### 6.3 去抖与幂等

- 按下沿只锁存一次；
- 弹起不重复触发；
- 去抖窗口由版本化 profile 冻结；
- 同一 episode 首次有效请求获胜；
- 重复按压记录为 ignored_duplicate；
- 非 RECORDING 状态的按压记录并忽略；
- 设备断连产生 lifecycle event，不伪造结束。

### 6.4 安全边界

脚踏板：

- 不能停止实体电机；
- 不能替代急停；
- 不能替代真机 watchdog；
- 不能解除 safety hold；
- 不能直接调用 Isaac API；
- 只向 collection supervisor 提交有序 stop request。

## 7. Episode Recorder 生命周期

### 7.1 每条独立 recorder

每个 episode：

1. 生成唯一 run_id/episode_id；
2. 创建全新 run root；
3. 写 started manifest；
4. 启动独立 rosbag2 recorder wrapper；
5. 等待完整 frozen topic subscription；
6. 进入 COUNTDOWN/READY；
7. 录制；
8. 结束后发布 terminal boundary；
9. 停止该 recorder；
10. finalize metadata/receipt/checksum；
11. 关闭后不再写该目录。

不得依靠一个 rosbag split file 把多个 episode 伪装成独立 artifact，除非 rosbag2 能为每段
提供独立 metadata、receipt、checksum 和 crash closure；初版优先每 episode 独立 recorder
process。

### 7.2 Trace publisher

长驻 Isaac consumer 可以持续创建 trace publisher，但只在 RECORDING 窗口发布候选 episode
facts，或为所有 tick发布并由 recorder订阅窗口物理裁定。必须选择一种并冻结：

- 推荐：publisher长驻，recorder只在episode窗口订阅；
- episode boundary提供第一/最后有效tick；
- 非录制期控制仍可运行hold/reset，但不会进入任何episode MCAP。

不得复用上一 episode 的 run_id；每条消息必须使用当前 active episode identity。

### 7.3 Recorder 启停扰动

启动/关闭 rosbag 发生在 hold 状态，不计入 episode rows。进入 RECORDING 前重新检查：

- control 60 Hz；
- RTF；
- source age；
- mailbox稳定；
- recorder-ready。

若 recorder process 启动造成 scheduler miss，清空候选起点并等待连续稳定窗口，不能把 miss
藏在 ready 之前后直接开始。

## 8. Finish 请求与精确终点

踏板按下 host time不是数据集终点。

流程：

1. supervisor 在 RECORDING 锁存 finish_requested；
2. 向 Isaac consumer提交 stop request；
3. consumer 完成当前 control tick；
4. 确认左右 action、两个physics substep、post-feedback完整；
5. 记录 effective final control index；
6. 进入 hold；
7. 发布 episode stop/closed boundary；
8. 等待 recorder收到 terminal status；
9. 有序停止 rosbag；
10. finalize。

若 stop request 到达当前 tick 左右 action 提交之间，必须完成该共享 physics phase或将整 tick
判废，不能保留单侧 transition。

reject_requested 使用相同关闭流程，只在 finalize 后自动添加 rejected annotation。

## 9. 自动 Reset 合同

### 9.1 Reset 范围

每次 reset 至少恢复：

- 左右 NERO q7 初始位；
- 左右 Hand 2 q20 rest；
- articulation qvel；
- drive target；
- dynamic object pose；
- dynamic object linear/angular velocity；
- sleep/kinematic状态；
- scene simulation time/index的episode局部原点；
- 已声明的随机初值；
- contact/solver缓存的可用清理路径；
- camera和fixture固定状态验证。

不只写 random seed；manifest保存实际 reset readback。

### 9.2 Reset 不进入 episode

RESETTING、SETTLING、WAITING_INPUTS、WAITING_REFERENCE 和 COUNTDOWN 的 tick：

- 不进入上一 episode；
- 不进入下一 episode；
- 不生成LeRobot rows；
- 可写collection-level诊断日志；
- 不得污染episode q54 stats。

### 9.3 Settle Gate

进入下一 recorder 前验证：

- 左右关节位置在初始容差内；
- qdot低于阈值；
- dynamic object位置/速度在初始容差内；
- 桌面/fixture无漂移；
- physics无持续爆炸/contact异常；
- 连续若干physics step稳定；
- GUI/viewer状态正常。

### 9.4 Reference Rearm

场景和机器人回到初始位后，旧 Tracker reference 不应无条件继续控制新 episode。

推荐：

1. reset期间强制 arm/hand hold；
2. 撤销上一 episode 的 arm reference；
3. 等待左右 Tracker/Glove 同 epoch 稳定；
4. 操作者回到约定自然起始姿态；
5. 重新建立左右 reference；
6. 检查第一条映射无大跳变；
7. 进入 countdown；
8. 下一 episode ready。

单侧 reference失败时不允许只开始另一侧正式 q54 episode。

## 10. Countdown 与操作者反馈

自动 reset 后不能在操作者未知时突然开始录制。COUNTDOWN 至少提供：

- 终端清晰状态；
- 剩余秒数；
- 左右 source/reference ready；
- 当前 episode序号；
- 可选非侵入提示音；
- 取消/暂停入口。

倒计时结束后的第一个完整 control tick 是 episode ready 起点。操作者提前运动会被记录为
pre-ready诊断，不进入episode。

若 COUNTDOWN 中输入失效，退回 WAITING_INPUTS，不带病开始。

## 11. 快速验证与异步后处理

每条闭合后只执行足够决定能否继续 reset 的快速检查：

- receipt/checksum/metadata；
- topic inventory；
- episode boundary；
- 左右 tick count；
- schedule miss；
- fatal source/reference/trace failure。

通过后标记 candidate，立即 reset 下一条。

以下耗时工作在 collection session 后批量执行：

- 完整 analyzer；
- 30 Hz alignment；
- 离线三目渲染；
- vision checksum；
- LeRobot export；
- 质量图和人工审阅。

这样不会让每条之间等待 GPU离线渲染，也不会让后处理失败破坏下一条raw采集。

若快速验证失败：

- 当前 episode自动quarantine；
- 禁止自动开始下一条；
- session进入PAUSED；
- 显示原因并等待人工决定resume或stop。

## 12. Collection 目录

建议：

    artifacts/collections/<collection_id>/
    ├── collection_manifest.json
    ├── collection_events.jsonl
    ├── collection_receipt.json
    ├── episodes.jsonl
    └── checksums.sha256

每条 episode仍保存在008定义的独立run root。collection manifest只引用：

- episode_id/run_id；
- 相对路径；
- 开始/结束顺序；
- candidate/rejected/incomplete；
- source/session/profile hash；
- 快速验证结果。

collection文件不复制MCAP，不承担单帧事实。

## 13. 故障处理

| 故障 | 行为 |
|---|---|
|脚踏板重复触发 |首个有效，后续幂等忽略并记录 |
|脚踏板断连 |当前episode可继续由软件stop；结束后session暂停 |
|source epoch改变 |当前episode闭合为incomplete/quarantine，重新rearm |
|reference revoke |当前episode停止并quarantine |
|recorder退出 |立即hold，session failed，不自动reset继续 |
|receipt/checksum失败 |quarantine，session paused |
|reset失败 |session failed，不启动下一recorder |
|settle超时 |session paused，保留reset诊断 |
|schedule miss |当前episode reject，不自动迁就 |
|GUI关闭 |视为session stop请求，不是episode finish |
|进程崩溃 |已闭合episode保持；active episode标incomplete |
|磁盘不足 |不启动新episode，保持hold并关闭session |

恢复不得复用未闭合episode_id。

## 14. 配置与事实所有权

### Hardware/local binding

- 脚踏板设备路径；
- reader runtime；
- 权限；
- debounce profile引用。

### Deployment

- collection supervisor process；
- pedal source process；
- episode recorder wrapper；
- namespace和report root。

### Session/Workcell

- reset profile；
- 初始robot/object状态；
- settle Gate；
- scene revision。

### Dataset profile

- episode边界；
- q54/camera合同；
- 快速和完整Gate；
- accepted/rejected规则。

启动参数不得复制设备identity、reset pose或debounce数值。

## 15. 实施切片

### P0：硬件 spike 与 ADR

- 选择脚踏板；
- 验证Linux稳定设备路径；
- 冻结finish/reject语义；
- 修订ADR-0009；
- 确认脚踏板不是安全设备。

### P1：Pedal source

- 独立node/adapter；
- raw event/lifecycle；
- debounce/sequence/epoch；
- 无GUI焦点依赖；
- 断连恢复。

### P2：Collection supervisor

- 状态机；
- collection/episode identity；
- stop request；
- pause/resume/stop；
- collection manifest。

### P3：动态 episode recorder

- 每episode启动/ready/terminal/finalize；
- consumer长驻；
- run_id切换；
- shutdown ownership；
- 异常closure。

### P4：Scene reset

- robot/object全状态reset；
- settle；
- 实际readback；
- reference revoke/rearm；
- countdown。

### P5：快速验证与 quarantine

- 同步结构Gate；
- candidate/rejected/incomplete；
- 失败暂停；
- 与008 episode manager复用。

### P6：Workstation2 连续资格验证

顺序：

1. synthetic pedal events，3 episode；
2. 实体pedal，无Tracker/Glove；
3. 实体pedal + 四路input，自由空间3 episode；
4. 短任务5 episode；
5. 故障注入；
6. 完整离线批处理。

## 16. 测试矩阵

### 普通 Python

- state machine全transition；
- 非法transition；
- debounce；
- duplicate；
- collection/episode identity；
- path safety；
- crash recovery；
- manifest/checksum；
- reset profile validation。

### ROS2

- pedal QoS/lifecycle；
- consumer/recorder discovery；
- 动态recorder启停；
- run_id切换；
- terminal ack；
- source epoch/restart；
- 未知旧epoch拒绝。

### Isaac headless

- robot/object reset；
- qvel/drive target；
- settle；
- reference rearm；
- simulation time/episode origin；
- 连续episode无状态串扰。

### Isaac GUI/HIL

- 脚踏结束；
- hold；
- 自动reset；
- countdown；
- 下一条ready；
- 60 Hz零miss；
- 连续5条artifact闭合。

### 故障注入

- 踏板bounce/断连；
- recorder crash；
- consumer crash；
- reset exception；
- disk full；
- checksum failure；
- source epoch change；
- reference revoke；
- 重复finish；
- finish落在tick各phase。

## 17. 性能 Gate

记录阶段沿用008：

- 60 Hz ±2%；
- 120 Hz physics；
- 20 Hz GUI；
- RTF ≥0.95；
- schedule miss=0；
- input age满足dataset profile。

额外要求：

- recorder启动发生在非episode hold窗口；
- reset/settle耗时单独报告，不混入episode；
- 踏板event到stop request host延迟可见；
- stop request到effective final tick不超过一个完整control边界加有序drain；
- episode之间没有source/identity泄漏；
- 连续5条无进程资源泄漏、publisher重复或subscriber残留。

## 18. 完成定义

未来功能完成必须满足：

1. 实体脚踏板可稳定产生finish_requested；
2. 事件去抖、序列、epoch和断连可审计；
3. 脚踏板不承担success或急停语义；
4. Isaac/source长驻；
5. 每episode独立MCAP/receipt/checksum；
6. 结束请求落在完整control tick边界；
7. episode关闭后自动reset robot/object；
8. reset不进入任一episode；
9. 左右reference重新建立；
10. 下一recorder ready后自动countdown/开始；
11. 连续至少5条episode无状态串扰；
12. 任一硬故障会hold并暂停，不盲目进入下一条；
13. 离线三目和LeRobot仍遵守008 exact-tick合同；
14. 当前单episode入口仍可继续使用；
15. 未引入实体机器人命令或安全能力误解。

## 19. 当前停止点

本计划现在只落盘，不实施。008 的当前数据采集继续采用：

- 一次启动；
- 一次录制；
- Ctrl+C结束；
- 完全退出；
- 下一条重新启动；
- 失败后单步reject。

在脚踏板实际选型、到货和用户明确启动本feature前，不新增pedal依赖、自动reset、动态recorder
或collection supervisor代码。
