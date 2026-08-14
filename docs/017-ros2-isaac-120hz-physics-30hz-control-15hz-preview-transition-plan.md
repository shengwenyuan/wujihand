# 017：ROS2—Isaac 120/30/15 正式调度与高质量 Preview 迁移计划

- 状态：基础 120/30/15 已实施；正式采集、Tracker 遮挡恢复和 bundle 闭环已通过，多视角 operator UI 待实施
- 日期：2026-08-13
- 基线：`2953abb`，已清理过渡链路的 T-frame + NERO gripper flange collision proxy + Hand2 Beta1 v2026.8.3
- 上游：[014：Dual NERO T 型架 Isaac + ROS2 Record 并列场景开发计划](014-dual-nero-t-frame-isaac-ros2-record-scene-development-plan.md)
- 历史资格基线：[010：确定性 ROS2—Isaac—GUI 端到端 Qualification 计划](010-deterministic-ros-isaac-gui-e2e-qualification-plan.md)

## 1. 决策

新的正式仿真遥操作与采集入口采用以下调度标准：

| 链路 | 新标准 | 约束 |
|---|---:|---|
| PhysX | 120 Hz | 不放宽；固定 `dt=1/120 s` |
| control / IK / q54 action | 30 Hz | 每个 control tick 执行 4 个连续 physics step |
| operator preview | 15 Hz | 独立、被动、高质量、零 missed period |
| scene + 双腕 D405 数据图像 | 30 Hz | 保持现有分辨率、相位和数据身份 |
| ROS 输入源 | 保持现状 | Tracker/Glove 采样、身份、QoS 与最新值 mailbox 不因 control 降频 |
| MCAP / replay / quality / bundle | 保持现有契约 | topic、schema、q54、校验和及因果边界不变，频率事实改为真实 30 Hz |

每个 30 Hz control tick 在开始时选取最新合格输入，完成一次双臂 IK、双手 retarget 与安全决策，
将同一个 q54 target 保持到随后 4 个 120 Hz physics step。不得在中间伪造第二个 control action，
也不得用重复行把 30 Hz 录制伪装成 60 Hz。

本计划是单向断代迁移。主线不再保留 `120/60/20` Deployment、profile、loader 分支或回放兼容；
新入口使用新的 ID/hash，防止旧 episode 被误认为新标准数据。旧数据退出训练、性能基线和当前
版本验收范围，只保留既有产物与 Git 历史用于问题追溯；确需复现时 checkout `2953abb`。

## 2. 取舍依据

当前 T-frame + 双 NERO + 双 Hand2 + 动态物体链路中，两个 PhysX step 已接近 60 Hz control
周期的主要预算。按现有观测粗略外推，4 个 step 约占 26 ms，叠加输入选择、IK、q54 应用、
truth 提取和录制入队后约为 29–31 ms；30 Hz 提供 33.33 ms 周期，具有小但可验证的余量。

因此本计划明确让出 60 Hz control，不让出 120 Hz physics。4–8 个静置或 sleeping 物体通常只
增加有限开销；多物体持续接触、凹腔物体、反复撞击和双手同时操作是必须单独覆盖的压力场景，
不能用静态场景结果替代。

15 Hz preview 的周期为 66.67 ms。释放出的 GPU/CPU 预算用于更清晰的操作者画面，而不是回流
到主 control 进程。preview 仍是 `visual_replay_only`，不拥有 PhysX、控制权或数据集事实。当前
20 Hz proxy 实测虽达到 `19.99 Hz`，但出现 1 个 missed period，最慢帧为 `82.85 ms`，因此不得
一次叠加降频、提分辨率、光照和抗锯齿变更。

## 3. 不变边界

以下内容不随调度迁移改变：

- PhysX 频率、碰撞材料、动态刚体语义、gripper flange collision proxy 和 Hand2 自碰撞策略；
- NERO/Hand2 q7 + q20 定义、关节限位、Lula 模型、左右映射和 Tracker→T-frame 标定；
- ROS topic、消息 schema、QoS、producer identity、sequence、transport epoch 和 fail-closed 行为；
- D405 的 30 Hz、640×480 数据规格、CameraInfo/TF、pre-action 因果相位及离线渲染隔离；
- q54、14-link truth、动态物体 truth、MCAP 校验和、normalizer、replay、quality 和 bundle 责任边界；
- control schedule miss 保留完整 slot/miss mask 后进入 ABCD 质量分级并产生 warning，不自动拒绝
  episode；slot 与 miss mask 不一致、host time 倒退或重复仍是完整性硬拒绝；
- operator preview 的光照、背景和渲染覆盖不得写回共享 Workcell，不得改变 D405 或离线数据图像。

seconds-based stale、hold 和输入年龄阈值默认保持原值。所有以 tick 数表达但实际代表时间的窗口、
rate limiter、IK 连续失败计数和资格 fixture 必须逐项审计；只在保持原时间语义时换算，禁止统一
机械除以二。

## 4. 版本化调度事实

### 4.1 单一来源

新正式 Session 引用一个新的 mini-dataset timing profile，冻结：

```yaml
timing:
  physics_hz: 120
  control_hz: 30
  gui_preview_hz: 15
  policy_fps: 30
  selection: relative_all_control_index_no_interpolation_v1
  observation_phase: pre_action
```

主 runner、preview、manifest、receipt、validator、normalizer 和 release gate 都从该已解析 profile
读取频率，不再各自维护互相独立的 `60`、`20` 常量。主线 loader 仅接受
`120/30/15/30` 契约，不开放任意频率组合，也不为旧调度保留兼容分支。

### 4.2 断代入口替换

以当前 gripper-flange collision-proxy 生产入口为几何与控制基线，直接建立新的正式版本：

- 30 Hz mini-dataset profile；
- 120/30/15 Session；
- 对应 ROS Deployment 与 record-chain qualification；
- 对应 fixture/preview validator 身份。

新入口闭合后删除旧 Session、Deployment、timing profile、quality gate 及只验证旧调度的测试，
主线只保留一个正式采集入口。实施过程中不得复制遥操作控制器或 ROS route plan，只替换调度
事实与数据派生语义。

## 5. 主控制链改造

1. `CONTROL_HZ` 从 resolved timing 读取为 30；`PHYSICS_SUBSTEPS_PER_CONTROL` 由
   `120 / 30` 得到 4。
2. 每 tick 严格执行 4 个连续、可追溯的 physics step；receipt 与 MCAP 保存四个 substep index、
   simulation time 和 host timing。
3. 双臂 IK、双手 retarget、安全监督、q54 apply 与 post-action truth 每 control tick 各执行一次。
4. operator-preview state 每 2 个 control tick 发布一次，得到 15 Hz；该 topic 继续不写入 MCAP。
5. 录制构建与 ROS publish 保持异步有界队列，主循环内不得加入图像渲染、压缩或磁盘 I/O。
6. 所有依赖固定 `1/60 s` 的速度限制、滤波、hold、settle、控制时间戳和错误累计改用解析出的
   `control_dt=1/30 s`；已经使用真实 elapsed time 的逻辑不重复改写。
7. manifest 和 receipt 必须明确记录 `120/30/15`、每 tick 四个 substep、CPU affinity、stage timing
   与 missed-period counter，禁止沿用带 `60hz`/`20hz` 含义的 Gate 名称。

## 6. D405、录制与数据派生

### 6.1 D405 保持 30 Hz

现有 D405 profile 已声明 30 Hz 和每帧 4 个 physics substep。迁移后：

- `physics_substeps_per_capture` 仍为 4；
- `control_ticks_per_capture` 从旧入口的 2 变为新入口的 1；
- scene RGB、左右 D405 RGB 每个完整 30 Hz control tick 对应一帧 pre-action observation；
- 分辨率、投影、CameraInfo、TF、编码和 checksum 不变；
- 在线 operator preview 的质量设置不得影响上述数据图像。

camera loader 应根据 `capture_hz`、`control_hz` 和 `physics_hz` 校验调度关系，而不是硬编码旧值
`control_ticks_per_capture == 2`；当前唯一合法结果为 `control_ticks_per_capture == 1`。

### 6.2 raw 与 policy grid

新 raw MCAP 的 q54/control transition 本身就是 30 Hz；LeRobot policy 也是 30 fps，因此使用
`relative_all_control_index_no_interpolation_v1` 一一映射，不再做 relative-even 抽取。必须同步：

- mini-dataset profile loader；
- alignment manifest 和 artifact builder；
- normalizer、replay、quality、release 和 bundle；
- expected control rate、schedule gap、最大 interval 与质量分级阈值；
- 删除 `relative_even_control_index_no_interpolation_v1` 分支及其专用测试。

任何新派生数据都必须保留新 profile/hash。旧 60→30 artifact 不进入当前 loader、collection 或
release gate 的输入范围。

## 7. 高质量 operator preview

preview 继续使用独立进程和独立 CPU affinity，目标从 20 Hz 改为 15 Hz。视觉升级按以下顺序
逐级验证，每一级都必须先达到 15 Hz 零 missed period，再进入下一级：

1. 保持当前分辨率和渲染质量，仅把 preview 调度改为 15 Hz；
2. 提高背景灰度并增加 preview-local 柔和环境光，继续关闭重阴影和环境遮蔽；
3. 将分辨率提高到 800×500；
4. 开启不引入运动模糊或明显历史拖影的稳定抗锯齿。

最终资格候选为：

- 分辨率从 480×300 提高到 800×500；
- 开启稳定的抗锯齿，优先使用不引入运动模糊或明显历史拖影的方案；
- 背景首选 `[0.50, 0.50, 0.50]`，在操作者资格检查后冻结最终值；
- 增加 preview-local 的柔和环境光，使黑色 Hand2 仍有表面层次；
- 关闭重阴影和环境遮蔽；若残余阴影仍遮挡手指，preview 内完全关闭阴影；
- 不更改共享 HDR、Workcell 灯光、D405 renderer 或离线 RGB renderer；
- 不开启 preview 本地 physics、articulation drive、控制 callback 或 MCAP writer。

800×500 全 Gate 通过后，才允许在同一视觉 profile 下试验 960×600；最终只冻结一个通过资格的
分辨率。画质验收至少包含：双手轮廓清晰、拇指可见、对指间隙可辨、黑手不丢失立体感、T-frame
与 NERO 边界可分辨，并且 A→B→A 相同状态仍能确定性回放。

### 7.1 T-frame 多视角 operator UI

主操作视角不改为 top view，继续沿用第三路 scene D435 的斜向姿态和观察角度，重点覆盖操作桌面。
D435 的安装位置迁到 AgileX T-frame 粉色相机连接件附近；最终位置必须依据该连接件的 USD/CAD
安装面和 GUI 覆盖范围可视化标定后，冻结为命名 Workcell frame。UI 主视角与 D435 共用该版本化
位姿，但允许保持独立的 preview 分辨率、光照和渲染质量。D435 外参变化必须更新对应
Workcell/camera provenance，并重新关闭离线 replay 的图像身份与覆盖 Gate。

同一被动 preview 进程提供四个画面，不创建额外 physics 或控制入口：

- 主画面：D435 同姿态的操作桌视角，`800×500 @ 15 Hz`，保持当前高质量 preview；
- 左、右手画面：分别锁定到两侧 NERO 末端 `gripper_flange` 的 preview-only 相机，使用显式完整
  旋转冻结画面 roll，保证手指始终指向画面上方；
- 全景画面：保留当前初始化斜向视角，同时观察 T-frame、双 NERO、双 Hand2 和操作桌。

四路共享同一份已注入场景状态和 USD 资产，每路使用独立 camera/render product。第一版只按
分辨率和刷新率分级，继续共享 Minimal Rendering、抗锯齿、柔和环境光以及关闭阴影/AO 的策略；
手部画面建议从 `400×300 @ 15 Hz` 起步，全景从 `480×300 @ 5 Hz` 或按需刷新起步。未到刷新周期
的 render product 必须暂停更新并复用上一帧，不能让四路在每个 Kit tick 无条件渲染。

多视角验收仍以主画面 `15 Hz ±5%`、零 missed period、总 render transaction p99 不高于
`55 ms`、单次硬上限低于 `66.67 ms` 为准；同时要求 control/IK `30 Hz`、physics `120 Hz`、q54、
录制和三路离线数据图像不退化。若预算不足，优先降低全景刷新率，其次降低手部画面分辨率，不降低
主画面刷新率，也不改变 physics/control 标准。

## 8. 验收 Gate

| Gate | 要求 |
|---|---|
| 配置与迁移 | 新入口 resolve；旧 120/60/20 配置和分支退出主线；频率来自单一 profile |
| Physics | 120 Hz 网格；每个 control tick 恰好 4 个连续 substep；无缺步、重步或时间倒退 |
| Control | 30 Hz ±2% 为工程目标；可追溯 missed period 进入 ABCD 与 warning；IK、q54、反馈和输入 provenance 完整 |
| Control budget | 33.33 ms 周期内闭合；记录各 stage max/p95/p99；工程目标 p99 不高于 31 ms |
| Preview | 15 Hz ±5%；missed period 为 0；被动、无 physics、无控制权、无 MCAP 写入 |
| Preview budget | 单帧硬上限低于 66.67 ms，工程目标 p99 不高于 55 ms；分辨率与 AA readback 一致 |
| 视觉 | 背景/环境光仅 preview 生效；重阴影和 AO 关闭；手指、拇指和对指动作清晰 |
| D405 | scene + 双腕图像保持 30 Hz、640×480、pre-action；K/TF/frame identity 与 checksum 闭合 |
| Recording | q54/raw control 为真实 30 Hz；每 tick 四个 physics truth；topic/schema/有序关闭不退化 |
| 数据闭环 | 新 30→30 数据完成 normalize → replay → quality → bundle；旧 artifact 不被当前入口接受 |
| 旋转与碰撞 | gripper flange proxy 无 link5/link6 穿透；J5/J6/J7、手基座和 q54 无突跳或异常旋转 |

正式 release 遵循完整性/质量分离：control schedule miss 本身不构成硬拒绝；slot/miss mask 不闭合、
host time 顺序损坏、physics 网格损坏等不可恢复的因果错误才硬拒绝。preview 零 missed period 继续作为
独立的操作者显示验收目标；control 的 p95/p99、miss fraction、最大连续 miss 和最大 interval 进入
ABCD 分级与 warning。

## 9. 场景压力矩阵

按相同 120/30/15 入口逐级验证：

1. T-frame + 双机器人，无任务物体；
2. 当前 banana + bowl 两个动态物体；
3. 4–8 个静置或 sleeping 物体；
4. 4–8 个 awake、分散但无持续接触的物体；
5. 笔筒/容器凹腔内多点持续接触；
6. 反复撞击、物体滚落和重新抓取；
7. 双手同时接触不同物体，以及双手协同操作同一物体。

每一级都保存 stage timing、contact 数量、awake body 数量、control/preview miss、q54 连续性和物体
稳定性。若压力场景超过预算，优先优化 collision proxy、sleeping、场景资产和非控制工作；不得降低
120 Hz physics，也不得隐藏或修饰 schedule miss、slot/mask 和时间顺序事实。

## 10. 实施顺序

1. 第一阶段建立唯一的 120/30/15 timing 与 dataset profile，直接把 camera schedule、alignment
   selection 和 release gate 切换到新契约；删除旧配置和旧分支，先让 resolver、manifest 与静态
   测试闭合，不修改控制器和运行时循环。
2. 主 runner 与 preview 改为读取 resolved timing，保持控制器、route plan 和 ROS 输入链不变。
3. 同步 D405 schedule、30→30 alignment、normalizer、release 与 bundle 的版本语义。
4. 完成 headless 120/30 fixture，确认四 substep、schedule 事实闭合、q54 连续和无异常旋转；零
   control miss 是工程目标，少量可追溯 miss 只降低质量等级。
5. 接入 15 Hz 高质量 preview，冻结分辨率、AA、背景、环境光和阴影策略。
6. 完成 ROS A→B→A、recording、replay、quality、bundle 端到端资格验证。
7. 执行场景压力矩阵；通过后再做单侧和双侧真实穿戴输入验证。
8. 连续获得可复现的正式回执后，将新 Deployment 提升为唯一推荐入口，不重新引入旧调度。

首个开发停靠点要求：主线只剩新 profile 和新入口，新入口可静态 resolve，旧调度专用分支已删除；
随后 headless fixture 必须达到 120 Hz physics、30 Hz control、每 tick 4 个 physics substep、完整
slot/miss mask 与单调时间顺序，并完成 normalize → replay → bundle。该停靠点通过后才接入 15 Hz
高质量 preview；preview 自身仍须达到零 missed period。

控制降频预计涉及约 50–100 行生产代码、15–30 行配置和 80–150 行测试调整。多数是频率、调度
事实和枚举的直接替换；主要的小逻辑改动集中在 D405 的 `2→1` capture schedule、移除 2:1
下采样，以及 alignment/release 对 30→30 映射的处理。高质量 preview 的视觉调整另计。

## 11. 完成定义

只有同时满足以下条件，120/30/15 才可成为正式标准：

- 新入口连续通过 120 Hz physics 网格和 control 调度完整性 Gate；control miss 如实进入 ABCD 与
  warning，15 Hz preview 连续通过零 miss Gate；
- 双侧 tracker/glove 输入、IK、Hand2 对指、gripper flange collision 和 q54 均无异常旋转或突跳；
- D405/recording 的 30 Hz、图像身份、因果相位和 checksum 未发生退化；
- normalize、replay、quality、bundle 对新 30 Hz 原始控制完成闭环；
- 至少覆盖当前两物体场景及 4–8 物体、多接触、双手操作压力场景；
- 当前训练 collection、release gate 和推荐入口均不再接收 120/60/20 数据；
- operator preview 画质经人工确认，且其覆盖未改变共享灯光或任何数据图像。

## 12. 2026-08-14 实施与验收记录

T-frame 两侧 NERO base 朝向已按实际装配观察修正，并同步冻结新的悬垂 L 型初始 q7；该矫正属于
Workcell mount 与 startup profile 的版本化场景事实，不通过临时关节补偿或 runner 分支实现。

正式 episode `tframe-120-30-15-live-20260814-134233` 完成 1319 个 control tick：physics RTF
`0.992`，control `29.749 Hz`，11 个可追溯 schedule miss、最大连续 miss 为 1；preview
`15.008 Hz`、零 miss，render p99 `26.06 ms`。control tick p99 `33.14 ms`，因此完整性与正式
release 通过，但尚未达到本计划 `p99 <= 31 ms` 的工程余量目标。

操作者用戴手套的手抬起 Tracker 制造多段遮挡。所有 q7 缺失区间内对应 NERO arm target 保持量为
零，信号恢复后的 target 单步不超过 `0.028 rad`，未出现末端旋转或失控；该诊断行为使 episode
质量降为 D，不应解释为普通连续遥操作训练质量。nominal replay 生成三路各 1319 帧，共 3957 张
唯一图像；normalize、release、30→30 alignment、quality 与 bundle checksum 全部闭合。该 run
保留为 candidate 恢复诊断数据，不直接计入正式训练 episode。
