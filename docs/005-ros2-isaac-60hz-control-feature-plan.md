# 005：NV-5.1 ROS2—Isaac 60 Hz 控制迷你版本计划

- 状态：Headless Gate 已通过；GUI 20 Hz 的 10 s Gate 通过，30 s 仅零漏周期 Gate 未通过；
  真实设备重采集待执行
- 基线：`banana-grasp-pilot-20260803-02`
- 范围：只处理 ROS callback、Isaac physics/control/render 调度与对应录制事实

## 1. 问题与目标

Pilot -02 中原始 Tracker 为 `120 Hz`，左 Glove 为 `115.4 Hz`，但 Isaac tick 仅
`48.8 Hz`；每一路新 Tracker/Glove 样本实际进入控制器约 `9.3～9.5 Hz`，active sample
age P95 约 `87 ms`。根因是每个 Isaac tick 只执行一次非阻塞 `spin_once()`，五类 callback
轮流得到一次调度机会；GUI `world.step(render=True)` 又把控制和渲染绑在一起。

NV-5.1 冻结以下频率：

| 层 | 目标频率 |
|---|---:|
| Tracker / Glove 原始 ROS topic | 保留源频率，目标 120 Hz |
| Isaac physics | 120 Hz |
| q7/q20/q27 控制、feedback、tick、动态 scene state | 60 Hz |
| GUI 操作预览 | 20 Hz |

GUI 只服务操作者预览，不参与控制或数据集采样率定义。2026-08-04 Workstation2
实测证明同步 30 Hz GUI 会把 control 降至约 39 Hz，因此显式冻结为 20 Hz；physics 与
control 不降频。未来 Gemini/D435i 图像按设备时间戳独立录制，不使用 GUI render tick
代替相机帧率。

## 2. 架构要求

1. ROS 使用独立后台 `SingleThreadedExecutor.spin()`；callback 不调用 Isaac、不做 IK、
   retarget 或绘图，只校验并写 latest-only mailbox。
2. `LatestEpochInbox`、adapter pending epoch/reference flag、selection 和 counter 全部具备
   明确锁边界。Isaac 主线程每个 60 Hz tick 原子快照一次，不持锁执行控制或 physics。
3. 每个控制 target 连续执行两个 120 Hz physics step；GUI 每三个 control tick 独立刷新
   一次，render 不得推进 simulation time。GUI preview 明确关闭 Hydra `waitIdle`，允许画面
   最多滞后一帧；它不是相机数据或控制反馈，不能为等待 GPU 完成而阻塞 60 Hz 主循环。
4. Headless 调度超期即跳过；GUI 最多连续保留两个逾期 slot，以吸收同步 render 后跨越
   单个 deadline 的固定开销，但禁止无界追赶。平均 control 仍须达到 60 Hz，所有实际
   tick 时间继续逐条落盘。
5. Isaac 场景、retarget 和 ROS graph 完成初始化后，在首个 control tick 前执行一次 Python
   cyclic GC 并冻结当时的长期对象；control 窗口结束后恢复。不得让全代 GC 周期性扫描
   Isaac 长期对象并制造约 150 ms 的控制停顿。
6. 原始 input topic 继续独立按源速率录制。tick 只保存本次选中的 sequence/epoch/time，
   不把多速率原始数据在线重采样成 60 Hz。
7. ROS、Isaac 和 recorder 关闭顺序独立于本 feature；不得重新引入跨线程 Isaac 调用。
8. 主线只记录事实。频率、延迟、drop、抖动和图仍由离线 analyzer 计算。

## 3. 需要新增或确认的录制事实

- control tick 的 host monotonic time、simulation time、control index；
- 两个 physics substep 的 index/time，以及 target 生效区间；
- render 是否发生及 render index；
- callback 写入时间、tick 快照时间、所选 source sequence；
- executor/mailbox overflow、epoch change 和 superseded sample counter；
- physics/control/render 配置与实际 capability 写入 manifest。
- GUI `block_on_render` 实际值写入 manifest；正式 GUI run 必须为 `false`。
- Python GC policy 写入 manifest，实际 frozen object count 和关闭恢复状态写入 receipt。

若需要修改 `TeleoperationTickTrace`，必须版本化 schema；旧 bag analyzer 要明确拒绝或走
兼容读取，不能把缺失的 sim/substep 时间补零。

## 4. 实施切片

1. 为 inbox 与 ROS input adapter 增加线程安全和并发单元测试。
2. 把 executor 生命周期移出 Isaac tick，并验证 callback 线程只触达 transport adapter。
3. 引入显式 120/60/20 调度器，先 headless，再 GUI。
4. 扩展 tick timing 事实及 rosbag QoS/回放测试。
5. Workstation2 运行 synthetic 20～30 s、真实设备自由空间 pilot、最后运行抓放 pilot。

## 5. 验收 Gate

- control rate：有效频率 `60 Hz ±2%`；Headless P95 tick interval `<=20 ms`，GUI
  P95 `<=25 ms`；
- physics：每 control tick 恰好两个 substep，real-time factor `>=0.95`；
- GUI render：`20 Hz ±5%`，不得以降低 control rate 换取预览帧率；
- 每路 callback 接收速率与源频率一致，60 Hz tick 的新选择率接近 60 Hz；
- active sample age P95 `<20 ms`，不得再出现约 87 ms 的常态长尾；
- tick/sequence/epoch 无回退，q7+q20→q27 组合误差仍为 0；
- 录制关闭为 `complete`，包含 `started` 与 terminal status；
- 本地全套测试和 Workstation2 Jazzy/Isaac focused regression 通过。

Workstation2 的 Gate 验证与正式采集必须由本进程独占 Isaac/Vulkan 负载；并行 CAD/Isaac
预览产生的结果只用于诊断，不得作为频率验收依据。该限制不禁止非 GPU 的轻量后台任务。

只有上述 Gate 通过后，NV-5.1 才可作为正式三轮数据采集入口。

## 6. 2026-08-04 实施状态

- 已完成：后台 `SingleThreadedExecutor`、四路共享锁原子快照、线程安全 latest-only
  mailbox，以及 drained/discarded/overwritten/pending 守恒 counter。
- 已完成：120 Hz physics、60 Hz control、20 Hz GUI preview；Headless 超期 slot 显式跳过，
  GUI 最多连续双槽补偿且禁止无界追赶；每个 q7/q20/q27 target 恰好覆盖两个连续 physics
  substep。
- 已完成：新增独立 ROS 类型 `TeleoperationTickTraceV2` 和 schema `.v2`；原
  `TeleoperationTickTrace` 线布局保持不变，旧 MCAP 可继续真实反序列化。
- 已完成：离线 analyzer `0.2.1` 同时读取 v1/v2；v2 增加 scheduler、substep、simulation
  time、real-time factor、render cadence、mailbox 守恒 Gate 和第 13 张执行节拍图。
- 已完成：本地单元/契约/分析器测试及 Ruff、mypy；未触碰 camera mount CAD 文件。
- 已完成：Workstation2 隔离目录 Jazzy 重建与 30 s synthetic headless；离线 analyzer
  `0.2.0` 全部结构与性能 Gate 通过，control `60.000 Hz`、RTF `1.0003`、四路输入年龄
  P95 `<9 ms`、漏周期 `0`。
- 已完成：首 tick 前 `gc.collect()` + `gc.freeze()`，关闭时恢复；manifest/receipt 记录策略、
  frozen object count 和恢复结果。此前约 150 ms 的周期性 physics 停顿不再出现。
- 已完成：正确 RoboLab banana-bowl 场景的 10 s GUI synthetic 全 Gate 通过：control
  `59.9999 Hz`、render `20.0000 Hz`、RTF `1.0008`、漏周期 `0`、四路输入年龄 P95
  `9.23～9.36 ms`。
- 当前瓶颈：30 s GUI synthetic 的结构 Gate 全通过；control `59.2423 Hz`、P95
  `20.25 ms`、render `19.7472 Hz`、RTF `0.9877`、四路输入年龄 P95 `<9.5 ms`，但同步
  Kit `app.update()` 偶发约 78 ms 长阻塞，产生 `23/1800` 个 schedule miss。显式
  `block_on_render=false` 未消除该长尾；不再继续堆叠局部 renderer 开关。
- 待决策：保持“零 schedule miss”硬 Gate 时，下一步是把 GUI viewer 与 120/60 Hz
  simulation/control owner 解耦，属于独立 mini-feature；若接受非实时 Linux 的有界稀疏
  miss，则必须先另行冻结 miss ratio 与最大间隔 Gate，不能直接修改 analyzer 迁就本次结果。
- 停止点：device-free GUI 验证通过后，才需要操作者重新上电 Tracker/Glove 做自由空间与抓放
  pilot；目前不批准正式三轮采集。
