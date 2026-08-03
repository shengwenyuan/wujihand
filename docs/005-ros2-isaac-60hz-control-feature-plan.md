# 005：NV-5.1 ROS2—Isaac 60 Hz 控制迷你版本计划

- 状态：计划中；本轮只冻结需求，不修改控制循环
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
| GUI render | 30 Hz |

30 Hz 只作为性能验收失败后的显式 fallback，不作为默认目标。

## 2. 架构要求

1. ROS 使用独立后台 `SingleThreadedExecutor.spin()`；callback 不调用 Isaac、不做 IK、
   retarget 或绘图，只校验并写 latest-only mailbox。
2. `LatestEpochInbox`、adapter pending epoch/reference flag、selection 和 counter 全部具备
   明确锁边界。Isaac 主线程每个 60 Hz tick 原子快照一次，不持锁执行控制或 physics。
3. 每个控制 target 连续执行两个 120 Hz physics step；每两个控制 tick 渲染一次。
4. 原始 input topic 继续独立按源速率录制。tick 只保存本次选中的 sequence/epoch/time，
   不把多速率原始数据在线重采样成 60 Hz。
5. ROS、Isaac 和 recorder 关闭顺序独立于本 feature；不得重新引入跨线程 Isaac 调用。
6. 主线只记录事实。频率、延迟、drop、抖动和图仍由离线 analyzer 计算。

## 3. 需要新增或确认的录制事实

- control tick 的 host monotonic time、simulation time、control index；
- 两个 physics substep 的 index/time，以及 target 生效区间；
- render 是否发生及 render index；
- callback 写入时间、tick 快照时间、所选 source sequence；
- executor/mailbox overflow、epoch change 和 superseded sample counter；
- physics/control/render 配置与实际 capability 写入 manifest。

若需要修改 `TeleoperationTickTrace`，必须版本化 schema；旧 bag analyzer 要明确拒绝或走
兼容读取，不能把缺失的 sim/substep 时间补零。

## 4. 实施切片

1. 为 inbox 与 ROS input adapter 增加线程安全和并发单元测试。
2. 把 executor 生命周期移出 Isaac tick，并验证 callback 线程只触达 transport adapter。
3. 引入显式 120/60/30 调度器，先 headless，再 GUI。
4. 扩展 tick timing 事实及 rosbag QoS/回放测试。
5. Workstation2 运行 synthetic 20～30 s、真实设备自由空间 pilot、最后运行抓放 pilot。

## 5. 验收 Gate

- control rate：中位数 `60 Hz ±2%`，P95 tick interval `<=20 ms`；
- physics：每 control tick 恰好两个 substep，real-time factor `>=0.95`；
- render：`30 Hz ±5%`，降低画质/渲染频率优先于降低 control rate；
- 每路 callback 接收速率与源频率一致，60 Hz tick 的新选择率接近 60 Hz；
- active sample age P95 `<20 ms`，不得再出现约 87 ms 的常态长尾；
- tick/sequence/epoch 无回退，q7+q20→q27 组合误差仍为 0；
- 录制关闭为 `complete`，包含 `started` 与 terminal status；
- 本地全套测试和 Workstation2 Jazzy/Isaac focused regression 通过。

只有上述 Gate 通过后，NV-5.1 才可作为正式三轮数据采集入口。
