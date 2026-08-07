# 011：Mini 数据集完整性门禁与质量分级更新计划

- 状态：已实施并用 02/03/04 完成 replay、12-episode 导出与全量回读验收
- 日期：2026-08-06
- 上游：[008](008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)、[ADR-0011](decisions/0011-mini-dataset-causal-artifact-boundary.md)
- 现场基线：`banana_in_bowl-20260806-02`、`banana_in_bowl-20260806-03`、
  `banana_in_bowl-20260806-04`

## 1. 已对齐结论

数据集是否允许保留、对齐、replay 和导出，只由以下根本问题决定：

1. 必需的事实信号是否真实存在或能从不可变事实唯一恢复；
2. episode 边界以及事实的因果时间顺序是否能唯一恢复；
3. artifact 是否可读取、可校验且不存在无法消解的相互矛盾。

只有事实缺失、损坏或时间顺序不可恢复才自动拒绝。采样率偏差、scheduler miss、输入 age、
低置信度、clamp、hold、RTF、预览帧率和抖动用于说明“数据有多干净”，默认属于软质量指标，
不能仅凭阈值越界拒绝一个事实完整的 episode。

本结论替代 008 中“出现任意 schedule miss 即 reject 并重录”和“60/20 Hz 未严格达标即拒绝”
的方向。008 的不可变原始事实、q54 顺序、动作前视觉、checksum 和可追溯边界继续有效。

本计划以 02、03、04 三个现场包均能通过完整性判定为回归基线。这里的“通过”只表示数据事实
闭环并允许进入数据集，不表示质量达到 A，也不表示任务成功。三包当前控制部分均应得到
`control_complete`；完成三视角确定性 replay 并闭合图像检查后，才得到 `bundle_complete` 和
`release_eligible=true`。

## 2. 两层判定模型

### 2.1 硬完整性判定

硬判定只回答：这个 episode 能否形成无歧义、可追溯的 observation/action/vision 时间序列。

以下情况 fail closed：

- MCAP、manifest、alignment、vision 或必要 sidecar 无法读取或 checksum 不闭合；
- 必需 q54 observation/action 或三路目标 RGB 在某个 policy row 永久缺失；
- episode 起止边界无法定位；
- sequence/control/physics/frame identity 无法建立唯一递增顺序；
- 相同身份对应不同 payload，且没有更高优先级事实可消歧；
- q54 维度、单位或顺序未知，或值非 finite；
- 图像 payload 无法解码，且不能由相同 source state 确定性 replay；
- pre-action state、action、图像之间无法唯一追溯到同一 source tick；
- writer 未 finalize，导致 Parquet/MP4/metadata 无法完整重开。

实现使用以下固定硬门槛；它们属于版本化 gate profile，不作为启动参数暴露：

| 项目 | 硬门槛 |
|---|---:|
| MCAP、manifest、receipt、派生 artifact | 可读取、正常关闭且 checksum 闭合 |
| lifecycle | 唯一完成 `opened -> ready -> recording -> stop_requested -> closed` |
| q54 | 维度 54、finite，且每个实际 control tick 的 pre/post/applied 事实完整 |
| q54 数值闭环误差 | `<= 1e-8 rad` |
| control 权威索引 | 实际 tick 索引连续且不倒退；schedule slot 跳变必须与显式 miss mask 一致 |
| physics/frame 权威索引 | 严格递增；不允许同一 identity 对应冲突 payload |
| physics/control | 每个实际 control tick 严格 `2:1` |
| physics 整数格点与浮点时间误差 | `<= 5 us` |
| replay 连杆位置误差 | `< 2e-5 m`，超限不保存图像 |
| policy 输出 | `30 Hz`；每个保留 row 的 q54 state/action 和三路 RGB 完整 |
| RGB | 可解码、相机 identity 唯一、reference 递增；允许离线确定性 replay |
| writer/finalizer | 完成原子发布后能够 reopen、seek 并复核 checksum |

完整性规则不得硬编码香蕉、碗等任务资产。只有被场景 manifest 明确声明为静态参考的几何体才
检查漂移；当前场景可将桌面列为静态参考，默认平移容差为 `1e-6 m`、旋转容差为
`1e-6 rad`。动态物体以及未声明为静态参考的物体不参加该硬门槛。

raw tracker/glove reference 的规则为：非空引用必须唯一解析；空引用本身是合法事实，只要 null、
sample-and-hold/safety 决策和最终 applied q54 已被明确记录。q7 candidate、q20 新 intent 或 active
provenance 的缺失因此属于因果审计完整度问题，不得覆盖已经落盘的 applied q54，也不自动拒绝
episode。相同 identity、相同 payload 的 duplicate 可以确定性去重并写 recovery log；相同
identity、不同 payload 必须硬失败。

允许先做确定性恢复再判定：

- 文件到达顺序混乱，但 sequence ID 和时间身份足以唯一排序；
- 在线图像遗漏，但完整 q54/link truth 和 renderer 配置允许同状态确定性 replay；
- sample-and-hold tick 没有新 raw input，但 held/applied 事实和采用该结果的原因已明确记录；
- 浮点 simulation time 有微小累计误差，但整数 physics boundary 和固定 origin 完整；
- LeRobot float32 timestamp 与双精度 `frame_index / fps` 存在正常量化误差。

任何恢复都必须输出方法、原值、恢复值、受影响范围和 checksum；禁止静默填充。

### 2.2 软质量评估

以下项目默认生成数值、分布和 warning，不改变完整性结论：

- control/render/physics 的有效频率、jitter、lateness 和 missed periods；
- physics RTF；
- tracker/glove 输入 age、confidence、sample-and-hold 长度；
- clamp、rate limit、低运动幅度和静止比例；
- callback 延迟，但最终事实身份仍唯一；
- 图像清晰度、亮度、遮挡、运动模糊和重复帧比例；
- q54、qdot、末端和物体轨迹的平滑度；
- 接触、抓取、靠近或放置趋势；
- 操作者是否完成预期任务。

质量报告可以给出 grade、warning 和训练筛选建议，但不得把质量 grade 偷换为完整性状态或任务
success。训练者可按自己的用途选择阈值。

质量等级固定为 A/B/C/D，单个 episode 取所有适用指标中的最差等级。C/D 只表示降级和需要
人工关注，不改变 `integrity_status`，也不单独阻止 registry 接纳或 artifact 发布。

| 指标 | A | B | C | D |
|---|---:|---:|---:|---:|
| control 有效频率 | `>=59.5 Hz` | `>=58 Hz` | `>=55 Hz` | `<55 Hz` |
| physics real-time factor | `>=0.99` | `>=0.97` | `>=0.90` | `<0.90` |
| scheduler 漏周期率 | `<=0.5%` | `<=2%` | `<=3%` | `>3%` |
| 最大连续漏周期 | `<=1` | `2` | `3--5` | `>5` |
| 最大 control 间隔 | `<50 ms` | `<100 ms` | `<250 ms` | `>=250 ms` |
| 各路 selected 缺失率 | `<=1%` | `<=5%` | `<=15%` | `>15%` |
| 各路 active provenance 缺失率 | `<=1%` | `<=5%` | `<=15%` | `>15%` |
| 各臂 q7 candidate 缺失率 | `<=1%` | `<=5%` | `<=25%` | `>25%` |
| 四路同时 actionable 比例 | `>=95%` | `>=80%` | `>=70%` | `<70%` |
| 输入 age p95 | `<=20 ms` | `<=50 ms` | `<=100 ms` | `>100 ms` |
| critical-motion gap / candidate tick | `0` | `<=0.5%` | `<=3%` | `>3%` |
| rate-limit 比例 | `<=5%` | `<=15%` | `<=30%` | `>30%` |
| clamp 比例 | `<=1%` | `<=5%` | `<=15%` | `>15%` |

各行从 A 到 D 顺序判断并采用首个命中的等级，因此各列阈值不存在重叠解释。例如漏周期率
`0.3%` 为 A、`1.2%` 为 B、`2.5%` 为 C、`3.1%` 为 D。

所有比例按各自可观测分母计算，并同时保存 numerator、denominator，禁止只落百分比。输入 age 的
最大值、超过 50/100/250 ms 的计数和发生时刻继续报告，但不以单个孤立最大值决定等级。GUI
preview 20 Hz、物体运动幅度和任务完成情况只作诊断，不参加完整性或 ABCD 汇总。RGB 模糊度、
曝光、遮挡和重复帧先记录原始统计；glove confidence 同样先保留分布和低置信度计数。在三包
replay 和完整穿戴样本建立经验分布前，不为这些指标设置缺乏基线的等级阈值。

## 3. Episode 与断点规则

1. scheduler miss 不自动形成 episode 断点。
2. 事实序列仍连续时，保留一个完整 episode，并记录 miss 发生位置和持续时间。
3. 发生显式 reset、控制 epoch 切换或真实设备重启时，才建立语义边界。
4. 若真实缺失区间使前后因果关系不可恢复，原始 run 保留为 `incomplete/truncated`，不伪装成
   两个正常 episode。
5. 只有前后两段各自具有真实 lifecycle 起点、完整初始 observation 和独立任务语义时，才允许
   切分为两个 episode。
6. 不得通过规则时间戳、插值或复制动作掩盖真实事实缺失。
7. 30 Hz policy row 继续连续编号并保留全部可用 observation、action 和图像；相邻 policy row
   对应的 source 区间只要跨过 scheduler miss，就将起始 row 的 `transition_valid=false`。
8. 每个 policy row 至少保存 `gap_before_row`、`missed_control_periods` 和 `transition_valid`。
   `transition_valid` 明确表示“从本 row 到下一 row 是否可作为训练 transition”；末行恒为 false。
9. 无效 transition 不进入训练采样索引，但不删除 row、不拆 episode，也不伪造中间状态。
10. policy 的兼容时间仍为 `frame_index / 30`；真实经过时间和断点持续时间由 source timestamp、
    schedule slot 和 continuity sidecar 保存。由此兼顾 LeRobot 固定帧率与真实时间可追溯性。

建议将 disposition 与质量拆开保存：

| 字段 | 示例 | 含义 |
|---|---|---|
| `integrity_status` | complete / incomplete / unrecoverable | 是否可形成完整因果序列 |
| `artifact_stage` | control_complete / vision_pending / bundle_complete | 派生流程完成阶段 |
| `release_eligible` | true / false | 仅由完整性和 bundle 是否闭合决定 |
| `quality_grade` | A / B / C / D | 数据干净程度，不参与硬拒绝 |
| `quality_warnings` | schedule_miss、high_input_age | 软指标越界明细 |
| `recovery_log` | timing_snap、offline_replay | 确定性恢复记录 |
| `task_annotation` | banana grasp/place attempt | 任务描述，不是 success 标签 |

## 4. 时间与 LeRobot 导出更新

### 4.1 权威时间

按以下优先级保存和验证：

1. `dataset_frame_index`、`source_control_index`、`physics_boundary_index`；
2. 原始 `timestamp_ns` 和固定 physics origin；
3. `timestamp = frame_index / 30`，仅为 LeRobot 兼容派生值。

禁止用 float timestamp 反向决定事实顺序。float32 timestamp 的验证必须比较：

    stored_timestamp == float32(frame_index / fps)

或使用与该 float32 数值等价的 ULP-aware 判定。不能用固定 1 µs 与双精度值比较，因为 episode
变长后 float32 的正常量化误差会超过 1 µs。

### 4.2 完整 episode 导出

- 不因 float32 量化误差切段；
- `frame_index` 必须为 `0..N-1` 且严格递增；
- 三路视频各有 N 帧，帧率和 episode offset 闭合；
- q54 state/action、task、三路图像可按任意首/中/末索引回读；
- source sidecar 保留 source tick、simulation time、state digest、miss 和 continuity 信息；
- dataset row 和训练索引保留断点 mask，训练索引不得包含 `transition_valid=false` 的 transition；
- 完整解码、随机 seek、checksum 和原始 artifact 不变性全部通过后才原子发布。

## 5. 三个现场基线的预期判定

| Episode | 控制完整性 | 预期质量 | 当前 artifact 阶段 | 主要质量提示 |
|---|---|---:|---|---|
| `banana_in_bowl-20260806-02` | complete | B | vision_pending | 少量 scheduler miss 和 provenance 空缺 |
| `banana_in_bowl-20260806-03` | complete | C | vision_pending | 漏周期约 2.48%，较多断点处于物体运动期 |
| `banana_in_bowl-20260806-04` | complete | C | vision_pending | 右侧 provenance/actionability 较弱 |

这三包是新版 gate 的强制回归样本：实现完成后必须全部通过 control integrity。三包仍须分别完成
三视角离线 replay、逐 row identity 校验、完整视频解码和 checksum 闭环，才能把
`artifact_stage` 更新为 `bundle_complete`。若 replay 暴露真正不可恢复的图像或状态事实缺失，仍按
硬门槛失败，不能为了满足基线而放宽事实完整性。

## 6. 下一轮实施范围

1. 将 release validator 拆为 `integrity validator` 与 `quality evaluator`。
2. 建立版本化 gate profile，固化本文硬门槛与 ABCD 超参数，不增加 deployment 启动参数。
3. 把旧 hard Gate 逐项迁移到 integrity、quality 或 operator policy，禁止重复判定。
4. 为确定性恢复建立 versioned `recovery_log` schema。
5. 将 alignment 和 LeRobot 时间闭环改为整数身份优先、float32 同量化验证。
6. 在 alignment/export 中生成 gap 字段和训练 transition 索引，确保无效 transition 不被采样。
7. registry 仅因 integrity fail 或 bundle 未闭合而拒绝发布；quality grade 不参与拒绝。
8. dataset card 同时呈现完整性、artifact stage、ABCD 指标、warning 和恢复记录。
9. 为既有 episode 提供只读 re-evaluation，不改写 raw MCAP。
10. 用 02、03、04 做固定回归；再用合成缺 q54、冲突 payload 和不可解码 RGB 做硬失败反例。

## 7. 必测场景

- 有 scheduler miss 但所有 tick 身份连续：完整性通过并产生 warning；
- 58～60 Hz 波动但整数顺序闭合：完整性通过；
- input age 或 glove confidence 越界但 applied q54 明确：完整性通过；
- selected/active raw reference 为空，但 hold/safety 和 applied q54 明确：完整性通过并产生质量提示；
- 非空 reference 无法解析或同 identity 出现冲突 payload：完整性失败；
- scheduler miss 跨越相邻 policy row：episode 保留，相关 `transition_valid=false` 且训练索引排除；
- 中间 q54 或不可 replay 的 RGB 永久缺失：完整性失败；
- 两个不同 payload 共享同一权威 identity：完整性失败；
- 文件乱序但 sequence ID 唯一：重排后通过并记录恢复；
- 超过 32 秒和超过 120 秒的 30 Hz episode：float32 timestamp round-trip 通过；
- 4099 帧、三路 MP4 的完整 export/finalize/reopen/seek/checksum 通过；
- 场景声明桌面为静态参考：桌面漂移受检；未声明的碗、香蕉或其他任务资产不受该门槛约束；
- 02、03、04：控制完整性全部通过，质量等级分别稳定为 B、C、C；
- 完成全部派生后，原始 MCAP、manifest 和 receipt checksum 不变。

## 8. 本轮非目标

- 不修改 release、alignment、renderer 或 LeRobot exporter 主线代码；
- 不把临时 monkey patch、脚本或复制实现提交到仓库；
- 不回写原始 MCAP；
- 不进行 Pi0.5 训练或模型验收。

## 9. 实施与验收记录（2026-08-06）

- 完整性 gate 与 ABCD 质量评估已经拆分并由版本化 profile 固化；02、03、04 均通过硬完整性，质量等级稳定为 B、C、C。
- alignment 已保留 gap/missing 信息并生成 `transition_valid`；LeRobot 仅在训练 transition sidecar 中列出有效 transition，不删除原始 policy row。
- 三条 source episode 均完成 nominal replay，并各生成 warm/bright、cool/dim、neutral/high-key 三个仅视觉域随机化版本；q54、physics、相机位姿、物体状态和时间轴不随机化。
- 12 个 episode 共 14,552 个 30 Hz policy row、14,100 个有效训练 transition；三路 RGB、q54 state/action、source sidecar、质量等级和 visual variant provenance 全部闭合。
- LeRobot v0.6.1 固定 commit `7e241bd630a3719a56157a497ce5d08f244784f1` 已完成 finalize、全量 reopen/decode、随机访问与 checksum 验收。
- LeRobot row timestamp 继续按 `float32(frame_index / 30)` 位级比较；视频 seek 单独使用 `100 us` 容差覆盖长视频 float32 PTS 的正常 ULP，仍远小于一个 30 Hz 帧间隔，不允许选取相邻帧。
- 发布 revision 位于 Workstation2：`artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/revisions/banana_in_bowl-20260806-12ep-v1`。
