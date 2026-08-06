# 006：Isaac 60 Hz 到 LeRobot v3 的因果对齐与导出计划

- 状态：设计基线；导出器、数据相机与 episode lifecycle 尚未实现
- 上游控制计划：[005：NV-5.1 ROS2—Isaac 60 Hz 控制](005-ros2-isaac-60hz-control-feature-plan.md)
- 当前验证：[2026-08-04 NV-5.1 本地与 Workstation2 验证](validation/2026-08-04-nv51-ros2-isaac-60hz-local.md)
- 录制边界：[ADR-0009：ROS2 全因果链录制](decisions/0009-ros2-full-causal-recording-boundary.md)
- 目标格式：`LeRobotDataset v3.0`；实现前必须再固定准确的 LeRobot package 版本与 upstream commit

## 1. 决策摘要

本数据来自同一 Isaac 仿真进程，不采用真实多机传感器的 PTP 方案作为核心对齐手段。
正式 LeRobot 数据集冻结以下原则：

1. 以 Isaac `control_index/tick_id` 和 simulation time 为数据集主轴；host monotonic time
   只用于输入因果性、调度质量和运行审计，不写成 LeRobot episode timestamp。
2. LeRobot 全局 `fps=60`。第 `k` 行的 timestamp 由整数帧号计算为 `k / 60`，不从 rosbag
   到达时间、GUI render 时间或累计浮点 simulation time 反推。
3. 第 `k` 行严格表示一个 transition：动作前 observation `s_k`、本 tick 在同一 physics phase
   前向双侧提交的 target `a_k`，以及只用于审计的动作后状态 `s_{k+1}`。
4. `observation.state[k]` 使用左右 `pre_feedback_q27`；`action[k]` 使用左右
   `applied_target_q27`。两侧均按固定 joint-name 顺序拼成 q54。
5. 所有策略视觉必须在动作生效前、`simulation_time_before_s` 相位采集。当前 20 Hz GUI
   render 是操作预览，不是数据相机；动作后的 GUI 画面或 scene state 与 `action[k]` 同行会
   泄露该动作的结果。
6. raw Tracker、Glove、q7/q20 intent、post-step feedback、scene truth 和 safety/debug 字段
   保存在不可变源数据或审计 sidecar，不进入默认 policy-facing LeRobot features。
7. 核心 state/action/image 不做 nearest timestamp join、不插值、不 forward-fill、不删除坏帧后
   重编号。一个 episode 不能完整满足契约时，整体拒绝或保持为非正式诊断数据。
8. 初版正式数据只接受零 control schedule miss 的 headless control run；视觉采用不推进物理的
   固定状态注入离线渲染，避免 GUI/GPU 阻塞反向污染 60 Hz 控制轨迹。

## 2. 当前执行事实

当前 runner 的每个 60 Hz tick 顺序为：

```text
ROS callback latest-only mailbox
        |
        v
atomic snapshot at tick cutoff
        |
        v
pre_feedback q27 = s_k
        |
        v
mapping / IK / retarget / safety
        |
        v
left+right targets committed before one shared physics phase = a_k
        |
        v
120 Hz physics substep 2 次
        |
        v
optional 20 Hz GUI preview
        |
        v
post_feedback q27 + post-step scene = s_{k+1}
```

已有 `TeleoperationTickTraceV2` 为每侧保存相同的：

- `tick_id/control_index`、scheduled slot、host tick cutoff；
- `simulation_time_before_s/after_s`；
- 两个连续 physics substep 的 index、host time 和 simulation time；
- `pre_feedback_q27`、`applied_target_q27`、`post_feedback_q27`；
- 被 control tick 选中的 Tracker/Glove `source_id + producer + epoch + sequence`；
- q7/q20 intent、safety decision 和 q27 组合事实。

`TeleoperationTickTrace` 领域契约已禁止选择 callback 晚于 tick cutoff 的输入，并要求两个
physics substep 连续、target effective interval 覆盖这两个 substep。当前 analyzer 也已检查
双侧 trace 完整性、q7+q20→q27 组合、schedule miss 和 physics cadence。

这里的“双侧同 tick 动作”是 simulation-time 语义：左右 articulation API 依次写入，但两次写入
之间没有 physics step，随后才共同推进物理；不能把它表述成两次 host API 调用同时发生。

当前仍有六个直接阻塞：

1. 20-topic rosbag allowlist 没有数据相机帧；20 Hz viewport 不能替代。
2. `SceneRigidBodyState.v1` 是 post-step 状态，但消息本身没有显式 `phase` 和
   `simulation_time_s`；不得靠 converter 猜测相位。
3. 录制没有正式 `episode_start/success/abort/truncated` 事件，run 边界不能自动等同 episode。
4. 仓库没有 LeRobot 依赖、格式版本 pin、converter 或 LeRobot round-trip 测试。
5. 本验证使用 120 Hz synthetic fixture source，只能作为 exporter/gate 的 golden fixture；在
   manifest 增加 `source_mode` 和 `dataset_eligible` 并由 exporter fail closed 前，不得进入正式
   demonstration 训练集。
6. 当前 manifest 只有 q7/q20 partition 和 layout ID，没有权威的 q27 DOF-name 顺序与 canonical
   reorder；补齐 54 个名称、单位和 source index 前禁止导出。

因此现有 run 可用于实现 state/action 对齐器和验证 transition，但尚不能宣称是正式视觉
LeRobot 数据集。

## 3. 时钟、序号及其职责

| 时间 | 当前来源 | 本计划中的职责 |
|---|---|---|
| simulation time | tick V2 的 `simulation_time_before/after` | 定义物理状态和动作有效区间 |
| control ordinal | `tick_id/control_index` | 精确 join、连续性检查和 LeRobot frame index 来源 |
| host monotonic | callback、snapshot、control、apply、physics stage time | 证明输入在 cutoff 前可用，检查调度和 host 延迟 |
| ROS/bag record time | rosbag/ROS message 层 | 只用于容器读取和诊断，不参与核心对齐 |
| GUI render time/index | tick V2 render 字段 | 只审计预览负载，不进入 observation |

episode 内定义：

```text
frame_index_k = control_index_k - episode_start_control_index
timestamp_k   = frame_index_k / 60
```

`timestamp_k` 必须从整数除法逐行生成，不能使用前一 timestamp 累加 `1/60`。源 run 的绝对
simulation time、host time 和 tick ID 写入转换审计表，不混入 policy timestamp。converter 必须
先验证 `simulation_time_before_k - episode_origin == frame_index_k / 60`（冻结容差内），而不是
用规则化 timestamp 掩盖源 simulation timeline 的不连续。

当前不同输入流的 `receive_time_ns` 并非同一语义：Tracker trace 中它等同 Isaac consumer
callback time，Hand trace 中它保留 source node 的 receive time。因此，判断某条输入在 control
cutoff 前是否真正可用，一律使用 consumer `callback_time_ns <= tick_time_ns`；source/receive
时间只分别用于 freshness 与传输诊断，不能跨流直接比较。四路输入各自按
`side + source_id + producer_instance + transport_epoch + sequence` 唯一命中 raw sample，不能因
synthetic fixture 恰好复用相同 sequence 就把四路按 sequence 数值打包。

当 `schedule_slot != control_index` 或存在 missed period 时，不在时间轴上补空帧。初版正式
profile 直接拒绝该 episode；不能把 host 的一次长停顿伪装成均匀的遥操作示范。

## 4. 冻结的 transition 语义

对 episode 中每个有效 tick `k`：

```text
o_k = {
  state: concat(left.pre_feedback_q27, right.pre_feedback_q27),
  images: pre-action cameras at simulation_time_before_s,
  task: episode task
}

a_k = concat(left.applied_target_q27, right.applied_target_q27)

next_state_k = concat(left.post_feedback_q27, right.post_feedback_q27)
```

动作 `a_k` 的物理有效区间是 trace 中记录的：

```text
[target_effective_start_sim_time_s,
 target_effective_end_sim_time_s)
```

它必须恰好覆盖两个 120 Hz physics substep。相邻 tick 必须满足：

```text
post_feedback[k] == pre_feedback[k + 1]
simulation_time_after[k] == simulation_time_before[k + 1]
physics_substep_indices[k][-1] + 1 == physics_substep_indices[k + 1][0]
```

比较使用数据集 profile 中冻结的数值容差；容差和观测最大误差都写入 converter manifest，
不能由 converter 临时放宽。

字段名虽为 `applied_target_q27`，其事实语义是经过 retarget/safety/clamp 后、已向 articulation
API 提交的 commanded target，不是 backend 对 drive target 的 readback。左右 API 写入在 host
上依次发生，但中间不推进 physics；任一侧写入异常、runner 非完整退出或缺失 post-feedback，
整个 tick/run 都不可导出。实际运动结果只由后续 `post_feedback` 审计。

### 4.1 首帧与末帧

- 首帧必须直接记录/重放 episode 第一条 action 之前的 `s_0` 和相机 observation。
- 每个 LeRobot row 都必须有真实执行过的 action；不能为了保留 terminal observation 伪造
  一条 hold/no-op action。
- 最后一条 action 后的 `s_T` 和 scene truth 保存在审计 sidecar，作为回放和终止验证事实。
- 若下游格式将来支持无 action 的 terminal observation，可另做版本升级；不能静默改变本版
  row 语义。

## 5. 信号到 LeRobot 的映射

初版 policy-facing artifact 只包含部署时可获得的输入和策略需要输出的动作：

| 源事实 | 目标字段 | 策略可见 | 说明 |
|---|---|---:|---|
| 左右 `pre_feedback_q27` | `observation.state`，q54 | 是 | 真实动作前关节状态，单位 rad |
| pre-action RGB | `observation.images.<logical_camera>` | 是 | 每相机每 tick 恰好一帧 |
| 左右 `applied_target_q27` | `action`，q54 | 标签 | 向 Isaac 提交的绝对关节 command target，非 backend readback，单位 rad |
| task description | `task`/task metadata | 是 | episode 级冻结文本 |
| `post_feedback_q27` | alignment sidecar | 否 | 仅为 `s_{k+1}` 和回放验证 |
| Tracker/Glove raw | immutable raw + sidecar | 否 | operator privileged signal；部署时不存在 |
| q7 mapping/IK、q20 intent | sidecar | 否 | 解释 retarget/safety，不作为 policy action |
| safety state/reason/mask | sidecar/filter | 否 | 选择 episode，不作为视觉策略输入 |
| scene rigid-body/contact/task truth | sidecar/evaluation | 否 | 仿真 privileged truth，默认不进 observation |
| host/schedule/render facts | converter manifest/sidecar | 否 | 质量与可复现性审计 |

q54 顺序须在新增 dataset profile 中冻结为：

```text
[left.arm.q7 canonical names,
 left.hand.q20 canonical names,
 right.arm.q7 canonical names,
 right.hand.q20 canonical names]
```

profile 必须逐维给出 canonical name、单位和 source index；converter 按它显式 reorder，并在
LeRobot feature metadata 中写出全部 54 个语义名称。当前 manifest 尚不具备该顺序，不能只依赖
列位置、layout ID 或 q7/q20 长度推断。当前没有 velocity/effort capability，初版不补零、不用
未来差分派生。

若未来训练明确允许 scene state 或 teleop signal，必须输出不同的 dataset profile/repo，并在
名称中标明 `privileged`；不能向本 policy-facing artifact 增列后让训练端自行约定忽略。

## 6. 视觉采集策略

### 6.1 禁止使用 GUI preview

当前 GUI 每三个 control tick 渲染一次，且发生在两个 physics substep 之后。它同时违反：

- 20 Hz 与 LeRobot 60 Hz 主时间格不一致；
- 画面相位是 `s_{k+1}`，与 `action[k]` 同行会看到动作结果；
- Kit `app.update()/render` 已在 30 s 验证中造成约 78 ms 长阻塞和 schedule miss。

因此 GUI render index 只能作为运行诊断字段。

### 6.2 推荐：headless 控制 + 离线固定状态注入渲染

正式数据生成分成两个不可混淆的阶段：

1. headless control run 只产生完整 120/60 Hz 物理轨迹、raw teleop、tick V2 和 episode 事件；
2. renderer 从不可变 run 注入每个 `s_k`，在不推进 physics 的情况下渲染逻辑相机；
3. 每个图像记录 `run_id/episode_id/tick_id/frame_index/camera_id/phase=pre_action/`
   `simulation_time_s` 和 camera/profile hash；
4. converter 只按 exact `episode_id + tick_id + phase` join，不使用 nearest timestamp。

为保证可重放，原始轨迹必须增加或明确冻结：

- 每个 tick 的 pre-action 双侧 articulation q54；
- 每个 dynamic rigid body 的 pre-action pose；
- camera prim、intrinsics、resolution、renderer、lighting、USD dependency closure 和版本 hash；
- episode 初态、随机种子和 reset/config identity；
- motion blur 关闭。若未来需要 exposure/motion blur，必须改为显式曝光区间的仿真 sensor
  契约，不能用静态 state replay 冒充。

当前 `SceneRigidBodyState.v1` 只在 post-step 发布。实现时应新增版本化的 pre-action scene
observation，或新增完整 `SimulationObservationFrame`；不得修改 v1 CDR 布局，也不得让
converter 通过 `tick-1` 隐式移位补首帧。

这里承诺的是当前刚体 banana-bowl 场景的逐帧状态注入渲染，不是从初态重新执行 action 的
动力学回放。后者若要升级为硬 Gate，必须另行记录初始 articulation qpos/qvel、左右 drive
target、动态物体 pose/twist、simulation step/time、随机种子、solver/config/Isaac identity，并
验证接触动力学确定性；仅有 q54 与物体 pose 不足以作此声明。

初版每个逻辑相机都按 60 Hz 生成视频帧。若后续需要 30 Hz 低成本版本，应输出独立派生
dataset，明确 30 Hz controller/action 语义；不能在 60 Hz artifact 中无标记复制旧图像。

## 7. Episode 边界

正式录制需新增 versioned episode lifecycle 事实，至少包含：

```text
run_id / episode_id / task_id / task_text
initial_condition_id / random_seed / operator_session_id
event = start | success | abort | truncated
control_index / simulation_time_s / host_monotonic_time_ns
requested_host_monotonic_time_ns
effective_terminal_control_index / effective_terminal_simulation_time_s
reason / evaluator_id / evaluator_version
```

规则如下：

1. `start` 只能发生在场景 reset/settle 完成、四路输入均 actionable、左右 reference 已建立后。
2. reference revoke、transport epoch change、simulation reset、trace failure 或 hard Gate failure
   发生后，该 episode 终止为 abort/incomplete；不能删除中间坏帧再拼接。
3. `success` 必须来自版本化的 task evaluator 或显式操作者事件。当前 capability
   `task_truth=false` 时，不能根据最终物体位置事后猜 success。
4. run 可以包含多个 episode；只有显式声明“单 run 单 episode”且 lifecycle 完整时，两者才
   可一一对应。
5. warm-up、reference establishment、reset 和 terminal hold 不进入 LeRobot action rows。
6. terminal 请求可能在 tick 中途到达，runner 应完成或明确判废该 tick。lifecycle 同时记录请求
   时刻与生效边界；只有最后一个左右命令均已提交、physics/post-feedback 完整的 transition 能
   成为 terminal row，不能直接用请求时刻裁切 MCAP。

现有没有 lifecycle 的 pilot 只能标为 `diagnostic`，不能自动把首尾 tick 裁剪后并入正式集。

## 8. 转换流水线与不可变性

### 8.1 输入

只接受：

- `receipt.state=complete`、recorder/consumer 零退出；
- checksum、manifest、topic inventory 和 MCAP 全部闭合；
- `TeleoperationTickTraceV2`，不把 v1 缺失字段补零；
- dataset profile、episode lifecycle 和相机 profile 均有 version/hash。

### 8.2 派生顺序

```text
immutable raw run
  -> structural/causal validator
  -> exact-tick alignment audit table
  -> fixed-state pre-action rendering
  -> policy feature whitelist
  -> LeRobot v3 writer
  -> finalize + reopen + round-trip validator
  -> split-specific artifacts and train-only statistics
```

源 run 永远只读。输出写入新目录并生成 converter manifest，至少记录：

- 所有源 checksum；
- converter 版本、Git commit/dirty state；
- LeRobot package 版本与 upstream commit；
- Isaac/renderer 版本和 dataset/camera profile hash；
- episode/tick/frame 数、拒绝原因、Gate 配置与结果；
- LeRobot 所有 Parquet/MP4/metadata checksum；
- 每个 LeRobot frame 到 source `run_id/episode_id/tick_id` 的映射。

转换失败不得留下可被 loader 误认的半成品；临时目录成功校验后再原子发布，并拒绝覆盖已存在
的 dataset revision。

## 9. 硬 Gate

### 9.1 每个 raw episode

- 每个 tick 恰好一条 left、一条 right trace，无 duplicate/unpaired tick；
- 双侧 `times` 和 `execution` 完全相同；
- `tick_id/control_index` 连续，`schedule_slot == control_index`，missed period 总数为 0；
- 每 tick 恰好两个连续 physics substep，simulation time 增量为 `1/60 s`；
- target effective interval 与该 tick 的 simulation interval 完全一致；
- 四路 active source 均存在，source epoch/reference 在 episode 内合法且连续；
- 所有 selected callback time 不晚于 tick cutoff；input age 满足 dataset profile 的逐帧上限；
- q7+q20 decision 按 manifest 分区重组后与 `applied_target_q27` 在冻结容差内一致；
- `post_feedback[k]` 与 `pre_feedback[k+1]` 连续；新增 pre-action scene schema 后，
  `post_scene[k]` 还须与 `pre_scene[k+1]` 连续；
- state/action 全部 finite，joint layout、单位与 profile 完全一致；
- 无 trace publish failure、simulation reset 或不明 terminal 状态。

position clamp/rate limit 是已执行动作事实，不自动视为泄露。是否允许及最大比例由 dataset
profile 预先冻结；非 tracking/missing/rejected action 一律不能进入正式 episode。
现有验证文档中的 input age P95 `<20 ms` 只是 NV-5.1 运行参考，不直接升级为数据集发布阈值；
正式 profile 必须在 pilot 后预注册逐帧上限及统计 Gate。`source_mode=synthetic`、fixture producer
或 `dataset_eligible!=true` 的 run 只能进入 golden/diagnostic 集合。

### 9.2 每个视觉流

- 每个 accepted `(episode_id, tick_id, camera_id)` 恰好一帧；
- phase 必须为 `pre_action`，simulation time 必须等于该 tick 的 `simulation_time_before_s`；
- camera/profile/hash 在 episode 内不发生未声明改变；
- 解码帧数、PTS 顺序、分辨率和 channel layout 与 metadata 一致；
- 没有 duplicated stale frame、GUI preview frame 或跨 episode 编码错位。

### 9.3 LeRobot artifact

- `fps == 60`，每行 `timestamp == frame_index / 60`；
- `observation.state` 和 `action` 均为 q54，name/order/unit 与 profile 一致；
- 每 camera 的帧数等于 action row 数；
- 用固定 LeRobot 版本可 `finalize -> reopen -> 随机访问/顺序解码`；
- delta timestamp 测试只在 episode 内取得历史 observation 和当前/未来 action；
- 固定状态注入渲染逐帧反查唯一 pre-action q54/scene/camera profile；在未补齐完整动力学初态
  与 solver identity 前，不把 physics action replay 列为发布 Gate；
- policy feature whitelist 不含 teleop raw、post-feedback、scene truth、safety/debug 或未来帧；
- frame-to-source 映射为一一对应，checksum 完整。

## 10. 防止数据泄露

### 10.1 时序泄露

以下情况全部 hard fail：

- 将 `post_feedback[k]` 或 post-step image/scene 当作 `observation[k]`；
- 用 nearest join 选中 `simulation_time_before[k]` 之后生成的图像；
- 对 state 使用包含未来点的中心差分/双边插值；
- action chunk 穿过 episode 终点、reset 或 invalid tick；
- 为保持固定长度而复制未来帧、跨 episode padding 或删除坏帧后重编号。

### 10.2 特权信息泄露

policy artifact 使用显式白名单，而不是“把所有字段写入后由训练者忽略”。raw teleop、仿真
scene truth、task evaluator、post-state 和 supervisor reason 只存在于独立 sidecar。训练入口
必须声明允许的 feature keys，并在 CI 中拒绝未知 observation key。

### 10.3 train/validation/test 泄露

在生成窗口和统计量之前，先按以下 group 分割 episode：

```text
source run / operator session / task / scene revision /
initial-condition family / random seed family / asset revision
```

不按 frame 随机切分。相邻或重叠 action chunk 永远留在同一 split。正式发布优先输出独立的
train/validation/test LeRobot artifact；策略归一化只使用 train split 统计，不使用全数据集
`meta/stats.json` 评估 validation/test。

## 11. 实施切片

1. 新增并冻结 dataset profile：LeRobot pin、60 Hz、q54 order、camera inventory、单位、
   allowed safety states、input-age/tolerance Gate 和 episode 规则。
2. 新增 versioned episode lifecycle 与 pre-action simulation observation/scene schema；保留旧
   ROS 消息真实兼容，不原地扩展 CDR。
3. 扩展 raw validator，加入相邻 transition、episode、pre/post phase 和数据集发布 Gate。
4. 实现只读 exact-tick alignment table，并为每行保留 source provenance。
5. 实现 headless fixed-state injection renderer；先覆盖当前 rigid banana-bowl 场景和冻结相机集。
6. 实现 LeRobot v3 exporter、原子 finalize、checksum 和 round-trip validator。
7. 构造最小 synthetic golden episode，故意注入 post-image 泄露、tick gap、左右不配对、
   PTS 错位、跨 episode action window，确认全部 hard fail。
8. Workstation2 先跑短 headless pilot，完成固定状态注入渲染与视觉逐帧核对，再批准正式采集；
   完整动力学 action replay 作为补齐全状态后的增强 Gate。

## 12. 正式采集前停止条件

以下任一未闭合时，不生成“正式 LeRobot”标签：

- 仍使用 GUI preview 作为视觉；
- episode lifecycle 未实现；
- pre-action scene/image phase 不能精确证明；
- 30 s control run 仍有 schedule miss；
- q54 layout、LeRobot 版本或 camera profile 未 pin；
- converter 需要 nearest timestamp、插值、补帧或猜测 terminal action 才能完成；
- policy artifact 与 privileged sidecar 未物理隔离；
- train-only split/statistics 尚未冻结。

## 13. 外部格式参考

- [LeRobotDataset v3.0 官方文档](https://huggingface.co/docs/lerobot/main/en/lerobot-dataset-v3)
- [LeRobot timestamp/delta_timestamps 示例](https://github.com/huggingface/lerobot/blob/main/examples/dataset/load_lerobot_dataset.py)
