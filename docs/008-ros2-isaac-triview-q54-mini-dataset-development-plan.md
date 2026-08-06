# 008：ROS2—Isaac 三目 q54 Mini 仿真数据集采集与 LeRobot 导出开发计划

- 状态：开发、部署与确定性无设备验收完成；S12 的 `-05` 诊断回放、独立 pre-ready GUI、
  performance governor 下 120/60/20 满负载，以及 010 的两次 ROS2—Isaac—GUI 像素闭环均已
  通过；仅剩一次真设备短确认，随后录制新的正式 accepted episode 并完成真实 collection
  round-trip
- 日期：2026-08-04
- 实施收口：2026-08-05
- 运行说明：[ROS2—Isaac 三目 q54 mini 数据集组件](components/ros2-isaac-triview-q54-mini-dataset.md)
- 验收记录：[2026-08-05 008 无设备验收](validation/2026-08-05-ros2-isaac-triview-q54-mini-dataset.md)
- GUI 回归门：[2026-08-06 确定性 ROS2—Isaac—GUI Qualification](validation/2026-08-06-deterministic-ros-isaac-gui-qualification.md)
- 目标：在现有双 NERO、双 Wuji Hand 2 Beta、Tracker、Glove、ROS2 Jazzy 和 Isaac
  120/60 Hz 因果录制链上，完成少于 20 条轨迹的三目 RGB mini 仿真数据集
- 原始 episode：一次 Isaac 启动、一次 rosbag2 MCAP 录制、一次有序结束
- 策略数据：三路 RGB、动作前 q54 observation、绝对 q54 action 和 episode 级任务文本
- 数据用途：验证数据管线和模型预测趋势，不以训练有效策略、任务成功率或真机部署为目标
- 上游控制计划：[005：ROS2—Isaac 60 Hz 控制](005-ros2-isaac-60hz-control-feature-plan.md)
- 当前控制验证：[2026-08-04 NV-5.1 60 Hz 验证](validation/2026-08-04-nv51-ros2-isaac-60hz-local.md)
- 上游因果设计：[006：Isaac 60 Hz 到 LeRobot v3](006-isaac-60hz-lerobot-causal-alignment-plan.md)
- 相机装配设计：[007：双腕 D405 仿真集成](007-isaac-dual-wrist-d405-simulation-plan.md)
- 原始录制边界：[ADR-0009](decisions/0009-ros2-full-causal-recording-boundary.md)
- 未来连续采集：[009：脚踏板与自动 reset](009-pedal-auto-reset-multi-episode-mini-feature-plan.md)

## 1. 文档地位与冲突优先级

本计划是正式采集前的综合开发基线。它不把 006、007 中尚未实现的设计描述成当前能力。
发生冲突时按以下优先级执行：

1. 005 与 2026-08-04 验证继续定义当前 120 Hz physics、60 Hz control、20 Hz GUI preview
   的真实能力和 Gate。
2. 本计划覆盖 006 中的策略视觉频率、episode 组织、success 语义和模型验收范围：
   - 原始控制事实仍为 60 Hz；
   - 策略视觉与 LeRobot 行为网格改为 30 Hz；
   - 一个 run 等于一个 episode；
   - 不实现 task success、reward 或 rollout 成功率；
   - 模型训练与模型验收不属于本计划。
3. 本计划覆盖 007 中的数据相机型号、模态和捕获相位：
   - 三路相机为固定 scene D435i 和左右腕 D405；
   - 仅 RGB，不录 depth；
   - 数据相机不在控制循环内 post-action capture；
   - 采用动作前固定状态离线渲染。
4. 007 的以下内容仍可复用：
   - 被动 mount/sensor 的五层所有权；
   - 左右 wrist rig 装配、镜像、碰撞和 canonical frame；
   - USD 到 ROS optical 坐标转换；
   - Camera API readback、内外参 provenance 和同帧验证思路。
5. 当前 D405 140° 腕部画面是纯仿真特殊投影。它与未来实体 D405 的真实内参、畸变、
   rolling/global shutter、曝光和标定无继承关系。

相机的仿真限定不是启动参数，也不是操作者可切换选项。实现中不得新增
simulation-only、physical-calibration 或 140-degree 的 launch 参数。该事实只通过本计划、
固定 profile 身份、必要代码注释和实际相机参数 provenance 表达。未来实机功能必须建立
独立 profile、独立标定和独立验证，不允许运行时 fallback。

## 2. 已冻结决策

| 项目 | 本版决策 |
|---|---|
| 机器人 | 双 Agile NERO q7 + 双 Wuji Hand 2 Beta q20，总状态/动作 q54 |
| 人机输入 | 左右 Tracker SE(3) + 左右 Glove 21×3 landmarks |
| 场景相机 | 固定 scene D435i 仿真相机 |
| 腕部相机 | 左右 D405 仿真相机，当前使用 140° 特殊广角效果 |
| 视觉模态 | 三路 RGB-only |
| physics | 120 Hz |
| control/raw action | 60 Hz |
| GUI preview | 20 Hz，仅供操作者观察 |
| 数据图像 | 离线动作前渲染，30 Hz |
| LeRobot fps | 30 |
| episode | 一次启动、一次录制、一次 Ctrl+C 有序结束 |
| episode 结果 | accepted 或 rejected；不使用 success/failure 任务语义 |
| 自动 reset | 当前不实现 |
| 脚踏板 | 当前不实现，见 009 |
| 原始事实 | rosbag2 MCAP + manifest/receipt/checksum |
| 图像位置 | 可独立于控制 MCAP，但必须逐帧 exact-tick 可追溯 |
| 数据规模 | 少于 20 条；建议 2 条诊断、6～12 条正式 accepted；保留轨迹硬上限 18 条（含诊断） |
| 模型 | 训练和验收不在本计划；只为未来 Pi0.5 保留兼容信息 |
| success evaluator | 不实现 |
| depth/effort | 不实现 |
| raw contact | 尽力实现，不作为正式采集阻塞条件 |

## 3. 当前真实基线

本节保留 2026-08-04 制定计划时的输入基线；它不是当前实施状态。当前状态以上述实施收口、
组件文档和 2026-08-05 验收记录为准。

### 3.1 已实现

当前主线已经具备：

- raw 左右 Tracker typed sample 和 Tracker lifecycle；
- raw 左右 Glove canonical 21×3 landmarks；
- 独立 ROS executor、latest-only mailbox 和同 tick cutoff 原子快照；
- arm mapping、reference、IK q7 candidate/residual/reason；
- Glove retarget q20、confidence、model/config identity；
- q7/q20 safety、position clamp、rate limit 和 reason；
- 左右同 physics phase 前提交的 applied q27；
- 每侧 pre_feedback_q27、applied_target_q27、post_feedback_q27；
- 每 control tick 两个连续 120 Hz physics substep；
- 60 Hz control、20 Hz GUI preview 和调度事实；
- post-step dynamic rigid-body pose，以及固定物体 manifest pose；
- run-unique rosbag2 MCAP、manifest、recorder metadata、receipt 和 checksum；
- analyzer 对现有 20-topic 合同、raw 到 selected source、左右 tick、q7+q20 到 q27、
  mailbox、60/120 Hz、RTF 和 schedule miss 的离线检查。

### 3.2 计划制定时尚未实现

以下全部是本计划需要开发或验证的能力：

- 三路生产级数据相机和离线 renderer；
- episode 精确动作起点和终点事件；
- pre-action scene observation；
- q54 权威名称、顺序、单位、source index 和限位；
- qdot54 backend readback；
- link7、palm、fingertip pose；
- frame truth、相机静态/逐帧外参和 exact-tick join；
- source_mode、dataset_eligible 和 fixture 排除；
- episode accept/reject/quarantine 管理；
- 30 Hz alignment artifact；
- LeRobot 依赖锁、exporter、finalize/reopen 和 round-trip validator；
- 相机与新 schema 接入后的 release-gate CLI；
- Tracker/Glove 真设备下的 60 Hz 短 pilot。

007 后续已完成 mount、双腕 Camera 和自碰撞验证；008 在其固定装配合同上实现了 raw facts、
固定状态三目渲染、release/quality/bundle/registry 和 LeRobot 导出。真设备 pilot 与真实 episode
仍必须单独通过 S12，不能由无设备结果替代。

## 4. 目标与非目标

### 4.1 必须达到的目标

1. 每个 accepted episode 保存解释每个 q54 控制决定所需的完整事实。
2. raw q21、retarget q20、arm q7 和最终 applied q54 同时保留，不能互相替代。
3. 三路 RGB 对应动作前状态，不观察同一行 action 已产生的结果。
4. 控制 MCAP 与视觉 artifact 即使物理分开，也能通过 episode_id、tick_id、camera_id、
   phase、simulation_time 和 checksum 一一追溯。
5. 原始控制事实保持 60 Hz；策略 artifact 明确为 30 Hz，不伪装成完整 60 Hz policy grid。
6. 主线只录原始事实；校验、派生、筛选、绘图和 LeRobot 转换全部离线进行。
7. rejected episode 可用一条简单命令从候选数据集中排除，并保留可恢复路径。
8. LeRobot artifact 能在固定版本下 finalize、reopen、顺序/随机访问和解码。
9. 少量轨迹足以显示接近物体、手指闭合、物体运动等趋势；不做泛化或任务成功声明。

### 4.2 明确不做

- 不控制 NERO 或 Hand 2 真机；
- 不连接实体 D435i/D405；
- 不把仿真 140° 参数称为实体 D405 能力；
- 不录 depth；
- 不实现 effort；
- 不实现 reward、success predicate、success rate 或 task evaluator；
- 不实现自动 scene reset；
- 不实现连续多 episode session；
- 不实现脚踏板；
- 不训练、修改或验收 Pi0.5；
- 不要求闭环 rollout 成功；
- 不进行多任务、跨场景、跨资产或跨操作者泛化；
- 不要求动力学 action replay 的确定性；
- 不把 raw Tracker/Glove 或仿真 privileged truth暴露为默认 policy observation；
- 不在生产进程计算 RMSE、coverage、percentile 或图片。

## 5. 总体架构

正式路径分成控制采集、离线渲染、数据转换三个彼此隔离的阶段：

    ROS2 Tracker/Glove sources
                  |
                  v
    Isaac consumer: 120 Hz physics / 60 Hz control / 20 Hz preview
                  |
                  +--> immutable control MCAP
                  |      raw input + q7/q20/q27 + pre/post q54
                  |      pre-action scene + qdot/link truth + timing
                  |
                  v
        structural and causal validator
                  |
                  v
        exact 30 Hz alignment table
                  |
                  v
        fixed-state pre-action renderer
                  |
                  +--> lossless three-camera vision artifact
                  |
                  v
         episode bundle validator
                  |
                  v
        LeRobot v3 exporter at 30 Hz
                  |
                  v
        finalize / reopen / decode / report

### 5.1 在线平面职责

在线只允许：

- 完成已有遥操作控制；
- 读取已经存在的 backend 状态；
- 发布版本化原始事实；
- 保存静态 profile/hash/provenance；
- 有序关闭录制。

在线禁止：

- 为数据质量额外插值或平滑；
- 对低 confidence 自动美化 q20；
- 计算 task success；
- 生成训练统计；
- 执行视频编码；
- 为等待数据相机渲染阻塞 60 Hz control。

### 5.2 离线平面职责

离线负责：

- artifact/checksum 验证；
- exact-tick 对齐；
- 30 Hz 选帧；
- 固定状态注入渲染；
- 视觉与控制 bundle；
- episode accept/reject；
- LeRobot 导出；
- 统计、图片和质量报告。

离线工具不得 import 或启动 ROS publisher、OpenVR、Glove SDK、Isaac control loop 或任何
真实设备 adapter。离线 renderer 是唯一可启动 Isaac 的例外，但它只能读取 immutable
episode 并写入新的派生目录，不能发布 command。

## 6. Episode、run 与 artifact 身份

### 6.1 身份规则

本版严格冻结：

    collection_id  = 一批少量轨迹的人工组织身份
    run_id         = 当前已有 run-unique 身份
    episode_id     = run_id
    one run        = one episode
    one launch     = one episode

collection_id 只用于把少于 20 条 episode 组成一个候选数据集，不跨 episode 提供时钟或
控制语义。每次启动必须产生新的 run_id；拒绝覆盖旧目录。

### 6.2 Episode bundle

建议目录：

    artifacts/runs/<program>/<episode_id>/
    ├── manifest.json
    ├── recorder.json
    ├── raw/
    │   └── rosbag2/
    │       ├── metadata.yaml
    │       └── *.mcap
    ├── receipt.json
    ├── checksums.sha256
    ├── annotation.json
    └── derived/
        ├── alignment/
        │   ├── frames.parquet
        │   ├── rejected_ticks.json
        │   └── manifest.json
        ├── vision/
        │   ├── scene_rgb/
        │   ├── left_wrist_rgb/
        │   ├── right_wrist_rgb/
        │   ├── frame_index.parquet
        │   ├── manifest.json
        │   └── checksums.sha256
        └── quality/
            ├── summary.json
            ├── summary.csv
            ├── plots/
            └── report.html

raw 目录及其现有 closure 文件仍是原始事实。derived 目录只能由只读转换生成；任何阶段
失败时不得改写 raw 或 receipt。

视觉是否采用逐帧 PNG、lossless video 或其他无损容器由版本化 vision storage profile
决定，不由启动参数决定。第一版数据量很小，优先采用可逐帧 checksum 和无损解码的方案；
LeRobot MP4 是派生训练载荷，不替代 lossless vision truth。

### 6.3 跨 artifact 追溯

每个视觉帧索引至少包含：

- collection_id；
- episode_id/run_id；
- dataset_frame_index；
- source control_index/tick_id；
- camera_id；
- phase，固定为 pre_action；
- capture_simulation_time_s；
- source state digest；
- camera profile hash；
- scene/session/assembly hash；
- RGB payload checksum；
- renderer/version/hash。

episode bundle manifest 保存 control artifact、alignment artifact、vision artifact 和
LeRobot export 的相对路径与 checksum。任何视觉帧无法唯一反查 control tick 时，整个
vision artifact 不得发布。

## 7. 最小 Episode 生命周期

当前不实现完整多 episode supervisor，只新增足以证明单 episode 行边界的最小事实。

### 7.1 生命周期状态

建议新增 versioned DatasetEpisodeBoundary 事实：

    opened
      -> ready
      -> recording
      -> stop_requested
      -> closed

它不包含 success、reward 或 task completion。

至少记录：

- schema；
- run_id/episode_id；
- event；
- reason；
- host monotonic time；
- control_index/tick_id；
- simulation_time_s；
- recorder ready 状态；
- input/reference ready 状态；
- scene settled 状态；
- stop request signal；
- effective final control index；
- source_mode 和 dataset_eligible。

### 7.2 开始边界

ready 只能在以下条件全部满足后产生：

1. 场景加载、reset 和 settle 完成；
2. 两侧 q27 articulation、partition 和 limits 验证完成；
3. rosbag recorder 已发现全部 frozen topics；
4. 左右 Tracker、左右 Glove 均 actionable；
5. 左右 arm reference 已建立；
6. transport epoch 稳定；
7. dataset profile、camera set 和 q54 profile 已写入 manifest；
8. 当前 source_mode 不是 synthetic fixture；
9. GUI preview 已进入 20 Hz 正常状态。

ready 对应的下一个完整 control tick 是 episode 的第一条候选 transition。操作者必须在终端
打印 DATASET EPISODE READY 后才开始动作。ready 之前的 warm-up、reference 和 settle
仍可存在于 MCAP，但 exporter 不得写入 LeRobot。

### 7.3 结束边界

当前操作者用 Ctrl+C 请求结束：

1. signal handler 只锁存 stop request；
2. runner 完成正在执行的 control tick；
3. 左右 action 均提交，两个 physics substep 和 post-feedback 完整；
4. 发布 stop_requested 和有效 final control index；
5. 进入 hold，不再生成候选 action row；
6. 发布 closed/terminal recording status；
7. consumer receipt、rosbag SIGINT、metadata finalize 和 checksum 按 ADR-0009 有序完成。

不能直接用 signal host time 截断 MCAP，不能保留只有单侧 action 或缺失 post-state 的末 tick。

### 7.4 Episode 结果

本版只区分：

- accepted：结构完整且人工愿意纳入候选集合；
- rejected：轨迹不需要、操作失误、画面不合适或任一硬 Gate 失败；
- incomplete：进程或 artifact closure 异常。

accepted 不是 task success。rejected 不是 task failure。annotation 可以保存简短 operator
note，但它不进入默认策略输入。

## 8. 时钟和 30 Hz 因果 transition

### 8.1 时钟职责

| 时间/序号 | 职责 |
|---|---|
| control_index/tick_id | 60 Hz 原始 transition 权威序号 |
| simulation_time_before/after | 物理状态和动作有效区间 |
| physics substep index | 证明每个 action 覆盖两个 120 Hz substep |
| host monotonic | 输入因果、调度、回调和结束请求审计 |
| ROS/bag time | 容器读取与诊断，不做核心 join |
| GUI render index/time | 预览负载审计，不进策略 observation |
| dataset_frame_index | 30 Hz LeRobot 行序号 |

### 8.2 60 Hz 原始 transition

每个 source tick k：

    s_k     = concat(left.pre_feedback_q27, right.pre_feedback_q27)
    a_k     = concat(left.applied_target_q27, right.applied_target_q27)
    s_k+1   = concat(left.post_feedback_q27, right.post_feedback_q27)

动作有效区间必须覆盖 trace 中的两个连续 physics substep。左右 API 写入是同一 physics
phase，不表述为 host 同时调用。

### 8.3 30 Hz 数据集 transition

令 episode ready 后第一个完整 tick 为 k0。数据集第 j 行选择：

    k = k0 + 2*j

    observation.state[j] = s_k
    observation.image[j] = 三路相机在 simulation_time_before[k] 的动作前图像
    action[j]            = a_k
    timestamp[j]         = j / 30

规则：

- 只按相对 k0 的偶数 tick 选择；
- 不按 bag timestamp nearest join；
- 不插值 q54；
- 不复制 30 Hz 图像填充 60 Hz 行；
- 不把奇数 tick action 打包进 q108；
- 奇数 tick 的完整 60 Hz q54 仍保存在 raw MCAP 和 alignment sidecar；
- 30 Hz artifact 明确不声称复现每个 60 Hz command；
- episode 末尾若只剩一个非 anchor tick，保留在 raw，但不产生虚假 30 Hz row；
- action window 不跨 episode；下游 action padding 必须使用显式 mask 或排除末尾 anchor。

### 8.4 相邻连续性

raw validator 必须验证：

    post_feedback[k] ≈ pre_feedback[k+1]
    simulation_time_after[k] ≈ simulation_time_before[k+1]
    physics_substep_last[k] + 1 == physics_substep_first[k+1]

pre/post scene schema 完成后，还验证 dynamic object pose/twist 的相邻相位闭合。容差由
dataset profile 冻结并写入 manifest，不能由单次 converter 临时放宽。

## 9. 完整信号合同

### 9.1 Policy-facing 字段

第一版只允许：

| 字段 | Shape | 单位/语义 |
|---|---:|---|
| observation.state | 54 | 左 q7、左 q20、右 q7、右 q20；动作前实际位置，rad |
| action | 54 | 同顺序；已向 Isaac 提交的绝对 target，rad |
| observation.images.scene_rgb | RGB | 固定 scene D435i 仿真画面 |
| observation.images.left_wrist_rgb | RGB | 左腕 D405 140°纯仿真画面 |
| observation.images.right_wrist_rgb | RGB | 右腕 D405 140°纯仿真画面 |
| task | episode 字符串 | 简短动作描述，不表示 success predicate |
| timestamp | scalar | dataset_frame_index / 30 |

### 9.2 Raw 必录 P0

#### 人手输入

- 左右 Tracker 原始 SE(3)、tracking 状态、device identity、sequence/epoch/time；
- 左右 Glove canonical q21 landmarks、confidence/status、sequence/epoch/time；
- consumer callback time 和 tick cutoff；
- 本 tick selected source provenance；
- reference establish/revoke 与 reason。

#### Arm q7

- mapped target pose；
- tracker/workcell translation 与 rotation delta；
- mapping accepted、clamp 和 reason；
- IK candidate q7；
- solver success、position/orientation residual；
- q7 command；
- safety state/reason、position clamp、rate limit。

#### Hand q20

- q21 原始 observation；
- q20 retarget intent；
- retarget confidence/model/config/status；
- hand intent new/held/rejected；
- q20 command；
- hand safety state/reason、position clamp、rate limit。

q21 和 q20 必须同时存在。q21 是人手事实，q20 是机器人重定向结果，二者不得合并。

#### Execution q54

- 左右 pre_feedback_q27；
- 左右 applied_target_q27；
- 左右 post_feedback_q27；
- 左右 pre-action qdot27 backend readback；
- apply start/end/result；
- target effective simulation interval；
- physics substep index/time；
- scheduler slot/lateness/missed periods；
- GUI render fact；
- finite、limit 和 partition identity。

#### Kinematic truth

每侧每 60 Hz tick 保存：

- link7 world pose；
- hand base/palm world pose；
- 全部声明的 fingertip world pose；
- logical link ID、USD prim path、source API 和 frame convention；
- position、wxyz quaternion；
- capability/readback failure。

这些字段默认不进入 policy observation，用于判断 q20 是否产生合理的 fingertip 运动。

#### Scene truth

每个 recording inventory 内动态物体每 tick保存：

- logical object ID 与 prim path；
- pre-action pose/twist；
- post-action pose/twist；
- sleeping、kinematic、validity；
- scene/session/workcell/asset revision；
- reset 后的实际初值，而不只保存 random seed。

固定桌面、固定相机支架和其他 fixture 至少在 manifest 保存一次 pose/readback/hash。短 episode
结束后可再次读回固定 fixture，确认没有非预期漂移。

#### Episode/runtime/provenance

- episode boundary；
- source_mode 和 dataset_eligible；
- Deployment、Session、Assembly、Workcell、mapping、qualification hash；
- Isaac、renderer、ROS、RMW、Python、Git commit/dirty state；
- physics/control/render 配置；
- q54/camera/dataset profile hash；
- recorder discovery、关闭和 checksum；
- privacy 与设备 identity 假名化规则。

### 9.3 Raw contact P1

raw contact 尽力实现，推荐事实包括：

- physics substep index；
- body/link pair；
- contact point；
- normal；
- separation；
- impulse/force；
- source API 和单位。

若 Isaac 6.0.1 API 接入成本过高、影响 120/60 Hz 或无法证明 link/point/单位，本版允许：

    raw_contact: false
    reason: unsupported_in_dataset_profile

这不会阻止 episode 进入 mini 数据集，但报告必须明确不能用该数据归因具体抓取接触。
禁止用物体运动、距离阈值或 q20 闭合伪造 raw contact。

### 9.4 明确不录

- depth；
- tactile；
- joint effort；
- reward；
- success predicate；
- task completion；
- audio；
- 实体相机 USB/firmware/device timestamp；
- 实体 NERO/Hand 2 command/ack；
- 所有 USD prim 的盲目逐帧状态。

## 10. q54 Canonical Profile

新增独立 versioned dataset joint profile，逐维冻结：

- global index 0～53；
- canonical name；
- side；
- group，arm 或 hand；
- group-local index；
- source articulation DOF name；
- source index；
- source binding/layout ID；
- unit，固定 rad；
- sign；
- zero convention；
- lower/upper limit；
- nominal maximum velocity；
- state/action availability；
- real-hardware mapping status，当前为 future/unverified。

唯一顺序：

    left.arm.q7
    left.hand.q20
    right.arm.q7
    right.hand.q20

converter 必须按 name/source index 显式 reorder，不依赖当前 articulation 列位置。manifest
同时保存 runtime readback DOF names 和 profile hash；任一名称、数量、source index 或限位
不一致即 fail closed。

LeRobot metadata 和数据卡必须列出全部 54 个名称，不能只写 shape=54。

## 11. Pre-action Simulation Observation

当前 SceneRigidBodyState.v1 是 post-step 且没有 phase/simulation time，不足以离线渲染。
不得原地扩展其 CDR 布局。

推荐新增 companion schema，至少表达：

- run_id/episode_id；
- tick_id/control_index；
- phase，pre_action 或 post_action；
- simulation_time_s；
- physics substep boundary；
- 左右 q27/qdot27 readback；
- dynamic object inventory 和状态闭合计数；
- link/palm/fingertip state 闭合计数；
- frame payload digest 或事实集合 digest；
- validity/capability。

可以使用一条 aggregate frame event 加多条 bounded object/link event，也可以采用一个 bounded
aggregate message；实现前以 ROS2 Jazzy IDL 大小、序列化成本和 Isaac API 可用性做 S0 spike。
无论采用哪种线布局，必须满足：

1. 每个 tick 的 pre-action state 可独立完整重建；
2. 首帧不依赖 tick-1；
3. object/link 缺失可被计数和 fail closed；
4. v1 bag 仍能真实读取；
5. publisher 异常只把 recording 标为 incomplete，不修改控制决策。

## 12. 三路相机合同

### 12.1 逻辑身份和所有权

| camera_id | 载体 | 所有权 |
|---|---|---|
| scene_rgb | D435i 外壳/仿真 camera | Workcell |
| left_wrist_rgb | 左 D405 wrist rig | 左 Assembly subtree |
| right_wrist_rgb | 右 D405 wrist rig | 右 Assembly subtree |

策略字段使用逻辑名，不使用 serial 或 vendor 名作为 feature key。

Session 冻结所选 camera set；Deployment 只负责进程和 namespace，不拥有 K、外参或 140°。
相机设置不通过 launch args 注入。

### 12.2 仿真投影

双腕 D405：

- 640×480；
- 30 Hz 数据集 cadence；
- 水平 FOV 140°；
- pinhole、无畸变的纯仿真投影；
- 当前依赖该广角画面完成近距离手部/物体可见性；
- 不代表实体 D405 的 factory calibration 或实际 FOV。

scene D435i：

- 640×480、30 Hz policy output；
- 使用独立 D435i scene simulation profile；
- 不继承 D405 140°；
- K/D/R/P、focal/aperture/clipping 由 profile author 后通过 Isaac 6.0.1 API readback 冻结；
- profile 的准确数值和 mount pose 必须在正式采集前通过投影 spike，不在 runner 中复制。

所有相机均：

- RGB-only；
- motion blur 关闭；
- distortion_model 与 D 必须匹配实际仿真投影；
- 输出 RGB8；
- 禁止 resize、crop、颜色交换和 gamma 修正进入 lossless vision truth；
- 训练所需 resize/letterbox 属于未来模型 processor。

### 12.3 必要代码注释

在 D405 Camera prim authoring、camera profile resolver、manifest/export adapter 附近保留
明确注释，说明 140° 是当前仿真视觉设计，不是实体 D405 规格。该注释是固定设计说明，
不是 runtime option。

未来实机实现不得读取本仿真 profile 作为 calibration fallback。

### 12.4 静态相机 provenance

每个 camera 保存：

- logical camera ID；
- USD camera prim；
- parent/canonical/optical frame；
- resolution；
- focal length、horizontal/vertical aperture、aperture offset；
- clipping range；
- K/D/R/P；
- projection/distortion model；
- horizontal/vertical FOV；
- source profile/hash；
- mount/asset/assembly/session/workcell hash；
- renderer、lighting、color space、motion blur；
- authored、API readback 或 derived provenance；
- T_parent_from_camera_optical；
- pixel-centre 和矩阵 convention。

### 12.5 每帧 truth

每帧保存：

- episode_id；
- camera_id；
- dataset_frame_index；
- source tick/control index；
- phase=pre_action；
- capture simulation time；
- T_world_from_camera_optical；
- wrist camera 对应 T_world_from_hand/palm；
- RGB shape/dtype/channel；
- payload checksum；
- completed render identity；
- camera profile hash。

验证：

    T_world_from_camera_optical
    ≈ T_world_from_parent * T_parent_from_camera_optical

并检查旋转正交、det(R)=+1、wxyz 归一和双向矩阵互逆。

## 13. 固定状态离线渲染

### 13.1 选择该方案的原因

当前 GUI 30 秒验证存在 Kit app.update 偶发约 78 ms 阻塞。把三路数据相机放入 control owner
会重新污染 60 Hz，且 post-step capture 会产生动作结果泄露。

本版因此：

- GUI 20 Hz 只供操作者看；
- control MCAP 不要求包含 RGB；
- 录制结束后按 30 Hz anchor 逐帧离线渲染；
- renderer 不推进 physics；
- render 时不运行 teleop source 或 command publisher。

### 13.2 渲染输入

每个 anchor tick注入：

- 左右 pre-action q27；
- 所有 dynamic rigid body pre-action pose；
- 如渲染依赖，注入 articulation/link transform 的权威状态；
- 相同 Session/Assembly/Workcell/asset dependency closure；
- 相同 camera profile；
- 相同 lighting、background 和 renderer configuration。

本版承诺的是固定状态画面重建，不是重新执行 action。接触 manifold、solver warm state 和
未录内部状态不用于图像结果时，不需要动力学确定性 replay。

### 13.3 渲染生命周期

    validate source episode
      -> create isolated render output temp directory
      -> load pinned scene and dependencies
      -> create three render products
      -> read back static camera facts
      -> render calibration/golden frame
      -> for each 30 Hz anchor:
           inject pre-action state
           update transforms without advancing physics
           complete scene_rgb
           complete left_wrist_rgb
           complete right_wrist_rgb
           save frame truth and checksum
      -> drain renderer
      -> validate counts/order/content
      -> atomic publish derived/vision

### 13.4 Frames-in-flight Gate

Isaac 6.0.1 API spike必须证明：

- completed RGB 对应当前注入 tick，而非前一帧；
- 保存 pose 与 image 使用同一 completed frame identity；
- 左右腕和 scene 的三帧属于同一 simulation state；
- render 不推进 simulation time；
- 首帧 warm-up 不混入 dataset_frame_index=0。

无法证明时停止，不得用 bag time、当前 USD pose 或 zero-delay 文字替代证据。

## 14. GUI Preview 与控制 Gate

当前不开发独立 viewer。正式采集使用现有 GUI 20 Hz preview，数据相机仍离线生成。

每个 accepted episode 保持：

- control rate 60 Hz ±2%；
- GUI render 20 Hz ±5%；
- 每 control tick 两个 physics substep；
- physics RTF ≥0.95；
- GUI tick interval P95 ≤25 ms；
- schedule miss 总数为 0；
- 四路 selected input age P95 <20 ms，或后续真实设备 pilot 冻结的更严格 profile；
- 无 callback 晚于 tick cutoff；
- 无 transport epoch change/reference revoke。

由于轨迹少且 episode 短，本版采用“出现 miss 就 reject 并重录”，不放宽 Gate。如果相同
机器在隔离负载下连续三个短 episode 都因 GUI 阻塞被拒绝，停止正式采集，再建立独立 viewer
mini-feature；不能静默接受有限 miss。

Workstation2 正式录制期间必须独占 Isaac/Vulkan 负载，不并行运行 CAD、材质预览、训练或
其他 GPU 作业。

## 15. Episode 接受、拒绝与清理

### 15.1 单步 reject

实现一个只操作指定 episode 的离线管理命令，交互目标类似：

    manage_episode reject <episode_id> --reason operator_rejected

行为：

1. 解析明确 episode_id，不接受通配符；
2. 验证 root、manifest 和 run_id 一致；
3. 若已有 LeRobot 派生，先将其标记 stale；
4. 原子写 reject annotation；
5. 同文件系统移动到 rejected quarantine，或更新权威 registry；
6. 保留原始 checksum；
7. 输出是否可恢复和新位置；
8. 幂等重复执行。

exporter 只读取 accepted registry；rejected episode 不需要被物理删除即可从数据集消失。

### 15.2 Restore

提供对应 restore：

    manage_episode restore <episode_id>

restore 只恢复候选身份，不自动判定 accepted；必须重新通过 validator。

### 15.3 Purge

真正物理删除是独立显式操作：

- 只允许 rejected episode；
- 要求完整 episode_id 二次确认；
- 先显示路径、大小和 checksum；
- 优先移动到系统 trash 或项目 trash；
- 不得接受 workspace root、report root、空字符串、通配符或符号链接越界；
- 删除后写 collection tombstone。

普通采集工作流不调用 purge。

## 16. 数据质量 Gate

### 16.1 Raw artifact 硬 Gate

- receipt.state=complete；
- manifest/recorder/receipt/schema/run_id 一致；
- metadata 和 MCAP 非空；
- checksum 全通过；
- source_mode=live_device；
- dataset_eligible=true；
- 完整 frozen topic inventory；
- episode opened/ready/stop_requested/closed 完整；
- 每 tick 左右 trace 恰各一条；
- control index 连续；
- schedule miss=0；
- 每 tick 两个连续 physics substep；
- simulation time 连续；
- target interval 正确；
- q7+q20 与 applied q27 一致；
- post q54 与下一 tick pre q54 连续；
- q54/qdot 全 finite；
- q54 profile/name/order/unit 完全一致；
- 四路 raw selected sample 能唯一命中；
- source epoch/reference 在 episode 内稳定；
- 无 trace publish failure；
- 无不明 simulation reset；
- 最终 tick 完整。

### 16.2 Pre-action truth 硬 Gate

- 每个候选 tick恰有一个 pre-action observation frame closure；
- dynamic inventory 没有缺项/重复；
- link/palm/fingertip inventory 没有缺项/重复；
- object/link quaternion 被完整保存，不得在 reader 中丢弃；
- pre/post scene 相邻闭合；
- 固定 fixture 首尾无非预期漂移；
- qdot 是 backend readback，不是补零。

### 16.3 视觉硬 Gate

- 每个 30 Hz anchor 恰有 scene/left/right 各一帧；
- frame identity、phase 和 source tick exact match；
- 无 gap/duplicate/reorder；
- 无 stale frame hash 连续重复，静止场景例外必须由 state digest 解释；
- shape、dtype、RGB channel、resolution 一致；
- 无全黑、全白、解码失败或空 payload；
- K/D/R/P finite 且与 resolution 一致；
- 静态/逐帧外参闭合；
- camera/profile/hash 在 episode 内不变；
- 三路 frame 数等于 dataset row 数；
- payload checksum 完整；
- projection golden test 达到冻结像素误差，目标 ≤1 px，>2 px hard fail；
- vision artifact 原子发布且可完整重开。

### 16.4 不自动拒绝的诊断项

以下默认报告但不自动拒绝：

- Glove confidence 较低但 retarget 仍明确输出；
- 正常 position clamp/rate limit；
- 短暂 hold，但 source/reference 未失效；
- raw contact capability=false；
- 任务没有完成；
- 物体没有被真正抓起；
- 一部分 q20 关节运动幅度很小。

若 retarget 明确 rejected、动作非 finite、reference revoked、source stale 超过冻结上限或 safety
进入 missing/rejected，则属于结构问题，仍 hard fail。

## 17. LeRobot 导出合同

### 17.1 依赖锁

实现开始时固定：

- LeRobot stable package version；
- upstream Git tag 和 commit；
- Python/PyTorch/datasets/codec lock；
- 本项目 exporter commit；
- 目标 dataset schema readback。

以 2026-08-04 官方仓库 tag 为基线，验证 LeRobot 源码版本 0.6.1、tag v0.6.1、commit
`7e241bd630a3719a56157a497ce5d08f244784f1`。当日 PyPI 最新正式包仍为 0.6.0，不能把
`lerobot==0.6.1` 当作已发布 wheel；exporter 使用该 Git commit 和独立 Python ≥3.12 环境。
若 Workstation2 兼容性要求改变，只能更新 versioned exporter profile 和 golden tests，不能
浮动使用 main。

### 17.2 Feature mapping

| LeRobot key | 来源 |
|---|---|
| observation.state | anchor tick pre_feedback q54 |
| action | anchor tick applied_target q54 |
| observation.images.scene_rgb | same-tick pre-action scene RGB |
| observation.images.left_wrist_rgb | same-tick pre-action left wrist RGB |
| observation.images.right_wrist_rgb | same-tick pre-action right wrist RGB |
| task | episode annotation/task profile |
| timestamp | frame_index / 30 |
| frame_index | 0...N-1 |
| episode_index | collection 内稳定索引 |

policy artifact 不包含：

- q21；
- Tracker；
- q7 candidate/q20 intent；
- qdot；
- post q54；
- object/link/contact truth；
- safety reason；
- host timing；
- camera extrinsics；
- 任何未来 state。

这些字段保存在 control/alignment/quality sidecar。

### 17.3 Action 语义

action 是绝对 joint target q54，单位 rad，不是：

- delta joint；
- velocity；
- torque；
- backend drive target readback；
- human q21；
- post-state。

LeRobot dataset card 必须写明 60 Hz raw control 被明确派生为 30 Hz policy grid，奇数 raw
tick 不在 action 表内。

### 17.4 Timestamp

    timestamp = frame_index / 30

逐行从整数 frame_index 计算，禁止累加浮点 dt。converter 先验证 source simulation time 与
30 Hz anchor 一致，再生成规则 timestamp，不能用规则化 timestamp 掩盖源 gap。

### 17.5 Finalize 与 round-trip

exporter 使用临时目录：

1. 写 metadata、Parquet 和三路视频；
2. finalize；
3. 关闭 writer；
4. 用相同 pinned LeRobot 重新打开；
5. 检查 episode/frame/task/feature shape；
6. 顺序解码所有帧；
7. 随机访问首、中、末帧；
8. 验证 delta timestamp 不跨 episode；
9. 反查 frame-to-source mapping；
10. 生成所有文件 checksum；
11. 原子 rename 到正式 revision。

任何步骤失败，临时目录不得被 loader 识别为正式数据集。

### 17.6 Split 和统计

本数据量不足以作泛化评估：

- 只发布 train/candidate 语义；
- 不随机按 frame 切分；
- 不声称 validation/test；
- 所有 overlapping action window留在原 episode；
- stats 只从 accepted episode 生成；
- 近零方差 q20 不删除、不补噪声；
- 归一化必须有 epsilon，并同时保留 physical joint-range 统计。

未来若需要正式 split，应从 immutable episode 重新生成新 dataset revision。

## 18. Pi0.5 兼容预埋

Pi0.5 训练、模型修改和模型验收不属于本计划，但数据合同需要避免提前锁死。

预埋要求：

1. q54 永远完整保存，不因 Pi0.5 默认 padding/action dimension 压缩、分组丢失或改成
   hand synergy。
2. state 与 action 的 54 个名称、单位、顺序和 limits 写入 metadata/sidecar。
3. 三路图像使用稳定语义 key，未来通过 LeRobot rename map 或 policy processor 映射，
   不重命名 raw artifact。
4. raw 图像保持 640×480；Pi0.5 所需 square resize/letterbox 属于模型 processor。
5. action 保持绝对 rad。若未来 Pi0.5 使用 relative action，只能在 processor 中派生，
   不改写 dataset source。
6. 当前 Pi-family 实现常见默认 max_state_dim/max_action_dim 为 32，q54 需要未来单独验证
   max dimension、padding mask、checkpoint projection 和 output unpadding。不得在本计划中
   假定现成 checkpoint 已支持 54 维。
7. 未来训练至少按 left arm、left hand、right arm、right hand 分组查看 loss，避免 40 个
   hand DoF 在逐维平均中掩盖 14 个 arm DoF。
8. 本计划完成定义不包含 Pi0.5 loss、rollout、success rate 或趋势图。

参考：[LeRobot action representations](https://huggingface.co/docs/lerobot/action_representations)、
[rename map](https://huggingface.co/docs/lerobot/rename_map)。

## 19. 少量轨迹配方

### 19.1 规模

冻结：

- 诊断轨迹：2 条，不进入正式 train candidate；
- 正式 accepted：最少 6 条，建议 8～12 条；
- 目标保留量：2 条诊断 + 6～12 条正式 accepted，即 8～14 条；
- 硬上限：整个 collection 最多保留 18 条轨迹，包含诊断和正式 accepted；
- rejected/incomplete 不属于数据集，不计入保留量；
- 每条建议有效动作窗口 10～20 秒，原则上不超过 30 秒。

若 6 条已经足以验证导出和模型趋势，可以停止，不为凑数量继续录制。

### 19.2 两条诊断轨迹

1. 静止与缓慢双侧 sweep：
   -短静止；
   -左右 arm 分别小幅运动；
   -左右 hand 分别张合；
   -用于检查噪声、方向、q54 order、相机遮挡和左右镜像。
2. 同时运动：
   -左右 arm 同时靠近工作区；
   -左右 hand 执行不同闭合节奏；
   -允许触碰物体；
   -用于检查 54 维并发、视角覆盖和 tick 配对。

### 19.3 正式轨迹最低分布

至少：

| 轨迹角色 | 最少条数 | 目的 |
|---|---:|---|
| 左侧主动作 | 2 | 左 arm q7 + 左 hand q20 发生接近、闭合和物体趋势 |
| 右侧主动作 | 2 | 不能依赖代码左右对称替代右侧数据 |
| 双侧同时运动 | 2 | 两侧 q27 同时非静止，证明 q54 数据和视觉并发 |

剩余 2～6 条可重复最稳定的动作，不需要增加新 task。

### 19.4 动作阶段

每条尽量包含：

1. 初始短静止；
2. arm 接近物体；
3. hand 预成型；
4. 手指闭合；
5. 物体接触或运动趋势；
6. lift/transport 尝试；
7. release 或结束姿态；
8. Ctrl+C 前短稳定尾段。

不要求每条完成全部阶段，不要求香蕉进入碗，不要求物体离桌。操作失误且不希望保留时，
录制闭合后执行 reject。

### 19.5 Task 文本

使用简短、稳定、可观察的描述，例如：

- Move the left hand toward the banana and attempt to grasp it.
- Move the right hand toward the object and attempt to grasp it.
- Move both hands toward the tabletop objects.

文本描述动作意图，不包含未经证明的结果，例如 successfully place、complete 或 without
dropping。少量单任务数据不能用于声明 language grounding 能力。

## 20. 质量报告和图

每个 accepted episode 生成：

### 20.1 数字表

- artifact/receipt/checksum；
- topic/schema/count；
- episode start/end/duration；
- control/physics/GUI rate；
- schedule miss、RTF、tick P50/P95/P99/max；
- 四路 input receive/select rate 和 age；
- q54 finite/limit/clamp/rate-limit；
- frame count、drop、duplicate、decode；
- camera K/extrinsic closure；
- fixed/dynamic scene inventory；
- capability matrix；
- reject/accept annotation。

### 20.2 轨迹图

- 左右 q7 command/pre/post overlay；
- 左右 q20 command/pre/post 分组 overlay；
- applied 与 actual error；
- qdot；
- 各 joint 5～95% span；
- arm/hand/side coverage heatmap；
- source age、tick interval、RTF；
- object position/rotation；
- palm/fingertip trajectories；
- 若 contact 可用，contact timeline。

### 20.3 视觉图

- 三路同帧 contact sheet；
- 首、中、末帧；
- frame hash/重复帧 timeline；
- 黑白帧、亮度和动态范围摘要；
- 腕部手指/物体遮挡示例；
- 相机外参闭合和 golden projection 图。

这些结果用于确认数据趋势和完整性，不输出 task success。

## 21. 实施切片

### S0：冻结 ADR、profile 与 API spike

交付：

- 本计划对应 ADR，冻结 one-run-one-episode、30 Hz pre-action、三目 RGB-only；
- q54 dataset profile；
- camera set profile；
- LeRobot version pin；
- Isaac camera completed-frame、fixed-state render、qdot/link readback spike；
- pre-action scene schema选型；
- raw contact成本结论。

退出：

- 没有未决 field semantics；
- D435i/D405 camera prim和 K readback 可证明；
- 140°代码注释位置明确；
- q54 profile 54 维完整；
- LeRobot 0.6.1 golden dataset可创建、finalize和reopen。

### S1：Episode boundary 与 dataset identity

交付：

- versioned DatasetEpisodeBoundary；
- source_mode/dataset_eligible；
- ready Gate；
- Ctrl+C effective final tick；
- run_id=episode_id manifest；
- synthetic fixture强制 diagnostic。

测试：

- ready 前动作不进入 candidate；
- signal 发生在 tick 各 phase；
- 最后完整 tick保留；
- 半 tick/incomplete被拒；
- v1/v2 recording status兼容。

### S2：q54 canonical schema 和 runtime readback

交付：

- 逐维 profile；
- runtime DOF name/index readback；
- manifest q54 inventory；
- converter explicit reorder；
- limits/unit/sign/zero 验证。

测试：

- 左右交换；
- q7/q20 partition错位；
- 重复/缺失 name；
- source index变化；
- 单位或限位不一致；
- 54 维 round-trip。

### S3：Pre-action state、qdot 和 kinematic truth

交付：

- 新 versioned schema；
- 每 tick pre-action q54/qdot；
- pre/post dynamic object state；
- link7/palm/fingertip pose；
- closure digest/count；
- 现有 SceneRigidBodyState.v1 保持兼容。

测试：

- 首 tick独立；
- quaternion保真；
- object/link缺失；
- pre/post相邻；
- 固定 fixture漂移；
- publisher failure使receipt incomplete；
- 性能不扰动60 Hz。

### S4：相机五层接入

交付：

- scene D435i Workcell camera；
- 左右 D405 Assembly wrist camera；
- camera profile resolver；
- Camera prim/readback；
- 坐标 adapter；
- mount/asset/session hash；
- 无 depth；
- 无新启动参数；
- 必要 140°仿真说明注释。

测试：

- 三逻辑 camera唯一；
- 左右镜像无负 scale；
- optical frame方向；
- K/P推导；
- 静态外参；
- 碰撞和 q27 articulation不变；
- record=false不创建数据 render product。

### S5：Raw recording contract 扩展

交付：

- episode/pre-action/kinematic topics加入 versioned allowlist；
- QoS profile；
- recorder-ready discovery；
- manifest inventory/capabilities；
- analyzer reader接受新 schema并真实拒绝未知版本。

测试：

- QoS compatibility；
- topic缺失/重复；
- recorder晚启动；
- bag回放；
- shutdown/drain/checksum；
- 旧20-topic run仍可读。

### S6：Release validator

交付：

- 独立 fail-closed release-gate CLI；
- raw、episode、transition、source、q54、scene硬 Gate；
- 非零 exit code；
- 机器可读 rejection reason；
- quality analyzer与release decision职责分离。

测试：

- tick gap；
- 左右不配对；
- post/pre不连续；
- late callback；
- fixture source；
- epoch改变；
- schedule miss；
- last tick不完整；
- qdot/object/link缺项。

### S7：30 Hz exact alignment

交付：

- 只读 alignment table；
- k0 与偶数 tick selection；
- 30 Hz frame index/timestamp；
- full source mapping；
- odd tick audit sidecar；
- atomic output。

测试：

- 奇偶 episode长度；
- 非零 k0；
- tick gap；
- 末尾单 tick；
- 禁止 nearest/interpolation/fill；
- 重跑 checksum一致。

### S8：离线三目 renderer

交付：

- 隔离 renderer CLI；
- fixed-state injection；
- 三路 completed frame identity；
- lossless frame storage；
- frame truth/index/checksum；
- atomic vision publish。

测试：

- frames-in-flight；
- 首帧 warm-up；
- 不推进 physics；
- RGB channel；
- 三相机同 state digest；
- 投影误差；
- camera pose closure；
- 缺帧、重复和解码失败。

### S9：Episode bundle 与 reject 管理

交付：

- bundle manifest；
- accepted/rejected registry；
- reject/restore；
- 安全 purge；
- derived stale invalidation；
- collection inventory。

测试：

- 幂等；
- 路径越界；
- 错误 run_id；
- 符号链接；
- 已导出 episode reject；
- restore 后重新 Gate；
- 并发操作锁。

### S10：LeRobot exporter

交付：

- 独立环境和依赖锁；
- policy whitelist；
- Parquet/MP4/metadata；
- q54 names sidecar；
- frame-to-source map；
- finalize/reopen；
- dataset checksum和data card。

测试：

- one/two/multiple episode；
- 三相机随机访问；
- 30 Hz timestamp；
- shape/dtype；
- 末尾 action window；
- 未知 privileged key；
- 半成品不发布；
- 重复 revision拒绝覆盖。

### S11：质量报告

交付：

- camera/episode/q54 metrics；
- 新增表和图；
- 不输出 success；
- 报告 provenance；
- deterministic report checksum。

### S12：Workstation2 资格验证

顺序：

1. [完成] device-free synthetic golden，并以 010 A→B→A fixture 连续两次闭合到 GUI 像素；
2. [待操作者] 真实 Tracker/Glove、无任务动作的短 GUI pilot；
3. [诊断完成，正式待录] 单条真实遥操作诊断 episode；
4. [实现并诊断验证] 离线三目 render；
5. [合成 fixture 通过，真实待录] LeRobot round-trip；
6. [实现，真实待录] reject/restore 演练；
7. [待录] 批准少于 20 条正式采集。

## 22. 测试矩阵

| 层 | Fast test | ROS/Jazzy | Isaac headless | Isaac GUI/HIL |
|---|---:|---:|---:|---:|
| schema/profile/q54 | 是 | 否 | 否 | 否 |
| episode boundary转换 | 是 | 是 | 否 | 否 |
| recorder allowlist/QoS | 是 | 是 | 否 | 否 |
| causal validator | 是 | 可回放 | 否 | 否 |
| qdot/link/object readback | 否 | 否 | 是 | 是 |
| camera K/pose/frame identity | 部分 | 否 | 是 | 是 |
| offline renderer | 部分 | 否 | 是 | 否 |
| LeRobot exporter | 是 | 否 | 否 | 否 |
| 60/120/20性能 | 否 | 是 | 是 | 是 |
| Tracker/Glove device pilot | 否 | 是 | 是 | 是 |

Fast tests 不要求 GPU、ROS graph 或设备。LeRobot exporter 测试使用合成图片和短 golden
episode。GPU 测试单独标记，不能进入普通单元测试默认集合。

## 23. 正式采集运行卡

### 23.1 开始前

- 独占 Workstation2 GPU/Isaac；
- 确认磁盘空间；
- 确认当前 Git/config/asset hash；
- 上电并连接 base、Tracker、Glove；
- 检查左右设备身份；
- 检查 scene D435i、左右 D405 Camera prim；
- 确认 banana/bowl/桌面初态；
- 确认录制根目录没有同名 run；
- 运行短 preflight。

### 23.2 单条 episode

1. 启动一次 record=true GUI run；
2. 等待 recorder-ready；
3. 等待四路 input/reference；
4. 看到 DATASET EPISODE READY；
5. 执行 10～20 秒动作；
6. 保持短稳定尾段；
7. Ctrl+C；
8. 等待 CLOSED/receipt/checksum；
9. 运行 release validator；
10. 运行30 Hz alignment和离线 renderer；
11. 查看三目 contact sheet和q54轨迹；
12. 执行 accept 或单步 reject；
13. 完全退出后重新启动下一条，依靠场景重载完成 reset。

### 23.3 禁止

- 未出现 READY 就开始动作；
- 用 kill -9 正常结束；
- 在 artifact finalize 前重启同 run_id；
- 采集时并行打开 CAD/材质/GPU训练；
- 因 miss手工删中间 tick；
- 编辑 MCAP；
- 把 rejected轨迹复制进LeRobot；
- 把仿真D405内参用于实体相机。

## 24. 采集前停止条件

任一成立即停止：

- 真实 Tracker/Glove 60 Hz pilot未通过；
- 短 GUI episode仍有schedule miss；
- episode ready/final tick不能证明；
- q54 name/order未冻结；
- pre-action scene不完整；
- qdot/link/fingertip任一P0缺失；
- 三路图像不是exact pre-action；
- renderer需要nearest或tick-1补首帧；
- vision无法反查control tick；
- D405 140°与实体参数边界未在代码注释/计划中保留；
- release validator失败仍返回0；
- LeRobot不能finalize/reopen；
- reject命令存在误删宽路径风险；
- raw与derived责任混淆；
- source_mode=synthetic fixture；
- artifact依赖人工修改才能导出。

raw contact=false、没有success evaluator、没有depth/effort和没有Pi0.5验证不属于停止条件。

## 25. 完成定义

本计划完成必须同时满足：

1. one-run-one-episode 的 opened/ready/stop/closed 可精确定位；
2. current control chain在真实四路设备下维持120/60/20和零miss；
3. 每个raw tick q21、q20、q7、q27、pre/post q54、qdot、scene和kinematic truth完整；
4. q54逐维identity/order/unit/limit冻结；
5. scene D435i与双腕D405三路RGB profile/pose/K可复现；
6. D405 140°仅作为纯仿真特殊视觉，未形成实体fallback；
7. 每个30 Hz row有三路动作前图像和唯一source tick；
8. control MCAP、vision、alignment、quality和LeRobot由bundle checksum闭合；
9. release validator对所有hard Gate fail closed；
10. reject/restore安全且单步可用；
11. LeRobot固定版本下round-trip通过；
12. 两条诊断轨迹通过；
13. 正式 accepted 为 6～12 条，连同诊断轨迹后的总保留量少于 19 条；
14. 报告显示左右arm/hand和物体趋势，但不宣称task success；
15. 未修改生产边界为在线统计或在线模型处理；
16. 脚踏板、自动reset和Pi0.5训练仍明确在本计划之外。

## 26. 预计修改面

实施阶段预计涉及：

- docs/decisions/ 新 ADR；
- configs/profiles/ 的 dataset/q54/camera/vision storage profile；
- configs/assets、bindings、assemblies、sessions、workcells 中的三目身份；
- src/wujihand/specs 与 resolver 的 passive/simulated sensor 支持；
- src/wujihand/domain/recording.py 的新 versioned facts；
- src/wujihand/runtime/isaac_dual_scene.py 的 pre-action/qdot/link readback；
- ros2/wujihand_interfaces 新消息；
- ros2/wujihand_ros2 conversion/QoS/recording allowlist；
- tools/run_isaac_nero_hand2_ros.py 的 episode boundary和raw事实；
- 独立 offline alignment/renderer/exporter/episode manager；
- analysis/teleoperation_quality 的reader、validator、metrics和plots；
- tests/contract、tests/unit、ROS integration和Workstation2 validation文档。

不得修改：

- 固定NERO joint origin/axis；
- Hand 2 canonical q20语义；
- Tracker mapping的既有物理定义；
- 实体CAN/Hand 2 adapter；
- 当前UDP控制入口；
- 旧ROS消息的线布局；
- 历史raw artifact。

## 27. 风险与控制

| 风险 | 控制 |
|---|---|
| GUI偶发阻塞 | 短episode、零miss Gate、失败重录；三次重复后停止 |
| 离线图像错一帧 | completed-frame identity、state digest、golden moving-marker测试 |
| 140°被误认实体参数 | 固定文档与必要代码注释；独立future real profile |
| q54列顺序漂移 | canonical name/source-index显式reorder和runtime readback |
| 少量数据关节不动 |两条诊断和左右/双侧最低分布；只声明趋势 |
| Glove低confidence |保留raw/confidence；不在线修饰；报告但不默认拒绝 |
| contact实现代价高 |P1 capability，不能伪造，不阻塞 |
|视觉文件过多 |小于20条；versioned lossless storage profile；LeRobot再编码 |
|误删轨迹 |quarantine/restore优先；purge二次确认和路径边界 |
|Pi0.5维度不兼容 |保持q54原始合同；未来单独model processor/action-head工作 |
|旧文档被当实现 |本计划明确current vs planned，validation才证明完成 |

## 28. 未来真机兼容边界

本计划只为未来保留稳定语义：

- q54 canonical names；
- 绝对 rad action；
- scene/left_wrist/right_wrist camera key；
- camera profile和calibration epoch；
- backend/capability；
- episode/frame provenance。

未来实体 D435i/D405 必须重新完成：

- 设备发现和稳定serial绑定；
- driver/firmware pin；
- 真实可用RGB mode；
- device/host timestamp；
- 跨相机同步；
- 逐序列号内参；
- 每次remount外参；
- rolling/global shutter和曝光；
- USB/CPU/磁盘drop；
- 真实joint feedback timestamp；
- 安全和真机actuation ADR。

实体数据不能与当前140°仿真数据放入同一未标版本，也不能通过同名camera key掩盖不同
calibration/profile。相同逻辑key只表示角色一致，不表示像素投影相同。

## 29. 外部参考及采用范围

- [LeRobotDataset v3](https://huggingface.co/docs/lerobot/main/en/lerobot-dataset-v3)：
  采用Parquet、MP4、metadata、episode边界和delta timestamp接口。
- [LeRobot dataset tools](https://huggingface.co/docs/lerobot/using_dataset_tools)：
  参考episode删除/分割和video转换，但本项目raw artifact仍保持独立不可变。
- [ALOHA/ACT](https://tonyzhaozh.github.io/aloha/)：
  参考joint-position action、多相机和高频双臂数据；不采用其episode数量作为本mini要求。
- [RoboTwin](https://robotwin-platform.github.io/doc/tasks/)：
  参考head/left/right三视角和双臂任务组织。
- [RoboMIND 2.0](https://log2r.github.io/RoboMIND2.0/)：
  参考双臂、多视角和仿真/实体数字孪生分层。
- [DROID](https://droid-dataset.github.io/droid/the-droid-dataset)：
  参考多相机calibration、raw与训练载荷分离。
- [RH20T](https://rh20t.github.io/)：
  参考多频率、接触与标定事实；本版不要求其力/触觉规模。
- [DexMimicGen](https://dexmimicgen.github.io/) 与
  [Bi-DexHands](https://github.com/PKU-MARL/DexterousHands)：
  参考双手task/contact truth分类；本版不做数据扩增或success benchmark。

这些资料只支持格式、视角、时序和事实完整性设计，不把其他机器人的动作空间、标定或
success标准直接移植到NERO—Hand 2。

## 30. 已清零事项

计划落盘时已明确：

- scene D435i + 左右 D405；
- 双腕140°只用于当前仿真；
- RGB-only；
- 相机边界不做启动参数；
- 120/60/20与30 Hz离线视觉；
- 当前一次启动等于一个episode；
- 当前Ctrl+C结束；
- 视觉与control MCAP可分离但必须exact trace；
- 失败episode单步reject并可恢复；
- 不实现success；
- 不实现脚踏板/自动reset；
- raw contact不阻塞；
- 少于20条；
- Pi0.5只做兼容预埋，不做模型验收。

实施期间若出现会改变上述事实源、时序语义、q54动作含义或仿真/实体边界的新问题，必须先
更新ADR和本计划，不得在实现中隐式决定。
