# ROS2—Isaac 三目 q54 mini 数据集

## 当前状态

008 的 S0～S11 已实现并通过本机和 Workstation2 无设备验收。2026-08-06 又完成真实 ROS 图、
独立 GUI preview 和 performance governor 下的满负载 fixture 复验。当前可以解析固定 Deployment、
构建 ROS 接口、记录完整控制事实、离线生成 30 Hz 三目 RGB、执行 release gate、生成质量报告、
管理 accepted/rejected episode、封装 bundle，并用固定 LeRobot v0.6.1 导出不可变 revision。

确定性 A→B→A fixture 也已连续两次闭合到主 120/60 Hz、preview 20 Hz、完整 pose replay 与
active viewport RGB 像素，详见
[2026-08-06 验收记录](../validation/2026-08-06-deterministic-ros-isaac-gui-qualification.md)。
`-05` 已完成真实左侧抓取的诊断回放，但因右 Tracker provenance 和 clamp 不满足 hard gate，
不能作为 accepted episode。尚未完成的唯一采集门是完成一次真设备短 pilot，并录制第一条满足
主线 release gate 的新 episode。当前不实现脚踏板、自动 reset、连续多 episode、depth、reward
或 success evaluator。

## 固定身份和边界

- Deployment：`isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3`
- 数据集 profile：`isaac_nero_hand2_triview_q54_mini_dataset_v1`
- collection id：`isaac_nero_hand2_triview_q54_mini_dataset_v1`
- episode：一个 run 等于一个 episode；当前由一次启动和一次有序 Ctrl+C 结束
- raw：60 Hz ROS2/MCAP 控制事实
- policy grid：从 raw 精确选取相对偶数 control tick，30 Hz，不插值
- observation：动作前 q54 和同一动作前固定状态的三路 RGB
- action：下一 30 Hz anchor 的绝对 q54，单位 rad
- episode 结果：accepted、rejected 或 incomplete；没有 task success 语义
- 数量：总保留硬上限 18，实际目标少于 20 条

相机配置固定为 scene D435i 逻辑载体和左右腕 D405 逻辑载体，均为 640×480 RGB-only。
scene 使用 90° 合成投影，双腕使用 140° 合成广角。两者都标记为 simulation-only，不能复用为
未来实体 D435i/D405 的内参、畸变、曝光或标定。

## 实时执行架构

`gui:=true record:=true` 在 dataset Deployment 下自动拆成两个 Isaac 进程：

- headless control owner 使用 CPU `0-15`，独占 120 Hz physics、60 Hz control 和 raw facts；
- 被动 GUI preview 使用 CPU `16-27`，只按最新 post-action state 以 20 Hz render；
- Tracker/Glove source 和 rosbag wrapper/child 使用 CPU `28-31`；
- preview 没有控制 authority，不推进主仿真，不写 MCAP，也不作为训练图像来源；
- preview 使用独立的 `operator_preview/simulation_state`。它从第一个控制 tick 起工作，不再受
  双侧 dataset READY gate 阻塞；正式 `dataset/simulation_state` 仍只包含 candidate pre/post；
- preview 不再复制第二套 PhysX 或注入 q54；它仅 materialize USD 视觉资产，不创建 runtime
  articulation/rigid-body view，也不执行 `World.reset()`；主进程通过独立 topic 提供完整
  link/object world pose；preview 核验完整 70 link + 1 object inventory，只对其中 45 个拥有
  可见 render-purpose Gprim 的 pose owner 在停止的 timeline 上按 6.0.1 `EpisodeReplayer` 语义
  父先子后写入匿名 USD session sublayer；这里固定采用官方对嵌套 hierarchy 推荐的 `usd`
  backend，不使用可能产生 parent lag 的 `usdrt/fabric` benchmarking 路径；
- steady-state 每个状态只有一次 active-viewport Replicator transaction：附加轻量
  `ReferenceTime` annotator，调用 `orchestrator.step(rt_subframes=0, delta_time=0,
  wait_for_render=True)`，不读取 annotator payload、不推进 timeline/physics；
- 仅独立、无 physics 的 operator preview 启动时关闭 multi-tick rendering，以满足 20 Hz 零 miss；
  6.0.1 官方警告该设置不适合泛化，因此主 Isaac、正式 MCAP 和离线三目 renderer 都保留默认
  multi-tick 路径；
- GUI 路径采用最多 2 tick 的 bounded catch-up，但任何 accepted episode 仍要求零 missed period。

主线程只完成输入选择、控制、physics 和 simulator truth 冻结。link7、双掌和十个 fingertip
通过 PhysX articulation batch transform 读取；post snapshot 与下一 tick pre snapshot 必须在
q54、qdot、simulation time 和 physics boundary 上 exact 相等才允许复用。canonical JSON、
SHA-256、ROS message 构造和发布放在容量 4 的后台队列中；队列满、后台异常、少一帧或乱序都会
终止录制，不允许 drop 或降采样。Workstation2 在 CPU governor=`performance` 下又完成 1800
tick / 1783 candidate tick 的完整 fixture episode，主控制和 preview 均为零 miss，MCAP、
normalization 与 release gate 均通过。

随后两次确定性 GUI qualification 均完成 1080 control tick / 2160 physics step 且主链零 miss；
preview 分别为 20.014 Hz 与 20.011 Hz、零 miss，最大 render 时延 36.83 ms 与 38.65 ms，
pose 闭合最大误差均为 `1.20e-7 m`，A-return 重复图像像素差为 0。

## 主线必须录下的事实

主线只录事实，不在线计算质量指标或保存训练图像。每条 episode 必须同时保留：

- 左右 Tracker 原始 sample、设备身份、lifecycle、选择时刻和 source age；
- 左右 Glove 原始 21×3 landmarks、置信度、选择时刻和 source age；
- arm q7 candidate、reference/IK residual、拒绝或降级 reason；
- glove retarget 后的 q20、retarget model/config identity、confidence 和 reason；
- q7/q20 safety 前后结果、clamp/rate-limit reason；
- 每侧 pre-feedback q27、同相位 applied target q27、post-feedback q27；
- canonical q54 名称、顺序、source index、单位、限位，以及 pre-action q54、qdot54 backend
  readback 和 absolute q54 action；
- 每 tick 两个连续 120 Hz physics substep、60 Hz control timing、20 Hz GUI timing、RTF 和
  schedule facts；
- banana/bowl 等动态刚体真值、固定 fixture 清单、双臂 link7、双掌和十个 fingertip pose；
- episode ready/start/stop/closed 边界、recorder-ready、manifest、receipt 和 checksum。

因此 raw glove q21 和 retarget 后 q20 都必须保存；二者分别回答“人手传感器看到了什么”和
“机械手最终收到了什么”，不能互相替代。三路 RGB 不要求进入同一个 MCAP，但 vision artifact
必须以 episode_id、tick_id、phase、state digest、camera_id 和 checksum 回指同一 raw 单元。

## 固定状态离线渲染

渲染器复用 007 的相机装配和 USD optical frame，但不复用在线 CameraSensor tick gate。它先在
真值边界外完成三路 render product warm-up，随后 pause Isaac timeline，逐个注入 pre-action
q54/qdot54、动态物体和 link truth，并使用 Replicator 的
`delta_time=0.0, pause_timeline=True` 同步提交。

每次提交必须满足：

- 三路各恰好一个 completed RGB，reference time 三路一致且跨提交严格递增；
- submission 前后 simulation time 和 physics step index 完全不变；
- 渲染后的 q54、qdot54、刚体和 link truth 与注入状态闭合；
- 相机 K/D/R/P、parent→optical 和 world→camera 外参闭合；
- RGB 为 lossless PNG rgb8，逐帧 checksum，vision 目录原子发布；
- motion blur 禁用，renderer、lighting、color space、profile/config/asset hash 写入 provenance。

## 单条 episode 操作流程

在 Workstation2 的部署根目录执行。`RUN_ID` 必须唯一，例如
`mini-20260805-001`。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH=/home/lenovo/swy/wujihand_mount/src:/home/lenovo/swy/wujihand_mount/ros2/wujihand_ros2:$PYTHONPATH
ros2 launch wujihand_ros2 dual_teleoperation.launch.py \
  project_root:=/home/lenovo/swy/wujihand_mount \
  deployment:=configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml \
  local_runtime_binding:=configs/local/workstation2_nv5_ros_v2.yaml \
  gui:=true record:=true run_id:=<RUN_ID> isaac_cpu_affinity:=0-15
```

正式采集前确认 `/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor` 和代表 E-core 的
`cpu16` 都为 `performance`；若不是，先执行已验证的
`sudo cpupower frequency-set -g performance`。看到 `DATASET EPISODE READY` 后再动作；GUI 出现和
运动不等于 episode 已 READY。任一 Tracker 持续为 `calibrating` 时 Ctrl+C 会按设计生成
incomplete run。结束时按一次 Ctrl+C，等待 final complete tick、recorder closed 和 receipt
落盘。raw 根目录为：

```text
artifacts/runs/nv5/mini-dataset/<RUN_ID>
```

当前不自动 reset。下一条 episode 使用新的 run id 重新启动 Isaac。

## 离线处理顺序

以下示例把 registry 放在固定 collection 根。先注册 candidate 并添加任务文本，再依次产生
normalized、release、alignment、vision、quality 和 bundle；最后才 accept。

```bash
python3 tools/manage_mini_dataset_episode.py \
  --collection-root artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/collection \
  --collection-id isaac_nero_hand2_triview_q54_mini_dataset_v1 \
  register --episode-id <RUN_ID> \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID>

python3 tools/manage_mini_dataset_episode.py \
  --collection-root artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/collection \
  --collection-id isaac_nero_hand2_triview_q54_mini_dataset_v1 \
  annotate --episode-id <RUN_ID> --task "move toward banana and attempt grasp"

python3 tools/normalize_mini_dataset_run.py \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID>
python3 tools/validate_mini_dataset_release.py \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID>
python3 tools/build_mini_dataset_alignment.py \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID>

/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/render_mini_dataset_episode.py \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID>

python3 tools/build_mini_dataset_quality.py \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID>
python3 tools/build_mini_dataset_bundle.py \
  --run-root artifacts/runs/nv5/mini-dataset/<RUN_ID> \
  --collection-id isaac_nero_hand2_triview_q54_mini_dataset_v1

python3 tools/manage_mini_dataset_episode.py \
  --collection-root artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/collection \
  --collection-id isaac_nero_hand2_triview_q54_mini_dataset_v1 \
  accept --episode-id <RUN_ID> \
  --release-artifact-root artifacts/runs/nv5/mini-dataset/<RUN_ID>/derived/release
```

任一步失败都不得 accept。要从候选集合排除失败 episode，执行可恢复的 reject：

```bash
python3 tools/manage_mini_dataset_episode.py \
  --collection-root artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/collection \
  --collection-id isaac_nero_hand2_triview_q54_mini_dataset_v1 \
  reject --episode-id <RUN_ID> --reason "operator aborted"
```

`restore` 会恢复为需重新 gate 的 candidate。`purge` 需要 episode id 二次确认，仅进入隔离清理
流程；不要直接 `rm` raw run。

## 质量输出

`derived/quality/` 同时提供机器可读表和人工图：

- `joint_metrics.csv`、`group_metrics.csv`：q54 每关节和 arm/hand 分组范围、速度、跟踪误差；
- `input_age_metrics.csv`：Tracker/Glove age、缺失和 freshness；
- `object_metrics.csv`、`camera_metrics.csv`、`episode_metrics.csv`：物体、相机和 episode 完整性；
- `q54_groups.svg`、`tracking_error.svg`、`input_age.svg`、`camera_motion.svg`、
  `object_xy.svg`：关节、控制误差、输入延迟、相机和物体趋势；
- `vision_samples.html`：三视角抽样和遮挡/黑帧/左右镜像人工检查；
- `report.json`、`report.html`、manifest/checksum：完整 provenance 和确定性报告身份。

报告不产生 success rate。高成本 raw contact 允许缺失，但 q21→q20→q27/q54、时间因果、物理
步、对象/link truth、三路 RGB 和 provenance 是正式 accepted episode 的硬门。

## LeRobot revision

导出器只读取 registry 中已 accepted、bundle 已闭合且 release digest 未过期的 episode。依赖
固定在 LeRobot v0.6.1 commit `7e241bd630a3719a56157a497ce5d08f244784f1`；输出包含 30 Hz
三路图像、pre-action q54 observation、absolute q54 action、任务文本和 source-map sidecar，
不携带 raw q21、Tracker 或 privileged simulation truth。

具体环境和命令见
[`analysis/mini_dataset_export/README.md`](../../analysis/mini_dataset_export/README.md)。

## 真机前置条件

当前 q54/NERO 限位、零位、符号和 J7 解释只对已验证仿真 profile 成立。未来切到真机前必须按
机身铭牌/二维码、序列号、固件 readback 和安全限位逐台核对 q7，特别确认 J2/J7、零位和符号；
实体 D435i/D405 也必须建立独立 profile、设备身份和标定。不得把本组件的仿真 hash 或相机
参数直接视为真机资格证明。
