# 010 确定性 ROS2—Isaac—GUI 端到端 Qualification 计划

状态：开发、部署与 Workstation2 无设备验收完成；等待一次真设备短确认

日期：2026-08-06

前置：[008 ROS2—Isaac 三目 q54 mini 数据集开发计划](008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)

实施记录：[2026-08-06 确定性 ROS2—Isaac—GUI Qualification](validation/2026-08-06-deterministic-ros-isaac-gui-qualification.md)

## 0. 实施结论

- 已实现固定 A→B→A Tracker/Glove fixture、synthetic/ineligible 隔离、统一 validator 和 GUI
  像素闭环；无需 Base、Tracker、Glove 或操作者。
- Workstation2 连续两次完整运行通过：主 Isaac 1080 control tick / 2160 physics step、60/120 Hz
  零 miss；preview 20 Hz 零 miss；A/B 画面变化且相同 A 重复帧逐像素一致。
- preview 接收并核验 70 个 articulation link 和 1 个动态刚体 pose；只向 45 个拥有可见
  render-purpose Gprim 的 pose owner 写入 USD，避免对无视觉几何 prim 做无效 authoring。
- 当前停止在真设备穿戴前。qualification 不替代 SteamVR identity、workcell 标定、遮挡和手套
  佩戴确认。

## 1. 目标

建立一条不依赖 Base、Tracker、Glove 或操作者穿戴的自动化链路，稳定验证：

```text
固定 Tracker/Glove fixture
  → ROS2 输入选择与 provenance
  → 双臂 IK / 双手 retarget / safety command
  → Isaac 120 Hz physics 与 60 Hz q54 truth
  → 20 Hz operator preview
  → 活跃 GUI viewport RGB 像素
```

该 qualification 是正式采集前的回归门，不是 episode 数据。所有产物必须标记为
`source_mode=synthetic_fixture`、`dataset_eligible=false`，写入 diagnostics/qualification 目录，
禁止进入 mini 数据集 collection。

## 2. 已知问题与边界

- `-15` 已证明真实 Tracker 输入、ROS 控制、主 Isaac q54/link truth 会变化；GUI viewport 仍可能
  保持初始化画面。
- 旧 `-14` fixture 只检查消息、q54/link 数值和频率，没有检查 viewport 像素，并错误沿用了
  `live_teleoperation / dataset_eligible=true`。本需求修正这两个缺口。
- qualification 不证明 SteamVR 空间系、设备身份、遮挡、手套佩戴质量或真实动作语义；这些只保留
  一次最终真设备 E2E 确认。
- 不修改正式采集的 120/60/20 Hz、q54、MCAP topic、三目离线 RGB 或 release 规则。
- Isaac Sim 6.0.1 官方边界如下：`RenderingManager.render()` 可在不推进 physics 时刷新一次
  stage；Replicator `orchestrator.step(delta_time=0.0, wait_for_render=True)` 会等待一个精确完成的
  capture frame 且不推进 timeline；官方 `EpisodeReplayer` 对嵌套层级推荐父先子后的 `usd`
  backend，`usdrt/fabric` 仅保留给扁平场景 benchmark，可能出现 parent lag。实现不得混用未公开的
  `physxfabric.force_update()` 作为正式接口。
- 6.0.1 的 multi-tick rendering 默认开启。官方明确指出关闭
  `/rtx/hydra/supportMultiTickRate` 是复现 5.x render-every-frame 行为的兼容方式，同时警告多数
  6.0 代码路径未在关闭状态下验证。因此本项目只在无 physics、无数据相机、无 MCAP 写入的独立
  operator-preview 进程关闭它；主 Isaac 和离线三目 renderer 保持 6.0.1 默认值。
- 依据：[Physics Data Flow](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/physics/new_physics_engine.html)、
  [Rendering Manager](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/extensions/isaacsim.core.rendering_manager/docs/index.html)、
  [Replicator Orchestrator Step](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/replicator_tutorials/tutorial_replicator_sdg_workflows.html)、
  [Multi-Tick Rendering](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_multitick_rendering.html)、
  [Teleoperation Synthetic Data Generation](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/synthetic_data_generation/tutorial_replicator_teleop_sdg.html)。
  `XformPrim` 属于 6.0.1 experimental Core API，但也是本版本内置 `EpisodeReplayer` 的实际写入接口；
  因此本项目固定 6.0.1 并以黑盒 qualification 防止接口漂移。

## 3. 固定 fixture 轨迹

fixture 以 120 Hz 发布四路 canonical ROS 输入和一条 Tracker lifecycle，所有身份、序列、时间域、
置信度和 phase 都确定。轨迹采用 A→B→A，而不是随机波形。

| phase | 持续 | Tracker | Glove | 目的 |
|---|---:|---|---|---|
| discover | 最长 180 s | 不发布 | 不发布 | 等待 subscriber，并以匹配 run id 的 `recording/status=started` 作为 A/B/A 时钟屏障 |
| A-reference | 5 s | 双侧固定参考位姿 | 双侧固定张开手 skeleton | 建立 reference、READY，并让状态滤波收敛到稳定基线 |
| B-motion | 5 s | 左右采用镜像、可达、无 clamp 的平移/小旋转 | 双侧固定明显屈曲 skeleton | 形成明显不同的 q7/q20 与 GUI 图像 |
| A-return | 5 s | 精确回到 A | 精确回到 A | 让状态滤波重新收敛，并检查可重复性与静止像素确定性 |

具体输入常量在 fixture 模块中只定义一次，并在 receipt 中完整写出 SHA256。不得通过正式 deployment
启动参数修改状态。B 的 Tracker 位移必须小于 mapping clamp；B 的手 skeleton 必须先在固定 Wuji
SDK 版本上证明 q20 与 A 有显著差异。若 IK 不可达或 safety clamp，调整 checked-in fixture 常量并
重新审核，不能放宽验收门。

## 4. 代码结构

### 4.1 fixture source

扩展 `ros2/wujihand_ros2/test/fixture_sources.py`：

- 支持固定 A→B→A qualification profile；
- 每个 sample 使用严格递增 sequence 与 host monotonic time；
- producer、epoch、source/calibration/device identity 必须与 resolved route 一致；
- 输出机器可读 fixture receipt，包含 profile id/hash、phase 边界、发布数、实际频率与 miss；
- fixture 完成时正常发布 STOPPED lifecycle，不以 kill 模拟结束。

### 4.2 运行隔离

现有 launch 增加仅供 qualification 使用的显式模式：

- 不启动真实 Vive/Glove source；
- 启动 fixture source、headless control owner、passive GUI preview 和 rosbag recorder；
- consumer 明确使用 `DatasetSourceMode.SYNTHETIC_FIXTURE`；
- run id 和输出根必须属于 qualification；
- 固定 CPU：control `0-15`、preview `16-27`、fixture/recorder `28-31`；
- 任一子进程非零退出均使整次 qualification 失败。

正式模式默认值及命令不变；qualification 标志不能和无限时长正式录制混用。

### 4.3 preview 可见性验收

`run_isaac_dataset_live_preview.py` 保持被动、无控制权、不写 MCAP，并把 viewport 验收改为确定性事务：

1. 主 Isaac 在不录入 MCAP 的 operator-preview topic 中发送 post-action q54、物体 truth 和完整
   articulation link truth；该 topic 使用独立的 `OperatorPreviewStateFrame` 有界传输
   （最多 128 个 link），不得复用正式 `SimulationStateFrame` 的 32-link DDS 上限；正式 MCAP
   schema、topic 与计划规定的 14 个 privileged link 均保持不变；
2. 主 Isaac 的 PhysX q54/link truth 由 MCAP 与主 receipt 验证；preview 使用
   `visual_replay_only` materialization：只 author USD 资产、装配与视觉，不创建 Articulation/
   RigidPrim runtime view、不执行 `World.reset()`，因此不存在第二套 PhysX；preview 也不注入
   q54 Tensor 或调用 articulation kinematic update；
3. preview 保持 Kit timeline 停止；完整接收并验证同一帧 70 link + 1 object world pose，只选择
   其中拥有可见 render-purpose Gprim 的 45 个 pose owner，按 6.0.1 `EpisodeReplayer` 方式写入
   匿名 session sublayer，并严格父先子后。采用官方对嵌套 hierarchy 推荐的 `usd` pose backend；
   不使用仅建议扁平场景 benchmarking 的 `usdrt/fabric` 路径，也不依赖
   `GLOBAL_EVENT_UPDATE` 回调先后顺序；
4. timeline 停止后只允许一次 `RenderingManager.render()` 做初始化刷新。随后 steady-state 与
   terminal A/B/A capture 均在 active viewport render product 上附加轻量 `ReferenceTime`
   annotator，并对每个状态只调用一次
   `rep.orchestrator.step(rt_subframes=0, pause_timeline=False, delta_time=0.0,
   wait_for_render=True)`；不读取 annotator payload，不推进 timeline/physics；
5. 仅该独立 `visual_replay_only` preview 进程使用
   `--/rtx/hydra/supportMultiTickRate=false`，以消除 stopped-timeline 下每次 capture 的额外 app
   update；主控制进程、正式 MCAP 和离线三相机渲染不继承此设置；
6. 保存相距明显的 A/B source frame；
7. terminal 后在 physics time/index 不变的前提下，依次渲染并捕获 `A1、B、A2`；其中
   `A2/A2-repeat` 在同一次 active viewport render reference 上完成两次独立 RGB readback；
8. `A1↔B` 必须达到最小物质像素变化；
9. `A2↔A2-repeat` 必须逐像素一致；
10. 主链逐 tick 验证 q54 与 PhysX link truth；preview 对首次状态及最终 `A/B/A` capture 状态验证
   replay backend world pose，位置误差 `<2e-5 m`；
11. capture 必须来自 active viewport LDR RGB，不能用 X11 窗口截图代替。

同步实现属于被测对象。任何 USD/Hydra 修复只有在上述黑盒门全部通过后才可保留；
临时 print、截图 hash 猜测或单层 readback 不能作为修复结论。

### 4.4 统一 validator

新增单命令 validator，读取 fixture receipt、主 receipt、preview receipt 和 MCAP，只输出一个总 receipt。
它必须覆盖：

| 层 | 硬门 |
|---|---|
| fixture | A/B/A phase 完整；四路发布数一致；120 Hz 无 miss；identity 与 route binding 一致 |
| ROS 输入 | 四路 source topic 与 lifecycle 均存在；sequence 严增；source age 合法 |
| provenance | active source 为 fixture producer/epoch；左右 source 不串线；READY 后 candidate 无缺口 |
| command | 左右 arm q7、hand q20 均有 A/B plateau；B-A 最大差超过固定下限；无 clamp/hold/reject |
| q54 | command partition 与 post-action q54 对齐；54 维 inventory/hash 正确；A/B 差异覆盖四组 |
| PhysX/link | 主 Isaac q54 readback 闭环；MCAP 14-link truth 完整；preview 全 link truth 完整；pose-backend replay 位置误差 `<2e-5 m`；preview 不拥有第二套 physics |
| GUI | A/B viewport 变化达标；同状态重复 RGB SHA256 完全一致；capture 尺寸/profile 正确 |
| 性能 | physics/control/preview 分别 120/60/20 Hz；main 与 preview missed period 都为 0 |
| 隔离 | synthetic fixture、dataset ineligible、operator preview 使用独立消息且不在 MCAP、产物不在 collection |

数值门先使用物理意义明确的最小变化与闭环误差；首个通过的 Workstation2 结果再生成 reviewed golden
摘要，固定 SDK/config/asset/session hash。后续回归既比较硬门，也比较 golden plateau，禁止因环境漂移
静默重建 golden。

## 5. 自动化测试分层

1. 快速单元测试：fixture phase、identity、A/B skeleton 差异、像素差、q54 partition、receipt fail-closed。
2. ROS contract 测试：qualification 模式不启动真实设备，source mode 永远 synthetic/ineligible。
3. Workstation2 GPU qualification：完整四进程运行和 MCAP validator；无需设备上电。
4. 重复性：连续运行两次，A/B command plateau、q54 和截图 hash 在规定容差内一致。
5. 负例：故意冻结 preview、错 producer、交换左右或制造一个 missed period 时 validator 必须失败。

## 6. 开发顺序

1. [完成] 撤回未纳入本计划的临时同步探针，保留通用像素比较能力。
2. [完成] 实现 fixture A→B→A profile、receipt 与单元测试。
3. [完成] 接通隔离 qualification launch/source mode，确认不会打开真实设备。
4. [完成] 完成 MCAP/provenance/q54/频率 validator。
5. [完成] 重构 preview capture 为 A/B/A-repeat 确定性事务。
6. [完成] 在 Workstation2 根据黑盒门修复 render-only 同步。
7. [完成] 连续两次全通过并保存验证记录。
8. [完成] 更新 008 验收状态；最后只保留一次真实设备短动作确认。

## 7. 完成标准

- 一个命令可在设备断电时启动、运行、结束并返回 0；
- 两次连续 qualification 的所有硬门通过；
- GUI 画面在 B 明显变化，A 重复帧逐像素一致；
- 主 60 Hz 与 preview 20 Hz 均零 miss，不放宽现有验收；
- fixture 产物无法被 release/collection 接受；
- 删除或改坏任一关键层时测试 fail-closed；
- 文档给出最终命令、产物路径和真实设备剩余验收动作。
