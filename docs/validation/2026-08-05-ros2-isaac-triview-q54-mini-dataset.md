# 2026-08-05 ROS2—Isaac 三目 q54 mini 数据集无设备验收

## 结论

008 的开发与无设备验收通过。ROS2 接口、dataset Deployment/Profile、完整 raw facts、严格
120/60 Hz 录制、隔离 20 Hz GUI preview、30 Hz 因果对齐、固定状态三目 RGB、
release/quality/bundle/registry 和固定 LeRobot 导出链均已落盘。

真实遥操作 `-05` 已用于诊断回放，但它缺少右 Tracker 的完整 active provenance，且存在大量
clamp，因此不能进入正式 release。当前不宣称正式数据采集完成；下一步仍需操作者重新上电和
穿戴后录制一条满足主线 hard gate 的新 episode。

## 本机回归

| 检查 | 结果 |
|---|---|
| root pytest | 848 passed，4 skipped，11 deselected |
| teleoperation_quality | 22 passed |
| mini_dataset_export | 4 passed，包含固定 LeRobot 的 finalize/reopen/解码 round-trip |
| root Ruff / mypy | PASS；mypy 149 source files；Isaac runner 单独 strict PASS |
| analyzer Ruff / mypy | PASS；mypy 12 source files |
| exporter Ruff / mypy | PASS；mypy 3 source files |
| `git diff --check` | PASS |

四个 skip 均为本机未安装 MuJoCo或未恢复上游 Hand 2 source asset，不属于 008 路径。

## Workstation2 隔离验收

为避免覆盖既有部署，资格验证在隔离目录执行：

```text
/home/lenovo/swy/wujihand_008_20260805.tcr6Ak
```

- ROS2 Jazzy `colcon build --base-paths ros2 --symlink-install`：2 packages 完成；
- `colcon test` / `colcon test-result --verbose`：4 tests，0 failures；
- 008 device-free Python 选择集：96 passed；
- Isaac Sim 6.0.1 headless、RTX 5090：三目固定状态 probe PASS；
- probe 日志：
  `/home/lenovo/swy/wujihand_008_20260805.tcr6Ak/validation_evidence/renderer-qualification.log`。

Isaac probe 的关键结果：

- Deployment/profile/q54 身份解析通过；q54 profile SHA-256 为
  `a96db8d2542b684b5c6a15cfa17eb44c23da02a812b1d47fb88a03df8fc20b77`；
- 三路均为 640×480 rgb8 PNG，payload checksum 两两不同；
- scene D435i 合成投影读回 HFOV 90°，左右 D405 合成投影读回 HFOV 约 140°；
- K/D/R/P、parent→optical、world→camera 和左右静态 mount/camera/report hash 完整；
- source state digest 获得 exact acknowledgement；
- fixed-state capture 前后 physics step index 均为 2，未推进 physics；
- motion blur 关闭，三路 completed reference 同步，render product provenance 落盘。

## 正式目录部署复验

隔离验收通过后，008 overlay 已部署到：

```text
/home/lenovo/swy/wujihand_mount
```

同步未删除远端文件，并显式排除 `configs/local/`，因此 Workstation2 设备绑定保持原样。部署后
再次完成 ROS2 build、4/4 ROS tests 和 96/96 device-free tests。正式目录最初缺少 source-lock
中的 RoboLab tree；已从同机 NV4 的同 commit 缓存以 hard link 补齐，`banana_bowl.usda`
SHA-256 与 lock 均为
`caf59864ffd777e53fb14e88e5620e19a967bbaa8d448bf98a8fdd41cb8832a0`。

正式目录 Isaac probe 随后再次 PASS，证据位于：

```text
/home/lenovo/swy/wujihand_mount/artifacts/validation/2026-08-05-008-renderer-qualification.log
```

scene D435i profile 仍保留 `projection marker` 资格项；本 probe 完成 API readback、非黑/非重复
图像和外参闭合，不把合成相机解释为实体 D435i 标定。

## 60 Hz 录制与隔离 GUI 复验

正式目录采用两个 Isaac 进程：headless control owner 固定在 CPU `0-15`，被动 GUI preview
固定在 `16-27`，Tracker/Glove source 和 rosbag 固定在 `28-31`。preview 只订阅独立的
`operator_preview/simulation_state`，按 post-action q54/scene truth 以 20 Hz 显示，不拥有控制权、
不推进主仿真，也不写 MCAP 或数据图像。正式 `dataset/simulation_state` 仍只含 candidate
pre/post。主进程每 tick 读取 PhysX batch link truth，冻结不可变
pre/post snapshot；JSON、digest、ROS message 构造和发布由有界后台 worker 完成。队列容量为 4，
任何满队列或后台异常均 fail closed，不允许 drop。

GUI/recording 路径最多允许连续 catch up 2 个 tick 来吸收瞬时抖动，但 release gate 仍要求
`scheduler.missed_control_periods == 0`。`-18` 出现 1 次 miss，仅保留为调优证据，不计入通过。
同一最终版本随后连续两次通过：

| run | control | effective Hz | physics | main miss | worker queue max | preview Hz / miss | link max error |
|---|---:|---:|---:|---:|---:|---:|---:|
| `mini-synth60-splitpreview-deferredall-20260805-19` | 600 | 60.00024 | 1200 | 0 | 1 | 19.99899 / 0 | `1.19e-7 m` |
| `mini-synth60-splitpreview-deferredall-20260805-20` | 600 | 60.00011 | 1200 | 0 | 1 | 19.99925 / 0 | `1.49e-8 m` |

两次均满足：consumer/recorder/preview exit code 为 0、receipt `complete`、MCAP finalize 完成、
每个 control tick 有左右两条 runtime trace、所有 candidate tick 有成对 pre/post state、q54
preview closure 为 0、无消息 drop。最终代码在 Workstation2 另完成 23 个相关 device-free
测试。

## 2026-08-06 真实 ROS 图与 performance 复验

真实设备上电但未完整穿戴时，发现旧 preview 只消费正式 candidate state：右 Tracker 仍为
`calibrating` 会阻止双侧 READY，左侧控制虽然已在主仿真生效，GUI 仍保持中立。现已把 operator
preview 与 dataset candidate 状态拆开：preview 从第一个完整 control tick 起按 20 Hz 接收，
该 topic 不进入 rosbag allowlist；dataset lifecycle、candidate pre/post 和 hard gate 均未放宽。

同时固定进程隔离并撤回无收益的 8-thread 实验。操作者已把所有 CPU governor 设置为
`performance`。关键对照如下：

| run | 场景 | control / candidate | main miss | preview applied / Hz / miss | 结果 |
|---|---|---:|---:|---:|---|
| `mini-smoke-affinity-20260806-10` | 真实 source，旧 governor | 2505 / 2487 | 7 | 832 / 20.00046 / 0 | complete，性能调优证据 |
| `mini-smoke-affinity-8threads-20260806-11` | 8-thread 实验 | 2793 / 2776 | 10 | 闭合通过 | 实验撤回 |
| `mini-smoke-affinity-performance-20260806-12` | 未穿戴，旧 preview 耦合 | 4759 / 0 | 0 | 0 | 右 Tracker calibrating，按设计 incomplete |
| `mini-smoke-preready-preview-20260806-13` | 未穿戴，独立 preview | 2990 / 0 | 0 | 995 / 19.99962 / 0 | GUI 链通过；未 READY，按设计 incomplete |
| `mini-fixture-performance-20260806-14` | 双侧 deterministic ROS fixture | 1800 / 1783 | 0 | 600 / 20.00029 / 0 | complete |

`-13` 在 episode 未 READY 时仍发布并应用 995 个 operator state，q54 变化范围 1.1793 rad，q54
注入误差为 0，link 最大位置误差为 `1.1920929e-7 m`；MCAP 中 operator preview topic 不存在。
这证明 GUI 不再被另一侧 reference gate 冻结。它的 incomplete 原因仅为右 Tracker 持续
`calibrating`，不是 GUI 或录制异常。

`-14` 使用仓库现有 fixture 替代穿戴设备，覆盖 READY 后的真实 MCAP 满负载：1800 个 60 Hz
control tick、3600 个 120 Hz physics step、1783 个 candidate tick、3566 个 candidate state，
publisher queue 最大深度 1，主控制与 preview 均零 miss。preview q54 误差为 0、link 最大误差
为 `1.49e-8 m`；consumer、preview、recorder 均完整闭合。随后 normalizer 读取 1783 tick 通过，
结构/因果 release gate 通过。该 fixture 只证明链路和性能，不替代真实手套手型、Tracker 空间
映射、动作质量和工作台任务验收。

### GUI viewport 静止：旧结论作废

操作者随后在真实 `mini-diag-dual-banana-20260806-13/-14` 中发现窗口肉眼完全静止。两次
preview receipt 实际分别接收 462/298 帧并应用 462/287 帧，q54 最大变化 1.90/1.52 rad；
MCAP 的正式 post-action state 进一步确认 `-14` 左臂 q7 最大变化 0.83 rad、右臂 0.52 rad。
这些数据只能证明 tracker→ROS→主 Isaac truth 已变化，不能证明 GUI viewport 已变化。此前把
`update_transformations(True, True, True)` 与 X11 窗口截图认定为修复的结论，已被后续
`-15` 与确定性 active-viewport 像素检查否定；对应实现不再保留。

正式收敛改由 [010 确定性 ROS2—Isaac—GUI 端到端 Qualification](../010-deterministic-ros-isaac-gui-e2e-qualification-plan.md)
完成：设备无关 A/B/A fixture、完整 link truth、Kit app-update 内的官方 USD pose replay、active
viewport RGB 差分与同状态逐像素重复性共同构成硬门。此处不再以窗口截图 hash 或 tensor/link
闭环代替 viewport 证据。

## `-05` 诊断回放

源 run：

```text
/home/lenovo/swy/wujihand_mount/artifacts/runs/nv5/mini-dataset/mini-diag-left-banana-20260805-05
```

只为观察历史真实轨迹，选择了靠近、首次位移、抬升、转移、碗区和放置后等 14 个关键状态。
诊断工具仅在该 run 内绕过右侧 provenance 和 clamp release gate；主线没有对应旁路。最终
tensor readback 回放产物为：

```text
/home/lenovo/swy/wujihand_mount/artifacts/diagnostics/
mini-diag-left-banana-20260805-05-triview-p0-tensor-v2
```

结果：

- 注入 q54 后执行 articulation kinematic update，再做一次不推进 physics 的 render；
- 14 个状态生成 42 张 RGB，三相机每状态共用同一 completed reference；
- 每状态只发生一次 render transaction，reference 跨状态严格递增；
- replay 时间为 `dataset_frame_index / 30`，原始 simulation time 和固定 physics origin `302`
  只保存在 provenance 中；
- q54 最大误差为 0，link 位置最大误差为 `1.1920929e-7 m`，严格小于 `2e-5 m`；
- 场景视角能看到香蕉抬升、向碗转移和放置，左腕视角能看到实际接触区域。

该结果证明 replay P0 和三视角观察链可用，不改变 `-05` 的
`diagnostic_only=true`、`formal_release_passed=false`。其明确 blocker 是 136 个 candidate tick
缺少右 Tracker active provenance；大量 clamp 同样不允许在正式主线绕过。

## 验收边界

| S12 项 | 状态 |
|---|---|
| device-free synthetic golden | PASS |
| 隔离 GUI 下 120/60/20 性能 | PASS；performance 模式满负载 `-14` 零 miss |
| pre-ready GUI 状态更新 | PASS；`-13` 995 帧，未写 MCAP |
| 左侧 Tracker/Glove GUI pilot | PASS；双侧真实完整 pilot 仍待新录制 |
| 单条真实遥操作诊断 episode | `-05` 已完成；release-ineligible |
| `-05` 离线三目 render | PASS；14 状态 / 42 RGB，仅诊断 |
| Workstation2 上真实 collection LeRobot round-trip | 待新的 accepted episode；本机固定依赖 round-trip 已 PASS |
| 真实 episode reject/restore 演练 | 待真实 episode |
| 少于 20 条正式采集批准 | 未开始 |

下一步只需要一次双侧短 pilot/新 episode；不再需要修改 schema、replay 或离线架构。当前明确
停在需要操作者穿戴和动作的步骤前。正式开始前必须确认左右 Tracker 均为 `RUNNING`，不能有一侧
持续 `calibrating`。新 episode 若暴露设备身份、source
age、左右 mapping、60/120 Hz、相机遮挡或控制安全问题，必须先修复并重新 gate，再开始正式
少量 episode 采集。
