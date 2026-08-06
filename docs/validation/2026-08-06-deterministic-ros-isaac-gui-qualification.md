# 2026-08-06 确定性 ROS2—Isaac—GUI Qualification

## 结论

状态：通过。

Workstation2 在 Base、Tracker、Glove 均不参与的条件下，连续两次完成：

```text
A→B→A fixture
  → ROS2 source selection / provenance
  → 双臂 IK、双手 retarget、safety command
  → Isaac q54、120 Hz physics、60 Hz control
  → 完整 link/object pose transport
  → 20 Hz passive operator preview
  → active viewport RGB 像素
```

所有产物均为 `source_mode=synthetic_fixture`、`dataset_eligible=false`，不允许进入正式 mini
dataset collection。本记录证明软件链和 GUI 回归门，不证明 SteamVR 空间系、真实设备身份、遮挡、
Glove 佩戴质量或真实任务语义。

## 固定实现

- fixture：120 Hz、A-reference 5 s、B-motion 5 s、A-return 5 s；四路输入 identity、sequence、
  epoch 和 provenance 固定。
- 主进程：独占 PhysX，120 Hz physics / 60 Hz control；完整写 MCAP；operator preview topic 不进
  MCAP。
- preview：`visual_replay_only`，不创建 articulation/runtime physics，不注入 q54，不执行
  `World.reset()`。
- pose：每帧运输并验证 70 个 articulation link + 1 个动态刚体；只 author 45 个拥有可见
  render-purpose Gprim 的 pose owner；匿名 session sublayer、USD backend、父先子后。
- render：timeline 停止后仅一次 `RenderingManager.render()` 做初始化刷新；稳定态和 terminal
  A/B/A 均为一次 active-viewport Replicator step，参数为 `rt_subframes=0`、`delta_time=0.0`、
  `wait_for_render=True`。`ReferenceTime` 只用于激活对应 render graph，不读取 payload。
- multi-tick：仅该独立 preview 进程使用
  `--/rtx/hydra/supportMultiTickRate=false`；主 Isaac 和离线三目 renderer 保留默认值。

## 官方 6.0.1 依据

- [Replicator Orchestrator Step](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/replicator_tutorials/tutorial_replicator_sdg_workflows.html)：
  `delta_time=0.0` 不推进 timeline，`wait_for_render=True` 等待该帧完成。
- [Teleoperation Episode Replay](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/synthetic_data_generation/tutorial_replicator_teleop_sdg.html)：
  嵌套 hierarchy 推荐父先子后的 USD backend；USDRT/Fabric 可能出现 parent lag。
- [Rendering Manager](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/extensions/isaacsim.core.rendering_manager/docs/index.html)：
  `RenderingManager.render()` 只刷新 render，不推进 physics。
- [Multi-Tick Rendering](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_multitick_rendering.html)：
  关闭 `supportMultiTickRate` 会恢复逐 app-frame render；官方同时警告 6.0 多数其他路径未在关闭
  状态下验证，因此本项目严格限制在独立 visual-only preview。

## 验收结果

| 指标 | `dataset-preview-qual-20260806-26` | `dataset-preview-qual-20260806-27` | 门 |
|---|---:|---:|---:|
| overall / failures | pass / 0 | pass / 0 | pass / 0 |
| control tick / physics step | 1080 / 2160 | 1080 / 2160 | 1:2 |
| main missed control period | 0 | 0 | 0 |
| preview effective rate | 20.014 Hz | 20.011 Hz | 20 Hz ±5% |
| preview missed period | 0 | 0 | 0 |
| render mean / max | 16.08 / 36.83 ms | 17.13 / 38.65 ms | 无 50 ms miss |
| pose apply mean / max | 9.44 / 11.42 ms | 9.75 / 11.37 ms | 50 ms budget 内 |
| q54 command closure max | 0 rad | 0 rad | `<2e-5` |
| pose position closure max | `1.192e-7 m` | `1.192e-7 m` | `<2e-5 m` |
| A/B changed pixel fraction | 0.007639 | 0.006708 | `≥0.0005` |
| A/B pixel max delta | 208 | 201 | `>0` |
| A-repeat pixel max delta | 0 | 0 | 0 |
| visible geometry matrix max delta | 1.180 | 1.190 | `≥1e-4` |
| slow render events | 0 | 0 | 0 |

两次运行均核验四组 q54 有明显 A/B 变化、A-return 收敛、MCAP provenance 完整、正式 14-link
truth 完整、operator-preview topic 未录入 MCAP，以及 qualification artifact 不具备 dataset 资格。

软件回归结果：本机 `tests/unit + tests/contract` 为 855 passed、3 skipped；3 个 skip 分别是本机
未安装 MuJoCo 和未恢复可选的 `wuji-description` source-lock fixture，与本需求无关。Workstation2
使用已部署 analyzer 源码完成 `analysis/teleoperation_quality/tests`，22 passed。

## 已撤回的无效方向

- steady-state 仅用 `RenderingManager.render()`：GUI 无像素变化，已删除；只保留 timeline stop 后
  一次初始化刷新。
- warm-up 后 detach annotator：GUI 无像素变化，已删除。
- 每帧读取 `ReferenceTime.get_data()`：不能改善时延且 stopped timeline 下无有效增量，已删除。
- 私有 `physxfabric.force_update()`、`useFabricSceneDelegate=false`、`GLOBAL_EVENT_UPDATE`、
  `carb.eventdispatcher`、USDRT/Fabric hierarchy readback、preview 第二套 q54/PhysX 注入：均不在最终
  运行路径；contract test 防止重新引入。

保留的 `slow_render_events` 是正式结构化 receipt telemetry，不是临时 debug print。

## 复验命令

```bash
source /opt/ros/jazzy/setup.bash
source /home/lenovo/swy/wujihand_mount/install/setup.bash
cd /home/lenovo/swy/wujihand_mount
DISPLAY=:0 /home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/qualify_dataset_preview_e2e.py --run-id <UNIQUE_RUN_ID>
```

输出位于：

```text
artifacts/diagnostics/dataset-preview-qualification/<RUN_ID>
```

总结果读取 `qualification/receipt.json`。任何单门失败均返回非零；不得把失败结果移入 collection。

## 剩余真设备短确认

自动验收已完成并在此停止。下一步需要操作者穿戴后做一次 15～20 秒短 pilot：先中立静止 3～5 秒，
再做左右臂可见小幅位移和双手张合 8～10 秒，最后静止 3 秒并 Ctrl+C。验收要求：真实 producer/
device identity 正确、无 candidate provenance 缺口、无 clamp/hold/reject、主 60 Hz 和 preview
20 Hz 均零 miss、GUI 随 q54 明显运动且有序关闭。该 pilot 通过后才开始正式 episode 采集。
