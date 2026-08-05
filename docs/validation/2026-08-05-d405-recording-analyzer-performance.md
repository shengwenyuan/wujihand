# D405 双腕相机录制、Analyzer 与 60 Hz 性能验证

- 日期：2026-08-05
- 主机：Workstation2 / `lenovo-piper2`
- Isaac：6.0.1，headless，RTX 5090
- 范围：007 计划 S7；纯仿真 `140°` D405，不代表实体 RealSense 参数或标定

## 结论

双侧 `640×480 @ 30 Hz` RGB、optical-Z depth、CameraInfo、frame truth、TF 与
manifest/receipt 可完整录制并离线验证。相机采用
`paused_post_control_replay_v1`：在线 120/60 Hz 控制段只保存 30 Hz 场景状态，控制段结束后
再渲染并写入同一 artifact；该设计只适用于确定性仿真采集，不能解释为实时相机或实机路径。

正式 30 秒 P-core 试验通过既有性能 Gate：

- control：`59.999965 Hz`
- tick interval P95：`16.725941 ms`，Gate `<= 20 ms`
- physics RTF：`1.000317`
- schedule miss：`0 / 1800`
- physics：`3600` 个连续 substep
- camera：左右各 `900` 帧，`29.9999999997 Hz`
- MCAP：`3,884,618,477 bytes`，按 30 秒仿真窗口为 `129.49 MB/s`
- RGB/depth/CameraInfo/truth：每侧均为 `900/900/900/900`
- dynamic/static TF：`1800/2` 条，外参、左右 identity 和标定 provenance Gate 全通过

## 正式产物

- record-off 对照：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s7_record_off_pcores_1800.json`
- record-on：
  `/home/lenovo/swy/wujihand_mount/artifacts/runs/nv5/`
  `s7-d405-accepted-pcores-1800-20260805as`
- analyzer `0.3.0`：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s7_analyzer_accepted_as_v030`
- 抽检图像：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s7_visual_samples_as`

record-off 同样完成 `1800` control tick、`3600` physics substep、零 schedule miss，且
`capture_enabled=false`、render product 数为 `0`。record-on 在线控制段复用 post-physics q27，
30 Hz 回放快照平均约 `27.6 us`、最大 `76.4 us`；相机 render 与 ROS 图像发布均发生在控制段
之后。

## 时间与完整性合同

`SimulationCameraFrameTruth` 使用 v2 schema：

- ROS Image、CameraInfo、TF 与 truth 的 `stamp_ns` 来自确定性 30 Hz 有理数调度网格；
- RTX 原始 completed-frame rational identity 独立保存在 `reference_time_*` 与
  `completed_frame_identity`；
- 两者允许的浮点漂移上限为 `1 us`，超限、倒序、缺帧、重复帧或左右不一致均 fail closed；
- replay reset 后先做一次不公开的 priming render，正式帧仍从 control tick `1` 开始。

长程试验确认 Isaac 原始 reference 偶尔产生 `33,333,332 ns` 间隔，因此不能直接作为
ROS 的 30 Hz 调度时钟；保留原始 identity 并另用确定性 stamp，避免隐藏仿真事实。

## Analyzer 说明

Analyzer `0.3.0` 对相机、TF 和性能相关 Gate 全部通过。该试验使用 arm-only deployment，
且有意只 configure、未 activate Vive source，所以 tracker/lifecycle 与 scene topic 为空；因此
全局 `structural_gates_passed` 和 `planned_targets_passed` 为 `false`。这不属于相机或性能失败，
也不应被误写为具备真实输入质量结论。

## 复现命令

```bash
ros2 launch wujihand_ros2 dual_teleoperation.launch.py \
  project_root:=/home/lenovo/swy/wujihand_mount \
  deployment:=configs/deployments/isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml \
  local_runtime_binding:=configs/local/workstation2_nv5_ros_v2.yaml \
  gui:=false \
  frames:=1800 \
  record:=true \
  isaac_cpu_affinity:=0-15 \
  run_id:=replace-with-unique-run-id

python tools/analysis/analyze_teleoperation_run.py \
  --run-root /path/to/complete-run \
  --output-root /path/to/new-analysis-directory
```

P-core 亲和是当前 Workstation2 正式性能资格条件；无亲和短测只用于功能诊断，不能替代
005/007 的性能 Gate。
