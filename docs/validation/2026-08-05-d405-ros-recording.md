# 双侧合成 D405 ROS2 录制资格验证

- 日期：2026-08-05
- 环境：workstation2，Isaac Sim 6.0.1，ROS 2 Jazzy，RTX 5090
- 结论：007 S6 通过
- 运行 ID：`s6-d405-smoke-20260805m`

## record=false

使用默认 D405 ROS Deployment 完成 8 tick headless 运行。左右 wrist rig、Camera prim、
visual/collision 均存在；report 明确记录 `capture_enabled=false`、数据 render product 为 0、
camera publisher 为 0。此路径没有 30 Hz 数据渲染、相机 topic 或 rosbag 负载。

报告：
`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s6_record_off/report.json`。

## record=true

同一 Deployment 的有界运行完成 8 个 control tick、16 个 physics substep，并正常关闭 Isaac
consumer 与 rosbag wrapper。consumer receipt 和最终 recording receipt 均为 `complete`；
manifest、recorder、receipt、metadata 与 MCAP 的 SHA-256 全部通过。

左右相机各 capture/publish 4 帧，frame index 均为 `0..3`；首末 stamp 为
`933333333/1033333333 ns`。逐帧 truth 对应 control tick `1,3,5,7` 和 physics substep
`3,7,11,15`，符合 120/60/30 Hz 调度。关闭阶段各侧仅补齐 1 个 in-flight frame，未推进
simulation time。

MCAP 共 93 条消息、16.6 MiB。左右每侧均包含：

- 4 条 `rgb8 640x480`；
- 4 条 `32FC1 640x480` optical-Z depth；
- 4 条 CameraInfo；
- 4 条 SimulationCameraFrameTruth。

离线逐条反序列化验证了每个 `(side, stamp, optical_frame_id)` 恰好一份
RGB/depth/info/truth，无 gap、duplicate 或跨帧配对；K/D/R/P shape、finite 和分辨率一致。
TF 仅包含每帧一对 `world -> hand_base` dynamic edge，以及每侧一条
`hand_base -> optical` static edge；没有 direct `world -> optical`。

运行 artifact：
`/home/lenovo/swy/wujihand_mount/artifacts/runs/nv5/s6-d405-smoke-20260805m/`。
逐帧检查报告和左右首帧：
`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s6_record_on_mcap/`。

## 相机与外参事实

manifest 的 RTX Camera readback 为 `640x480 @ 30 Hz`、pinhole、
`139.999999°` HFOV，左右 K/P 相同，D 为仿真 authored zero。逐帧 truth 同时保存
`world_from_hand_base`、`world_from_camera_optical` 和固定
`hand_base_from_camera_optical`；运行时已验证三者闭合。

Replicator rational `reference_time` 与 float simulation time 的纳秒舍入可相差数十纳秒。
实现以 rational identity 作为 frame stamp，并只允许在显式 `1000 ns` 容差内关联 post-step
pose；超出立即失败，不做无界最近邻或错误跨帧关联。

该 `140°` 光学配置是纯仿真特殊镜头，不是实体 RealSense D405 规格或标定，也没有预埋
任何实体相机路径。

## 已知非阻断提示

Headless 验证没有 SteamVR，VIVE lifecycle activation 因此失败；consumer 以既有无输入安全
保持模式完成，recorder 已在启动前订阅完整 topic allowlist，且 camera 录制与 finalization
均闭合。Hand 2 OmniPBR `texture_wrap_u/v`、低分辨率 DLSS 和 host-buffer copy 提示也未导致
物理、相机、消息或存储失败。
