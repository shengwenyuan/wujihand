# ADR-0010：D405 140°纯仿真腕部组件边界

- 状态：已接受；2026-08-05 修订 self-collision 生产边界
- 日期：2026-08-04
- 前置决策：[ADR-0003：五层 Session 组合](0003-five-layer-session-composition.md)、
  [ADR-0005：NERO 模型来源](0005-nero-model-source-and-provisional-limits.md)、
  [ADR-0009：ROS2 全因果录制](0009-ros2-full-causal-recording-boundary.md)
- 实施计划：[007：双腕 D405 纯仿真集成](../007-isaac-dual-wrist-d405-simulation-plan.md)

## 背景

双 NERO—Hand 2 仿真需要在左右腕部默认加载 v2 mount、D405 外壳、碰撞代理和数据相机。
当前五层 schema 只允许带控制组的机器人/虚拟机构，无法诚实表达无 DoF 的 mount 和相机；
当前录制链也没有同帧 RGB、optical-Z depth、仿真内参和逐帧外参合同。

本项目选择的 `140°` 水平视场是为了腕部仿真数据采集而定义的合成 pinhole 镜头，不是
RealSense D405 的产品规格或实体标定。D405 上游资料在本功能中只提供外壳 mesh、安装面
和偏心光心参考。

## 决策

1. 新增 `wujihand.asset_manifest.v2` 与 `wujihand.backend_binding.v2`；v1 解析和不变量
   保持不变。v2 增加 `passive_component`、`simulated_sensor`，且只有这两类允许空
   `control_groups`/`group_bindings`。
2. v2 Isaac Binding 必须显式引用 visual artifact、collision artifact、canonical frame；
   `simulated_sensor` 还必须引用一个版本化 camera profile。被动实例必须使用 `prefix`
   namespace，不新增 route、DoF、controller、process 或 execution owner。
3. `140°`、`640×480 @ 30 Hz`、pinhole、零畸变、RGB/depth encoding 和 clipping range
   只由 `isaac_d405_synthetic_wide_angle_140_v1` profile 拥有。所有创建 Camera prim 或
   导出相机 metadata 的实现点必须保留以下等价注释：

   ```text
   SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense D405 specification or calibration.
   ```

4. 左右 mount 与 D405 都生成独立、无负 scale 的镜像资产。由于 D405 光心横向偏置，
   左右 `rear_mount -> optical` transform 独立保存，不能复用一个 `side=none` transform。
5. 仿真内参由 Isaac 6.0.1 Camera API 的 focal length、aperture、offset 和 resolution
   readback 推导；`D=0`、`R=I` 是 authored 合同。外参来自同一 completed capture 的
   stage transform，不是实体标定。
6. RGB 与 `distance_to_image_plane` 必须由同一个 CameraSensor render product、同一个
   Replicator Writer 回调产生；该回调的 `reference_time` 是完成帧身份。标准 ROS 消息共享
   `(stamp, optical_frame_id)`；自定义 frame-truth 消息保存唯一 `camera_frame_index`、
   physics substep、完成帧 `reference_time` 和同帧外参。Isaac 6.0.1 实测中的
   `swhFrameNumber` 恒为 `0`，不得作为帧身份。
7. `record=false` 保留 visual、collision 与 Camera prim，但不创建数据 render product；
   `record=true` 才启用双侧 30 Hz 发布和 MCAP 录制。
8. 日常 Isaac、ROS2、static inspector 和 D405 render 入口保持 merged-q27
   self-collision disabled；mount/D405 collision proxy 仍加载并对外部刚体生效。已有
   self-collision qualification 工具、profile 与报告作为隔离实验设施保留，不属于 D405
   发布 Gate，也不证明 Hand 2 internal 或 finger—accessory contact 可用于动态遥操作。
9. mount/D405 collision 作为 Hand 2 base rigid body 的 compound child shapes；不得新增
   rigid body、MassAPI、joint、articulation root 或 DoF，也不得把有空隙的桁架替换为
   单一大 convex hull。
10. 固定 NERO URDF/USD、q7 限位、`link6` 表示对齐与
    `link7 -> hand_base [0.023,0,-0.0235] m + Ry(+90°)` 保持不变。这些仍是当前 pinned
    资产的 simulation-nominal 事实，不能据此推断实体 NERO 或实体 D405。

## S0 Isaac 6.0.1 实测决议

workstation2 的 `CameraSensor/RtxCamera` API spike 冻结以下实现边界：

- CameraSensor 的 `30 Hz` tick 由 committed timeline 的 app update 驱动；连续
  `rep.orchestrator.step()` 不会稳定重复触发该 sensor，且对持续 sensor workflow 调用
  `wait_until_complete()` 会无界等待，因此两者都不是生产捕获驱动或完成条件。
- Writer 在一个回调中提供 `rgb` 的 `uint8[480,640,4]` RGBA、
  `distance_to_image_plane` 的 `float32[480,640]` 和共同 `reference_time`。RGB alpha
  不是固定 `255`，转换只保留前三个 RGBA channel；depth no-hit 是 `+inf`，统一转换为
  canonical quiet NaN `0x7fc00000`。
- Camera API readback 推导采用 `cx=width/2`、`cy=height/2`；正 horizontal aperture
  offset 令 ROS raster `cx` 左移，正 USD-up vertical offset 令 ROS-down `cy` 下移。两个
  marker 的 render/K 误差分别为 `0.543 px` 与 `0.769 px`，通过 `2 px` Gate。
- 改变 stage 状态后实测出现一个旧 completed frame。运行时必须保留按 rational
  `reference_time` 索引的 capture pose/history，并把 writer payload 连接到对应历史快照；
  不得在回调到达后读取“当前 pose”。warm-up/discard 数量及实际 frames-in-flight 仍写入
  run manifest。

S0 证据记录见
[2026-08-04 D405 Camera API spike](../validation/2026-08-04-d405-camera-api-spike.md)。

## 来源固定

- D405 上游固定到 `realsenseai/realsense-ros` tag `4.56.4`、commit
  `bafc21080c5c8e259dadbb309797949aee0dd950`。
- 只恢复 Apache-2.0 `LICENSE`、`realsense2_description/meshes/d405.stl`、
  `realsense2_description/urdf/_d405.urdf.xacro` 与最小 package metadata；具体 hash
  由 `third_party/sources.lock.yaml` 管理。
- 上游 xacro 明确将部分值称为 approximate/nominal，并警告 inertial 不可靠；本功能
  不导入其质量、惯量、实体校准或设备驱动逻辑。

## 验证责任

- 普通 Python：v1/v2 schema round-trip、空控制组边界、source lock、左右镜像、相机
  profile、图像转换和 Session 八实例/四控制组合同。
- Isaac headless：CameraSensor/RtxCamera API、completed-frame identity、K/P provenance、
  self-collision disabled readback、外部 mount/D405 probe、双 q27 root/DoF、质量/惯量不变
  和 30 Hz capture。
- Isaac GUI：tabletop 姿态、完整左右装配、接口近景、collision debug 及左右 140°视图。
- ROS2/MCAP：同帧 join、单父 TF、recorder readiness、finalize、回放、吞吐和离线完整性。

## 未采用方案

- 把 mount/camera 伪装成带 dummy DoF 的机器人：破坏控制路由事实。
- 把 `140°` 写成 D405 nominal/calibrated：会误导未来实体部署。
- 在 USD 使用 `scale=(1,-1,1)`：会引入 reflection、法线和碰撞错误。
- 同时发布 `hand_base -> optical` 与 `world -> optical`：会形成 TF 双父节点。
- 把静态 self-collision 报告直接作为动态遥操作发布依据：覆盖不到 tracker 驱动的 q7
  工作空间与关节限位保持。
