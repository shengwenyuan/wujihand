# 双侧 D405 collision、Camera prim 与 S4 视觉验收

- 日期：2026-08-05
- 环境：workstation2，Isaac Sim 6.0.1，RTX 5090
- 结论：007 S4 / Gate C3 通过
- Session hash：`85cdf69707da499f38fa1ee306524665d78f72b22b69535d9bdc4eaf9395c889`

## C3 collision

在 C2 的左右各 14 个 mount compound child shapes 基础上，每侧新增一个 D405 housing box
collision。配件仍全部附着于既有 Hand 2 base rigid body；没有新增 rigid body、MassAPI、joint、
articulation root 或 DoF。左右单侧和双侧 self-collision 三次运行均通过 rest、慢速 close、
grasp hold、慢速 open、final rest；无深穿透、未解释 rest contact 或跨侧 contact。

双侧运行的 q7/q20 最大稳态误差为 `0.043011/0.189439 rad`，最大 hold drift 为
`0.001140 rad`，最大 hand-base 位移漂移为 `0.0538 mm`。每侧 35 个 articulation body 的
runtime mass、COM、principal axes 与 inertia 和 C1 baseline 在 `1e-12` 绝对容差内一致。
稳定性快照完成后，左右隔离 probe 均实际命中对应 D405 housing box。

- 左：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s4_c3_d405_left_accepted/report.json`
- 右：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s4_c3_d405_right_accepted/report.json`
- 双侧：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s4_c3_d405_both_accepted/report.json`

## Camera prim

shared materializer 为左右 housing 各 author 一个 USD Camera。相机局部变换只由版本化
`hand -> mount -> rear_mount -> optical` 链解析；ROS optical
`+X right/+Y down/+Z forward` 到 USD camera `+Y up/-Z forward` 的变换只在 materializer
执行一次。API readback 与 profile 一致：`640x480`、pinhole、`140°` HFOV、
`128.225992°` VFOV、clipping `[0.02, 5.0] m`，`fx=116.470478`、`fy=116.470473`、
`cx=320`、`cy=240`。左右 Camera prim 都保留 profile ID、30 Hz、optical frame convention
和 simulation-only warning。

`140°` 是纯仿真特殊镜头，不是 RealSense D405 的实体规格或标定；本阶段未加入任何实体
相机逻辑。

## headless 截图观察

资格工具在 tabletop q7、双手 q20 rest、双侧 self-collision 与完整 wrist-rig collision
开启时 settle，然后 `world.pause()`。暂停后仅做 render update，保存 5 张 `640x480`
截图；渲染前后的双 q27、hand-base 与 Camera world transform 完全一致。

- 双臂全景：左右 NERO 保持 tabletop 初始姿态，左右 mount 与 D405 均存在；
- 左右装配近景：flange 连接、四根蓝色连杆、camera plate 与 D405 housing 连续可见；
- 左右 140° optical rest：画面镜像一致，手背与拇指/食指根部位于近场边缘，主要工作空间
  未被遮挡；未人为旋转画面以掩盖真实外参。

报告与截图目录：
`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s4_render_accepted/`。

已知 Hand 2 OmniPBR `texture_wrap_u/v` warning 以及低分辨率 viewport 的 DLSS 提示均为
非阻断渲染提示；未发现 physics、camera 或截图失败。
