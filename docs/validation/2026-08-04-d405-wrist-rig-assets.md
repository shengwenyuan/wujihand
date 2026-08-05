# 2026-08-04 D405 双腕资产生成验证

- Gate：S1
- 主机：Workstation2 / Ubuntu 24.04
- OpenSCAD：2021.01
- 生成报告：
  `hardware/camera_mounts/nero_hand2_beta1_realsense_d405/generated/generation_report.json`
- 产物树 SHA-256：
  `044bfb05931e26638336cb3b058ac3652f585d9fc2dbfaaf160aff5e697ae24c`

右件保持已接受的 v2 几何；左件由同一 SCAD 在导出阶段固化 `Y -> -Y`，USD 不需要负
scale。官方 `realsense-ros@bafc2108` D405 mesh 以 proper rotation 对齐 rear plane 后分别
生成左右 visual。

内置检查和 workstation2 上独立 `trimesh 4.11.1` 交叉检查均得到：四份 visual
`body_count == 1`、shared/split component 为 1、watertight、winding consistent。左右
mount bounds 镜像误差和左右 D405 bounds 镜像误差均为 `0 mm`；body/optical rotation
determinant 均为 `+1`。

mount collision 使用 14 个 compound pieces（基座/胶囊 key/相机板 box 与 8 段连杆
capsule），左右 visual vertex coverage 均为 `98.98%`，两个桁架空隙哨点均保持为空。
D405 collision 为独立 housing box，coverage 为 `100%`。visual 单体 Gate 与 collision
覆盖/空隙 Gate 分开判定。

相机 profile 不由本资产生成器复制。这里的偏心光心仅冻结几何 frame：右侧
`rear_mount -> optical = [0,+9,24] mm`，左侧为 `[0,-9,24] mm`。

> SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense D405
> specification or calibration.
