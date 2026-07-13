# ADR-0001：Hand 2 固定法兰采用 D6 三轴转向

状态：Accepted

日期：2026-07-13

## 背景

目标是在既有 MediaPipe 右手到 q20 的链路上增加手掌转向，并在 Isaac 桌面抓取动态球。当前阶段不引入 wrist XYZ，以隔离掌面姿态、旋转约束和刚性接触问题。

官方 `wuji-description v2026.6.27` 提供 Hand 2 Beta 1 right 模型，但不定义本任务需要的三轴腕部安装机构。

## 决策

1. 固定法兰位置 `[0, 0, 0.454] m`，task home 为本地 `+Y` pitch `+60°`。
2. 在只读 Hand 2 USD 上添加 world anchor + generic D6 overlay；锁定 transXYZ，限制相对 roll/pitch 为 `±89°`，yaw 周期连续。
3. 禁用上游 fixed root，使新 world fixed joint 成为唯一 ArticulationRoot；按 name + USD path 发现 wrist3/finger20。
4. 将 `r_base_link` principal-axes quaternion 的平方根对称写入 D6 两侧 joint frame，保证驱动轴与可见手掌轴一致。
5. 姿态统一采用 active、relative、`wxyz` quaternion；Euler 只用于安全包络和 D6 target。
6. 保留 `wujihand.q20.v1`；新增严格、原子的 `wujihand.hand_command.v2` 携带 q20、姿态、质量、时间和 calibration epoch。
7. 球必须是动态刚体；成功只由 PhysX 接触、球体高度、掌心相对滑移和连续保持判定。

## 未采用方案

| 方案 | 原因 |
|---|---|
| 每帧设置 root world pose | 属于 teleport，不能作为物理抓取验收路径 |
| root XYZ + RPY 六自由度 | 扩大绝对空间标定范围，不符合本阶段目标 |
| SphericalJoint | 不便表达独立 roll/pitch 限位和周期 yaw |
| 扩展 q20 v1 | 会破坏已有 receiver 的 schema 语义 |
| 第一代 soft-pad 模型 | 产品代际不匹配 |

## 后果

- 优点：无需相机尺度与绝对位置标定，姿态、手指和球体仍处于同一 PhysX 求解中，旧 q20 链路保持兼容。
- 代价：articulation 从 20 DOF 变为 23 DOF；组合角接近奇异区时需限制操作幅度；模型 principal axes 变化后必须重验。
- 能力边界：固定 XYZ 只形成受限抓取漏斗，不是通用桌面 pick。

## 验证责任

合入前必须通过唯一 articulation/23 DOF、固定法兰漂移、三转轴响应、动态球物理抓取、整手桌面净空、pinned 资产哈希和人工小角度方向检查。当前结果记录在 `docs/validation/2026-07-13-hand2-rotation-ball.md`。
