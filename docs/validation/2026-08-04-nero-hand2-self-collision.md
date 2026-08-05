# NERO—Hand 2 self-collision qualification

- 日期：2026-08-04
- 环境：workstation2，Isaac Sim 6.0.1，RTX 5090
- 结论：固定姿态资格证据保留；动态遥操作不通过，日常入口已回退为 disabled
- 范围：前半为无配件 C0/C1；文末追加 wrist-rig 遥操作 A/B 与命令回放

> 2026-08-05 修订：本文前半部分保留当时 C0/C1 的原始实验事实，不再作为 007
> mount/D405 发布 Gate。当前生产策略见文末“动态遥操作诊断与回退”。

## 冻结合同

- qualification profile：
  `configs/profiles/isaac_nero_hand2_self_collision_qualification_v1.yaml`
  (`sha256=c5d1bd9c5489f00eab5713e63a72e268f34b4b63f2968a3407c257c5d60caa46`)
- filtered-pair profile：
  `configs/profiles/isaac_nero_hand2_self_collision_filtered_pairs_v1.yaml`
  (`sha256=7d638b8fb7817434e08859e5bb4e4e53094721018b222cf9c408d8df6176508a`)
- physics：120 Hz；每次 1080 frames，覆盖 rest、慢速 close、grasp hold、慢速 open、final rest。
- q7/q20 稳态 target error 阈值分别为 0.08/0.25 rad；self penetration 阈值为 2 mm。

两个 profile 和所有结论均为固定资产上的仿真资格事实，不适用于实体 NERO 或 Hand 2。

## C0：self-collision disabled

`--enabled-sides none` 通过：两棵 articulation root 与每侧 27 DoF 保持不变，q7/q20 最大
稳态误差分别为 0.043012/0.189379 rad，无 self/cross-side contact，左右 hand-base 位移漂移
分别为 0.0532/0.0537 mm。

报告：
`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s2_c0_final/report.json`

## C1 原始证据与最小过滤

未过滤的左右单侧 C1 均只出现同一固有碰撞：NERO `link5` rigid body 与 `link7` rigid body
所含 collision mesh 持续重叠。左侧最深 2.295 mm，右侧最深 2.387 mm；两次运行都覆盖全部
1080 frames。没有 Hand 2 内部 pair、跨侧 pair 或配件 collision。

因此只过滤固定 NERO 资产的 `link5 <-> link7` 一对 rigid body。过滤通过
`UsdPhysics.FilteredPairsAPI` 写入；根 articulation 的
`physxArticulation:enabledSelfCollisions` 保持实际 readback 为 `true`。Hand 2 内部、
NERO 其他 pair 以及未来配件 pair 均不被过滤。

原始证据：

- 左：`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s2_c1_left/report.json`
- 右：`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s2_c1_right_unfiltered/report.json`

## C1 过滤后结果

| 运行 | 结果 | q7 最大稳态误差 | q20 最大稳态误差 | 最大 hand-base 位移漂移 | self/cross-side contact |
|---|---:|---:|---:|---:|---:|
| left | PASS | 0.043011 rad | 0.189379 rad | 0.0537 mm | 0 |
| right | PASS | 0.043012 rad | 0.189439 rad | 0.0538 mm | 0 |
| both | PASS | 0.043011 rad | 0.189439 rad | 0.0538 mm | 0 |

三次运行均满足：两棵 q27 root/DoF/topology 不变、feedback finite、root self-collision
readback 与请求一致、无未解释 rest contact、无深穿透、无跨侧 contact、静态漂移与 hold
drift 达标，并确认没有提前 author mount/D405 collision。

过滤后报告：

- 左：`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s2_c1_left_filtered/report.json`
- 右：`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s2_c1_right_filtered/report.json`
- 双侧：`/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s2_c1_both_filtered/report.json`

## 已知非阻断项

Hand 2 上游 OmniPBR 的 `texture_wrap_u/v ... not available in the MDL representation`
warning 仍出现，但与本 Gate 的 collision、articulation 和姿态结果无关。本阶段未修改上游
Hand 2 材质。

## 动态遥操作诊断与回退（2026-08-05）

固定 q7 与慢速 q20 轨迹没有覆盖 tracker 驱动的 NERO 动态工作空间。左侧遥操作 A/B
得到以下结果；右侧未用于根因判定：

| merged-q27 self-collision | wrist-rig collision | 结果 |
|---|---|---|
| enabled | all | NERO joint5 feedback 多圈旋转；命令保持有界 |
| disabled | all | 相同量级 tracker 动作下 command/feedback 一致，无越限 |
| enabled | none | joint5 再次多圈旋转，排除 mount/D405 collision 是直接原因 |

第一轮 enabled/all 中，joint5 command 范围为 `158.371°`，feedback 范围为
`4919.156°`；disabled/all 对照中 feedback 范围仅 `40.033°`。enabled/none 中 feedback
范围仍达到 `4022.122°`。因此异常不是 tracker 请求多圈旋转，也不是 wrist-rig 配件碰撞
直接施加的结果。

用 enabled/none 轮次保存的 `2215` 个左 q7 command 做 headless 确定性回放，`4430`
physics frames 中 joint5 在 frame `1997` 首次越过 `±157.563°` 限位，最终范围为
`2483.130°`。首次越限前后唯一持续相关的 articulation 内 contact pair 是：

```text
Hand2Left/l_base_link collision
<-> NeroLeft/.../link6/link6_1 collision
```

该 pair 共出现 `3558` 个 contact frames，最深 separation 为 `-2.748 mm`，并连续覆盖首次
越限窗口。最近一次人工动作未再次触发只说明该姿态窗口未命中；保存命令的回放已复现
contact 与越限的时序关联。

诊断原始报告保存在 workstation2：

- `/home/lenovo/swy/wujihand_ik_p0_scratch/live_round01_left_motion_probe.json`；
- `/home/lenovo/swy/wujihand_ik_p0_scratch/live_round02_no_self_collision_probe.json`；
- `/home/lenovo/swy/wujihand_ik_p0_scratch/live_round03_no_wrist_collision_probe.json`；
- `/home/lenovo/swy/wujihand_ik_p0_scratch/live_round03_contact_replay.json`。

生产处置如下：

- ROS2、native dual twin、static inspector 与 D405 render 入口保持 merged-q27
  self-collision disabled；
- mount/D405 visual、collision proxy、Camera prim 与外部碰撞继续保留；
- 专用 qualification runner、profile、filtered-pair 与本历史证据继续保留；
- “启动阶段把尚未稳定姿态作为遥操作零点”的独立 settling 修复继续保留；
- 临时环境变量、probe 与 replay 诊断逻辑不进入版本管理。

### 回退后的可视化验收

正式 `isaac_nero_hand2_ros_dual_live_v2` GUI 在 self-collision disabled、wrist-rig collision
all 下完成验收。启动 readiness 用 `5` 个窗口、`300` physics steps 收敛，最终 q27 window
delta 为 `0.001521 rad`；左右 q7 target error 分别为 `0.042945/0.042514 rad`，均低于
`0.08 rad`。

只操作左 tracker 的连续 `60 s` 内获得 `2914` 对 arm command/feedback：joint5 command
范围为 `60.75°—136.76°`，feedback 为 `62.35°—138.27°`，范围宽度分别为
`76.01°/75.92°`，没有越过 `±157.563°` 或出现多圈累计。GUI 人工观察同时确认未见小臂
自转。正式运行报告：
`/home/lenovo/swy/wujihand_mount/artifacts/runs/nv5/`
`isaac_nero_hand2_ros_dual_live_v2-20260805T092050Z.json`。

未来若重新启用 self-collision，至少需要覆盖动态 q7 工作空间、Hand2 base—NERO terminal
pair、joint-limit 强制保持和 enabled/disabled 反事实；不能仅复用本文早期固定姿态结果。
