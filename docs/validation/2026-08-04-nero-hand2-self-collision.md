# NERO—Hand 2 self-collision qualification

- 日期：2026-08-04
- 环境：workstation2，Isaac Sim 6.0.1，RTX 5090
- 结论：007 Gate C0、C1 通过；C2 尚未开始
- 范围：两棵 NERO—Hand 2 q27 articulation，不加载 mount/D405 collision

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
