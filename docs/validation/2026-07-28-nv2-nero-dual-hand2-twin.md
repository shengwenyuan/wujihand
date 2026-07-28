# 2026-07-28 NV-2 NERO 双实例、物理 Hand 2 与 Glove 链路阶段验证

状态：**PARTIAL / tabletop v5 已通过 82/82；live Glove、deliberate
contact/penetration、self-collision 最终决策与 measured Workcell/attachment 尚未闭合**。

已通过的范围是：

- NERO 固定来源、Isaac Sim 6.0.1 固定 recipe 导入及可复现性；
- NERO-only 双实例 pre-composition q7 smoke；
- 双 NERO + 双侧完整物理 Hand 2 + nominal Workcell + Session v1 的五层闭合；
- Isaac 中两棵同侧 q27 articulation；
- tabletop v5 的 82 项检查：保留 scripted physical v2 的左右 q7、双侧五指逐指、
  双侧组合手型、另一手/两臂隔离、有限值与限位、命令后 topology reset、回到批准
  初态和 post-reset recovery 等 68 项检查，并增加 14 项 tabletop 几何和准备位检查；
- Assembly 显式拥有 Hand 2 local `Ry(+90°)` attachment；Workcell 显式拥有桌面、
  同一近侧桌沿双 mount 和相机 frame；Session 唯一引用的 typed qualification
  profile 显式拥有左右分侧 q7 准备位、Isaac-only q7 drive gain 与验证阈值；
- fixed external Workcell collider 存在，初始、每个 scripted hand baseline 与 reset
  后的双 q27 静置均有界收敛。

尚未通过的范围是：

- 专用网口 `enx6c1ff7cd0e76` 尚待临时配置 `192.168.1.10/24`，因此实际
  `hand_skeleton → canonical → retarget → supervision → Isaac` live smoke 尚未执行；
- fixed collider 与 bounded rest settling 不能替代 deliberate contact/unknown
  penetration 场景；异常穿透量化和近景视觉 Gate 尚未执行；
- merged q27 的最终 self-collision policy 尚待项目负责人确认；
- measured Workcell、法兰与转接件 attachment 尚未建立。

因此本报告不能将 NV-2 标为完整完成。以下全部仿真均未启动或控制真实 NERO、
真实 Hand 2、CAN 或 ROS 2 command。

## 验证环境

| 项目 | 实际值 |
|---|---|
| 主机 | `lenovo@Workstation2`，SSH alias `lenovo-piper2` |
| OS | Ubuntu 24.04.4 LTS |
| GPU | NVIDIA RTX 5090 |
| NVIDIA driver / driver CUDA API 上限 | `595.58.03` / `13.2` |
| Isaac environment | `/home/lenovo/.venvs/isaacsim-6.0.1` |
| Isaac distribution | `isaacsim 6.0.1.0` |
| Isaac Python | `3.12.3` |
| URDF importer extension | `3.11.2` |
| Asset Transformer / rules | `1.2.5` / `1.7.10` |
| 项目 checkout | `/home/lenovo/swy/wujihand` |

SteamVR/OpenVR tracking 不是本次 NV-2 仿真路径的前置条件。本轮没有初始化真实机械臂
或手部执行 SDK。

## NERO 来源与导入 Gate

NERO 固定来源：

| 项目 | 固定值 |
|---|---|
| repository | `https://github.com/agilexrobotics/agx_arm_urdf.git` |
| commit | `f6642ce0d7872c686f29c99e9e10cd23d1d49313` |
| URDF | `nero/urdf/nero_description.urdf` |
| URDF SHA-256 | `c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278` |
| mesh 集合 | 8 个 visual DAE + 8 个 collision STL |
| mesh tree SHA-256 | `2323405c805cbc4187f451a22d02954fc952a8a2b67b9602119ad26e1ee5031e` |

固定 recipe 在 Isaac 6.0.1 中确认：

- stage 单位为 `1 m/unit`；
- default prim 为 `/nero`；
- articulation root 为 `/nero/Geometry/world`；
- 固定基座 joint 为 `/nero/Physics/world_to_base_link`；
- 恰好 7 个 revolute joint，顺序为 `joint1..joint7`；
- 恰好 8 个 rigid body：`base_link` 与 `link1..link7`；
- parent/child、axis、limit、origin、mass、center of mass 和 inertia 与固定 URDF 对照；
- 8 个 collision mesh 均来自来源 STL；
- NERO 派生层显式设置 `enabledSelfCollisions=false`。

连续两次独立导入得到相同 package 和 report：

| 对象 | SHA-256 |
|---|---|
| package tree | `07ba62ca6d7ab79cb76a2148e76743cda78671e1bfa40ad418158554179214a0` |
| main USD | `d8ca2ceb58ad57cab0dc521323b560631207ba1838d4be0c1ddd8f85a27d5289` |
| import report | `236915c248585f7a849eab22b21f1f54cd76826ea15b7a45ad64fe96f421e6fe` |
| generator | `9b724d413808cb419732d1d26b17169084c5c0a77268df47ff38d654a7d1dd83` |
| import adapter | `e403835886a38df1aa82de5c784d7a622966d929ae42f257f9450cac68272dbc` |

这些检查证明派生表示符合固定来源和 recipe，不把 URDF effort、drive gain 或临时
q7 profile 提升为真机安全事实。

## 历史 NERO-only pre-composition 证据

在完整 Hand 2 组合前，先运行：

```bash
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tests/integration/isaac_nero_dual_asset_smoke.py \
  --frames-per-phase 240 \
  --amplitude-rad 0.08 \
  --report artifacts/validation/nv2/nero-dual-asset-smoke.json
```

该脚本只引用同一个 NERO 派生 USD 两次，使用 `±0.75 m` 临时分离位置。该位置不是
正式 Workcell 测量。

| 检查 | 观测 | 结果 |
|---|---:|---|
| left `joint1` 响应 `+0.08 rad` | `0.0798324 rad` | 通过 |
| phase 1 right 保持隔离 | 最大绝对偏差 `< 0.01 rad` | 通过 |
| right `joint2` 响应 `-0.08 rad` | `-0.0818661 rad` | 通过 |
| phase 2 left 保持上一目标 | `joint1=0.0799972 rad` | 通过 |
| feedback finite 且在临时 profile limits 内 | `true` | 通过 |

证据：

```text
/home/lenovo/swy/wujihand/artifacts/validation/nv2/nero-dual-asset-smoke.json
```

此结果只作为 NERO Asset/Binding 的独立历史证据；完整 NV-2 结论以下一节为准。

## 完整五层 Session

本轮正式组合入口是：

```text
configs/assets/agilex_nero_v1.yaml
configs/assets/wuji_hand2_beta1_left_v1.yaml
configs/assets/wuji_hand2_beta1_right_v1.yaml
configs/bindings/isaac/agilex_nero_f6642ce0_isaac_6_0_1_v1.yaml
configs/bindings/isaac/wuji_hand2_beta1_left_v2026_6_27_physical_v1.yaml
configs/bindings/isaac/wuji_hand2_beta1_right_v2026_6_27_physical_v1.yaml
configs/assemblies/nero_dual_hand2_physical_simulation_nominal_v1.yaml
configs/workcells/isaac_nero_dual_hand2_simulation_nominal_v1.yaml
configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml
```

Session v1 恰好解析四条显式 commanded route：

| 侧别 | NERO | Hand 2 | logical DoF |
|---|---|---|---:|
| left | `arm_joints / agilex_nero_q7_v1` | `finger_joints / wuji_hand2_left_firmware_v1` | 27 |
| right | `arm_joints / agilex_nero_q7_v1` | `finger_joints / wuji_hand2_right_firmware_v1` | 27 |

总数为 54 logical DoF。没有 `inactive` group，也没有 Session v2。

Workcell 中的 `1.20 m × 1.20 m × 0.08 m` 桌面和 `0.80 m` table top 保持
`simulation_nominal`。两台底座位于同一近侧桌沿：

| 侧别 | mount position（m） | mount yaw |
|---|---|---|
| left | `[-0.32, -0.52, 0]` | `+90°` |
| right | `[+0.32, -0.52, 0]` | `+90°` |

Session 的 `runtime.compatibility_profile` 引用
`configs/profiles/isaac_nero_dual_tabletop_qualification_v1.yaml`，保存：

| 侧别 | q7 准备位（deg） |
|---|---|
| left | `[-10, -60, 0, -30, -90, 0, 0]` |
| right | `[+10, -60, 0, -30, -90, 0, 0]` |

该 profile 还将 q7 Isaac drive gain 设为 `stiffness=3000`、
`damping=150`。这些值仅用于本次 Isaac qualification；它们没有修改通用 NERO
profile、来源 USD 或硬件控制器参数。桌面、mount、q7 准备位与 drive gain 都只服务
功能联调，不形成现场 clearance、安装精度或安全包络证据。

## q27 组合方法

左右 Hand 2 固定到 `wuji-description v2026.6.27` commit
`aee64892ebcf8e3237bedc30231bb09476cbc71d`，直接使用完整物理 USD：

| 侧别 | USD SHA-256 |
|---|---|
| left | `646287f10ac0a2097bf602facc02c9af17f0f1cf8982c38037f69bb695492eca` |
| right | `3cb3dcb18b07621a52a47a8daa98ab82794e3c77d36275d068b3b5b0516e5f00` |

每只来源 Hand 2 USD 自带 world-fixed `root_joint` 和 20 个 revolute DoF。若只把它作为
独立 reference 放在 NERO 法兰旁，Hand 会固定到 world，不会成为臂链的一部分。

`src/wujihand/adapters/simulation/nero_hand2_twin.py` 在 PhysX 初始化前：

1. 根据 Assembly transform 放置 Hand 2；本轮左右 `link7 → hand_base` 均为
   local `Ry(+90°)`，即 `quat_wxyz=[0.70710678, 0, 0.70710678, 0]`；
2. 禁用 Hand 2 world `root_joint`；
3. 移除该 prim 的 `ArticulationRootAPI`；
4. author `NERO link7 → Hand 2 base` FixedJoint；
5. 检查 stage 最终恰好剩两个 articulation root；
6. 以 joint name 和 USD joint path 分出每侧 q7 与 q20。

来源 USD 不被改写。最终 Isaac 物理拓扑是：

```text
left  q27 = NERO left q7  + Hand 2 left q20
right q27 = NERO right q7 + Hand 2 right q20
```

四条 logical command route 与两棵 physical articulation 是不同层次的事实。

## Tabletop v5 仿真

运行入口：

```bash
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_dual_twin.py \
  --session configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml \
  --frames-per-phase 120 \
  --report artifacts/validation/nv2/nero-dual-hand2-tabletop-v5.json \
  --screenshot artifacts/validation/nv2/nero-dual-hand2-tabletop-oblique-v5.png \
  --top-screenshot artifacts/validation/nv2/nero-dual-hand2-tabletop-top-v5.png
```

报告头：

| 字段 | 实际值 |
|---|---|
| schema | `wujihand.isaac_nero_dual_hand2_physical_smoke.v2` |
| Session | `isaac_nero_dual_hand2_physical_simulation_nominal_v1` |
| Session hash | `c082dcc4e6c67375cab30435bd1fbe77fa136e8d1b43d525ba467f0464ed7b4e` |
| tabletop profile hash | `dac2127aa57e9509d041d74a50d6ebf96445654dbd2f5635df640442890ba5af` |
| Isaac | `6.0.1.0` |
| physics rate | `120 Hz` |
| frames per phase | `120` |
| result | `82/82 checks true`，`passed=true` |
| Glove live | `enabled=false` |
| scope | NV-2 simulation only；无 ROS、CAN、NERO 真机或 Hand 2 真机 |

运行先驱动 left arm J1 和 right arm J2，再对左右手分别执行五个单指 phase、一个
组合手型 phase，随后 reset stage/topology、回到批准初态，并以 left middle PIP
完成 post-reset recovery。原 scripted physical v2 的 68 项检查全部保留；v5 增加
14 项 tabletop Gate，合计 82 项全部为 `true`：

| 类别 | checks |
|---|---|
| 全局与 workcell | 全部 feedback finite；固定 `/World/Workcell/simulation_nominal_table` collider 存在 |
| q7 | 左右分侧准备位到达；左右选定臂关节响应；同侧手、另一侧完整 q27 保持；reset 后准备位重新到达 |
| tabletop 几何 | 左右 attachment 轴对齐；端口假设轴朝桌外；手轴近水平并朝桌内；掌面向下 |
| 单指 | 左右 thumb/index/middle/ring/pinky 分别响应；同手其他手指、两臂 q7 与另一侧保持 |
| 组合手型 | 左右各 15 个屈曲关节响应；两臂 q7 与另一侧保持 |
| limits | 左右命令前后及 post-reset feedback 均在 canonical limits 内 |
| reset/topology | reset 前后恰好两棵 q27 root、partition 稳定、回到批准初态 |
| recovery | reset 后 left middle PIP 再次响应；同手其他手指、selected arm 与另一侧保持 |
| settling | 初始、每个 scripted hand baseline 与 reset 后的双 q27 静置均 finite 且有界收敛 |

单指 phase 的命令增量是 `0.4 rad`；左右组合手型各对 15 个 flexion joint 使用
`0.2 rad` 增量。单指检查把“同手其他手指”作为隔离 Gate，容差是 `0.03 rad`；
同一手指中未直接命令的机械联动仅作为诊断，不作为 Gate。

reset 前后的 articulation root path 相同，q7/q20 partition 保持稳定，左右 q7
均在 `0.08 rad` qualification threshold 内回到对应准备位。初始与 post-reset 的
最终最大反馈变化均为 `0.0019331 rad`，低于 `0.005 rad` settling 容差；初始和
post-reset 均在 4 个 window 内收敛，其余 scripted hand baseline 均在 2 个 window
内收敛。

证据已拉取到当前仓库的忽略目录；目标机执行路径是在表中路径前加
`/home/lenovo/swy/wujihand/`：

| Artifact | 路径 | 身份记录 |
|---|---|---|
| v5 完整报告 | `artifacts/validation/nv2/nero-dual-hand2-tabletop-v5.json` | `27812360eea72ba10f5a7730113400e3c477e1b0ffc60cbcebf3cbf0feab521f` |
| v5 斜视截图 | `artifacts/validation/nv2/nero-dual-hand2-tabletop-oblique-v5.png` | `c92b692edf2200b9948409e3afdc8b12f47334ab6127e5c057c5fc499eab3f9f` |
| v5 俯视截图 | `artifacts/validation/nv2/nero-dual-hand2-tabletop-top-v5.png` | `09fe5d2c3a727e6126754546460232a22361d8d683043ac56c204f23493f816a` |
| v2 完整报告 | `artifacts/validation/nv2/nero-dual-hand2-physical-v2.json` | 68/68 历史基线；`5623b8552f54cfd186640a5b857179bca2b7cbd935f4d847451cf1912573b20f` |
| v2 渲染截图 | `artifacts/validation/nv2/nero-dual-hand2-physical-v2.png` | 历史截图；`2366d024a8d26f85ca97fd2c79aa190a148270ac8e01be85592587a79e201a6a` |
| v1 报告 | `artifacts/validation/nv2/nero-dual-hand2-physical-headless.json` | 仅保留为 17 项历史基线 |
| v1 截图 | `artifacts/validation/nv2/nero-dual-hand2-physical.png` | 仅保留为历史整体场景截图 |

v5 的斜视和俯视截图均使用 `/OmniverseKit_Persp`。斜视相机 world position 为
`[1.15, -1.55, 1.45] m`，目标为 `[0, -0.03, 1.02] m`；俯视相机 world position
为 `[0, -0.10, 2.25] m`，目标为 `[0, -0.05, 0.80] m`。报告验证两个相机 frame
与 camera prim 有效，并在每次设置机位后执行三个 render settle frame。两张图共同
覆盖同侧桌沿安装、左右分侧准备位、手与小臂连续朝桌内及掌面向下的 nominal 场景。

v5 报告直接从 Isaac stage 测得：

| 几何检查 | left | right | threshold |
|---|---:|---:|---:|
| Hand 2 纵轴与小臂轴点积 | `0.9999999999999756` | `0.9999999999999938` | `>=0.999` |
| 端口假设轴朝桌外点积 | `1.0` | `1.0` | `>=0.999` |
| 手纵轴朝桌内点积 | `0.9815807` | `0.9817971` | `>=0.90` |
| 手纵轴竖直分量绝对值 | `0.0902464` | `0.0877118` | `<=0.10` |
| 掌面朝下点积 | `0.9958896` | `0.9960705` | `>=0.99` |

其中 attachment、手纵轴和掌面结果是仿真几何测量。端口轴采用
`base local -X`，该方向来自固定 NERO base mesh 外凸特征推断；`1.0` 只证明此推断轴
在 nominal `yaw=+90°` mount 下朝桌外，**不是端口实物测量或真机安装确认**。该项在
物理对应前仍需项目负责人观察实物确认。

v5 报告的 external collision 证据边界是：

```text
fixed external colliders retained;
bounded q27 rest convergence before every scripted hand baseline and after reset;
no deliberate unknown penetration/contact scenario introduced
```

报告明确记录 `deliberate_unknown_penetration_probe=false`。因此 82/82 证明当前
nominal stage 下的几何 contract，以及当前 trajectory 下的运动、隔离、
reset/recovery、limits 与有界 rest settling；它不证明 deliberate contact、未知
穿透、attachment 的实物精度、端口物理侧别或现场 clearance。

当前报告记录的 self-collision policy 是：

```text
merged_q27_disabled; external collisions retained;
Hand2 internal self-collision not qualified
```

也就是说，外部 collider/contact shape 保留，但同一 q27 articulation 内的
finger/palm self-collision 没有资格化。该策略仍需项目负责人确认，不能由本次
`passed=true` 自动升级为最终产品决策。

## Glove canonical 与 retarget 验证

已经建立的代码边界：

```text
Wuji Glove hand_skeleton
  -> WujiGloveHandSkeletonAdapter
  -> CanonicalHandObservation
       MediaPipe named 21 points
       (21, 3) metres
       side/frame/confidence/time/calibration
  -> WujiHand2RetargetAdapter
       RetargetSession.for_hand(HandModel.WujiHand2, side=...)
  -> HandIntent q20 radians + explicit side-specific layout
  -> GloveHand2SimulationController
  -> JointCommandSupervisor
  -> compose_q27_hand_target
  -> same-side q20 partition in the q27 articulation
```

以下无硬件测试覆盖 Glove SDK adapter、canonical contract、retarget adapter、
controller/supervisor composition 和 SDK-independent replay：

```text
tests/unit/test_hand_teleoperation_contract.py
tests/unit/test_wuji_glove_adapter.py
tests/unit/test_wuji_hand2_retarget_adapter.py
tests/unit/test_glove_hand2_simulation_controller.py
tests/contract/test_hand_observation_jsonl_fixture.py
tests/unit/test_hand_observation_replay_adapter.py
```

测试覆盖 side、header/sequence、device clock、canonical 21 点顺序、latest-frame、
freshness、confidence、missing point、NaN、q20 shape/layout、source epoch 与
retarget reset。`hand_joint_angles` 从未作为 Hand 2 q20 输入。置信度策略是：
`<0.90` 拒绝，`[0.90, 0.95)` 只允许以 `DEGRADED` 状态经过 supervisor，
`>=0.95` 才是 success。

缺帧、错误 side/layout、低置信度、NaN 或 retarget failure 不会创建新的
input-derived `HandIntent`。已有最后有效命令可在 supervisor 的 `0.25 s` freshness
窗内继续 hold；超时后由 supervisor 输出渐进 return-to-rest 安全命令。后者不是由
invalid input 派生的新 q20 intent。

当前 v5 报告记录 `glove_live.enabled=false`。专用网口
`enx6c1ff7cd0e76` 尚待临时配置 `192.168.1.10/24`，因此：

- 未记录实际 Glove serial/side；
- 未形成 named SDK user 与 handedness calibration manifest；
- 未取得实际 `hand_skeleton` frame；
- 未运行真人 live retarget 或 live Isaac q20 command。

该结果必须按“未执行”处理，不能由 fake SDK/composition test 或 82 项 tabletop
仿真代替。

## Gate NV-2 对照

| Gate 项 | 状态 | 证据或缺口 |
|---|---|---|
| 五层 Session v1 闭合、4 route、54 logical DoF | 通过 | contract + Session hash |
| 左右完整物理 Hand 2 来源与 q20 layout | 通过 | source lock、Binding、USD hash |
| stage 恰好两棵 q27 articulation | 通过 | runner 启动前结构检查与报告 |
| Hand world root 禁用、FixedJoint attachment、q7/q20 分区 | 通过 | adapter fail-closed 检查 |
| Assembly `Ry(+90°)` 与手—小臂轴对齐 | 通过（nominal） | v5 attachment dot 左右均约 `1.0`；实物 adapter CAD 待补 |
| 同侧桌沿 mount、端口假设轴朝外 | 通过（nominal） | `x=±0.32, y=-0.52, yaw=+90°`；port-axis dot=`1.0`，轴向为 mesh 推断待实物确认 |
| 左右 q7 准备位与 reset 后回位 | 通过（nominal） | `[∓10,-60,0,-30,-90,0,0]°`；v5 初始/post-reset checks |
| 手朝桌内、近水平且掌面向下 | 通过（nominal） | inward、vertical 与 palm-down 五项 stage 几何测量均过 threshold |
| q7 响应与双实例隔离 | 通过 | tabletop v5 82/82，包含历史 v2 相关 Gate |
| 左右逐指与组合手型 fixture | 通过 | 双侧五指单指 phase + 双侧 15-joint 组合 phase |
| sampled feedback finite 且在 canonical limits 内 | 通过 | v5 全局、左右及 post-reset checks |
| 命令后 reset/topology/recovery | 通过 | 两根 q27 重验、partition stable、回到初态并恢复命令 |
| fixed external collider 与 bounded rest settling | 通过 | table collider + 每个 baseline/reset 后 `0.005 rad` 容差 |
| deliberate contact/unknown penetration 与近景证据 | **未执行** | `deliberate_unknown_penetration_probe=false` |
| nominal 场景渲染证据 | 通过 | v5 斜视/俯视 PNG、相机 frame 与 SHA-256 已冻结 |
| 实际 Glove `hand_skeleton` live smoke | **未执行** | 专用 NIC 待临时配置 `192.168.1.10/24` |
| composition-level invalid fail-closed | 通过（无硬件） | controller/supervisor/composer 测试；live fault injection 随 live 补验 |
| merged q27 最终 self-collision policy | **待确认** | 当前仅验证 disabled policy |
| measured Workcell / 实物 attachment | 后续阶段 | 不阻塞 nominal 功能联调，不构成现场几何事实 |

结论：**NV-2 的配置、adapter/controller 边界、双 q27 拓扑和 tabletop v5 82/82
已闭合；历史 scripted physical v2 68/68 继续作为前一版基线。完整 NV-2 仍缺 live
Glove Gate、deliberate contact/异常穿透近景、self-collision 最终决策，以及后续
measured Workcell/attachment。**

## 明确未执行

- 未发送真实 NERO 或 Hand 2 command；
- 未执行 CAN read/write、ROS 2 command 或机械臂运动；
- 未把 nominal attachment 宣称为真实安装；
- 未完成实际 Glove inventory、calibration 或 live hand tracking；
- 未资格化 Hand 2 internal self-collision；
- 未引入 deliberate contact/unknown penetration 场景，未量化异常穿透或完成近景
  视觉检查；
- 未完成 measured Workcell clearance、精度或真实双臂安全包络；
- 未执行长时间 soak、抓取任务、触觉、力控或真机闭环。

## 后续所需材料

1. 专用网口 `enx6c1ff7cd0e76` 的临时 `192.168.1.10/24` 配置，以及可被目标机
   Wuji SDK 识别的 Glove side、serial、SDK version；
2. named SDK user 和每个 handedness 的 calibration revision；
3. 至少一段脱敏 live `hand_skeleton` fixture 与对应 q20/rejection 记录；
4. deliberate contact/unknown penetration、异常穿透量化和近景视觉证据；
5. 真人 live failure injection 与监督日志；
6. 项目负责人对 merged q27 self-collision policy 的确认；
7. 后续物理对应阶段的 NERO 法兰、Hand 2 转接件 CAD、桌面和底座 mount 实测。

第 7 项不阻塞 nominal 仿真功能联调，但在做真实几何、clearance 或真机结论前必须完成。

## 官方依据

- 本体二维码页，《机械臂PIPER NERO（7F）》，“有效负载”“关节运动范围”
  （页面更新时间 2026-07-17）：
  [source_url](https://qr61.cn/oMm9uo/q4oW6ZW)。
- Wuji Technology，《Wuji Description 版本发布记录》，“2026.07.01”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-description/latest/release-notes.md)。
- Wuji Technology，《Wuji Description 集成指南》，“3.2.3 Isaac Sim USD”与
  “3.3.2 Wuji Hand 2（Beta 1）整机结构件”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-description/latest/integration.md)。
- Wuji Glove，《EMF 与手部追踪》，“手部追踪产物”“HandSkeleton”
  “HandJointAngles”“输出率调节”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking)。
- Wuji Glove，《手型标定》，“何时需要标定”“SDK 用户前提”“URDF 查找顺序”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/calibration)。
- Wuji SDK，《手部重定向（Retargeting）》，“快速开始”“输入格式”“实时遥操作示例”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting)。
- Wuji Retargeting，《API 介绍》，“输入格式”“输出格式”“不同输入模式的配置”：
  [source_url](https://docs.wuji.tech/docs/zh/wuji-retargeting/latest/api)。
- 松灵机器人，《NERO用户手册》V1.0.0，“性能参数”“机械安装说明”
  “关节限制设置”“尺寸图纸”：
  [source_url](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/air57k7k3nhgeuxb)。
- NVIDIA，[URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html)。
