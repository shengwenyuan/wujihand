# NERO 双实例 + 物理 Hand 2 Isaac 数字孪生

状态：**PARTIAL / 部分完成（inward-port tabletop v15 已在 Workstation2
通过 90/90；左右 Glove live 均已现场打通，2026-07-29）**。

当前组件已建立双 NERO、双侧完整物理 Hand 2、nominal 工作台和 Session v1 的五层组合。
目标机 Isaac Sim 6.0.1 运行形成左右两棵独立的 q27 articulation。tabletop v15
的 90 项检查全部通过：保留 scripted physical v2 的 68 项 q7、逐指/组合手型、
隔离、finite/limits、reset/topology/recovery 检查，并增加 22 项 tabletop
attachment anchor、基座端面中心/平行度、同侧桌沿 mount、左右准备位、手部朝向、
小臂近水平及 `link6` 表示
资格检查。
结果不包含真实 NERO 或 Hand 2 运动。接电侧朝桌内来自项目负责人实物确认；端口
local 轴、nominal 装配和工作台数值仍不解释为现场测量事实。

NV-2 尚不能标记为完整完成：

- 左右实际 Glove live 均已取得
  `hand_skeleton → canonical → retarget → supervision → Isaac` 证据；两次轻量
  live-only 运行各 9000 帧、接收 8999 帧、拒绝 0 帧。报告生成后 Session
  资产快照继续演进，当前 hash 下的复验仍需单独记录；
- 合并 q27 articulation 的最终 self-collision policy 仍待项目负责人确认；
- fixed external collider 与 bounded rest settling 已通过，但 deliberate
  contact/unknown penetration 场景和异常穿透量化尚未执行；
- 设备 J7 frame、`link6` clocking、法兰孔位、桌面和底座 mount 仍待 measured
  revision。

## 能力与边界

| 项目 | 当前状态 | 结论边界 |
|---|---|---|
| NERO 来源与 Isaac 表示 | 已验证 | 固定 URDF、mesh、导入 recipe、派生 USD 和结构报告 |
| 左右 Hand 2 表示 | 已建立 | 直接使用固定版本的完整物理 USD，保留 rigid body、collision、drive 和 q20 |
| 五层组合 | 已建立 | 4 个资产实例、2 个 Assembly root、nominal Workcell、Session v1 |
| 逻辑命令 | 已建立 | 左/右 NERO q7 + 左/右 Hand 2 q20，4 个显式 route、54 logical DoF |
| Isaac 物理拓扑 | 已验证 | 左右各一棵 q27 articulation，共 2 棵 |
| tabletop v6 | 历史通过 | 84/84：旧 attachment 定义的回归基线 |
| inward-port tabletop v15 | 通过 | 90/90；接电侧朝桌内、基座端面中心、平行度、attachment anchor 与圆柱—小臂轴 Gate 均通过 |
| external collision settling | 部分通过 | fixed collider 与各 baseline/reset 后有界静置通过；deliberate contact/penetration 待执行 |
| Glove canonical/retarget/supervision 代码边界 | 已建立并以 fake SDK/composition 测试 | 外部 SDK 类型不进入 domain/ports；invalid/missing 不创建新 input-derived intent |
| 真实 Glove live 路径 | 左右均已执行 | 各 8999/9000 帧接收、0 帧拒绝；快速 live-only Gate 均通过 |
| Tracker → 右 NERO 平移 | 人工通过 | Workstation2 三方向已核对；仅为仿真输入映射 |
| Tracker → 右 NERO rotation | 待按当前定义复验 | 旧 corrected-J7 合成报告已降为历史证据；当前 Lula 使用固定来源 URDF |
| self-collision policy | 待确认 | 当前 smoke 关闭合并 articulation 自碰撞，保留外部碰撞 |
| 真机 | 未接入 | 未执行 CAN、ROS command、NERO SDK command 或 Hand 2 command |

## 五层组合

本组件严格沿用：

```text
Asset Manifest
  -> Backend Binding
  -> Assembly Spec
  -> Workcell
  -> Session
```

### Asset Manifest

- `configs/assets/agilex_nero_v1.yaml` 定义 NERO 产品身份、`base_link`、
  `forearm_proximal=link4`、`forearm_distal=link5`、`wrist_housing=link6`、
  `tool_flange=link7` 和 canonical q7 layout。
- `configs/assets/wuji_hand2_beta1_left_v1.yaml` 与
  `configs/assets/wuji_hand2_beta1_right_v1.yaml` 定义左右 Hand 2 身份、侧别和
  side-specific q20 layout。
- Asset 不保存 USD 路径、Isaac prim path、world pose 或 Glove SDK 对象。
- NERO q7 profile 仍为 `provisional_simulation_pending_device_readback`，不能用于批准
  真实机械臂运动。

### Backend Binding

- NERO Binding 固定到 Isaac Sim 6.0.1 的派生 USD，并显式映射
  `base_link`、`link4`、`link5`、`link6`、`link7` 和 `joint1..joint7`。Binding
  compatibility profile 只在 live Isaac stage 中旋转 `link6` 的
  visual/collision 表示并同步 mass properties；J7 joint frame、`link7` 和 Tracker
  Lula URDF 保持固定来源定义。
- 左右 Hand 2 Binding 直接引用 `wuji-description v2026.6.27` 的完整物理 USD，
  分别映射 20 个 canonical finger joint。
- 不生成 visual-only 派生资产；不以省略 q20 route 的方式伪造“不可控手”。
- `namespace_policy: prefix` 保证四个资产实例的 backend symbol 不碰撞。

### Assembly

Assembly 是以两台 NERO 为 root 的 forest：

```text
nero_left  (root) -> link7 -> hand_left  (physical q20 child)
nero_right (root) -> link7 -> hand_right (physical q20 child)
```

当前左右 `link7 → hand_base` attachment 是直接装配：

```text
position_m = [0.023, 0, -0.0235] m
quat_wxyz = [0.70710678, 0, 0.70710678, 0]  # Ry(+90°)
```

平移抵消固定 J7 origin 的 `-23.5 mm` 横向偏置，并把 Hand 2 基座耦合面放到
mesh-derived `link6 +X` 端面中心；`Ry(+90°)` 保持既有手部工作朝向并令盘面平行。
该变换是 simulation-nominal 接口映射，不表示存在直角转接结构。物理对应仍待设备
J7 轴、零位、符号、`link6` clocking 和螺孔方向只读核对。

### Workcell

`configs/workcells/isaac_nero_dual_hand2_simulation_nominal_v1.yaml` 定义：

- `1.20 m × 1.20 m × 0.08 m` nominal 桌面；
- nominal table top 高度 `0.80 m`；
- 左右 NERO mount 位于同一近侧桌沿，分别为
  `[-0.32, -0.52, 0] m` 和 `[+0.32, -0.52, 0] m`；
- 两个 mount 的 yaw 均为 `-90°`，接电侧朝桌内。

接电侧朝桌内由项目负责人通过当前实物确认；`base local -X` 仍是固定 base mesh
外凸特征对应的表示轴，不是接口轴测量。上述 mount 位置与精确 yaw 也不是现场测量。
它们足以关闭资产组合、关节隔离和基础功能仿真 Gate，但不能支持 clearance、
可达空间、精度或安全包络结论。实测 Workcell revision 留给物理对应和真机阶段。

### Session

`configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml`
沿用 Session v1，恰好声明四条 commanded control route：

| instance | group | layout | DoF |
|---|---|---|---:|
| `nero_left` | `arm_joints` | `agilex_nero_q7_v1` | 7 |
| `hand_left` | `finger_joints` | `wuji_hand2_left_firmware_v1` | 20 |
| `nero_right` | `arm_joints` | `agilex_nero_q7_v1` | 7 |
| `hand_right` | `finger_joints` | `wuji_hand2_right_firmware_v1` | 20 |

总计为 54 logical DoF。此 Session 不需要 `inactive` 或 Session v2，也不声明 transport
contract；它是 NV-2 scripted/fixture 仿真的唯一 composition root。Session 的
`runtime.compatibility_profile` 引用
`configs/profiles/isaac_nero_dual_tabletop_qualification_v1.yaml`，只在 Session 层保存：

| 侧别 | q7 准备位（deg） |
|---|---|
| left | `[-10, -45, 0, -45, -90, 0, 0]` |
| right | `[+10, -45, 0, -45, -90, 0, 0]` |

同一 profile 将 NERO q7 Isaac drive gain 设为 `stiffness=6500`、
`damping=220.79402165819616`，并把 Asset/Binding 解析出的 `link4 → link5`
世界轴竖直分量限制为 `<=0.02`。这些是 Isaac-only qualification 数值，不修改通用
NERO profile、固定来源 URDF/USD 或硬件控制器事实。J7 初值仍为 `0°`，J7 限位
不变；Binding overlay 不改动 J7 origin。Tracker Lula 直接使用固定来源 URDF，
Assembly 保持上述固定平移与 `Ry(+90°)` 接口映射。

## 两棵 q27 物理 articulation

五层中的四个 logical control group 不等于四棵 Isaac articulation。
`src/wujihand/adapters/simulation/nero_hand2_twin.py` 在 PhysX 初始化前为每一侧：

1. 从 NERO Binding profile 对 live stage 的 `link6` visual/collision 及 mass
   properties 施加 source-locked 表示对齐，不改 joint frame；
2. 从已解析 Assembly 读取带固定平移和 `Ry(+90°)` 的
   `link7 → hand_base` transform；
3. 把 Hand 2 base 放到目标法兰位姿，避免约束建立时产生 snap impulse；
4. 禁用 Hand 2 USD 原有的 world-fixed `root_joint`；
5. 从该 prim 移除 `ArticulationRootAPI`；
6. author `NERO link7 → Hand 2 base` FixedJoint；
7. 按 joint name 和 USD joint path 核对 q7/q20 分区。

固定上游 USD 保持只读；修改只存在于当前 live stage overlay。最终 stage 恰好有左右
两个 articulation root，每个 articulation 为：

```text
NERO q7 + 同侧 Hand 2 q20 = q27
```

当前资格运行将合并 q27 的 `enabledSelfCollisions` 设为 `false`。所有外部碰撞几何和
外部 contact 仍保留，但同一 articulation 内部的 Hand 2 finger/palm self-collision
不在本次资格范围。若改为启用，必须先增加 NERO collision filtering 并重新执行完整
碰撞资格验证。

## Wuji Glove 到 Hand 2

手部链路保持在五层之外的 input/retargeting adapter 边界：

```text
Wuji Glove hand_skeleton
  -> WujiGloveHandSkeletonAdapter
  -> CanonicalHandObservation
       named MediaPipe 21 landmarks
       float position (21, 3), metres
       side/frame/confidence/time/calibration provenance
  -> WujiHand2RetargetAdapter
       RetargetSession.for_hand(HandModel.WujiHand2, side=...)
    -> HandIntent
         side-specific Hand 2 q20, radians, explicit layout
    -> GloveHand2SimulationController
    -> JointCommandSupervisor
    -> compose_q27_hand_target
    -> corresponding same-side q20 partition in the q27 articulation
```

`hand_joint_angles` 表示人手 21 DoF 结果，不是 Hand 2 q20，不能直接透传。adapter
会拒绝错误 side、缺失点、stale、非单调 sequence/time、NaN 和错误输出 shape；
来源、标定或 frame epoch 切换后必须先 reset retargeter。

完整且 finite 的 skeleton 不再因低置信度被硬阻塞：21 个 landmark 的最低置信度
`<0.60` 时产生 `DEGRADED` intent，`>=0.60` 时为 `SUCCESS`；默认硬拒绝下限为
`0.0`。缺帧或被拒绝的观测不会创建新的 input-derived `HandIntent`；supervisor
可在 `0.25 s` freshness 窗内保持最后有效命令，超时后生成渐进 return-to-rest
安全输出。该安全输出不是由 invalid input 派生的新 q20 intent。

当前 domain、ports、adapter、controller 与 q27 partition composer 已有无硬件测试，
左右实际 live 链路也已执行。设备 identity、正式 handedness calibration revision
与脱敏 replay 仍需补齐为可复现实验材料。

## 来源与可复现性

| 对象 | 固定身份 |
|---|---|
| NERO source | `agilexrobotics/agx_arm_urdf@f6642ce0d7872c686f29c99e9e10cd23d1d49313` |
| NERO URDF SHA-256 | `c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278` |
| NERO mesh tree SHA-256 | `2323405c805cbc4187f451a22d02954fc952a8a2b67b9602119ad26e1ee5031e` |
| NERO derived package tree | `07ba62ca6d7ab79cb76a2148e76743cda78671e1bfa40ad418158554179214a0` |
| NERO main USD | `d8ca2ceb58ad57cab0dc521323b560631207ba1838d4be0c1ddd8f85a27d5289` |
| Hand 2 source | `wuji-description v2026.6.27` / `aee64892ebcf8e3237bedc30231bb09476cbc71d` |
| Hand 2 left USD | `646287f10ac0a2097bf602facc02c9af17f0f1cf8982c38037f69bb695492eca` |
| Hand 2 right USD | `3cb3dcb18b07621a52a47a8daa98ab82794e3c77d36275d068b3b5b0516e5f00` |
| link6 geometry alignment profile | `0dafd7b44904e69cd8742df41088db81b6c5bbcbb49d186273ff645c571e63ae` |
| Lula URDF | 固定来源 URDF：`c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278` |
| tabletop qualification profile | `2c0b4b7d476d03ad63e04deda983523952925e56b7bd23ae771a252f12acd447` |
| simulation nominal Workcell | `488e8ecee966199e87a7d06e978c523586d2af1e607a2f185f1491df2a07edf5` |
| full Session hash（v15） | `46f543efdaa3eff26227ed73150902c20027b90afdc19cb35e9d814098800601` |

## 运行与验证入口

```bash
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_dual_twin.py \
  --session configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml \
  --frames-per-phase 120 \
  --report artifacts/validation/nv2/nero-dual-hand2-tabletop-v15.json \
  --interface-screenshot artifacts/validation/nv2/nero-dual-hand2-right-interface-v15.png
```

### Glove 快速 live 入口

显式 `--glove-live` 进入独立的 `glove_live_only` 分支。它仍先完成五层 Session
解析、双 q27 root、q7/q20 分区、关节限位和固定 workcell collider 检查，但跳过双臂、
逐指、组合手型、reset 与 recovery 的完整 scripted qualification。

输入连接前使用 application 层的 `nv2.glove_live_q27_readiness.v1`：每窗 15 个仿真
帧，最少 2 窗、最多 4 窗，q27 窗间阈值 `0.03 rad`。达到阈值即可提前继续；最多
60 帧后即使未严格收敛也继续，因为该入口只拥有仿真手部命令，不控制真实 NERO 或
Hand 2。默认不带 `--glove-live` 时，完整 scripted 路径继续使用严格的 60 帧窗口、
最多 8 窗和 `0.005 rad` 收敛门。

频繁交互测试使用：

```bash
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_dual_twin.py \
  --session configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml \
  --gui \
  --glove-live \
  --glove-side right \
  --glove-calibration-id wuji_sdk.default_user.builtin.sdk_2026.7.21 \
  --glove-frames 600 \
  --report artifacts/validation/input-smoke/glove-right-hand2/live-right-fast-v1.json
```

控制台依次打印 `GLOVE LIVE READINESS`、`GLOVE LIVE CONNECTING`、
`GLOVE LIVE ARMED` 和首次有效输入时的 `GLOVE LIVE ACTIVE`。报告单独记录就绪
等待时长、SDK connect-to-armed、首个 intent 和首个命令变化时间，不再把 GUI 已出现
到 scripted 阶段结束的等待误认为 Glove SDK 延迟。

### Tracker live 映射入口

Tracker 的现场轴映射不属于 Asset、Binding、Assembly、Workcell 或 Session 产品事实，
因此存放在五层之外的 simulation-only calibration：

```text
configs/calibrations/vive_tracker_workcell_workstation2.yaml
```

其中 `tracker_to_workcell` 是唯一的 `3×3` 轴映射入口，同时用于平移和旋转。当前
Workstation2 人工三方向结果为：

```text
Workcell +X = Tracker -Z
Workcell +Y = Tracker -X
Workcell +Z = Tracker +Y
```

canonical calibration 的平移位移和速度增益均为 `1.0`，逐轴 Workcell 限幅为
`±0.4 m`，对角最大 target 位移约为 `0.693 m`。超过包络继续使用既有逐轴 clamp；
包络内不可达目标仍交给 IK 故障逻辑。runner 与全部 Deployment 只引用该文件。

rotation 使用 reference epoch 的空间相对量：

```text
ΔR_workcell = C · (R_tracker_current · R_tracker_referenceᵀ) · Cᵀ
R_link7_target = ΔR_workcell · R_link7_reference
```

默认保持历史 translation-only 行为；只有显式增加 `--tracker-rotation` 才向 Lula IK
传入变化的 link7 orientation。单独验证 rotation 时再增加
`--tracker-freeze-translation`，mapper 会忽略 Tracker 位移并固定 reference link7
position；这两个开关必须联用。YAML 还持有 translation/rotation scale 与限幅，CLI
override 只用于有意实验。此 calibration 标记为 `simulation_only`，不能直接解释为
真实 NERO TCP 标定或真机安全参数。

OpenVR 的 `Calibrating_InProgress/OutOfRange` 是显式非 actionable 状态，不能因
`bPoseIsValid`、相邻帧曾为 `Running_OK` 或 UDP 仍连通而改写成 running。Tracker
producer 持续发送 running 和非 running 的 canonical sample；状态切换日志中的
`udp=sent` 与 `actionable=false` 分别表示“传输仍连续”和“该帧不可用于新命令”。

Isaac consumer 明确分为两种生命周期：

| 模式 | reference 与结束条件 |
|---|---|
| `--gui --tracker-live` | 交互会话；首个 fresh canonical `Running_OK` 自动建立 reference，不等待回车，也不使用启动稳定窗；窗口由操作者关闭 |
| headless `--tracker-live` | 有界资格验证；默认要求 `0.25 s` 连续 `Running_OK`，执行 `--tracker-frames` 后给出通过/失败并退出 |

GUI 控制状态机位于 application teleoperation：

```text
WAITING_REFERENCE -> TRACKING -> HOLD -> WAITING_REFERENCE
                         ^                    |
                         +---- fresh RUNNING--+
```

短暂非 running 或无新包只保持最后目标且不刷新有效输入时间；持续超过 freshness
窗口后停止生成新右臂命令并等待。恢复时以右臂当时的 q7 feedback 计算 link7 pose，
再用当前 Tracker pose 建立新的 relative epoch，因此不会跳回启动姿态。连续 5 次 IK
失败也只撤销当前映射 epoch 并回到等待状态，不结束 GUI。`--tracker-frames` 在 GUI
模式下忽略；交互报告不执行 Gate 判定，`passed` 为 `null`。

交互报告 v2 将 reference epoch 重建与机械臂回到 rest 明确区分，并保留：

- 每次 reference 重建的直接原因与 Tracker 状态；
- 每个 IK 失败 target 的帧间位移/转速、当前 q7、solver candidate q7 及其关节限位
  余量和 FK residual；
- JointCommandSupervisor 的 position clamp、rate limit 与 reason 计数。

其中关节余量来自固定 URDF 对应的 canonical 仿真 `JointLayout`，只用于区分仿真
IK 失败机制，不是实机安全限位。

headless 路径继续逐包检查启动稳定窗；任一 calibrating/out-of-range/lost sample
或超过 freshness 阈值的空窗都会重置窗口，且不会被同一次 UDP drain 中较新的
running 包遮蔽。旧 `--tracker-auto-reference` 只作为命令兼容选项保留，两个模式都
已经自动建立 reference。

上述状态机不进入或改写五层配置；simulation-only calibration、rotation opt-in、
Lula `link7` frame 和 JointCommandSupervisor 边界保持不变。当前 UDP transport 仍
要求同一 receiver 生命周期内 sequence 严格递增；若 producer 进程重启并从 0
重新编号，应同时重启 consumer。跨 producer restart 的 transport epoch 是独立后续
需求，不与物理遮挡后的 tracking reacquisition 混为一谈。

依据为 Valve OpenVR `Driver_API_Documentation.md` 的 “Poses / ETrackingResult”：
[`Calibrating_*` 尚未完全就绪且不应使用，只有 `Running_OK` 表示已校准的有效姿态](https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md#poses)。

| 入口 | 责任 |
|---|---|
| `tests/contract/test_nero_dual_hand2_physical_session.py` | 五层闭合、四 route、54 logical DoF、来源和 nominal 假设 |
| `tests/unit/test_nero_hand2_twin.py` | attachment 配置与 q27 name/path 分区 |
| `tests/unit/test_hand_teleoperation_contract.py` | canonical observation、HandIntent 和 ports |
| `tests/unit/test_wuji_glove_adapter.py` | Glove SDK 边界、side、header、latest-frame 与失败路径 |
| `tests/unit/test_wuji_hand2_retarget_adapter.py` | `(21,3) m → q20 rad`、side、freshness、confidence 与 reset |
| `tests/unit/test_glove_hand2_simulation_controller.py` | input/retarget/supervisor composition、失败语义与 q27 partition |
| `tests/unit/test_live_readiness.py` | 快速 live 与完整 scripted 的显式 q27 就绪策略 |
| `tests/unit/test_tracker_arm_teleoperation.py` | relative SE(3)、短暂 HOLD、失联后当前 pose 重建 reference 与 IK 失败恢复 |
| `tests/unit/test_tracker_diagnostics.py` | target 帧间运动量、速度和 canonical 仿真关节限位余量 |
| `tests/contract/test_tracker_interactive_runner.py` | GUI 路径无 stdin 阻塞、由 `SimulationApp.is_running()` 持续驱动并输出 reset 诊断证据 |
| `tests/contract/test_hand_observation_jsonl_fixture.py` | strict canonical JSONL contract |
| `tests/unit/test_hand_observation_replay_adapter.py` | SDK-independent bounded replay 与时间重基准 |
| `tests/integration/isaac_nero_dual_asset_smoke.py` | 历史 NERO-only pre-composition 双 q7 smoke |
| `tools/run_isaac_nero_hand2_dual_twin.py` | 完整五层 Session 的双 q27 scripted physical/live opt-in smoke |

证据已拉取到当前仓库的忽略目录：

```text
artifacts/validation/nv2/nero-dual-asset-smoke.json
artifacts/validation/nv2/nero-dual-hand2-tabletop-v15.json
artifacts/validation/nv2/nero-dual-hand2-right-interface-v15.png
artifacts/validation/input-smoke/tracker-right-nero/reference-readiness-current-lula-v1.json
artifacts/validation/input-smoke/tracker-right-nero/tracker-bounded-lifecycle-v1.json
artifacts/validation/input-smoke/tracker-right-nero/synthetic-rotation-corrected-flange-final-v3.json  # 已撤销 J7 定义下的历史证据
artifacts/validation/input-smoke/tracker-right-nero/synthetic-se3-corrected-flange-final-v2.json       # 已撤销 J7 定义下的历史证据
artifacts/validation/nv2/nero-dual-hand2-tabletop-v6.json       # 历史 84/84 基线
artifacts/validation/nv2/nero-dual-hand2-physical-v2.json       # 历史 68/68 基线
artifacts/validation/nv2/nero-dual-hand2-physical-v2.png        # 历史 v2 截图
artifacts/validation/nv2/nero-dual-hand2-physical-headless.json  # v1 历史基线
artifacts/validation/nv2/nero-dual-hand2-physical.png           # v1 历史截图
```

目标机执行时对应路径前缀为
`/home/lenovo/swy/wujihand/`。

当前 tabletop v15 报告的 90 项检查全部为 `true`。报告确认命令后 reset 前后均恰好有两棵
q27 root、q7/q20 partition 稳定、左右 q7 回到 Session qualification profile
指定准备位，并可在 reset 后恢复左中指 PIP 命令。固定工作台 collider
`/World/Workcell/simulation_nominal_table` 存在；初始、每个 scripted hand baseline
及 reset 后的双 q27 静置均在 `0.005 rad` 容差内有界收敛。

v15 的几何测量值来自 Isaac stage：

| Gate | left | right |
|---|---:|---:|
| `link6` 圆柱轴与 `link4 → link5` 小臂轴点积 | `0.999083` | `0.999097` |
| Hand 2 基座端面与 `link6` 端面平行点积 | `0.99999926` | `0.99999958` |
| Hand 2 基座中心—`link6` 端面中心误差 | `28.0 µm` | `21.2 µm` |
| attachment anchor 误差 | `35.1 nm` | `69.5 nm` |
| 接电侧表示轴朝桌内点积 | `1.0` | `1.0` |
| `link4 → link5` 竖直分量绝对值 | `0.01966` | `0.01931` |
| 手纵轴朝桌内点积 | `0.98328` | `0.98328` |
| 掌面朝下点积 | `0.99803` | `0.99805` |

其中 attachment、手轴与掌面值是仿真几何测量；端口点积仅验证
“pinned mesh 推断的 `base local -X` 轴 + nominal mount”的内部一致性。接电侧应朝
桌内已经实物确认，但该 local 轴和精确 mount 仍不得表述为接口轴实测。

报告明确记录 `deliberate_unknown_penetration_probe=false`：它没有引入 deliberate
contact/unknown penetration 场景。same-digit uncommanded linkage 仅作为诊断，
other-finger isolation 才是本轮 Gate。v15 报告 SHA-256 是
`4b689e66ce406f225269637b7e16eda895025daf7708ddde3b43e55f64224e56`，Workcell-owned
右侧接口近景 SHA-256 是
`a202c020f6aa69774a3fdf9a6047cfd1a46d6afe122b20c25b31c31f00496396`。

历史 scripted physical v2 的 68/68
（报告 `5623b8552f54cfd186640a5b857179bca2b7cbd935f4d847451cf1912573b20f`，
截图 `2366d024a8d26f85ca97fd2c79aa190a148270ac8e01be85592587a79e201a6a`）
仍保留为前一版基线；v6、v11 与 v12 均因 J7/Assembly 定义变化转为历史证据，当前
结论以 inward-port tabletop v15 为准。v1 文件只保留为更早历史基线。

## 尚需关闭

1. 记录已打通 Glove 的 side、serial、SDK version、named SDK user 和正式
   handedness calibration revision；当前 builtin calibration 只作为现场 smoke
   provenance。
2. 在当前 `abf48dd4…` Session 资产快照上复验左右快速 live-only 分支；现有左右
   报告属于此前 `4b9f97fb…` 快照，保留为控制链与隔离逻辑证据。
3. 由项目负责人确认 merged q27 self-collision 保持关闭，还是启用并增加 NERO
   collision filtering；确认后补对应 contact/GUI 证据。
4. 执行 deliberate external contact/unknown penetration 场景和异常穿透量化；
   不得以当前 bounded rest settling 或法兰接口近景代替。
5. 首次 live 后保存脱敏 canonical `hand_skeleton` replay 与对应 q20/rejection 记录。
6. 进入真机阶段前取得实体 `link6`/法兰螺孔 clocking 近景或接口图，以及 J7 轴、
   零位、符号和限位只读回读，形成 measured Binding / Assembly revision。

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
