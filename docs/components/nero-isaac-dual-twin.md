# NERO 双实例 + 物理 Hand 2 Isaac 数字孪生

状态：**PARTIAL / 部分完成（corrected-flange tabletop v11 已在 Workstation2
通过 88/88；纯旋转与有界 relative SE(3) 均通过 240/240；右侧 Glove live 已现场
打通，2026-07-29）**。

当前组件已建立双 NERO、双侧完整物理 Hand 2、nominal 工作台和 Session v1 的五层组合。
目标机 Isaac Sim 6.0.1 运行形成左右两棵独立的 q27 articulation。tabletop v11
的 88 项检查全部通过：保留 scripted physical v2 的 68 项 q7、逐指/组合手型、
隔离、finite/limits、reset/topology/recovery 检查，并增加 20 项 tabletop
attachment、同侧桌沿 mount、左右准备位、手部朝向、小臂近水平及法兰接口资格检查。
结果不包含真实 NERO 或 Hand 2 运动，也不把 nominal 装配、端口轴假设和工作台数值
解释为现场测量事实。

NV-2 尚不能标记为完整完成：

- 右侧实际 Glove live 已取得
  `hand_skeleton → canonical → retarget → supervision → Isaac` 证据；最近一次长运行
  2400 帧中接收 2399 帧、拒绝 0 帧，当前轻量启动分支仍待目标机复验；
- 合并 q27 articulation 的最终 self-collision policy 仍待项目负责人确认；
- fixed external collider 与 bounded rest settling 已通过，但 deliberate
  contact/unknown penetration 场景和异常穿透量化尚未执行；
- 设备 J7 frame、法兰孔位 clocking、桌面和底座 mount 仍待 measured revision。

## 能力与边界

| 项目 | 当前状态 | 结论边界 |
|---|---|---|
| NERO 来源与 Isaac 表示 | 已验证 | 固定 URDF、mesh、导入 recipe、派生 USD 和结构报告 |
| 左右 Hand 2 表示 | 已建立 | 直接使用固定版本的完整物理 USD，保留 rigid body、collision、drive 和 q20 |
| 五层组合 | 已建立 | 4 个资产实例、2 个 Assembly root、nominal Workcell、Session v1 |
| 逻辑命令 | 已建立 | 左/右 NERO q7 + 左/右 Hand 2 q20，4 个显式 route、54 logical DoF |
| Isaac 物理拓扑 | 已验证 | 左右各一棵 q27 articulation，共 2 棵 |
| tabletop v6 | 历史通过 | 84/84：旧 attachment 定义的回归基线 |
| corrected flange tabletop v11 | 通过 | 88/88；法向、clocking 和连接原点三类接口 Gate 均通过 |
| external collision settling | 部分通过 | fixed collider 与各 baseline/reset 后有界静置通过；deliberate contact/penetration 待执行 |
| Glove canonical/retarget/supervision 代码边界 | 已建立并以 fake SDK/composition 测试 | 外部 SDK 类型不进入 domain/ports；invalid/missing 不创建新 input-derived intent |
| 真实 Glove live 路径 | 右侧已执行 | 2399/2400 帧接收、0 帧拒绝；当前迭代进一步拆出快速 live-only 分支 |
| Tracker → 右 NERO 平移 | 人工通过 | Workstation2 三方向已核对；仅为仿真输入映射 |
| Tracker → 右 NERO rotation | 合成通过、待人工验证 | corrected Lula 上 pure-rotation 与有界 relative SE(3) 均 240/240；`--tracker-rotation` 显式启用 |
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
  `forearm_proximal=link4`、`forearm_distal=link5`、`tool_flange=link7` 和
  canonical q7 layout。
- `configs/assets/wuji_hand2_beta1_left_v1.yaml` 与
  `configs/assets/wuji_hand2_beta1_right_v1.yaml` 定义左右 Hand 2 身份、侧别和
  side-specific q20 layout。
- Asset 不保存 USD 路径、Isaac prim path、world pose 或 Glove SDK 对象。
- NERO q7 profile 仍为 `provisional_simulation_pending_device_readback`，不能用于批准
  真实机械臂运动。

### Backend Binding

- NERO Binding 固定到 Isaac Sim 6.0.1 的派生 USD，并显式映射
  `base_link`、`link4`、`link5`、`link7` 和 `joint1..joint7`。Binding 的
  compatibility profile 拥有 source-locked J7/法兰坐标修正；live Isaac stage 与
  Tracker Lula URDF 必须消费同一 profile。
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
position_m = [0, 0, 0]
quat_wxyz = [1, 0, 0, 0]
```

此前位于 Assembly 的 local `Ry(+90°)` 已迁移到 NERO Binding 的 J7/法兰机械帧。
因此 `J7=0` 时 Hand 2 已确认的世界位姿保持不变，但 Assembly 不再描述不存在的
直角转接结构。物理对应仍待设备 J7 轴、零位、符号和螺孔 clocking 只读核对。

### Workcell

`configs/workcells/isaac_nero_dual_hand2_simulation_nominal_v1.yaml` 定义：

- `1.20 m × 1.20 m × 0.08 m` nominal 桌面；
- nominal table top 高度 `0.80 m`；
- 左右 NERO mount 位于同一近侧桌沿，分别为
  `[-0.32, -0.52, 0] m` 和 `[+0.32, -0.52, 0] m`；
- 两个 mount 的 yaw 均为 `+90°`。

`base local -X` 被当前 qualification profile 作为端口方向；它来自固定 base mesh
外凸特征推断，不是实物端口测量。上述 mount 数值也不是现场测量。它们足以关闭资产
组合、关节隔离和基础功能仿真 Gate，但不能支持真实端口朝向、clearance、可达空间、
精度或安全包络结论。实测 Workcell revision 留给物理对应和真机阶段。

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

同一 profile 将 NERO q7 Isaac drive gain 设为 `stiffness=6000`、
`damping=212.13203435596427`，并把 Asset/Binding 解析出的 `link4 → link5`
世界轴竖直分量限制为 `<=0.02`。这些是 Isaac-only qualification 数值，不修改通用
NERO profile、固定来源 URDF/USD 或硬件控制器事实。J7 初值仍为 `0°`，J7 限位
不变；Binding overlay 将 J7 origin 从来源
`[0.70710678, 0.70710678, 0, 0]` 修正为 `[0.5, 0.5, 0.5, 0.5]`。生成的 Lula
URDF 使用同一修正，Assembly 则保持 identity。

## 两棵 q27 物理 articulation

五层中的四个 logical control group 不等于四棵 Isaac articulation。
`src/wujihand/adapters/simulation/nero_hand2_twin.py` 在 PhysX 初始化前为每一侧：

1. 从 NERO Binding profile 对 live stage 的 J7 joint frame 与 `link7` 施加同一
   source-locked 修正；
2. 从已解析 Assembly 读取 identity `link7 → hand_base` transform；
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
`<0.90` 时产生 `DEGRADED` intent，`>=0.90` 时为 `SUCCESS`；默认硬拒绝下限为
`0.0`。缺帧或被拒绝的观测不会创建新的 input-derived `HandIntent`；supervisor
可在 `0.25 s` freshness 窗内保持最后有效命令，超时后生成渐进 return-to-rest
安全输出。该安全输出不是由 invalid input 派生的新 q20 intent。

当前 domain、ports、adapter、controller 与 q27 partition composer 已有无硬件测试，
右侧实际 live 链路也已执行。设备 identity、正式 handedness calibration revision
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
| flange correction profile | `56581f483267761308ecd88ecaba158155bf2e50c6ab937d2157626a872356df` |
| corrected Lula URDF | `aba11058236393943abb9f0a37b32a3d19008436dddf1276d6150bacb22bcd4b` |
| tabletop qualification profile | `f95d0bd34ac592619111112adb851364ac76788f6b14f175a25c49baea22b18c` |
| simulation nominal Workcell | `c507b8f7d0548156949d7d49a0bc5cb7ec1764f355ed4ee8b53ef241f06b40b8` |
| full Session hash（v11） | `5d52774611677a7b4be2b67eaa4462f647b1ee5896c96b045d97adcaadd60bda` |

## 运行与验证入口

```bash
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_dual_twin.py \
  --session configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml \
  --frames-per-phase 120 \
  --report artifacts/validation/nv2/nero-dual-hand2-tabletop-v11.json \
  --interface-screenshot artifacts/validation/nv2/nero-dual-hand2-right-interface-v11.png
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
configs/calibrations/vive_tracker_workcell_workstation2_v1.yaml
```

其中 `tracker_to_workcell` 是唯一的 `3×3` 轴映射入口，同时用于平移和旋转。当前
Workstation2 人工三方向结果为：

```text
Workcell +X = Tracker -Z
Workcell +Y = Tracker -X
Workcell +Z = Tracker +Y
```

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

| 入口 | 责任 |
|---|---|
| `tests/contract/test_nero_dual_hand2_physical_session.py` | 五层闭合、四 route、54 logical DoF、来源和 nominal 假设 |
| `tests/unit/test_nero_hand2_twin.py` | attachment 配置与 q27 name/path 分区 |
| `tests/unit/test_hand_teleoperation_contract.py` | canonical observation、HandIntent 和 ports |
| `tests/unit/test_wuji_glove_adapter.py` | Glove SDK 边界、side、header、latest-frame 与失败路径 |
| `tests/unit/test_wuji_hand2_retarget_adapter.py` | `(21,3) m → q20 rad`、side、freshness、confidence 与 reset |
| `tests/unit/test_glove_hand2_simulation_controller.py` | input/retarget/supervisor composition、失败语义与 q27 partition |
| `tests/unit/test_live_readiness.py` | 快速 live 与完整 scripted 的显式 q27 就绪策略 |
| `tests/contract/test_hand_observation_jsonl_fixture.py` | strict canonical JSONL contract |
| `tests/unit/test_hand_observation_replay_adapter.py` | SDK-independent bounded replay 与时间重基准 |
| `tests/integration/isaac_nero_dual_asset_smoke.py` | 历史 NERO-only pre-composition 双 q7 smoke |
| `tools/run_isaac_nero_hand2_dual_twin.py` | 完整五层 Session 的双 q27 scripted physical/live opt-in smoke |

证据已拉取到当前仓库的忽略目录：

```text
artifacts/validation/nv2/nero-dual-asset-smoke.json
artifacts/validation/nv2/nero-dual-hand2-tabletop-v11.json
artifacts/validation/nv2/nero-dual-hand2-right-interface-v11.png
artifacts/validation/input-smoke/tracker-right-nero/synthetic-rotation-corrected-flange-final-v3.json
artifacts/validation/input-smoke/tracker-right-nero/synthetic-se3-corrected-flange-final-v2.json
artifacts/validation/nv2/nero-dual-hand2-tabletop-v6.json       # 历史 84/84 基线
artifacts/validation/nv2/nero-dual-hand2-physical-v2.json       # 历史 68/68 基线
artifacts/validation/nv2/nero-dual-hand2-physical-v2.png        # 历史 v2 截图
artifacts/validation/nv2/nero-dual-hand2-physical-headless.json  # v1 历史基线
artifacts/validation/nv2/nero-dual-hand2-physical.png           # v1 历史截图
```

目标机执行时对应路径前缀为
`/home/lenovo/swy/wujihand/`。

当前 tabletop v11 报告的 88 项检查全部为 `true`。报告确认命令后 reset 前后均恰好有两棵
q27 root、q7/q20 partition 稳定、左右 q7 回到 Session qualification profile
指定准备位，并可在 reset 后恢复左中指 PIP 命令。固定工作台 collider
`/World/Workcell/simulation_nominal_table` 存在；初始、每个 scripted hand baseline
及 reset 后的双 q27 静置均在 `0.005 rad` 容差内有界收敛。

v11 的几何测量值来自 Isaac stage：

| Gate | left | right |
|---|---:|---:|
| Hand 2 纵轴与 corrected 法兰法向点积 | `≈1.0` | `≈1.0` |
| Hand 2 clocking 轴与 corrected 法兰 clocking 点积 | `≈1.0` | `≈1.0` |
| 法兰—Hand 2 base 原点误差 | `1.207e-7 m` | `4.480e-16 m` |
| 端口假设轴朝桌外点积 | `1.0` | `1.0` |
| `link4 → link5` 竖直分量绝对值 | `0.01807` | `0.01780` |
| 手纵轴朝桌内点积 | `0.98329` | `0.98346` |
| 手纵轴竖直分量绝对值 | `0.05834` | `0.05631` |
| 掌面朝下点积 | `0.99815` | `0.99811` |

其中 attachment、手轴与掌面值是仿真几何测量；端口点积仅验证
“pinned mesh 推断的 `base local -X` 轴 + nominal mount”的内部一致性，仍待实物确认，
不得表述为真实端口实测。

报告明确记录 `deliberate_unknown_penetration_probe=false`：它没有引入 deliberate
contact/unknown penetration 场景。same-digit uncommanded linkage 仅作为诊断，
other-finger isolation 才是本轮 Gate。v11 报告 SHA-256 是
`550b0e2bcbde225b75de77be77cd852913ba057a98600496001903e8c4bceb85`，Workcell-owned
右侧接口近景 SHA-256 是
`0321edea66c3ec3b8ca836db808a4b2fcaebdb6082a694d335c97e08ad61c866`。

历史 scripted physical v2 的 68/68
（报告 `5623b8552f54cfd186640a5b857179bca2b7cbd935f4d847451cf1912573b20f`，
截图 `2366d024a8d26f85ca97fd2c79aa190a148270ac8e01be85592587a79e201a6a`）
仍保留为前一版基线；v6 也因 J7/Assembly 定义变化转为历史证据，当前结论以
corrected-flange tabletop v11 为准。v1 文件只保留为更早历史基线。

## 尚需关闭

1. 记录已打通 Glove 的 side、serial、SDK version、named SDK user 和正式
   handedness calibration revision；当前 builtin calibration 只作为现场 smoke
   provenance。
2. 在目标机复验快速 live-only 分支的就绪时长、首个 intent 与首个命令变化时间；
   另一侧可继续使用相同 contract 的 fixture。
3. 由项目负责人确认 merged q27 self-collision 保持关闭，还是启用并增加 NERO
   collision filtering；确认后补对应 contact/GUI 证据。
4. 执行 deliberate external contact/unknown penetration 场景和异常穿透量化；
   不得以当前 bounded rest settling 或法兰接口近景代替。
5. 首次 live 后保存脱敏 canonical `hand_skeleton` replay 与对应 q20/rejection 记录。
6. 进入真机阶段前取得实体法兰螺孔 clocking 近景/接口图，以及 J7 轴、零位、符号
   和限位只读回读，形成 measured Binding / Assembly revision。

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
