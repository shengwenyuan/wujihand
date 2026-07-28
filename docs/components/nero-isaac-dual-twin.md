# NERO 双实例 + 物理 Hand 2 Isaac 数字孪生

状态：**PARTIAL / 部分完成（NV-2 五层组合与 tabletop v5 82/82 已通过；live Glove、
deliberate contact/penetration、self-collision 最终 Gate 与 measured
Workcell/attachment 未闭合，2026-07-28）**。

当前组件已建立双 NERO、双侧完整物理 Hand 2、nominal 工作台和 Session v1 的五层组合。
目标机上的 Isaac Sim 6.0.1 运行形成左右两棵独立的 q27 articulation。当前
tabletop v5 的 82 项检查全部通过：保留 scripted physical v2 的 68 项 q7、逐指/
组合手型、隔离、finite/limits、reset/topology/recovery 检查，并增加 tabletop
attachment、同侧桌沿 mount、左右准备位及手部朝向的 14 项资格检查。结果不包含真实
NERO 或 Hand 2 运动，也不把 nominal 装配、端口轴假设和工作台数值解释为现场测量事实。

NV-2 尚不能标记为完整完成：

- 专用网口 `enx6c1ff7cd0e76` 尚待临时配置 `192.168.1.10/24`，因此尚无
  `hand_skeleton → canonical → retarget → supervision → Isaac` 的实际 Glove live
  证据；
- 合并 q27 articulation 的最终 self-collision policy 仍待项目负责人确认；
- fixed external collider 与 bounded rest settling 已通过，但 deliberate
  contact/unknown penetration 场景和近景视觉资格尚未执行；
- 正式法兰/转接件 CAD、桌面和底座 mount 仍待 measured revision。

## 能力与边界

| 项目 | 当前状态 | 结论边界 |
|---|---|---|
| NERO 来源与 Isaac 表示 | 已验证 | 固定 URDF、mesh、导入 recipe、派生 USD 和结构报告 |
| 左右 Hand 2 表示 | 已建立 | 直接使用固定版本的完整物理 USD，保留 rigid body、collision、drive 和 q20 |
| 五层组合 | 已建立 | 4 个资产实例、2 个 Assembly root、nominal Workcell、Session v1 |
| 逻辑命令 | 已建立 | 左/右 NERO q7 + 左/右 Hand 2 q20，4 个显式 route、54 logical DoF |
| Isaac 物理拓扑 | 已验证 | 左右各一棵 q27 articulation，共 2 棵 |
| tabletop v5 | 已通过 | 82/82：保留 scripted physical v2 的 68 项，并增加 attachment、mount、q7 准备位和手部朝向 Gate |
| external collision settling | 部分通过 | fixed collider 与各 baseline/reset 后有界静置通过；deliberate contact/penetration 与近景待执行 |
| Glove canonical/retarget/supervision 代码边界 | 已建立并以 fake SDK/composition 测试 | 外部 SDK 类型不进入 domain/ports；invalid/missing 不创建新 input-derived intent |
| 真实 Glove live 路径 | 未执行 | 专用 NIC 待配置静态 IPv4，尚未取得 live `hand_skeleton` |
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
  `tool_flange=link7` 和 canonical q7 layout。
- `configs/assets/wuji_hand2_beta1_left_v1.yaml` 与
  `configs/assets/wuji_hand2_beta1_right_v1.yaml` 定义左右 Hand 2 身份、侧别和
  side-specific q20 layout。
- Asset 不保存 USD 路径、Isaac prim path、world pose 或 Glove SDK 对象。
- NERO q7 profile 仍为 `provisional_simulation_pending_device_readback`，不能用于批准
  真实机械臂运动。

### Backend Binding

- NERO Binding 固定到 Isaac Sim 6.0.1 的派生 USD，并显式映射
  `base_link`、`link7` 和 `joint1..joint7`。
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

当前左右 `link7 → hand_base` attachment 都使用 local `Ry(+90°)`：

```text
position_m = [0, 0, 0]
quat_wxyz = [0.70710678, 0, 0.70710678, 0]
```

该 transform 显式标记为
`simulation_nominal_pitch_plus_90_pending_adapter_cad_measurement`，用于使 Hand 2
纵轴与小臂轴对齐。它只服务功能仿真，不表示真实 NERO 法兰已安装 Hand 2，也不替代
转接件 CAD 或物理测量。

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
| left | `[-10, -60, 0, -30, -90, 0, 0]` |
| right | `[+10, -60, 0, -30, -90, 0, 0]` |

同一 profile 将 NERO q7 Isaac drive gain 设为 `stiffness=3000`、
`damping=150`。这些是 Isaac-only qualification 数值，不修改通用 NERO profile、
固定来源 USD 或硬件控制器事实。

## 两棵 q27 物理 articulation

五层中的四个 logical control group 不等于四棵 Isaac articulation。
`src/wujihand/adapters/simulation/nero_hand2_twin.py` 在 PhysX 初始化前为每一侧：

1. 从已解析 Assembly 读取 `link7 → hand_base` transform；
2. 把 Hand 2 base 放到目标法兰位姿，避免约束建立时产生 snap impulse；
3. 禁用 Hand 2 USD 原有的 world-fixed `root_joint`；
4. 从该 prim 移除 `ArticulationRootAPI`；
5. author `NERO link7 → Hand 2 base` FixedJoint；
6. 按 joint name 和 USD joint path 核对 q7/q20 分区。

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
还会拒绝错误 side、缺失点、低置信度、stale、非单调 sequence/time、NaN 和错误输出
shape；来源、标定或 frame epoch 切换后必须先 reset retargeter。

置信度策略固定为：`<0.90` 拒绝，`[0.90, 0.95)` 只允许以 `DEGRADED` 状态经过
supervisor，`>=0.95` 才是 success。缺帧或被拒绝的观测不会创建新的 input-derived
`HandIntent`；supervisor 可在 `0.25 s` freshness 窗内保持最后有效命令，超时后生成
渐进 return-to-rest 安全输出。该安全输出不是由 invalid input 派生的新 q20 intent。

当前 domain、ports、adapter、controller 与 q27 partition composer 已有无硬件测试；
实际 live 链路仍等待专用 NIC 配置和设备 identity/calibration 证据。

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
| tabletop qualification profile | `dac2127aa57e9509d041d74a50d6ebf96445654dbd2f5635df640442890ba5af` |
| full Session hash | `c082dcc4e6c67375cab30435bd1fbe77fa136e8d1b43d525ba467f0464ed7b4e` |

## 运行与验证入口

```bash
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_dual_twin.py \
  --session configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml \
  --frames-per-phase 120 \
  --report artifacts/validation/nv2/nero-dual-hand2-tabletop-v5.json \
  --screenshot artifacts/validation/nv2/nero-dual-hand2-tabletop-oblique-v5.png \
  --top-screenshot artifacts/validation/nv2/nero-dual-hand2-tabletop-top-v5.png
```

| 入口 | 责任 |
|---|---|
| `tests/contract/test_nero_dual_hand2_physical_session.py` | 五层闭合、四 route、54 logical DoF、来源和 nominal 假设 |
| `tests/unit/test_nero_hand2_twin.py` | attachment 配置与 q27 name/path 分区 |
| `tests/unit/test_hand_teleoperation_contract.py` | canonical observation、HandIntent 和 ports |
| `tests/unit/test_wuji_glove_adapter.py` | Glove SDK 边界、side、header、latest-frame 与失败路径 |
| `tests/unit/test_wuji_hand2_retarget_adapter.py` | `(21,3) m → q20 rad`、side、freshness、confidence 与 reset |
| `tests/unit/test_glove_hand2_simulation_controller.py` | input/retarget/supervisor composition、失败语义与 q27 partition |
| `tests/contract/test_hand_observation_jsonl_fixture.py` | strict canonical JSONL contract |
| `tests/unit/test_hand_observation_replay_adapter.py` | SDK-independent bounded replay 与时间重基准 |
| `tests/integration/isaac_nero_dual_asset_smoke.py` | 历史 NERO-only pre-composition 双 q7 smoke |
| `tools/run_isaac_nero_hand2_dual_twin.py` | 完整五层 Session 的双 q27 scripted physical/live opt-in smoke |

证据已拉取到当前仓库的忽略目录：

```text
artifacts/validation/nv2/nero-dual-asset-smoke.json
artifacts/validation/nv2/nero-dual-hand2-tabletop-v5.json
artifacts/validation/nv2/nero-dual-hand2-tabletop-oblique-v5.png
artifacts/validation/nv2/nero-dual-hand2-tabletop-top-v5.png
artifacts/validation/nv2/nero-dual-hand2-physical-v2.json       # 历史 68/68 基线
artifacts/validation/nv2/nero-dual-hand2-physical-v2.png        # 历史 v2 截图
artifacts/validation/nv2/nero-dual-hand2-physical-headless.json  # v1 历史基线
artifacts/validation/nv2/nero-dual-hand2-physical.png           # v1 历史截图
```

目标机执行时对应路径前缀为
`/home/lenovo/swy/wujihand/`。

tabletop v5 报告的 82 项检查全部为 `true`。报告确认命令后 reset 前后均恰好有两棵
q27 root、q7/q20 partition 稳定、左右 q7 回到 Session qualification profile
指定准备位，并可在 reset 后恢复左中指 PIP 命令。固定工作台 collider
`/World/Workcell/simulation_nominal_table` 存在；初始、每个 scripted hand baseline
及 reset 后的双 q27 静置均在 `0.005 rad` 容差内有界收敛。

v5 的几何测量值来自 Isaac stage：

| Gate | left | right |
|---|---:|---:|
| Hand 2 纵轴与小臂轴点积 | `≈1.0` | `≈1.0` |
| 端口假设轴朝桌外点积 | `1.0` | `1.0` |
| 手纵轴朝桌内点积 | `0.98158` | `0.98180` |
| 手纵轴竖直分量绝对值 | `0.09025` | `0.08771` |
| 掌面朝下点积 | `0.99589` | `0.99607` |

其中 attachment、手轴与掌面值是仿真几何测量；端口点积仅验证
“pinned mesh 推断的 `base local -X` 轴 + nominal mount”的内部一致性，仍待实物确认，
不得表述为真实端口实测。

报告明确记录 `deliberate_unknown_penetration_probe=false`：它没有引入 deliberate
contact/unknown penetration 场景。same-digit uncommanded linkage 仅作为诊断，
other-finger isolation 才是本轮 Gate。v5 报告 SHA-256 是
`27812360eea72ba10f5a7730113400e3c477e1b0ffc60cbcebf3cbf0feab521f`，斜视和俯视截图
SHA-256 分别是
`c92b692edf2200b9948409e3afdc8b12f47334ab6127e5c057c5fc499eab3f9f` 与
`09fe5d2c3a727e6126754546460232a22361d8d683043ac56c204f23493f816a`。

历史 scripted physical v2 的 68/68
（报告 `5623b8552f54cfd186640a5b857179bca2b7cbd935f4d847451cf1912573b20f`，
截图 `2366d024a8d26f85ca97fd2c79aa190a148270ac8e01be85592587a79e201a6a`）
仍保留为前一版基线，但当前结论以 tabletop v5 为准；v1 文件只保留为更早历史基线。

## 尚需关闭

1. 为专用网口 `enx6c1ff7cd0e76` 临时配置 `192.168.1.10/24`，让目标机 Wuji SDK
   识别实际 Glove，并记录 side、serial、SDK version、named SDK user 和对应
   handedness calibration revision。
2. 完成至少一侧 live
   `hand_skeleton → canonical (21,3) m → side-specific retarget → q20 → Isaac`
   自由空间 smoke；另一侧可继续使用相同 contract 的 fixture。
3. 由项目负责人确认 merged q27 self-collision 保持关闭，还是启用并增加 NERO
   collision filtering；确认后补对应 contact/GUI 证据。
4. 执行 deliberate external contact/unknown penetration 场景、异常穿透量化和近景
   视觉检查；不得以当前 bounded rest settling 代替。
5. 首次 live 后保存脱敏 canonical `hand_skeleton` replay 与对应 q20/rejection 记录。
6. 后续取得法兰/转接件 CAD、真实桌面和底座 mount 测量，建立 measured Assembly /
   Workcell revision。该材料不阻塞当前 nominal 功能联调，但在物理对应或真机阶段前
   必须闭合。

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
