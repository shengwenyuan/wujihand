# 五层 Session 组合

状态：2026-07-23，五层配置、无 backend resolver 和既有 runner 兼容桥已接入。

项目以
`Asset Manifest → Backend Binding → Assembly Spec → Workcell → Session`
作为仿真运行配置的单向组合链。它解决“产品是什么、由哪个 backend 表示、如何装配、
放在哪个工作空间、这次运行采用什么合同”五类事实互相混杂的问题；本轮没有新增 UR5、
双臂 Franka、左手、ROS 或 Tracker/Glove 映射。

## 各层事实源

| 层 | 实现 | 配置目录 | 拥有的事实 |
|---|---|---|---|
| Asset Manifest | `wujihand.specs.AssetManifest` | `configs/assets/` | 产品、代际、侧别、语义 frame、control group、layout、DoF 和来源身份 |
| Backend Binding | `wujihand.specs.BackendBinding` | `configs/bindings/<backend>/` | asset revision/side、loader、固定 source revision/artifact、namespace policy、backend frame/joint/actuator 映射和 procedural builder |
| Assembly Spec | `wujihand.specs.AssemblySpec` | `configs/assemblies/` | 带 namespace 的实例、语义 attachment、局部变换和一个或多个 root |
| Workcell | `wujihand.specs.WorkcellSpec` | `configs/workcells/` | world/语义 frame、mount，以及 plane/box/sphere/frustum 物理 primitive |
| Session | `wujihand.specs.SessionSpec` | `configs/sessions/` | backend、assembly/workcell 引用、逐实例 binding、逐 root placement、运行角色、wire contract 和 control layout |

五层 spec 是冻结的值对象，只负责字段和局部不变量，不读取 YAML，也不 import
MuJoCo、Isaac、MediaPipe 或 Wuji SDK。项目相对路径、YAML 加载和期望 ID 校验由
`wujihand.runtime.ConfigRepository` 负责。YAML 内的 `ConfigRef` 必须是规范化的项目
相对路径，拒绝绝对路径、`..`、模糊的 `.`/重复分隔符和越界；CLI 交给 repository
的顶层配置入口可使用仓库内绝对路径，但解析后的跨层引用仍遵守前述规则。strict
YAML loader 会在所有层级拒绝重复 key。

Asset 的 `canonical_profile`/control group `joint_profile` 可空；当前 Asset 不用
backend profile 定义产品身份。FR3 和项目自有 wrist 的 identity provenance 是
`configs/assets/provenance/` 下的 backend-neutral snapshot，Hand 2 则引用官方来源
锁记录。Asset provenance 与 Binding artifact source 相互独立，两份证据都会进入
resolved snapshot。

`wujihand.runtime.SessionResolver` 在 simulator 或设备 SDK 初始化前闭合所有引用，并
校验：

- Session binding 是否恰好覆盖 Assembly 的全部实例；
- Asset 的 revision、side、frame、control group 和 `dof_count` 是否与 Binding
  完全一致；
- Binding backend 是否与 Session backend 一致，artifact 是否存在于
  `third_party/sources.lock.yaml`，且显式 `source_revision` 是否与 lock 的同类
  revision 精确一致；
- attachment 是否引用合法语义 frame，Assembly root 是否恰好放到已有 Workcell
  mount；
- 每个 control group 是否只按 Asset 声明的 `layout_id` 路由一次；
- teleop producer/consumer 是否声明 wire contract，非 teleop Session 是否没有
  transport contract。

解析结果是 `ResolvedSession`。它内部保存不可变的 canonical JSON，并按需返回快照
副本；内容包含规范化五层配置、Asset provenance、使用到的 source-lock revision、
兼容 profile 文件 hash，以及保留类型的显式 CLI override。override 指向现存文件时，
文件内容 hash 也进入快照，因此整数/字符串不会混同，文件内容改变也会改变稳定的
`session_hash`。默认解析只核对
锁文件中的路径和预期 hash，因此无需安装 simulator 或恢复第三方 checkout；需要对
本地 artifact 和 mesh tree 真正计算 hash 时使用
`SessionResolver.resolve(..., verify_artifacts=True)`。实际 runner 仍执行各自既有的
文件和模型结构检查；MuJoCo table、Isaac fixed-hand 与 rotation-ball 路径都会计算
实际资产 hash。

`namespace_policy: prefix` 会在 resolved snapshot 中对 root、frame、joint 和 actuator
形成逻辑限定名并执行全类别碰撞检查；当前 compatibility runner 只接受已验证的既有
实例集合和 topology，尚不会把任意限定名编译成 MuJoCo/Isaac 原生对象。

## 当前 7 个 Session

| Session | 角色 | 组合与合同 | 默认使用者 |
|---|---|---|---|
| `isaac_hand2_fixed_preview_v1` | `simulation` | fixed Hand 2，20 指关节，无 transport | fixed-hand runner 的 `scripted` |
| `isaac_hand2_teleop_v1` | `teleop_consumer` | fixed Hand 2，`wujihand.q20.v1` | fixed-hand runner 的 `udp` |
| `isaac_hand2_right_rotation_ball_qualification_v1` | `qualification` | D6 wrist3 + Hand 2 finger20，无 transport | rotation-ball runner 的 `scripted` |
| `isaac_hand2_right_rotation_ball_teleop_v1` | `teleop_consumer` | D6 wrist3 + Hand 2 finger20，`wujihand.hand_command.v2` | rotation-ball runner 的 `udp` |
| `mediapipe_hand2_q20_udp_v1` | `teleop_producer` | Hand 2 finger20，`wujihand.q20.v1` | MediaPipe 默认或 `--publish-udp-port` |
| `mediapipe_hand2_hand_command_udp_v1` | `teleop_producer` | wrist3 + finger20，`wujihand.hand_command.v2` | MediaPipe `--publish-hand-command-port` |
| `mujoco_fr3v2_hand2_right_table_v1` | `simulation` | FR3 q7 + Hand 2 q20，固定法兰 attachment | MuJoCo table runner |

producer 与 consumer 保持两个进程。`validate_transport_pair()` 会同时核对 wire
contract、layout multiplicity、产品、代际、侧别、语义、DoF 和 command interface：
q20 v1 只能配 fixed-hand consumer，q20+pose v2 只能配 rotation-ball consumer。
Session 是每个进程的 composition root，不是新的 launch graph 或消息总线。未来多个
相同 layout/同侧实例仍需 transport schema 增加显式 stream/channel mapping。

## Workcell 所有权与兼容叶

当前迁移故意保留两种状态：

- `configs/workcells/isaac_hand2_table_v1.yaml` 已直接声明 fixed-hand 场景的 ground，
  table 两个物理 primitive，并拥有手安装 mount 和相机 eye/target 语义 frame；
  runner 验证默认 ground 等价性，并从解析后的 Workcell 读取 table、mount 和相机
  数值。Workcell v1 没有 typed camera/light 分类，fixed-hand 灯光仍由专用 runner
  author。
- MuJoCo table 的完整桌面/四棱台/灯光/physics，以及 rotation-ball 的法兰、桌面、
  球、D6 限位和资格参数，仍分别单点保存在
  `configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml` 和
  `configs/base/hand2_rotation_ball_v1.yaml`。相应 Workcell 只声明当前语义 mount，
  并引用 typed compatibility profile。

`src/wujihand/runtime/session_compat.py` 把已解析 Session 转成现有专用 runner 需要的 typed
配置，并对 Binding、Assembly、Workcell 与 compatibility profile 的关键事实做一致性
检查。这是 strangler 迁移边界；在通用 entity compiler 能消费某类几何前，不把同一
数值复制到五层 YAML。compatibility profile 不是第二个 Session，也不改变 Session
作为唯一组合入口的约束。

legacy MuJoCo typed 数据类已下沉到标准库专用的 `src/wujihand/compat/`，让 runtime
loader 与 adapter 共享合同而不形成 `adapters -> runtime`；公共文件/目录 hash 位于
`src/wujihand/integrity.py`，SourceLock 与 adapter 共同使用并拒绝资产树中的 symlink。

## 固定资产与表示

Hand 资产身份为 `wuji_hand2_beta1_right`。MuJoCo 和 Isaac Binding 都固定到
`wuji-description v2026.6.27`
（commit `aee64892ebcf8e3237bedc30231bb09476cbc71d`），分别使用
`hand2_beta/body/mjcf/right.xml` 和
`hand2_beta/body/usd/right/wujihand.usd`。这些路径属于 Backend Binding，不属于
Asset Manifest；滚动官方文档或后续 release 出现新目录时不得静默改写当前 Session。

FR3 Binding 固定到
`mujoco_menagerie@71f066ad0be9cd271f7ed58c030243ef157af9f4`。所有预期文件和
mesh tree hash 的规范来源是 `third_party/sources.lock.yaml`。现有
`configs/base/`、`configs/profiles/` 中仍有为 legacy loader 保留的 commit/hash
副本。compatibility bridge 会交叉校验运行时实际消费的 artifact 路径/hash、关节
顺序和场景事实；未被读取的 legacy commit/tag 文本只是报告元数据，不构成独立
source-lock gate。后续提升 leaf 时应删除这些镜像，而不是让它们成为第二个规范来源。

## Runner 与兼容参数

四个 runner 都接受 `--session`：

```bash
.venv/bin/python tools/run_mujoco_fr3_hand2_table.py \
  --session configs/sessions/mujoco_fr3v2_hand2_right_table_v1.yaml \
  --duration-s 5
```

```bash
"$ISAAC_SIM_ROOT/python.sh" tools/run_isaac_hand2_teleop.py \
  --session configs/sessions/isaac_hand2_fixed_preview_v1.yaml \
  --command-source scripted --frames 600
```

```bash
"$ISAAC_SIM_ROOT/python.sh" tools/run_isaac_hand2_rotation_ball.py \
  --session configs/sessions/isaac_hand2_right_rotation_ball_qualification_v1.yaml \
  --command-source scripted --frames 1200
```

```bash
.venv/bin/python tools/run_mediapipe_hand2_teleop.py \
  --session configs/sessions/mediapipe_hand2_hand_command_udp_v1.yaml \
  --publish-hand-command-port 49152
```

省略 `--session` 时，runner 按上表和 `--command-source`/publish 参数选择兼容默认值，
所以既有命令不需要新增必填参数。显式 Session 的 backend、runtime role 或 transport
contract 与当前命令模式不一致时会在启动前拒绝。

保留的旧参数是显式 override：

| Runner | 兼容 override |
|---|---|
| MuJoCo table | `--scene-profile` |
| Isaac fixed-hand | `--asset`、`--profile` |
| Isaac rotation-ball | `--asset`、`--profile`、`--scene-profile` |
| MediaPipe | `--publish-udp-port` 与 `--publish-hand-command-port` 继续选择 v1/v2 wire contract |

override 会连同原始类型及文件内容 hash 进入解析快照和 `session_hash`。它只用于
复现旧命令或有意实验，不会修改五层 YAML；专用 runner 后续仍执行相应 profile、
实际资产 hash、关节布局和模型检查。需要特别区分：fixed-hand 同时覆盖
`--asset`/`--profile` 时，只证明二者内容 hash、layout/rest 与模型结构自洽，可有意
脱离 Session Binding/source lock；只有默认 Session 路径保留 pinned provenance
结论。`session_hash` 会记录该实验输入，但不会把它重新变成锁定 representation。
MuJoCo 与 Isaac 报告新增 `session`/`session_hash`，MediaPipe 启动日志打印相同信息。

## 验证

五层 fast suite 不需要 GPU、相机、MuJoCo 或第三方资产 checkout：

```bash
.venv/bin/python -m pytest \
  tests/unit/test_common_spec.py \
  tests/unit/test_asset_spec.py \
  tests/unit/test_backend_binding_spec.py \
  tests/unit/test_assembly_spec.py \
  tests/unit/test_workcell_spec.py \
  tests/unit/test_session_spec.py \
  tests/unit/test_config_repository.py \
  tests/unit/test_session_resolver.py \
  tests/unit/test_session_compat.py \
  tests/contract/test_layered_sessions.py \
  tests/contract/test_architecture_dependencies.py
```

其中 contract suite 解析全部 7 个 Session、核对 v1/v2 producer-consumer 配对、确认
fixed-hand Workcell 数值归属，并把 MuJoCo/rotation-ball 五层组合与现有 typed leaf
逐项对照。依赖测试阻止 `specs -> runtime/adapters/外部 SDK` 和
`adapters -> runtime`。

恢复固定资产并安装 `mujoco` extra 后，再运行真实 MJCF 组合与 headless 回归：

```bash
.venv/bin/python -m pytest \
  tests/contract/test_mujoco_fr3_hand2_model.py \
  tests/integration/test_mujoco_fr3_hand2_smoke.py
```

Isaac authored-stage、PhysX 抓取和真人 MediaPipe 回归不能由普通 fast suite 代替。它们
分别需要 Isaac Sim 5.1.0 + 可用 GPU/驱动，以及 D435/兼容相机、MediaPipe 模型和 Wuji
retarget 依赖；执行命令与人工验收项见
[MediaPipe 控制 Hand 2 转向抓球指南](../guides/mediapipe-hand2-rotation-ball.md)。
缺少这些环境时应记录为“未执行”，不能视为通过。

本次实际资产、MuJoCo、Isaac GPU、q20 UDP 和 D435I/MediaPipe smoke 结果见
[2026-07-23 五层架构与既有仿真链路验证](../validation/2026-07-23-five-layer-architecture.md)。

## 当前边界

- 五层 schema 已能表达 multi-root forest，但当前 7 个 Session 没有新增 UR5、双臂、
  左手或任意新资产。
- resolver 负责配置闭合和 provenance，不是跨 MuJoCo/Isaac 的万能 scene compiler。
- `session_hash` 证明规范化配置输入一致，不证明 backend 执行成功；只有
  `verify_artifacts=True` 或实际 runner 的 hash/模型检查才能证明本地资产内容一致。
- 当前 MediaPipe producer Session 为保证现有 runner/consumer 配对仍选择目标 Isaac
  backend、Binding 和 Workcell。这是兼容期选择，不是 backend-neutral teleop target
  合同；未来 MuJoCo/ROS 编排应把输入生产、目标 layout 与 backend launch 映射进一步
  分离。
- multi-root/prefix 已证明配置表达、逻辑 symbol 资格化与碰撞防护，不代表现有
  MuJoCo/Isaac compiler 已支持任意双臂；多同构遥操作流还缺显式 stream/channel
  correspondence。
- 当前 compatibility bridge 仍面向三个既有场景；提升任何 typed leaf 前都必须先明确
  Workcell/Binding/Session 的唯一事实所有者，并补行为对照测试。

架构决策与重验触发器见
[ADR-0003](../decisions/0003-five-layer-session-composition.md)。
