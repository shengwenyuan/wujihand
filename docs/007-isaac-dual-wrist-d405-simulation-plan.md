# 007：双 NERO—Hand 2—v2 Mount—D405 140°纯仿真集成计划

- 状态：实施中（S0—S1 已通过）
- 日期：2026-08-04
- 范围：Isaac Sim 6.0.1 双侧腕部组件、碰撞、纯仿真相机与 ROS2 录制
- 前置基线：`edd0e74`（v2 mount）、`1ff6ad1`（ROS2—Isaac 60 Hz）
- 相关计划：[005：ROS2—Isaac 60 Hz 控制](005-ros2-isaac-60hz-control-feature-plan.md)
- 相关决策：[ADR-0005](decisions/0005-nero-model-source-and-provisional-limits.md)、
  [ADR-0009](decisions/0009-ros2-full-causal-recording-boundary.md)

## 1. 目标与不可越界边界

本计划把左右两个 Hand 2 Beta1 腕部都装配为：

```text
NERO link7 -> Hand 2 base -> v2 mount -> D405 housing -> synthetic optical camera
```

所有 NERO 双臂 Isaac GUI 和 ROS2 主线默认加载左右两套组件。组件始终具有外观、
碰撞和 Camera prim；只有 `record=true` 才创建双侧数据 render product、发布相机数据并
进入 rosbag2 MCAP。

本功能是**纯仿真数据采集功能**，不得加入或预埋以下能力：

- RealSense 设备发现、序列号、USB、固件或 `pyrealsense2` 路径；
- factory calibration、实体相机标定文件或实体 mount 误差模型；
- 仿真参数向实机参数的 fallback、自动选择或“以后可复用”分支；
- NERO、Hand 2、CAN 或任何实体设备命令。

若未来需要实体 D405，必须建立独立功能、独立配置和独立验收；不得复用本计划中的
`140°` 内参或把本计划的外参称为实机标定。

## 2. 已冻结决策

| 项目 | 决策 | 事实分类 |
|---|---|---|
| 镜头 | 水平 FOV 固定为 `140°` | 用户选择的纯仿真特殊配置 |
| 图像 profile | 左右均为 `640×480 @ 30 Hz` | 用户选择的纯仿真配置 |
| RGB | `sensor_msgs/Image`，`rgb8` | 仿真输出合同 |
| 深度 | `sensor_msgs/Image`，`32FC1`、单位米、optical `+Z` depth | 仿真输出合同 |
| 畸变 | pinhole，`D=0` | 仿真模型事实，不代表实体镜头 |
| 左右关系 | Hand 2 base 的 XZ 平面镜像，即 `Y -> -Y` | 固定左右资产下的仿真假设 |
| 初始 q7 | 左右 `[∓10°, 45°, 0°, 45°, 90°, 0°, 0°]` | `isaac_nero_dual_tabletop_qualification_v1` 仿真准备位 |
| NERO—Hand 2 | `[0.023, 0, -0.0235] m + Ry(+90°)` | ADR-0005 simulation-nominal Assembly |
| static inspector 生命周期 | settle 后 `pause`，再截图/打开 GUI；退出 GUI 后才 `stop` | Isaac 检查入口合同 |
| self-collision | 先对现有合并 q27 articulation 单独开启并通过 Gate | 用户要求的仿真碰撞阶段 |
| mount/camera collision | self-collision Gate 通过后才加入 | 用户要求的阶段依赖 |

`140°` 与 `640×480` 的组合在当前 pinhole 模型下产生约 `128.226°` 的垂直 FOV。
代码、配置、日志、manifest、报告和截图说明必须使用明确标识，例如：

```text
synthetic_wide_angle_140_simulation_only
simulation_only: true
```

不得使用 `D405 nominal intrinsics`、`D405 calibrated` 或其他可能被解释为实体设备能力的
名称。D405 在本功能中只定义外壳、安装面和仿真传感器的载体身份；`140°` 投影不是
RealSense D405 的物理规格。

未来所有 author Camera prim 或导出 camera metadata 的实现点，都必须紧邻保留等价代码
注释；它是审核要求，不是可省略的说明：

```text
SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense D405 specification or calibration.
```

## 3. 当前基线与已确认问题

### 3.1 几何与姿态

- 已提交右侧
  `hardware/camera_mounts/nero_hand2_beta1_realsense_d405/`
  `nero_hand2_beta1_realsense_d405_wrist_mount_v2.scad`。
- 当前右件 SCAD 的 D405 `rear_mount`/camera plate 参考原点为
  `(-55, +90, +30) mm`，housing attachment 姿态为 `Rz(-55°) Ry(+58°)`；左件按 XZ
  平面镜像后为 `(-55, -90, +30) mm`、`Rz(+55°) Ry(+58°)`。这些量不是偏心 optical
  origin 或 ROS optical rotation；后者必须由第 5.2 节的左右 D405 资产合同解析。
- 右侧已验证 mesh `body_count == 1`。左侧必须独立导出、独立验证，不能由“镜像不改
  拓扑”代替产物 Gate。
- 固定 NERO URDF/USD、`link6` 表示对齐和 `link7 -> hand_base` Assembly 均保持不变；
  不旋转 Hand 2，也不改写 NERO joint origin、axis 或 q7 定义。

### 3.2 GUI 姿态复位

临时 D405 runner 在写完截图与报告后、进入 GUI loop 前调用 `world.stop()`；Isaac 随后
恢复默认 articulation 状态，所以 GUI 一打开便看到双臂零位朝上和右手大角度变向。
这不是 Session、qualification profile 或 Assembly 配置丢失。

新的 D405 static inspector 必须遵循：

```text
PLAY -> apply targets -> settle -> PAUSE -> pose snapshot
-> render-only capture -> GUI inspection -> STOP on close
```

native/ROS live GUI 仍持续执行既有 `120/60/20 Hz` 循环，不套用 static inspector 的
pause 生命周期。

`--hand-pose rest` 只选择 Hand 2 q20 rest；NERO q7 始终来自 Session 引用的 tabletop
qualification。新入口不保留含混的 `--right-hand-pose rest` 语义。

### 3.3 材质 warning

`texture_wrap_v ... not available in the MDL representation` 指向上游 Hand2Left 的
OmniPBR 材质，不影响 articulation、mount 或相机 pose。只要最终外观正确，该 warning
记录为已知非阻断项；本功能不修改上游 Hand 2 材质以消除日志噪声。

### 3.4 录制缺口

当前 `record=true` allowlist 没有 image、CameraInfo 或外参 topic；现有 `render_index`
只代表 GUI 预览，headless 下不会产生数据相机帧。现有 manifest 也没有双侧活动相机的
实际 K/D/R/P、分辨率或逐帧外参。该缺口必须在本功能内闭合，不能由 bag 到达时间或
当前 USD pose 离线猜测。

## 4. 五层架构与事实所有权

腕部 mount 和相机随 Hand 2 运动，因此属于 Assembly，不属于固定 world Workcell。
Workcell 继续只拥有桌面、NERO root mount 和固定观察相机 frame。

### 4.1 被动 Asset/Binding 的版本化扩展

现有 `wujihand.asset_manifest.v1` 只允许 arm/hand/virtual mechanism，且强制至少一个
control group。不得用 dummy DoF 把 mount 或相机伪装成可控机构。

实施时新增版本化的 Asset/Binding v2：

- Asset kind 增加 `passive_component` 与 `simulated_sensor`；
- 只有上述两类允许 `control_groups=[]`；
- Binding v2 允许空 `group_bindings`，并分别声明 visual、collision、canonical frame；
- `simulated_sensor` Binding 引用一个 Isaac camera profile；
- mount canonical frame 固定为 `hand_interface`、`camera_interface`；左右 D405 housing
  分别固定 `rear_mount`、`optical`，由于光心横向偏置，二者不得复用同一个
  `rear_mount -> optical` transform；静态外参只能由 Assembly attachment 链解析；
- 左右实例都使用 `namespace_policy=prefix`，不得在 backend root 或 frame 上碰撞；
- resolver 同时读取 v1/v2，但不放宽 v1 的既有不变量；
- Session binding 仍必须覆盖 Assembly 全部实例，control layout 只覆盖真正存在的
  control group。

新增后端无关身份：

- `nero_hand2_beta1_d405_mount_v2_left`；
- `nero_hand2_beta1_d405_mount_v2_right`；
- `realsense_d405_housing_sim_left`；
- `realsense_d405_housing_sim_right`。

左右 D405 资产共享同一份 pinned source、140° camera profile 和生成器，但拥有独立的
visual/collision hash 与镜像后的 `rear_mount -> optical` transform。不得用一个
`side=none` D405 资产掩盖偏心光心的左右差异。

新增 Isaac camera profile：

```text
configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml
```

该 profile 单点拥有 `140°`、`640×480`、`30 Hz`、pinhole、`D=0`、clipping range、
RGB/depth encoding 和 `simulation_only` 标记。runner、launch、SCAD 和 Deployment 不得
复制这些数值。

### 4.2 Assembly

新增版本化 Assembly，保留旧 Assembly 供历史报告复现：

```text
nero_left  -> hand_left  -> mount_left  -> d405_left
nero_right -> hand_right -> mount_right -> d405_right
```

roots 仍只有 `nero_left`、`nero_right`；commanded route 仍只有左右 q7 与左右 q20。
mount/camera 不新增 route、DoF、controller、process 或 execution owner。

### 4.3 Session 与 Deployment

为现有七个 NERO Session 建立显式 D405 版本，并把日常默认入口切换到新版本；旧 Session
文件和 ID 保留，不原地改变历史身份：

- physical simulation nominal；
- teleop；
- native teleop；
- RoboLab base-empty；
- RoboLab banana-bowl simulation；
- RoboLab banana-bowl teleop；
- RoboLab workdesk。

为四个 native/UDP Deployment 和三个 ROS Deployment 分别新增版本化 D405 配置，再把
runner/launch 默认值切换过去；旧 Deployment ID/hash 同样保留，不原地改写。
Deployment 仍只拥有 process、route source、namespace 和 report root，不拥有 camera
intrinsics、mesh transform 或 attachment 数值。

### 4.4 共享 Isaac materializer

`DualNeroHand2IsaacScene` 是唯一生产构景点。新增 side-neutral wrist-rig materializer，
供静态 inspector、native GUI 和 ROS consumer 共用；不得在三个 runner 中复制 overlay。
Session resolver 同时产出 backend-neutral `ResolvedCameraPlan`，使 ROS launch 在 Isaac
启动前即可确定双侧 topic/QoS/recording inventory；launch 不读取 Isaac profile，也不
硬编码左右 camera topic。

当前 `resolve_dual_side_runtimes()` 不能再把 Assembly 中每条 attachment 都解释为
NERO→Hand 2。它必须按 instance role 与 canonical frame 只解析 arm→hand；新的 wrist-rig
resolver 单独解析 hand→mount→camera，并拒绝缺边、重复边、错误 side 或 frame。

物理构建顺序：

1. 加载左右 NERO 与 Hand 2；
2. author NERO—Hand 2 attachment，形成两棵 q27 articulation；
3. 按 resolved Assembly 找到左右被动 wrist-rig subtree；
4. 在 physics 初始化前 author mount/D405 visual、collision 和 Camera prim；
5. reset、施加 tabletop q7 与 q20 rest、settle；
6. 验证两棵 q27 的 root、DoF、partition 和 attachment 未变化。

mount/D405 collision 作为现有 Hand 2 base rigid body 的 compound child shapes；不得新增
RigidBodyAPI、MassAPI、joint、articulation root 或 DoF。本阶段不臆造 mount/D405 质量、
质心或惯量，manifest 明确 `accessory_mass_model: false`。每个碰撞阶段还需比较 Hand 2
base 的 mass、COM 与 inertia readback，证明 PhysX 没有因新增 child shape 隐式改写它们。

## 5. 资产生成、镜像、外观与碰撞代理

### 5.1 可复现来源

- 固定 Hand 2 继续使用
  `wuji-description@aee64892ebcf8e3237bedc30231bb09476cbc71d`。
- 固定 NERO 继续使用
  `agx_arm_urdf@f6642ce0d7872c686f29c99e9e10cd23d1d49313`。
- 实施前把使用到的 `realsense-ros` D405 mesh/xacro 固定到明确 commit、license 和 hash，
  写入 `third_party/sources.lock.yaml`；不得引用 rolling branch。
- mount STL、左右 collision proxy 与对齐后的左右 D405 visual/collision mesh 均由确定性
  工具生成，记录 generator hash、输入 hash、输出 hash、单位和轴约定。

### 5.2 左右镜像

同一 v2 SCAD 以右件作为 canonical geometry；默认输出必须与当前已接受右件几何等价。
左件在 OpenSCAD 导出阶段执行 `Y -> -Y` 并生成独立 STL。D405 同样以对齐后的右侧资产为
canonical source，确定性生成独立左侧 visual/collision 资产。禁止在 USD 中使用
`scale=(1,-1,1)`，避免 determinant、法线、winding 和 collision 错误。

令 `M=diag(1,-1,1)`，以下量均表达在各自 Hand 2 base frame。body/interface frame 使用
proper rotation `R_body_left=M R_body_right M`。optical frame 不能直接套用 body frame 公式；
必须按以下关系从偏心光心、forward 与 up 轴重新构造：

```text
o_left = M o_right
f_left = M f_right
u_left = M u_right
r_left = f_left × u_left = -M r_right
R_optical_left = [r_left, -u_left, f_left]
```

左右 D405 资产分别保存由此得到的 `rear_mount -> optical` transform，不能共享右侧偏置，
也不能把 reflection matrix 直接当 rotation。左右分别验证：

- finite vertices、合法 bounds、watertight；
- welded-vertex-connected `body_count == 1`，并用独立 shared-edge/face component 工具
  交叉验证为一个连通 body；
- 圆孔、两枚胶囊 key、完整中间连杆和 D405 安装板存在；
- 镜像后的 optical rotation 是右手系，`det(R)=+1`；
- 左右孔位、`rear_mount` origin、偏心 optical origin 和 camera interface 满足 XZ 镜像合同；
- rest 与 representative grasp 下不与 pinned moving-link visual/collision 非预期相交。

### 5.3 外观

mount 与 D405 使用稳定、简单的材质，优先 `UsdPreviewSurface` 或确定性 display color；
不依赖有版本歧义的 OmniPBR 参数。左右 visual prim、材质和 mesh hash 进入构景报告。

### 5.4 碰撞代理

不得把带大空隙的 mount 桁架简化为单一 convex hull，否则会在空隙处制造假碰撞。
使用可审计的 compound convex proxy：

- 基座与相机板：box 或小型 convex pieces；
- 每段连杆：capsule/cylinder 或独立 convex piece；
- D405 外壳：box proxy；
- 必要时采用固定参数的多 hull decomposition，并记录产物 hash。

visual 的 `body_count == 1` 与 collision proxy 的覆盖/间隙是两个独立 Gate，不能互相替代。

## 6. Self-collision 分阶段 Gate

当前每侧 NERO 与 Hand 2 已合并为一棵 q27 articulation。Isaac/PhysX 的
USD 属性 `physxArticulation:enabledSelfCollisions` 位于合并后的 articulation root
（通过 `PhysxArticulationAPI.CreateEnabledSelfCollisionsAttr` author）；因此“打开 Hand 2
self-collision”在实现上会先打开整棵 NERO—Hand 2 q27 的 self-collision，而不是一个
仍然独立存在的 Hand 2 articulation。

必须严格按以下顺序实施：

### Gate C0：现有场景基线

- 不加载 mount/camera collision；
- 保持当前 self-collision disabled；
- 重跑 tabletop qualification、q27 partition、两棵 articulation root 和稳定性基线；
- 保存 q27、hand-base transform、solver/contact 基线。

### Gate C1：原始 self-collision 单独测试

- 仅把合并 q27 articulation 的 self-collision 打开；
- 不加入 mount/camera collision；
- 左右单侧分别测试，再运行双侧；
- 覆盖 q20 rest、当前 representative power-grasp 和一组缓慢开合轨迹；
- 记录启动 overlap/contact pair、持续 contact、penetration、solver warning、q27 error、
  NaN/Inf、articulation root/DoF 变化和静态漂移。

rest 下不得存在未解释的持续穿透或姿态爆炸；grasp 中允许有意的手指接触，但不得出现
深穿透、发散或错误跨侧 contact。碰撞阈值必须在版本化 qualification profile 中按
固定 collision mesh 分辨率冻结，不散落在 runner。

如果原始 C1 因 NERO 相邻 link 或上游 Hand 2 collision 的既有重叠失败：

1. 停止，不加入 mount/camera collision；
2. 输出具体 prim pair、时间、penetration/contact 和截图；
3. 设计最小的版本化 filtered-pair profile；
4. 保持 root self-collision 打开，只过滤有证据的固有相邻/重叠 pair；
5. 重新通过 C1。不得回退为全局关闭，也不得静默忽略 contact。

### Gate C2：加入 mount collision

- 只加入左右 mount compound collision；
- 重跑 C1 的全部姿态和轨迹；
- 验证 mount 与手指、NERO terminal、桌面和测试物体的 contact pair；
- 验证 mount 不引入新 rigid body、joint、root 或 DoF。

### Gate C3：加入 D405 collision

- 加入 D405 box collision；
- 重跑 rest/grasp/慢速开合与外部物体接触 smoke；
- 验证相机与手指的自碰撞响应、外部环境碰撞和无初始穿透；
- C0—C3 的 q27 几何、稳定性和控制 Gate 全部通过后，碰撞集成才算完成。

## 7. 140°仿真相机与同帧真值

### 7.1 Camera prim 与坐标合同

左右各一个 wrist optical camera。统一使用 ROS optical 右手系：

```text
+X right, +Y down, +Z forward
```

统一定义 `T_A_from_B`：

```text
p_A = T_A_from_B * p_B
```

单位为米，四元数序为 `wxyz`。不得把 USD camera 的 `+Y up/-Z forward` 原始矩阵直接
当作 ROS optical 外参；转换只在一个 adapter 中完成一次。

### 7.2 静态与逐帧参数

每侧 manifest 静态保存，并为每个字段标明 `authored`、`api_readback` 或 `derived`
provenance：

- camera prim、render product、parent frame、optical frame ID；
- 从 Isaac 6.0.1 API 直接 read back 的 resolution、focal length、horizontal/vertical
  aperture、aperture offset、projection 与 clipping range；
- 从上述 readback 按版本化 pixel-centre/aperture 约定推导的 K、P、horizontal/vertical FOV；
- 合同 author 的 `distortion_model=plumb_bob`、`D=[0,0,0,0,0]`、`R=I`；
- 由 `hand -> mount -> housing -> optical` Assembly 链解析得到的
  `T_hand_base_from_camera_optical`；
- mount/D405/camera profile/Session/Assembly hash；
- `simulation_only=true`、`projection_classification=synthetic_wide_angle_140_simulation_only`、
  `distortion_source=simulator_authored_zero`；
- RGB 与 depth 来自同一个 optical camera、同一个 render product 和同一次 completed
  capture，共享一份 CameraInfo；
- depth annotator 固定为 `distance_to_image_plane`，即 optical `+Z` 深度、`float32`、单位米，
  不是 Euclidean `distance_to_camera`，也不是 D405 stereo/noise/firmware 输出。

S0 API spike 必须冻结 K/P 推导所用的 pixel-centre 位置、aperture-offset 正负号和矩阵布局；
再用固定 3D 标记点的 render pixel 与 K 投影交叉验证。未达到版本化像素误差阈值时，不得
发布 CameraInfo，也不得把推导值标为准确仿真内参。

每帧保存：

- `run_id`、side、`camera_frame_index`；
- 对应 control tick 与完成后的 physics substep index；
- `float64 capture_sim_time_s`、host capture start/end；physics substep index 是精确离散时刻，
  ROS nanosecond stamp 按文档化规则由 simulation time 舍入；
- 同一 capture 的 `T_world_from_hand_base` 与 `T_world_from_camera_optical`；
- RGB、depth、CameraInfo 和 truth 的共同 frame identity。

每帧验证：

```text
T_world_from_camera_optical
≈ T_world_from_hand_base * T_hand_base_from_camera_optical
```

并检查 rotation 正交、`det(R)=+1`、四元数归一及双向矩阵互逆。

### 7.3 图像载荷转换合同

Isaac adapter 只接受并验证以下 completed-frame shape/dtype；不匹配即 fail closed：

- color：`uint8[480,640,4]`、RGBA channel order；取 `RGBA[...,0:3]`，不交换通道、不缩放，
  以 contiguous row-major 发布为 `rgb8`，`height=480`、`width=640`、`step=1920`、
  `is_bigendian=0`；
- depth：`float32[480,640]` 的 `distance_to_image_plane`；finite positive 样本逐值保留，
  annotator 的 no-hit sentinel、非 finite 或非正样本统一写为 canonical IEEE-754 quiet NaN
  `0x7fc00000`，以 `32FC1`、`step=2560`、`is_bigendian=0` 发布；
- 禁止 resize、crop、插值、gamma、颜色校正、深度 noise 或 hole filling。

manifest 记录 source shape/dtype/channel order、source no-hit encoding、canonical conversion
规则和 converter version/hash。S0 必须用 Isaac 6.0.1 实际输出证明 RGBA channel order 与
no-hit source encoding；若 API 行为不同，只能更新版本化合同和测试，不能在运行时猜测。

### 7.4 捕获调度

相机 `30 Hz` 是 simulation-time cadence，独立于 GUI `20 Hz` preview；wall/bag rate
受实际 RTF 影响并单独报告。不能复用 viewport camera、GUI
`render_index` 或截图 render。每个 camera period 恰好包含两个 `60 Hz` control tick、每个
control tick 恰好包含两个 `120 Hz` physics substep；捕获固定在第二个 control tick 的
第二个 substep，即该 camera period 的第 4 个 physics substep 完成且 stage transform
更新之后。

Isaac 渲染存在 frames-in-flight 风险。S0 必须先完成 Isaac 6.0.1 API spike，固定使用
`isaacsim.sensors.experimental.rtx` 的 CameraSensor/RtxCamera、render product 与 annotator
接口，并验证 RGB/depth 能取得同一个 completed-frame identity。若当前 6.0.1 API 无法证明
completed frame 与保存 pose 同帧，则该 Gate 停止，不进入录制实现。

S0 实测确认普通 RGB/depth annotator `info` 为空，`swhFrameNumber` 恒为 `0`；最终合同改为
同一 Replicator Writer 回调绑定 RGB、depth 与 rational `reference_time`。CameraSensor 的
`30 Hz` tick 由 committed timeline app update 驱动；不得用无法重复触发 sensor 的连续
`orchestrator.step()`，也不得对持续 sensor workflow 无界调用 `wait_until_complete()`。
状态切换实测存在一个旧 completed frame，因此生产实现必须按 `reference_time` 连接有界
pose/history，不能用回调到达时的当前 stage pose。

RGB、depth、逐帧外参和时间必须来自同一个已完成 capture tick；对应 CameraInfo 的 K/D/R/P
必须来自同一 active render product 的静态 provenance snapshot 并共享 frame identity。不得在
异步图像返回后读取“当前”USD pose 并贴上同一时间戳。最终采用的有界 app-update/
writer-callback、publisher threading 参数、Writer `reference_time` 与实测 frames-in-flight
都写入 manifest，不以含糊的
“zero-delay”标签代替证据。

捕获生命周期固定为：

```text
create render products/publishers
-> discard documented warm-up frames
-> read back static parameters and write manifest
-> wait recorder-ready
-> reset camera_frame_index to 0 and enable 30 Hz capture
-> stop new captures on shutdown
-> drain in-flight frames
-> terminal status / consumer receipt / rosbag finalize
```

## 8. ROS2 与录制合同

### 8.1 Topic

`record=true` 时每侧发布：

```text
/<root>/<side>/wrist_camera/color/image_raw
/<root>/<side>/wrist_camera/depth/image_raw
/<root>/<side>/wrist_camera/camera_info
/<root>/<side>/wrist_camera/frame_truth
```

新增纯仿真消息 `SimulationCameraFrameTruth`，至少包含第 7.2 节的逐帧字段。ROS consumer
的 TF 必须保持单父节点树：优先复用既有 `world -> ... -> hand_base` robot TF 链，只新增
`hand_base -> optical` static transform。若某 Deployment 不发布 robot TF，才由唯一 owner
发布 `world -> hand_base` dynamic transform；不得与其他 hand-base parent 并存，也不得再
同时发布 `world -> optical`。数据集 join 的权威事实仍是 `frame_truth`，不依赖 TF 到达顺序。

标准 Image/CameraInfo 没有 sequence 字段。四类消息共享精确的 `(stamp,
optical_frame_id)`，只有 `frame_truth` 保存 `camera_frame_index`；禁止把 index 编进
`frame_id`。离线先按 `(side, stamp, optical_frame_id)` 关联 truth，再验证每个 frame
index 恰有一份 RGB、一份 depth、一份 CameraInfo 和一份 truth，不允许 gap、duplicate
或跨帧配对。

### 8.2 `record=false`

- 左右 mount/D405 visual、collision 和 Camera prim 仍存在；
- 不创建数据 render product；
- 不发布 camera topics；
- 不增加 rosbag、camera publisher 或 30 Hz 数据渲染负载；
- GUI 仍只按既有 20 Hz preview 策略工作。

### 8.3 `record=true`

- camera topics 加入 frozen allowlist、QoS、recorder readiness 和 rosbag2 MCAP；
- manifest 在首帧前写入静态 camera inventory；
- consumer receipt 只保存左右 capture/publish count、首末 frame/time、in-flight drain 和
  关闭状态；recorded count 属于 rosbag metadata/finalizer，gap/drop/join 只由 finalize
  后的离线 validator 判断，`receipt.complete` 不证明没有丢帧；
- MCAP、manifest、receipt 与 checksum 继续沿用 ADR-0009 的完整闭合流程；
- RGB/depth 使用无损存储策略；若启用 MCAP/chunk compression，算法和参数必须入 manifest，
  不允许有损图像压缩被当作原始数据。

双侧 raw payload 约为 `129 MB/s`（十进制，不含 ROS/MCAP 开销）。正式采集前必须完成
磁盘持续写入、MCAP finalize、回放和空间预算 Gate；不得因吞吐不足静默降分辨率、降帧率
或丢帧。image/info/truth QoS、rosbag cache、MCAP chunk/storage/compression 参数和最短
持续写入时长必须在版本化 profile 中冻结；吞吐 Gate 至少覆盖一次计划中的最长正式
episode 时长。

### 8.4 离线验证

当前 analyzer 对未知 topic fail closed。相机接入必须同步扩展 reader/validator，但本功能
只增加完整性验证，不增加图像质量、视觉任务成功率或实体相机指标：

- topic/schema/profile 与 manifest 一致；
- frame index、stamp、RGB/depth/info/truth 一一对应；
- 每侧实际 30 Hz、`camera_frame_index` 单调、无 gap/duplicate；
- K/D/R/P shape、finite、分辨率一致；
- 静态/动态外参闭合；
- consumer capture/publish count、rosbag metadata count 与 reader count 按职责闭合。

## 9. 入口覆盖

| 入口 | 必须行为 |
|---|---|
| 新 D405 static inspector | 双侧 visual/collision/camera；settle 后 pause；截图与 GUI 状态一致 |
| `tools/run_isaac_nero_hand2_dual_twin.py` | 默认 D405 Session，左右组件始终存在，不启动 ROS camera recording |
| `tools/run_isaac_nero_hand2_ros.py` | 默认 D405 ROS Deployment；`record=false/true` 按第 8 节分流 |
| `dual_teleoperation.launch.py` | 三个 ROS Deployment 均使用 D405 Session；record allowlist 可在 Isaac 启动前确定 |

Hand2-only、rotation-ball、URDF importer、MuJoCo 和通用 workcell validator 不属于本次
NERO wrist-rig 范围，不应被隐式添加 D405。

现有 Gemini 305 inspector 是旧相机专用、显式启动的 legacy 工具，不是日常双臂 GUI
默认入口。它保持历史复现但从当前启动文档中标为 deprecated；不得让其 overlay、FOV
或配置进入新的 D405 shared materializer。

## 10. 静态 Inspector 与截图验收

新增正式入口：

```text
tools/run_isaac_nero_hand2_realsense_d405_mount_inspection.py
```

它只消费版本化 Session/Assembly/Binding/profile，不接受 CLI 重复输入 `rear_mount`/
optical origin、tilt、FOV 或 collision 参数。默认双手 q20 rest，可显式选择双方一致的
representative grasp；不通过变换 Hand 2 base 实现姿态。

自动保存并人工简单观察：

1. 双臂全景：两臂保持 tabletop q7，左右 mount+D405 全部存在；
2. 左右各一张外侧近景：完整连杆、camera plate 和 D405 housing 可见；
3. 左右各一张接口下侧或爆炸近景：胶囊 key、圆孔和 flange 接触面可见；
4. 左右 `140°` rest optical frame：只让手部占画面小部分，主体 workspace 不被遮挡；
5. 左右 representative grasp optical frame：拇指/食指根部可见，主体仍不被遮挡；
6. collision debug view：mount compound pieces 与 D405 box 可辨认且无虚假跨空隙 hull。

settle 最后一帧保存第一份双 q27、左右 hand-base 与 wrist-rig transform；`world.pause()`
后只执行 render-only `simulation_app.update()`，在输出 `GUI READY` 前保存第二份快照，
要求两者一致。`world.stop()` 只能在 GUI loop 退出后的 `finally` 执行。

## 11. 验证矩阵

### 11.1 普通 Python / 无 GPU

- Asset/Binding v1/v2 schema round-trip 与严格字段；
- passive asset 允许空 control group，robot asset 仍禁止空 group；
- Session binding 覆盖八个实例，control layout 仍恰好四组；
- Assembly forest、左右镜像、frame map、profile 单点事实；
- SCAD/STL body count、hash、bounds、mirror 与 proper rotation；
- collision proxy primitive/compound 合同；
- camera topic allowlist、QoS、RGBA→RGB/depth sentinel conversion、manifest/receipt schema；
- offline RGB/depth/info/truth join 与 gap/duplicate 反例。

### 11.2 Isaac headless

- C0—C3 collision Gate；
- 始终恰好两棵 q27 articulation、每棵 27 DoF；
- self-collision 实际 readback 为 enabled；
- 左右各一套 mount visual/collision、D405 visual/collision 和 Camera prim；
- 无负 scale、无新增 rigid body/root/joint；
- tabletop q7、q20 rest、hand-base 与 camera transforms 达标；
- 两侧 140° authored/readback/derived provenance 与最终 K/D/R/P、profile/分辨率一致；
- headless `record=true` 可产生完整 30 Hz camera frames。

### 11.3 Isaac GUI

- 按第 10 节输出截图；
- static inspector GUI 一打开仍为 settled tabletop，而不是全零 NERO pose；
- 切换 Perspective/left optical/right optical 不改变 stage；
- known OmniPBR warning 不影响 mount/D405 外观。

### 11.4 ROS2 与性能

- 三个 ROS Deployment 的 `record=false` 和 `record=true` 静态展开；
- recorder-ready 必须覆盖新增 camera topics；
- bounded synthetic run、MCAP 回放、receipt/checksum 完整；
- physics `120 Hz`、control `60 Hz`、GUI preview `20 Hz`、双 camera `30 Hz`；
- 不降低 005 已冻结的 control/RTF/延迟 Gate，不把 camera render 算成 GUI render；
- 比较 record off/on 的 schedule miss、RTF、CPU/GPU、磁盘吞吐与内存；
- camera load 若破坏 60 Hz Gate，停止并优化/解耦渲染，不能下调已冻结 camera profile
  或 control rate 规避失败。

正式性能 Gate 必须独占 Isaac/Vulkan/GPU；并行 CAD 或 GUI 预览结果只用于诊断。

## 12. 实施切片与停止点

| 阶段 | 内容 | 通过条件 |
|---|---|---|
| S0 | ADR/schema、source pin 与 Isaac 6.0.1 camera API spike | passive/sensor、K provenance、payload conversion 与 completed-frame identity 可证明 |
| S1 | SCAD 双侧导出与 visual/collision 资产 | 左右 body_count、镜像、hash、proxy Gate 通过 |
| S2 | C0/C1 self-collision 独立资格验证 | 合并 q27 root 保持 enabled 且稳定 |
| S3 | shared materializer + mount collision | C2 通过，所有入口共享一份实现 |
| S4 | D405 collision + Camera prim | C3、双侧外观与 140°截图通过 |
| S5 | static inspector 与 pause 生命周期 | GUI READY 前后 pose 不变 |
| S6 | ROS image/info/truth 与 recorder | 同帧合同、MCAP 闭合、headless 30 Hz 通过 |
| S7 | analyzer、性能与正式文档 | 完整性/60 Hz/吞吐 Gate 通过 |

任何阶段不得越过前一 Gate。特别是 C1 未通过时，不得开始 mount/camera collision；
同帧 camera truth 未通过时，不得把输出称为正式数据集。

## 13. 预计修改面

预计新增或修改：

- `hardware/camera_mounts/nero_hand2_beta1_realsense_d405/`；
- `third_party/sources.lock.yaml` 与确定性 asset builder；
- `src/wujihand/specs/asset.py`、`backend_binding.py` 及 resolver/tests；
- `configs/assets/`、`configs/bindings/isaac/`、`configs/assemblies/`、
  `configs/profiles/`、D405 Session/Deployment；
- 新的 side-neutral D405 wrist-rig simulation adapter；
- `src/wujihand/runtime/isaac_dual_scene.py`；
- 新 static inspector；
- `ros2/wujihand_interfaces/msg/SimulationCameraFrameTruth.msg`；
- ROS conversion、QoS、recording allowlist、launch 与 tests；
- `tools/run_isaac_nero_hand2_ros.py` 的 camera capture/manifest/receipt；
- `analysis/teleoperation_quality` 的 camera integrity reader/validator；
- 对应 component、guide、ADR 和 validation 文档。

不得修改固定 NERO URDF/USD、Hand 2 上游 USD、NERO joint/limit、Tracker mapping、
Lula kinematics 或实体设备 adapter。

## 14. 并行开发与提交顺序

当前 `docs/006-isaac-60hz-lerobot-causal-alignment-plan.md` 与 `docs/index.md` 由另一项
高优先级工作修改。本功能实现前继续遵守：

- 不覆盖未提交的 006/index 变更；
- 出现重叠代码时先在独立 worktree/临时目录保存并让行；
- 等高优先级提交稳定后再基于最新主线重放本功能；
- 每个提交只包含本功能文件，不夹带 `skills/`、`tmp/` 或其他工作树改动。

推荐提交序列：

1. `docs/adr`：纯仿真 140°、passive asset 与 collision Gate；
2. `feat`：双侧 mount/D405 可复现 visual 资产；
3. `feat`：self-collision qualification；
4. `feat`：shared wrist-rig visual/collision/camera materializer；
5. `feat`：static inspector 与截图证据；
6. `feat`：ROS2 camera frame truth 与 record pipeline；
7. `test/docs`：offline integrity、性能、运行指南与 validation。

## 15. 完成条件

- 所有日常 NERO 双臂 Isaac/ROS2 默认 Session 均包含左右 v2 mount 与 D405 housing；
- 左右 visual、compound collision 与 Camera prim 存在，镜像正确，无负 scale；
- 合并 q27 articulation self-collision 保持 enabled，C0—C3 全通过；
- static inspector 使用 pause，双臂不再在窗口打开时恢复全零姿态；native/ROS live GUI
  继续运行既有调度；
- 双侧 `140°`、`640×480 @ 30 Hz` RGB+optical-Z-depth 画面通过人工截图观察；
- `record=true` 的 RGB/depth/CameraInfo/truth 同帧、外参闭合、MCAP 可回放；
- `record=false` 不产生 camera 数据渲染负载；
- 120/60/20/30 调度、RTF、录制闭合和磁盘吞吐 Gate 通过；
- 所有代码注释、配置、manifest 和报告明确标注 `140°` 是纯仿真特殊镜头，无法用于
  或推断实体 D405；
- 仓库不存在实体相机标定、设备发现或未来实机兼容占位逻辑。

## 16. 参考设计

- [Isaac Sim 6.0.1 ROS 2 Camera Publishing](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_camera_publishing.html)
- [Isaac Sim Coordinate Conventions](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/reference_conventions.html)
- [Isaac Sim Episode Recorder](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/extensions/isaacsim.replicator.episode_recorder/docs/index.html)
- [ManiSkill camera observation parameters](https://maniskill.readthedocs.io/en/latest/user_guide/concepts/observation.html)
- [RLBench per-observation camera parameters](https://github.com/stepjam/RLBench/blob/master/rlbench/backend/scene.py#L502-L518)

这些参考只支持“逐帧保存相机参数与 pose”的设计方式；本项目仍以第 7、8 节的明确
坐标、时间和同帧合同为唯一规范。
