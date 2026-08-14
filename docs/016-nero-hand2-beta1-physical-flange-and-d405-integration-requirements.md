# 016：NERO—Wuji Hand2 Beta1 V1 法兰与 D405 集成需求

状态：V1 参数化 CAD 与 Isaac Gate 已完成；待 3D 打印无动力装配和生产尺寸补全

日期：2026-08-13

适用对象：当前项目登记的 NERO 7F 实机、Wuji Hand2 Beta1 `v2026.8.3`、
`hardware/camera_mounts/nero_hand2_beta1_realsense_d405/` 腕部相机方案

本文只冻结结构理念、证据边界和后续 Gate，不授权直接加工、装机通电或真机运动。
当前所有设计版本均先通过 3D 打印样件进行安装与配合测试；测试通过后再由工厂制作金属版，
金属版完成资格验证后才用于实际控制。

V1 保留 Hand2 原有的四胶囊孔法兰盘，不拆解 Hand2，也不设计铁盘下方的腕部直连接口。

## 1. 本轮结论

1. 当前 NERO 真机上的杯形母座，按操作者在 2026-08-12 的照片与实物核对，确认与
   AgileX 固定提交
   [`gripper_flange.stl`](https://github.com/agilexrobotics/agx_arm_urdf/blob/f6642ce0d7872c686f29c99e9e10cd23d1d49313/nero/meshes/gripper_flange.stl)
   的外形一致。该结论只适用于当前已登记实机，不外推到全部 NERO 年代或批次。
2. 历史 Assembly 的 `link7 -> hand_base [0.023, 0, -0.0235] m + Ry(+90°)` 只是一套
   仿真接口坐标映射，不能解释为真机实体转接姿态。它保留用于历史结果复现；新的实体装配
   必须另建版本化 Assembly，不能原地覆盖旧配置。
3. V1 保留 Hand2 原有带 4 个胶囊孔的法兰盘。自研主转接芯的 Hand2 侧形成 4 个胶囊定位
   凸台及对应 M3 主连接位，与原法兰盘配合；不拆除或替换该盘。
4. 自研主转接芯的 NERO 侧形成插入杯形母座的公头、防呆面、端面肩部和 4 个径向 M3
   接口。NERO 原厂杯体及两侧承力挡板继续保留。
5. NERO 杯轴、主转接芯轴和 Hand2 法兰盘轴必须同轴。Hand2 只沿共同中心轴调整远近，
   轴向距离由主转接芯本体厚度确定，不使用额外垫片修正。
6. 主转接芯侧面必须预留新的 D405 转接件专用 M3 螺丝接口。D405 转接件可拆，但不得夹在
   主转接芯与 Hand2 原法兰盘之间，也不得占用或共用 Hand2 的 4 个主连接位。
7. 当前 V1 已实现为参数化 OpenSCAD：NERO `Ø39.6 mm` D 形打印公头、`0.4 mm` 杯口薄
   止挡肩、Hand2 四胶囊定位和独立 `4×M3 + 定位键` D405 副接口。D405 位于各手非拇指侧。
   生成件与 Isaac 结果见
   [转接件目录](../hardware/adapters/nero_hand2_beta1_v1/README.md) 和
   [2026-08-13 验证报告](validation/2026-08-13-nero-hand2-beta1-adapter-v1.md)。

## 2. 事实、版本与假设边界

| 层级 | 当前结论 | 限制 |
|---|---|---|
| 当前实机事实 | 杯形母座外形与固定版本 `gripper_flange.stl` 一致 | 尚未完成杯口内径、深度、孔轴高度、孔位时钟方向和同轴度实测 |
| 厂商资料 | 松灵《NERO用户手册》要求工具插入杯形法兰时注意防呆结构；杯/挡板使用 M3 紧固 | 安装说明没有给出生产公差、螺纹深度、材料和锁紧扭矩 |
| 固定软件版本 | AgileX `agx_arm_urdf@f6642ce...` 给出杯形母座 mesh 与 `link7 -> gripper_flange` 变换 | 属于厂商模型值，不等于当前实机测量值 |
| 补充 CAD | 松灵模型页提供 NERO+强脑与 NERO+两指夹爪法兰文件；当前本地有两份 STEP 和一份 STL | STEP 可作为名义几何依据；STL 只用于位置、包络和避障，不作为孔位/公差依据 |
| Hand2 固定版本 | Wuji Description `v2026.8.3` 提供左右 Hand2 Beta1 整机 STEP，其中包含原四胶囊孔法兰盘 | 官方未提供当前修订版的独立适配挂载件或生产级法兰接口图 |
| V1 项目决策 | 保留 Hand2 原四胶囊孔法兰盘，自研主转接芯与其配合 | 胶囊孔生产尺寸、M3 螺纹深度仍须从真机和固定 STEP 交叉核验 |
| 历史仿真假设 | `[0.023, 0, -0.0235] m + Ry(+90°)` 已用于既有仿真与验证 | 不能据此加工实体件，也不能证明真机 Hand2 安装方向 |

因此，当前可以按官方几何开始 V1 参数化建模，但仍不能释放金属生产图。

## 3. 权威几何与补充资源

### 3.1 AgileX 固定版本资源

项目继续以 `agilexrobotics/agx_arm_urdf` 提交
`f6642ce0d7872c686f29c99e9e10cd23d1d49313` 为上游版本。以下资源已经作为独立固定来源
`agilex-agx-arm-urdf-nero-gripper-flange` 登记在 `third_party/sources.lock.yaml`，不得改为
读取滚动分支：

| 路径 | SHA-256 | 用途 |
|---|---|---|
| `nero/urdf/nero_with_gripper_flange_description.xacro` | `eba3a5e7b7ac82a9d6fd376a8fc9314e0b65fcf07d2dff43c9a34822dd53f44b` | 杯形母座 frame 与质量属性 |
| `nero/meshes/gripper_flange.stl` | `1ae6d1c5af001582a3328564839e8eb5f2acec02e7e2b19e65fc3c43f8a9c95a` | 碰撞、实物外形核对；完整 mesh 包络约 `44 × 55 × 59.984 mm` |
| `nero/meshes/dae/gripper_flange.dae` | `93b4fd381b2edd42a3ec946567027aa3392c65be2f3f00c5711b86f4070980fa` | 视觉模型 |
| `nero/urdf/nero_with_gripper_description.xacro` | `3d7a7bd643156fbef72fe4a418a27a4804e22b89e1c9827ef5af2db3d9fff066` | 原厂两指夹爪装配关系参考 |
| `nero/meshes/gripper_base.stl` | `041b1c8ffa29bff9a5e9350ec9d3e1a0129a5abcba1f347b29b5ad7367ba5f5a` | 原厂插入式公头/防呆外形参考，不作为生产 BREP |
| `nero/meshes/dae/gripper_base.dae` | `92f7c3116b219168c36678a44f56f49c1e63fc080129f8a27fad213727ea8a7d` | 公头视觉参考 |

固定 Xacro 给出的厂商模型变换为：

```text
link7 -> gripper_flange
xyz = [0.031, 0, -0.0235] m
rpy = [-1.5708, 0, -1.5708] rad
```

该变换只定义模型 frame。新实体装配不得再把历史 `Ry(+90°)` 直接叠加到它上面。

### 3.2 松灵/强脑补充 CAD

松灵《NERO 模型文件》公开列出了“NERO+强脑标准版灵巧手法兰连接件”和
“NERO+两指夹爪法兰 STEP”。当前工作站文件先按内容哈希登记；在授权和再分发边界明确前，
不复制进普通 Git：

| 文件 | SHA-256 | 设计用途 |
|---|---|---|
| [NERO+两指夹爪-法兰.STEP](</Users/shengwenyuan/Downloads/NERO+强脑标准版灵巧手 法兰连接件-左右前/NERO+两指夹爪-法兰.STEP>) | `b581f8f5a6eea517a03f055b34fe5e49e1b727e6dbaceec31a5e404dad3826e2` | 杯形母座毫米制 AP203 BREP；NERO 侧主尺寸依据 |
| [NERO-强脑标准手-法兰侧板 1.STEP](</Users/shengwenyuan/Downloads/NERO+强脑标准版灵巧手 法兰连接件-左右前/NERO-强脑标准手-法兰侧板 1.STEP>) | `5dcea4b9a7b9061f442f5b71246e2fc40a855bc3b0d83e2c7ed9b2bfa9398a8f` | 承力侧板和电机输出端孔组的毫米制 AP203 BREP |
| [NERO-强脑标准手-侧板 2.STL](</Users/shengwenyuan/Downloads/NERO+强脑标准版灵巧手 法兰连接件-左右前/NERO-强脑标准手-侧板 2.STL>) | `bbf10dbb540143d86049155306ff6098eaeec5b77ea5bb1eb51c2f84b9c56e7d` | 装配位置、包络和避障参考 |

由这些文件得到的当前名义输入为：

- 杯形母座：内径约 `Ø40 mm`、外圆约 `Ø44 mm`、轴向包络约 `12 mm`；
- 杯上 4 个径向孔：`Ø3.4 mm` M3 间隙孔，按 `90°` 分布，孔轴相对杯端的名义高度约
  `6.5 mm`；
- 法兰侧板：约 `56.36 × 30.85 × 5.3 mm`；电机输出端孔组为
  `6 × Ø3.4 mm, PCD 20 mm`，另含杯形法兰连接孔。

这些数值是 CAD 名义值，不是当前实机公差或验收尺寸。尤其不能把名义 `Ø40` 直接做成
零间隙公头。按“设计信息覆盖度”估计，它们约解决了 NERO 侧 `80%` 的建模输入；这不是
生产成熟度，缺失的独立公头 BREP、配合公差和真机测量仍是释放加工图的硬阻塞项。

### 3.3 Wuji Hand2 Beta1 与现有 D405 资源

Hand2 侧固定使用 Wuji Description `v2026.8.3`、提交
`8271644a78d69ed9a4adcf9165d882c64ad33dfa`：

| 文件 | SHA-256 |
|---|---|
| [`wuji-hand2-description-left_beta1_with_mount_step.STEP`](../third_party/src/wuji-description/v2026.8.3/hand2/hand2_beta1/body/step/wuji-hand2-description-left_beta1_with_mount_step.STEP) | `62f731fcc11e48fa32478f4a730b2e488bdfa05e99c843fe05b670debab74b18` |
| [`wuji-hand2-description-right_beta1_with_mount_step.STEP`](../third_party/src/wuji-description/v2026.8.3/hand2/hand2_beta1/body/step/wuji-hand2-description-right_beta1_with_mount_step.STEP) | `df9ba9fec37f91dc0a2bb0d0b1ff2873159a9df0f7098b31e49059a8159a7067` |

整机 STEP 用于重建原 Hand2 法兰盘安装面、4 个胶囊孔和周边避障。在生产级接口图缺失时，
从装配体反求出的尺寸仍需左右真机复测。官方集成页明确没有提供当前 Beta1 修订版的独立
适配挂载件。

现有 [D405 支架目录](../hardware/camera_mounts/nero_hand2_beta1_realsense_d405/README.md)
是仿真/原型资源。已验证的 D405 `mount_v2` 和当前 v4 right 候选只使用 Hand2 同一侧的两个
胶囊位，与“V1 主转接芯占用全部 4 个胶囊连接位”存在接口冲突。旧支架只保留相机包络、
视轴、支撑路径和手指避障方面的参考；新的 D405 转接件必须改接主转接芯的独立副接口。

## 4. V1 主转接芯结构理念

```text
NERO link7
  -> 原厂 gripper_flange 杯形母座与侧板
    -> V1 主转接芯：Ø40 级插入公头 + 防呆面 + 端面肩部 + 4×径向 M3
      -> Hand2 侧：4×胶囊定位凸台 + 4×M3
        -> Hand2 原四胶囊孔法兰盘（保留）
          -> Wuji Hand2 Beta1

V1 主转接芯侧面
  -> 独立 D405 M3 副接口
    -> 可拆 D405 支架与相机
```

### 4.1 载荷路径

- `Ø40` 级长插入配合面约束径向位移并承担主要剪切/弯矩，防呆面约束时钟方向；杯口薄
  止挡肩只定义轴向到位和端面基准，不把它视为主要抗弯截面。
- NERO 杯上的 4 颗径向 M3 主要负责夹紧、轴向保持和防松，不得作为唯一定位或唯一抗弯结构。
- Hand2 侧由 4 个胶囊凸台与原盘胶囊孔负责平面内定位和防转，原盘贴合面承载，4 颗 M3
  提供夹紧。螺栓间隙不能代替胶囊定位面。
- D405 载荷先进入主转接芯的独立副接口，不通过相机夹层或仅两颗 Hand2 安装螺钉形成偏载。

### 4.2 NERO 侧要求

- 公头、杯内圆柱面、防呆面和端面肩部必须形成完整定位基准体系；公头不能只做一段悬空圆柱。
- 4 个径向孔与杯上 `Ø3.4 mm` 孔按真实孔轴对齐。预期主转接芯侧为 M3 内螺纹，但螺纹
  方向、有效深度、入口倒角和螺钉长度必须由实物与生产图确认。
- 公头直径、圆柱度、同轴度和表面处理后尺寸由实测杯口统计量决定。首次只做可调参数与
  配合试块，不写死生产公差。
- 缺口/防呆方向、四孔时钟方向和出线间隙共同定义唯一安装朝向；不能用视觉上“转 90°”
  代替坐标求解。

### 4.3 Hand2 侧要求

- 保留 Hand2 原有带 4 个胶囊孔的法兰盘，不拆解 Hand2，不访问铁盘下方接口。
- 主转接芯提供 4 个与 Hand2 Beta1 原盘胶囊孔配合的定位凸台，并保留对应 4 个 M3 主连接位。
- 四个胶囊凸台的宽度、中心距、突出高度、根部圆角和配合间隙必须从固定版本 STEP 反求后，
  再用左右真机共同复测；当前 D405 SCAD 中的原型参数不能直接升格为生产尺寸。
- Hand2 原法兰盘中心轴与 NERO 杯轴保持共线。径向平移默认为零；轴向距离由主转接芯本体
  厚度决定，不得使用外加垫片补偿。
- 主转接芯与原法兰盘的贴合面必须完整落座，不得靠螺钉拉弯零件消除间隙。
- 左右手若原法兰盘镜像或出线方向不同，应共享同一 NERO 公头和孔制，仅把 Hand2 端时钟
  方向、相机副接口方向做成显式左右参数。
- 3D 打印 V1 样件首先验证装配、贴合、孔位、螺钉长度和拆装空间，不承载实际控制的动态
  载荷。最终金属件须在材料、螺纹啮合、强度与紧固方案复核后另行制造。

### 4.4 D405 集成要求

- 主转接芯侧面预留独立定位面和一组 M3 螺丝孔，供新的可拆 D405 转接件使用；孔数、孔距、
  有效螺纹深度和工具空间在新 D405 支架定型时冻结。
- 3D 打印样件阶段使用适合反复拆装的热熔螺母/嵌件或捕获螺母，金属版再改为经强度校核的
  M3 内螺纹。
- 新 D405 转接件沿用现有相机包络、视场和线缆方向，以便独立迭代相机俯仰、维护相机并
  复测外参。
- 禁止把相机板夹在主转接芯与 Hand2 原法兰盘的贴合面之间；D405 不得占用 Hand2 的
  4 个胶囊定位位或 4 颗主连接螺钉。
- 若后续把相机承力臂与主转接芯一体加工，相机本体仍保留可拆固定面、重复定位基准和
  外参标定基准。
- 必须复核 D405 视场是否被杯壁、主转接芯、线缆或手背遮挡，并核查全手指运动包络、
  J7 周边包络、连接器弯曲半径与拆装工具空间。

## 5. 坐标系与仿真更新要求

新的 V1 物理装配应显式拆为：

```text
T_link7_hand = T_link7_gripper_flange
             * T_gripper_flange_adapter_core
             * T_adapter_core_hand_flange
```

- `T_link7_gripper_flange` 先采用固定 AgileX Xacro 值，并保留“厂商模型、未实测”标签。
- `T_gripper_flange_adapter_core` 由杯轴、端面肩部、防呆面和 4 个径向孔定义。
- `T_adapter_core_hand_flange` 的径向平移为零，轴向距离由主转接芯厚度求解，时钟方向由
  胶囊孔、手背朝向、出线和 D405 方向共同定义。
- 不再直接填写一个裸 `Ry(+90°)` 来“看起来对齐”。最终变换必须能由上述三个配合 frame
  复算，并在 CAD、URDF/USD 和真机测量表中使用同名基准。
- 旧 Assembly、旧 D405 仿真资产和历史 validation 保持不变；新建 `physical_measured` 或等价
  版本化资产。新资产通过静态装配、碰撞和外参 Gate 后，才允许场景选择它。

## 6. 当前缺失输入

以下任一项缺失时不得发布金属加工图：

1. 可独立制造的 NERO 插入式公头 STEP/BREP，或从原厂装配体中经验证重建出的等价 BREP；
2. 公头与杯的配合公差、表面处理、圆柱度/同轴度和端面垂直度要求；
3. 当前 NERO 真机杯口至少两个正交方向的内径、有效插入深度、肩部位置、4 孔轴高、
   孔位时钟方向和螺钉可达性；
4. 左右 Hand2 Beta1 原法兰盘 4 个胶囊孔的生产级中心距、宽度、深度、圆角、M3 螺纹
   有效深度和允许螺钉长度；
5. 打印样件已冻结 `4×M3 + 定位键` 副接口；金属版仍缺最终孔公差、有效螺纹深度、视轴、
   外参重复定位指标和实物 D405/线缆配置；
6. Hand2、D405、线缆和支架总质量、组合质心、目标最大加速度/急停载荷与安全系数；
7. 主转接芯材料、热处理/表面处理、紧固件等级、锁紧方式与扭矩。

## 7. 实施与验收 Gate

### Gate A：资源与坐标基线

- 校验第 3.1 节已固定资源的 source lock、文件哈希和本地路径；
- 把三份松灵/强脑文件按再分发策略移入受控 restricted source，或明确只保留外部哈希引用；
- 从 NERO、主转接芯、Hand2 原法兰盘和 D405 各自建立可复算的 mating frame；
- 用 CAD 装配证明新模型不依赖历史 `Ry(+90°)`，并保留 Hand2 原四胶囊孔法兰盘。

状态：已完成。最终 D-flat 与厂商杯坐标 `+X` 对齐；内部坐标补偿没有变成实体 90° 转向。

### Gate B：尺寸资格

- 形成真机测量表和测量工具/不确定度记录；
- 制作杯口配合试块，验证装配间隙、镀层余量、插拔和无晃动要求；
- 左右 Hand2 分别复测原法兰盘 4 个胶囊孔、贴合面和 M3 螺纹深度；
- 冻结首版公差链与螺钉长度，证明不会顶底或拉弯贴合面。

### Gate C：无动力样件装配

- Hand2 原四胶囊孔法兰盘保留并参与装配，不拆解 Hand2；
- 杯肩、主转接芯和 Hand2 原法兰盘完整落座；中心轴同轴，防呆方向唯一，轴向距离不靠
  额外垫片修正；
- 4 颗 NERO 径向 M3 和 Hand2 侧 4 颗 M3 均可使用规定工具独立拆装；
- 主转接芯上的 D405 独立 M3 接口可拆装，且不占用或共用 Hand2 的 4 个主连接位；
- 人工静载下无可感知晃动，螺钉不是唯一抗剪/抗弯路径；
- D405 可独立拆装，线缆、相机视场和手指/J7 包络无干涉。

### Gate D：数字资产与安全验证

- 由资格通过的 CAD 生成新 URDF/USD visual、collision、质量和质心数据；
- 新数字资产保留 Hand2 原四胶囊孔法兰盘，并为 D405 副接口建立独立 frame；
- 完成左右装配、全关节包络、D405 遮挡与外参回归；
- 记录与旧仿真 Assembly 的有意差异，不把旧数据集结果外推到新实体装配；
- 真机通电或运动前，按原厂安全说明由人工再次确认载荷、紧固、急停和工作区安全。

状态：打印样件前数字 Gate 已完成。Isaac 离散检查覆盖 49 个 q20 姿态和 29 个 q7 姿态，
结构接触为 0；合成视场和双 q27 稳定性通过。该结果不替代连续碰撞证明、实物 D405 标定或
真机安全验证，详见 [验证报告](validation/2026-08-13-nero-hand2-beta1-adapter-v1.md)。

## 8. 参考资料

- 松灵机器人：[NERO 模型文件](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/alsnhlaawartt3k8)，
  “NERO+强脑标准版灵巧手法兰连接件”“NERO+两指夹爪法兰 STEP”。
- 松灵机器人：[NERO用户手册，2.3.2 末端工具安装说明](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/air57k7k3nhgeuxb)：
  挡板 `6×M3×6`、杯形法兰与挡板 `4×M3×8`、插入时注意防呆结构；灵巧手固定使用
  `4×M3×6`，出线位置与法兰缺口居中。螺钉规格是原厂对应装配说明，不自动成为 V1
  主转接芯的紧固件规格或长度。
- AgileX：固定版本
  [`nero_with_gripper_flange_description.xacro`](https://github.com/agilexrobotics/agx_arm_urdf/blob/f6642ce0d7872c686f29c99e9e10cd23d1d49313/nero/urdf/nero_with_gripper_flange_description.xacro)。
- Wuji：[Description 集成指南，3.3.2 Wuji Hand 2（Beta 1）整机结构件](https://docs.wuji.tech/docs/zh/wuji-description/latest/integration/#332-wuji-hand-2beta-1整机结构件)。
- 项目历史边界：[ADR-0005](decisions/0005-nero-model-source-and-provisional-limits.md)、
  [ADR-0010](decisions/0010-d405-140-simulation-wrist-rig-boundary.md)。
