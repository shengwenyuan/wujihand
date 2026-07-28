# ADR-0005：NERO 模型来源与临时仿真限位

- 状态：已接受（NV-2 仿真范围）
- 日期：2026-07-28
- 影响范围：NERO Asset、Isaac Binding、q7 profile、NV-2 tabletop qualification

## 背景

机械臂本体二维码指向《机械臂PIPER NERO（7F）》页面。页面于
2026-07-17 更新，标称负载为 1.5 kg，关节范围如下：

| 关节 | 本体二维码页 | 固定 URDF（约） |
| --- | ---: | ---: |
| J1 | `-157°～157°` | `-155°～155°` |
| J2 | `-102°～102°` | `-99.69°～99.69°` |
| J3 | `-160°～160°` | `-157.56°～157.56°` |
| J4 | `-60°～125°` | `-57.87°～122.61°` |
| J5 | `-160°～160°` | `-157.56°～157.56°` |
| J6 | `-44°～57°` | `-41.83°～54.43°` |
| J7 | `-97°～97°` | `-90°～90°` |

公开《NERO用户手册》V1.0.0“1.2 性能参数”则标称负载 3 kg、J2
`-15°～190°`。两者不是舍入误差，应视为不同产品修订、零位定义或产品资料，
不得拼成同一套参数。固定 URDF 的每个关节范围都是二维码页对应范围的严格子集，
并与固定 `pyAgxArm` 常量的近对称 J2 定义一致。

NV-2 需要先得到可复现的数字孪生，但不得把仿真参数误称为真机安全边界。

## 决策

1. NERO 几何、`base_link → link7` 拓扑、`joint1..joint7` 名称、局部轴和关节
   origin 固定到 `agilexrobotics/agx_arm_urdf` commit
   `f6642ce0d7872c686f29c99e9e10cd23d1d49313`。
2. 二维码页与公开手册分别保留来源和版本冲突，不互相补齐。NV-2 的临时 q7
   位置范围继续采用固定 URDF，并用固定 SDK 常量和本体二维码范围交叉核对。
   profile 状态固定为 `provisional_simulation_pending_device_readback`。
3. 通用 `agilex_nero_q7_provisional_v1` profile 的零位姿态和限位语义不因本轮
   tabletop 布局而改变。左右 tabletop 准备位只存放在
   `isaac_nero_dual_tabletop_qualification_v1`，由对应 Session 显式引用，且标记为
   `simulation_nominal`，不是可下发真机的安全姿态。
4. 仿真速度上限仍为临时配置，不能证明真机当前最大速度。URDF 的
   `effort="100"`、质量、惯量、碰撞体、阻尼，以及 tabletop qualification 的
   drive gain 均不是硬件额定值或控制器事实；drive gain 仅用于 Isaac
   qualification 的稳定性和到位误差验证。
5. canonical `tool_flange` 对应 `link7`。ROS 中零偏移 `tcp_link` 和可选
   `gripper_flange` 不替代此语义。
6. Isaac 资产只由固定的 Isaac Sim 6.0.1 / URDF Importer 3.11.2 recipe 派生；
   来源 URDF、引用 mesh、recipe 和输出 package 分别校验 hash。
7. 初始导入保留来源碰撞几何并关闭 self-collision；其能力范围标记为未完成，
   直到 NV-2 collision review 给出证据。
8. 在真实 NERO 运动前，分别只读获取两台设备的固件、当前 q7 范围和最大速度，
   对齐零位、符号与 J2 坐标定义。若与临时 profile 不兼容，则暂停并修订本 ADR。

## 结果

- NV-2 可在 tracking 和真机控制环境未完成时独立推进。
- 同一 NERO Asset 可以复用为左右实例，所有 backend symbol 由 Binding/Session
  namespace 隔离。
- 固定 URDF 可作为当前二维码对应 NERO 的保守 NV-2 仿真范围；这一判断不等于完成
  真机型号、零位或限位验证。
- 通用 q7 profile 与 tabletop qualification 姿态分离；后者不会污染可复用的机械臂
  模型事实。
- Hand 2 到 `link7` 的变换仍是独立的 `simulation_nominal` Assembly attachment
  assumption，不由本 ADR 推断为真实机械适配关系。

## 依据

- 本体二维码页，《机械臂PIPER NERO（7F）》（页面更新时间 2026-07-17）：
  - Source URL：
    [https://qr61.cn/oMm9uo/q4oW6ZW](https://qr61.cn/oMm9uo/q4oW6ZW)
  - 取回日期：2026-07-28
  - 内容快照 SHA-256：
    `67663ff94a05e642a43162c2ff4a1a95d1926a6236114f9904d1544b66e9c700`
- 松灵机器人，《NERO用户手册》V1.0.0：
  - “1.2 性能参数”
  - “3.3 机械臂DH参数说明”
  - “6.8.1 关节限制设置”
  - Source URL：
    [https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/air57k7k3nhgeuxb](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/air57k7k3nhgeuxb)
- 松灵机器人，《Nero-7轴机械臂使用资料》，“结构模型”“二次开发链接”：
  [https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/hf8x32y0tevqyi3g](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/hf8x32y0tevqyi3g)
- AgileX，固定版本
  [`nero_description.urdf`](https://github.com/agilexrobotics/agx_arm_urdf/blob/f6642ce0d7872c686f29c99e9e10cd23d1d49313/nero/urdf/nero_description.urdf)
- AgileX，固定版本
  [`pyAgxArm` joint constants](https://github.com/agilexrobotics/pyAgxArm/blob/cc498c00af0bcb9e297943e94f4792c0e3ee5b2c/pyAgxArm/api/constants.py)
- NVIDIA，
  [Isaac Sim URDF Importer Python API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.asset.importer.urdf/config/python_api.html)
