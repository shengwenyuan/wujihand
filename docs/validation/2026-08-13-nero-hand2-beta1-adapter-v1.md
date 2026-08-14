# NERO—Hand2 Beta1 V1 转接件 CAD 与 Isaac 验证

日期：2026-08-13

结论：V1 已达到 **3D 打印无动力装配样件** 状态；不构成金属生产图，也不授权真机通电或
运动。最终 Isaac 报告 `passed=true`。

## 冻结的样件几何

- 保留 Hand2 Beta1 原四胶囊孔法兰盘；主芯提供 4 个胶囊定位凸台和 4 个 M3 间隙孔。
- NERO 侧为 `Ø39.6 mm` D 形打印公头、`5.8 mm` 有效插入段、`0.9 mm` 导入段；另提供
  `Ø39.4/39.6/39.8 mm` 三个配合试块。
- 杯口薄止挡肩为 `OD 44.5 mm × 0.4 mm`，只作为轴向到位基准；径向载荷和弯矩由长
  圆柱/D 形配合面承担。
- NERO 4 个径向 M3 位置使用 `Ø2.7 mm` 打印导孔；孔轴距杯口名义值 `3.5 mm`。
- 主芯厚度 `10.0 mm`，Hand2 法兰外表面到 NERO 杯口为 `10.4 mm`；Assembly 的
  `gripper_flange -> hand_base` 轴向解为 `22.4 mm`。
- D405 使用独立 `4×M3 + 定位键` 副接口，不占用 Hand2 主连接螺钉。左右副接口均放在
  各自非拇指侧，相机光轴指向内掌设计目标 `[20, ±20, 90] mm`。
- D-flat 与厂商杯坐标 `+X` 对齐；坐标链内部已补偿净 `Rz(+90°)`，实体装配不保留历史
  90° 转向。

源文件和打印件见
[hardware/adapters/nero_hand2_beta1_v1](../../hardware/adapters/nero_hand2_beta1_v1/README.md)。

## OpenSCAD 资格结果

远端使用 OpenSCAD `2021.01`，每个打印件重复导出两次并在三角面规范化后比较哈希：

- 7 个打印件全部逐字节可重复；
- 左右 D405 支架显式镜像 Jaccard=`1.0`；
- 左右主芯 NERO 接口区域同一性 Jaccard=`1.0`；
- 所有 STL 均为单一连通体、watertight、winding consistent，退化三角形为 0。

生成报告：
[generation_report.json](../../hardware/adapters/nero_hand2_beta1_v1/generated/generation_report.json)，
SHA-256 `5d801e20e0072d59dabb7b379c3e8cf335083d5befdefbdc518b220b83940f58`。

OpenSCAD 在该无显示远端不能生成依赖 OpenGL/X Server 的 PNG 预览；CSG/STL 导出和上述
拓扑 Gate 正常完成，视觉复核由 Isaac headless RTX 截图承担。

## Isaac 迭代与最终结果

迭代中先发现拇指会穿过拇指侧的副接口/支撑；把左右 D405 接口改到非拇指侧后，拇指干涉
清零。随后把相机壳体沿非拇指侧外移并沿掌宽收中，消除了全上界手势时的小指干涉。

最终检查使用两段明确的物理策略：

1. 开启 merged-q27 self-collision，检查配件与同一 articulation 内的 NERO/Hand2 接触；
2. 关闭 self-collision，恢复项目正式运行策略并 reset/settle，检查双 q27 稳定性。

最终结果：

- q20：49 个姿态，包括 rest、脚本姿态、每个关节上下界和全上下界；结构接触 0；
- q7：29 个姿态，包括 nominal、左右各 7 关节独立上下界；可行 workcell 姿态中配件—环境
  接触 0；
- 19 张 `800×600` 截图全部非空；左右装配、接口近景、碰撞代理和合成手势视图均完成；
- 杯口/薄肩轴向误差最大 `0.021 µm`，D-flat/杯轴/OEM 法兰轴点积均约为 `1.0`；
- 合成 `110°` 设计视场覆盖左右内掌目标盒；这不是实物 D405 标定结果；
- 正式运行策略下最终 60 帧 q27 最大漂移：左 `1.10e-5 rad`、右 `4.88e-5 rad`。

最终报告：
[report.json](../../artifacts/validation/nero_hand2_beta1_adapter_v1/isaac_final/report.json)，
SHA-256 `60d1bdd33c80d5b2d25b7950e14683c1b095aea2566ffecf19f6448b9897f7be`。

代表截图：
[右侧装配](../../artifacts/validation/nero_hand2_beta1_adapter_v1/isaac_final/right_assembly_rest.png)、
[右侧接口近景](../../artifacts/validation/nero_hand2_beta1_adapter_v1/isaac_final/right_interface_under.png)、
[右侧全上界碰撞代理](../../artifacts/validation/nero_hand2_beta1_adapter_v1/isaac_final/right_q20_all_upper_collision.png)、
[左侧合成光学视图](../../artifacts/validation/nero_hand2_beta1_adapter_v1/isaac_final/left_optical_combined.png)。

## 已承认的边界

- 原始 q7 临时下限中，`left J4 lower`、`right J1 lower`、`right J4 lower` 的裸机 Hand2
  本身会进入地面。左 J4 下限时 D405 壳体也有 `0.94 mm` 触地，但同姿态裸机小指已穿地
  `29.1 mm`；该姿态属于现有 workcell 的预先不可行状态，不是配件新增的可用包络损失。
- 这里只离散采样 49 个 q20 和 29 个 q7 姿态，不证明连续配置空间全局无碰撞。
- 合成相机使用设计 FOV，不包含真实 D405 内参、畸变、外参、线缆和连接器包络。
- 当前杯口、公头、Hand2 胶囊孔与 M3 深度仍有未实测项；打印件只能用于手动配合、孔位、
  贴合、拆装和线缆检查。
- 金属版仍需真机量测、公差链、材料/强度、质量/质心、螺纹啮合、紧固扭矩、线缆弯曲半径
  和 D405 标定，之后重新生成并完成数字与实物资格验证。
