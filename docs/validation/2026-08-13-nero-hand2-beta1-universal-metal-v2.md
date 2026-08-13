# NERO—Hand2 Beta1 通用薄型金属芯 V2：截图迭代记录

## 目标与隔离

本轮仅新增 `hardware/adapters/nero_hand2_beta1_universal_metal_v2`，没有修改既有 V1 打印芯、D405 支架、Isaac Session/Assembly 或其他需求路径。

## 收敛结果

- 左右件合并为一个 `core_universal`；
- V1 的 10.0 mm 主体降到 3.5 mm；NERO 法兰至 Hand2 外端面由 22.4 mm 降到 15.9 mm，减少 6.5 mm；
- 删除旧单侧相机凸台，改为 62 × 62 mm 近方形四向骨架；
- 每边 2×Ø3.4 M3 轴向通孔，孔距 20 mm；
- 保留 Ø44.5 × 0.4 mm 杯口薄止挡肩；
- Hand2 主孔改为 90°沉头候选，避免在 3.5 mm 主体内使用 V1 的 3.3 mm 深圆柱沉孔。

## NERO 公头定向修正

- 公头保持 D 形防呆，D 弦与官方 `gripper_flange.stl` 的 +X 弦面零旋转对齐；
- 四个径向 M3 打印攻丝底孔整体旋转 45°，最终 STL 孔口轴角为 45°/135°/225°/315°；
- 插入深度由 5.8 mm 增至 7.8 mm，孔轴仍距杯口 3.5 mm；孔内端材料由 0.95 mm 增至 2.95 mm；
- 线缆缺口、主体、Hand2 胶囊孔和四向附件框全部冻结。

松灵官方手册只明确了防呆插入、四颗 M3 锁紧和出线对准缺口；本次 45°孔位与插深采用当前真机事实，并继续要求先做 3D 打印安装测试。

## Gate

- OpenSCAD 2021.01 连续两次生成 STL 的 SHA-256 相同；
- 51,670 个三角形，单一连通、封闭、绕向一致，无退化三角形；
- 修正前 V2 对照网格 SHA-256 为 `9577dd71923bf183a8bf6531940ec1fa611ca4f9af5c8f03259593665ad17bf0`；最终网格为 `64abd1a384f44abffbc20991f862d31c66a804502ec8bf89920f992e2d62673b`；
- 公头以外冻结区域 23,356 个顶点完全一致，Hausdorff 差为 0；
- 最终 STL 反算 D 弦包络为 X=[-19.8, 18.0] mm、Y=[-19.8, 19.8] mm，孔口轴角为 45°/135°/225°/315°；
- Hand2 四个胶囊键顶端 21,896 个顶点全部保留；没有新增顶点，删去的仅为旧 V1 单侧凸台的 4 个顶点；
- Isaac Sim 6.0.1 最终 14 张 900 × 700 截图全部非空，报告 `passed=true`；
- 未加载 D405 支架，未运行关节或带电控制。

关键证据：

- `artifacts/validation/nero_hand2_beta1_universal_metal_v2/isaac_plug_fix_final/nero_plug_d_flat_face.png`
- `artifacts/validation/nero_hand2_beta1_universal_metal_v2/isaac_plug_fix_final/nero_plug_m3_axis45.png`
- `artifacts/validation/nero_hand2_beta1_universal_metal_v2/isaac_plug_fix_final/nero_plug_m3_axis135.png`
- `artifacts/validation/nero_hand2_beta1_universal_metal_v2/isaac_plug_fix_final/assembly_exploded_interfaces.png`
- `artifacts/validation/nero_hand2_beta1_universal_metal_v2/isaac_plug_fix_final/report.json`

## 尚未满足的生产 Gate

这轮只确认功能布局、名义接口与仿真视觉对齐，不能证明金属强度或真机安全。下一步仍须先 3D 打印安装测试；金属加工前补真机测量、螺钉/螺纹深度、材料与制造圆角、静强度/疲劳、紧固扭矩及线缆空间验证。
