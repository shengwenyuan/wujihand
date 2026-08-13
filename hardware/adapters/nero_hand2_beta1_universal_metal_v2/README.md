# NERO—Hand2 Beta1 通用薄型金属芯 V2

本目录是独立于 `nero_hand2_beta1_v1` 和 D405 支架的候选设计，不覆盖旧路径。

## 当前几何

- 单一通用件，无左右版本；
- 主体厚度 3.5 mm，NERO 法兰到 Hand2 外端面轴向距离 15.9 mm；
- 62 × 62 mm 近方形骨架，中央 Ø42 承力环、四个胶囊孔局部圆台和四条对角载荷路径；
- 四个方向各 2 个轴向 Ø3.4 mm M3 间隙孔，中心距 20 mm；
- 保留 V1 的 Hand2 四胶囊定位键、线缆通道和 Ø44.5 × 0.4 mm 杯口薄止挡肩；
- NERO 公头为 Ø39.6 mm、+X 方向 D 弦防呆面，无额外 90°转向；
- 公头插入深度 7.8 mm；四个径向 M3 打印攻丝底孔为 Ø2.7 mm，孔组相对 X/Y 轴旋转 45°，孔轴距杯口 3.5 mm，孔后保留 2.95 mm 材料；
- Hand2 主螺钉采用 Ø6.4→Ø3.4、90°沉头候选结构。

## 文件

- `nero_hand2_beta1_universal_metal_v2.scad`：参数化源文件；
- `generated/print/core_universal.stl`：当前候选 STL；
- `generated/reference/core_universal_pre_plug_fix_reference.stl`：仅用于证明本次修改范围的修正前 V2 对照网格；
- `generated/generation_report.json`：拓扑、重复生成、公头修正范围和 Hand2 接口保留 Gate。

生成命令：

```bash
python tools/build_nero_hand2_beta1_universal_metal_v2.py \
  --openscad openscad \
  --overwrite \
  --verify-repeatability
```

## 使用边界

当前仍是“金属意图、先 3D 打印安装测试”的候选件，不授权带电运动。金属加工前必须完成真机杯径/薄肩、Hand2 胶囊孔与螺纹深度、沉头螺钉兼容性、材料/圆角/疲劳/扭矩及线缆弯曲空间验证。
