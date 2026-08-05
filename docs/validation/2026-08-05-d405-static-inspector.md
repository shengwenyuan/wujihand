# 双侧 D405 static inspector 与 pause 生命周期

- 日期：2026-08-05
- 环境：workstation2，Isaac Sim 6.0.1，RTX 5090
- 结论：007 S5 通过
- 正式入口：`tools/run_isaac_nero_hand2_realsense_d405_mount_inspection.py`

> 2026-08-05 修订：当前正式入口保持 merged-q27 self-collision disabled。下述已保存
> rest/grasp 数值来自修订前报告，仅作为 pause 生命周期与画面历史证据。

## 构景与姿态合同

CLI 只选择版本化 Session、settling/pose qualification profile、`rest/grasp` 和初始视图；
不接受 rear-mount、optical origin、tilt、FOV 或 Hand 2 base transform。默认双手使用 q20 rest；
`--hand-pose grasp` 使用 qualification profile 的同一 representative grasp 规则分别生成
左右 q20。兼容参数 `--right-hand-pose rest|grasp` 也轴对称地作用于双手，不再单独旋转
右 Hand 2 base。

两种姿态都保留 tabletop q7、左右两棵 q27 articulation，并加载左右各 14 个 mount
compound shapes、一个 D405 box 与一个 synthetic 140° Camera prim。当前入口不加载
self-collision filtered-pair，也不打开 articulation self-collision。

## 生命周期 Gate

入口按以下顺序执行：

```text
settle final physics frame -> snapshot -> world.pause()
-> snapshot -> render-only screenshots/view selection -> snapshot
-> GUI READY -> world.stop() only in finally after GUI exit
```

rest 与 grasp 两次报告均确认：timeline 为 paused 而非 stopped；三个快照中的双 q27、左右
hand-base、wrist-rig root 与 Camera world transform 完全一致。最终 q27 target error 均低于
版本化 `0.25 rad` 阈值：rest 左右为 `0.188383/0.188423 rad`，grasp 左右为
`0.181191/0.180055 rad`。

- rest：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s5_inspector_rest_final/report.json`
- grasp：
  `/home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s5_inspector_grasp_final_accepted/report.json`

## 截图观察

rest 运行保存双臂全景、左右装配外侧近景、左右 exploded flange 接口、左右 140° optical
frame 和左右 collision debug，共 9 张。exploded 接口图清楚显示每侧两个圆孔和两个 45°
胶囊 key；它只是 transient stage 中的临时视觉副本，保存后立即隐藏，不改变正式装配。
Isaac 6.0.1 会在运行期删除任意 prim 时使 deprecated Articulation physics view 失效，因此
副本随 Kit stage 一并销毁。collision debug 将 mount compound 标为橙色、D405 box 标为
半透明绿色，可辨认桁架空隙没有被单一 hull 填满。

grasp 运行只补左右两张 140° optical frame，避免重复 rest 已覆盖的结构截图。左右画面保持
镜像，拇指/食指根部与少量手背可见，主要 workspace 仍未被遮挡；不对图像做人为旋转。

GUI 启动命令：

```bash
cd /home/lenovo/swy/wujihand_mount
/home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_realsense_d405_mount_inspection.py \
  --gui \
  --right-hand-pose rest \
  --output-dir /home/lenovo/swy/wujihand_d405_wrist_rig_scratch/isaac_gui_d405_v2
```

出现 `WRIST MOUNT GUI READY ... timeline=paused` 后，可在 Camera 菜单切换 Perspective、
left optical 与 right optical；只切换 viewport camera 不改变 stage。已知 Hand 2 OmniPBR
`texture_wrap_u/v` warning 不影响外观、相机或生命周期 Gate。
