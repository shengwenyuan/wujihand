# 2026-07-31 RoboLab banana-in-bowl ROS Deployment 验证

| 字段 | 值 |
|---|---|
| 状态 | `PASS` |
| 目标机 | Workstation2，Isaac Sim 6.0.1 / ROS 2 Jazzy |
| 场景 | RoboLab `banana_bowl.usda` |
| 具身体 | 双 NERO + 双 Wuji Hand 2 |
| 范围 | 仿真遥操作；不连接或控制真实机器人 |

## 修正与布局判断

`validate_isaac_workcell.py` 是 environment-only 验证器，不注入机器人；原命令又未带
`--gui`，因此看不到机械臂是预期行为，不是加载失败。

三个 RoboLab profile 已统一改用
`assets/backgrounds/indoors/photo_studio_01_2k.hdr`，DomeLight 参数为
`intensity=800`、`exposure=0`。

原可见背景不是景深或镜头畸变：Isaac 相机 `fStop=0`，景深关闭，水平视场约
`60°`；模糊和尺度错觉来自 2048 × 1024 HDR 全景在无限远 Dome 上的显示。HDR 现只
用于照明（`visibleInPrimaryRay=false`），主视线使用固定中性灰背景
`[0.12, 0.12, 0.12]`，避免虚假房间尺度和无视差背景。

官方 61 个整理场景中，40 个直接使用同一 `table_oak.usd`，51 个引用
`franka_table.usd`。banana-in-bowl 的桌面顶面约为 `0.70 m × 1.00 m`，足以承载多数
桌面类初始布局；其余场景还包括货架、箱筐和 workdesk 等自有 fixture，不能由一张放大
桌面统一替代。

RoboLab 在 world 原点预留机器人锚点，主操作桌位于其 `+X` 方向。Workcell 将整个
RoboLab scene 绕 Z 轴旋转 `+90°`，并把该锚点对齐到旧“一张桌子”布局的双臂中心
`(0, -0.52, 0.80)`。这样：

- 双 NERO 保持原 mount：`x=±0.32`、`y=-0.52`、间距 `0.64 m`、朝向 `+Y`；
- 两个 NERO 底座均完整落在上游 `franka_table` 顶面；
- 主桌及 banana/bowl 转到双臂正前方；
- 删除临时 fixed plinth，不缩放、不复制、不修改上游 fixture。

这是纯 Workcell frame 变换；materializer 和机器人运行时没有场景专用分支。未来确有
超出台面的单个场景时，再新增该 Workcell 专属且不重叠的 extension。

## ROS Deployment

新增：

```text
configs/sessions/isaac_nero_dual_hand2_robolab_banana_bowl_teleop_v1.yaml
configs/deployments/isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2.yaml
```

Deployment 复用 `workstation2_nv5_ros_v2` 本地绑定、双 Tracker、双 Glove、现有 q7/q20
映射和 ROS QoS，只把 composition root 切到 banana-in-bowl rich Workcell。它保留静态
初始布局和物体原有物理属性，不引入 Task 层。

启动：

```bash
cd /home/lenovo/swy/wujihand_nv4
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$PWD/src:$PWD/ros2/wujihand_ros2:$PYTHONPATH"

ros2 launch wujihand_ros2 dual_teleoperation.launch.py \
  project_root:="$PWD" \
  deployment:=configs/deployments/isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2.yaml \
  local_runtime_binding:=configs/local/workstation2_nv5_ros_v2.yaml \
  gui:=true record:=false
```

## Workstation2 结果

- banana-in-bowl 双 NERO scripted qualification：`passed: true`。
- base-empty、workdesk 旋转后静态 smoke：PASS。
- 原“一张桌子”双 NERO qualification：`passed: true`；rich Workcell 的左右 mount
  world pose 与该基线逐值相同。
- rich ROS Deployment resolver：4 条控制 route，资产闭包校验通过。
- Isaac ROS consumer：120 帧完成，`passed: true`，Session 和 Deployment identity
  与新配置一致。
- 运行边界为 simulation-only；本次未启动真实机器人输出。

自动检查：

```text
本机 pytest           663 passed, 4 skipped, 11 deselected
本机 ruff             PASS
本机 mypy src         PASS
Workstation2 focused  11 passed
```

证据根：

```text
/home/lenovo/swy/wujihand_nv4/artifacts/validation/robolab-static/banana-bowl-aligned-v3/
```

| 证据 | SHA-256 |
|---|---|
| `nero.json` | `de13c58c884965c79c670e52406d993c9d73b6977bdb7895de8e2e38ccb72868` |
| `nero.png` | `f538b67ef8635fc4577830a4c43456dedbf21993751531300d071846e02d4727` |
| `nero-top.png` | `5ebfc731a5632b0786a221a26062dab180ddc02b8b6939313a95c690aa4b2b1f` |
| `ros-consumer.json` | `4acb305b99c88aafb94014ade220307aa5f5c3fb64e61784c54a100b9e06f025` |

Isaac Sim 6.0.1 仍会对少量上游 OmniPBR 参数打印非阻塞 MDL warning；未出现 unresolved
reference、非有限关节状态或场景加载失败。
