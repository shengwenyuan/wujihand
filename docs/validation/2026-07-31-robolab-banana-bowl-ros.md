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
`intensity=800`、`exposure=0`。Workstation2 GUI 截图确认 workdesk 和
banana-in-bowl 的桌面、物体、双 NERO 与双 Hand 2 均清晰可见。

官方 61 个整理场景中，40 个直接使用同一 `table_oak.usd`，51 个引用
`franka_table.usd`。banana-in-bowl 的桌面顶面约为 `0.70 m × 1.00 m`，足以承载多数
桌面类初始布局；其余场景还包括货架、箱筐和 workdesk 等自有 fixture，不能由一张放大
桌面统一替代。

因此本次不改变上游桌面，也不增加重叠碰撞体。左右机械臂 mount 保持既有位置和
`0.64 m` 间距，继续落在独立 fixed plinth 上。未来确有超出台面的单个场景时，应新增
该 Workcell 专属、与原桌不重叠的 extension，而不是全局放大所有场景。

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
- rich ROS Deployment resolver：4 条控制 route，资产闭包校验通过。
- Isaac ROS consumer：120 帧完成，`passed: true`，Session 和 Deployment identity
  与新配置一致。
- 运行边界为 simulation-only；本次未启动真实机器人输出。

自动检查：

```text
本机 pytest           663 passed, 4 skipped, 11 deselected
本机 ruff             PASS
本机 mypy src         PASS
Workstation2 focused  9 passed
```

证据根：

```text
/home/lenovo/swy/wujihand_nv4/artifacts/validation/robolab-static/banana-bowl-ros/
```

| 证据 | SHA-256 |
|---|---|
| `nero-lighting-v2.json` | `6d1aed27c1aa6cddf05b3cc619cc4fe5e107c9a7a95f1d1a8319b9671276af79` |
| `nero-lighting-v2.png` | `7c0d186c244be0495f4bc4b9388c555b20ae4f9caff913a9d687260ced0609cc` |
| `ros-consumer.json` | `f2403829206de93c28679dab4e730db46a440556a0caaf482b0304ca7277b85a` |

Isaac Sim 6.0.1 仍会对少量上游 OmniPBR 参数打印非阻塞 MDL warning；未出现 unresolved
reference、非有限关节状态或场景加载失败。
