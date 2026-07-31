# ROS 2 Jazzy 双侧仿真遥操作

## 边界

ROS 2 链路只控制 Isaac 中左右两棵 q27 articulation，不连接 NERO CAN 或 Hand 2
真机。五层 Session、Tracker mapping、IK、retarget、supervisor 与 native/UDP 共用。

```text
OpenVR -> vive_source ----\
                           +-> Isaac ROS consumer -> q7 + q20 -> q27
Gloves -> glove_source ---/
```

配置入口：

- `configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml`
- `configs/deployments/isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml`
- `configs/deployments/isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2.yaml`
- `configs/profiles/ros2_jazzy_dual_teleoperation_qos_v1.yaml`
- `configs/local/workstation2_nv5_ros_v2.yaml`（忽略、不提交）

## Workstation2 环境

```bash
cd /home/lenovo/swy/wujihand_nv4
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$PWD/src:$PWD/ros2/wujihand_ros2:$PYTHONPATH"
```

构建：

```bash
colcon build --base-paths ros2 --symlink-install
colcon test --base-paths ros2
colcon test-result --verbose
```

## 启动

双臂双手：

```bash
ros2 launch wujihand_ros2 dual_teleoperation.launch.py \
  project_root:="$PWD" \
  deployment:=configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml \
  local_runtime_binding:=configs/local/workstation2_nv5_ros_v2.yaml \
  gui:=true record:=false
```

双臂 only：

```bash
ros2 launch wujihand_ros2 dual_teleoperation.launch.py \
  project_root:="$PWD" \
  deployment:=configs/deployments/isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml \
  local_runtime_binding:=configs/local/workstation2_nv5_ros_v2.yaml \
  gui:=true record:=false
```

RoboLab banana-in-bowl rich Workcell：

```bash
ros2 launch wujihand_ros2 dual_teleoperation.launch.py \
  project_root:="$PWD" \
  deployment:=configs/deployments/isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2.yaml \
  local_runtime_binding:=configs/local/workstation2_nv5_ros_v2.yaml \
  gui:=true record:=false
```

该 Deployment 只替换 Session 所选 Workcell；双 NERO + Hand 2、Tracker/Glove
source、控制映射和 supervisor 均沿用既有链路。banana 与 bowl 是可交互初始布局，
不包含 reset、任务判定或 reward。

逐进程入口由 local runtime binding 中的 Python 执行：

```text
<vive_python>  -m wujihand_ros2.nodes.vive_source ...
<glove_python> -m wujihand_ros2.nodes.glove_source ...
<isaac_python> tools/run_isaac_nero_hand2_ros.py ...
```

## 诊断

```bash
ros2 node list
ros2 lifecycle get /wujihand/v1/teleop/vive_source
ros2 lifecycle get /wujihand/v1/teleop/glove_source
ros2 topic list | sort
ros2 topic info --verbose /wujihand/v1/teleop/input/tracker/right/sample
```

单侧 Tracker lifecycle/epoch 改变只撤销该侧 reference；单侧 Glove epoch 变化只重置
该侧 retarget state。source 断流不应退出 Isaac GUI。

Glove producer epoch 改变时，application controller 在下一 control tick 原子执行一次
side-local hold，再接受新 epoch 的 observation；同一 tick 内不得同时调用
`hold()` 与 `step()`。Isaac consumer 的未处理异常会保留堆栈并以非零状态退出，不应
再把异常误判为 GUI 正常关闭。

HIL 记录见
[2026-07-31 NV-5 ROS 2 Jazzy Workstation2 HIL](../validation/2026-07-31-nv5-ros2-jazzy-hil.md)。
RoboLab rich Workcell 验证见
[2026-07-31 RoboLab banana-in-bowl ROS Deployment](../validation/2026-07-31-robolab-banana-bowl-ros.md)。
