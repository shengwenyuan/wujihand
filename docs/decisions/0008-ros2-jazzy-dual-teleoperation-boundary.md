# ADR-0008：ROS 2 Jazzy 双侧遥操作边界

- 状态：已接受，待 Workstation2 硬件 HIL 收口
- 日期：2026-07-30
- 上位计划：[001：NERO + VIVE 双臂遥操作版本计划](../001-nero-vive-dual-arm-teleoperation-version-plan.md)
- 前置决策：[ADR-0007：NV-4 原生双侧遥操作与 Deployment 边界](0007-nv4-native-dual-teleoperation-deployment.md)

## 背景

NV-4 已通过双 Tracker、双 Glove、双 NERO/Hand 2 Isaac 控制。NV-5 要引入 ROS 2
Jazzy 的 typed interface、QoS、Lifecycle、namespace、launch 和 rosbag2，同时保持
五层事实、控制语义、Workstation2 mapping、IK、retarget、supervisor 与 q27 owner
不变。

## 决策

### 1. ROS 2 是适配与部署边界

五层仍为：

```text
Asset -> Binding -> Assembly -> Workcell -> Session
```

ROS graph、QoS、namespace 和本机进程环境属于 Deployment/local runtime binding，
不进入五层。domain、ports 和 application 不依赖 `rclpy` 或 generated message。

### 2. UDP 与 ROS 使用独立入口，共用执行核

native/UDP 继续由 `run_isaac_nero_hand2_dual_twin.py` 启动；ROS 由
`run_isaac_nero_hand2_ros.py` 启动。不增加 transport flag、UDP↔ROS relay 或统一
runner。

两个入口共用中性 Session/control profile、四 route plan、canonical controllers、
双 NERO/Hand 2 Isaac scene materialization、Lula/retarget/supervisor composition 和
每侧一次 atomic q27 apply。

### 3. 最小 ROS graph

生产 graph 固定为：

```text
vive_source LifecycleNode  ----\
                                +-> isaac_consumer -> left/right q27
glove_source LifecycleNode ----/
```

一个 VIVE source 独占 OpenVR runtime 并发布双 Tracker；一个 Glove source 管理双
Glove；Isaac consumer 是唯一 articulation command owner。ROS command topics 只用于
观测，不存在回灌 subscriber。

### 4. Deployment v2 与本机环境分离

`wujihand.deployment.v2` 记录 ROS node/process graph、namespace、QoS 引用和唯一 owner。
`wujihand.ros_local_runtime_binding.v2` 记录 `ROS_DOMAIN_ID`、RMW、各进程 Python 与
overlay。设备 identity 只保存在忽略的 local binding。

native 和 ROS Deployment 引用同一 Session、control profile 与 mapping。禁止根据
Deployment v1 猜测 ROS graph；本机绑定转换必须显式提供三个进程环境。

### 5. Typed IDL、QoS 与 epoch

控制输入使用五个版本化 custom message；Tracker quaternion 保持 `wxyz`，Hand 保持
固定 21 landmark 顺序。输入 subscriber 使用 depth=1 latest inbox，并按
`producer_instance + transport_epoch + sequence` 拒绝重复、倒序和旧 producer。

Tracker lifecycle 或消息 epoch 变化只撤销对应臂 reference；Glove epoch 变化只清除
对应手 last intent 并 reset retargeter。ROS clock 不参与安全 freshness。

### 6. 生命周期和启动

source 在 `configure` 阶段只解析配置并建立 typed publishers，在 `activate` 阶段打开
设备并创建新 activation epoch，在 `deactivate/cleanup` 阶段停止采集并释放资源。
launch 使用标准 lifecycle service 驱动，不依赖固定 sleep 或日志文本。

## 未采用方案

| 方案 | 原因 |
|---|---|
| 把 ROS 作为第六层或 simulator backend | 污染五层事实与依赖方向 |
| UDP↔ROS relay | 增加延迟、owner 与双重故障语义 |
| 把 mapping/IK/retarget 拆成 DDS 节点 | 破坏同 tick 决策与调试边界 |
| public joint-command subscriber | 产生第二 execution owner |
| `ros2_control`、MoveIt 2、真机 driver | 超出 NV-5 纯仿真范围 |
| source 启动固定 sleep | discovery 速度变化时不可靠 |

## 验证责任

- core 无 ROS import，native/UDP 全量回归；
- IDL/converter/inbox/QoS/Deployment strict tests；
- Lifecycle configure/graph/topic 测试；
- 无设备四 route fixture 回放；
- UDP/ROS canonical 等价与 fault injection；
- Workstation2 双臂 only、双臂双手及单侧 fault/restart HIL。

HIL 前不得把 NV-5 标记完成，也不得据此进入真机 command path。
