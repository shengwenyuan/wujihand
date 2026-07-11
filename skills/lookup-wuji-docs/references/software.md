# 软件开发工具、API 边界与选型

## 目录

- [快速选型](#快速选型)
- [Wuji Studio](#wuji-studio)
- [Wuji SDK](#wuji-sdk)
- [Wuji Hand ROS2](#wuji-hand-ros2)
- [Wuji Hand HMI 与 Upgrader](#wuji-hand-hmi-与-upgrader)
- [Wuji Hand SDK wujihandpy](#wuji-hand-sdk-wujihandpy)
- [防混淆规则](#防混淆规则)
- [查阅工作流](#查阅工作流)

## 快速选型

以下状态核对于 2026-07-11。版本标识只用于定位该快照，回答时应打开各文档集的 `release-notes/`。

| 需求 | 文档/组件 | 当前产品边界 |
|---|---|---|
| 通用发现、连接、订阅、MCAP、时间同步、C API | [Wuji SDK](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/) | Glove、Hand v1、Hand 2 |
| Hand 2 资源、MIT 参数、诊断、20 关节命令 | [Hand 2 SDK 接口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/) + Wuji SDK | Hand 2 Beta 1 |
| GUI 设备管理、可视化、录制、标定、升级 | [Wuji Studio](https://docs.wuji.tech/docs/zh/wuji-studio/latest/) | 当前指引列 Glove、Hand 2 |
| 第一代低层 USB Python/C++ | [wujihandpy](https://docs.wuji.tech/docs/zh/wujihandpy/latest/) | Hand v1 |
| ROS2 driver/topic/service/RViz | [Wuji Hand ROS2](https://docs.wuji.tech/docs/zh/wujihandros2/latest/) | 当前发布主要实证 Hand v1；Hand 2 声明需动态核验 |
| 第一代 GUI 调试/演示 | [Wuji Hand HMI](https://docs.wuji.tech/docs/zh/wuji-hand-hmi/latest/) | Hand v1 |
| 第一代 Bootloader/固件升级 | [Wuji Hand Upgrader](https://docs.wuji.tech/docs/zh/wuji-hand-upgrader/latest/) | Hand v1；Hand 2 用 Studio |

当前快照版本族：Wuji SDK/Studio 使用日历版本；ROS2、wujihandpy、HMI、Upgrader 使用 semver。不能按版本号数值跨组件比较兼容性。

## Wuji Studio

文档树详见 [navigation.md](navigation.md)。重点能力：

- 自动发现与设备管理；设备名会影响可视化 Topic 路径。
- Plot、触觉矩阵、Raw Messages、3D 手骨架/指尖/EMF/点云/TF。
- 把已订阅 Topic 录制为 MCAP，并监控丢帧率、同步偏移等质量指标。
- Hand 2 固件升级、设备标定、Glove 触觉接触标定。
- 用户档案隔离标定资产；默认用户与具名 profile 的持久化语义不同。

与 teleop 相关时同时核对：

1. Studio 当前 release 的默认用户/具名 profile 建议。
2. Glove 标定文件是否写入预期的 `~/.wuji/` 用户目录。
3. 运行 teleop 前 Studio 是否仍占用只允许单客户端的设备。
4. 旧 MCAP 的触觉布局是否与当前 Studio 兼容。

Studio 首页或发布记录出现的 extension/teleop 能力不一定有独立侧栏页。找不到页面时先查 release notes，再查官方仓库 tag，不要从 UI 名称推断 API。

## Wuji SDK

### 核心路由

常用页面：

- [快速开始](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/quick-start/)
- [设备发现与连接](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/device-connection/)
- [数据订阅](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/data-subscription/)
- [数据录制](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/data-recording/)
- [数据结构](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/data-reference/)
- [参数与标定](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/device-params/)
- [时间同步](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/time-sync/)
- [Retargeting](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting/)
- [C API](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/c-sdk-reference/)

典型流程是 `SdkManager.instance()` → `scan()` → `connect()`/`auto_connect()` → 获取资源 → 订阅或发布。连接可按 SN、地址、USB 或手性选择；`device_name` 是本地资源别名，不是设备筛选条件。

订阅有异步、同步非阻塞和 callback 三种模式。跨设备 `tf` / `tf_static` 是 manager 级全局资源。录制支持压缩、暂停/恢复、episode 切换、质量指标和摘要；设计数据采集系统时优先复用这些语义而非另造不兼容的时间戳模型。

### Hand 2 破坏性边界

2026.7.1 起 Hand 2 使用统一资源模型。当前概念形态：

```python
sub = hand.joint_states().subscribe()
pub = hand.joint_command().publish()
pub.send([JointCommand(...), ...])  # 20 项

hand.mit_params().get()
hand.effort_limit().get()
hand.enable(joints=mask)
hand.disable(joints=mask)
hand.clear_fault(joints=mask)
hand.emergency_stop()
```

使用前必须回查精确签名。高风险差异：

- 反馈可能只包含在线关节，按 `nid` 对齐，不能假设返回列表索引固定。
- `mit_params` 和 `effort_limit` 的持久化语义要以产品页为准。
- 旧三数组 `send(positions, velocities, efforts)` 与旧控制模式设置不可套用。
- 命令要求整手 20 项，产品专属资源页比通用示例更权威。

### Hand v1 在通用 SDK 中不是 Hand 2 的同名替身

当前两代的概念差异包括：

```text
Hand v1                         Hand 2
joint_state()                   joint_states()
publisher()                     publish()
20 个 position                 20 个 JointCommand
USB + realtime_controller       Ethernet + MIT 资源式控制
```

这些名称可能继续演进。跨代封装应通过本项目的 port 统一，不应在业务层用 `hasattr` 猜设备类型。

### SDK 内 Retargeting

当前文档给出的入口：

```bash
pip install "wuji-sdk[retarget]"
```

典型契约是 `RetargetSession.for_hand(...)`、`step((21, 3)) -> (20,)`、`reset()`。当前预编译扩展的平台边界与 teleop 示例选项要查该页和 SDK release notes。

2026.7.1 起 Retargeting 随 Wuji SDK 发布，独立开源 `wuji-retargeting` 的发布节奏可能滞后。选路：

- 需要当前 SDK 支持路径、简单实时真机示例：先查 SDK Retargeting。
- 需要 MuJoCo `teleop_sim.py`、多输入源、自定义模型/config、可视化调参或算法实现：查独立 Wuji Retargeting，同时记录其 tag。

不要假设两个入口的参数、默认配置或维护状态同步。

## Wuji Hand ROS2

当前文档覆盖安装、快速开始、多手配置、topic/service 和 RViz。现行接口线索包括：

```text
/{hand_name}/joint_states
/{hand_name}/joint_commands
/{hand_name}/hand_diagnostics
/{hand_name}/robot_description
TF / TF static
```

消息常用 `sensor_msgs/msg/JointState`，并有诊断消息、使能/失能和清错服务。精确包名、launch 名、默认频率和参数应从当前 tag 的文档读取。

### 已知官方不一致

官网总览曾声明 ROS2 同时支持 Hand USB 与 Hand 2 Ethernet；但 2026-07-11 时：

- 独立 ROS2 阅读指引的适用产品只列第一代 Wuji Hand。
- v1.1.0 安装仍依赖第一代 `wujihandcpp`。
- 仓库 README、示例和故障排查仍主要围绕 USB。

因此遇到“Hand 2 是否已由这个 ROS2 包支持”时，不作静态肯定。至少同时检查：

```text
ROS2 release notes + 安装页 + 仓库当前 tag/README + transport 实现
```

若四者没有对齐，明确报告文档冲突，并为 Hand 2 优先考虑通用 SDK adapter 或自行维护的 ROS2 bridge。

## Wuji Hand HMI 与 Upgrader

两者都只面向第一代 Wuji Hand。

### HMI

HMI 用于整手/单轴监控、使能、清错、回零、演示和诊断日志。旧名称包括 Qt HMI、WujiHand Qt HMI。`Effort Limit` 属于电流空间作用量，不应描述成实测接触力或实际输出力矩。

### Upgrader

Upgrader 负责第一代 USB Bootloader 固件流程。旧名称包括 OTA HMI、WujiHand OTA。升级过程涉及断电/上电、USB、Bootloader、镜像校验与分阶段烧录；不得把它用于 Hand 2。Hand 2 的升级入口是 Studio。

## Wuji Hand SDK wujihandpy

这是第一代专用 SDK，不等于通用 `wuji-sdk`。

```python
import wujihandpy

hand = wujihandpy.Hand(side="left")
hand.write_joint_enabled(True)
hand.finger(1).joint(0).write_joint_target_position(1.57)
positions = hand.read_joint_actual_position()
```

结构是 `Hand → Finger → Joint`，5×4 零基索引。同步 `read_*` / `write_*` 会阻塞，不能直接放入高频控制环；实时控制使用它自己的 `realtime_controller()` / filter 路径。文档还覆盖 async、unchecked、多线程和 USB 断连。

看到以下 import 时立即分流：

```text
import wuji_sdk       -> 通用 SDK，日历版本，多产品
import wujihandpy     -> 第一代独立 SDK，semver，USB
```

## 防混淆规则

1. “SDK”可能指通用 Wuji SDK、第一代 wujihandpy，或 Hand 2 产品页中的专属资源接口。
2. Hand 1/2 即使都暴露在 `wuji_sdk` 里，也不能假定方法名、命令 schema 和 transport 相同。
3. 两代固件可能出现相似版本号；相同字符串不代表镜像通用。
4. `effort`、电流、诊断电流、力矩和接触力不能互换表述。
5. 关节顺序随 API 核对：wujihandpy 的 5×4、ROS2 名称、Hand 2 flat index/`nid`、Description 解剖命名彼此不同。
6. Hand 2 → Studio 升级；Hand v1 → Upgrader。
7. Qt HMI 归一到 HMI；OTA HMI 归一到 Upgrader。
8. 搜索引擎缓存的旧 `/wuji-hand/latest/...` 页面不能作为产品代际证据。
9. Retargeting 内置 SDK 入口与独立仓库入口需分别记录版本。
10. ROS2 对 Hand 2 的支持必须动态核验，不能只引用总览营销句。

## 查阅工作流

软件问题至少输出以下证据维度：

```text
目标产品/代际
组件与包名
文档 URL
release/tag 或文档快照日期
transport（Ethernet / USB / ROS2 / in-process）
API/数据 schema
已知兼容性或文档冲突
```

设计代码前再把外部 API 隔离到 adapter；业务流水线只依赖本项目的 canonical ports。项目落位规则见 [project-architecture.md](project-architecture.md)。
