# 仿真、Retargeting 与 Teleop 路由

## 目录

- [先选正确入口](#先选正确入口)
- [Wuji Description 模型资产](#wuji-description-模型资产)
- [Wuji Retargeting](#wuji-retargeting)
- [Wuji SDK Retargeting](#wuji-sdk-retargeting)
- [MuJoCo 与 Isaac Lab 最小示例](#mujoco-与-isaac-lab-最小示例)
- [Wuji Hand Teleop ROS2 整栈](#wuji-hand-teleop-ros2-整栈)
- [输入输出与调参边界](#输入输出与调参边界)
- [高优先级易错点](#高优先级易错点)
- [本项目的落位规则](#本项目的落位规则)

## 先选正确入口

以下状态核对于 2026-07-11。仿真和 teleop 分散在多个官方入口，不能把名称相似的项目当成同一条流水线。

| 目标 | 首选 | 定位 |
|---|---|---|
| 找 URDF/MJCF/USD/STL/STEP | [Wuji Description](https://docs.wuji.tech/docs/zh/wuji-description/latest/) | 模型资产与集成说明 |
| 最小 MuJoCo 载入/轨迹回放 | [MuJoCo Sim](https://docs.wuji.tech/docs/zh/mujoco-sim/latest/) | smoke demo，不是 teleop/训练框架 |
| 最小 Isaac Lab USD 载入/轨迹回放 | [Isaac Lab Sim](https://docs.wuji.tech/docs/zh/isaaclab-sim/latest/) | GPU/Isaac 入口，但公开示例仍是最小回放 |
| Vision Pro/视频/RealSense/录制数据 → 仿真或真机 | [Wuji Retargeting](https://docs.wuji.tech/docs/zh/wuji-retargeting/latest/) | `teleop_sim.py`、`teleop_real.py`、调参、自定义模型 |
| 当前 SDK 内建 21 点 → 20 关节 | [Wuji SDK Retargeting](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting/) | 当前发布支持路径和简单真机示例 |
| 第一代双手 + Glove + 可选机械臂的 ROS2 真机整栈 | [wuji-hand-teleop](https://github.com/wuji-technology/wuji-hand-teleop) | Docker/ROS2，非仿真，默认第一代输出 |

对于用户重点的“仿真 teleop”，通常从独立 Wuji Retargeting 的 `teleop_sim.py` 开始；对于当前产品化 SDK 真机链路，再比较 Wuji SDK 内建 Retargeting。

## Wuji Description 模型资产

官方仓库：[wuji-description](https://github.com/wuji-technology/wuji-description)。查模型时记录 release/tag 和消费仓库锁定的 submodule commit。

### 产品路径

```text
wuji-description/
├── hand/                         第一代 Wuji Hand
│   ├── body/                     骨骼 URDF/MJCF/USD/mesh/RViz
│   ├── body-with-soft/           第一代软垫变体
│   └── attachment/               法兰与机械集成资产
├── hand2_beta/body/              Wuji Hand 2 Beta 1
│   ├── urdf/{left,right}.urdf
│   ├── urdf/{left,right}-ros.urdf
│   ├── mjcf/{left,right}.xml
│   ├── usd/{left,right}/wujihand.usd
│   ├── meshes/{left,right}/
│   └── step/
└── glove/body/                   Wuji Glove 21 DoF 人手骨架
```

### 选择规则

- `hand/` 是第一代；`hand2_beta/` 才是 Hand 2 Beta 1。
- `hand/body-with-soft/` 不是 Hand 2 软体模型。Hand 2 当前使用约束说明带软体仿真模型暂未提供。
- Glove 模型是 21 DoF 人手骨架；机器人手目标是 20 关节。
- 普通 URDF 常用相对 mesh 路径，`-ros.urdf` 使用 `package://`。ROS2/RViz 按消费场景选，不要只改文件名。
- `_simplified` 一般用于降低碰撞网格复杂度；视觉几何和物理参数仍须核对模型文件。
- 克隆完整仓库；不要只复制一个 body 子目录后期待 ROS 安装规则和附件路径仍有效。

### 集成入口

[集成指南](https://docs.wuji.tech/docs/zh/wuji-description/latest/integration/) 分别覆盖 MuJoCo、ROS2/RViz、Isaac Sim USD、URDF 预览和机械集成。环境要求按工具分开，不要把最新 Description 的 ROS2 版本要求套到旧 Teleop Docker 环境。

## Wuji Retargeting

官方仓库：[wuji-retargeting](https://github.com/wuji-technology/wuji-retargeting)。独立文档包含：

```text
installation/   系统、依赖、输入设备
quick-start/    teleop_sim.py / teleop_real.py
tuning/         tuning_tool 与配置调参
api/            Retargeter、输入输出、模型映射
appendix/       算法与故障排查
```

### 核心契约

```text
输入：MediaPipe 21 个 3D landmarks，shape=(21, 3)，单位=米
输出：20 个机器人关节角，shape=(20,)，单位=弧度
模型：配置中的 URDF 用于运动学，MJCF 用于 MuJoCo 可视化/执行
```

算法内部或 YAML 的距离参数可能使用与 API 输入不同的单位；`huber_delta`、pinch threshold 等必须从当前 API/调参页确认，不能由输入单位外推。

### 常见仿真入口

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wuji-retargeting.git
cd wuji-retargeting
git lfs pull
pip install -r requirements.txt
pip install -e .

cd example
python teleop_sim.py --play data/avp1.pkl --hand left
python teleop_sim.py --input visionpro --ip <VISION_PRO_IP> --hand left
python teleop_sim.py --video <VIDEO.mp4> --hand right --show-video
python teleop_sim.py --realsense --hand right --show-video
```

输入源和 CLI 会随 tag 变化。官方仓库当前可能比文档中心先加入 Wuji Glove、ZED 或其他入口；回答时同时检查 tag 的 README、CHANGELOG 和 `--help`，不要只复制滚动文档命令。

macOS 的 MuJoCo GUI 入口可能需要 `mjpython`。这是平台差异，不要把 `mjpython` 当作 Linux 固定命令。

### 真机代际

- 第一代：USB 路径，Linux 串口/USB 权限问题常见。
- Hand 2：Ethernet，使用 Hand 2 对应 config 和通用 `wuji_sdk`；多设备时显式地址/手性。

不要用第一代的 USB 权限排障指导 Hand 2 网络连接，也不要仅凭 `--hand right` 推断模型代际。Hand 2 配置应通过 `optimizer.urdf_path`、`mjcf_path` 和 `link_naming` 指向正确模型。

### 调参入口

`tuning_tool.py` 用于并排查看原始 21 点、缩放后的目标骨架和机器人 FK。推荐顺序：

1. 先确认左右手、坐标系、单位、模型路径和关节映射。
2. 校正 `segment_scaling`。
3. 再处理低通/归一化带来的抖动。
4. 再调 pinch threshold。
5. 最后调整损失权重或优化器预算。

启动时加载的 YAML 与正在编辑的 YAML 必须是同一个。Hand 2 调参时看到第一代模型，优先检查 config 路由而不是继续调数值。

## Wuji SDK Retargeting

当前 SDK 文档提供安装扩展、`RetargetSession`、输入格式和真机示例。它与独立开源仓库的关系会随发布变化。

```bash
pip install "wuji-sdk[retarget]"
```

适合：

- 使用当前 SDK 发布中受支持的 `21×3 → 20` 会话接口。
- 简化相机/MediaPipe 到真机的支持路径。
- 希望统一设备连接、时间同步和控制输出。

独立仓库仍适合：

- MuJoCo `teleop_sim.py`。
- Vision Pro/录制/视频/RealSense 等完整示例。
- 自定义 URDF/MJCF、link naming、可视化调参和算法研究。

若 SDK release notes 说明独立仓库暂停更新，以 SDK 作为“当前支持”来源，但不要因此删除独立仓库在仿真/算法研究中的用途。报告两个版本，不混合其参数。

## MuJoCo 与 Isaac Lab 最小示例

### MuJoCo Sim

[官方单页](https://docs.wuji.tech/docs/zh/mujoco-sim/latest/) 的当前流程：

```bash
git clone --recursive https://github.com/wuji-technology/mujoco-sim.git
cd mujoco-sim
pip install -r requirements.txt
python run_sim.py
```

它默认加载右手并循环回放 `data/wave.npy`。当前文档切左手需要修改脚本中的 `side`；不要臆造 `--side` CLI。它是模型/执行器 smoke test，不提供手部追踪、数据集或训练流水线。

### Isaac Lab Sim

[官方单页](https://docs.wuji.tech/docs/zh/isaaclab-sim/latest/) 的当前流程：

```bash
git clone --recurse-submodules https://github.com/wuji-technology/isaaclab-sim.git
cd isaaclab-sim
# 先按目标 tag 配置 Isaac Lab
python run_sim.py
python run_sim.py --side left
```

它默认右手并回放预录轨迹。官方把 Isaac Lab 定位为 GPU 并行/强化学习入口，但公开仓库当前仍是 USD 加载 + `wave.npy` 的最小示例，不等于完整 RL task、奖励函数或数据采集系统。

该最小仓库也不能作为 Hand 2 已适配的证据。为 Hand 2 设计时检查目标 tag 的 `.gitmodules`、`run_sim.py`、实际 USD 路径和 joint names；若仍指向第一代描述/命名，显式改用锁定 commit 的 `hand2_beta/body/usd/...` 并建立 `JointLayout` 映射。

两个示例都通过 submodule 锁定模型。可复现实验记录 submodule commit；不要无条件 `git submodule update --remote` 后仍声称复现原结果。

## Wuji Hand Teleop ROS2 整栈

[官方仓库](https://github.com/wuji-technology/wuji-hand-teleop) 是第一代真机整栈，而不是仿真入口。

```text
Wuji Glove
  -> wuji_sdk（controller 进程内）
  -> retargeting
  -> left/right wujihand_controller
  -> /{side}_hand/joint_commands
  -> 第一代 Wuji Hand

HTC Vive / PICO（可选手臂输入）
  -> ROS2/TF 或 Pose
  -> Tianji arm output
```

当前根 README 把 Docker 作为唯一受支持部署路径，环境围绕 Ubuntu 22.04/ROS2 Humble。它没有明确的 Hand 2 Ethernet output，也没有物理仿真模式。

常用入口：

```bash
docker exec -it wuji-hand-teleop bash
ros2 run wuji_teleop_monitor monitor
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py
ros2 topic hz /left_hand/joint_commands
ros2 topic hz /right_hand/joint_commands
```

手部默认输入是 Wuji Glove。HTC/PICO 是机械臂路径；相机管线用于视觉/回传时不能自动视为 hand-retarget 输入。PICO 当前使用独立 `pico_teleop.launch.py`，不要套旧文档中的 `arm_input:=pico`。

Hand 2 项目优先从独立/SDK Retargeting + Hand 2 `wuji_sdk` 输出设计，不把此仓库当作生产基线。

## 输入输出与调参边界

在任何 teleop 设计中显式记录：

```text
输入设备与驱动版本
原始坐标系、单位、手性、置信度、时间戳
标定/profile/URDF 来源
Retargeting 实现与 config hash
目标模型、关节名和顺序
命令单位、频率、限位和安全状态
实际状态/仿真状态与命令的时间对齐
记录器 schema、episode 和丢帧指标
```

MediaPipe 21 点只是 canonical observation 的一种来源。Glove、外骨骼、Vision Pro 等 adapter 应输出同一带时间戳/坐标元数据的 observation，再由 retargeting 消费；不要让算法直接依赖某个设备 SDK 对象。

## 高优先级易错点

1. `hand/`、`hand2_beta/`、`glove/` 分属不同模型和 DoF 语义。
2. `wuji-hand-teleop` 是第一代 ROS2 真机整栈，没有物理仿真和明确 Hand 2 output。
3. MuJoCo/Isaac Lab 单页仓库只做最小回放，不是 teleop 或完整 RL 系统。
4. 独立 Retargeting 文档与 GitHub README/CHANGELOG/CLI 可能不同步；报告差异和 tag。
5. 模型、Retargeting 和 Teleop 大量使用 submodule/LFS；克隆缺失会表现为找不到模型或二进制。
6. 最新 Description 的 ROS2 环境与旧 Teleop Docker 环境不同，不能统一升级依赖。
7. Glove 21 DoF 不能直接透传机器人 20 关节。
8. 左右手、坐标轴、米/厘米/弧度和关节顺序必须在边界处验证。
9. 真机和仿真配置不能只靠文件名推断；记录 config 内容 hash 和模型 commit。
10. `latest/` 会漂移；可复现结果同时记录 docs URL、tag、commit、submodule、SDK 和固件。

## 本项目的落位规则

本仓库采用 canonical contracts + ports/adapters：

- MediaPipe、Glove、外骨骼：`src/wujihand/adapters/input/`
- 纯 retargeting/use case：`src/wujihand/application/retargeting/`
- `wuji_sdk.retargeting` 等外部实现包装：`src/wujihand/adapters/retargeting/`
- Isaac/MuJoCo：`src/wujihand/adapters/simulation/`
- Wuji SDK/ROS2/PI：`src/wujihand/adapters/output/` 或 `transport/`
- 信号监督与安全门：`src/wujihand/application/supervision/`
- 轨迹录制与数据集：`src/wujihand/application/recording/` + storage adapter

更完整的架构与迁移规则见 [project-architecture.md](project-architecture.md)。
