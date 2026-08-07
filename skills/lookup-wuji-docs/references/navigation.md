# 舞肌官方文档导航

## 目录

- [快照与用法](#快照与用法)
- [硬件设备](#硬件设备)
- [开发工具](#开发工具)
- [算法与仿真](#算法与仿真)
- [按任务选入口](#按任务选入口)
- [结构化目录](#结构化目录)

## 快照与用法

本导航是本地 fallback 路由，不是当前事实源。优先使用官方在线 `wuji-docs` MCP；只有 MCP 不可用时才使用生成目录定位官方页面。`latest/` 会漂移，涉及命令、参数、兼容性、Beta 硬件或模型版本时，必须重新读取目标页和发布记录。

全局入口：

- [文档中心首页](https://docs.wuji.tech/zh/)：三大分支和快速开始。
- [全站发布记录](https://docs.wuji.tech/docs/zh/release-notes/)：按日期聚合跨硬件、软件、算法组件的变更，适合先判断兼容性边界。

先在这里选择文档集，再读取相应专题 reference。需要页内标题级定位时，不要加载完整 JSON；运行：

```bash
python scripts/search_catalog.py "查询词" --limit 10
```

相对路径以本 skill 目录为基准。

## 硬件设备

### Wuji Hand 2 (Beta 1)

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-hand/latest/)

```text
├── 版本发布记录        release-notes/
├── 文档阅读指引        /
├── 入门
│   ├── 产品介绍        overview/
│   ├── 用户须知        user-notice/
│   └── 使用约束        usage-constraints/
└── 开发指南
    └── SDK 接口         sdk-reference/
```

官方明确将当前手册限定为 Beta 1 阶段样机，参数和表现不代表最终产品。发布记录还可能包含 Beta 2-only 硬件与固件；任何真实设备结论都要先确定硬件阶段。模型工作还必须检查 `wuji-description v2026.7.23` 的破坏性 Hand 2 revision 边界。

### Wuji Hand（第一代 / v1）

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-hand/v1/)

```text
├── 版本发布记录        release-notes/
├── 文档阅读指引        /
└── 产品资料
    ├── 产品介绍        overview/
    └── 用户须知        user-notice/
```

### Wuji Glove

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-glove/latest/)

```text
├── 版本发布记录        release-notes/
├── 文档阅读指引        /
├── 产品资料
│   ├── 产品介绍        introduction/
│   └── 使用前准备      getting-started/
├── 参考材料
│   ├── SDK 数据参考    位于产品介绍/指引链接的页内章节
│   ├── 原始数据转换    offline-pipeline/
│   └── 坐标系说明      coordinate-frames/
└── 附录
    └── 故障排查        troubleshooting/
```

硬件代际与约束详见 [hardware-versions.md](hardware-versions.md)。

## 开发工具

### Wuji Studio

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-studio/latest/)

```text
├── 版本发布记录
├── 文档阅读指引
├── 入门：概述、使用前准备
├── 使用指南：设备管理、固件升级、设备标定、触觉接触标定
└── 参考：故障排查
```

### Wuji SDK

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：产品介绍、快速开始
├── 开发指南
│   ├── 设备发现与连接
│   ├── 数据订阅
│   ├── 数据录制
│   ├── 数据结构参考
│   ├── 设备参数与标定
│   ├── 时间同步与时间戳
│   └── 手部重定向（Retargeting）
└── 参考：最佳实践、故障排查、C 接口参考
```

### Wuji Hand ROS2

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wujihandros2/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：安装指南、快速开始
├── 配置：启动与配置、ROS2 接口
└── 参考：附录
```

### Wuji Hand HMI

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-hand-hmi/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：启动 HMI
├── 使用：面板介绍、连接与使能
└── 参考：故障排除
```

### Wuji Hand Upgrader

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-hand-upgrader/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：软件安装、连接设备
├── 操作：固件升级
└── 参考：故障排除、卸载软件
```

### Wuji Hand SDK（wujihandpy）

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wujihandpy/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：入门、教程
└── 参考：API 参考
```

### Wuji CLI

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-cli/latest/)

用于查询当前官方 CLI 的安装、命令和版本边界；命令行为仍以目标发布为准。

### Wuji Docs MCP

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/mcp/latest/)

这是官方 `wuji-docs` MCP 自身的文档入口，用于核对工具能力与接入方式，不代替具体产品页面。

软件产品边界、API 选型与常用入口详见 [software.md](software.md)。

## 算法与仿真

### Wuji Description

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-description/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：概述
├── 使用：集成指南
└── 参考：相关仓库、故障排查
```

### Wuji Retargeting

基准：[文档阅读指引](https://docs.wuji.tech/docs/zh/wuji-retargeting/latest/)

```text
├── 版本发布记录、文档阅读指引
├── 入门：安装与配置、快速开始
├── 进阶：调参指南、API 介绍
└── 参考：附录
```

### 单页与外部入口

```text
├── MuJoCo Sim      https://docs.wuji.tech/docs/zh/mujoco-sim/latest/
├── Isaac Lab Sim   https://docs.wuji.tech/docs/zh/isaaclab-sim/latest/
└── Wuji Hand Teleop（官方 GitHub）
    https://github.com/wuji-technology/wuji-hand-teleop
```

仿真、模型、retargeting 与 teleop 的选路详见 [simulation-teleop.md](simulation-teleop.md)。

## 按任务选入口

| 问题 | 首选入口 | 随后核对 |
|---|---|---|
| Hand 2 Python 控制、反馈、MIT 参数 | Hand 2 `sdk-reference/` | Wuji SDK 数据订阅/数据结构/发布记录 |
| 跨设备发现、订阅、录制、时间同步 | Wuji SDK | 对应硬件产品的数据参考 |
| 第一代低层 USB/1 kHz 控制 | wujihandpy | 第一代发布记录、ROS2 文档 |
| ROS2 topic/service/RViz | Wuji Hand ROS2 | 明确其产品代际；模型再看 Description |
| URDF/MJCF/USD/STEP | Wuji Description | release/tag 与模型路径 |
| MuJoCo/Isaac 最小加载 smoke test | MuJoCo Sim / Isaac Lab Sim | Description 的模型资产与 submodule commit |
| Vision Pro/视频/RealSense 到仿真或真机 | Wuji Retargeting | Quick Start、API、调参、GitHub tag |
| Wuji Glove 到第一代双手/机械臂 ROS2 整栈 | Wuji Hand Teleop GitHub | Docker、ROS2、Retargeting、Studio 标定 |
| Glove 原始数据、EMF、IMU、坐标系 | Wuji Glove | Wuji SDK 与 Studio |
| 固件升级、设备 GUI、标定 | Studio（Hand 2/Glove）或 HMI/Upgrader（第一代） | 产品代际和发布记录 |

## 结构化目录

[official-catalog.json](official-catalog.json) 是机器生成的 fallback 目录；具体文档集、页面数量和生成时间以文件内 `docsets`、`pages` 与 `generated_at` 为准，不在本页复制易过期计数。站内页面含 h1–h4 标题和锚点。用 [search_catalog.py](../scripts/search_catalog.py) 查询，不要默认整份读入上下文。

用 [refresh_catalog.py](../scripts/refresh_catalog.py) 从官网重建目录。脚本只有在所有文档集和页面都成功解析时才原子替换旧文件；失败时保留旧索引。
