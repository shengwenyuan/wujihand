# ADR-0003：以五层配置构建仿真资产与运行 Session

状态：Accepted

日期：2026-07-23

## 背景

现有 MuJoCo FR3 v2 + Hand 2 桌面场景、Isaac fixed Hand 2 遥操作和
Isaac rotation-ball 场景已经分别固化了资产路径、装配关系、世界几何、控制布局和
运行参数。这些事实目前分散在 typed profile、adapter 与 runner 中。继续以单个场景
配置或 runner 常量扩展，会产生以下耦合：

- “Hand 2 right”这一稳定产品身份与某个 MJCF/USD 路径、prim/body 名和上游版本
  绑定；
- 机械臂—手的装配拓扑与桌面、球体、相机等工作空间几何混在一起；
- 单根机器人树被误当作全局约束，难以表达左右臂并列、多手或其他多 root workcell；
- 相同长度的控制数组可能在未核对产品代际、侧别和 layout 的情况下被错误复用；
- runner 同时承担配置解析、跨文件引用校验和 simulator 初始化，无法在无 GPU、
  摄像头或 simulator 的环境中验证组合正确性。

本轮需要先调整架构并保持已有入口的外部行为，再为后续新增资产和单双臂组合保留
扩展面。新增 UR5、双臂 Franka、左手资产、ROS1/ROS2 运行时和具体 Tracker/Glove
映射均不属于本 ADR 的实现范围。

## 决策

### 1. 采用五层单向组合

运行配置按以下顺序构建：

```text
Asset Manifest
  -> Backend Binding
  -> Assembly Spec
  -> Workcell
  -> Session
```

各层职责如下：

| 层 | 拥有的事实 | 明确不拥有 |
|---|---|---|
| Asset Manifest | 产品身份、代际、侧别、canonical frame、control group、layout 和来源身份 | MJCF/USD 路径、prim/body 名、世界位姿和仿真参数 |
| Backend Binding | 一个 Asset 在指定 backend 的 loader、锁定 artifact、frame map、joint/actuator map 和兼容 profile | 实例间 attachment、桌面几何、输入和监督策略 |
| Assembly Spec | 资产实例、namespace、语义 frame 间 attachment、局部 transform 和 root 集合 | backend 原生名称、world geometry、physics 和 transport |
| Workcell | world/语义 frame、mount，以及封闭集合中的物理 primitive entity | 机器人实例、typed camera/light 分类、控制频率、输入源和 supervisor |
| Session | backend、assembly/workcell 引用、逐实例 binding、逐 root placement，以及运行、传输和控制组合 | 下层事实的复制品 |

每个文件携带显式 schema 版本、稳定 ID 和项目相对引用。加载器拒绝未知字段、绝对
路径、`..` 越界和引用 ID 不匹配；跨层 resolver 在启动 backend 前完成引用闭合、
layout 路由、source lock、placement 和稳定配置指纹校验。

Workcell v1 的 entity 词汇只包含 `plane | box | sphere | frustum`、
`fixed | dynamic` 和可选质量。相机可暂以语义 frame 表达；灯光和尚未提升的完整场景
事实继续留在专用 runner 或 typed compatibility leaf，不能把它们描述成已实现的通用
entity compiler。

### 2. Asset 身份与 backend 表示分离

Asset Manifest 表达跨 backend 稳定的产品语义；MJCF、USD 或 procedural builder
属于 Backend Binding。引用外部 artifact 的 Binding 必须使用
`third_party/sources.lock.yaml` 中固定的来源和 artifact，不跟随滚动文档路径静默
升级；项目自有 procedural Binding 可令 artifact 为空，但 builder 必须来自封闭
registry，身份依据由对应 Asset provenance 记录。

因此，同一 Hand 2 right manifest 可同时拥有 MuJoCo 与 Isaac binding，而 backend
资产升级只产生新的 binding revision，不改变 Asset 身份。产品代际、左右侧和
control layout 始终显式表达，不能用 “20 DoF” 或同长度数组推断兼容。

### 3. Assembly 使用 forest，而不是强制单 root tree

Assembly 由带稳定 `instance_id` 与 namespace 的资产实例组成。每个非 root 实例
恰好有一个 parent，attachment 图必须无环；一个 assembly 可有一个或多个 root。
每个 root 在 Session 中独立放置到 Workcell mount。

这一约束可表达现有单臂和 fixed-hand 场景，也可在不改变 schema 的情况下表达将来
的左右臂、多个独立机器人或手—臂组合。它不预先实现任何新的双臂资产或协同控制。
attachment 只引用 Asset 声明的语义 frame；`fr3v2_link8`、`r_base_link`、USD
prim path 等后端原生名称留在 binding。

### 4. Session 是唯一 composition root

runner 只从一个 Session 开始构建运行依赖。Session 必须为当前 backend 上的每个
资产实例选择且只选择一个 binding，并为每个 assembly root 提供且只提供一个
workcell placement。被执行的 control group 必须按明确的 `layout_id` 路由一次。

MediaPipe producer 与 Isaac consumer 仍可保持两个进程和现有 UDP 边界；它们分别由
Session 定义，并通过显式 transport contract 与 control layout 校验配对。五层配置
不是多进程 launch graph，也不要求两个 simulator 共用一个万能执行器。

### 5. 以 compatibility leaf 进行 strangler 迁移

现有 MuJoCo table、Isaac fixed-hand 和 rotation-ball typed profile/adapter 暂时作为
五层组合末端的 compatibility leaf。新层通过引用这些 profile 复用已验证的几何、
physics、D6、控制和报告事实，不在新旧配置中复制同一组数值。

迁移期间：

- 新的 `--session` 是首选组合入口，旧 CLI 参数与 API 作为显式 override/兼容入口
  保留；
- override 必须进入最终配置快照和指纹；
- simulator 专属 profile 可逐项提升为通用 schema，但每次提升都需保持单一事实源
  并补齐行为对照测试；
- compatibility leaf 可单独回退，不要求一次性改写 simulator scene compiler。

### 6. 固定依赖方向

```text
specs / compat / integrity -> Python 标准库
runtime -> specs + compat + integrity + application + adapters
adapters -> specs + compat + integrity + domain + ports + external SDK
application -> domain + ports
ports -> domain
domain -> no simulator/device/middleware dependency
```

`specs` 只定义不可变数据与局部不变量，不读取 YAML 或 import simulator/SDK。
`compat/` 是迁移期共享的标准库纯数据合同，`integrity.py` 是 simulator-neutral
内容 hash 边界；两者不得 import runtime、adapter 或外部 SDK。runtime 负责安全路径、
strict YAML repository、跨层 resolver 和依赖装配。adapter 消费解析后的 spec/compat
合同，不反向 import runtime；外部 SDK 类型不得越过 adapter。

## 验证责任

五层不能只靠一个端到端 smoke 验收。每层必须有独立 unit test，并按边界增加
contract、integration 或 golden：

| 层 | 最低验证 |
|---|---|
| Asset | schema/ID/side/frame/control group/layout 不变量与 manifest golden |
| Binding | loader/map/path/source lock、asset/frame/group 合同与 artifact 结构 |
| Assembly | 多实例、forest、cycle、多 parent、frame 与 transform |
| Workcell | frame DAG、mount、primitive、pose/尺寸/质量与 compatibility profile |
| Session | 引用闭合、backend/binding 匹配、逐 root placement、layout/transport 配对和稳定 hash |

另以架构测试阻止 `specs -> runtime/adapters/external SDK` 和
`adapters -> runtime`。无硬件 fast suite 必须能解析所有正式 Session；MuJoCo、
Isaac 和真实摄像头验证按环境能力分别执行并如实记录，环境缺失不等同于通过。

## 未采用方案

| 方案 | 原因 |
|---|---|
| 为每个场景继续维护单体 YAML | 资产、后端、装配、世界和运行事实会继续混杂，难以复用和独立测试 |
| 把 MJCF/USD 路径写进 Asset | 稳定产品身份会随 backend 与上游版本漂移 |
| Assembly 强制只有一个 root | 无法自然表达左右臂或同 workcell 的多个独立机器人 |
| 立即建立跨 Isaac/MuJoCo 的通用 scene compiler | 扩大迁移面，且会重复已有 typed profile 的已验证事实 |
| 以动态插件发现替代显式 binding | 降低可复现性，使 backend、artifact 和 layout 选择变得隐式 |
| 本轮同时引入 ROS 和新资产 | 无法隔离架构重构与设备/中间件行为变化的回归来源 |

## 后果

- 优点：资产、后端表示、装配和工作空间可独立演进；同一 schema 可表达单/双、多
  root 组合；跨层错误可在 simulator 初始化前失败；Session 快照可用于复现和数据
  provenance。
- 代价：配置文件数量增加；resolver 必须维护严格的引用和错误诊断；兼容迁移期会
  同时存在五层配置和 typed compatibility profile。
- 风险：若 compatibility profile 与五层重复保存几何或 control layout，会重新形成
  双事实源；若为了“通用”而隐藏产品代际、侧别或 layout，则会失去安全边界。
- 限制：resolver 已对 multi-root、`prefix` namespace 和所有 backend symbol 做资格化
  与碰撞检查，但现有专用 simulator compiler 仍只接受已验证 topology；多个同构
  teleop 实例还需要未来 transport schema 提供显式 stream/channel mapping。

## 与既有 ADR 的关系

- [ADR-0001](0001-hand2-fixed-flange-d6-rotation.md) 的 D6 三轴腕部、23 DoF 与
  q20+pose v2 决策保持不变；五层只把这些事实放入对应 binding/assembly/workcell/
  Session 边界。
- [ADR-0002](0002-mujoco-fr3v2-hand2-composition.md) 的 pinned 资产、identity
  attachment 假设、27-DoF control groups 和 MuJoCo 全局物理事实保持不变；`27`
  是该 Session 的解析结果，不是通用 adapter 不变量。

## 重验触发器

五层 schema、引用语义、Asset identity、binding source lock、assembly attachment、
workcell mount、control layout、Session hash 规范或 compatibility leaf 提升规则变化
时，必须重跑对应层 unit/contract、全部正式 Session 的无 backend 解析，以及受影响
simulator 的既有回归。
