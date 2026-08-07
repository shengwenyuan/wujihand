# 本仓库全局架构路由

## 目录

- [目标](#目标)
- [外部资料路由](#外部资料路由)
- [依赖方向](#依赖方向)
- [信号契约](#信号契约)
- [代码落位](#代码落位)
- [配置与上游迁移](#配置与上游迁移)
- [数据集最低完备性](#数据集最低完备性)
- [验证落位](#验证落位)
- [文档规则](#文档规则)
- [最近一次部署快照](#最近一次部署快照)
- [增改与交付检查](#增改与交付检查)

## 目标

本仓库最初的垂直切片是：

```text
Google MediaPipe
  -> canonical 21×3 hand observation
  -> retargeting
  -> 20-joint intent
  -> supervision
  -> Isaac execution
  -> multi-stream trajectory dataset
```

当前已实现边界必须从代码、组件文档和验证共同判断：

- 已实现：真实双 Wuji Glove 与双 VIVE Tracker 输入、ROS2 transport、Isaac Sim 6.0.1 双 NERO + 双 Hand 2 仿真遥操作、监督、全因果录制、回放、GUI qualification 和数据导出。
- 已实现但独立：MuJoCo FR3 v2 + Hand 2 桌面场景。
- 尚未实现：Hand 2 真机执行或验证、NERO + Hand 2 MuJoCo 对等场景与同链遥操作采集、外骨骼等未落代码的 adapter。

这里的 HIL 是真实输入设备接入仿真，不是 Hand 2 真机闭环。Hand 2 仍是 beta 产品；当前运行时锁定 `wuji-description v2026.6.27`，不得把 v2026.7.23+ 的模型语义静默套入。正式基线见仓库根目录 `docs/000-project-charter-and-architecture.md`，当前文档入口见 `docs/index.md`。

## 外部资料路由

涉及 Wuji、NERO/Songling、Orbbec、RealSense 或 SteamVR/VIVE 的设计和代码修改前，使用仓库 skill `use-wujihand-robotics-mcps` 选择检索通道。`wuji-docs` 是 Wuji 官方在线 MCP；其他当前 MCP 是本地检索服务，结论权威性来自其返回的具体官方原文。MCP 只建立文档事实；在线设备、固件、网络、ROS 图和标定状态仍必须通过目标环境只读检查确认。不要在本架构参考中复制易变化的工具清单。

## 依赖方向

```text
runtime -> application + adapters
application -> domain + ports
adapters -> domain + ports + external SDKs
ports -> domain
domain -> no MediaPipe / Isaac / ROS2 / Wuji SDK dependency
```

外部 SDK 类型不得越过 adapter。核心算法不得 import 仿真器、设备或 middleware。

## 信号契约

按以下阶段分别建模和记录：

```text
raw input
-> canonical observation
-> joint intent
-> safety decision
-> sent command
-> backend feedback
```

Recorder 旁路记录各多速率流，不能阻塞控制环。不要把 intent 当成实际下发命令，也不要把 current、effort、torque 和 contact force 混为一项。

边界必须显式携带 schema version、时间域、单位、frame、handedness、joint layout、配置/标定引用和 source provenance。相同长度的 20 维数组不代表 Hand v1、Hand 2、Description、SDK 和 Isaac 的关节顺序相同。

## 代码落位

| 内容 | 路径 |
|---|---|
| canonical types、单位、坐标、joint layout | `src/wujihand/domain/` |
| 外部无关的协议 | `src/wujihand/ports/` |
| 标定、retargeting、supervision、recording、teleop 用例 | `src/wujihand/application/` |
| MediaPipe/Glove/外骨骼 | `src/wujihand/adapters/input/` |
| `wuji_sdk.retargeting` 等外部算法包装 | `src/wujihand/adapters/retargeting/` |
| Isaac/MuJoCo | `src/wujihand/adapters/simulation/` |
| Wuji SDK 真机 | `src/wujihand/adapters/robot/` |
| ROS2/PI server-client | `src/wujihand/adapters/transport/` |
| MCAP/Parquet/文件存储 | `src/wujihand/adapters/storage/` |
| CLI、config loading、依赖装配 | `src/wujihand/runtime/` |
| 可丢弃原型 | `experiments/`；生产代码不得 import |

## 配置与上游迁移

- `configs/upstream/`：带 URL/tag/commit/license 的官方原样配置。
- `configs/base/`、`configs/profiles/`：项目维护配置；派生项写 `derived_from`。
- `configs/sessions/`：只组合 profile 和运行 override。
- `configs/local/`：IP、SN、个人路径；Git 忽略。
- `third_party/sources.lock.yaml`：锁定官方包、仓库、模型、submodule 和 checksum。
- 能通过 adapter 包装时不复制官方算法；必须 fork/vendor 时补 license、ADR 和行为对照测试。

滚动 `latest` URL 用于查阅，不用于复现实验。模型/算法/SDK 都记录固定 tag 或 commit。

## 数据集最低完备性

`teleop_trajectory_v1` 至少包括 raw/canonical landmarks、joint intent、supervision decision、sent command、feedback、时序/掉帧/错误事件，以及带配置、模型、标定和版本来源的 manifest。

先保留原始多速率流，离线对齐时记录插值方法、容差和 missing mask。MCAP 与 Parquet 等容器中只允许一个被 ADR 指定为规范事实源。

## 验证落位

```text
tests/unit/          纯转换、映射、不变量、监督规则
tests/contract/      每个 adapter 的 port 契约
tests/integration/   MediaPipe / Isaac / storage 边界
tests/e2e/           完整 slice、失联和停止路径
tests/golden/        小型确定性预期输出
tests/fixtures/      可提交输入与配置
```

默认快速测试不依赖 GPU、摄像头或真机。每次新增 adapter 同时加入 contract test；每次修改 joint layout、单位、坐标或 schema 同时加入兼容性/golden test。

## 文档规则

- `plans/` 是 Git 忽略的本地动态区，不是长期事实源；`plans/last_edits.md` 仅在长任务完成部署或部署 checkpoint 后作为可选交接页更新。
- `docs/` 是正式、版本化文档。
- 已实现功能在 `docs/components/` 标出真实代码、配置和测试入口。
- 稳定架构写 `docs/architecture/`，操作流程写 `docs/guides/`，验收证据写 `docs/validation/`，重大取舍写 `docs/decisions/`。

## 最近一次部署快照

`plans/last_edits.md` 已降级为可选的本地长任务部署交接页。只有同时满足以下条件才更新：

1. 当前工作被判断为需要跨会话延续或正式交接的长任务；
2. 可用结果已经部署到目标环境，或完成了等价的部署 checkpoint。

纯查询、只读诊断、计划、docs/skills 维护、局部修改、生成文件刷新、仅本地实现、尚未部署的功能和失败/阻塞尝试都不更新。文件数量、修改区域数量或任务耗时本身不能触发。

触发时整体覆盖，不追加历史、不建立日期副本。只记录已部署结果、真实入口、实际执行的检查和剩余限制；不能替代 `docs/` 中的正式组件文档、验证或 ADR。建议结构：

```markdown
# 上一次长任务部署

| 字段 | 值 |
|---|---|
| 更新时间 | YYYY-MM-DD HH:mm，Asia/Shanghai |
| 部署主题 | ... |
| 目标环境 | ... |
| 状态 | 完成 / 部署 checkpoint |

> 本文件是本地临时部署交接页，会被下一次长任务部署整体覆盖，不是长期事实源。

## 部署结论

...

## 主要变化

- ...

## 主要入口

| 路径 | 用途 | 本次变化 |
|---|---|---|
| `...` | ... | ... |

## 已观测效果

- ...

## 验证

| 检查 | 结果 |
|---|---|
| `...` | 通过 / 未运行 / 待人工 |

## 限制与下一步

- ...
```

只写实际执行过的检查；区分自动检查、人工验收和未运行项。不要放完整 diff、原始长日志、逐文件流水账、密钥/设备凭据或不可恢复的唯一信息。

更新后确认文件存在且仍被忽略：

```bash
test -f plans/last_edits.md
git check-ignore -q plans/last_edits.md
git ls-files -- plans/last_edits.md  # 必须无输出
```

## 增改与交付检查

增改前：

1. 用本 skill 确认产品代际、官方 API、模型路径和版本边界。
2. 选择已有 port；确需新语义才新增 port。
3. 外部差异留在 adapter，canonical pipeline 不出现 SDK 类型。
4. 同步定义配置 provenance、数据记录字段和故障行为。
5. 同时安排 unit/contract/integration/golden 中的合适验证。

交付前：

1. 更新 component 文档和必要的 validation/ADR；未实现的未来目录不得写成已支持。
2. 执行与风险相称的验证，只报告真实结果和待人工项。
3. 只有长任务已完成部署或部署 checkpoint 时，才覆盖 `plans/last_edits.md` 并确认它未被 Git 跟踪。
