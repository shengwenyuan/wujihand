# 本仓库全局架构路由

## 目录

- [目标](#目标)
- [依赖方向](#依赖方向)
- [信号契约](#信号契约)
- [代码落位](#代码落位)
- [配置与上游迁移](#配置与上游迁移)
- [数据集最低完备性](#数据集最低完备性)
- [验证落位](#验证落位)
- [文档规则](#文档规则)
- [最近一次交付快照](#最近一次交付快照)
- [增改与交付检查](#增改与交付检查)

## 目标

本仓库的首个垂直切片是：

```text
Google MediaPipe
  -> canonical 21×3 hand observation
  -> retargeting
  -> 20-joint intent
  -> supervision
  -> Isaac execution
  -> multi-stream trajectory dataset
```

Glove、外骨骼、MuJoCo、Wuji 真机、ROS2 和 PI server-client 都是未来 adapter，不是当前已实现能力。正式基线见仓库根目录 `docs/000-project-charter-and-architecture.md`。

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

- `plans/current.md` 描述下一步计划；`plans/last_edits.md` 描述上一次合格交付。两者都是本地动态快照，Git 忽略，不是长期事实源。
- `docs/` 是正式、版本化文档。
- 已实现功能在 `docs/components/` 标出真实代码、配置和测试入口。
- 稳定架构写 `docs/architecture/`，操作流程写 `docs/guides/`，验收证据写 `docs/validation/`，重大取舍写 `docs/decisions/`。

## 最近一次交付快照

在实现、相关正式文档和主要验证完成后，最终交付回复之前，若满足以下任一条件，整体覆盖 `plans/last_edits.md`：

- 完成长程计划的 Gate、里程碑或可独立验收的阶段性交付。
- 新增、移除或显著改变可运行的大功能、CLI、adapter、backend 或端到端链路。
- 改变架构边界、依赖方向、port/canonical contract、schema、配置事实源、数据格式或部署拓扑。
- 改变安全、故障、停止、恢复、兼容性或主要依赖/运行时版本基线。
- 大型重构跨模块改变职责、入口或预期行为，即使外部 API 基本不变。

若没有上述语义触发，但下列范围信号中至少两项成立，也按大幅交付处理：跨越至少 3 个逻辑区域；修改至少 6 个非生成、非 lock 的主要文件；新增或改变运行/配置/验证/正式文档入口；工作跨多个会话或 agents 且需要正式交接。文件数只用于兜底，不能豁免只涉及单文件的安全、契约或架构变化。

纯查询或只读诊断、计划讨论、格式/拼写/链接修正、不改变契约与入口的局部小修、机械性生成或 lock 更新，以及没有形成可用切片的阻塞工作，不得覆盖最近一次有效交付。若交付的是有价值的阶段性切片，将状态写为 `阶段性交付`，并明确尚未完成的边界。

每次触发都重写整个文件，不追加历史、不建立日期副本。正文以 3–5 分钟读完为准，通常控制在 800–1500 个中文字符、4–10 个主要入口和 1–6 项验证；超出的原理与证据迁入正式 `docs/`。使用以下结构：

```markdown
# 上一次大幅交付

| 字段 | 值 |
|---|---|
| 更新时间 | YYYY-MM-DD HH:mm，Asia/Shanghai |
| 交付主题 | ... |
| 状态 | 完成 / 阶段性交付 |

> 本文件是本地临时交接页，会被下一次合格交付整体覆盖，不是长期事实源。

## 交付结论

...

## 主要变化

- ...

## 主要入口

| 路径 | 用途 | 本次变化 |
|---|---|---|
| `...` | ... | ... |

## 预期效果

- ...

## 验证

| 检查 | 结果 |
|---|---|
| `...` | 通过 / 未运行 / 待人工 |

## 限制与下一步

- ...
```

只写实际执行过的验证；区分自动检查、人工验收和未运行项。不要放完整 diff、原始长日志、逐文件流水账、密钥/设备凭据或不可恢复的唯一信息。入口路径必须真实存在；删除的入口须明确标为已删除。稳定行为、长期架构、完整证据和重大取舍仍须进入对应 `docs/` 或 ADR。

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
3. 若达到大幅交付触发条件，最后覆盖 `plans/last_edits.md` 并确认它未被 Git 跟踪。
