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
- [增改代码前检查](#增改代码前检查)

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

- `plans/` 是本地动态 TODO，Git 忽略；不要从 skill 读取它作为长期事实。
- `docs/` 是正式、版本化文档。
- 已实现功能在 `docs/components/` 标出真实代码、配置和测试入口。
- 稳定架构写 `docs/architecture/`，操作流程写 `docs/guides/`，验收证据写 `docs/validation/`，重大取舍写 `docs/decisions/`。

## 增改代码前检查

1. 用本 skill 确认产品代际、官方 API、模型路径和版本边界。
2. 选择已有 port；确需新语义才新增 port。
3. 外部差异留在 adapter，canonical pipeline 不出现 SDK 类型。
4. 同步定义配置 provenance、数据记录字段和故障行为。
5. 同时安排 unit/contract/integration/golden 中的合适验证。
6. 更新 component 文档；未实现的未来目录不得写成已支持。
