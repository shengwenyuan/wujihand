# 012：PI05 q54 / H25 Mini 数据集过拟合开发计划

- 状态：计划冻结，待在 `wuji_overfit` 分支实施
- 日期：2026-08-07
- 数据上游：[008](008-ros2-isaac-triview-q54-mini-dataset-development-plan.md)、
  [011](011-mini-dataset-integrity-vs-quality-gate-update-plan.md)
- 代码仓库：`gpux1-scut:/workspace/codes/pi05base-lerobot`
- 基线：LeRobot `main@f1efa588b858e657d8bf31fb370cb3f5c8c8da05`
- 开发分支：`wuji_overfit`
- 预训练权重：`/common/models/pi05_base`
- PaliGemma 资产：`/common/models/paligemma-3b-pt-224`

## 1. 目标与边界

本需求只验证一件事：现有三条真实遥操作轨迹及其视觉随机化版本，能否让
`pi05_base` 在离线监督微调后学到双臂双手 q54 动作的明显阶段趋势，例如靠近香蕉、闭合手指、
移动并放置。它是小数据过拟合验证，不是泛化实验，也不证明真实机器人任务成功率。

实施目标：

1. 将 PI05 连续 state/action 接口从 32 维准确改为 q54；
2. 将训练 action chunk 从 50 步改为 25 步，并只采样不跨断点的有效窗口；
3. 保留三路 RGB、任务文本和 q54 的 LeRobot 原生数据流；
4. 严格复用 PI05、LeRobotDataset、processor、sampler、trainer 和 checkpoint 结构；
5. 完成单 batch、短训练、保存恢复和正式过拟合训练闭环。

明确非目标：

- 不训练可泛化模型，不建立 success 指标或 held-out 成功率；
- 不增加新的人工录制，除非本计划的静态数据验收推翻现有窗口统计；
- 不下采样到 15 Hz；数据和训练均保持 30 Hz；
- 不接 ROS2、Isaac 控制或真机部署；只为后续 rollout 保留兼容边界；
- 不引入 FAST action tokenizer、PCA、q32 action codec 或动作降维；
- 不建立通用机器人适配框架、插件、registry、兼容层或容错模块。

本计划替代远端旧计划中的 `15 Hz`、`q54 -> PCA32`、`48 episodes` 和默认 `H=50` 方案。

## 2. 已冻结的核心决策

| 项目 | 当前基线 | 本需求 | 固定结论 |
|---|---:|---:|---|
| 数据频率 | 30 Hz | 30 Hz | 不下采样 |
| state 维度 | PI05 上限 32 | q54 | 直接使用 54 维，无 PCA/截断 |
| action 维度 | PI05 上限 32 | q54 | 扩展连续 action 投影到 54 维 |
| action chunk | 50 | 25 | 训练张量固定为 `[B, 25, 54]` |
| 初始执行步数 | 50 | 10 | `n_action_steps=10`，与训练 horizon 解耦 |
| flow inference steps | 10 | 10 | 保持不变 |
| text token 上限 | 200 | 256 | q54 state prompt 实测需要 |
| 相对动作 | 可选 | 关闭 | q54 保持绝对关节角 |
| 数据 split | train | 全部 train | `eval_split=0`，只验证过拟合 |
| FAST tokenizer | 已另行下载 | 不使用 | 不训练、不接入 PI05 flow head |
| 新录 episode | 可选 | 0 | 现有有效窗口足够 |

`n_action_steps=10` 只控制将来 rollout 每次执行多少预测动作，不改变训练 label 的 25 步长度。
初次仿真部署若需更频繁闭环，可仅把它改为 5，无需重新训练；本计划不为此增加代码分支。

## 3. 当前数据事实

数据 revision：

`/home/lenovo/swy/wujihand_mount/artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/revisions/banana_in_bowl-20260806-12ep-v1`

基础 metadata：

| 项目 | 数值 |
|---|---:|
| LeRobot schema | v3.0 |
| episode 数 | 12 |
| 独立控制轨迹 | 3 |
| 每条轨迹视觉版本 | nominal + 3 个域随机化版本 |
| 总帧数 | 14,552 |
| 总时长 | 485.07 s |
| FPS | 30 |
| state/action | float32 q54 / float32 q54 |
| RGB | scene、left wrist、right wrist |
| RGB 编码 | 640×480、AV1、yuv420p、30 Hz |
| task | `place the banana in the bowl using dual arms and dexterous hands` |
| success | 未记录、未评价 |

q54 顺序是唯一合同：

```text
[0:7]   left arm q7
[7:27]  left hand q20
[27:34] right arm q7
[34:54] right hand q20
```

state 与 action 必须共享同一 54 项名称、顺序、单位和 profile hash。任何 reorder 都在数据准备阶段
直接失败，不在 model 内做适配。

12 个 episode 不是 12 条独立动作示范，而是 3 条控制轨迹各自对应 4 个视觉版本。视觉版本增加外观
变化，不增加动作多样性。这个结构适合当前过拟合目标，不可用于声称策略泛化。

## 4. P0：32 维到 q54 的精确迁移

### 4.1 FAST 与 q54 的边界

FAST 是把连续 action chunk 变为离散 action token 序列的另一条策略路线，不是把 q54 自动压成
PI05 所需的 32 维连续空间。当前 LeRobot PI05 是 flow-matching action head，因此：

- 已下载 FAST tokenizer 不进入本次 dataset、processor、model 或 loss；
- FAST tokenizer 不训练；
- PaliGemma tokenizer只处理任务文本和离散化后的 state prompt，也不参与训练；
- action 始终是归一化后的连续 q54，由 flow-matching head直接预测。

若未来切换 π0-FAST，应另立需求并重新定义 action tokenization、loss 与 checkpoint，不能混入本分支。

### 4.2 state q54

PI05 将 state 量化后拼入文本 prompt。把 `max_state_dim` 改成 54 还不够，必须同时避免 prompt 被
截断。对当前 14,552 行真实 q54 进行完整 tokenizer 实测：

| 统计 | token 数 |
|---|---:|
| min | 185 |
| p50 | 207 |
| p90 | 219 |
| p95 | 221 |
| p99 | 222 |
| max | 224 |
| `>200` | 13,400 / 14,552 |
| `>256` | 0 / 14,552 |

因此固定：

- `max_state_dim=54`；
- `tokenizer_max_length=256`；
- processor 测试必须覆盖全部 14,552 行，并确认 prompt 尾部 `Action:` 未被截断；
- 不靠扩大到任意超长值掩盖问题，也不增加动态 token 上限逻辑。

### 4.3 action q54 与 checkpoint

需要变更的网络边界只有：

```text
action_in_proj:  Linear(32, expert_width) -> Linear(54, expert_width)
action_out_proj: Linear(expert_width, 32) -> Linear(expert_width, 54)
```

迁移规则固定如下：

1. 以确定 seed 初始化 q54 模型；
2. `action_in_proj` 与 `action_out_proj` 的完整 weight/bias 全部重新初始化；
3. 不复制旧 projection 的前 32 维。q54 前 27 维恰好是完整左侧，复制会制造明显左右不对称；
4. VLM、vision encoder、action expert、time MLP 等其余 tensor 必须逐项加载预训练值；
5. 只允许上述两个 projection 模块的四个 tensor 发生 shape replacement；
6. 其他 missing、unexpected 或 shape mismatch 立即报错，不允许返回随机模型继续训练；
7. 两个新 projection 模块的全部参数均参与训练。

不实现通用 `N -> M` checkpoint adapter，也不建立新的 policy 类型。迁移逻辑留在 PI05 既有
`from_pretrained` 路径中，并由精确 key 集合约束。

## 5. P0：50 步到 25 步及有效窗口

### 5.1 H25 的含义

`chunk_size=25` 不是 batch size，也不是一次优化中的样本数。它表示每个 observation 对应的未来
action 序列长度：

```text
observation[t] -> action[t : t + 25]  # delta index 0..24
```

30 Hz 下，25 个采样点对应 0.833 s 的 action budget；首末 action 的时间戳跨度是 `24/30=0.8 s`。
模型输入输出与 loss 的核心形状固定为：

```text
state:          [B, 54]
action target:  [B, 25, 54]
predicted flow: [B, 25, 54]
loss_per_dim:   [B, 25, 54]
```

### 5.2 断点语义

沿用 011 的定义：`transition_valid[i]` 表示 row `i -> i+1` 的 action transition 真实有效。H25
anchor `t` 只有在下式成立时才能训练：

```text
all(transition_valid[t : t + 25])
```

这里必须检查 25 个 action transition，不是 24 个；最后一个 label `action[t+24]` 也必须有真实
有效性。不得插值、复制、跨 gap 或依靠 episode 尾部 padding 构造窗口。

### 5.3 训练就绪 revision

采用一次性、直接的数据转换：

1. 读取现有 12-episode revision 与 `wujihand_valid_transitions.jsonl`；
2. 将每个 episode 的连续 `transition_valid=true` 区间切成 maximal run；
3. 仅保留长度 `>=25` 的 run，并把每个 run 写成一个 LeRobot episode；
4. 每个派生 episode 保存 source episode、source row 起止、source run ID 和 visual variant；
5. 使用现有 `EpisodeAwareSampler`，配置 `drop_n_last_frames=24`；
6. 重新计算派生数据的 state/action quantile stats；
7. 保留三路 RGB 与 task，不生成 padding row。

计划输出：

`/workspace/datasets/nero-hand2/banana-in-bowl-20260806-12ep-v1-h25`

固定审计结果：

| 项目 | 三条独立轨迹 | 含四个视觉版本 |
|---|---:|---:|
| 原始帧 | 3,638 | 14,552 |
| 有效 transition | 3,525 | 14,100 |
| 无效 transition | 113 | 452 |
| 长度 `>=25` 的 maximal run | 22 | 88 |
| 派生 episode 内 action row | 2,885 | 11,540 |
| H25 有效 anchor | 2,357 | 9,428 |

因此不需要新增人工 episode。9,428 个窗口足够完成小数据过拟合；它们仍只来自三条独立动作轨迹，
这一事实必须保留在训练 run metadata 中。

## 6. 最小代码改动图

只允许以下范围：

| 文件位置 | 计划改动 |
|---|---|
| `src/lerobot/policies/pi05/configuration_pi05.py` | 支持本 run 的 q54、H25、256 token 和 `drop_n_last_frames=24` 配置；不改变上游默认 q32/H50 |
| `src/lerobot/policies/pi05/modeling_pi05.py` | 精确重初始化 action projections；其余权重严格加载；移除会静默返回随机模型的宽泛 fallback |
| `examples/port_datasets/port_wujihand_q54.py` | 一次性生成 H25 训练 revision，直接复用 LeRobotDataset writer/stats/video 路径 |
| `tests/policies/pi0_pi05/` | q54 forward/inference、projection migration、H25 tensor shape |
| `tests/processor/test_pi05_processor.py` | q54 prompt 与 256 token 不截断 |
| 现有 dataset/sampler tests | 88 segments、9,428 anchors、无 padding、无跨 gap |

明确禁止：

- 新建 `wujihand_policy`、action codec、adapter package 或 robot registry；
- 修改 ROS2/Isaac 采集主线；
- 把 sidecar 逻辑塞进通用 DataLoader；
- 复制 LeRobot 的 normalization、video decoder、dataset writer 或 trainer；
- 添加 silent fallback、catch-all exception、自动猜测维度/顺序或兼容历史 q32 的分支；
- 为单次实验创建抽象基类、factory、feature flag 或长期迁移框架。

## 7. 代码风格与工具纪律

### 7.1 代码风格

- 先沿现有 LeRobot 调用链找最小入口，再修改；命名、dataclass、typing 和 test fixture 与相邻代码一致；
- 函数短且单一职责，优先使用已有对象，不包一层只转发参数的 helper；
- q54 顺序、projection 全量重初始化原因、H25 的 25-transition 语义是仅需保留的关键注释；
- 配置值显式、数据流线性、错误直接暴露；不做防御性兜底；
- 不在生产代码中加入临时日志、monkey patch、环境探测或远端路径猜测；
- 不顺手重构无关 PI05/LeRobot 代码。

### 7.2 Skill 与信息源

每一阶段开始前按职责使用远端已有 skill：

- `.agents/skills/ground-pi05-theory`：确认 PI05 flow-matching、FAST 边界及训练 claim；
- `.agents/skills/trace-pi05-implementation`：追踪 config → dataset → processor → model → checkpoint → test 的真实调用链。

代码事实以当前 checkout 为第一优先，理论与上游差异再核对：

- [OpenPI 官方仓库](https://github.com/Physical-Intelligence/openpi)
- [OpenPI 训练配置](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/config.py)
- [LeRobot FAST 文档](https://github.com/huggingface/lerobot/blob/main/docs/source/pi0fast.mdx)

MCP 按权威范围使用，不为“用工具”而调用：

- 后续 Isaac rollout 涉及 6.0.1 API、render 或 articulation 行为时，只查 `isaac-sim-601_docs` MCP；
- 后续切换真机涉及 NERO/Hand2 限位、符号或固件事实时，使用 Wuji/NERO 硬件 MCP 与
  `verify-nero-hardware-facts` skill；
- 当前 LeRobot 训练阶段不让 Isaac 或硬件文档覆盖 live LeRobot/OpenPI 代码事实。

开发命令统一通过项目环境执行：搜索用 `rg`，Python/测试/格式化用 `uv run`。不修改远端未跟踪的
`.agents/`、`SKILL.md` 和 `plans/`。

## 8. 分阶段实施

### Phase A：冻结基线与数据合同

1. 在 `wuji_overfit` 分支记录基线 commit、模型路径和 dataset checksum；
2. 验证 12 episodes、14,552 rows、三路视频和 q54 name/order；
3. 对 sidecar 重算 88 segments 与 9,428 H25 anchors；
4. 输出只读 audit summary，数值不一致时停止，不进入模型改动。

完成标准：数据合同与本文逐项一致。

### Phase B：生成 H25 训练 revision

1. 实现单个 port script；
2. 创建 88 个派生 episode，保留完整 lineage；
3. 重新计算 q54 quantile stats；
4. 完整 reopen 三路视频和随机首/中/末帧；
5. 枚举全 sampler，确认恰好 9,428 anchors 且 `action_is_pad` 全 false。

完成标准：任意 batch 都不跨 source episode、visual variant 或 invalid transition。

### Phase C：q54 / H25 PI05

1. 配置 q54、H25、256 token、absolute action；
2. 精确迁移 checkpoint，仅 action projections 全量重初始化；
3. 校验除四个 projection tensor 外的权重逐 tensor 等于 source checkpoint；
4. 对真实数据构造 `[B,25,54]` batch；
5. 完成一次 forward/backward，确认 loss finite 且 54 个 projection row 均有 finite gradient。

完成标准：没有 padding、截断、隐式 q32 或随机 backbone。

### Phase D：训练闭环

固定起始配置：

| 参数 | 数值 |
|---|---:|
| dtype | bfloat16 |
| batch size | 2 |
| gradient checkpointing | true |
| compile | false |
| freeze vision encoder | false |
| train expert only | false |
| optimizer | PI05 现有 AdamW preset |
| learning rate | `2.5e-5` |
| chunk / execute | 25 / 10 |
| train/eval | 12/0，所有派生 segment 进入 train |
| seed | 固定并写入 run metadata |

执行顺序：

1. 1 batch 前向/反向，无保存；
2. 20 steps smoke，执行 save → reload → resume；
3. 500 steps 检查训练 loss 和离线动作趋势；
4. 通过后跑到 3,000 steps，按 500 steps 保存 checkpoint；
5. 固定一组左臂、左手、右臂、右手 probe windows，输出分组误差曲线和 q54 预测/目标轨迹图。

正式输出目录：

`/workspace/artifacts/pi05base-lerobot/wuji_overfit/banana_in_bowl_q54_h25`

本阶段只要求训练 loss 明显下降、输出轨迹向示范阶段收敛；不以 success、泛化或真机 rollout
作为完成门槛。

### Phase E：后续预埋，不在本需求实施

- Isaac 中以三视角图像和 q54 state 调用 checkpoint；
- `n_action_steps=10` 起步，需要更强闭环时只改为 5；
- 加入 q54 限位、速度与真机安全层前，先核对 NERO/Hand2 权威硬件事实；
- 真机数据与当前 140° 仿真视角差异单独处理，不改写本训练 revision。

## 9. 必测验收

### 数据

- source revision checksum 不变；
- 88 个派生 episodes、11,540 rows、9,428 anchors 精确成立；
- 每个 anchor 都满足连续 25 个 `transition_valid=true`；
- 三路 RGB、state、action、task 与 source lineage 可回溯；
- sampler 全枚举中 `action_is_pad` 全 false。

### 维度与 tokenizer

- q54 名称和顺序逐项一致；
- processor 输出 state 54、action `[25,54]`；
- 14,552 个真实 prompt 在上限 256 下无截断，且 `Action:` 尾标存在；
- model projection 形状为 `(expert_width,54)` 和 `(54,expert_width)`；
- forward、loss、sample action 均为 H25/q54。

### checkpoint

- 两个 action projection 模块完整重新初始化；
- 其余预训练 tensor 零 missing、零 unexpected、数值完全一致；
- 任意其他 shape mismatch 直接失败；
- 不允许出现“加载失败后继续使用随机模型”。

### 训练

- 1 batch forward/backward finite；
- q54 全维均有 finite gradient；
- 20-step checkpoint 能严格 reload 和 resume；
- 500/3,000-step run 保存 config、processor、stats、seed、dataset revision 与 source commit；
- 输出四组曲线：左臂 q7、左手 q20、右臂 q7、右手 q20；
- loss 和轨迹只用于过拟合趋势判断，不包装成 success 指标。

### 工程质量

- `uv run ruff format --check`；
- `uv run ruff check`；
- `uv run pytest` 运行 PI05、processor、sampler 的聚焦测试；
- 新脚本 `--help` 和小样本 dry run 可执行；
- diff 中不存在新框架、防御性 fallback、无关重构或本机绝对路径。

## 10. 完成定义

满足以下条件即完成本需求：

1. H25 训练 revision 通过全量窗口审计；
2. PI05 以真实 q54/H25 batch 完成训练、保存、恢复；
3. 预训练主体被严格复用，只有 action 输入/输出 projection 重新初始化；
4. 3,000-step run 产出可回读 checkpoint 和四组离线轨迹对比；
5. 代码保持 LeRobot 原有架构，没有 FAST/PCA、通用 adapter 或防御性模块；
6. 文档记录数据、代码、模型和训练 artifact 的精确 revision。

当前没有待用户选择的架构项；后续开发应直接按本计划依次推进。
