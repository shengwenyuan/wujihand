# ROS2 遥操作离线质量分析器

- 状态：已实现
- 当前版本：`0.1.2`
- 支持录制契约：`wujihand.teleoperation_tick_trace.v1`
- 代码：[analysis/teleoperation_quality](../../analysis/teleoperation_quality/README.md)
- CLI：[analyze_teleoperation_run.py](../../tools/analysis/analyze_teleoperation_run.py)

## 1. 边界

分析器是独立 Python package，不被 `src/wujihand`、ROS2 节点或 Isaac 控制循环导入。
生产主线只录制原始事实；所有 count、ratio、percentile、误差和图片均在完整 run 关闭后计算。

输入 run 必须只读。分析器拒绝 incomplete/非零退出/未闭合录包、schema 或 run ID 不一致、
topic/MCAP inventory 不一致、checksum 缺失或不匹配。输出必须位于输入 run 外的新目录；工具
先写临时目录，成功后原子发布，并拒绝覆盖已有目录。

## 2. 当前可重建的因果链

| 层 | Arm | Hand |
|---|---|---|
| 人体原始观测 | Tracker SE(3)、valid/state/sequence/time | Glove q21 三维 landmark、valid/confidence/sequence/receive time |
| 算法结果 | mapping target、IK candidate q7、残差/status | retarget intent q20、confidence/status |
| 安全决策 | q7 command、state/reason、clamp/rate-limit | q20 command、state/reason、clamp/rate-limit |
| Isaac 应用与反馈 | 合并前反馈、applied q27、world-step 后 q27 | 同一原子 q27 中的 Hand2 分区 |
| 场景 | 逐 tick dynamic rigid-body pose/可用 velocity | 与 arm 使用同一 scene/tick truth |

raw Tracker/Glove 与 tick 中选中的 source 使用
`kind + side + source_id + producer_instance + transport_epoch + sequence` 精确连接。每个引用
必须只命中一个 raw sample；q7/q20 按 manifest 的 27 关节分区重组后必须与 applied q27 一致。

因此，q21 和计算后的 q20 都必须保存：q21 用于重放 retarget、分析输入质量；q20 intent 用于
区分 retarget 与 supervisor/Isaac 后段问题。只保存其中一个会切断因果定位。

## 3. 冻结统计口径

- raw intrinsic rate：同一 raw stream 内 `(N-1)/(last_time-first_time)`；表示源自身节奏。
- trace-selected full-window rate：`(N_selected-1)/(last_tick-first_tick)`；表示完整录制窗口内
  真正被 control tick 取走的新输入吞吐，不用 raw rate 或 tick ratio 代替。
- control rate：唯一双侧 tick 的 `(N-1)/(last_tick-first_tick)`。
- sequence discontinuity：按同一 producer/epoch 的 observed sequence gap 统计；只能称为
  “观测到的序列不连续”，不能直接归因于 UDP、DDS 或网络丢包。
- comparable input age：有可信 acquisition/source time 时使用它；否则显式回退到 host
  receive time。Glove acquisition time 缺失保持 `NA`，不补零。
- actionability：`safety_state=tracking` 且 active source 存在；coverage 按相邻 tick 时间加权，
  同时保留 tick ratio。四流 coverage 要求左右 arm/hand 在同一 tick 全部 actionable。
- inbox 对账：每路 `accepted = trace-selected + overwritten`。它只能解释 callback 已到达后的
  latest-only inbox，不替代 DDS/executor 诊断。
- command-feedback error：command 与同 tick `world.step()` 后 feedback 的逐关节误差；当前只做
  描述统计，不在没有关节归一化范围时比较 arm 与 hand 优劣。
- stage duration：从录制的 monotonic stage timestamps 相减；pipeline 是总量，dominant stage
  只在互斥的 spin/control/apply/world-step 中比较 P95。

默认 `60 Hz ±2%`、tick interval P95 `<=20 ms`、comparable input age P95 `<20 ms` 是
NV-5.1 计划参考，不是未经重复试验冻结的数据发布阈值。

## 4. 输出

一次成功分析固定生成：

- `summary.json` / `summary.csv`：机器可读总览；
- 每项指标的 CSV 和 `derived/` 下完整对齐样本表；
- 12 张确定性 PNG 和 `figure_manifest.csv`；
- `report.html`；
- `analyzer_manifest.json`：分析器版本、配置、输入 checksum、全部分析源码 hash；
- `checksums.sha256`：除自身外所有输出文件的 hash。

分析窗口固定为完整 control tick span，不自动裁剪 warm-up、静止段或人工选择的成功片段。

## 5. 当前不能得出的结论

capability 不存在时，报告明确保留 `NA/unsupported`，不得推断：任务成功、接触/力/穿透、
掌心或指尖质量、桌子/碗运行时漂移、真机 NERO/Hand2 跟随质量。没有预注册激励窗口时也不
估计 command-feedback lag。

## 6. 执行与验证

在能够读取录包 custom message 的 ROS2 Jazzy 环境中执行：

```bash
python tools/analysis/analyze_teleoperation_run.py \
  --run-root /absolute/path/to/complete-run \
  --output-root /absolute/path/to/new-analysis-directory
```

package 自带 synthetic golden tests，覆盖已知 50 Hz、已知 source age、已知 0.01 rad
feedback error、q21/q20/q27 字段、duration-weighted coverage、sequence gap、因果 join、q27
组合、inbox 对账、输入只读、输出原子性和 checksum。正式 run 还必须执行一次完整 ROS2 reader
集成分析并校验输出 checksum。
