# 2026-08-03 ROS2 banana grasp pilot -05 质量分析

- Run：`banana-grasp-pilot-20260803-05`
- Analyzer：`teleoperation_quality 0.1.2`
- 分析窗口：完整 control tick span，不裁剪 warm-up 或任务片段
- 结论：录制与因果链基本完整，但**不满足当前四流 60 Hz 数据集基线**

## 1. 可复现产物

Workstation2 输入：
`/home/lenovo/swy/wujihand_recording_20260803/artifacts/runs/nv5/robolab-banana-bowl/banana-grasp-pilot-20260803-05`

Workstation2 输出：
`/home/lenovo/swy/wujihand_recording_20260803/artifacts/analysis/teleoperation-quality-v0.1.2/banana-grasp-pilot-20260803-05`

本地镜像：
`artifacts/analysis/teleoperation-quality-v0.1.2/banana-grasp-pilot-20260803-05`

- input MCAP SHA-256：`46cbedd8d0e92b48dfa950ec98c898ebb874121443c5aae8e897e030bcccbd5b`
- `summary.json` SHA-256：`dd8550977a3d509d4ab22a7580cd93b7dd7957ca880f315e521081f912a0b8fe`
- 输出 `checksums.sha256` SHA-256：`f1d3b3b39c09c402ddda12589fdb189f574ace659c414839464f521557a8b392`

远端与本地均已对 `checksums.sha256` 全量校验通过。分析器源码 hash、输入 checksum 和配置
保存在 `analyzer_manifest.json`。

## 2. 分析器资格验证

- Ruff：PASS；strict mypy：PASS；synthetic golden tests：`12 passed`；
- Workstation2 ROS2 reader 全量解码：`102129/102129` 条 message schema/shape 校验通过；
- topic：`20/20` 非空，decoded count 与 rosbag metadata 完全一致；
- tick：`4918` 个唯一 tick、`9836` 条双侧 trace；无缺 tick、重复、单边记录或双侧 stage-time
  不一致；
- 输入 artifact 分析前后 checksum 不变；输出目录原子创建且不可覆盖。

这些结果支持当前 recording schema 下的实现逻辑。它们不验证尚未录制的 contact、task truth
或真机 readback 指标。

## 3. -05 数据质量

### 完整且正确的部分

- 每个 tick 均有 arm 与 hand trace；raw Tracker/Glove source reference 全部精确命中唯一 raw
  sample，无 duplicate key；
- Glove q21 完整保存：左 `11241`、右 `10893` 个 raw sample，21 个 landmark valid ratio 均为
  `1.0`；
- 计算后 q20 intent、q20 command、q7 command、pre/applied/post q27 均已保存；左右 hand 各有
  `971` 个 new q20 intent；
- inbox 对账通过：左右 Tracker 均 `accepted=992`，左右 Glove 均 `accepted=971`，四路
  `overwritten=0`，全部等于 trace-selected count；
- q7/q20 到 applied q27 的最大组合误差为 `0 rad`；
- banana 动态状态逐 tick 完整：`4918/4918`，无重复且 prim inventory 一致。

唯一失败的 structural gate 是 raw source sequence 连续性：右 Glove 观测到 `348` 个 sequence
hole，discontinuity ratio `0.0309581`，原始有效频率 `116.285 Hz`；另外三路约 `120 Hz` 且
无 gap/duplicate/reorder。当前证据不能把这 348 个 hole 直接命名为网络或 DDS 丢包。

### 不满足数据集基线的部分

| 指标 | 代码输出 | 当前判断 |
|---|---:|---|
| control rate | `51.800 Hz` | 未达 60 Hz ±2% |
| tick interval P95 | `25.043 ms` | 未达 <=20 ms |
| tick 超过 16.667 ms 比例 | `0.814521` | 控制周期普遍偏慢 |
| tick 超过 20 ms 比例 | `0.293268` | 长尾明显 |
| arm trace-selected rate（L/R） | `10.440 / 10.440 Hz` | raw 120 Hz 未被控制链充分消费 |
| hand trace-selected rate（L/R） | `10.219 / 10.219 Hz` | 同上 |
| comparable input age P95（L arm/hand） | `87.881 / 85.006 ms` | 未达 <20 ms |
| comparable input age P95（R arm/hand） | `88.850 / 86.152 ms` | 未达 <20 ms |
| four-stream actionable coverage | `0.0` | 没有四路同时 actionable 的 tick 区间 |

world-step 是 P95 最大的互斥 stage：`22.088 ms`；spin/control/apply P95 分别为
`0.515/0.890/0.284 ms`。这证明当前主要 tick 周期成本在 Isaac world-step；raw 输入到
trace-selected 仅约 10 Hz 的边界还需按 NV-5.1 executor/mailbox 调度计划单独修复，不能只靠
提高传感器发布频率解决。

### Arm 与 Hand 状态

- 左/右 arm duration-weighted actionable coverage 为 `0.171416/0.143875`；主要 degraded
  reason 是 tracker non-actionable hold 与等待 reference。左 arm rate-limited tick ratio
  `0.099837`，右 arm 为 `0.017283`。
- 左/右 hand actionable coverage 为 `0.985629/0.985081`。左 Glove minimum confidence 低于
  `0.6` 的 raw sample ratio 为 `0.209946`；右侧为 `1.0`。当前没有低 confidence rejection：
  左 hand retarget 为 `3813 success + 1040 degraded`，右侧 `4851` 个均为 degraded，rejection
  count 均为 `0`。
- 右 hand position-clamped tick ratio 为 `0.986377`；这是当前姿态/映射状态的显著事实，
  不能被“右侧基本静止”解释为无问题。
- post-step command-feedback RMSE：左/右 arm 为 `0.012279/0.018860 rad`，左/右 hand 为
  `0.080250/0.083837 rad`。当前没有冻结 joint-normalized release threshold，因此只做描述，
  不据此给 arm 与 hand 排名。

banana 轨迹记录显示 path length `1.263895 m`、最终位移 `0.282419 m`、最大抬高
`0.122105 m`。当前没有 task truth/contact/fingertip 信号，因此这些数值只能证明物体发生移动和
抬高，不能由分析器判定“抓取成功”或“放到碗上成功”。桌子和碗是 manifest-only fixed body，
也不能验证其运行时漂移。

## 4. 决策

`-05` 适合作为录制契约、因果 join、q21→q20→q27 链和性能瓶颈的诊断样本，保留为不可变
pilot。它不应直接作为“60 Hz、四流同步、任务成功”的正式轨迹数据集基线。下一步按
[NV-5.1 60 Hz 控制计划](../005-ros2-isaac-60hz-control-feature-plan.md) 修复 executor/control/
physics/render 调度，再用相同 analyzer 做重复 trial；任务成功率要等待 task truth/contact 能力。
