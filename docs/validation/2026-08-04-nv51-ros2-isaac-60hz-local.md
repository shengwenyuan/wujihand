# NV-5.1 ROS2—Isaac 60 Hz 实现与 Workstation2 验证

- 日期：2026-08-04
- 范围：不连接 Tracker、Glove 等真实输入设备，使用 120 Hz synthetic source
- 结论：headless 30 s 全 Gate 通过；GUI 20 Hz 的 10 s 全 Gate 通过；GUI 30 s 仅“零调度漏周期”Gate 未通过，暂不进入正式采集

## 实现结论

1. ROS callback 由独立 executor 执行，只做契约转换与 latest-only 写入；控制线程按唯一 tick cutoff 原子快照。
2. 调度目标固定为 physics 120 Hz、control 60 Hz、GUI preview 20 Hz；GUI 不定义数据集采样率。
3. 每个 q7/q20/q27 target 覆盖两个连续 physics substep；V2 trace 记录输入 receipt、cutoff、调度、物理与 render 事实。
4. analyzer `0.2.1` 可离线验证 wire/schema、mailbox 守恒、两步物理、频率、输入年龄、RTF 与漏周期。
5. GUI 已设置 `block_on_render=false` 并冻结 Python GC；前者不能消除 Kit `app.update()` 的偶发长阻塞。

## Workstation2 结果

### Headless 30 s：全 Gate 通过

- run：`/home/lenovo/swy/wujihand_nv51_20260804/artifacts/runs/nv5/robolab-banana-bowl/nv51-synthetic-headless-pcores-20260804-04`
- control：`59.99985 Hz`
- tick interval P95：`17.005 ms`
- physics RTF：`1.00026`
- schedule miss：`0`
- 四路输入年龄 P95：均小于 `8.8 ms`

### GUI 20 Hz，10 s：全 Gate 通过

- run：`/home/lenovo/swy/wujihand_nv51_20260804/artifacts/runs/nv5/robolab-banana-bowl/nv51-synthetic-gui20-catchup2-gcfreeze-smoke-20260804-02`
- control：`59.99988 Hz`
- tick interval P95：`19.892 ms`
- render：`20.00001 Hz`
- physics RTF：`1.00083`
- schedule miss：`0`
- 四路输入年龄 P95：`9.23～9.36 ms`

### GUI 20 Hz，30 s：结构 Gate 通过，零漏周期 Gate 未通过

- run：`/home/lenovo/swy/wujihand_nv51_20260804/artifacts/runs/nv5/robolab-banana-bowl/nv51-synthetic-gui20-nonblocking-20260804-01`
- 录制：`35,466` 条消息，`20/20` topics 完整
- control：`59.24227 Hz`，满足 `60 Hz ±2%`
- tick interval P95：`20.247 ms`，满足 GUI `<=25 ms`
- render：`19.74716 Hz`
- physics RTF：`0.98767`
- 四路输入年龄 P95：均小于 `9.48 ms`
- schedule miss：`23/1800`，最大 control interval `78.843 ms`

这次运行消除了此前约 `150 ms` 的 Python GC physics stall，但 Kit `app.update()` 仍偶发约 `78 ms` 阻塞。因此不能通过继续堆叠局部 renderer 开关来宣称正式采集达标。

## 回归结果

- 主仓库：`709 passed, 4 skipped, 11 deselected`。
- analyzer：`15 passed`。
- mypy：核心 `122` 个 source files与 analyzer `11` 个 source files均通过。
- Ruff：目标文件与 analyzer 均通过。
- 最后一次 runner 调整后：编译、目标 Ruff、scheduler/cpu/executor 聚焦测试 `20 passed`、`git diff --check` 均通过。

## 停止点

停止继续做局部 GUI 微调。若坚持正式采集“零 schedule miss”，下一步应把 GUI viewer 与 120/60 Hz 采集进程解耦；若允许有限 miss，必须先单独冻结 miss ratio 与最大间隔 Gate。真实输入 pilot 尚未执行。
