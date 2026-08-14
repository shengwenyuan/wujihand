# Wuji Hand2 Beta1 右手 H0–H2 只读资格验证

- 日期：2026-08-11 至 2026-08-12
- 设备：Wuji Hand2 Beta1 右手，SN `WH2KA…03016`，`192.168.1.111:7447`
- 软件：Wuji SDK / Studio / CLI `2026.8.3`
- 设备 readback：hardware `0.2.0`，firmware `2.2.3`
- 范围：独立真机台架、只读；未启动 Studio、Isaac、ROS2、Glove、NERO 或 recorder
- 命令能力：关闭；未 enable、未写参数、未发送 `joint_command`

## 结论

右手 H0 和公共 H1 已通过。H2 已证明身份、20 关节映射、连续状态/诊断流、退出释放、日志导出及
只读边界可用；按 2026-08-12 当时的严格门槛为 **未通过**：长时观测在约 `497 s` 时触发相对温升
`+5.000 °C` guard，重上电后的 30 秒严格 preflight 仍出现两个离散 `85%` response-rate 窗口。

这些现象没有伴随 joint timeout 增量、E2E loss、SDK drop、RPC/CRC/UART 错误、fault、limit 或
非有限值，说明当前状态面总体稳定；但尚无官方依据允许把 `85%` 窗口等同于正常，也没有完成热平衡
资格，因此当时不能把 H2 写成完全通过。

### 2026-08-13 项目策略更新

后续 H3 完整 sequence 与人工观察证明孤立 `85%/87%` 窗口不妨碍本轮受控动作，且仍未伴随
timeout、E2E loss、SDK drop、fault、limit 或非有限值。项目据此接受 `85%`（含）为当前固定栈的
response-rate 下限；低于 `85%` 才阻断，timeout/transport/`comm_diag` counter 只记录不叠加 Gate。

因此 H2 的通信项按当前项目标准关闭；约 497 秒仍未达到热平衡的事实继续保留为长期未关闭项。`85%`
不是 Wuji 官方指标，也不得外推到左手或不同 firmware/SDK/网络拓扑。

## H0：发现与身份

- 专用路由为 `192.168.1.110/31 -> eno1`，`never-default=yes`；Hand2 与两只 Glove 的路由不冲突。
- 右手 `.111` 连通，SDK 8.3 唯一发现目标设备，side、address、serial、hardware 和 firmware 一致。
- 固件已由操作者在 Studio 8.3 中完成升级；正式 SDK 验证前 Studio 已退出。
- 未启动左手，右手结论不得外推到左手。

结论：H0 通过。

## H1：独立 hardware package

已建立 `packages/wujihand-hand2-hardware`：

- 固定 `wuji-sdk==2026.8.3`，不导入主仓库 `wujihand`、ROS2、Isaac、USD 或 dataset runtime；
- 以 SN + side + address fail closed，按 `nid` 重建 q20，不把 SDK 返回数组位置当固定顺序；
- 当前公开 CLI 只具备只读资格验证能力，代码中没有 enable、参数写入或 command 发送入口；
- fake/contract tests 覆盖映射、身份、缺失/重复关节、stale、诊断与 import 边界；开发时 pytest
  `20 passed`，Ruff、strict mypy、独立 wheel 构建通过；
- 同一 package 已在 `lenovo-piper2` 的隔离环境执行真实设备只读验证。

结论：H1 通过，且未修改既有仿真入口行为。

## H2：三组主要证据

| Artifact | 时长/帧 | 温度 | 通信与状态 | 结论 |
|---|---:|---|---|---|
| `wuji-hand2-right-temperature-observe-20260812-01` | `30.000 s`；state/diagnostics 各 `30,000` 帧，sequence gap `0` | baseline max `51.919 °C`，max `52.441 °C`，rise `0.522 °C` | 31 个窗口全为 `100%`；timeout delta `0`；`Ready` / error `0` | 通过 |
| `wuji-hand2-right-temperature-observe-20260812-02` | `496.973 s`；各 `496,968` 帧，sequence gap `0` | baseline max `52.880 °C`，max `57.881 °C`，rise `5.000 °C` | 497 个窗口中 32 个低于 `100%`；timeout delta `0`；`Ready` / error `0` | 触发温升 guard，未通过 |
| `wuji-hand2-right-h2-repower-preflight-20260812-02` | 重上电后 `30.000 s`；各 `30,001` 帧，sequence gap `0` | baseline max `43.738 °C`，max `44.476 °C`，rise `0.738 °C` | NID 18 `ring_S3` 与 NID 21 `pinky_S1` 各出现一次 `85%`；timeout delta `0` | 严格通信 Gate 未通过 |

三组运行均满足：20 个预期 NID 在线且唯一，状态只有 `Ready`，error code 只有 `0`，active limit
样本为 `0`，非有限值为 `0`，总线电压约为 `12.20–12.50 V`，state/diagnostics 无 sequence gap。

长时运行的 32 个低响应窗口分散在多节点，其中 NID 19 `ring_S4` 为 22 次；重上电后异常转移到
NID 18 和 NID 21，因而当前证据不支持“固定某一关节损坏”的判断。最长运行的最高温节点为 NID 13
`middle_S3`；运行在温度仍上升时被项目 guard 主动终止，不能视为热平衡完成。

## 日志与工具差异

- 官方 Wuji CLI 8.3 导出的支持包为
  `wuji-hand2-right-support-20260812-01.zip`，SHA-256：
  `1d799409c19cd4e2f7a047484a0bbcf4e63d6b3fdfa368ddcd00b7deb88c71f5`。
- 支持包内当前 Hand2 SDK 日志未发现 WARN/ERROR。
- CLI diagnosis 对 device type `0x02` 没有诊断 recipe，属于当前 CLI 能力缺口，不等于设备诊断失败。
- 官方滚动 SDK 文档列出设备日志导出能力，但已安装的 Python SDK 8.3 runtime/stub 未暴露对应方法；
  本轮采用同版官方 CLI 导出，没有在项目 adapter 中伪造接口。

## 后续 Gate（2026-08-12 历史交接）

可以进入 H3 的无设备开发与真机只读预检，但在首次运动前必须同时满足：

1. fake SDK 下的单关节 mask、完整 q20 command、watchdog、disable/estop 和 journal 全部通过；
2. 再次读取目标关节实际位置、MIT 参数与 effort limit，不写参数，不采用 Isaac drive 数值；
3. 空载净空、Studio 关闭、唯一 command owner、操作者在场且物理断能路径明确；
4. 新鲜只读 preflight 无 fault/limit/stale，response 不低于项目 `85%` 下限；通信计数保留 receipt；
5. 首次只允许右手单关节、小位移、低速、短时，动作后立即回基线并 disable。

因此当前状态应写作：**右手 H3-dev 可开始，H3-live 尚未授权；左手仍从 H0 开始。**

## 官方依据

本轮于 2026-08-12 通过官方 `wuji-docs` MCP 核对：

- [Hand2 使用约束](https://docs.wuji.tech/docs/zh/wuji-hand/latest/usage-constraints/)
- [Hand2 SDK 接口](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)
- [Hand2 发布记录](https://docs.wuji.tech/docs/zh/wuji-hand/latest/release-notes/)
- [Wuji SDK](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/)

Hand2 Beta1 不具备可用于本项目闭环接触判断的指尖触觉；电流也不能被当作可靠力/接触真值。
