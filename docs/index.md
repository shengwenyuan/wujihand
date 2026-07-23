# 正式文档索引

本目录保存可版本化、可复现的项目正式文档。动态执行计划不放在这里，而放入被 Git 忽略的 `plans/`。

## 当前文档

- [000：项目章程、执行预演与全局架构](000-project-charter-and-architecture.md)
- [MediaPipe—Wuji Hand 2—Isaac 控制链路](components/mediapipe-isaac-control.md)
- [MuJoCo FR3 v2—Wuji Hand 2 桌面环境](components/mujoco-fr3-hand2-table.md)
- [ADR-0001：Hand 2 固定法兰采用 D6 三轴转向](decisions/0001-hand2-fixed-flange-d6-rotation.md)
- [ADR-0002：MuJoCo 采用 FR3 v2 + Hand 2 的运行时组合](decisions/0002-mujoco-fr3v2-hand2-composition.md)
- [MediaPipe 控制 Hand 2 转向抓球指南](guides/mediapipe-hand2-rotation-ball.md)
- [MuJoCo FR3—Wuji Hand 2 桌面运行指南](guides/mujoco-fr3-hand2-table.md)
- [2026-07-13 垂直切片验证报告](validation/2026-07-13-mediapipe-isaac-vertical-slice.md)
- [2026-07-13 Hand 2 固定法兰转向抓球最终验证](validation/2026-07-13-hand2-rotation-ball.md)
- [2026-07-13 MuJoCo FR3 v2—Hand 2 桌面初版验证（历史）](validation/2026-07-13-mujoco-fr3-hand2-table.md)
- [2026-07-14 MuJoCo FR3 v2—Hand 2 长边侧置四棱台验证](validation/2026-07-14-mujoco-fr3-hand2-table-layout.md)

## 后续维护位置

- `architecture/`：稳定的系统结构、数据流和部署视图。
- `components/`：已实现功能、真实代码入口、配置入口和限制；未实现的能力不得先写成“已支持”。
- `guides/`：可执行的安装、运行、采集、回放和排障流程。
- `reference/`：项目数据 schema、坐标、关节布局、配置字段等长期参考。
- `validation/`：验证矩阵、基准结果和验收证据。
- `decisions/`：影响边界、依赖、数据事实源或安全策略的 ADR。

每个功能合入时，至少同步更新对应 component 文档中的代码入口和验证入口。
