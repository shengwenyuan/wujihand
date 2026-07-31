# 2026-07-31 RoboLab 静态 Workcell 部署验证

| 字段 | 值 |
|---|---|
| 状态 | `PASS` |
| 范围 | RoboLab 静态场景、ModelScope 恢复、共享 Isaac Workcell materializer |
| 首要具身体 | 双 NERO + 双 Wuji Hand 2 |
| 目标机 | Workstation2，Isaac Sim 6.0.1 / Python 3.12 |
| 隔离部署 | `/home/lenovo/swy/wujihand_nv4` |
| 未包含 | Task 第六层、IsaacLab、模型、reset/randomization/training |

## 结论

- Workcell v1 typed compatibility leaf 已归一为
  `ResolvedIsaacWorkcellPlan`；primitive-only、USD import 和 hybrid Workcell 共用同一
  materializer。
- `base_empty`、`banana_bowl`、`workdesk` 三套布局均只通过 profile、Workcell 和
  Session 配置切换，没有场景专用 Python 分支。
- 三套场景的 environment-only smoke 与双 NERO + 双 Hand 2 scripted qualification
  全部通过；原有单 Hand 2 简单桌面也通过共享 materializer 回归。
- 现有五层 schema 未升级，Task、IsaacLab 和模型边界未实现。

## ModelScope 资产

固定来源：

```text
dataset   sss225/robolab-assets
commit    377fb4959532d2ee6055d3a874f25c4b327e2894
manifest  abbf0a4c3799a7470fa41918b802288aa70e3e8b1d3938d547836481a39ea427
```

Workstation2 已从原有完整下载 seed 到项目相对 canonical path：

```text
third_party/src/modelscope/sss225/robolab-assets/
  377fb4959532d2ee6055d3a874f25c4b327e2894/
```

完整校验结果为 3,285 blobs、663 trees、6,814,007,813 bytes。再次使用
`--no-network` ensure 返回 `action: ready`。没有使用 Git LFS 或 symlink；下载、
staging、完整校验、原子提升和 receipt 均只发生在 Workstation2。

## Isaac 6.0.1 场景矩阵

每个静态 smoke 运行 60 帧；每个 NERO smoke 使用既有 q27 qualification，
`frames_per_phase=120`。

| 场景 | collider | rigid body | 静态 smoke | NERO + Hand 2 |
|---|---:|---:|---:|---:|
| `base_empty` | 8 | 1 | PASS | PASS |
| `banana_bowl` | 10 | 3 | PASS | PASS |
| `workdesk` | 22 | 15 | PASS | PASS |

三套最终 stage 均只有一个 `PhysicsScene`、一个 project-selected HDR light，USD
dependency closure 完整，`runtime_module_dependencies` 为空。NERO 报告的
`passed` 均为 `true`，没有失败 check。

后续 reachability 对齐保持原合格 Workcell 的双臂 world pose 和 0.64 m 间距不变，
将 RoboLab 的原始 Franka/DROID anchor 对齐到双臂中心，并把场景 `+X` 方向旋转到
项目 `+Y`。两个底座由上游 `franka_table` 直接支撑，首版临时 fixed plinth 已删除；
主操作桌位于双臂正前方。详见
[banana-in-bowl ROS Deployment 验证](2026-07-31-robolab-banana-bowl-ros.md)。

## 向后兼容

原 `isaac_hand2_fixed_preview_v1` primitive-only Workcell 通过同一个
`ResolvedIsaacWorkcellPlan -> materializer` 路径运行 120 帧：

```text
dof_count               20
limits_match             true
actual_finite            true
feedback_within_limits   true
physics_scene_count      1
```

因此丰富场景没有替换或破坏之前的简单桌面表达。

## 自动验证

本机：

```text
pytest
661 passed, 4 skipped, 11 deselected

ruff check .
All checks passed

mypy src
Success: no issues found in 97 source files
```

Workstation2 配置、source-lock 和 ensure focused tests：

```text
8 passed
```

## 证据

Workstation2 报告根：

```text
/home/lenovo/swy/wujihand_nv4/artifacts/validation/robolab-static/
```

| 报告 | SHA-256 |
|---|---|
| `base-empty/workcell.json` | `e0ee1c9a8ab5638f6f357a14c7220dc287921f3eb4bc625a060c2ff8964c095f` |
| `base-empty/nero.json` | `c0ea1a0eb442debbc849bf9d4aa3700390ffd27559e4e606a283a88b17ac3524` |
| `banana-bowl/workcell.json` | `7e83419f5b36e4825b2c5902b8ce8b8df15c8babee3320fd7c03e487b851801e` |
| `banana-bowl/nero.json` | `e4fcdee45c29a61ff97507ad3d63649c0a11c2a5c0b3ed554f9c314f48b12bd5` |
| `workdesk/workcell.json` | `fae362543a38f615d749ea4444f9e90b66bc505ffbcf544e429281cb9a099dcf` |
| `workdesk/nero.json` | `cc93eebf1adb328c3e3a603508f795352e058eaabe7f1434a151ddd68895a795` |
| `fixed-hand-regression/validation.json` | `39552f05c5d7aca32d27e856a11439d083f6d087d433a79d2d6e1d2fc50445b6` |

## 已知兼容警告

Isaac Sim 6.0.1 对部分上游 OmniPBR `metallic`、`roughness`、
`roughness_texture` 和 Hand 2 `texture_wrap_*` 参数打印 MDL representation
warning。它们没有造成 unresolved reference、缺材质、非有限刚体状态或
qualification 失败；当前按非阻塞渲染兼容警告记录。

本轮没有做实时帧率基准、episode reset、随机化、任务判定、训练或真实硬件动作。
