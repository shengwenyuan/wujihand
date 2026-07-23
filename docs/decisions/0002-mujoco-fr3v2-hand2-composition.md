# ADR-0002：MuJoCo 采用 FR3 v2 + Hand 2 的运行时组合

状态：Accepted

日期：2026-07-13

## 背景

当前需求是在桌边固定一台单臂，法兰连接 Wuji Hand 2 right，并保留未来接入多种机械臂/手部遥操作和数据采集方式的边界。Wuji 官方提供 Hand 2 Beta 1 MJCF 和通用 STEP 挂载件，但没有给出与某款桌面机械臂配套的已认证数值安装变换。

2026-07-14 布局修订把 FR3 从桌面移到长边外侧的较低四棱台；这只改变 scene geometry，不改变本 ADR 的资产组合、法兰拓扑或 27-DoF 控制决策。

## 决策

1. 机械臂采用 Menagerie `franka_fr3_v2`，固定 commit `71f066ad…`；不用已经停止产品支持的 Panda 作为新基线。
2. 固定 MuJoCo `3.10.0`。FR3 与 Hand 2 上游 MJCF 保持只读，用 `MjSpec` 在运行时解析、附着、编译。
3. 在空末端 body `fr3v2_link8` 原点新增 `fr3_flange_to_hand2` frame，将 Hand 2 `r_base_link` 固定附着；不加入 Isaac ADR-0001 的 D6 腕部自由度。
4. 当前 attachment 使用 identity，并在配置中强制保留 `identity_until_physical_adapter_transform_is_measured` 标记；不得称为机械图纸真值。
5. 当前固定资产名称互不冲突，attachment 使用空 prefix 以保留 Hand 2 canonical 关节名。组合 spec 只在内存编译，不导出 XML；若未来需要可移植组合资产，改用显式 asset/VFS 打包并重新做 license 与 round-trip 验证。
6. 桌面 scene 是唯一全局物理事实源。父、子 spec 在 attach 前统一设置 `2 ms + implicitfast + Newton + sparse + 100 iterations + 1e-8`，不依赖 child `<option>` 合并行为。
7. 控制 contract 分为 `arm_q7` 和 `hand_q20`，按关节名发现地址和唯一执行器。拒绝匿名 27 维 action，也不改变 `HandCommand v2`。
8. 第一版仅提供严格位置目标和反馈。未来 arm teleop 新增末端位姿 intent/controller；网络失联时默认 hold，不自动扫回 home。

## 未采用方案

| 方案 | 原因 |
|---|---|
| 直接修改/复制两份 MJCF | 上游升级、来源、license 和差异难以追踪 |
| Panda/FER | 历史生态成熟，但不作为新硬件方向 |
| 在 Hand 2 前增加 D6 wrist3 | 与真实 FR3 法兰拓扑不符，也会把 27 DoF 变成 30 DoF |
| 给 Hand 2 全部名称加 prefix | 会破坏现有 canonical q20 精确名称；当前 direct compile 无重名 |
| 把 arm + hand 填入统一裸数组 | 无法表达产品/layout，容易把偶然索引当契约 |
| 自动 clamp 超限目标 | 掩盖输入或映射错误；基线 adapter 应 fail closed |
| 立即实现 IK/OSC/遥操作 | 当前需求只要求正确机械基线，控制方式尚未唯一确定 |

## 后果

- 优点：得到单一 27-DoF 运动学树；上游资产可替换；Hand 2 q20 与现有 domain/profile 一致；普通项目导入不强制安装 MuJoCo。
- 代价：运行时依赖两份 source tree；无法把 `to_xml()` 输出当作可恢复资产；identity 转接件仍需物理标定。
- 风险：Hand 2 上游碰撞过滤不是全表面碰撞；Menagerie FR3 v2 模型和真实硬件版本变化都要求重新验收。

## 重验触发器

MuJoCo、任一 MJCF/hash、FR3 硬件小版本、Hand 2 产品代际、法兰 transform、全局 solver/contact 参数、关节名称/限位或碰撞 profile 变化时，必须重新执行 27-DoF contract、10 秒 home hold、平滑桌面接触和离屏渲染目检。
