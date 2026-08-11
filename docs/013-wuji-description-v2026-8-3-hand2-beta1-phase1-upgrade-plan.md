# 013：Wuji SDK + Description v2026.8.3 / Hand2 Beta1 同版链阶段一计划

- 状态：实施中；阶段一 matched-chain preflight、Studio 8.3 具名用户左右校准 URDF、SDK 8.3
  静态/Glove/Isaac 验收已完成；并列的双 NERO + 双 Hand2 8.3 + Glove/Tracker + record 配置、
  fail-closed 入口及无设备严格零漏全链已完成，待操作者 live qualification record
- 日期：2026-08-11
- 历史基线：`wuji-sdk==2026.7.21` + `wuji-description v2026.6.27`
  (`aee64892ebcf8e3237bedc30231bb09476cbc71d`)
- 唯一目标链：`wuji-sdk==2026.8.3` + `wuji-description v2026.8.3`；其中 Hand2 Beta1
  Description 模型的实质修订来自 `v2026.7.23`
- 范围：SDK/Description 同版锁定、第三方资产双版本共存、加载/组合工具适配、Hand2 独立
  静态/动态资格验证和真人穿戴 Glove 的匹配链验收
- 当前拓展：收口双 NERO + 双 Hand2 8.3 仿真 record 全链；Hand2 Beta1 左右真机 bring-up、
  ROS 2 hardware executor 与 Sim/Real 对照仍另立中等需求

## 0. 结论

阶段一不是只替换 `wuji-description` 目录，也不允许继续用旧 SDK 对新版 Description 下结论。
本轮有效验收身份必须同时满足：

```text
sdk_package_version = 2026.8.3
retarget_model_id   = wuji_sdk.WujiHand2.2026.8.3
description_release = v2026.8.3
```

阶段一提交不切换现有 NERO、ROS 2、D405、数据集和真机入口；Glove 先只进入独立 Hand2
qualification 链。第 10 节后继拓展通过全新的版本化 Assembly/Session/Deployment 并列接入完整
pipeline，历史入口和默认 SDK 锁仍不改变。

本阶段采用以下策略：

1. `v2026.6.27` 与 `v2026.8.3` 两套 Description 来源、配置和本地恢复目录同时存在；
2. 历史 `wuji-sdk==2026.7.21` 环境证据与目标 `wuji-sdk==2026.8.3` 资格验证环境隔离，
   目标版本先由显式 overlay 选择，禁止在共享 Isaac 环境中原地覆盖后再解释旧报告；
3. 所有拥有 SDK/Description 版本事实的文件、锁和报告均带明确版本；
4. 所有调用方通过配置或锁文件显式选择具体版本，禁止 `latest`、`current`、无版本别名和运行时
   自动择新；
5. 现有复杂 Session 继续显式锁定历史链，阶段一只开放独立 Hand2 的 `2026.8.3 +
   v2026.8.3` static、Isaac 和 Glove qualification 入口；
6. 先消除 SDK API/版本、加载器、resolver、compatibility bridge 和仿真 adapter 中的旧版本
   隐含假设，再进行同版链验证；
7. `SDK 2026.7.21 + Description v2026.6.27` 的左右 Glove 结果只作为历史诊断基线，不能
   冒充目标链验收；
8. 阶段一通过只表示新版数字孪生与穿戴输入链可复现、可选择、可测量，不表示与真机一致，也不授权
   任何真机控制参数。

## 1. 官方版本事实与明显警告

### 1.1 版本事实

当前锁定版 `v2026.6.27` 使用：

- 目录：`hand2_beta/body/...`；
- Isaac USD：`usd/{left,right}/wujihand.usd`；
- 根链接：`{l,r}_base_link`；
- 当前仓库 source lock commit：
  `aee64892ebcf8e3237bedc30231bb09476cbc71d`。

目标 source release 为 `v2026.8.3`，但 Hand2 模型内容继承 `v2026.7.23` 的重新标定版本：

- 目录：`hand2/hand2_beta1/body/...`；
- Isaac USD：`usd/{left,right}/wujihand2.usd`；
- 根链接：`{l,r}_wrist`；
- 关节轴、坐标、link/joint/actuator 命名规则重新标定并自该版起冻结；
- 分层 USD 增加五个指尖查询 site；
- 全部 link 采用 mesh convex hull 碰撞，只排除官方声明的 10 组装配重叠对；
- 指尖软垫 mesh 随包提供，但未作为碰撞几何挂载。

`v2026.8.3` 对 Hand2 模型没有新的几何或动力学修订，只新增 PICO 4 Ultra 手柄安装座 CAD。
本项目仍锁定最新 release，以获得明确的完整上游快照；配置内同时记录：

```text
description_release = v2026.8.3
hand2_model_revision = v2026.7.23
```

目标 Python SDK 必须锁定为 `wuji-sdk==2026.8.3`。2026-08-10 已在 Workstation2 建立隔离 overlay：

```text
/home/lenovo/.venvs/wuji-sdk-overlays/2026.8.3-8acaf6a9
wheel_sha256 = 8acaf6a90e42c78f9523bf007f8c298cee6bfbeddb200550906bf0e069e63da8
```

该 overlay 已通过 CPython 3.12 / Isaac 6.0.1 基础环境上的 import/API、左右静态 retarget 和
Description v2026.8.3 离线 FK 验证。项目 `pyproject.toml`、`uv.lock` 与共享 Isaac 环境仍保持
`2026.7.21`，这是为了避免尚未迁移的 Description v2026.6.27 业务 Session 被隐式组合为混版链，
不是目标资格环境缺失。默认依赖只在后续双 NERO + 双 Hand2 8.3 record 全链通过后切换。

目标资格报告至少同时记录：

```text
sdk_distribution       = wuji-sdk
sdk_package_version    = 2026.8.3
sdk_wheel_sha256       = <verified wheel hash>
retarget_model_id      = wuji_sdk.WujiHand2.2026.8.3
retarget_config_id     = <reported SDK config id>
description_release    = v2026.8.3
description_commit     = <verified source commit>
description_tree_hash  = <verified tree hash>
```

### 1.2 必须长期保留的警告

> **警告：Wuji Hand2 仍是 Beta1。`v2026.7.23` 是一次包含坐标、根链接、目录和碰撞语义的
> 高风险修订；后续官方仍可能更新质量参数和几何细节。任何 `latest` 文档或新 release 都不得
> 自动替换本计划锁定的 source、配置或验证结论。**

此外：

- SDK 与 Description 只要任一版本变化，匹配链静态、Isaac 和穿戴 Gate 全部失效，必须重跑；
- SDK 升级可能改变内置 retarget 配置、默认手型 URDF、标定格式和 API 行为；不得只检查
  `pip` 元数据后沿用旧 q20 结论；
- 官方标定说明明确指出新版不加载旧格式标定 URDF；若使用具名用户，左右手必须在
  `2026.8.3` 下重新标定，禁止复用旧标定产物；
- 当前 USD `kp/kv` 沿用上一代 Wuji Hand 平台，尚未基于 Hand2 真机系统辨识；
- 官方没有提供带软体的 Hand2 仿真模型；
- 新版仍使用凸包碰撞，不能预设它已经解决视觉表面与碰撞轮廓偏移；
- 阶段一的 drive 参数只属于 Isaac 仿真，不得复用为真机 MIT `kp/kd` 或
  `effort_limit`。

官方依据：

- [Wuji Description 发布记录](https://docs.wuji.tech/docs/zh/wuji-description/latest/release-notes/)
- [Wuji Description 集成指南](https://docs.wuji.tech/docs/zh/wuji-description/latest/integration/)
- [Wuji SDK 产品介绍与安装](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/introduction/)
- [Wuji SDK 手部重定向](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting/)
- [Wuji Glove 手型标定](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/calibration/)
- [Wuji Hand 2 使用约束](https://docs.wuji.tech/docs/zh/wuji-hand/latest/usage-constraints/)

## 2. 阶段一范围

### 2.1 本阶段必须完成

- 获取并锁定 `wuji-sdk==2026.8.3` 的 CPython 3.12 / Linux x86_64 wheel、hash 和安装来源；
- 在隔离的 Isaac 6.0.1 Python 环境中验证 SDK 2026.8.3，不原地覆盖当前 2026.7.21 环境；
- 保存 SDK 8.3 版本化环境 receipt，并建立 adapter/preflight contract tests；项目默认依赖锁切换
  延后到上层 8.3 业务链通过之后；
- 恢复并校验两个固定 Wuji Description source tree；
- 建立左右手旧/新两套 Profile、Asset、Isaac Binding；
- 保留当前右手 MuJoCo Binding，并建立对应新版结构加载能力；
- 把 Hand2 source 版本变成显式、可解析、可报告的配置事实；
- 消除工具中对旧目录、`wujihand.usd` 和 `{l,r}_base_link` 的隐式依赖；
- 消除测试、CLI 示例和报告中对 SDK `2026.7.21` 的硬编码或默认继承；
- 建立旧/新版本静态差异报告；
- 用 SDK 2026.8.3 对固定 21×3 canonical skeleton fixture 完成左右静态 retarget 验证；
- 在 Isaac Sim 6.0.1 中完成左右手独立加载、关节、驱动、碰撞、指尖 site 和对指轨迹验证；
- 为 Studio 8.3 具名用户及其左右校准 URDF 建立 hash、side、Glove serial 和 SDK 版本闭合的
  fail-closed preflight；
- 只在 `SDK 2026.8.3 + Description v2026.8.3` 独立 Hand2 场景中完成左右 Glove 真人穿戴
  五指、张合和四组对指验收；
- 证明现有复杂入口仍解析到 `v2026.6.27`，没有被新版污染；
- 产出阶段一 qualification 报告和可重复命令。

### 2.2 本阶段明确不做

- 不连接、发现、使能或控制 Hand2 真机；
- 不更新 Hand2 固件、Wuji Studio 或独立的 Wuji Retargeting 仓库；
- 不把 Glove qualification 扩张为 Hand2 真机 teleop；
- 不切换双 NERO、T 型架、RoboLab、D405、ROS 2 或数据集 Session 到新版；
- 不以新版重新采集 episode，不修改 Pi0.5 训练链；
- 不做仿真参数到真机参数的映射；
- 不宣称凸包碰撞与真实软体指腹一致。

本阶段允许的唯一设备是 Wuji Glove。Glove 只提供 `hand_skeleton`，控制对象只允许是 Isaac 中
独立的 Hand2；不得启动 Tracker、NERO、ROS 2、CAN、Hand2 hardware executor 或 dataset writer。

现有本地 `plans/hand2-real-digital-twin-ros2-reuse-decoupling.md` 只作为阶段二参考，不是本计划
依赖，也不在本阶段更新。

### 2.3 2026-08-10 已完成证据与当前判断

Wuji Studio 8.3 已为同一具名 SDK 用户完成左右手型标定：

```text
user_id = u_8c18aa8e4fb240feaa6e7a6e5760070b
left_urdf  = ~/.wuji/sdk/users/<user_id>/models/left_hand.urdf
left_sha256  = 86b133cf1de0421c51be01a9d2178eb543e4fe7e0538bfab60012cf00af1c483
right_urdf = ~/.wuji/sdk/users/<user_id>/models/right_hand.urdf
right_sha256 = b41f5bf0713d1966c473bcaa0bdf5655e7139d65e6cf24ead49338e5a039fe47
```

SDK 8.3 运行日志已明确记录左右 `online hand_model` 与 `offline hand_model` 均加载上述用户 URDF，
没有回落到内置默认模型。用户 URDF 属于 Glove 在线手骨架求解的 host-local 校准输入，不替换、
不修改 Description 中的 Hand2 机器人 URDF/USD，也不得复制进仓库。

静态匹配链报告位于：

```text
artifacts/validation/wuji-sdk-2026-8-3-static-beta-20260810/
```

它已证明 SDK 8.3 wheel/API、左右固定 fixture retarget、reset 重复性、q20 contract 和 Description
v2026.8.3 离线 FK 均闭合。固定 synthetic fixture 中 SDK 7.21 与 8.3 的 q20 完全相同；该结论只表示
SDK API/retarget 没有意外漂移，不表示穿戴手指必然贴合。

具名用户穿戴报告位于：

```text
artifacts/validation/wuji-sdk-2026-8-3-worn-named-user-20260810/left-01/
artifacts/validation/wuji-sdk-2026-8-3-worn-named-user-20260810/right-01/
```

左右各录得 5521 帧、46 秒、约 120 Hz；supervisor 全程 `tracking`，无 stale/reject、无位置限幅，
左/右分别只有 1/16 次瞬时 rate-limit。对指最后 1 秒的中位距离为：

| 对指 | 左：校准人手骨架 → Hand2 site | 右：校准人手骨架 → Hand2 site |
|---|---:|---:|
| thumb-index | 5.5 mm → 6.5 mm | 7.6 mm → 10.6 mm |
| thumb-middle | 6.1 mm → 5.3 mm | 5.5 mm → 4.3 mm |
| thumb-ring | 11.2 mm → 7.7 mm | 4.5 mm → 4.9 mm |
| thumb-pinky | 6.0 mm → 9.7 mm | 5.5 mm → 7.4 mm |

四组目标手指与拇指均有明显 q20 响应，校准人手骨架距离与 Hand2 site 距离相关性均大于 0.98。
因此“对指完全没有映射”不成立；当前主要残差集中在右食指和左右小指，约 2–4 mm。物理皮肤已经
贴合时，校准骨架端点仍保留约 3–11 mm 距离，说明 SDK landmark/URDF 末端不等于真实皮肤接触面，
不能把人手物理接触预设为关键点距离必须为零。

左/右 `degraded` 分别为 4.3%/22.8%；右侧主要发生在独立屈指、握拳和食指阶段。这一状态来自
最低 landmark confidence 低于 0.6，当前 hard floor 为 0，故没有造成拒绝或控制冻结。阶段一将其
作为质量警告而非功能失败；进入正式 dataset 前必须保留逐帧 confidence，并另行冻结可采纳门槛。

Description 8.3 的左右 URDF 中 26/26 个 link、MJCF 中 21/21 组 visual/collision geom 均引用同一
mesh 和相同变换，没有独立放大的碰撞文件。但 Isaac USD 使用 convex hull，运行时 cooked collider
仍可能填平视觉 mesh 的凹陷并显得更大。

### 2.4 2026-08-11 Isaac 穿戴复验与全链实现状态

正式 matched-chain preflight 已闭合 SDK 8.3、Description v2026.8.3、两个 SDK 运行上下文、具名
用户左右 URDF/hash、Glove serial 和 `{l,r}_wrist`。操作者随后完成左右穿戴 Isaac 复验，判断新版
贴合明显改善且原欠闭合问题已解决；残余间隙可接受。`self_collision=false` 保持不变，因此部分
极端闭合姿态可见轻微指间穿模；这是已知、非当前阻塞项，不以修改官方 drive 或 collider 掩盖。
该结论只适用于仿真视觉与运动学，不表示真实 Hand2 指腹或碰撞已验证。

并列的 8.3 全链已新增独立 Assembly、Session、Deployment 和 record-chain qualification policy。
入口在启动 Glove 或 Isaac 前 fail closed，并要求 Glove source 与 Isaac consumer 都解析到 SDK 8.3；
所有录制固定标记 `qualification_only=true`、`dataset_eligible=false`。无设备 stub 已证明双臂双手
q54、完整场景、operator preview、rosbag、checksum 与离线验证能够闭合；最终 live record 仍需
操作者给 Glove/Tracker 上电并完成短时动作。NERO 与 Hand2 真机在该链中保持断电且不被访问。

严格通过 run 为
`artifacts/diagnostics/dataset-preview-qualification/hand2-83-stub-20260811-10/`：qualification
无失败项；fixture 为 2261 帧、119.995 Hz、0 漏周期，主控制为 1080 tick/2160 physics step、
0 漏周期，preview 为 20.024 Hz、0 漏周期，record/checksum/preflight receipt 完整。

## 3. 版本身份与文件命名规则

### 3.1 四类版本不得混用

| 版本类型 | 示例 | 含义 |
|---|---|---|
| SDK distribution version | `2026.7.21`、`2026.8.3` | Python wheel、内置 retarget/默认 URDF、SDK API 与标定格式 |
| upstream source release | `v2026.6.27`、`v2026.8.3` | Git tag、commit、artifact/tree hash |
| Hand2 model revision | `v2026.7.23` | Hand2 模型语义实际发生变化的发布版 |
| 项目 schema/config revision | 文件尾部 `_v1` | 本项目 YAML schema 或局部配置修订 |

文件名使用下划线编码 upstream release：`v2026_6_27`、`v2026_8_3`；YAML 内容和报告使用
原始 tag：`v2026.6.27`、`v2026.8.3`。

禁止使用以下命名：

- `latest`、`current`、`new`、`old`；
- 无 upstream 后缀的 Hand2 source-coupled 配置；
- 用 `_v2` 表示上游版本变化；
- 让同一个 Asset `revision: beta1` 同时代表两套不兼容根链接。

一个报告只有在 `sdk_package_version=2026.8.3` 与
`description_release=v2026.8.3` 同时满足时才能标记为 `current_matched_chain`。其余组合必须标记
为 `historical_baseline` 或 `mismatch_diagnostic`，即使数值 Gate 全部为 true 也不能升级身份。

### 3.2 双版本 source lock

把当前单一 `wuji-description` 记录拆成两个不可变记录：

| source name | 本地恢复目录 | 作用 |
|---|---|---|
| `wuji-description-v2026-6-27` | `third_party/src/wuji-description/v2026.6.27` | 保留全部历史入口和证据 |
| `wuji-description-v2026-8-3` | `third_party/src/wuji-description/v2026.8.3` | 阶段一新版资格验证 |

每个记录独立保存 tag、commit、license hash、sparse paths、artifact hash 和 asset-tree hash。
恢复或清理其中一个目录不得改变另一个目录。Binding 的 `source` 和 Asset 的
`provenance_source` 必须引用带版本的 source name。

目标 tag 的 commit 和全部 hash 必须在实施时从实际 checkout 计算并写入 source lock；计划文档
不预填未经校验的 commit。

### 3.2.1 SDK 环境锁与升级边界

SDK 是二进制 Python distribution，不与 Description Git source 混用一种恢复机制：

| environment identity | SDK | Description | 用途 |
|---|---|---|---|
| `isaac601_wuji_sdk_2026_7_21_desc_2026_6_27` | `2026.7.21` | `v2026.6.27` | 历史报告复验与回退证据 |
| `isaac601_wuji_sdk_2026_8_3_desc_2026_8_3` | `2026.8.3` | `v2026.8.3` | 本阶段唯一目标资格环境 |

目标环境必须记录解释器路径、Python/Isaac/SDK 版本、wheel 文件名、wheel SHA-256、平台 tag、
`pip freeze`/lock hash 和创建时间。先建立隔离环境并通过无设备 import/API smoke；全部 Gate 通过前
不得在当前 `/home/lenovo/.venvs/isaacsim-6.0.1` 中原地升级。

阶段一不修改 `pyproject.toml` 与 `uv.lock` 的默认 `2026.7.21` 选择；所有目标资格命令必须显式
使用上述 8.3 overlay，并在报告中记录解释器与模块根。待阶段一通过并完成后续双 NERO + 双 Hand2
8.3 record 全链后，再把默认依赖精确切换到 `wuji-sdk==2026.8.3`。历史 `2026.7.21` 的可恢复性
通过 Git 历史、wheel receipt 和独立环境证据保留，不在同一 Python 环境中尝试并存两个同名
distribution 版本。

### 3.3 source-coupled 配置对

以下文件拥有第三方模型事实，旧版保留并统一补足 upstream 后缀，新版建立一一对应文件：

| 层 | `v2026.6.27` | `v2026.8.3` |
|---|---|---|
| left profile | `configs/profiles/hand2_left_v2026_6_27.yaml` | `configs/profiles/hand2_left_v2026_8_3.yaml` |
| right profile | `configs/profiles/hand2_right_v2026_6_27.yaml` | `configs/profiles/hand2_right_v2026_8_3.yaml` |
| left Asset | `configs/assets/wuji_hand2_beta1_left_v2026_6_27_v1.yaml` | `configs/assets/wuji_hand2_beta1_left_v2026_8_3_v1.yaml` |
| right Asset | `configs/assets/wuji_hand2_beta1_right_v2026_6_27_v1.yaml` | `configs/assets/wuji_hand2_beta1_right_v2026_8_3_v1.yaml` |
| left Isaac physical Binding | `.../wuji_hand2_beta1_left_v2026_6_27_physical_v1.yaml` | `.../wuji_hand2_beta1_left_v2026_8_3_physical_v1.yaml` |
| right Isaac physical Binding | `.../wuji_hand2_beta1_right_v2026_6_27_physical_v1.yaml` | `.../wuji_hand2_beta1_right_v2026_8_3_physical_v1.yaml` |
| right Isaac standalone Binding | `.../wuji_hand2_beta1_right_v2026_6_27_v1.yaml` | `.../wuji_hand2_beta1_right_v2026_8_3_v1.yaml` |
| right MuJoCo Binding | `.../wuji_hand2_beta1_right_v2026_6_27_v1.yaml` | `.../wuji_hand2_beta1_right_v2026_8_3_v1.yaml` |
| rotation compatibility profile | `configs/base/hand2_rotation_ball_v2026_6_27_v1.yaml` | `configs/base/hand2_rotation_ball_v2026_8_3_v1.yaml` |
| MuJoCo compatibility profile | `configs/base/mujoco_fr3v2_hand2_right_table_v2026_6_27_v1.yaml` | `configs/base/mujoco_fr3v2_hand2_right_table_v2026_8_3_v1.yaml` |

Asset ID 可以保持产品身份稳定，例如 `wuji_hand2_beta1_left`，但 `revision` 必须区分来源：

```text
beta1_description_v2026_6_27
beta1_description_v2026_8_3
```

Binding 的 `asset_revision`、`source`、`source_revision`、artifact path、root 和 frame map 必须
同时匹配，不允许只改路径。

### 3.4 Assembly、Session 与 Deployment 的选择规则

Assembly 是第一个实际选择 Asset 文件的调用层：

- 所有现有 Assembly 引用从无版本 Asset 文件改为明确的
  `..._v2026_6_27_v1.yaml`；
- 当前五个直接选择 Hand2 Asset 的 Assembly 文件自身也补
  `_v2026_6_27_v1.yaml` 后缀，Session 随之更新明确路径；不保留无版本 alias；
- Hand2 独立 fixed/rotation qualification Assembly 建立 `v2026_6_27` 与 `v2026_8_3`
  两套带版本文件；
- NERO、D405、FR3 等复杂 Assembly 在阶段一只迁移为显式选择旧版，不建立可运行的新版业务入口。

最后一条是刻意的隔离边界：不能仅机械复制一个 `v2026_8_3` NERO/D405/FR3 Assembly，因为新版
根坐标可能改变 attachment 的实际几何含义。未完成相应装配验证前，伪造“对应新版 Assembly”比
缺少该入口更危险。

Session 必须同时显式选择：

1. 具体 Assembly；
2. 每个 Hand2 instance 的具体 Binding；
3. 如兼容桥仍需要，具体版本的 compatibility profile。

阶段一新增的 Description 运行入口矩阵：

```text
isaac_hand2_{left,right}_fixed_qualification_v2026_8_3_v1.yaml
isaac_hand2_{left,right}_collision_qualification_v2026_8_3_v1.yaml
```

在上述 fixed/collision 静态和动态 Gate 通过后，才允许新增两个同版 Glove 资格入口：

```text
isaac_hand2_{left,right}_glove_qualification_v2026_8_3_v1.yaml
```

Glove 入口必须由一个版本化 qualification manifest 同时钉住 SDK distribution、Description
Session、side、Glove serial、SDK user/calibration revision 和输出目录；禁止只靠 CLI 文本声称
版本匹配。SDK 对象继续留在 adapter/composition root，不进入五层 Session schema。

并保留对应旧版 comparator：

```text
isaac_hand2_{left,right}_fixed_qualification_v2026_6_27_v1.yaml
isaac_hand2_{left,right}_collision_qualification_v2026_6_27_v1.yaml
```

现有双 NERO Glove/ROS/D405/dataset Session 不复制新版，不接受 `--description-version` 之类的
运行时字符串切换；它们继续属于历史链。阶段一的 Glove 验收只使用上面的单 Hand2 新版
qualification Session。未来向上升级时，应新增明确的同版链 Session，而不是修改旧 Session
的引用。

业务 Deployment 不因本阶段复制新版；但 Glove qualification manifest 必须记录目标 SDK
解释器/environment receipt，因为 SDK 版本不属于 Description Session。

### 3.5 禁止隐式版本选择

- CLI 默认值必须指向一个带版本的 Session 常量；
- `--asset`、`--profile` 等兼容 override 若来自不同 upstream release，启动前失败；
- `SessionResolver` 的快照和报告必须包含 source name、tag、commit、Asset revision、Binding ID；
- qualification preflight 必须读取已安装 distribution 元数据并与 manifest 的
  `expected_sdk_version=2026.8.3` 比较，不匹配时在连接 Glove 或启动 Isaac 前失败；
- 不提供无版本 symlink、YAML alias 文件或“找不到新版则回退旧版”的逻辑；
- 不直接拼接 `third_party/src/wuji-description/...`，路径必须从已解析 Binding/SourceLock 获得；
- 测试对禁止模式做仓库级扫描，防止之后重新引入。

## 4. 目标调用链

静态与仿真资产链：

```text
显式 Session
  -> 显式 Assembly（选择带版本 Asset）
  -> 显式 Backend Binding（选择带版本 SourceLock + artifact + root/frame map）
  -> canonical Hand2 Profile（选择带版本 q20/limit/provenance）
  -> SessionResolver 闭合版本一致性
  -> compatibility adapter 只消费 resolved facts
  -> Isaac / MuJoCo loader
```

真人穿戴匹配链：

```text
Wuji Glove（显式 side + serial + SDK user/calibration）
  -> wuji-sdk==2026.8.3 hand_skeleton（21×3 m）
  -> Wuji SDK 2026.8.3 RetargetSession.for_hand(WujiHand2, side)
  -> canonical q20 + retarget_model/config provenance
  -> supervisor（freshness / limits / rate）
  -> 显式 Description v2026.8.3 Hand2 qualification Session
  -> Isaac q20 feedback + fingertip site + contact/visual metrics
```

preflight 必须先在不连接设备、不启动 Isaac 的条件下闭合 SDK、用户 URDF、Glove local binding
和 Description Session；通过后启动并验证 Isaac Stage，最后连接单侧 Glove。任何 mismatch 都不得
降级继续。Glove 输入与仿真反馈可以记录为有界 qualification trajectory，但不能标记为正式
dataset。

canonical 语义 frame 继续使用 `hand_base`。它由 Binding 映射到：

```text
v2026.6.27 -> l_base_link / r_base_link
v2026.8.3  -> l_wrist / r_wrist
```

Assembly attachment、wrist rig 或 rotation mount 不得看到上游根链接字面量，只能引用
`hand_base` 并由 Binding 解析。q20 canonical layout 若经静态核验确实没有变化，则继续保持现有
firmware order；不得因为 USD DOF 枚举顺序不同而修改 domain layout。

## 5. 实施工作包

### P0：冻结旧版语义与证据

1. 保存当前 SDK distribution/wheel receipt、source lock、Profile、Asset、Binding 和 resolved
   Session 快照；
2. 记录旧版左右 USD/MJCF/URDF 的 hash、root、关节顺序、限位、Drive 属性和 prim 数量；
3. 固化旧版 fixed-hand、rotation、MuJoCo 与双 NERO fast contract tests；
4. 对已有验证报告注明仍归属于 `SDK 2026.7.21 + Description v2026.6.27`，禁止事后重解释
   为新版结果；
5. 2026-08-10 左右 Glove 9000 帧报告仅冻结为历史 mismatch baseline：它们证明输入、retarget、
   supervisor 和旧仿真手响应，但不证明目标同版链。

Gate：在任何重命名或 resolver 修改前，旧版快照可重复生成。

### P1：建立双版本第三方来源

1. 把旧记录迁移为 `wuji-description-v2026-6-27`；
2. 恢复 `v2026.8.3` 精确 tag，记录 commit；
3. 仅锁定阶段一需要的 Hand2 URDF、MJCF、USD、mesh、指尖 mesh/site 依赖和 license；
4. 对分层 USD 递归解析所有 sublayer/reference/payload/texture 依赖；
5. 计算文件和目录树 hash，拒绝缺失、越界引用和 symlink；
6. 验证两个 checkout 可分别恢复和校验。

Gate：`verify_artifacts=True` 对两套 source 同时通过，且不存在无版本 source name。

### P1.5：建立 SDK 2026.8.3 隔离资格环境

状态：2026-08-10 已完成。

1. 获取官方 PyPI `wuji-sdk==2026.8.3` 的 CPython 3.12 / Linux x86_64 wheel；
2. 记录 wheel URL、文件名、SHA-256、platform tag 和 distribution metadata；
3. 从现有 Isaac 6.0.1 环境建立隔离资格环境，不在共享环境中原地升级；
4. 无设备验证 import、`SdkManager`、`HandModel.WujiHand2`、`RetargetSession.for_hand`、
   `hand_skeleton` 所需 API 和显式版本读回；
5. 使用固定 21×3 fixture 运行左右 retarget smoke，确认 `(20,)` finite q20、reset 和重复性；
6. 运行项目 adapter/architecture tests，确认 SDK API 类型没有越过 adapter 边界。

Gate：解释器、SDK、wheel hash 和 API receipt 全部闭合；任一 API/ABI 不兼容时停止，不进入 Isaac
或 Glove。

### P2：建立版本化配置对

1. 重命名无 upstream 后缀的旧 Asset 与 compatibility profile；
2. 为新版本建立左右 Profile、Asset、Isaac Binding 和右手 MuJoCo Binding；
3. 将新 root/frame、路径、source commit 和 hash 写入对应 Binding；
4. 用版本化 Asset `revision` 阻止旧 Asset 与新 Binding 交叉组合；
5. 更新所有旧 Assembly/Session 调用点，明确选择 `v2026.6.27`；
6. 创建左右手 × fixed/collision × old/new 的八个独立 comparator/qualification Session；
7. resolved snapshot 显式输出版本事实。

此外建立版本化 Glove qualification manifest；它同时引用 SDK 环境 receipt 与 Description
Session，不把 SDK 版本错误地写入 Asset/Binding。

Gate：以下错误组合必须 fail closed：

- old Asset + new Binding；
- new Asset + old Profile；
- new Binding + old source revision；
- new root + old frame map；
- 一次 Session 中左右手选择不同 Description release，除非 Session schema 未来显式声明
  `mixed_source_experiment=true`；阶段一不提供该声明。
- `SDK 2026.7.21 + Description v2026.8.3` 或
  `SDK 2026.8.3 + Description v2026.6.27` 被标记为 `current_matched_chain`；两者都必须拒绝。

### P3：消除工具影响面

当前已确认的旧版硬编码至少位于：

- `src/wujihand/runtime/session_compat.py`：固定 Asset/Binding/Assembly 路径和 `r_base_link`；
- `src/wujihand/adapters/simulation/hand2_rotation_mount.py`：固定 `r_base_link` 与旧 root topology；
- `src/wujihand/runtime/mujoco_table_config.py`：固定 `fr3v2_link8 -> r_base_link`；
- `src/wujihand/runtime/isaac_d405_wrist_rig.py`：固定左右 `{l,r}_base_link`；
- integration/contract tests：直接拼接旧 checkout 路径或断言旧 root。
- `pyproject.toml`、`uv.lock` 当前有意固定历史默认 `wuji-sdk 2026.7.21`；runner 帮助文本、unit
  fixtures 和 validation 文档不得把这一默认值误写为目标资格环境或唯一 SDK 事实。

处理原则：

1. compatibility bridge 按 resolved Session 的明确 allowlist/contract 验证，不按一个全局旧路径判断；
2. root、base body、fixed root joint 和 artifact 路径从 Binding/解析后的 USD topology 获得；
3. rotation mount config 显式接收 base/root-joint backend frame，禁止内部默认旧名字；
4. D405 只消除根链接硬编码并保持原 Session 锁旧版；本阶段不运行新版 D405 验证；
5. MuJoCo loader 支持新路径和 root，但只做模型加载/关节 contract smoke，不扩展场景需求；
6. 测试 fixture 由版本矩阵生成，避免复制两套测试逻辑；
7. retarget adapter 从 distribution metadata 记录真实 SDK 版本，qualification manifest 负责
   预期版本 Gate；禁止 CLI 伪造版本字符串；
8. 不增加全局环境变量或隐式进程状态选择版本。

Gate：代码与 source-coupled 配置中不再出现未被版本 fixture、历史依赖锁、旧版测试或明确
environment receipt 拥有的 `hand2_beta/body`、`wujihand.usd`、`r_base_link`、`l_base_link` 或
`2026.7.21` 硬编码。

### P4：静态模型差异与结构资格验证

对左右手、旧/新两版生成机器可读 diff：

| 类别 | 检查内容 |
|---|---|
| source | tag、commit、文件/树 hash、license、依赖闭包 |
| URDF | root、joint/link 名、关节类型、axis、origin、limit、mass/inertia、mesh path |
| MJCF | root body、joint/actuator 顺序、range、gain/bias、collision geom、exclude pair、site |
| USD | default prim、articulation root、fixed root joint、rigid body、Drive、mass/inertia、layer 依赖 |
| mesh | 左右手文件集合、单位、AABB、三角面数量、空 mesh、法向/拓扑异常 |
| collision | collider 数量、approximation、contact/rest offset、过滤关系、软指尖是否参与 |
| canonical contract | q20 名称、顺序、单位、限位、左右镜像和 firmware layout |

关节名称和限位只有在实际文件证明一致后才能复用现有 domain layout。若有任一差异，必须在
Profile/Binding 中显式表达，不能用 runtime index 猜测。

Gate：所有预期差异被分类为 `path/name/frame`、`geometry`、`physics` 或 `unexpected`；存在
`unexpected` 时停止动态验证。

### P5A：SDK 2026.8.3 静态 Retarget 资格验证

状态：2026-08-10 已完成，已纳入 matched-chain receipt 与后续 Isaac/record preflight。

不连接 Glove、不启动 Isaac，分别对左右手执行：

1. SDK distribution/API identity 与 wheel receipt 校验；
2. rest/open、五指独立、整手闭合和四组对指的固定 21×3 MediaPipe fixture；
3. 同一 fixture 在 reset 前后和独立新 Session 中重复求解；
4. 检查 q20 shape、finite、layout/limit、左右 side、retarget model/config ID；
5. 对比 SDK 2026.7.21 与 2026.8.3 的 q20 delta，但只有 2026.8.3 可进入目标链；
6. 把 SDK 2026.8.3 q20 输入 Description v2026.8.3 的离线 FK/site 计算，记录四组指尖距离，
   不把“人手关键点接触”预设为“机器人 mesh 必须接触”。

Gate：SDK 2026.8.3 左右输出稳定、有限、版本身份真实且与 Description v2026.8.3 q20 contract
兼容；不存在未经解释的 SDK API/layout 变化。

### P5B：Isaac 独立动态与碰撞资格验证

状态：加载、q20/feedback、视觉贴合和 `self_collision=false` 穿戴子集已通过；完整 visual/collider
AABB、球/方块 contact 与 self-collision off/on 数值比较已按 2.4 的决策延后，不再阻塞当前 record
qualification。

只使用独立 Hand2 Workcell，不加载 NERO、Glove、ROS、D405 或数据集 writer。

#### P5B.1 加载与 articulation

- 左右手分别加载；
- 验证 stage 单位、default prim、root/fixed joint、20 个驱动关节和 DOF 映射；
- `Play -> reset -> Play` 重复至少 10 次，prim/DOF 身份和 root 数量保持稳定；
- 检查分层 USD 的所有依赖在离线 checkout 中可解析；
- 记录空载静止 5 秒的 q20 漂移、速度、告警和 simulation exception。

#### P5B.2 q20 控制轨迹

使用相同 canonical q20 脚本驱动旧/新两版：

1. rest/open；
2. 每个关节独立小幅正负 step；
3. 五指闭合/张开；
4. thumb-index、thumb-middle、thumb-ring、thumb-pinky 对指候选；
5. 慢速闭合接触球体与方块。

每条轨迹记录 command、feedback、error、velocity、Drive target、contact event、physics step 和
版本身份。P5B 不接 retargeter，确保问题只来自模型、加载器、驱动或碰撞；retargeter 只在
P5A/P5C 出现。

#### P5B.3 驱动参数验证

官方 USD 原值作为只读 baseline。参数实验通过版本化 Isaac overlay profile 注入，不修改上游 USD：

- baseline 官方 `kp/kv`；
- conservative low/mid 两档，仅用于观察稳定性；
- 比较保持误差、step 最终误差、超调、稳定时间、峰值速度和接触振荡；
- 报告明确写 `simulation_only=true`、`hardware_reusable=false`。

阶段一目标是确认新版驱动可测量、无数值爆炸，并建立后续调参基线；不在没有真机系统辨识的
情况下选定“最终 Hand2 控制参数”。

#### P5B.4 碰撞与视觉轮廓验证

- 同时渲染 visual、collision 和 fingertip site overlay；
- 统计每个 link 的 visual AABB、collider AABB 和体积/边界偏差；
- 验证全部预期 collider 和官方 10 组 exclude，检查额外过滤是否出现；
- 对四组对指轨迹记录：首次碰撞时的 q20、指尖 site 距离、visual mesh 最短距离、collider
  最短距离和接触 link pair；
- 对球/方块记录接触前视觉间隙、接触点、法向、穿透和持续振荡；
- 对软指尖 mesh 明确报告 `visual_only/not_collision`；
- 不以截图主观判断通过，截图只作为数值报告的辅助证据。

Gate：能够明确回答碰撞是否早于视觉接触、偏差发生在哪些手指/link，以及新版相对旧版是改善、
退化还是没有实质变化。没有真机对照时，不评价真实指腹准确度。

### P5C：SDK + Description 同版真人穿戴 Glove 验收

P5A 已通过；P5B 的当前必要子集已通过，完整碰撞归因另行延后。独立正式验收只连接一侧 Glove、
加载同侧独立 Hand2；左右串行执行，不加载 NERO、Tracker、ROS 或 dataset writer。

#### P5C.0 轻量 matched-chain preflight

新增一个小型、可独立撤回的 preflight 模块和薄 CLI，不建设通用测试框架。它在连接 Glove 和启动
Isaac 前同时校验：

1. distribution/module/wheel receipt 均为 SDK 8.3，模块来自显式 overlay；
2. `SdkManager` 中存在唯一、非默认的目标用户，并显式选择该用户；禁止无声回落默认用户；
3. 左右校准 URDF 位于该用户的 SDK 管理目录，side、格式与 SHA-256 匹配；
4. local binding 中的 Glove serial、side 与校准模型匹配；连接后再校验设备报告的 handedness；
5. resolved Description Session 精确指向 v2026.8.3、对应 commit/hash、`{l,r}_wrist` 和 q20 layout；
6. 首帧 retarget model/config ID 真实来自 SDK 8.3；
7. Wuji Studio 不持有同一 Glove 的 command/subscription ownership。

preflight 产出不可变 receipt，并由 `user_id + side + URDF hash + SDK version` 生成真实
`calibration_id`；qualification composition 不再接受任意手写 provenance。可移植策略进入版本化
配置，用户 ID、设备 serial、绝对路径和 URDF hash 留在 ignored host-local binding 与输出 manifest。
用户 URDF 不复制进仓库。

原生入口与 ROS Glove source 复用同一验证器。进入后续 ROS 全链时，Glove source 进程负责验证并
加载用户 URDF，Isaac consumer 中的 retarget 进程另行验证 SDK 8.3 distribution；两个进程不得一侧
8.3、一侧 7.21。

#### P5C.1 具名用户穿戴离线链

状态：2026-08-10 已完成左右诊断采集；2026-08-11 matched-chain preflight 与左右 Isaac 穿戴复验
闭合，结果见 2.4。

每侧固定执行张开、五指独立、握拳/张开、四组对指与恢复张开。正式 runner 复用现有 input port、
retarget、supervisor 和 JSONL storage，只负责动作提示、有界记录、overwrite protection 与摘要；
不得把 `/tmp` 验证脚本演化成长期复杂测试系统。

逐帧记录 canonical skeleton、SDK q20、supervised q20、五个 Description fingertip site、
clamp/rate-limit/stale/confidence 和完整版本 provenance。产物必须标记
`qualification_only=true`、`dataset_eligible=false`。

#### P5C.2 Isaac 同版链与碰撞归因

状态：`self_collision=false` 的左右穿戴视觉与运动学验收已完成。操作者确认贴合问题解决；关闭
self-collision 带来的少量穿模已记录并接受为当前非阻塞限制。确定性的 off/on contact replay 延后到
专门碰撞质量需求，不作为本轮 8.3 qualification record 的通过条件。

1. 在 `self_collision=false` 下用具名用户 live 输入验收 skeleton→q20→Isaac feedback/site；
2. 保存 canonical observation 与 q20，禁止靠操作者第二次动作充当可比输入；
3. 对相同 q20 启用 `self_collision=true` replay，记录首次 contact、site 距离、visual surface
   距离、collider 距离和 feedback；
4. 同时显示 visual/collider/site overlay，但通过判断使用数值报告，截图只辅助定位；
5. 分离“校准骨架端点不等于皮肤表面”“SDK q20 目标残差”“Description 运动学/visual mesh”和
   “Isaac convex-hull collider 新增间隙”。

首轮经验门槛为：所有帧有限、无 reject/stale/持续 clamp；四组对指均有持续 q20/site 响应；最佳
持续窗口的 Hand2 site 距离不高于约 12 mm；robot-human 残差超过 5 mm 时告警。confidence 单独
分级，右手当前 22.8% `degraded` 不阻塞功能 Gate，但正式 dataset 资格不得忽略该字段。

Gate：左右五指语义正确、四组对指都有明确 q20/site/feedback 响应，报告能量化 convex hull 是否
使视觉间隙增加，并解释残余空隙属于哪一层。若关闭 self-collision 后仍出现超过门槛的系统性残差，
记录为 calibration/retarget/kinematics 限制，不用 drive 或 collider 调参掩盖。

### P6：旧版回归与影响关闭

1. 所有既有 Session/Deployment 的 resolved source 必须仍为 `v2026.6.27`；
2. 旧版 fixed-hand、rotation、MuJoCo、双 NERO contract tests 全部通过；
3. 旧版至少完成一次 Isaac fixed-hand smoke 和一次当前双 NERO qualification smoke；
4. 新版独立 Hand2 入口与显式版本化的双 NERO record qualification 入口可运行；任何新版录制均
   保持 qualification-only 且不得替换历史业务入口；
5. 普通测试和默认业务 Deployment 不会恢复、加载或选择新版 source；
6. 新版失败时删除其 Session/配置调用，不影响旧 source 恢复和旧入口运行；
7. 阶段一报告记录 SDK wheel/environment receipt、全部配置 hash、source commit、Isaac 版本和
   运行命令；
8. 目标 SDK 环境失败时可删除整个隔离环境并返回历史环境，不修改设备、历史报告或旧 source。
9. `pyproject.toml`、`uv.lock` 与旧业务 Deployment 在本阶段继续保持 SDK 7.21；任何 8.3
   qualification 必须由显式解释器/receipt 启动，不允许环境回落。

Gate：工具升级对旧入口行为为零功能变化；新版资格验证结果不能改变任何历史报告或数据集身份。

## 6. 验证矩阵

| 维度 | 历史链：SDK `2026.7.21` + Description `v2026.6.27` | 目标链：SDK `2026.8.3` + Description `v2026.8.3` |
|---|---|---|
| wheel/environment identity | 回退 receipt | 必测、hash/ABI/API 闭合 |
| device-free SDK retarget | 历史 comparator | 左右 fixture qualification |
| left URDF/Profile contract | 必测 | 必测 |
| right URDF/Profile contract | 必测 | 必测 |
| left Isaac USD load/hold/q20 | 必测 | 必测 |
| right Isaac USD load/hold/q20 | 必测 | 必测 |
| root/frame mapping | `{l,r}_base_link` | `{l,r}_wrist` |
| collision/site inventory | 必测 | 必测 |
| 四组对指轨迹 | comparator | qualification |
| 球/方块接触 | comparator | qualification |
| right MuJoCo model load | 回归 | 兼容 smoke |
| 真人 Glove | 9000 帧历史 mismatch baseline | Studio 8.3 具名用户左右校准 URDF 必测 |
| self-collision 对指归因 | 历史 live 关闭 | 当前保持关闭并记录轻微穿模；off/on replay 延后 |
| NERO/ROS/D405/dataset | 历史入口回归 | 并列 qualification-only 8.3 record 入口 |
| 真机 | 不做 | 不做 |

静态/Isaac comparator 使用相同 canonical skeleton/q20 输入和相同指标定义；真人动作不能直接作为
old/new 精确 comparator，必须先录为 SDK-independent canonical/q20 qualification trajectory，再做
确定性 replay。不同 SDK/Description 组合的报告、截图和 JSON 输出目录必须同时包含两个版本，
禁止覆盖。

## 7. 阶段一验收门槛

### G1：来源与配置闭合

- SDK 2026.8.3 wheel、hash、platform tag、解释器和 environment receipt 闭合；
- 两套 source 可同时离线恢复并通过 hash；
- 所有 source-coupled 文件名、Binding ID 和 Asset revision 有 upstream 版本；
- 不存在 `latest/current` 或无版本 fallback；
- Asset/Profile/Binding/source revision 交叉混用全部被拒绝；
- resolved/qualification report 能唯一回答实际使用的 SDK version/wheel、Description tag/commit、
  retarget model/config 和 artifact。

### G2：结构闭合

- SDK 2026.8.3 固定 fixture 左右 retarget 输出为 finite q20，并匹配对应 firmware layout；
- 左右手各 20 个预期关节，名称、顺序、限位和单位全部闭合；
- old root 和 new root 分别由 Binding 映射到 canonical `hand_base`；
- USD/MJCF/URDF 依赖完整，无丢失 sublayer、mesh 或纹理；
- 新版差异表不存在未解释的 `unexpected` 项。

### G3：动态稳定

- 所有轨迹无 NaN/Inf、无 articulation 丢失、无 physics exception；
- 10 次 reset 后 DOF/root 身份稳定；
- 5 秒空载保持不出现持续发散；
- q20 command/feedback、step 指标和版本身份完整落入报告；
- 参数 overlay 不修改上游 USD，不生成真机参数建议。

首轮不预设过严的 tracking 数值阈值。先由官方 baseline 和 conservative overlay 建立经验分布；
如果出现明显发散、持续高频振荡或关节越限，直接失败。具体稳定性数值门槛在第一次可重复 baseline
报告后冻结，并作为同一阶段的第二轮 qualification profile 提交。

### G4：碰撞结论可解释

状态：完整数值 Gate 延后。当前只要求确认 visual 贴合已改善、self-collision 关闭导致的少量穿模
可复现且已记录，不据此声称真实碰撞准确；下列原始数值项保留为后续碰撞质量需求。

- visual/collision/site 三层可同时检查；
- 四组对指和两类物体接触均有数值记录；
- 首次碰撞 link pair、q20 和 visual gap 可追溯；
- 官方 10 组 exclude 与实际 stage 一致；
- 报告明确区分“碰撞体一致性”“视觉贴合度”和“真实软体准确度未知”。

### G5：旧入口零功能影响

- 现有复杂 Session 仍显式选择 `v2026.6.27`；
- 当前 fast suite 与选定 smoke 全部通过；
- 新版只进入独立 Hand2 qualification 和显式并列的双 NERO qualification record，不替换历史
  NERO、ROS、D405、采集或训练入口；
- 历史 SDK 2026.7.21 环境/receipt 可回退，目标环境不会原地覆盖它；
- 历史 artifact、report 和 dataset checksum 不变。

### G6：SDK + Description 同版穿戴链闭合

- preflight 在设备连接和 Isaac 启动前确认 SDK `2026.8.3`、Description `v2026.8.3`；
- 报告中的 retarget model/config ID 真实来自 SDK，不由 CLI 手工填写；
- 左右 Glove identity、side、具名 SDK user、校准 URDF 路径/hash 与 calibration ID 明确；
- 五指独立动作、张合和四组对指均有 skeleton→q20→feedback→site 的逐帧证据；
- self-collision 保持关闭；由此产生的少量穿模必须记录，不能误写为 calibrated URDF 或新版
  Description 失败；off/on replay 属于后续碰撞质量需求；
- 两个 SDK 运行上下文若同时出现，必须都为 8.3，禁止 source/retarget 进程混版；
- 无 reject/stale/持续 clamp、无非有限值，所有产物明确不可进入正式 dataset。

## 8. 实施顺序与提交边界

建议拆为可独立回退的提交：

1. **旧版显式化**（已完成）：source lock 改为版本名，旧 Asset/compat profile 补版本后缀；
2. **双 source 能力**（已完成）：增加 `v2026.8.3` source record、恢复工具和 tests；
3. **新版叶配置**（已完成）：新增 Profile/Asset/Binding 和交叉版本拒绝测试；
4. **工具去硬编码**（已完成主体）：root/path 从 resolved Binding 获取，旧版回归；
5. **独立 qualification 入口**（已完成 fixed/collision）：新增 old/new Hand2-only Sessions；
6. **SDK 隔离环境与静态链**（已完成）：锁定 8.3 wheel/environment receipt，完成 API、fixture、
   offline FK 与具名用户穿戴诊断；
7. **轻量 preflight**（已完成）：新增用户模型/SDK/Description 闭合、真实 calibration ID 和
   fail-closed tests；
8. **Isaac 动态与碰撞验证**（当前范围已完成）：左右 feedback/visual/site 与穿戴观察通过；
   self-collision off/on contact replay 已明确延后；
9. **正式同版 Glove qualification**（已完成）：使用具名用户各侧复跑，保留 confidence 与已知
   self-collision-off 限制；
10. **影响关闭**（已完成代码回归，待最终 live record）：fast suite、旧入口 contract 与新版 stub
    全链通过；本阶段不切项目默认 SDK 锁；
11. **8.3 并列 record 全链**（实施中）：F1、F2 已完成；首次 live qualification 已证明双臂、双手
    与 record 主链可用，并暴露、修正 8.3 wrist 根坐标系装配朝向问题，仍需带 D405 的最终复录收口。

任何一步失败时，回退当前提交即可；不得用单次大提交同时改 source lock、复杂 Session 和运行参数。

## 9. 预计影响与难度

当前已知影响面：

- 2 个 Hand2 Profile；
- 2 个无 upstream 后缀的 Hand2 Asset；
- 4 个现有 Hand2 Binding；
- 2 个 source-coupled compatibility profile；
- 5 个直接选择 Hand2 Asset 的 Assembly；
- 多个 Session 对 Binding/Profile 的显式引用；
- Workstation2 SDK 8.3 overlay、版本化 environment receipt，以及后续默认依赖切换边界；
- Glove input/retarget adapter 的 SDK 版本 contract、fixtures 和报告 provenance；
- 一个轻量 matched-chain preflight、host-local 用户模型 binding 与正式 qualification runner；
- 至少 4 处 runtime/adapter 根链接或路径硬编码；
- 13 个左右 contract/unit/integration test 文件含旧路径、旧版本或旧 root 假设。

难度判断：

| 工作 | 难度 | 风险 |
|---|---|---|
| 双版本 source lock 与恢复 | 中 | hash、分层 USD 依赖闭包 |
| 配置重命名和显式引用 | 中 | 漏调用点、交叉版本组合 |
| root/path 去硬编码 | 中高 | compatibility runner 和 rotation mount topology |
| SDK 2026.8.3 wheel/ABI/API 升级 | 中 | CPython 3.12 wheel、二进制兼容、API/标定格式变化 |
| SDK 静态 retarget fixture | 中 | 内置模型改变、左右 layout/provenance 漂移 |
| 静态差异工具 | 中 | 多格式语义对齐 |
| Isaac 动态/碰撞 qualification | 中高 | 凸包、contact、官方 inherited gains |
| matched-chain preflight | 中 | SDK 全局用户状态、host-local URDF/hash、启动顺序 |
| 真人 Glove 同版验收 | 中 | 右手 confidence 警告、可重复动作、正式 runner 保持轻量 |
| 旧复杂入口回归 | 中 | 远端 Isaac 运行时间 |

截至 2026-08-11，SDK 8.3 API、静态链、具名用户左右标定、matched-chain preflight、Isaac 穿戴
复验和并列 record 的 device-free 严格全链均已证明可用，没有发生 retarget 大规模重写。首次操作者
live qualification 已完成并证明控制/写盘主链；剩余工作是使用已修正的 Hand2 朝向、带 D405 复录
一次最终 qualification 并完成离线完整性检查，预计约半日。项目默认 SDK 锁、历史 Deployment 和
正式 dataset 身份仍不切换。

## 10. 阶段一后的全链路拓展与阶段二边界

阶段一已交付一个 SDK/Description 同版、可选择、可测量、不会污染旧业务入口的新版 Hand2
数字孪生和 Glove→仿真手资格链。2026-08-11 已按原边界启动并基本完成独立的中等拓展需求，
把同版链扩展到双 NERO + 双 Hand2 仿真 record；它使用并列配置，不修改旧 Session，也不通过
运行时字符串切换版本。

### 10.1 双 NERO + 双 Hand2 8.3 record 拓展计划

#### F1：并列建立 8.3 业务配置

状态：已完成。

- 新建显式 v2026.8.3 双 NERO Assembly、Session 和 Deployment，保留全部 v2026.6.27 入口；
- 重新验证 `{l,r}_wrist` 根链接下的 NERO→Hand2 attachment，不复用旧 `{l,r}_base_link` 位姿假设；
- Glove local binding 使用 preflight 生成的具名用户 calibration ID；host-local 用户事实不进入
  portable Session；
- 不在一个 Session 中混用左右不同 Description release 或两个 SDK version。

#### F2：闭合原生与 ROS 两套运行环境

状态：已完成。原生 consumer smoke 与 ROS record-chain preflight 均要求 SDK 8.3；两个 SDK
process receipt 已闭合。

- 先运行 device-free q54 replay，证明新 Assembly 的 q7+q20 partition、feedback 和 scene reset；
- 原生路径显式使用 SDK 8.3 overlay；
- ROS 路径中 Glove source 进程使用 SDK 8.3 加载 Studio 用户 URDF，Isaac consumer 中的 retarget
  进程也必须使用 SDK 8.3；
- preflight receipt 同时记录两个解释器/module root/environment ID，任一为 7.21 都 fail closed；
- ROS source 继续只发布 canonical skeleton，retarget/supervisor 不下沉到 source node。

#### F3：逐层进入 live 与 record

状态：无设备 q54/scene/preview/rosbag/replay 严格全链已由
`hand2-83-stub-20260811-10` 零漏通过。首次操作者 live run
`hand2-83-live-20260811-01` 已完成：双臂、双手均有显著运动，q54 合成误差为 0，手部无 reject、
clamp 或 rate-limit，操作者确认对指贴合明显改善；该次没有 D405 topic，且使用了尚未补偿的 8.3
wrist 根朝向，因此只作为控制/record 诊断证据，不作为最终 qualification 或训练数据。

8.3 Description 将历史 `{l,r}_base_link` 根改为 `{l,r}_wrist`，并重新定义左右 wrist 坐标轴。
NERO 法兰的历史物理装配仍为 `link7 -> hand_base`、平移 `[0.023, 0, -0.0235] m` 与 `Ry(+90°)`；
新版 Assembly 额外按左右侧组合仅朝向的 Description-root 补偿，禁止改动该平移或把补偿回灌到
旧版 Assembly。修正后的 `hand2-83-orientation-stub-20260811-13/-14` 画面已恢复五指朝向工作区，且
q54/record/可见运动通过；严格时序仍分别出现 1 个 preview miss，以及 1 个 main + 1 个 preview
miss，属于调度门而非装配朝向失败，因此仍需最终 live 复录收口。

按以下顺序验收，前一层失败不得继续：

```text
device-free q54 replay
  -> 双 Glove、双 NERO 固定
  -> Glove + Trackers 双臂双手 live
  -> 短时 qualification record
  -> replay / q54 / 图像 / checksum / 时间对齐检查
```

短时 episode 必须标记 `qualification_only=true`、`dataset_eligible=false`；在 confidence、手指动作、
抓取意图、抖动和时序质量另行通过前，不进入正式训练集。现有旧数据集 checksum 与身份不变。

#### F4：默认依赖切换与影响关闭

状态：未执行，按设计保持延后。当前 live qualification 通过也不会自动修改默认依赖。

- 只有 F1–F3 全部通过后，才把 `pyproject.toml`、`uv.lock` 和正式 Workstation2 环境切到
  `wuji-sdk==2026.8.3`；
- 切换与新版 Deployment/环境 receipt 放在独立提交，禁止在 preflight 或单 Hand2 提交中提前完成；
- 历史 7.21 wheel/receipt 与 v2026.6.27 Session 继续可恢复，但不得由新版默认入口隐式选择；
- 最终重新运行旧入口 contract tests、新版 native/ROS smoke、record/replay 和数据完整性检查。

### 10.2 Hand2 真机阶段二

下列内容必须另立阶段二中等需求：

- 左右真机 identity、固件、零位和 diagnostics 只读盘点；
- 真机低 effort/低增益单关节 bring-up；
- simulation drive 与 hardware MIT 参数的强解耦；
- ROS 2 canonical q20 复用、hardware executor 和 safety state machine；
- 同轨迹 Sim/Real q20、指尖和接触位置对照；
- Glove retarget 驱动 Hand2 真机后的真实对指与接触验证。

阶段二不得在阶段一 G1–G6 未通过时开始向真机发送关节命令。

## 11. Definition of Done

阶段一只有同时满足以下条件才完成：

1. old/new source、配置和恢复目录并存且 hash 闭合；
2. SDK 2026.8.3 wheel/hash/API/overlay environment receipt 闭合；项目默认依赖切换明确延后；
3. 全部调用点显式选择具体版本，SDK/Description 交叉版本身份 fail closed；
4. 旧复杂入口仍为历史链且回归通过，历史左右 Glove 报告没有被重解释；
5. 新 `SDK 2026.8.3 + Description v2026.8.3` 存在 Hand2 独立 qualification 与显式并列的
   qualification-only record 入口，二者都不能隐式替换历史入口；
6. 左右手新版结构、静态 retarget、q20、驱动、site 和对指报告完整；完整碰撞数值报告按 G4
   明确延后；
7. 当前只记录视觉贴合改善和 self-collision-off 穿模限制，不把操作者观察冒充碰撞或真机数值；
8. 左右真人穿戴 Glove 的 skeleton→SDK q20→Isaac feedback/site 链通过；self-collision 关闭及
   其少量穿模已明确记录，off/on replay 延后；
9. Studio 8.3 具名用户左右校准 URDF、hash 与正式复验闭合，残余限制已按层归档；
10. 所有仿真控制参数明确标记为 simulation-only；阶段一提交没有 Hand2/NERO 真机、Tracker、
    ROS、D405、正式采集或训练功能混入；
11. 后继全链拓展只有在双 Glove/Tracker live qualification record、离线完整性和回放检查通过后
    才完成；在此之前始终 `dataset_eligible=false`，且不切默认 SDK 锁。
