# 013：Wuji Description v2026.8.3 / Hand2 Beta1 阶段一隔离升级计划

- 状态：实施中；双版本来源、版本化配置及 2026-08-10 局部结构/动态验收已完成，完整
  P5/P6 与 G1–G5 尚未完成
- 日期：2026-08-10
- 当前基线：`wuji-description v2026.6.27`
  (`aee64892ebcf8e3237bedc30231bb09476cbc71d`)
- 目标来源：`wuji-description v2026.8.3`；其中 Hand2 Beta1 模型的实质修订来自
  `v2026.7.23`
- 范围：第三方资产双版本共存、版本化配置、加载/组合工具适配、Hand2 独立仿真资格验证
- 后继需求：Hand2 Beta1 左右真机 bring-up、ROS 2 复用与 Sim/Real 对照，另立中等需求

## 0. 结论

阶段一不是把仓库中的 `wuji-description` 目录直接升级到最新，也不把现有 NERO、Glove、
ROS 2、D405 或数据集入口切到新版 Hand2。

本阶段采用以下策略：

1. `v2026.6.27` 与 `v2026.8.3` 两套第三方来源、配置和本地恢复目录同时存在；
2. 所有拥有 Wuji Description 版本事实的文件名均带明确版本后缀；
3. 所有调用方通过配置文件显式选择具体版本，禁止 `latest`、`current`、无版本别名和运行时
   自动择新；
4. 现有复杂 Session 继续显式锁定 `v2026.6.27`，阶段一只开放独立 Hand2
   `v2026.8.3` qualification Session；
5. 先消除加载器、resolver、compatibility bridge 和仿真 adapter 中的旧路径/旧根链接硬编码，
   再进行新版结构、碰撞与驱动验证；
6. 阶段一通过只表示新版数字孪生资产可复现、可选择、可测量，不表示与真机一致，也不授权
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

### 1.2 必须长期保留的警告

> **警告：Wuji Hand2 仍是 Beta1。`v2026.7.23` 是一次包含坐标、根链接、目录和碰撞语义的
> 高风险修订；后续官方仍可能更新质量参数和几何细节。任何 `latest` 文档或新 release 都不得
> 自动替换本计划锁定的 source、配置或验证结论。**

此外：

- 当前 USD `kp/kv` 沿用上一代 Wuji Hand 平台，尚未基于 Hand2 真机系统辨识；
- 官方没有提供带软体的 Hand2 仿真模型；
- 新版仍使用凸包碰撞，不能预设它已经解决视觉表面与碰撞轮廓偏移；
- 阶段一的 drive 参数只属于 Isaac 仿真，不得复用为真机 MIT `kp/kd` 或
  `effort_limit`。

官方依据：

- [Wuji Description 发布记录](https://docs.wuji.tech/docs/zh/wuji-description/latest/release-notes/)
- [Wuji Description 集成指南](https://docs.wuji.tech/docs/zh/wuji-description/latest/integration/)
- [Wuji Hand 2 使用约束](https://docs.wuji.tech/docs/zh/wuji-hand/latest/usage-constraints/)

## 2. 阶段一范围

### 2.1 本阶段必须完成

- 恢复并校验两个固定 Wuji Description source tree；
- 建立左右手旧/新两套 Profile、Asset、Isaac Binding；
- 保留当前右手 MuJoCo Binding，并建立对应新版结构加载能力；
- 把 Hand2 source 版本变成显式、可解析、可报告的配置事实；
- 消除工具中对旧目录、`wujihand.usd` 和 `{l,r}_base_link` 的隐式依赖；
- 建立旧/新版本静态差异报告；
- 在 Isaac Sim 6.0.1 中完成左右手独立加载、关节、驱动、碰撞、指尖 site 和对指轨迹验证；
- 证明现有复杂入口仍解析到 `v2026.6.27`，没有被新版污染；
- 产出阶段一 qualification 报告和可重复命令。

### 2.2 本阶段明确不做

- 不连接、发现、使能或控制 Hand2 真机；
- 不更新 Hand2 固件、Wuji SDK、Wuji Studio 或 Wuji Retargeting；
- 不调试 Wuji Glove、个性化手型标定或 teleop；
- 不切换双 NERO、T 型架、RoboLab、D405、ROS 2 或数据集 Session 到新版；
- 不以新版重新采集 episode，不修改 Pi0.5 训练链；
- 不做仿真参数到真机参数的映射；
- 不宣称凸包碰撞与真实软体指腹一致。

现有本地 `plans/hand2-real-digital-twin-ros2-reuse-decoupling.md` 只作为阶段二参考，不是本计划
依赖，也不在本阶段更新。

## 3. 版本身份与文件命名规则

### 3.1 三类版本不得混用

| 版本类型 | 示例 | 含义 |
|---|---|---|
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

阶段一新增且仅新增以下新版运行入口矩阵：

```text
isaac_hand2_{left,right}_fixed_qualification_v2026_8_3_v1.yaml
isaac_hand2_{left,right}_collision_qualification_v2026_8_3_v1.yaml
```

并保留对应旧版 comparator：

```text
isaac_hand2_{left,right}_fixed_qualification_v2026_6_27_v1.yaml
isaac_hand2_{left,right}_collision_qualification_v2026_6_27_v1.yaml
```

现有 NERO/Glove/ROS/D405/dataset Session 不复制新版，不接受 `--description-version` 之类的运行时
字符串切换；它们在 YAML 内继续锁定 `v2026.6.27`。未来向上升级时，应新增明确的新版 Session，
而不是修改旧 Session 的引用。

Deployment 不拥有第三方资产版本，只引用 Session；因此不因本阶段复制新版 Deployment。

### 3.5 禁止隐式版本选择

- CLI 默认值必须指向一个带版本的 Session 常量；
- `--asset`、`--profile` 等兼容 override 若来自不同 upstream release，启动前失败；
- `SessionResolver` 的快照和报告必须包含 source name、tag、commit、Asset revision、Binding ID；
- 不提供无版本 symlink、YAML alias 文件或“找不到新版则回退旧版”的逻辑；
- 不直接拼接 `third_party/src/wuji-description/...`，路径必须从已解析 Binding/SourceLock 获得；
- 测试对禁止模式做仓库级扫描，防止之后重新引入。

## 4. 目标调用链

```text
显式 Session
  -> 显式 Assembly（选择带版本 Asset）
  -> 显式 Backend Binding（选择带版本 SourceLock + artifact + root/frame map）
  -> canonical Hand2 Profile（选择带版本 q20/limit/provenance）
  -> SessionResolver 闭合版本一致性
  -> compatibility adapter 只消费 resolved facts
  -> Isaac / MuJoCo loader
```

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

1. 保存当前 source lock、Profile、Asset、Binding 和 resolved Session 快照；
2. 记录旧版左右 USD/MJCF/URDF 的 hash、root、关节顺序、限位、Drive 属性和 prim 数量；
3. 固化旧版 fixed-hand、rotation、MuJoCo 与双 NERO fast contract tests；
4. 对已有验证报告注明仍归属于 `v2026.6.27`，禁止事后重解释为新版结果。

Gate：在任何重命名或 resolver 修改前，旧版快照可重复生成。

### P1：建立双版本第三方来源

1. 把旧记录迁移为 `wuji-description-v2026-6-27`；
2. 恢复 `v2026.8.3` 精确 tag，记录 commit；
3. 仅锁定阶段一需要的 Hand2 URDF、MJCF、USD、mesh、指尖 mesh/site 依赖和 license；
4. 对分层 USD 递归解析所有 sublayer/reference/payload/texture 依赖；
5. 计算文件和目录树 hash，拒绝缺失、越界引用和 symlink；
6. 验证两个 checkout 可分别恢复和校验。

Gate：`verify_artifacts=True` 对两套 source 同时通过，且不存在无版本 source name。

### P2：建立版本化配置对

1. 重命名无 upstream 后缀的旧 Asset 与 compatibility profile；
2. 为新版本建立左右 Profile、Asset、Isaac Binding 和右手 MuJoCo Binding；
3. 将新 root/frame、路径、source commit 和 hash 写入对应 Binding；
4. 用版本化 Asset `revision` 阻止旧 Asset 与新 Binding 交叉组合；
5. 更新所有旧 Assembly/Session 调用点，明确选择 `v2026.6.27`；
6. 创建左右手 × fixed/collision × old/new 的八个独立 comparator/qualification Session；
7. resolved snapshot 显式输出版本事实。

Gate：以下错误组合必须 fail closed：

- old Asset + new Binding；
- new Asset + old Profile；
- new Binding + old source revision；
- new root + old frame map；
- 一次 Session 中左右手选择不同 Description release，除非 Session schema 未来显式声明
  `mixed_source_experiment=true`；阶段一不提供该声明。

### P3：消除工具影响面

当前已确认的旧版硬编码至少位于：

- `src/wujihand/runtime/session_compat.py`：固定 Asset/Binding/Assembly 路径和 `r_base_link`；
- `src/wujihand/adapters/simulation/hand2_rotation_mount.py`：固定 `r_base_link` 与旧 root topology；
- `src/wujihand/runtime/mujoco_table_config.py`：固定 `fr3v2_link8 -> r_base_link`；
- `src/wujihand/runtime/isaac_d405_wrist_rig.py`：固定左右 `{l,r}_base_link`；
- integration/contract tests：直接拼接旧 checkout 路径或断言旧 root。

处理原则：

1. compatibility bridge 按 resolved Session 的明确 allowlist/contract 验证，不按一个全局旧路径判断；
2. root、base body、fixed root joint 和 artifact 路径从 Binding/解析后的 USD topology 获得；
3. rotation mount config 显式接收 base/root-joint backend frame，禁止内部默认旧名字；
4. D405 只消除根链接硬编码并保持原 Session 锁旧版；本阶段不运行新版 D405 验证；
5. MuJoCo loader 支持新路径和 root，但只做模型加载/关节 contract smoke，不扩展场景需求；
6. 测试 fixture 由版本矩阵生成，避免复制两套测试逻辑；
7. 不增加全局环境变量或隐式进程状态选择版本。

Gate：代码与 source-coupled 配置中不再出现未被版本 fixture/旧版测试明确拥有的
`hand2_beta/body`、`wujihand.usd`、`r_base_link`、`l_base_link` 硬编码。

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

### P5：Isaac 独立动态与碰撞资格验证

只使用独立 Hand2 Workcell，不加载 NERO、Glove、ROS、D405 或数据集 writer。

#### P5.1 加载与 articulation

- 左右手分别加载；
- 验证 stage 单位、default prim、root/fixed joint、20 个驱动关节和 DOF 映射；
- `Play -> reset -> Play` 重复至少 10 次，prim/DOF 身份和 root 数量保持稳定；
- 检查分层 USD 的所有依赖在离线 checkout 中可解析；
- 记录空载静止 5 秒的 q20 漂移、速度、告警和 simulation exception。

#### P5.2 q20 控制轨迹

使用相同 canonical q20 脚本驱动旧/新两版：

1. rest/open；
2. 每个关节独立小幅正负 step；
3. 五指闭合/张开；
4. thumb-index、thumb-middle、thumb-ring、thumb-pinky 对指候选；
5. 慢速闭合接触球体与方块。

每条轨迹记录 command、feedback、error、velocity、Drive target、contact event、physics step 和
版本身份。阶段一不接 retargeter，确保问题只来自模型、加载器、驱动或碰撞。

#### P5.3 驱动参数验证

官方 USD 原值作为只读 baseline。参数实验通过版本化 Isaac overlay profile 注入，不修改上游 USD：

- baseline 官方 `kp/kv`；
- conservative low/mid 两档，仅用于观察稳定性；
- 比较保持误差、step 最终误差、超调、稳定时间、峰值速度和接触振荡；
- 报告明确写 `simulation_only=true`、`hardware_reusable=false`。

阶段一目标是确认新版驱动可测量、无数值爆炸，并建立后续调参基线；不在没有真机系统辨识的
情况下选定“最终 Hand2 控制参数”。

#### P5.4 碰撞与视觉轮廓验证

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

### P6：旧版回归与影响关闭

1. 所有既有 Session/Deployment 的 resolved source 必须仍为 `v2026.6.27`；
2. 旧版 fixed-hand、rotation、MuJoCo、双 NERO contract tests 全部通过；
3. 旧版至少完成一次 Isaac fixed-hand smoke 和一次当前双 NERO qualification smoke；
4. 新版只有左右手 fixed/collision 四个独立 Hand2 qualification 入口可运行；
5. 普通测试和默认 Deployment 不会恢复、加载或选择新版 source；
6. 新版失败时删除其 Session/配置调用，不影响旧 source 恢复和旧入口运行；
7. 阶段一报告记录全部配置 hash、source commit、Isaac 版本和运行命令。

Gate：工具升级对旧入口行为为零功能变化；新版资格验证结果不能改变任何历史报告或数据集身份。

## 6. 验证矩阵

| 维度 | `v2026.6.27` | `v2026.8.3` |
|---|---|---|
| left URDF/Profile contract | 必测 | 必测 |
| right URDF/Profile contract | 必测 | 必测 |
| left Isaac USD load/hold/q20 | 必测 | 必测 |
| right Isaac USD load/hold/q20 | 必测 | 必测 |
| root/frame mapping | `{l,r}_base_link` | `{l,r}_wrist` |
| collision/site inventory | 必测 | 必测 |
| 四组对指轨迹 | comparator | qualification |
| 球/方块接触 | comparator | qualification |
| right MuJoCo model load | 回归 | 兼容 smoke |
| NERO/ROS/D405/dataset | 旧入口回归 | 本阶段禁止入口 |
| 真机 | 不做 | 不做 |

所有矩阵行使用相同 canonical q20 输入和相同指标定义。不同源版本的报告、截图和 JSON 输出目录
必须包含版本后缀，禁止覆盖。

## 7. 阶段一验收门槛

### G1：来源与配置闭合

- 两套 source 可同时离线恢复并通过 hash；
- 所有 source-coupled 文件名、Binding ID 和 Asset revision 有 upstream 版本；
- 不存在 `latest/current` 或无版本 fallback；
- Asset/Profile/Binding/source revision 交叉混用全部被拒绝；
- resolved report 能唯一回答实际加载的是哪个 tag、commit 和 artifact。

### G2：结构闭合

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

- visual/collision/site 三层可同时检查；
- 四组对指和两类物体接触均有数值记录；
- 首次碰撞 link pair、q20 和 visual gap 可追溯；
- 官方 10 组 exclude 与实际 stage 一致；
- 报告明确区分“碰撞体一致性”“视觉贴合度”和“真实软体准确度未知”。

### G5：旧入口零功能影响

- 现有复杂 Session 仍显式选择 `v2026.6.27`；
- 当前 fast suite 与选定 smoke 全部通过；
- 新版没有进入 NERO、Glove、ROS、D405、采集或训练入口；
- 历史 artifact、report 和 dataset checksum 不变。

## 8. 实施顺序与提交边界

建议拆为可独立回退的提交：

1. **旧版显式化**：source lock 改为版本名，旧 Asset/compat profile 补版本后缀，更新调用方；
2. **双 source 能力**：增加 `v2026.8.3` source record、恢复工具和 source-lock tests；
3. **新版叶配置**：新增 Profile/Asset/Binding 和交叉版本拒绝测试；
4. **工具去硬编码**：root/path 从 resolved Binding 获取，旧版全回归；
5. **独立 qualification 入口**：新增 old/new Hand2-only Sessions；
6. **静态与 Isaac 验证**：生成 diff、动态、碰撞报告；
7. **影响关闭**：运行旧入口 smoke，更新正式 validation 文档。

任何一步失败时，回退当前提交即可；不得用单次大提交同时改 source lock、复杂 Session 和运行参数。

## 9. 预计影响与难度

当前已知影响面：

- 2 个 Hand2 Profile；
- 2 个无 upstream 后缀的 Hand2 Asset；
- 4 个现有 Hand2 Binding；
- 2 个 source-coupled compatibility profile；
- 5 个直接选择 Hand2 Asset 的 Assembly；
- 多个 Session 对 Binding/Profile 的显式引用；
- 至少 4 处 runtime/adapter 根链接或路径硬编码；
- 13 个左右 contract/unit/integration test 文件含旧路径、旧版本或旧 root 假设。

难度判断：

| 工作 | 难度 | 风险 |
|---|---|---|
| 双版本 source lock 与恢复 | 中 | hash、分层 USD 依赖闭包 |
| 配置重命名和显式引用 | 中 | 漏调用点、交叉版本组合 |
| root/path 去硬编码 | 中高 | compatibility runner 和 rotation mount topology |
| 静态差异工具 | 中 | 多格式语义对齐 |
| Isaac 动态/碰撞 qualification | 中高 | 凸包、contact、官方 inherited gains |
| 旧复杂入口回归 | 中 | 远端 Isaac 运行时间 |

正常预计 4–7 人日；如果新版分层 USD、root joint topology 或 collision API 与当前 adapter 假设差异
较大，预留到 7–10 人日。代码改动预计集中在 resolver/compatibility/adapter 与测试，不应扩散到
retarget、ROS、数据采集或训练模块。

## 10. 与阶段二的边界

阶段一完成后，只交付一个可选择、可测量、不会污染旧入口的新版 Hand2 数字孪生底座。
本阶段验收通过后，还需在后续需求中扩展到双 NERO + 双 Hand2 仿真 record 录制全链路，完成
控制、碰撞、录制、回放与数据产物的回归检查验收。

下列内容必须另立阶段二中等需求：

- 左右真机 identity、固件、零位和 diagnostics 只读盘点；
- 真机低 effort/低增益单关节 bring-up；
- simulation drive 与 hardware MIT 参数的强解耦；
- ROS 2 canonical q20 复用、hardware executor 和 safety state machine；
- 同轨迹 Sim/Real q20、指尖和接触位置对照；
- 最终 Glove retarget 与真实对指验证。

阶段二不得在阶段一 G1–G5 未通过时开始向真机发送关节命令。

## 11. Definition of Done

阶段一只有同时满足以下条件才完成：

1. old/new source、配置和恢复目录并存且 hash 闭合；
2. 全部调用点显式选择具体版本，交叉版本 fail closed；
3. 旧复杂入口仍为 `v2026.6.27` 且回归通过；
4. 新 `v2026.8.3` 只存在 Hand2 独立 qualification 入口；
5. 左右手新版结构、q20、驱动、碰撞、site 和对指报告完整；
6. 已明确新版碰撞相对旧版的改善、退化或无变化，不用截图代替数值；
7. 所有仿真控制参数明确标记为 simulation-only；
8. 没有真机、Glove、NERO、ROS、D405、采集或训练功能混入本阶段提交。
