# MuJoCo FR3 v2—Wuji Hand 2 桌面环境

状态：长边侧置四棱台版本于 2026-07-14 完成验证；2026-07-23 接入五层 Session 组合。

本组件在 MuJoCo 中组合一台固定于桌面长边外侧四棱台、面向桌面中央的 Franka Research 3 v2，以及刚接在末端法兰上的 Wuji Hand 2 Beta 1 right。它提供确定性 reset、分离的 `arm_q7` / `hand_q20` 位置目标、状态观测、headless runner、GUI viewer 和离屏渲染入口。

## 固定版本

| 项目 | 固定值 |
|---|---|
| Python | `3.11.*` |
| MuJoCo | `3.10.0` |
| FR3 v2 | `mujoco_menagerie@71f066ad0be9cd271f7ed58c030243ef157af9f4` |
| FR3 MJCF SHA-256 | `75e0f526e503614680cad07a6f4b48de00e7783db7440d55e22d6686d96f9877` |
| FR3 mesh tree SHA-256 | `0ee1f659bb749fb88c1c2cca93215ec42460de9626eab31018bbc7bf1c88d7bf` |
| Hand 2 | `wuji-description v2026.6.27@aee64892ebcf8e3237bedc30231bb09476cbc71d` |
| Hand 2 right MJCF SHA-256 | `d29651a7d68d1a6d5ba813edfa10b46b26748cf99430c9752defe8583c7c1001` |
| Hand 2 right mesh tree SHA-256 | `4f1a7e96cafb13403ed82c5ef2f18d52a40afb49776ce56ee8f2224280ffcc13` |
| 组合自由度 | `7 arm + 20 hand = 27`，无 free joint |

Wuji 官方明确提供 Hand 2 Beta 1 的左右手 MJCF 和供用户继续适配目标机械臂孔位的 STEP 挂载件；这不构成“Wuji 官方认证 FR3 组合”。模型恢复路径和复合 license 原文均锁定在 `third_party/sources.lock.yaml`。

## 场景和坐标

世界为右手系，`+Z` 向上。桌面长轴沿 `X`，尺寸为 `1.60 × 1.00 × 0.06 m`，面积 `1.60 m²`，上表面高度 `0.75 m`；因此 `y_min/y_max` 是两条 `1.60 m` 长边。

FR3 位于 `y_max` 长边外侧的凸四棱台上。台座底面与桌投影相隔 `0.02 m`，中心 `[0, 0.82] m`，高 `0.47 m`，顶面 `0.42 × 0.42 m`、底面 `0.60 × 0.60 m`。FR3 base 位于 `[0, 0.82, 0.47] m`，绕 `Z` 轴旋转 `-90°`，使机械臂 local `+X` 对准 world `-Y` 和桌心，同时强制 local `+Z` 保持竖直。编译契约还要求 link0 collision footprint 完整落在台座顶面内。锁定 MJCF 的 joint2 轴心比 base 高 `0.333 m`；编译态实际 `z=0.803 m`，高出桌面 `0.053 m`。

```text
world
├── floor
├── tabletop + four fixed legs
├── arm_pedestal (fixed convex frustum beside y_max)
└── FR3 base (fixed on pedestal top, facing table center)
    └── fr3v2_link0 ... fr3v2_link8
        └── fr3_flange_to_hand2 (fixed MjSpec frame)
            └── r_base_link
                └── Hand 2 finger20
```

四棱台由 8 个顶点和 12 个三角面程序化生成，不新增外部 mesh 资产或自由度。使用 FR3 上游 `home=[0, 0, 0, -1.57079, 0, 1.57079, -0.7853]` 时，法兰位于桌面范围上方，Hand 2 的 `+Z` 延伸方向朝下。Hand 2 reset 为 canonical q20 全零张手位。

场景禁用 MuJoCo 默认 camera headlight，只保留一盏 `observation_light`：directional、无阴影、低高光，只服务 GUI/EGL 观察，不影响动力学。overview 相机从台座侧斜视，确保桌、台座、FR3 和 Hand 2 同时入镜。

## 组合与控制

运行配置从
`configs/sessions/mujoco_fr3v2_hand2_right_table_v1.yaml` 开始：Asset Manifest
声明 FR3 v2 与 Hand 2 Beta 1 right 的稳定身份，两个 MuJoCo Binding 选择固定 MJCF
及 frame/joint/actuator 映射，Assembly 用语义 `tool_flange -> hand_base` 表达 identity
attachment，Session 再把 Assembly root 放到 Workcell mount。

完整桌面、四棱台、灯光、相机和 physics 数值仍由
`configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml` 单点持有。当前 Workcell 只声明
语义 mount 并引用这份 typed compatibility profile；兼容桥在构建 adapter 前对照
Binding artifact/hash 和 Assembly attachment，不在五层文件中复制既有物理数值。

运行时分别解析两份只读 MJCF，通过 `mujoco.MjSpec` 在内存中把 `r_base_link` 附着到 `fr3v2_link8` 下的新 frame。当前不加名称前缀，因为两份固定资产不存在重名，且必须保留 Hand 2 的 canonical q20 名称；组合模型不会导出为 XML，因为不同 asset root 的 attached spec 不能可靠 round-trip。

场景在 attach 前把父、子 spec 的全局 option 统一覆盖为：

```text
timestep=0.002 s
integrator=implicitfast
solver=Newton
jacobian=sparse
iterations=100
tolerance=1e-8
control=100 Hz × 5 physics substeps
```

adapter 不依赖 `qpos[:7]` 或 `ctrl[7:]` 的偶然顺序。每个关节都经过：

```text
canonical joint name
  -> joint id
  -> qpos / dof address
  -> unique direct-joint actuator id
```

目标必须 shape 正确、finite 且在固定 joint range 内；超限直接拒绝，不静默 clamp。现有 `HandCommand v2` 保持 Isaac 的 q20 + 固定 XYZ D6 姿态语义，未被扩展为 27 维。未来机械臂失联策略也不能直接复用会自动扫回 rest 的手指 supervisor；arm 默认应 hold，再由独立 stop policy 决定后续动作。

## 主要入口

| 职责 | 入口 |
|---|---|
| 五层 composition root | `configs/sessions/mujoco_fr3v2_hand2_right_table_v1.yaml` |
| Asset Manifest | `configs/assets/franka_fr3_v2_v1.yaml`、`configs/assets/wuji_hand2_beta1_right_v1.yaml` |
| MuJoCo Binding | `configs/bindings/mujoco/franka_fr3_v2_menagerie_71f066a_v1.yaml`、`configs/bindings/mujoco/wuji_hand2_beta1_right_v2026_6_27_v1.yaml` |
| Assembly / Workcell | `configs/assemblies/fr3v2_hand2_right_identity_v1.yaml`、`configs/workcells/mujoco_long_edge_table_pedestal_v1.yaml` |
| typed 场景兼容叶 | `configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml` |
| FR3 模型契约 | `configs/profiles/fr3_v2_menagerie_71f066a.yaml` |
| Hand 2 模型契约 | `configs/profiles/hand2_right_v2026_6_27.yaml` |
| 五层解析与兼容桥 | `src/wujihand/runtime/session_resolver.py`、`src/wujihand/runtime/session_compat.py` |
| typed 场景严格加载 | `src/wujihand/runtime/mujoco_table_config.py` |
| FR3 profile 加载 | `src/wujihand/adapters/simulation/fr3_model.py` |
| MjSpec 组合、控制和观测 | `src/wujihand/adapters/simulation/mujoco_fr3_hand2.py` |
| headless / GUI runner | `tools/run_mujoco_fr3_hand2_table.py` |
| 上游恢复和 hash | `third_party/sources.lock.yaml` |

`MujocoFr3Hand2.observe()` 返回 arm/hand 的 q、dq，法兰和掌根的世界位姿、五个指尖 site 的世界坐标、接触数和仿真时间。指尖 site 只是运动学标记，不是触觉传感器。

runner 的 `--session` 是首选入口；省略时仍选择上述默认 Session，因此既有命令不增加
必填参数。保留的 `--scene-profile` 是显式 compatibility override，会进入
`session_hash`，并且必须继续通过五层 Binding/Assembly 一致性检查。JSON 报告新增
`session` 和 `session_hash`，其余既有字段与退出语义保持不变。

## 能力边界

- `T_fr3v2_link8__r_base_link` 当前为 identity，只是已验证的仿真假设。官方 STEP 未给出该数值变换；真实装配前必须根据转接件 CAD、质量、惯量、线缆和孔位重新标定。
- FR3 v2 Menagerie 模型来自当前提交，但成熟度低于历史更久的 FR3/Panda 模型；若实机确定为后续 v2.1/v2.2，须重新核对 `franka_description`。
- Hand 2 MJCF 是刚性骨骼模型。上游碰撞过滤会禁用 proximal-flex 段的外部碰撞，并让 proximal-abd 使用 bit 2；本组件保留原样，没有伪造“整只手完整接触”。
- 没有 IK、OSC、末端位姿命令、任务对象、奖励、遥操作、采集、ROS2、FCI/libfranka 或真机安全链路。
- MuJoCo position actuator 是仿真控制契约，不是 FR3 实机的速度、力矩、急停或认证安全契约。

运行方法见 [MuJoCo FR3—Hand 2 桌面指南](../guides/mujoco-fr3-hand2-table.md)，
五层职责见 [五层 Session 组合](five-layer-session-composition.md)，组合设计取舍见
[ADR-0002](../decisions/0002-mujoco-fr3v2-hand2-composition.md) 与
[ADR-0003](../decisions/0003-five-layer-session-composition.md)，初版模型证据见
[2026-07-13 验证报告](../validation/2026-07-13-mujoco-fr3-hand2-table.md)，最新布局
证据见
[2026-07-14 验证报告](../validation/2026-07-14-mujoco-fr3-hand2-table-layout.md)，
五层接线后的实际资产与 runner 回归见
[2026-07-23 验证报告](../validation/2026-07-23-five-layer-architecture.md)。
