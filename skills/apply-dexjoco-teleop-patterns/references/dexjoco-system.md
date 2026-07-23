# DexJoCo arm + hand teleop 源码复原

## 目录

- [范围与结论](#范围与结论)
- [系统拓扑](#系统拓扑)
- [硬件与物理装配](#硬件与物理装配)
- [Rokoko 手部输入](#rokoko-手部输入)
- [GeoRT 手部重定向](#geort-手部重定向)
- [Vive 腕部输入](#vive-腕部输入)
- [仿真 wrapper 与腕部映射](#仿真-wrapper-与腕部映射)
- [MuJoCo 执行与记录语义](#mujoco-执行与记录语义)
- [固定版本的工程缺口](#固定版本的工程缺口)
- [可迁移结论](#可迁移结论)

## 范围与结论

本页复原 DexJoCo 仓库 commit `8d23b0fab23b17a58c4b55f3942e17013aaf8267` 的 teleop 行为。它描述的是 Franka Panda + Allegro Hand 在 MuJoCo 中的示范采集原型，不是面向任意真机、Isaac 或 MuJoCo 资产的通用协议。

核心拆分：

```text
global wrist motion: HTC Vive Tracker -> Franka target EE pose
local hand shape:     Rokoko glove -> GeoRT -> Allegro joint targets
```

两条输入链在仿真 wrapper 才合并。论文把腕部称为 delta action；代码先计算 tracker delta，再与启用时 EE pose 合成为世界坐标绝对目标。最终记录和执行的是绝对 EE pose + 绝对 hand joint target。

## 系统拓扑

固定版本的进程与默认裸 UDP 载荷：

```text
Rokoko Smartgloves
  -> Rokoko Studio JSON v3 LZ4
  -> RokokoReceiver, default listen :14044
  -> canonical float32[21,3]
       right -> UDP :5013 -> right GeoRT -> float64[16] -> UDP :5014
       left  -> UDP :5015 -> left  GeoRT -> float64[16] -> UDP :5016
                                                            \
                                                             -> sim_teleop wrapper
                                                            /
Vive Tracker(s) -> SteamVR/OpenVR -> float64[3,4] -> UDP :5012
```

单臂默认监听 `5012` 和 `5014`。双臂监听 `5012`、`5014`、`5016`。这些端口只是 DexJoCo 脚本默认值，不是协议标准。

## 硬件与物理装配

### 机器人侧

论文使用：

- Rethink Robotics mount 作为安装底座；
- Franka Panda 作为每臂 7 DoF manipulator；
- Allegro Hand 作为每手 16 DoF 执行器。

arm action 不是 7 个 Franka joint target，而是 EE/TCP 世界坐标绝对 pose。hand action 是 16 个绝对 Allegro joint target。

### 操作者侧

教程 BOM：

| 部件 | 数量 | 作用 |
|---|---:|---|
| Rokoko Smartgloves | 1 对 | 手部骨架/关键点 |
| HTC Vive Tracker | 2 | 左右腕 6DoF |
| HTC Vive Base Station | 2 | Lighthouse 定位 |
| camera stand | 2 | 对角安装 Base Station |
| 3D-printed mounting adapter | 2 | tracker 固定到 glove |
| 1/4-20 TF3010 screw | 2 | 连接 tracker 与安装件 |
| 5 GHz Wi-Fi router | 1 | glove 数据网络 |
| 5V/2A power bank | 1 | 穿戴侧供电 |

论文估计整套成本约 2300 USD。教程要求两个 Base Station 对角相向，以减少身体/手臂遮挡。

### 固定装配假设

教程第一页的装配图规定 tracker 在 glove 手背上的方向。教程明确说 tracker frame orientation 被假定与 simulator world frame 对齐。代码没有在线估计：

- tracker-to-glove/wrist rigid transform；
- tracking world 到 simulator world 的一般外参；
- tracker frame 到 EE command frame 的旋转映射。

因此固定安装不是单纯机械便利，而是运动映射的一部分。迁移时应把这部分变成可版本化标定，而不是复制安装方向后继续假设单位外参。

## Rokoko 手部输入

### Studio 设置

教程要求：

1. 左右 glove 分别通过 USB 配置；
2. 连接 5 GHz Wi-Fi；
3. 在 Rokoko Studio 新建 scene；
4. 在 Streaming 中填写 receiver 地址/端口；
5. 选择 `JSON v3 LZ4` 并激活。

教程示例中的 IP/端口是现场配置，不是代码协议常量。

### JSON 解码与 21 点布局

`teleoperation/rokoko/common.py` 从：

```text
scene -> actors[0] -> body
```

提取每侧 21 个位置：

```text
hand/wrist,
thumb proximal/medial/distal/tip,
index proximal/medial/distal/tip,
middle proximal/medial/distal/tip,
ring proximal/medial/distal/tip,
little proximal/medial/distal/tip
```

parser 先尝试明文 JSON，再尝试多种 LZ4 frame/block 形式。

### 代码中的 canonical frame

`hand_to_canonical` 用以下点构造局部坐标：

```text
origin = point[0]
z      = normalize(point[9] - point[0])
y_aux  = normalize(point[13] - point[5])   # left
y_aux  = normalize(point[5] - point[13])   # right
x      = normalize(cross(y_aux, z))
y      = normalize(cross(z, x))
R      = columns[x, y, z]
```

再把 21 点变换到该局部 frame。由此全局 wrist translation/orientation 被去掉，手型链与 tracker 全局 pose 链解耦。

需要注意：

- 代码不做显式单位转换；它把 Rokoko 数值原样送入 GeoRT。
- 退化轴时只减去 wrist origin 并返回，没有 quality/degeneracy flag。
- 输出是裸 `float32[21,3]`，没有 side、frame、单位、时间戳、sequence 或 calibration ID。
- 双手 bridge 从同一个 Rokoko packet 分别计算左右手，但发到两个独立 UDP 端口。

## GeoRT 手部重定向

### 输入与目标模型

每侧 GeoRT 进程接收 `float32[21,3]`，再按 checkpoint config 中的 `human_hand_id` 选择四个指尖。它不是固定的“thumb、index、middle、ring”数组顺序；顺序由目标手配置决定。

固定版本右手 config：

| 输出关节块 | robot fingertip | human point |
|---|---|---:|
| `joint_0.0..3.0` | index | 8 |
| `joint_4.0..7.0` | middle | 12 |
| `joint_8.0..11.0` | ring | 16 |
| `joint_12.0..15.0` | thumb | 4 |

固定版本左手 config：

| 输出关节块 | robot fingertip | human point |
|---|---|---:|
| `joint_0.0..3.0` | ring | 16 |
| `joint_4.0..7.0` | middle | 12 |
| `joint_8.0..11.0` | index | 8 |
| `joint_12.0..15.0` | thumb | 4 |

这说明即使左右手输出都是 16 维，也不能共享无 side/layout 的数组语义。

### 部署网络

每个目标 fingertip/joint group 使用一个独立 MLP：

```text
3 -> 128 -> LeakyReLU -> BatchNorm
  -> 128 -> LeakyReLU -> BatchNorm
  -> number_of_group_joints -> Tanh
```

四个网络的输出写入 config 指定的 joint indices。`HandFormatter` 再按 checkpoint config 中的上下限从 `[-1,1]` 反归一化为目标关节角。

部署代码：

- 用 `.cuda()` 强制 GPU；
- 通过 checkpoint 名称 substring 选择第一个匹配目录；
- 输出函数虽注释为 float32，实际转换为 `float64` 再发 UDP；
- 只在收到新手部 packet 时推理和发送；
- 不发送 solver quality、checkpoint ID、layout ID 或采样时间。

### 个体 calibration / 训练

DexJoCo 的“calibration”是采集操作者 fingertip workspace，再训练个人 GeoRT 映射：

1. 记录 canonical `frames × 21 × 3` `.npy`；
2. 自然伸展/弯曲各指并覆盖 workspace；
3. 做多种 pinch；
4. 用目标 URDF 和 joint limits 随机采样机器人 FK 数据；
5. 训练 neural FK surrogate；
6. 用人手 workspace 训练四个 fingertip-to-joint MLP；
7. 回放检查后再接实时 UDP。

固定版本 neural FK 默认使用 100,000 个机器人姿态、batch 256、Adam `5e-4`、200 epochs。retarget MLP 默认使用 AdamW `1e-4`、batch 2048，并包含 direction、workspace/chamfer、curvature/flatness 和 pinch 目标。

论文总损失包含 collision 项，但代码中的 classifier 路径被注释，`collision_loss` 固定为 0，默认 `w_collision=0.0`。不要把论文方法式等同于该 commit 的实际 checkpoint 训练行为。

GeoRT 目录有独立 CC BY-NC 许可。即使 DexJoCo 根仓库是 MIT，也不能由根许可推断 GeoRT 代码/checkpoint 可按 MIT 商用复用。

## Vive 腕部输入

### SteamVR/OpenVR

教程在没有 HMD 时把 SteamVR `requireHmd` 设为 `false`，并要求两个 Base Station 分别使用 B/C 模式。

`send_vive_pose.py`：

- 以 `VRApplication_Other` 初始化 OpenVR；
- 默认使用 `TrackingUniverseStanding`；
- 可按 OpenVR device index 或 serial substring 选 tracker；
- 默认发送到 `127.0.0.1:5012`，频率 90 Hz；
- 单 tracker payload 是连续 `float64[3,4]`；
- 双 tracker payload 是 primary 的 12 个 float64 后接 secondary 的 12 个 float64。

双臂 wrapper 把第一个 pose 解释成 right，第二个解释成 left。`--two-trackers` 只按设备发现顺序选前两个 tracker，并不验证侧别。若只收到单 pose，双臂 wrapper 会把同一 pose 同时赋给左右臂。

sender 首次启动要求在 warm-up 期间出现有效 pose。运行中若当前 pose 无效，则继续发送之前缓存的最后有效 pose；payload 中没有 validity 或 age。

## 仿真 wrapper 与腕部映射

### 启用与 reference capture

MuJoCo GUI 中按 `;` 切换 intervention。启用时：

1. 在 GLFW key callback 内阻塞等待 2 秒；
2. 读取“当前最新” tracker pose；
3. 读取环境当前 EE target pose；
4. 保存两者为 reference。

这两个 reference 没有共享时间戳或原子采样保证。代码也没有先检查 hand packet、tracker age、tracking quality 或 backend health。

### SE(3) 映射

单侧计算：

```text
tracker_delta = inverse(tracker_start_world) @ tracker_now
tracker_delta.translation *= pose_scale
target_pose = ee_start @ tracker_delta
```

旋转不缩放。默认 `pose_scale=1.5`；部分单臂任务 config 使用 `2.0`。该右乘公式依赖 tracker reference frame 与 EE reference command frame 对齐。

输出姿态为：

```text
[x, y, z, qw, qx, qy, qz]
```

因此输入是相对 tracker motion，wrapper 输出却是世界/环境中的绝对 EE target。

### hand 合并

intervention 激活时，wrapper 取缓存的最后一个 16 维 hand qpos。单臂返回：

```text
[absolute_EE_pose_wxyz(7), absolute_Allegro_joints(16)]
```

双臂返回：

```text
{
  "right": right_pose7 + right_hand16,
  "left":  left_pose7  + left_hand16,
}
```

没有 hand packet freshness、左右手同步或 arm-hand pairing。接收 hand qpos 时，少于 16 个值会补零，多于 16 个值会截断。

## MuJoCo 执行与记录语义

单臂环境把 EE pose 写入 MuJoCo mocap target，再用 operational-space controller 计算 Franka torque；Allegro 16 维目标直接写入对应 actuator `ctrl`。双臂环境对左右 mocap target 和两组 Allegro actuator 做同类处理。

所以 DexJoCo 的可观察语义是：

```text
tracker delta
  -> absolute mocap target
  -> operational-space torque for simulated Franka

GeoRT qpos
  -> absolute Allegro actuator target
```

teleop wrapper 把实际采用的 command 写入 `info["intervene_action"]`，供示范记录。原始 tracker/glove packet、接收时间、staleness、supervision decision 和 command-feedback error 不在该 action 中。

论文/笔记中的单臂 23 维、双臂 46 维是这个 embodiment 的数据布局，不是可迁移接口。尤其双臂采集组块与部分 policy flat layout 还需要额外重排。

## 固定版本的工程缺口

以下是代码事实，不是对论文方法价值的否定：

| 缺口 | 结果 |
|---|---|
| 裸 UDP 无 header | 无法确认 schema、side、source、sequence、frame 或版本 |
| 无 source/receive timestamp | 无法测量 age、跨流同步或乱序 |
| 永久缓存 latest sample | tracker/glove 失联可表现为继续保持旧命令 |
| 启用门控不检查数据健康 | 首帧缺失时可能使用 identity tracker 或 zero hand |
| 固定安装代替外参 | 更换 mount/asset 后旋转映射易错误 |
| 双 tracker 侧别靠顺序 | 设备重枚举可能左右互换 |
| hand payload pad/truncate | schema 错误可能被静默伪装成有效 16 维 |
| arm/hand 独立 socket | 没有一致性快照或可见 skew |
| 无 supervisor/stop contract | 仿真 hold 行为不能安全迁移到真机 |
| GeoRT 只看瞬时 fingertip | 不表达接触、触觉、物体、历史或操作者施力 |
| 名义采样时间 | 默认 30 Hz 数据视图不能恢复真实接收抖动 |

GeoRT 的小 MLP 前向速度不等于端到端 teleop 频率；整体还受 glove 输出、网络、进程调度、仿真 step 和 recorder 限制。

## 可迁移结论

### 可直接迁移的模式

- 用独立传感器分别承担 wrist global pose 与 local hand shape。
- 用 clutch/reference pose 产生相对主端运动。
- 在从端输出前再合成为绝对目标。
- 用目标模型、joint layout 和个体 workspace 训练/配置 hand retargeter。
- 让小型、低延迟 hand retargeter 独立于下游 policy。

### 必须替换的具体物

- Allegro URDF、16 维布局、limits 和 GeoRT checkpoint；
- Franka/MuJoCo mocap-body controller；
- 固定 `5012/5013/5014/5015/5016` 裸 UDP；
- 以固定安装方向代替外参的做法；
- 无 freshness/watchdog 的 latest-value cache；
- 23/46 维 flat action；
- 缺少 raw/canonical/intent/decision/command/feedback 分层的记录方式。

### 仍需目标系统决定

- tracker/glove 的具体型号与 SDK；
- arm 使用 Cartesian controller、IK 还是 joint control；
- hand retargeting 采用官方 SDK、优化器、学习映射或混合方式；
- 三 backend 的 hold/stop/recovery；
- 双臂同步、碰撞和相对几何约束；
- 触觉/力觉是否进入 supervision 与数据集。
