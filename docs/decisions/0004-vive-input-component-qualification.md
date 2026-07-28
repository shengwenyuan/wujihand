# ADR-0004：NV-1 作为 VIVE 输入组件资格验证

状态：已接受
日期：2026-07-28
对应计划：NERO-VIVE R1，NV-1

## 背景

现有五层 `Session v1` 表达一个具体的 simulator backend、Asset Binding、Assembly、
Workcell 和执行合同。NV-1 的目标则是在不启动 Isaac、ROS 或 NERO 的情况下，只读验证
VIVE Tracker 的身份、6-DoF pose、失联和按钮事件。

把 Tracker 伪装成 Asset/Backend Binding，或借用一个与 VIVE 无关的 Isaac Session，
都会制造错误 provenance；为 NV-1 单独扩写 Session backend 也会把设备采集和目标执行
混成一层。

## 决定

1. NV-1 是 `VIVE input component qualification`，不是正式遥操作 runtime。
2. NV-1 不新增或修改 Asset、Backend Binding、Assembly、Workcell、Session，也不改变
   现有 Session hash。
3. canonical `TrackedRigidBodySample`、`ClutchEvent` 和 tracking input port 位于
   domain/ports；OpenVR 类型只能存在于 `adapters/input`。
4. qualification CLI 是显式 diagnostic wiring：只枚举/采集/记录，不发布 UDP/ROS
   command，不导入 Isaac/NERO，也不接受 `--session`。
5. Tracker 以 OpenVR `Prop_SerialNumber_String` 为稳定身份；临时 device index 只在
   adapter 单次扫描内使用。
6. NV-3 出现真实 NERO Isaac consumer 后，目标端仍由五层 Session 作为 composition
   root；跨进程 producer、transport、recorder 和目标 Session 由 `DeploymentSpec`
   编排。

## 结果

- NV-1 可以在没有仿真器和机械臂的情况下独立部署、回放和做 HIL。
- 失联样本和按钮事件仍经过 canonical contract，后续无需把 OpenVR SDK 对象带入
  application。
- NV-1 Gate 只证明输入组件合格，不等价于 Session、遥操作链路或真机安全放行。
- 如果未来要求 NV-1 自身成为正式多进程部署，必须先定义 DeploymentSpec schema 和
 兼容性测试，不能回填虚假五层配置。
