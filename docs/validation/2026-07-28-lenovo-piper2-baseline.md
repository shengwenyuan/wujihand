# 2026-07-28 `lenovo-piper2` 目标机基线

状态：`NV-0/0A PROCEED`；软件环境验证通过，设备盘点尚未关闭

本记录是 `lenovo@Workstation2` 的只读盘点与 NV-0 验证快照。SSH alias 为
`lenovo-piper2`，用户目录为 `/home/lenovo`；项目 checkout 位于
`/home/lenovo/swy/wujihand`。

## 环境矩阵

| 项目 | 实测值 |
|---|---|
| OS / kernel | Ubuntu 24.04.4 LTS / `6.14.0-27-generic` |
| GPU / 显存 | NVIDIA RTX 5090 / 32607 MiB |
| NVIDIA driver | `595.58.03` |
| `nvidia-smi` CUDA | `13.2`，表示当前驱动支持的 CUDA API 上限 |
| 系统 CUDA toolkit | `/usr/local/cuda -> cuda-12.8`；`nvcc 12.8.61` |
| Isaac Warp toolkit | `12.9` |
| system Python | `3.12.3` |
| Isaac Sim | `isaacsim 6.0.1.0` / Python 3.12.3 |
| Isaac environment | `/home/lenovo/.venvs/isaacsim-6.0.1` |
| project environment | `/home/lenovo/swy/venvs/wujihand-py312-nv0` |
| ROS 2 / RMW | Jazzy / `rmw_fastrtps_cpp 8.4.4` |
| source commit | `b8c68bcd4ab9381b2aaa3d444fda116333e67989` |

因此目标机不能笼统记录为“已安装 CUDA 13.2”：13.2、12.8.61 和 12.9 分别来自
driver API 上限、系统 toolkit 和 Isaac Warp 运行时报告，含义不同。

## 已通过验证

- 基础 package 已放宽为 Python `>=3.11,<3.13`。Python 3.11 保留为最低版本及
  perception/retarget worker 回归基线；项目与 ROS 使用独立 Python 3.12 环境。
- 干净 Python 3.12 项目环境安装成功，五层/domain/transport fast 集合为
  `133 passed`。
- 恢复 `wuji-description` 固定提交
  `aee64892ebcf8e3237bedc30231bb09476cbc71d` 后，42 个 sparse asset 的 Git blob
  SHA 与 SourceLock 一致；固定 Hand 2 Session 在开启 artifact 校验时解析通过，
  session hash 为
  `246899c5dce7b8c7c1de75e963e6c7b4aede0e780c755b1929c33d17d48c6472`。
- Isaac Compatibility Checker 结果为 `PASSED`。当前唯一需保留的性能提示是 CPU
  governor 为 `powersave`；它不改变兼容性 Gate 结论。
- Isaac GUI 空场景在活动显示会话中启动并报告
  `isaac_gui_empty_scene=PASS`。
- 固定 Hand 2 Session 的 headless smoke 完成 8 帧 stepping：USD/source lock、
  articulation root、20 DoF、joint layout、limits 和 finite feedback 均通过，进程
  退出码为 `0`，并生成 960×720 截图。
- 最终 runner 复验从 Isaac API 读取并记录 `isaac_sim=6.0.1`，不再使用旧的硬编码
  版本；Session hash、20 DoF、limits 和 feedback 检查继续通过。
- ROS 2 Jazzy 的 `ros2 doctor --report` 退出码为 `0`，报告 middleware 为
  `rmw_fastrtps_cpp`。在临时测试域 `ROS_DOMAIN_ID=91` 中，独立 `ros2 topic pub`
  与 `ros2 topic echo` 完成一次 `std_msgs/String` Fast DDS loopback，双方退出码为
  `0`。
- Isaac 6.0.1 ROS 2 core ABI checker 退出码为 `0`；headless Kit 加载 system Jazzy
  后，`isaacsim.ros2.bridge`、`isaacsim.ros2.core` 均 enabled，core startup status
  为 `true`。

Isaac vendor venv 保持厂商 numeric stack，不在其中安装项目 NumPy/SciPy 依赖；runner
从受控 checkout 加载项目源码。

## ROS 2 状态

ROS 2 Jazzy 已从官方 apt source 安装。源包保留在：

```text
/home/lenovo/swy/bootstrap/ros2-apt-source_1.2.0.noble_all.deb
```

其 SHA-256 为
`0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5`。
已安装 `ros-jazzy-ros-base 0.11.0`、`ros-jazzy-rmw-fastrtps-cpp 8.4.4` 和
`python3-colcon-common-extensions 0.3.0`。`RMW_IMPLEMENTATION` 在测试中显式固定为
`rmw_fastrtps_cpp`；测试域 91 只用于本次 loopback，不是后续部署的默认域。

## SteamVR、VIVE 与 NERO 盘点

Steam 客户端已更新并完成登录；SteamVR App 250820 已完整安装，BuildID 为
`23791826`，runtime 日志报告 SteamVR `2.16.7` / VR Server `v1781734990`。
目标机禁用了非特权 user namespace，Steam URI 的新 runtime wrapper 因此不能启动；
本轮没有修改该内核安全策略，而是继承已运行 Steam 客户端的 runtime library path
启动官方 `vrmonitor.sh`，`vrmonitor` 与 `vrserver` 均进入运行态。

USB 当前可见：

| USB ID | 枚举名称 | 已知事实 |
|---|---|---|
| `28de:2101` | Valve Software Watchman Dongle | serial `3C15A0301B` |
| `28de:2300` | Valve Software LHR | serial `LHR-24B6E288` |

USB 名称不足以完成 Tracker logical role 分配；必须等 SteamVR/OpenVR inventory 返回
device class、model 和稳定 serial 后确认。Base Station 通过供电和光学链路工作时也
不一定作为 USB 设备枚举。SteamVR lighthouse 日志已把 `LHR-24B6E288` 识别为与
`HTC Tracker 3` 相符的设备并打开 IMU、Optical 和 VrController HID，但当前没有 HMD
枚举，OpenVR background application 被明确拒绝为
`InitError_Init_HmdNotFound`。因此尚未完成正式 inventory、tracking origin、
6-DoF pose、遮挡/重捕获或按钮 HIL。

盘点时没有 `/dev/serial/by-id` 条目、CAN network interface 或可识别的
NERO/USB-CAN 设备。本记录只说明目标机当时未枚举到这些设备，不推断机械臂状态。

## 证据位置

证据根目录：

```text
/home/lenovo/swy/wujihand/artifacts/nv0-compat/
```

| 证据 | SHA-256 |
|---|---|
| `python312-editable-install.log`（首次 Python 约束暂停） | `e09c80aa916695547c114f9c35fc29c3e4afa01f5c65a19f94c942dc5e7c2934` |
| `python312-editable-install-relaxed.log` | `103e58195e9a58a9c76a5bfe1344388a4161f536a38be140337ef158ec905e82` |
| `python312-five-layer-fast.log` | `8a7cfe3b95481a07a0ca008702dbebff6f21ca3ad0c575cde98671d32f220860` |
| `python312-session-resolver-artifact-gate.log` | `c98a494346a6e2a3dc14373b7779e23ab19aa336987cdb30ad457216c728a2d7` |
| `isaac601-runtime-versions.log` | `24dc3ad9f6ee081c2b92141e99cf60fb8d233aae5a2606c3392049ecfd541c3e` |
| `isaac601-compatibility-check.log` | `e0b8f969ab431b82b834f70bc8f10aeb6517dbd9927b131112b73a9616333163` |
| `isaac601-gui-empty-scene.log` | `6ba2f4cf765d57cd6d7c1047be604aae9619200d128e47c1a5b8c6c55616b3a6` |
| `isaac601-fixed-8.log` | `160b411873f03981503a1f0ff08e3af15d71ec28853226ac59192b460ecce746` |
| `isaac601-fixed-8/validation.json` | `239fa6b2513560eeac08da1aecfbef1a0e94d191702c22bc7a16023eb4d43d5a` |
| `isaac601-fixed-8/hand2_table.png` | `4f572282831357328fa2f4a0cd9b55c08b01df3b48be8e1e5d6847f88eac4c80` |
| `isaac601-fixed-8-final.log` | `0762ef13c94e20ad2062bbb636cd132a91391d8d27e609800b591a7a023d51fd` |
| `isaac601-fixed-8-final/validation.json` | `e724085e137773aa7f4f1dc14f9f972825654ac37fb1d872968dd28b511c5d18` |
| `isaac601-fixed-8-final/hand2_table.png` | `72507ed65b6387e78496db75e73155ace0a8206cd5444a40570815c87d5ea95c` |
| `ros2-doctor-report.log` | `1e2fd9f91fe59e11f347339cc101a20dcd9797eefa943aa74e88b1ec066088c4` |
| `ros2-fastdds-publisher.log` | `d736a828be9c0b3d3d82dcd361e396dcbe557521fc03022e9ff0af672e2d91be` |
| `ros2-fastdds-listener.log` | `b83073a4416cc7dd124794c0b3ffac2d811f035d0e1cccee60e19ab1f8e7e4b5` |
| `isaac601-ros2-core-check.log` | `26df4d25f0f619a6b40b5a9278d50ceafff7a11e4b2c4bfe7e0c4d5f74dcf74c` |
| `isaac601-ros2-bridge-smoke.log` | `4e18ea47c2859d57568696d9cbd364a2d40d87fdfd77e10713d2a45800a12178` |

## NV-0 Gate 剩余项

NV-0/0A 与软件环境部分已通过。完整设备 Gate 仍需：

1. HMD 接入后完成 OpenVR inventory，并记录 tracking origin。
2. 在 USB-CAN/NERO 接入后做只读设备、接口、序列号和固件盘点。
