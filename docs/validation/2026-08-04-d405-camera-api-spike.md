# 2026-08-04 D405 140°纯仿真 Camera API 验证

状态：`PASS`（计划 007 / S0）

## 结论

workstation2 的 Isaac Sim `6.0.1.0` / RTX 5090 已验证版本化
`isaac_d405_synthetic_wide_angle_140_v1` profile：

- API readback：`640×480`、pinhole、HFOV `139.9999989°`、VFOV `128.2259920°`、
  clipping `[0.0199999996, 5.0] m`；
- Writer 同一回调：RGBA `uint8[480,640,4]`、optical-Z depth
  `float32[480,640]`、共同 rational `reference_time`；
- depth marker：`0.94999999 m` / `1.95000017 m`，符合两个 authored 前表面
  `0.95 m` / `1.95 m`；背景 no-hit 为 `+inf`；
- K/render marker error：`0.543 px` / `0.769 px`，低于 `2 px` Gate；
- `swhFrameNumber` 两帧均为 `0`，已明确拒绝为 identity；有效完成帧身份是 Writer
  `reference_time`；
- 第二个状态前丢弃一个旧完成帧，证明生产录制必须以 `reference_time` join pose history。

实验还确认：连续 `rep.orchestrator.step()` 不能稳定重复触发带 tick gate 的 CameraSensor，
对持续 sensor workflow 调用 `rep.orchestrator.wait_until_complete()` 会等待不返回。正式实现
使用 committed timeline app update 驱动 CameraSensor，并以有界等待 writer callback 为完成
条件，不采用上述两条失败路径。

## 入口与边界

本地版本化入口：

```bash
tools/qualify_isaac_d405_camera_api.py \
  --project-root /path/to/wujihand \
  --output-dir /path/to/output
```

远端执行环境与证据根：

```text
SSH alias / hostname: lenovo-piper2 / Workstation2
Isaac Python: /home/lenovo/.venvs/isaacsim-6.0.1/bin/python
Evidence: /home/lenovo/swy/wujihand_d405_wrist_rig_scratch/s0_api_probe/
```

该结果只属于纯仿真特殊镜头。`140°` 不是 RealSense D405 物理规格或标定，不得用于实体
相机内外参。

## 证据 SHA-256

| 证据 | SHA-256 |
|---|---|
| `output/report.json` | `1a2e18ec792f763e70b1d30075f5f7b2e7ce8053453f76f8c332e1056b45566d` |
| `output/capture_00_red_near_right_rgba.png` | `8616be9166ab15d7b43339ed87138fad2cbee30268ef059a6558202afed593ca` |
| `output/capture_01_red_far_left_rgba.png` | `5e74fd1c5ca801615b40480fdb9eefd742e80e9ad39cd476215c764e65882652` |
| `run.log` | `f12493dfddc10ea7e299d9ea67bb876cbb7c8bb544f5bf64bd74c24fbaaf3795` |

实现过程中的失败探针不作为通过证据；最终入口退出码为 `0`。
