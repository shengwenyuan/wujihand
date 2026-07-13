# 2026-07-13 Hand 2 固定法兰转向抓球验证

## 结论

需求开发与验收结束。

- 确定性 Isaac PhysX 严格抓取通过，隔离重复试验 `10/10` 通过。
- 真人 MediaPipe 已完成抓球功能测试并成功；操作流畅度受 landmark 抖动、遮挡和大角度姿态跳变限制。
- 本结论覆盖固定 XYZ、三轴转向和刚性球抓取，不覆盖腕部平移、软体指腹、触觉或真机抓力。

## 固定范围

| 项目 | 值 |
|---|---|
| 产品 / 模型 | Wuji Hand 2 Beta 1 right / `wuji-description v2026.6.27` |
| Isaac Sim / GPU | 5.1.0 standalone / RTX 4090 |
| Wuji SDK / MediaPipe | 2026.7.2 / 0.10.35 |
| Hand 2 USD SHA-256 | `3cb3dcb18b07621a52a47a8daa98ab82794e3c77d36275d068b3b5b0516e5f00` |
| 法兰 task home | `[0, 0, 0.454] m`，物理 pitch `+60°` |
| 球 | center `[0.130, 0.025, 0.410] m`，radius `0.030 m`，mass `0.050 kg` |
| 频率 | physics `120 Hz`，command `60 Hz` |

精确来源固定在 `third_party/sources.lock.yaml`。runner 计算实际 USD hash，并与模型/场景 profile 的 repository、tag、commit、路径和 hash 交叉校验。

## 自动物理结果

规范命令：

```bash
export ISAAC_SIM_ROOT=/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64
"$ISAAC_SIM_ROOT/python.sh" \
  tools/run_isaac_hand2_rotation_ball.py \
  --command-source scripted --frames 1200 --require-grasp-success \
  --validation-output-dir artifacts/runs/isaac_hand2_rotation_ball_60deg_home_final
```

| Gate / 指标 | 结果 |
|---|---:|
| runtime DOF | `23`，wrist3 + finger20 name/path 分区通过 |
| 最大固定法兰平移误差 | `3.408145944502017e-08 m` |
| 首次抓取通过 / 最佳连续保持 | `8.875 s / 2.125 s` |
| 球底最大桌面净空 | `28.92 mm`，要求 `≥20 mm` |
| 通过接触组 | thumb + middle + ring |
| 合格窗口最大 ball-in-palm 滑移 | `0.235 mm`，要求 `≤5 mm` |
| 整手/table contact | `[27, 1, 3]` 有效矩阵；`0` 接触帧，峰值 `0 N` |
| 结构 / 运动 / 抓取 | `true / true / true` |

球体全程为动态刚体；脚本只写 q20 与 D6 target，没有 attachment、teleport 或 kinematic 搬运。contact view 为空、非有限或 shape 变化时 runner fail closed。

重复入口 `tools/run_isaac_hand2_rotation_ball_trials.py` 以 10 个独立 Isaac 进程执行，结果 `10/10`，高于 profile 要求的 `8/10`。证据位于被 Git 忽略的：

- `artifacts/runs/isaac_hand2_rotation_ball_60deg_home_final/validation.json`
- `artifacts/runs/isaac_hand2_rotation_ball_60deg_home_final/hand2_rotation_ball.png`
- `artifacts/runs/isaac_hand2_rotation_ball_60deg_home_trials_final/summary.json`

## 人工结果

操作者使用真实右手完成了 neutral、转向、五指收拢和抓起球体，功能测试成功。实测操作不够顺畅，主要限制来自 MediaPipe 的 landmark 抖动、遮挡和大角度估计跳变；采用物理 `60°` task home 后只需较小相对 pitch，已降低操作难度。

人工结果证明链路可用，但不声明视觉输入具有确定性脚本的 `10/10` 重复率。后续若要提高体验，应作为独立的视觉滤波/跟踪需求处理。

## 回归与边界

最终回归包括 Ruff、mypy、默认 pytest、真实 loopback socket、Isaac author-only overlay 和单轮严格 PhysX gate。确切命令与最终计数以本次 Git 提交和 `plans/last_edits.md` 留痕为准。

Isaac 仍可能输出 pinned 上游资产的 fingertip visual unresolved-reference、wrist collision Fabric 和禁用旧 root 的 disjointed-transform warning；它们未阻断唯一 articulation、23 DOF、有限状态、固定法兰或抓取 gate。资产或 Isaac 版本变化后必须重验。

Hand 2 Beta 1 官方当前只有刚性骨骼仿真模型，也不应把关节电流当作外部接触力依据。因此这里的成功只代表 pinned USD 与当前 PhysX/material 参数下的刚性接触抓取。
