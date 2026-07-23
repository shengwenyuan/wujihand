# 2026-07-14 MuJoCo FR3 v2—Hand 2 长边侧置四棱台验证

## 结论

桌面扩大、FR3 长边侧置四棱台和观察光修订通过自动配置/模型契约、10 秒静置、分区小阶跃、平滑 Hand 2 桌面接触、headless runner 和 EGL 离屏渲染目检。组合仍严格为 `7 + 20 = 27 DoF`；结果只覆盖锁定刚体模型下的仿真布局，不覆盖实体台座承载、抗倾覆、真实转接件或真机安全。

## 固定布局

| 项目 | 值 |
|---|---|
| 桌面 | `1.60 × 1.00 × 0.06 m`；顶面 `z=0.750 m` |
| 四棱台位置 | `y_max` 长边外侧；底面与桌投影间隙 `0.020 m` |
| 四棱台 | 中心 `[0, 0.82, 0.235] m`；高 `0.470 m`；顶/底 `0.42/0.60 m` 方形 |
| FR3 base | `[0, 0.82, 0.47] m`；yaw `-90°`，local `+X` 朝桌心 |
| joint2 | 实测 `[0, 0.82, 0.803] m`；高于桌面 `0.053 m` |
| 观察光 | 单盏 directional；归一化斜向下；无阴影、低高光 |
| 组合维度 | `nq=27, nv=27, nu=27`；无新增 pedestal joint |

四棱台由 8 个顶点、12 个三角面构成闭合凸网格。编译后的世界顶点包围盒底部与地面误差 `<2e-8 m`，顶部与配置台面高度误差 `<2e-8 m`。

## 自动验证

目标切片：

```bash
.venv/bin/python -m pytest \
  tests/unit/test_mujoco_table_config.py \
  tests/contract/test_mujoco_fr3_hand2_model.py \
  tests/integration/test_mujoco_fr3_hand2_smoke.py -q
```

结果：`23 passed in 1.46s`。

全仓回归：`143 passed, 7 deselected in 1.64s`；被排除的是既有真实 socket 标记用例。

| Gate | 结果 |
|---|---|
| 配置 | 桌面长轴、`y_max` 外侧间隙、四棱台上下尺寸/高度、base-on-pedestal、朝桌心且保持竖直、joint2 声明净高和观察光均严格校验 |
| 模型 | 27/27/27、canonical q7/q20、唯一 direct actuator、32 条 Hand 2 contact exclude 保持不变 |
| 四棱台 | `arm_pedestal` 为固定 8 顶点/12 面 mesh geom，世界底/顶分别落在 `z=0/0.47 m`；FR3 base footprint 完整落在顶面内，不增加自由度 |
| joint2 | 编译后用 `xanchor` 实测净高 `0.053 m`，与 scene profile 一致 |
| 观察光 | 默认 headlight 关闭；仅 1 盏命名 directional light；方向/颜色来自配置，`castshadow=false` |
| reset / hold | 初态无接触、最低指尖高于桌面；10 秒 finite，末态 arm/hand 最大速度约 `5.47e-14 / 2.61e-15 rad/s` |
| 平滑接触 | 3 秒 smoothstep + 3 秒 hold；`2.89 s` 首次由 `r_middle_finger_distal` 接触桌面，最深穿入 `0.840 mm` |
| 接触末态 | 保持 1 个 Hand 2—tabletop 接触，无非手部桌面接触；arm/hand 最大速度 `1.75e-6 / 2.40e-4 rad/s`，finite |
| runner | JSON 新增桌、台座、base 配置与 joint2 编译实测几何；0.1 秒 subprocess contract 通过 |

静态检查：

```text
.venv/bin/python -m ruff check src tests tools
All checks passed!

.venv/bin/python -m mypy src
Success: no issues found in 30 source files

UV_CACHE_DIR=/tmp/wujihand-uv-cache uv lock --check
Resolved 73 packages
```

## 10 秒 runner 与渲染

执行：

```bash
MUJOCO_GL=egl .venv/bin/python tools/run_mujoco_fr3_hand2_table.py \
  --duration-s 10 \
  --report /tmp/mujoco_fr3_hand2_pedestal_final.json \
  --render-ppm /tmp/mujoco_fr3_hand2_pedestal_final.ppm
```

退出码 `0`；MuJoCo `3.10.0`；`finite=true`；仿真时间 `10.000000000000009 s`；末态 `0` contact；法兰—掌根平移漂移 `0 m`，相对四元数对齐度 `0.9999999999999976`。

EGL 图像人工确认：扩大后的桌面与腿部未被画面边界裁切，蓝灰四棱台、FR3 和 Hand 2 均在 overview 画面内；base 明确位于台座顶面而非桌面；台面低于桌面、joint2 略高于桌面的关系可见；无初始可见穿模。观察光消除了旧版硬阴影，但不作为像素级确定性证据。

## 尚未验证

- GUI viewer 本轮未进行真人交互；headless、相同模型的 EGL 相机和 runner 已验证。
- 台座尺寸/颜色、`53 mm` 净高和 `20 mm` 桌边间隙是当前仿真假设，不是结构、抗倾覆或人体工学结论。
- identity 法兰转接件仍没有 CAD/真机数值证据。
- 没有验证 IK/OSC、遥操作、采集、FCI/libfranka、急停或真机碰撞安全。
