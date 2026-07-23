# 2026-07-13 MuJoCo FR3 v2—Hand 2 桌面验证

> 历史证据：本页验证的是已被 2026-07-14 长边侧置四棱台布局取代的桌面安装初版。当前布局与结果见 [2026-07-14 验证报告](2026-07-14-mujoco-fr3-hand2-table-layout.md)。

## 结论

第一版 MuJoCo 桌面机械基线通过自动 contract、10 秒静置、分区小阶跃、平滑桌面接触、headless runner 和 EGL 离屏渲染目检。结论只覆盖锁定模型下的刚体仿真，不覆盖真实转接件、实机 FR3、软体/触觉或遥操作。

## 固定范围

| 项目 | 值 |
|---|---|
| Python / MuJoCo | `3.11.13` / `3.10.0` |
| FR3 v2 | Menagerie `71f066ad0be9cd271f7ed58c030243ef157af9f4` |
| Hand 2 right | `wuji-description v2026.6.27` / `aee64892ebcf8e3237bedc30231bb09476cbc71d` |
| 组合维度 | `nq=27, nv=27, nu=27` |
| physics / control | `2 ms implicitfast + Newton + sparse` / `100 Hz, 5 substeps` |
| 安装 | 桌后沿内缩 `0.12 m`，根 identity 朝 `+X` |
| 法兰变换 | identity 仿真假设；待真实 adapter 标定 |

## 自动验证

新增切片：

```bash
.venv/bin/python -m pytest \
  tests/unit/test_mujoco_table_config.py \
  tests/contract/test_mujoco_fr3_hand2_model.py \
  tests/integration/test_mujoco_fr3_hand2_smoke.py -q
```

结果：`16 passed in 1.34s`。

全仓回归结果：`136 passed, 7 deselected in 1.41s`；被排除的是项目既有的真实 socket 标记用例。

| Gate | 结果 |
|---|---|
| 配置 | rear `x_min`、真实朝桌心的 `+X`、桌面 Z、控制周期、MJCF/mesh-tree hash 和 identity assumption 严格校验 |
| 模型 | 7 FR3 + canonical 20 Hand 2，全 hinge、全唯一 direct actuator、无 free joint；Hand 2 `32` 条 contact exclude 保留 |
| attachment | `r_base_link` 的 parent 为 `fr3v2_link8`，局部 pose 与配置一致 |
| 标记 | 五个 fingertip site、workspace site、桌面/地面/四腿均存在 |
| reset | 两次 reset 的 q、法兰、掌根和五指尖逐值相同；初态 `0` contact |
| 10 秒 hold | 全状态 finite；法兰—掌根平移漂移 `<1e-12 m`；末态 arm/hand 最大速度 `<1e-9 rad/s` |
| 分区目标 | q7 和 q20 各自小阶跃响应；ctrl 写入由名称派生的独立 actuator ids |
| 桌面接触 | 3 秒 smoothstep 到接触 pose + 3 秒 hold；存在 tabletop contact、最大穿入 `<2 mm`、末态 finite 且最大关节速度 `<0.01 rad/s` |
| runner | 0.1 秒 subprocess 输出 `MuJoCo 3.10.0`、27/27/27、实际 hash 和零 attachment 平移漂移 |

另执行正式 10 秒 runner 并保存 `/tmp/mujoco_fr3_hand2_final.json`：退出码 `0`，两份 MJCF 与两棵 mesh tree hash 均匹配，仿真时间 `10.000000000000009 s`，`finite=true`，末态 `0` contact，attachment 平移漂移 `0 m`、相对四元数对齐度 `0.9999999999999978`，末态 arm/hand 最大速度分别为 `5.47e-14 / 1.22e-15 rad/s`。

额外 10 秒诊断观测到：最大瞬态关节速度 `0.54043 rad/s`，最大 home 位置偏差 `0.051324 rad`（主要为 Hand 2 弱位置执行器在重力下的弹性偏差），末态速度降至约 `5.5e-14 / 1.3e-15 rad/s`，全程无桌面接触。平滑接触诊断首次接触约 `2.87 s`，峰值 `3` 个 contact，最大穿入约 `1.589 mm`，保持结束仍为有限稳定接触。

静态质量检查：

```text
.venv/bin/python -m ruff check src tests tools
All checks passed!

.venv/bin/python -m mypy src
Success: no issues found in 30 source files

UV_CACHE_DIR=/tmp/wujihand-uv-cache uv lock --check
Resolved 73 packages
```

## 渲染目检

执行：

```bash
MUJOCO_GL=egl .venv/bin/python tools/run_mujoco_fr3_hand2_table.py \
  --duration-s 0.1 --render-ppm /tmp/mujoco_fr3_hand2.ppm
```

EGL 离屏渲染成功。人工查看确认：桌面和四腿完整；FR3 基座位于后沿中央区域并朝桌心伸展；Hand 2 无可见法兰错位，张手朝下；机械臂、手和桌面无初始可见穿模。该图是临时验证产物，未作为唯一证据提交。

## 尚未验证

- GUI viewer 的本轮真人交互操作未执行；只验证了相同 runner 的 headless 路径和 EGL 相机。
- identity 转接件没有 CAD/真机尺寸证据；真实加工与负载结论不得从本报告推出。
- 没有验证 IK/OSC、遥操作延迟、采集同步、FCI/libfranka、急停或真机碰撞安全。
- Hand 2 上游 collision bit 和 contact excludes 保持原样；接触通过不表示整只手表面均有碰撞几何。
