# nfocc_ph0 — NFOCC PH-0 几何内核（Python 参考实现）

纯 Python/NumPy 实现，**供原理调试与算法验证**，不是产品代码。
产品实现见 `docs/SPEC-*.md`（C99），本目录是它的可执行参照物。

对应规范：`docs/SPEC-COORD.md`、`docs/SPEC-LUT.md`、`docs/SPEC-EPIPOLAR.md`、
`docs/SPEC-P0-SYNTH.md`、`docs/PARAM.md`。

---

## 快速开始

```bash
cd /Volumes/Workspace/NFOCC
python3 -m nfocc_ph0.selftest          # 20 项自检，必须全 PASS
python3 -m nfocc_ph0.epipolar          # 极线几何表（定理验证）
python3 -m nfocc_ph0.voxel_grid        # 体素网格与 Hash
```

单模块调试：

```python
from nfocc_ph0.camera_kb import make_default_camera
from nfocc_ph0.view_lut import ViewParams, build_view_lut, warp
from nfocc_ph0.synth import generate, synth_selfcheck

cam = make_default_camera()            # f=550, 1920x1536, k=0
vp  = ViewParams(height=-1)            # 环带 10°-60°，高度自动算
lut = build_view_lut(cam, vp)
print(lut["stats"])                    # 各道有效性淘汰计数

meta = generate("/tmp/s", "S_BOX", "FV", move_mm=120)
print(meta["stats"])                   # 真值视差/深度分布
```

---

## 模块

| 文件 | 内容 | 规范 |
|---|---|---|
| `camera_kb.py` | KB 正/反投影、θ_d 单调性、Newton 反解 | `SPEC-COORD` §3 |
| `view_lut.py` | 视口 LUT（环带、四道有效性）、双线性重采样 | `SPEC-LUT` |
| `epipolar.py` | 极线方向定理、相机重定向、视口重排 | `SPEC-EPIPOLAR` |
| `voxel_grid.py` | 体素索引↔坐标、Hash 打包、柱体编号 | `SPEC-COORD` §5 |
| `synth.py` | 合成场景渲染、真值视差/深度、生成器自检 | `SPEC-P0-SYNTH` |
| `selftest.py` | 统一自检入口（含负对照） | `SPEC-VERIFY` |

**依赖**：仅 numpy。不需要 OpenCV / torch。

---

## 自检覆盖（20 项，含 4 个负对照）

| 编号 | 判据 | 结果 |
|---|---|---|
| V-CO-2 | KB 往返 < 0.01 px | 实测 **3.2e-13** |
| V-CO-3/4 | 光轴→主点、图像轴未翻转 | PASS |
| V-CO-6 | 非单调 k 被拒绝 | 负对照 PASS |
| V-CO-4b | y 轴朝上被检测 | 负对照 PASS |
| V-LUT | 环带 10°–60°、地面 404–3970 mm | PASS |
| V-LUT-3 | r_valid 收窄 → d2 淘汰 | PASS |
| V-LUT | height 与环带不符被拒绝 | 负对照 PASS |
| V-MASK-2 | 车身掩码生效 | PASS |
| V-EPI-1 | t_y=t_z=0 ⇒ 竖直视差 = 0 | 实测 0.00e+00 |
| V-EPI-2 | 竖直视差 ∝ t_z | 散布 **2.66%** |
| V-EPI-3 | 45° 运动四路全不可用 | PASS |
| V-EPI-4 | 重定向后 t_z = 0 | PASS |
| V-EPI-4b | 错误复合顺序被抓 | 负对照 PASS |
| V-VOX-1 | 75×60×18，Z 末格 100 mm | PASS |
| V-VOX-6 | 柱体编号 `iy*nx+ix`（非方阵） | PASS |
| V-VOX-7 | nx>256 时 Hash 报错 | 负对照 PASS |
| V-SYN-1..5 | 确定性、三角化闭合、量化、容量、非负 | 闭合 **2.1e-14**（n=1.10M） |

---

## 调试中确认的五个关键事实（都有代码依据，不是推论）

### 1. 极线角度反映的是"平移方向相对光轴"

直行时 **FV/RV = 90°**（平移沿光轴 → FoE 退化）、**ML/MR = 0°**（可用）；
横移时正好互补；45° 时四路全 90°。见 `python3 -m nfocc_ph0.epipolar` 的表。

我最初把 `t_cam.y` 单当成判据，导致全表为 0 —— 正确判据是
`atan2(hypot(t_y, t_z), |t_x|)`。

### 2. 前向平移的像移是**斜的**，不只是水平

实测某点：像素 `(960.5, 1200.5)` 在 120 mm 前移后变成 `(960.56, 1252.01)`，
**竖直移动 51.5 px**，与水平分量同量级。原因：物点在前下方，相机前移改变俯视角。

后果：真值必须存 `(du, dv)`。只存 `du` 无法重建对应点，三角化验证会假 FAIL。

### 3. 前向平移不能用 `Z = f·b/d`

该式要求平移垂直于视线。前向平移时：

| 同一点 | `Z=f·b/d` 相对误差 |
|---|---|
| `\|u−cx\| ∈ [0,100)` | **10.3** |
| `\|u−cx\| ∈ [300,600)` | 0.49 |
| `\|u−cx\| ∈ [600,960)` | 0.095 |

必须用射线三角化（最小二乘），它对任意平移方向成立。

### 4. 重定向的复合顺序：`R' = R_cam @ R_cv`

`x_new` 是**相机系**向量。直接写进 `R'` 的行等于把它当**车辆系**，A2 断言立刻失败。
这是本项目第四次踩坐标约定，`V-EPI-4b` 专门做负对照。

### 5. 视口 `height` 必须由垂直张角反推

`height=384` + `f_v=277` 得到实际垂直张角 69.6°，而不是环带的 50°。
传 `height=-1` 自动算（本例 258 行）。

---

## 已知未做（属于 PH-1 及以后）

| 项 | 归属 |
|---|---|
| 视差引擎（SGBM / 模型） | PH-1 |
| 三角化 + σ 传播（批量实现） | PH-1 |
| 基线状态机 | PH-2 |
| 体素 CMA 融合、zMap | PH-3 |
| 车身掩码标定（现在只有接口与合成测试） | 依赖实车数据 |
| 视口重排的实图验证（`reorient_view_R` 已实现但合成测试只覆盖固定视口） | PH-1 |
