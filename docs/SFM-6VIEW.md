# SFM 6 视口：左右针孔 + 前后成对 90°

## 结论

pipeline 已从“每相机一个视口”改为“每个虚拟视口一套 LUT/缓冲”，一次编排 6 个理想针孔：

| 视口 | 相机 | yaw | 张角 | 尺寸 | f (px/rad) | 匹配路径 |
|---|---|---|---|---|---|---|
| FV -45 | 0 FV | -45° | 110° | 590×716 | 206.6 | 径向（极点在边缘） |
| FV +45 | 0 FV | +45° | 110° | 590×716 | 206.6 | 径向 |
| RV -135 | 1 RV | -135° | 110° | 590×716 | 206.6 | 径向 |
| RV +135 | 1 RV | +135° | 110° | 590×716 | 206.6 | 径向 |
| ML +90 | 2 MVL | +90° | 150° | 1024×475 | 137.2 | SGBM |
| MR -90 | 3 MVR | -90° | 150° | 1024×475 | 137.2 | SGBM |

前后两组中心相隔 90°。张角取 110° 而不是 90°，是因为 90° 时两个视口只在 0° 交于一点，正前方目标同时落在两个视口的边缘；110° 留 20° 重叠。解析场景实测：竖面中位 1213 mm（90°）→ 791 mm（110°）。

真实 hanqtech 帧 164、5 km/h、278 mm 基线：

| 模式 | 点 | 有效帧 | 水平距均值 | z<-100 mm |
|---|---:|---:|---:|---:|
| 4 视口（默认） | 1,999 | 179.4 ms | 5,854 mm | 247 (12.4%) |
| 6 视口 `--sfm6` | 2,019 | 104.6 ms | 5,009 mm | 233 (11.5%) |

6 视口更快是因为默认 4 视口仍用 sample=2/coarse=1.0，而 `--sfm6` 自动用 sample=4/coarse=4.0；点数和近场覆盖没有下降，水平距均值反而更小。

解析场景真值（前向 FV+RV，1–5 m）：

| 指标 | 4 视口（150° 单视口/相机） | 6 视口（±45° 对） |
|---|---:|---:|
| 点 | 806 | 1,793 |
| 1–5 m 点 | 248 | 368 |
| 表面中位 | 402 mm | **354 mm** |
| 竖面中位 | 410 mm | **382 mm** |
| 地下点 | 132 | 199 |

侧向 ML+MR 在两模式下逐项相同（720 点、表面 672 mm、竖面 718 mm），说明 6 视口改造没有改动侧视链路。

回归：21 个 ctest 目标通过 20 个，180 个用例 / 2,361 次断言；`depth_accuracy` 的 6 个用例中 4 个旧验收用例仍失败。新增 `test_pipeline::pipe_six_view_plan`（12 条断言）验证 6 视口映射与路径，`test_camera::convention_viewplan_yaw_signs` 增加 RV ±135 与 110/150 张角断言。

## 为什么这样建视口

纯前向平移下，单个朝前的视口横向基线为 0，SGBM 结构性失效；前后各偏 ±45° 后，每个视口拿到的横向分量是 `T·sin45°=0.707T`，两个视口合起来仍覆盖前向。左右视口必须单个覆盖整侧，所以保持 150°。

虚拟相机的基向量（`view_lut.cpp::pixel_to_ray_impl`）：

```
forward = ( cos yaw,  sin yaw, 0)   -> 视口 +z
right   = ( sin yaw, -cos yaw, 0)   -> 视口 +x
down    = ( 0, 0, -1)               -> 视口 +y
```

注意 `(right, down, forward)` 不是右手排列，不能直接用 `Rz(±yaw)` 转。车体位移 `t_V` 投影到视口：

```
t_x = t_V·right =  t_x^V·sin(yaw) - t_y^V·cos(yaw)
t_z = t_V·forward = t_x^V·cos(yaw) + t_y^V·sin(yaw)
```

反变换回 `t_V`：

```
t_x^V = t_x·sin(yaw) + t_z·cos(yaw)
t_y^V = -t_x·cos(yaw) + t_z·sin(yaw)
```

历史上这里写成 `Rz(+yaw)`，在 yaw=0 时把前向位移放到车体 z（上），在 ±45° 时方向错 45°。6 视口必须用上面的投影/反投影，不能用 `Rz`。

## 极点与匹配路径

纯平移下所有对应点都在以极点为中心的射线上。极点由 `t_view` 投影：

```
u_e = cx + f · t_x / t_z
v_e = cy + f · t_y / t_z
```

- `|t_z|/|t| > 0.25`（前后 ±45/±135、正视前向）：极点在有限位置，用径向匹配，FoE 取 `(u_e, v_e)`，不用图像中心。
- `|t_z|/|t| ≈ 0`（左右 ±90）：极点在无穷远，极线近水平，才用 SGBM。

这条判据替代了“σ 横向分量”判据：±45° 的 σ=0.707，按旧判据会被送进 SGBM，但它的极线是 45° 斜线，SGBM 的水平搜索结构性错配。实测 6 视口在旧判据下水平距均值 5.36 m、地下 16.3%；改成极点判据后降到 3.53 m、12.5%，再修掉前向路径的 `pose->R` 二次旋转后到 5.00 m、11.5%（真实数据）。

前向路径还有一处必须与立体路径一致：`nf_view_pixel_to_ray` 返回的已经是**车系**方向，不能再乘 `pose->R`；乘了等于把 FV 的 +X 光轴转成 −Y。修复后前视 90° 分段表面中位 **307/305/460/857 mm**（1–2/2–3/3–4/4–5 m）；只修 `tt`、仍保留 `pose->R` 二次旋转时是 309/433/773/781 mm。近段 1.3% 的改善有限，远段变化是点集分布变化，不是精度提升。旧报告里的 223/222/367/408 mm 是在前向 `t`/射线基不一致的代码下测的，不能当作当前基线。

## 代码改动

- `NFPipelineConfig`：新增 `view_plan[6]`、`n_views`；`n_views=0` 保持旧 4 视口行为。
- `NFFrameInput`：新增 `t_veh[3]`（车体系位移）。三个分量全 0 时回退 `t_cam`，兼容录包/旧调用方。
- `NFFrameOutput`：新增 `n_views`、`view_mode[6]`、`view_status[6]`、`num_points_per_view[6]`、`view_cam[6]`；`mode/cam_status/num_points_per_cam` 保持按相机聚合。
- `NFPipeline`：`NFViewState views[6]`（LUT/当前/参考/有效位/视差），前向匹配缓冲按最大视口分配，SGBM 引擎全 pipeline 一个、串行复用。
- `nfocc_run --sfm6`：装 6 视口表；`--forward-sample`、`--forward-coarse` 可覆盖默认 4/4。
- `NF_V_VIEW_HFOV_DEG_FRONT_PAIR=110`：前后成对视口的张角；`NF_V_VIEW_HFOV_DEG_FRONT=90` 仍是旧 4 视口/前视 90° 测试用值。
- 测试：`test_pipeline::pipe_six_view_plan` 校验 6 视口映射与前向/立体路径；
  `test_camera::convention_viewplan_yaw_signs` 增加 RV ±135 与 110/150 分档张角断言。
- 输入源（synthetic/scene/hanqtech）填 `t_veh`；`file_io` 无 `t_veh` 时回退 `t_cam`。

## 限制

1. 前视 90°/6 视口的前向精度仍未达到 150 mm@1–2 m；6 视口把前向竖面误差从 410 降到 382 mm，但绝对值仍大。
2. 前向径向搜索上限仍是 64 px（近场 0.5 m 需要更大），继续放大到 256 px 会在周期纹理上引入假峰：前视 90° 表面中位 223→499 mm（旧几何口径）。近场覆盖与误匹配的取舍还没解决。
3. 地面以下点 11.5% 仍在；6 视口没有单独修它。
4. RV 两视口在真实数据上只出 27/188 个插入点（后向场景弱纹理），覆盖贡献小；这是数据/场景问题，不是 LUT 失败。
5. 15 km/h、833 mm 基线下，前向 110° 视口的径向搜索范围仍受 64 px 限制；侧视 SGBM d_max=228 px，256 容量够用。
6. 真实数据无点云真值，以上是链路/几何一致性证据，不是绝对精度验收。
7. `nfocc_viz` 的 6 视口仍是旧的全局 1024 宽实现，FV/RV 成对视口会 LUT 参数不匹配；
   6 视口验收用 `probe_sfm6` 或 `nfocc_run --sfm6`，viz 未重构。

## 复现

```bash
cd /Volumes/Workspace/NFOCC/cpp
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Release \
      -DNF_WITH_VCAM=ON -DNF_BUILD_TESTS=ON
cmake --build build/host -j8

# 6 视口真实数据
./build/host/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164 \
    --frames 2 --stats --bench --sfm6

# 4 vs 6 解析场景真值
./build/host/probe_sfm6

# 回归（20/21；depth_accuracy 的 4 个旧验收用例仍失败）
ctest --test-dir build/host --output-on-failure
```
