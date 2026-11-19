# SPEC-CONVENTION — 坐标系与 t_cam 语义（唯一权威）

本文件定义跨模块共用的坐标约定。**其他地方只引用，不重定义。**

---

## 1. 三个坐标系

| 名字 | 符号 | 定义 |
|---|---|---|
| 车体 | V | ISO 8855：原点=后轴中心地面投影，X 前、Y 左、Z 上 |
| 相机 | C | OpenCV：x 右、y 下、z 前（光轴） |
| 虚拟视口 | S | 虚拟相机：pitch=0、roll=0，只有 yaw；x 右、y 下、z 沿 yaw 方向 |

关系：

```
v_V = R · v_C + t         NFPose（R 的**列**是相机轴在车系下的方向）
v_S = Rz(-yaw) · v_V      虚拟视口系 = 车系绕 Z 轴反向转 yaw
```

### 1.1 为什么 Y 必须向左

ISO 8855 是右手系：`X × Y = Z`。X 向前、Z 向上时，Y 只能向左；写
"X 前 / Y 右 / Z 上"会得到左手系（前 × 右 = 下），与 ISO 8855 矛盾。
后果不是注释错，而是 MVL/MVR 与 ML/MR 视口整体翻面、外参 Y 符号、
掩码和镜像判据全反。

hanqtech 标定提供了独立证据：`00165_MVL.json` 的 translation y=+0.957 m，
`00166_MVR.json` 为 −0.957 m。MVL 在 +Y，即 +Y = 左。

本项目按 **X 前、Y 左、Z 上** 实现；若上游/实车确实使用 Y 右，必须先做
一次全链路坐标迁移（外参、viewplan yaw、D1 掩码、SGBM mirror、测试），
不能只改文档或某一个符号。回归测试见 `test_camera.cpp::convention_viewplan_yaw_signs`
与 `test_hanqtech.cpp::ws_ego_y_axis_left`。


---

## 2. t_cam 语义（本节就是 G2 的结论）

### 2.1 定义

```
t_cam[c] = **相机 c 的视口系 (S) 下**，当前帧光心相对参考帧光心的位移
         = ( Δt_V·right, 0, Δt_V·forward )          单位 mm
```

其中 `right=(sin yaw, -cos yaw, 0)`、`forward=(cos yaw, sin yaw, 0)`
（`view_lut.cpp::pixel_to_ray_impl` 的基），`yaw` 为视口车系绝对 yaw。

**即：t_cam 用视口系表达，不是相机系、不是车系。**

### 2.2 为什么是视口系

三角化、门控、SGBM 镜像判断都在同一基下做。`nf_view_pixel_to_ray` 返回的是
**车系**方向，`t_cam` 是视口系分量；两者用 §2.4 的投影/反投影互转。若 t 用别的
坐标系，`|t_perp|/|t|` 与射线就不在同一基下，判据和深度都会错。

实测教训：早期 `t_cam` 一个实现给车系、一个给相机系，门控偏转后仍报 FORWARD、45° 偏转解出负深度（`s0 = −364.6`）。

### 2.3 符号

`t_cam = P_now − P_ref`（**当前减参考**）。

由此参考帧光心相对当前帧的位置是 `−t_cam`，三角化里写作 `c1 = −t_cam`。写反会让物点落到相机后方（负深度）。

### 2.4 上游来源（生产环境）

上游给 `NFEgoMotion`（车体系位姿 + 速度）。换算：

```
Δt_V   = pos_now − pos_ref                    车系位移
t_cam  = ( Δt_V·right, 0, Δt_V·forward )
       = ( Δt_x·sin(yaw) − Δt_y·cos(yaw), 0,
           Δt_x·cos(yaw) + Δt_y·sin(yaw) )
```

**不要写成 `Rz(-yaw)ᵀ·Δt_V`**。`view_lut.cpp::pixel_to_ray_impl` 的基是
`f=(cos,sin)`、`r=(sin,-cos)`、`d=(0,0,-1)`，`(r,d,f)` 不是右手排列；
`Rz` 会把 yaw=0 的前向位移放到车体 z，把 yaw=90 的前向放到车体 y。6 视口
必须用上面的投影。反变换：

```
Δt_x = t_x·sin(yaw) + t_z·cos(yaw)
Δt_y = -t_x·cos(yaw) + t_z·sin(yaw)
```

`NFFrameInput` 新增 `t_veh[3]`（车体系位移）。pipeline 对每个视口用上式投影；
`t_veh` 全 0 时回退到 `t_cam`（录包/旧调用方）。

`nf_view_pixel_to_ray` 返回的已经是**车系**方向，前向/立体三角化都不要再乘
`pose->R`。乘了等于把 FV 的 +X 光轴转成 −Y（实测前视表面中位 309→305 mm，
且点云方向错）。

### 2.5 合成源与真实数据源

| 源 | 给什么 | 换算到 t_cam |
|---|---|---|
| `SyntheticSource` | 车系位移 (move,0,0) 或 (0,0,move) | 按 yaw 转到视口系；同时填 `t_veh` |
| `hanqtechSource` | `ego_distance` × 符号 → 车系 (move,0,0) | 按 yaw 转到视口系；同时填 `t_veh=(move,0,0)` |
| `SceneSource` | 同合成源 | 同上；填 `t_veh` |

**输入源填 `t_cam` 时完成到该路视口系的换算；6 视口另填 `t_veh`。**
pipeline 对 6 视口按每个视口的 yaw 把 `t_veh` 投影成 `t_view`；
4 视口且 `t_veh` 全 0 时回退到 `t_cam`。

---

## 3. 雷达式核查清单

改到这几处时逐条对照：

- [ ] 新输入源填 `t_cam` 时，已转到**该路**的视口系（每路 yaw 不同）
- [ ] 三角化 `c1 = −t_cam`
- [ ] 门控 `nf_gate_decide` 收到的是视口系的 t；6 视口由 `t_veh` 投影得到
- [ ] `view_pixel_dir` 已是车系，前向/立体都不再乘 `pose->R`
- [ ] SGBM `mirror` 判据读当前视口的 `t_view[0]`（横向分量）
- [ ] `nf_forward_range` / `nf_triangulate_rays` 的射线与 t 在同一基
