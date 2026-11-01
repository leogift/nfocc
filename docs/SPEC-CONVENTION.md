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

---

## 2. t_cam 语义（本节就是 G2 的结论）

### 2.1 定义

```
t_cam[c] = **相机 c 的视口系 (S) 下**，当前帧光心相对参考帧光心的位移
         = R_Sᵀ · (P_cam,now − P_cam,ref)          单位 mm
```

其中 `R_S = Rz(yaw_c)`，`yaw_c = cfg.view_yaw_deg[c]`。

**即：t_cam 用视口系表达，不是相机系、不是车系。**

### 2.2 为什么是视口系

三角化、门控、SGBM 镜像判断都在视口系里做（射线是视口系射线）。若 t 用别的坐标系，`|t_perp|/|t|` 与射线就不在同一基下，判据和深度都会错。

实测教训：早期 `t_cam` 一个实现给车系、一个给相机系，门控偏转后仍报 FORWARD、45° 偏转解出负深度（`s0 = −364.6`）。

### 2.3 符号

`t_cam = P_now − P_ref`（**当前减参考**）。

由此参考帧光心相对当前帧的位置是 `−t_cam`，三角化里写作 `c1 = −t_cam`。写反会让物点落到相机后方（负深度）。

### 2.4 上游来源（生产环境）

上游给 `NFEgoMotion`（车体系位姿 + 速度）。换算：

```
Δt_V   = pos_now − pos_ref                    车系位移
t_cam  = Rz(-yaw_c)ᵀ · (R_camᵀ · Δt_V)        = Rz(-yaw_c)ᵀ · Δt_V
```

第二等号成立是因为视口系直接建在车系上（虚拟相机），与真实相机外参无关。

### 2.5 合成源与真实数据源

| 源 | 给什么 | 换算到 t_cam |
|---|---|---|
| `SyntheticSource` | 车系位移 (move,0,0) 或 (0,0,move) | 按 yaw 转到视口系 |
| `hanqtechSource` | `ego_distance` × 符号 → 车系 (move,0,0) | 按 yaw 转到视口系 |
| `SceneSource` | 同合成源 | 同上 |

**所有源必须在填入 `NFFrameInput.t_cam` 前完成到视口系的换算。** pipeline 不再做坐标转换。

---

## 3. 雷达式核查清单

改到这几处时逐条对照：

- [ ] 新输入源填 `t_cam` 时，已转到**该路**的视口系（每路 yaw 不同）
- [ ] 三角化 `c1 = −t_cam`
- [ ] 门控 `nf_gate_decide` 收到的是视口系的 t
- [ ] SGBM `mirror` 判据读 `t_cam[0]`（视口系横向分量）
- [ ] `nf_forward_range` / `nf_triangulate_rays` 的射线与 t 在同一基
