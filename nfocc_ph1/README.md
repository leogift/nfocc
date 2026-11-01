# nfocc_ph1 — 引擎、三角化与产品链路（Python 参考实现）

PH-0 几何内核之上的第二层：从两帧图像产出带不确定度的 3D 点云。
**纯 Python/NumPy + OpenCV**，供原理调试与算法验证，不是产品代码。

```bash
cd /Volumes/Workspace/NFOCC
python3 -m nfocc_ph1.selftest --report out/ph1_report.json   # 18 PASS / 0 FAIL / 3 SKIP
python3 out/probe_product.py                                  # 五工况端到端
```

---

## 核心结论（先说这个）

**纯前向平移不能用 SGBM。** 实测三条：像移方向左右翻转（`±38.2 px`）、
像移是斜的（`|dv|/|du| = 0.23`）、同一行极线不平行（**175.9°**）。
任何"按行单向搜索"的匹配器结构上不适用，调参无效。

**解法是径向膨胀解析测距**：纯平移时物点只做径向移动，由对应点可直接解出距离，
不需要极线平行。见 `forward_range.py`，实测精度 `7.8e-15`。

---

## 模块

| 文件 | 作用 | 规范 |
|---|---|---|
| `engine.py` | 视差引擎接口 + SGBM + Null 骨架 | `SPEC-ENGINE` |
| `triangulate.py` | 射线最小二乘三角化 + σ 传播 | `SPEC-EPIPOLAR` §4 |
| `forward_range.py` | **前向径向膨胀解析测距** | `SPEC-FORWARD` §2 |
| `gating.py` | 方向门控 + 基线状态机 | `SPEC-FORWARD` §4 |
| `blockmatch.py` | 分块方向立体（默认关闭，见下） | `SPEC-FORWARD` §5.3 |
| `product.py` | 端到端产品管线 | — |
| `pipeline.py` | 实验管线（PH-1 早期，保留对比） | — |
| `selftest.py` | 统一自检（含 4 个负对照） | `PH1-ACCEPTANCE` |

---

## 产品管线用法

```python
import numpy as np
from nfocc_ph1.product import run_product

# 直行 120mm
r = run_product("S_MULTI", "FV", t_veh=np.array([120., 0, 0]))
print(r.mode.value, r.points.n_total, r.metrics["dist_median_mm"])
# -> FORWARD 247680 857.0

# 侧移 120mm
r = run_product("S_MULTI", "FV", t_veh=np.array([0., 120., 0]))
# -> STEREO 53456 1446.0
```

`run_product` 返回 `ProductResult(mode, points, metrics)`。
`points` 为 `None` 表示**冻结态**（基线不足/超限/丢帧），不是"没有障碍物"。

---

## 门控规则

```
σ = |t_perp| / |t|          # t_perp = (t_x, t_y)，垂直于光轴的分量

gap > GAP_MAX      -> FROZEN_GAP      不产出
B   < B_MIN        -> FROZEN_NEAR     不产出
B   > B_MAX        -> FROZEN_FAR      不产出
σ   ≥ 0.80         -> STEREO          整幅 SGBM
σ   ≤ 0.35         -> FORWARD         径向膨胀解析测距
否则               -> HYBRID          两条路都跑
```

---

## 精度与灵敏度（这是下一阶段的硬指标）

前向测距的误差**全部**来自对应点精度，解析式本身无损：

| 对应点精度 | 深度误差中位 | p90 | 对 200mm 体素 |
|---|---|---|---|
| 1.0 px | 110 mm | 457 mm | 勉强 |
| 0.5 px | **55 mm** | 232 mm | 满足 |

**要做到体素精度，对应点必须 ≤0.5 px（亚像素）。**

---

## 自检覆盖（21 项，含 4 个负对照）

| 组 | 项 | 状态 |
|---|---|---|
| 引擎契约 | V-ENG-1/2/6/7 | PASS |
| 三角化 | V-TRI-1/3/4 | PASS（闭合 1.4e-12） |
| 分半匹配 | V-SPLIT-1 | PASS |
| 门控 | V-GATE-1 | PASS |
| 前向测距 | V-FR-1/2 | PASS（7.8e-15，灵敏度线性 0.49） |
| 产品链路 | V-PROD-1/2/3 | PASS |
| 负对照 | N-1..N-4 | PASS（全部正确失败） |
| SGBM 前向精度 | V-ENG-3/5 | **SKIP**（见下） |

### 为什么有 3 个 SKIP 而不是 FAIL

`V-ENG-3`（精度 ≤1px）、`V-ENG-5`（端到端 3D 精度）描述的是
**"只用 SGBM 做前向"的能力上限**（实测 16.0 px），而 PH-1 的结论正是这条路走不通。
产品路径已改用解析测距绕过。**保留 SKIP 是为了记录那条路的代价，不删。**

---

## 调试中踩的坑（都写进了代码注释）

1. **`Z` 是距离 `|q₀|` 不是光轴深度 `q₀.z`** —— 混用解出 393 而真值 921。
2. **输入必须是方向向量不是像素** —— 视口的 `v` 定义为"相对环带中轴仰角"，
   与标准针孔 `f·y/z` 差 **208 px**。
3. **合成纹理不能有随机项** —— 否则同一 3D 点两帧灰度不同，
   SGBM 输出 3.6 px 而真值 133 px（"成功返回"但全错）。
4. **配对方向由 `t_cam.x` 符号决定** —— 写反 100% 得负深度。
5. **块宽必须 > numDispatities** —— 否则 OpenCV 报 `Insufficient memory`（负尺寸分配）。

---

## 未完成

| 项 | 阻塞 |
|---|---|
| **对应点提取** | 前向测距现用几何真值，需实现光流/特征匹配 |
| 侧向残余误差 9–77 px | 极线不平行，分块经实测不划算（`SPEC-FORWARD` §5.3） |
| 真实数据验证 | 需标定与录包 |
