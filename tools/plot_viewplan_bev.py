#!/usr/bin/env python
"""BEV 俯视图：6 个虚拟视口的地面覆盖范围。

## 用途
一眼看出视口之间是否有缝隙、覆盖重叠多少、哪些方位没有视口。

## 几何
虚拟相机 pitch=0、roll=0，只有 yaw。光心高 H。视口垂直张角 vfov
给出**绝对俯角**范围 [-(vfov/2), +(vfov/2)]（pitch=0 时视口中心水平）。

地面可见距离：z = H / tan(俯角)
  俯角 = vfov/2  -> 最近可见距离
  俯角 = 0        -> 无限远（地平线）
所以每个视口在 BEV 上是一条**扇形**，内半径 = 最近距离，外半径截断到网格边界。

    python tools/plot_viewplan_bev.py
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
# 中文字体：DejaVu Sans 无 CJK，不指定会全是方框
for _fjn in ("PingFang SC", "Hiragino Sans GB", "STHeiti", "Songti SC"):
    try:
        matplotlib.font_manager.findfont(_fjn, fallback_to_default=False)
        plt.rcParams["font.family"] = _fjn
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False
import numpy as np
from matplotlib.patches import Polygon, Rectangle, Wedge

OUT = Path("/Volumes/Workspace/NFOCC/docs")

# 安装高度与网格（与 nfocc_default.h 一致）
H_CAM = 700.0        # mm
GRID_X = 9000.0      # ±9 m
GRID_Y = 7500.0      # ±7.5 m
VEH_L, VEH_W = 5000.0, 2000.0
REAR_TO_FRONT = 3500.0

# 视口规划（分类张角）
V = 120.0
VIEWS = [
    # tag,          cam, yaw,   hfov,  color,           linestyle
    ("FV -45°",     0,  -45.0,  90.0, "#e74c3c", "-"),
    ("FV +45°",     0,   45.0,  90.0, "#e67e22", "-"),
    ("RV -135°",    1, -135.0,  90.0, "#27ae60", "-"),
    ("RV +135°",    1,  135.0,  90.0, "#16a085", "-"),
    ("ML +90°",     2,   90.0, 150.0, "#8e44ad", "-"),
    ("MR -90°",     3,  -90.0, 150.0, "#c0392b", "--"),
]
CAM_POS = {
    0: (REAR_TO_FRONT, 0.0),
    1: (REAR_TO_FRONT - VEH_L, 0.0),
    2: (REAR_TO_FRONT - VEH_L * 0.5, VEH_W * 0.5),
    3: (REAR_TO_FRONT - VEH_L * 0.5, -VEH_W * 0.5),
}


def build(fname, pre_vfov_fn, title_suffix=""):
    fig, ax = plt.subplots(figsize=(11, 9.6), dpi=130)
    ax.set_facecolor("#fbfbfd")

    Rmax = math.hypot(GRID_X, GRID_Y)

    for name, cam, yaw, hfov, col, ls in VIEWS:
        s = VIEWS.index((name, cam, yaw, hfov, col, ls))
        vfov = pre_vfov_fn(cam)
        near = H_CAM / math.tan(math.radians(vfov / 2.0)) / 1000.0   # m
        ox, oy = CAM_POS[cam]
        ox, oy = ox / 1000.0, oy / 1000.0

        a0 = yaw - hfov / 2.0
        a1 = yaw + hfov / 2.0
        th = np.linspace(math.radians(a0), math.radians(a1), 120)

        # 外弧（截到网格边界）
        outer = []
        for t in th:
            dx, dy = math.cos(t), math.sin(t)
            # 与网格边求交，取最小 t
            ts = []
            if abs(dx) > 1e-9:
                for b in (GRID_X / 1000.0 - ox, -(GRID_X / 1000.0) - ox):
                    k = b / dx
                    y = oy + k * dy
                    if k > 0 and -GRID_Y / 1000.0 <= y <= GRID_Y / 1000.0:
                        ts.append(k)
            if abs(dy) > 1e-9:
                for b in (GRID_Y / 1000.0 - oy, -(GRID_Y / 1000.0) - oy):
                    k = b / dy
                    x = ox + k * dx
                    if k > 0 and -GRID_X / 1000.0 <= x <= GRID_X / 1000.0:
                        ts.append(k)
            k = min(ts) if ts else Rmax
            outer.append((ox + k * dx, oy + k * dy))

        # 内弧
        inner = []
        for t in th:
            inner.append((ox + near * math.cos(t), oy + near * math.sin(t)))

        poly = Polygon(inner + outer[::-1], closed=True, facecolor=col,
                       alpha=0.20, edgecolor=col, linewidth=1.6,
                       linestyle=ls, zorder=2)
        ax.add_patch(poly)

        # 中轴虚线
        am = math.radians(yaw)
        k = Rmax * 0.55
        ax.plot([ox, ox + k * math.cos(am)], [oy, oy + k * math.sin(am)],
                color=col, lw=1.0, ls=":", alpha=0.85, zorder=3)

        # 标注：放在扇区中轴、近距与远距的中点
        lm = near + (Rmax * 0.42 - near) * 0.55
        lx, ly = ox + lm * math.cos(am), oy + lm * math.sin(am)
        ax.annotate(name, (lx, ly), fontsize=9.5, fontweight="bold",
                    color=col, ha="center", va="center", zorder=6,
                    bbox=dict(boxstyle="round,pad=0.22", fc="white",
                              ec=col, alpha=0.9, lw=1.0))
        # 最近可见距离标注
        nx = ox + near * math.cos(am) * 0.82
        ny = oy + near * math.sin(am) * 0.82
        ax.plot([nx], [ny], marker="|", ms=8, color=col, zorder=5)

    # 车体
    ax.add_patch(Rectangle(xy=((REAR_TO_FRONT - VEH_L) / 1000.0, -VEH_W / 2000.0),
                           width=VEH_L / 1000.0, height=VEH_W / 1000.0,
                           facecolor="#2c3e50", edgecolor="none",
                           alpha=0.85, zorder=8))
    ax.text(REAR_TO_FRONT / 1000.0 - VEH_L / 2000.0, 0, "车体", color="white",
            fontsize=11, ha="center", va="center", zorder=9, fontweight="bold")

    # 相机位置
    for c, (px, py) in CAM_POS.items():
        ax.plot(px / 1000.0, py / 1000.0, "s", ms=7, color="#111",
                zorder=10, markeredgecolor="white", markeredgewidth=1.2)

    ax.set_xlim(-GRID_X / 1000.0, GRID_X / 1000.0)
    ax.set_ylim(-GRID_Y / 1000.0, GRID_Y / 1000.0)
    ax.set_aspect("equal")
    ax.grid(True, ls=":", lw=0.6, color="#c8ccd4", alpha=0.9, zorder=1)
    ax.axhline(0, color="#9aa3ae", lw=0.9, zorder=1)
    ax.axvline(0, color="#9aa3ae", lw=0.9, zorder=1)
    ax.set_xlabel("X (m)  车头方向 →", fontsize=11)
    ax.set_ylabel("Y (m)  车左方向 →", fontsize=11)

    ax.set_title("NFOCC 六视口 BEV 地面覆盖" + title_suffix, fontsize=15,
                 fontweight="bold", pad=14)

    # 图例
    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], color=c, lw=8, alpha=0.45, label=n)
               for n, _, _, _, c, _ in VIEWS]
    ax.legend(handles=handles, loc="upper left", fontsize=9, ncol=2,
              framealpha=0.95, title="视口", title_fontsize=10)

    txt = ("虚拟相机 pitch=0/roll=0；内弧为最近可见距离 H/tan(vfov/2)，"
           "外弧截断到 ±9m×±7.5m 网格\n"
           "黑色方块 = 相机光心；点线 = 视口中轴")
    fig.text(0.5, 0.015, txt, ha="center", fontsize=9, color="#5a6a80")

    fig.tight_layout(rect=(0.0, 0.035, 1.0, 1.0))
    fig.savefig(OUT / fname, facecolor="white")
    plt.close(fig)
    print(f"{OUT / fname}")


def main():
    # 当前实现：前后 90°，左右 150°，全部 vfov=120°
    build("viewplan_bev_current.png", lambda cam: V,
          "（当前参数：前后 90°×120°，左右 150°×120°）")

    # 建议方案：前后 90°×60°，左右 150°×120°
    build("viewplan_bev_proposed.png",
          lambda cam: 60.0 if cam in (0, 1) else 120.0,
          "（建议参数：前后 90°×60°，左右 150°×120°）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
