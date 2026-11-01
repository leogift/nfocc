#!/usr/bin/env python
"""三种视口方案俯视对比图。

车体: 5000x2000mm, 后轴中心为原点(车系 x 前 y 左)。
鱼眼: f=550, 水平视场 ±100°, 四角 ±128°。
视口: 水平 ±60°, 环带俯角 10-60°。

## 绘图约定
- 每个视口画成从相机光心出发的**扇形**, 张角 ±60°
- 扇形半径按"视口可见地面最远点"画, 即环带远端 60° 俯角对应的距离
- 实线 = 该方案独有的视口; 虚线 = 与基准方案相同
"""
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文字体: 系统自带 Hiragino Sans GB / Arial Unicode MS
plt.rcParams["font.sans-serif"] = ["Hiragino Sans GB", "Arial Unicode MS",
                                   "Heiti SC", "STHeiti", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
from matplotlib.patches import Rectangle, Wedge, Circle, FancyArrowPatch
import numpy as np

# ---------------- 几何参数 ----------------
VEH_LEN, VEH_WID = 5000.0, 2000.0
REAR_TO_FRONT = 3500.0          # 后轴到前端
CAM_Z = 700.0                   # 相机安装高度
EL_MIN, EL_MAX = 10.0, 60.0     # 视口环带俯角
VP_HALF = 60.0                  # 视口水平半角

# 相机位置 (车系, mm): x 前, y 左
CAMS = {
    "FV": (REAR_TO_FRONT, 0.0),
    "RV": (-(VEH_LEN - REAR_TO_FRONT), 0.0),
    "ML": (0.0, VEH_WID / 2),
    "MR": (0.0, -VEH_WID / 2),
}
# 光轴方位角 (deg, ISO8855, 0=+X 前, 逆时针正)
CAM_YAW = {"FV": 0.0, "RV": 180.0, "ML": 90.0, "MR": -90.0}

# 视口可见地面的最远距离 = CAM_Z / tan(el_min)
GROUND_FAR = CAM_Z / math.tan(math.radians(EL_MIN))
GROUND_NEAR = CAM_Z / math.tan(math.radians(EL_MAX))


def to_xy(x, y):
    """车系 -> 绘图坐标 (matplotlib 直接用 x, y)"""
    return x, y


def draw_vehicle(ax):
    """车身框 (后轴为原点)"""
    x0 = -(VEH_LEN - REAR_TO_FRONT)
    ax.add_patch(Rectangle((x0, -VEH_WID / 2), VEH_LEN, VEH_WID,
                           fill=True, facecolor="#f0f0f0",
                           edgecolor="#333333", lw=2, zorder=5))
    # 车头标记
    ax.annotate("", xy=(REAR_TO_FRONT + 600, 0), xytext=(REAR_TO_FRONT, 0),
                arrowprops=dict(arrowstyle="-|>", lw=2.5, color="#d62728"),
                zorder=6)
    # 后轴中心
    ax.plot(0, 0, "o", ms=9, mfc="white", mec="#333333", mew=2, zorder=7)
    ax.annotate("后轴中心\n(车系原点)", (0, 0), textcoords="offset points",
                xytext=(0, -30), ha="center", fontsize=8, color="#333333")


def draw_view(ax, cam, offset_deg, color, label=None, dashed=False,
              alpha_fill=0.16, lw=1.6):
    """画一个视口的俯视扇形"""
    cx, cy = CAMS[cam]
    az = CAM_YAW[cam] - offset_deg          # 车系方位角
    lo, hi = az - VP_HALF, az + VP_HALF

    # 扇形: 从相机光心出发
    w = Wedge((cx, cy), GROUND_FAR, lo, hi,
              facecolor=color, alpha=alpha_fill, edgecolor="none", zorder=3)
    ax.add_patch(w)

    # 边界线
    for a in (lo, hi):
        ax.plot([cx, cx + GROUND_FAR * math.cos(math.radians(a))],
                [cy, cy + GROUND_FAR * math.sin(math.radians(a))],
                color=color, lw=lw, ls="--" if dashed else "-",
                alpha=0.9, zorder=4)
    # 中轴 (视口中心方向)
    ax.plot([cx, cx + GROUND_FAR * math.cos(math.radians(az))],
            [cy, cy + GROUND_FAR * math.sin(math.radians(az))],
            color=color, lw=lw + 0.6, ls="--" if dashed else "-",
            alpha=0.95, zorder=4)

    # 相机位置
    ax.plot(cx, cy, "s", ms=5, color=color, mec="black", mew=0.5, zorder=8)

    if label:
        r = GROUND_FAR * 0.62
        lx = cx + r * math.cos(math.radians(az))
        ly = cy + r * math.sin(math.radians(az))
        ax.text(lx, ly, label, fontsize=9, ha="center", va="center",
                color=color, fontweight="bold", zorder=9,
                bbox=dict(boxstyle="round,pad=0.2", fc="white",
                          ec=color, alpha=0.85, lw=0.8))


def setup_ax(ax, title, sub):
    ax.set_xlim(-3200, 4200)
    ax.set_ylim(-4300, 4300)
    ax.set_aspect("equal")
    ax.grid(True, ls=":", lw=0.5, alpha=0.5)
    ax.axhline(0, color="#999999", lw=0.6, zorder=1)
    ax.axvline(0, color="#999999", lw=0.6, zorder=1)
    ax.set_xlabel("车系 x (mm, 前 +)", fontsize=9)
    ax.set_ylabel("车系 y (mm, 左 +)", fontsize=9)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.text(0.5, -0.13, sub, transform=ax.transAxes, ha="center",
            fontsize=8.5, color="#444444")
    ax.tick_params(labelsize=7)


def main():
    fig, axes = plt.subplots(1, 3, figsize=(21, 7.6))

    # ============ 方案 A: 原有 4 视口 ============
    ax = axes[0]
    setup_ax(ax, "方案 A: 原有 4 视口",
             "每个相机 1 个视口, 中轴即光轴 (FV/RV 偏转 0°)\n"
             "直行时 FV/RV 平移沿光轴 → 横向基线 = 0 → 视差深度消失")
    draw_vehicle(ax)
    for cam, c in [("FV", "#d62728"), ("RV", "#9467bd"),
                   ("ML", "#1f77b4"), ("MR", "#2ca02c")]:
        draw_view(ax, cam, 0.0, c, label=cam)
    # 标注问题
    ax.annotate("直行: 平移沿此轴\n横向基线 = 0",
                xy=(REAR_TO_FRONT, 0), xytext=(REAR_TO_FRONT - 300, -2200),
                fontsize=8.5, color="#d62728",
                arrowprops=dict(arrowstyle="->", color="#d62728", lw=1.4),
                bbox=dict(boxstyle="round,pad=0.3", fc="#fff0f0",
                          ec="#d62728", lw=1))

    # ============ 方案 B: ±30° ============
    ax = axes[1]
    setup_ax(ax, "方案 B: 用户建议 ±30°",
             "FV/RV 各 2 个视口 (±30°), ML/MR 不变 → 共 6 个\n"
             "偏 30° 后横向基线 = T·sin30° = 0.5T")
    draw_vehicle(ax)
    draw_view(ax, "ML", 0.0, "#1f77b4", label="ML (0°)")
    draw_view(ax, "MR", 0.0, "#2ca02c", label="MR (0°)")
    for off, a in [(-30, 0.32), (+30, 0.32)]:
        draw_view(ax, "FV", off, "#d62728",
                  label=f"FV {off:+d}°", alpha_fill=a)
        draw_view(ax, "RV", off, "#9467bd",
                  label=f"RV {off:+d}°", alpha_fill=a)
    ax.annotate("两个偏转视口的\n重叠区落在正前方边缘",
                xy=(REAR_TO_FRONT + 900, 0), xytext=(REAR_TO_FRONT - 2800, 2700),
                fontsize=8.5, color="#d62728",
                arrowprops=dict(arrowstyle="->", color="#d62728", lw=1.4),
                bbox=dict(boxstyle="round,pad=0.3", fc="#fff0f0",
                          ec="#d62728", lw=1))

    # ============ 方案 C: 0° + 45° ============
    ax = axes[2]
    setup_ax(ax, "方案 C: 建议改为 0° + 45°",
             "FV/RV 保留 0° (前向解析测距, 已验证 0.045px)\n"
             "再加 45° 视口 → 横向基线 = T·sin45° = 0.707T")
    draw_vehicle(ax)
    draw_view(ax, "ML", 0.0, "#1f77b4", label="ML (0°)")
    draw_view(ax, "MR", 0.0, "#2ca02c", label="MR (0°)")
    # 0° 用浅色虚线表示保留
    for cam, c in [("FV", "#d62728"), ("RV", "#9467bd")]:
        draw_view(ax, cam, 0.0, c, label=f"{cam} 0°", dashed=True,
                  alpha_fill=0.07, lw=1.2)
    for off in (-45, +45):
        draw_view(ax, "FV", off, "#d62728", label=f"FV {off:+d}°",
                  alpha_fill=0.34)
        draw_view(ax, "RV", off, "#9467bd", label=f"RV {off:+d}°",
                  alpha_fill=0.34)
    ax.annotate("0° 视口继续走前向解析测距\n(远场精度最好)",
                xy=(REAR_TO_FRONT + 700, 300), xytext=(REAR_TO_FRONT - 3000, -2900),
                fontsize=8.5, color="#555555",
                arrowprops=dict(arrowstyle="->", color="#555555", lw=1.3),
                bbox=dict(boxstyle="round,pad=0.3", fc="#f7f7f7",
                          ec="#888888", lw=1))

    # ---------- 三个方案的数值对比表 ----------
    txt = (
        "关键数值对比 (基线 T=200mm, 视口 f=277.13 px/rad)\n"
        "─────────────────────────────────────────────────────────────\n"
        "方案        视口数  FV 直行横向基线   Z=3m 视差   Z=9m 视差   前向测距\n"
        "A 原有        4        0 mm            0.00 px     0.00 px    ✓ 已用\n"
        "B ±30°        6      100 mm            9.24 px     3.08 px    ✗ 被替代\n"
        "C 0°+45°      6      141 mm           13.07 px     4.36 px    ✓ 保留"
    )
    fig.text(0.5, 0.035, txt, ha="center", va="bottom", fontsize=9.5,
             family="Hiragino Sans GB",
             bbox=dict(boxstyle="round,pad=0.5", fc="#fafafa",
                       ec="#666666", lw=1))

    plt.tight_layout(rect=[0, 0.13, 1, 1])
    out = "/Volumes/Workspace/NFOCC/docs/viewplan_compare.png"
    plt.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
    print("saved:", out)

    # 同时输出数值表供核对
    print("\n数值核对:")
    f = 277.13
    for name, off in [("A 原有", None), ("B ±30°", 30), ("C 0°+45°", 45)]:
        if off is None:
            bp = 0.0
        else:
            bp = 200 * math.sin(math.radians(off))
        d3 = f * bp / 3000
        d9 = f * bp / 9000
        print(f"  {name:10s} 横向基线 {bp:6.1f} mm, Z=3m 视差 {d3:6.2f} px, "
              f"Z=9m 视差 {d9:5.2f} px")


if __name__ == "__main__":
    main()
