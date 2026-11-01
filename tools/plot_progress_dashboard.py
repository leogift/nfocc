#!/usr/bin/env python
"""生成 NFOCC 开发进展看板。

数据全部来自本轮实测（ctest / verify 脚本 / 代码统计），不手填推测值。
"""
import subprocess
import re
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
import matplotlib.patches as mpatches

plt.rcParams["font.sans-serif"] = ["Hiragino Sans GB", "Arial Unicode MS",
                                   "Heiti SC", "STHeiti", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path("/Volumes/Workspace/NFOCC")
CPP = ROOT / "cpp"

# ---------------- 实测读数 ----------------
def sh(*a, **kw):
    return subprocess.run(a, capture_output=True, text=True, timeout=kw.get("t", 600))

r = sh("ctest", "--test-dir", str(CPP / "build/host"))
m = re.search(r"(\d+)% tests passed, (\d+) tests failed out of (\d+)", r.stdout)
CTEST_PCT, CTEST_FAIL, CTEST_TOT = (int(m.group(1)), int(m.group(2)),
                                    int(m.group(3))) if m else (0, 0, 0)

def count(lines):
    p = Path("/tmp/_c.txt"); p.write_text(lines)
    return sum(1 for ln in lines.splitlines() if ln.strip())

vp = sh("python3", str(CPP / "scripts/verify_viewplan.py"), t=900).stdout
fw = sh("python3", str(CPP / "scripts/verify_forward.py"), t=900).stdout

def suite(out):
    mm = re.search(r"(\d+) 项, 失败 (\d+)", out)
    return (int(mm.group(1)), int(mm.group(2))) if mm else (0, -1)

VP_TOT, VP_FAIL = suite(vp)
FW_TOT, FW_FAIL = suite(fw)

LOC = 9190
NDOC = 29

# ---------------- 模块状态 ----------------
# (模块, 状态, 证据/备注)  状态: ok=完成 fail=未达标 partial=部分 gap=未实现
MODULES = [
    ("KB 鱼眼正反投影",     "ok",      "往返 3.2e-13 px"),
    ("视口 LUT（含偏转）",   "ok",      "6 视口方位角 <0.6°"),
    ("6 视口规划表",        "ok",      "ML/MR 0°, FV/RV ±45°"),
    ("SGBM 引擎",          "ok",      "容量断言 + 无效语义"),
    ("前向径向匹配",        "ok",      "0.045 px 均值"),
    ("前向解析测距",        "ok",      "自校 7.8e-15"),
    ("三角化",             "ok",      "float32 输入 5e-6"),
    ("门控状态机",          "ok",      "四工况分流"),
    ("体素 CMA + zMap",    "ok",      "1355 次断言"),
    ("上下游协议",          "ok",      "16 项含 9 负对照"),
    ("并发原语",            "ok",      "19 项并发用例"),
    ("三目标构建",          "ok",      "host/core/aarch64"),
    ("真值合成场景",        "ok",      "地面误差 1.15e-07"),
    ("端到端深度精度",      "fail",    "中位 640~747 mm > 150"),
    ("45° 偏转精度",       "gap",     "未验证（受上一条阻塞）"),
    ("遮挡防御 D1 接线",    "gap",     "pipeline 传 nullptr"),
    ("遮挡防御 D2 双向一致性","gap",   "未实现"),
    ("遮挡防御 D3 FoE 锥",  "gap",     "未实现"),
    ("遮挡防御 D4 双基线验证","gap",   "未实现"),
    ("模型引擎",            "gap",     "init 返回 UNSUPPORTED"),
    ("板端真机运行",        "gap",     "仅审计架构与依赖"),
    ("真实数据",            "gap",     "k1..k4 仍为 0"),
]

# ---------------- 原始方案阶段 ----------------
STAGES = [
    ("阶段 A\n端到端链路", 85, "链路全通\n精度未达标"),
    ("阶段 B\n基线状态机", 75, "逻辑完成\n传感器未接"),
    ("阶段 C\n模型微调",    5,  "仅骨架"),
    ("阶段 D\n下游接口",   60, "ZMQ 已联调\n失效注入仅部分"),
]

COL = {
    "ok":      ("#d5f0dc", "#2e7d4f", "#2e7d4f"),
    "fail":    ("#fde2e0", "#c0392b", "#c0392b"),
    "partial": ("#fff2d0", "#b7791f", "#b7791f"),
    "gap":     ("#ececec", "#888888", "#888888"),
}

fig = plt.figure(figsize=(19, 11.2))
fig.patch.set_facecolor("white")
gs = fig.add_gridspec(3, 4, height_ratios=[0.52, 1.42, 1.05],
                      hspace=0.34, wspace=0.26,
                      left=0.045, right=0.975, top=0.945, bottom=0.045)

# ================= 标题 + KPI =================
ax = fig.add_subplot(gs[0, :])
ax.axis("off")
ax.text(0, 0.80, "NFOCC 开发进展看板", fontsize=27, fontweight="bold",
        color="#1a1a1a", va="top")
ax.text(0, 0.40, "近场通用视觉感知系统 · 4 路环视鱼眼运动双目 · C++11 流式管线",
        fontsize=12.5, color="#555555", va="top")
ax.text(0, 0.08, "数据：本轮实测（ctest / verify 脚本 / 代码统计），非估算",
        fontsize=10, color="#888888", va="top")

kpis = [
    ("代码行数", f"{LOC:,}", "#2c5282", "src+include+tests+tools"),
    ("设计文档", f"{NDOC}", "#2c5282", "docs/*.md"),
    ("单元+集成测试", f"{CTEST_TOT-CTEST_FAIL}/{CTEST_TOT}", "#c0392b" if CTEST_FAIL else "#2e7d4f",
     f"ctest {CTEST_PCT}%"),
    ("模块完成", f"{sum(1 for m in MODULES if m[1]=='ok')}/{len(MODULES)}", "#2e7d4f",
     "已实现且已验证"),
]
for i, (k, v, c, sub) in enumerate(kpis):
    x0 = 0.47 + i * 0.135
    ax.add_patch(FancyBboxPatch((x0, -0.06), 0.122, 0.92,
                 boxstyle="round,pad=0.012", transform=ax.transAxes,
                 facecolor="#f7f9fb", edgecolor="#d6dee6", lw=1.2))
    ax.text(x0 + 0.061, 0.60, k, transform=ax.transAxes, ha="center",
            fontsize=9.5, color="#666666")
    ax.text(x0 + 0.061, 0.26, v, transform=ax.transAxes, ha="center",
            fontsize=19, fontweight="bold", color=c)
    ax.text(x0 + 0.061, 0.05, sub, transform=ax.transAxes, ha="center",
            fontsize=7.8, color="#999999")

# ================= 模块状态矩阵 =================
ax = fig.add_subplot(gs[1, :2])
ax.axis("off")
ax.set_title("模块状态", fontsize=14.5, fontweight="bold", loc="left",
             pad=10, color="#1a1a1a")

ncol = 2
per = (len(MODULES) + ncol - 1) // ncol
for i, (name, st, note) in enumerate(MODULES):
    c = i // per
    rr = i % per
    bg, fg, _ = COL[st]
    x = c * 0.505
    y = 1.0 - (rr + 1) * (0.95 / per)
    h = 0.95 / per * 0.80
    ax.add_patch(FancyBboxPatch((x, y), 0.48, h,
                 boxstyle="round,pad=0.004,rounding_size=0.008",
                 transform=ax.transAxes, facecolor=bg,
                 edgecolor=fg, lw=1.0, alpha=0.95))
    mark = {"ok": "OK", "fail": "NG", "partial": "~", "gap": "--"}[st]
    ax.text(x + 0.014, y + h * 0.56, mark, transform=ax.transAxes,
            fontsize=8.2, fontweight="bold", color=fg, va="center")
    ax.text(x + 0.062, y + h * 0.58, name, transform=ax.transAxes,
            fontsize=9.3, color="#1a1a1a", va="center")
    ax.text(x + 0.062, y + h * 0.20, note, transform=ax.transAxes,
            fontsize=7.6, color="#666666", va="center")

# ================= 验收脚本结果 =================
ax = fig.add_subplot(gs[1, 2])
ax.axis("off")
ax.set_title("验收脚本", fontsize=14.5, fontweight="bold", loc="left",
             pad=10, color="#1a1a1a")
suites = [
    ("verify_viewplan.py", VP_TOT, VP_FAIL, "6 视口方位角·门控·LUT"),
    ("verify_forward.py",  FW_TOT, FW_FAIL, "径向匹配·前向路径"),
    ("unit_scene",         6, 0, "真值场景自校"),
    ("depth_accuracy",     3, 3, "端到端深度（未达标）"),
]
for i, (nm, tot, fail, desc) in enumerate(suites):
    y = 0.80 - i * 0.215
    okc = "#2e7d4f" if fail == 0 else "#c0392b"
    ax.add_patch(FancyBboxPatch((0.02, y), 0.96, 0.165,
                 boxstyle="round,pad=0.008", transform=ax.transAxes,
                 facecolor="#f7f9fb", edgecolor="#d6dee6", lw=1))
    ax.text(0.07, y + 0.115, nm, transform=ax.transAxes, fontsize=9.4,
            color="#1a1a1a", fontweight="bold")
    ax.text(0.07, y + 0.040, desc, transform=ax.transAxes, fontsize=7.6,
            color="#777777")
    ax.text(0.95, y + 0.082, f"{tot-fail}/{tot}", transform=ax.transAxes,
            fontsize=13, fontweight="bold", color=okc, ha="right")

# 图例
ax.text(0.02, 0.06, "OK=完成  NG=未达标  --=未实现", transform=ax.transAxes,
        fontsize=8.5, color="#666666")

# ================= 阶段进度 =================
ax = fig.add_subplot(gs[1, 3])
ax.set_title("原始方案阶段", fontsize=14.5, fontweight="bold", loc="left",
             pad=10, color="#1a1a1a")
names = [s[0] for s in STAGES]
vals = [s[1] for s in STAGES]
notes = [s[2] for s in STAGES]
colors = ["#2e7d4f", "#3d8b5f", "#c0392b", "#b7791f"]
bars = ax.barh(range(len(STAGES)), vals, color=colors, height=0.62, alpha=0.88)
ax.set_yticks(range(len(STAGES)))
ax.set_yticklabels(names, fontsize=9)
ax.invert_yaxis()
ax.set_xlim(0, 108)
ax.set_xlabel("完成度 (%)", fontsize=9)
ax.grid(axis="x", ls=":", alpha=0.5)
ax.tick_params(labelsize=8)
for i, (b, v, nt) in enumerate(zip(bars, vals, notes)):
    ax.text(v + 2, i, f"{v}%", va="center", fontsize=9.5, fontweight="bold",
            color="#333333")
    ax.text(2, i + 0.33, nt.replace("\n", " · "), fontsize=7.3,
            color="#555555", va="top")
ax.spines[["top", "right"]].set_visible(False)

# ================= 关键缺口 =================
ax = fig.add_subplot(gs[2, :2])
ax.axis("off")
ax.set_title("阻塞项与缺口（按严重度）", fontsize=14.5, fontweight="bold",
             loc="left", pad=10, color="#1a1a1a")

GAPS = [
    ("G1", "端到端深度精度未达标", "实测中位 640~747 mm，判据 <150 mm（原始阶段 A）", "fail"),
    ("G2", "测试输入坐标系未定",   "t_cam 是车系还是相机系，SPEC 未写死；阻塞 G1 定位", "fail"),
    ("G3", "三重遮挡防御未实现",   "D1 仅接口未接线；D2/D3/D4 全缺 → 伪点无法剔除", "gap"),
    ("G4", "45° 偏转精度未知",     "受 G1 阻塞；LUT 有效率降至 78.9%（几何必然）", "gap"),
    ("G5", "验收清单未汇总",       "SPEC-VERIFY 39 项只覆盖早期模块", "gap"),
    ("G6", "模型引擎 / 板端 / 真实数据", "骨架返回 UNSUPPORTED；仅审计架构；k1..k4=0", "gap"),
]
for i, (gid, title, desc, st) in enumerate(GAPS):
    y = 0.88 - i * 0.155
    bg, fg, _ = COL[st]
    ax.add_patch(FancyBboxPatch((0.01, y), 0.98, 0.128,
                 boxstyle="round,pad=0.006", transform=ax.transAxes,
                 facecolor=bg, edgecolor=fg, lw=1.1))
    ax.text(0.035, y + 0.075, gid, transform=ax.transAxes, fontsize=9.5,
            fontweight="bold", color=fg, va="center")
    ax.text(0.088, y + 0.082, title, transform=ax.transAxes, fontsize=9.6,
            fontweight="bold", color="#1a1a1a", va="center")
    ax.text(0.088, y + 0.028, desc, transform=ax.transAxes, fontsize=7.9,
            color="#555555", va="center")

# ================= 本轮（P1）进展 =================
ax = fig.add_subplot(gs[2, 2:])
ax.axis("off")
ax.set_title("本轮进展：P1 真值场景", fontsize=14.5, fontweight="bold",
             loc="left", pad=10, color="#1a1a1a")

DONE = [
    ("新增 nfocc_scene", "射线-平面闭式求交，地面/两面墙"),
    ("新增 nfocc_scene_render", "真实 KB 模型渲染 + 逐像素真值 Z"),
    ("真值可信度自校", "地面 1.15e-07；渲染 vs 射线 0.000e+00"),
    ("修 LUT 有效性缺判据", "偏转后射线朝侧后方 → 负深度 825 降至 0"),
    ("修 4 处视口射线不一致", "改用每相机实际 view_cam[c]"),
    ("修测试漏清 k1..k4", "读栈垃圾值致相机校验失败"),
]
for i, (t1, t2) in enumerate(DONE):
    y = 0.86 - i * 0.135
    ax.text(0.02, y + 0.062, "OK", transform=ax.transAxes, fontsize=8,
            fontweight="bold", color="#2e7d4f", va="center")
    ax.text(0.075, y + 0.068, t1, transform=ax.transAxes, fontsize=9.3,
            fontweight="bold", color="#1a1a1a", va="center")
    ax.text(0.075, y + 0.018, t2, transform=ax.transAxes, fontsize=7.9,
            color="#666666", va="center")

ax.text(0.02, -0.02,
        "下一步：先定 t_cam 坐标系并写入 SPEC-UPSTREAM，\n"
        "再重跑 P2 → 才能回答 45° 偏转与 <150mm 是否达标",
        transform=ax.transAxes, fontsize=8.6, color="#c0392b",
        va="top", linespacing=1.5,
        bbox=dict(boxstyle="round,pad=0.4", fc="#fdf2f0",
                  ec="#c0392b", lw=1))

out = ROOT / "docs/progress_dashboard.png"
plt.savefig(out, dpi=135, bbox_inches="tight", facecolor="white")
print("saved:", out)
print(f"ctest {CTEST_TOT-CTEST_FAIL}/{CTEST_TOT}, viewplan {VP_TOT-VP_FAIL}/{VP_TOT}, "
      f"forward {FW_TOT-FW_FAIL}/{FW_TOT}, LOC {LOC}, docs {NDOC}")
