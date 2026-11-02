#!/usr/bin/env python3
"""生成 NFOCC 报告的 SVG 图与 PDF：进展看板 / 工程架构 / 数据流 / 算法逻辑。

## 为什么自己画 SVG
架构图与流程图是**结构化的**，用 Python 生成 SVG 比手画或截图可维护：
改一处文字不用重画，且能内联进 HTML 保持矢量文字（可搜索、可选中）。

## 为什么内联 <svg> 而不是 <img src="data:...">
Chromium 会把 `<img>` 引用的 SVG **栅格化成位图**，图内文字不进文本层
（实测搜不到 "接入层"），PDF 体积还会膨胀到 9 MB。内联 `<svg>` 保持矢量，
文字可选中、体积降一个数量级。

注入必须发生在 pandoc **之后** —— markdown 里直接写 `<svg>` 会被
raw_html 白名单剥掉。

用法:
    python3 tools/build_arch_report.py
"""
import base64
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path("/Volumes/Workspace/NFOCC")
OUT = ROOT / "docs" / "report"
EDGE = "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge"

FONT = '"PingFang SC","Hiragino Sans GB",sans-serif'

C_BG = "#ffffff"
C_BOX = "#f4f7fb"
C_DONE = "#e8f5e9"      # 已完成
C_PART = "#fff8e1"      # 部分
C_TODO = "#fdecea"      # 未做
C_BORDER = "#5a6b7d"
C_OK = "#2e7d32"
C_WARN = "#e65100"
C_ERR = "#c62828"


# ---------------------------------------------------------------- 绘图原语

def svg_open(w, h):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
            f'width="{w}" height="{h}">'
            f'<rect width="{w}" height="{h}" fill="{C_BG}"/>'
            f'<style>text{{font-family:{FONT};}}</style>')


def svg_close():
    return "</svg>"


def txt(x, y, s, size=13, color="#233", anchor="start", bold=False):
    w = ' font-weight="600"' if bold else ""
    return (f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" '
            f'text-anchor="{anchor}"{w}>{s}</text>')


def box(x, y, w, h, label, sub="", fill=C_BOX, border=C_BORDER, fs=13):
    s = (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="5" '
         f'fill="{fill}" stroke="{border}" stroke-width="1.5"/>')
    s += txt(x + w / 2, y + h / 2 + (0 if not sub else -7), label,
             fs, "#1a2b3c", "middle", True)
    if sub:
        s += txt(x + w / 2, y + h / 2 + 11, sub, fs - 2, "#556", "middle")
    return s


def arrow(x1, y1, x2, y2, label="", color="#456", dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    s = (f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" '
         f'stroke-width="1.8" marker-end="url(#ar)"{d}/>')
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        s += txt(mx, my - 5, label, 11, color, "middle")
    return s


DEFS = ('<defs><marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" '
        'markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
        '<path d="M 0 0 L 10 5 L 0 10 z" fill="#456"/></marker></defs>')


# ---------------------------------------------------------------- 图 1 看板

def fig_dashboard():
    """开发进展看板：按模块列状态，颜色区分完成度。"""
    W, H = 1180, 760
    rows = [
        ("相机模型", "KB + 径向多项式，vcam 桥接 0.117/0.190 px 对齐", "done"),
        ("视口 LUT", "6 视口，左右 150°×1024 / 前后 90°×590，四道淘汰计数", "done"),
        ("SGBM 视差", "已接主链，35.6 ms/帧（单相机串行）", "done"),
        ("前向径向测距", "解析解，误差 0.000 mm（合成验证）", "done"),
        ("三角化 / 体素", "Hash + CMA，RSS 162 MB（预算 291 MB）", "done"),
        ("车身掩码 D1", "5000×2000 矩形占位，负对照 661→195 点", "done"),
        ("FoE 锥过滤 D3", "已接主链，pipeline 两处调用", "done"),
        ("σ 传播 / 落库", "已接主链，σ<10cm 占 66%", "done"),
        ("ZMQ 零拷贝", "接收零拷贝 + 发送所有权转移，8 用例", "done"),
        ("网格哈希 + DBSCAN", "150k 点 45 ms（纯网格哈希）", "done"),
        ("动静分离", "位姿补偿 + 点级速度 + 簇级投票，负对照守住", "done"),
        ("双基线状态机", "6 用例通过，但 pipeline 仍用旧门控", "part"),
        ("里程计积分", "speed+yaw_rate 中点航向法，圆弧误差 0.05%", "done"),
        ("D2 双向一致性", "函数在，主链 0 处调用（耗时 1.8 倍）", "part"),
        ("D4 交叉验证", "函数在，需双基线晋升 / 真实定位", "part"),
        ("在线自校准", "方案已定（地平面单应 + IMU），未实现", "todo"),
        ("精度达标 150mm@5m", "需 σ_d≤0.34px，实测 14-42px，未证实", "todo"),
        ("aarch64 上板", "产物已出，未在 RK3588/M57 运行", "todo"),
        ("Neon/SIMD 优化", "全库无 SIMD 代码", "todo"),
        ("RKNN 模型后端", "onnxruntime 通，NPU 未接", "todo"),
    ]
    s = svg_open(W, H) + DEFS
    s += txt(30, 38, "NFOCC 开发进展看板", 22, "#1a2b3c", bold=True)
    s += txt(30, 60, "2026-11-02  ·  代码 7900 行  ·  ctest 17/18  ·  文档 40 篇全绿",
             13, "#667")

    color = {"done": (C_DONE, C_OK, "已完成"),
             "part": (C_PART, C_WARN, "部分"),
             "todo": (C_TODO, C_ERR, "未做")}

    # 图例
    lx = 780
    for k, (_, bc, label) in color.items():
        s += (f'<rect x="{lx}" y="24" width="14" height="14" rx="3" '
              f'fill="{color[k][0]}" stroke="{bc}" stroke-width="1.5"/>')
        s += txt(lx + 20, 36, label, 12, "#445")
        lx += 95

    # 四列布局
    cols = 2
    per = (len(rows) + cols - 1) // cols
    bw, bh = 545, 58
    for i, (name, desc, st) in enumerate(rows):
        cx = 30 + (i // per) * (bw + 40)
        cy = 90 + (i % per) * (bh + 8)
        fill, bc, _ = color[st]
        s += (f'<rect x="{cx}" y="{cy}" width="{bw}" height="{bh}" rx="6" '
              f'fill="{fill}" stroke="{bc}" stroke-width="1.5"/>')
        s += txt(cx + 14, cy + 22, name, 14, "#1a2b3c", bold=True)
        s += txt(cx + 14, cy + 42, desc, 11.5, "#556")

    s += txt(30, H - 16,
             "「部分」= 代码已实现并通过单测，但主链未接入或依赖缺失的外部数据。"
             "这类最容易被误报为完成。", 11.5, "#777")
    return s + svg_close()


# ---------------------------------------------------------------- 图 2 架构

def fig_arch():
    W, H = 1180, 620
    s = svg_open(W, H) + DEFS
    s += txt(30, 34, "NFOCC 工程架构", 20, "#1a2b3c", bold=True)

    # 四层
    layers = [
        ("应用层", 70, 95, [
            ("nfocc_run", "命令行运行"),
            ("nfocc_viz", "中间结果可视化"),
            ("nfocc_disp_probe", "视差探针"),
            ("nfocc_disturb_gen", "外参扰动生成"),
        ]),
        ("主链层", 190, 95, [
            ("pipeline", "6 视口调度 + 门控"),
            ("input_source", "文件 / hanqtech / ZMQ"),
            ("frame_pool", "帧缓冲池"),
        ]),
        ("算法层", 310, 95, [
            ("lut", "视口 LUT"),
            ("geometry", "三角化/门控/FoE/径向"),
            ("engine", "SGBM / 模型双引擎"),
            ("voxel", "Hash + CMA 融合"),
        ]),
        ("基础设施", 430, 95, [
            ("camera", "KB / 径向多项式"),
            ("cluster", "网格哈希 + DBSCAN"),
            ("baseline", "双基线 + 里程计"),
            ("io", "文件 / ZMQ 零拷贝"),
        ]),
    ]

    for label, y, _, items in layers:
        s += (f'<rect x="24" y="{y - 22}" width="1132" height="76" rx="8" '
              f'fill="#fafbfd" stroke="#dde3ea" stroke-width="1"/>')
        s += txt(40, y + 20, label, 14, "#5a6b7d", bold=True)
        n = len(items)
        iw = min(250, (1080 - (n - 1) * 16) / n)
        sx = 150
        for i, (nm, sub) in enumerate(items):
            s += box(sx + i * (iw + 16), y - 10, iw, 52, nm, sub)

    # 层间箭头
    for y in (148, 268, 388):
        s += arrow(590, y, 590, y + 20, "", "#8aa")

    # 右侧：数据流向
    s += (f'<rect x="24" y="520" width="1132" height="80" rx="8" '
          f'fill="#f0f7ff" stroke="#9cc" stroke-width="1.2"/>')
    s += txt(40, 546, "数据流", 14, "#2a5d8f", bold=True)
    flow = ["4 路鱼眼 1280×966", "视口 LUT", "SGBM 视差", "三角化", "体素融合", "点云 + zMap"]
    fw = 160
    for i, f in enumerate(flow):
        fx = 60 + i * (fw + 22)
        s += (f'<rect x="{fx}" y="558" width="{fw}" height="30" rx="15" '
              f'fill="#ffffff" stroke="#2a5d8f" stroke-width="1.5"/>')
        s += txt(fx + fw / 2, 578, f, 12, "#1a3d5f", "middle", True)
        if i < len(flow) - 1:
            s += arrow(fx + fw + 2, 573, fx + fw + 20, 573, "", "#2a5d8f")

    return s + svg_close()


# ---------------------------------------------------------------- 图 3 Pipeline

def fig_pipeline():
    W, H = 1180, 700
    s = svg_open(W, H) + DEFS
    s += txt(30, 34, "数据流 Pipeline（单帧 200 ms 预算）", 20, "#1a2b3c", bold=True)

    # 主循环
    steps = [
        ("输入", "4 路 Y 通道\n1280×966", 30, 80),
        ("视口投影", "LUT 构建\n6 视口", 210, 80),
        ("视差匹配", "SGBM\n35.6 ms/相机", 390, 80),
        ("三角化", "射线最小二乘\n或前向解析", 570, 80),
        ("体素融合", "Hash + CMA\nσ 落库", 750, 80),
        ("输出", "点云 + zMap\n冻结也发", 930, 80),
    ]
    for nm, sub, x, y in steps:
        lines = sub.split("\n")
        s += box(x, y, 160, 66, nm, "")
        s += txt(x + 80, y + 42, lines[0], 11, "#556", "middle")
        if len(lines) > 1:
            s += txt(x + 80, y + 56, lines[1], 11, "#556", "middle")
    for i in range(len(steps) - 1):
        s += arrow(steps[i][2] + 162, 113, steps[i + 1][2] - 2, 113, "", "#456")

    # 防御层（并行挂在主链上）
    s += txt(30, 205, "防御层（主链旁路，按状态开关）", 14, "#5a6b7d", bold=True)
    guards = [
        ("D1 车身掩码", "LUT 构建期剔除\n已接入", C_DONE, C_OK),
        ("D2 双向一致性", "反向匹配\n未接入", C_PART, C_WARN),
        ("D3 FoE 锥", "逐相机过滤\n已接入", C_DONE, C_OK),
        ("D4 交叉验证", "双基线互检\n未接入", C_PART, C_WARN),
        ("σ 传播", "逐点不确定度\n已接入", C_DONE, C_OK),
    ]
    for i, (nm, sub, fill, bc) in enumerate(guards):
        x = 30 + i * 226
        s += (f'<rect x="{x}" y="225" width="210" height="64" rx="6" '
              f'fill="{fill}" stroke="{bc}" stroke-width="1.5"/>')
        s += txt(x + 105, 248, nm, 13, "#1a2b3c", "middle", True)
        for j, ln in enumerate(sub.split("\n")):
            s += txt(x + 105, 266 + j * 14, ln, 11, "#556", "middle")
    s += arrow(590, 148, 590, 220, "旁路", "#8aa", "4 4")

    # 门控决策
    s += txt(30, 355, "门控与基线决策", 14, "#5a6b7d", bold=True)
    gate = [
        ("FROZEN_NEAR", "基线不足\nb < 278 mm", C_TODO),
        ("STEREO", "短/长基线\n278 ~ 833 mm", C_DONE),
        ("FROZEN_ROT", "航向漂移\n> 15°", C_TODO),
        ("FROZEN_FAR", "基线过大\nb > 833 mm", C_TODO),
    ]
    for i, (nm, sub, fill) in enumerate(gate):
        x = 30 + i * 285
        s += (f'<rect x="{x}" y="372" width="265" height="58" rx="6" '
              f'fill="{fill}" stroke="{C_BORDER}" stroke-width="1.2"/>')
        s += txt(x + 132, 394, nm, 12.5, "#1a2b3c", "middle", True)
        for j, ln in enumerate(sub.split("\n")):
            s += txt(x + 132, 412 + j * 13, ln, 10.5, "#556", "middle")

    # 时序预算
    s += txt(30, 490, "时序预算（实测，单相机串行 35.6 ms/帧）", 14, "#5a6b7d", bold=True)
    bars = [("视口 LUT", 8), ("SGBM", 18), ("三角化", 4), ("体素", 5), ("输出", 1)]
    total = sum(b[1] for b in bars)
    bx = 30
    for nm, v in bars:
        bw = v / total * 1000
        s += (f'<rect x="{bx}" y="505" width="{bw}" height="26" '
              f'fill="#cfe3f5" stroke="#3a6ea5" stroke-width="1"/>')
        if bw > 60:
            s += txt(bx + bw / 2, 523, f"{nm} {v}ms", 10.5, "#1a3d5f", "middle")
        bx += bw + 3
    s += txt(30, 552, f"合计约 {total} ms，占 200 ms 预算的 {total/200*100:.0f}%。"
                       f"6 视口全开需并行，否则线性放大。", 11.5, "#666")

    # 说明
    s += (f'<rect x="30" y="578" width="1120" height="94" rx="8" '
          f'fill="#fffbea" stroke="#e0c060" stroke-width="1.2"/>')
    s += txt(48, 602, "当前阻塞", 13, "#8a6d00", bold=True)
    notes = [
        "· depth_accuracy 失败：合成场景只有地面 + 一面墙，纹理规则周期，SGBM 锁错周期（墙面理论 140.8px 实测 2.2px）",
        "· 聚类受限：帧 164 仅 1256 点 → 82 簇、噪声 926、每簇 4 点。密度不足以聚出物体，根因同上",
        "· 150mm@5m 未证实：需 σ_d ≤ 0.34px，SGBM 典型值 0.3-0.5px，理论边界",
    ]
    for i, n in enumerate(notes):
        s += txt(48, 624 + i * 16, n, 11.5, "#6b5300")

    return s + svg_close()


# ---------------------------------------------------------------- 图 4 算法

def fig_algo():
    W, H = 1180, 640
    s = svg_open(W, H) + DEFS
    s += txt(30, 34, "算法逻辑：从鱼眼像素到体素", 20, "#1a2b3c", bold=True)

    # 左：视口投影
    s += txt(30, 72, "① 视口投影（虚拟相机）", 14, "#2a5d8f", bold=True)
    s += box(30, 88, 300, 120, "", "")
    lines = [
        "视口像素 (u,v)",
        "→ 视口系射线 d_view",
        "→ 车体系 d_veh = R_view · d_view",
        "→ 相机系 d_cam = R_camᵀ · d_veh",
        "→ 鱼眼投影 (u_fish, v_fish)",
        "→ 查表得原图坐标 + 有效性",
    ]
    for i, ln in enumerate(lines):
        s += txt(48, 114 + i * 18, ln, 11.5, "#334")

    # 中：三角化
    s += txt(360, 72, "② 三角化（射线最小二乘）", 14, "#2a5d8f", bold=True)
    s += box(360, 88, 340, 120, "", "")
    tri = [
        "当前帧射线 u_hat（相机系）",
        "参考帧射线 v_hat + 基线 t",
        "Z = Σ((u×v)·(t×v)) / Σ‖u×v‖²",
        "粗差判据：残差 > τ 则剔除",
        "前向平移时改用径向膨胀解析解",
    ]
    for i, ln in enumerate(tri):
        s += txt(378, 114 + i * 20, ln, 11.5, "#334")

    # 右：体素
    s += txt(730, 72, "③ 体素融合", 14, "#2a5d8f", bold=True)
    s += box(730, 88, 420, 120, "", "")
    vox = [
        "体素索引 (ix,iy,iz) = floor(p / 200mm)",
        "哈希表开放寻址，空槽探测",
        "CMA 更新：存在概率 p ← p·(1-w) + w",
        "σ 传播：深度误差 → 空间不确定度 (cm)",
        "同体素多次观测取加权质心",
    ]
    for i, ln in enumerate(vox):
        s += txt(748, 114 + i * 20, ln, 11.5, "#334")

    # 下：聚类
    s += txt(30, 258, "④ 聚类与动静分离", 14, "#2a5d8f", bold=True)
    cl = [
        ("网格哈希", "格子 (ix,iy,iz)\n→ 点列表 CSR", C_DONE),
        ("DBSCAN", "eps=120mm\nminPts=3", C_DONE),
        ("点级速度", "本帧→上帧\n最近邻位移/dt", C_DONE),
        ("簇级投票", "动态点占比\n> 50% 判动态", C_DONE),
    ]
    for i, (nm, sub, fill) in enumerate(cl):
        x = 30 + i * 290
        s += (f'<rect x="{x}" y="274" width="270" height="66" rx="6" '
              f'fill="{fill}" stroke="{C_OK}" stroke-width="1.5"/>')
        s += txt(x + 135, 298, nm, 13, "#1a2b3c", "middle", True)
        for j, ln in enumerate(sub.split("\n")):
            s += txt(x + 135, 316 + j * 14, ln, 11, "#556", "middle")
        if i < len(cl) - 1:
            s += arrow(x + 272, 307, x + 288, 307, "", C_OK)

    # 实测数据
    s += txt(30, 392, "实测精度（合成场景，5 km/h，b=278mm）", 14, "#5a6b7d", bold=True)
    data = [("前 -45°", "41.6 px"), ("前 +45°", "28.9 px"), ("后 -135°", "27.6 px"),
            ("后 +135°", "15.1 px"), ("左 90°", "15.3 px"), ("右 -90°", "14.1 px")]
    for i, (nm, v) in enumerate(data):
        x = 30 + i * 190
        s += (f'<rect x="{x}" y="408" width="175" height="46" rx="5" '
              f'fill="#fdf3e7" stroke="#e0a060" stroke-width="1.2"/>')
        s += txt(x + 87, 428, nm, 12, "#1a2b3c", "middle", True)
        s += txt(x + 87, 446, v, 11.5, "#a05000", "middle")
    s += txt(30, 476, "判据 0.6 px，实测差 20-70 倍。但这是**纹理伪影**不是算法能力 ——"
                      "规则周期纹理让 SGBM 锁错周期，见 PROBE-RESULTS.md。",
             11.5, "#a05000")

    # 关键约定
    s += (f'<rect x="30" y="496" width="1120" height="120" rx="8" '
          f'fill="#f0f7ff" stroke="#9cc" stroke-width="1.2"/>')
    s += txt(48, 520, "关键约定（易错点）", 13, "#2a5d8f", bold=True)
    conv = [
        "· 环带角是**绝对俯角**：hanqtech MVL 光轴下俯 74°，当相对角处理会看到 0.43 m 地面而非 1-4 m",
        "· 位姿补偿方向：P_prev = Rᵀ·P_cur + t（不是减 t）。负对照守住：不给位姿时静止墙报 3080 mm/s",
        "· SGBM 输出恒为正：反向视差不能取负，符号在引擎层已丢",
        "· 车身掩码按原图像素索引，尺寸必须与实际图像一致（曾硬编 1920 而实际 1280，stride 全错）",
    ]
    for i, c in enumerate(conv):
        s += txt(48, 544 + i * 17, c, 11.5, "#1a3d5f")

    return s + svg_close()


# ---------------------------------------------------------------- 构建

def b64_img(svg_text, mime="image/svg+xml"):
    payload = base64.b64encode(svg_text.encode("utf-8")).decode("ascii")
    return f"data:{mime};base64,{payload}"


def png_data_uri(path: Path):
    """位图内嵌为 data URI。jpeg 用 image/jpeg，否则浏览器不认。"""
    mime = "image/jpeg" if path.suffix.lower() in (".jpg", ".jpeg") else "image/png"
    payload = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime};base64,{payload}"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    figs = {
        "DASH": fig_dashboard(),
        "ARCH": fig_arch(),
        "PIPE": fig_pipeline(),
        "ALGO": fig_algo(),
    }
    for k, v in figs.items():
        (OUT / f"fig_{k.lower()}.svg").write_text(v, encoding="utf-8")
    print(f"生成 {len(figs)} 张 SVG: {', '.join(figs)}")

    md = OUT / "ARCH-REPORT.md"
    html = OUT / "ARCH-REPORT.html"
    pdf = OUT / "NFOCC-工程解析报告.pdf"

    r = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "md2html.py"), str(md), str(html)],
        capture_output=True, text=True)
    if r.returncode != 0:
        print("md2html 失败:\n" + r.stdout + r.stderr)
        return 1

    doc = html.read_text(encoding="utf-8")
    n_fig = 0

    # SVG 内联（不是 <img>）—— Chromium 会把 img 的 SVG 栅格化，
    # 图内文字不进文本层。注入在 pandoc 之后，不会被 raw_html 白名单剥掉。
    for k, svg in figs.items():
        ph = re.compile(r"<p>\s*FIG_%s\s*</p>" % k)
        if ph.search(doc):
            body = svg.replace("<svg ", '<svg style="max-width:100%;height:auto" ', 1)
            fig = f'<figure class="figwrap">{body}</figure>'
            doc = ph.sub(lambda _m, i=fig: i, doc)
            n_fig += 1

    # 真实数据位图。
    #
    # **优先用 _r.jpg 缩图**：原始 PNG 是 1280x966 起的无损图，
    # 06_body_mask 是 4 联竖排（1280x3864）有 3.8 MB —— 直接内嵌让 PDF
    # 涨到 14.7 MB。转成 1400 宽、q=82 的 JPEG 后 0.75 MB，肉眼无差。
    # 缩图由 tools/shrink_report_images.py 生成。
    def pick(name):
        small = ROOT / "viz" / f"{name}_r.jpg"
        return small if small.exists() else ROOT / "viz" / f"{name}.png"

    png_map = {
        "SHOT_Y": pick("sheet_1_y"),
        "SHOT_VIEW": pick("sheet_2_view"),
        "SHOT_DISP": pick("sheet_4_disp"),
        "SHOT_BEV": pick("05_bev"),
        "SHOT_MASK": pick("06_body_mask"),
        "SHOT_CLUST": pick("07_clusters"),
    }
    for k, p in png_map.items():
        ph = re.compile(r"<p>\s*%s\s*</p>" % k)
        if ph.search(doc) and p.exists():
            img = (f'<figure class="figwrap"><img alt="{k}" '
                   f'src="{png_data_uri(p)}"/></figure>')
            doc = ph.sub(lambda _m, i=img: i, doc)
            n_fig += 1

    html.write_text(doc, encoding="utf-8")

    # 断言在 HTML 层，不看 PDF 观感
    n_data = doc.count("data:image/")
    n_inline = doc.count("<svg ")
    assert n_data + n_inline >= n_fig, (
        f"图数不符: data-URI={n_data} inline-svg={n_inline} 期望图={n_fig}")
    assert 'src="viz/' not in doc and 'src="docs/' not in doc, "存在相对路径图"
    # 只能查"占位符形态"（<p>TOKEN</p>），不能查裸 token ——
    # alt="SHOT_Y" 是合法输出，裸查会误报。
    leaked = re.findall(r"<p>\s*(?:FIG_|SHOT_)[A-Z_]*\s*</p>", doc)
    assert not leaked, f"占位符泄漏: {leaked}"
    print(f"HTML 断言通过: 内嵌 {n_fig} 张图，无相对路径、无占位符残留")

    r = subprocess.run([EDGE, "--headless", "--disable-gpu", "--no-sandbox",
                        f"--print-to-pdf={pdf}", "--no-pdf-header-footer",
                        f"file://{html}"], capture_output=True, text=True)
    if r.returncode != 0:
        print("Chromium 打印失败:\n" + r.stderr[-800:])
        return 1
    print(f"PDF: {pdf}  ({pdf.stat().st_size / 1e6:.1f} MB)")

    # PDF 层断言：页数与文字进文本层
    try:
        import fitz
        d = fitz.open(str(pdf))
        n_pages = d.page_count
        all_text = "".join(p.get_text() for p in d)
        d.close()
        must = ["NFOCC", "开发进展看板", "工程架构", "数据流 Pipeline", "算法逻辑"]
        missing = [m for m in must if m not in all_text]
        assert n_pages >= 5, f"页数过少: {n_pages}"
        assert not missing, f"关键文字未进文本层: {missing}"
        print(f"PDF 断言通过: {n_pages} 页，关键文字均可搜索")
    except ImportError:
        print("   (跳过 PDF 文字层检查: 无 pymupdf)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
