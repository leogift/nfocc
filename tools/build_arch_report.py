#!/usr/bin/env python3
"""生成 NFOCC 架构/流程图的 SVG，并构建图文并茂的 PDF 报告。

## 为什么自己画 SVG
架构图、数据流图、决策流程图都是**结构化的**，用 Python 直接生成 SVG
比手画或截图更可维护：改一处文字不用重画。

## 为什么要 base64 内嵌
Chromium 解析 `<img src="相对路径">` 是**按页面 URL**解析的，
移动文件或换 cwd 就会静默丢图（markdown-to-pdf skill 的 pitfall #8）。
base64 data URI 免疫这个问题，且 HTML 自身可交付。

用法:
    python tools/build_arch_report.py
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

# 配色（与 BEV 视口色一致，便于对照）
C_BG = "#ffffff"
C_BOX = "#f4f7fb"
C_BORDER = "#3b5b8c"
C_TEXT = "#1a2333"
C_ACCENT = "#c0392b"
C_OK = "#2e7d32"
C_WARN = "#e67e22"
C_INFRA = "#6c5ce7"


def svg_open(w, h):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
            f'viewBox="0 0 {w} {h}">'
            f'<rect width="{w}" height="{h}" fill="{C_BG}"/>')
def box(x, y, w, h, label, sub="", color=C_BORDER, fill=C_BOX, fs=15):
    s = (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6" '
         f'fill="{fill}" stroke="{color}" stroke-width="1.6"/>')
    cy = y + h / 2 + (fs * 0.35)
    if sub:
        cy = y + h / 2 - 4
        s += (f'<text x="{x + w/2}" y="{cy}" text-anchor="middle" '
              f'font-family={FONT!r} font-size="{fs}" fill="{C_TEXT}" '
              f'font-weight="600">{label}</text>')
        s += (f'<text x="{x + w/2}" y="{cy + 16}" text-anchor="middle" '
              f'font-family={FONT!r} font-size="{fs - 3}" fill="#5a6a80">{sub}</text>')
    else:
        s += (f'<text x="{x + w/2}" y="{cy}" text-anchor="middle" '
              f'font-family={FONT!r} font-size="{fs}" fill="{C_TEXT}">{label}</text>')
    return s


def arrow(x1, y1, x2, y2, label="", color="#456", dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    s = (f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" '
         f'stroke-width="1.8" marker-end="url(#ah)"{d}/>')
    if label:
        s += (f'<text x="{(x1+x2)/2}" y="{(y1+y2)/2 - 6}" text-anchor="middle" '
              f'font-family={FONT!r} font-size="11" fill="{color}">{label}</text>')
    return s


MARKER = ('<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" '
          'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
          '<path d="M 0 0 L 10 5 L 0 10 z" fill="#456"/></marker>'
          '<marker id="ahr" viewBox="0 0 10 10" refX="9" refY="5" '
          'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
          f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{C_ACCENT}"/></marker></defs>')


# ---------------------------------------------------------------- 架构分层图
def fig_arch():
    W, H = 980, 620
    s = svg_open(W, H) + MARKER

    s += (f'<text x="{W/2}" y="30" text-anchor="middle" font-family={FONT!r} '
          f'font-size="19" font-weight="700" fill="{C_TEXT}">'
          f'NFOCC 分层架构</text>')

    layers = [
        (60, "接入层", "#e8f0fe", C_BORDER, [
            ("hanqtech 加载器", "标定/PNG/ego"),
            ("文件回放", ".nfp"),
            ("ZMQ 上游", "图像+ego"),
            ("真值场景源", "解析渲染"),
        ]),
        (215, "几何与视口层", "#e6f4ea", C_OK, [
            ("虚拟相机视口", "pitch=0 roll=0"),
            ("视口 LUT", "反投影采样表"),
            ("D1 车身掩码", "多边形光栅化"),
            ("四次多项式内参", "ρ(θ)=Σk_iθ^i"),
        ]),
        (370, "算法层", "#fff3e0", C_WARN, [
            ("SGBM 视差", "8bit 支持"),
            ("径向膨胀匹配", "前向对应点"),
            ("双基线状态机", "短/长基线交替"),
            ("D2/D3/D4 防御", "软权重/锥/交叉"),
        ]),
        (525, "输出与基础设施", "#f3e8fd", C_INFRA, [
            ("体素哈希 + 柱体排布", "zMap/zIndex O(1)"),
            ("σ 方差传播", "σ_Z 全微分"),
            ("点云 / 下游协议", "CRC16+计数"),
            ("TaskExecutor", "线程池+环形队列"),
        ]),
    ]

    for y0, name, fill, color, items in layers:
        s += (f'<rect x="40" y="{y0}" width="{W-80}" height="130" rx="10" '
              f'fill="{fill}" stroke="{color}" stroke-width="1.2" '
              f'stroke-opacity="0.55"/>')
        s += (f'<text x="58" y="{y0+26}" font-family={FONT!r} font-size="14" '
              f'font-weight="700" fill="{color}">{name}</text>')
        bw, gap = 205, 18
        for i, (lbl, sub) in enumerate(items):
            bx = 58 + i * (bw + gap)
            s += box(bx, y0 + 40, bw, 70, lbl, sub, color=color, fill="#ffffff",
                     fs=13)

    # 层间箭头
    for y0 in (190, 345, 500):
        s += arrow(W / 2, y0, W / 2, y0 + 22, color="#456")
    s += (f'<text x="{W-52}" y="206" text-anchor="end" font-family={FONT!r} '
          f'font-size="11" fill="#6b7a8f">车辆系 ISO 8855 贯穿</text>')
    s += "</svg>"
    return s


# ---------------------------------------------------------------- 数据流图
def fig_pipeline():
    W, H = 1180, 470
    s = svg_open(W, H) + MARKER
    s += (f'<text x="{W/2}" y="28" text-anchor="middle" font-family={FONT!r} '
          f'font-size="19" font-weight="700" fill="{C_TEXT}">'
          f'单帧数据流（每路相机独立、四路并行）</text>')

    stages = [
        ("原图", "1280×966\nY 通道", C_BORDER),
        ("视口 LUT", "640×297\npitch=0/roll=0", C_BORDER),
        ("重采样", "双线性\nD1 掩码置无效", C_BORDER),
        ("SGBM", "视差图\nd_max=46.8", C_WARN),
        ("三角化", "叉积闭式\ns0<20m", C_WARN),
        ("D3 FoE", "8° 锥剔除", C_ACCENT),
        ("σ 传播", "cm 量化\n1..254", C_INFRA),
        ("体素", "200mm\nCMA 聚合", C_INFRA),
    ]
    n = len(stages)
    bw, gap = 118, 15
    x0 = (W - (n * bw + (n - 1) * gap)) / 2
    y0 = 90
    for i, (lbl, sub, col) in enumerate(stages):
        bx = x0 + i * (bw + gap)
        s += box(bx, y0, bw, 86, lbl, "", color=col, fill="#ffffff", fs=14)
        for j, line in enumerate(sub.split("\n")):
            s += (f'<text x="{bx + bw/2}" y="{y0 + 54 + j*14}" '
                  f'text-anchor="middle" font-family={FONT!r} font-size="11" '
                  f'fill="#5a6a80">{line}</text>')
        if i < n - 1:
            s += arrow(bx + bw, y0 + 43, bx + bw + gap, y0 + 43)

    # 汇合 -> 输出
    s += (f'<path d="M {x0 + n*(bw+gap) - gap} {y0+86} L {x0 + n*(bw+gap) - gap} 250 '
          f'L {W/2} 250 L {W/2} 285" fill="none" stroke="#456" stroke-width="1.8" '
          f'marker-end="url(#ah)"/>')
    s += box(W / 2 - 210, 290, 420, 74, "融合去重", "四路 + 六视口 -> 单一体素表",
             color=C_OK, fill="#e6f4ea", fs=15)
    s += arrow(W / 2, 364, W / 2, 398)
    s += box(W / 2 - 250, 402, 500, 52, "zMap / zIndex / 点云（O(1) 柱体索引）",
             color=C_INFRA, fill="#f3e8fd", fs=14)

    s += (f'<text x="60" y="{H-16}" font-family={FONT!r} font-size="11" '
          f'fill="#6b7a8f">单帧预算 200 ms（5 Hz）；实测 34 ms/帧</text>')
    s += "</svg>"
    return s


# ---------------------------------------------------------------- 决策流程图
def fig_flow():
    W, H = 900, 690
    s = svg_open(W, H) + MARKER
    s += (f'<text x="{W/2}" y="28" text-anchor="middle" font-family={FONT!r} '
          f'font-size="19" font-weight="700" fill="{C_TEXT}">'
          f'门控与三角化决策流</text>')

    cx = 300
    s += box(cx - 130, 52, 260, 52, "输入 t_view（视口系）", "", color=C_BORDER,
             fill="#e8f0fe", fs=14)
    s += arrow(cx, 104, cx, 132)

    s += box(cx - 150, 134, 300, 56, "Rz(yaw) 转到车系", "t_veh", color=C_OK,
             fill="#e6f4ea", fs=14)
    s += arrow(cx, 190, cx, 218)

    s += box(cx - 165, 220, 330, 56, "Δψ > 15° ?", "急转 / 原地掉头", color=C_ACCENT,
             fill="#fdecea", fs=14)
    s += arrow(cx + 165, 248, cx + 255, 248, "是", C_ACCENT)
    s += box(cx + 265, 222, 150, 52, "FROZEN_ROT", "重置队列", color=C_ACCENT,
             fill="#fdecea", fs=13)

    s += arrow(cx, 276, cx, 304)
    s += box(cx - 165, 306, 330, 56, "b > b_max (460mm) ?", "", color=C_WARN,
             fill="#fff3e0", fs=14)
    s += arrow(cx + 165, 334, cx + 255, 334, "是", C_WARN)
    s += box(cx + 265, 308, 150, 52, "降级/冻结", "重置队列", color=C_WARN,
             fill="#fff3e0", fs=13)

    s += arrow(cx, 362, cx, 390)
    s += box(cx - 165, 392, 330, 56, "b ≥ b_target (300mm) ?", "", color=C_OK,
             fill="#e6f4ea", fs=14)
    s += arrow(cx + 165, 420, cx + 255, 420, "是", C_OK)
    s += box(cx + 265, 394, 150, 52, "用长基线", "短基线晋升", color=C_OK,
             fill="#e6f4ea", fs=13)

    s += arrow(cx, 448, cx, 476)
    s += box(cx - 165, 478, 330, 56, "b_short ≥ b_min (200mm) ?", "", color=C_BORDER,
             fill="#e8f0fe", fs=14)
    s += arrow(cx + 165, 506, cx + 255, 506, "是", C_BORDER)
    s += box(cx + 265, 480, 150, 52, "用短基线", "", color=C_BORDER,
             fill="#e8f0fe", fs=13)

    s += arrow(cx, 534, cx, 562)
    s += box(cx - 165, 564, 330, 52, "STEREO：SGBM → 三角化", "", color=C_INFRA,
             fill="#f3e8fd", fs=14)
    s += arrow(cx, 616, cx, 640)
    s += box(cx - 210, 642, 420, 40, "D3 锥剔除 → σ 量化 → 体素插入",
             color=C_INFRA, fill="#f3e8fd", fs=13)

    s += (f'<text x="{W-40}" y="{H-14}" text-anchor="end" font-family={FONT!r} '
          f'font-size="11" fill="#6b7a8f">'
          f'任一条件不满足即 FROZEN_NEAR（不产出，但不丢帧）</text>')
    s += "</svg>"
    return s


def b64_img(svg_text, mime="image/svg+xml"):
    payload = base64.b64encode(svg_text.encode("utf-8")).decode("ascii")
    return f"data:{mime};base64,{payload}"


def png_data_uri(path: Path):
    payload = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:image/png;base64,{payload}"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    figs = {"ARCH": fig_arch(), "PIPE": fig_pipeline(), "FLOW": fig_flow()}
    for k, v in figs.items():
        (OUT / f"fig_{k.lower()}.svg").write_text(v, encoding="utf-8")
    print(f"生成 {len(figs)} 张 SVG: {', '.join(figs)}")

    md = OUT / "ARCH-REPORT.md"
    html = OUT / "ARCH-REPORT.html"
    pdf = OUT / "NFOCC-架构与流程解析.pdf"

    r = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "md2html.py"), str(md), str(html)],
        capture_output=True, text=True)
    if r.returncode != 0:
        print("md2html 失败:\n" + r.stdout + r.stderr)
        return 1

    doc = html.read_text(encoding="utf-8")
    n_fig = 0

    # SVG 图：**内联**而不是 <img src=data:...>。
    #
    # 用 <img> 时 Chromium 会把 SVG 栅格化成位图，图内文字**不进文本层**
    # （实测 "接入层" 搜不到），且 PDF 体积膨胀到 9 MB。
    # 内联 <svg> 保持矢量：文字可选中、体积降一个数量级。
    #
    # 内联是安全的 —— 注入发生在 pandoc **之后**，不会被它的 raw_html
    # 白名单剥掉（markdown-to-pdf skill 的 pitfall #7 说的是 Markdown 里
    # 直接写 <svg>，那是另一回事）。
    for k, svg in figs.items():
        ph = re.compile(r"<p>\s*FIG_%s\s*</p>" % k)
        if ph.search(doc):
            body = svg.replace("<svg ", '<svg style="max-width:100%;height:auto" ', 1)
            fig = f'<figure class="figwrap">{body}</figure>'
            doc = ph.sub(lambda _m, i=fig: i, doc)
            n_fig += 1

    # 真实数据 PNG 图
    png_map = {
        "SHOT_Y": ROOT / "viz" / "sheet_1_y.png",
        "SHOT_VIEW": ROOT / "viz" / "sheet_2_view.png",
        "SHOT_DISP": ROOT / "viz" / "sheet_4_disp.png",
        "SHOT_BEV": ROOT / "viz" / "05_bev_legend.png",
    }
    for k, p in png_map.items():
        ph = re.compile(r"<p>\s*%s\s*</p>" % k)
        if ph.search(doc) and p.exists():
            img = (f'<figure class="figwrap"><img alt="{k}" '
                   f'src="{png_data_uri(p)}"/></figure>')
            doc = ph.sub(lambda _m, i=img: i, doc)
            n_fig += 1

    html.write_text(doc, encoding="utf-8")

    # --- 断言（在 HTML 层，不看 PDF 观感）---
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
    if not pdf.exists():
        print("PDF 生成失败:\n" + r.stdout[-800:] + r.stderr[-800:])
        return 1
    print(f"PDF: {pdf} ({pdf.stat().st_size/1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
