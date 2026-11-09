#!/usr/bin/env python3
"""装配 NFOCC 项目报告 PDF。

运行解释器：/usr/bin/python3（PyMuPDF/fitz 装在该解释器下；项目 python3 没有）。

流程：Markdown → pandoc HTML → 注入 15 张内联 SVG + 6 张实测 PNG →
Chromium 打印 A4 → PyMuPDF 加页脚/元数据。两遍渲染，第二遍写入目录页码。
"""
from __future__ import annotations

import base64
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path("/Volumes/Workspace/NFOCC")
V2 = ROOT / "tools" / "report_v2"
REPORT = ROOT / "docs" / "report" / "v2" / "report.md"
IMG = ROOT / "docs" / "report" / "v2" / "img"
OUT = ROOT / "docs" / "report" / "NFOCC-项目报告-2026-11.pdf"
HTML = ROOT / "docs" / "report" / "v2" / "REPORT.html"

EDGE = "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge"
PANDOC = shutil.which("pandoc") or str(Path.home() / ".local/bin/pandoc")

FIGS = {
    "FIG_PHYSICS": ("fig01_physics.svg", "图 1  速度—基线—深度误差的关系（σ_d=0.5 px，计算值）"),
    "FIG_ARCH": ("fig02_arch.svg", "图 3  四层架构与各层实测代码规模"),
    "FIG_CODESIZE": ("fig03_codesize.svg", "图 4  代码规模与工程构成"),
    "FIG_CALLGRAPH": ("fig04_callgraph.svg", "图 5  模块与函数调用关系（实线主链，虚线离线诊断）"),
    "FIG_PIPE": ("fig05_pipeline.svg", "图 6  单帧数据流与两条测距分支"),
    "FIG_GATE": ("fig06_gate.svg", "图 10  默认单帧门控（左）与双基线状态机（右）"),
    "FIG_FORWARD": ("fig07_forward.svg", "图 2  前向径向膨胀的解析测距与 SGBM 失效证据"),
    "FIG_DEFENSE": ("fig08_defense.svg", "图 11  防御体系的接入状态与拦截对象"),
    "FIG_DVF": ("fig09_dvf.svg", "图 12  五阶段体素精简的逐级点数（实测单帧）"),
    "FIG_PERF": ("fig10_perf.svg", "图 14  实测延时：有效帧 vs 摊薄（对数轴）"),
    "FIG_TEST": ("fig11_test.svg", "图 18  测试现状：21 个目标、179 个用例、2,347 次断言"),
    "FIG_STATUS": ("fig12_status.svg", "图 19  开发进展看板（三色：完成 / 部分 / 未做）"),
    "FIG_COMPLETE": ("fig13_complete.svg", "图 20  功能完成度自评（口径见图内）"),
    "FIG_LIMITS": ("fig14_limits.svg", "图 15  已知局限：影响 × 证据 × 对策"),
    "FIG_TIMELINE": ("fig15_timeline.svg", "图 17  研发时间线与本次复核"),
}

IMGS = {
    "IMG_Y": ("y_fv.png", "图 7  原始鱼眼 Y 通道（FV，1280×966）", ""),
    "IMG_VIEW": ("view_ml.png", "图 8  ML 视口重采样（1024×475）。红=无效或被车身掩码剔除", ""),
    "IMG_DISP": ("disp_ml.png", "图 9  ML 视差图（SGBM）。黑=无效，红/黄=近，蓝=远", ""),
    "IMG_BEV": ("bev.png", "图 13  真实数据 BEV（1,255 点 / 546 个占用栅格）", ""),
    "IMG_MASK": ("body_mask.png", "图 16  D1 车身掩码（左上 FV、右上 RV、左下 MVL、右下 MVR；红=判为车身）", "mid"),
}

SECTIONS = [
    ("摘要", "摘要"),
    ("一、项目定位与业务逻辑", "一、项目定位与业务逻辑"),
    ("二、系统架构与代码调用", "二、系统架构与代码调用"),
    ("三、数据流 Pipeline", "三、数据流 Pipeline"),
    ("四、算法详解", "四、算法详解"),
    ("五、关键特性与亮点", "五、关键特性与亮点"),
    ("六、实测性能与资源", "六、实测性能与资源"),
    ("七、系统局限性与风险", "七、系统局限性与风险"),
    ("八、开发进展与功能完成度", "八、开发进展与功能完成度"),
    ("九、结论与下一步", "九、结论与下一步"),
    ("附录 A：关键参数", "附录 A：关键参数"),
    ("附录 B：主要函数清单", "附录 B：主要函数清单"),
    ("附录 C：证据边界与复现", "附录 C：证据边界与复现"),
]

CSS = r"""
@page { size: A4; margin: 16mm 15mm 18mm 15mm; }
* { box-sizing: border-box; }
html, body { margin: 0; padding: 0; }
body {
  font-family: "PingFang SC","Hiragino Sans GB","Heiti SC",sans-serif;
  font-size: 9.8pt; line-height: 1.72; color: #1b2b3a;
  -webkit-font-smoothing: antialiased;
}
p { margin: 0 0 7px; text-align: justify; }
h1 {
  page-break-before: always; break-before: page;
  font-size: 19pt; color: #1e5a96; margin: 0 0 12px;
  padding-bottom: 7px; border-bottom: 2.5px solid #1e5a96;
  letter-spacing: .5px;
}
h1:first-of-type { page-break-before: avoid; break-before: avoid; }
h2 {
  font-size: 13pt; color: #173f69; margin: 18px 0 7px;
  padding-left: 9px; border-left: 4px solid #1e5a96;
  page-break-after: avoid; break-after: avoid;
}
h3 { font-size: 10.8pt; color: #2a4a6a; margin: 13px 0 5px; }
strong { color: #10263a; }
ul, ol { margin: 4px 0 8px 20px; padding: 0; }
li { margin: 2px 0; }
table {
  width: 100%; border-collapse: collapse; margin: 9px 0 12px;
  font-size: 8.5pt; line-height: 1.45;
}
th {
  background: #eaf1f9; color: #173f69; font-weight: 600;
  border: 1px solid #c9d6e3; padding: 4px 6px; text-align: left;
}
td { border: 1px solid #dbe3ea; padding: 3.5px 6px; vertical-align: top; }
tr { page-break-inside: avoid; break-inside: avoid; }
code {
  font-family: "SF Mono","Menlo",monospace; font-size: 8.6pt;
  background: #f2f5f8; padding: 1px 3px; border-radius: 2px; color: #16405f;
}
pre {
  background: #f5f8fb; border: 1px solid #dde5ec; border-left: 3px solid #1e5a96;
  padding: 7px 10px; margin: 7px 0 11px; font-size: 8.2pt; line-height: 1.55;
  white-space: pre-wrap; page-break-inside: avoid; break-inside: avoid;
}
pre code { background: none; padding: 0; }
blockquote {
  background: #f1f7fd; border-left: 3px solid #1e5a96;
  margin: 9px 0 11px; padding: 8px 12px; color: #26445f;
  font-size: 9.2pt; page-break-inside: avoid;
}
blockquote p { margin: 0; }
figure.fig {
  margin: 6px 0 13px; page-break-inside: avoid; break-inside: avoid;
  text-align: center;
}
figure.fig svg { width: 100%; height: auto; display: block; }
figure.fig img { width: 100%; height: auto; display: block; }
figure.fig.small { width: 55%; margin-left: auto; margin-right: auto; }
figure.fig.mid { width: 70%; margin-left: auto; margin-right: auto; }
.pb { page-break-before: always; break-before: page; height: 0; }
.findings {
  background: #f4f8fc; border: 1px solid #cfdde9; border-left: 4px solid #1e5a96;
  padding: 4mm 5mm 3mm; margin: 5mm 0 0; page-break-inside: avoid;
}
.findings b { display: block; font-size: 10.2pt; color: #173f69; margin-bottom: 2mm; }
.findings p { font-size: 8.7pt; line-height: 1.55; margin: 0 0 1.2mm; color: #26445f; text-align: left; }
figcaption {
  font-size: 8.2pt; color: #5d6d7e; margin-top: 4px;
  text-align: center; line-height: 1.4;
}
hr { border: none; border-top: 1px solid #dbe3ea; margin: 14px 0; }

/* ---------- cover ---------- */
.cover {
  height: 263mm; page-break-after: always; break-after: page;
  position: relative; padding: 0 4mm;
}
.cover .band { height: 9mm; background: #1e5a96; margin: 0 -4mm 18mm; }
.cover .kicker { font-size: 10pt; letter-spacing: 3px; color: #1e5a96; font-weight: 600; }
.cover h1.ct {
  font-size: 34pt; line-height: 1.28; color: #10263a; margin: 8mm 0 5mm;
  border: none; padding: 0; letter-spacing: 1px;
}
.cover .sub { font-size: 12.5pt; color: #4b5f72; line-height: 1.7; }
.cover .meta { font-size: 9.5pt; color: #7a8a99; margin-top: 6mm; }
.cover .stats { display: flex; gap: 4mm; margin-top: 20mm; }
.cover .stat {
  flex: 1; border-top: 3px solid #1e5a96; background: #f4f8fc;
  padding: 4mm 4mm 3mm;
}
.cover .stat b { display: block; font-size: 20pt; color: #1e5a96; line-height: 1.2; }
.cover .stat span { display: block; font-size: 8.6pt; color: #5d6d7e; margin-top: 2mm; }
.cover .verdict {
  margin-top: 14mm; background: #fdf4e5; border-left: 4px solid #b26a00;
  padding: 5mm 6mm; font-size: 10.5pt; color: #6b4a10; line-height: 1.75;
}
.cover .verdict b { color: #8a5300; }
.cover .foot {
  position: absolute; bottom: 4mm; left: 4mm; right: 4mm;
  font-size: 8pt; color: #8a98a6; border-top: 1px solid #dbe3ea; padding-top: 3mm;
  display: flex; justify-content: space-between;
}

/* ---------- toc ---------- */
.toc { page-break-after: always; break-after: page; }
.toc h1 { page-break-before: avoid; border-bottom: 2.5px solid #1e5a96; }
.toc .lead { font-size: 9.2pt; color: #5d6d7e; margin-bottom: 7mm; }
.toc .row {
  display: flex; align-items: baseline; font-size: 10.2pt;
  padding: 2.6mm 0; border-bottom: 1px dotted #c9d6e3;
}
.toc .row .n { width: 12mm; color: #1e5a96; font-weight: 600; }
.toc .row .t { flex: 1; }
.toc .row .p { width: 12mm; text-align: right; color: #5d6d7e; }
.toc .note {
  margin-top: 12mm; background: #f4f8fc; border: 1px solid #dbe3ea;
  padding: 5mm 6mm; font-size: 8.8pt; color: #4b5f72; line-height: 1.7;
}
"""


def pandoc_html(md_text: str) -> str:
    p = subprocess.run(
        [PANDOC, "-f", "markdown+pipe_tables+raw_html", "-t", "html5",
         "--wrap=none"],
        input=md_text, capture_output=True, text=True, check=True)
    return p.stdout


def inline_svg(path: Path, suffix: str) -> str:
    s = path.read_text(encoding="utf-8")
    s = re.sub(r"<\?xml[^>]*\?>", "", s).strip()
    # 每个图有独立的 marker id，内联到同一文档时必须改名，避免 id 冲突
    for mid in ("ar", "arb", "arg", "arr"):
        s = s.replace(f'id="{mid}"', f'id="{mid}{suffix}"')
        s = s.replace(f"url(#{mid})", f"url(#{mid}{suffix})")
    return s


def inject(html: str) -> tuple[str, int, int]:
    n_svg = n_img = 0
    for i, (key, (fn, cap)) in enumerate(FIGS.items()):
        path = V2 / "fig" / fn
        if not path.exists():
            raise SystemExit(f"缺图: {path}")
        svg = inline_svg(path, f"_{i}")
        block = f'<figure class="fig">{svg}<figcaption>{cap}</figcaption></figure>'
        new = html.replace(f"<p>{key}</p>", block)
        if new == html:
            raise SystemExit(f"占位符未命中: {key}")
        html = new
        n_svg += 1
    for key, (fn, cap, cls) in IMGS.items():
        path = IMG / fn
        if not path.exists():
            raise SystemExit(f"缺图: {path}")
        b64 = base64.b64encode(path.read_bytes()).decode("ascii")
        block = (f'<figure class="fig {cls}"><img alt="{key}" '
                 f'src="data:image/png;base64,{b64}"/>'
                 f"<figcaption>{cap}</figcaption></figure>")
        new = html.replace(f"<p>{key}</p>", block)
        if new == html:
            raise SystemExit(f"占位符未命中: {key}")
        html = new
        n_img += 1
    leftover = re.findall(r"<p>\s*(?:FIG_|IMG_)[A-Z_]*\s*</p>", html)
    if leftover:
        raise SystemExit(f"占位符泄漏: {leftover}")
    return html, n_svg, n_img


def toc_html(pages: dict[str, int]) -> str:
    rows = []
    for i, (label, key) in enumerate(SECTIONS, 1):
        pg = pages.get(key, "")
        rows.append(
            f'<div class="row"><div class="n">{i:02d}</div>'
            f'<div class="t">{label}</div><div class="p">{pg}</div></div>')
    return ('<section class="toc"><h1>目录</h1>'
            '<div class="lead">A4 竖版 · 正文 9.8 pt · 图表 15 张矢量图 + 5 张实测图。'
            '页码为 PDF 物理页。</div>' + "".join(rows) +
            '<div class="note"><b>怎么读这份报告。</b>第 1–4 章是系统说明（业务约束、架构、'
            '数据流、算法）；第 5–6 章是亮点与实测；第 7–8 章是局限、进展与完成度；'
            '第 9 章是结论与优先级。所有实测数字标注了来源；计算值与文档值分开标注。'
            '本次复核发现的 8 处文档/代码偏差集中在 8.5 节。</div></section>')


def cover_html() -> str:
    stats = [
        ("11,004", "行核心代码（src + include）"),
        ("4 × 1280×966", "鱼眼输入，单 Y 通道"),
        ("0.5–5 m", "近场覆盖，输出点云 + zMap"),
        ("20 / 21", "ctest 通过；179 用例 / 2,347 断言"),
    ]
    cards = "".join(
        f'<div class="stat"><b>{v}</b><span>{k}</span></div>' for v, k in stats)
    return f'''<section class="cover">
  <div class="band"></div>
  <div class="kicker">NFOCC · 技术项目报告</div>
  <h1 class="ct">近场鱼眼<br/>运动双目稠密感知系统</h1>
  <div class="sub">业务逻辑 · 架构与类调用 · 数据流 Pipeline · 算法流程<br/>
  关键特性 · 实测性能 · 系统局限 · 开发进展与功能完成度</div>
  <div class="meta">2026-11-09　·　基于 commit d2d4049（v0.3）与本次独立复核<br/>
  复核环境：macOS / Apple Silicon，Release，NF_WITH_VCAM=ON，OpenCV SGBM</div>
  <div class="stats">{cards}</div>
  <div class="verdict"><b>结论：链路已打通，精度未证实。</b><br/>
  STEREO 有效帧 104 ms，在 5 Hz / 200 ms 预算内；FORWARD 有效帧 11.2 s，超预算 56 倍。
  深度精度 4/4 分段未达，真实数据无点云真值 —— 能否交付取决于精度验收，而不是代码行数。</div>
  <div class="foot"><span>内部资料 · 未经授权请勿外传</span><span>NFOCC-DOC v0.3</span></div>
</section>'''


def build(pages: dict[str, int]) -> Path:
    md = REPORT.read_text(encoding="utf-8")
    body, n_svg, n_img = inject(pandoc_html(md))
    html = f'''<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8"/>
<title>NFOCC 近场鱼眼运动双目稠密感知系统 · 项目报告</title>
<style>{CSS}</style></head><body>
{cover_html()}
{toc_html(pages)}
{body}
</body></html>'''
    HTML.write_text(html, encoding="utf-8")
    print(f"  HTML: {HTML.stat().st_size/1e6:.2f} MB, SVG {n_svg}, IMG {n_img}")
    shutil.rmtree("/tmp/nfocc_report_edge", ignore_errors=True)
    run = [EDGE, "--headless", "--disable-gpu", "--no-sandbox",
           "--user-data-dir=/tmp/nfocc_report_edge", "--no-pdf-header-footer",
           f"--print-to-pdf={OUT}", HTML.as_uri()]
    for attempt in range(2):
        r = subprocess.run(run, capture_output=True)
        if r.returncode == 0 and OUT.exists():
            break
        shutil.rmtree("/tmp/nfocc_report_edge", ignore_errors=True)
    else:
        raise SystemExit("Edge 打印失败: " + r.stderr.decode(errors="ignore")[-800:])
    return OUT


def find_pages(pdf: Path) -> dict[str, int]:
    import fitz
    doc = fitz.open(pdf)
    pages: dict[str, int] = {}
    for label, key in SECTIONS:
        needle = key.split("：")[0] if "：" in key else key
        for i, page in enumerate(doc):
            if i < 2:          # 跳过封面与目录，否则每节都命中目录页
                continue
            txt = page.get_text("text").replace("\n", "")
            if needle in txt:
                pages[key] = i + 1
                break
    doc.close()
    return pages


def add_footer(pdf: Path) -> None:
    import fitz
    doc = fitz.open(pdf)
    n = doc.page_count
    for i, page in enumerate(doc):
        if i == 0:
            continue
        txt = f"NFOCC Project Report 2026-11    {i+1} / {n}"
        w = fitz.get_text_length(txt, fontname="helv", fontsize=7.2)
        page.insert_text(((page.rect.width - w) / 2, page.rect.height - 22),
                         txt, fontname="helv", fontsize=7.2,
                         color=(0.42, 0.48, 0.54))
    doc.set_metadata({
        "title": "NFOCC 近场鱼眼运动双目稠密感知系统 · 项目报告",
        "author": "NFOCC",
        "subject": "业务逻辑 / 架构与调用 / 数据流 / 算法 / 实测性能 / 局限与进展",
        "keywords": "NFOCC, fisheye, motion stereo, SGBM, voxel, edge NPU",
        "creator": "NFOCC report_v2 builder",
    })
    doc.saveIncr()
    doc.close()


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    print("[1/3] 首遍渲染（取页码）")
    build({})
    pages = find_pages(OUT)
    missing = [k for _, k in SECTIONS if k not in pages]
    print(f"  找到 {len(pages)}/{len(SECTIONS)} 个章节页码；缺: {missing}")
    print("[2/3] 二遍渲染（写目录页码）")
    build(pages)
    print("[3/3] 页脚与元数据")
    add_footer(OUT)
    import fitz
    doc = fitz.open(OUT)
    print(f"  PDF: {OUT}  {OUT.stat().st_size/1e6:.2f} MB  {doc.page_count} 页")
    doc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
