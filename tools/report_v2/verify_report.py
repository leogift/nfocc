#!/usr/bin/env python3
"""NFOCC 报告 PDF 的交付校验。

检查的是"能不能拿去打印/汇报"这件事本身：页数、A4、文本层、图表嵌入方式、
目录页码、版面越界、占位符泄漏。运行：/usr/bin/python3 verify_report.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import fitz

ROOT = Path("/Volumes/Workspace/NFOCC")
PDF = ROOT / "docs" / "report" / "NFOCC-项目报告-2026-11.pdf"
HTML = ROOT / "docs" / "report" / "v2" / "REPORT.html"

SECTIONS = [
    ("摘要", 3), ("一、项目定位与业务逻辑", 4), ("二、系统架构与代码调用", 7),
    ("三、数据流 Pipeline", 11), ("四、算法详解", 14), ("五、关键特性与亮点", 19),
    ("六、实测性能与资源", 20), ("七、系统局限性与风险", 22),
    ("八、开发进展与功能完成度", 24), ("九、结论与下一步", 28),
    ("附录 A：关键参数", 29), ("附录 B：主要函数清单", 30),
    ("附录 C：证据边界与复现", 31),
]

fails: list[str] = []


def check(cond, msg):
    print(("PASS  " if cond else "FAIL  ") + msg)
    if not cond:
        fails.append(msg)


def main() -> int:
    check(PDF.exists(), f"PDF 存在：{PDF.name}")
    doc = fitz.open(PDF)
    check(25 <= doc.page_count <= 40, f"页数合理：{doc.page_count}")
    check(abs(doc[0].rect.width - 595) < 2 and abs(doc[0].rect.height - 842) < 2,
          "页面为 A4 纵向（595×842 pt）")
    text = "\n".join(p.get_text("text") for p in doc)

    # 文本层：正文与图内标签都必须可选中（图若被栅格化，图内词会消失）
    for tok in ["五阶段稠密体素精简", "FROZEN_NEAR", "τ 随距离自适应",
                "NFEngineOps", "2,077 → 1,534", "NFOCC"]:
        check(tok in text, f"文本层含图内词：{tok}")

    # 连续性：CJK 不能被拆成逐字空格
    check("远距有毫米波和激光雷达覆盖" in text.replace("\n", ""),
          "CJK 文本层连续（未打散）")

    # 占位符/构建残留
    for bad in ["FIG_", "IMG_", "<svg", "</svg>", "data:image"]:
        check(bad not in text, f"无构建残留：{bad}")

    # 目录页码与实际标题页一致
    for title, expected in SECTIONS:
        needle = title.split("：")[0]
        pages = [i + 1 for i, p in enumerate(doc)
                 if i >= 2 and needle in p.get_text("text").replace("\n", "")]
        check(bool(pages) and pages[0] == expected,
              f"目录页码：{title} -> {expected}（实际 {pages[0] if pages else '?'}）")

    # 版面：正文块不得越出页边距（15 mm ≈ 42.5 pt，留 30 pt 容差到页边）
    out = []
    for i, p in enumerate(doc):
        for b in p.get_text("blocks"):
            x0, y0, x1, y1 = b[:4]
            if x0 < 30 or x1 > p.rect.width - 30 or y0 < 20 or y1 > p.rect.height - 15:
                out.append((i + 1, round(x0), round(x1)))
    check(not out, f"无文本越出页边（{len(out)} 处）")

    # 图表：PDF 里只有 5 张实测位图；15 张 SVG 必须保持矢量（否则内嵌位图会 >5）
    n_img = sum(len(p.get_images(full=True)) for p in doc)
    check(n_img == 5, f"PDF 内嵌位图 = 5 张实测图（实际 {n_img}）")

    html = HTML.read_text(encoding="utf-8")
    check(html.count("<svg ") == 15, "HTML 内联 SVG = 15")
    check(html.count("data:image/png;base64") == 5, "HTML 内嵌 PNG = 5")
    check('src="fig/' not in html, "HTML 不依赖外部图片路径")
    check(not re.findall(r"<p>\s*(?:FIG_|IMG_)", html), "HTML 无占位符泄漏")

    # 页脚：除封面外每页都有页码
    missing = []
    for i, p in enumerate(doc):
        if i == 0:
            continue
        if f"{i + 1} / {doc.page_count}" not in p.get_text("text"):
            missing.append(i + 1)
    check(not missing, f"页脚页码完整（缺 {missing}）")

    size_mb = PDF.stat().st_size / 1e6
    check(size_mb < 15, f"文件体积合理：{size_mb:.2f} MB")

    print("-" * 60)
    if fails:
        print(f"FAILED {len(fails)} 项")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
