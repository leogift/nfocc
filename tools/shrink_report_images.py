#!/usr/bin/env python3
"""把 viz/ 下的报告用图压成适合嵌入 PDF 的尺寸。

## 为什么需要这一步
`build_arch_report.py` 会把位图 base64 内嵌进 HTML 再打印成 PDF。
原始 PNG 是无损的：

    06_body_mask.png   3.78 MB（4 联竖排，1280×3864）
    sheet_1_y.png      3.68 MB
    sheet_2_view.png   1.72 MB
    sheet_4_disp.png   0.93 MB

直接内嵌让 PDF 到 **14.7 MB**，其中 90% 是这几张图。转成 1400 宽、
q=82 的 JPEG 后合计约 2.4 MB，PDF 降到 4.3 MB，肉眼无差别。

## 为什么不改 build 脚本内部转
保持 build 脚本只做"组装"，不偷偷改用户数据。缩图是独立、可复现的一步：
产物带 `_r.jpg` 后缀，原始 PNG 保持不动（viz 那边还要用）。

用法:
    python3 tools/shrink_report_images.py
"""
import sys
from pathlib import Path

from PIL import Image

ROOT = Path("/Volumes/Workspace/NFOCC")
VIZ = ROOT / "viz"

# 报告里引用的 6 张图（与 build_arch_report.py 的 png_map 对应）
NAMES = ["06_body_mask", "sheet_1_y", "sheet_2_view",
         "sheet_4_disp", "05_bev", "07_clusters"]

MAX_W = 1400      # 报告页宽下再大也看不出差别
QUALITY = 82      # 肉眼无差，体积约为 q=95 的 1/3


def main():
    total_in = total_out = 0
    for name in NAMES:
        src = VIZ / f"{name}.png"
        if not src.exists():
            print(f"  跳过 {name}: 缺 {src}")
            continue
        im = Image.open(src).convert("RGB")
        w, h = im.size
        if w > MAX_W:
            im = im.resize((MAX_W, int(h * MAX_W / w)), Image.LANCZOS)

        dst = VIZ / f"{name}_r.jpg"
        im.save(dst, quality=QUALITY, optimize=True)

        a, b = src.stat().st_size, dst.stat().st_size
        total_in += a
        total_out += b
        print(f"  {name}: {a/1e6:5.2f} MB -> {b/1e6:5.2f} MB")
        # 缩图必须真的变小，否则说明参数没起作用
        assert b < a, f"{name} 缩图后反而变大"

    if total_in:
        print(f"  合计 {total_in/1e6:.2f} MB -> {total_out/1e6:.2f} MB "
              f"（-{100*(1-total_out/total_in):.0f}%）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
