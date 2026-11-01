#!/usr/bin/env python
"""给 BEV 图叠加视口图例（文字标注）。

在 nfocc_viz 生成的 05_bev.png 上加图例，说明每个颜色对应哪个虚拟视口。
C++ 侧已画出 FOV 扇形与按视口着色的占用格，这里只补可读的标注。

    python tools/annotate_bev_legend.py <viz_dir>
"""
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

VIEWS = [
    ("FV 左前",  -45.0, (230,  60,  60)),
    ("FV 右前",  +45.0, (250, 140,  40)),
    ("RV 左后", -135.0, ( 60, 170,  70)),
    ("RV 右后", +135.0, ( 40, 190, 200)),
    ("ML 左",    +90.0, (120,  90, 220)),
    ("MR 右",    -90.0, (230,  70, 180)),
]


def main():
    d = Path(sys.argv[1] if len(sys.argv) > 1 else "/Volumes/Workspace/NFOCC/viz")
    src = d / "05_bev.png"
    if not src.exists():
        print(f"缺 {src}")
        return 1
    im = Image.open(src).convert("RGB")

    # 顶部留出图例条
    pad_top, pad_bot = 76, 30
    out = Image.new("RGB", (im.width, im.height + pad_top + pad_bot), "white")
    out.paste(im, (0, pad_top))
    dr = ImageDraw.Draw(out)

    try:
        f_big = ImageFont.truetype("/System/Library/Fonts/Hiragino Sans GB.ttc", 20)
        f_sm = ImageFont.truetype("/System/Library/Fonts/Hiragino Sans GB.ttc", 16)
    except Exception:
        f_big = f_sm = ImageFont.load_default()

    # 标题
    dr.text((10, 8), "Voxel 分层 BEV - 6 虚拟视口 FOV 分色", font=f_big, fill="#111111")
    dr.text((10, 34),
            "虚拟相机 pitch=0/roll=0，水平 150°/垂直 120°；"
            "彩色=单视口覆盖，灰白=多视口重叠（更可信），细线=各视口 FOV 边界",
            font=f_sm, fill="#666666")

    # 图例条：6 个色块横排
    x = 12
    y = 58
    for name, yaw, col in VIEWS:
        dr.rectangle([x, y, x + 18, y + 12], fill=col)
        dr.text((x + 24, y - 2), f"{name} {yaw:+.0f}°", font=f_sm, fill="#222222")
        x += 150
    # 重叠色
    dr.rectangle([x, y, x + 18, y + 12], fill=(170, 170, 170))
    dr.text((x + 24, y - 2), "多视口重叠", font=f_sm, fill="#222222")

    # 底部：车体与坐标说明
    dr.text((10, out.height - 26),
            "黑框=车体 5000x2000mm（后轴原点），+X 向上为车头，+Y 向左",
            font=f_sm, fill="#666666")

    dst = d / "05_bev_legend.png"
    out.save(dst)
    print(f"{dst}  {out.size}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
