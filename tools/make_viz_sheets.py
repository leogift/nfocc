#!/usr/bin/env python
"""把 viz/*.png 拼成四张分类总览图，每类一行四路。

    python tools/make_viz_sheets.py
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

VIZ = Path("/Volumes/Workspace/NFOCC/viz")
OUT = VIZ
# 6 视口（与 nfocc_viz 的 kViews 一致）
VIEWS = ["FV_p-45", "FV_p+45", "RV_p-45", "RV_p+45", "ML", "MR"]
VIEWS_LABEL = ["FV 左前 -45°", "FV 右前 +45°", "RV 左后 -135°",
               "RV 右后 +135°", "ML +90°", "MR -90°"]
CAMS = ["FV", "MVL", "MVR", "RV"]

try:
    FONT = ImageFont.truetype("/System/Library/Fonts/Hiragino Sans GB.ttc", 26)
    FONT_S = ImageFont.truetype("/System/Library/Fonts/Hiragino Sans GB.ttc", 20)
except Exception:
    FONT = FONT_S = ImageFont.load_default()


def sheet(name, files, title, note, target_w=1180, pad=14):
    ims = []
    for f in files:
        p = VIZ / f
        if not p.exists():
            continue
        im = Image.open(p).convert("RGB")
        s = target_w / im.width
        ims.append((f, im.resize((target_w, int(im.height * s)))))
    if not ims:
        print(f"skip {name}: 无输入")
        return
    head = 92
    total_h = head + sum(im.height + 46 + pad for _, im in ims)
    canvas = Image.new("RGB", (target_w + pad * 2, total_h), "white")
    dr = ImageDraw.Draw(canvas)
    dr.text((pad, 14), title, font=FONT, fill="#111111")
    dr.text((pad, 52), note, font=FONT_S, fill="#777777")
    y = head
    for f, im in ims:
        canvas.paste(im, (pad, y + 30))
        dr.text((pad, y + 2), f, font=FONT_S, fill="#444444")
        y += im.height + 46 + pad
    out = OUT / name
    canvas.save(out)
    print(f"{out}  {canvas.size}")


# 1) Y 通道
sheet("sheet_1_y.png",
      [f"01_y_{c}.png" for c in CAMS],
      "① Y 通道灰度图（BT.601，Y=0.299R+0.587G+0.114B）",
      "原图 1280x966，取自 hanqtech 的 RGB PNG；后续所有匹配都在 Y 通道上做")

# 2) 视口投影
sheet("sheet_2_view.png",
      [f"02_view_{v}.png" for v in VIEWS],
      "② 视口投影（当前帧，6 视口，640x297，f=85.74，红色=无效）",
      "虚拟相机 pitch=0/roll=0，水平 150°/垂直 120°，yaw 按 6 方向；真实外参只决定采样到哪张鱼眼")

# 3) 前一帧对齐
sheet("sheet_3_view_ref.png",
      [f"03_view_ref_{v}.png" for v in VIEWS],
      "③ 前一帧对齐到当前帧视口（6 视口，同一 LUT，红色=无效）",
      "两帧共用同一视口 LUT，故已在视口方向对齐；视差体现为沿径向的缩小")

# 4) 视差
sheet("sheet_4_disp.png",
      [f"04_disp_{v}.png" for v in VIEWS],
      "④ 视差图（6 视口，红=近，蓝=远，黑=无效）",
      "SGBM，numDisp=256，d_max=100.9px；有效像素占比 13-19%")

# 5) BEV 单独
p = VIZ / "05_bev.png"
if p.exists():
    im = Image.open(p).convert("RGB")
    canvas = Image.new("RGB", (im.width + 28, im.height + 92), "white")
    dr = ImageDraw.Draw(canvas)
    dr.text((14, 14), "⑤ Voxel 分层 BEV（俯视，x 向上为车头 +X）",
            font=FONT, fill="#111111")
    dr.text((14, 52),
            "每像素 = 200mm 体素柱；颜色按该柱占用层数（蓝=1 层，红=6+ 层）；黑框=车体 5000x2000",
            font=FONT_S, fill="#777777")
    canvas.paste(im, (14, 86))
    out = VIZ / "sheet_5_bev.png"
    canvas.save(out)
    print(f"{out}  {canvas.size}")
