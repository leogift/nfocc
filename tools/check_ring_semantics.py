#!/usr/bin/env python
"""判定环带语义：在真实 FV 图像上，两种解释各投影到哪个像素。

环带 10-60° 可能有两种含义：
  A 相对光轴（当前实现）：视口俯角 = 安装俯角 + 环带角
  B 绝对俯角（相对地面）：视口俯角 = 环带角，与安装角无关

用真实标定把两种解释下的地面点投到图像，看谁落在合理的路面区域。
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image, ImageDraw

BASE = "/Volumes/Workspace/NFOCC/datasets/fisheye"


def load(nm):
    d = json.load(open(f"{BASE}/calibration/calibration/{nm}.json"))
    M = R.from_quat(d["extrinsic"]["quaternion"]).as_matrix()
    t = np.array(d["extrinsic"]["translation"]) * 1000.0
    i = d["intrinsic"]
    return dict(R=M, t=t, k1=i["k1"], k2=i["k2"], k3=i["k3"], k4=i["k4"],
                cx=i["cx_offset"] + i["width"]/2 - 0.5,
                cy=i["cy_offset"] + i["height"]/2 - 0.5,
                w=int(i["width"]), h=int(i["height"]), name=d["name"])


def proj(C, Pw):
    rel = Pw - C["t"]
    Pc = C["R"].T @ rel
    X, Y, Z = Pc
    chi = np.hypot(X, Y)
    th = np.arctan2(chi, Z)
    rho = C["k1"]*th + C["k2"]*th**2 + C["k3"]*th**3 + C["k4"]*th**4
    if chi > 1e-12:
        up, vp = rho*X/chi, rho*Y/chi
    else:
        up = vp = 0.0
    return up + C["cx"], vp + C["cy"]


for nm in ("00164_FV", "00167_RV"):
    C = load(nm)
    z = C["R"][:, 2]
    el_inst = np.degrees(np.arcsin(-z[2]))
    az_inst = np.degrees(np.arctan2(z[1], z[0]))
    print(f"\n{'='*70}\n{nm} ({C['name']}): 安装俯角 {el_inst:.1f}°, 方位 {az_inst:.1f}°")
    print(f"{'='*70}")
    print(f"{'解释':<22}{'视口俯角(绝对)':<18}{'地面距离mm':<14}{'投影像素':<20}")
    for label, base_el in (("A 相对光轴", el_inst), ("B 绝对俯角", 0.0)):
        for el_ring in (10, 35, 60):
            el_abs = base_el + el_ring
            if el_abs >= 89:
                continue
            # 地面点: 沿光轴方位, 距离 = h/tan(el_abs)
            h = C["t"][2]
            d = h / np.tan(np.radians(el_abs))
            a = np.radians(az_inst)
            Pw = np.array([C["t"][0] + d*np.cos(a),
                           C["t"][1] + d*np.sin(a), 0.0])
            u, v = proj(C, Pw)
            inside = (0 <= u < C["w"]) and (0 <= v < C["h"])
            print(f"{label:<22}{el_abs:<18.1f}{d:<14.0f}"
                  f"({u:7.1f},{v:6.1f}) {'图内' if inside else '出图'}  环带{el_ring}°")

print()
print("=" * 70)
print("在真实图像上标出两种解释的投影点，便于目视判断")
print("=" * 70)
C = load("00164_FV")
im = Image.open(f"{BASE}/rgb_images/00164_FV.png").convert("RGB")
dr = ImageDraw.Draw(im)
z = C["R"][:, 2]
el_inst = np.degrees(np.arcsin(-z[2]))
az_inst = np.degrees(np.arctan2(z[1], z[0]))
h = C["t"][2]
colors = {"A": (255, 60, 60), "B": (40, 200, 80)}
for label, base_el in (("A", el_inst), ("B", 0.0)):
    for el_ring in range(10, 61, 10):
        el_abs = base_el + el_ring
        if el_abs >= 88:
            continue
        for daz in (-20, 0, 20):
            a = np.radians(az_inst + daz)
            d = h / np.tan(np.radians(el_abs))
            Pw = np.array([C["t"][0] + d*np.cos(a), C["t"][1] + d*np.sin(a), 0.0])
            u, v = proj(C, Pw)
            if 0 <= u < C["w"] and 0 <= v < C["h"]:
                r = 6 if daz == 0 else 3
                dr.ellipse([u-r, v-r, u+r, v+r], outline=colors[label], width=2)
im.save("/Volumes/Workspace/NFOCC/docs/ring_semantics_check.png")
print("saved docs/ring_semantics_check.png")
print("红点 = 解释A(相对光轴), 绿点 = 解释B(绝对俯角), 每 10° 一个, 方位 -20/0/+20")
