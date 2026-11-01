#!/usr/bin/env python
"""验证 hanqtech 标定：投影公式、主点约定、外参轴向。

先证明"我对标定的理解是对的"，再拿去改 C++ 实现。
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image

BASE = "/Volumes/Workspace/NFOCC/datasets/fisheye"


def load(name):
    d = json.load(open(f"{BASE}/calibration/calibration/{name}.json"))
    e, i = d["extrinsic"], d["intrinsic"]
    Rm = R.from_quat(e["quaternion"]).as_matrix()
    t = np.array(e["translation"], float) * 1000.0     # m -> mm
    intr = dict(w=int(i["width"]), h=int(i["height"]),
                cx=i["cx_offset"], cy=i["cy_offset"],
                k1=i["k1"], k2=i["k2"], k3=i["k3"], k4=i["k4"],
                ar=i["aspect_ratio"])
    return d["name"], Rm, t, intr


def project(intr, Pc):
    """hanqtech 投影: 相机系 3D -> 像素。严格按 readme 的公式。"""
    X, Y, Z = Pc
    chi = np.hypot(X, Y)
    theta = np.arctan2(chi, Z)
    k1, k2, k3, k4 = intr["k1"], intr["k2"], intr["k3"], intr["k4"]
    rho = k1 * theta + k2 * theta**2 + k3 * theta**3 + k4 * theta**4
    if chi != 0:
        up = rho * X / chi
        vp = rho * Y / chi
    else:
        up = vp = 0.0
    u = up + intr["cx"] + intr["w"] / 2.0 - 0.5
    v = vp * intr["ar"] + intr["cy"] + intr["h"] / 2.0 - 0.5
    return u, v, rho, theta


def unproject(intr, u, v):
    """像素 -> 相机系单位射线。rho 多项式反解 theta (牛顿法)。"""
    k1, k2, k3, k4 = intr["k1"], intr["k2"], intr["k3"], intr["k4"]
    up = u - intr["cx"] - intr["w"] / 2.0 + 0.5
    vp = (v - intr["cy"] - intr["h"] / 2.0 + 0.5) / intr["ar"]
    rho = np.hypot(up, vp)
    # 反解 rho(theta) = rho
    th = rho / k1 if k1 else 0.0
    for _ in range(60):
        f = k1*th + k2*th**2 + k3*th**3 + k4*th**4 - rho
        df = k1 + 2*k2*th + 3*k3*th**2 + 4*k4*th**3
        if abs(df) < 1e-12:
            break
        step = f / df
        th -= step
        if abs(step) < 1e-12:
            break
    if chi_ok := (rho > 1e-9):
        X = up / rho * np.sin(th)
        Y = vp / rho * np.sin(th)
    else:
        X = Y = 0.0
    Z = np.cos(th)
    d = np.array([X, Y, Z])
    return d / np.linalg.norm(d), th


print("=" * 74)
print("1. 图像实际尺寸 vs 标定")
print("=" * 74)
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, Rm, t, intr = load(nm)
    im = Image.open(f"{BASE}/rgb_images/{nm}.png")
    ok = (im.size == (intr["w"], intr["h"]))
    print(f"  {nm} ({name:3s}): 标定 {intr['w']}x{intr['h']}, "
          f"实际 {im.size[0]}x{im.size[1]}  {'OK' if ok else '!! 不一致'}")

print()
print("=" * 74)
print("2. 外参: 光轴朝向 + 安装俯角/方位角")
print("=" * 74)
AZ = {"FV": 0, "MVL": 90, "MVR": -90, "RV": 180}
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, Rm, t, intr = load(nm)
    z = Rm[:, 2]                       # 相机 z 轴在车系
    az = np.degrees(np.arctan2(z[1], z[0]))
    el = np.degrees(np.arcsin(-z[2]))  # 向下俯为正
    print(f"  {name:3s}: 光心=({t[0]:7.1f},{t[1]:7.1f},{t[2]:6.1f})mm  "
          f"光轴=({z[0]:+.3f},{z[1]:+.3f},{z[2]:+.3f})  "
          f"方位{az:+7.1f}°  俯角{el:+5.1f}°  "
          f"(名义{AZ[name]:+d}°)")

print()
print("=" * 74)
print("3. 投影/反投影自洽 (往返误差)")
print("=" * 74)
name, Rm, t, intr = load("00164_FV")
worst = 0.0
for u in np.linspace(100, intr["w"] - 100, 7):
    for v in np.linspace(100, intr["h"] - 100, 6):
        d, th = unproject(intr, u, v)
        u2, v2, _, _ = project(intr, d)
        e = np.hypot(u2 - u, v2 - v)
        worst = max(worst, e)
print(f"  往返最大误差 = {worst:.3e} px")

print()
print("=" * 74)
print("4. 有效视场 (theta 上限, 由 rho 单调性定)")
print("=" * 74)
for nm in ("00164_FV", "00167_RV"):
    name, Rm, t, intr = load(nm)
    k1, k2, k3, k4 = intr["k1"], intr["k2"], intr["k3"], intr["k4"]
    ths = np.linspace(0, np.pi / 2, 2000)
    rho = k1*ths + k2*ths**2 + k3*ths**3 + k4*ths**4
    mono = np.all(np.diff(rho) > 0)
    imax = np.argmax(rho)
    mth = ths[imax] if not mono else ths[-1]
    # 图像四角对应的 theta
    corners = [(0, 0), (intr["w"], 0), (0, intr["h"]), (intr["w"], intr["h"])]
    cth = []
    for cu, cv in corners:
        _, th = unproject(intr, cu, cv)
        cth.append(np.degrees(th))
    print(f"  {name:3s}: 多项式{'单调' if mono else f'在 {np.degrees(mth):.1f}° 达峰'}; "
          f"四角 theta = {', '.join(f'{c:.1f}°' for c in cth)}")

print()
print("=" * 74)
print("5. 主点约定核对 (cx_offset 很小 => 主点接近几何中心)")
print("=" * 74)
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, Rm, t, intr = load(nm)
    pcx = intr["cx"] + intr["w"] / 2.0 - 0.5
    pcy = intr["cy"] + intr["h"] / 2.0 - 0.5
    print(f"  {name:3s}: 实际主点 = ({pcx:7.2f}, {pcy:7.2f})  "
          f"图像中心 = ({intr['w']/2:.1f}, {intr['h']/2:.1f})")
