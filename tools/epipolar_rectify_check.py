#!/usr/bin/env python3
"""P0 验收探针: 运动双目在 4 路环视下的极线方向, 以及极线重定向的效果与代价。

背景(见 notes/01-numcheck.md C12/C13)
------------------------------------
运动双目的极线方向由**帧间平移方向**决定, 与图像行无关。SGBM / cost-volume 网络只沿水平搜索。
直行时侧相机的视差方向接近垂直 -> 直接上 SGBM 完全失效。

三种处理方式对比
---------------
raw    : 不重定向, 视口光轴固定朝向车身外。
rot2d  : 把像平面绕**原光轴**转一个角度。能压掉平均方向, 压不掉方向散布(受俯仰影响)。
rot3d  : 构造新相机坐标系, 使 t 在新系下 z 分量为 0(x'=t, y'=z_old×x', z'=x'×y')。
         任何深度的点都只有水平像移, 垂直分量严格 0。
         代价: 新光轴被顶到与 t 垂直, 与原光轴的夹角 = 原光轴与 t 的夹角,
         相机有俯仰时视场中心偏移, 用"视场保留率"量化。

判据
----
重定向后视差垂直分量 < 2 px(SGBM 的 blockSize 与行带能容忍 ±2 px 错位), 且视场保留率 > 50%。

用法
----
    python3 tools/epipolar_rectify_check.py
    python3 tools/epipolar_rectify_check.py --yaw 30
    python3 tools/epipolar_rectify_check.py --pitch 0    # 去掉俯仰看纯几何
"""
from __future__ import annotations

import argparse
import math

import numpy as np

BODY_HALF_LEN = 2400.0
BODY_HALF_WID = 950.0
CAM_HEIGHT = 700.0
CAMS = {
    "FV": {"pos": np.array([BODY_HALF_LEN, 0.0, CAM_HEIGHT]), "yaw": 0.0},
    "RV": {"pos": np.array([-BODY_HALF_LEN, 0.0, CAM_HEIGHT]), "yaw": 180.0},
    "ML": {"pos": np.array([0.0, BODY_HALF_WID, CAM_HEIGHT]), "yaw": 90.0},
    "MR": {"pos": np.array([0.0, -BODY_HALF_WID, CAM_HEIGHT]), "yaw": -90.0},
}
FOCAL_PX = 400.0
PITCH_DEG = -10.0
FOV_H_DEG = 120.0
FOV_V_DEG = 90.0
TOL_VERT_PX = 2.0


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array(
        [[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]]
    )


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array(
        [[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]]
    )


def cam_R(cam: dict, pitch_deg: float = PITCH_DEG) -> np.ndarray:
    """相机轴(车辆系下)排成矩阵的行。相机轴 OpenCV: x 右 / y 下 / z 前。

    project() 用 (P - C) @ R 得到相机系坐标, 故 R 的第 i 行 = 相机第 i 轴在车辆系的方向。
    """
    return rot_z(cam["yaw"]) @ rot_y(-pitch_deg) @ np.diag([1.0, -1.0, 1.0])


def project(P: np.ndarray, C: np.ndarray, R: np.ndarray, f: float) -> np.ndarray:
    pc = (P - C) @ R
    z = pc[:, 2]
    ok = z > 1e-6
    uv = np.full((len(P), 2), np.nan)
    uv[ok, 0] = f * pc[ok, 0] / z[ok]
    uv[ok, 1] = f * pc[ok, 1] / z[ok]
    return uv


def sample_points(cam: dict, n: int = 6000, seed: int = 1, pitch_deg: float = PITCH_DEG) -> np.ndarray:
    """在相机原视锥内采样 3D 点(车辆系)。点集固定, 三种 mode 用同一批点, 保证可比。"""
    rng = np.random.default_rng(seed)
    hh, hv = math.radians(FOV_H_DEG / 2), math.radians(FOV_V_DEG / 2)
    az, el = rng.uniform(-hh, hh, n), rng.uniform(-hv, hv, n)
    r = rng.uniform(500.0, 5000.0, n)
    d_cam = np.stack([np.sin(az) * np.cos(el), np.sin(el), np.cos(az) * np.cos(el)], axis=1)
    d_veh = d_cam @ cam_R(cam, pitch_deg)
    return cam["pos"][None, :] + d_veh * r[:, None]


def redirect_3d(R: np.ndarray, t_veh: np.ndarray) -> np.ndarray:
    """3D 极线重定向: 新相机系下 t 的 z 分量为 0, 视差严格沿新 x 轴。

    x' = normalize(t_c); y' = normalize(z_old × x'); z' = x' × y'。
    z' 因此严格垂直 t, 与原光轴夹角 = 原光轴与 t 的夹角(相机俯仰时视场中心会偏移)。
    约定: 车辆系向量 v 在相机系的坐标 = R^T v (因为 p_cam = R_axis_rows @ (P-C) = (P-C) @ R)。
    """
    t_c = R.T @ t_veh
    nt = np.linalg.norm(t_c)
    if nt < 1e-9:
        return R
    x_new = t_c / nt
    z_old = np.array([0.0, 0.0, 1.0])
    y_new = np.cross(z_old, x_new)
    ny = np.linalg.norm(y_new)
    if ny < 1e-6:
        return R  # 平移沿光轴: FoE 退化
    y_new /= ny
    z_new = np.cross(x_new, y_new)
    if z_new @ z_old < 0:
        y_new, z_new = -y_new, -z_new
    return np.stack([x_new, y_new, z_new], axis=0)


def measure(cam: dict, t_veh: np.ndarray, mode: str, pitch_deg: float = PITCH_DEG):
    """返回 (视差中位 px, 垂直分量中位 px, 仍落在新视场内的比例)。"""
    P = sample_points(cam, pitch_deg=pitch_deg)
    R = cam_R(cam, pitch_deg)
    C = cam["pos"]
    uv1 = project(P, C, R, FOCAL_PX)
    uv2 = project(P, C + t_veh, R, FOCAL_PX)
    good = np.isfinite(uv1).all(axis=1) & np.isfinite(uv2).all(axis=1)
    d0 = (uv2 - uv1)[good]
    P_ok = P[good]
    mag = np.linalg.norm(d0, axis=1)
    sel = mag > 2.0
    d0, mag, P_ok = d0[sel], mag[sel], P_ok[sel]
    if len(d0) < 20:
        return float("nan"), float("nan"), 0.0

    if mode == "raw":
        d, P_use, R_use = d0, P_ok, R
    elif mode == "rot2d":
        t_c = R.T @ t_veh
        phi = math.atan2(t_c[1], -t_c[0])
        c, s = math.cos(phi), math.sin(phi)
        d, P_use, R_use = d0 @ np.array([[c, -s], [s, c]]).T, P_ok, R
    elif mode == "rot3d":
        R_use = redirect_3d(R, t_veh)
        u1 = project(P_ok, C, R_use, FOCAL_PX)
        u2 = project(P_ok, C + t_veh, R_use, FOCAL_PX)
        ok = np.isfinite(u1).all(axis=1) & np.isfinite(u2).all(axis=1)
        d, P_use = (u2 - u1)[ok], P_ok[ok]
    else:
        raise ValueError(mode)

    # 视场保留率: 重定向后仍落在 ±FOV/2 视锥内的点占比(相对原始有效点)
    pc = (P_use - C) @ R_use
    z = pc[:, 2]
    keep = z > 1e-6
    if keep.sum():
        ax = np.abs(np.degrees(np.arctan2(pc[keep, 0], z[keep])))
        ay = np.abs(np.degrees(np.arctan2(pc[keep, 1], z[keep])))
        inside = (ax <= FOV_H_DEG / 2) & (ay <= FOV_V_DEG / 2)
        keep_rate = float(inside.mean())
    else:
        keep_rate = 0.0
    return float(np.median(np.linalg.norm(d, axis=1))), float(np.median(np.abs(d[:, 1]))), keep_rate


def report(yaw_deg: float, pitch_deg: float) -> None:
    t = rot_z(yaw_deg) @ np.array([1000.0, 0.0, 0.0])
    print(f"=== 运动方向偏转 {yaw_deg:+.0f} deg / 平移 1000 mm / f={FOCAL_PX:.0f} px/rad / 俯仰 {pitch_deg:+.0f} deg ===")
    print(
        f"{'cam':>4} {'视差px':>7} {'raw垂直':>8} {'rot2d垂直':>10}"
        f" {'rot3d垂直':>10} {'rot3d视场保留':>13} {'合格?':>6}"
    )
    for name, cam in CAMS.items():
        m0, v0, _ = measure(cam, t, "raw", pitch_deg)
        _, v1, _ = measure(cam, t, "rot2d", pitch_deg)
        _, v2, keep = measure(cam, t, "rot3d", pitch_deg)
        ok = "yes" if (v2 <= TOL_VERT_PX and keep > 0.5) else "NO"
        print(f"{name:>4} {m0:>7.1f} {v0:>8.2f} {v1:>10.2f} {v2:>10.2f} {keep:>12.1%} {ok:>6}")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yaw", type=float, default=None)
    ap.add_argument("--pitch", type=float, default=PITCH_DEG)
    args = ap.parse_args()
    if args.yaw is not None:
        report(args.yaw, args.pitch)
        return
    for y in (0, 20, 45, 70, 90):
        report(float(y), args.pitch)


if __name__ == "__main__":
    main()
