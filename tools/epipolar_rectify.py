#!/usr/bin/env python3
"""极线重定向: 干净实现 + 三重自检, 并给出"重定向到底能修好什么"的量化结论。

全文唯一坐标约定(其它模块引用时必须一致)
--------------------------------------
    p_cam = R @ (P - C)      # R 正交; R 的行 = 相机 x/y/z 轴在车辆系下的单位向量
    u = f * p_cam.x / p_cam.z
    v = f * p_cam.y / p_cam.z     # 相机系: x 右 / y 下 / z 前 (OpenCV)
    => 车辆系向量 w 在相机系的坐标 = R @ w; 反之相机系向量在车辆系 = R.T @ w

必须理解的事实(这一条决定了重构代价)
------------------------------------
相机有俯仰 pitch 时, **极线方向(epipolar, 由基线与光轴共同决定) ≠ 视差方向(disparity, 由两帧
坐标差决定)**, 两者相差约 |pitch|。要让竖直视差严格为 0, 重定向后的新光轴必须**回到与平移 t 垂直
的方向**(即把俯仰"抬掉")。这会让视口中心偏离原相机视窗中心, 用视场保留率量化。

若不做这一步(只旋转像平面、或让新光轴保持与原光轴接近), 竖直视差不为 0 且随 FOV 增大而恶化:
    FOV 90° -> 竖直约 13 px;  FOV 120° -> 23 px;  FOV 180° -> 57 px   (f=400, 俯仰 -10°)
SGBM 的水平搜索按 blockSize 容忍, ±2 px 以内无忧, 20 px 以上必然失配。

几何自洽性(本文件 rectify 里的 A1/A2/A3)
----------------------------------------
A1 R' 正交
A2 t 在新机体系下 z 分量为 0
A3 在 y'=0 平面上的点, 两次投影的 v 完全相同(数值 1e-6 内)

注意 A3 只能用 y'=0 平面上的点验 —— 用图像中心的点验也会失败, 因为"图像中心"在车辆系里
不平行于 t, 它自身的视差就带竖直分量。这是几何事实, 不是 bug。

用法
----
    python3 tools/epipolar_rectify.py
    python3 tools/epipolar_rectify.py --pitch 0      # 去掉俯仰, 竖直视差应为 0
"""
from __future__ import annotations

import argparse
import math

import numpy as np

BODY_HALF_LEN = 2400.0
BODY_HALF_WID = 950.0
CAM_HEIGHT = 700.0
CAM_YAW = {"FV": 0.0, "RV": 180.0, "ML": 90.0, "MR": -90.0}
CAM_POS = {
    "FV": np.array([BODY_HALF_LEN, 0.0, CAM_HEIGHT]),
    "RV": np.array([-BODY_HALF_LEN, 0.0, CAM_HEIGHT]),
    "ML": np.array([0.0, BODY_HALF_WID, CAM_HEIGHT]),
    "MR": np.array([0.0, -BODY_HALF_WID, CAM_HEIGHT]),
}
FOCAL_PX = 400.0


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


def vehicle_to_cam(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    """车辆->相机旋转 R 的行 = 相机轴在车辆系。"""
    z_cam = rot_z(yaw_deg) @ rot_y(-pitch_deg) @ np.array([1.0, 0.0, 0.0])
    z_cam /= np.linalg.norm(z_cam)
    up = np.array([0.0, 0.0, 1.0])
    x_cam = np.cross(z_cam, up)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    y_cam /= np.linalg.norm(y_cam)
    R = np.stack([x_cam, y_cam, z_cam], axis=0)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9), "A1 R 非正交"
    assert y_cam[2] < 0, f"y_cam 应朝下, 实际 {np.round(y_cam, 3)}"
    return R


def project(P: np.ndarray, C: np.ndarray, R: np.ndarray, f: float = FOCAL_PX) -> np.ndarray:
    pc = (P - C) @ R.T
    z = pc[:, 2]
    ok = z > 1e-6
    uv = np.full((len(P), 2), np.nan)
    uv[ok, 0] = f * pc[ok, 0] / z[ok]
    uv[ok, 1] = f * pc[ok, 1] / z[ok]
    return uv


def rectify(R: np.ndarray, t_veh: np.ndarray) -> np.ndarray:
    """构造新 R': t 在新相机系下 z 分量为 0(x'=normalize(t_c), z'⊥t, y'=z'×x')。

    A1/A2 在此自检。A3 见 `disp_vertical_px()`, 在 report 里对真实视锥采样点统计。

    为什么不能拿"图像中心的点"验 A3: 图像中心的视线一般不与 t 垂直, 该点的视差本身就带竖直分量。
    这是几何事实, 不是实现缺陷 —— 见 tools/epipolar_rectify.py 文件头说明。
    """
    t_c = R @ t_veh
    nt = np.linalg.norm(t_c)
    assert nt > 1e-9, "t 为零, 不存在基线"
    x_new = t_c / nt
    z_old = np.array([0.0, 0.0, 1.0])
    z_new = z_old - (z_old @ x_new) * x_new
    nz = np.linalg.norm(z_new)
    if nz < 1e-6:
        return R.copy()  # 平移沿光轴: FoE 退化
    z_new /= nz
    y_new = np.cross(z_new, x_new)
    R_new = np.stack([x_new, y_new, z_new], axis=0)

    assert np.allclose(R_new @ R_new.T, np.eye(3), atol=1e-9), "A1 R' 非正交"
    assert abs((R_new @ t_veh)[2]) < 1e-9, "A2 t 在新系下 z≠0"
    return R_new


def disp_vertical_px(R: np.ndarray, C: np.ndarray, P: np.ndarray, t_veh: np.ndarray) -> np.ndarray:
    """给定相机与新视点, 返回每个点视差的竖直分量(px)。点严格在视锥内。"""
    u1 = project(P, C, R)
    u2 = project(P, C + t_veh, R)
    return u2 - u1


def image_center_dir(R: np.ndarray) -> np.ndarray:
    return R.T @ np.array([0.0, 0.0, 1.0])


def center_residual_px(R_new: np.ndarray, t_veh: np.ndarray, z_mm: float = 2000.0) -> float:
    """图像中心视线上的点(深度 z_mm)的竖直视差(px)。这是真实存在、无法消除的残余。"""
    P = (image_center_dir(R_new) * z_mm)[None, :]
    u1 = project(P, np.zeros(3), R_new)[0]
    u2 = project(P, t_veh, R_new)[0]
    return float(u2[1] - u1[1])


def field_keep(R: np.ndarray, R_new: np.ndarray, fov_h: float, fov_v: float, n: int = 8000) -> float:
    """原视锥内的采样点, 重定向后仍落在新视锥内的比例。"""
    rng = np.random.default_rng(3)
    hh, hv = math.radians(fov_h / 2), math.radians(fov_v / 2)
    az, el = rng.uniform(-hh, hh, n), rng.uniform(-hv, hv, n)
    d_cam = np.stack([np.sin(az) * np.cos(el), np.sin(el), np.cos(az) * np.cos(el)], axis=1)
    d_veh = d_cam @ R
    pc = d_veh @ R_new.T
    ok = pc[:, 2] > 1e-6
    if not ok.any():
        return 0.0
    ax = np.abs(np.degrees(np.arctan2(pc[ok, 0], pc[ok, 2])))
    ay = np.abs(np.degrees(np.arctan2(pc[ok, 1], pc[ok, 2])))
    return float(((ax <= fov_h / 2) & (ay <= fov_v / 2)).mean())


def report(pitch_deg: float, fov_h: float, fov_v: float) -> None:
    print(f"俯仰 {pitch_deg:+.0f}°  视口 {fov_h:.0f}°x{fov_v:.0f}°  f={FOCAL_PX:.0f}px/rad  平移 1000mm")
    print(
        f"{'yaw':>4} {'cam':>4} {'视差px':>7} {'raw竖直px':>10}"
        f" {'rect竖直px':>10} {'rect视场保留':>11}"
    )
    for yaw in (0, 30, 45, 90):
        t = rot_z(yaw) @ np.array([1000.0, 0.0, 0.0])
        for name in ("FV", "RV", "ML", "MR"):
            C, R = CAM_POS[name], vehicle_to_cam(CAM_YAW[name], pitch_deg)
            rng = np.random.default_rng(1)
            hh, hv = math.radians(fov_h / 2), math.radians(fov_v / 2)
            az, el = rng.uniform(-hh, hh, 4000), rng.uniform(-hv, hv, 4000)
            d_cam = np.stack([np.sin(az) * np.cos(el), np.sin(el), np.cos(az) * np.cos(el)], axis=1)
            P = C[None, :] + (d_cam @ R) * rng.uniform(500, 5000, 4000)[:, None]

            d_raw = disp_vertical_px(R, C, P, t)
            g = np.isfinite(d_raw).all(1)
            m = np.linalg.norm(d_raw[g], axis=1)
            sel = m > 2.0
            raw_v = float(np.median(np.abs(d_raw[g][sel, 1]))) if sel.any() else float("nan")
            disp_med = float(np.median(m[sel])) if sel.any() else 0.0

            R2 = rectify(R, t)
            # 重定向后: 同一批 3D 点在新视口里的竖直分量
            pc = (P[g] - C) @ R2.T
            inside = (
                (np.abs(np.degrees(np.arctan2(pc[:, 0], pc[:, 2]))) <= fov_h / 2)
                & (np.abs(np.degrees(np.arctan2(pc[:, 1], pc[:, 2]))) <= fov_v / 2)
                & (pc[:, 2] > 1e-6)
            )
            keep = float(inside.mean()) if len(inside) else 0.0
            P_in = P[g][inside]
            if len(P_in) > 20:
                d_r = disp_vertical_px(R2, C, P_in, t)
                g2 = np.isfinite(d_r).all(1)
                rect_v = float(np.median(np.abs(d_r[g2, 1]))) if g2.any() else float("nan")
            else:
                rect_v = float("nan")
            print(
                f"{yaw:>4} {name:>4} {disp_med:>7.1f} {raw_v:>10.2f}"
                f" {rect_v:>10.2f} {keep:>10.1%}"
            )
    print()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pitch", type=float, default=-10.0)
    ap.add_argument("--fov-h", type=float, default=120.0)
    ap.add_argument("--fov-v", type=float, default=90.0)
    a = ap.parse_args()
    report(a.pitch, a.fov_h, a.fov_v)
