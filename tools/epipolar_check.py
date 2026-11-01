#!/usr/bin/env python3
"""验证 C12/C13: 运动双目在 4 路环视下的极线方向, 以及 SGBM 水平搜索的适用范围。

命题
----
P1 极线方向由**平移方向**决定, 不是由图像行决定。
P2 直行时前/后相机的极线接近水平, 侧相机(ML/MR)接近垂直。
P3 因此 SGBM(只沿水平方向搜索) 直行时对侧相机不可用。

做法(必须这样做, 否则会得出错误结论)
------------------------------------
不能只看"相机光心平移向量"本身 —— 它和相机坐标系无关, 投影到焦平面永远是同一个方向,
会把所有相机算出相同的 0 deg。错在这里过: 见 tools/ 的 git 历史注释。

正确做法是**把视场内的一批 3D 点投影到两帧图像上, 求像点位移(视差)的主方向**:
  1. 在世界(车辆)坐标系里采样视场内的点 P (0.5-5 m, 覆盖视口 FOV 的锥角);
  2. 帧 1: 相机在 C, 位姿 R; 帧 2: 相机在 C + t, 同一 R (纯平移);
  3. 两次投影, 得像点 p1, p2; disp = p2 - p1;
  4. 视差的**主方向**(所有点 disp 方向的平均, 用绝对值消符号)就是极线在图像上的倾角。

若倾角小, 视差主要沿水平 → SGBM 可用; 若接近 90 deg → SGBM 失效。

用法
----
    python3 tools/epipolar_check.py             # 扫多个运动方向
    python3 tools/epipolar_check.py --yaw 30    # 单个运动方向
"""
from __future__ import annotations

import argparse
import math

import numpy as np

# 360 环视典型布局 (mm)。车系 X 前 / Y 左 / Z 上。
BODY_HALF_LEN = 2400.0
BODY_HALF_WID = 950.0
CAM_HEIGHT = 700.0

CAMS = {
    "FV": {"pos": np.array([BODY_HALF_LEN, 0.0, CAM_HEIGHT]), "yaw": 0.0},
    "RV": {"pos": np.array([-BODY_HALF_LEN, 0.0, CAM_HEIGHT]), "yaw": 180.0},
    "ML": {"pos": np.array([0.0, BODY_HALF_WID, CAM_HEIGHT]), "yaw": 90.0},
    "MR": {"pos": np.array([0.0, -BODY_HALF_WID, CAM_HEIGHT]), "yaw": -90.0},
}

FOCAL_PX = 400.0  # 视口等效焦距 (px/rad) 量级
PITCH_DEG = -10.0  # 相机略向下俯, 典型环视安装
FOV_H_DEG = 120.0  # 单相机视口水平张角(展开后)
FOV_V_DEG = 90.0


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(deg: float) -> np.ndarray:
    """绕 Y(左上) 旋转 = 俯仰。正角抬头。"""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def cam_R(cam: dict) -> np.ndarray:
    """相机->车辆 的旋转 (车辆系为基)。相机轴 OpenCV: x 右 / y 下 / z 前。"""
    # 先按 yaw 把 z 轴指向该相机的朝向, 再叠加俯仰
    return rot_z(cam["yaw"]) @ rot_y(-PITCH_DEG) @ np.diag([1.0, -1.0, 1.0])


def project(P: np.ndarray, C: np.ndarray, R_cv: np.ndarray, f: float) -> np.ndarray:
    """P(N,3) 世界点 -> 图像 (N,2)。R_cv 是 相机->世界; 这里求 世界->相机 用转置。"""
    pc = (P - C) @ R_cv  # 等价于 R^T (P - C) 逐行
    z = pc[:, 2]
    ok = z > 1e-6
    u = np.full(len(P), np.nan)
    v = np.full(len(P), np.nan)
    u[ok] = f * pc[ok, 0] / z[ok]
    v[ok] = f * pc[ok, 1] / z[ok]
    return np.stack([u, v], axis=1)


def sample_points(cam: dict, n: int = 4000, seed: int = 0) -> np.ndarray:
    """在相机视锥内采样 3D 点 (世界系)。锥角按 FOV 生成, 距离 0.5-5 m。"""
    rng = np.random.default_rng(seed)
    half_h = math.radians(FOV_H_DEG / 2)
    half_v = math.radians(FOV_V_DEG / 2)
    az = rng.uniform(-half_h, half_h, n)
    el = rng.uniform(-half_v, half_v, n)
    rng_m = rng.uniform(500.0, 5000.0, n)
    # 相机系方向
    d_cam = np.stack([np.sin(az) * np.cos(el), np.sin(el), np.cos(az) * np.cos(el)], axis=1)
    R = cam_R(cam)
    d_veh = d_cam @ R.T
    return cam["pos"][None, :] + d_veh * rng_m[:, None]


def epipolar_angle(cam: dict, t_veh: np.ndarray) -> tuple[float, float, float]:
    """返回 (极线倾角 deg, 视差像素中位数, 有效点比例)。"""
    P = sample_points(cam)
    R = cam_R(cam)
    p1 = project(P, cam["pos"], R, FOCAL_PX)
    p2 = project(P, cam["pos"] + t_veh, R, FOCAL_PX)
    disp = p2 - p1
    good = np.isfinite(disp).all(axis=1)
    disp = disp[good]
    if len(disp) == 0:
        return float("nan"), float("nan"), 0.0
    mag = np.linalg.norm(disp, axis=1)
    sel = mag > 0.5  # 只看有实际视差的点, 否则方向噪声大
    if sel.sum() < 10:
        return float("nan"), float(np.median(mag)), float(sel.mean())
    # 主方向: 用绝对值消掉"靠近/远离"的符号, 再做加权平均方向
    a = np.abs(disp[sel])
    w = mag[sel]
    mean_dir = (a * w[:, None]).sum(axis=0) / w.sum()
    ang = math.degrees(math.atan2(mean_dir[1], mean_dir[0]))  # 0=水平, 90=垂直
    return ang, float(np.median(mag)), float(good.mean())


def report(yaw_deg: float) -> None:
    t_veh = rot_z(yaw_deg) @ np.array([1000.0, 0.0, 0.0])  # 1000 mm 平移
    print(f"=== 运动方向 = 车头偏转 {yaw_deg:+.0f} deg, 平移 1000 mm, f={FOCAL_PX:.0f} px/rad ===")
    print(f"{'cam':>4} {'极线倾角':>10} {'中位视差px':>12} {'SGBM':>8}")
    for name, cam in CAMS.items():
        ang, dpx, ratio = epipolar_angle(cam, t_veh)
        if not math.isfinite(ang):
            verdict = "n/a"
        elif ang <= 10.0:
            verdict = "yes"
        elif ang <= 30.0:
            verdict = "marginal"
        else:
            verdict = "NO"
        print(f"{name:>4} {ang:>9.1f}d {dpx:>12.1f} {verdict:>8}")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yaw", type=float, default=None)
    args = ap.parse_args()
    if args.yaw is not None:
        report(args.yaw)
    else:
        for y in (0, 20, 45, 70, 90):
            report(float(y))


if __name__ == "__main__":
    main()
