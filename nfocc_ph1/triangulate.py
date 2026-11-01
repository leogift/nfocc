"""NFOCC PH-1.2: 三角化 + sigma 传播.

**为什么不用 Z = f*b/d**
------------------------
`Z = f*b/d` 是"标准双目"的结论, 前提是**平移方向垂直于视线**。
本项目相机俯角 0°、车辆直行, 视线与平移方向的夹角从 0°(光轴) 到接近 90°(边缘),
两者不等, 套该式在光轴附近相对误差可达 10 倍(见 nfocc_ph0 的实测)。

正确做法是射线三角化:
    P = C0 + s0*r0 = C1 + s1*r1
    => [r0, -r1] [s0, s1]^T = t        (最小二乘, 对任意平移方向成立)
`Z = f*b/d` 仅作为快速近似保留在 `depth_fast()` 里, 并标注适用条件。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .engine import DisparityMap


@dataclass
class TriangulationResult:
    points_veh: np.ndarray      # (N,3) 车辆系 mm
    depth_mm: np.ndarray        # (N,) 参考帧距离
    sigma_mm: np.ndarray        # (N,) 深度不确定度
    sigma_d_px: float
    n_in: int                   # 输入像素数
    n_out: int                  # 有效点数


def triangulate_rays(C0: np.ndarray, r0: np.ndarray,
                     C1: np.ndarray, r1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """批量射线三角化。r0/r1 为 (N,3) 单位向量(同一坐标系)。返回 (s0, s1)。"""
    t = np.asarray(C1, dtype=np.float64) - np.asarray(C0, dtype=np.float64)
    r0 = np.asarray(r0, dtype=np.float64).reshape(-1, 3)
    r1 = np.asarray(r1, dtype=np.float64).reshape(-1, 3)
    a11 = np.einsum("ij,ij->i", r0, r0)
    a12 = -np.einsum("ij,ij->i", r0, r1)
    a22 = np.einsum("ij,ij->i", r1, r1)
    b1 = r0 @ t
    b2 = -r1 @ t
    det = a11 * a22 - a12 * a12
    s0 = np.full(len(r0), np.nan)
    s1 = np.full(len(r0), np.nan)
    ok = np.abs(det) > 1e-12
    s0[ok] = (a22[ok] * b1[ok] - a12[ok] * b2[ok]) / det[ok]
    s1[ok] = (a11[ok] * b2[ok] - a12[ok] * b1[ok]) / det[ok]
    return s0, s1


def effective_baseline(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    """有效基线 = t 在垂直于视线方向上的分量大小。sigma_Z 必须用它, 不是 |t|。

    前向平移时 b_perp << |t|, 直接拿 |t| 会把 sigma_Z 高估数倍。
    """
    r = np.asarray(r, dtype=np.float64).reshape(-1, 3)
    t = np.asarray(t, dtype=np.float64).reshape(3)
    if len(r) == 0:
        return np.zeros(0, dtype=np.float64)
    # 逐点: perp_i = t - (r_i·t) r_i
    # 这里必须用广播, np.outer 会把两个向量拉平后做外积, 形状完全错
    # (踩过: (N,3) 与 (3,) 的 outer -> (3N,3) 再广播失败)
    proj = (r @ t)[:, None] * r          # (N,3)
    perp = t[None, :] - proj
    return np.linalg.norm(perp, axis=1)


def sigma_depth(z_mm: np.ndarray, f_px: float, b_perp_mm: np.ndarray,
                sigma_d_px: float, sigma_b_ratio: float = 0.02) -> np.ndarray:
    """sigma_Z = sqrt( (Z^2/(f*b_perp))^2 * sigma_d^2 + (Z*sigma_b_ratio)^2 )。

    第一项: 视差误差传播 (dZ/dd = -Z^2/(f*b_perp))
    第二项: 基线尺度误差 (dZ/db = Z/b, 相对误差 1:1)
    """
    z = np.asarray(z_mm, dtype=np.float64)
    bp = np.maximum(np.asarray(b_perp_mm, dtype=np.float64), 1e-9)
    s_disp = z * z / (f_px * bp) * sigma_d_px
    s_base = z * sigma_b_ratio
    return np.sqrt(s_disp * s_disp + s_base * s_base)


def depth_fast(disp_px: np.ndarray, f_px: float, b_mm: float,
               sin_alpha: np.ndarray) -> np.ndarray:
    """快速近似 Z = f*b*sin(alpha)/d。

    **仍需 sin(alpha)**: 只有当视线与平移垂直(sin=1)时才退化成 Z=f*b/d。
    纯 f*b/d 在本项目的光轴附近不成立, 仅用于量级估计。
    """
    d = np.maximum(np.asarray(disp_px, dtype=np.float64), 1e-9)
    return f_px * b_mm * np.asarray(sin_alpha, dtype=np.float64) / d


def triangulate_disparity(dm: DisparityMap,
                          cam_unproject,
                          R_cv: np.ndarray,
                          C0: np.ndarray, C1: np.ndarray,
                          f_px: float,
                          sigma_d_px: float,
                          sigma_b_ratio: float = 0.02,
                          valid_extra: np.ndarray | None = None) -> TriangulationResult:
    """从视差图得到 3D 点。

    Args:
        dm          : 引擎输出(disp + valid)。**disp 视为无符号水平位移**。
        cam_unproject: 像素 -> 相机系单位射线的函数, 签名 unproject(uv)->(ray,ok)
        R_cv        : 车辆->相机旋转(行=相机轴), p_cam = R_cv @ (P - C)
        C0/C1       : 参考帧/当前帧相机光心(车辆系, mm)
        valid_extra : 额外有效性(如视口 LUT 的有效性), 与引擎 valid 取与

    注意配对方向: 本项目的真值是无符号 |du|, 但对应点可能在左或右。
    这里按"左半图向右搜、右半图向左搜"的镜像规则重建 u' —— 与 synth 的真值一致。
    """
    h, w = dm.disp.shape
    R_cv = np.asarray(R_cv, dtype=np.float64)
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64) + 0.5,
                         np.arange(h, dtype=np.float64) + 0.5)
    valid = dm.valid.copy()
    if valid_extra is not None:
        valid = valid & valid_extra

    ys, xs = np.nonzero(valid)
    if len(ys) == 0:
        return TriangulationResult(np.zeros((0, 3)), np.zeros(0), np.zeros(0),
                                   sigma_d_px, 0, 0)
    u = uu[ys, xs]
    v = vv[ys, xs]
    d = dm.disp[ys, xs].astype(np.float64)

    ray0_c, ok0 = cam_unproject(np.stack([u, v], axis=1))
    r0 = ray0_c @ R_cv

    # 镜像配对: 用真值一致的规则重建当前帧像素。
    # 判定依据: 该像素与"平移方向在车系中的横向分量 + 相机横轴"的关系。
    t_veh = np.asarray(C1, dtype=np.float64) - np.asarray(C0, dtype=np.float64)
    tx_cam = float(R_cv @ t_veh @ np.array([1.0, 0.0, 0.0])) if np.ndim(R_cv @ t_veh) == 1 else 0.0
    # 本项目约定: 相机向右移动 -> u 减小。左边(u<cx) 与右边镜像。
    # 具体方向由 t 在相机 x 轴的符号决定; 符号为正(向右移)时:
    #   左半图对应点在左(往 u 减小方向搜) -> u' = u - d
    #   右半图对应点在右                 -> u' = u + d
    cx = w / 2.0
    sign = 1.0 if tx_cam >= 0 else -1.0
    u2 = np.where(u < cx, u - sign * d, u + sign * d)

    ray1_c, ok1 = cam_unproject(np.stack([u2, v], axis=1))
    r1 = ray1_c @ R_cv

    s0, _ = triangulate_rays(C0, r0, C1, r1)
    good = ok0 & ok1 & np.isfinite(s0) & (s0 > 0)
    # 注意: 先把布尔 mask 转成索引再去筛, 否则 s0 已被重新赋值,
    # np.nonzero(good) 得到的是"位置序号"而非原数组索引 -> 广播形状错误。
    idx = np.nonzero(good)[0]
    if len(idx) == 0:
        return TriangulationResult(np.zeros((0, 3)), np.zeros(0), np.zeros(0),
                                   sigma_d_px, int(valid.sum()), 0)
    s0 = s0[idx]
    r0g = r0[idx]

    b_perp = effective_baseline(r0g, t_veh)
    sig = sigma_depth(s0, f_px, b_perp, sigma_d_px, sigma_b_ratio)

    pts = np.asarray(C0, dtype=np.float64)[None, :] + r0g * s0[:, None]
    return TriangulationResult(pts, s0, sig, sigma_d_px, int(valid.sum()), int(len(s0)))
