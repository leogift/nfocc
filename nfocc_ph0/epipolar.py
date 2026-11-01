"""NFOCC PH-0.3: 极线判定与视口重排方向(SPEC-EPIPOLAR §1/§3).

核心定理(本文档反复用, 也是全部代码的依据)
-------------------------------------------
两帧仅平移 t、无旋转时, 相机系下
    v1 - v2 = f (y t_z - z t_y) / (z (z - t_z))
对任意点恒为 0  **<=>  t_y = t_z = 0**
即: 平移必须既垂直于光轴、又沿图像横轴。

推论: 4 路环视任意时刻最多两路满足。直行时侧相机极线竖直, SGBM 全废。
      这不是边界情况, 是泊车工况的常态。

两级处理
--------
A. 极线重定向(rotate_camera): 造新相机系使 t 无 z 分量。
   能把竖直视差归零, 但视场中心被抬到与 t 垂直的方向, 夹角=光轴与 t 的夹角,
   因此夹角大时丢视场。适用于"偏一点点"。
B. 视口重排(reorient_view): 不改光轴, 只把视口横轴对齐 t 在像平面的投影。
   保留视场, 但视口每行的有效宽度不同(交给 SPEC-LUT 的 valid_x0)。
   这是本项目的默认做法。

用法
----
    python3 -m nfocc_ph0.epipolar
"""
from __future__ import annotations

import math

import numpy as np


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


def rot_y(deg: float) -> np.ndarray:
    """绕车辆 Y 轴(朝左)旋转。正角把 +X 压向 -Z(即抬头), 故装俯角用负值。"""
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)],
                     [0.0, 1.0, 0.0],
                     [-math.sin(a), 0.0, math.cos(a)]])


def vehicle_to_cam(yaw_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """车辆->相机的旋转 R, 满足 p_cam = R @ (P - C)。

    R 的**行**是相机 x/y/z 轴在车辆系下的单位向量(OpenCV: x 右 y 下 z 前)。
    三条自检(C1 正交 / y 朝下 / 光轴朝向)必须在正确输入上通过, 在错误输入上失败。
    """
    z_cam = rot_z(yaw_deg) @ rot_y(-pitch_deg) @ rot_z(roll_deg) @ np.array([1.0, 0.0, 0.0])
    z_cam = z_cam / np.linalg.norm(z_cam)
    up = np.array([0.0, 0.0, 1.0])
    x_cam = np.cross(z_cam, up)
    if np.linalg.norm(x_cam) < 1e-9:          # 光轴竖直: 退化, 取任意横轴
        x_cam = np.array([0.0, 1.0, 0.0])
    x_cam = x_cam / np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    y_cam = y_cam / np.linalg.norm(y_cam)
    R = np.stack([x_cam, y_cam, z_cam], axis=0)

    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9), "R 非正交"
    assert y_cam[2] < 1e-6, f"y 轴应朝下(车辆系 z 上), 实际 {np.round(y_cam, 3)}"
    horiz = rot_z(yaw_deg) @ np.array([1.0, 0.0, 0.0])
    assert z_cam @ horiz > 0.9, f"光轴朝向错(z_cam={np.round(z_cam, 3)})"
    return R


def cam_to_vehicle(yaw_deg: float, pitch_deg: float = 0.0) -> np.ndarray:
    """相机->车辆 = R^T(仅作可读性封装, 不要另立约定)。"""
    return vehicle_to_cam(yaw_deg, pitch_deg).T


# ---------------------------------------------------------------- 竖直视差
def disparity_vertical_px(t_cam: np.ndarray, P_cam: np.ndarray,
                          f: float) -> np.ndarray:
    """解析竖直视差(px), 用于验证定理 (1.1), 不依赖投影函数。

    由 v1 - v2 = f (y t_z - z t_y) / (z (z - t_z)) 直接算。
    z - t_z == 0 的点(平移后过光心)单独标 nan。
    """
    t = np.asarray(t_cam, dtype=np.float64)
    P = np.asarray(P_cam, dtype=np.float64).reshape(-1, 3)
    z = P[:, 2]
    num = P[:, 1] * t[2] - z * t[1]
    den = z * (z - t[2])
    out = f * num / np.where(np.abs(den) < 1e-12, np.nan, den)
    return out


def epipolar_angle_deg(t_veh: np.ndarray, R_cv: np.ndarray) -> float:
    """视差方向与图像横轴的夹角(0=可用 SGBM, 90=完全不可用)。

    相机系 x 右 / y 下 / z 前。**像移的竖直分量只来自 t_cam.y**:
    由定理  v1-v2 = f(y t_z - z t_y)/(z(z-t_z)),  t_y=0 且 t_z=0 时竖直视差恒零。
    而"是否可用"关心的是像移方向偏离横轴多少 —— 由 (t_z, t_y) 共同决定:
        t 沿 z(光轴) -> 视线与平移平行 -> 视差趋零 -> FoE 退化, 判"不可用"
        t 沿 x(横轴) -> 视差纯水平            -> 可用
        t 沿 y(纵轴) -> 视差纯竖直            -> 不可用
    因此角度定义为像移方向 (t_z, t_y) 与 x 轴的夹角:
        angle = atan2(sqrt(t_y^2 + t_z^2), |t_x|)
    """
    t_cam = R_cv @ np.asarray(t_veh, dtype=np.float64)
    perp = math.hypot(t_cam[1], t_cam[2])     # 垂直于图像横轴的分量
    return float(math.degrees(math.atan2(perp, abs(t_cam[0]))))


def epipolar_usable(t_veh: np.ndarray, R_cv: np.ndarray, tol_deg: float = 10.0) -> bool:
    return epipolar_angle_deg(t_veh, R_cv) <= tol_deg


# ------------------------------------------------------------ A. 相机重定向
def rectify_camera(R_cv: np.ndarray, t_veh: np.ndarray) -> tuple[np.ndarray, float]:
    """构造新 R', 使 t 在新机体系下 z 分量为 0(x' = t_c)。

    返回 (R_new, alpha_deg)。alpha 是新光轴与原光轴的夹角 = 原光轴与 t 的夹角,
    也就是"视野中心偏移了多少", 用于判断视场还能保留多少。

    **复合顺序(极易错, 已由 A2 抓住过)**: 先在**相机系**内构造旋转 R_cam,
    再叠加原外参:  R' = R_cam @ R_cv。
    若直接把 x_new(相机系向量)写进 R' 的行, 就等于把它当成**车辆系**向量, A2 必然失败。

    自检: A1 R' 正交; A2 (R' t).z == 0。
    """
    t_c = R_cv @ t_veh
    nt = np.linalg.norm(t_c)
    if nt < 1e-9:
        return R_cv.copy(), 0.0
    x_new = t_c / nt
    z_old = np.array([0.0, 0.0, 1.0])
    z_new = z_old - (z_old @ x_new) * x_new
    nz = np.linalg.norm(z_new)
    if nz < 1e-6:                # 平移沿光轴: FoE 退化, 重定向无意义
        # 返回原 R, 且 alpha 取 nan 而不是 0 —— 0 会让人误以为"无需重定向且已成功",
        # 实际是"这个场景根本不能用运动双目测深度"。
        return R_cv.copy(), float("nan")
    z_new /= nz
    y_new = np.cross(z_new, x_new)
    R_cam = np.stack([x_new, y_new, z_new], axis=0)   # 相机系内的旋转
    R_new = R_cam @ R_cv                              # 叠加原外参

    assert np.allclose(R_new @ R_new.T, np.eye(3), atol=1e-9), "A1 R' 非正交"
    assert abs((R_new @ t_veh)[2]) < 1e-9, f"A2 t 在新系下 z!=0 ({(R_new @ t_veh)[2]:.2e})"
    alpha = math.degrees(math.acos(np.clip(z_new @ z_old, -1.0, 1.0)))
    return R_new, alpha


# ------------------------------------------------------------ B. 视口重排
def reorient_view_R(R_cv: np.ndarray, t_veh: np.ndarray) -> np.ndarray:
    """视口重排: 返回 R_pc(视口系 -> 相机系)的新旋转, 使视差沿视口横轴。

    与 A 的区别: **不改相机光轴**, 只把视口的横轴 x_v 转到 t 在像平面的投影方向,
    纵轴取 z_cam × x_v (仍尽量贴近原纵轴)。视场由视口张角决定, 基本不丢。
    """
    t_c = R_cv @ t_veh
    nt = np.linalg.norm(t_c)
    if nt < 1e-9:
        return np.eye(3)
    x_v = t_c / nt
    z_cam = np.array([0.0, 0.0, 1.0])
    # t 若几乎沿光轴, 投影退化 -> 保持原视口
    if abs(x_v @ z_cam) > 0.999:
        return np.eye(3)
    y_v = np.cross(z_cam, x_v)
    y_v /= np.linalg.norm(y_v)
    z_v = np.cross(x_v, y_v)          # 仍尽量朝前, 保证 z_v.z > 0
    R_pc = np.stack([x_v, y_v, z_v], axis=0)   # 行 = 视口轴在相机系
    assert np.allclose(R_pc @ R_pc.T, np.eye(3), atol=1e-9), "R_pc 非正交"
    assert z_v[2] > 0, "视口光轴朝后了"
    return R_pc


def self_test(verbose: bool = True) -> dict:
    """跑一遍定理验证 + 两种方案的对比, 返回关键数字。"""
    f = 550.0
    out = {}
    rng = np.random.default_rng(0)
    P = np.stack([rng.uniform(-800, 800, 2000),
                  rng.uniform(-800, 800, 2000),
                  rng.uniform(500, 5000, 2000)], axis=1)

    # V-EPI-1: t 满足 ty=tz=0 -> 竖直视差恒 0
    dv = disparity_vertical_px(np.array([120.0, 0.0, 0.0]), P, f)
    out["v_epi1_max"] = float(np.nanmax(np.abs(dv)))
    assert out["v_epi1_max"] < 1e-9, f"V-EPI-1 失败: {out['v_epi1_max']}"

    # V-EPI-2: 竖直视差随 t_z 线性增长
    ratios = []
    for tz in (10.0, 50.0, 100.0):
        dv2 = disparity_vertical_px(np.array([120.0, 0.0, tz]), P, f)
        ratios.append(np.nanmedian(np.abs(dv2)) / tz)
    out["v_epi2_ratios"] = ratios
    spread = (max(ratios) - min(ratios)) / max(ratios)
    assert spread < 0.05, f"V-EPI-2 失败: 非线性, ratios={ratios}"

    # 4 路极线角度表
    table = {}
    for yaw in (0, 20, 45, 70, 90):
        t = rot_z(yaw) @ np.array([1000.0, 0.0, 0.0])
        row = {}
        for cid, cyaw in (("FV", 0.0), ("RV", 180.0), ("ML", 90.0), ("MR", -90.0)):
            R = vehicle_to_cam(cyaw, 0.0)
            row[cid] = epipolar_angle_deg(t, R)
        table[yaw] = row
    out["epipolar_table"] = table

    # 重定向代价: alpha 随夹角变化
    costs = {}
    for yaw in (0, 30, 60, 90):
        t = rot_z(yaw) @ np.array([1000.0, 0.0, 0.0])
        R = vehicle_to_cam(0.0, 0.0)      # FV
        _, alpha = rectify_camera(R, t)
        costs[yaw] = alpha
    out["rectify_alpha"] = costs

    if verbose:
        print(f"V-EPI-1 最大竖直视差(t_y=t_z=0): {out['v_epi1_max']:.2e} px  (要求 <1e-9)")
        print(f"V-EPI-2 |dv|/t_z 三档: {[f'{r:.4f}' for r in ratios]} 相对散布 {spread:.2%} (要求 <5%)")
        print("\n各相机极线与水平夹角(度), 平移方向相对车头:")
        print(f"{'yaw':>4} " + " ".join(f"{c:>7}" for c in ("FV", "RV", "ML", "MR")))
        for yaw, row in table.items():
            print(f"{yaw:>4} " + " ".join(f"{row[c]:>7.1f}" for c in ("FV", "RV", "ML", "MR")))
        print("\n相机重定向的代价(新光轴偏移 = 光轴与 t 的夹角):")
        for yaw, a in costs.items():
            print(f"  yaw={yaw:>3}: alpha={a:>5.1f}°  (视场中心偏 {a:.0f}°)")
    return out


if __name__ == "__main__":
    self_test()
