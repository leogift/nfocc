"""NFOCC PH-0.1: KB 鱼眼相机模型(正投影/反投影) + 自检.

严格实现 SPEC-COORD.md §3, 约定:

    p_cam = R @ (P - C)          # R 是 车辆->相机 的旋转(3x3, 正交)
    u = f * p_cam.x / p_cam.z + cx   # 仅针孔; 鱼眼走 KB

KB (Kannala-Brandt):
    theta  = atan2(sqrt(x^2+y^2), z)                    # 入射角
    theta_d = theta*(1 + k1 t^2 + k2 t^4 + k3 t^6 + k4 t^8)
    r      = f * theta_d
    u      = cx + r * x/chi ,  v = cy + r * y/chi        # chi = sqrt(x^2+y^2)

反投影用 Newton 迭代解 theta_d(theta) = rho, 导数系数是 1,3,5,7,9
(写成 1,2,3,4 是最常见的错误, 由 V-CO-2 的往返误差抓)。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ----------------------------------------------------------------------------
# 常量
# ----------------------------------------------------------------------------
EPS_CHI = 1e-9      # chi -> 0 的极限阈值
EPS_RHO = 1e-12
NEWTON_MAX_ITER = 20
NEWTON_TOL = 1e-12


class KbError(ValueError):
    """KB 模型非法(非单调/参数越界)。对应 NF_E_CAM_DISTORT_INV。"""


@dataclass
class KbCamera:
    """KB 鱼眼内参。字段名与 SPEC-ICAL 的 NFCameraModel 对应。"""

    f: float
    cx: float
    cy: float
    k: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    r_valid: float = 960.0
    width: int = 1920
    height: int = 1536
    cam_id: str = "?"
    max_theta: float | None = None   # None -> 由四角自动推(见 theta_max_auto)

    # ------------------------------------------------------------------ 基础
    def __post_init__(self) -> None:
        if self.f <= 0:
            raise KbError(f"f 必须 > 0, 实际 {self.f}")
        if len(self.k) != 4:
            raise KbError("k 必须是 4 个系数")
        self._check_monotonic()

    @property
    def r_corner(self) -> float:
        return math.hypot(self.width / 2.0, self.height / 2.0)

    def theta_max_auto(self) -> float:
        """四角对应的 theta(等距近似下 = r_corner/f; k!=0 时用反解)。

        与 tools/calib_gen.py 的 max_theta 保持一致, 避免"手填 vs 自动"漂移。
        """
        rho = self.r_corner / self.f
        return float(self.rho_to_theta(np.array([rho]))[0])

    def effective_max_theta(self) -> float:
        return self.theta_max_auto() if self.max_theta is None else float(self.max_theta)

    # ------------------------------------------------------------ theta_d / 逆
    def theta_d(self, theta: np.ndarray) -> np.ndarray:
        k1, k2, k3, k4 = self.k
        t2 = theta * theta
        # errstate: 负系数/大 theta 时高次项会溢出, 这是**预期内**的
        # (调用方随后会用 isfinite 判定无效), 不必刷屏警告
        with np.errstate(over="ignore", invalid="ignore"):
            return theta * (1.0 + k1 * t2 + k2 * t2**2 + k3 * t2**3 + k4 * t2**4)

    def theta_d_prime(self, theta: np.ndarray) -> np.ndarray:
        k1, k2, k3, k4 = self.k
        t2 = theta * theta
        # 系数 1,3,5,7,9 —— 写成 1,2,3,4 会让 Newton 收敛到错解
        with np.errstate(over="ignore", invalid="ignore"):
            return (1.0 + 3.0 * k1 * t2 + 5.0 * k2 * t2**2
                    + 7.0 * k3 * t2**3 + 9.0 * k4 * t2**4)

    def _check_monotonic(self) -> None:
        """theta_d 在 [0, theta_max] 上必须单调增, 否则反解不唯一。"""
        tmax = self.theta_max_auto() * 1.05
        th = np.linspace(0.0, tmax, 2000)
        with np.errstate(over="ignore", invalid="ignore"):
            dp = self.theta_d_prime(th)
        if not np.all(np.isfinite(dp)) or np.any(dp <= 0):
            bad = ~np.isfinite(dp) | (dp <= 0)
            i = int(np.argmax(bad))
            raise KbError(
                f"theta_d 在 theta={th[i]:.4f} 处非单调或发散(导数 {dp[i]:.3e})。"
                f"k={self.k} 不合法, 反解无唯一解。"
            )

    def rho_to_theta(self, rho: np.ndarray) -> np.ndarray:
        """解 theta_d(theta) = rho。Newton, 初值取 rho。"""
        rho = np.asarray(rho, dtype=np.float64)
        theta = rho.copy()
        for _ in range(NEWTON_MAX_ITER):
            fval = self.theta_d(theta) - rho
            fprime = self.theta_d_prime(theta)
            fprime = np.where(fprime <= 1e-12, 1e-12, fprime)
            step = fval / fprime
            theta = theta - step
            if np.all(np.abs(step) < NEWTON_TOL):
                break
        else:
            # 未在 max_iter 内收敛
            resid = np.max(np.abs(self.theta_d(theta) - rho))
            if resid > 1e-9:
                raise KbError(f"rho_to_theta 未收敛, 残差 {resid:.3e}")
        return theta

    # ---------------------------------------------------------------- 正投影
    def project(self, p_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """3D 相机坐标 (N,3) -> (uv (N,2), valid (N,), theta (N,)).

        **职责边界(重要)**: 本函数只做**几何投影**, 只判"是否有限"。
        半径/角度上限等有效性判定交给调用方(SPEC-LUT 的 D2/D3),
        否则调用方永远统计不到 d2 淘汰了多少 —— 这是实测踩到的 bug。
        valid 在这里 == isfinite。
        """
        p = np.asarray(p_cam, dtype=np.float64).reshape(-1, 3)
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        chi = np.hypot(x, y)
        theta = np.arctan2(chi, z)

        rho = self.theta_d(theta)
        safe = np.where(chi < EPS_CHI, 1.0, chi)
        r = self.f * rho
        uv = np.stack([self.cx + r * x / safe, self.cy + r * y / safe], axis=1)

        valid = np.isfinite(uv).all(axis=1)
        return uv, valid, theta

    def project_checked(self, p_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """带有效性判定的投影(半径 + 角度上限)。需要独立使用相机时用这个。"""
        uv, finite, theta = self.project(p_cam)
        r = self.f * self.theta_d(theta)
        ok = (
            finite
            & (theta <= self.effective_max_theta() + 1e-12)
            & (r <= self.r_valid + 1e-9)
        )
        return np.where(ok[:, None], uv, np.nan), ok, theta

    # --------------------------------------------------------------- 反投影
    def unproject(self, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """像素 (N,2) -> 相机系单位射线 (N,3), valid (N,).

        实现严格按 SPEC-COORD 3.6-3.9:
            lens = (uv - c)/f ; rho = |lens| ; theta = rho_to_theta(rho)
            ray = (sin(theta) * lens.x/rho, sin(theta) * lens.y/rho, cos(theta))
        """
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        lx = (uv[:, 0] - self.cx) / self.f
        ly = (uv[:, 1] - self.cy) / self.f
        rho = np.hypot(lx, ly)

        theta = self.rho_to_theta(rho)
        sin_t = np.sin(theta)
        safe = np.where(rho < EPS_RHO, 1.0, rho)
        scale = np.where(rho < EPS_RHO, 0.0, sin_t / safe)  # rho->0 时 x,y 分量->0

        ray = np.stack([scale * lx, scale * ly, np.cos(theta)], axis=1)
        n = np.linalg.norm(ray, axis=1)
        ray = ray / np.where(n < 1e-15, 1.0, n)[:, None]

        inside_image = (
            (uv[:, 0] >= 0) & (uv[:, 0] < self.width)
            & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)
        )
        r_img = self.f * self.theta_d(theta)
        valid = inside_image & (r_img <= self.r_valid + 1e-9)
        return ray, valid

    def project_ray(self, ray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """单位射线 (N,3) -> 像素。直接复用 project(单位向量即可)。"""
        return self.project(ray)[0:2]


def make_default_camera(**kw) -> KbCamera:
    """本项目定案参数 (SPEC-CALIB-CFG): f=550, 1920x1536, k=0, r_valid=960。"""
    base = dict(f=550.0, cx=960.0, cy=768.0, k=(0.0, 0.0, 0.0, 0.0),
                r_valid=960.0, width=1920, height=1536, cam_id="FV")
    base.update(kw)
    return KbCamera(**base)
