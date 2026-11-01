"""NFOCC PH-0.2: 视口 LUT 构建 + 重采样(SPEC-LUT).

概念
----
视口(viewpoint)是一个**虚拟针孔相机**, 它覆盖相机视场中的"俯角环带"
(默认 10°-60°, 见 PARAM 的 V_RING_EL_MIN/MAX)。LUT 记录:
    视口像素 (u_v, v_v)  ->  鱼眼像素 (u_f, v_f) + 有效性

为什么用环带而不是全视场
------------------------
f=550、H=1536 时垂直半视场 80°。全视场会把地平线(视差=0)和车底(仰视过头)都收进来。
环带 [el_min, el_max] 保证每一行都对应真实的地面深度段。

约定(与 SPEC-COORD 一致)
-------------------------
    p_cam = R_cv^T (P - C)          R_cv: 相机->车辆
    视口系 P 的轴与相机系同向(x 右/y 下/z 前)
    不重排时: ray_C = R_pc @ ray_P, R_pc = I(视口就是相机内嵌的一个针孔)
    环带裁剪: 视口光轴相对相机光轴下俯 (el_min+el_max)/2, 垂张角 = el_max-el_min

四道有效性(顺序不可换, SPEC-LUT §4)
    D1 图像边界  D2 有效半径  D3 theta 上限  D4 车身掩码
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .camera_kb import KbCamera


@dataclass
class ViewParams:
    """视口参数。f_v 由宽度与水平张角决定; 垂直张角由环带决定。"""

    width: int = 960
    height: int = 384
    el_min_deg: float = 10.0      # 环带下界(相对水平俯角)
    el_max_deg: float = 60.0      # 环带上界
    hfov_deg: float = 120.0       # 视口水平张角
    cam_pitch_deg: float = 0.0    # 相机安装俯角(正=向下), 影响环带相对光轴的位置
    reorient: bool = False        # 是否按平移方向重排(本文件实现为可选的 R_pc)

    # 由上面推出的量(在 __post_init__ 里算)
    f_v: float = field(init=False)
    cx_v: float = field(init=False)
    cy_v: float = field(init=False)
    vfov_deg: float = field(init=False)

    def __post_init__(self) -> None:
        if not (0 < self.el_min_deg < self.el_max_deg < 90):
            raise ValueError(f"环带非法: {self.el_min_deg}..{self.el_max_deg}")
        self.f_v = (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        self.cx_v = self.width / 2.0
        self.vfov_deg = self.el_max_deg - self.el_min_deg
        # **关键**: height 必须由 vfov 反算, 不能独立指定。
        # 独立指定会让实际垂直张角 = 2*atan((height/2)/f_v) 与环带不符
        # (例如 height=384、f_v=277 时实际 69.6°, 而不是环带的 50°)。
        # height 传入值仅作"期望值"; 以 vfov 为准重算, 不一致就报错。
        need = 2.0 * self.f_v * math.tan(math.radians(self.vfov_deg) / 2.0)
        if self.height <= 0:
            self.height = int(round(need))
        elif abs(need - self.height) > 1.0:
            raise ValueError(
                f"height={self.height} 与环带不符: 环带 {self.vfov_deg:.1f}° 在 f_v={self.f_v:.2f} "
                f"下应为 {need:.1f} 行。请让 height=-1 自动计算, 或改 el/hfov。"
            )
        self.cy_v = self.height / 2.0

    def pixel_to_ray(self, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """视口像素 -> 视口系单位射线(轴同相机系: x 右/y 下/z 前)。

        视口光轴的绝对俯角 = (el_min+el_max)/2, 所以第 v 行的绝对俯角
            el(v) = el_mid + (v - cy)/f_v      [rad]
        (v 增大 = 图像下方 = 俯角更大)
        """
        el_mid = math.radians((self.el_min_deg + self.el_max_deg) / 2.0)
        az = np.arctan((u - self.cx_v) / self.f_v)          # 水平方位角
        el_off = np.arctan((v - self.cy_v) / self.f_v)      # 相对光轴的下俯角
        el = el_mid + el_off
        cos_el = np.cos(el)
        # 世界(相机)系: x 右, y 下, z 前
        return np.stack([np.sin(az) * cos_el, np.sin(el), np.cos(az) * cos_el], axis=-1)

    def ray_to_pixel(self, ray: np.ndarray) -> np.ndarray:
        """视口系射线 -> 视口像素。与 pixel_to_ray 严格互逆。

        必须做"仰角还原"再减 el_mid:
            el = atan2(y, hypot(x,z))       # 射线相对视口光轴的仰角
            v  = cy + f * tan(el - el_mid)  # ???
        **不对**: pixel_to_ray 里 el = el_mid + atan((v-cy)/f), 所以
            atan((v-cy)/f) = el_ray - el_mid
            v = cy + f * tan(atan2(y,hypot(x,z)) - el_mid)
        这才是严格互逆的形式。直接 v = cy + f*y/z 会少掉 el_mid 的补偿,
        实测导致往返误差 804px(竖直方向)。
        """
        ray = np.asarray(ray, dtype=np.float64)
        el_mid = math.radians((self.el_min_deg + self.el_max_deg) / 2.0)
        az = np.arctan2(ray[..., 0], ray[..., 2])
        el = np.arctan2(ray[..., 1], np.hypot(ray[..., 0], ray[..., 2]))
        u = self.cx_v + self.f_v * np.tan(az)
        v = self.cy_v + self.f_v * np.tan(el - el_mid)
        return np.stack([u, v], axis=-1)


def build_view_lut(cam: KbCamera, vp: ViewParams,
                   mask: np.ndarray | None = None,
                   R_pc: np.ndarray | None = None) -> dict:
    """构建视口 LUT。

    Args:
        cam : KB 相机
        vp  : 视口参数(含环带)
        mask: 车身掩码, (H,W) bool True=车身; None 表示不启用
        R_pc: 视口系->相机系的旋转(3x3)。None 表示不重排(视口即相机内嵌)

    Returns dict:
        uf, vf   (Hv,Wv) float64 鱼眼像素坐标(无效处为 nan)
        valid    (Hv,Wv) bool
        el_deg   (Hv,Wv) 每行的绝对俯角(便于调试)
        stats    dict 各道检查的淘汰计数
    """
    Hv, Wv = vp.height, vp.width
    uu, vv = np.meshgrid(np.arange(Wv, dtype=np.float64) + 0.5,
                         np.arange(Hv, dtype=np.float64) + 0.5)
    ray_P = vp.pixel_to_ray(uu, vv)                       # (Hv,Wv,3)

    if R_pc is None:
        ray_C = ray_P
    else:
        ray_C = ray_P @ np.asarray(R_pc, dtype=np.float64).T

    # 转成"相机光轴系": pixel_to_ray 已经给出相机系下的向量(视口与相机轴同向)
    uf, v_ok, theta = cam.project(ray_C.reshape(-1, 3))
    uf = uf.reshape(Hv, Wv, 2)
    theta = theta.reshape(Hv, Wv)

    r_img = cam.f * cam.theta_d(theta)

    # ---- 四道有效性 ----
    # 每道都独立记录"若没有它会被淘汰多少", 所以计数是在**假设前面各道都通过**的前提下
    # 统计"仅因这一道失败"的像素, 三者互斥且可加。
    finite = np.isfinite(uf[..., 0])
    d1 = (uf[..., 0] >= 0) & (uf[..., 0] < cam.width) & \
         (uf[..., 1] >= 0) & (uf[..., 1] < cam.height)
    d2 = r_img <= cam.r_valid + 1e-9
    d3 = theta <= cam.effective_max_theta() + 1e-12

    valid = finite & d1 & d2 & d3

    # 计数按"唯一淘汰原因"归类, 便于定位是环带宽了还是半径小
    only_d1 = finite & ~d1
    only_d2 = finite & d1 & ~d2
    only_d3 = finite & d1 & d2 & ~d3
    stats = {
        "total": int(Hv * Wv),
        "d1_out_of_image": int(only_d1.sum()),
        "d2_out_of_r_valid": int(only_d2.sum()),
        "d3_over_max_theta": int(only_d3.sum()),
    }

    if mask is not None:
        ui = np.clip(np.nan_to_num(uf[..., 0], nan=0.0).astype(np.int32), 0, cam.width - 1)
        vi = np.clip(np.nan_to_num(uf[..., 1], nan=0.0).astype(np.int32), 0, cam.height - 1)
        on_body = mask[vi, ui] & valid
        stats["d4_body_mask"] = int(on_body.sum())
        valid = valid & ~on_body
    else:
        stats["d4_body_mask"] = 0

    stats["valid"] = int(valid.sum())
    stats["valid_ratio"] = stats["valid"] / max(1, stats["total"])

    # 静默全空是最危险的失败模式: 视口配置错了却看起来"跑通了"
    if stats["valid"] == 0:
        raise ValueError(
            f"视口 LUT 全无效 (0/{stats['total']})。配置自相矛盾, 请检查: "
            f"hfov={vp.hfov_deg}° 环带={vp.el_min_deg}-{vp.el_max_deg}° r_valid={cam.r_valid} "
            f"max_theta={math.degrees(cam.effective_max_theta()):.1f}°。"
            f"淘汰统计: {stats}"
        )

    el_mid = math.radians((vp.el_min_deg + vp.el_max_deg) / 2.0)
    el_deg = np.degrees(
        el_mid + np.arctan((vv - vp.cy_v) / vp.f_v))

    uf_out = np.where(valid[..., None], uf, np.nan)
    return {"uf": uf_out[..., 0], "vf": uf_out[..., 1], "valid": valid,
            "el_deg": el_deg, "stats": stats}


def view_ground_range(vp: ViewParams, cam_z: float) -> tuple[float, float]:
    """环带对应的地面距离范围(mm)。相机俯角不影响结果(环带是绝对俯角)。"""
    near = cam_z / math.tan(math.radians(vp.el_max_deg))
    far = cam_z / math.tan(math.radians(vp.el_min_deg))
    return near, far


def warp(img: np.ndarray, lut: dict) -> tuple[np.ndarray, np.ndarray]:
    """按 LUT 重采样原图 -> 视口图。双线性, 无效点不参与权重(SPEC-LUT §5)。

    Returns (out (Hv,Wv) uint8/float, valid (Hv,Wv) bool)
    """
    uf, vf, valid = lut["uf"], lut["vf"], lut["valid"]
    H, W = img.shape[:2]
    u0 = np.floor(uf)
    v0 = np.floor(vf)
    du = uf - u0
    dv = vf - v0
    out = np.zeros_like(uf, dtype=np.float64)
    wsum = np.zeros_like(uf, dtype=np.float64)

    for dyi, dxi, wgt in ((0, 0, (1 - du) * (1 - dv)),
                          (0, 1, du * (1 - dv)),
                          (1, 0, (1 - du) * dv),
                          (1, 1, du * dv)):
        yy = np.clip(v0 + dyi, 0, H - 1).astype(np.int32)
        xx = np.clip(u0 + dxi, 0, W - 1).astype(np.int32)
        ok = valid & (v0 + dyi >= 0) & (v0 + dyi < H) & (u0 + dxi >= 0) & (u0 + dxi < W)
        w = np.where(ok, wgt, 0.0)
        out += w * img[yy, xx].astype(np.float64)
        wsum += w

    out = np.where(wsum > 1e-12, out / np.maximum(wsum, 1e-12), 0.0)
    out_valid = valid & (wsum > 1e-6)
    return out, out_valid
