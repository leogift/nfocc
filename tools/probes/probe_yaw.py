"""关键验证: 车辆转弯时, 前视相机的极线是否变得近似平行?

这决定了"只处理侧向/转弯帧"这个方案是否可行。

背景: 直行时 t 平行于 FV 光轴 -> 极线从 FoE 扇形散开(实测角度跨 175°)。
若车辆有横摆(yaw), t 相对相机有横向分量, 极线应趋于平行。
本探针量化: 横摆多少度时, 同一行内极线方向变化降到可接受范围(比如 <5°)。
"""
import numpy as np
from nfocc_ph0.epipolar import rot_z
from nfocc_ph0.view_lut import ViewParams
from nfocc_ph0.synth import Rig, SCENES


def epi_angle_spread(yaw_deg: float, move_mm: float = 120.0) -> dict:
    rig = Rig()
    cam, R_cv, C0 = rig.cameras()["FV"]
    # 车辆沿 yaw 方向前移(转向时的瞬时运动)
    t_veh = rot_z(yaw_deg) @ np.array([move_mm, 0.0, 0.0])
    C1 = C0 + t_veh
    sc = SCENES["S_BOX"]()
    vp = ViewParams(width=480, height=-1, hfov_deg=120, el_min_deg=10, el_max_deg=60)
    H, W = vp.height, vp.width
    uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)

    rayP = vp.pixel_to_ray(uu, vv)
    dirs = rayP @ R_cv
    flat = dirs.reshape(-1, 3)
    t_hit, which = sc.raycast(C0, flat)
    P = (C0[None, :] + flat * np.where(np.isfinite(t_hit), t_hit, 0)[:, None]).reshape(H, W, 3)
    hit = (np.isfinite(t_hit) & (which >= 0)).reshape(H, W)

    r1 = ((P - C1) @ R_cv.T)
    r1 /= np.linalg.norm(r1, axis=-1, keepdims=True)
    p0 = np.stack([uu, vv], axis=-1)
    p1 = vp.ray_to_pixel(r1)
    ex = p1[..., 0] - p0[..., 0]
    ey = p1[..., 1] - p0[..., 1]
    ang = np.degrees(np.arctan2(ey, ex))
    mag = np.hypot(ex, ey)

    # 同一行内的极线角度离散度
    spreads = []
    for r in range(H):
        m = hit[r] & (mag[r] > 0.5)
        if m.sum() < 20:
            continue
        a = ang[r][m]
        # 处理 ±180 环绕: 用圆周统计
        c = np.exp(1j * np.radians(a))
        mean = np.angle(c.mean())
        dev = np.degrees(np.abs(np.angle(c / np.exp(1j * mean))))
        spreads.append(np.percentile(dev, 95) * 2)
    return {
        "yaw": yaw_deg,
        "ang_min": float(np.nanmin(ang[hit & (mag > 0.5)])),
        "ang_max": float(np.nanmax(ang[hit & (mag > 0.5)])),
        "row_spread_p90": float(np.percentile(spreads, 90)) if spreads else float("nan"),
        "disp_median": float(np.median(mag[hit & (mag > 0.5)])),
    }


print(f"{'yaw':>5} {'角度范围':>18} {'同⾏内离散p90':>14} {'|位移|中位':>10}")
for yaw in (0, 5, 10, 20, 30, 45, 60, 90):
    r = epi_angle_spread(yaw)
    print(f"{yaw:>5} [{r['ang_min']:>7.1f},{r['ang_max']:>7.1f}]"
          f"{r['row_spread_p90']:>14.1f}°{r['disp_median']:>10.1f}px")
