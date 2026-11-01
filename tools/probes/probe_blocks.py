"""O4 可行性: 把视口切块后, 块内极线方向离散能否降到 <5°?

若可行, SGBM 可以按块调用(每块先用该块的极线方向做旋转/剪切校正),
这是保留 SGBM 的最小改动路径。
"""
import numpy as np
from nfocc_ph0.epipolar import rot_z
from nfocc_ph0.view_lut import ViewParams
from nfocc_ph0.synth import Rig, SCENES


def measure(yaw_deg: float, block_w: int, block_h: int):
    rig = Rig()
    cam, R_cv, C0 = rig.cameras()["FV"]
    t_veh = rot_z(yaw_deg) @ np.array([120.0, 0.0, 0.0])
    C1 = C0 + t_veh
    sc = SCENES["S_BOX"]()
    vp = ViewParams(width=960, height=-1, hfov_deg=120, el_min_deg=10, el_max_deg=60)
    H, W = vp.height, vp.width
    uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)

    rayP = vp.pixel_to_ray(uu, vv)
    dirs = (rayP @ R_cv).reshape(-1, 3)
    th, wh = sc.raycast(C0, dirs)
    P = (C0[None, :] + dirs * np.where(np.isfinite(th), th, 0)[:, None]).reshape(H, W, 3)
    hit = (np.isfinite(th) & (wh >= 0)).reshape(H, W)

    r1 = (P - C1) @ R_cv.T
    r1 /= np.linalg.norm(r1, axis=-1, keepdims=True)
    p0 = np.stack([uu, vv], axis=-1)
    p1 = vp.ray_to_pixel(r1)
    ex = p1[..., 0] - p0[..., 0]
    ey = p1[..., 1] - p0[..., 1]
    mag = np.hypot(ex, ey)
    ang = np.degrees(np.arctan2(ey, ex))

    # 每块的圆周离散度
    devs = []
    for by in range(0, H, block_h):
        for bx in range(0, W, block_w):
            m = hit[by:by+block_h, bx:bx+block_w] & (mag[by:by+block_h, bx:bx+block_w] > 0.5)
            if m.sum() < 50:
                continue
            a = np.radians(ang[by:by+block_h, bx:bx+block_w][m])
            c = np.exp(1j * a)
            mean = np.angle(c.mean())
            dev = np.degrees(np.abs(np.angle(c / np.exp(1j * mean))))
            devs.append(np.percentile(dev, 95) * 2)
    if not devs:
        return None
    return {
        "n_blocks": len(devs),
        "p50": float(np.median(devs)),
        "p90": float(np.percentile(devs, 90)),
        "max": float(np.max(devs)),
    }


print("横移 90deg(最有利工况) 与 直行 0deg(最不利) 下的块内极线离散(度):")
print(f"{'yaw':>4} {'块大小':>10} {'块数':>5} {'p50':>7} {'p90':>7} {'max':>7}")
for yaw in (0, 90):
    for bw, bh in ((960, 258), (240, 64), (120, 32), (60, 16)):
        r = measure(yaw, bw, bh)
        if r:
            print(f"{yaw:>4} {bw:>4}x{bh:<5} {r['n_blocks']:>5} "
                  f"{r['p50']:>7.1f} {r['p90']:>7.1f} {r['max']:>7.1f}")
