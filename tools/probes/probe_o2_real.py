"""O2 真实场景最终验证: 用方向向量 + 真实几何。

这一版用 pixel_to_ray 得到**方向**, 避开像素定义差异(视口 v 含俯角补偿)。
"""
import numpy as np
from nfocc_ph0.view_lut import ViewParams
from nfocc_ph0.synth import Rig, SCENES
from nfocc_ph1.pipeline import _project_scene_view
from nfocc_ph1.forward_range import forward_range_from_rays

rig = Rig()
cam, R_cv, C0 = rig.cameras()["FV"]
C1 = C0 + np.array([120.0, 0, 0])
t_view = R_cv @ (C1 - C0)

vp = ViewParams(width=960, height=-1, hfov_deg=120, el_min_deg=10, el_max_deg=60)
Hv, Wv = vp.height, vp.width
uu, vv = np.meshgrid(np.arange(Wv) + 0.5, np.arange(Hv) + 0.5)

for scene_id in ("S_FLAT", "S_MULTI"):
    sc = SCENES[scene_id]()
    P, hit, _ = _project_scene_view(cam, R_cv, C0, sc, vp, rig.cam_z)
    dist_gt = np.linalg.norm(P - C0, axis=-1)

    # 参考帧方向: 由视口像素直接给
    u_hat = vp.pixel_to_ray(uu, vv).reshape(-1, 3)
    # 当前帧方向: 把同一 3D 点投到视口像素, 再换成方向
    uv1 = vp.ray_to_pixel((P - C1) @ R_cv.T)
    v_hat = vp.pixel_to_ray(uv1[..., 0], uv1[..., 1]).reshape(-1, 3)

    r = forward_range_from_rays(u_hat, v_hat, t_view)
    ok = hit.reshape(-1) & r.valid & (dist_gt.reshape(-1) > 300) & (dist_gt.reshape(-1) < 4000)
    e = np.abs(r.dist_mm - dist_gt.reshape(-1))
    print(f"\n{scene_id}: n={ok.sum()} ({ok.sum()/max(1,hit.sum()):.1%})")
    print(f"  误差 中位 {np.median(e[ok]):.3f} mm  p90 {np.percentile(e[ok],90):.3f}  max {np.max(e[ok]):.3f}")
    dg = dist_gt.reshape(-1)
    for lo, hi in ((400, 800), (800, 1500), (1500, 2500), (2500, 4000)):
        m = ok & (dg >= lo) & (dg < hi)
        if m.sum() > 100:
            print(f"    D∈[{lo},{hi}): n={m.sum():>6} 中位 {np.median(e[m]):>7.3f} mm")

print("\n判定: 真值对应点下误差 ~0 表示方法无损;")
print("      实际精度 = 对应点精度 x 灵敏度(1px -> 110mm, 0.5px -> 55mm)")
