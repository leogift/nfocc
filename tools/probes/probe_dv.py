"""诊断: 前向平移时视口内的竖直位移有多大。

若 |dv| 与 |du| 同量级, 则"只做水平搜索"的 SGBM 结构上就不可能准 ——
这就解释了 16px 的中位误差, 也说明视口必须按极线方向重排(或做 2D 搜索)。
"""
import numpy as np
from nfocc_ph0.view_lut import ViewParams
from nfocc_ph0.synth import Rig, SCENES
from nfocc_ph1.pipeline import _project_scene_view

rig = Rig()
cam, R_cv, C0 = rig.cameras()["FV"]
C1 = C0 + np.array([120.0, 0, 0])
sc = SCENES["S_BOX"]()

for w in (960, 1920):
    vp = ViewParams(width=w, height=-1, hfov_deg=120, el_min_deg=10, el_max_deg=60)
    P, hit, t = _project_scene_view(cam, R_cv, C0, sc, vp, rig.cam_z)
    uv0 = vp.ray_to_pixel((P - C0) @ R_cv.T)
    uv1 = vp.ray_to_pixel((P - C1) @ R_cv.T)
    du = uv0[..., 0] - uv1[..., 0]
    dv = uv0[..., 1] - uv1[..., 1]
    s = hit
    print(f"视口 {w}x{vp.height}:")
    print(f"  |du| 中位 {np.median(np.abs(du[s])):6.2f} px   p90 {np.percentile(np.abs(du[s]),90):6.2f}")
    print(f"  |dv| 中位 {np.median(np.abs(dv[s])):6.2f} px   p90 {np.percentile(np.abs(dv[s]),90):6.2f}")
    print(f"  比值 |dv|/|du| 中位 {np.median(np.abs(dv[s])/np.maximum(np.abs(du[s]),1e-6)):6.2f}")
    # 每行的 du/dv 稳定度: SGBM 按行匹配, 若同一行内 du 变化大也不行
    row_spread = []
    for r in range(0, vp.height, max(1, vp.height // 8)):
        m = s[r]
        if m.sum() < 20:
            continue
        row_spread.append(np.nanmax(du[r][m]) - np.nanmin(du[r][m]))
    print(f"  同一行内 du 变化范围 (抽样): {[f'{x:.1f}' for x in row_spread[:6]]}")
    print()
