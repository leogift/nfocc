"""对比: 侧向平移下 整幅匹配 vs 分块方向匹配 的精度与产出。"""
import numpy as np
from nfocc_ph0.view_lut import ViewParams
from nfocc_ph0.synth import Rig, SCENES
from nfocc_ph1.product import _render_view
from nfocc_ph1.engine import EngineConfig, EngineKind, DisparityRange, make_engine
from nfocc_ph1.blockmatch import make_blocks, match_blocks
from nfocc_ph1.triangulate import triangulate_rays

rig = Rig()
cam, R_cv, C0 = rig.cameras()["FV"]
t = np.array([0.0, 120.0, 0.0])
C1 = C0 + t
t_cam = R_cv @ t
vp = ViewParams(width=960, height=-1, hfov_deg=120, el_min_deg=10, el_max_deg=60)
sc = SCENES["S_MULTI"]()
img0, P0, hit0, _ = _render_view(sc, cam, R_cv, C0, vp)
img1, _, _, _ = _render_view(sc, cam, R_cv, C1, vp)

# 真值视差
uv0 = vp.ray_to_pixel((P0 - C0) @ R_cv.T)
uv1 = vp.ray_to_pixel((P0 - C1) @ R_cv.T)
du_true = np.abs(uv0[..., 0] - uv1[..., 0])
dist_true = np.linalg.norm(P0 - C0, axis=-1)

_, eng = make_engine(EngineConfig(kind=EngineKind.SGBM, max_disp_px=256))
rng = DisparityRange(0, 256)
L = np.ascontiguousarray(img0)
R = np.ascontiguousarray(img1)

results = {}
st, dm_full = eng.run(L, R, rng)
results["整幅"] = dm_full

# 块宽必须 > numDisp(256), 否则 OpenCV 分配负尺寸内存
for bw, bh in ((480, 128), (320, 64)):
    blocks = make_blocks(vp, R_cv, C0, C1, bw=bw, bh=bh)
    st, dm = match_blocks(eng, L, R, blocks, rng)
    results[f"分块 {bw}x{bh} ({len(blocks)}块)"] = dm

print(f"真值: |du| 中位 {np.median(du_true[hit0]):.1f}px, 距离中位 {np.median(dist_true[hit0]):.0f}mm")
print()
print(f"{'方法':>24} {'有效比例':>8} {'视差误差中位':>10} {'深度误差中位':>12}")
for name, dm in results.items():
    if dm is None:
        print(f"{name:>24} {'失败':>8}")
        continue
    v = dm.valid & hit0
    if v.sum() < 100:
        print(f"{name:>24} {v.mean():>7.1%} {'点数不足':>10}")
        continue
    derr = np.abs(dm.disp[v] - du_true[v])
    # 三角化
    ys, xs = np.nonzero(v)
    u = xs + 0.5
    vv = ys + 0.5
    d = dm.disp[ys, xs].astype(np.float64)
    sign = 1.0 if float(t_cam[0]) < 0.0 else -1.0
    r0 = vp.pixel_to_ray(u, vv) @ R_cv
    r1 = vp.pixel_to_ray(u + sign * d, vv) @ R_cv
    s0, _ = triangulate_rays(C0, r0, C1, r1)
    ok = np.isfinite(s0) & (s0 > 0)
    terr = np.abs(s0[ok] - dist_true[ys[ok], xs[ok]])
    print(f"{name:>24} {v.mean():>7.1%} {np.median(derr):>10.1f} {np.median(terr):>11.0f}mm")

print()
print("注: 视差误差用 |du| 比较, 深度误差用射线三角化与真值距离比较。")
