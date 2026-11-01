"""诊断: 视口图的左右顺序与真值方向是否一致。

SGBM 假设 left 在左、right 在右。前向平移时, 物点在新帧里:
  左半图(u<cx) 向 u 减小方向移动, 右半图(u>cx) 向 u 增大方向移动 (或相反, 取决于 t 符号)。
若把 new 当 right 传给 SGBM, 则只有"一个方向"能被匹配到, 另一半必然大误差。
"""
import numpy as np
from nfocc_ph0.view_lut import ViewParams
from nfocc_ph0.synth import Rig, SCENES
from nfocc_ph1.pipeline import _project_scene_view

rig = Rig()
cam, R_cv, C0 = rig.cameras()["FV"]
C1 = C0 + np.array([120.0, 0, 0])
sc = SCENES["S_BOX"]()
vp = ViewParams(width=960, height=-1, hfov_deg=120, el_min_deg=10, el_max_deg=60)

P, hit, t = _project_scene_view(cam, R_cv, C0, sc, vp, rig.cam_z)
uv0 = vp.ray_to_pixel((P - C0) @ R_cv.T)
uv1 = vp.ray_to_pixel((P - C1) @ R_cv.T)
du = uv0[..., 0] - uv1[..., 0]
cx = vp.width / 2.0
uu, _ = np.meshgrid(np.arange(vp.width) + 0.5, np.arange(vp.height) + 0.5)

for name, m in (("左半 u<cx", uu < cx), ("右半 u>cx", uu >= cx)):
    s = du[m & hit]
    print(f"{name}: du 中位 {np.median(s):+.2f} px  "
          f"min {s.min():+.1f} max {s.max():+.1f}")

print()
print("SGBM 需要: right(x,y) = left(x - d, y), 即 left 像素在 x, 对应点在 x-d。")
print("若 du = u0-u1 > 0, 说明对应点在左边, 可被 SGBM 找到(当 right=new 时)。")
print("若 du < 0, 对应点在右边, SGBM 搜不到 -> 必须反转或分半图。")
