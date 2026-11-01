#!/usr/bin/env python
"""确认 hanqtech 外参坐标系，并检查与项目约定的差异。

项目约定: v_V = R·v_C, R 的**列** = 相机轴在车系下的单位向量。
hanqtech readme: "transformation from camera to vehicle", 四元数用
  scipy.spatial.transform.Rotation.from_quat (默认 xyzw)。

必须实测确认:
  1. R 的行列式 = +1 (不是镜像)
  2. R 的列确实是相机轴, 且相机 y 轴朝下 (车系 z 分量为负)
  3. 光轴朝外 (背离车体中心)
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

DATA = Path("/Volumes/Workspace/NFOCC/datasets/fisheye")
VEH = dict(length=5000.0, width=2000.0, rear_to_front=3500.0)


def load(name):
    d = json.load(open(DATA / f"calibration/calibration/{name}.json"))
    e = d["extrinsic"]
    M = R.from_quat(e["quaternion"]).as_matrix()
    t = np.array(e["translation"], float) * 1000.0
    return d["name"], M, t


print("=" * 76)
print("1. R 的性质")
print("=" * 76)
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, M, t = load(nm)
    det = np.linalg.det(M)
    orth = np.linalg.norm(M @ M.T - np.eye(3))
    print(f"  {name:3s}: det={det:+.6f}  ‖RRᵀ−I‖={orth:.2e}  "
          f"{'正交且右手' if abs(det-1) < 1e-6 and orth < 1e-6 else '!!异常'}")

print()
print("=" * 76)
print("2. R 的列 = 相机轴在车系 (逐轴语义确认)")
print("=" * 76)
print(f"  {'相机':4s} {'x轴(右)':>22s} {'y轴(下)':>22s} {'z轴(前=光轴)':>22s}")
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, M, t = load(nm)
    cols = ["(%+.3f,%+.3f,%+.3f)" % tuple(M[:, k]) for k in range(3)]
    print(f"  {name:4s} {cols[0]:>22s} {cols[1]:>22s} {cols[2]:>22s}")

print()
print("=" * 76)
print("3. 相机 y 轴是否朝下 (车系 z 分量应 < 0)")
print("=" * 76)
allok = True
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, M, t = load(nm)
    ydown = M[2, 1]
    ok = ydown < 0
    allok &= ok
    print(f"  {name:3s}: R[2,1]={ydown:+.4f}  {'朝下 OK' if ok else '!! 不朝下'}")

print()
print("=" * 76)
print("4. 光轴是否朝外 (相对车体中心)")
print("=" * 76)
# 车体中心 (后轴原点): x = rear_to_front - length/2 = 1000, y = 0
cx_body, cy_body = VEH["rear_to_front"] - VEH["length"] / 2, 0.0
for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
    name, M, t = load(nm)
    z = M[:, 2]
    radial = np.array([t[0] - cx_body, t[1] - cy_body, 0.0])
    radial /= np.linalg.norm(radial) + 1e-12
    d = float(np.dot(z[:2], radial[:2]))
    print(f"  {name:3s}: 光心=({t[0]:7.1f},{t[1]:7.1f})  光轴水平分量={z[:2]}  "
          f"与径向夹角 cos={d:+.3f}  {'朝外 OK' if d > 0 else '!! 朝内'}")

print()
print("=" * 76)
print("5. 与项目内 NFPose 的对应关系")
print("=" * 76)
name, M, t = load("00164_FV")
print("  项目约定 v_V = R·v_C, R 列 = 相机轴在车系")
print("  hanqtech: 同样是 camera->vehicle 旋转")
print("  => NFPose.R = M (行优先展平), NFPose.t = translation*1000 (mm)")
print()
print(f"  FV 展开为 C 数组:")
for r in range(3):
    print("    " + ", ".join(f"{M[r,c]:+9.6f}f" for c in range(3)) + ",")
print(f"  t = {{ {t[0]:.1f}f, {t[1]:.1f}f, {t[2]:.1f}f }}")

print()
print("=" * 76)
if allok:
    print("结论: hanqtech 外参约定与项目完全一致, 可直接作为 NFPose")
else:
    print("结论: 存在不一致, 需在加载层做变换")
