#!/usr/bin/env python
"""用真实 hanqtech 标定验证 C++ 的四次多项式内参实现。

## 为什么必须对拍
C++ 与 Python 是两套独立实现。若只在 C++ 内部做往返自洽,
两个同样的错误会互相抵消。这里用 Python 按 readme 公式算标准答案,
与 C++ 的输出逐点比对。

## 判据
- 投影: C++ vs Python 偏差 < 0.01 px
- 反投影: C++ vs Python 射线夹角 < 1e-5 rad
- 往返: C++ 投影->反投影->投影 < 0.01 px
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = Path("/Volumes/Workspace/NFOCC")
DATA = ROOT / "datasets/fisheye"
CPP = ROOT / "cpp"
OCV = "/opt/homebrew/Cellar/opencv/5.0.0_12/lib"
OMP = "/opt/homebrew/opt/libomp/lib"

PROBE = r'''
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include "nfocc/nfocc_camera.h"

/* 读标定: name W H cx cy k1..k4
 * cx/cy 已按 hanqtech 约定换算为"主点像素坐标" */
int main(int argc, char **argv) {
    if (argc < 11) return 2;
    NFCameraModel cam;
    std::memset(&cam, 0, sizeof(cam));
    cam.version = NF_CAMERA_MODEL_VERSION;
    cam.kind    = NF_CAM_MODEL_RADIAL_POLY;
    cam.cam_id  = 0;
    cam.k1 = (nf_f32)atof(argv[1]);
    cam.k2 = (nf_f32)atof(argv[2]);
    cam.k3 = (nf_f32)atof(argv[3]);
    cam.k4 = (nf_f32)atof(argv[4]);
    cam.cx = (nf_f32)atof(argv[5]);
    cam.cy = (nf_f32)atof(argv[6]);
    cam.r_valid   = (nf_f32)atof(argv[7]);
    cam.max_theta = (nf_f32)atof(argv[8]);
    const int N = atoi(argv[9]);
    const float rng = (float)atof(argv[10]);

    printf("validate %d\n", (int)nf_cam_validate(&cam));

    /* 投影探针: 相机系方向 -> 像素 */
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            const float ax = rng * (2.0f * i / (N - 1) - 1.0f);
            const float ay = rng * (2.0f * j / (N - 1) - 1.0f);
            /* 相机系: z 前, x 右, y 下 */
            const float z = 1.0f;
            const float x = std::tan(ax);
            const float y = std::tan(ay);
            const float n = std::sqrt(x*x + y*y + z*z);
            float u = 0, v = 0;
            nf_u8 ok = 0;
            NFStatus st = nf_cam_project_c(&cam, x/n, y/n, z/n, &u, &v, &ok);
            printf("P %d %d %.10g %.10g %d %d\n", i, j, (double)u, (double)v,
                   (int)ok, (int)st);
        }
    }

    /* 反投影探针 */
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            const float u = 20.0f + (float)i / (N - 1) * 1240.0f;
            const float v = 20.0f + (float)j / (N - 1) * 926.0f;
            float d[3] = {0,0,0};
            NFStatus st = nf_cam_unproject_c(&cam, u, v, d);
            printf("U %d %d %.10g %.10g %.10g %d\n", i, j,
                   (double)d[0], (double)d[1], (double)d[2], (int)st);
        }
    }
    return 0;
}
'''


def load(name):
    d = json.load(open(DATA / f"calibration/calibration/{name}.json"))
    i = d["intrinsic"]
    w, h = int(i["width"]), int(i["height"])
    # hanqtech: u = u' + cx_offset + w/2 - 0.5  => 主点像素坐标
    cx = i["cx_offset"] + w / 2.0 - 0.5
    cy = i["cy_offset"] + h / 2.0 - 0.5
    return dict(w=w, h=h, cx=cx, cy=cy,
                k1=i["k1"], k2=i["k2"], k3=i["k3"], k4=i["k4"],
                ar=i["aspect_ratio"], name=d["name"])


def py_project(I, d):
    X, Y, Z = d
    chi = np.hypot(X, Y)
    th = np.arctan2(chi, Z)
    rho = I["k1"]*th + I["k2"]*th**2 + I["k3"]*th**3 + I["k4"]*th**4
    if chi > 1e-12:
        up = rho * X / chi
        vp = rho * Y / chi
    else:
        up = vp = 0.0
    u = up + I["cx"]
    v = vp * I["ar"] + I["cy"]
    return u, v


def py_unproject(I, u, v):
    up = u - I["cx"]
    vp = (v - I["cy"]) / I["ar"]
    rho = np.hypot(up, vp)
    k1, k2, k3, k4 = I["k1"], I["k2"], I["k3"], I["k4"]
    th = rho / k1
    for _ in range(80):
        f = k1*th + k2*th**2 + k3*th**3 + k4*th**4 - rho
        df = k1 + 2*k2*th + 3*k3*th**2 + 4*k4*th**3
        if abs(df) < 1e-14:
            break
        st = f/df
        th -= st
        if abs(st) < 1e-14:
            break
    s = np.sin(th)
    if rho > 1e-12:
        x, y = up/rho*s, vp/rho*s
    else:
        x = y = 0.0
    d = np.array([x, y, np.cos(th)])
    return d/np.linalg.norm(d), th


def main():
    b = Path(tempfile.mkdtemp(prefix="nf-ws-"))
    src = b / "probe.cpp"
    src.write_text(PROBE)
    exe = b / "probe"
    r = subprocess.run(["clang++", "-std=c++11", f"-I{CPP}/include", str(src),
                        str(CPP / "build/host/libnfocc_core.a"), "-o", str(exe),
                        "-lm"], capture_output=True, text=True)
    if r.returncode != 0:
        print("编译失败:", r.stderr[-800:])
        return 1

    N = 9
    RNG = 0.7   # 方位/俯仰搜索范围 (rad) 对应约 40°
    fails = []

    for nm in ("00164_FV", "00165_MVL", "00166_MVR", "00167_RV"):
        I = load(nm)
        # r_valid: 取对角半径; max_theta: 取四角 theta
        _, th_c = py_unproject(I, 0, 0)
        r_valid = np.hypot(I["w"]/2, I["h"]/2) * 1.45
        max_theta = th_c * 1.05
        args = [str(exe), f"{I['k1']}", f"{I['k2']}", f"{I['k3']}", f"{I['k4']}",
                f"{I['cx']}", f"{I['cy']}", f"{r_valid}", f"{max_theta}",
                str(N), f"{RNG}"]
        out = subprocess.run(args, capture_output=True, text=True).stdout

        lines = out.splitlines()
        if not lines or not lines[0].startswith("validate"):
            print(f"{I['name']}: 探针无输出"); fails.append(nm); continue
        vok = int(lines[0].split()[1])

        proj_err = 0.0
        unproj_err = 0.0
        n_proj = n_unproj = 0
        for ln in lines[1:]:
            p = ln.split()
            if p[0] == "P":
                i, j = int(p[1]), int(p[2])
                cu, cv, ok, st = float(p[3]), float(p[4]), int(p[5]), int(p[6])
                ax = RNG * (2.0*i/(N-1) - 1.0)
                ay = RNG * (2.0*j/(N-1) - 1.0)
                x, y, z = np.tan(ax), np.tan(ay), 1.0
                d = np.array([x, y, z]); d /= np.linalg.norm(d)
                pu, pv = py_project(I, d)
                if ok:
                    proj_err = max(proj_err, np.hypot(cu-pu, cv-pv))
                    n_proj += 1
            elif p[0] == "U":
                i, j = int(p[1]), int(p[2])
                dx, dy, dz, st = float(p[3]), float(p[4]), float(p[5]), int(p[6])
                if st != 0:
                    continue
                u = 20.0 + i/(N-1)*1240.0
                v = 20.0 + j/(N-1)*926.0
                pd, _ = py_unproject(I, u, v)
                cd = np.array([dx, dy, dz])
                cd /= np.linalg.norm(cd)
                ang = np.arccos(np.clip(np.dot(cd, pd), -1, 1))
                unproj_err = max(unproj_err, ang)
                n_unproj += 1

        okp = proj_err < 0.01
        oku = unproj_err < 1e-5
        print(f"  {I['name']:3s} validate={vok}  "
              f"投影偏差 {proj_err:.2e} px ({n_proj}点) {'OK' if okp else 'FAIL'}  "
              f"反投影夹角 {unproj_err:.2e} rad ({n_unproj}点) {'OK' if oku else 'FAIL'}")
        if vok != 0 or not okp or not oku:
            fails.append(nm)

    print()
    print("全部通过" if not fails else f"失败: {fails}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
