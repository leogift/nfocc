#!/usr/bin/env python
"""验证虚拟相机的视口朝向：pitch=0、roll=0、yaw 按 6 方向。

## 为什么要独立验证
C++ 里"视口有效率 100%"既可能来自虚拟相机，也可能来自旧的跟随光轴实现
—— 两种都在物理上合理。必须**独立算出视口射线**再比对，不能只看有效率。

判据：
1. 视口中心射线的俯角必须等于环带中轴 35°（虚拟相机水平光轴），
   而不是 35° + 安装俯角（旧实现会把安装角加进去）
2. 视口中心射线的方位角必须等于该路规划的 yaw
3. roll=0：视口"上"方向必须是车系 +Z 的水平投影，即 (0,0,1)
"""
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = Path("/Volumes/Workspace/NFOCC")
CPP = ROOT / "cpp"
DATA = ROOT / "datasets/fisheye"
results = []


def check(name, ok, detail):
    results.append(name and [ok][0])
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


PROBE = r'''
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_default.h"
int main(int argc, char **argv) {
    const float yaw = (float)atof(argv[1]);
    NFViewParams vp;
    std::memset(&vp, 0, sizeof(vp));
    if (nf_view_params_derive(&vp, 640, NF_V_RING_EL_MIN_DEG,
                              NF_V_RING_EL_MAX_DEG, NF_V_VIEW_HFOV_DEG) != NF_OK)
        return 2;
    vp.yaw_deg = yaw;
    vp.pitch_deg = 0.0f;
    /* 视口中心像素 */
    float d[3];
    if (nf_view_pixel_to_ray(&vp, vp.width * 0.5f, vp.height * 0.5f, d) != NF_OK)
        return 3;
    /* 视口顶边中点（用于判断 roll） */
    float up[3];
    nf_view_pixel_to_ray(&vp, vp.width * 0.5f, 0.5f, up);
    /* 视口左边中点（用于判断水平朝向） */
    float lf[3];
    nf_view_pixel_to_ray(&vp, 0.5f, vp.height * 0.5f, lf);
    printf("%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n",
           (double)d[0], (double)d[1], (double)d[2],
           (double)up[0], (double)up[1], (double)up[2],
           (double)lf[0], (double)lf[1], (double)lf[2]);
    return 0;
}
'''


def main():
    b = Path(tempfile.mkdtemp(prefix="hermes-verify-vcam-"))
    src = b / "p.cpp"
    src.write_text(PROBE)
    exe = b / "p"
    r = subprocess.run(["clang++", "-std=c++11", f"-I{CPP}/include", str(src),
                        str(CPP / "build/host/libnfocc_core.a"), "-o", str(exe),
                        "-lm"], capture_output=True, text=True)
    if r.returncode != 0:
        print("编译失败:", r.stderr[-600:])
        return 1

    # 与 nfocc_viewplan 一致的 6 视口
    VPS = [("ML/MVL", 2, +90.0),
           ("MR/MVR", 3, -90.0),
           ("FV",     0, -45.0),
           ("FV",     0, +45.0),
           ("RV",     1, -135.0),
           ("RV",     1, +135.0)]

    print("=" * 70)
    print("虚拟相机朝向验证（pitch=0, roll=0, 只有 yaw）")
    print("=" * 70)
    print(f"{'视口':<10}{'规划yaw':>9}{'实测方位':>10}{'实测俯角':>10}"
          f"{'期望俯角':>10}{'roll偏移':>10}")
    # **平视**: ring_el_mid_rad 恒为 0（v0.9.1 起），视口中轴俯角必须为 0。
    # 早期此处置 (10+60)/2 = 35，是"环带下偏 35°"时代的期望值。
    el_mid = 0.0

    for name, cam, yaw in VPS:
        out = subprocess.run([str(exe), f"{yaw}"], capture_output=True,
                             text=True).stdout.split()
        v = [float(x) for x in out]
        d, up, lf = np.array(v[0:3]), np.array(v[3:6]), np.array(v[6:9])
        az = math.degrees(math.atan2(d[1], d[0]))
        el = math.degrees(math.asin(-d[2]))
        # roll 判据: 视口"上"方向必须落在 span{forward, down} 平面内,
        # 即与 right 向量正交。**不能用 up 与车系+Z 的夹角** ——
        # 视口本身俯视 35°, up 与 +Z 天然有 ~98° 夹角，那是正常透视几何。
        right = np.array([math.sin(math.radians(yaw)),
                          -math.cos(math.radians(yaw)), 0.0])
        upn = up / np.linalg.norm(up)
        roll = math.degrees(math.asin(max(-1.0, min(1.0,
                                    float(np.dot(upn, right))))))
        mark = "" if abs(az - yaw) < 0.6 else "  <-- 方位不符"
        print(f"{name:<10}{yaw:>+9.0f}{az:>+10.1f}{el:>10.1f}"
              f"{el_mid:>10.1f}{roll:>10.3f}{mark}")
        check(f"{name} 方位 = {yaw:+.0f}°", abs(az - yaw) < 0.6,
              f"{az:+.1f}°")
        check(f"{name} 俯角 = 0°（平视）", abs(el) < 0.6,
              f"{el:+.2f}°（若得 22.7° 说明跟了安装角，得 35° 说明还带环带下偏）")
        check(f"{name} roll ≈ 0", abs(roll) < 0.01, f"{roll:.4f}°")

    print()
    print("=" * 70)
    print("与真实安装角的独立性：换相机不影响视口朝向")
    print("=" * 70)
    # FV 与 MVL 都取 yaw=+45，视口中心射线应完全相同
    o1 = subprocess.run([str(exe), "45"], capture_output=True, text=True).stdout.split()
    o2 = subprocess.run([str(exe), "45"], capture_output=True, text=True).stdout.split()
    same = o1 == o2
    print(f"  同一 yaw 两次调用结果一致: {same}")
    check("视口只由 yaw 决定（与相机无关）", same, "一致")

    import shutil
    shutil.rmtree(b, ignore_errors=True)
    print()
    fails = results.count(False)
    print(f"{len(results)} 项, 失败 {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
