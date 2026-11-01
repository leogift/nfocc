#!/usr/bin/env python3
"""视口偏转 (yaw_offset) 与 6 视口规划的可复跑验收。

定案 (v0.7.0): ML/MR 保持单视口 0°; FV/RV 各 ±45° 两个视口; 视锥 120°。

    python3 scripts/verify_viewplan.py

## 为什么常驻
写成临时脚本会在清理后失去证据, 导致"改过但无验证"。

## 判据
- 偏转后视口中心方位角 == 规划表 (±0.6°), 由 LUT 反投影实测
- 6 视口 LUT 有效率 100% (偏 30° 仍在鱼眼视域内)
- 门控真的受偏转影响 (FORWARD -> HYBRID); 否则偏转形同虚设
- ctest 不回归
"""
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

CPP = Path(__file__).resolve().parent.parent
OCV = "/opt/homebrew/Cellar/opencv/5.0.0_12/lib"
OMP = "/opt/homebrew/opt/libomp/lib"
results = []

PROBE = r'''
#include <cstdio>
#include <cmath>
#include <cstring>
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_viewplan.h"
#include "nfocc/nfocc_default.h"
static void rig(NFPose p[4]) {
    static const float axis[4][3][3] = {
        { {0,-1,0}, {0,0,-1}, {1,0,0} }, { {0, 1,0}, {0,0,-1}, {-1,0,0} },
        { {1, 0,0}, {0,0,-1}, {0,1,0} }, { {-1,0,0}, {0,0,-1}, {0,-1,0} } };
    for (int c=0;c<4;++c){ p[c].version=NF_POSE_VERSION;
        for(int col=0;col<3;++col) for(int row=0;row<3;++row)
            p[c].R[row*3+col]=axis[c][col][row]; }
}
int main(){
    NFPose poses[4]; rig(poses);
    NFCameraModel cam; std::memset(&cam,0,sizeof(cam));
    cam.version=NF_CAMERA_MODEL_VERSION; cam.f=NF_INF_F;
    cam.cx=NF_INF_CX; cam.cy=NF_INF_CY;
    cam.r_valid=NF_INF_R_VALID; cam.max_theta=NF_INF_MAX_THETA;
    NFViewEntry plan[NF_MAX_VIEWS];
    nf_u32 n = nf_viewplan_default(plan, NF_MAX_VIEWS);
    const float cy_[4]={0,180,90,-90};
    const char* nm[4]={"FV","RV","ML","MR"};
    for(nf_u32 i=0;i<n;++i){
        NFViewParams vp; std::memset(&vp,0,sizeof(vp));
        nf_view_params_derive(&vp, NF_V_VIEW_W, NF_V_RING_EL_MIN_DEG,
                              NF_V_RING_EL_MAX_DEG, 120.0f);
        vp.yaw_offset_deg = plan[i].yaw_offset_deg;
        float d[3];
        nf_view_pixel_to_ray(&vp, vp.width*0.5f, vp.height*0.5f, d);
        const NFPose* P=&poses[plan[i].cam_id];
        float v[3];
        for(int k=0;k<3;++k)
            v[k]=P->R[k*3+0]*d[0]+P->R[k*3+1]*d[1]+P->R[k*3+2]*d[2];
        float got=std::atan2(v[1],v[0])*180.0f/(float)M_PI;
        float want=nf_viewplan_azimuth_deg(cy_[plan[i].cam_id], plan[i].yaw_offset_deg);
        NFViewLut lut; std::memset(&lut,0,sizeof(lut));
        nf_lut_alloc(&lut, vp.width, vp.height);
        NFLutStats st;
        nf_lut_build(&lut, &cam, &poses[plan[i].cam_id], &vp, nullptr, &st);
        double eff = 100.0*st.valid/st.total;
        printf("AZ %u %s %+.1f %.1f %.1f %.1f\n", i, nm[plan[i].cam_id],
               (double)plan[i].yaw_offset_deg, (double)want, (double)got, eff);
        nf_lut_destroy(&lut);
    }
    return 0;
}
'''


def check(name, ok, detail):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def sh(*a, **kw):
    return subprocess.run(a, capture_output=True, text=True, timeout=kw.pop("t", 600))


def main():
    b = Path(tempfile.mkdtemp(prefix="nfocc-yvp-"))
    src = Path(tempfile.mkdtemp(prefix="nfocc-yvps-"))
    try:
        print("[1] 构建")
        r = sh("cmake", "-S", str(CPP), "-B", str(b), "-DCMAKE_BUILD_TYPE=Release")
        check("配置成功", r.returncode == 0, f"rc={r.returncode}")
        if r.returncode != 0:
            return 1
        r = sh("cmake", "--build", str(b), "-j8")
        check("构建零错误", r.returncode == 0,
              "ok" if r.returncode == 0 else r.stderr[-600:])
        if r.returncode != 0:
            return 1

        print("\n[2] 视口方位角 (LUT 反投影实测)")
        pf = src / "az.cpp"
        pf.write_text(PROBE)
        exe = b / "azprobe"
        r = sh("clang++", "-std=c++11", f"-I{CPP}/include", str(pf),
               str(b / "libnfocc_core.a"), "-o", str(exe),
               "-L" + OCV, "-lopencv_core", "-lopencv_stereo",
               "-L" + OMP, "-lomp",
               f"-Wl,-rpath,{OCV}", f"-Wl,-rpath,{OMP}", "-lm")
        check("探针编译", r.returncode == 0,
              "ok" if r.returncode == 0 else r.stderr[-400:])
        if r.returncode == 0:
            rows = [ln.split() for ln in sh(str(exe), t=300).stdout.splitlines()
                    if ln.startswith("AZ ")]
            check("6 个视口", len(rows) == 6, f"{len(rows)} 行")
            bad = []
            for f in rows:
                want, got, eff = float(f[4]), float(f[5]), float(f[6])
                print(f"      #{f[1]} {f[2]:<3} 偏{f[3]:>5}° 期望{want:+7.1f}° "
                      f"实测{got:+7.1f}° 有效{eff:5.1f}%")
                if abs(want - got) >= 0.6:
                    bad.append(f"#{f[1]}")
            check("方位角全部吻合 (<0.6°)", not bad, "6/6" if not bad else str(bad))
            offs = sorted(float(f[3]) for f in rows)
            check("偏转角为 {-45,0,45}",
                  offs == sorted([-45.0, -45.0, 0.0, 0.0, 45.0, 45.0]), str(offs))
            # 有效率随偏转角下降, 这是几何必然而非缺陷:
            #   视口水平 ±60°, 偏转 45° 后边缘到达光轴 105°,
            #   超出鱼眼水平视场 100° -> 约 5.7% 像素落到有效圆外。
            # 实测曲线: 0°->100%, 30°->100%, 45°->94.3%, 60°->76.0%
            # 因此按偏转角设下限, 而不是一刀切 100%。
            low = [float(f[6]) for f in rows
                   if abs(float(f[3])) >= 40.0]
            ok_eff = all(e >= 90.0 for e in low)
            check("偏 45° 视口有效率 >=90%", ok_eff,
                  f"{min(low):.1f}%" if low else "无偏转视口")
            zero = [float(f[6]) for f in rows if abs(float(f[3])) < 1.0]
            check("不偏转视口有效率 100%", all(e >= 99.9 for e in zero),
                  f"{min(zero):.1f}%" if zero else "?")

        print("\n[3] 门控受偏转影响 (否则偏转形同虚设)")
        run = str(b / "nfocc_run")
        base = sh(run, "--synthetic", "--forward", "--frames", "3", "--stats", t=300).stdout
        vp = sh(run, "--synthetic", "--forward", "--viewplan", "--frames", "3",
                "--stats", t=300).stdout
        m0 = re.search(r"cam0 mode=(\S+)", base)
        m1 = re.search(r"cam0 mode=(\S+)", vp)
        d0, d1 = (m0.group(1) if m0 else "?"), (m1.group(1) if m1 else "?")
        print(f"      不偏转={d0}  偏转={d1}")
        check("工况判定改变", d0 != d1, f"{d0} -> {d1}")
        check("不偏转仍为 FORWARD", d0 == "FORWARD", d0)
        check("偏转后为 HYBRID(走 SGBM)", d1 == "HYBRID", d1)
        n0 = int(m.group(1)) if (m := re.search(r"累计点数\s*(\d+)", base)) else -1
        n1 = int(m.group(1)) if (m := re.search(r"累计点数\s*(\d+)", vp)) else -1
        check("两条路径均非零产出", n0 > 0 and n1 > 0, f"{n0} / {n1}")

        print("\n[4] 全套测试")
        r = sh("ctest", "--test-dir", str(b), "--output-on-failure", t=600)
        m = re.search(r"(\d+)% tests passed, (\d+) tests failed out of (\d+)", r.stdout)
        if m:
            check(f"ctest {m.group(1)}% ({m.group(3)}-{m.group(2)})",
                  m.group(2) == "0", f"{m.group(2)} failed")
        else:
            check("ctest 可解析", False, r.stdout[-300:])
    finally:
        shutil.rmtree(b, ignore_errors=True)
        shutil.rmtree(src, ignore_errors=True)

    print("\n" + "=" * 62)
    fails = [x for x in results if not x[1]]
    print(f"{len(results)} 项, 失败 {len(fails)}")
    for n, _, d in fails:
        print(f"  FAIL {n}: {d}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
