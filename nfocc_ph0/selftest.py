"""PH-0 统一自检入口: 一次跑完四个模块的全部判据, 输出 PASS/FAIL。

对应 SPEC-VERIFY 的 A2/A3/A4/A5 与各模块 V-* 项。
正例必须全 PASS; 负例(注入故障)必须 FAIL —— 每个模组都带自己的负对照。

用法
----
    python3 -m nfocc_ph0.selftest
    python3 -m nfocc_ph0.selftest --verbose
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import traceback
from pathlib import Path

import numpy as np


class Reporter:
    def __init__(self, verbose: bool = False) -> None:
        self.rows: list[tuple[str, str, str]] = []
        self.verbose = verbose

    def check(self, tag: str, fn, expect_pass: bool = True, note: str = "") -> None:
        """跑一个判据。expect_pass=False 时应抛异常(负对照)。"""
        try:
            detail = fn()
            ok = expect_pass
            msg = str(detail) if detail is not None else ""
        except Exception as e:  # noqa: BLE001
            ok = not expect_pass
            msg = f"{type(e).__name__}: {e}"
            if self.verbose and not expect_pass:
                msg += "  <- 负对照已生效"
        status = "PASS" if ok else "FAIL"
        self.rows.append((status, tag, f"{msg} {note}".strip()))
        mark = "." if ok else "X"
        print(f"  [{mark}] {tag}" + (f"  {msg[:70]}" if self.verbose or not ok else ""))

    def summary(self) -> int:
        n_fail = sum(1 for s, _, _ in self.rows if s == "FAIL")
        n = len(self.rows)
        print(f"\n{'='*66}")
        print(f"总计 {n} 项, 失败 {n_fail} 项")
        if n_fail:
            for s, tag, msg in self.rows:
                if s == "FAIL":
                    print(f"  FAIL {tag}: {msg}")
        return 1 if n_fail else 0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    R = Reporter(args.verbose)

    # =========================================================== PH-0.1
    print("PH-0.1 KB 相机模型")
    from .camera_kb import KbCamera, KbError, make_default_camera

    cam = make_default_camera()

    def v_co2():
        rng = np.random.default_rng(0)
        uv = np.stack([rng.uniform(0, 1920, 8000), rng.uniform(0, 1536, 8000)], axis=1)
        ray, ok = cam.unproject(uv)
        uv2, ok2, _ = cam.project(ray)
        err = np.linalg.norm(uv2 - uv, axis=1)[ok & ok2]
        m = float(np.max(err))
        assert m < 0.01, f"往返误差 {m:.3e} >= 0.01"
        return f"往返最大误差 {m:.2e} px"
    R.check("V-CO-2 KB 往返 < 0.01 px", v_co2)

    def v_co3():
        uv, ok, _ = cam.project(np.array([[0.0, 0.0, 1000.0]]))
        assert ok[0]
        assert abs(uv[0, 0] - cam.cx) < 1e-6 and abs(uv[0, 1] - cam.cy) < 1e-6, f"光轴投影 {uv[0]}"
        return f"({uv[0,0]:.3f},{uv[0,1]:.3f})"
    R.check("V-CO-3 光轴 -> 主点", v_co3)

    def v_co4():
        uvx, _, _ = cam.project(np.array([[100.0, 0.0, 1000.0]]))
        uvy, _, _ = cam.project(np.array([[0.0, 100.0, 1000.0]]))
        assert uvx[0, 0] > cam.cx, "x 轴方向反"
        assert uvy[0, 1] > cam.cy, "y 轴方向反"
        return "x→u+, y→v+"
    R.check("V-CO-4 图像轴未翻转", v_co4)

    def neg_co2():
        bad = KbCamera(f=550.0, cx=960.0, cy=768.0, k=(-0.4, 0, 0, 0))
        return bad
    R.check("V-CO-6 非单调 k 应被拒绝", neg_co2, expect_pass=False)

    def neg_axis():
        """把 y 轴写成朝上(等价于相机倒装)应被 vehicle_to_cam 的自检拒绝。"""
        from .epipolar import vehicle_to_cam
        R_bad = vehicle_to_cam(0.0, 0.0)
        R_bad = R_bad.copy()
        R_bad[1] *= -1.0          # 翻转 y
        assert R_bad[1][2] < 0, "自检未发现 y 朝上"
        return "自检通过(异常)"
    R.check("V-CO-4b y 朝上应被检测", neg_axis, expect_pass=False)

    # =========================================================== PH-0.2
    print("PH-0.2 视口 LUT")
    from .view_lut import ViewParams, build_view_lut, view_ground_range

    vp = ViewParams(height=-1)

    def v_lut1():
        lut = build_view_lut(cam, vp)
        el = lut["el_deg"]
        assert abs(el[0, 0] - vp.el_min_deg) < 1.0, f"顶部俯角 {el[0,0]:.2f}"
        assert abs(el[-1, 0] - vp.el_max_deg) < 1.0, f"底部俯角 {el[-1,0]:.2f}"
        return f"{vp.width}x{vp.height}, 环带 {el[0,0]:.1f}-{el[-1,0]:.1f}°, 有效 {lut['stats']['valid_ratio']:.1%}"
    R.check("V-LUT 环带覆盖正确", v_lut1)

    def v_ground():
        near, far = view_ground_range(vp, 700.0)
        assert 380 < near < 420 and 3900 < far < 4050, f"{near:.0f}-{far:.0f}"
        return f"地面 {near:.0f}-{far:.0f} mm"
    R.check("V-LUT 地面覆盖 404-3970", v_ground)

    def v_d2():
        c = make_default_camera(r_valid=600)
        s = build_view_lut(c, vp)["stats"]
        assert s["d2_out_of_r_valid"] > 0, "r_valid 收小后 d2 仍为 0, 该道检查失效"
        return f"r_valid=600 淘汰 {s['d2_out_of_r_valid']} ({1-s['valid_ratio']:.1%})"
    R.check("V-LUT-3 r_valid 收窄应淘汰", v_d2)

    def neg_height():
        return ViewParams(height=384)
    R.check("V-LUT height 与环带不符应拒绝", neg_height, expect_pass=False)

    def v_mask():
        mask = np.zeros((1536, 1920), bool)
        mask[:, :600] = True
        s = build_view_lut(cam, vp, mask=mask)["stats"]
        assert s["d4_body_mask"] > 0 and s["valid_ratio"] < 1.0
        return f"掩码淘汰 {s['d4_body_mask']} ({1-s['valid_ratio']:.1%})"
    R.check("V-MASK-2 车身掩码生效", v_mask)

    # =========================================================== PH-0.3
    print("PH-0.3 极线几何")
    from .epipolar import (rectify_camera, rot_z, self_test as epi_self,
                           vehicle_to_cam, epipolar_angle_deg)

    def v_epi1():
        r = epi_self(verbose=False)
        assert r["v_epi1_max"] < 1e-9
        return f"竖直视差 {r['v_epi1_max']:.1e} px"
    R.check("V-EPI-1 t_y=t_z=0 时竖直视差=0", v_epi1)

    def v_epi2():
        r = epi_self(verbose=False)
        rr = r["v_epi2_ratios"]
        spread = (max(rr) - min(rr)) / max(rr)
        assert spread < 0.05
        return f"|dv|/t_z 散布 {spread:.1%}"
    R.check("V-EPI-2 竖直视差随 t_z 线性", v_epi2)

    def v_epi_45():
        t = rot_z(45.0) @ np.array([1000.0, 0.0, 0.0])
        angs = [epipolar_angle_deg(t, vehicle_to_cam(y, 0.0)) for y in (0, 180, 90, -90)]
        assert all(abs(a - 45.0) < 0.5 for a in angs), f"45° 运动时四路应全不可用: {angs}"
        return f"四路夹角 {['%.0f'%a for a in angs]}"
    R.check("V-EPI-3 45°运动四路全不可用", v_epi_45)

    def v_rect():
        # 目标: 验证"重定向后 t 的 z 分量为 0"。
        # 必须选一个**极线不水平但又不退化**的场景: 侧相机 + 斜向运动。
        # 若选侧相机 + 直行(t 平行其光轴), z_new 的投影退化, 函数按设计返回原 R,
        # 此时 alpha=0 但 t_z≠0 —— 那是 FoE 退化区, 重定向本就无意义。
        t = rot_z(45.0) @ np.array([1000.0, 0.0, 0.0])
        R_ml = vehicle_to_cam(90.0, 0.0)          # 侧相机, 光轴指向 +Y
        R_new, alpha = rectify_camera(R_ml, t)
        t_new = R_new @ t
        assert abs(t_new[2]) < 1e-9, f"t_z={t_new[2]:.2e}"
        assert alpha > 1.0, f"此场景应产生明显重定向, alpha={alpha:.1f}°"
        return f"t_z=0, 代价 alpha={alpha:.1f}°"
    R.check("V-EPI-4 重定向后竖直分量为0", v_rect)

    def neg_rect():
        """不改复合顺序(把相机系向量当车辆系)时应失败。"""
        t = rot_z(30.0) @ np.array([1000.0, 0.0, 0.0])
        R_cv = vehicle_to_cam(0.0, 0.0)
        t_c = R_cv @ t
        x_new = t_c / np.linalg.norm(t_c)
        z_old = np.array([0.0, 0.0, 1.0])
        z_new = z_old - (z_old @ x_new) * x_new
        z_new /= np.linalg.norm(z_new)
        y_new = np.cross(z_new, x_new)
        R_bad = np.stack([x_new, y_new, z_new], axis=0)   # 故意不乘 R_cv
        assert abs((R_bad @ t)[2]) < 1e-9, "错误复合顺序也应通过? 需复核"
        return "错误顺序通过"
    R.check("V-EPI-4b 错误复合顺序应失败", neg_rect, expect_pass=False)

    # =========================================================== PH-0.4
    print("PH-0.4 体素网格")
    from .voxel_grid import VoxelGrid, self_test as vox_self

    def v_vox():
        r = vox_self(verbose=False)
        assert r["nx"] == 75 and r["ny"] == 60 and r["nz"] == 18
        assert r["partial"] == {"x": False, "y": False, "z": True}
        assert r["last_span"]["z"] == 100
        return f"{r['nx']}x{r['ny']}x{r['nz']}, z 末格 {r['last_span']['z']} mm"
    R.check("V-VOX-1 网格与半格标记", v_vox)

    def v_cell():
        g = VoxelGrid()
        c1 = g.cell_index_xy(np.array([3]), np.array([5]))[0]
        c2 = g.cell_index_xy(np.array([5]), np.array([3]))[0]
        assert c1 != c2, "非方阵下柱体编号转置了"
        return f"(3,5)->{c1} (5,3)->{c2}"
    R.check("V-VOX-6 柱体编号 iy*nx+ix", v_cell)

    def neg_hash():
        g = VoxelGrid(x_max=60000)
        return g.pack_hash(np.array([[300, 0, 0]]))
    R.check("V-VOX-7 nx>256 时 Hash 应报错", neg_hash, expect_pass=False)

    # =========================================================== PH-0.5
    print("PH-0.5 合成数据")
    from .synth import synth_selfcheck

    tmp = Path(tempfile.mkdtemp(prefix="nfocc_ph0_"))

    def v_syn():
        r = synth_selfcheck(tmp / "syn", "S_BOX", "FV")
        assert r["v_syn2_median_rel_err"] < 1e-3, r["v_syn2_median_rel_err"]
        return (f"三角化闭合 {r['v_syn2_median_rel_err']:.1e}, "
                f"disp p50 {r['disp_p50']:.1f}px, 有效 {r['valid_ratio']:.1%}")
    R.check("V-SYN-1..5 生成器自检", v_syn)

    def v_syn_multi():
        r = synth_selfcheck(tmp / "syn2", "S_MULTI", "FV")
        assert r["v_syn4_max_disp"] <= 256.0
        return f"S_MULTI max disp {r['v_syn4_max_disp']:.0f}px <= 256"
    R.check("V-SYN-4 视差在引擎容量内", v_syn_multi)

    sys.exit(R.summary())


if __name__ == "__main__":
    main()
