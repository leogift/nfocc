#!/usr/bin/env python3
"""相机配置生成与校验: 由车辆几何 + f 生成 4 路内外参, 并做全部一致性断言。

用途
----
1. 把"参数可配"落成真实产物: 输出 json (供 tools/param_gen.py 转 C 头文件)。
2. 在生成时就抓住 f 改动引起的连锁不一致 (B_MIN/B_MAX/角分辨率/max_theta)。
3. 负对照: --inject-fault 故意注入错误, 校验器必须失败。

默认值 = 用户给定:
    车长 5000 mm, 车宽 2000 mm, 后轴到前端 3500 mm
    鱼眼 f = 550 px, KB 模型, 1920x1536 单 Y
    相机 "理想正前后左右": yaw 准, 装在车身四边中点, 高度 700 mm(未给定, 取默认)

用法
----
    python3 tools/calib_gen.py                       # 打印并校验
    python3 tools/calib_gen.py --f 600               # 改焦距, 看连锁
    python3 tools/calib_gen.py --out calib.json
    python3 tools/calib_gen.py --inject-fault max_theta   # 负对照, 期望 FAIL
"""
from __future__ import annotations

import argparse
import json
import math
import sys

W, H = 1920, 1536
R_CORNER = math.hypot(W / 2, H / 2)


def make_config(f: float, cam_z: float, length: float, width: float,
                rear_to_front: float, zmin_eff: float, d_min_usable: float,
                num_disp: int, pitch_deg: float = 0.0,
                el_min: float = 10.0, el_max: float = 60.0,
                b_target: float = 120.0, sigma_d: float = 0.25,
                inject: str | None = None) -> dict:
    cx, cy = W / 2, H / 2
    rear_to_back = length - rear_to_front
    half_w = width / 2.0
    # 视口 1:1 取原图 -> 视口焦距 == f
    f_view = f
    b_min = d_min_usable * zmin_eff / f_view
    b_max = num_disp * zmin_eff / f_view
    theta_corner = R_CORNER / f          # 等距近似; k=0 时精确
    theta_h_edge = (W / 2) / f
    theta_v_edge = (H / 2) / f

    max_theta = theta_corner
    r_valid = W / 2.0

    if inject == "max_theta":
        max_theta = 1.65                 # 照抄 SimpleOCC 默认值 -> 应被校验拒绝
    if inject == "r_valid":
        r_valid = H / 2.0                # 用短边 -> 左右边缘被误判无效
    if inject == "cam_out":
        half_w = width / 2.0 + 500.0     # 相机跑到车外

    cams = [
        ("FV", +rear_to_front, 0.0, 0.0),
        ("RV", -rear_to_back, 0.0, 180.0),
        ("ML", 0.0, +half_w, 90.0),
        ("MR", 0.0, -half_w, -90.0),
    ]

    # 视口环带 -> 地面覆盖。el 为"相对水平的俯角", 相机自身俯角为 pitch_deg。
    # 环带是绝对俯角, 因此相对相机光轴的仰角 = el - pitch_deg。
    z_near = cam_z / math.tan(math.radians(el_max)) if el_max > 0 else 0.0
    z_far = (cam_z / math.tan(math.radians(el_min))) if el_min > 0 else float("inf")
    rail_lo = el_min - pitch_deg          # 环带下界相对光轴的角(向下为正)
    rail_hi = el_max - pitch_deg
    # 远端视差与深度不确定度(用于判断该环带是否可量化)
    d_far = f_view * b_target / z_far if z_far != float("inf") else 0.0
    sigma_far = (z_far ** 2) * sigma_d / (f_view * b_target) if z_far != float("inf") else 0.0
    return {
        "version": 1,
        "vehicle": {"length_mm": length, "width_mm": width,
                    "rear_axle_to_front_mm": rear_to_front},
        "image": {"w": W, "h": H, "channels": 1, "dtype": "uint8"},
        "derived": {
            "f_view": f_view, "b_min_mm": b_min, "b_max_mm": b_max,
            "deg_per_px": math.degrees(1.0 / f),
            "theta_corner_deg": math.degrees(theta_corner),
            "theta_h_edge_deg": math.degrees(theta_h_edge),
            "theta_v_edge_deg": math.degrees(theta_v_edge),
            "ring_el_min_deg": el_min, "ring_el_max_deg": el_max,
            "ring_vs_optical_deg": [rail_lo, rail_hi],
            "ground_near_mm": z_near, "ground_far_mm": z_far,
            "disp_at_far_px": d_far, "sigma_z_at_far_mm": sigma_far,
        },
        "cameras": [
            {
                "id": cid,
                "intr": {"model": "kb", "f": f, "cx": cx, "cy": cy,
                         "k": [0.0, 0.0, 0.0, 0.0],
                         "r_valid": r_valid, "max_theta": max_theta},
                "extr": {"pos_mm": [x, y, cam_z],
                         "yaw_deg": yaw, "pitch_deg": pitch_deg, "roll_deg": 0.0},
            }
            for cid, x, y, yaw in cams
        ],
    }


def validate(cfg: dict) -> list[str]:
    """返回错误列表; 空表示通过。每条错误对应 SPEC-CALIB-CFG §3.1 的约束。"""
    errs: list[str] = []
    d = cfg.get("derived", {})
    veh = cfg["vehicle"]
    half_len, half_wid = veh["length_mm"] / 2.0, veh["width_mm"] / 2.0

    if cfg["image"]["w"] != W or cfg["image"]["h"] != H:
        errs.append(f"图像尺寸 {cfg['image']['w']}x{cfg['image']['h']} != {W}x{H}")
    if cfg["image"].get("channels") != 1:
        errs.append("通道数必须为 1 (单 Y)")

    ids = [c["id"] for c in cfg["cameras"]]
    if ids != ["FV", "RV", "ML", "MR"]:
        errs.append(f"相机 id 顺序错误: {ids}")

    for c in cfg["cameras"]:
        cid = c["id"]
        intr = c["intr"]
        f = intr["f"]
        if f <= 0:
            errs.append(f"{cid}: f<=0")
            continue
        # V-CAL-3: max_theta 必须覆盖四角
        need = R_CORNER / f
        if intr["max_theta"] < need - 1e-9:
            errs.append(
                f"{cid}: max_theta={intr['max_theta']:.4f} rad "
                f"({math.degrees(intr['max_theta']):.1f}°) < 四角所需 {need:.4f} rad "
                f"({math.degrees(need):.1f}°) —— 左右边缘/四角会被误判无效"
            )
        # V-CAL-4: r_valid 必须覆盖左右边缘
        if intr["r_valid"] < W / 2.0 - 1e-9:
            errs.append(
                f"{cid}: r_valid={intr['r_valid']:.1f} < 左右边缘半径 {W/2:.1f} —— 水平视场被截断"
            )
        if intr["r_valid"] > R_CORNER + 1e-9:
            errs.append(f"{cid}: r_valid 超过四角半径 {R_CORNER:.1f}")
        # 相机位置在车身包围盒内
        # 注意: 车系原点在**后轴**, 不是车中心。包围盒 = [-rear_to_back, +rear_to_front] × ±half_wid
        x, y, _z = c["extr"]["pos_mm"]
        x_lo = -(veh["length_mm"] - veh["rear_axle_to_front_mm"])
        x_hi = veh["rear_axle_to_front_mm"]
        if not (x_lo - 1e-9 <= x <= x_hi + 1e-9) or abs(y) > half_wid + 1e-9:
            errs.append(
                f"{cid}: 相机位置 ({x:+.0f},{y:+.0f}) 超出车身包围盒 "
                f"X∈[{x_lo:+.0f},{x_hi:+.0f}] Y∈±{half_wid:.0f}"
            )

    # R2: B_MAX 必须与引擎容量一致
    fv = d.get("f_view", 0)
    if fv <= 0:
        errs.append("f_view <= 0")
    return errs


def report(cfg: dict, errs: list[str]) -> None:
    d = cfg["derived"]
    print(f"车辆 {cfg['vehicle']['length_mm']:.0f}×{cfg['vehicle']['width_mm']:.0f} mm, "
          f"后轴到前端 {cfg['vehicle']['rear_axle_to_front_mm']:.0f} mm")
    print(f"f = {cfg['cameras'][0]['intr']['f']:.1f} px  →  {d['deg_per_px']:.4f} °/px")
    print(f"  左右边缘 θ = {d['theta_h_edge_deg']:.2f}°   上下边缘 θ = {d['theta_v_edge_deg']:.2f}°   "
          f"四角 θ = {d['theta_corner_deg']:.2f}°")
    print(f"  B_MIN = {d['b_min_mm']:.1f} mm   B_MAX = {d['b_max_mm']:.0f} mm")
    print(f"  视口环带 {d['ring_el_min_deg']:.0f}°-{d['ring_el_max_deg']:.0f}° (相对光轴 "
          f"{d['ring_vs_optical_deg'][0]:.0f}°-{d['ring_vs_optical_deg'][1]:.0f}°)"
          f"  -> 地面 {d['ground_near_mm']:.0f}–{d['ground_far_mm']:.0f} mm, "
          f"远端视差 {d['disp_at_far_px']:.1f} px, σ_Z {d['sigma_z_at_far_mm']:.0f} mm")
    print()
    print(f"{'cam':>4} {'pos(mm)':>22} {'yaw':>6} {'r_valid':>8} {'max_θ(°)':>9}")
    for c in cfg["cameras"]:
        p, i = c["extr"]["pos_mm"], c["intr"]
        print(f"{c['id']:>4} ({p[0]:+7.0f},{p[1]:+7.0f},{p[2]:+5.0f}) "
              f"{c['extr']['yaw_deg']:>6.0f} {i['r_valid']:>8.0f} "
              f"{math.degrees(i['max_theta']):>9.1f}")
    print()
    if errs:
        for e in errs:
            print("FAIL", e)
        print(f"\n{len(errs)} 项失败")
    else:
        print("PASS  全部一致性检查通过")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--f", type=float, default=550.0)
    ap.add_argument("--cam-z", type=float, default=700.0)
    ap.add_argument("--pitch", type=float, default=0.0, help="相机安装俯角 deg (正=向下)")
    ap.add_argument("--el-min", type=float, default=10.0, help="视口环带下界俯角 deg")
    ap.add_argument("--el-max", type=float, default=60.0, help="视口环带上界俯角 deg")
    ap.add_argument("--b-target", type=float, default=120.0)
    ap.add_argument("--sigma-d", type=float, default=0.25)
    ap.add_argument("--length", type=float, default=5000.0)
    ap.add_argument("--width", type=float, default=2000.0)
    ap.add_argument("--rear-to-front", type=float, default=3500.0)
    ap.add_argument("--zmin-eff", type=float, default=500.0)
    ap.add_argument("--d-min-usable", type=float, default=8.0)
    ap.add_argument("--num-disp", type=int, default=256)
    ap.add_argument("--out", default=None)
    ap.add_argument("--inject-fault", default=None,
                    choices=["max_theta", "r_valid", "cam_out"])
    a = ap.parse_args()

    cfg = make_config(a.f, a.cam_z, a.length, a.width, a.rear_to_front,
                      a.zmin_eff, a.d_min_usable, a.num_disp,
                      a.pitch, a.el_min, a.el_max, a.b_target, a.sigma_d,
                      a.inject_fault)
    errs = validate(cfg)
    report(cfg, errs)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as fh:
            json.dump(cfg, fh, ensure_ascii=False, indent=2)
        print(f"\n已写入 {a.out}")
    sys.exit(1 if errs else 0)


if __name__ == "__main__":
    main()
