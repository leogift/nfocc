#!/usr/bin/env python3
"""NFOCC 文档数字核算探针。

用途: 把 doc/ 三份文档里的关键数字与我们建议的整定值放在一起验算。
改 PARAM 区即可复跑, 不需要读文档。

数学约定
--------
* 运动双目: Z = f_px * b / d,  d 为视差(px), b 为基线(mm), f_px 为视口等效焦距(px)。
* 深度量化: dZ = Z^2 * delta_d / (f_px * b)
* equirect 展开: f_eq = W / Theta, W 为展开图宽(px), Theta 为覆盖的方位角(rad)。
  注意这是**每弧度多少像素**, 当且仅当视口 1:1 采样展开图时等于针孔等效焦距。

用法
----
    python3 tools/numcheck.py                # 打印预设场景表
    python3 tools/numcheck.py --sweep        # 扫 f_eq / b / Zmin 的关系
"""
from __future__ import annotations

import argparse
import math

# ---------------------------------------------------------------- 核心公式


def disparity(f_px: float, b_mm: float, z_mm: float) -> float:
    """视差(px)。"""
    return f_px * b_mm / z_mm


def near_limit(f_px: float, b_mm: float, d_max_px: float) -> float:
    """最近可测距离(mm)。d_max_px 为视差搜索上限(px, 已统一单位)。"""
    return f_px * b_mm / d_max_px


def far_limit(f_px: float, b_mm: float, d_min_px: float) -> float:
    """最远可测距离(mm)。"""
    return f_px * b_mm / d_min_px


def depth_resolution(f_px: float, b_mm: float, z_mm: float, delta_d_px: float) -> float:
    """由视差量化单位引起的深度分辨率(mm)。"""
    return z_mm * z_mm * delta_d_px / (f_px * b_mm)


def f_eq(width_px: float, theta_rad: float = 2 * math.pi) -> float:
    """展开图等效焦距 px/rad。"""
    return width_px / theta_rad


# ---------------------------------------------------------------- 预设场景
# fence 表示"只打印不判定", 判定值在下方 PRESETS 里显式写死作为回归基线。


def preset_rows() -> list[dict]:
    """把文档中的配置与建议配置并列, 用于人工比对。"""
    rows = []

    def add(tag, f, b, dmax_px, dmin_px, delta_d, probes):
        r = {
            "tag": tag,
            "f_px": f,
            "b_mm": b,
            "d_max_px": dmax_px,
            "d_min_px": dmin_px,
            "delta_d_px": delta_d,
            "z_near_mm": near_limit(f, b, dmax_px),
            "z_far_mm": far_limit(f, b, dmin_px),
        }
        # 固定列: 5 m 超出部分档位视差不足 1/16px 时填 NaN, 打印成 "-"。
        for z in PROBE_Z_M:
            r[col(z)] = depth_resolution(f, b, z * 1000.0, delta_d)
        rows.append(r)
        return r

    # doc1 §3.1: f_px=300, b=200mm, 1px 量化
    add("doc1 §3.1", 300, 200, 191, 1, 1.0, PROBE_Z_M)
    # doc2 附录A: SDE 191px @ b=500
    add("doc2 A/SDE191", 300, 500, 191, 1, 1.0, PROBE_Z_M)
    # doc3 表6: f_eq=200, b=200, numDisp=256 -> 16px
    add("doc3 表6 b200", 200, 200, 16, 0.0625, 1 / 16, PROBE_Z_M)
    # doc3 表6 b_min=40
    add("doc3 表6 b40", 200, 40, 16, 0.0625, 1 / 16, PROBE_Z_M)
    # 建议: 柱面视口 ±90deg, W=1280 -> f=407; b=100; dmax=256px
    add("suggest cyl1280", 1280 / math.pi, 100, 256, 0.0625, 1 / 16, PROBE_Z_M)
    # 建议: 高分辨率视口 W=1024 覆盖 120deg -> f=489
    add("suggest cyl120 1024w", 1024 / math.radians(120), 100, 256, 0.0625, 1 / 16, PROBE_Z_M)
    return rows


PROBE_Z_M = (0.5, 1, 2, 3, 5)


def col(z_m: float) -> str:
    return f"dz@{z_m:g}m"


def fmt(v) -> str:
    if isinstance(v, str):
        return f"{v:>12}"
    if math.isnan(v) or v > 99999:
        return f"{'-':>12}"
    return f"{v:>12.1f}"


def print_presets() -> None:
    rows = preset_rows()
    keys = list(rows[0].keys())
    hdr = " | ".join(f"{k:>12}" for k in keys)
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(" | ".join(fmt(r[k]) for k in keys))
    print()
    print("注: z_near 用 d_max 作视差上限; z_far 用 d_min。dz@Nm 是 1 个量化单位对应的深度误差(mm)。")


def print_sweep() -> None:
    print("f_eq 对近距极限与精度的影响 (b=100mm, dmax=256px, 量化 1/16px):")
    print(f"{'W(px)':>8} {'theta':>8} {'f_eq':>8} {'Zmin(mm)':>10} {'dZ@2m':>8} {'dZ@3m':>8}")
    for width in (640, 960, 1280, 1920, 2560):
        for theta_deg in (90, 120, 180, 360):
            f = f_eq(width, math.radians(theta_deg))
            print(
                f"{width:>8} {theta_deg:>8} {f:>8.1f}"
                f" {near_limit(f, 100, 256):>10.0f}"
                f" {depth_resolution(f, 100, 2000, 1 / 16):>8.0f}"
                f" {depth_resolution(f, 100, 3000, 1 / 16):>8.0f}"
            )
    print()
    print("同一条鱼眼(190deg FOV, 1920px 宽)的原始角分辨率:")
    print(f"  中心 ~ {1920 / math.radians(190):.1f} px/rad")
    print(f"  1:1 展开成全 360deg 全景需要宽度 {1920 / math.radians(190) * 2 * math.pi:.0f} px")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sweep", action="store_true", help="打印 f_eq 扫描表")
    args = ap.parse_args()
    if args.sweep:
        print_sweep()
    else:
        print_presets()


if __name__ == "__main__":
    main()
