#!/usr/bin/env python3
"""SGBM 在近场大视差下能不能跑: 实测耗时、内存与正确性。

背景: 运动双目近场视差量级 100-200 px(见 01-numcheck C13c), 常规 SGBM 配置
numDisparities=256 能否覆盖, 是本项目选型的前提。

实测要点(踩过的坑, 别再踩)
--------------------------
1. OpenCV `numDisparities` 的单位是**像素**, 不是 1/16 像素。
   `compute()` 返回 CV_16S, 数值单位是 1/16 px, 要除以 16 才是像素视差。
2. 生成测试对时**不能**用 warpAffine + BORDER_REFLECT:
   反射边界会让左图的右边缘内容跑到右图左边缘, 在近场大视差下制造虚假匹配,
   使小 numDisparities 也能"测出"大视差 —— 第一版探针就是这么得出错误结论的。
   正确做法: 直接切片搬运, 无效区留 0。
3. 正确性必须用一个已知视差来验, 只看"耗时"会把错误当成功。

用法
----
    python3 tools/sgbm_bench.py
    python3 tools/sgbm_bench.py --w 1920 --h 1080
    python3 tools/sgbm_bench.py --max-disp 512
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np


def make_pair(w: int, h: int, disparity: int, texture: str = "rich") -> tuple[np.ndarray, np.ndarray]:
    """生成一对已知视差的灰度图。

    关系定义(手数过, 不要再改错): disp(x) = D  <=>  left[x] 与 right[x - D] 是同一物点,
    等价于 right[j] = left[j + D]。相机右移时物点像左移, 故 D > 0。
    构造: 右图左侧留 D 列黑边(无内容), 左图右侧裁掉 D 列(无内容)。
    """
    rng = np.random.default_rng(0)
    if texture == "rich":
        src = cv2.GaussianBlur(rng.integers(0, 256, (h, w), dtype=np.uint8), (5, 5), 0)
    else:
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        base = 90.0 + 20.0 * np.sin(xx / 120.0) + 10.0 * np.cos(yy / 200.0)
        src = np.clip(base + rng.normal(0, 2.0, (h, w)), 0, 255).astype(np.uint8)
    d = abs(disparity)
    left = src.copy()
    right = np.zeros_like(src)
    if disparity > 0:
        # 目标 left[x] == right[x-d]  =>  right[j] = src[j+d]
        right[:, : w - d] = src[:, d:]
    else:
        right[:, d:] = src[:, : w - d]
    # 自检: 有效区内必须逐像素对得上, 否则后面的统计全是假的
    for _x in (d + 3, w // 2, w - 3):
        if _x - d >= 0 and _x < w:
            assert np.array_equal(left[:, _x], right[:, _x - d]), f"pair 构造错误 x={_x} d={d}"
    return left, right


def valid_mask(w: int, h: int, disparity: int) -> np.ndarray:
    """左图 x 在 [d, w) 时在右图有对应; 右图 j 在 [0, w-d) 有内容 => 共同有效列 [d, w)。"""
    d = abs(disparity)
    m = np.zeros((h, w), dtype=bool)
    m[:, d:] = True
    return m


def build(num_disp: int, block: int = 5) -> cv2.StereoSGBM:
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block,
        P1=8 * block * block,
        P2=32 * block * block,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def bench(w: int, h: int, disp_list: list[int], texture: str) -> None:
    true_d = 180
    left, right = make_pair(w, h, disparity=true_d, texture=texture)
    print(f"图像 {w}x{h} 纹理={texture} 真实视差={true_d}px opencv={cv2.__version__}")
    print(
        f"{'numDisp':>8} {'容量px':>7} {'耗时ms':>8} {'cost vol MB':>12}"
        f" {'中位视差px':>10} {'有效率':>7} {'正确?':>6} {'中位误差px':>10}"
    )
    for nd in disp_list:
        try:
            sgbm = build(nd)
            sgbm.compute(left, right)  # 预热
            t0 = time.perf_counter()
            n = 3
            for _ in range(n):
                disp16 = sgbm.compute(left, right)
            dt = (time.perf_counter() - t0) / n
        except cv2.error as e:
            msg = (e.err or str(e)).strip().splitlines()[-1]
            print(f"{nd:>8} 失败: {msg[:60]}")
            continue
        disp = disp16.astype(np.float32) / 16.0
        roi = valid_mask(w, h, true_d)
        disp_roi = disp[roi]
        valid = disp_roi[disp_roi > 0]
        med = float(np.median(valid)) if len(valid) else float("nan")
        # 有效率只在"有对应内容"的区域统计, 否则黑边伪匹配会稀释结果
        rate = len(valid) / max(1, disp_roi.size)
        # 容量: numDisparities 即视差搜索上限(px)
        ok = "yes" if abs(med - true_d) < 1.0 else "NO"
        mem = nd * w * h * 2 / 1e6
        err = float(np.median(np.abs(valid - true_d))) if len(valid) else float("nan")
        print(
            f"{nd:>8} {nd:>7} {dt * 1000:>8.1f} {mem:>12.0f} {med:>10.1f}"
            f" {rate:>6.1%} {ok:>6} {err:>8.2f}"
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    ap.add_argument("--texture", choices=["rich", "weak"], default="rich")
    ap.add_argument("--max-disp", type=int, default=1024)
    args = ap.parse_args()
    seq = [64, 128, 192, 256, 384, 512, 768, 1024]
    seq = [d for d in seq if d <= args.max_disp]
    bench(args.w, args.h, seq, args.texture)


if __name__ == "__main__":
    main()
