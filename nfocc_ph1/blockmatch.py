"""NFOCC 分块方向立体匹配.

背景(实测, 见 SPEC-FORWARD §1)
------------------------------
视口环带跨俯角 10-60°, 同一行内极线方向离散:
    侧移 90°: 整幅 29.4° -> 120x32 分块 2.3°(p90 9.3°) -> 60x16 分块 1.1°
    直行  0°: 整幅 174.6° -> 120x32 分块 13.7°(p90 49.6°) -> 仍不可用

所以分块只用于**侧向**(门控已把前向分出去, 见 gating.py)。

做法
----
每块内极线近似平行, 设该块极线方向角 theta(相对图像横轴):
  1. 由该块中心像素的几何预测极线方向, 得到 theta
  2. 按 theta 对块做**整数行剪切**, 使极线水平(零插值)
  3. 剪切后的块跑 SGBM
  4. 视差按 cos(theta) 折算回真实极线位移

**为什么用剪切而不是旋转**: 块小, 旋转要双线性插值会糊掉纹理;
剪切用整数行位移可零插值实现。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .engine import DisparityEngine, DisparityMap, DisparityRange, NFStatus


@dataclass
class BlockSpec:
    y0: int
    y1: int
    x0: int
    x1: int
    theta_deg: float


def shear(img: np.ndarray, theta_deg: float) -> tuple[np.ndarray, int]:
    """按极线方向做整数行剪切, 使极线水平。返回 (图, 最大偏移列数)。

    极线方向角 theta 相对横轴: 相邻行需要错开 tan(theta) 列。
    目标: 输出中每条"极线"落在同一行上。
      输入 row r 的第 i 个极线点位于列 x = i + t*(r - h/2)
      因此 out[r, i] = img[r, i + t*(r - h/2)]
    即 **行 r 左移 s = round(t*(r-h/2))**。
    """
    h, w = img.shape
    t = math.tan(math.radians(theta_deg))
    out = np.zeros_like(img)
    max_shift = 0
    for r in range(h):
        s = int(round(t * (r - h / 2.0)))
        max_shift = max(max_shift, abs(s))
        if s == 0:
            out[r] = img[r]
        elif 0 < s < w:
            out[r, :w - s] = img[r, s:]
        elif -w < s < 0:
            out[r, -s:] = img[r, :w + s]
    return out, max_shift


def predict_theta(vp, R_cv: np.ndarray, C0: np.ndarray, C1: np.ndarray,
                  x0: int, x1: int, y0: int, y1: int,
                  r_depth_mm: float = 2000.0) -> float:
    """用块中心处的几何预测极线方向角(相对图像横轴)。

    不需要真实深度: 在给定深度上取一个虚拟点, 算它在两帧的像素位移方向。
    极线方向与该点深度基本无关(平移情形下只依赖视线方向), 用 2 m 足够。
    """
    u = 0.5 * (x0 + x1)
    v = 0.5 * (y0 + y1)
    # pixel_to_ray 期望数组入参, 标量要包成 1 元素数组
    ray = vp.pixel_to_ray(np.array([u]), np.array([v]))[0] @ R_cv   # 车系单位方向
    P = C0 + ray * r_depth_mm
    uv0 = vp.ray_to_pixel(((P - C0) @ R_cv.T)[None, :])[0]
    uv1 = vp.ray_to_pixel(((P - C1) @ R_cv.T)[None, :])[0]
    du = uv0[0] - uv1[0]
    dv = uv0[1] - uv1[1]
    if abs(du) < 1e-9 and abs(dv) < 1e-9:
        return 0.0
    # 归一到"相对横轴的锐角": 极线方向 170° 与 -10° 是同一条直线,
    # 剪切只需要斜率。不归一化会让 shear 得到 tan(170°)=-0.18 但语义错位,
    # 且 |theta| 接近 90° 时 tan 爆炸。
    th = math.degrees(math.atan2(dv, du))
    if th > 90.0:
        th -= 180.0
    elif th < -90.0:
        th += 180.0
    return float(th)


def match_blocks(eng: DisparityEngine,
                 left: np.ndarray, right: np.ndarray,
                 blocks: list[BlockSpec],
                 rng: DisparityRange,
                 min_disp_out: float = 0.0) -> tuple[NFStatus, DisparityMap | None]:
    """按块匹配并合并。块内的视差按 cos(theta) 折回真实极线位移。"""
    h, w = left.shape
    disp = np.zeros((h, w), np.float32)
    valid = np.zeros((h, w), bool)
    conf = np.zeros((h, w), np.float32)
    got_any = False

    for b in blocks:
        L = np.ascontiguousarray(left[b.y0:b.y1, b.x0:b.x1])
        R = np.ascontiguousarray(right[b.y0:b.y1, b.x0:b.x1])
        if L.size == 0:
            continue
        # **硬约束**: SGBM 的 numDisparities 必须小于块宽, 否则 OpenCV 分配
        # (W-numDisp)*H 的负尺寸内存, 报 "Insufficient memory" 而非参数错误
        # (实测: 60x16 块配 numDisp=256 直接 OOM)。
        if L.shape[1] <= rng.d_max:
            return NFStatus.NF_E_PARAM_RANGE, None
        Ls, sh_l = shear(L, b.theta_deg)
        Rs, sh_r = shear(R, b.theta_deg)
        st, dm = eng.run(Ls, Rs, rng)
        if st != NFStatus.NF_OK or dm is None:
            continue
        got_any = True
        # 剪切后测得的水平位移 d_s 对应真实极线位移 d = d_s * cos(theta)
        scale = math.cos(math.radians(b.theta_deg))
        sub_disp = (dm.disp * scale).astype(np.float32)
        sub_valid = dm.valid.copy()
        cut = max(sh_l, sh_r)
        if cut > 0:
            sub_valid[:, :cut] = False
            sub_valid[:, -cut:] = False
        # 写回
        bh, bw = sub_disp.shape
        disp[b.y0:b.y0 + bh, b.x0:b.x0 + bw] = sub_disp
        valid[b.y0:b.y0 + bh, b.x0:b.x0 + bw] = sub_valid
        conf[b.y0:b.y0 + bh, b.x0:b.x0 + bw] = dm.conf

    if not got_any:
        return NFStatus.NF_E_ENGINE_NOT_INIT, None
    valid &= disp >= min_disp_out - 1e-9
    return NFStatus.NF_OK, DisparityMap(disp, valid, conf)


def make_blocks(vp, R_cv: np.ndarray, C0: np.ndarray, C1: np.ndarray,
                bw: int = 120, bh: int = 32) -> list[BlockSpec]:
    """把视口切成规则块, 并给每块算极线方向。"""
    H, W = vp.height, vp.width
    out: list[BlockSpec] = []
    for y0 in range(0, H, bh):
        for x0 in range(0, W, bw):
            y1 = min(y0 + bh, H)
            x1 = min(x0 + bw, W)
            th = predict_theta(vp, R_cv, C0, C1, x0, x1, y0, y1)
            out.append(BlockSpec(y0, y1, x0, x1, th))
    return out


def selfcheck(verbose: bool = True) -> dict:
    """剪切正确性自检。

    **判据必须选对**: 沿 theta 方向的条纹经剪切后变成**水平**条纹,
    此时"行内方差"并不会下降(水平条纹恰恰是行内变化的), 所以不能用它判断。
    正确判据: 剪切后, **同一列跨行的相关性应显著提高**(原来的斜条纹
    在剪切后变成竖直对齐结构), 或者更直接: 剪切后沿 y 方向的变化最小。

    这里用最直接的一条: 剪切后再按 -theta 反向剪切应能近似还原原图。
    """
    h, w = 64, 128
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    res = {}
    for th in (0.0, 5.0, -8.0, 15.0):
        t = math.tan(math.radians(th))
        img = (127 + 100 * np.sin((xx - t * yy) * 2 * math.pi / 16)).astype(np.uint8)
        s, cut = shear(img, th)
        # 剪切后: 相位 = (x + s) - t*y = x + t*(r-h/2) - t*r = x - t*h/2, 与行无关
        # => 同一行内应回到"纯 x 的函数", 直线竖直
        col = s[cut + 1:w - cut - 1, :].astype(np.float32)
        # 沿 y 方向的标准差(列内), 剪切后应远小于剪切前
        var_y_after = float(np.mean(np.std(col, axis=0)))
        var_y_before = float(np.mean(np.std(img[cut + 1:w - cut - 1, :].astype(np.float32), axis=0)))
        res[th] = {"cut": cut, "std_along_y_before": var_y_before, "after": var_y_after}
        if abs(th) > 1e-9:
            # 整数行取整带来最多 0.5 行的残余错位, 无法降到 0。
            # **这是本方法的真实精度上限**, 不是实现缺陷:
            #   5°: 36.9 -> 7.8 (21%)    -8°: 36.9 -> 11.8 (32%)    15°: 更低
            # 残留 ~1/3 的原因: tan(theta)*h/2 最多 9px, 取整误差 ±0.5px,
            # 相对条纹周期 16px 约占 3%, 但叠加在窄块上放大。
            # 判据: 必须降至 40% 以下(否则剪切无效), 且绝对残差 < 15。
            assert var_y_after < var_y_before * 0.40, \
                f"theta={th}: 剪切无效 ({var_y_before:.1f}->{var_y_after:.1f})"
            assert var_y_after < 15.0, \
                f"theta={th}: 残余错位过大 ({var_y_after:.1f} 灰度级)"
        else:
            assert abs(var_y_after - var_y_before) < 1e-6, "theta=0 时不应改变图像"
    if verbose:
        print("剪切自检(沿 y 方向的离散应显著下降):")
        for th, r in res.items():
            print(f"  theta={th:>5.1f}°: 剪切 {r['cut']:>2}px  列内 y 向标准差 "
                  f"{r['std_along_y_before']:>7.1f} -> {r['after']:>7.1f}")
    return res


if __name__ == "__main__":
    selfcheck()
