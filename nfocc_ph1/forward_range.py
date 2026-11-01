"""NFOCC 前向运动测距(径向膨胀解析解).

**为什么需要它**
----------------
PH-1 实测: 纯前向平移时极线从 FoE 辐射(同一行内方向差 175.9°),
"按行单向搜索"的匹配器(含 SGBM)结构上失效, 调参无效。
但前向平移有另一条路: 物点两帧间只做**径向**移动, 由对应点可**解析**求距离,
不要求极线平行, 也不要求单向搜索。

推导(相机/视口系, 平移 t)
-------------------------
    q0 = P - c0 = Z * u_hat        (Z = |q0|, u_hat 为单位方向)
    q1 = P - c1 = q0 - t
    q1 与观测方向 v_hat 应同向:  q1 × v_hat = 0
=>  Z*(u_hat × v_hat) = t × v_hat        (三向量共面)
对每个分量取等号后最小二乘解 Z。

**两个已踩过的坑(不要重犯)**
1. Z 是**距离 |q0|**, 不是光轴深度 q0.z。混用解出 393 而真值 921。
2. 输入必须是**方向向量**, 不能是像素。视口的像素 v 定义成"相对环带中轴的仰角",
   与标准针孔 v = f*y/z 相差 208px(实测), 直接当针孔用会引入系统误差。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ForwardRangeResult:
    dist_mm: np.ndarray
    valid: np.ndarray
    cond: np.ndarray
    t_view: np.ndarray


def forward_range_from_rays(u_hat: np.ndarray, v_hat: np.ndarray,
                            t_view: np.ndarray,
                            min_sin: float = 1e-4) -> ForwardRangeResult:
    """由参考帧/当前帧的**单位方向向量**解参考帧距离 |q0|。

    Args:
        u_hat : (N,3) 参考帧射线(相机/视口系单位向量)
        v_hat : (N,3) 当前帧对应点射线(同一坐标系)
        t_view: (3,)   平移(相机系, mm)。c1 = c0 + t, 其中 c0 在原点。

    解:  Z*(u × v) = t × v
        => 最小二乘 Z = sum((u×v)·(t×v)) / sum|u×v|^2
    """
    u = np.asarray(u_hat, dtype=np.float64).reshape(-1, 3)
    v = np.asarray(v_hat, dtype=np.float64).reshape(-1, 3)
    # 归一化(容忍调用方传入未单位化的方向)
    u = u / np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-12)
    v = v / np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)
    t = np.asarray(t_view, dtype=np.float64).reshape(3)

    uv_cross = np.cross(u, v)                       # (N,3)
    tv_cross = np.cross(t[None, :], v)              # (N,3)
    num = np.einsum("ij,ij->i", uv_cross, tv_cross)
    den = np.einsum("ij,ij->i", uv_cross, uv_cross)
    with np.errstate(divide="ignore", invalid="ignore"):
        z = num / den
    valid = np.isfinite(z) & (z > 0) & (den > min_sin * min_sin)
    return ForwardRangeResult(z, valid, np.sqrt(np.maximum(den, 0.0)), t)


def selfcheck(verbose: bool = True) -> dict:
    """自校验: 构造真值点, 验证公式精确性与灵敏度。"""
    t = np.array([0.0, 0.0, 120.0])
    rng = np.random.default_rng(0)
    Z = rng.uniform(500, 4000, 3000)
    u = rng.uniform(-1.2, 1.2, 3000)
    v = rng.uniform(-0.2, 1.6, 3000)
    n = np.sqrt(u * u + v * v + 1.0)
    u_hat = np.stack([u / n, v / n, 1.0 / n], axis=1)
    q0 = Z[:, None] * u_hat
    q1 = q0 - t
    v_hat = q1 / np.linalg.norm(q1, axis=1, keepdims=True)

    r = forward_range_from_rays(u_hat, v_hat, t)
    rel = np.abs(r.dist_mm[r.valid] - Z[r.valid]) / Z[r.valid]
    out = {"n": int(r.valid.sum()),
           "max_rel_err": float(np.max(rel)) if r.valid.any() else float("nan")}
    assert out["max_rel_err"] < 1e-9, f"自校验失败 {out['max_rel_err']:.2e}"

    # 灵敏度: 对应点角度误差 -> 距离误差
    for ang_px, f_px in ((1.0, 277.0), (0.5, 277.0)):
        th = ang_px / f_px
        # 把 v_hat 绕某个垂直轴扰动 th
        axis = np.cross(v_hat, rng.normal(size=v_hat.shape))
        axis /= np.maximum(np.linalg.norm(axis, axis=1, keepdims=True), 1e-12)
        v2 = (v_hat * np.cos(th)
              + np.cross(axis, v_hat) * np.sin(th))
        r2 = forward_range_from_rays(u_hat, v2, t)
        e = np.abs(r2.dist_mm - Z)
        ok = r2.valid & np.isfinite(e)
        out[f"dz_median_{ang_px}px"] = float(np.median(e[ok]))
        out[f"dz_p90_{ang_px}px"] = float(np.percentile(e[ok], 90))

    if verbose:
        print(f"前向测距自校验: n={out['n']} 最大相对误差 {out['max_rel_err']:.2e} (应<1e-9)")
        for k in ("dz_median_1.0px", "dz_p90_1.0px", "dz_median_0.5px", "dz_p90_0.5px"):
            if k in out:
                print(f"  {k}: {out[k]:.0f} mm")
    return out


if __name__ == "__main__":
    selfcheck()
