"""NFOCC PH-1 端到端流水线: 合成场景 -> 引擎 -> 三角化 -> 指标.

这一层是"功能真的能用"的证据链。selftest 与 demo 都走同一条路径,
避免"自检通过但 demo 跑不通"的分裂。

流水线
------
    nfocc_ph0.synth 生成两帧鱼眼图 + 真值
      -> 视口 LUT 展开一帧到视口图          PH-0.2
      -> 用第二帧做同样展开, 得到视口对
      -> 引擎输出视差                        PH-1.1
      -> 射线三角化 + sigma                  PH-1.2
      -> 与真值比对, 产出指标
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from nfocc_ph0.camera_kb import KbCamera
from nfocc_ph0.epipolar import vehicle_to_cam
from nfocc_ph0.synth import SCENES, Rig
from nfocc_ph0.view_lut import ViewParams, build_view_lut, warp

from .engine import (DisparityMap, DisparityRange, EngineConfig, EngineKind,
                     NFStatus, make_engine)
from .triangulate import triangulate_disparity


def run_disparity_split_half(eng, left: np.ndarray, right: np.ndarray,
                             rng: DisparityRange) -> tuple[NFStatus, DisparityMap | None]:
    """前向平移专用: 把左右半图分别匹配, 各自构成单向视差场。

    **为什么必须分半**
    ------------------
    SGBM 等单向搜索匹配器假设 right(x,y) = left(x-d, y), 即 d >= 0。
    但前向(沿光轴)平移时, 纯平移产生的像移方向随像素位置翻转(实测):
        u < cx : du = +38 px   (对应点在左边)
        u > cx : du = -38 px   (对应点在右边)
    整幅图一起喂进去, 只有一半能被匹配, 另一半必然出错。

    分半后: 左半图 d=+du 直接可用; 右半图把两图水平镜像再匹配,
    得到的 d 即为 |du|, 结果再镜像回来。

    这等价于"向左搜"与"向右搜"两个独立匹配器, 是前向平移的正确做法。
    """
    h, w = left.shape
    cx = w // 2
    # --- 左半: 直接用 ---
    l_left = left[:, :cx]
    l_right = right[:, :cx]
    st, dm_l = eng.run(np.ascontiguousarray(l_left), np.ascontiguousarray(l_right), rng)
    if st != NFStatus.NF_OK or dm_l is None:
        return st, None
    # 右半图的有效范围受左边界限制: 在镜像空间里, 边界在镜像图的左侧
    out_disp = np.zeros((h, w), np.float32)
    out_valid = np.zeros((h, w), bool)
    out_conf = np.zeros((h, w), np.float32)
    out_disp[:, :cx] = dm_l.disp
    out_valid[:, :cx] = dm_l.valid
    out_conf[:, :cx] = dm_l.conf

    # --- 右半: 水平镜像后匹配 ---
    r_left = np.ascontiguousarray(left[:, cx:][:, ::-1])
    r_right = np.ascontiguousarray(right[:, cx:][:, ::-1])
    st, dm_r = eng.run(r_left, r_right, rng)
    if st != NFStatus.NF_OK or dm_r is None:
        return st, None
    # 镜像回来: 结果列序翻转
    out_disp[:, cx:] = dm_r.disp[:, ::-1]
    out_valid[:, cx:] = dm_r.valid[:, ::-1]
    out_conf[:, cx:] = dm_r.conf[:, ::-1]

    return NFStatus.NF_OK, DisparityMap(disp=out_disp, valid=out_valid, conf=out_conf)


@dataclass
class PipelineResult:
    status: NFStatus
    metrics: dict = field(default_factory=dict)
    artifacts: dict = field(default_factory=dict)


def _project_scene_view(cam: KbCamera, R_cv: np.ndarray, C: np.ndarray,
                        scene, vp: ViewParams, cam_z: float,
                        z_far_max: float = 20000.0):
    """把场景直接渲染到**视口**坐标系, 供配对与真值使用。

    注意: 不能"先展开鱼眼图再当视口图"来算真值——那样真值会被重采样污染。
    真值必须由几何直接给出。
    """
    Hv, Wv = vp.height, vp.width
    uu, vv = np.meshgrid(np.arange(Wv, dtype=np.float64) + 0.5,
                         np.arange(Hv, dtype=np.float64) + 0.5)
    ray_P = vp.pixel_to_ray(uu, vv)                 # 视口系 (Hv,Wv,3)
    dirs = ray_P @ R_cv                             # 视口轴与相机轴同向
    flat = dirs.reshape(-1, 3)
    t, which = scene.raycast(C, flat)
    t = t.reshape(Hv, Wv)
    which = which.reshape(Hv, Wv)
    hit = np.isfinite(t) & (which >= 0) & (t <= z_far_max)
    P = C[None, None, :] + dirs * np.where(np.isfinite(t), t, 0.0)[..., None]
    return P, hit, t


def run(scene_id: str = "S_BOX", cid: str = "FV", move_mm: float = 120.0,
        engine_kind: EngineKind = EngineKind.SGBM, max_disp: int = 256,
        downscale: int = 2, view_w: int = 960, sigma_d: float = 0.25,
        seed: int = 0, style: str = "rich") -> PipelineResult:
    rig = Rig()
    cams = rig.cameras()
    cam, R_cv, C0 = cams[cid]
    C1 = C0 + np.array([move_mm, 0.0, 0.0])
    scene = SCENES[scene_id]()

    # ---- 视口(高度由环带算) ----
    vp_full = ViewParams(width=view_w, height=-1, hfov_deg=120.0,
                         el_min_deg=10.0, el_max_deg=60.0)
    if downscale > 1:
        vp = ViewParams(width=max(16, vp_full.width // downscale),
                        height=-1, hfov_deg=120.0,
                        el_min_deg=10.0, el_max_deg=60.0)
    else:
        vp = vp_full

    # ---- 展开两帧 ----
    lut0 = build_view_lut(cam, vp)
    img0, _ = _render(cam, R_cv, C0, scene, seed, style)
    img1, _ = _render(cam, R_cv, C1, scene, seed, style)
    view0, v0 = warp(img0, lut0)
    view1, v1 = warp(img1, lut0)

    # ---- 引擎 ----
    cfg = EngineConfig(kind=engine_kind, max_disp_px=max_disp, downscale=downscale)
    st, eng = make_engine(cfg)
    if st != NFStatus.NF_OK or eng is None:
        return PipelineResult(status=st)
    rng = DisparityRange(d_min=0.0, d_max=float(max_disp))
    left = np.clip(view0, 0, 255).astype(np.uint8)
    right = np.clip(view1, 0, 255).astype(np.uint8)
    st, dm = run_disparity_split_half(eng, left, right, rng)
    if st != NFStatus.NF_OK or dm is None:
        return PipelineResult(status=st)
    dm.valid &= v0

    # ---- 真值(直接从几何生成到视口坐标系) ----
    P_gt, hit_gt, depth_gt = _project_scene_view(cam, R_cv, C0, scene, vp, rig.cam_z)
    uv0 = vp.ray_to_pixel((P_gt - C0) @ R_cv.T)
    uv1 = vp.ray_to_pixel((P_gt - C1) @ R_cv.T)
    du = uv0[..., 0] - uv1[..., 0]
    gt_disp = np.abs(du)
    gt_valid = hit_gt & np.isfinite(uv0[..., 0]) & np.isfinite(uv1[..., 0])

    # ---- 指标 ----
    both = dm.valid & gt_valid
    if both.sum() > 0:
        err = np.abs(dm.disp[both] - gt_disp[both])
        med_err = float(np.median(err))
        p90_err = float(np.percentile(err, 90))
    else:
        med_err = p90_err = float("nan")

    # ---- 三角化 ----
    tri = triangulate_disparity(dm, cam.unproject, R_cv, C0, C1,
                                f_px=vp.f_v, sigma_d_px=sigma_d)
    # 与真值深度比较(同一批有效像素)
    metrics = {
        "disp_median_abs_err_px": med_err,
        "disp_p90_abs_err_px": p90_err,
        "valid_ratio": float(dm.valid.mean()),
        "engine_valid_ratio": float(dm.stats()["valid_ratio"]),
        "gt_valid_ratio": float(gt_valid.mean()),
        "both_ratio": float(both.mean()),
        "n_points": tri.n_out,
        "sigma_median_mm": float(np.median(tri.sigma_mm)) if tri.n_out else float("nan"),
        "view_size": (vp.width, vp.height),
        "f_view": vp.f_v,
    }

    arts = {
        "ref": img0,
        "cur": img1,
        "view_ref": view0,
        "view_cur": view1,
        "disp_sgbm": dm.disp,
        "disp_gt": gt_disp,
        "err": np.where(both, np.abs(dm.disp - gt_disp), 0.0),
        "valid": dm.valid,
        "gt_valid": gt_valid,
        "points": tri.points_veh,
        "depth": tri.depth_mm,
        "sigma": tri.sigma_mm,
    }
    return PipelineResult(status=NFStatus.NF_OK, metrics=metrics, artifacts=arts)


def _render(cam: KbCamera, R_cv: np.ndarray, C: np.ndarray, scene,
            seed: int, style: str):
    from nfocc_ph0.synth import render
    rng = np.random.default_rng(seed)
    return render(cam, R_cv, C, scene, rng, style)[0:2]
