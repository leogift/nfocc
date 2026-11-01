"""NFOCC 产品级端到端管线: 门控 -> (立体 | 前向测距) -> 3D 点 + sigma.

这是产品路径。与 nfocc_ph1.pipeline(实验路径)的区别:
  - 按**平移方向**门控, 不再假定只有前向
  - 前向用解析测距(不依赖 SGBM), 侧向用分块方向 SGBM
  - 输出统一为 NFPoint 数组 + sigma, 供体素融合
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from nfocc_ph0.camera_kb import KbCamera
from nfocc_ph0.synth import Rig, SCENES
from nfocc_ph0.view_lut import ViewParams

from .engine import (DisparityMap, DisparityRange, EngineConfig, EngineKind,
                     NFStatus, make_engine)
from .gating import BaselineParams, Mode, decide
from .pipeline import _project_scene_view


@dataclass
class NfPoint:
    xyz: np.ndarray       # (N,3) 车辆系 mm
    sigma: np.ndarray     # (N,) mm
    src: np.ndarray       # (N,) int8: 0=stereo 1=forward
    n_total: int = 0


@dataclass
class ProductResult:
    status: NFStatus
    mode: Mode | None = None
    points: NfPoint | None = None
    metrics: dict = field(default_factory=dict)


def _render_view(scene, cam: KbCamera, R_cv, C, vp: ViewParams):
    """把场景渲染到视口坐标系, 返回 (img, P, hit, dist)。

    **纹理必须是 3D 点的确定性函数**(无逐像素随机噪声), 否则同一物点在两帧
    会得到不同灰度, SGBM 完全匹配不上(实测: 真值位移 133px, SGBM 输出 3.6px)。
    """
    Hv, Wv = vp.height, vp.width
    uu, vv = np.meshgrid(np.arange(Wv) + 0.5, np.arange(Hv) + 0.5)
    dirs = vp.pixel_to_ray(uu, vv) @ R_cv
    flat = dirs.reshape(-1, 3)
    t, which = scene.raycast(C, flat)
    hit = (np.isfinite(t) & (which >= 0)).reshape(Hv, Wv)
    P = C[None, None, :] + dirs * np.where(np.isfinite(t), t, 0.0).reshape(Hv, Wv, 1)

    img = deterministic_texture(P.reshape(-1, 3), which).reshape(Hv, Wv)
    img = np.where(hit, img, 0).astype(np.uint8)
    return img, P, hit, t.reshape(Hv, Wv)


def deterministic_texture(P: np.ndarray, which: np.ndarray) -> np.ndarray:
    """由 3D 坐标确定性地生成纹理(无随机项)。

    用多频正弦叠加: 空间频率足够高以保证 SGBM 有纹理, 又完全可复现,
    同一 3D 点在任意帧、任意视角下灰度相同 —— 这是跨帧匹配成立的前提。
    """
    P = np.asarray(P, dtype=np.float64).reshape(-1, 3)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    base = (55.0 * np.sin(x / 37.0) * np.cos(y / 41.0)
            + 45.0 * np.sin((x + y) / 23.0)
            + 35.0 * np.cos(z / 29.0)
            + 30.0 * np.sin((x - y + z) / 17.0)
            + 128.0)
    return np.clip(base, 0, 255)


def run_product(scene_id: str = "S_MULTI", cid: str = "FV",
                t_veh: np.ndarray | None = None,
                bparams: BaselineParams | None = None,
                engine_kind: EngineKind = EngineKind.SGBM,
                max_disp: int = 256, view_w: int = 960,
                sigma_d_forward: float = 0.5, sigma_d_stereo: float = 0.5,
                use_blocks: bool = False, block_w: int = 320,
                block_h: int = 64) -> ProductResult:
    """跑一帧产品管线。

    Args:
        t_veh: 车辆系平移(mm), 默认纯前向 120mm
        sigma_d_forward: 前向测距的对应点精度(px), 决定深度精度
        sigma_d_stereo : 立体匹配精度(px)
    """
    rig = Rig()
    cams = rig.cameras()
    cam, R_cv, C0 = cams[cid]
    if t_veh is None:
        t_veh = np.array([120.0, 0.0, 0.0])
    t_veh = np.asarray(t_veh, dtype=np.float64)
    C1 = C0 + t_veh
    t_cam = R_cv @ t_veh

    bp = bparams or BaselineParams()
    dec = decide(t_cam, 1, bp)

    scene = SCENES[scene_id]()
    vp = ViewParams(width=view_w, height=-1, hfov_deg=120,
                    el_min_deg=10.0, el_max_deg=60.0)

    img0, P0, hit0, dist0 = _render_view(scene, cam, R_cv, C0, vp)
    img1, P1, hit1, dist1 = _render_view(scene, cam, R_cv, C1, vp)

    if not dec.mode.produces_depth:
        return ProductResult(NFStatus.NF_OK, dec.mode, None,
                             {"reason": dec.reason, "baseline_mm": dec.baseline_mm})

    if dec.mode is Mode.FORWARD:
        return _run_forward(dec, vp, R_cv, C0, C1, t_cam, img0, img1,
                            hit0, dist0, sigma_d_forward, scene_id)

    # STEREO / HYBRID 走立体(此版本 HYBRID 先按立体处理)
    return _run_stereo(dec, vp, R_cv, C0, C1, t_cam, img0, img1,
                       hit0, dist0, engine_kind, max_disp, sigma_d_stereo,
                       use_blocks, block_w, block_h)


def _run_forward(dec, vp, R_cv, C0, C1, t_cam, img0, img1, hit0, dist0,
                 sigma_d, scene_id) -> ProductResult:
    """前向: 径向膨胀解析测距。

    **对应点来源**: 本函数用几何真值作为对应点, 验证的是**测距链路**本身。
    真实系统里对应点须由光流/特征给出(下一阶段), 其精度由 sigma_d 参数表达,
    并直接线性影响深度误差(实测 1px -> 110mm)。
    """
    from .forward_range import forward_range_from_rays
    from .triangulate import effective_baseline, sigma_depth

    scene = SCENES[scene_id]()
    Hv, Wv = vp.height, vp.width
    uu, vv = np.meshgrid(np.arange(Wv) + 0.5, np.arange(Hv) + 0.5)
    u_hat = vp.pixel_to_ray(uu, vv).reshape(-1, 3)

    # 参考帧方向 -> 与场景求交 -> 3D 点 -> 投到当前帧 -> 对应方向
    dirs = u_hat @ R_cv
    t_hit, which = scene.raycast(C0, dirs)
    hit = (np.isfinite(t_hit) & (which >= 0)).reshape(Hv, Wv)
    P = (C0[None, :] + dirs * np.where(np.isfinite(t_hit), t_hit, 0.0)[:, None])
    uv1 = vp.ray_to_pixel((P - C1) @ R_cv.T)
    v_hat = vp.pixel_to_ray(uv1[..., 0], uv1[..., 1])

    r = forward_range_from_rays(u_hat, v_hat, t_cam)
    ok = hit.reshape(-1) & r.valid
    dist = r.dist_mm[ok]
    pts_veh = (u_hat[ok] * dist[:, None]) @ R_cv + C0

    bp = effective_baseline(u_hat[ok], t_cam)
    sig = sigma_depth(dist, vp.f_v, bp, sigma_d, sigma_b_ratio=0.02)

    return ProductResult(
        NFStatus.NF_OK, Mode.FORWARD,
        NfPoint(pts_veh, sig, np.ones(int(ok.sum()), np.int8), int(ok.sum())),
        {"mode": "FORWARD", "sigma_lateral": dec.sigma, "epi_angle": dec.epi_angle_deg,
         "n_points": int(ok.sum()), "baseline_mm": dec.baseline_mm,
         "dist_median_mm": float(np.median(dist)) if len(dist) else float("nan"),
         "sigma_median_mm": float(np.median(sig)) if len(sig) else float("nan")},
    )


def _run_stereo(dec, vp, R_cv, C0, C1, t_cam, img0, img1, hit0, dist0,
                engine_kind, max_disp, sigma_d,
                use_blocks: bool = False, block_w: int = 320,
                block_h: int = 64) -> ProductResult:
    """侧向平移的立体匹配。

    **配对方向很重要**: SGBM 假定 right(x) = left(x - d), 即 d>=0。
    实测侧移 120mm 时 du 中位 −80px(对应点在右边), 直接喂进去 SGBM 只搜到 4.4px
    (看似成功实则全错)。所以先按 t 的横向符号决定是否镜像:
        t 使图像向左移(du>0) -> 原顺序
        t 使图像向右移(du<0) -> 两图都水平镜像后再匹配, 结果不变(位移取绝对值)
    """
    from .triangulate import effective_baseline, sigma_depth, triangulate_rays

    cfg = EngineConfig(kind=engine_kind, max_disp_px=max_disp)
    st, eng = make_engine(cfg)
    if st != NFStatus.NF_OK or eng is None:
        return ProductResult(st, dec.mode)

    rng = DisparityRange(0.0, float(max_disp))
    # t 在相机系 x 轴的符号决定图像平移方向: t.x>0(向右移) 使 u 减小 -> du<0
    mirror = float(t_cam[0]) > 0.0
    L = np.ascontiguousarray(img0[:, ::-1] if mirror else img0)
    R = np.ascontiguousarray(img1[:, ::-1] if mirror else img1)

    if use_blocks:
        from .blockmatch import make_blocks, match_blocks
        blocks = make_blocks(vp, R_cv, C0, C1, bw=block_w, bh=block_h)
        st2, dm = match_blocks(eng, L, R, blocks, rng)
    else:
        st2, dm = eng.run(L, R, rng)
    if st2 != NFStatus.NF_OK or dm is None:
        return ProductResult(st2, dec.mode)
    if mirror:
        dm = DisparityMap(disp=dm.disp[:, ::-1].copy(),
                          valid=dm.valid[:, ::-1].copy(),
                          conf=dm.conf[:, ::-1].copy())

    dm.valid &= hit0
    ys, xs = np.nonzero(dm.valid)
    if len(ys) == 0:
        return ProductResult(NFStatus.NF_OK, dec.mode,
                             NfPoint(np.zeros((0, 3)), np.zeros(0), np.zeros(0, np.int8)),
                             {"n_points": 0, "mode": dec.mode.value})
    u = xs + 0.5
    v = ys + 0.5
    d = dm.disp[ys, xs].astype(np.float64)
    r0 = vp.pixel_to_ray(u, v) @ R_cv
    # 对应点位置: SGBM 的 d 是"左图该像素在右图中的水平偏移量"。
    # 实测本项目(x 轴指向图像右侧、t_cam.x<0 时对应点在 +u 方向):
    #   u2 = u + d  -> s0 100% 为正, 中位 1446mm(合理)
    #   u2 = u - d  -> s0 100% 为负(物理上不可能)
    # 方向由 t_cam.x 的符号决定: t_cam.x<0 时 u2 = u + d, 否则 u2 = u - d。
    sign = 1.0 if float(t_cam[0]) < 0.0 else -1.0
    u2 = u + sign * d
    r1 = vp.pixel_to_ray(u2, v) @ R_cv
    s0, _ = triangulate_rays(C0, r0, C1, r1)
    good = np.isfinite(s0) & (s0 > 0)
    s0 = s0[good]
    pts = C0[None, :] + r0[good] * s0[:, None]
    bp = effective_baseline(r0[good], t_cam)
    sig = sigma_depth(s0, vp.f_v, bp, sigma_d, 0.02)
    return ProductResult(
        NFStatus.NF_OK, dec.mode,
        NfPoint(pts, sig, np.zeros(len(s0), np.int8), int(good.sum())),
        {"mode": dec.mode.value, "sigma_lateral": dec.sigma,
         "epi_angle": dec.epi_angle_deg, "n_points": int(good.sum()),
         "baseline_mm": dec.baseline_mm, "mirrored": mirror,
         "use_blocks": use_blocks,
         "disp_median_px": float(np.median(d)) if len(d) else float("nan"),
         "dist_median_mm": float(np.median(s0)) if len(s0) else float("nan"),
         "sigma_median_mm": float(np.median(sig)) if len(sig) else float("nan")},
    )
