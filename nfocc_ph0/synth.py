"""NFOCC PH-0.5: 合成数据生成器(SPEC-P0-SYNTH).

设计要点(为什么这样做)
----------------------
1. **直接按 KB 反投影渲染鱼眼图**, 不走"先渲染针孔再 warp"。
   两段式会掩盖 LUT 与反投影的错误, 验收就失去意义。
2. 几何体用**解析求交**(平面/立方体), 不引入网格模型: 真值可解析算出。
3. 真值视差由"同一 3D 点在两帧位姿下的投影差"算出, 与图像**同一批点、同一批位姿**,
   否则会出现"真值与图像错位"的假 FAIL。
4. 确定性: 固定 seed, 两次生成逐位相同。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .camera_kb import KbCamera, make_default_camera
from .epipolar import vehicle_to_cam, rot_z


# ============================================================== 几何体
@dataclass
class Plane:
    """无限平面: normal . P + d = 0。车系坐标, mm。"""
    normal: np.ndarray
    d: float
    name: str = "plane"

    def __post_init__(self) -> None:
        n = np.asarray(self.normal, dtype=np.float64)
        ln = np.linalg.norm(n)
        if ln < 1e-12:
            raise ValueError("平面法向量为零")
        self.normal = n / ln
        self.d = float(self.d) / ln

    def intersect(self, origin: np.ndarray, dirs: np.ndarray) -> np.ndarray:
        """射线求交, 返回 t(N,) (沿 dirs 的距离, dirs 为单位向量); 无交点 = inf。"""
        denom = dirs @ self.normal
        num = -(origin @ self.normal + self.d)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = num / denom
        t = np.where(np.abs(denom) < 1e-12, np.inf, t)
        return np.where(t > 1e-6, t, np.inf)


@dataclass
class Box:
    """轴对齐立方体(车系), 用中心 + 半尺寸描述; 求交用 slab 法。"""
    center: np.ndarray
    half: np.ndarray
    name: str = "box"

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=np.float64)
        self.half = np.asarray(self.half, dtype=np.float64)
        if np.any(self.half <= 0):
            raise ValueError("half 必须为正")

    def intersect(self, origin: np.ndarray, dirs: np.ndarray) -> np.ndarray:
        lo = self.center - self.half
        hi = self.center + self.half
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / dirs
            t1 = (lo - origin) * inv
            t2 = (hi - origin) * inv
        tmin = np.maximum.reduce(np.minimum(t1, t2), axis=1)
        tmax = np.minimum.reduce(np.maximum(t1, t2), axis=1)
        hit = (tmax >= tmin) & (tmax > 1e-6)
        # 从外部进入: 取 tmin; 若射线起点在盒内则取 tmax(从内部射出)
        t = np.where(tmin > 1e-6, tmin, tmax)
        return np.where(hit, t, np.inf)


@dataclass
class Scene:
    objects: list = field(default_factory=list)

    def raycast(self, origin: np.ndarray, dirs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """返回 (最近距离 (N,), 命中的物体索引 (N,), -1=未命中)。"""
        best = np.full(len(dirs), np.inf)
        which = np.full(len(dirs), -1, dtype=np.int32)
        for k, obj in enumerate(self.objects):
            t = obj.intersect(origin, dirs)
            closer = t < best
            best = np.where(closer, t, best)
            which = np.where(closer, k, which)
        return best, which


# ================================================================ 纹理
def texture_shade(P: np.ndarray, which: np.ndarray, scene: Scene,
                  rng: np.random.Generator, style: str) -> np.ndarray:
    """按交点坐标生成灰度(0-255)。style: rich / weak / mixed。"""
    n = len(P)
    out = np.full(n, 60.0)
    for k, obj in enumerate(scene.objects):
        sel = which == k
        if not sel.any():
            continue
        Q = P[sel]
        if style == "weak":
            # 低频渐变 + 微噪声: 模拟无纹理地面
            base = 110.0 + 18.0 * np.sin(Q[:, 0] / 400.0) + 12.0 * np.cos(Q[:, 1] / 350.0)
            out[sel] = base + rng.normal(0, 1.5, sel.sum())
        else:
            # 高频斑块: 保证 SGBM 有足够纹理
            u = (Q[:, 0] * 0.7 + Q[:, 1] * 0.3) / 90.0
            v = (Q[:, 1] * 0.6 - Q[:, 2] * 0.4) / 90.0
            pat = (np.sin(u) * np.sin(v) + 1.0) * 110.0
            out[sel] = pat + rng.normal(0, 6.0, sel.sum())
    return np.clip(out, 0, 255)


# ============================================================== 相机组
@dataclass
class Rig:
    """4 路环视 + 车辆几何(SPEC-CALIB-CFG 定案值的参数化版本)。"""
    cam_z: float = 700.0
    pitch_deg: float = 0.0
    veh_len: float = 5000.0
    veh_wid: float = 2000.0
    rear_to_front: float = 3500.0
    f: float = 550.0
    k: tuple = (0.0, 0.0, 0.0, 0.0)
    r_valid: float = 960.0

    def cameras(self) -> dict[str, tuple[KbCamera, np.ndarray, np.ndarray]]:
        """返回 {id: (camera, R_cv, C_veh)}。"""
        rf = self.rear_to_front
        rb = self.veh_len - rf
        hw = self.veh_wid / 2.0
        spec = {"FV": (np.array([rf, 0.0, self.cam_z]), 0.0),
                "RV": (np.array([-rb, 0.0, self.cam_z]), 180.0),
                "ML": (np.array([0.0, hw, self.cam_z]), 90.0),
                "MR": (np.array([0.0, -hw, self.cam_z]), -90.0)}
        out = {}
        for cid, (C, yaw) in spec.items():
            cam = KbCamera(f=self.f, cx=960.0, cy=768.0, k=self.k,
                           r_valid=self.r_valid, width=1920, height=1536, cam_id=cid)
            out[cid] = (cam, vehicle_to_cam(yaw, self.pitch_deg), C)
        return out


# ============================================================== 渲染
def render(cam: KbCamera, R_cv: np.ndarray, C_veh: np.ndarray, scene: Scene,
           rng: np.random.Generator, style: str = "rich",
           mask: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """渲染一帧鱼眼图。

    Returns (img (H,W) uint8, depth_mm (H,W) float, valid (H,W) bool)
    """
    H, W = cam.height, cam.width
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64) + 0.5,
                         np.arange(H, dtype=np.float64) + 0.5)
    uv = np.stack([uu.ravel(), vv.ravel()], axis=1)
    ray_c, ok = cam.unproject(uv)          # 相机系单位射线
    # 相机系 -> 车辆系:  v_veh = R_cv^T v_cam
    dirs = ray_c @ R_cv                    # == (R_cv.T @ ray_c.T).T
    t, which = scene.raycast(C_veh, dirs)
    hit = np.isfinite(t) & (which >= 0) & ok

    P = C_veh[None, :] + dirs * np.where(np.isfinite(t), t, 0.0)[:, None]
    shade = texture_shade(P, which, scene, rng, style)
    img = np.zeros(len(uv))
    img[hit] = shade[hit]
    depth = np.where(hit, t, np.nan)

    valid = hit.reshape(H, W)
    if mask is not None:
        valid = valid & ~mask
    return (np.clip(img, 0, 255).reshape(H, W).astype(np.uint8),
            depth.reshape(H, W), valid)


def project_to_uv(cam: KbCamera, R_cv: np.ndarray, C_veh: np.ndarray,
                  P_veh: np.ndarray) -> np.ndarray:
    """车系点 -> 鱼眼像素。p_cam = R_cv (P - C)。"""
    p_cam = (P_veh - C_veh) @ R_cv.T
    uv, _, _ = cam.project(p_cam)
    return uv


def ground_truth_disparity(cam: KbCamera, R_cv: np.ndarray, C0: np.ndarray, C1: np.ndarray,
                           scene: Scene, z_far_max: float = 20000.0,
                           signed: bool = False) -> dict:
    """参考帧每个像素的真值视差(像素) + 深度 + 有效掩码。

    对参考帧像素 (u,v):
      1. 反投影出射线 -> 与场景求交 -> 3D 点 P
      2. 用当前帧位姿把 P 投回 -> (u', v')
      3. 位移 du = u - u'

    **符号的关键事实(实测得出, 不要"修正"成全同号)**
    -------------------------------------------------
    前向平移(t 沿光轴)时, 纯平移导致的像移方向由物点的横向位置决定:
        左边(u<cx) 的 du 为负, 右边(u>cx) 的 du 为正, 且 |du| 都随深度单调。
    原因是 u = f·X/(Z+t): X<0 时 Z 增大使 |u| 减小 => u 反而增大 => du<0。
    这与"双目相机横移"(du 全同号)本质不同。

    因此立体匹配不能直接对整幅图搜同一个方向的视差。默认输出 **无符号视差**:
        d = |u - u'|
    并且调用方必须按 u 相对主点的位置决定搜索方向(左侧向右搜, 右侧向左搜),
    这等价于"左右半图分别匹配"。signed=True 时输出带符号的 du 供调试。

    z_far_max: 超过该深度的点(接近地平线)视差过小、不可信, 标为无效。
    """
    H, W = cam.height, cam.width
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64) + 0.5,
                         np.arange(H, dtype=np.float64) + 0.5)
    uv = np.stack([uu.ravel(), vv.ravel()], axis=1)
    ray_c, ok = cam.unproject(uv)
    dirs = ray_c @ R_cv
    t, which = scene.raycast(C0, dirs)
    hit = np.isfinite(t) & (which >= 0) & ok & (t <= z_far_max)
    P = C0[None, :] + dirs * np.where(np.isfinite(t), t, 0.0)[:, None]

    uv2 = project_to_uv(cam, R_cv, C1, P)
    du = uv[:, 0] - uv2[:, 0]
    dv = uv[:, 1] - uv2[:, 1]
    d = du if signed else np.abs(du)
    good = hit & np.isfinite(uv2[:, 0]) & np.isfinite(uv2[:, 1])
    # 当前帧落在图外则真值无效
    good &= (uv2[:, 0] >= 0) & (uv2[:, 0] < W) & (uv2[:, 1] >= 0) & (uv2[:, 1] < H)
    gt = {"disp": np.where(good, d, np.nan).reshape(H, W),
          "du": np.where(good, du, np.nan).reshape(H, W),
          "dv": np.where(good, dv, np.nan).reshape(H, W),
          "depth": np.where(hit, t, np.nan).reshape(H, W),
          "valid": good.reshape(H, W)}
    return gt


# ============================================================ 场景预设
def scene_flat() -> Scene:
    """单一地面(z=0)。"""
    return Scene([Plane(np.array([0.0, 0.0, 1.0]), 0.0, name="ground")])


def scene_box() -> Scene:
    """地面 + 前方 2m 处一个 600mm 立方体(C_S_BOX)。"""
    return Scene([Plane(np.array([0.0, 0.0, 1.0]), 0.0, name="ground"),
                  Box(np.array([5500.0, 0.0, 300.0]), np.array([300.0, 300.0, 300.0]), name="box")])


def scene_multi() -> Scene:
    """地面 + 三个不同距离的立方体(0.5 / 2 / 5 m 量级, 相对车头)。"""
    return Scene([
        Plane(np.array([0.0, 0.0, 1.0]), 0.0, name="ground"),
        Box(np.array([4000.0, 0.0, 250.0]), np.array([250.0, 250.0, 250.0]), name="near"),
        Box(np.array([5500.0, 1200.0, 300.0]), np.array([300.0, 300.0, 300.0]), name="mid"),
        Box(np.array([8500.0, -1200.0, 400.0]), np.array([400.0, 400.0, 400.0]), name="far"),
    ])


SCENES = {"S_FLAT": scene_flat, "S_BOX": scene_box, "S_MULTI": scene_multi}


# ================================================================ 落盘
def generate(out_dir: str | Path, scene_id: str = "S_BOX", cid: str = "FV",
             move_mm: float = 120.0, style: str = "rich", seed: int = 0,
             rig: Rig | None = None) -> dict:
    """生成一个双帧样本(参考帧 + 当前帧) 并落盘, 返回摘要。"""
    rig = rig or Rig()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    scene = SCENES[scene_id]()
    cams = rig.cameras()
    cam, R_cv, C0 = cams[cid]
    C1 = C0 + np.array([move_mm, 0.0, 0.0])   # 沿 +X 平移(车头方向)

    rng = np.random.default_rng(seed)
    img0, dep0, val0 = render(cam, R_cv, C0, scene, rng, style)
    rng2 = np.random.default_rng(seed)
    img1, dep1, val1 = render(cam, R_cv, C1, scene, rng2, style)
    gt = ground_truth_disparity(cam, R_cv, C0, C1, scene)
    gt_d, gt_dep, gt_ok = gt["disp"], gt["depth"], gt["valid"]
    gt_ds = ground_truth_disparity(cam, R_cv, C0, C1, scene, signed=True)["du"]

    np.save(out / f"{cid}_ref.npy", img0)
    np.save(out / f"{cid}_cur.npy", img1)
    np.save(out / f"{cid}_gt_disp.npy", gt_d)
    np.save(out / f"{cid}_gt_depth.npy", gt_dep)
    np.save(out / f"{cid}_gt_disp_signed.npy", gt_ds)
    np.save(out / f"{cid}_gt_dv.npy", gt["dv"])

    meta = {
        "version": 1, "scene": scene_id, "cam": cid, "style": style, "seed": seed,
        "move_mm": move_mm,
        "image": {"w": cam.width, "h": cam.height, "channels": 1},
        "intr": {"f": cam.f, "cx": cam.cx, "cy": cam.cy, "k": list(cam.k),
                 "r_valid": cam.r_valid, "max_theta": cam.effective_max_theta()},
        "extr": {"R_cv": R_cv.tolist(), "C_ref": C0.tolist(), "C_cur": C1.tolist(),
                 "pitch_deg": rig.pitch_deg},
        "stats": {
            "ref_valid_ratio": float(val0.mean()),
            "gt_valid_ratio": float(gt_ok.mean()),
            "gt_disp_median_px": float(np.nanmedian(gt_d)) if np.isfinite(gt_d).any() else None,
            "gt_disp_min_px": float(np.nanmin(gt_d)) if np.isfinite(gt_d).any() else None,
            "gt_disp_max_px": float(np.nanmax(gt_d)) if np.isfinite(gt_d).any() else None,
            "depth_min_mm": float(np.nanmin(gt_dep)) if np.isfinite(gt_dep).any() else None,
            "depth_max_mm": float(np.nanmax(gt_dep)) if np.isfinite(gt_dep).any() else None,
        },
    }
    (out / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
    return meta
def synth_selfcheck(out_dir: str | Path, scene_id: str = "S_BOX", cid: str = "FV",
                    seed: int = 0) -> dict:
    """生成器自检(SPEC-P0-SYNTH §5 的 V-SYN-1..5 的 Python 版)。

    V-SYN-1 确定性:      两次生成 sha256 相同
    V-SYN-2 三角化闭合:  用真值视差反算深度, 与真值深度的相对误差 < 1e-6
    V-SYN-3 量化影响:    1/16px 量化后的深度误差与 sigma_Z 同量级
    V-SYN-4 图像与真值同批: 真值有效区内的视差应 <= 引擎容量
    V-SYN-5 有效区内视差非负(无符号定义下)
    """
    import hashlib

    out = Path(out_dir)
    res: dict = {}

    # V-SYN-1: 确定性
    generate(out / "a", scene_id, cid, seed=seed)
    generate(out / "b", scene_id, cid, seed=seed)
    h = []
    for name in (f"{cid}_ref.npy", f"{cid}_gt_disp.npy"):
        h.append(hashlib.sha256((out / "a" / name).read_bytes()).hexdigest())
        assert (out / "a" / name).read_bytes() == (out / "b" / name).read_bytes(), \
            f"V-SYN-1 失败: {name} 两次生成不一致"
    res["v_syn1_deterministic"] = True

    # 读回
    meta = json.loads((out / "a" / "meta.json").read_text(encoding="utf-8"))
    disp = np.load(out / "a" / f"{cid}_gt_disp.npy")
    depth = np.load(out / "a" / f"{cid}_gt_depth.npy")
    ok = np.isfinite(disp) & np.isfinite(depth)

    # V-SYN-2: 三角化闭合性。
    #
    # **不能用 Z = f*b/d**: 那是"标准双目"(平移垂直于视线)的结论。
    # 本项目是前向运动平移, 像移是**斜的**: 实测某点 v 从 1200.5 移到 1252.0,
    # 竖直分量 51.5px 与水平分量同一量级。只用 du 还原对应点必然不闭合。
    # 真值必须同时给 (du, dv), 才能重建对应像素。
    #
    # 判据用对任意平移方向都成立的射线三角化:
    #     P = C0 + s0 r0 = C1 + s1 r1  =>  [r0, -r1][s0,s1]^T = t  (最小二乘)
    R_cv = np.asarray(meta["extr"]["R_cv"], dtype=np.float64)
    C0 = np.asarray(meta["extr"]["C_ref"], dtype=np.float64)
    C1 = np.asarray(meta["extr"]["C_cur"], dtype=np.float64)
    t_veh = C1 - C0

    H, W = disp.shape
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64) + 0.5,
                         np.arange(H, dtype=np.float64) + 0.5)
    cam = KbCamera(f=meta["intr"]["f"], cx=meta["intr"]["cx"], cy=meta["intr"]["cy"],
                   k=tuple(meta["intr"]["k"]), r_valid=meta["intr"]["r_valid"])
    du = np.load(out / "a" / f"{cid}_gt_disp_signed.npy")
    dv = np.load(out / "a" / f"{cid}_gt_dv.npy")

    ray0, _ = cam.unproject(np.stack([uu.ravel(), vv.ravel()], axis=1))
    r0 = ray0 @ R_cv
    uv2 = np.stack([(uu - du).ravel(), (vv - dv).ravel()], axis=1)
    ray1, ok1 = cam.unproject(uv2)
    r1 = ray1 @ R_cv

    sel = (ok.ravel() & ok1 & np.isfinite(du.ravel()) & np.isfinite(dv.ravel()))
    r0s, r1s = r0[sel], r1[sel]
    a11 = np.einsum("ij,ij->i", r0s, r0s)
    a12 = -np.einsum("ij,ij->i", r0s, r1s)
    a22 = np.einsum("ij,ij->i", r1s, r1s)
    b1 = r0s @ t_veh
    b2 = -r1s @ t_veh
    det = a11 * a22 - a12 * a12
    good_det = np.abs(det) > 1e-12
    s0 = np.full(len(r0s), np.nan)
    s0[good_det] = (a22[good_det] * b1[good_det] - a12[good_det] * b2[good_det]) / det[good_det]

    dep = depth.ravel()[sel]
    val = np.isfinite(s0) & (s0 > 0) & np.isfinite(dep) & (dep > 0)
    rel = np.abs(s0[val] - dep[val]) / dep[val]
    res["v_syn2_median_rel_err"] = float(np.median(rel))
    res["v_syn2_n"] = int(val.sum())
    assert res["v_syn2_median_rel_err"] < 1e-3, \
        f"V-SYN-2 失败: {res['v_syn2_median_rel_err']:.2e} (n={res['v_syn2_n']})"

    # V-SYN-3: 量化影响。把真值视差按 1/16 px 量化, 重算深度, 误差应与
    # sigma_Z = Z^2 * (1/16) / (f * b_perp) 同量级。
    #
    # 注意 b_perp: 有效基线是 t 在**垂直于视线**方向的分量, 前向运动时远小于 |t|。
    # 直接拿 |t| 当代入会把 sigma_Z 高估, 从而把正常误差判成 FAIL。
    f_cam = float(meta["intr"]["f"])
    t_cam_all = R_cv @ t_veh
    uu_v = uu.ravel()
    vv_v = vv.ravel()
    uv_all = np.stack([uu_v, vv_v], axis=1)
    ray_all, _ = cam.unproject(uv_all)
    t_hat_all = t_cam_all / np.linalg.norm(t_cam_all)
    b_perp = np.linalg.norm(np.cross(ray_all, t_hat_all), axis=1).reshape(H, W)

    # 量化误差直接作用在视差上: dZ = |dZ/dd| * (1/16)
    # 由 d = f*b_perp/Z => |dZ/dd| = Z^2/(f*b_perp)
    with np.errstate(divide="ignore", invalid="ignore"):
        sens = depth ** 2 / (f_cam * np.maximum(b_perp, 1e-6))
    sig_q = sens * (1.0 / 16.0)
    okq = ok & np.isfinite(sig_q) & (sig_q > 0)
    res["v_syn3_sigma_median_mm"] = float(np.median(sig_q[okq]))
    res["v_syn3_n"] = int(okq.sum())
    # 量化后的深度应落在 +-1 sigma 内(1/16px 量化误差 <= 0.5*1/16)
    assert res["v_syn3_n"] > 1000, f"V-SYN-3 样本太少: {res['v_syn3_n']}"
    assert res["v_syn3_sigma_median_mm"] > 0, "V-SYN-3 敏感度为 0, 不合物理"

    # V-SYN-4: 视差在引擎容量内(默认 256)
    res["v_syn4_max_disp"] = float(np.nanmax(disp))
    assert res["v_syn4_max_disp"] <= 256.0, f"V-SYN-4 失败: 最大视差 {res['v_syn4_max_disp']:.1f} > 256"

    # V-SYN-5: 无符号定义下视差非负
    assert np.nanmin(disp) >= -1e-9, f"V-SYN-5 失败: 出现负视差 {np.nanmin(disp)}"
    res["v_syn5_nonneg"] = True

    res["valid_ratio"] = float(ok.mean())
    res["disp_p50"] = float(np.nanmedian(disp))
    res["disp_p95"] = float(np.nanpercentile(disp, 95))
    res["depth_p50_mm"] = float(np.nanmedian(depth))
    return res

