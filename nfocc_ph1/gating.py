"""NFOCC 前向/侧向门控与基线状态机 (SPEC-BASELINE-SM 落地).

**核心裁决规则**
----------------
不按"直行/转弯"这种工况标签决策, 而按**平移方向相对相机光轴的角度**:

    侧向分量占比 sigma = |t_perp| / |t|        (t_perp = t 在垂直于光轴分量)

    sigma > SIGMA_STEREO_MIN  -> STEREO   用 SGBM(分块方向搜索)
    sigma < SIGMA_FORWARD_MAX -> FORWARD  用径向膨胀解析测距
    中间                      -> HYBRID  两者都跑, 按置信度融合

为什么不用固定阈值: 侧向平移的极线离散度随 sigma 单调下降(实测 90°->31°,
倾角 120x32 分块 -> 2.3°), 所以 sigma 是唯一的物理量。

冻结语义(SPEC-BASELINE-SM §4)
------------------------------
B < B_MIN          -> FROZEN_NEAR  积累不足, 输出上一帧
B > B_MAX          -> FROZEN_FAR   基线过大, 重投影误差超限
frame_gap > GAP_MAX -> FROZEN_GAP   丢帧, 基线/位姿不可信
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field

import numpy as np


class Mode(enum.Enum):
    STEREO = "STEREO"
    FORWARD = "FORWARD"
    HYBRID = "HYBRID"
    FROZEN_NEAR = "FROZEN_NEAR"
    FROZEN_FAR = "FROZEN_FAR"
    FROZEN_GAP = "FROZEN_GAP"

    @property
    def produces_depth(self) -> bool:
        return self in (Mode.STEREO, Mode.FORWARD, Mode.HYBRID)


@dataclass
class BaselineParams:
    b_min_mm: float = 12.0
    b_max_mm: float = 233.0
    b_target_mm: float = 120.0
    gap_max_frames: int = 2
    sigma_stereo_min: float = 0.80
    sigma_forward_max: float = 0.35


@dataclass
class ModeDecision:
    mode: Mode
    baseline_mm: float
    sigma: float              # |t_perp|/|t|
    epi_angle_deg: float      # 极线相对水平的角度
    reason: str


def lateral_ratio(t_cam: np.ndarray) -> float:
    """sigma = |t_perp|/|t|, t_perp 为垂直于光轴(z)的分量。"""
    t = np.asarray(t_cam, dtype=np.float64).reshape(3)
    nt = np.linalg.norm(t)
    if nt < 1e-9:
        return 0.0
    return float(math.hypot(t[0], t[1]) / nt)


def epi_angle_deg(t_cam: np.ndarray) -> float:
    """极线与图像横轴的夹角。见 nfocc_ph0.epipolar 的说明。"""
    t = np.asarray(t_cam, dtype=np.float64).reshape(3)
    perp = math.hypot(t[1], t[2])
    return float(math.degrees(math.atan2(perp, abs(t[0]))))


def decide(t_cam: np.ndarray, frame_gap: int = 1,
           p: BaselineParams | None = None) -> ModeDecision:
    p = p or BaselineParams()
    t = np.asarray(t_cam, dtype=np.float64).reshape(3)
    b = float(np.linalg.norm(t))
    sig = lateral_ratio(t)
    ang = epi_angle_deg(t)

    if frame_gap > p.gap_max_frames:
        return ModeDecision(Mode.FROZEN_GAP, b, sig, ang,
                            f"frame_gap={frame_gap} > {p.gap_max_frames}")
    if b < p.b_min_mm:
        return ModeDecision(Mode.FROZEN_NEAR, b, sig, ang,
                            f"B={b:.1f} < B_MIN={p.b_min_mm}")
    if b > p.b_max_mm:
        return ModeDecision(Mode.FROZEN_FAR, b, sig, ang,
                            f"B={b:.1f} > B_MAX={p.b_max_mm}")
    if sig >= p.sigma_stereo_min:
        return ModeDecision(Mode.STEREO, b, sig, ang,
                            f"sigma={sig:.2f} >= {p.sigma_stereo_min} (极线 {ang:.0f}°)")
    if sig <= p.sigma_forward_max:
        return ModeDecision(Mode.FORWARD, b, sig, ang,
                            f"sigma={sig:.2f} <= {p.sigma_forward_max} (前向)")
    return ModeDecision(Mode.HYBRID, b, sig, ang,
                        f"sigma={sig:.2f} 在中间带")


@dataclass
class BaselineState:
    """累积基线状态机。参考帧固定, 当前帧推进。"""
    params: BaselineParams = field(default_factory=BaselineParams)
    ref_pose: np.ndarray | None = None
    frame_gap: int = 0
    t_cam: np.ndarray = field(default_factory=lambda: np.zeros(3))
    decision: ModeDecision | None = None

    def set_reference(self, pose: np.ndarray, keep_decision: bool = False) -> None:
        self.ref_pose = np.asarray(pose, dtype=np.float64).reshape(3)
        self.frame_gap = 0
        self.t_cam = np.zeros(3)
        if not keep_decision:
            self.decision = None

    def update(self, pose_cam: np.ndarray) -> ModeDecision:
        """推进一帧。pose_cam 为相机在**当前帧相机系**下的位置增量累计。

        约定: 调用方传入的是"当前帧相对参考帧、表达在相机系"的平移 t_cam。
        """
        if self.ref_pose is None:
            self.set_reference(np.zeros(3))
        self.frame_gap += 1
        self.t_cam = np.asarray(pose_cam, dtype=np.float64).reshape(3)
        d = decide(self.t_cam, self.frame_gap, self.params)
        self.decision = d
        if d.mode.produces_depth:
            # 产出后重设参考帧。**保留本次决策**供调用方读取 ——
            # 早先版本在这里清空 decision, 导致 update() 返回 None(自检抓到)。
            self.set_reference(np.zeros(3), keep_decision=True)
        return d


def selfcheck(verbose: bool = True) -> dict:
    """门控规则的边界自检。"""
    p = BaselineParams()
    out = {}

    # 直行 120mm: 沿光轴 -> sigma=0 -> FORWARD
    d = decide(np.array([0.0, 0.0, 120.0]), 1, p)
    assert d.mode is Mode.FORWARD, d.mode
    out["forward"] = (d.mode.value, round(d.sigma, 3), round(d.epi_angle_deg, 1))

    # 侧移 120mm: t 沿 x -> sigma=1 -> STEREO
    d = decide(np.array([120.0, 0.0, 0.0]), 1, p)
    assert d.mode is Mode.STEREO, d.mode
    out["stereo"] = (d.mode.value, round(d.sigma, 3), round(d.epi_angle_deg, 1))

    # 45° 斜向: sigma=0.707 -> 中间带 HYBRID
    t = 120.0 * np.array([math.cos(math.radians(45)), 0.0, math.sin(math.radians(45))])
    d = decide(t, 1, p)
    assert d.mode is Mode.HYBRID, d.mode
    out["hybrid"] = (d.mode.value, round(d.sigma, 3))

    # 基线过小 / 过大 / 丢帧
    assert decide(np.array([0, 0, 5.0]), 1, p).mode is Mode.FROZEN_NEAR
    assert decide(np.array([0, 0, 300.0]), 1, p).mode is Mode.FROZEN_FAR
    assert decide(np.array([0, 0, 120.0]), 5, p).mode is Mode.FROZEN_GAP
    out["frozen"] = "NEAR/FAR/GAP 均正确触发"

    # 状态机: 产出后重设参考帧
    st = BaselineState()
    st.set_reference(np.zeros(3))
    d1 = st.update(np.array([0.0, 0.0, 120.0]))
    assert d1.mode is Mode.FORWARD
    assert st.frame_gap == 0, f"产出后应重置 gap, 实际 {st.frame_gap}"
    out["sm_reset"] = "OK"

    if verbose:
        for k, v in out.items():
            print(f"  {k}: {v}")
    return out


if __name__ == "__main__":
    print("基线门控自检:")
    selfcheck()
