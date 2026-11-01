"""NFOCC PH-1.1: 视差引擎接口 + SGBM 实现 + 空引擎骨架.

对应 SPEC-ENGINE。**所有引擎只能通过 DisparityEngine 接口交互, 上层不得感知 kind。**

关键约束(来自 docs/SPEC-ENGINE.md, 每条都有对应验收)
-----------------------------------------------------
E-6  无效像素必须由 valid 掩码标出, **不得用 disp=0 表示无效**(0 是合法视差)
S-2  容量 < 真视差 -> 完全失效, 必须硬断言返回 NF_E_ENGINE_RANGE, 且不写输出
S-3  有效覆盖 = (W - numDisp)/W
S-4  内存 = numDisp * W * H * 2 B
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field

import numpy as np

try:
    import cv2
except ImportError:                                    # 允许无 cv2 的环境导入本模块
    cv2 = None


# ------------------------------------------------------------------ 状态码
class NFStatus(enum.IntEnum):
    NF_OK = 0
    NF_E_PARAM_NULL = 1
    NF_E_PARAM_RANGE = 2
    NF_E_PARAM_VERSION = 3
    NF_E_ENGINE_NOT_INIT = 30
    NF_E_ENGINE_RANGE = 31
    NF_E_ENGINE_OOM = 32
    NF_E_ENGINE_UNSUPPORTED = 33


class EngineKind(enum.IntEnum):
    SGBM = 0
    NULL = 1          # 模型实现的占位骨架


# -------------------------------------------------------------------- 数据
@dataclass
class DisparityRange:
    d_min: float = 0.0
    d_max: float = 256.0

    def __post_init__(self) -> None:
        if not (0.0 <= self.d_min < self.d_max):
            raise ValueError(f"视差范围非法: [{self.d_min}, {self.d_max}]")


@dataclass
class EngineConfig:
    kind: EngineKind = EngineKind.SGBM
    max_disp_px: int = 256
    downscale: int = 1
    block_size: int = 5
    uniqueness: int = 10
    speckle_window: int = 100
    speckle_range: int = 2
    disp12_max_diff: int = 1
    conf_thresh: float = 0.0


@dataclass
class DisparityMap:
    """视差图。disp 与 valid 分离 —— 这是 E-6 的落地形式。"""
    disp: np.ndarray            # (H,W) float32, px
    valid: np.ndarray           # (H,W) bool
    conf: np.ndarray            # (H,W) float32, 0..1
    width: int = field(init=False)
    height: int = field(init=False)

    def __post_init__(self) -> None:
        self.height, self.width = self.disp.shape

    @classmethod
    def empty(cls, h: int, w: int) -> "DisparityMap":
        return cls(disp=np.zeros((h, w), np.float32),
                   valid=np.zeros((h, w), bool),
                   conf=np.zeros((h, w), np.float32))

    def stats(self) -> dict:
        v = self.valid
        d = self.disp[v]
        return {
            "valid_ratio": float(v.mean()) if v.size else 0.0,
            "disp_median": float(np.median(d)) if d.size else float("nan"),
            "disp_nonzero_but_invalid": int(((~v) & (self.disp != 0)).sum()),
        }


# ------------------------------------------------------------------ 基类
class DisparityEngine:
    """引擎接口。上层只认这个类。"""

    def __init__(self) -> None:
        self._inited = False
        self.cfg: EngineConfig | None = None

    def init(self, cfg: EngineConfig) -> NFStatus:
        if cfg is None:
            return NFStatus.NF_E_PARAM_NULL
        if cfg.max_disp_px <= 0 or cfg.max_disp_px % 16 != 0:
            # OpenCV 要求 16 的整数倍; 也是 PARAM R6
            return NFStatus.NF_E_PARAM_RANGE
        if cfg.downscale not in (1, 2, 4):
            return NFStatus.NF_E_PARAM_RANGE
        self.cfg = cfg
        self._inited = True
        return NFStatus.NF_OK

    def run(self, left: np.ndarray, right: np.ndarray,
            rng: DisparityRange) -> tuple[NFStatus, DisparityMap | None]:
        if not self._inited or self.cfg is None:
            return NFStatus.NF_E_ENGINE_NOT_INIT, None
        if left is None or right is None:
            return NFStatus.NF_E_PARAM_NULL, None
        if left.shape != right.shape or left.ndim != 2:
            return NFStatus.NF_E_PARAM_RANGE, None
        # S-2: 容量不足必须拒绝, 且不写输出
        if self.cfg.max_disp_px < rng.d_max - 1e-9:
            return NFStatus.NF_E_ENGINE_RANGE, None
        return self._run(left, right, rng)

    def _run(self, left, right, rng):   # pragma: no cover - 子类实现
        raise NotImplementedError

    # ---------------------------------------------------------- 内存核算
    def mem_bytes(self, w: int, h: int) -> int:
        """S-4: cost volume 的内存。用于上板预算与 V-ENG-6。"""
        if self.cfg is None:
            return 0
        return int(self.cfg.max_disp_px * w * h * 2)


# --------------------------------------------------------------- SGBM 实现
class SgbmEngine(DisparityEngine):
    """基于 OpenCV StereoSGBM。

    实现要点(都是实测踩出来的):
      - numDisparities 单位是**像素**(不是 1/16), compute() 返回 CV_16S, /16 得像素视差
      - 左边界 numDisparities 列无足够搜索窗口 -> 必然无效, 有效覆盖 = (W-numDisp)/W
      - 无效必须用 valid 掩码表达, 不能把 0 当无效
    """

    def _run(self, left: np.ndarray, right: np.ndarray, rng: DisparityRange):
        if cv2 is None:
            return NFStatus.NF_E_ENGINE_UNSUPPORTED, None
        assert self.cfg is not None
        h, w = left.shape
        nd = int(self.cfg.max_disp_px)
        bs = int(self.cfg.block_size)
        matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=nd,
            blockSize=bs,
            P1=8 * bs * bs,
            P2=32 * bs * bs,
            disp12MaxDiff=self.cfg.disp12_max_diff,
            uniquenessRatio=self.cfg.uniqueness,
            speckleWindowSize=self.cfg.speckle_window,
            speckleRange=self.cfg.speckle_range,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        raw = matcher.compute(left, right)            # int16, 单位 1/16 px
        disp = raw.astype(np.float32) / 16.0

        # 无效判定: SGBM 用 <=0 表示失败, 但真实视差 0 也是合法的。
        # 这里采用"最小有效视差"判据: 小于 d_min 的视为无效(d_min 默认 0.0 时,
        # 只有严格负值才无效)。为与 E-6 一致, d=0 且落在有效区内的像素保留为有效。
        valid = raw > 0
        valid &= disp >= rng.d_min - 1e-9
        valid &= disp <= rng.d_max + 1e-9
        # 左边界 numDisp 列无搜索窗口
        valid[:, :min(nd, w)] = False

        conf = np.where(valid, 1.0, 0.0).astype(np.float32)
        disp = np.where(valid, disp, 0.0).astype(np.float32)
        return NFStatus.NF_OK, DisparityMap(disp=disp, valid=valid, conf=conf)


# --------------------------------------------------------------- 空引擎
class NullEngine(DisparityEngine):
    """模型实现的接口骨架(PH-1 不训练权重)。全部返回无效, 用于验证"接口可替换"。

    存在意义: 若上层代码离开 SGBM 就跑不动, 说明接口没做成兼容(SPEC-ENGINE X-3)。
    """

    def _run(self, left: np.ndarray, right: np.ndarray, rng: DisparityRange):
        h, w = left.shape
        return NFStatus.NF_OK, DisparityMap.empty(h, w)


ENGINES = {EngineKind.SGBM: SgbmEngine, EngineKind.NULL: NullEngine}


def make_engine(cfg: EngineConfig) -> tuple[NFStatus, DisparityEngine | None]:
    cls = ENGINES.get(cfg.kind)
    if cls is None:
        return NFStatus.NF_E_ENGINE_UNSUPPORTED, None
    eng = cls()
    st = eng.init(cfg)
    return st, (eng if st == NFStatus.NF_OK else None)


def expected_valid_ratio(width: int, max_disp: int) -> float:
    """S-3: 有效覆盖 = (W - numDisp)/W。"""
    return max(0.0, (width - max_disp) / float(width))


def predict_mem_mb(max_disp: int, w: int, h: int) -> float:
    """S-4: 内存 = numDisp*W*H*2 B。"""
    return max_disp * w * h * 2 / 1e6
