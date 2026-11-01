"""NFOCC PH-0.4: 体素索引 <-> 坐标双向映射 + Hash 打包(SPEC-COORD §5, SPEC-VOXEL).

约定(唯一, 不可另立)
--------------------
    ix = floor((X_max - X_V) / vox)      # X 反向: 靠近 X_max 的体素 ix=0
    iy = floor((Y_max - Y_V) / vox)      # Y 反向
    iz = floor((Z_V - Z_min) / vox)      # Z 正向

体素中心:
    X_c = X_max - (ix + 0.5) * vox
    Y_c = Y_max - (iy + 0.5) * vox
    Z_c = Z_min + (iz + 0.5) * vox

反向的原因: 与原始 NFOCC 的 Hash 拼接位序一致(高位纵向)。反向不一致时,
多帧融合会把同一物理点放进不同体素, 且不易察觉。

Z 方向常见的坑: Z 范围 3500mm / 200mm = 17.5, 必须**向上取整**为 18,
最后一格只有半个体素 —— 这半格必须被显式标记, 不能装作满格。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VoxelGrid:
    vox_mm: int = 200
    x_min: int = -5000
    x_max: int = 10000
    y_min: int = -6000
    y_max: int = 6000
    z_min: int = -1000
    z_max: int = 2500

    def __post_init__(self) -> None:
        if self.vox_mm <= 0:
            raise ValueError("vox_mm 必须 > 0")
        for lo, hi, name in ((self.x_min, self.x_max, "x"),
                             (self.y_min, self.y_max, "y"),
                             (self.z_min, self.z_max, "z")):
            if hi <= lo:
                raise ValueError(f"{name} 范围非法: {lo}..{hi}")

    # ------------------------------------------------------------------ 维度
    @property
    def nx(self) -> int:
        return int(math.ceil((self.x_max - self.x_min) / self.vox_mm))

    @property
    def ny(self) -> int:
        return int(math.ceil((self.y_max - self.y_min) / self.vox_mm))

    @property
    def nz(self) -> int:
        span = self.z_max - self.z_min
        n = int(math.ceil(span / self.vox_mm))
        return n

    def last_cell_is_partial(self) -> dict[str, bool]:
        """哪些轴的最后一格不足一个完整体素。"""
        return {
            "x": (self.x_max - self.x_min) % self.vox_mm != 0,
            "y": (self.y_max - self.y_min) % self.vox_mm != 0,
            "z": (self.z_max - self.z_min) % self.vox_mm != 0,
        }

    def last_cell_span(self) -> dict[str, int]:
        """最后一格的实际厚度(mm)。满格 == vox_mm。"""
        return {
            "x": (self.x_max - self.x_min) - (self.nx - 1) * self.vox_mm,
            "y": (self.y_max - self.y_min) - (self.ny - 1) * self.vox_mm,
            "z": (self.z_max - self.z_min) - (self.nz - 1) * self.vox_mm,
        }

    # ------------------------------------------------------- 点 -> 索引(批量)
    def point_to_index(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """pts (N,3) mm -> (idx (N,3) int32, inside (N,) bool)。

        越界点不进网格: inside=False。**不做钳位** —— 钳位会把远处点堆到边界上,
        在体素图边缘造出一堵假墙。
        """
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        ix = np.floor((self.x_max - p[:, 0]) / self.vox_mm)
        iy = np.floor((self.y_max - p[:, 1]) / self.vox_mm)
        iz = np.floor((p[:, 2] - self.z_min) / self.vox_mm)
        idx = np.stack([ix, iy, iz], axis=1).astype(np.int64)
        inside = (
            (idx[:, 0] >= 0) & (idx[:, 0] < self.nx)
            & (idx[:, 1] >= 0) & (idx[:, 1] < self.ny)
            & (idx[:, 2] >= 0) & (idx[:, 2] < self.nz)
        )
        return idx, inside

    # ------------------------------------------------------ 索引 -> 中心(批量)
    def index_to_center(self, idx: np.ndarray) -> np.ndarray:
        i = np.asarray(idx, dtype=np.float64).reshape(-1, 3)
        X = self.x_max - (i[:, 0] + 0.5) * self.vox_mm
        Y = self.y_max - (i[:, 1] + 0.5) * self.vox_mm
        Z = self.z_min + (i[:, 2] + 0.5) * self.vox_mm
        return np.stack([X, Y, Z], axis=1)

    # --------------------------------------------------------------- Hash
    def pack_hash(self, idx: np.ndarray) -> np.ndarray:
        """(ix<<16)|(iy<<8)|iz, 要求 nx,ny,nz <= 256。越界索引直接报错。"""
        i = np.asarray(idx, dtype=np.int64).reshape(-1, 3)
        for k, (n, name) in enumerate(((self.nx, "nx"), (self.ny, "ny"), (self.nz, "nz"))):
            hi = i[:, k].max() if len(i) else 0
            lo = i[:, k].min() if len(i) else 0
            if hi > 255 or lo < 0:
                raise ValueError(
                    f"{name}={n} 超出 8 位可表达范围(索引 {lo}..{hi})。"
                    f"32 位位拼接 (8|8|8) 失效, 必须改用散列(SPEC-COORD §5.3)。")
        return ((i[:, 0] << 16) | (i[:, 1] << 8) | i[:, 2]).astype(np.uint32)

    @staticmethod
    def unpack_hash(h: np.ndarray) -> np.ndarray:
        h = np.asarray(h, dtype=np.uint32)
        return np.stack([(h >> 16) & 0xFF, (h >> 8) & 0xFF, h & 0xFF], axis=1).astype(np.int64)

    # -------------------------------------------------------- 柱体编号(输出)
    def cell_index_xy(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        """柱体编号 = iy * nx + ix  (SPEC-VOXEL 4.2)。

        **不是 ix * ny + iy** —— 二者在 nx==ny 时才相同。本项目 nx=75, ny=60,
        用错会让整张高程图转置, 且方阵测试测不出来。
        """
        return np.asarray(iy, dtype=np.int64) * self.nx + np.asarray(ix, dtype=np.int64)


def self_test(verbose: bool = True) -> dict:
    g = VoxelGrid()
    out = {"nx": g.nx, "ny": g.ny, "nz": g.nz,
           "partial": g.last_cell_is_partial(), "last_span": g.last_cell_span()}

    # V-CO-5: 索引 <-> 中心 往返
    ii, jj, kk = np.meshgrid(np.arange(g.nx), np.arange(g.ny), np.arange(g.nz),
                             indexing="ij")
    idx_all = np.stack([ii.ravel(), jj.ravel(), kk.ravel()], axis=1)
    centers = g.index_to_center(idx_all)
    idx_back, inside = g.point_to_index(centers)
    assert inside.all(), "中心点被判越界"
    assert np.array_equal(idx_back, idx_all), "索引->中心->索引 不闭合"
    out["roundtrip_ok"] = True

    # 边界点: 每个轴的极小/极大点必须落在正确的首末格
    edge_pts = np.array([
        [g.x_max - 1e-6, g.y_max - 1e-6, g.z_min + 1e-6],   # x/y 最小索引, z 最小
        [g.x_min + 1e-6, g.y_min + 1e-6, g.z_max - 1e-6],   # 全最大索引
    ])
    ei, eo = g.point_to_index(edge_pts)
    assert eo.all()
    assert tuple(ei[0]) == (0, 0, 0), f"Xmax/Ymax/Zmin 应映射到 (0,0,0), 实际 {ei[0]}"
    assert tuple(ei[1]) == (g.nx - 1, g.ny - 1, g.nz - 1), f"实际 {ei[1]}"
    out["edge_ok"] = True

    # 反向性检查: X 更大的点 ix 应更小
    _, _ = g.point_to_index(np.array([[9000.0, 0, 0], [-4000.0, 0, 0]]))
    a = g.point_to_index(np.array([[9000.0, 0.0, 0.0]]))[0][0, 0]
    b = g.point_to_index(np.array([[-4000.0, 0.0, 0.0]]))[0][0, 0]
    assert a < b, "X 方向未反向"

    # Hash 往返
    h = g.pack_hash(idx_all)
    assert len(np.unique(h)) == len(h), "Hash 冲突(nx*ny*nz 应 <= 256^3)"
    assert np.array_equal(g.unpack_hash(h), idx_all), "Hash 解包不闭合"
    out["hash_ok"] = True

    # 柱体编号: 非方阵用例(V-VOX-6)
    nx, ny = g.nx, g.ny
    assert nx != ny, "本用例要求 nx != ny 才能暴露转置"
    c1 = g.cell_index_xy(np.array([3]), np.array([5]))[0]
    c2 = g.cell_index_xy(np.array([5]), np.array([3]))[0]
    assert c1 != c2, "柱体编号把 (3,5) 与 (5,3) 混了"
    assert c1 == 5 * nx + 3
    out["cell_ok"] = True

    # 负例: Hash 位宽越界必须报错
    try:
        VoxelGrid(x_max=60000).pack_hash(np.array([[300, 0, 0]]))
        raise AssertionError("nx 超 256 时未报错")
    except ValueError:
        out["neg_hash_ok"] = True

    if verbose:
        print(f"网格 {g.nx}x{g.ny}x{g.nz} = {g.nx*g.ny*g.nz} 体素 "
              f"(每体素 9 B -> {g.nx*g.ny*g.nz*9/1024:.0f} KB)")
        print(f"末格是否半格: {out['partial']}  末格厚度: {out['last_span']}")
        print(f"索引往返 {out['roundtrip_ok']}, 边界 {out['edge_ok']}, "
              f"Hash {out['hash_ok']}, 柱体编号 {out['cell_ok']}, 位宽负例 {out['neg_hash_ok']}")
    return out


if __name__ == "__main__":
    self_test()
