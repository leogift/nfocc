/**
 * @file octree.cpp
 * @brief 纯网格哈希空间索引实现。
 *
 * ## 全部结构只有两样
 *   1. 一张开放寻址哈希表：`格子坐标(ix,iy,iz) -> 格序号`
 *   2. CSR 点列表：`csr_start[格] .. csr_start[格+1]` 是该格的点索引
 *
 * 没有父子链、没有深度、没有节点编号 —— 前六版的所有 bug 都来自
 * 这几个结构之间的不一致，去掉它们就不可能不一致。
 *
 * ## 为什么固定格边长就够
 * 前几版纠结"稠密区要细分"，是想当然。算一下就清楚：
 *
 *   150k 点在 9x7.5x2 m 的 5% 体积 ≈ 6.75 m³ = 6.75e9 mm³
 *   每点占 45000 mm³，即平均间距 ≈ 36 mm
 *
 * 格边长取 40mm（略大于平均间距）时，绝大多数格只有 1-2 点。
 * DBSCAN 的 eps=120mm 覆盖 3x3x3=27 到 7x7x7=343 个格，
 * 每格 1-2 点，一次查询约 300-600 次比较，完全够。
 *
 * **"稠密区单格点数多"只在点云极度不均时才发生**，而那种情况
 * 靠调 cell_mm 解决，不需要为它引入整棵树的复杂度。
 */
#include "nfocc/nfocc_octree.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {

const nf_i32 kEmptySlot = -1;

/** 三维格坐标 -> 线性键。各 21 位，覆盖 ±100 万格 */
inline nf_i64 cell_key(nf_i32 ix, nf_i32 iy, nf_i32 iz) {
    return ((nf_i64)(ix + 1048576) << 42)
         | ((nf_i64)(iy + 1048576) << 21)
         |  (nf_i64)(iz + 1048576);
}

inline nf_u32 hash_key(nf_i64 k) {
    nf_u64 x = (nf_u64)k;
    x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return (nf_u32)(x & 0xFFFFFFFFull);
}

}  // namespace

struct NFOctreeCtx {
    nf_u32   version;
    nf_u32   n;
    nf_f32  *pts;          /* n*3 拷贝 */

    nf_f32   cell_mm;
    nf_f32   origin[3];    /* 包围盒下角，用于坐标 -> 格号 */

    /* 哈希表 */
    nf_i64  *keys;         /* 槽键，kEmptySlot 表示空 */
    nf_i32  *cell_id;      /* 槽 -> 格序号 */
    nf_u32   n_slots;
    nf_u32   n_cells;

    /* CSR：格 i 的点区间 [csr_start[i], csr_start[i+1]) */
    nf_u32  *csr_start;
    nf_u32  *csr_items;

    nf_u32   max_cell;
};

extern "C" {

NFStatus nf_octree_build(NFOctreeCtx **out, const nf_f32 *pts, nf_u32 n,
                         const NFOctreeConfig *cfg) {
    if (!out || (!pts && n > 0)) return NF_E_PARAM_NULL;
    *out = nullptr;
    if (n == 0) return NF_E_PARAM_RANGE;

    const nf_f32 cell = (cfg && cfg->cell_mm > 0.0f)
                      ? cfg->cell_mm : (nf_f32)NF_HASH_CELL_MM;

    NFOctreeCtx *c = (NFOctreeCtx *)std::calloc(1, sizeof(NFOctreeCtx));
    if (!c) return NF_E_ENGINE_OOM;
    c->version = NF_OCTREE_VERSION;
    c->n = n;
    c->cell_mm = cell;

    c->pts = (nf_f32 *)std::malloc((size_t)n * 3 * sizeof(nf_f32));
    if (!c->pts) { std::free(c); return NF_E_ENGINE_OOM; }
    std::memcpy(c->pts, pts, (size_t)n * 3 * sizeof(nf_f32));

    /* 包围盒下角作为格坐标原点 */
    c->origin[0] = pts[0]; c->origin[1] = pts[1]; c->origin[2] = pts[2];
    for (nf_u32 i = 1; i < n; ++i)
        for (int k = 0; k < 3; ++k)
            if (pts[i*3+k] < c->origin[k]) c->origin[k] = pts[i*3+k];

    /* 哈希表：按点数给容量，负载因子 <= 1/3 */
    nf_u32 slots = 64u;
    while (slots < n * 3u) slots <<= 1u;
    c->n_slots = slots;
    c->keys = (nf_i64 *)std::malloc((size_t)slots * sizeof(nf_i64));
    c->cell_id = (nf_i32 *)std::malloc((size_t)slots * sizeof(nf_i32));
    if (!c->keys || !c->cell_id) {
        std::free(c->pts); std::free(c->keys); std::free(c->cell_id);
        std::free(c);
        return NF_E_ENGINE_OOM;
    }
    for (nf_u32 i = 0; i < slots; ++i) c->keys[i] = kEmptySlot;

    /* 每个点的格序号 */
    nf_u32 *cell_of = (nf_u32 *)std::malloc((size_t)n * sizeof(nf_u32));
    if (!cell_of) {
        std::free(c->pts); std::free(c->keys); std::free(c->cell_id);
        std::free(c);
        return NF_E_ENGINE_OOM;
    }

    const nf_u32 mask = slots - 1u;
    for (nf_u32 i = 0; i < n; ++i) {
        const nf_i32 ix = (nf_i32)std::floor((pts[i*3+0] - c->origin[0]) / cell);
        const nf_i32 iy = (nf_i32)std::floor((pts[i*3+1] - c->origin[1]) / cell);
        const nf_i32 iz = (nf_i32)std::floor((pts[i*3+2] - c->origin[2]) / cell);
        const nf_i64 key = cell_key(ix, iy, iz);
        nf_u32 s = hash_key(key) & mask;
        for (nf_u32 probe = 0; probe < slots; ++probe) {
            if (c->keys[s] == kEmptySlot) {
                c->keys[s] = key;
                c->cell_id[s] = (nf_i32)c->n_cells++;
                cell_of[i] = (nf_u32)c->cell_id[s];
                break;
            }
            if (c->keys[s] == key) { cell_of[i] = (nf_u32)c->cell_id[s]; break; }
            s = (s + 1) & mask;
        }
    }

    /* CSR 计数 */
    c->csr_start = (nf_u32 *)std::calloc((size_t)c->n_cells + 1u, sizeof(nf_u32));
    c->csr_items = (nf_u32 *)std::malloc((size_t)n * sizeof(nf_u32));
    if (!c->csr_start || !c->csr_items) {
        std::free(cell_of); std::free(c->pts); std::free(c->keys);
        std::free(c->cell_id); std::free(c->csr_start); std::free(c->csr_items);
        std::free(c);
        return NF_E_ENGINE_OOM;
    }
    for (nf_u32 i = 0; i < n; ++i) c->csr_start[cell_of[i] + 1u]++;
    for (nf_u32 t = 0; t < c->n_cells; ++t) c->csr_start[t + 1u] += c->csr_start[t];

    /* CSR 填充：用临时游标 */
    {
        nf_u32 *cur = (nf_u32 *)std::malloc((size_t)c->n_cells * sizeof(nf_u32));
        if (!cur) {
            std::free(cell_of); std::free(c->pts); std::free(c->keys);
            std::free(c->cell_id); std::free(c->csr_start);
            std::free(c->csr_items); std::free(c);
            return NF_E_ENGINE_OOM;
        }
        std::memcpy(cur, c->csr_start, (size_t)c->n_cells * sizeof(nf_u32));
        for (nf_u32 i = 0; i < n; ++i) c->csr_items[cur[cell_of[i]]++] = i;
        std::free(cur);
    }

    c->max_cell = 0;
    for (nf_u32 t = 0; t < c->n_cells; ++t) {
        const nf_u32 cnt = c->csr_start[t + 1u] - c->csr_start[t];
        if (cnt > c->max_cell) c->max_cell = cnt;
    }

    std::free(cell_of);
    *out = c;
    return NF_OK;
}

void nf_octree_destroy(NFOctreeCtx *c) {
    if (!c) return;
    std::free(c->pts);
    std::free(c->keys);
    std::free(c->cell_id);
    std::free(c->csr_start);
    std::free(c->csr_items);
    std::free(c);
}

NFStatus nf_octree_radius(const NFOctreeCtx *c, nf_f32 x, nf_f32 y, nf_f32 z,
                          nf_f32 radius, nf_u32 *out_idx, nf_u32 capacity,
                          nf_u32 *out_count, nf_u8 *out_truncated) {
    if (!c || !out_idx || !out_count) return NF_E_PARAM_NULL;
    if (!(radius > 0.0f)) return NF_E_PARAM_RANGE;
    *out_count = 0;
    if (out_truncated) *out_truncated = 0;

    const nf_f32 r2 = radius * radius;
    const nf_f32 cell = c->cell_mm;

    /**
     * 球的 AABB 覆盖哪些格：由坐标直接算格区间，不需要遍历。
     *
     * x_lo = floor((x - r - origin) / cell)
     * x_hi = floor((x + r - origin) / cell)
     *
     * 用 floor 而不是 ceil 边界，保证球恰好落在格边界上时不漏格。
     */
    const nf_i32 x_lo = (nf_i32)std::floor((x - radius - c->origin[0]) / cell);
    const nf_i32 x_hi = (nf_i32)std::floor((x + radius - c->origin[0]) / cell);
    const nf_i32 y_lo = (nf_i32)std::floor((y - radius - c->origin[1]) / cell);
    const nf_i32 y_hi = (nf_i32)std::floor((y + radius - c->origin[1]) / cell);
    const nf_i32 z_lo = (nf_i32)std::floor((z - radius - c->origin[2]) / cell);
    const nf_i32 z_hi = (nf_i32)std::floor((z + radius - c->origin[2]) / cell);

    const nf_u32 mask = c->n_slots - 1u;
    nf_u32 cnt = 0;
    nf_u8 trunc = 0;

    for (nf_i32 iz = z_lo; iz <= z_hi; ++iz)
    for (nf_i32 iy = y_lo; iy <= y_hi; ++iy)
    for (nf_i32 ix = x_lo; ix <= x_hi; ++ix) {
        const nf_i64 key = cell_key(ix, iy, iz);
        nf_u32 s = hash_key(key) & mask;
        nf_i32 cid = kEmptySlot;
        for (nf_u32 probe = 0; probe < c->n_slots; ++probe) {
            if (c->keys[s] == kEmptySlot) break;
            if (c->keys[s] == key) { cid = c->cell_id[s]; break; }
            s = (s + 1) & mask;
        }
        if (cid == kEmptySlot) continue;

        const nf_u32 b = c->csr_start[cid];
        const nf_u32 e = c->csr_start[cid + 1u];
        for (nf_u32 k = b; k < e; ++k) {
            const nf_u32 pi = c->csr_items[k];
            const nf_f32 dx = c->pts[pi*3+0] - x;
            const nf_f32 dy = c->pts[pi*3+1] - y;
            const nf_f32 dz = c->pts[pi*3+2] - z;
            if (dx*dx + dy*dy + dz*dz <= r2) {
                if (cnt < capacity) out_idx[cnt] = pi;
                else trunc = 1;
                ++cnt;
            }
        }
    }

    *out_count = cnt;
    if (out_truncated) *out_truncated = trunc;
    return NF_OK;
}

NFStatus nf_octree_stats(const NFOctreeCtx *c, nf_u32 *out_cells,
                         nf_u32 *out_max_cell, nf_f32 *out_avg_cell,
                         nf_f32 *out_cell_mm) {
    if (!c) return NF_E_PARAM_NULL;
    if (out_cells)    *out_cells = c->n_cells;
    if (out_max_cell) *out_max_cell = c->max_cell;
    if (out_avg_cell) *out_avg_cell = c->n_cells
        ? (nf_f32)c->n / (nf_f32)c->n_cells : 0.0f;
    if (out_cell_mm)  *out_cell_mm = c->cell_mm;
    return NF_OK;
}

NFStatus nf_octree_point(const NFOctreeCtx *c, nf_u32 idx, nf_f32 out[3]) {
    if (!c || !out) return NF_E_PARAM_NULL;
    if (idx >= c->n) return NF_E_PARAM_RANGE;
    out[0] = c->pts[idx*3+0];
    out[1] = c->pts[idx*3+1];
    out[2] = c->pts[idx*3+2];
    return NF_OK;
}

nf_u32 nf_octree_count(const NFOctreeCtx *c) {
    return c ? c->n : 0u;
}

} /* extern "C" */
