/**
 * @file voxel.cpp
 * @brief 体素哈希 + CMA + zMap。对应 Python 原型 nfocc_ph0/voxel_grid.py。
 */
#include "nfocc/nfocc_voxel.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>

struct NFVoxelCtx {
    NFVoxelGridParams g;
    NFVoxelEntry     *slots;
    nf_u32            capacity;
    nf_u32            count;
    nf_u32            collisions;
    nf_f32            n_eff_max;
};

namespace {

inline nf_u32 hash_of(nf_i32 ix, nf_i32 iy, nf_i32 iz) {
    return ((nf_u32)ix << 16) | ((nf_u32)iy << 8) | (nf_u32)iz;
}

/* 开放寻址: 线性探测 */
inline nf_u32 probe(nf_u32 h, nf_u32 i, nf_u32 cap) {
    return (h + i) & (cap - 1u);
}

inline bool is_pow2(nf_u32 v) { return v != 0u && (v & (v - 1u)) == 0u; }

} // namespace

extern "C" {

NFStatus nf_voxel_grid_init(NFVoxelGridParams *g, nf_i32 vox_mm,
                            nf_i32 x_min, nf_i32 x_max,
                            nf_i32 y_min, nf_i32 y_max,
                            nf_i32 z_min, nf_i32 z_max) {
    if (!g) return NF_E_PARAM_NULL;
    if (vox_mm <= 0) return NF_E_PARAM_RANGE;
    if (x_max <= x_min || y_max <= y_min || z_max <= z_min) return NF_E_PARAM_RANGE;

    g->version = NF_VOXEL_GRID_VERSION;
    g->vox_mm = vox_mm;
    g->x_min = x_min; g->x_max = x_max;
    g->y_min = y_min; g->y_max = y_max;
    g->z_min = z_min; g->z_max = z_max;

    /* 向上取整: Z 范围 3500/200 = 17.5 -> 18 层, 末格是 100mm 半格 */
    g->nx = (nf_u32)((x_max - x_min + vox_mm - 1) / vox_mm);
    g->ny = (nf_u32)((y_max - y_min + vox_mm - 1) / vox_mm);
    g->nz = (nf_u32)((z_max - z_min + vox_mm - 1) / vox_mm);
    if (g->nx == 0 || g->ny == 0 || g->nz == 0) return NF_E_PARAM_RANGE;

    /* 32 位位拼接 (8|8|8) 要求各维 <= 256 */
    if (g->nx > 256 || g->ny > 256 || g->nz > 256) return NF_E_PARAM_RANGE;
    /* zMap 用 uint32, bit31 保留为 0 */
    if (g->nz > 31) return NF_E_PARAM_RANGE;
    return NF_OK;
}

NFStatus nf_voxel_last_cell_partial(const NFVoxelGridParams *g,
                                    nf_i32 *span_x, nf_i32 *span_y, nf_i32 *span_z) {
    if (!g) return NF_E_PARAM_NULL;
    if (span_x) *span_x = (g->x_max - g->x_min) - (nf_i32)(g->nx - 1) * g->vox_mm;
    if (span_y) *span_y = (g->y_max - g->y_min) - (nf_i32)(g->ny - 1) * g->vox_mm;
    if (span_z) *span_z = (g->z_max - g->z_min) - (nf_i32)(g->nz - 1) * g->vox_mm;
    return NF_OK;
}

NFStatus nf_voxel_index(const NFVoxelGridParams *g, nf_f32 x, nf_f32 y, nf_f32 z,
                        nf_i32 *ix, nf_i32 *iy, nf_i32 *iz, nf_u8 *inside) {
    if (!g || !ix || !iy || !iz || !inside) return NF_E_PARAM_NULL;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        *inside = 0; return NF_OK;
    }
    const float v = (float)g->vox_mm;
    /* X/Y 反向, Z 正向 —— 见头文件 */
    const nf_i32 a = (nf_i32)std::floor(((float)g->x_max - x) / v);
    const nf_i32 b = (nf_i32)std::floor(((float)g->y_max - y) / v);
    const nf_i32 c = (nf_i32)std::floor((z - (float)g->z_min) / v);
    *ix = a; *iy = b; *iz = c;
    *inside = (a >= 0 && a < (nf_i32)g->nx &&
               b >= 0 && b < (nf_i32)g->ny &&
               c >= 0 && c < (nf_i32)g->nz) ? 1u : 0u;
    return NF_OK;
}

NFStatus nf_voxel_center(const NFVoxelGridParams *g, nf_i32 ix, nf_i32 iy, nf_i32 iz,
                         nf_f32 *x, nf_f32 *y, nf_f32 *z) {
    if (!g || !x || !y || !z) return NF_E_PARAM_NULL;
    const float v = (float)g->vox_mm;
    *x = (float)g->x_max - ((float)ix + 0.5f) * v;
    *y = (float)g->y_max - ((float)iy + 0.5f) * v;
    *z = (float)g->z_min + ((float)iz + 0.5f) * v;
    return NF_OK;
}

NFStatus nf_voxel_pack_hash(const nf_i32 *ix, const nf_i32 *iy, const nf_i32 *iz,
                            nf_u32 n, nf_u32 *hash) {
    if (!ix || !iy || !iz || !hash) return NF_E_PARAM_NULL;
    for (nf_u32 i = 0; i < n; ++i) {
        if (ix[i] < 0 || iy[i] < 0 || iz[i] < 0 ||
            ix[i] > 255 || iy[i] > 255 || iz[i] > 255) {
            return NF_E_PARAM_RANGE;   /* 8 位拼接失效, 需改散列 */
        }
        hash[i] = hash_of(ix[i], iy[i], iz[i]);
    }
    return NF_OK;
}

NFStatus nf_voxel_create(NFVoxelCtx **ctx, const NFVoxelGridParams *g,
                         nf_u32 capacity, nf_f32 n_eff_max) {
    if (!ctx || !g) return NF_E_PARAM_NULL;
    if (!is_pow2(capacity)) return NF_E_PARAM_RANGE;   /* 用掩码代替取模 */

    NFVoxelCtx *c = (NFVoxelCtx *)std::malloc(sizeof(NFVoxelCtx));
    if (!c) return NF_E_ENGINE_OOM;
    c->slots = (NFVoxelEntry *)std::calloc(capacity, sizeof(NFVoxelEntry));
    if (!c->slots) { std::free(c); return NF_E_ENGINE_OOM; }

    c->g = *g;
    c->capacity = capacity;
    c->count = 0;
    c->collisions = 0;
    c->n_eff_max = (n_eff_max > 0.0f) ? n_eff_max : (nf_f32)NF_V_N_EFF_MAX;
    *ctx = c;
    return NF_OK;
}

void nf_voxel_destroy(NFVoxelCtx *ctx) {
    if (!ctx) return;
    std::free(ctx->slots);
    std::free(ctx);
}

void nf_voxel_clear(NFVoxelCtx *ctx) {
    if (!ctx) return;
    std::memset(ctx->slots, 0, (size_t)ctx->capacity * sizeof(NFVoxelEntry));
    ctx->count = 0;
    ctx->collisions = 0;
}

NFStatus nf_voxel_insert(NFVoxelCtx *ctx, nf_f32 x, nf_f32 y, nf_f32 z,
                         nf_f32 conf, nf_u8 source_bit) {
    if (!ctx) return NF_E_PARAM_NULL;

    nf_i32 ix, iy, iz;
    nf_u8 inside = 0;
    nf_voxel_index(&ctx->g, x, y, z, &ix, &iy, &iz, &inside);
    if (!inside) return NF_OK;      /* 越界丢弃, 不算错误 */

    const nf_u32 h = hash_of(ix, iy, iz);
    nf_f32 w = (conf > 0.0f) ? conf : 0.0f;
    if (w > 1.0f) w = 1.0f;

    for (nf_u32 i = 0; i < ctx->capacity; ++i) {
        const nf_u32 s = probe(h, i, ctx->capacity);
        NFVoxelEntry *e = &ctx->slots[s];
        if (!e->used) {
            /* 空槽: 新建 */
            if (ctx->count + 1 > ctx->capacity - ctx->capacity / 8) {
                return NF_E_VOXEL_FULL;   /* 负载因子超 7/8 */
            }
            e->used = 1;
            e->hash = h;
            e->ix = ix; e->iy = iy; e->iz = iz;
            e->cx = x; e->cy = y; e->cz = z;
            e->n_eff = (w > 0.0f) ? w : 1.0f;
            e->source_mask = source_bit;
            ctx->count++;
            return NF_OK;
        }
        if (e->hash == h) {
            /* 命中: 加权 CMA */
            const nf_f32 n_old = e->n_eff;
            if (w <= 0.0f) {
                e->source_mask |= source_bit;
                return NF_OK;
            }
            const nf_f32 denom = n_old + w;
            const nf_f32 k = (denom > 1e-6f) ? (w / denom) : 0.5f;
            e->cx += k * (x - e->cx);
            e->cy += k * (y - e->cy);
            e->cz += k * (z - e->cz);

            /* 质心钳位在体素内 ±vox/2, 防止漂移到邻格 */
            nf_f32 vx, vy, vz;
            nf_voxel_center(&ctx->g, ix, iy, iz, &vx, &vy, &vz);
            const nf_f32 half = (nf_f32)ctx->g.vox_mm * 0.5f;
            if (e->cx < vx - half) e->cx = vx - half;
            if (e->cx > vx + half) e->cx = vx + half;
            if (e->cy < vy - half) e->cy = vy - half;
            if (e->cy > vy + half) e->cy = vy + half;
            if (e->cz < vz - half) e->cz = vz - half;
            if (e->cz > vz + half) e->cz = vz + half;

            /* 计数饱和: 衰减一半, 避免旧观测永久压制新观测 */
            e->n_eff = n_old + w;
            if (e->n_eff > ctx->n_eff_max) e->n_eff *= 0.5f;
            e->source_mask |= source_bit;
            return NF_OK;
        }
        ctx->collisions++;
    }
    return NF_E_VOXEL_FULL;
}

NFStatus nf_voxel_stats(const NFVoxelCtx *ctx, nf_u32 *num_entries,
                        nf_u32 *num_slots, nf_u32 *collisions) {
    if (!ctx) return NF_E_PARAM_NULL;
    if (num_entries) *num_entries = ctx->count;
    if (num_slots)   *num_slots = ctx->capacity;
    if (collisions)  *collisions = ctx->collisions;
    return NF_OK;
}

NFStatus nf_grid_flatten(const NFVoxelCtx *ctx, NFGridOutput *out) {
    if (!ctx || !out) return NF_E_PARAM_NULL;
    if (out->version != NF_GRID_OUTPUT_VERSION) return NF_E_PARAM_VERSION;

    const nf_u32 n_cells = ctx->g.nx * ctx->g.ny;
    if (out->n_cells != n_cells || !out->zmap || !out->zindex) return NF_E_PARAM_RANGE;

    std::memset(out->zmap, 0, (size_t)n_cells * sizeof(nf_u32));
    std::memset(out->zindex, 0, (size_t)(n_cells + 1) * sizeof(nf_u32));

    /* 先统计每柱的层数, 再置位 */
    for (nf_u32 s = 0; s < ctx->capacity; ++s) {
        const NFVoxelEntry *e = &ctx->slots[s];
        if (!e->used) continue;
        if (e->iz < 0 || e->iz >= (nf_i32)ctx->g.nz) continue;
        if (e->ix < 0 || e->ix >= (nf_i32)ctx->g.nx) continue;
        if (e->iy < 0 || e->iy >= (nf_i32)ctx->g.ny) continue;
        /* 柱体编号: iy*nx+ix (不是 ix*ny+iy) */
        const nf_u32 cell = (nf_u32)e->iy * ctx->g.nx + (nf_u32)e->ix;
        out->zmap[cell] |= (1u << (nf_u32)e->iz);
    }

    /* 前缀和: zindex[c] = 该柱及之前所有柱的累计点数 */
    nf_u32 acc = 0;
    out->zindex[0] = 0;
    for (nf_u32 c = 0; c < n_cells; ++c) {
        nf_u32 bits = out->zmap[c];
        nf_u32 cnt = 0;
        while (bits) { cnt += (bits & 1u); bits >>= 1; }
        acc += cnt;
        out->zindex[c + 1] = acc;
    }
    return NF_OK;
}

void nf_grid_output_destroy(NFGridOutput *out) {
    if (!out) return;
    std::free(out->zmap);
    std::free(out->zindex);
    out->zmap = nullptr;
    out->zindex = nullptr;
    out->n_cells = 0;
}

NFStatus nf_pointcloud_alloc(NFPointCloud *pc, nf_u32 max_points) {
    if (!pc) return NF_E_PARAM_NULL;
    if (max_points == 0) return NF_E_PARAM_RANGE;
    pc->points = (NFPoint *)std::calloc(max_points, sizeof(NFPoint));
    if (!pc->points) return NF_E_ENGINE_OOM;
    pc->version = NF_POINT_CLOUD_VERSION;
    pc->num_points = 0;
    pc->max_points = max_points;
    pc->overflow = 0;
    return NF_OK;
}

void nf_pointcloud_destroy(NFPointCloud *pc) {
    if (!pc) return;
    std::free(pc->points);
    pc->points = nullptr;
    pc->num_points = 0;
    pc->max_points = 0;
}

namespace {
struct ExportItem { nf_f32 prob; nf_u32 slot; };
inline bool cmp_desc(const ExportItem &a, const ExportItem &b) { return a.prob > b.prob; }
} // namespace

NFStatus nf_voxel_export_points(const NFVoxelCtx *ctx, NFPointCloud *pc) {
    if (!ctx || !pc) return NF_E_PARAM_NULL;
    if (!pc->points) return NF_E_PARAM_NULL;

    /* 收集有效条目 */
    nf_u32 n = 0;
    ExportItem *items = (ExportItem *)std::malloc((size_t)ctx->count * sizeof(ExportItem));
    if (!items && ctx->count) return NF_E_ENGINE_OOM;

    for (nf_u32 s = 0; s < ctx->capacity; ++s) {
        const NFVoxelEntry *e = &ctx->slots[s];
        if (!e->used) continue;
        /* 存在概率: 饱和映射 (n_eff=1 -> ~50, n_eff=n_max -> 100) */
        nf_f32 p = e->n_eff / (e->n_eff + 1.0f);
        if (p > 1.0f) p = 1.0f;
        items[n].prob = p;
        items[n].slot = s;
        ++n;
    }

    const nf_u32 cap = pc->max_points;
    if (n > cap) {
        std::partial_sort(items, items + cap, items + n, cmp_desc);
        pc->overflow = 1;
        n = cap;
    }

    for (nf_u32 i = 0; i < n; ++i) {
        const NFVoxelEntry *e = &ctx->slots[items[i].slot];
        NFPoint *pt = &pc->points[i];
        pt->x_mm = (nf_i32)(e->cx >= 0.0f ? e->cx + 0.5f : e->cx - 0.5f);
        pt->y_mm = (nf_i32)(e->cy >= 0.0f ? e->cy + 0.5f : e->cy - 0.5f);
        pt->z_mm = (nf_i32)(e->cz >= 0.0f ? e->cz + 0.5f : e->cz - 0.5f);
        pt->source_camera = e->source_mask;
        pt->sigma_cm = 255;                /* 由调用方按 σ 填充 */
        nf_f32 p = items[i].prob * 100.0f;
        pt->existence_prob = (nf_u8)(p > 100.0f ? 100.0f : p);
    }
    pc->num_points = n;
    std::free(items);
    return pc->overflow ? NF_E_OUTPUT_OVERFLOW : NF_OK;
}

} /* extern "C" */
