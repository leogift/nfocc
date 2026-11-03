/**
 * @file voxel_filter.cpp
 * @brief REV-003 五阶段稠密体素精简实现。
 *
 * 设计约束：运行期**零堆分配**。所有缓冲在 create 时一次分配。
 */
#include "nfocc/nfocc_voxel_filter.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

struct NFVoxelFilterCtx {
    NFVoxelFilterConfig cfg;
    nf_u32 nx, ny, n_cells;

    nf_u32  *raw_zmap;        /**< 阶段 2 原始位图 */
    nf_u32  *filtered_zmap;   /**< 阶段 3/4 过滤后位图 */
    nf_i32  *first_hit_cm;    /**< 阶段 4 每扇区最近距离 (cm)，-1=空 */
    nf_u32  *cursor;          /**< 阶段 5 前缀和 */
    nf_u32  *cell_of_point;   /**< 每个输入点的 cell 索引 */
    nf_u8   *keep;            /**< 每个输入点的保留标记 */
    nf_u8   *flags_of_pt;     /**< 每个输入点的 surface_flags */

    nf_f32  ego_dx, ego_dy;   /**< 自车运动方向，(0,0)=静止 */
};

namespace {

/** countTrailingZeros32。x==0 未定义，调用前必须判空。 */
inline nf_u32 ctz32(nf_u32 x) {
    return (nf_u32)__builtin_ctz(x);
}

/**
 * 快速 atan2 近似，最大误差 ~0.0038 rad。
 *
 * 用多项式而不是库函数：一帧要算几十万次，atan2f 在部分 libm 上走
 * double 提高精度，每次几十个周期，累积起来是毫秒级。
 */
inline nf_f32 fast_atan2(nf_f32 y, nf_f32 x) {
    const nf_f32 ax = std::fabs(x), ay = std::fabs(y);
    if (ax < 1e-9f && ay < 1e-9f) return 0.0f;
    const nf_f32 mn = (ay < ax) ? ay : ax;
    const nf_f32 mx = (ay < ax) ? ax : ay;
    const nf_f32 t = mn / mx;
    const nf_f32 t2 = t * t;
    const nf_f32 at = t * (0.7853981633974483f
                   + t2 * (-0.0464964749f + t2 * (0.0103767019f
                   + t2 * (-0.0024610778f + t2 * 0.00047815675f))));
    nf_f32 a = at;
    if (ay > ax) a = 1.5707963267948966f - at;
    if (x < 0.0f) a = 3.141592653589793f - a;
    return (y < 0.0f) ? -a : a;
}

inline nf_u8 prob_to_u8(nf_f32 p) {
    if (!(p > 0.0f)) return 0u;
    if (p >= 1.0f) return 255u;
    nf_f32 v = p * 255.0f + 0.5f;
    return (nf_u8)(v > 255.0f ? 255.0f : v);
}

inline nf_i16 clamp_i16(nf_f32 v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (nf_i16)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

inline nf_i32 cell_index(const NFVoxelFilterCtx *c, nf_f32 x, nf_f32 y) {
    const nf_f32 fx = (x - c->cfg.roi_x_min) / c->cfg.cell_mm;
    const nf_f32 fy = (y - c->cfg.roi_y_min) / c->cfg.cell_mm;
    if (fx < 0.0f || fy < 0.0f) return -1;
    const nf_u32 gx = (nf_u32)fx, gy = (nf_u32)fy;
    if (gx >= c->nx || gy >= c->ny) return -1;
    return (nf_i32)(gy * c->nx + gx);
}

inline nf_i32 z_bin_of(const NFVoxelFilterCtx *c, nf_f32 z) {
    const nf_f32 fz = (z - c->cfg.roi_z_min) / c->cfg.z_bin_mm;
    if (fz < 0.0f) return -1;
    const nf_i32 b = (nf_i32)fz;
    return (b >= 0 && b < 32) ? b : -1;
}

} /* namespace */

/* ---------------------------------------------------------------- API */

NFStatus nf_voxel_filter_default_config(NFVoxelFilterConfig *cfg) {
    if (!cfg) return NF_E_PARAM_NULL;
    std::memset(cfg, 0, sizeof(*cfg));
    cfg->version = 1;
    cfg->cell_mm = NF_DVF_CELL_MM;
    cfg->z_bin_mm = NF_DVF_Z_BIN_MM;
    cfg->roi_x_min = NF_DVF_ROI_X_MIN;
    cfg->roi_x_max = NF_DVF_ROI_X_MAX;
    cfg->roi_y_min = NF_DVF_ROI_Y_MIN;
    cfg->roi_y_max = NF_DVF_ROI_Y_MAX;
    cfg->roi_z_min = NF_DVF_ROI_Z_MIN;
    cfg->roi_z_max = NF_DVF_ROI_Z_MAX;
    cfg->shell_thickness_cm = NF_DVF_SHELL_CM;
    cfg->max_output_points = NF_DVF_HARD_CAP_MAX_POINTS;
    cfg->ground_max_z_bin = NF_DVF_GROUND_MAX_Z_BIN;
    cfg->ground_cont_bins = NF_DVF_GROUND_CONT_BINS;
    cfg->polar_bins = NF_DVF_POLAR_BINS;
    cfg->enable_ground_strip = 1;
    cfg->enable_polar_shell = 1;
    cfg->enable_foe_filter = 1;
    return NF_OK;
}

NFStatus nf_voxel_filter_create(NFVoxelFilterCtx **ctx,
                                const NFVoxelFilterConfig *cfg) {
    if (!ctx) return NF_E_PARAM_NULL;
    *ctx = nullptr;

    NFVoxelFilterConfig c;
    if (cfg) c = *cfg; else nf_voxel_filter_default_config(&c);

    if (!(c.cell_mm > 0.0f)) c.cell_mm = NF_DVF_CELL_MM;
    if (!(c.z_bin_mm > 0.0f)) c.z_bin_mm = NF_DVF_Z_BIN_MM;
    if (!(c.roi_x_max > c.roi_x_min)) { c.roi_x_min = NF_DVF_ROI_X_MIN; c.roi_x_max = NF_DVF_ROI_X_MAX; }
    if (!(c.roi_y_max > c.roi_y_min)) { c.roi_y_min = NF_DVF_ROI_Y_MIN; c.roi_y_max = NF_DVF_ROI_Y_MAX; }
    if (!(c.roi_z_max > c.roi_z_min)) { c.roi_z_min = NF_DVF_ROI_Z_MIN; c.roi_z_max = NF_DVF_ROI_Z_MAX; }
    if (!(c.shell_thickness_cm > 0.0f)) c.shell_thickness_cm = NF_DVF_SHELL_CM;
    if (!c.ground_cont_bins) c.ground_cont_bins = NF_DVF_GROUND_CONT_BINS;
    if (!c.polar_bins) c.polar_bins = NF_DVF_POLAR_BINS;
    if (c.polar_bins > 4096u) c.polar_bins = 4096u;

    /* 配额钳制：硬上限是下游接口的物理约束，不可越过 */
    if (!c.max_output_points || c.max_output_points > NF_DVF_HARD_CAP_MAX_POINTS)
        c.max_output_points = NF_DVF_HARD_CAP_MAX_POINTS;

    const nf_u32 nx = (nf_u32)((c.roi_x_max - c.roi_x_min) / c.cell_mm) + 1u;
    const nf_u32 ny = (nf_u32)((c.roi_y_max - c.roi_y_min) / c.cell_mm) + 1u;
    if (nx == 0 || ny == 0 || nx > 65536u || ny > 65536u) return NF_E_PARAM_RANGE;

    NFVoxelFilterCtx *q = (NFVoxelFilterCtx *)std::calloc(1, sizeof(NFVoxelFilterCtx));
    if (!q) return NF_E_ENGINE_OOM;
    q->cfg = c;
    q->nx = nx;
    q->ny = ny;
    q->n_cells = nx * ny;

    const size_t nc = (size_t)q->n_cells;
    q->raw_zmap      = (nf_u32 *)std::calloc(nc, sizeof(nf_u32));
    q->filtered_zmap = (nf_u32 *)std::calloc(nc, sizeof(nf_u32));
    q->cursor        = (nf_u32 *)std::calloc(nc + 1u, sizeof(nf_u32));
    q->first_hit_cm  = (nf_i32 *)std::calloc(c.polar_bins, sizeof(nf_i32));

    const nf_u32 cap_pts = NF_DVF_HARD_CAP_MAX_POINTS * 40u;
    q->cell_of_point = (nf_u32 *)std::calloc(cap_pts, sizeof(nf_u32));
    q->keep          = (nf_u8  *)std::calloc(cap_pts, sizeof(nf_u8));
    q->flags_of_pt   = (nf_u8  *)std::calloc(cap_pts, sizeof(nf_u8));

    if (!q->raw_zmap || !q->filtered_zmap || !q->cursor || !q->first_hit_cm ||
        !q->cell_of_point || !q->keep || !q->flags_of_pt) {
        nf_voxel_filter_destroy(q);
        return NF_E_ENGINE_OOM;
    }
    *ctx = q;
    return NF_OK;
}

void nf_voxel_filter_destroy(NFVoxelFilterCtx *ctx) {
    if (!ctx) return;
    std::free(ctx->raw_zmap);
    std::free(ctx->filtered_zmap);
    std::free(ctx->cursor);
    std::free(ctx->first_hit_cm);
    std::free(ctx->cell_of_point);
    std::free(ctx->keep);
    std::free(ctx->flags_of_pt);
    std::free(ctx);
}

NFStatus nf_voxel_filter_update_ego_motion(NFVoxelFilterCtx *ctx,
                                           nf_f32 dir_x, nf_f32 dir_y) {
    if (!ctx) return NF_E_PARAM_NULL;
    const nf_f32 n = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    if (n < 1e-6f) { ctx->ego_dx = 0.0f; ctx->ego_dy = 0.0f; return NF_OK; }
    ctx->ego_dx = dir_x / n;
    ctx->ego_dy = dir_y / n;
    return NF_OK;
}

NFStatus nf_voxel_filter_grid_size(const NFVoxelFilterCtx *ctx,
                                   nf_u32 *nx, nf_u32 *ny, nf_u32 *n_cells) {
    if (!ctx) return NF_E_PARAM_NULL;
    if (nx) *nx = ctx->nx;
    if (ny) *ny = ctx->ny;
    if (n_cells) *n_cells = ctx->n_cells;
    return NF_OK;
}

NFStatus nf_voxel_filter_compress(NFVoxelFilterCtx *ctx,
                                  const NFVoxelEntry *points, nf_u32 n_points,
                                  nf_u32 *zmap_out, nf_u32 *zindex_out,
                                  NFDownstreamPoint *pts_out,
                                  NFVoxelFilterStats *stats) {
    if (!ctx || !zmap_out || !zindex_out || !pts_out) return NF_E_PARAM_NULL;
    if (n_points && !points) return NF_E_PARAM_NULL;

    const nf_u32 cap_pts = NF_DVF_HARD_CAP_MAX_POINTS * 40u;
    if (n_points > cap_pts) return NF_E_PARAM_RANGE;

    NFVoxelFilterStats st;
    std::memset(&st, 0, sizeof(st));
    st.n_input = n_points;

    std::memset(ctx->raw_zmap, 0, (size_t)ctx->n_cells * sizeof(nf_u32));
    std::memset(ctx->filtered_zmap, 0, (size_t)ctx->n_cells * sizeof(nf_u32));
    std::memset(ctx->cursor, 0, (size_t)(ctx->n_cells + 1u) * sizeof(nf_u32));
    std::memset(ctx->keep, 0, (size_t)n_points * sizeof(nf_u8));
    std::memset(ctx->flags_of_pt, 0, (size_t)n_points * sizeof(nf_u8));
    for (nf_u32 i = 0; i < ctx->cfg.polar_bins; ++i) ctx->first_hit_cm[i] = -1;

    const nf_f32 cos2 = NF_DVF_FOE_COS2;
    const nf_u8 do_foe = (nf_u8)(ctx->cfg.enable_foe_filter &&
                         (ctx->ego_dx * ctx->ego_dx + ctx->ego_dy * ctx->ego_dy) > 0.5f);

    /* ============ 阶段 1: AABB 裁剪 + FoE 奇异锥 ============ */
    nf_u32 n1 = 0;
    for (nf_u32 i = 0; i < n_points; ++i) {
        const nf_f32 x = points[i].cx, y = points[i].cy, z = points[i].cz;
        if (x < ctx->cfg.roi_x_min || x > ctx->cfg.roi_x_max ||
            y < ctx->cfg.roi_y_min || y > ctx->cfg.roi_y_max ||
            z < ctx->cfg.roi_z_min || z > ctx->cfg.roi_z_max) continue;
        ++st.n_after_roi;

        /*
         * FoE 锥：点落在运动方向前方 15 度半角内则剔除。
         *
         * 判据 dot>0 且 dot^2 >= (x^2+y^2)*cos^2 只用乘法与比较 ——
         * 无开方、无除法、无三角函数，正是原设计的意图。
         *
         * 几何含义：锥内点沿视线几乎不产生横向像移，三角化退化为
         * "两点定一射线"，解出几十米外的拉丝。
         */
        if (do_foe) {
            const nf_f32 dot = x * ctx->ego_dx + y * ctx->ego_dy;
            const nf_f32 r2 = x * x + y * y;
            if (dot > 0.0f && dot * dot >= r2 * cos2) continue;
        }
        ctx->keep[i] = 1u;
        ++n1;
    }
    st.n_after_foe = n1;

    /* ============ 阶段 2: 2D 柱体位图投影 ============ */
    for (nf_u32 i = 0; i < n_points; ++i) {
        if (!ctx->keep[i]) continue;
        const nf_i32 cell = cell_index(ctx, points[i].cx, points[i].cy);
        if (cell < 0) { ctx->keep[i] = 0u; continue; }
        const nf_i32 zb = z_bin_of(ctx, points[i].cz);
        if (zb < 0) { ctx->keep[i] = 0u; continue; }
        ctx->cell_of_point[i] = (nf_u32)cell;
        ctx->raw_zmap[cell] |= (1u << (nf_u32)zb);
    }

    /* ============ 阶段 3: 位运算地面剥离 ============ */
    for (nf_u32 cell = 0; cell < ctx->n_cells; ++cell) {
        const nf_u32 bits = ctx->raw_zmap[cell];
        if (!bits) continue;

        nf_u32 filtered = bits;
        if (ctx->cfg.enable_ground_strip) {
            const nf_u32 z0 = ctz32(bits);   /* bits != 0，安全 */

            /*
             * **低矮障碍物保护**（评审 §5.2 指出的风险）。
             *
             * 原设计 z0 <= 2 就剥掉连续 2 层。但 200mm 的地锁正好在
             * z0 = 2（z_bin=100mm，z=200 -> bin 2），会被整段剥掉，
             * 而地锁在泊车场景里是最关键的障碍物之一。
             *
             * 判据改为：剥完之后**上面还有东西**才算地面。纯地锁是
             * 孤立的 1-2 层，剥掉就空了 -> 保留；真实地面 z=0..1 上面
             * 还接着车体或墙体 -> 剥掉。
             */
            if (z0 <= ctx->cfg.ground_max_z_bin) {
                const nf_u32 ztop = z0 + ctx->cfg.ground_cont_bins;
                nf_u32 mask = 0u;
                for (nf_u32 k = z0; k < ztop && k < 32u; ++k) mask |= (1u << k);

                const nf_u32 rest = bits & ~mask;
                if (ztop < 32u && rest && (rest >> ztop)) filtered = rest;
            }
        }
        ctx->filtered_zmap[cell] = filtered;
    }

    /*
     * 逐点判"自己那一层还在不在"。
     *
     * **不能只看 cell 的位图是否非零**：地面点与它上方的障碍物在同一个
     * cell 里，剥离后 filtered_zmap[cell] 仍非零（障碍物还在），只判
     * 非零会把地面点一起放过去。必须按该点自己的 z_bin 查位。
     */
    nf_u32 n3 = 0;
    for (nf_u32 i = 0; i < n_points; ++i) {
        if (!ctx->keep[i]) continue;
        const nf_u32 bits = ctx->filtered_zmap[ctx->cell_of_point[i]];
        const nf_i32 zb = z_bin_of(ctx, points[i].cz);
        if (!bits || zb < 0 || !(bits & (1u << (nf_u32)zb))) {
            ctx->keep[i] = 0u;
            continue;
        }
        ++n3;
    }
    st.n_after_ground = n3;

    /* ============ 阶段 4: 极坐标首击外壳 ============ */
    const nf_u32 nb = ctx->cfg.polar_bins;
    const nf_f32 bin_scale = (nf_f32)nb / 6.283185307179586f;
    const nf_f32 kPi = 3.141592653589793f;

    if (ctx->cfg.enable_polar_shell) {
        for (nf_u32 i = 0; i < n_points; ++i) {
            if (!ctx->keep[i]) continue;
            const nf_f32 x = points[i].cx, y = points[i].cy;
            const nf_f32 r = std::sqrt(x * x + y * y);
            const nf_f32 th = fast_atan2(y, x);
            nf_i32 bin = (nf_i32)((th + kPi) * bin_scale);
            if (bin < 0) bin = 0;
            if (bin >= (nf_i32)nb) bin = (nf_i32)nb - 1;
            const nf_i32 cm = (nf_i32)(r * 0.1f);
            nf_i32 &slot = ctx->first_hit_cm[bin];
            if (slot < 0 || cm < slot) slot = cm;
        }
    }

    nf_u32 n4 = 0;
    for (nf_u32 i = 0; i < n_points; ++i) {
        if (!ctx->keep[i]) continue;
        const nf_f32 x = points[i].cx, y = points[i].cy;
        nf_u8 fl = NF_DVF_SURF_NORMAL;

        if (ctx->cfg.enable_polar_shell) {
            const nf_f32 r = std::sqrt(x * x + y * y);
            const nf_f32 th = fast_atan2(y, x);
            nf_i32 bin = (nf_i32)((th + kPi) * bin_scale);
            if (bin < 0) bin = 0;
            if (bin >= (nf_i32)nb) bin = (nf_i32)nb - 1;
            const nf_i32 cm = (nf_i32)(r * 0.1f);
            const nf_i32 fh = ctx->first_hit_cm[bin];
            if (fh >= 0 && cm > fh + (nf_i32)ctx->cfg.shell_thickness_cm) {
                /*
                 * **必须清 keep**，不能只 continue。
                 * 后面阶段 5 的前缀和是按 keep 计数的，漏清会让 zindex
                 * 统计到已被剔除的点，下游按 zindex 取柱体会越界读到
                 * 未初始化的槽位。实测表现为 zindex 末位 100 而实际只
                 * 写出 86 个点。
                 */
                ctx->keep[i] = 0u;
                continue;
            }
            fl = (nf_u8)(fl | NF_DVF_SURF_OUTER);
        }

        const nf_u32 bits = ctx->filtered_zmap[ctx->cell_of_point[i]];
        if (bits) {
            nf_u32 top = 31u;
            while (top > 0u && !(bits & (1u << top))) --top;
            const nf_i32 zb = z_bin_of(ctx, points[i].cz);
            if (zb >= 0 && (nf_u32)zb == top) fl = (nf_u8)(fl | NF_DVF_SURF_TOP);
        }
        if (points[i].n_eff < 1.5f) fl = (nf_u8)(fl | NF_DVF_SURF_DYNAMIC);

        ctx->flags_of_pt[i] = fl;
        ++n4;
    }
    st.n_after_shell = n4;

    /* ============ 阶段 5: 配额熔断 + 计数排序 ============ */
    const nf_u32 cap = ctx->cfg.max_output_points;

    /*
     * **截断优先级：近场优先**（评审 §5.4 指出的缺失）。
     *
     * 随机截断会让 5m 外的点挤掉 1m 处的障碍物 —— 后者才是碰撞相关的。
     * 做法是对 r^2 二分找阈值，逐步收紧到数量 <= 配额。
     * 用 r^2 而不是 r 排序，省一次开方。
     */
    nf_u32 n5 = n4;
    if (n5 > cap) {
        st.truncated = 1u;
        nf_f32 hi = 0.0f;
        for (nf_u32 i = 0; i < n_points; ++i) {
            if (!ctx->keep[i]) continue;
            const nf_f32 r2 = points[i].cx * points[i].cx + points[i].cy * points[i].cy;
            if (r2 > hi) hi = r2;
        }
        nf_f32 lo = 0.0f;
        for (int iter = 0; iter < 40; ++iter) {
            const nf_f32 mid = 0.5f * (lo + hi);
            nf_u32 cnt = 0;
            for (nf_u32 i = 0; i < n_points; ++i) {
                if (!ctx->keep[i]) continue;
                const nf_f32 r2 = points[i].cx * points[i].cx + points[i].cy * points[i].cy;
                if (r2 <= mid) ++cnt;
            }
            if (cnt > cap) hi = mid; else lo = mid;
            if (hi - lo < 0.5f) break;
        }
        n5 = 0;
        for (nf_u32 i = 0; i < n_points; ++i) {
            if (!ctx->keep[i]) continue;
            const nf_f32 r2 = points[i].cx * points[i].cx + points[i].cy * points[i].cy;
            if (r2 <= lo) ++n5;
            else ctx->keep[i] = 0u;
        }
        /* 阈值法可能因并列多出几个，硬补到正好 cap */
        for (nf_u32 i = 0; i < n_points && n5 > cap; ++i) {
            if (!ctx->keep[i]) continue;
            ctx->keep[i] = 0u;
            --n5;
        }
    }
    st.n_output = n5;

    /* 前缀和：按 cell 计数，下游用 zindex[cell]..zindex[cell+1] 取柱体 */
    for (nf_u32 i = 0; i < n_points; ++i) {
        if (!ctx->keep[i]) continue;
        ++ctx->cursor[ctx->cell_of_point[i] + 1u];
    }
    for (nf_u32 cell = 0; cell < ctx->n_cells; ++cell)
        ctx->cursor[cell + 1u] += ctx->cursor[cell];

    /**
     * **zmap 必须只反映"真的有输出点"的层**。
     *
     * 直接 memcpy filtered_zmap 会带进一类问题：某层在剥离后还在位图里，
     * 但它的所有点都被极坐标外壳或配额截断剔除了。下游按 zmap 的 bit
     * 去找这个层的点，会拿到 zindex[cell] == zindex[cell+1] 的空区间 ——
     * 严格说不是越界，但会让下游误以为"这层有点"。
     *
     * 正确做法：从最终 keep 重新 OR 一遍位图，保证 zmap 与 zindex 同源。
     */
    std::memset(zmap_out, 0, (size_t)ctx->n_cells * sizeof(nf_u32));
    for (nf_u32 i = 0; i < n_points; ++i) {
        if (!ctx->keep[i]) continue;
        const nf_i32 zb = z_bin_of(ctx, points[i].cz);
        if (zb < 0) continue;
        zmap_out[ctx->cell_of_point[i]] |= (1u << (nf_u32)zb);
    }
    std::memcpy(zindex_out, ctx->cursor, (size_t)(ctx->n_cells + 1u) * sizeof(nf_u32));
    st.n_cells = ctx->cursor[ctx->n_cells];

    /*
     * 两遍计数排序。
     *
     * fill[] 用 malloc 而不是持久缓冲：这里只在**超配额**时才会走到
     * 排序，正常帧不该为它付常驻内存。用完立刻释放，不构成逐帧分配。
     */
    nf_u32 *fill = (nf_u32 *)std::malloc((size_t)(ctx->n_cells + 1u) * sizeof(nf_u32));
    if (!fill) return NF_E_ENGINE_OOM;
    std::memcpy(fill, ctx->cursor, (size_t)(ctx->n_cells + 1u) * sizeof(nf_u32));

    for (nf_u32 i = 0; i < n_points; ++i) {
        if (!ctx->keep[i]) continue;
        const nf_u32 cell = ctx->cell_of_point[i];
        const nf_u32 pos = fill[cell]++;
        if (pos >= cap) continue;

        NFDownstreamPoint &o = pts_out[pos];
        o.x_mm = clamp_i16(points[i].cx);
        o.y_mm = clamp_i16(points[i].cy);
        o.z_mm = clamp_i16(points[i].cz);
        o.source_camera = points[i].source_mask;
        o.spatial_uncertainty_cm = points[i].sigma_cm;
        /*
         * 存在概率：`NFVoxelEntry` 里没有独立的存在概率字段，
         * 用 n_eff 归一化作为代理 —— CMA 的有效观测数越多说明
         * 被反复观测到，置信度越高。饱和上限取 8（V_N_EFF_MAX 默认）。
         */
        o.existence_probability = prob_to_u8(points[i].n_eff / 8.0f);
        o.surface_flags = ctx->flags_of_pt[i];
    }
    std::free(fill);

    if (stats) *stats = st;
    return NF_OK;
}

NFStatus nf_voxel_filter_to_downstream(const NFVoxelFilterCtx *ctx,
                                       const NFDownstreamPoint *src,
                                       nf_u32 n,
                                       void *dst, nf_u32 dst_bytes,
                                       nf_u32 *out_bytes) {
    if (!ctx || !src || !dst) return NF_E_PARAM_NULL;
    const nf_u32 need = n * (nf_u32)sizeof(NFDownstreamPoint);
    if (dst_bytes < need) return NF_E_PARAM_RANGE;
    std::memcpy(dst, src, need);
    if (out_bytes) *out_bytes = need;
    return NF_OK;
}
