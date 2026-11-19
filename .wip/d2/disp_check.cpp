/**
 * @file disp_check.cpp
 * @brief D2 双向一致性检查实现。
 */
#include "nfocc/nfocc_disp_check.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_geometry.h"

#include <cmath>
#include <cstring>

extern "C" {

NFStatus nf_disp_check_combine(const nf_f32 *d_fwd, const nf_u8 *valid_fwd,
                               const nf_f32 *d_bwd, const nf_u8 *valid_bwd,
                               nf_u32 w, nf_u32 h,
                               const NFDispCheckConfig *cfg,
                               nf_u8 *out_keep,
                               nf_u32 *out_count_occluded,
                               nf_u32 *out_count_kept) {
    if (!d_fwd || !valid_fwd || !out_keep) return NF_E_PARAM_NULL;
    if (w == 0 || h == 0) return NF_E_PARAM_RANGE;

    const float tau    = cfg ? cfg->tau : NF_OCC_TAU;
    const float lambda = cfg ? cfg->lambda : NF_OCC_LAMBDA;
    const float w_min  = cfg ? cfg->w_min : 0.1f;

    nf_u32 occ = 0, kept = 0;

    for (nf_u32 y = 0; y < h; ++y) {
        for (nf_u32 x = 0; x < w; ++x) {
            const size_t i = (size_t)y * w + x;
            if (!valid_fwd[i]) { out_keep[i] = 0; ++occ; continue; }

            /* 关闭 D2 时不看反向 */
            if (cfg && !cfg->enable) { out_keep[i] = 1; ++kept; continue; }

            if (!d_bwd || !valid_bwd) { out_keep[i] = 1; ++kept; continue; }

            /*
             * 沿正向视差在右图里找对应像素，取该处的反向视差。
             *
             * 方向由 mirror 决定（见头文件说明）：
             *   未镜像: xr = x - d
             *   已镜像: xr = (w-1-x) + d
             */
            /* 镜像时对应点 x_ref = w-1-x+d，否则 x-d */
            const float xr_f = (cfg && cfg->mirror)
                             ? ((float)(w - 1u - x) + d_fwd[i])
                             : ((float)x - d_fwd[i]);
            if (xr_f < 0.0f || xr_f >= (float)w) { out_keep[i] = 0; ++occ; continue; }

            const int xr = (int)(xr_f + 0.5f);
            if (xr < 0 || xr >= (int)w) { out_keep[i] = 0; ++occ; continue; }

            const size_t ib = (size_t)y * w + (size_t)xr;
            if (!valid_bwd[ib]) { out_keep[i] = 0; ++occ; continue; }

            /*
             * **不取负**。
             *
             * 几何上 d_bwd = xR - xL = -d_fwd，但 SGBM 的输出恒为
             * 正值（raw 是 CV_16S 的 1/16 px，且实现里 `r > 0` 才算有效，
             * 见 engine_sgbm.cpp:147），符号信息在引擎里就丢了。
             * 所以反向图给出的数值已经是 |d|，直接比即可。
             *
             * 取负会让残差变成 d_fwd + d_bwd，几乎全部判成遮挡 ——
             * 这正是 test_disp_check 的 dc_sign_convention 守住的。
             */
            const float d_back = d_bwd[ib];
            const float wgt = nf_occlusion_weight(d_fwd[i], d_back, tau, lambda);

            if (wgt < w_min) { out_keep[i] = 0; ++occ; }
            else             { out_keep[i] = 1; ++kept; }
        }
    }

    if (out_count_occluded) *out_count_occluded = occ;
    if (out_count_kept)     *out_count_kept = kept;
    return NF_OK;
}

} /* extern "C" */
