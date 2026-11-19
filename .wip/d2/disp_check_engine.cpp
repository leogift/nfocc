/**
 * @file disp_check_engine.cpp
 * @brief D2 双向一致性中**依赖引擎**的那一半。
 *
 * 拆成独立文件的原因：`nf_disp_check_combine`（纯逐点比对）属于 core，
 * 交叉编译（无 OpenCV）也要能编；`nf_disp_check_run` 要调 SGBM，
 * 只能在 NF_SGBM_ON 下构建。混在一起会让 aarch64 构建链接失败。
 */
#include "nfocc/nfocc_disp_check.h"
#include "nfocc/nfocc_default.h"

#include <cstdlib>
#include <cstring>

extern "C" {
NFStatus nf_sgbm_engine_init(NFEngineCtx **ctx, const NFEngineConfig *cfg);
NFStatus nf_sgbm_engine_run(NFEngineCtx *ctx, const NFImageView *l,
                            const NFImageView *r,
                            const NFDisparityRange *range, NFDisparityMap *out);
NFStatus nf_sgbm_engine_deinit(NFEngineCtx *ctx);
}

extern "C" {

NFStatus nf_disp_check_run(NFEngineCtx *eng,
                           const NFImageView *left,
                           const NFImageView *right,
                           const NFDisparityRange *range,
                           NFDisparityMap *fwd,
                           nf_u8 *out_keep,
                           const NFDispCheckConfig *cfg,
                           nf_u8 *out_fallback) {
    if (!eng || !left || !right || !range || !fwd || !out_keep) return NF_E_PARAM_NULL;
    if (out_fallback) *out_fallback = 0;

    const nf_u32 w = left->width;
    const nf_u32 h = left->height;
    const size_t np = (size_t)w * h;

    /* ---- 正向匹配 ---- */
    NFStatus st = nf_sgbm_engine_run(eng, left, right, range, fwd);
    if (st != NF_OK) return st;

    /* 关闭 D2：全部放行 */
    if (cfg && !cfg->enable) {
        std::memset(out_keep, 1, np);
        return NF_OK;
    }

    /* ---- 反向匹配：左右交换 ---- */
    /* 核心库禁 STL，用 malloc（与 voxel.cpp 的 ExportItem 一致做法） */
    {
        nf_f32 *d_bwd = (nf_f32 *)std::malloc(np * sizeof(nf_f32));
        nf_u8  *v_bwd = (nf_u8 *)std::malloc(np);
        nf_f32 *c_bwd = (nf_f32 *)std::malloc(np * sizeof(nf_f32));
        if (!d_bwd || !v_bwd || !c_bwd) {
            std::free(d_bwd); std::free(v_bwd); std::free(c_bwd);
            std::memset(out_keep, 1, np);
            if (out_fallback) *out_fallback = 1;
            return NF_OK;   /* 不阻断，标记 fallback */
        }
        std::memset(v_bwd, 0, np);

        NFDisparityMap bmap;
        std::memset(&bmap, 0, sizeof(bmap));
        bmap.version = NF_DISPARITY_MAP_VERSION;
        bmap.width = w; bmap.height = h;
        bmap.stride_f = (nf_i32)w;
        bmap.stride_b = (nf_i32)w;
        bmap.disp = d_bwd; bmap.valid = v_bwd; bmap.conf = c_bwd;

        /* 交换左右：右图当左图 */
        st = nf_sgbm_engine_run(eng, right, left, range, &bmap);
        if (st != NF_OK || bmap.width != w) {
            std::free(d_bwd); std::free(v_bwd); std::free(c_bwd);
            std::memset(out_keep, 1, np);
            if (out_fallback) *out_fallback = 1;
            return NF_OK;
        }

        nf_disp_check_combine(fwd->disp, fwd->valid, d_bwd, v_bwd,
                              w, h, cfg, out_keep, nullptr, nullptr);

        std::free(d_bwd); std::free(v_bwd); std::free(c_bwd);
    }
    return NF_OK;
}

} /* extern "C" */
