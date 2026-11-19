/**
 * @file engine_util.cpp
 * @brief 视差图与引擎的**无依赖**辅助函数。
 *
 * ## 为什么放在核心库
 * nf_dmap_alloc 原先写在 engine_sgbm.cpp 里, 导致核心库(pipeline)调用它时
 * 必须链接引擎库 —— 直接违反 ADR-006「核心库零第三方依赖」,
 * 实测表现是单独链接 libnfocc_core.a 时报 undefined reference。
 * 这些函数只用 malloc, 与 OpenCV 无关, 应属核心库。
 */
#include "nfocc/nfocc_engine.h"

#include <cstdlib>
#include <cstring>

NFStatus nf_dmap_alloc(NFDisparityMap *dm, nf_u32 w, nf_u32 h) {
    if (!dm) return NF_E_PARAM_NULL;
    if (w == 0 || h == 0) return NF_E_PARAM_RANGE;
    const size_t n = (size_t)w * (size_t)h;
    dm->disp  = (nf_f32 *)std::calloc(n, sizeof(nf_f32));
    dm->valid = (nf_u8  *)std::calloc(n, sizeof(nf_u8));
    dm->conf  = (nf_f32 *)std::calloc(n, sizeof(nf_f32));
    if (!dm->disp || !dm->valid || !dm->conf) {
        std::free(dm->disp); std::free(dm->valid); std::free(dm->conf);
        dm->disp = nullptr; dm->valid = nullptr; dm->conf = nullptr;
        return NF_E_ENGINE_OOM;
    }
    dm->version = NF_DISPARITY_MAP_VERSION;
    dm->width = w;
    dm->height = h;
    dm->stride_f = (nf_i32)w;
    dm->stride_b = (nf_i32)w;
    return NF_OK;
}

void nf_dmap_destroy(NFDisparityMap *dm) {
    if (!dm) return;
    std::free(dm->disp);
    std::free(dm->valid);
    std::free(dm->conf);
    dm->disp = nullptr; dm->valid = nullptr; dm->conf = nullptr;
    dm->width = dm->height = 0;
}

extern "C" {

nf_f32 nf_engine_expected_valid_ratio(nf_u32 width, nf_i32 max_disp) {
    if (width == 0) return 0.0f;
    const float r = ((float)width - (float)max_disp) / (float)width;
    return r > 0.0f ? r : 0.0f;
}

NFStatus nf_engine_mem_bytes(const NFEngineConfig *cfg, nf_u32 w, nf_u32 h,
                             nf_u64 *out_bytes) {
    if (!cfg || !out_bytes) return NF_E_PARAM_NULL;
    if (cfg->max_disp_px <= 0 || w == 0 || h == 0) return NF_E_PARAM_RANGE;
    /* S-4: cost volume = numDisp * W * H * 2 B (int16) */
    *out_bytes = (nf_u64)cfg->max_disp_px * (nf_u64)w * (nf_u64)h * 2ull;
    return NF_OK;
}

} /* extern "C" */
