/**
 * @file engine_model_null.cpp
 * @brief 模型引擎的接口骨架(无 onnxruntime 时).
 *
 * 存在意义: SPEC-ENGINE X-3 要求 SGBM 与模型两个实现**可互换**。
 * 若上层离开 SGBM 就跑不动, 说明接口没做兼容。
 * 本实现全部返回无效像素(valid=0), 用于验证"换引擎不改上层"。
 */
#include "nfocc/nfocc_engine.h"
#include "engine_internal.h"

#include <cstdlib>
#include <cstring>

struct NFEngineCtx {
    NFEngineConfig cfg;
};

extern "C" {

NFStatus nf_model_engine_init(NFEngineCtx **ctx, const NFEngineConfig *cfg) {
    if (!ctx || !cfg) return NF_E_PARAM_NULL;
    if (cfg->version != NF_ENGINE_CONFIG_VERSION) return NF_E_PARAM_VERSION;
    if (cfg->kind != NF_ENGINE_MODEL) return NF_E_ENGINE_UNSUPPORTED;
    NFEngineCtx *c = (NFEngineCtx *)std::malloc(sizeof(NFEngineCtx));
    if (!c) return NF_E_ENGINE_OOM;
    c->cfg = *cfg;
    *ctx = c;
    return NF_OK;
}

NFStatus nf_model_engine_run(NFEngineCtx *ctx, const NFImageView *left,
                       const NFImageView *right, const NFDisparityRange *range,
                       NFDisparityMap *out) {
    if (!ctx || !left || !right || !range || !out) return NF_E_PARAM_NULL;
    if (!out->disp || !out->valid || !out->conf) return NF_E_PARAM_NULL;
    /* 骨架: 全无效。**不得用 disp=0 冒充有效**(E-6) */
    const size_t n = (size_t)out->width * out->height;
    std::memset(out->disp, 0, n * sizeof(nf_f32));
    std::memset(out->valid, 0, n * sizeof(nf_u8));
    std::memset(out->conf, 0, n * sizeof(nf_f32));
    return NF_OK;
}

NFStatus nf_model_engine_deinit(NFEngineCtx *ctx) {
    if (!ctx) return NF_E_PARAM_NULL;
    std::free(ctx);
    return NF_OK;
}

} /* extern "C" */
