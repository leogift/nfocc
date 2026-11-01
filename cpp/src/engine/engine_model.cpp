/**
 * @file engine_model.cpp
 * @brief 模型引擎封装(onnxruntime)。
 *
 * ## 当前状态: 骨架, 尚未接入模型
 * SPEC-ENGINE X-3 要求 SGBM 与模型两个实现可互换。本文件提供**接口完整**的
 * 实现, 但模型推理部分留待 PH-4:
 *   - 模型权重未提供(需先训练/导出)
 *   - 输入预处理(视口图 -> 网络输入)依赖具体网络结构
 *
 * 未接模型时, run() 输出**全无效**(valid=0), 不返回错误 ——
 * 这样上层可以照常跑完链路, 并在指标上看到"引擎无产出"而不是崩溃。
 * **绝不用 disp=0 冒充有效**(E-6)。
 *
 * ## onnxruntime 接入要点(实测记录)
 *   - Session 创建是线程安全的, 但 Run() 不是; 每线程一个 Session
 *   - 输入张量名由模型决定, 不能硬编码
 *   - Ort::Value 的生命周期必须覆盖 Run()
 */
#include "nfocc/nfocc_engine.h"
#include "engine_internal.h"

#include <cstdlib>
#include <cstring>

#ifdef NF_WITH_ORT
#include <onnxruntime_cxx_api.h>
#include <memory>
#include <vector>
#endif

struct NFEngineCtx {
    NFEngineConfig cfg;
#ifdef NF_WITH_ORT
    std::unique_ptr<Ort::Env> env;
    std::unique_ptr<Ort::Session> session;
#endif
};

extern "C" {

NFStatus nf_model_engine_init(NFEngineCtx **ctx, const NFEngineConfig *cfg) {
    if (!ctx || !cfg) return NF_E_PARAM_NULL;
    if (cfg->version != NF_ENGINE_CONFIG_VERSION) return NF_E_PARAM_VERSION;
    if (cfg->kind != NF_ENGINE_MODEL) return NF_E_ENGINE_UNSUPPORTED;

    NFEngineCtx *c = (NFEngineCtx *)std::malloc(sizeof(NFEngineCtx));
    if (!c) return NF_E_ENGINE_OOM;
    c->cfg = *cfg;
#ifdef NF_WITH_ORT
    /* 模型载入留待 PH-4: 当前没有可用权重, 不创建 Session。
       保留 env 以便后续接入。 */
#endif
    *ctx = c;
    return NF_OK;
}

NFStatus nf_model_engine_run(NFEngineCtx *ctx, const NFImageView *left,
                       const NFImageView *right, const NFDisparityRange *range,
                       NFDisparityMap *out) {
    if (!ctx || !left || !right || !range || !out) return NF_E_PARAM_NULL;
    if (out->version != NF_DISPARITY_MAP_VERSION) return NF_E_PARAM_VERSION;
    if (!out->disp || !out->valid || !out->conf) return NF_E_PARAM_NULL;

    /* S-2: 容量不足必须拒绝且不写输出 —— 与 SGBM 实现保持同一语义 */
    if ((nf_f32)ctx->cfg.max_disp_px < range->d_max - 1e-6f) {
        return NF_E_ENGINE_RANGE;
    }

#ifdef NF_WITH_ORT
    /* 模型未接入: 明确返回"不支持", 而不是静默给全 0。
       NF_E_ENGINE_UNSUPPORTED 让上层能区分"模型缺失"与"模型跑出无结果"。 */
    return NF_E_ENGINE_UNSUPPORTED;
#else
    const size_t n = (size_t)out->width * out->height;
    std::memset(out->disp, 0, n * sizeof(nf_f32));
    std::memset(out->valid, 0, n * sizeof(nf_u8));
    std::memset(out->conf, 0, n * sizeof(nf_f32));
    return NF_OK;
#endif
}

NFStatus nf_model_engine_deinit(NFEngineCtx *ctx) {
    if (!ctx) return NF_E_PARAM_NULL;
#ifdef NF_WITH_ORT
    /* unique_ptr 负责释放 */
#endif
    std::free(ctx);
    return NF_OK;
}

} /* extern "C" */
