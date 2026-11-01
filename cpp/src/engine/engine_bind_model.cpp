/**
 * @file engine_bind_model.cpp
 * @brief 把模型引擎注册到 pipeline 的晚绑定表。
 *
 * 与 engine_bind_sgbm.cpp 分开是**必须的**: 两个 bind 函数都取 nf_engine_init
 * 的地址, 而该符号在 SGBM 库与模型库中各有一份。写在同一个 TU 里会让
 * "注册了哪个引擎"取决于链接顺序(实测导致 STEREO 路径返回 UNSUPPORTED)。
 */
#include "nfocc/nfocc_engine_bind.h"
#include "nfocc/nfocc_engine.h"
#include "engine_internal.h"

struct NFEngineOpsMirror {
    NFStatus (*init)(NFEngineCtx **, const NFEngineConfig *);
    NFStatus (*run)(NFEngineCtx *, const NFImageView *, const NFImageView *,
                    const NFDisparityRange *, NFDisparityMap *);
    NFStatus (*deinit)(NFEngineCtx *);
};

extern "C" NFStatus nf_pipeline_bind_model(void) {
    static const NFEngineOpsMirror ops = {
        nf_model_engine_init, nf_model_engine_run, nf_model_engine_deinit
    };
    return nf_pipeline_bind_engine(&ops);
}
