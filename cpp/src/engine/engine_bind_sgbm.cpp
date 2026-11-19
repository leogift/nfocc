/**
 * @file engine_bind_sgbm.cpp
 * @brief 把 SGBM 引擎注册到 pipeline 的晚绑定表。
 *
 * ## 为什么这里不能共用一份实现
 * 最初 `nf_pipeline_bind_sgbm()` 与 `nf_pipeline_bind_model()` 写在同一个
 * `engine_bind.cpp` 里, 两者都取 `nf_engine_init` 的地址 —— 但那个符号在
 * SGBM 库和模型库中各有一份, 于是**注册进 pipeline 的是哪个引擎取决于链接器
 * 挑中了谁**。实际现象: 主程序同时链接两个引擎库, 结果跑 STEREO 时返回
 * NF_E_ENGINE_UNSUPPORTED —— 因为最终注册的是模型引擎, 而模型引擎对
 * kind==SGBM 的配置直接拒绝。
 *
 * 修法: 每个引擎库各自提供自己的 bind 符号, 只引用**本库**的 nf_engine_*。
 * 目标名不同 -> 不会互相覆盖。
 */
#include "nfocc/nfocc_engine_bind.h"
#include "nfocc/nfocc_engine.h"
#include "engine_internal.h"

/** 与 pipeline.cpp 的 NFEngineOps 布局一致(纯 POD, 三个函数指针) */
struct NFEngineOpsMirror {
    NFStatus (*init)(NFEngineCtx **, const NFEngineConfig *);
    NFStatus (*run)(NFEngineCtx *, const NFImageView *, const NFImageView *,
                    const NFDisparityRange *, NFDisparityMap *);
    NFStatus (*deinit)(NFEngineCtx *);
};

extern "C" NFStatus nf_pipeline_bind_sgbm(void) {
    static const NFEngineOpsMirror ops = {
        nf_sgbm_engine_init, nf_sgbm_engine_run, nf_sgbm_engine_deinit
    };
    return nf_pipeline_bind_engine(&ops);
}
