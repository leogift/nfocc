/**
 * @file engine_internal.h
 * @brief 引擎实现的内部符号声明（**非对外接口**）。
 *
 * ## 为什么需要带后缀的符号
 * SGBM 与模型两个引擎库可能被同一进程同时链接。若两者都导出
 * `nf_engine_init`, 静态链接只保留**先被满足的那一个** —— 实测:
 * nfocc_run 同时链两个库时, 注册进 pipeline 的恒是模型引擎,
 * STEREO 路径全部返回 NF_E_ENGINE_UNSUPPORTED, 且从日志完全看不出原因
 * (nf_engine_init 被调用了, 只是调错了实现)。
 *
 * 因此各引擎库导出**带后缀**的符号, 由各自的 bind 文件引用。
 * 公共名 `nf_engine_*` 只在"只链一个引擎库"的场合使用(如单元测试)。
 *
 * ## 归属
 * 本头文件不属于 `include/nfocc/` 对外 ICD 的一部分, 不应被外部包含。
 */
#ifndef NFOCC_ENGINE_INTERNAL_H
#define NFOCC_ENGINE_INTERNAL_H

#include "nfocc/nfocc_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SGBM 实现 (engine_sgbm.cpp) */
NFStatus nf_sgbm_engine_init  (NFEngineCtx **ctx, const NFEngineConfig *cfg);
NFStatus nf_sgbm_engine_run   (NFEngineCtx *ctx,
                               const NFImageView *left, const NFImageView *right,
                               const NFDisparityRange *range, NFDisparityMap *out);
NFStatus nf_sgbm_engine_deinit(NFEngineCtx *ctx);

/* 模型实现 (engine_model.cpp / engine_model_null.cpp) */
NFStatus nf_model_engine_init  (NFEngineCtx **ctx, const NFEngineConfig *cfg);
NFStatus nf_model_engine_run   (NFEngineCtx *ctx,
                                const NFImageView *left, const NFImageView *right,
                                const NFDisparityRange *range, NFDisparityMap *out);
NFStatus nf_model_engine_deinit(NFEngineCtx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_ENGINE_INTERNAL_H */
