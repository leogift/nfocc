/**
 * @file nfocc_engine_bind.h
 * @brief 把引擎实现注册到 pipeline 的晚绑定表。
 *
 * 为什么需要这一层: SPEC-BUILD §3 / ADR-006 要求**核心库不链接 OpenCV**
 * (V-BUILD-2 用 ldd 检查)。pipeline 属于核心库, 但它要跑引擎,
 * 所以用函数指针表倒置依赖: 核心库不知道引擎来自哪里, 由宿主程序注册。
 */
#ifndef NFOCC_ENGINE_BIND_H
#define NFOCC_ENGINE_BIND_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** pipeline 侧: 接收 ops(内部结构, 传指针即可) */
NFStatus nf_pipeline_bind_engine(const void *ops);

/** 引擎库侧: 把当前编译进去的引擎注册进去 */
NFStatus nf_pipeline_bind_sgbm(void);
NFStatus nf_pipeline_bind_model(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_ENGINE_BIND_H */
