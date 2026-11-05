/**
 * @file nfocc_engine.h
 * @brief 视差引擎接口。规范: docs/SPEC-ENGINE.md
 *
 * **上层只能通过本接口与引擎交互, 不得感知具体实现**(SPEC-ENGINE X-3)。
 * SGBM 与模型两个实现必须可互换。
 *
 * 关键契约(每条都有验收):
 *   S-2  numDisparities < range.d_max -> 返回 NF_E_ENGINE_RANGE, **不写输出**
 *   S-3  有效覆盖 = (W - numDisp)/W
 *   S-4  内存 = numDisp * W * H * 2 B
 *   E-6  无效像素必须由 valid 掩码标出, **不得用 disp=0 表示无效**
 *   E-8  同输入两次调用结果逐位相同(确定性)
 */
#ifndef NFOCC_ENGINE_H
#define NFOCC_ENGINE_H

#include "nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NF_ENGINE_SGBM  = 0,
    NF_ENGINE_MODEL = 1
} NFEngineKind;

/** 视差范围 (SPEC-ICD §4.1) */
typedef struct {
    nf_u32 version;     /**< = NF_DISPARITY_RANGE_VERSION */
    nf_f32 d_min;       /**< px, 通常 0 */
    nf_f32 d_max;       /**< px, 由基线 b 与 Z_min 反推 */
    nf_f32 b_mm;        /**< 基线, 仅用于三角化, 引擎不用 */
} NFDisparityRange;

/** 视差图 (SPEC-ICD §4.2) */
typedef struct {
    nf_u32 version;     /**< = NF_DISPARITY_MAP_VERSION */
    nf_f32 *disp;       /**< [h*w], px */
    nf_u8  *valid;      /**< [h*w], 1=有效 */
    nf_f32 *conf;       /**< [h*w], 0..1 */
    nf_u32 width, height;
    nf_i32 stride_f;    /**< 元素数/行 */
    nf_i32 stride_b;
} NFDisparityMap;

/** 引擎配置 (SPEC-ICD §4.3) */
typedef struct {
    nf_u32 version;     /**< = NF_ENGINE_CONFIG_VERSION */
    NFEngineKind kind;
    nf_i32 max_disp_px; /**< 必须 >= range.d_max */
    nf_i32 downscale;   /**< 1 | 2 | 4 */
    nf_f32 conf_thresh; /**< 仅 MODEL 用 */
    nf_i32 block_size;
    nf_i32 uniqueness;
    nf_i32 speckle_window;
} NFEngineConfig;

/** 引擎上下文(不透明) */
typedef struct NFEngineCtx NFEngineCtx;

/** 图像视图 (SPEC-ICD §2.1) */
typedef struct {
    nf_u32 version;
    const nf_u8 *data;
    nf_u32 width, height;
    nf_u32 stride;
    const nf_i32 *valid_x0;   /**< 每行有效起点; NULL 表示整行有效 */
    nf_u64 timestamp_us;
    nf_u32 frame_id;
} NFImageView;

/* ## 引擎实现的符号约定
 *
 * 同一进程里可能同时链接 SGBM 与模型两个引擎库。若两者都导出
 * `nf_engine_init`, 静态链接只会保留**先被满足的那一个** —— 实测
 * nfocc_run 同时链接两个库时, 注册进 pipeline 的恒是模型引擎,
 * 导致 STEREO 路径返回 NF_E_ENGINE_UNSUPPORTED。
 *
 * 因此: **各引擎库导出带后缀的内部符号**, 公共名 `nf_engine_*` 由
 * `nfocc_engine_bind.h` 的 bind 函数在运行时选择, 不在链接期绑定。

/*
 * 注意：**没有** `nf_engine_init/run/deinit` 这样的无后缀自由函数。
 *
 * 各引擎库导出带后缀的符号（`nf_sgbm_engine_*` / `nf_model_engine_*`），
 * 公共入口由 `nf_pipeline_bind_engine()` 在运行时选择，不在链接期绑定。
 *
 * 曾经这里声明过三个无后缀版本，但从未实现 —— 有工具用 `#define` 把它们
 * 宏替换成 SGBM 版本才编得过。那等于把"声明与实现不一致"藏在宏里，
 * 任何人照着头文件调用都会在链接期才失败。已删除。
 */


/** 预估内存(不分配), 供 init 期预算检查 (S-4) */
NFStatus nf_engine_mem_bytes(const NFEngineConfig *cfg, nf_u32 w, nf_u32 h,
                             nf_u64 *out_bytes);

/** 有效覆盖率的解析值: (W - numDisp)/W (S-3) */
nf_f32 nf_engine_expected_valid_ratio(nf_u32 width, nf_i32 max_disp);

/** 视差图分配/释放。**全部在 init 期分配**(SPEC-BUILD §5)。 */
NFStatus nf_dmap_alloc(NFDisparityMap *dm, nf_u32 w, nf_u32 h);
void     nf_dmap_destroy(NFDisparityMap *dm);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_ENGINE_H */
