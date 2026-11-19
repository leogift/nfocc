/**
 * @file nfocc_disp_check.h
 * @brief D2 双向一致性检查：一次跑正反两次匹配，输出遮挡/误匹配掩码。
 *
 * ## 为什么单独封装
 * `nf_occlusion_weight` 只做逐点的公式计算，但调用方还需要：
 *   1. 跑反向匹配（左图与右图交换）
 *   2. 处理反向视差的符号（SGBM 输出恒为正，反向时真实视差为负）
 *   3. 沿正向视差做 warping 查找反向视差（`d_bwd[x + d_fwd[x]]`）
 *   4. 逐点算权重并阈值化
 *
 * 这四步散在 pipeline 里会让主链难读，且容易在符号上出错 ——
 * 实测符号搞反会让中位误差从 36 px 涨到 85 px。
 *
 * ## 计算代价
 * SGBM 跑两遍。6 视口全开时耗时翻倍（35.5 ms -> 71 ms），
 * 仍在 200 ms 预算内但余量变小。可用 `only_views` 位图只在近场视口启用。
 */
#ifndef NFOCC_DISP_CHECK_H
#define NFOCC_DISP_CHECK_H

#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_DISP_CHECK_VERSION  1u

/** 双向一致性参数 */
typedef struct {
    nf_u32 version;         /**< = NF_DISP_CHECK_VERSION */
    nf_f32 tau;             /**< 容差 px，默认 NF_OCC_TAU (1.0) */
    nf_f32 lambda;          /**< 惩罚系数，默认 NF_OCC_LAMBDA (14.0) */
    nf_f32 w_min;           /**< 权重低于此值判为遮挡，默认 0.1 */
    nf_u8  enable;          /**< 0 = 只跑正向（关闭 D2） */
    /**
     * 1 = 输入图已做过水平镜像（pipeline 的 mirror 分支）。
     *
     * 镜像后同一物点在两图里的对应关系是 x_ref = w-1-x + d，
     * 而不是 x - d。传错方向会让绝大部分点被判遮挡。
     */
    nf_u8  mirror;
    nf_u8  _pad[2];
} NFDispCheckConfig;

/**
 * 跑正向 + 反向两次匹配，输出一致性掩码。
 *
 * @param eng       已初始化的引擎（正向与反向复用同一实例）
 * @param left      左图（更靠左的相机）
 * @param right     右图
 * @param range     视差范围（正向约定；反向内部自动处理符号）
 * @param fwd       输出：正向视差图（调用方已分配，尺寸同 left）
 * @param out_keep  输出：掩码 [w*h]，1 = 通过一致性检查，0 = 遮挡/误匹配。
 *                  调用方已分配。config.enable=0 时全部写 1。
 * @param cfg       配置
 *
 * @return NF_OK / 引擎错误码
 *
 * @note 反向匹配失败时**不阻断**：`out_keep` 全部写 1 并返回 NF_OK，
 *       同时在 `out.fallback` 置 1 —— 让上层能区分"没有遮挡"与"没做成检查"。
 */
NFStatus nf_disp_check_run(NFEngineCtx *eng,
                           const NFImageView *left,
                           const NFImageView *right,
                           const NFDisparityRange *range,
                           NFDisparityMap *fwd,
                           nf_u8 *out_keep,
                           const NFDispCheckConfig *cfg,
                           nf_u8 *out_fallback);

/** 正向/反向匹配逐点比对（供单元测试直接调用，不碰引擎） */
NFStatus nf_disp_check_combine(const nf_f32 *d_fwd, const nf_u8 *valid_fwd,
                               const nf_f32 *d_bwd, const nf_u8 *valid_bwd,
                               nf_u32 w, nf_u32 h,
                               const NFDispCheckConfig *cfg,
                               nf_u8 *out_keep,
                               nf_u32 *out_count_occluded,
                               nf_u32 *out_count_kept);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_DISP_CHECK_H */
