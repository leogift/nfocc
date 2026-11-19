/**
 * @file engine_sgbm.cpp
 * @brief SGBM 引擎实现 (OpenCV)。
 *
 * 实测过的坑(见 Python 原型 nfocc_ph1/engine.py 注释):
 *  1. `compute()` 返回 CV_16S, 单位 1/16 px, 必须 /16 得像素视差。
 *  2. SGBM 用 <=0 表示失败, 但**真实视差 0 也是合法的** —— 不能一律当无效。
 *     本实现用 "raw > 0" 判无效, 并在 d_min>0 时额外过滤 (E-6)。
 *  3. 左边界 numDisparities 列没有足够搜索窗口 -> 必然无效。
 *     有效覆盖 = (W - numDisp)/W (S-3)。
 *  4. numDisparities 必须是 16 的整数倍。
 *  5. numDisparities < d_max 时必须**拒绝且不写输出**(S-2) —— 实测过
 *     真视差 180px 配 numDisp=128 时中位误差 89px 但函数"成功返回"。
 */
#include "nfocc/nfocc_engine.h"
#include "engine_internal.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <cstdlib>
#include <cstring>
#include <cmath>

struct NFEngineCtx {
    NFEngineConfig cfg;
    cv::Ptr<cv::StereoSGBM> sgbm;
};

extern "C" {





NFStatus nf_sgbm_engine_init(NFEngineCtx **ctx, const NFEngineConfig *cfg) {
    if (!ctx || !cfg) return NF_E_PARAM_NULL;
    if (cfg->version != NF_ENGINE_CONFIG_VERSION) return NF_E_PARAM_VERSION;
    if (cfg->max_disp_px <= 0) return NF_E_PARAM_RANGE;
    /* OpenCV 要求 16 的整数倍 */
    if (cfg->max_disp_px % 16 != 0) return NF_E_PARAM_RANGE;
    if (cfg->downscale != 1 && cfg->downscale != 2 && cfg->downscale != 4)
        return NF_E_PARAM_RANGE;
    if (cfg->kind != NF_ENGINE_SGBM) return NF_E_ENGINE_UNSUPPORTED;

    NFEngineCtx *c = (NFEngineCtx *)std::malloc(sizeof(NFEngineCtx));
    if (!c) return NF_E_ENGINE_OOM;
    c->cfg = *cfg;

    const int bs = (cfg->block_size > 0) ? cfg->block_size : 5;
    /* **不使用 try/catch 捕获 OpenCV 异常**。
     *
     * 实测: Homebrew 的 OpenCV 5.0 用 C++17 编译, 而本工程是 C++11;
     * 跨标准边界的异常对象 ABI 不一致, catch(const cv::Exception&) 匹配失败
     * (lldb 显示停在 nf_sgbm_engine_init 的 __cxa_begin_catch 上, 进程 SIGTRAP)。
     * 给本文件加 -fexceptions -frtti 都不能修好。
     *
     * 正确做法: **在调用前把所有能触发 OpenCV 异常的条件自查掉**,
     * 保证 cv::StereoSGBM::create 不会抛。下面每个 return 都对应
     * OpenCV 源码里的一个 CV_Assert。
     */
    if (cfg->max_disp_px % 16 != 0) return NF_E_PARAM_RANGE;      /* numDisp%16 */
    if (cfg->max_disp_px <= 0)      return NF_E_PARAM_RANGE;
    if (bs < 1 || (bs % 2) == 0)    return NF_E_PARAM_RANGE;      /* blockSize 奇数 */

    c->sgbm = cv::StereoSGBM::create(
        0,                                   /* minDisparity */
        cfg->max_disp_px,                    /* numDisparities */
        bs,                                  /* blockSize */
        8 * bs * bs,                         /* P1 */
        32 * bs * bs,                        /* P2 */
        1,                                   /* disp12MaxDiff */
        (cfg->uniqueness > 0) ? cfg->uniqueness : 10,
        (cfg->speckle_window >= 0) ? cfg->speckle_window : 100,
        2,                                   /* speckleRange */
        63,                                  /* preFilterCap */
        cv::StereoSGBM::MODE_SGBM_3WAY);
    if (!c->sgbm) { std::free(c); return NF_E_ENGINE_OOM; }
    *ctx = c;
    return NF_OK;
}

NFStatus nf_sgbm_engine_deinit(NFEngineCtx *ctx) {
    if (!ctx) return NF_E_PARAM_NULL;
    std::free(ctx);
    return NF_OK;
}

NFStatus nf_sgbm_engine_run(NFEngineCtx *ctx,
                       const NFImageView *left,
                       const NFImageView *right,
                       const NFDisparityRange *range,
                       NFDisparityMap *out) {
    if (!ctx || !left || !right || !range || !out) return NF_E_PARAM_NULL;
    if (left->version != NF_IMAGE_VIEW_VERSION) return NF_E_PARAM_VERSION;
    if (range->version != NF_DISPARITY_RANGE_VERSION) return NF_E_PARAM_VERSION;
    if (out->version != NF_DISPARITY_MAP_VERSION) return NF_E_PARAM_VERSION;
    if (!left->data || !right->data) return NF_E_PARAM_NULL;
    if (left->width != right->width || left->height != right->height)
        return NF_E_PARAM_RANGE;
    if (out->width != left->width || out->height != left->height)
        return NF_E_PARAM_RANGE;
    if (!out->disp || !out->valid || !out->conf) return NF_E_PARAM_NULL;

    /* S-2: 容量不足必须**拒绝且不写输出** */
    if ((nf_f32)ctx->cfg.max_disp_px < range->d_max - 1e-6f) {
        return NF_E_ENGINE_RANGE;
    }

    const int w = (int)left->width;
    const int h = (int)left->height;

    /* 零拷贝包装: 直接用调用方的 stride */
    cv::Mat ml(h, w, CV_8UC1, (void *)left->data,  (size_t)left->stride);
    cv::Mat mr(h, w, CV_8UC1, (void *)right->data, (size_t)right->stride);

    /* compute 前把能触发的条件自查: 尺寸一致、类型 8UC1、非空。
       这些都在上面已校验, 这里再确认一次以免 OpenCV 内部 CV_Assert。 */
    if (ml.size() != mr.size() || ml.type() != CV_8UC1) {
        nf_log_msg(NF_LOG_ERROR, "输入不合法: L=%dx%d type=%d R=%dx%d type=%d",
                   ml.cols, ml.rows, ml.type(), mr.cols, mr.rows, mr.type());
        return NF_E_PARAM_RANGE;
    }

    cv::Mat raw;
    ctx->sgbm->compute(ml, mr, raw);   /* CV_16S, 单位 1/16 px */
    if (raw.empty()) {
        nf_log_msg(NF_LOG_ERROR, "SGBM compute 返回空: in=%dx%d out=%dx%d",
                   (int)left->width, (int)left->height, (int)out->width, (int)out->height);
        return NF_E_ENGINE_UNSUPPORTED;
    }

    const int nd = ctx->cfg.max_disp_px;
    const float inv16 = 1.0f / 16.0f;

    for (int y = 0; y < h; ++y) {
        const nf_i16 *row = raw.ptr<nf_i16>(y);
        nf_f32 *drow = out->disp + (size_t)y * out->stride_f;
        nf_u8  *vrow = out->valid + (size_t)y * out->stride_b;
        nf_f32 *crow = out->conf + (size_t)y * out->stride_b;
        const nf_i32 x0 = left->valid_x0 ? left->valid_x0[y] : 0;

        for (int x = 0; x < w; ++x) {
            const nf_i16 r = row[x];
            /* SGBM 用 <=0 表示失败。**真实视差 0 属于边界情形**,
               这里按 raw>0 判有效, 与 d_min 过滤配合 (E-6)。 */
            bool ok = (r > 0);
            float d = (float)r * inv16;
            if (ok && d < range->d_min - 1e-6f) ok = false;
            if (ok && d > range->d_max + 1e-6f) ok = false;
            /* 左边界 numDisp 列无搜索窗口 (S-3) */
            if (ok && x < nd) ok = false;
            /* 每行有效起点 (E-3) */
            if (ok && x < x0) ok = false;

            if (ok) {
                drow[x] = d;
                vrow[x] = 1;
                crow[x] = 1.0f;      /* SGBM 不输出置信度, 填 1.0 */
            } else {
                drow[x] = 0.0f;
                vrow[x] = 0;
                crow[x] = 0.0f;
            }
        }
    }
    return NF_OK;
}

} /* extern "C" */
