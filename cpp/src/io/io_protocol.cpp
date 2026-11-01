/**
 * @file io_protocol.cpp
 * @brief 上下游消息解析/序列化。规范: docs/SPEC-UPSTREAM.md §2, SPEC-DOWNSTREAM.md §2
 *
 * ## 解析纪律
 * 跨进程来的数据一律当**不可信输入**: 每个长度、每个偏移都要验。
 * 一个伪造的 num_points 就能让下游越界读。这里所有校验失败都返回
 * NF_E_PARAM_RANGE 而不是继续解析。
 */
#include "nfocc/nfocc_io.h"
#include "nfocc/nfocc_default.h"

#include <cstring>

extern "C" {

/* ==================================================================
 * 上游解析
 * ================================================================== */

NFStatus nf_up_parse(const void *head, nf_u32 head_bytes,
                     const void *const *frames, const nf_u32 *frame_bytes,
                     nf_u32 n_frames, NFUpFrameView *out) {
    if (!head || !frames || !frame_bytes || !out) return NF_E_PARAM_NULL;
    if (head_bytes < sizeof(NFUpHeader)) return NF_E_PARAM_RANGE;

    std::memset(out, 0, sizeof(*out));
    std::memcpy(&out->header, head, sizeof(NFUpHeader));

    if (out->header.magic != NF_UP_MAGIC) return NF_E_PARAM_VERSION;
    if (out->header.version != NF_UP_VERSION) return NF_E_PARAM_VERSION;

    /* n_cameras 必须等于 4 —— 少一路覆盖不全, 多一路我们不认识 */
    if (out->header.n_cameras != NF_MAX_CAMERAS) return NF_E_PARAM_RANGE;
    if (n_frames != NF_MAX_CAMERAS) return NF_E_PARAM_RANGE;

    /* 位姿质量降级: 不是格式错误, 用状态码表达, 由上层决定是否冻结 */
    NFStatus st = NF_OK;
    if (out->header.ego.quality < 50) st = NF_E_BASELINE_DEGRADED;

    nf_u32 w = 0, h = 0;
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        if (!frames[i] || frame_bytes[i] < sizeof(NFUpImageHeader))
            return NF_E_PARAM_RANGE;

        NFUpImageHeader ih;
        std::memcpy(&ih, frames[i], sizeof(NFUpImageHeader));

        /* 尺寸严格: SPEC-UPSTREAM §2.1 */
        if (ih.width != (nf_u32)NF_IMG_W || ih.height != (nf_u32)NF_IMG_H)
            return NF_E_PARAM_RANGE;
        if (ih.format != 0) return NF_E_PARAM_RANGE;   /* 只支持 GRAY8 */

        /* **data_bytes 必须 == stride*height**: 防止上游截断后被当成完整帧解析 */
        const nf_u32 expect = ih.data_bytes;
        if (ih.stride < ih.width) return NF_E_PARAM_RANGE;
        if ((nf_u64)ih.stride * ih.height != (nf_u64)expect) return NF_E_PARAM_RANGE;
        /* 整段消息必须装得下头 + 像素 */
        if ((nf_u64)sizeof(NFUpImageHeader) + expect > (nf_u64)frame_bytes[i])
            return NF_E_PARAM_RANGE;

        /* 四路尺寸必须一致 */
        if (i == 0) { w = ih.width; h = ih.height; }
        else if (ih.width != w || ih.height != h) return NF_E_PARAM_RANGE;

        /* 相机 id 不得重复 */
        for (nf_u32 j = 0; j < i; ++j)
            if (out->cam_id[j] == ih.cam_id) return NF_E_PARAM_RANGE;
        if (ih.cam_id >= NF_MAX_CAMERAS) return NF_E_PARAM_RANGE;

        out->cam_id[i] = ih.cam_id;
        out->pixels[i] = (const nf_u8 *)frames[i] + sizeof(NFUpImageHeader);

        /* 各相机时间戳极差检查(SPEC-UPSTREAM §1.2) */
        const nf_i64 skew = (nf_i64)ih.timestamp_us
                          - (nf_i64)out->header.timestamp_us;
        const nf_i64 lim = (nf_i64)(NF_SYNC_MAX_SKEW_MS * 1000.0f);
        if (skew > lim || skew < -lim) return NF_E_PARAM_RANGE;
    }
    out->width = w;
    out->height = h;
    return st;
}

/* ==================================================================
 * 下游序列化
 * ================================================================== */

NFStatus nf_out_size(const NFOutHeader *h, nf_u32 *out_bytes) {
    if (!h || !out_bytes) return NF_E_PARAM_NULL;
    if (h->magic != NF_OUT_MAGIC) return NF_E_PARAM_VERSION;
    if (h->version != NF_OUT_VERSION) return NF_E_PARAM_VERSION;

    /* 各段长度必须与声明一致, 否则下游按 header 里的偏移解析会越界 */
    if (h->points_bytes != (nf_u64)h->num_points * sizeof(NFPoint))
        return NF_E_PARAM_RANGE;
    if (h->zmap_cells > 0) {
        if (h->zmap_bytes != h->zmap_cells * 4u) return NF_E_PARAM_RANGE;
        if (h->zindex_bytes != (h->zmap_cells + 1u) * 4u) return NF_E_PARAM_RANGE;
    } else {
        if (h->zmap_bytes != 0 || h->zindex_bytes != 0) return NF_E_PARAM_RANGE;
    }

    const nf_u64 total = (nf_u64)sizeof(NFOutHeader) + h->points_bytes
                       + h->zmap_bytes + h->zindex_bytes;
    if (total > 0xFFFFFFFFull) return NF_E_OUTPUT_OVERFLOW;
    *out_bytes = (nf_u32)total;
    return NF_OK;
}

} /* extern "C" */
