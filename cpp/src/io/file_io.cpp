/**
 * @file file_io.cpp
 * @brief 文件回放 IO(无 ZeroMQ 时的替代)。用于 SIL 调试与录包重放。
 *
 * 数据格式(.nfp 简易容器, 小端):
 *   magic  "NFPK"          4 B
 *   version u32            4 B   = 1
 *   width   u32            4 B
 *   height  u32            4 B
 *   n_frames u32           4 B
 *   --- 之后每帧 ---
 *   frame_id u32           4 B
 *   timestamp_us u64       8 B
 *   t_cam[4][3] f32       48 B   各相机平移(相机系, mm)
 *   pixels  u8[w*h*4]      每相机 1 通道, 按 FV/RV/ML/MR 顺序
 */
#include "nfocc/nfocc_pipeline.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#define NFPK_MAGIC "NFPK"
#define NFPK_VERSION 1u

typedef struct {
    FILE *fp;
    nf_u32 width, height, n_frames, read;
} NFPlayback;

extern "C" {

NFStatus nf_playback_open(NFPlayback **pb, const char *path) {
    if (!pb || !path) return NF_E_PARAM_NULL;
    *pb = nullptr;
    FILE *fp = std::fopen(path, "rb");
    if (!fp) return NF_E_PARAM_NULL;

    char magic[4];
    nf_u32 ver = 0, w = 0, h = 0, n = 0;
    if (std::fread(magic, 1, 4, fp) != 4 || std::memcmp(magic, NFPK_MAGIC, 4) != 0) {
        std::fclose(fp);
        return NF_E_PARAM_VERSION;
    }
    if (std::fread(&ver, 4, 1, fp) != 1 || ver != NFPK_VERSION) {
        std::fclose(fp);
        return NF_E_PARAM_VERSION;
    }
    if (std::fread(&w, 4, 1, fp) != 1 || std::fread(&h, 4, 1, fp) != 1 ||
        std::fread(&n, 4, 1, fp) != 1) {
        std::fclose(fp);
        return NF_E_PARAM_RANGE;
    }
    if (w == 0 || h == 0) { std::fclose(fp); return NF_E_PARAM_RANGE; }

    NFPlayback *p = (NFPlayback *)std::calloc(1, sizeof(NFPlayback));
    if (!p) { std::fclose(fp); return NF_E_ENGINE_OOM; }
    p->fp = fp; p->width = w; p->height = h; p->n_frames = n; p->read = 0;
    *pb = p;
    return NF_OK;
}

/**
 * 读一帧。调用方提供 4 路缓冲(每路 w*h 字节)。
 * @return NF_OK / NF_E_BASELINE_IDLE(已到末尾)
 */
NFStatus nf_playback_read(NFPlayback *p, NFFrameInput *in,
                          nf_u8 *buf0, nf_u8 *buf1, nf_u8 *buf2, nf_u8 *buf3) {
    if (!p || !in || !buf0 || !buf1 || !buf2 || !buf3) return NF_E_PARAM_NULL;
    if (p->read >= p->n_frames) return NF_E_BASELINE_IDLE;

    nf_u32 fid = 0; nf_u64 ts = 0;
    float t[12];
    if (std::fread(&fid, 4, 1, p->fp) != 1) return NF_E_PARAM_RANGE;
    if (std::fread(&ts, 8, 1, p->fp) != 1) return NF_E_PARAM_RANGE;
    if (std::fread(t, 4, 12, p->fp) != 12) return NF_E_PARAM_RANGE;

    nf_u8 *bufs[4] = { buf0, buf1, buf2, buf3 };
    const size_t sz = (size_t)p->width * p->height;
    for (int c = 0; c < 4; ++c) {
        if (std::fread(bufs[c], 1, sz, p->fp) != sz) return NF_E_PARAM_RANGE;
    }

    in->width = p->width;
    in->height = p->height;
    in->stride = p->width;
    in->frame_id = fid;
    in->timestamp_us = ts;
    in->body_mask = nullptr;
    in->frame_gap = 1;
    for (int c = 0; c < 4; ++c) {
        in->img[c] = bufs[c];
        for (int k = 0; k < 3; ++k) in->t_cam[c][k] = t[c * 3 + k];
    }
    ++p->read;
    return NF_OK;
}

void nf_playback_close(NFPlayback *p) {
    if (!p) return;
    if (p->fp) std::fclose(p->fp);
    std::free(p);
}

/** 写一帧(供 tools/nfocc_gen 生成测试包) */
NFStatus nf_playback_write_header(FILE *fp, nf_u32 w, nf_u32 h, nf_u32 n) {
    if (!fp) return NF_E_PARAM_NULL;
    const nf_u32 ver = NFPK_VERSION;
    if (std::fwrite(NFPK_MAGIC, 1, 4, fp) != 4) return NF_E_OUTPUT_OVERFLOW;
    if (std::fwrite(&ver, 4, 1, fp) != 1) return NF_E_OUTPUT_OVERFLOW;
    if (std::fwrite(&w, 4, 1, fp) != 1) return NF_E_OUTPUT_OVERFLOW;
    if (std::fwrite(&h, 4, 1, fp) != 1) return NF_E_OUTPUT_OVERFLOW;
    if (std::fwrite(&n, 4, 1, fp) != 1) return NF_E_OUTPUT_OVERFLOW;
    return NF_OK;
}

NFStatus nf_playback_write_frame(FILE *fp, nf_u32 fid, nf_u64 ts,
                                 const nf_f32 t_cam[4][3],
                                 const nf_u8 *const img[4],
                                 nf_u32 w, nf_u32 h) {
    if (!fp || !t_cam || !img) return NF_E_PARAM_NULL;
    if (std::fwrite(&fid, 4, 1, fp) != 1) return NF_E_OUTPUT_OVERFLOW;
    if (std::fwrite(&ts, 8, 1, fp) != 1) return NF_E_OUTPUT_OVERFLOW;
    if (std::fwrite(t_cam, 4, 12, fp) != 12) return NF_E_OUTPUT_OVERFLOW;
    const size_t sz = (size_t)w * h;
    for (int c = 0; c < 4; ++c) {
        if (!img[c]) return NF_E_PARAM_NULL;
        if (std::fwrite(img[c], 1, sz, fp) != sz) return NF_E_OUTPUT_OVERFLOW;
    }
    return NF_OK;
}

} /* extern "C" */
