/**
 * @file zmq_io.cpp
 * @brief ZeroMQ 输入/输出。用于实时部署与 SIL 联调。
 *
 * ## 拓扑
 *   输入: SUB  (连接图像源, 收 4 路图 + 时间戳 + 平移量)
 *   输出: PUB  (发点云 + zMap)
 *
 * ## 为什么用 SUB/PUB 而不是 REQ/REP
 * 感知链是"一直消费最新帧", 不需要应答。SUB 的 HWM 会自动丢旧帧,
 * 天然避免积压 —— 这是实时系统的正确行为, 而不是缺陷。
 *
 * ## 消息格式(与 file_io 的 .nfp 帧体一致, 便于互换)
 *   输入单帧 = u32 frame_id | u64 ts | f32 t_cam[4][3] | u8 img[4][w*h]
 *   输出单帧 = u32 frame_id | u8 mode[4] | u32 n_points | NFPoint[n_points]
 */
#include "nfocc/nfocc_pipeline.h"

#include <zmq.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

/** ZeroMQ 输入源 */
typedef struct {
    void *ctx;
    void *sock;
    nf_u32 width, height;
    int    timeout_ms;
} NFZmqSource;

/** ZeroMQ 输出汇 */
typedef struct {
    void *ctx;
    void *sock;
} NFZmqSink;

extern "C" {

/* ---------------------------------------------------------------- 源 */

NFStatus nf_zmq_source_open(NFZmqSource **src, const char *endpoint,
                            nf_u32 width, nf_u32 height, int timeout_ms) {
    if (!src || !endpoint) return NF_E_PARAM_NULL;
    *src = nullptr;

    void *ctx = zmq_ctx_new();
    if (!ctx) return NF_E_ENGINE_OOM;
    void *sock = zmq_socket(ctx, ZMQ_SUB);
    if (!sock) { zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }

    int hwm = 2;   /* 只保留最新帧, 避免积压 */
    zmq_setsockopt(sock, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(sock, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(sock, endpoint) != 0) {
        zmq_close(sock); zmq_ctx_destroy(ctx);
        return NF_E_PARAM_RANGE;
    }

    NFZmqSource *s = (NFZmqSource *)std::calloc(1, sizeof(NFZmqSource));
    if (!s) { zmq_close(sock); zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }
    s->ctx = ctx; s->sock = sock;
    s->width = width; s->height = height;
    s->timeout_ms = timeout_ms;
    *src = s;
    return NF_OK;
}

/**
 * 收一帧。缓冲由调用方提供(4 路, 各 w*h 字节)。
 * @return NF_OK / NF_E_BASELINE_IDLE(超时, 无新帧) / NF_E_PARAM_RANGE(格式不符)
 */
NFStatus nf_zmq_source_recv(NFZmqSource *s, NFFrameInput *in,
                            nf_u8 *buf0, nf_u8 *buf1, nf_u8 *buf2, nf_u8 *buf3) {
    if (!s || !in || !buf0 || !buf1 || !buf2 || !buf3) return NF_E_PARAM_NULL;

    const size_t img_sz = (size_t)s->width * s->height;
    const size_t expect = 4 + 8 + 48 + img_sz * 4;

    zmq_pollitem_t item;
    item.socket = s->sock; item.fd = 0; item.events = ZMQ_POLLIN; item.revents = 0;
    const int rc = zmq_poll(&item, 1, s->timeout_ms);
    if (rc == 0) return NF_E_BASELINE_IDLE;      /* 超时 */
    if (rc < 0) return NF_E_ENGINE_UNSUPPORTED;

    zmq_msg_t msg;
    zmq_msg_init(&msg);
    if (zmq_msg_recv(&msg, s->sock, 0) < 0) { zmq_msg_close(&msg); return NF_E_PARAM_RANGE; }
    const size_t sz = zmq_msg_size(&msg);
    if (sz != expect) { zmq_msg_close(&msg); return NF_E_PARAM_RANGE; }

    const nf_u8 *p = (const nf_u8 *)zmq_msg_data(&msg);
    nf_u32 fid = 0; nf_u64 ts = 0; float t[12];
    std::memcpy(&fid, p, 4);          p += 4;
    std::memcpy(&ts, p, 8);           p += 8;
    std::memcpy(t, p, 48);            p += 48;

    nf_u8 *bufs[4] = { buf0, buf1, buf2, buf3 };
    for (int c = 0; c < 4; ++c) {
        std::memcpy(bufs[c], p, img_sz);
        p += img_sz;
    }
    zmq_msg_close(&msg);

    in->width = s->width; in->height = s->height; in->stride = s->width;
    in->frame_id = fid; in->timestamp_us = ts;
    in->body_mask = nullptr; in->frame_gap = 1;
    for (int c = 0; c < 4; ++c) {
        in->img[c] = bufs[c];
        for (int k = 0; k < 3; ++k) in->t_cam[c][k] = t[c * 3 + k];
    }
    return NF_OK;
}

void nf_zmq_source_close(NFZmqSource *s) {
    if (!s) return;
    if (s->sock) zmq_close(s->sock);
    if (s->ctx) zmq_ctx_destroy(s->ctx);
    std::free(s);
}

/* ---------------------------------------------------------------- 汇 */

NFStatus nf_zmq_sink_open(NFZmqSink **sink, const char *endpoint) {
    if (!sink || !endpoint) return NF_E_PARAM_NULL;
    *sink = nullptr;

    void *ctx = zmq_ctx_new();
    if (!ctx) return NF_E_ENGINE_OOM;
    void *sock = zmq_socket(ctx, ZMQ_PUB);
    if (!sock) { zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }

    int hwm = 4;
    zmq_setsockopt(sock, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    if (zmq_bind(sock, endpoint) != 0) {
        zmq_close(sock); zmq_ctx_destroy(ctx);
        return NF_E_PARAM_RANGE;
    }
    NFZmqSink *k = (NFZmqSink *)std::calloc(1, sizeof(NFZmqSink));
    if (!k) { zmq_close(sock); zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }
    k->ctx = ctx; k->sock = sock;
    *sink = k;
    return NF_OK;
}

NFStatus nf_zmq_sink_send(NFZmqSink *k, const NFFrameOutput *out) {
    if (!k || !out) return NF_E_PARAM_NULL;

    const size_t n = out->cloud.num_points;
    const size_t sz = 4 + 4 + 4 + n * sizeof(NFPoint);
    zmq_msg_t msg;
    if (zmq_msg_init_size(&msg, sz) != 0) return NF_E_ENGINE_OOM;

    nf_u8 *p = (nf_u8 *)zmq_msg_data(&msg);
    std::memcpy(p, &out->frame_id, 4);           p += 4;
    std::memcpy(p, out->mode, 4);                p += 4;
    const nf_u32 cnt = (nf_u32)n;
    std::memcpy(p, &cnt, 4);                     p += 4;
    if (n) std::memcpy(p, out->cloud.points, n * sizeof(NFPoint));

    const int rc = zmq_msg_send(&msg, k->sock, ZMQ_DONTWAIT);
    zmq_msg_close(&msg);
    /* 发送队列满(EAGAIN)不是错误 —— 下游慢时丢帧是正确行为 */
    return (rc < 0) ? NF_E_OUTPUT_OVERFLOW : NF_OK;
}

void nf_zmq_sink_close(NFZmqSink *k) {
    if (!k) return;
    if (k->sock) zmq_close(k->sock);
    if (k->ctx) zmq_ctx_destroy(k->ctx);
    std::free(k);
}

} /* extern "C" */
