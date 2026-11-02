/**
 * @file zmq_io.cpp
 * @brief ZeroMQ 收发实现（零拷贝）。
 *
 * ## 接收侧为什么能真正零拷贝
 * ZeroMQ 的消息在 `zmq_msg_recv` 之后就已经在它自己的缓冲里。
 * 只要不 `zmq_msg_close`，这块内存就一直有效，`zmq_msg_data()`
 * 返回的指针可以直接交给 `nf_up_parse`。
 *
 * 旧实现每帧 memcpy 4.95 MB（1280×966×4 路）到调用方缓冲，
 * 20 Hz 下 99 MB/s —— 纯粹为了"缓冲归调用方所有"这个便利。
 * 现在改为**消息存活到 release**，调用方用 view 里的 const 指针。
 *
 * ## 生命周期
 * `s->msgs[]` 持有当前帧的 n_cameras+1 条消息（头部 + 每路图像）。
 * `nf_zmq_source_release()` 一次性关闭它们，并把计数归零。
 * release 前 view 里的指针有效，release 后立即失效。
 *
 * ## 发送侧的取舍
 * `nf_zmq_sink_send` 仍需一次 memcpy（点云 -> 连续消息体），
 * 因为 `NFFrameOutput.cloud.points` 的布局与线上格式不同。
 * 这一步无法在不把 ZeroMQ 布局知识泄漏到核心库的前提下消除。
 * 但用的是 `zmq_msg_init_data`，**省掉了 ZeroMQ 内部的第二次拷贝**。
 */
#include "nfocc/nfocc_zmq_io.h"

#include <zmq.h>
#include <cstdlib>
#include <cstring>

struct NFZmqSource {
    void       *ctx;
    void       *sock;
    nf_i32      timeout_ms;

    /* 当前帧持有的消息。收到新帧前必须先 release 旧的。 */
    zmq_msg_t   msgs[NF_ZMQ_MAX_PARTS];
    nf_u32      n_msgs;
    nf_u8       has_frame;
};

struct NFZmqSink {
    void  *ctx;
    void  *sock;
};

/* ---------------------------------------------------------------- 释放回调 */

namespace {

/**
 * `zmq_msg_init_data` 的释放回调。
 *
 * ZeroMQ 在消息发完（或在 close 时）调用它。**必须用 malloc/free
 * 配对**，不能用 new/delete —— ZeroMQ 可能在别的线程调用。
 */
void nf_zmq_free_cb(void *data, void *hint) {
    (void)hint;
    std::free(data);
}

}  // namespace

extern "C" {

/* ================================================================ 接收 */

NFStatus nf_zmq_source_open(NFZmqSource **out, const char *endpoint,
                            nf_i32 timeout_ms) {
    if (!out || !endpoint) return NF_E_PARAM_NULL;
    *out = nullptr;

    void *ctx = zmq_ctx_new();
    if (!ctx) return NF_E_ENGINE_OOM;
    void *sock = zmq_socket(ctx, ZMQ_SUB);
    if (!sock) { zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }

    int hwm = 2;   /* 只保留最新帧，避免积压 */
    zmq_setsockopt(sock, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(sock, ZMQ_SUBSCRIBE, "", 0);

    if (zmq_connect(sock, endpoint) != 0) {
        zmq_close(sock); zmq_ctx_destroy(ctx);
        return NF_E_PARAM_RANGE;
    }

    NFZmqSource *s = (NFZmqSource *)std::calloc(1, sizeof(NFZmqSource));
    if (!s) { zmq_close(sock); zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }
    s->ctx = ctx;
    s->sock = sock;
    s->timeout_ms = timeout_ms;
    *out = s;
    return NF_OK;
}

/**
 * 收一帧。
 *
 * **必须先释放上一帧**：ZeroMQ 的 RCVHWM=2 意味着最多积压 2 帧，
 * 如果调用方一直不 release，`msgs[]` 会越堆越多（虽然本实现会先
 * release 再收，但提前提醒避免误用）。
 */
NFStatus nf_zmq_source_recv(NFZmqSource *s, NFUpFrameView *out) {
    if (!s || !out) return NF_E_PARAM_NULL;

    /* 上一帧还在用的话先放掉 —— 不这么做会泄漏 */
    nf_zmq_source_release(s);

    zmq_pollitem_t item;
    item.socket = s->sock; item.fd = 0;
    item.events = ZMQ_POLLIN; item.revents = 0;
    const int rc = zmq_poll(&item, 1, (long)s->timeout_ms);
    if (rc == 0) return NF_E_BASELINE_IDLE;          /* 超时，无新帧 */
    if (rc < 0) return NF_E_ENGINE_UNSUPPORTED;

    /**
     * 零拷贝的关键：**不 memcpy，只把消息指针交出去**。
     *
     * `zmq_msg_recv` 之后消息数据就位，`zmq_msg_data()` 返回的就是
     * ZeroMQ 自己缓冲区的地址。只要不 close，这块内存有效。
     *
     * 用 ZMQ_RCVMORE 判断是否还有后续段（multipart）。
     */
    nf_u32 n = 0;
    while (n < NF_ZMQ_MAX_PARTS) {
        zmq_msg_t *m = &s->msgs[n];
        zmq_msg_init(m);
        if (zmq_msg_recv(m, s->sock, 0) < 0) {
            /* 收到一半失败：已收的全部关掉，回到无帧状态 */
            for (nf_u32 i = 0; i <= n; ++i) zmq_msg_close(&s->msgs[i]);
            return NF_E_PARAM_RANGE;
        }
        ++n;

        int more = 0;
        size_t more_sz = sizeof(more);
        zmq_getsockopt(s->sock, ZMQ_RCVMORE, &more, &more_sz);
        if (!more) break;
    }
    s->n_msgs = n;
    s->has_frame = 1;

    /* 至少要有 1 个头部段 + 4 路图像段 */
    if (n < 2) { nf_zmq_source_release(s); return NF_E_PARAM_RANGE; }

    /* 组装给 nf_up_parse 的指针数组 —— 全是消息内部地址，零拷贝 */
    const void *frames[NF_MAX_CAMERAS];
    nf_u32     frame_bytes[NF_MAX_CAMERAS];
    const nf_u32 n_img = n - 1u;
    if (n_img != NF_MAX_CAMERAS) {
        nf_zmq_source_release(s);
        return NF_E_PARAM_RANGE;
    }
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        frames[i] = zmq_msg_data(&s->msgs[i + 1u]);
        frame_bytes[i] = (nf_u32)zmq_msg_size(&s->msgs[i + 1u]);
    }

    const NFStatus st = nf_up_parse(zmq_msg_data(&s->msgs[0]),
                                    (nf_u32)zmq_msg_size(&s->msgs[0]),
                                    frames, frame_bytes, n_img, out);
    if (st != NF_OK && st != NF_E_BASELINE_DEGRADED) {
        nf_zmq_source_release(s);
        return st;
    }
    /* 注意：降级（位姿质量不足）仍返回 view，由上层决定是否冻结 */
    return st;
}

void nf_zmq_source_release(NFZmqSource *s) {
    if (!s || !s->has_frame) return;
    for (nf_u32 i = 0; i < s->n_msgs; ++i) zmq_msg_close(&s->msgs[i]);
    s->n_msgs = 0;
    s->has_frame = 0;
}

void nf_zmq_source_close(NFZmqSource *s) {
    if (!s) return;
    nf_zmq_source_release(s);
    if (s->sock) zmq_close(s->sock);
    if (s->ctx) zmq_ctx_destroy(s->ctx);
    std::free(s);
}

/* ================================================================ 发送 */

NFStatus nf_zmq_sink_open(NFZmqSink **out, const char *endpoint, nf_i32 hwm) {
    if (!out || !endpoint) return NF_E_PARAM_NULL;
    *out = nullptr;

    void *ctx = zmq_ctx_new();
    if (!ctx) return NF_E_ENGINE_OOM;
    void *sock = zmq_socket(ctx, ZMQ_PUB);
    if (!sock) { zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }

    int h = (hwm > 0) ? (int)hwm : 4;
    zmq_setsockopt(sock, ZMQ_SNDHWM, &h, sizeof(h));
    if (zmq_bind(sock, endpoint) != 0) {
        zmq_close(sock); zmq_ctx_destroy(ctx);
        return NF_E_PARAM_RANGE;
    }

    NFZmqSink *k = (NFZmqSink *)std::calloc(1, sizeof(NFZmqSink));
    if (!k) { zmq_close(sock); zmq_ctx_destroy(ctx); return NF_E_ENGINE_OOM; }
    k->ctx = ctx;
    k->sock = sock;
    *out = k;
    return NF_OK;
}

/**
 * 把一块 caller 所有的内存**转移**给 ZeroMQ。
 *
 * 这是完全零拷贝路径：不再复制任何字节，只是把指针和 free_fn 交出去。
 * 发送完成后 ZeroMQ 调 `nf_zmq_free_cb` 释放。
 */
NFStatus nf_zmq_sink_send_moved(NFZmqSink *k, void *buf, nf_u32 bytes) {
    if (!k || !buf) return NF_E_PARAM_NULL;
    if (bytes == 0) { std::free(buf); return NF_E_PARAM_RANGE; }

    zmq_msg_t msg;
    /* free_fn 负责释放 buf —— 之后无论成功失败都不再由本函数 free */
    if (zmq_msg_init_data(&msg, buf, bytes, nf_zmq_free_cb, nullptr) != 0) {
        std::free(buf);           /* init 失败，所有权还在我们手里 */
        return NF_E_ENGINE_OOM;
    }

    const int rc = zmq_msg_send(&msg, k->sock, ZMQ_DONTWAIT);
    /**
     * 无论是否发出，都要 close：ZeroMQ 在 close 时若仍持有所有权，
     * 会调用 free_fn 释放。**不能自己 free(buf)** —— 会双重释放。
     */
    zmq_msg_close(&msg);

    /* EAGAIN = 队列满。下游慢时丢帧是正确行为，不是错误 */
    return (rc < 0) ? NF_E_OUTPUT_OVERFLOW : NF_OK;
}

NFStatus nf_zmq_sink_send(NFZmqSink *k, const NFFrameOutput *out) {
    if (!k || !out) return NF_E_PARAM_NULL;

    /**
     * 按 ICD 布局算总字节：[NFOutHeader][points][zmap][zindex]
     *
     * 用 nf_out_size 而不是自己加 —— 布局的定义处只有那一份，
     * 自己算会和下游解析悄悄对不上。
     */
    NFOutHeader h;
    std::memset(&h, 0, sizeof(h));
    h.magic = NF_OUT_MAGIC;
    h.version = NF_OUT_VERSION;
    h.frame_id = out->frame_id;
    /* is_frozen=1 表示冻结，此时 cloud 不可用；线上用 is_valid 表达可信度 */
    h.is_valid = out->is_frozen ? 0u : 1u;
    h.mode = (nf_u8)out->dominant_mode;
    h.num_points = out->cloud.num_points;
    h.num_cameras_ok = 0;
    for (nf_u32 i = 0; i < NF_NUM_CAMERAS; ++i)
        if (out->cam_status[i] == NF_OK) ++h.num_cameras_ok;
    h.points_bytes = out->cloud.num_points * (nf_u32)sizeof(NFPoint);

    nf_u32 total = 0;
    const NFStatus st = nf_out_size(&h, &total);
    if (st != NF_OK || total == 0) return st ? st : NF_E_PARAM_RANGE;

    /* 一次 malloc，直接把 ZeroMQ 的缓冲给我们用 —— 避免它内部再拷一份 */
    nf_u8 *buf = (nf_u8 *)std::malloc(total);
    if (!buf) return NF_E_ENGINE_OOM;

    std::memcpy(buf, &h, sizeof(h));
    nf_u8 *p = buf + sizeof(h);
    const size_t nbytes = (size_t)out->cloud.num_points * sizeof(NFPoint);
    if (nbytes) std::memcpy(p, out->cloud.points, nbytes);

    return nf_zmq_sink_send_moved(k, buf, total);
}

void nf_zmq_sink_close(NFZmqSink *k) {
    if (!k) return;
    if (k->sock) zmq_close(k->sock);
    if (k->ctx) zmq_ctx_destroy(k->ctx);
    std::free(k);
}

} /* extern "C" */
