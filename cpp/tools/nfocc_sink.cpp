/**
 * @file nfocc_sink.cpp
 * @brief ZMQ 端到端联调的下游接收端：SUB 订阅并打印结果。
 *
 * 用途：
 *   - 验证上游 nfocc_gen -> 本系统 -> 下游 的全链路
 *   - 检查冻结帧是否照常发出（SPEC-DOWNSTREAM §1 核心原则）
 *
 * 用法：
 *   nfocc_sink --sub tcp://127.0.0.1:5556 --timeout 3000
 */
#include "nfocc/nfocc_io.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef NF_HAS_ZMQ
#include <zmq.h>
#else
int main() {
    std::printf("本程序需要 ZeroMQ（构建时 NF_WITH_ZMQ=ON）\n");
    return 2;
}
#endif

#ifdef NF_HAS_ZMQ
int main(int argc, char **argv) {
    const char *endpoint = "tcp://127.0.0.1:5556";
    int timeout_ms = 3000;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--sub") && i + 1 < argc) endpoint = argv[++i];
        else if (!std::strcmp(argv[i], "--timeout") && i + 1 < argc)
            timeout_ms = std::atoi(argv[++i]);
    }

    void *ctx = zmq_ctx_new();
    void *sub = zmq_socket(ctx, ZMQ_SUB);
    int rcvhwm = 2;
    zmq_setsockopt(sub, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(sub, endpoint) != 0) {
        std::printf("连接失败: %s\n", endpoint);
        return 1;
    }
    std::printf("已连接 %s, 等待消息 (超时 %d ms/条)\n", endpoint, timeout_ms);

    int got = 0, valid = 0, frozen = 0;
    for (;;) {
        zmq_pollitem_t item = { sub, 0, ZMQ_POLLIN, 0 };
        const int rc = zmq_poll(&item, 1, timeout_ms);
        if (rc == 0) break;            /* 超时: 认为上游停了 */
        if (rc < 0) break;

        /* 段 0: 头部 */
        zmq_msg_t m0;
        zmq_msg_init(&m0);
        if (zmq_msg_recv(&m0, sub, 0) < 0) { zmq_msg_close(&m0); break; }
        if (zmq_msg_size(&m0) < sizeof(NFOutHeader)) { zmq_msg_close(&m0); continue; }

        NFOutHeader h;
        std::memcpy(&h, zmq_msg_data(&m0), sizeof(h));
        /* more 必须在 close 之前问 —— close 后再查是未定义行为 */
        const int has_more = zmq_msg_more(&m0);
        zmq_msg_close(&m0);

        if (h.magic != NF_OUT_MAGIC) {
            /* 不是我们的消息: 仍需吃掉后续段, 否则会错位 */
            if (has_more) {
                zmq_msg_t skip;
                zmq_msg_init(&skip);
                if (zmq_msg_recv(&skip, sub, 0) >= 0) zmq_msg_close(&skip);
            }
            std::printf("  [跳过] magic 不符\n");
            continue;
        }

        /* 段 1: 负载 */
        if (has_more) {
            zmq_msg_t m1;
            zmq_msg_init(&m1);
            if (zmq_msg_recv(&m1, sub, 0) >= 0) zmq_msg_close(&m1);
        }

        ++got;
        if (h.is_valid) ++valid; else ++frozen;

        std::printf("帧 %3u: valid=%d status=%2u mode=%u pts=%5u "
                    "base=%6.1fmm sigma=%6.1fmm latency=%5.1fms\n",
                    h.frame_id, h.is_valid, h.status, h.mode, h.num_points,
                    h.baseline_mm, h.sigma_med_mm, h.latency_us / 1000.0);
    }

    std::printf("\n共收到 %d 条 (有效 %d, 冻结 %d)\n", got, valid, frozen);
    std::printf("冻结帧比例 %.0f%% (冻结也必须发 -> 应 == 输入帧数)\n",
                got ? 100.0 * frozen / got : 0.0);

    zmq_close(sub);
    zmq_ctx_destroy(ctx);
    return got > 0 ? 0 : 1;
}
#endif
