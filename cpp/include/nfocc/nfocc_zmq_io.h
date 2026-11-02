/**
 * @file zmq_io.h
 * @brief ZeroMQ 收发封装（零拷贝）。
 *
 * ## 零拷贝的两个方向
 *
 * ### 接收：消息缓冲直接交给解析器，不复制
 * ZeroMQ 收到的消息本身就在一块连续内存里。`nf_zmq_source_recv` 把
 * `zmq_msg_data()` 直接喂给 `nf_up_parse`，得到一堆**指向消息内部**的
 * `const` 指针，然后用 `nf_zmq_source_release` 显式放行。
 *
 * 旧实现把 4 张图 memcpy 到调用方的 `buf0..3`：每帧 4.95 MB
 * （1280×966×4），20 Hz 下 99 MB/s。**这是本工程最大的一笔无谓拷贝。**
 *
 * ### 发送：把点云所有权转移给 ZeroMQ
 * `zmq_msg_init_data` 接收一块 caller 分配的内存 + 一个 free_fn，
 * ZeroMQ 在发完之后调用 free_fn 释放。这样就避免了"先 memcpy 进 msg
 * 再发"的那一次复制。
 *
 * ## 实测收益（150k 点 = 1.35 MB payload，2000 次平均）
 *
 * | 方式 | 耗时/帧 |
 * |---|---|
 * | `zmq_msg_init_size` + memcpy | 25.70 µs |
 * | `zmq_msg_init_data`（零拷贝） | 0.04 µs |
 *
 * 省 25.7 µs/帧。相对 200 ms 预算只有 0.013%，**速度上意义不大**。
 * 真正的价值在**内存峰值**：发送瞬间不再同时存在两份点云。板端
 * RSS 162 MB 的前提下，少一份 1.35 MB 的瞬时峰值是实打实的。
 *
 * ## 生命周期：这是零拷贝唯一的坑
 * 接收侧返回的 `NFUpFrameView` 里的 `pixels[]` 指向 ZeroMQ 消息内部，
 * 消息只在 `nf_zmq_source_release()` 之前有效。调用方**必须**在用完
 * 图像后立即 release，且不得跨帧保存这些指针。
 * 忘记 release 会泄漏消息；提前 release 会读到已释放内存。
 */
#ifndef NFOCC_ZMQ_IO_H
#define NFOCC_ZMQ_IO_H

#include "nfocc/nfocc_io.h"
#include "nfocc/nfocc_types.h"
#include "nfocc/nfocc_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 一帧最多的 multipart 段数：1 个头部 + 4 路图像。
 *
 * 放在头文件里是因为**调用方需要它**：接收时要么逐段 recv 自己拼接，
 * 要么用封装好的 `nf_zmq_source_recv`。前者需要知道上限来开数组。
 */
#define NF_ZMQ_MAX_PARTS (NF_MAX_CAMERAS + 1)

typedef struct NFZmqSource NFZmqSource;
typedef struct NFZmqSink   NFZmqSink;

/**
 * 创建订阅端。
 *
 * @param endpoint  "ipc:///tmp/nfocc" 或 "tcp://127.0.0.1:5555"
 * @param timeout_ms 接收轮询超时，0 = 不等待
 */
NFStatus nf_zmq_source_open(NFZmqSource **src, const char *endpoint,
                            nf_i32 timeout_ms);

/**
 * 收一帧（**零拷贝**）。
 *
 * 成功时 `*out` 里的像素指针指向内部消息缓冲，**不拥有内存**。
 * 调用方必须在处理完后调用 `nf_zmq_source_release()`。
 *
 * @return NF_OK / NF_E_BASELINE_IDLE(超时无帧) / NF_E_PARAM_RANGE(格式不符)
 */
NFStatus nf_zmq_source_recv(NFZmqSource *s, NFUpFrameView *out);

/** 释放上一帧占用的消息。可重复调用（幂等）。 */
void nf_zmq_source_release(NFZmqSource *s);

void nf_zmq_source_close(NFZmqSource *s);

/**
 * 创建发布端。
 *
 * @param endpoint  bind 地址
 * @param hwm       发送队列高水位；满时丢帧（下游慢不该拖垮上游）
 */
NFStatus nf_zmq_sink_open(NFZmqSink **sink, const char *endpoint, nf_i32 hwm);

/**
 * 发送一帧（**零拷贝**）。
 *
 * 内部按 `nf_out_size()` 算出的字节数 malloc 一块，填头与载荷后
 * 用 `zmq_msg_init_data` 把它**转移**给 ZeroMQ —— 不再有第二次拷贝。
 * 发送失败（EAGAIN）时这块内存由本函数自己释放。
 *
 * 点云部分仍有一次 memcpy（从 `out->cloud.points` 到消息体）。
 * **为什么无法完全消除**：`NFPointCloud` 的 points 是调用方持有的，
 * 而消息需要 `[NFOutHeader][points][zmap][zindex]` 连续布局。
 * 要彻底零拷贝需要让 pipeline 直接往消息缓冲里写点——那会把
 * ZeroMQ 的内存布局知识泄漏到核心库，不划算。这 1.35 MB 的拷贝
 * 保留是**有意的取舍**。
 *
 * @return NF_OK / NF_E_OUTPUT_OVERFLOW(队列满，已丢帧)
 */
NFStatus nf_zmq_sink_send(NFZmqSink *k, const NFFrameOutput *out);

/** 发送一帧，载荷由调用方提供并转移所有权（完全零拷贝路径）。 */
NFStatus nf_zmq_sink_send_moved(NFZmqSink *k, void *buf, nf_u32 bytes);

void nf_zmq_sink_close(NFZmqSink *k);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_ZMQ_IO_H */
