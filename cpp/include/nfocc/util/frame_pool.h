/**
 * @file frame_pool.h
 * @brief 帧缓冲池: 避免运行期 malloc。
 *
 * 思路源自 ZLToolKit src/Util/RingBuffer.h 的**引用计数 + 单写多读**模型
 * (https://github.com/ZLMediaKit/ZLToolKit, MIT License)。
 *
 * ## 与 RingBuffer 的差异(为什么不能照搬)
 * RingBuffer 为"GOP 缓存 + 多播放器追赶"设计, 默认缓存 512 帧。
 * 本项目 5 Hz 实时处理, 缓存历史帧只会增加延迟 —— 所以只借用它的
 * **缓冲复用**思想, 不借用缓存语义。
 *
 * ## 语义
 * - Acquire: 从空闲链表取; 无则新建。返回引用计数 = 1 的帧。
 * - Release: 计数归零时回空闲链表, **不 free**。
 * - 帧只按尺寸分档复用(避免尺寸频繁变化导致链表爆炸)。
 *
 * ## SPEC-BUILD §5 要求
 * "全部在 nf_init 一次分配, 运行期零 malloc"。
 * 池在构造时预分配 capacity 个帧, 运行期 Acquire 不再 malloc。
 */
#ifndef NFOCC_FRAME_POOL_H
#define NFOCC_FRAME_POOL_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace nf {

struct Frame {
    uint8_t *data = nullptr;    /**< 像素缓冲, w*h 字节 */
    uint32_t width = 0;
    uint32_t height = 0;
    uint64_t timestamp_us = 0;
    uint32_t frame_id = 0;
    uint8_t cam_id = 0;
    std::atomic<int> refs{0};   /**< 引用计数, 0 = 在池中空闲 */

    size_t Bytes() const { return (size_t)width * height; }
};

class FramePool {
public:
    /**
     * @param capacity 预分配帧数(默认 8 = 4 路 × 2 帧, 供参考帧+当前帧)
     * @param width    帧宽
     * @param height   帧高
     * @note 构造时一次性分配 capacity × w × h 字节。失败时 capacity 记 0,
     *       后续 Acquire 返回 nullptr(不抛异常)。
     */
    FramePool(size_t capacity, uint32_t width, uint32_t height);
    ~FramePool();

    FramePool(const FramePool &) = delete;
    FramePool &operator=(const FramePool &) = delete;

    /** 取一个可写帧, 引用计数置 1。池空则返回 nullptr。 */
    Frame *Acquire();

    /** 归还。计数归零后回到空闲链表。传 nullptr 安全。 */
    void Release(Frame *f);

    /** 额外增加引用(共享给下游时用)。 */
    void AddRef(Frame *f);

    size_t Capacity() const { return capacity_; }
    size_t InUse() const { return in_use_.load(std::memory_order_relaxed); }
    size_t Free() const { return capacity_ - InUse(); }
    /** 池耗尽累计次数(监控用, 非零说明 capacity 太小) */
    size_t Exhausted() const { return exhausted_.load(std::memory_order_relaxed); }

    uint32_t Width() const { return width_; }
    uint32_t Height() const { return height_; }

    /**
     * 改变帧尺寸并重新分配**空闲**帧。
     * 有帧在用(in_use>0)时返回 false —— 不能在别人持有缓冲时换尺寸,
     * 那会让持有者访问到已释放的内存。
     */
    bool Resize(uint32_t width, uint32_t height);

private:
    std::vector<Frame *> frames_;
    std::vector<Frame *> free_list_;
    size_t capacity_ = 0;
    uint32_t width_ = 0, height_ = 0;
    mutable std::mutex mtx_;
    std::atomic<size_t> in_use_{0};
    std::atomic<size_t> exhausted_{0};
};

}  // namespace nf

#endif  // NFOCC_FRAME_POOL_H
