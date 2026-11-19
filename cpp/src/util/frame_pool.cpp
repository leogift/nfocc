/**
 * @file frame_pool.cpp
 * @brief FramePool 实现。
 */
#include "nfocc/util/frame_pool.h"

#include <cstdlib>
#include <cstring>
#include <new>

namespace nf {

FramePool::FramePool(size_t capacity, uint32_t width, uint32_t height)
    : capacity_(capacity), width_(width), height_(height) {
    if (capacity == 0 || width == 0 || height == 0) {
        capacity_ = 0;
        return;
    }
    frames_.reserve(capacity);
    free_list_.reserve(capacity);
    const size_t bytes = (size_t)width * height;
    for (size_t i = 0; i < capacity; ++i) {
        Frame *f = (Frame *)std::calloc(1, sizeof(Frame));
        if (!f) break;
        // 用 placement new 初始化 atomic 成员(calloc 不做构造)
        new (f) Frame();
        f->data = (uint8_t *)std::malloc(bytes);
        if (!f->data) {
            f->~Frame();
            std::free(f);
            break;
        }
        f->width = width;
        f->height = height;
        f->refs.store(0, std::memory_order_relaxed);
        frames_.push_back(f);
        free_list_.push_back(f);
    }
    capacity_ = frames_.size();   // 实际分配成功的数量
}

FramePool::~FramePool() {
    for (Frame *f : frames_) {
        std::free(f->data);
        f->~Frame();
        std::free(f);
    }
    frames_.clear();
    free_list_.clear();
}

Frame *FramePool::Acquire() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (free_list_.empty()) {
        exhausted_.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    Frame *f = free_list_.back();
    free_list_.pop_back();
    f->refs.store(1, std::memory_order_release);
    in_use_.fetch_add(1, std::memory_order_relaxed);
    return f;
}

void FramePool::AddRef(Frame *f) {
    if (!f) return;
    f->refs.fetch_add(1, std::memory_order_relaxed);
}

void FramePool::Release(Frame *f) {
    if (!f) return;
    // 归零才回池。用 acq_rel 保证 —— release 前对帧的写入对下一次 Acquire 可见
    if (f->refs.fetch_sub(1, std::memory_order_acq_rel) != 1) return;

    std::lock_guard<std::mutex> lk(mtx_);
    // 不清 data, 只重置元信息 —— 清 3MB 比 memset 更贵的场合应避免
    f->timestamp_us = 0;
    f->frame_id = 0;
    f->cam_id = 0;
    free_list_.push_back(f);
    in_use_.fetch_sub(1, std::memory_order_relaxed);
}

bool FramePool::Resize(uint32_t width, uint32_t height) {
    if (width == 0 || height == 0) return false;
    if (width == width_ && height == height_) return true;
    std::lock_guard<std::mutex> lk(mtx_);
    // 有人正持有缓冲时不能换 —— 否则其 data 指针会被 free
    if (in_use_.load(std::memory_order_relaxed) != 0) return false;

    const size_t bytes = (size_t)width * height;
    for (Frame *f : frames_) {
        uint8_t *nb = (uint8_t *)std::malloc(bytes);
        if (!nb) return false;      // 分配失败: 保持原状, 已换过的不回滚会不一致
        std::free(f->data);
        f->data = nb;
        f->width = width;
        f->height = height;
    }
    width_ = width;
    height_ = height;
    return true;
}

}  // namespace nf
