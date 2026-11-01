/**
 * @file timing.h
 * @brief 计时、逐阶段耗时统计。思路源自 ZLToolKit src/Util/TimeTicker.h
 *        (https://github.com/ZLMediaKit/ZLToolKit, MIT License), 精简改写。
 *
 * SPEC-BUILD §6 要求"每帧必带 frame_id 与逐阶段耗时"。
 * ScopedTimer 是 RAII 的: 作用域结束自动记录, 不会漏。
 */
#ifndef NFOCC_TIMING_H
#define NFOCC_TIMING_H

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace nf {

/** 单调时钟, 微秒。系统时间跳变不影响它。 */
uint64_t NowUs();

class ScopedTimer {
public:
    explicit ScopedTimer(double *out_ms)
        : start_(NowUs()), out_(out_ms) {}
    ~ScopedTimer() {
        if (out_) *out_ = ElapsedMs();
    }
    ScopedTimer(const ScopedTimer &) = delete;
    ScopedTimer &operator=(const ScopedTimer &) = delete;

    double ElapsedMs() const {
        return (double)(NowUs() - start_) / 1000.0;
    }

private:
    uint64_t start_;
    double *out_;
};

/**
 * 分位数统计(固定容量滑窗, 无动态分配)。
 * 用于逐阶段耗时监控: P50/P95 比均值更有判别力 ——
 * 均值 20ms 可能是"全部 20ms", 也可能是"95% 是 5ms + 5% 是 300ms",
 * 后者在实时系统里才是问题。
 */
class StageStats {
public:
    explicit StageStats(size_t window = 128) : window_(window ? window : 128) {
        samples_.assign(window_, 0.0);
    }

    void Add(double ms) {
        samples_[next_] = ms;
        next_ = (next_ + 1) % window_;
        if (count_ < window_) ++count_;
    }

    double Percentile(double p) const {
        if (count_ == 0) return 0.0;
        std::vector<double> v(samples_.begin(), samples_.begin() + count_);
        std::sort(v.begin(), v.end());
        size_t idx = (size_t)(p * (double)(count_ - 1) + 0.5);
        if (idx >= count_) idx = count_ - 1;
        return v[idx];
    }

    double P50() const { return Percentile(0.50); }
    double P95() const { return Percentile(0.95); }
    double Max() const { return Percentile(1.0); }
    size_t Count() const { return count_; }
    void Reset() { next_ = 0; count_ = 0; }

    /** 一行摘要, 供日志直接打印 */
    void Format(char *buf, size_t n, const char *name) const {
        std::snprintf(buf, n, "%s p50=%.2f p95=%.2f max=%.2f ms",
                      name, P50(), P95(), Max());
    }

private:
    size_t window_;
    std::vector<double> samples_;
    size_t next_ = 0;
    size_t count_ = 0;
};

}  // namespace nf

#endif  // NFOCC_TIMING_H
