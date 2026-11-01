/**
 * @file task_executor.cpp
 * @brief TaskExecutor 实现。思路源自 ZLToolKit, 见头文件说明。
 */
#include "nfocc/util/task_executor.h"

#include <algorithm>

#if defined(__APPLE__)
#include <pthread.h>
#endif

namespace nf {

namespace {

/** 设置当前线程名。macOS 用 pthread_setname_np(只改当前线程),
 *  Linux 用 pthread_setname_np(可指定线程), 两者签名不同。 */
void set_thread_name(const std::string &name, int idx) {
#if defined(__APPLE__)
    // macOS 限制 63 字节
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s%d", name.c_str(), idx);
    pthread_setname_np(buf);
#elif defined(__linux__)
    char buf[16];   // Linux 限制 15 字符 + '\0'
    std::snprintf(buf, sizeof(buf), "%s%d", name.c_str(), idx);
    pthread_setname_np(pthread_self(), buf);
#else
    (void)name;
    (void)idx;
#endif
}

}  // namespace

TaskExecutor::TaskExecutor(const char *name, size_t threads)
    : name_(name ? name : "nf") {
    if (threads == 0) {
        // 不创建线程: 记录当前线程 id, Async 时直接执行
        std::lock_guard<std::mutex> lk(ids_mtx_);
        ids_.push_back(std::this_thread::get_id());
        return;
    }
    threads_.reserve(threads);
    for (size_t i = 0; i < threads; ++i) {
        threads_.emplace_back([this, i] {
            set_thread_name(name_, (int)i);
            {
                std::lock_guard<std::mutex> lk(ids_mtx_);
                ids_.push_back(std::this_thread::get_id());
            }
            Run();
        });
    }
}

TaskExecutor::~TaskExecutor() { Stop(); }

void TaskExecutor::Async(std::function<void()> task, bool may_sync) {
    if (!task) return;
    // may_sync: 已在目标线程 -> 直接执行, 避免"投递后等自己"的死锁
    if (may_sync && IsCurrentThread()) {
        task();
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!running_) return;
        queue_.push_back(std::move(task));
        pending_.fetch_add(1, std::memory_order_relaxed);
    }
    cv_.notify_one();
}

void TaskExecutor::AsyncFirst(std::function<void()> task) {
    if (!task) return;
    if (IsCurrentThread()) {
        task();
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!running_) return;
        queue_.push_front(std::move(task));
        pending_.fetch_add(1, std::memory_order_relaxed);
    }
    cv_.notify_one();
}

void TaskExecutor::Run() {
    for (;;) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this] { return !running_ || !queue_.empty(); });
            if (!running_ && queue_.empty()) return;
            task = std::move(queue_.front());
            queue_.pop_front();
        }
        task();
        const size_t left = pending_.fetch_sub(1, std::memory_order_relaxed) - 1;
        if (left == 0) {
            std::lock_guard<std::mutex> lk(mtx_);
            cv_done_.notify_all();
        }
    }
}

void TaskExecutor::Sync() {
    if (IsCurrentThread()) {
        // 在自身线程里 Sync 会永远等不到 —— 直接把队列跑空
        for (;;) {
            std::function<void()> task;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (queue_.empty()) return;
                task = std::move(queue_.front());
                queue_.pop_front();
            }
            task();
            pending_.fetch_sub(1, std::memory_order_relaxed);
        }
    }
    std::unique_lock<std::mutex> lk(mtx_);
    cv_done_.wait(lk, [this] { return pending_.load(std::memory_order_relaxed) == 0; });
}

bool TaskExecutor::IsCurrentThread() const {
    const std::thread::id me = std::this_thread::get_id();
    std::lock_guard<std::mutex> lk(ids_mtx_);
    return std::find(ids_.begin(), ids_.end(), me) != ids_.end();
}

size_t TaskExecutor::Pending() const {
    return pending_.load(std::memory_order_relaxed);
}

void TaskExecutor::Stop() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!running_.exchange(false)) return;
    }
    cv_.notify_all();
    for (auto &t : threads_) {
        if (t.joinable()) t.join();
    }
    threads_.clear();
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.clear();
    pending_.store(0, std::memory_order_relaxed);
}

}  // namespace nf
