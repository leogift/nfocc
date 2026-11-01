/**
 * @file task_executor.h
 * @brief 任务执行器: 带线程绑定的 FIFO 任务队列。
 *
 * 思路源自 ZLToolKit src/Thread/TaskExecutor.h
 * (https://github.com/ZLMediaKit/ZLToolKit, MIT License), 按本项目需求精简。
 *
 * ## 保留的三条关键语义(理由都来自实际故障)
 *
 * 1. **may_sync**: 若当前已在目标线程, 直接执行而不入队。
 *    防的是"线程内投递再等待自己"的死锁 —— 这是任务队列最典型的坑。
 *
 * 2. **AsyncFirst**: 投递到队首。故障/冻结事件必须优先于常规帧处理,
 *    否则一帧 SGBM(几十 ms)会把故障通知压在后面。
 *
 * 3. **线程名**: 上板后 `top -H -p <pid>` 要看得出哪个线程在烧 CPU。
 *
 * ## 与 ZLToolKit 的差异
 *   - 去掉可变模板与返回值包装(本项目任务都是 void())
 *   - 去掉 ThreadLoadCounter(板上用 /proc/stat 更准)
 *   - 不抛异常: 队列满/线程启动失败用返回值表示
 *
 * ## 用法
 * ```cpp
 * nf::TaskExecutor ex("sgbm", 4);
 * ex.Async([&]{ do_work(); });
 * ex.Sync();          // 等队列清空
 * ```
 */
#ifndef NFOCC_TASK_EXECUTOR_H
#define NFOCC_TASK_EXECUTOR_H

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nf {

class TaskExecutor {
public:
    /**
     * @param name    线程名前缀(给 top -H 看)
     * @param threads 线程数; 0 表示**不创建线程**, 只保留当前线程标识
     *                (调用 Async 时直接执行, 用于单线程 SIL 调试)
     */
    explicit TaskExecutor(const char *name, size_t threads = 1);
    ~TaskExecutor();

    TaskExecutor(const TaskExecutor &) = delete;
    TaskExecutor &operator=(const TaskExecutor &) = delete;

    /** 投递到队尾。may_sync=true 且当前在本线程 -> 直接执行。 */
    void Async(std::function<void()> task, bool may_sync = true);

    /** 投递到队首(高优先级)。语义同 Async。 */
    void AsyncFirst(std::function<void()> task);

    /** 等待队列清空并所有任务执行完。 */
    void Sync();

    /** 当前线程是否就是本执行器的工作线程。 */
    bool IsCurrentThread() const;

    /** 待处理任务数(近似值, 供监控)。 */
    size_t Pending() const;

    /** 线程数。 */
    size_t ThreadCount() const { return threads_.size(); }

    /** 停止: 清空队列并 join 所有线程。析构会自动调用。 */
    void Stop();

private:
    void Run();

    std::string name_;
    std::vector<std::thread> threads_;
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::condition_variable cv_done_;
    std::deque<std::function<void()>> queue_;
    std::atomic<size_t> pending_{0};
    std::atomic<bool> running_{true};
    std::vector<std::thread::id> ids_;   /**< 本执行器线程的 id 集合 */
    mutable std::mutex ids_mtx_;
};

}  // namespace nf

#endif  // NFOCC_TASK_EXECUTOR_H
