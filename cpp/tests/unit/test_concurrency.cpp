/**
 * @file test_concurrency.cpp
 * @brief TaskExecutor / FramePool / EventBus 的并发正确性测试。
 *
 * 这些组件移植自 ZLToolKit 的思路, 但**自己重写了**。并发代码"看起来对"
 * 是最危险的 —— 每条判据都设计成能真实失败。
 */
#include "nf_test.h"
#include "nfocc/util/task_executor.h"
#include "nfocc/util/frame_pool.h"
#include "nfocc/util/event_bus.h"
#include "nfocc/util/timing.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace {

/* ---------- TaskExecutor ---------- */

/* 任务全部执行: 计数器必须等于投递数 */
NF_TEST(te_all_tasks_run) {
    nf::TaskExecutor ex("t", 4);
    std::atomic<int> n{0};
    const int N = 1000;
    for (int i = 0; i < N; ++i) ex.Async([&] { n.fetch_add(1); }, false);
    ex.Sync();
    NF_CHECK_EQ_INT(n.load(), N);
    NF_CHECK_EQ_INT(ex.Pending(), 0);
}

/* may_sync: 在工作线程内投递不得入队死锁, 必须直接执行 */
NF_TEST(te_may_sync_no_deadlock) {
    nf::TaskExecutor ex("t", 1);
    std::atomic<int> inner{0};
    std::atomic<bool> done{false};

    ex.Async([&] {
        // 此时已在工作线程: may_sync=true 应直接执行, 不能排队等自己
        ex.Async([&] { inner.fetch_add(1); }, true);
        done.store(true);
    }, false);

    // 若 may_sync 失效, 这里会死等
    ex.Sync();
    NF_CHECK_EQ_INT(done.load(), 1);
    NF_CHECK_EQ_INT(inner.load(), 1);
}

/* Sync 必须在所有任务执行完后才返回 */
NF_TEST(te_sync_waits) {
    nf::TaskExecutor ex("t", 2);
    std::atomic<int> n{0};
    for (int i = 0; i < 200; ++i) {
        ex.Async([&] {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            n.fetch_add(1);
        }, false);
    }
    ex.Sync();
    NF_CHECK_EQ_INT(n.load(), 200);   // Sync 返回时已全部完成
}

/* AsyncFirst 必须插到队首: 串行执行器上先跑 */
NF_TEST(te_async_first_priority) {
    nf::TaskExecutor ex("t", 1);
    std::vector<int> order;
    std::mutex mtx;
    std::atomic<bool> gate{false};

    // 先塞一个阻塞任务, 让后续任务排队
    ex.Async([&] {
        while (!gate.load()) std::this_thread::yield();
    }, false);
    // 排队: 正常任务
    ex.Async([&] { std::lock_guard<std::mutex> l(mtx); order.push_back(1); }, false);
    ex.Async([&] { std::lock_guard<std::mutex> l(mtx); order.push_back(2); }, false);
    // 插队: 高优先级
    ex.AsyncFirst([&] { std::lock_guard<std::mutex> l(mtx); order.push_back(99); });

    gate.store(true);
    ex.Sync();

    NF_CHECK(order.size() == 3);
    if (order.size() == 3) {
        NF_CHECK_MSG(order[0] == 99, "AsyncFirst 的任务必须最先执行");
    }
}

/* threads=0: 不建线程, 任务在当前线程同步执行(单线程 SIL 调试用) */
NF_TEST(te_zero_threads_inline) {
    nf::TaskExecutor ex("t", 0);
    std::atomic<int> n{0};
    const std::thread::id me = std::this_thread::get_id();
    std::atomic<bool> same{false};
    ex.Async([&] {
        n.fetch_add(1);
        same.store(std::this_thread::get_id() == me);
    });
    NF_CHECK_EQ_INT(n.load(), 1);
    NF_CHECK_EQ_INT(same.load(), 1);
    NF_CHECK(ex.IsCurrentThread());
}

/* IsCurrentThread 在外部线程为 false */
NF_TEST(te_is_current_thread) {
    nf::TaskExecutor ex("t", 1);
    std::atomic<int> r{-1};
    ex.Async([&] { r.store(ex.IsCurrentThread() ? 1 : 0); }, false);
    ex.Sync();
    NF_CHECK_EQ_INT(r.load(), 1);
    NF_CHECK_MSG(!ex.IsCurrentThread(), "外部线程不应被判为工作线程");
}

/* ---------- FramePool ---------- */

/* 预分配: 构造后全部空闲 */
NF_TEST(fp_prealloc) {
    nf::FramePool pool(8, 64, 32);
    NF_CHECK_EQ_INT(pool.Capacity(), 8);
    NF_CHECK_EQ_INT(pool.Free(), 8);
    NF_CHECK_EQ_INT(pool.InUse(), 0);
}

/* 取空后返回 nullptr, 且计数正确 */
NF_TEST(fp_exhaust) {
    nf::FramePool pool(4, 16, 16);
    std::vector<nf::Frame *> got;
    for (int i = 0; i < 4; ++i) got.push_back(pool.Acquire());
    for (auto *f : got) NF_CHECK(f != nullptr);
    NF_CHECK_EQ_INT(pool.InUse(), 4);
    NF_CHECK_EQ_INT(pool.Free(), 0);

    nf::Frame *extra = pool.Acquire();
    NF_CHECK_MSG(extra == nullptr, "池空时应返回 nullptr");
    NF_CHECK_EQ_INT(pool.Exhausted(), 1);

    for (auto *f : got) pool.Release(f);
    NF_CHECK_EQ_INT(pool.Free(), 4);
}

/* 引用计数: 多引用时 Release 一次不得回池 */
NF_TEST(fp_refcount) {
    nf::FramePool pool(2, 8, 8);
    nf::Frame *a = pool.Acquire();
    NF_CHECK(a != nullptr);
    NF_CHECK_EQ_INT(a->refs.load(), 1);

    pool.AddRef(a);
    NF_CHECK_EQ_INT(a->refs.load(), 2);

    pool.Release(a);
    NF_CHECK_EQ_INT(a->refs.load(), 1);
    NF_CHECK_MSG(pool.Free() == 1, "仍有引用时不得回池");

    pool.Release(a);
    NF_CHECK_EQ_INT(a->refs.load(), 0);
    NF_CHECK_EQ_INT(pool.Free(), 2);
}

/* 复用: 同一帧被反复取还, 指针不变(证明没有 malloc) */
NF_TEST(fp_reuse_same_ptr) {
    nf::FramePool pool(1, 8, 8);
    nf::Frame *first = pool.Acquire();
    NF_CHECK(first != nullptr);
    pool.Release(first);
    nf::Frame *second = pool.Acquire();
    NF_CHECK_MSG(first == second, "池化应复用同一块内存");
    pool.Release(second);
}

/* 多线程并发取还: 总引用守恒, 不崩 */
NF_TEST(fp_concurrent) {
    nf::FramePool pool(16, 32, 32);
    std::atomic<int> fails{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < 4; ++t) {
        ts.emplace_back([&] {
            for (int i = 0; i < 2000; ++i) {
                nf::Frame *f = pool.Acquire();
                if (!f) { fails.fetch_add(1); continue; }
                f->frame_id = (unsigned)i;
                pool.Release(f);
            }
        });
    }
    for (auto &t : ts) t.join();
    // 池满时取不到是正常的; 关键是结束后全部归还
    NF_CHECK_EQ_INT(pool.InUse(), 0);
    NF_CHECK_EQ_INT(pool.Free(), 16);
}

/* ---------- EventBus ---------- */

/* 订阅后能收到; 取消后收不到 */
NF_TEST(eb_subscribe_unsubscribe) {
    nf::EventBus bus;
    std::atomic<int> n{0};
    auto id = bus.Subscribe(nf::Event::kFrozen, [&](const nf::EventData &) {
        n.fetch_add(1);
    });
    NF_CHECK(id != 0);

    nf::EventData d;
    d.event = nf::Event::kFrozen;
    bus.Publish(d);
    NF_CHECK_EQ_INT(n.load(), 1);

    bus.Unsubscribe(id);
    bus.Publish(d);
    NF_CHECK_MSG(n.load() == 1, "取消订阅后不应再收到");
}

/* 不同事件互不干扰 */
NF_TEST(eb_event_filter) {
    nf::EventBus bus;
    std::atomic<int> frozen{0}, fault{0};
    bus.Subscribe(nf::Event::kFrozen, [&](const nf::EventData &) { frozen.fetch_add(1); });
    bus.Subscribe(nf::Event::kCameraFault, [&](const nf::EventData &) { fault.fetch_add(1); });

    nf::EventData d;
    d.event = nf::Event::kCameraFault;
    bus.Publish(d);
    NF_CHECK_EQ_INT(frozen.load(), 0);
    NF_CHECK_EQ_INT(fault.load(), 1);
}

/* 回调里取消自己不得死锁/崩 (快照遍历的关键) */
NF_TEST(eb_unsubscribe_in_callback) {
    nf::EventBus bus;
    std::atomic<int> n{0};
    uint64_t id = 0;
    id = bus.Subscribe(nf::Event::kFrozen, [&](const nf::EventData &) {
        n.fetch_add(1);
        bus.Unsubscribe(id);      // 在回调里取消自己
    });
    nf::EventData d;
    d.event = nf::Event::kFrozen;
    bus.Publish(d);
    NF_CHECK_EQ_INT(n.load(), 1);
    bus.Publish(d);               // 第二次不应再触发
    NF_CHECK_EQ_INT(n.load(), 1);
}

/* 事件名不能为空 */
NF_TEST(eb_event_names) {
    NF_CHECK(nf::EventName(nf::Event::kFrozen) != nullptr);
    NF_CHECK(nf::EventName(nf::Event::kEngineRange) != nullptr);
}

/* ---------- Timing ---------- */

/* NowUs 单调不减 */
NF_TEST(tm_now_us_monotonic) {
    uint64_t a = nf::NowUs();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    uint64_t b = nf::NowUs();
    NF_CHECK_MSG(b > a, "NowUs 必须单调递增");
    NF_CHECK_MSG(b - a >= 1000, "2ms 睡眠应 >= 1000us");
}

/* ScopedTimer 在析构时写值 */
NF_TEST(tm_scoped_timer) {
    double ms = -1.0;
    {
        nf::ScopedTimer t(&ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    NF_CHECK_MSG(ms >= 0.5, "析构应写入耗时");
}

/* StageStats 分位数 */
NF_TEST(tm_stage_stats) {
    nf::StageStats s(100);
    for (int i = 1; i <= 100; ++i) s.Add((double)i);
    NF_CHECK_EQ_INT(s.Count(), 100);
    NF_CHECK_NEAR(s.P50(), 50.0, 2.0);
    NF_CHECK_NEAR(s.Max(), 100.0, 1.0);
    NF_CHECK_MSG(s.P95() >= s.P50(), "P95 应 >= P50");

    /* 空统计返回 0 */
    nf::StageStats e(16);
    NF_CHECK_NEAR(e.P50(), 0.0, 1e-9);
}

/* 滑窗只保留最近 window 个 */
NF_TEST(tm_stage_window) {
    nf::StageStats s(10);
    for (int i = 0; i < 100; ++i) s.Add(1000.0);   // 旧的
    for (int i = 0; i < 10; ++i) s.Add(5.0);       // 新的覆盖全部
    NF_CHECK_EQ_INT(s.Count(), 10);
    NF_CHECK_MSG(s.Max() <= 5.0 + 1e-9, "滑窗应已丢弃旧样本");
}

}  // namespace

int main() { return nftest::run_all(); }
