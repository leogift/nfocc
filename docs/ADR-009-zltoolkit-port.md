# 从 ZLToolKit 移植的取舍（ADR-009）

参考：https://github.com/ZLMediaKit/ZLToolKit (MIT)
读了 `src/Thread`、`src/Poller`、`src/Util` 三个目录，逐项判断是否适用于本项目。

**判断依据不是"它写得好"，而是"本项目有没有这个需求"**。ZLToolKit 是流媒体服务器
（大量 socket、rtmp/rtsp 协议栈），本项目是 5 Hz 定频的感知管线，
两者的压力点完全不同。照搬它的 socket 框架只会引入无用的复杂度。

---

## 结论总表

| ZLToolKit 组件 | 本项目是否需要 | 处置 |
|---|---|---|
| `TaskExecutor` / `ThreadPool` | **需要**：按相机并行、任务取消 | 移植精简版 `nf::TaskExecutor` |
| `TaskQueue.h`（无锁队列） | **需要**：帧队列跨线程传递 | 自行实现（它的依赖 `List.h` 过重） |
| `EventPoller`（epoll/kqueue 轮询） | **不需要**：本项目没有大量 fd | 不移植 |
| `Timer` / `doDelayTask` | **部分需要**：超时看门狗 | 简化成 `nf::Watchdog` |
| `RingBuffer.h`（GOP 缓存） | **不需要**：感知不缓存历史帧 | 不移植 |
| `RingBuffer` 的单写多读模型 | **部分需要**：帧池 | 移植其**思路**，不是代码 |
| `Util/logger.h` | **需要**：分级日志 + 异步落盘 | 移植思路，重写 |
| `Util/util.h::exDemangle` 等 | **部分需要** | 按需取用 |
| `Util/NoticeCenter`（事件总线） | **需要**：模块间解耦通知 | 移植精简版 |
| `Util/TimeTicker.h`（耗时统计） | **需要**：逐阶段耗时 | 移植精简版 |
| `Util/SpeedStatistic` | **不需要** | 不移植 |
| `Util/ResourcePool.h`（对象池） | **需要**：帧缓冲复用，避免运行期 malloc | 移植精简版 |
| `Util/onceToken.h` | **需要**：一处初始化 | 移植 |
| `Util/function_traits.h` | **不需要**：C++11 下用途有限 | 不移植 |
| `Poller/Pipe` | **不需要**：没有跨 fd 唤醒需求 | 不移植 |
| `Poller/SelectWrap` | **不需要** | 不移植 |

---

## ADR-009.1 TaskExecutor：移植，但去掉模板魔法

ZLToolKit 的 `TaskCancelableImp<R(ArgTypes...)>` 用可变模板 + `std::enable_if`
做返回值转换，代码量约 180 行，可读性差。本项目只需要：

```cpp
class TaskExecutor {
public:
    explicit TaskExecutor(const char* name, size_t threads = 1);
    // 投递任务。may_sync=true 且当前就在本线程 -> 直接执行(避免死锁)
    void Async(std::function<void()> task, bool may_sync = true);
    // 投递到队首(用于优先级高的事件)
    void AsyncFirst(std::function<void()> task);
    // 等待队列清空
    void Sync();
    bool IsCurrentThread() const;
    size_t Pending() const;
};
```

**保留的关键语义**：
1. `may_sync` —— 若已在目标线程则**直接调用**而非入队。这一条防的是一类
   典型死锁：线程内投递并等待自己。ZLToolKit 实现得很对，直接沿用。
2. `AsyncFirst` —— 故障/冻结事件需要优先于常规帧处理。
3. 线程名 —— 上板后 `top -H` 才能分清哪个线程烧 CPU。

**去掉的**：
- 模板参数推导与返回值包装（本项目任务都是 `void()`）。
- `ThreadLoadCounter`（CPU 负载统计）—— 板上用 `/proc/stat` 更准。

---

## ADR-009.2 帧池：借鉴 RingBuffer 的思路，不搬代码

ZLToolKit 的 `RingBuffer` 是为"GOP 缓存 + 多播放器追赶"设计的，有
`max_gop_size`、key frame 标记、读指针共享等概念。本项目只需要"避免每帧 malloc"。

采用它的**单写多读 + 引用计数**思想，实现 `nf::FramePool`：

```cpp
class FramePool {
public:
    // 取一个可写帧(从空闲链表拿, 无则新建)。引用计数=1
    FramePtr Acquire(nf_u32 w, nf_u32 h);
    // 归还: 引用计数归零时回到空闲链表, 不 free
    void Release(Frame*);
    size_t InUse() const;
};
```

**为什么不照搬 RingBuffer**：它会缓存历史帧(默认 512 帧 × 6 MB = 3 GB),
本项目 5 Hz 实时处理, 历史帧只会变成延迟。

---

## ADR-009.3 NoticeCenter：移植，但改成类型安全

ZLToolKit 的 `NoticeCenter` 用 `std::string` 做事件键，`Any` 传参，
运行时才发现类型错误。本项目事件少(冻结/故障/丢帧)，用枚举 + 结构体：

```cpp
enum class Event { kFrozen, kCameraFault, kFrameDropped, kPoolExhausted };
using EventHandler = std::function<void(Event, const EventData&)>;
void Subscribe(Event, EventHandler);
void Publish(Event, const EventData&);
```

**为什么值得移植**：pipeline 与日志/监控解耦。没有事件总线，pipeline
就得直接调用日志模块，测试时没法拦截。

---

## ADR-009.4 TimeTicker：移植，逐阶段耗时

ZLToolKit 的 `Ticker` 是 RAII 计时器，用起来是：

```cpp
{
    Ticker ticker;
    // ... 某阶段
    auto ms = ticker.elapsedTime();
}
```

这个模式对本项目**直接有用**：SPEC-BUILD §6 要求"每帧必带 frame_id 与
逐阶段耗时"。移植成 `nf::ScopedTimer`，并接上 P50/P95 统计。

---

## ADR-009.5 EventPoller：不移植，理由要说清

`EventPoller` 是 ZLToolKit 的核心(epoll/kqueue/select 抽象 + 定时器 +
跨线程唤醒 pipe)。本项目**没有**这个需求：

| ZLToolKit 的动机 | 本项目的情况 |
|---|---|
| 同时服务上千 socket | ZMQ 内部已管理 fd，上层一个 SUB 一个 PUB |
| 事件驱动、无固定节拍 | 5 Hz 定频，节拍由上游帧到达决定 |
| 需要毫秒级定时器 | 只有超时看门狗，`std::condition_variable::wait_for` 够用 |

**如果硬搬**，会引入：kqueue/epoll 两套实现、pipe 唤醒、fd 生命周期管理，
而这些对本项目 0 收益。判断依据是需求，不是"它写得精妙"。

---

## 移植的许可与归属

ZLToolKit 是 MIT 许可，允许修改与再分发。移植部分在文件头注明：

```
// 思路源自 ZLToolKit (https://github.com/ZLMediaKit/ZLToolKit, MIT License)
// 本项目按需求做了精简与改写, 原始版权归 ZLToolKit 作者
```

**不逐字复制代码**：一是本项目 C++11 + 无异常，与它的环境不同；二是逐字
复制会带入它 `Util/List.h` 等重依赖。
