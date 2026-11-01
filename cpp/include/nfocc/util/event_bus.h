/**
 * @file event_bus.h
 * @brief 事件总线: pipeline 与日志/监控解耦。
 *
 * 思路源自 ZLToolKit src/Util/NoticeCenter.h
 * (https://github.com/ZLMediaKit/ZLToolKit, MIT License)。
 *
 * ## 与 NoticeCenter 的差异
 * NoticeCenter 用 std::string 做键 + Any 传参, 类型错误要运行时才炸。
 * 本项目事件只有几种, 改成 enum class + 固定结构体, 编译期即可查错。
 *
 * ## 为什么需要它
 * 没有事件总线, pipeline 必须直接调用日志模块 —— 测试时无法拦截,
 * 也做不到"故障时只通知不打印"。有了总线, 单测可以订阅事件做断言。
 */
#ifndef NFOCC_EVENT_BUS_H
#define NFOCC_EVENT_BUS_H

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

namespace nf {

enum class Event {
    kFrozen = 0,        /**< 基线不足/超限, 本帧不产出 */
    kCameraFault,       /**< 单相机失效(外参、引擎错误) */
    kFrameDropped,      /**< 丢帧 */
    kPoolExhausted,     /**< 帧池耗尽 */
    kEngineRange,       /**< 引擎容量不足 (NF_E_ENGINE_RANGE) */
    kCount
};

struct EventData {
    Event event = Event::kFrozen;
    uint32_t frame_id = 0;
    uint8_t cam_id = 0;
    int32_t code = 0;        /**< 对应的 NFStatus */
    double value = 0.0;      /**< 附加量(如基线 mm) */
};

using EventHandler = std::function<void(const EventData &)>;

class EventBus {
public:
    /** 订阅。返回订阅 id, 可用于取消。 */
    uint64_t Subscribe(Event e, EventHandler h);
    void Unsubscribe(uint64_t id);

    /** 发布。同步调用所有订阅者 —— 不做异步, 避免顺序不确定。 */
    void Publish(const EventData &d);

    /** 清空(测试用)。 */
    void Clear();

    static EventBus &Instance();

private:
    struct Entry {
        uint64_t id;
        Event e;
        EventHandler h;
    };
    std::mutex mtx_;
    std::vector<Entry> handlers_;
    uint64_t next_id_ = 1;
};

const char *EventName(Event e);

}  // namespace nf

#endif  // NFOCC_EVENT_BUS_H
