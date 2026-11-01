/**
 * @file timing_event.cpp
 * @brief 计时与事件总线的实现。
 */
#include "nfocc/util/timing.h"
#include "nfocc/util/event_bus.h"

#include <chrono>

namespace nf {

uint64_t NowUs() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<microseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------- EventBus

EventBus &EventBus::Instance() {
    static EventBus bus;
    return bus;
}

uint64_t EventBus::Subscribe(Event e, EventHandler h) {
    if (!h) return 0;
    std::lock_guard<std::mutex> lk(mtx_);
    const uint64_t id = next_id_++;
    handlers_.push_back(Entry{id, e, std::move(h)});
    return id;
}

void EventBus::Unsubscribe(uint64_t id) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (size_t i = 0; i < handlers_.size(); ++i) {
        if (handlers_[i].id == id) {
            handlers_.erase(handlers_.begin() + (long)i);
            return;
        }
    }
}

void EventBus::Publish(const EventData &d) {
    // 拷贝一份再回调: 订阅者可能在回调里 Unsubscribe, 直接遍历会失效
    std::vector<Entry> snapshot;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        snapshot = handlers_;
    }
    for (const auto &e : snapshot) {
        if (e.e == d.event && e.h) e.h(d);
    }
}

void EventBus::Clear() {
    std::lock_guard<std::mutex> lk(mtx_);
    handlers_.clear();
}

const char *EventName(Event e) {
    switch (e) {
        case Event::kFrozen:        return "FROZEN";
        case Event::kCameraFault:   return "CAM_FAULT";
        case Event::kFrameDropped:  return "FRAME_DROP";
        case Event::kPoolExhausted: return "POOL_EXHAUSTED";
        case Event::kEngineRange:   return "ENGINE_RANGE";
        case Event::kCount:         break;
    }
    return "UNKNOWN";
}

}  // namespace nf
