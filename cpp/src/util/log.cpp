/**
 * @file log.cpp
 * @brief 日志实现 (SPEC-BUILD §6)。全局回调, 无动态分配。
 */
#include "nfocc/nfocc_types.h"

#include <cstdarg>
#include <cstdio>

namespace {
NFLogFn g_fn = nullptr;
void   *g_user = nullptr;
}  // namespace

extern "C" {

/* 状态码转字符串。放在基础层(camera 等模块都会用到, 且测试需要它) */
const char *nf_status_str(NFStatus s) {
    switch (s) {
    case NF_OK:                   return "NF_OK";
    case NF_E_PARAM_NULL:         return "NF_E_PARAM_NULL";
    case NF_E_PARAM_RANGE:        return "NF_E_PARAM_RANGE";
    case NF_E_PARAM_VERSION:      return "NF_E_PARAM_VERSION";
    case NF_E_CAM_NOT_CALIBRATED: return "NF_E_CAM_NOT_CALIBRATED";
    case NF_E_CAM_DISTORT_INV:    return "NF_E_CAM_DISTORT_INV";
    case NF_E_CAM_OUT_OF_VALID:   return "NF_E_CAM_OUT_OF_VALID";
    case NF_E_EXTRINSIC_INVALID:  return "NF_E_EXTRINSIC_INVALID";
    case NF_E_BASELINE_IDLE:      return "NF_E_BASELINE_IDLE";
    case NF_E_BASELINE_OVER:      return "NF_E_BASELINE_OVER";
    case NF_E_BASELINE_ROTATION:  return "NF_E_BASELINE_ROTATION";
    case NF_E_BASELINE_DEGRADED:  return "NF_E_BASELINE_DEGRADED";
    case NF_E_ENGINE_NOT_INIT:    return "NF_E_ENGINE_NOT_INIT";
    case NF_E_ENGINE_RANGE:       return "NF_E_ENGINE_RANGE";
    case NF_E_ENGINE_OOM:         return "NF_E_ENGINE_OOM";
    case NF_E_ENGINE_UNSUPPORTED: return "NF_E_ENGINE_UNSUPPORTED";
    case NF_E_VOXEL_FULL:         return "NF_E_VOXEL_FULL";
    case NF_E_OUTPUT_OVERFLOW:    return "NF_E_OUTPUT_OVERFLOW";
    }
    return "NF_E_UNKNOWN";
}


void nf_log_set(NFLogFn fn, void *user) {
    g_fn = fn;
    g_user = user;
}
NFLogFn nf_log_get(void) { return g_fn; }
void   *nf_log_get_user(void) { return g_user; }

void nf_log_msg(NFLogLevel level, const char *fmt, ...) {
    if (!g_fn || !fmt) return;
    /* 固定缓冲, 不做堆分配 —— 实时路径可用 */
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    buf[sizeof(buf) - 1] = '\0';
    g_fn(level, buf, g_user);
}

} /* extern "C" */
