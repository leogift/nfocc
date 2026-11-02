/**
 * @file viewplan.cpp
 * @brief 6 视口规划表（分类张角）实现。
 */
#include "nfocc/nfocc_viewplan.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstring>

namespace {

/** 角度差归一化到 [-180, 180] */
float ang_diff(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

}  // namespace

extern "C" {

nf_u32 nf_viewplan_default(NFViewPlanEntry *out, nf_u32 max_entries) {
    if (!out || max_entries == 0) return 0;

    /* 前后用 90°，左右用 150° —— 见头文件说明 */
    const float hf_front = NF_V_VIEW_HFOV_DEG_FRONT;
    const float hf_side  = NF_V_VIEW_HFOV_DEG_SIDE;
    const float vf       = NF_V_VIEW_VFOV_DEG;

    const NFViewPlanEntry all[6] = {
        /* cam  yaw     hfov     vfov  side */
        { 0u,  -45.0f, hf_front, vf, 0u, {0,0,0} },   /* FV 左前 */
        { 0u,  +45.0f, hf_front, vf, 0u, {0,0,0} },   /* FV 右前 */
        { 1u, -135.0f, hf_front, vf, 0u, {0,0,0} },   /* RV 左后 */
        { 1u, +135.0f, hf_front, vf, 0u, {0,0,0} },   /* RV 右后 */
        { 2u,  +90.0f, hf_side,  vf, 1u, {0,0,0} },   /* ML */
        { 3u,  -90.0f, hf_side,  vf, 1u, {0,0,0} },   /* MR */
    };

    const nf_u32 n = (max_entries < 6u) ? max_entries : 6u;
    std::memcpy(out, all, (size_t)n * sizeof(NFViewPlanEntry));
    return n;
}

const NFViewPlanEntry *nf_viewplan_find(const NFViewPlanEntry *plan, nf_u32 n,
                                        nf_u32 cam_id, nf_f32 yaw_deg) {
    if (!plan) return nullptr;
    const NFViewPlanEntry *best = nullptr;
    float best_d = 1e9f;
    for (nf_u32 i = 0; i < n; ++i) {
        if (plan[i].cam_id != cam_id) continue;
        const float d = std::fabs(ang_diff(plan[i].yaw_deg, yaw_deg));
        if (d < best_d) { best_d = d; best = &plan[i]; }
    }
    /* 容差 1°：调用方可能传入 45.0001 这种 */
    return (best && best_d < 1.0f) ? best : nullptr;
}

} /* extern "C" */
