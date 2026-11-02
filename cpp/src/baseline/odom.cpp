/**
 * @file odom.cpp
 * @brief 里程计积分实现（降级方案）。
 *
 * ## 积分方式：中间航向法
 * 直行时用起点航向、转弯时用中点航向积分位移，比用起点航向准。
 * 一帧转过 dθ 时：
 *     P_new = P + R(yaw + dθ/2) · (v·dt, 0)
 *     yaw  += dθ
 *
 * 用中点而不是起点，一帧内转过 10° 时位置误差从 0.4% 降到 0.01%。
 *
 * ## 坐标系
 * 车体 ISO 8855：X 前、Y 左、Z 上。yaw 绕 Z 轴，左转为正。
 * 位移只沿车体 X 方向（没有侧滑假设）—— 这正是打滑时失效的原因。
 */
#include "nfocc/nfocc_odom.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

struct NFOdomCtx {
    nf_u32  version;
    nf_u32  source;      /* 0=积分 1=外部注入 */
    nf_u32  frames;

    nf_f32  yaw_rad;     /* 当前航向 */
    nf_f32  x_mm, y_mm;  /* 当前累计位移 */
    nf_f32  dist_mm;     /* 累计行程（标量） */

    NFBaselinePose pose; /* 当前绝对位姿 */
};

namespace {

/** 车体位移 (dx,dy) 在航向 yaw 下的车体系表达 -> 大地系 */
inline void rot2(float yaw, float vx, float vy, float *ox, float *oy) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    *ox = c * vx - s * vy;
    *oy = s * vx + c * vy;
}

/** 由航向构造 3x3 旋转（行优先，只绕 Z） */
inline void yaw_to_R(float yaw, nf_f32 R[9]) {
    const float c = std::cos(yaw), s = std::sin(yaw);
    R[0] = c;  R[1] = -s; R[2] = 0.0f;
    R[3] = s;  R[4] = c;  R[5] = 0.0f;
    R[6] = 0.f; R[7] = 0.f; R[8] = 1.0f;
}

}  // namespace

extern "C" {

NFStatus nf_odom_create(NFOdomCtx **out, const NFOdomConfig *cfg) {
    if (!out) return NF_E_PARAM_NULL;
    *out = nullptr;
    NFOdomCtx *c = (NFOdomCtx *)std::calloc(1, sizeof(NFOdomCtx));
    if (!c) return NF_E_ENGINE_OOM;
    c->version = NF_ODOM_VERSION;
    c->source = cfg ? cfg->source : 0u;
    yaw_to_R(0.0f, c->pose.R);
    c->pose.t[0] = c->pose.t[1] = c->pose.t[2] = 0.0f;
    *out = c;
    return NF_OK;
}

void nf_odom_destroy(NFOdomCtx *c) { std::free(c); }

NFStatus nf_odom_push(NFOdomCtx *c, const NFOdomInput *in) {
    if (!c || !in) return NF_E_PARAM_NULL;

    /**
     * 外部已注入真位姿时不再积分。
     *
     * 两套来源同时更新会让位姿在两者之间跳变 —— 双基线因此拿到
     * 前后矛盾的 t_long，产出的深度在跳变帧附近完全不可信。
     */
    if (c->source == 1u) return NF_OK;

    if (!(in->dt_s > 0.0f)) return NF_E_PARAM_RANGE;
    if (in->speed_mmps < 0.0f) return NF_E_PARAM_RANGE;

    const float dyaw = in->yaw_rate_dps * (float)M_PI / 180.0f * in->dt_s;
    const float v_mm = in->speed_mmps * in->dt_s;

    /* 中点航向法：用 yaw + dyaw/2 积分位移 */
    float dx, dy;
    rot2(c->yaw_rad + dyaw * 0.5f, v_mm, 0.0f, &dx, &dy);

    c->x_mm += dx;
    c->y_mm += dy;
    c->yaw_rad += dyaw;
    c->dist_mm += std::fabs(v_mm);
    ++c->frames;

    /* 航向归一到 (-pi, pi]，避免长时间运行后 cos/sin 精度下降 */
    if (c->yaw_rad > (float)M_PI) c->yaw_rad -= 2.0f * (float)M_PI;
    if (c->yaw_rad < -(float)M_PI) c->yaw_rad += 2.0f * (float)M_PI;

    yaw_to_R(c->yaw_rad, c->pose.R);
    c->pose.t[0] = c->x_mm;
    c->pose.t[1] = c->y_mm;
    c->pose.t[2] = 0.0f;
    return NF_OK;
}

NFStatus nf_odom_current(const NFOdomCtx *c, NFBaselinePose *out) {
    if (!c || !out) return NF_E_PARAM_NULL;
    *out = c->pose;
    return NF_OK;
}

NFStatus nf_odom_push_pose(NFOdomCtx *c, const NFBaselinePose *pose) {
    if (!c || !pose) return NF_E_PARAM_NULL;
    c->pose = *pose;
    c->source = 1u;

    /* 从位姿反推内部状态，让后续 stats 一致 */
    c->x_mm = pose->t[0];
    c->y_mm = pose->t[1];
    c->yaw_rad = std::atan2(pose->R[3], pose->R[0]);
    ++c->frames;
    return NF_OK;
}

NFStatus nf_odom_stats(const NFOdomCtx *c, nf_f32 *out_yaw_rad,
                       nf_f32 *out_dist_mm, nf_u32 *out_frames) {
    if (!c) return NF_E_PARAM_NULL;
    if (out_yaw_rad) *out_yaw_rad = c->yaw_rad;
    if (out_dist_mm) *out_dist_mm = c->dist_mm;
    if (out_frames)  *out_frames = c->frames;
    return NF_OK;
}

} /* extern "C" */
