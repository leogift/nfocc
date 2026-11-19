/**
 * @file nfocc_odom.h
 * @brief 里程计积分：由 speed + yaw_rate 生成双基线需要的绝对位姿。
 *
 * ## 为什么需要这个模块
 * `NFBaselineManager` 要的是 `NFBaselinePose`（车体**大地系**位姿：
 * 3x3 旋转 + 3 位移）。而 pipeline 从上游只拿到 `t_cam`（相邻帧
 * 相对位移的标量），不是位姿。
 *
 * 这个模块是**降级方案**：用 `speed × dt` 和 `yaw_rate × dt` 把里程
 * 积分成位姿。它的精度边界必须说清（见下），它只是让双基线能跑起来，
 * 不是替代真正的定位。
 *
 * ## 精度边界（这是降级，不是等效）
 *
 * | 情形 | 后果 |
 * |---|---|
 * | 匀速直行 | 速度积分准（误差 = 轮速误差 ~2%），可用 |
 * | 急转向 | yaw_rate 有延迟与噪声，航向累积误差；**而且打滑时轮速根本不对** |
 * | 原地打滑 | 轮速说有位移，实际没动 —— 位姿完全错，双基线会产出错误深度 |
 * | 上下坡 | speed 是轮速不是车速的水平分量，会有 cos(pitch) 的系统偏差 |
 *
 * 结论：本模块只用于**原型验证**。产品必须接真实定位（视觉里程计/
 * GNSS-INS）。接口留了 `nf_odom_push_pose()` 让外部直接注入真位姿。
 *
 * ## 与 NFPose 的区别
 * `NFPose` 是**相机相对车体**的外参；`NFBaselinePose` 是**车体在大地系**
 * 的位姿。两者语义完全不同，不能混用。
 */
#ifndef NFOCC_ODOM_H
#define NFOCC_ODOM_H

#include "nfocc/nfocc_baseline.h"
#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_ODOM_VERSION 1u

typedef struct NFOdomCtx NFOdomCtx;

typedef struct {
    nf_u32 version;         /**< = NF_ODOM_VERSION */
    nf_u32 source;          /**< 0=积分(降级) 1=外部注入 */
} NFOdomConfig;

/** 上游给的里程增量（每帧一份） */
typedef struct {
    nf_u32 version;
    nf_f32 speed_mmps;      /**< 车速 (mm/s)，非负 */
    nf_f32 yaw_rate_dps;    /**< 偏航率 (deg/s)，左正右负 */
    nf_f32 dt_s;            /**< 距上一帧的时间 (s) */
} NFOdomInput;

NFStatus nf_odom_create(NFOdomCtx **ctx, const NFOdomConfig *cfg);
void     nf_odom_destroy(NFOdomCtx *ctx);

/** 推入一帧里程增量，内部积分并更新当前位姿。 */
NFStatus nf_odom_push(NFOdomCtx *ctx, const NFOdomInput *in);

/** 取当前绝对位姿（拷贝给调用方） */
NFStatus nf_odom_current(const NFOdomCtx *ctx, NFBaselinePose *out);

/**
 * 外部注入绝对位姿（真定位接入时用）。
 *
 * 调用后 `source` 自动切到 1，后续 `nf_odom_push` 不再积分 ——
 * 避免两套来源互相覆盖。
 */
NFStatus nf_odom_push_pose(NFOdomCtx *ctx, const NFBaselinePose *pose);

/** 累计航向 (rad) 与行程 (mm)，用于诊断漂移量级 */
NFStatus nf_odom_stats(const NFOdomCtx *ctx, nf_f32 *out_yaw_rad,
                       nf_f32 *out_dist_mm, nf_u32 *out_frames);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_ODOM_H */
