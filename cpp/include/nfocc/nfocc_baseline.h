/**
 * @file nfocc_baseline.h
 * @brief 自适应多帧历史与并行双基线状态机。
 *
 * ## 为什么需要它
 * 原来的 `nf_gate_decide` 是**单帧无状态**判定：只看当前帧与前一个参考帧的
 * 位移。低速时基线不足 -> 冻结；速度波动 -> 频繁冻结/解冻交替。
 * 结果是一条"周期性感知静默"的锯齿，对 APA 泊车是致命的
 * （泊车全程都在低速段）。
 *
 * ## 两个关键机制
 *
 * ### 1. 车体基准位移（Datum Delta）
 * 转弯时外侧相机线速度明显高于内侧。若每路相机各自判定，四路的
 * 参考帧会不同步，融合后体素在时间上错位。
 * 解：**统一用后轴中心的位移模长**作为门控量，四路共享同一个判定结果。
 *
 * ### 2. 并行双基线（Parallel Dual Baselines）
 * 维护两个参考帧：SHORT 和 LONG。每帧同时算两条基线：
 *   - `|t_long| >= b_target` -> 用长基线高精度输出，然后
 *     长基线退役、短基线晋升为新长基线、当前帧成为新短基线
 *   - 否则不输出，但**短基线仍随帧推进**（否则队列停滞）
 *
 * ## 与原规范的两处补充（原规范遗漏，会导致错误）
 * - **必须设 b_max**：高速时 |t_long| 可能到 1m，重叠视场不足、匹配陡崩。
 *   超过 b_max 强制重置队列。
 * - **未触发时的更新规则**：|t_long| < b_target 时，当前帧仍要推入历史并
 *   成为新的短基线参考，否则队列永远停在初始状态。
 */
#ifndef NFOCC_BASELINE_H
#define NFOCC_BASELINE_H

#include "nfocc/nfocc_types.h"
#include "nfocc/nfocc_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_BASELINE_HIST_CAP   16u     /**< 历史环形队列容量 */
#define NF_BASELINE_VERSION    1u

/** 姿态：3x3 行优先旋转 + 平移 (车体大地系) */
typedef struct {
    nf_f32 R[9];
    nf_f32 t[3];        /**< mm */
} NFBaselinePose;

/** 一帧历史记录 */
typedef struct {
    NFBaselinePose pose;
    nf_u64 ts_us;
    nf_u32 frame_id;
    nf_u8  valid;       /**< 1 = 该槽有有效记录 */
    nf_u8  _pad[3];
} NFHistoryFrame;

/** 双基线状态机 */
typedef struct {
    nf_u32 version;

    /* ---- 标定/配置 ---- */
    nf_f32 b_min_mm;        /**< 低于此值不产出 (默认 200) */
    nf_f32 b_max_mm;        /**< 高于此值强制重置 (默认 460) */
    nf_f32 b_target_mm;     /**< 长基线触发目标 (默认 300) */
    nf_f32 max_yaw_deg;     /**< 航向漂移上限 (默认 15) */

    /* ---- 历史队列 ---- */
    NFHistoryFrame hist[NF_BASELINE_HIST_CAP];
    nf_u32 head;            /**< 下一个写入位置 */
    nf_u32 count;           /**< 有效帧数 */

    /* ---- 双基线索引 ---- */
    nf_u32 idx_short;       /**< 短基线参考帧在 hist 中的下标 */
    nf_u32 idx_long;        /**< 长基线参考帧在 hist 中的下标 */
    nf_u8  has_short;
    nf_u8  has_long;
    nf_u8  _pad2[2];

    /* ---- 诊断 ---- */
    nf_u32 n_reset;         /**< 重置次数（旋转超限 / 超 b_max） */
    nf_u32 n_decide;        /**< decide 调用次数 */
} NFBaselineManager;

/** 决策输出 */
typedef struct {
    nf_u32 version;
    NFMode  mode;
    nf_f32  baseline_mm;        /**< 本次使用的基线长度；冻结时为 0 */
    nf_u32  ref_frame_id;       /**< 参考帧 id；冻结时为 0 */
    nf_f32  t_veh[3];           /**< 车体基准位移 (当前 - 参考), mm */
    nf_f32  yaw_drift_deg;      /**< 两帧航向漂移 */
    nf_u8   used_long;          /**< 1 = 用了长基线 */
    nf_u8   is_new_ref;         /**< 1 = 本帧已更新参考帧队列 */
    nf_u8   _pad[2];
} NFBaselineDecision;

#define NF_BASELINE_DECISION_VERSION 1u

/** 初始化。b_min/b_max/b_target 传 <=0 用默认 200/460/300。 */
NFStatus nf_baseline_manager_init(NFBaselineManager *mgr,
                                  nf_f32 b_min_mm, nf_f32 b_max_mm,
                                  nf_f32 b_target_mm);

/**
 * 推入一帧位姿（车体大地系）。
 *
 * 每帧到达时都应调用，**无论是否会产出**。这正是"未触发时也要更新"
 * 规则的落点 —— 队列不推进的话双基线永远无法晋升。
 */
NFStatus nf_baseline_manager_push(NFBaselineManager *mgr,
                                  const NFBaselinePose *pose,
                                  nf_u64 ts_us, nf_u32 frame_id);

/**
 * 决策：决定本帧用哪条基线、是否产出。
 *
 * 内部按优先级判定：
 *   1. 历史不足 -> FROZEN_NEAR
 *   2. 航向漂移 > max_yaw_deg -> FROZEN_ROT + **重置队列**
 *   3. |t_long| > b_max -> FROZEN_FAR + 重置队列（并尝试用短基线降级）
 *   4. |t_long| >= b_target -> STEREO，用长基线，之后晋升
 *   5. |t_short| >= b_min -> STEREO，用短基线
 *   6. 否则 -> FROZEN_NEAR
 */
NFStatus nf_baseline_manager_decide(NFBaselineManager *mgr,
                                    NFBaselineDecision *out);

/** 清空队列（保留配置）。用于外部触发重置。 */
void nf_baseline_manager_reset(NFBaselineManager *mgr);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_BASELINE_H */
