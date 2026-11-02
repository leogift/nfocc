/**
 * @file baseline_manager.cpp
 * @brief 自适应多帧历史 + 并行双基线状态机实现。
 */
#include "nfocc/nfocc_baseline.h"

#include <cmath>
#include <cstring>

namespace {

const float kDeg2Rad = 3.14159265358979323846f / 180.0f;

/** 从 R_curr^T · R_ref 提取航向漂移角 (deg)，取绝对值 */
float yaw_drift_deg(const float *Rc, const float *Rr) {
    /* R_rel = Rc^T · Rr */
    float R01 = 0.0f, R00 = 0.0f;
    for (int k = 0; k < 3; ++k) {
        R00 += Rc[k * 3 + 0] * Rr[k * 3 + 0];
        R01 += Rc[k * 3 + 0] * Rr[k * 3 + 1];
    }
    return std::fabs(std::atan2(R01, R00)) / kDeg2Rad;
}

/** 车体基准位移：当前 - 参考 */
void delta_t(const NFBaselinePose *cur, const NFBaselinePose *ref,
             float out[3]) {
    out[0] = cur->t[0] - ref->t[0];
    out[1] = cur->t[1] - ref->t[1];
    out[2] = cur->t[2] - ref->t[2];
}

float norm3(const float v[3]) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

extern "C" {

NFStatus nf_baseline_manager_init(NFBaselineManager *mgr,
                                  nf_f32 b_min_mm, nf_f32 b_max_mm,
                                  nf_f32 b_target_mm) {
    if (!mgr) return NF_E_PARAM_NULL;
    std::memset(mgr, 0, sizeof(*mgr));
    mgr->version = NF_BASELINE_VERSION;

    mgr->b_min_mm    = (b_min_mm    > 0.0f) ? b_min_mm    : 200.0f;
    mgr->b_max_mm    = (b_max_mm    > 0.0f) ? b_max_mm    : 460.0f;
    mgr->b_target_mm = (b_target_mm > 0.0f) ? b_target_mm : 300.0f;
    mgr->max_yaw_deg = 15.0f;

    if (!(mgr->b_min_mm < mgr->b_max_mm)) return NF_E_PARAM_RANGE;
    if (mgr->b_target_mm < mgr->b_min_mm ||
        mgr->b_target_mm > mgr->b_max_mm) return NF_E_PARAM_RANGE;

    nf_baseline_manager_reset(mgr);
    return NF_OK;
}

void nf_baseline_manager_reset(NFBaselineManager *mgr) {
    if (!mgr) return;
    for (nf_u32 i = 0; i < NF_BASELINE_HIST_CAP; ++i) mgr->hist[i].valid = 0;
    mgr->head = 0;
    mgr->count = 0;
    mgr->has_short = 0;
    mgr->has_long = 0;
    mgr->idx_short = 0;
    mgr->idx_long = 0;
}

NFStatus nf_baseline_manager_push(NFBaselineManager *mgr,
                                  const NFBaselinePose *pose,
                                  nf_u64 ts_us, nf_u32 frame_id) {
    if (!mgr) return NF_E_PARAM_NULL;
    if (!pose) return NF_E_PARAM_NULL;

    const nf_u32 slot = mgr->head;
    NFHistoryFrame *f = &mgr->hist[slot];
    f->pose = *pose;
    f->ts_us = ts_us;
    f->frame_id = frame_id;
    f->valid = 1;

    mgr->head = (mgr->head + 1u) % NF_BASELINE_HIST_CAP;
    if (mgr->count < NF_BASELINE_HIST_CAP) ++mgr->count;

    /**
     * **短基线指向"推入当前帧之前的那个参考帧"**，不是当前帧自己。
     *
     * 最初我把这里写成 `idx_short = 当前帧`，结果短基线恒为 0：
     * decide 时短基线长度是 |cur - cur| = 0，永远达不到 b_min，
     * 只能靠长基线产出，一旦长基线刚晋升成当前帧，当帧就必然冻结。
     * 实测 150mm/帧 的步进里出现 STEREO/FROZEN 交替。
     *
     * 正确做法：push 时把"上一个 head"作为短基线参考。
     * 这与 NFORM 的语义一致 —— 短基线 = 相邻两帧，
     * 长基线 = 更早的某一帧（累积到 b_target）。
     */
    if (mgr->count >= 2) {
        /* 上一帧 = head 往前两格（head 已指向下一个空槽） */
        const nf_u32 prev = (mgr->head + NF_BASELINE_HIST_CAP - 2u)
                            % NF_BASELINE_HIST_CAP;
        mgr->idx_short = prev;
        mgr->has_short = 1;
    } else {
        /* 第一帧：短基线暂时指向自己，decide 会判 FROZEN_NEAR */
        mgr->idx_short = slot;
        mgr->has_short = 1;
    }

    /* 长基线：首次进入指向第一帧；之后由 decide 在晋升时更新 */
    if (!mgr->has_long) {
        mgr->idx_long = (mgr->count >= 2)
                        ? (mgr->head + NF_BASELINE_HIST_CAP - 2u) % NF_BASELINE_HIST_CAP
                        : slot;
        mgr->has_long = 1;
    }
    return NF_OK;
}

NFStatus nf_baseline_manager_decide(NFBaselineManager *mgr,
                                    NFBaselineDecision *out) {
    if (!mgr || !out) return NF_E_PARAM_NULL;
    std::memset(out, 0, sizeof(*out));
    out->version = NF_BASELINE_DECISION_VERSION;
    ++mgr->n_decide;

    /* 当前帧 = 最近一次 push 的槽 */
    const nf_u32 cur_idx =
        (mgr->head + NF_BASELINE_HIST_CAP - 1u) % NF_BASELINE_HIST_CAP;
    const NFHistoryFrame *cur = &mgr->hist[cur_idx];

    if (!cur->valid || !mgr->has_short) {
        out->mode = NF_MODE_FROZEN_NEAR;
        return NF_OK;
    }

    /* ---- 长基线 ---- */
    float t_long[3] = { 0, 0, 0 };
    bool long_ok = false;
    if (mgr->has_long && mgr->hist[mgr->idx_long].valid &&
        mgr->idx_long != cur_idx) {
        delta_t(&cur->pose, &mgr->hist[mgr->idx_long].pose, t_long);
        long_ok = true;
    }

    /* ---- 短基线 ---- */
    float t_short[3] = { 0, 0, 0 };
    bool short_ok = false;
    if (mgr->idx_short != cur_idx) {
        delta_t(&cur->pose, &mgr->hist[mgr->idx_short].pose, t_short);
        short_ok = true;
    } else if (mgr->count >= 2) {
        /* 短基线就是当前帧本身（push 时已更新）—— 回退到前一帧 */
        const nf_u32 prev = (cur_idx + NF_BASELINE_HIST_CAP - 1u)
                            % NF_BASELINE_HIST_CAP;
        if (mgr->hist[prev].valid) {
            delta_t(&cur->pose, &mgr->hist[prev].pose, t_short);
            short_ok = true;
        }
    }

    /* ---- 1) 航向保护 ---- */
    if (long_ok) {
        const nf_f32 yd = yaw_drift_deg(cur->pose.R,
                                        mgr->hist[mgr->idx_long].pose.R);
        out->yaw_drift_deg = yd;
        if (yd > mgr->max_yaw_deg) {
            /* 急转/原地掉头：极线搜索失效。
             * **必须重置队列**否则恢复后会用到过期参考帧。 */
            out->mode = NF_MODE_FROZEN_ROT;
            ++mgr->n_reset;
            nf_baseline_manager_reset(mgr);
            return NF_OK;
        }
    }

    const float b_long = long_ok ? norm3(t_long) : 0.0f;
    const float b_short = short_ok ? norm3(t_short) : 0.0f;

    /* ---- 2) 超上限 ---- */
    if (long_ok && b_long > mgr->b_max_mm) {
        /* 长基线过大：重叠视场不足，匹配必然陡崩。
         * 不只是报错 —— 先尝试用短基线降级，短基线也不行才冻结。 */
        ++mgr->n_reset;
        if (short_ok && b_short >= mgr->b_min_mm && b_short <= mgr->b_max_mm) {
            out->mode = NF_MODE_STEREO;
            out->baseline_mm = b_short;
            out->ref_frame_id = mgr->hist[mgr->idx_short].frame_id;
            out->t_veh[0] = t_short[0];
            out->t_veh[1] = t_short[1];
            out->t_veh[2] = t_short[2];
            out->used_long = 0;
            /* 队列重置，避免下一次又撞上限 */
            const NFHistoryFrame keep = *cur;
            nf_baseline_manager_reset(mgr);
            nf_baseline_manager_push(mgr, &keep.pose, keep.ts_us,
                                     keep.frame_id);
            out->is_new_ref = 1;
            return NF_OK;
        }
        out->mode = NF_MODE_FROZEN_FAR;
        const NFHistoryFrame keep = *cur;
        nf_baseline_manager_reset(mgr);
        nf_baseline_manager_push(mgr, &keep.pose, keep.ts_us, keep.frame_id);
        out->is_new_ref = 1;
        return NF_OK;
    }

    /* ---- 3) 长基线达标 -> 输出并晋升 ---- */
    if (long_ok && b_long >= mgr->b_target_mm) {
        out->mode = NF_MODE_STEREO;
        out->baseline_mm = b_long;
        out->ref_frame_id = mgr->hist[mgr->idx_long].frame_id;
        out->t_veh[0] = t_long[0];
        out->t_veh[1] = t_long[1];
        out->t_veh[2] = t_long[2];
        out->used_long = 1;

        /* 长基线退役，短基线晋升为新的长基线；当前帧成为新短基线。
         * 这样消除固定基线等待周期，系统始终处在高置信产出状态。 */
        if (mgr->has_short) mgr->idx_long = mgr->idx_short;
        out->is_new_ref = 1;
        return NF_OK;
    }

    /* ---- 4) 短基线可用 ---- */
    if (short_ok && b_short >= mgr->b_min_mm) {
        out->mode = NF_MODE_STEREO;
        out->baseline_mm = b_short;
        out->ref_frame_id = mgr->hist[mgr->idx_short].frame_id;
        out->t_veh[0] = t_short[0];
        out->t_veh[1] = t_short[1];
        out->t_veh[2] = t_short[2];
        out->used_long = 0;
        return NF_OK;
    }

    /* ---- 5) 都不够 ---- */
    out->mode = NF_MODE_FROZEN_NEAR;
    return NF_OK;
}

} /* extern "C" */
