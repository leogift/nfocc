/**
 * @file test_d4_baseline.cpp
 * @brief D4 双基线 + yaw_rate 积分降级的端到端测试。
 *
 * ## 测什么
 * 1. 里程计积分是否给出正确的绝对位姿（含转弯，验证中点航向法）
 * 2. 双基线状态机在"直行累积"下能否从短基线晋升到长基线
 * 3. 急转向时是否按时冻结（航向漂移超限）
 * 4. **负对照**：不给 yaw_rate 而实际在转，位姿必然错 —— 证明第 1 条
 *    不是恒等成立
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "nfocc/nfocc_baseline.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_odom.h"
#include "nf_test.h"

namespace {
const float kPi = 3.14159265358979f;
}

/* ---------- 里程计积分 ---------- */

/* 匀速直行 1 s：位移应等于速度×时间，航向不变 */
NF_TEST(odom_straight) {
    NFOdomCtx *o = nullptr;
    NF_CHECK_STATUS(nf_odom_create(&o, nullptr), NF_OK);

    const float v = 1389.0f;        /* 5 km/h */
    const float dt = 0.2f;          /* 5 Hz */
    NFOdomInput in;
    std::memset(&in, 0, sizeof(in));
    in.version = NF_ODOM_VERSION;
    in.speed_mmps = v;
    in.dt_s = dt;
    in.yaw_rate_dps = 0.0f;

    for (int i = 0; i < 5; ++i) NF_CHECK_STATUS(nf_odom_push(o, &in), NF_OK);

    NFBaselinePose p;
    nf_odom_current(o, &p);
    const float expect = v * dt * 5.0f;
    std::printf("      直行 1s: x=%.1f (期望 %.1f), yaw=%.4f\n",
                (double)p.t[0], (double)expect, (double)std::atan2(p.R[3], p.R[0]));
    NF_CHECK_MSG(std::fabs(p.t[0] - expect) < 1.0f, "位移应等于 v*t");
    NF_CHECK_MSG(std::fabs(p.t[1]) < 1.0f, "直行不应有横向位移");
    NF_CHECK_MSG(std::fabs(std::atan2(p.R[3], p.R[0])) < 1e-4f, "航向应保持 0");

    nf_odom_destroy(o);
}

/**
 * 圆弧运动：每秒转 30°，速度 1000 mm/s，共 3 秒（90°）。
 *
 * 90° 后位移应是 2R·sin(45°) = R·√2，R = v/ω。
 * 中点航向法的误差应小于 1%（一帧内转角 6°）。
 */
NF_TEST(odom_arc_midpoint_accuracy) {
    NFOdomCtx *o = nullptr;
    nf_odom_create(&o, nullptr);

    const float v = 1000.0f;
    const float w_dps = 30.0f;
    const float dt = 0.2f;
    const int n = 15;               /* 3 s = 90° */
    NFOdomInput in;
    std::memset(&in, 0, sizeof(in));
    in.version = NF_ODOM_VERSION;
    in.speed_mmps = v;
    in.dt_s = dt;
    in.yaw_rate_dps = w_dps;

    for (int i = 0; i < n; ++i) nf_odom_push(o, &in);

    NFBaselinePose p;
    nf_odom_current(o, &p);

    /* 理论：R = v/ω，转 θ=90° 后 |位移| = 2R sin(θ/2) = R√2 */
    const float w = w_dps * kPi / 180.0f;
    const float R = v / w;
    const float expect_disp = R * std::sqrt(2.0f);
    const float got = std::sqrt(p.t[0]*p.t[0] + p.t[1]*p.t[1]);
    const float yaw = std::atan2(p.R[3], p.R[0]);
    const float yaw_err_pct = std::fabs(yaw - kPi*0.5f) / (kPi*0.5f) * 100.0f;

    std::printf("      圆弧 90°: |位移|=%.1f (理论 %.1f, 误差 %.2f%%), yaw=%.2f° (误差 %.3f%%)\n",
                (double)got, (double)expect_disp,
                (double)(std::fabs(got - expect_disp) / expect_disp * 100.0f),
                (double)(yaw * 180.0f / kPi), (double)yaw_err_pct);

    NF_CHECK_MSG(std::fabs(got - expect_disp) / expect_disp < 0.01f,
                 "圆弧位移误差应 < 1%（中点航向法）");
    NF_CHECK_MSG(yaw_err_pct < 0.01f, "航向应恰好 90°");

    nf_odom_destroy(o);
}

/**
 * **负对照**：同一个圆弧，但不给 yaw_rate。
 *
 * 若上一条总是成立（比如实现恒返回理论值），这条也会"通过"。
 * 实际不给偏航率时，位移会退化成直线 —— 必须显著偏离圆弧。
 */
NF_TEST(odom_arc_without_yaw_is_wrong) {
    NFOdomCtx *o = nullptr;
    nf_odom_create(&o, nullptr);

    const float v = 1000.0f, dt = 0.2f;
    NFOdomInput in;
    std::memset(&in, 0, sizeof(in));
    in.version = NF_ODOM_VERSION;
    in.speed_mmps = v;
    in.dt_s = dt;
    in.yaw_rate_dps = 0.0f;          /* 故意不给 */

    for (int i = 0; i < 15; ++i) nf_odom_push(o, &in);

    NFBaselinePose p;
    nf_odom_current(o, &p);
    const float got = std::sqrt(p.t[0]*p.t[0] + p.t[1]*p.t[1]);
    const float w = 30.0f * kPi / 180.0f;
    const float expect_arc = (v / w) * std::sqrt(2.0f);
    const float expect_line = v * dt * 15.0f;

    std::printf("      [对照] 无 yaw_rate: |位移|=%.1f (圆弧理论 %.1f, 直线 %.1f)\n",
                (double)got, (double)expect_arc, (double)expect_line);
    NF_CHECK_MSG(std::fabs(got - expect_line) < 1.0f,
                 "无偏航率时应走直线（证明上一条不是恒成立）");
    NF_CHECK_MSG(std::fabs(got - expect_arc) > 100.0f,
                 "直线位移应与圆弧位移明显不同");

    nf_odom_destroy(o);
}

/* 外部注入位姿后，push 不再改变它 */
NF_TEST(odom_external_pose_wins) {
    NFOdomCtx *o = nullptr;
    nf_odom_create(&o, nullptr);

    NFBaselinePose ext;
    std::memset(&ext, 0, sizeof(ext));
    ext.R[0] = ext.R[4] = ext.R[8] = 1.0f;
    ext.t[0] = 12345.0f;
    nf_odom_push_pose(o, &ext);

    NFOdomInput in;
    std::memset(&in, 0, sizeof(in));
    in.version = NF_ODOM_VERSION;
    in.speed_mmps = 99999.0f;
    in.dt_s = 1.0f;
    nf_odom_push(o, &in);

    NFBaselinePose p;
    nf_odom_current(o, &p);
    NF_CHECK_MSG(std::fabs(p.t[0] - 12345.0f) < 1e-3f,
                 "外部位姿注入后不应被积分覆盖");

    nf_odom_destroy(o);
}

/* ---------- 双基线状态机 ---------- */

/**
 * 匀速直行累积：b_min(278) -> b_target(500)。
 *
 * ## 判据为什么不是"最终触发 b_max"
 * 长基线达标输出后会**立即退役**（`idx_long = idx_short`），短基线
 * 晋升为新的长基线。所以 b_long 稳定在 1.5~2 倍步长附近，永远到不了
 * b_max —— 这是**期望行为**，消除固定等待周期。
 *
 * b_max 只在"中间丢了帧、间隔突然拉大"时才起作用（见下一条测试）。
 * 我第一版把"应触发 b_max"当判据，是错的：那等于要求系统在正常运行
 * 中撞上限重置，与设计意图相反。
 */
NF_TEST(baseline_promotes_to_long) {
    NFBaselineManager mgr;
    NF_CHECK_STATUS(nf_baseline_manager_init(&mgr, 278.0f, 833.0f, 500.0f), NF_OK);

    const float step = 278.0f;      /* 每帧前进 278mm（5km/h@5Hz） */
    int n_short = 0, n_long = 0, n_reset = 0, n_frozen = 0;

    for (int i = 0; i < 10; ++i) {
        NFBaselinePose p;
        std::memset(&p, 0, sizeof(p));
        p.R[0] = p.R[4] = p.R[8] = 1.0f;
        p.t[0] = step * (float)i;

        nf_baseline_manager_push(&mgr, &p, (nf_u64)i * 200000ull, (nf_u32)i);

        NFBaselineDecision d;
        std::memset(&d, 0, sizeof(d));
        d.version = NF_BASELINE_DECISION_VERSION;
        nf_baseline_manager_decide(&mgr, &d);

        if (d.mode == NF_MODE_STEREO) {
            if (d.used_long) { ++n_long; }
            else { ++n_short; }
            std::printf("      frame %d: STEREO %s b=%.0fmm\n",
                        i, d.used_long ? "长" : "短", (double)d.baseline_mm);
        } else if (d.mode == NF_MODE_FROZEN_FAR) {
            ++n_reset;
            std::printf("      frame %d: FROZEN_FAR\n", i);
        } else {
            ++n_frozen;
        }
    }

    std::printf("      统计: 短基线 %d 次, 长基线 %d 次, FAR 重置 %d 次, 冻结 %d 次\n",
                n_short, n_long, n_reset, n_frozen);
    NF_CHECK_MSG(n_short > 0, "应有短基线产出");
    NF_CHECK_MSG(n_long > 0, "累积到 b_target 后应晋升长基线");
    /* 正常直行不应撞 b_max —— 撞了说明退役/晋升机制失效 */
    NF_CHECK_MSG(n_reset == 0, "正常直行不应触发 FAR 重置（退役机制应避免撞上限）");

    nf_baseline_manager_reset(&mgr);
}

/* 急转向：航向漂移超限必须冻结并重置队列 */
NF_TEST(baseline_freezes_on_large_yaw) {
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 278.0f, 833.0f, 500.0f);

    /**
     * 每帧转 8°，max_yaw 默认 15°。第 3 帧累计 16° 就应触发 FROZEN_ROT。
     * 同时给足位移，确保不是因为距离不足而冻结。
     */
    int n_rot = 0;
    for (int i = 0; i < 6; ++i) {
        NFBaselinePose p;
        std::memset(&p, 0, sizeof(p));
        const float yaw = 8.0f * kPi / 180.0f * (float)i;
        p.R[0] = std::cos(yaw); p.R[1] = -std::sin(yaw);
        p.R[3] = std::sin(yaw); p.R[4] = std::cos(yaw);
        p.R[8] = 1.0f;
        p.t[0] = 400.0f * (float)i;

        nf_baseline_manager_push(&mgr, &p, (nf_u64)i * 200000ull, (nf_u32)i);
        NFBaselineDecision d;
        std::memset(&d, 0, sizeof(d));
        d.version = NF_BASELINE_DECISION_VERSION;
        nf_baseline_manager_decide(&mgr, &d);
        if (d.mode == NF_MODE_FROZEN_ROT) ++n_rot;
    }
    std::printf("      急转向: FROZEN_ROT 触发 %d 次 (重置 %u 次)\n",
                n_rot, mgr.n_reset);
    NF_CHECK_MSG(n_rot > 0, "航向漂移超 15° 应触发 FROZEN_ROT");

    nf_baseline_manager_reset(&mgr);
}

int main() { return nftest::run_all(); }
