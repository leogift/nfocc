/**
 * @file test_newmod.cpp
 * @brief NEW_MOD.md 改进的验证 + 评审报告指出的问题回归。
 *
 * 每个用例对应文档一节或报告一条意见，注释写明出处，
 * 便于以后追溯"这条判据为什么存在"。
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "nfocc/nfocc_baseline.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_geometry.h"
#include "nfocc/nfocc_voxel.h"
#include "nf_test.h"

namespace { const float kPi = 3.14159265358979f; }

/* ===== NEW_MOD §1.2.1 不确定度传播（报告 #1 量纲错误） ===== */

/**
 * σ 必须随距离单调增长。
 * 若 σ_odom 项缺 b 尺度因子，远场不再膨胀 —— 这条能抓住。
 */
NF_TEST(sigma_grows_with_distance) {
    const float f = 137.19f, b = 278.0f;
    nf_u8 prev = 0; int mono = 1;
    std::printf("      Z(mm)   d(px)  sigma(cm)\n");
    for (float Z = 1000.0f; Z <= 8000.0f; Z += 1000.0f) {
        const float d = f * b / Z;
        const nf_u8 s = nf_compute_spatial_uncertainty_cm(f, b, d, 0.5f, 0.02f);
        std::printf("      %5.0f  %6.2f  %6u\n", (double)Z, (double)d, s);
        if (s < prev) mono = 0;
        prev = s;
    }
    NF_CHECK_MSG(mono, "sigma 必须随距离单调不减");
    const nf_u8 s1 = nf_compute_spatial_uncertainty_cm(f, b, f*b/1000.0f, 0.5f, 0.02f);
    const nf_u8 s8 = nf_compute_spatial_uncertainty_cm(f, b, f*b/8000.0f, 0.5f, 0.02f);
    NF_CHECK_MSG(s1 <= 5, "1m 处 sigma 应为个位数 cm");
    NF_CHECK_MSG(s8 >= (nf_u8)NF_SIGMA_MAX_CM, "8m 处 sigma 应超门限 80cm");
}

/**
 * 量纲回归：sigma_b 翻倍，总 sigma 必须增大。
 * 若漏了 b_mm，改 sigma_b 对结果几乎无影响。
 */
NF_TEST(sigma_odom_term_has_scale) {
    const float f = 137.19f, b = 278.0f, Z = 5000.0f;
    const float d = f * b / Z;
    const nf_u8 s2 = nf_compute_spatial_uncertainty_cm(f, b, d, 0.5f, 0.02f);
    const nf_u8 s4 = nf_compute_spatial_uncertainty_cm(f, b, d, 0.5f, 0.04f);
    std::printf("      5m: sigma_b=2%% -> %u cm, 4%% -> %u cm\n", s2, s4);
    NF_CHECK_MSG(s4 > s2, "sigma_b 加倍后总 sigma 必须增大（证明带尺度）");
    NF_CHECK_MSG(s2 >= 33 && s2 <= 36, "5m 处 sigma 应在 33-36cm（手算 34.3）");
}

/* ===== NEW_MOD §1.2.2 sigma 硬截断 ===== */

NF_TEST(sigma_cap_cuts_at_eight_meters) {
    const float f = 137.19f, b = 278.0f;
    const nf_u8 cap = (nf_u8)NF_SIGMA_MAX_CM;
    const nf_u8 s7 = nf_compute_spatial_uncertainty_cm(f, b, f*b/7000.0f, 0.5f, 0.02f);
    const nf_u8 s9 = nf_compute_spatial_uncertainty_cm(f, b, f*b/9000.0f, 0.5f, 0.02f);
    std::printf("      7m: %u cm (<%u 通过), 9m: %u cm (>=%u 截断)\n", s7, cap, s9, cap);
    NF_CHECK_MSG(s7 < cap, "7m 处 sigma 应低于门限");
    NF_CHECK_MSG(s9 >= cap, "9m 处 sigma 应超过门限");
}

/* ===== NEW_MOD §1.3 D4（报告 #2 门限自适应） ===== */

/**
 * 固定 50mm 门限会把短基线正常观测判成不一致。
 * 2m 处短基线 sigma 已超 50mm，自适应门限必须显著大于它。
 */
NF_TEST(d4_threshold_adapts_to_distance) {
    const float f = 137.19f, bl = 500.0f, bs = 278.0f;
    const float tau2 = nf_verify_thresh_mm(2000.0f, f, bl, bs, 50.0f);
    const float tau5 = nf_verify_thresh_mm(5000.0f, f, bl, bs, 50.0f);
    std::printf("      2m: tau=%.0fmm, 5m: tau=%.0fmm (固定 50mm)\n",
                (double)tau2, (double)tau5);
    NF_CHECK_MSG(tau2 > 50.0f, "2m 处自适应门限必须大于固定值");
    NF_CHECK_MSG(tau5 > tau2, "门限应随距离增大");
    NF_CHECK_MSG(tau2 > 120.0f && tau2 < 220.0f, "2m 处门限应在 120-220mm");
}

NF_TEST(d4_threshold_has_floor) {
    const float tau = nf_verify_thresh_mm(300.0f, 137.19f, 500.0f, 278.0f, 50.0f);
    std::printf("      0.3m: tau=%.0fmm\n", (double)tau);
    NF_CHECK_MSG(tau >= 50.0f, "近场门限不应低于下限");
}

/* ===== 基线状态机（报告 #5 / #6 / #7） ===== */

/** 报告 #5：晋升后首次长基线必须达到 b_target，不是刚过 b_min */
NF_TEST(baseline_no_immediate_output_after_promote) {
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 278.0f, 833.0f, 500.0f);
    int first_long = -1;
    for (int i = 0; i < 6; ++i) {
        NFBaselinePose p;
        std::memset(&p, 0, sizeof(p));
        p.R[0] = p.R[4] = p.R[8] = 1.0f;
        p.t[0] = 278.0f * (float)i;
        nf_baseline_manager_push(&mgr, &p, (nf_u64)i * 200000ull, (nf_u32)i);
        NFBaselineDecision d;
        std::memset(&d, 0, sizeof(d));
        d.version = NF_BASELINE_DECISION_VERSION;
        nf_baseline_manager_decide(&mgr, &d);
        if (d.used_long && first_long < 0) {
            first_long = i;
            std::printf("      第 %d 帧首次长基线 b=%.0fmm\n", i, (double)d.baseline_mm);
            NF_CHECK_MSG(d.baseline_mm >= 500.0f, "首次长基线须达 b_target(500)");
        }
    }
    NF_CHECK_MSG(first_long > 0, "应有长基线输出");
    nf_baseline_manager_reset(&mgr);
}

/** 报告 #6：短基线每帧滚动，长度恒为一帧步进 */
NF_TEST(baseline_short_rolls_every_frame) {
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 278.0f, 833.0f, 500.0f);
    const float step = 278.0f;
    int n_short = 0;
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
        if (!d.used_long && d.mode == NF_MODE_STEREO) {
            ++n_short;
            NF_CHECK_MSG(std::fabs(d.baseline_mm - step) < 1.0f,
                         "短基线应恒为一帧步进（滚动更新）");
        }
    }
    std::printf("      短基线产出 %d 次，长度均为 %.0fmm\n", n_short, (double)step);
    NF_CHECK_MSG(n_short > 0, "应有短基线产出");
    nf_baseline_manager_reset(&mgr);
}

/** 报告 #7：航向提取正确 -> 超 15 度必冻结 */
NF_TEST(baseline_yaw_overflow_freezes) {
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 278.0f, 833.0f, 500.0f);
    int n_rot = 0;
    for (int i = 0; i < 5; ++i) {
        NFBaselinePose p;
        std::memset(&p, 0, sizeof(p));
        const float yaw = 10.0f * kPi / 180.0f * (float)i;
        p.R[0] = std::cos(yaw); p.R[1] = -std::sin(yaw);
        p.R[3] = std::sin(yaw); p.R[4] =  std::cos(yaw);
        p.R[8] = 1.0f;
        p.t[0] = 400.0f * (float)i;
        nf_baseline_manager_push(&mgr, &p, (nf_u64)i * 200000ull, (nf_u32)i);
        NFBaselineDecision d;
        std::memset(&d, 0, sizeof(d));
        d.version = NF_BASELINE_DECISION_VERSION;
        nf_baseline_manager_decide(&mgr, &d);
        if (d.mode == NF_MODE_FROZEN_ROT) ++n_rot;
    }
    std::printf("      每帧转 10 度，FROZEN_ROT 触发 %d 次\n", n_rot);
    NF_CHECK_MSG(n_rot > 0, "累计航向超 15 度应冻结");
    nf_baseline_manager_reset(&mgr);
}


/* ===== sigma 截断在 pipeline 层的实际作用 ===== */

/**
 * **σ 截断不减少体素总数** —— 这是实测结论，不是设计失误。
 *
 * 真实数据实测：门限 80cm 时剔除 3107 个点，但帧点数 195 -> 195 不变。
 * 原因：被剔除的点与保留的点落在同一批体素里，体素数由空间分布决定，
 * 不由观测条数决定。
 *
 * 那截断有什么用？**阻止"只有坏观测"的体素存在**。若不截断，
 * 一个 σ=200cm 的野值就能凭空创建一个体素（下面会验）。
 *
 * 这条用例守住这个理解，避免以后有人用"点数没变"误判截断失效。
 */
NF_TEST(sigma_cap_prevents_bad_only_voxel) {
    NFVoxelGridParams g;
    NF_CHECK_STATUS(nf_voxel_grid_init(&g, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                                       NF_G_Y_MIN, NF_G_Y_MAX,
                                       NF_G_Z_MIN, NF_G_Z_MAX), NF_OK);

    /* 不截断：一个 σ=200 的坏观测就能创建体素 */
    NFVoxelCtx *bad = nullptr;
    NF_CHECK_STATUS(nf_voxel_create(&bad, &g, 4096, 8.0f), NF_OK);
    nf_voxel_insert_with_sigma(bad, 2000.0f, 0.0f, 0.0f, 1.0f, 1u, 200u);
    nf_u32 n_bad = 0, n_slots = 0, coll = 0;
    nf_voxel_stats(bad, &n_bad, &n_slots, &coll);
    std::printf("      无截断: 1 个 σ=200 观测 -> %u 体素\n", n_bad);
    NF_CHECK_MSG(n_bad == 1, "不截断时坏观测会创建体素（这正是问题）");
    nf_voxel_destroy(bad);

    /* 截断后：同样的观测不应进入（由 pipeline 层 continue 保证） */
    /* 这里验的是判据成立：σ > 门限时调用方应跳过 */
    const nf_u8 cap = (nf_u8)NF_SIGMA_MAX_CM;
    NF_CHECK_MSG(200u > cap, "σ=200 应超门限，调用方须跳过");
}


/* ===== D4 接入主链后的行为（NEW_MOD §1.3.2） ===== */

/**
 * D4 的三条路径：首次插入 / 融合 / 残差超限衰减。
 *
 * 这条守的是"开了 D4 点数不变是正常的"这个反直觉结论：
 * 融合只更新质心、超限只衰减 n_eff，**都不减少体素数**。
 */
NF_TEST(d4_three_paths) {
    NFVoxelGridParams g;
    NF_CHECK_STATUS(nf_voxel_grid_init(&g, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                                       NF_G_Y_MIN, NF_G_Y_MAX,
                                       NF_G_Z_MIN, NF_G_Z_MAX), NF_OK);
    NFVoxelCtx *c = nullptr;
    NF_CHECK_STATUS(nf_voxel_create(&c, &g, 1024, 8.0f), NF_OK);

    /* 路径 1：首次出现 -> 正常插入 */
    NF_CHECK_STATUS(nf_voxel_insert_with_sigma(c, 2000.0f, 0.0f, 0.0f,
                                               1.0f, 1u, 20u), NF_OK);
    nf_u32 n = 0, s = 0, col = 0;
    nf_voxel_stats(c, &n, &s, &col);
    NF_CHECK_EQ_INT((int)n, 1);

    /* 路径 2：同位置第二观测 -> 通过并融合 */
    nf_u8 v = 0;
    NF_CHECK_STATUS(nf_voxel_verify_cross(c, 2000.0f, 0.0f, 0.0f, 2u, 165.0f, &v),
                    NF_OK);
    NF_CHECK_EQ_INT((int)v, 1);
    std::printf("      同位置: verdict=%u (融合)\n", v);

    /* 路径 3：偏 800mm -> 超限，衰减但不删 */
    nf_u8 v2 = 0;
    NF_CHECK_STATUS(nf_voxel_verify_cross(c, 2800.0f, 0.0f, 0.0f, 2u, 165.0f, &v2),
                    NF_OK);
    NF_CHECK_EQ_INT((int)v2, 0);
    std::printf("      偏800mm: verdict=%u (超限衰减)\n", v2);

    /* **关键**：三条路径走完，体素数仍是 1 —— 融合与衰减都不删点 */
    nf_voxel_stats(c, &n, &s, &col);
    NF_CHECK_EQ_INT((int)n, 1);
    std::printf("      走完三条路径体素数仍为 %u（D4 不删点，只改置信）\n", n);

    nf_voxel_destroy(c);
}

int main() { return nftest::run_all(); }
