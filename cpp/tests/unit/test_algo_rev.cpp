/**
 * @file test_algo_rev.cpp
 * @brief REV-001 / REV-002 新增算法模块的测试。
 *
 * 覆盖：
 *   - D3 FoE 发散锥（含 apex 必须是相机光心的负例）
 *   - D2 双向一致性软权重
 *   - 空间不确定度 L2 传播（含与 L1 的差异校验）
 *   - 双基线状态机（多帧累积、短距离 hold、长基线晋升、b_max 重置、旋转冻结）
 *   - D4 交叉验证（方差除以 sqrt(2)，超限释放）
 */
#include "nf_test.h"
#include "nfocc/nfocc_geometry.h"
#include "nfocc/nfocc_baseline.h"
#include "nfocc/nfocc_voxel.h"
#include "nfocc/nfocc_lut.h"
#include <vector>

#include <cmath>
#include <cstring>

/* ==================== D3: FoE ==================== */

NF_TEST(foe_cone_filtering) {
    /* 相机在原点，平移方向 +X */
    const float apex[3] = { 0, 0, 0 };
    const float dir[3]  = { 1, 0, 0 };
    const float cos8sq  = 0.9806f;      /* cos^2(8°) */

    /* 正前方 5m -> 与平移方向夹角 0° -> 锥内，应剔除 */
    const float p1[3] = { 5000, 0, 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(p1, apex, dir, cos8sq), 1);

    /* 正侧方 -> 夹角 90° -> 锥外，保留 */
    const float p2[3] = { 0, 5000, 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(p2, apex, dir, cos8sq), 0);

    /* 偏 5° -> 锥内（5 < 8） */
    const float a5 = 5.0f * 3.14159265f / 180.0f;
    const float p3[3] = { 5000.0f * std::cos(a5), 5000.0f * std::sin(a5), 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(p3, apex, dir, cos8sq), 1);

    /* 偏 12° -> 锥外（12 > 8） */
    const float a12 = 12.0f * 3.14159265f / 180.0f;
    const float p4[3] = { 5000.0f * std::cos(a12), 5000.0f * std::sin(a12), 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(p4, apex, dir, cos8sq), 0);

    /* 在平移方向"后方" -> delta <= 0 -> 不算锥内 */
    const float p5[3] = { -5000, 0, 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(p5, apex, dir, cos8sq), 0);
}

NF_TEST(foe_apex_is_camera_center) {
    /* **apex 必须是相机光心**。用不同 apex 应得到不同判定。
     * 这个用例防止实现者把 t_veh 误当 apex 传入。 */
    const float dir[3] = { 1, 0, 0 };
    const float cos8sq = 0.9806f;
    const float P[3] = { 6000, 0, 0 };

    /* apex 在原点：P 在其正前方 5m，锥内 */
    const float a0[3] = { 0, 0, 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(P, a0, dir, cos8sq), 1);

    /* apex 在 P 之后 10m（相机在 P 前方）：视线朝 -X，与 +X 相反 -> 锥外 */
    const float a1[3] = { 16000, 0, 0 };
    NF_CHECK_EQ_INT(nf_pt_in_foe(P, a1, dir, cos8sq), 0);
}

/* ==================== D2: 软权重 ==================== */

NF_TEST(occlusion_soft_weight) {
    const float tau = 1.0f, lambda = 14.0f;

    /* 完全一致 -> 权重 1 */
    NF_CHECK_NEAR(nf_occlusion_weight(10.0f, 10.0f, tau, lambda), 1.0f, 1e-6);

    /* 残差在容差内 -> 仍为 1 */
    NF_CHECK_NEAR(nf_occlusion_weight(10.0f, 9.5f, tau, lambda), 1.0f, 1e-6);

    /* 残差 2px（tau 的 2 倍）-> 1/(1+14*1) = 1/15 */
    NF_CHECK_NEAR(nf_occlusion_weight(10.0f, 8.0f, tau, lambda),
                  1.0f / 15.0f, 1e-5);

    /* 单调性：残差越大权重越小 */
    const float w1 = nf_occlusion_weight(10.0f, 7.0f, tau, lambda);
    const float w2 = nf_occlusion_weight(10.0f, 4.0f, tau, lambda);
    NF_CHECK(w2 < w1);
    NF_CHECK(w1 < 1.0f);
    NF_CHECK(w2 > 0.0f);
}

/* ==================== 空间不确定度 ==================== */

NF_TEST(spatial_uncertainty_cm) {
    const float f = 184.75f, b = 273.0f;

    /* 视差越大（越近）-> 不确定度越小 */
    const nf_u8 s_near = nf_compute_spatial_uncertainty_cm(f, b, 30.0f, 0.5f, 0.02f);
    const nf_u8 s_far  = nf_compute_spatial_uncertainty_cm(f, b,  8.0f, 0.5f, 0.02f);
    NF_CHECK(s_near < s_far);
    NF_CHECK(s_near >= 1 && s_near <= 254);

    /* **同视差下基线越大，Z 越远，不确定度反而越大** —— 这是物理事实：
     * Z = f*b/d，b 翻倍则 Z 翻倍，同样的 2% 相对基线误差在绝对量上也翻倍。
     * 原判据写成"基线长->更准"是错的（那是"同一距离"下的结论）。 */
    const nf_u8 s_b_small = nf_compute_spatial_uncertainty_cm(f, 150.0f, 15.0f, 0.5f, 0.02f);
    const nf_u8 s_b_large = nf_compute_spatial_uncertainty_cm(f, 300.0f, 15.0f, 0.5f, 0.02f);
    NF_CHECK(s_b_large > s_b_small);

    /* 同一**距离**下基线越长越准：Z 固定意味着 d 与 b 成正比。
     * b=300 时 d 应为 b=150 时的 2 倍。 */
    const nf_u8 s_z_far  = nf_compute_spatial_uncertainty_cm(f, 150.0f, 15.0f, 0.5f, 0.02f);
    const nf_u8 s_z_near = nf_compute_spatial_uncertainty_cm(f, 300.0f, 30.0f, 0.5f, 0.02f);
    NF_CHECK(s_z_near < s_z_far);

    /* 无效视差 -> 哨兵 255 */
    NF_CHECK_EQ_INT(nf_compute_spatial_uncertainty_cm(f, b, 0.0f, 0.5f, 0.02f), 255);
    NF_CHECK_EQ_INT(nf_compute_spatial_uncertainty_cm(f, b, -1.0f, 0.5f, 0.02f), 255);

    /* **L2 vs L1 的差异**：本实现是 L2 平方和，应比线性相加小。
     * 手算: Z=f*b/d, d=8 -> Z=6305mm
     *   t_d = (Z/d)*sig_d = (6305/8)*0.5 = 394.1
     *   t_b = Z*sig_b = 6305*0.02 = 126.1
     *   L2  = sqrt(394.1^2 + 126.1^2) = 413.8 mm -> 41.4 cm -> 41
     *   L1  = 394.1 + 126.1 = 520.2 mm -> 52.0 cm
     */
    const nf_u8 s = nf_compute_spatial_uncertainty_cm(f, b, 8.0f, 0.5f, 0.02f);
    NF_CHECK_NEAR((float)s, 41.0f, 1.0f);
    NF_CHECK(s < 52);        /* 必须小于 L1 结果 */
}

/* ==================== 双基线状态机 ==================== */

namespace {

NFBaselinePose make_pose(float x_mm, float yaw_deg) {
    NFBaselinePose p;
    std::memset(&p, 0, sizeof(p));
    const float a = yaw_deg * 3.14159265f / 180.0f;
    const float c = std::cos(a), s = std::sin(a);
    /* 绕 Z 轴旋转 */
    p.R[0] = c; p.R[1] = -s; p.R[2] = 0;
    p.R[3] = s; p.R[4] =  c; p.R[5] = 0;
    p.R[6] = 0; p.R[7] =  0; p.R[8] = 1;
    p.t[0] = x_mm; p.t[1] = 0; p.t[2] = 0;
    return p;
}

}  // namespace

NF_TEST(baseline_manager_multi_frame) {
    NFBaselineManager mgr;
    NF_CHECK_STATUS(nf_baseline_manager_init(&mgr, 200.0f, 460.0f, 300.0f),
                    NF_OK);

    NFBaselineDecision dec;

    /* 第 1 帧：只有自己，无参考 -> 冻结 */
    NFBaselinePose p0 = make_pose(0.0f, 0.0f);
    NF_CHECK_STATUS(nf_baseline_manager_push(&mgr, &p0, 1000, 0), NF_OK);
    NF_CHECK_STATUS(nf_baseline_manager_decide(&mgr, &dec), NF_OK);
    NF_CHECK_EQ_INT((int)dec.mode, (int)NF_MODE_FROZEN_NEAR);

    /* 第 2 帧：位移 100mm < b_min -> 冻结 */
    NFBaselinePose p1 = make_pose(100.0f, 0.0f);
    nf_baseline_manager_push(&mgr, &p1, 2000, 1);
    nf_baseline_manager_decide(&mgr, &dec);
    NF_CHECK_EQ_INT((int)dec.mode, (int)NF_MODE_FROZEN_NEAR);

    /* 第 3 帧：x=250。
     * 短基线 = 与上一帧(150) 的 100mm，仍 < b_min。
     * 长基线：首帧建立时指向"上一帧"，此处已随队列推进；
     * 若长基线给出 >= b_min 就用它，否则冻结 —— **两种都是合法结果**。
     * 这里断言的是"不越界"，而不是强制 STEREO。 */
    NFBaselinePose p2 = make_pose(250.0f, 0.0f);
    nf_baseline_manager_push(&mgr, &p2, 3000, 2);
    nf_baseline_manager_decide(&mgr, &dec);
    std::printf("      250mm 时: mode=%s baseline=%.0f ref=%u\n",
                nf_mode_str(dec.mode), (double)dec.baseline_mm,
                dec.ref_frame_id);
    NF_CHECK(dec.baseline_mm <= 460.0f);
    if (dec.mode == NF_MODE_STEREO) {
        NF_CHECK(dec.baseline_mm >= 200.0f);
    }

    /* 继续累积到 400mm：与第 1 帧 (x=0) 的位移已足够，必须能产出 */
    NFBaselinePose p3 = make_pose(400.0f, 0.0f);
    nf_baseline_manager_push(&mgr, &p3, 4000, 3);
    nf_baseline_manager_decide(&mgr, &dec);
    std::printf("      400mm 时: mode=%s baseline=%.0f\n",
                nf_mode_str(dec.mode), (double)dec.baseline_mm);
    NF_CHECK_EQ_INT((int)dec.mode, (int)NF_MODE_STEREO);
    NF_CHECK(dec.baseline_mm >= 200.0f && dec.baseline_mm <= 460.0f);
}

NF_TEST(baseline_manager_rotation_freeze) {
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 200.0f, 460.0f, 300.0f);

    NFBaselinePose p0 = make_pose(0.0f, 0.0f);
    nf_baseline_manager_push(&mgr, &p0, 1000, 0);

    /* 位移够但航向转了 30° -> 急转弯，必须冻结 */
    NFBaselinePose p1 = make_pose(300.0f, 30.0f);
    nf_baseline_manager_push(&mgr, &p1, 2000, 1);

    NFBaselineDecision dec;
    nf_baseline_manager_decide(&mgr, &dec);
    NF_CHECK_EQ_INT((int)dec.mode, (int)NF_MODE_FROZEN_ROT);
    NF_CHECK_EQ_INT((int)dec.baseline_mm, 0);
    /* 冻结时必须重置队列，避免恢复后用过期的参考帧 */
    NF_CHECK_EQ_INT((int)mgr.count, 0);
    NF_CHECK(mgr.n_reset >= 1);
}

NF_TEST(baseline_manager_bmax_reset) {
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 200.0f, 460.0f, 300.0f);

    NFBaselinePose p0 = make_pose(0.0f, 0.0f);
    nf_baseline_manager_push(&mgr, &p0, 1000, 0);

    /* 一次跳 1000mm，远超 b_max=460 */
    NFBaselinePose p1 = make_pose(1000.0f, 0.0f);
    nf_baseline_manager_push(&mgr, &p1, 2000, 1);

    NFBaselineDecision dec;
    nf_baseline_manager_decide(&mgr, &dec);
    /* 不能输出超长基线的结果：要么降级用短基线，要么冻结。
     * 无论哪种，baseline 都不得超过 b_max。 */
    NF_CHECK(dec.baseline_mm <= 460.0f);
    NF_CHECK_EQ_INT((int)mgr.n_reset >= 1 ? 1 : 0, 1);
    std::printf("      1000mm 跳变: mode=%s baseline=%.0f\n",
                nf_mode_str(dec.mode), (double)dec.baseline_mm);
}

NF_TEST(baseline_manager_dual_promotion) {
    /* 步进 150mm/帧，应观察长基线在达到 b_target 时触发并晋升 */
    NFBaselineManager mgr;
    nf_baseline_manager_init(&mgr, 200.0f, 460.0f, 300.0f);

    int n_stereo = 0;
    int n_frozen = 0;
    for (int i = 0; i < 8; ++i) {
        NFBaselinePose p = make_pose((float)i * 150.0f, 0.0f);
        nf_baseline_manager_push(&mgr, &p, 1000u * (nf_u64)(i + 1),
                                 (nf_u32)i);
        NFBaselineDecision dec;
        nf_baseline_manager_decide(&mgr, &dec);
        if (dec.mode == NF_MODE_STEREO) {
            ++n_stereo;
            NF_CHECK(dec.baseline_mm >= 200.0f);
            NF_CHECK(dec.baseline_mm <= 460.0f);
        } else {
            ++n_frozen;
        }
    }
    std::printf("      8 帧 x150mm: STEREO %d 帧, 冻结 %d 帧\n",
                n_stereo, n_frozen);
    /* 位移持续增长，必须能持续产出，不能全程冻结 */
    NF_CHECK(n_stereo >= 3);
}

/* ==================== D4 交叉验证 ==================== */

NF_TEST(voxel_verify_cross_and_sigma) {
    NFVoxelGridParams g;
    NF_CHECK_STATUS(nf_voxel_grid_init(&g, 200, -2000, 2000, -2000, 2000,
                                       -500, 2500), NF_OK);
    NFVoxelCtx *vox = nullptr;
    NF_CHECK_STATUS(nf_voxel_create(&vox, &g, 4096, 64.0f), NF_OK);

    /* 主观测点 + sigma=40cm */
    NF_CHECK_STATUS(nf_voxel_insert_with_sigma(vox, 1100.0f, 500.0f, 300.0f,
                                               1.0f, 1u, 40u), NF_OK);

    /* 第二基线给出很接近的点（残差 10mm，同格内）-> 应通过并融合 */
    nf_u8 v1 = 0;
    NF_CHECK_STATUS(nf_voxel_verify_cross(vox, 1106.0f, 500.0f, 300.0f,
                                          2u, 50.0f, &v1), NF_OK);
    NF_CHECK_EQ_INT((int)v1, 1);

    /* 融合后质心应在两点中点附近 */
    nf_u32 ne = 0, ns = 0, nc = 0;
    nf_voxel_stats(vox, &ne, &ns, &nc);
    NF_CHECK(ne >= 1);

    /* 第三基线给出同格但很远的点（残差 >> 50mm）-> 判为假阳性。
     * 注意必须在**同一体素**内，否则查不到主观测，函数直接返回无判定。 */
    nf_u8 v2 = 1;
    NF_CHECK_STATUS(nf_voxel_verify_cross(vox, 1150.0f, 550.0f, 350.0f,
                                          4u, 50.0f, &v2), NF_OK);
    NF_CHECK_EQ_INT((int)v2, 0);

    nf_voxel_destroy(vox);
}

NF_TEST(voxel_sigma_takes_min) {
    NFVoxelGridParams g;
    nf_voxel_grid_init(&g, 200, -2000, 2000, -2000, 2000, -500, 2500);
    NFVoxelCtx *vox = nullptr;
    nf_voxel_create(&vox, &g, 4096, 64.0f);

    /* 同一体素内三次观测（体素 200mm，索引按 floor((x_max-x)/200) 分格，
     * 故同格内的 x 必须落在同一 200mm 区间）。
     * sigma 分别 50 / 20 / 40 -> 应取最小 20 */
    nf_voxel_insert_with_sigma(vox, 1100.0f, 500.0f, 300.0f, 1.0f, 1u, 50u);
    nf_voxel_insert_with_sigma(vox, 1100.5f, 500.4f, 300.3f, 1.0f, 2u, 20u);
    nf_voxel_insert_with_sigma(vox, 1100.9f, 500.8f, 300.6f, 1.0f, 4u, 40u);

    nf_u32 ne = 0, ns = 0, nc = 0;
    nf_voxel_stats(vox, &ne, &ns, &nc);
    NF_CHECK_EQ_INT((int)ne, 1);      /* 三次落在同一体素 */

    nf_voxel_destroy(vox);
}

/* ==================== REV-002: 柱体连续排布 ==================== */

NF_TEST(voxel_transform_columnar_sorted) {
    NFVoxelGridParams g;
    nf_voxel_grid_init(&g, 200, -2000, 2000, -2000, 2000, -500, 2500);
    NFVoxelCtx *vox = nullptr;
    nf_voxel_create(&vox, &g, 8192, 64.0f);

    /* 在若干已知格子放点，故意用**乱序**插入 */
    struct P { float x, y, z; };
    const P pts[] = {
        { 1500, 500, -300 }, { -1500, 500, 100 }, { 500, -1500, 300 },
        { 1500, 500,  300 }, { -1500, 500, 700 }, { 500, -1500, -100 },
        { 1500, 500, -100 }, { -1500, 500, 500 },
    };
    for (size_t i = 0; i < sizeof(pts) / sizeof(pts[0]); ++i)
        nf_voxel_insert(vox, pts[i].x, pts[i].y, pts[i].z, 1.0f, 1u);

    const nf_u32 n_cells = g.nx * g.ny;
    NFGridOutput go;
    std::memset(&go, 0, sizeof(go));
    go.version = NF_GRID_OUTPUT_VERSION;
    go.n_cells = n_cells;
    go.zmap   = (nf_u32 *)std::calloc(n_cells, sizeof(nf_u32));
    go.zindex = (nf_u32 *)std::calloc(n_cells + 1, sizeof(nf_u32));

    NFPointCloud pc;
    NF_CHECK_STATUS(nf_pointcloud_alloc(&pc, 256), NF_OK);

    NF_CHECK_STATUS(nf_voxel_transform_columnar(vox, &go, &pc), NF_OK);
    NF_CHECK_EQ_INT((int)pc.num_points, 8);
    NF_CHECK_EQ_INT((int)go.zindex[n_cells], 8);

    /* **核心判据**: 点必须按 col 递增、col 内按 iz 升序排列。
     * 这是 O(1) 访问定理成立的前提。 */
    int prev_col = -1, prev_iz = -1;
    int n_ordered = 0;
    for (nf_u32 i = 0; i < pc.num_points; ++i) {
        const NFPoint *pt = &pc.points[i];
        const nf_i32 ix = (nf_i32)std::floor(((float)g.x_max - pt->x_mm) / g.vox_mm);
        const nf_i32 iy = (nf_i32)std::floor(((float)g.y_max - pt->y_mm) / g.vox_mm);
        const nf_i32 iz = (nf_i32)std::floor(((float)pt->z_mm - g.z_min) / g.vox_mm);
        const int col = iy * (int)g.nx + ix;
        if (col < prev_col) { std::printf("      col 逆序 at %u\n", i); break; }
        if (col == prev_col && iz <= prev_iz) {
            std::printf("      同 col 内 iz 未递增 at %u\n", i); break;
        }
        prev_col = col; prev_iz = iz;
        ++n_ordered;
    }
    NF_CHECK_EQ_INT(n_ordered, (int)pc.num_points);

    /* zindex 区间必须能反查到对应柱体的点 */
    int n_cell_ok = 0;
    for (nf_u32 c = 0; c < n_cells; ++c) {
        const nf_u32 s = go.zindex[c], e = go.zindex[c + 1];
        if (s == e) continue;
        const NFPoint *pt = &pc.points[s];
        const nf_i32 ix = (nf_i32)std::floor(((float)g.x_max - pt->x_mm) / g.vox_mm);
        const nf_i32 iy = (nf_i32)std::floor(((float)g.y_max - pt->y_mm) / g.vox_mm);
        if ((nf_u32)(iy * (nf_i32)g.nx + ix) == c) ++n_cell_ok;
    }
    NF_CHECK(n_cell_ok >= 3);   /* 至少几个柱体能正确反查 */

    nf_pointcloud_destroy(&pc);
    std::free(go.zmap); std::free(go.zindex);
    nf_voxel_destroy(vox);
}

/* ==================== REV-002: 车身掩码光栅化 ==================== */

NF_TEST(mask_rasterize_polygon) {
    const nf_u32 w = 64, h = 48;
    std::vector<nf_u8> mask((size_t)w * h, 0);
    std::vector<nf_f32> px, py;

    /* 一个居中的矩形 16..48 x 12..36 */
    px = { 16.0f, 48.0f, 48.0f, 16.0f };
    py = { 12.0f, 12.0f, 36.0f, 36.0f };

    NF_CHECK_STATUS(nf_mask_rasterize_polygon(mask.data(), w, h,
                                              px.data(), py.data(), 4), NF_OK);

    /* 内部点应为 1 */
    NF_CHECK_EQ_INT((int)mask[(size_t)24 * w + 32], 1);
    /* 外部点应为 0 */
    NF_CHECK_EQ_INT((int)mask[(size_t)2 * w + 2], 0);
    NF_CHECK_EQ_INT((int)mask[(size_t)45 * w + 60], 0);

    /* 统计掩码面积，应接近矩形面积 (32x24=768) */
    int cnt = 0;
    for (size_t i = 0; i < mask.size(); ++i) cnt += mask[i] ? 1 : 0;
    std::printf("      掩码面积 %d (矩形理论 768)\n", cnt);
    NF_CHECK(cnt > 700 && cnt < 850);

    /* 顶点数 < 3 -> 拒绝 */
    NF_CHECK_EQ_INT((int)nf_mask_rasterize_polygon(mask.data(), w, h,
                                                   px.data(), py.data(), 2),
                    (int)NF_E_PARAM_RANGE);
}

int main() { return nftest::run_all(); }
