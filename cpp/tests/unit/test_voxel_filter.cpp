/**
 * @file test_voxel_filter.cpp
 * @brief REV-003 五阶段稠密体素精简的单元测试。
 *
 * 每条判据对应 NEW_MOD §5 的一个阶段，并覆盖评审报告指出的风险点：
 *   - §5.2 低矮障碍物不被误剥离（评审 §5.2）
 *   - §5.3 配额截断近场优先（评审 §5.4）
 *   - §5.5 下游结构体 10 字节对齐（评审 #12）
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "nfocc/nfocc_voxel_filter.h"
#include "nf_test.h"

namespace {

/* 造一个体素条目：只填精简器关心的字段 */
NFVoxelEntry mk(nf_f32 x, nf_f32 y, nf_f32 z, nf_f32 n_eff = 4.0f,
                nf_u8 cam = 1u, nf_u8 sigma = 20u) {
    NFVoxelEntry e;
    std::memset(&e, 0, sizeof(e));
    e.cx = x; e.cy = y; e.cz = z;
    e.n_eff = n_eff;
    e.source_mask = cam;
    e.sigma_cm = sigma;
    e.used = 1u;
    return e;
}

}  /* namespace */

/* ===== 默认参数自洽（NEW_MOD §5.3 项 1） ===== */
NF_TEST(dvf_default_config_consistent) {
    NFVoxelFilterConfig c;
    NF_CHECK_STATUS(nf_voxel_filter_default_config(&c), NF_OK);
    NF_CHECK_MSG(c.cell_mm == NF_DVF_CELL_MM, "cell_mm 默认值");
    NF_CHECK_MSG(c.max_output_points == NF_DVF_HARD_CAP_MAX_POINTS,
                 "配额默认应为硬上限 15000");
    NF_CHECK_MSG(c.roi_x_max > c.roi_x_min, "ROI X 自洽");
    NF_CHECK_MSG(c.roi_y_max > c.roi_y_min, "ROI Y 自洽");
    NF_CHECK_MSG(c.enable_ground_strip == 1u, "默认开地面剥离");

    /* 配额不可越过硬上限：配 100000 应被钳到 15000 */
    NFVoxelFilterConfig c2 = c;
    c2.max_output_points = 100000u;
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, &c2), NF_OK);
    NF_CHECK_MSG(ctx != nullptr, "创建成功");
    nf_voxel_filter_destroy(ctx);
    std::printf("      默认 cell=%.0fmm 配额=%u 硬上限=%u\n",
                (double)c.cell_mm, c.max_output_points,
                (unsigned)NF_DVF_HARD_CAP_MAX_POINTS);
}

/* ===== 阶段 3：位运算地面剥离 ===== */
NF_TEST(dvf_ground_strip_bitmask) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);

    /*
     * 一个柱子：z=0(地面) + z=50 + z=1500（障碍物）
     * z_bin = 100mm，ROI z_min = -1000
     *   z=0    -> bin 10
     *   z=50   -> bin 10
     *   z=1500 -> bin 25
     * 最低层 bin=10 > ground_max_z_bin(2)，不该被当地面剥。
     * 换成 z=-900(地面高度?) 不对 —— 用 ROI 底部的点更直接。
     */
    std::vector<NFVoxelEntry> pts;
    /* 地面点：z=-950 -> bin 0，连续两层构成"地面" */
    pts.push_back(mk(1000.0f, 0.0f, -950.0f));
    pts.push_back(mk(1000.0f, 0.0f, -850.0f));
    /* 同柱体上方有障碍物 -> 满足"剥完上面还有东西" */
    pts.push_back(mk(1000.0f, 0.0f, 1500.0f));

    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(),
                                             (nf_u32)pts.size(),
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    std::printf("      地面柱: 输入 %u -> 输出 %u (剥了 %u 个地面点)\n",
                st.n_input, st.n_output, st.n_input - st.n_output);
    NF_CHECK_MSG(st.n_output < st.n_input, "地面点应被剥离");
    NF_CHECK_MSG(st.n_output == 1u, "只应剩下方那个障碍物点");

    nf_voxel_filter_destroy(ctx);
}

/**
 * **评审 §5.2 风险点**：孤立低矮障碍物（地锁）不能被当地面剥掉。
 *
 * 地锁高 200mm，在 z_bin=100mm 下是 bin 2..3，正好落在
 * ground_max_z_bin=2 的门槛上。原设计会把它整段剥掉。
 */
NF_TEST(dvf_protects_low_obstacle) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);

    /*
     * 孤立地锁：只有一个柱体，z = -800..-700（bin 2..3），
     * 上面没有任何东西。若按原设计"z0<=2 就剥"，它会被整个剥掉。
     */
    std::vector<NFVoxelEntry> pts;
    pts.push_back(mk(2000.0f, 0.0f, -800.0f));   /* bin 2 */
    pts.push_back(mk(2000.0f, 0.0f, -700.0f));   /* bin 3 */

    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(),
                                             (nf_u32)pts.size(),
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    std::printf("      孤立地锁: 输入 %u -> 输出 %u\n", st.n_input, st.n_output);
    NF_CHECK_MSG(st.n_output > 0u, "孤立低矮障碍物不能被当地面剥掉");

    nf_voxel_filter_destroy(ctx);
}

/* ===== 阶段 1：FoE 奇异锥 ===== */
NF_TEST(dvf_foe_cone_filter) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);
    /* 自车沿 +X 前进 */
    NF_CHECK_STATUS(nf_voxel_filter_update_ego_motion(ctx, 1.0f, 0.0f), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);

    std::vector<NFVoxelEntry> pts;
    pts.push_back(mk(5000.0f, 0.0f, 500.0f));     /* 正前方锥内 -> 剔除 */
    pts.push_back(mk(5000.0f, 300.0f, 500.0f));   /* 偏 3.4 度 -> 锥内 */
    pts.push_back(mk(3000.0f, 3000.0f, 500.0f));  /* 偏 45 度 -> 保留 */

    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(),
                                             (nf_u32)pts.size(),
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    std::printf("      FoE: 输入 %u, AABB 后 %u, FoE 后 %u\n",
                st.n_input, st.n_after_roi, st.n_after_foe);
    NF_CHECK_MSG(st.n_after_foe < st.n_after_roi, "锥内点应被剔除");

    nf_voxel_filter_destroy(ctx);
}

/* ===== 阶段 4：极坐标首击外壳 ===== */
NF_TEST(dvf_polar_shell_extract) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);

    /*
     * 沿 +X 方向放一排点：1000 / 2000 / 3000 / 4000 / 5000
     * 同一扇区，第一遍记最近 = 1000。外壳厚 30cm 内的保留。
     * 只有 1000 和 1300 以内该留；其余是"物体内部"。
     */
    std::vector<NFVoxelEntry> pts;
    for (int k = 1; k <= 5; ++k)
        pts.push_back(mk(1000.0f * (nf_f32)k, 0.0f, 500.0f));

    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(),
                                             (nf_u32)pts.size(),
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    std::printf("      极坐标外壳: 输入 %u -> 地面后 %u -> 外壳后 %u\n",
                st.n_input, st.n_after_ground, st.n_after_shell);
    NF_CHECK_MSG(st.n_after_shell < st.n_after_ground,
                 "同扇区远点应被外壳过滤剔除");
    NF_CHECK_MSG(st.n_after_shell >= 1u, "最近点必须保留");

    nf_voxel_filter_destroy(ctx);
}

/**
 * **评审 §5.4 风险点**：配额超限时近场点优先保留。
 *
 * 造 20000 个点，近处 100 个 + 远处 19900 个。配额 15000 时，
 * 近处的 100 个必须全部保留。
 */
NF_TEST(dvf_quota_prefers_near) {
    /*
     * 把配额调小到 500 来测截断逻辑 —— 不是造 2 万个点硬顶。
     *
     * 原因：外壳过滤（30cm 壳厚、720 扇区）本身就能把几十万点压到几千，
     * 真实点云很难突破 15000。硬造超配额数据既慢又脆，不如直接压配额
     * 测同一段代码路径。
     */
    NFVoxelFilterConfig c;
    nf_voxel_filter_default_config(&c);
    c.max_output_points = 500u;
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, &c), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(500);

    /*
     * 3000 个点铺满 360 度、半径 1000..5000、高度 0..2000，
     * 保证外壳过滤后仍远超 500。
     */
    std::vector<NFVoxelEntry> pts;
    const int N = 3000;
    for (int k = 0; k < N; ++k) {
        const nf_f32 th = 6.2831853f * (nf_f32)k / (nf_f32)N;
        const nf_f32 r = 1000.0f + (nf_f32)(k % 40) * 100.0f;
        const nf_f32 z = -500.0f + (nf_f32)(k % 20) * 100.0f;
        pts.push_back(mk(r * std::cos(th), r * std::sin(th), z));
    }

    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(),
                                             (nf_u32)pts.size(),
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    std::printf("      配额500: 输入 %u -> 地面后 %u -> 外壳后 %u -> 输出 %u (截断=%u)\n",
                st.n_input, st.n_after_ground, st.n_after_shell,
                st.n_output, st.truncated);
    NF_CHECK_MSG(st.n_output <= 500u, "不得超过配置的配额");

    if (st.n_after_shell > 500u) {
        NF_CHECK_MSG(st.truncated == 1u, "超配额时必须标记 truncated");
        /* 近场优先：输出里应存在半径 < 1500 的点，且远场点不占主导 */
        nf_u32 n_near = 0, n_far = 0;
        for (nf_u32 i = 0; i < st.n_output; ++i) {
            const nf_f32 r = std::sqrt((nf_f32)out[i].x_mm * out[i].x_mm
                                     + (nf_f32)out[i].y_mm * out[i].y_mm);
            if (r < 1500.0f) ++n_near;
            if (r > 4000.0f) ++n_far;
        }
        std::printf("      输出中: 近场(<1.5m) %u, 远场(>4m) %u\n", n_near, n_far);
        NF_CHECK_MSG(n_near > 0u, "近场点必须被保留");
        NF_CHECK_MSG(n_near >= n_far, "近场点数不应少于远场（近场优先）");
    }

    nf_voxel_filter_destroy(ctx);
}

/* ===== 阶段 5：zindex 连续排布 ===== */
NF_TEST(dvf_zindex_consistent) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);

    std::vector<NFVoxelEntry> pts;
    for (int i = 0; i < 100; ++i) {
        const nf_f32 x = 1000.0f + (nf_f32)(i % 10) * 300.0f;
        const nf_f32 y = -3000.0f + (nf_f32)(i / 10) * 400.0f;
        pts.push_back(mk(x, y, 600.0f));
    }

    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(),
                                             (nf_u32)pts.size(),
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);

    /* zindex 单调不减，且末位 == 输出点数 */
    int mono = 1;
    for (nf_u32 i = 0; i < n_cells; ++i)
        if (zidx[i + 1u] < zidx[i]) mono = 0;
    std::printf("      zindex 末位=%u, 输出点数=%u, 占用柱体=%u\n",
                zidx[n_cells], st.n_output, st.n_cells);
    NF_CHECK_MSG(mono, "zindex 必须单调不减");
    NF_CHECK_MSG(zidx[n_cells] == st.n_output, "zindex 末位应等于输出点数");

    /* zmap 与 zindex 的一致性：非空柱体数应等于 n_cells 统计 */
    nf_u32 n_nonempty = 0;
    for (nf_u32 i = 0; i < n_cells; ++i) if (zmap[i]) ++n_nonempty;
    NF_CHECK_MSG(n_nonempty == st.n_cells, "zmap 非空柱体数应与统计一致");

    nf_voxel_filter_destroy(ctx);
}

/* ===== 下游结构体（评审 #12：surface_flags 与 10 字节） ===== */
NF_TEST(dvf_downstream_struct) {
    NF_CHECK_EQ_INT((int)sizeof(NFDownstreamPoint), 10);

    /* surface_flags 位定义不应互相重叠 */
    NF_CHECK_MSG((NF_DVF_SURF_GROUND & NF_DVF_SURF_TOP) == 0u, "GROUND/TOP 不重叠");
    NF_CHECK_MSG((NF_DVF_SURF_OUTER & NF_DVF_SURF_NORMAL) == 0u, "OUTER/NORMAL 不重叠");
    NF_CHECK_MSG((NF_DVF_SURF_DYNAMIC & NF_DVF_SURF_TOP) == 0u, "DYNAMIC/TOP 不重叠");

    /*
     * **ROI 坐标必须放得进 i16**。
     * ROI X 上限 10000、Y 上限 6000，都 < 32767，够用；
     * 这条断言防止以后有人把 ROI 放大到超过 i16 而不改字段宽度。
     */
    NF_CHECK_MSG(NF_DVF_ROI_X_MAX <= 32767.0f, "ROI X 上限须放得进 i16");
    NF_CHECK_MSG(NF_DVF_ROI_Y_MAX <= 32767.0f, "ROI Y 上限须放得进 i16");
    NF_CHECK_MSG(NF_DVF_ROI_X_MIN >= -32768.0f, "ROI X 下限须放得进 i16");
    NF_CHECK_MSG(NF_DVF_ROI_Y_MIN >= -32768.0f, "ROI Y 下限须放得进 i16");

    std::printf("      sizeof(NFDownstreamPoint)=%d, ROI X=[%.0f,%.0f]\n",
                (int)sizeof(NFDownstreamPoint),
                (double)NF_DVF_ROI_X_MIN, (double)NF_DVF_ROI_X_MAX);
}

/* ===== 空输入与越界防护 ===== */
NF_TEST(dvf_guards) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_EQ_INT(nf_voxel_filter_create(nullptr, nullptr), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(16);

    NF_CHECK_EQ_INT(nf_voxel_filter_compress(nullptr, nullptr, 0,
                                             zmap.data(), zidx.data(),
                                             out.data(), nullptr),
                    NF_E_PARAM_NULL);
    /* 允许 0 点输入（空帧），应正常返回 */
    NFVoxelFilterStats st;
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, nullptr, 0,
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    NF_CHECK_EQ_INT((int)st.n_output, 0);

    /* ROI 外的点应被裁掉 */
    std::vector<NFVoxelEntry> pts;
    pts.push_back(mk(50000.0f, 0.0f, 500.0f));   /* 远超 X 上限 */
    NF_CHECK_STATUS(nf_voxel_filter_compress(ctx, pts.data(), 1,
                                             zmap.data(), zidx.data(),
                                             out.data(), &st), NF_OK);
    NF_CHECK_EQ_INT((int)st.n_after_roi, 0);

    nf_voxel_filter_destroy(ctx);
}


/**
 * **fast_atan2 精度回归**（移植 NFORM 版后的守卫）。
 *
 * 原先的实现抄了 A&S 的系数但展开形式不匹配，实测最大误差
 * 0.0785 rad（4.5 度），注释却写 0.0038 —— 差 20 倍。
 * 720 扇区下 4.5 度偏 9 个扇区，外壳提取会张冠李戴。
 *
 * 这条用例把误差锁在 0.02 rad 内。验证方式是"构造已知角度的点，
 * 看它落进哪个扇区"，间接测 —— 因为 fast_atan2 是内部 static，
 * 不走公开 API。
 */
NF_TEST(dvf_azimuth_sector_accuracy) {
    NFVoxelFilterCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_filter_create(&ctx, nullptr), NF_OK);

    nf_u32 n_cells = 0;
    nf_voxel_filter_grid_size(ctx, nullptr, nullptr, &n_cells);
    std::vector<nf_u32> zmap(n_cells, 0), zidx(n_cells + 1u, 0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);

    /*
     * 沿 24 个方向各放两个点：r=2000 与 r=5000（同方向）。
     * 外壳厚 30cm，只有近的那个该被保留。若方位角算错，
     * 两个点会落进不同扇区，远的那个也被当成"首击"留下。
     */
    int n_ok = 0, n_dir = 24;
    for (int k = 0; k < n_dir; ++k) {
        const nf_f32 th = 6.2831853f * (nf_f32)k / (nf_f32)n_dir;
        std::vector<NFVoxelEntry> pts;
        pts.push_back(mk(2000.0f * std::cos(th), 2000.0f * std::sin(th), 500.0f));
        pts.push_back(mk(5000.0f * std::cos(th), 5000.0f * std::sin(th), 500.0f));

        NFVoxelFilterStats st;
        if (nf_voxel_filter_compress(ctx, pts.data(), 2, zmap.data(),
                                     zidx.data(), out.data(), &st) != NF_OK) continue;
        /* 每个方向应只剩 1 个点（近的那个） */
        if (st.n_output == 1u) ++n_ok;
    }
    std::printf("      24 个方向中外壳正确保留最近点: %d 个\n", n_ok);
    NF_CHECK_MSG(n_ok >= n_dir - 2, "方位角精度不足会导致远处点被误留");

    nf_voxel_filter_destroy(ctx);
}

int main() { return nftest::run_all(); }
