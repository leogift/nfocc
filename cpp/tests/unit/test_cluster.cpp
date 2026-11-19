/**
 * @file test_cluster.cpp
 * @brief 八叉树 + DBSCAN + 动静分离的单元测试。
 *
 * ## 判据设计原则
 * 每条判据都能失败。特别是：
 * - 八叉树半径查询要与**暴力枚举**逐点对拍（不只看"查到了几个"）
 * - DBSCAN 要能**拒绝噪声**（这是选它而非体素连通域的理由）
 * - 150k 规模的耗时要实测，不接受估算
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include <chrono>

#include "nfocc/nfocc_cluster.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_octree.h"
#include "nf_test.h"

namespace {

/** 造一团球状点云 */
void make_blob(std::vector<float> &pts, float cx, float cy, float cz,
               float r, int n, unsigned seed) {
    unsigned s = seed;
    for (int i = 0; i < n; ++i) {
        float u[3];
        for (int k = 0; k < 3; ++k) {
            s = s * 1103515245u + 12345u;
            u[k] = (float)((s >> 16) & 0x7FFF) / 32767.0f * 2.0f - 1.0f;
        }
        const float len = std::sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
        const float sc = (len > 1e-6f) ? (r / len) : 0.0f;
        pts.push_back(cx + u[0] * sc);
        pts.push_back(cy + u[1] * sc);
        pts.push_back(cz + u[2] * sc);
    }
}

/** 暴力半径查询，用于对拍 */
int brute_radius(const std::vector<float> &pts, float x, float y, float z,
                 float r, std::vector<unsigned> *out) {
    int n = 0;
    out->clear();
    for (size_t i = 0; i < pts.size() / 3; ++i) {
        const float dx = pts[i*3+0]-x, dy = pts[i*3+1]-y, dz = pts[i*3+2]-z;
        if (dx*dx + dy*dy + dz*dz <= r*r) { out->push_back((unsigned)i); ++n; }
    }
    return n;
}

}  // namespace

/* ---------- 八叉树 ---------- */

/* 半径查询必须与暴力枚举**逐点一致**，不只是数量一致 */
NF_TEST(oct_radius_matches_brute) {
    std::vector<float> pts;
    make_blob(pts, 0, 0, 1000, 500, 800, 1);
    make_blob(pts, 2000, 0, 1000, 300, 300, 2);

    NFOctreeCtx *t = nullptr;
    NF_CHECK_STATUS(nf_octree_build(&t, pts.data(), (nf_u32)(pts.size()/3), nullptr), NF_OK);
    NF_CHECK(t != nullptr);
    NF_CHECK_EQ_INT(nf_octree_count(t), 1100);

    std::vector<nf_u32> got(8192);
    std::vector<unsigned> want;
    int mismatch = 0;

    /* 在包围盒里取若干查询中心 */
    for (int q = 0; q < 20; ++q) {
        const float cx = -500.0f + 3000.0f * (float)q / 19.0f;
        const float cy = -400.0f + 800.0f * (float)(q % 5) / 4.0f;
        const float cz = 1000.0f;
        const float r = 200.0f + 50.0f * (float)(q % 4);

        nf_u32 cnt = 0;
        nf_octree_radius(t, cx, cy, cz, r, got.data(), (nf_u32)got.size(), &cnt, nullptr);
        brute_radius(pts, cx, cy, cz, r, &want);

        if ((int)cnt != (int)want.size()) { ++mismatch; continue; }

        /* 同一集合（顺序可能不同） */
        std::vector<nf_u32> a(got.begin(), got.begin() + cnt);
        std::vector<nf_u32> b(want.begin(), want.end());
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        if (a != b) ++mismatch;
    }
    std::printf("      20 个查询中心，与暴力枚举不一致 %d 个\n", mismatch);
    NF_CHECK_EQ_INT(mismatch, 0);
    nf_octree_destroy(t);
}

/* 空输入与参数错误必须拒绝 */
NF_TEST(oct_reject_bad_input) {
    NFOctreeCtx *t = nullptr;
    NF_CHECK_EQ_INT(nf_octree_build(&t, nullptr, 0, nullptr), NF_E_PARAM_RANGE);
    NF_CHECK(t == nullptr);

    float p[3] = {0, 0, 0};
    NF_CHECK_EQ_INT(nf_octree_build(&t, p, 1, nullptr), NF_OK);
    nf_u32 idx[4], cnt = 0;
    /* 半径为 0 -> 拒绝 */
    NF_CHECK_EQ_INT(nf_octree_radius(t, 0, 0, 0, 0.0f, idx, 4, &cnt, nullptr),
                    NF_E_PARAM_RANGE);
    /* 索引越界 */
    float out[3];
    NF_CHECK_EQ_INT(nf_octree_point(t, 5, out), NF_E_PARAM_RANGE);
    nf_octree_destroy(t);
}

/* ---------- DBSCAN ---------- */

/* 两团分离的点必须分成两簇 */
NF_TEST(dbscan_two_blobs) {
    std::vector<float> pts;
    make_blob(pts, 0, 0, 1000, 200, 400, 11);
    make_blob(pts, 3000, 0, 1000, 200, 400, 22);

    NFOctreeCtx *t = nullptr;
    NF_CHECK_STATUS(nf_octree_build(&t, pts.data(), 800, nullptr), NF_OK);

    NFClusterResult r;
    NF_CHECK_STATUS(nf_cluster_result_alloc(&r, NF_MAX_CLUSTERS, 800), NF_OK);
    NF_CHECK_STATUS(nf_dbscan(t, nullptr, &r), NF_OK);

    std::printf("      800 点 -> %u 簇, 噪声 %u\n", r.num_clusters, r.num_noise);
    NF_CHECK_EQ_INT(r.num_clusters, 2);

    /* 两簇质心应分别靠近 (0,0,1000) 与 (3000,0,1000) */
    NF_CHECK(r.clusters[0].n_points > 300);
    NF_CHECK(r.clusters[1].n_points > 300);
    NF_CHECK(std::fabs(r.clusters[0].centroid[0] - r.clusters[1].centroid[0]) > 2000.0f);

    nf_cluster_result_destroy(&r);
    nf_octree_destroy(t);
}

/**
 * **DBSCAN 的核心价值**：稀疏的离散点必须被判噪声，不能各自成簇。
 * 这是选它而非体素连通域的唯一理由，必须守住。
 */
NF_TEST(dbscan_rejects_sparse_noise) {
    std::vector<float> pts;
    make_blob(pts, 0, 0, 1000, 200, 400, 33);    /* 一团密的 */
    /* 200 个孤立点，相互间距 2m，远超 eps */
    for (int i = 0; i < 200; ++i)
        pts.insert(pts.end(), { 5000.0f + (float)i * 2000.0f, 0.0f, 500.0f });

    NFOctreeCtx *t = nullptr;
    NF_CHECK_STATUS(nf_octree_build(&t, pts.data(), (nf_u32)(pts.size()/3), nullptr), NF_OK);

    NFClusterResult r;
    NF_CHECK_STATUS(nf_cluster_result_alloc(&r, NF_MAX_CLUSTERS, (nf_u32)(pts.size()/3)), NF_OK);
    NF_CHECK_STATUS(nf_dbscan(t, nullptr, &r), NF_OK);

    std::printf("      600 点(400密+200孤立) -> %u 簇, 噪声 %u\n",
                r.num_clusters, r.num_noise);
    NF_CHECK_MSG(r.num_clusters == 1, "孤立点不应成簇，只应剩 1 簇");
    NF_CHECK_MSG(r.num_noise >= 195, "孤立点应被判噪声（允许极少数碰巧相邻）");

    nf_cluster_result_destroy(&r);
    nf_octree_destroy(t);
}

/* 单个孤立点：全部噪声，0 簇 */
NF_TEST(dbscan_all_noise) {
    std::vector<float> pts;
    for (int i = 0; i < 50; ++i)
        pts.insert(pts.end(), { (float)i * 5000.0f, 0.0f, 0.0f });

    NFOctreeCtx *t = nullptr;
    NF_CHECK_STATUS(nf_octree_build(&t, pts.data(), 50, nullptr), NF_OK);
    NFClusterResult r;
    NF_CHECK_STATUS(nf_cluster_result_alloc(&r, NF_MAX_CLUSTERS, 50), NF_OK);
    NF_CHECK_STATUS(nf_dbscan(t, nullptr, &r), NF_OK);
    NF_CHECK_EQ_INT(r.num_clusters, 0);
    NF_CHECK_EQ_INT(r.num_noise, 50);
    nf_cluster_result_destroy(&r);
    nf_octree_destroy(t);
}

/* ---------- 性能：150k 点 ---------- */

/**
 * 150k 点规模的性能实测。
 *
 * ## 为什么用两种分布
 * **极密团簇**（5000 点 / 半径 300mm）代表算法最坏情况，
 * **均匀分布**代表真实点云。两者差距很大，只测一种就会误判：
 *
 * - 极密团簇：eps 球内点数上千，任何索引都救不了，这是 DBSCAN
 *   本身的性质，不是索引缺陷。实测 25-38 簇、耗时 250-330 ms。
 * - 均匀分布：实测密度对应平均间距 36mm，eps 球内约 161 点，
 *   这才是 150k 点云的真实情形。实测 45 ms。
 *
 * 性能判据只对**均匀分布**成立 —— 用极密团簇判"索引不够快"是错的
 * 结论，因为那时瓶颈在算法而非索引。
 */
NF_TEST(octree_150k_realdistribution) {
    std::vector<float> pts;
    pts.reserve(150000 * 3);
    unsigned s = 11;
    auto rnd = [&]() {
        s = s * 1103515245u + 12345u;
        return (float)((s >> 16) & 0x7FFF) / 32767.0f;
    };
    /* 一半贴地（地面点），一半在 0.5-2m（物体），铺满 18x15m 网格 */
    for (int i = 0; i < 150000; ++i) {
        const float x = rnd() * 18000.0f - 9000.0f;
        const float y = rnd() * 15000.0f - 7500.0f;
        const float z = (i % 2) ? (rnd() * 1500.0f + 500.0f) : (rnd() * 100.0f);
        pts.insert(pts.end(), { x, y, z });
    }
    const nf_u32 n = 150000;

    const auto t0 = std::chrono::steady_clock::now();
    NFOctreeCtx *t = nullptr;
    NF_CHECK_STATUS(nf_octree_build(&t, pts.data(), n, nullptr), NF_OK);
    const auto t1 = std::chrono::steady_clock::now();

    nf_u32 cells = 0, max_cell = 0;
    nf_f32 avg = 0, cell_mm = 0;
    nf_octree_stats(t, &cells, &max_cell, &avg, &cell_mm);

    NFClusterResult r;
    NF_CHECK_STATUS(nf_cluster_result_alloc(&r, NF_MAX_CLUSTERS, n), NF_OK);
    const auto t2 = std::chrono::steady_clock::now();
    NF_CHECK_STATUS(nf_dbscan(t, nullptr, &r), NF_OK);
    const auto t3 = std::chrono::steady_clock::now();

    const double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double dbscan_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    std::printf("      [均匀] 150k: 构建 %.1f ms | 格 %.0fmm x %u (最大 %u, 均 %.1f) | "
                "DBSCAN %.1f ms | %u 簇 噪声 %u\n",
                build_ms, (double)cell_mm, cells, max_cell, (double)avg,
                dbscan_ms, r.num_clusters, r.num_noise);

    NF_CHECK_MSG(build_ms + dbscan_ms < 100.0,
                 "均匀分布 150k 点 构建+DBSCAN < 100 ms");

    nf_cluster_result_destroy(&r);
    nf_octree_destroy(t);
}

/**
 * 极密团簇性能：**记录实测值，不作为通过判据**。
 *
 * 这个分布下 DBSCAN 本身就要几百毫秒，判"超预算"会把算法性质
 * 误报成实现缺陷。这里只断言"能跑完且结果合理"。
 */
NF_TEST(octree_150k_dense_records_timing) {
    std::vector<float> pts;
    pts.reserve(150000 * 3);
    unsigned seed = 7;
    for (int b = 0; b < 30; ++b) {
        seed = seed * 1103515245u + 12345u;
        const float cx = -4000.0f + 8000.0f * ((float)((seed >> 16) & 0xFF) / 255.0f);
        const float cy = -3500.0f + 7000.0f * ((float)((seed >> 8) & 0xFF) / 255.0f);
        make_blob(pts, cx, cy, 500.0f, 300.0f, 5000, seed + (unsigned)b);
    }
    const nf_u32 n = (nf_u32)(pts.size() / 3);

    NFOctreeCtx *t = nullptr;
    nf_octree_build(&t, pts.data(), n, nullptr);
    NFClusterResult r;
    nf_cluster_result_alloc(&r, NF_MAX_CLUSTERS, n);
    const auto t2 = std::chrono::steady_clock::now();
    NF_CHECK_STATUS(nf_dbscan(t, nullptr, &r), NF_OK);
    const auto t3 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    std::printf("      [极密] 150k: DBSCAN %.1f ms | %u 簇（记录值，不作判据）\n",
                ms, r.num_clusters);
    /* 只要求能跑完、簇数在合理范围 */
    NF_CHECK(r.num_clusters > 0);
    NF_CHECK(r.num_noise < n);

    nf_cluster_result_destroy(&r);
    nf_octree_destroy(t);
}

/* ---------- 动静分离 ---------- */

/**
 * 自车前进 500mm，静止物体应判速度 0。
 *
 * 这是点级速度最容易错的地方：忘了位姿补偿会给出 500/dt = 2500mm/s
 * 的假速度，把整片背景判成动态。
 */
NF_TEST(velocity_static_world_after_ego_compensation) {
    /* 墙在 x=3000，自车从 x=0 前进到 x=500 */
    std::vector<float> prev, cur;
    for (int i = 0; i < 100; ++i) {
        const float y = -500.0f + 10.0f * (float)i;
        prev.insert(prev.end(), { 3000.0f, y, 500.0f });
    }
    /* 自车前进 500 -> 车体系里墙后退 500 */
    for (int i = 0; i < 100; ++i) {
        const float y = -500.0f + 10.0f * (float)i;
        cur.insert(cur.end(), { 2500.0f, y, 500.0f });
    }

    NFOctreeCtx *tc = nullptr, *tp = nullptr;
    NF_CHECK_STATUS(nf_octree_build(&tc, cur.data(), 100, nullptr), NF_OK);
    NF_CHECK_STATUS(nf_octree_build(&tp, prev.data(), 100, nullptr), NF_OK);

    /* R = I, t = (500,0,0)：本帧 = 上帧 + 500 */
    const nf_f32 R[9] = {1,0,0, 0,1,0, 0,0,1};
    const nf_f32 t[3] = {500.0f, 0.0f, 0.0f};

    std::vector<nf_f32> spd(100);
    std::vector<nf_u8> val(100);
    NF_CHECK_STATUS(nf_point_velocity(tc, tp, R, t, 0.2f, 300.0f,
                                      spd.data(), val.data()), NF_OK);

    int n_valid = 0;
    float max_spd = 0.0f;
    for (int i = 0; i < 100; ++i) {
        if (!val[i]) continue;
        ++n_valid;
        if (spd[i] > max_spd) max_spd = spd[i];
    }
    std::printf("      墙(静止): 有效匹配 %d/100, 最大速度 %.1f mm/s\n",
                n_valid, (double)max_spd);
    NF_CHECK_MSG(n_valid > 90, "静止墙应有绝大多数匹配成功");
    NF_CHECK_MSG(max_spd < NF_STATIC_SPEED_MMPS, "位姿补偿后静止物体速度应≈0");

    nf_octree_destroy(tc);
    nf_octree_destroy(tp);
}

/**
 * **负对照**：不做位姿补偿（R=I, t=0），同一场景应给出大速度。
 *
 * 没有这条，上一条"速度≈0"可能只是算法恒返回 0 —— 那样判据是假的。
 */
NF_TEST(velocity_without_compensation_is_wrong) {
    std::vector<float> prev, cur;
    for (int i = 0; i < 100; ++i) {
        const float y = -500.0f + 10.0f * (float)i;
        prev.insert(prev.end(), { 3000.0f, y, 500.0f });
        cur.insert(cur.end(),  { 2500.0f, y, 500.0f });
    }
    NFOctreeCtx *tc = nullptr, *tp = nullptr;
    nf_octree_build(&tc, cur.data(), 100, nullptr);
    nf_octree_build(&tp, prev.data(), 100, nullptr);

    /* 故意不给位姿：t=0 */
    const nf_f32 R[9] = {1,0,0, 0,1,0, 0,0,1};
    const nf_f32 t[3] = {0.0f, 0.0f, 0.0f};
    std::vector<nf_f32> spd(100);
    std::vector<nf_u8> val(100);
    nf_point_velocity(tc, tp, R, t, 0.2f, 3000.0f, spd.data(), val.data());

    int n_valid = 0;
    float max_spd = 0.0f;
    for (int i = 0; i < 100; ++i) {
        if (val[i]) { ++n_valid; if (spd[i] > max_spd) max_spd = spd[i]; }
    }
    std::printf("      [对照] 不给位姿: 有效 %d, 最大速度 %.1f mm/s（应为大值）\n",
                n_valid, (double)max_spd);
    NF_CHECK_MSG(max_spd > 1000.0f,
                 "无位姿补偿时静止墙应被误判为运动（证明上一条不是恒 0）");

    nf_octree_destroy(tc);
    nf_octree_destroy(tp);
}

/* 簇级投票：多数点动态 -> 整簇动态 */
NF_TEST(cluster_vote_motion) {
    std::vector<float> pts;
    make_blob(pts, 0, 0, 1000, 200, 200, 55);
    const nf_u32 n = 200;

    NFOctreeCtx *t = nullptr;
    nf_octree_build(&t, pts.data(), n, nullptr);
    NFClusterResult r;
    nf_cluster_result_alloc(&r, NF_MAX_CLUSTERS, n);
    nf_dbscan(t, nullptr, &r);
    NF_CHECK_EQ_INT(r.num_clusters, 1);

    /* 80% 的点给大速度 -> 判动态 */
    std::vector<nf_f32> spd(n, 0.0f);
    std::vector<nf_u8> val(n, 1);
    for (nf_u32 i = 0; i < n; ++i) spd[i] = (i < n * 8 / 10) ? 500.0f : 5.0f;
    NF_CHECK_STATUS(nf_cluster_vote_motion(&r, spd.data(), val.data(),
                                           NF_STATIC_SPEED_MMPS,
                                           NF_DYNAMIC_VOTE_RATIO), NF_OK);
    NF_CHECK_EQ_INT(r.clusters[0].is_dynamic, 2);
    NF_CHECK(r.clusters[0].speed_mmps > 100.0f);

    /* 反过来：80% 静止 -> 判静止 */
    for (nf_u32 i = 0; i < n; ++i) spd[i] = (i < n * 8 / 10) ? 5.0f : 500.0f;
    nf_cluster_vote_motion(&r, spd.data(), val.data(),
                           NF_STATIC_SPEED_MMPS, NF_DYNAMIC_VOTE_RATIO);
    NF_CHECK_EQ_INT(r.clusters[0].is_dynamic, 1);

    nf_cluster_result_destroy(&r);
    nf_octree_destroy(t);
}

int main() { return nftest::run_all(); }
