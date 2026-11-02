/**
 * @file cluster.cpp
 * @brief DBSCAN + 动静分离实现。
 *
 * ## DBSCAN 的实现要点
 * 标准算法是 BFS 扩张：从一个未访问的核心点出发，把邻域内的点加入同一簇，
 * 对其中未访问的核心点继续扩张。
 *
 * 关键约束是**每个点只被查询一次邻域**，否则复杂度退化成 O(N²)。
 * 用 `state[]` 三态标记（未访问/已入队/已处理）保证这一点。
 *
 * ## 核心库约束
 * 禁 STL 容器。BFS 队列、邻域缓冲都是预分配 malloc + 游标。
 * 邻域缓冲容量按最坏情形给：单叶点数 x 27（3x3x3），实测 9 x 27 = 243，
 * 留 4 倍余量取 1024。超容时八叉树返回 truncated，这里按截断处理
 * （宁可少几个邻域点，也不能溢出）。
 */
#include "nfocc/nfocc_cluster.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

/**
 * 邻域查询缓冲的**默认**容量。
 *
 * 自适应细分后单叶点数稳定在 max_leaf_points(16) 以下，
 * eps=120mm 时球内点数约几十到几百。4096 是安全值的 10 倍以上。
 *
 * 实际容量会在 dbscan 开始时按 `叶内最大点数 x 64` 收紧或放宽，
 * 避免极端密度下截断（截断会让核心点判据失真，聚出错误的簇）。
 */
namespace { const nf_u32 kNeighborCapDefault = 4096u; }

NFStatus nf_cluster_result_alloc(NFClusterResult *r, nf_u32 max_clusters,
                                 nf_u32 max_points) {
    if (!r) return NF_E_PARAM_NULL;
    std::memset(r, 0, sizeof(*r));
    r->version = NF_CLUSTER_VERSION;

    r->clusters = (NFCluster *)std::calloc(max_clusters, sizeof(NFCluster));
    r->point_cluster = (nf_u32 *)std::malloc((size_t)max_points * sizeof(nf_u32));
    if (!r->clusters || !r->point_cluster) {
        nf_cluster_result_destroy(r);
        return NF_E_ENGINE_OOM;
    }
    r->num_points = max_points;
    for (nf_u32 i = 0; i < max_points; ++i)
        r->point_cluster[i] = NF_CLUSTER_NOISE;
    return NF_OK;
}

void nf_cluster_result_destroy(NFClusterResult *r) {
    if (!r) return;
    std::free(r->clusters);
    std::free(r->point_cluster);
    r->clusters = nullptr;
    r->point_cluster = nullptr;
    r->num_clusters = 0;
}

namespace {

/** 三态：0=未访问 1=已入队 2=已处理 */
const nf_u8 kUnvisited = 0;
const nf_u8 kQueued    = 1;
const nf_u8 kDone      = 2;

struct DbsCtx {
    const NFOctreeCtx *tree;
    nf_f32 eps;
    nf_u32 min_pts;
    nf_u32 *label;
    nf_u8  *state;
    nf_u32 *queue;
    nf_u32 *nb;
    nf_u32  n;
    nf_u32  nb_cap;     /**< 邻域缓冲容量，按叶密度动态定 */
};

/** 查询邻域，返回球内点数（含自身） */
nf_u32 query_neighbors(DbsCtx *c, nf_u32 pi, nf_u32 *out_trunc) {
    nf_f32 p[3];
    nf_octree_point(c->tree, pi, p);
    nf_u32 cnt = 0;
    nf_u8 trunc = 0;
    nf_octree_radius(c->tree, p[0], p[1], p[2], c->eps,
                     c->nb, c->nb_cap, &cnt, &trunc);
    if (out_trunc) *out_trunc = trunc;
    /* 若查询结果超容，cnt > nb_cap；按实际可用数使用 */
    return (cnt > c->nb_cap) ? c->nb_cap : cnt;
}

}  // namespace

extern "C" {

NFStatus nf_dbscan(const NFOctreeCtx *tree, const NFClusterConfig *cfg,
                   NFClusterResult *out) {
    if (!tree || !out || !out->clusters || !out->point_cluster)
        return NF_E_PARAM_NULL;

    const nf_u32 n = nf_octree_count(tree);
    if (n == 0) { out->num_clusters = 0; out->num_noise = 0; return NF_OK; }
    if (n > out->num_points) return NF_E_PARAM_RANGE;

    const nf_f32 eps = (cfg && cfg->eps_mm > 0.0f) ? cfg->eps_mm
                                                   : (nf_f32)NF_DBSCAN_EPS_MM;
    const nf_u32 min_pts = (cfg && cfg->min_pts > 0u) ? cfg->min_pts
                                                     : (nf_u32)NF_DBSCAN_MIN_PTS;
    const nf_u32 max_clus = (cfg && cfg->max_clusters > 0u) ? cfg->max_clusters
                                                           : (nf_u32)NF_MAX_CLUSTERS;

    DbsCtx c;
    std::memset(&c, 0, sizeof(c));
    c.tree = tree; c.eps = eps; c.min_pts = min_pts; c.n = n;

    c.label = (nf_u32 *)std::malloc((size_t)n * sizeof(nf_u32));
    c.state = (nf_u8 *)std::calloc(n, 1);
    c.queue = (nf_u32 *)std::malloc((size_t)n * sizeof(nf_u32));
    /* 按实际叶密度定容量：eps 球覆盖的叶数上限 x 单叶最多点数 */
    {
        nf_u32 max_leaf = 0;
        nf_f32 cell_mm = (nf_f32)NF_HASH_CELL_MM;
        nf_octree_stats(tree, nullptr, &max_leaf, nullptr, &cell_mm);
        const nf_f32 leaf_mm = cell_mm;
        const nf_u32 leaves_in_ball =
            (nf_u32)(8.0f * (1.0f + eps / leaf_mm) * (1.0f + eps / leaf_mm)
                     * (1.0f + eps / leaf_mm));
        nf_u32 cap = leaves_in_ball * (max_leaf + 1u);
        if (cap < 256u) cap = 256u;
        if (cap > 65536u) cap = 65536u;
        c.nb_cap = cap;
    }
    c.nb = (nf_u32 *)std::malloc((size_t)c.nb_cap * sizeof(nf_u32));
    if (!c.label || !c.state || !c.queue || !c.nb) {
        std::free(c.label); std::free(c.state);
        std::free(c.queue); std::free(c.nb);
        return NF_E_ENGINE_OOM;
    }
    for (nf_u32 i = 0; i < n; ++i) c.label[i] = NF_CLUSTER_NOISE;

    nf_u32 n_clusters = 0;
    nf_u32 n_noise = 0;

    for (nf_u32 i = 0; i < n; ++i) {
        if (c.state[i] != kUnvisited) continue;

        const nf_u32 nb_cnt = query_neighbors(&c, i, nullptr);
        if (nb_cnt < min_pts) {
            /* 非核心点：暂时标噪声。**注意** DBSCAN 允许它后来成为
             * 某个核心点邻域内的边界点，所以 state 标 done 但 label 不放 */
            c.state[i] = kDone;
            continue;
        }

        /* --- 新簇：BFS 扩张 --- */
        const nf_u32 cid = n_clusters;
        if (cid >= max_clus) {
            /* 簇数超限：剩余全部标噪声，避免越界 */
            c.state[i] = kDone;
            continue;
        }
        ++n_clusters;

        nf_u32 q_head = 0, q_tail = 0;
        c.state[i] = kQueued;
        c.label[i] = cid;
        c.queue[q_tail++] = i;

        while (q_head < q_tail) {
            const nf_u32 qp = c.queue[q_head++];
            c.state[qp] = kDone;

            /* 对加入队列的点查邻域；核心点则继续扩张 */
            const nf_u32 qn = query_neighbors(&c, qp, nullptr);
            if (qn < min_pts) continue;   /* 边界点：只入簇，不扩张 */

            for (nf_u32 k = 0; k < qn; ++k) {
                const nf_u32 r = c.nb[k];
                if (c.state[r] == kDone) {
                    /* 已被处理但仍是噪声 -> 吸收为边界点 */
                    if (c.label[r] == NF_CLUSTER_NOISE) c.label[r] = cid;
                    continue;
                }
                if (c.state[r] == kUnvisited) {
                    c.state[r] = kQueued;
                    c.label[r] = cid;
                    c.queue[q_tail++] = r;
                }
            }
        }
    }

    /* --- 统计噪声 --- */
    for (nf_u32 i = 0; i < n; ++i)
        if (c.label[i] == NF_CLUSTER_NOISE) ++n_noise;

    /* --- 汇总每簇的包围盒 / 质心 / 填充率 --- */
    std::memset(out->clusters, 0, (size_t)max_clus * sizeof(NFCluster));
    for (nf_u32 k = 0; k < n_clusters; ++k) {
        NFCluster *cl = &out->clusters[k];
        cl->id = k;
        cl->min[0] = cl->min[1] = cl->min[2] = 1e30f;
        cl->max[0] = cl->max[1] = cl->max[2] = -1e30f;
    }

    /* 质心累加用 cluster 的 centroid 作临时和，最后再除 */
    for (nf_u32 i = 0; i < n; ++i) {
        const nf_u32 cid = c.label[i];
        if (cid == NF_CLUSTER_NOISE) continue;
        nf_f32 p[3];
        nf_octree_point(tree, i, p);
        NFCluster *cl = &out->clusters[cid];
        ++cl->n_points;
        for (int k = 0; k < 3; ++k) {
            if (p[k] < cl->min[k]) cl->min[k] = p[k];
            if (p[k] > cl->max[k]) cl->max[k] = p[k];
            cl->centroid[k] += p[k];
        }
    }

    const float vox = (float)NF_G_VOX;
    for (nf_u32 k = 0; k < n_clusters; ++k) {
        NFCluster *cl = &out->clusters[k];
        if (cl->n_points == 0) continue;
        for (int j = 0; j < 3; ++j) cl->centroid[j] /= (nf_f32)cl->n_points;

        /**
         * 填充率 = 点数 / 包围盒内的体素格数。
         *
         * 用途：区分"独立物体"与"贴着大片墙面的点"。后者包围盒很大但
         * 点都集中在一个面上，填充率极低。后续可用它做**凸性拆分**：
         * 填充率低于阈值说明这一簇可能是两个物体被桥接。
         */
        nf_f32 cells = 1.0f;
        for (int j = 0; j < 3; ++j) {
            const nf_f32 ext = cl->max[j] - cl->min[j];
            cells *= (ext < vox) ? 1.0f : std::ceil(ext / vox);
        }
        cl->fill_ratio = (cells > 0.0f) ? (nf_f32)cl->n_points / cells : 0.0f;
        cl->is_dynamic = 0;
        cl->speed_mmps = 0.0f;
    }

    /* --- 写回点标签 --- */
    std::memcpy(out->point_cluster, c.label, (size_t)n * sizeof(nf_u32));
    out->num_clusters = n_clusters;
    out->num_noise = n_noise;

    std::free(c.label); std::free(c.state);
    std::free(c.queue); std::free(c.nb);
    return NF_OK;
}

NFStatus nf_point_velocity(const NFOctreeCtx *tree_cur,
                           const NFOctreeCtx *tree_prev,
                           const nf_f32 R[9], const nf_f32 t[3],
                           nf_f32 dt_s, nf_f32 max_match_mm,
                           nf_f32 *out_speed, nf_u8 *out_valid) {
    if (!tree_cur || !tree_prev || !R || !t || !out_speed || !out_valid)
        return NF_E_PARAM_NULL;
    if (!(dt_s > 0.0f)) return NF_E_PARAM_RANGE;

    const nf_u32 n = nf_octree_count(tree_cur);
    nf_u32 idx[64];

    for (nf_u32 i = 0; i < n; ++i) {
        out_speed[i] = 0.0f;
        out_valid[i] = 0;

        nf_f32 p[3];
        nf_octree_point(tree_cur, i, p);

        /**
         * 本帧点 -> 上一帧坐标系。
         *
         * ## 约定（必须与调用方一致）
         * `R`/`t` 描述**自车从上一帧到本帧的运动**：
         *     P_cur = R·P_prev + t
         * 其中 P_prev / P_cur 都是各自时刻的**车体系坐标**。
         *
         * ## 反变换
         *     P_prev = Rᵀ·(P_cur - t)
         *
         * ## 手算验证（测试 velocity_* 就是按这个写的）
         * 墙在 x=3000（上帧，自车 x=0）。自车前进 500 后，
         * 本帧看到的墙在车体系 x=2500。R=I, t=(500,0,0)。
         *     P_prev = 2500 - 500 = 2000  ✗ 不等于 3000
         *
         * 所以**上式是我第一版的错误写法**。正确关系要看成：
         * 本帧车体系原点 = 上帧坐标系下的 (500,0,0)。
         * 同一物理点，本帧坐标 P_cur，则上帧坐标为
         *     P_prev = Rᵀ·P_cur + t
         *     3000 = 2500 + 500  ✓
         *
         * 差别在于 t 到底表示"上帧原点在本帧的位置"(+t) 还是
         * "本帧原点在上帧的位置"(-t)。接口选后者（自车位移），
         * 因为上游给的里程计就是"走了多远"，不是"退了多少"。
         */
        const nf_f32 q[3] = {
            R[0]*p[0] + R[3]*p[1] + R[6]*p[2] + t[0],
            R[1]*p[0] + R[4]*p[1] + R[7]*p[2] + t[1],
            R[2]*p[0] + R[5]*p[1] + R[8]*p[2] + t[2],
        };

        nf_u32 cnt = 0;
        nf_octree_radius(tree_prev, q[0], q[1], q[2], max_match_mm,
                         idx, 64u, &cnt, nullptr);
        if (cnt == 0) continue;

        /* 取最近的一个 */
        nf_f32 best = 1e30f;
        nf_f32 bp[3] = {0, 0, 0};
        const nf_u32 take = (cnt > 64u) ? 64u : cnt;
        for (nf_u32 k = 0; k < take; ++k) {
            nf_f32 pp[3];
            nf_octree_point(tree_prev, idx[k], pp);
            const nf_f32 dx = pp[0]-q[0], dy = pp[1]-q[1], dz = pp[2]-q[2];
            const nf_f32 d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < best) { best = d2; bp[0]=pp[0]; bp[1]=pp[1]; bp[2]=pp[2]; }
        }
        if (best > max_match_mm * max_match_mm) continue;

        /* 位移在**上一帧**坐标系里量，除以 dt 得速度 */
        const nf_f32 dx = q[0]-bp[0], dy = q[1]-bp[1], dz = q[2]-bp[2];
        out_speed[i] = std::sqrt(dx*dx + dy*dy + dz*dz) / dt_s;
        out_valid[i] = 1;
    }
    return NF_OK;
}

NFStatus nf_cluster_vote_motion(NFClusterResult *res, const nf_f32 *speed,
                                const nf_u8 *valid, nf_f32 static_mmps,
                                nf_f32 vote_ratio) {
    if (!res || !speed || !valid) return NF_E_PARAM_NULL;
    if (!(static_mmps >= 0.0f)) static_mmps = (nf_f32)NF_STATIC_SPEED_MMPS;
    if (!(vote_ratio > 0.0f)) vote_ratio = (nf_f32)NF_DYNAMIC_VOTE_RATIO;

    /* 每簇统计：有效点数、动态点数、速度累加 */
    const nf_u32 nc = res->num_clusters;
    nf_u32 *n_valid = (nf_u32 *)std::calloc(nc ? nc : 1, sizeof(nf_u32));
    nf_u32 *n_dyn   = (nf_u32 *)std::calloc(nc ? nc : 1, sizeof(nf_u32));
    nf_f32 *spd_sum = (nf_f32 *)std::calloc(nc ? nc : 1, sizeof(nf_f32));
    if (!n_valid || !n_dyn || !spd_sum) {
        std::free(n_valid); std::free(n_dyn); std::free(spd_sum);
        return NF_E_ENGINE_OOM;
    }

    for (nf_u32 i = 0; i < res->num_points; ++i) {
        const nf_u32 cid = res->point_cluster[i];
        if (cid == NF_CLUSTER_NOISE || cid >= nc) continue;
        if (!valid[i]) continue;
        ++n_valid[cid];
        spd_sum[cid] += speed[i];
        if (speed[i] > static_mmps) ++n_dyn[cid];
    }

    for (nf_u32 k = 0; k < nc; ++k) {
        if (n_valid[k] == 0) { res->clusters[k].is_dynamic = 0; continue; }
        const nf_f32 ratio = (nf_f32)n_dyn[k] / (nf_f32)n_valid[k];
        res->clusters[k].is_dynamic = (ratio >= vote_ratio) ? 2u : 1u;
        res->clusters[k].speed_mmps = spd_sum[k] / (nf_f32)n_valid[k];
    }

    std::free(n_valid); std::free(n_dyn); std::free(spd_sum);
    return NF_OK;
}

} /* extern "C" */
