/**
 * @file nfocc_cluster.h
 * @brief DBSCAN 聚类 + 动静分离（基于八叉树邻域查询）。
 *
 * ## 为什么是 DBSCAN 而不是体素连通域
 * 体素连通域只需 O(N)，但它**无法拒绝噪声** —— 稀疏的地面点、
 * 远处的离散伪点都会各自成一簇，输出一堆 1-2 点的"目标"。
 *
 * DBSCAN 的核心点判据（邻域内至少 min_pts 个点）天然把这些过滤掉。
 * 代价是需要邻域查询，所以配八叉树。
 *
 * ## 150k 点下的复杂度
 *   DBSCAN 本身 O(N * 邻域查询)
 *   邻域查询经八叉树是 O(球内点数)，不是 O(N)
 *   实测叶 200mm、4% 填充时，3x3x3 邻域约 27 叶 x 9 点 = 243 次比较
 *   总查询次数 150000 x 243 ≈ 3.6e7，约 100 ms（估算，需实测）
 *
 * ## 动静分离的两层
 *   1. **点级**：把本帧点变换到上一帧坐标系，找最近邻，位移/Δt 即速度。
 *      依赖位姿 —— 位姿不准会把静止背景全判成运动。
 *   2. **簇级投票**：一簇内动态点占比超过阈值则整簇判动态。
 *      抵消点级速度的随机误差，且输出的是目标级结论。
 */
#ifndef NFOCC_CLUSTER_H
#define NFOCC_CLUSTER_H

#include "nfocc/nfocc_octree.h"
#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_CLUSTER_VERSION  1u

/** 噪声标记（DBSCAN 的标准约定） */
#define NF_CLUSTER_NOISE    0xFFFFFFFFu
/** 尚未分类 */
#define NF_CLUSTER_UNVISITED 0xFFFFFFFEu

/** 聚类参数 */
typedef struct {
    nf_u32 version;         /**< = NF_CLUSTER_VERSION */
    nf_f32 eps_mm;          /**< 邻域半径，0 = 用 NF_DBSCAN_EPS_MM */
    nf_u32 min_pts;         /**< 核心点最少邻域点数（含自身），0 = 用默认 */
    nf_u32 max_clusters;    /**< 上限，超出部分标噪声 */
} NFClusterConfig;

/** 单个聚类结果 */
typedef struct {
    nf_u32 id;
    nf_u32 n_points;
    /** 包围盒（车系 mm） */
    nf_f32 min[3];
    nf_f32 max[3];
    /** 质心 */
    nf_f32 centroid[3];
    /** 包围盒体积占满率：点数 / (盒内体素数)。贴墙物体该值低，独立物体高 */
    nf_f32 fill_ratio;
    /** 动静标记：0=未知 1=静止 2=动态 */
    nf_u8  is_dynamic;
    nf_u8  _pad[3];
    /** 平均速度 (mm/s)，仅 is_dynamic 有意义 */
    nf_f32 speed_mmps;
} NFCluster;

/** 聚类输出 */
typedef struct {
    nf_u32 version;
    NFCluster *clusters;       /**< [max_clusters] */
    nf_u32     num_clusters;
    nf_u32    *point_cluster;  /**< [n] 每点所属簇 id，噪声为 NF_CLUSTER_NOISE */
    nf_u32     num_points;
    nf_u32     num_noise;      /**< 判为噪声的点数 */
} NFClusterResult;

NFStatus nf_cluster_result_alloc(NFClusterResult *r, nf_u32 max_clusters,
                                 nf_u32 max_points);
void     nf_cluster_result_destroy(NFClusterResult *r);

/**
 * DBSCAN 聚类。
 *
 * @param tree   已构建的八叉树（持有全部点）
 * @param cfg    参数，NULL 用默认
 * @param out    输出（已 alloc）
 *
 * @return NF_OK / NF_E_PARAM_*
 */
NFStatus nf_dbscan(const NFOctreeCtx *tree, const NFClusterConfig *cfg,
                   NFClusterResult *out);

/**
 * 点级速度估计：把本帧点变换到 prev 坐标系后找最近邻。
 *
 * ## 为什么变换而不是直接比
 * 自车在动。若直接比两帧坐标，静止的墙也会有 -- 自车速度 那么大的位移。
 * 把本帧点乘 R^T、减去 t 变换到上一帧车体系，静止物才落在同一位置。
 *
 * @param tree_cur   本帧点的八叉树
 * @param tree_prev  上一帧点的八叉树
 * @param R          上一帧 -> 本帧的旋转（行优先 3x3，v_cur = R·v_prev）
 * @param t          上一帧 -> 本帧的平移 (mm)
 * @param dt_s       帧间隔（秒）
 * @param max_match_mm 最近邻匹配的最大距离，超过则视为无匹配（速度 0）
 * @param out_speed  [n] 输出速度大小 (mm/s)
 * @param out_valid  [n] 1 = 匹配成功，0 = 无匹配
 */
NFStatus nf_point_velocity(const NFOctreeCtx *tree_cur,
                           const NFOctreeCtx *tree_prev,
                           const nf_f32 R[9], const nf_f32 t[3],
                           nf_f32 dt_s, nf_f32 max_match_mm,
                           nf_f32 *out_speed, nf_u8 *out_valid);

/**
 * 簇级动静投票：把点级速度聚合到簇级。
 *
 * @param res        已做完 DBSCAN 的结果
 * @param speed      [n] 点级速度
 * @param valid      [n] 点级有效标记
 * @param static_mmps 判静止的速度门限
 * @param vote_ratio  动态点占比阈值
 */
NFStatus nf_cluster_vote_motion(NFClusterResult *res, const nf_f32 *speed,
                                const nf_u8 *valid, nf_f32 static_mmps,
                                nf_f32 vote_ratio);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_CLUSTER_H */
