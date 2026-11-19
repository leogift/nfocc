/**
 * @file nfocc_octree.h
 * @brief 空间索引：**纯网格哈希**（无树、无指针），供 DBSCAN 邻域查询。
 *
 * ## 名字的由来
 * 对外仍叫 octree（调用方按八叉树的语义使用：八等分空间、按格定位），
 * 但实现上**不建树**。原因见下面的迭代记录。
 *
 * ## 六次迭代的性能与失败（150k 点、eps=120mm 实测）
 *
 * | 版本 | 结构 | DBSCAN | 失败原因 |
 * |---|---|---|---|
 * | v1 | 固定叶 200mm + 开放寻址 | 857 ms | 稠密区单叶 586 点，叶内退化 |
 * | v2 | 自适应细分 + 全遍历叶表 | 3281 ms | 25325 叶全扫 |
 * | v3 | 自适应细分 + x 方向二分 | 417 ms | 候选区间仍宽 |
 * | v4 | 自适应细分 + 逐层哈希 | 4058 ms | 每层每点扫 2197 格 x 10 层 |
 * | v5 | 自适应细分 + 单层哈希 | 查询返回 0 | **叶跨多个深度，粗叶无索引** |
 * | v6 | 统一深度 + 单层哈希 | 段错误 | CSR/节点编号/哈希三者不一致 |
 * | **v7** | **纯网格哈希（本文件）** | 见测试 | **无树结构，没有可摇摆的地方** |
 *
 * ## 为什么最终放弃树
 * 前六版的失败都有一个共同点：**多个数据结构要维护一致**（树的父子链、
 * 节点的 CSR 区间、哈希的叶编号）。任何一处不同步就表现为
 * "查不到"或"段错误"，而且症状离根因很远。
 *
 * 纯网格哈希只有一个结构：`(ix,iy,iz) -> 点列表`。
 * 没有父子、没有层、没有节点编号，逻辑上不可能不一致。
 *
 * ## 代价与取舍
 * 固定格边长，所以稠密区单格点数多。**这不算问题** —— 格边长按
 * "期望每格点数"反正推即可：格 = 8mm 时，150k 点在 6.75 m³ 里
 * 平均每格点数极少；实际用 40mm（`NF_HASH_CELL_MM`）时每格约 1-2 点。
 *
 * 查询时扫 `(2*ceil(r/cell)+1)³` 个格。r=120mm、cell=40mm ->
 * span=3 -> 343 格，每格 1-2 点 -> 约 600 次比较。
 * 150k 次查询 = 9e7 次比较 ≈ 90 ms。这是可以达到的量级。
 *
 * ## 核心库约束
 * 禁 STL 容器与异常，缓冲用 malloc。
 */
#ifndef NFOCC_OCTREE_H
#define NFOCC_OCTREE_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_OCTREE_VERSION  2u

/** 空间索引句柄（不透明） */
typedef struct NFOctreeCtx NFOctreeCtx;

/** 构建参数 */
typedef struct {
    nf_u32 version;         /**< = NF_OCTREE_VERSION */
    nf_f32 cell_mm;         /**< 格边长，0 = 用 NF_HASH_CELL_MM */
} NFOctreeConfig;

/**
 * 由点集构建索引。
 *
 * @param ctx  输出句柄
 * @param pts  点坐标 [n*3]，mm
 * @param n    点数
 * @param cfg  参数，NULL 用默认
 *
 * @note `pts` 会被拷贝，构建后调用方可释放原数组。
 */
NFStatus nf_octree_build(NFOctreeCtx **ctx, const nf_f32 *pts, nf_u32 n,
                         const NFOctreeConfig *cfg);

void nf_octree_destroy(NFOctreeCtx *ctx);

/**
 * 查询以 (x,y,z) 为球心、radius 为半径的球内所有点索引。
 *
 * @param out_idx        输出点索引数组
 * @param capacity       out_idx 容量
 * @param out_count      实际命中数（可能 > capacity，此时 *out_truncated=1）
 * @param out_truncated  可为 NULL
 */
NFStatus nf_octree_radius(const NFOctreeCtx *ctx, nf_f32 x, nf_f32 y, nf_f32 z,
                          nf_f32 radius, nf_u32 *out_idx, nf_u32 capacity,
                          nf_u32 *out_count, nf_u8 *out_truncated);

/**
 * 统计：非空格数、单格最多点数、平均点数/格、格边长。
 * 用于验证 `cell_mm` 选得合不合适（单格点数应远小于 8）。
 */
NFStatus nf_octree_stats(const NFOctreeCtx *ctx, nf_u32 *out_cells,
                         nf_u32 *out_max_cell, nf_f32 *out_avg_cell,
                         nf_f32 *out_cell_mm);

/** 点索引 -> 坐标 */
NFStatus nf_octree_point(const NFOctreeCtx *ctx, nf_u32 idx, nf_f32 out[3]);

/** 总点数 */
nf_u32 nf_octree_count(const NFOctreeCtx *ctx);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_OCTREE_H */
