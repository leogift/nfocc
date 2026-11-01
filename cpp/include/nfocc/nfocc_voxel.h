/**
 * @file nfocc_voxel.h
 * @brief 体素哈希、CMA 融合、zMap/zIndex 输出。规范: docs/SPEC-VOXEL.md
 *
 * ## 索引约定 (唯一, 不可另立)
 *   ix = (X_MAX − X_V)/vox    **X 反向**
 *   iy = (Y_MAX − Y_V)/vox    **Y 反向**
 *   iz = (Z_V − Z_MIN)/vox    Z 正向
 * 反向的原因: 与原始 NFOCC 的 Hash 位序一致。反向不一致时多帧融合会把
 * 同一物理点放进不同体素, 且不易察觉。
 *
 * ## 柱体编号
 *   cell = iy * nx + ix      **不是 ix*ny+iy**
 * 本项目 nx=75, ny=60, 用错会让高程图转置, 方阵测试测不出来。
 *
 * ## zMap
 * uint32, bit k = 第 k 层存在。nz=18 时只用 bit0..bit17,
 * **bit18 及以上必须为 0**, bit31 恒 0 (V-VOX-3)。
 * Z 范围 3500/200 = 17.5 -> 18 层, **末格是 100mm 半格**, 必须显式标记。
 */
#ifndef NFOCC_VOXEL_H
#define NFOCC_VOXEL_H

#include "nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 体素网格参数 (SPEC-ICD §6.1) */
typedef struct {
    nf_u32 version;                    /**< = NF_VOXEL_GRID_VERSION */
    nf_i32 vox_mm;
    nf_i32 x_min, x_max;
    nf_i32 y_min, y_max;
    nf_i32 z_min, z_max;
    nf_u32 nx, ny, nz;                 /**< 必须与上面自洽 */
} NFVoxelGridParams;

/** 由范围推导 nx/ny/nz (向上取整) 并校验自洽 */
NFStatus nf_voxel_grid_init(NFVoxelGridParams *g, nf_i32 vox_mm,
                            nf_i32 x_min, nf_i32 x_max,
                            nf_i32 y_min, nf_i32 y_max,
                            nf_i32 z_min, nf_i32 z_max);

/** 最后一格是否为半格(不足 vox)。1=半格 */
NFStatus nf_voxel_last_cell_partial(const NFVoxelGridParams *g,
                                    nf_i32 *span_x, nf_i32 *span_y, nf_i32 *span_z);

/** 点 -> 索引(单个)。inside=0 表示越界(**不钳位** —— 钳位会在边缘造出假墙) */
NFStatus nf_voxel_index(const NFVoxelGridParams *g, nf_f32 x, nf_f32 y, nf_f32 z,
                        nf_i32 *ix, nf_i32 *iy, nf_i32 *iz, nf_u8 *inside);

/** 索引 -> 体素中心 */
NFStatus nf_voxel_center(const NFVoxelGridParams *g, nf_i32 ix, nf_i32 iy, nf_i32 iz,
                         nf_f32 *x, nf_f32 *y, nf_f32 *z);

/** Hash 打包 (ix<<16)|(iy<<8)|iz。索引超出 8 位时返回 NF_E_PARAM_RANGE */
NFStatus nf_voxel_pack_hash(const nf_i32 *ix, const nf_i32 *iy, const nf_i32 *iz,
                            nf_u32 n, nf_u32 *hash);

/** 体素表条目: 质心 + CMA 计数 + 来源位图 */
typedef struct {
    nf_u32 hash;
    nf_i32 ix, iy, iz;
    nf_f32 cx, cy, cz;      /**< 质心, mm */
    nf_f32 n_eff;           /**< 有效观测计数(置信度加权) */
    nf_u8  source_mask;     /**< bit0=FV .. bit3=MR */
    nf_u8  used;
} NFVoxelEntry;

/** 体素表上下文(不透明) */
typedef struct NFVoxelCtx NFVoxelCtx;

/**
 * 创建体素表。
 * @param capacity  槽数(开放寻址, 负载因子上限由实现保证)
 * @param n_eff_max CMA 有效观测数饱和上限 (V_N_EFF_MAX)
 */
NFStatus nf_voxel_create(NFVoxelCtx **ctx, const NFVoxelGridParams *g,
                         nf_u32 capacity, nf_f32 n_eff_max);
void     nf_voxel_destroy(NFVoxelCtx *ctx);
/** 清空(不释放内存), 用于每帧重置 */
void     nf_voxel_clear(NFVoxelCtx *ctx);

/**
 * 插入一个点(加权 CMA 融合)。
 *
 *   c_n = c_{n-1} + w·(p − c_{n-1}) / (n_eff + w)
 *   n_eff += w
 * 计数饱和: n_eff > n_eff_max 时衰减一半, 防止旧观测永久压制新观测。
 * 质心钳位在体素内 ±vox/2 (V-VOX-2: 同点重复插入漂移 < 1mm)。
 *
 * @param conf 置信度 0..1, 作为权重 w
 * @param sigma_mm 空间不确定度, 用于输出
 * @return NF_E_VOXEL_FULL 表示表满
 */
NFStatus nf_voxel_insert(NFVoxelCtx *ctx, nf_f32 x, nf_f32 y, nf_f32 z,
                         nf_f32 conf, nf_u8 source_bit);

/** 体素表统计 */
NFStatus nf_voxel_stats(const NFVoxelCtx *ctx, nf_u32 *num_entries,
                        nf_u32 *num_slots, nf_u32 *collisions);

/** 2D 高程柱输出 (SPEC-ICD §6.4) */
typedef struct {
    nf_u32 version;
    nf_u32 n_cells;
    nf_u32 *zmap;      /**< [n_cells], bit k = 第 k 层存在 */
    nf_u32 *zindex;    /**< [n_cells+1], 前缀和 */
} NFGridOutput;

/**
 * 压平为 zMap + zIndex。体素表 -> 高程柱。
 * **bit31 恒 0, bit18+ 必须为 0**(nz=18 时)。
 */
NFStatus nf_grid_flatten(const NFVoxelCtx *ctx, NFGridOutput *out);

/** 释放 NFGridOutput 内部缓冲 */
void nf_grid_output_destroy(NFGridOutput *out);

/** 点云输出 (SPEC-ICD §6.2/§6.3) */
typedef struct {
    nf_u32 version;
    NFPoint *points;
    nf_u32  num_points;
    nf_u32  max_points;
    nf_u8   overflow;      /**< 1=因超限被截断 (NF_E_OUTPUT_OVERFLOW) */
} NFPointCloud;

NFStatus nf_pointcloud_alloc(NFPointCloud *pc, nf_u32 max_points);
void     nf_pointcloud_destroy(NFPointCloud *pc);

/** 体素表 -> 点云。超 max_points 时按存在概率截断 */
NFStatus nf_voxel_export_points(const NFVoxelCtx *ctx, NFPointCloud *pc);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_VOXEL_H */
