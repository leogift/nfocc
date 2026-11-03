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
    /**
     * 空间不确定度 (cm)。1..254 有效，255 = 哨兵（不可知）。
     *
     * 由 `nf_compute_spatial_uncertainty_cm` 逐点算出，CMA 聚合时取
     * **最小**（观测越多越确定）。D4 交叉验证通过后可再减半。
     */
    nf_u8  sigma_cm;
    nf_u8  _pad_sigma[3];
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

/**
 * 带空间不确定度的插入（REV-001）。
 *
 * 与 `nf_voxel_insert` 的区别：额外携带 `sigma_cm`，CMA 聚合时按
 * **最小不确定度**聚合（不是平均）—— 多次观测里只要有一次特别准，
 * 该体素的位置就该按准的那次算。
 *
 * @param sigma_cm  [1,254]，传 255 表示不可知（内部按 254 处理）
 */
NFStatus nf_voxel_insert_with_sigma(NFVoxelCtx *ctx,
                                    nf_f32 x, nf_f32 y, nf_f32 z,
                                    nf_f32 conf, nf_u8 source_bit,
                                    nf_u8 sigma_cm);

/**
 * D4: 双基线交叉验证。
 *
 * 用另一条基线再算一次同一体素，得到 P2。与已有的 P1 比：
 *
 * ```
 * delta2 = |P1 - P2|^2
 * delta2 <= thresh_mm^2  ->  融合 P = (P1+P2)/2
 *                            sigma_cm <- max(1, round(sigma_cm / sqrt(2)))
 * delta2 >  thresh_mm^2  ->  判定为动态干扰/假阳性
 *                            n_eff <- n_eff * 0.2
 *                            n_eff < 0.2 时物理释放该体素
 * ```
 *
 * ## 为什么 sigma 除以 sqrt(2) 而不是 2
 * 两次独立等精度观测的信息融合：
 *   `1/sigma^2 = 1/sigma1^2 + 1/sigma2^2`
 * 等方差时 `sigma = sigma1/sqrt(2)`。
 * 原规范写 `sigma >>= 1`（除以 2）比统计最优更乐观，
 * 会低估不确定度，故此处用 sqrt(2)。
 *
 * @param ctx        体素表
 * @param x,y,z      第二基线的物点 (mm)
 * @param source_bit 来源相机位
 * @param thresh_mm  空间检验门限 (mm)，默认 50
 * @param out_verdict 可为 nullptr。0=超限剔除, 1=通过并融合
 */
NFStatus nf_voxel_verify_cross(NFVoxelCtx *ctx,
                               nf_f32 x, nf_f32 y, nf_f32 z,
                               nf_u8 source_bit, nf_f32 thresh_mm,
                               nf_u8 *out_verdict);

/**
 * D4 检验门限的**距离自适应**取值 (mm)。
 *
 * ## 为什么不能用固定 50mm
 * D4 拿两次观测的 3D 残差判一致性。但两次观测的精度不同：
 *
 *   - 长基线 b=500mm、短基线 b=278mm，深度误差比 ≈ 1.8
 *   - 2 m 处 σ 分别约 6cm 与 11cm
 *
 * 固定 50mm 门限时，短基线的正常误差（11cm）就已经超限，导致
 * **长基线的有效点被误剔除**。报告 §3.1 指出这条，是对的。
 *
 * ## 取法
 *   τ = max(τ_min, k · sqrt(σ1² + σ2²))
 *
 * 两项独立观测的残差标准差是 sqrt(σ1²+σ2²)，取 k=2 约覆盖 95%。
 * k=3 太松（近场几乎不剔除），k=1 太紧（远场全剔除）。
 *
 * @param z_mm     物点深度（取两次观测的平均）
 * @param f_px     视口焦距
 * @param b1_mm    第一次观测的基线
 * @param b2_mm    第二次观测的基线
 * @param tau_min_mm 下限，防止近场门限退化为 0
 * @return 门限 (mm)
 */
nf_f32 nf_verify_thresh_mm(nf_f32 z_mm, nf_f32 f_px,
                           nf_f32 b1_mm, nf_f32 b2_mm,
                           nf_f32 tau_min_mm);

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

/**
 * 体素表 -> **按柱体连续排布**的点云 + 高程压缩（REV-002）。
 *
 * ## 与 nf_voxel_export_points 的区别
 * 后者按哈希槽顺序导出，下游要查某个地表格子还得遍历。
 * 本函数保证：
 *   1. 点严格按 `col = iy*nx + ix` **递增**排列
 *   2. 同一 `col` 内按 `iz` **升序**排列
 *   3. `zindex[col]` / `zindex[col+1]` 正好是该柱点的起止下标
 *
 * ## O(1) 访问定理
 * 下游查地表网格 (ix, iy) 上的所有障碍点：
 * ```
 * col   = iy*nx + ix
 * start = zindex[col];
 * end   = zindex[col+1];
 * // 点就在 points[start .. end-1]，无需哈希查找
 * ```
 *
 * ## 前提（不满足则定理失效）
 * 输出顺序必须与 zindex 的构建顺序**完全一致**。
 * 本函数内部先建 zindex 再按 col 递增填点，故自洽。
 *
 * @param ctx  体素表
 * @param out  已分配 zmap/zindex 的 NFGridOutput（n_cells 必须 = nx*ny）
 * @param pc   已分配的点云缓冲；不够时按存在概率截断
 */
NFStatus nf_voxel_transform_columnar(const NFVoxelCtx *ctx,
                                     NFGridOutput *out,
                                     NFPointCloud *pc);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_VOXEL_H */
