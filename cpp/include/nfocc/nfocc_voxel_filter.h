/**
 * @file nfocc_voxel_filter.h
 * @brief 稠密体素精简与下游输出压缩（REV-003，NEW_MOD §5）。
 *
 * ## 为什么需要这一层
 * 近场稠密重建每帧可产 10 万~50 万点，直接送下游有三个硬问题：
 *
 * 1. **接口配额**：下游 SERL 标准的上限是 15000 点
 * 2. **冗余**：实测 70~80% 是平坦路面反射点
 * 3. **实心内部**：墙、车体内部填满体素，碰撞规划只需外壳
 *
 * ## 五阶段流水线（全部整数/位运算，无除法无三角函数）
 *
 *   阶段 1  AABB 裁剪 + FoE 锥代数过滤     点积判别，无开方
 *   阶段 2  2D 柱体高程位图投影            每 (X,Y) 一个 uint32 记 32 层
 *   阶段 3  位运算地面剥离                 CTZ32 找最低层，掩码清除
 *   阶段 4  极坐标首击表面外壳提取         360 扇区深度缓存 + shell 厚度
 *   阶段 5  配额熔断 + 两遍计数排序        按 cell 升序连续排布
 *
 * ## 为什么阶段 4 要从原始点数组读坐标
 * 阶段 2 的位图只记"某层有没有点"，**不记精确坐标**。阶段 4 要算
 * sqrt(x²+y²) 必须回到原始点。所以实现上保留点数组，位图只作索引 ——
 * 这是评审 §5.1 指出的坑，写在这里防止后来者"优化掉"点数组。
 */
#ifndef NFOCC_VOXEL_FILTER_H
#define NFOCC_VOXEL_FILTER_H

#include "nfocc/nfocc_types.h"
#include "nfocc/nfocc_voxel.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 下游接口硬上限（SERL 标准） */
#define NF_DVF_HARD_CAP_MAX_POINTS  15000u

/** 默认参数 */
#define NF_DVF_CELL_MM              200.0f  /**< 底面网格分辨率 */
#define NF_DVF_Z_BIN_MM             100.0f  /**< 垂直分层厚度（32 层 = 3.2m） */
#define NF_DVF_ROI_X_MIN          (-5000.0f)
#define NF_DVF_ROI_X_MAX           10000.0f
#define NF_DVF_ROI_Y_MIN          (-6000.0f)
#define NF_DVF_ROI_Y_MAX            6000.0f
#define NF_DVF_ROI_Z_MIN          (-1000.0f)
#define NF_DVF_ROI_Z_MAX            2500.0f
#define NF_DVF_FOE_COS2             0.933f  /**< cos²(15°) */
#define NF_DVF_GROUND_MAX_Z_BIN     2u      /**< 最低层 <= 2 才考虑当地面 */
#define NF_DVF_GROUND_CONT_BINS     2u      /**< 连续层数 */
#define NF_DVF_POLAR_BINS           720u    /**< 极坐标扇区数（0.5°/扇区） */
#define NF_DVF_SHELL_CM             30.0f   /**< 外壳厚度 */

/**
 * 下游交付点（10 字节，对齐下游 SERL 结构体）。
 *
 * ## 为什么是 10 字节
 * 下游接口按这个尺寸预分配缓冲。改大下游要重新编译，改小会浪费带宽。
 * 用 packed 明确告诉编译器不要插入填充 —— 否则 64 位对齐会变成 12 字节，
 * 下游按 10 字节解析时字段全错位。
 */
typedef struct __attribute__((packed)) {
    nf_i16 x_mm;    /**< X 坐标，mm（ROI 内 ±10m 需 i32？见下方说明） */
    nf_i16 y_mm;
    nf_i16 z_mm;
    nf_u8  source_camera;           /**< bit0=FV .. bit3=MR */
    nf_u8  spatial_uncertainty_cm;  /**< 0-254, 255=无效 */
    nf_u8  existence_probability;   /**< 0-255 映射 0..1 */
    nf_u8  surface_flags;           /**< 见 NF_DVF_SURF_* */
} NFDownstreamPoint;

/**
 * `surface_flags` 位定义（评审 #12 指出原文档缺失）。
 *
 * 下游用这几位判断"这个点该当什么处理"：地面点参与可行驶区域，
 * 顶面点不参与碰撞，外墙点才参与。
 */
#define NF_DVF_SURF_GROUND   0x01u  /**< 地面层剥离后的残留（少见） */
#define NF_DVF_SURF_TOP      0x02u  /**< 柱体最高层 = 物体顶面 */
#define NF_DVF_SURF_OUTER    0x04u  /**< 极坐标首击保留 = 朝自车的外壳 */
#define NF_DVF_SURF_NORMAL   0x08u  /**< 普通障碍物点 */
#define NF_DVF_SURF_DYNAMIC  0x10u  /**< 动态（n_eff 低或来源单一） */

/** 过滤器配置 */
typedef struct {
    nf_u32 version;
    nf_f32 cell_mm;             /**< 0 -> NF_DVF_CELL_MM */
    nf_f32 z_bin_mm;            /**< 0 -> NF_DVF_Z_BIN_MM */
    nf_f32 roi_x_min, roi_x_max;
    nf_f32 roi_y_min, roi_y_max;
    nf_f32 roi_z_min, roi_z_max;
    nf_f32 shell_thickness_cm;  /**< 0 -> NF_DVF_SHELL_CM */
    nf_u32 max_output_points;   /**< 0 -> 硬上限 15000；超过硬上限被钳制 */
    nf_u32 ground_max_z_bin;    /**< 0 -> 2 */
    nf_u32 ground_cont_bins;    /**< 0 -> 2 */
    nf_u32 polar_bins;          /**< 0 -> 720 */
    nf_u8  enable_ground_strip; /**< 1=剥离地面 */
    nf_u8  enable_polar_shell;  /**< 1=只留外壳 */
    nf_u8  enable_foe_filter;   /**< 1=剔除 FoE 锥内退化点 */
    nf_u8  _pad;
} NFVoxelFilterConfig;

typedef struct NFVoxelFilterCtx NFVoxelFilterCtx;

/** 单次压缩的统计，供诊断 */
typedef struct {
    nf_u32 n_input;         /**< 输入点数 */
    nf_u32 n_after_roi;     /**< 阶段 1 AABB 后 */
    nf_u32 n_after_foe;     /**< 阶段 1 FoE 后 */
    nf_u32 n_after_ground;  /**< 阶段 3 后 */
    nf_u32 n_after_shell;   /**< 阶段 4 后 */
    nf_u32 n_output;        /**< 阶段 5 后（<= max_output） */
    nf_u32 n_cells;         /**< 占用的柱体数 */
    nf_u8  truncated;       /**< 1=被配额截断 */
} NFVoxelFilterStats;

NFStatus nf_voxel_filter_create(NFVoxelFilterCtx **ctx,
                                const NFVoxelFilterConfig *cfg);
void     nf_voxel_filter_destroy(NFVoxelFilterCtx *ctx);
NFStatus nf_voxel_filter_default_config(NFVoxelFilterConfig *cfg);

/**
 * 更新自车运动方向（车体系），供阶段 1 的 FoE 锥判别。
 *
 * @param dir_x, dir_y  运动方向单位化后的分量；传 (0,0) 表示静止，
 *                      此时不做 FoE 过滤（静止没有退化方向）
 */
NFStatus nf_voxel_filter_update_ego_motion(NFVoxelFilterCtx *ctx,
                                           nf_f32 dir_x, nf_f32 dir_y);

/**
 * 执行五阶段压缩。
 *
 * @param ctx       过滤器
 * @param points    输入点（车系 mm + 属性），会被读但**不被修改**
 * @param n_points  输入点数
 * @param zmap_out  [n_cells] 输出位图，调用方预分配
 * @param zindex_out[n_cells+1] 输出前缀和，调用方预分配
 * @param pts_out   [max_output] 输出点，调用方预分配
 * @param stats     可为 nullptr
 *
 * @note 预分配尺寸用 nf_voxel_filter_grid_size() 查。
 */
NFStatus nf_voxel_filter_compress(NFVoxelFilterCtx *ctx,
                                  const NFVoxelEntry *points, nf_u32 n_points,
                                  nf_u32 *zmap_out, nf_u32 *zindex_out,
                                  NFDownstreamPoint *pts_out,
                                  NFVoxelFilterStats *stats);

/** 查询网格尺寸：n_cells = nx*ny */
NFStatus nf_voxel_filter_grid_size(const NFVoxelFilterCtx *ctx,
                                   nf_u32 *nx, nf_u32 *ny, nf_u32 *n_cells);

/**
 * 把内部结果转成下游紧凑结构（阶段 5 的排布已在 compress 内完成，
 * 这里只做字段映射，便于下游单独调用）。
 */
NFStatus nf_voxel_filter_to_downstream(const NFVoxelFilterCtx *ctx,
                                       const NFDownstreamPoint *src,
                                       nf_u32 n,
                                       void *dst, nf_u32 dst_bytes,
                                       nf_u32 *out_bytes);

#ifdef __cplusplus
}
#endif
#endif /* NFOCC_VOXEL_FILTER_H */
