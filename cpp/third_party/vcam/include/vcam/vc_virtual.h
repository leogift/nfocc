/**
 * @file vc_virtual.h
 * @brief 虚拟相机坐标映射网格构建 (含 OpenMP 并行与双线性采样表)。
 *
 * 链路 (一个虚拟像素怎么找到源像素):
 *   虚拟像素 (u,v)
 *     --虚拟相机 unproject-->  虚拟相机系射线 ray_dst
 *     --R_rel = R_src @ R_dst^T-->  源相机系射线 ray_src
 *     --源相机 project-->  源像素 (u_src, v_src)
 *
 * ## 并行策略
 *   按**行块**并行 (默认块高 = 16 行), 每块独立写自己的 map 区间, 无竞争。
 *   为什么不用 `#pragma omp parallel for` 直接按行: 每行才 640 像素、几十个
 *   浮点运算, 行级调度的 fork/join 开销会吃掉收益。按块并行可摊薄到
 *   640*16 = 10240 像素。
 *
 * ## 为什么统计要分开记
 *   无效像素有四种互不相同的原因, 混成一个数字就没法排查:
 *     out_of_source  -- 落在源图外 (正常: 虚拟 FOV 超出了源覆盖)
 *     unconverged    -- 反投影没收敛 (标定参数有问题)
 *     non_finite     -- NaN/Inf (数值溢出)
 *     (其余)         -- 正常
 */
#ifndef VCAM_VIRTUAL_H
#define VCAM_VIRTUAL_H

#include "vcam/vc_camera.h"
#include "vcam/vc_rotation.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 相机位姿变换 (纯旋转; 虚拟相机几何不需要平移)。 */
typedef struct {
    vc_u32 version; /**< = VC_POSE_VERSION */
    vc_f64 R[9];    /**< world->cam */
    VCModelKind _unused_kind;
} VCPose;

typedef struct {
    vc_u32 version; /**< = VC_VIRTUAL_CONVERSION_VERSION */
    VCCameraModel src;
    VCCameraModel dst;
    vc_f64 R_rel[9]; /**< v_src = R_rel @ v_dst */
} VCVirtualConversion;

/** 一次网格构建的统计。用于告警与调试。 */
typedef struct {
    vc_u32 version; /**< = VC_MAP_STATS_VERSION */
    vc_i32 width;   /**< 目标宽 */
    vc_i32 height;
    vc_i32 total;
    vc_i32 valid;
    vc_i32 out_of_source;
    vc_i32 unconverged;
    vc_i32 non_finite;
    vc_f64 max_theta_deg;
    char note[192];
} VCMapStats;

/** 双线性采样表: 4 个 tap 的扁平索引与权重。布局 (4, H*W)。 */
typedef struct {
    vc_i32 *idx; /**< 长度 4*H*W */
    vc_f32 *w;   /**< 长度 4*H*W */
    vc_i32 count;
} VCSampleTable;

/**
 * 建立一对相机的转换: 源相机 + 目标相机 + 姿态相对增量。
 * @param yaw_delta_deg   相对源相机的偏航增量, 正值 = 向左看
 * @param pitch_delta_deg 正值 = 抬头
 */
VCStatus vc_conv_init(VCVirtualConversion *out, const VCCameraModel *src,
                      const VCCameraModel *dst, vc_f64 yaw_delta_deg,
                      vc_f64 pitch_delta_deg, vc_f64 roll_delta_deg,
                      VCConventionKind conv);

/** 直接给定相对旋转矩阵 (跳过姿态计算, 供离线烘焙用)。 */
VCStatus vc_conv_init_from_R(VCVirtualConversion *out, const VCCameraModel *src,
                             const VCCameraModel *dst, const vc_f64 R_rel[9]);

/**
 * 构建 map_x / map_y。
 * @param map_x 长度 >= dst.width*dst.height, 行优先
 * @param stats 可为 NULL
 * @param chunk_rows 并行块高, <=0 用默认 16
 */
VCStatus vc_conv_build_maps(const VCVirtualConversion *conv, vc_f32 *map_x,
                            vc_f32 *map_y, VCMapStats *stats, vc_i32 chunk_rows);

/**
 * 只算一行 (调试与逐行流式处理用, 不涉及 OpenMP)。
 */
VCStatus vc_conv_build_rows(const VCVirtualConversion *conv, vc_i32 row_begin,
                            vc_i32 row_end, vc_f32 *map_x, vc_f32 *map_y,
                            vc_i32 map_stride);

/**
 * 生成双线性采样表 (边缘 NPU 上 Gather 比 GridSample 好部署)。
 * tap 顺序 [x0y0, x1y0, x0y1, x1y1]; 越界 tap 权重置 0 (等价 padding=zeros)。
 * @param alloc 由调用方提供的内存, idx 需 4*H*W 个 vc_i32, w 同长度 vc_f32
 */
VCStatus vc_conv_build_tables(const VCVirtualConversion *conv, const vc_f32 *map_x,
                              const vc_f32 *map_y, VCSampleTable *tables);

/**
 * 用采样表对一张 8 位灰度/彩色图做双线性重采样 (无需 OpenCV)。
 * @param img_channels 1 或 3
 */
VCStatus vc_sample_bilinear(const vc_u8 *src, vc_i32 src_w, vc_i32 src_h,
                            vc_i32 img_channels, const VCSampleTable *tables,
                            vc_u8 *dst, vc_i32 dst_w, vc_i32 dst_h);

/** 返回 1 表示本库编译时启用了 OpenMP。无参数、无副作用，可在任意线程调用。 */
int vc_have_openmp(void);

#ifdef __cplusplus
}
#endif

#endif /* VCAM_VIRTUAL_H */
