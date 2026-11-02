/**
 * @file nfocc_pipeline.h
 * @brief Stream pipeline 编排。规范: docs/ARCH-DECISIONS.md ADR-004/ADR-005
 *
 * ## 分级结构
 *   Source -> SyncStage -> BaselineStage -> (按相机) Warp/Detect/Tri -> Fusion -> Sink
 *
 * ## 关键设计 (ADR-005)
 * **Stage 是同步的, pipeline 才决定并发。**
 * 理由: V-ENG-2 要求同输入两次调用逐位相同。Stage 内部自带线程池会让确定性
 * 无法保证, 录包重放会漂。并发只在 Pipeline 层按相机维度开。
 *
 * ## 冻结语义 (ADR-003)
 * 冻结时输出 `points=NULL, num_points=0` 且带 NF_E_BASELINE_*,
 * **不是输出空点云** —— 下游必须能区分"没结果"与"没障碍"。
 */
#ifndef NFOCC_PIPELINE_H
#define NFOCC_PIPELINE_H

#include "nfocc_types.h"
#include "nfocc_camera.h"
#include "nfocc_engine.h"
#include "nfocc_geometry.h"
#include "nfocc_lut.h"
#include "nfocc_voxel.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_NUM_CAMERAS 4

/** 单帧处理配置 */
typedef struct {
    nf_u32 version;
    NFEngineConfig   engine;        /**< 引擎配置(按相机各一份) */
    NFViewParams     view;          /**< 视口参数(4 路共用) */
    NFVoxelGridParams grid;         /**< 体素网格 */
    nf_f32 b_min_mm, b_max_mm;
    nf_f32 sigma_d_px;              /**< 视差匹配标准差 (T_SIGMA_D) */
    nf_f32 sigma_b_ratio;           /**< 基线相对不确定度 (T_SIGMA_B) */
    nf_u32 gap_max_frames;
    nf_u32 max_points;
    nf_u32 thread_cameras;          /**< 1=串行(默认), 4=并行 */

    /**
     * 每相机虚拟视口的**车系绝对 yaw**（deg，0=+X 前，逆时针正）。
     * 与 `nfocc_viewplan` 的规划表对应：ML +90 / MR −90 / FV ±45 / RV ±135。
     *
     * ## 为什么按相机给而不是按视口给
     * 现有实现是"每相机一套缓冲 + 一套 LUT"(索引即 cam_id)。
     * 改成"每视口一套"等于重写整个 buffer 管理, 风险大且当前
     * SGBM 内存已经吃满。这里先支持**每相机一个主偏转角**,
     * 即把该相机的视口整体偏过去 —— 对 FV/RV 就能拿到横向基线。
     *
     * 需要同一相机同时产出 0°/+30° 两个视口时, 由上层建两个 pipeline
     * 实例(内存 ×2)或等后续的视口池重构。见 docs/SPEC-VIEWPLAN.md §5。
     */
    nf_f32 view_yaw_deg[NF_NUM_CAMERAS];

    /** 视口水平张角 (deg), 0 表示用 NF_V_VIEW_HFOV_DEG */
    nf_f32 view_hfov_deg;

    /**
     * D1 车身掩码 [view_w * view_h]，1 = 车身遮挡，NULL = 不启用。
     *
     * ## 为什么在配置里而不是每帧输入里
     * LUT 是 `nf_pipeline_create` 一次性构建的（相机内外参不变），
     * 掩码决定 LUT 的 `valid` 位，也属于**静态几何**，因此必须是
     * 创建期输入。放在帧输入里会让人误以为可以逐帧换。
     *
     * 由 `nf_mask_rasterize_polygon` 从车体轮廓多边形生成。
     */
    const nf_u8 *body_mask;
} NFPipelineConfig;

/** 单帧输入 */
typedef struct {
    const nf_u8 *img[NF_NUM_CAMERAS];   /**< 各相机原图(当前帧) [w*h] 单通道 */
    /** 参考帧原图(上一产出帧)。侧向立体需要两帧; 为 NULL 时退化为单帧
     *  (SGBM 输出全无效, 这是正确行为而非缺陷)。 */
    const nf_u8 *img_ref[NF_NUM_CAMERAS];
    nf_u32 width, height, stride;
    nf_u64 timestamp_us;
    nf_u32 frame_id;
    const nf_u8 *body_mask;             /**< 车身掩码 [w*h], 1=车身; NULL 不启用 */
    /** 各相机相对参考帧的平移(相机系, mm)。
     *  参考帧位姿是"上一产出帧", 由 pipeline 内部维护。 */
    nf_f32 t_cam[NF_NUM_CAMERAS][3];
    nf_u32 frame_gap;                   /**< 距参考帧的帧间隔 */
} NFFrameInput;

/** 单帧输出 */
typedef struct {
    nf_u32 version;
    NFMode  mode[NF_NUM_CAMERAS];
    NFStatus cam_status[NF_NUM_CAMERAS];
    nf_u32 num_points_per_cam[NF_NUM_CAMERAS];
    NFPointCloud cloud;          /**< 融合后的点云 */
    NFMode  dominant_mode;       /**< 主导模式(用于日志) */
    nf_u8   is_frozen;           /**< 1=本帧冻结, cloud 不可用 */
    nf_u32  frame_id;
} NFFrameOutput;

typedef struct NFPipeline NFPipeline;

NFStatus nf_pipeline_create(NFPipeline **pp, const NFPipelineConfig *cfg,
                            const NFCameraModel *cams,
                            const NFPose *poses);
NFStatus nf_pipeline_process(NFPipeline *p, const NFFrameInput *in,
                             NFFrameOutput *out);
NFStatus nf_pipeline_destroy(NFPipeline *p);

/** 内存预算(不分配), 供上层启动前检查 (V-BUILD-3) */
NFStatus nf_pipeline_mem_bytes(const NFPipelineConfig *cfg, nf_u64 *out_bytes);

/** 参数自检: R1-R6 关系 (V-PARAM-1) */
NFStatus nf_params_validate(const NFPipelineConfig *cfg,
                            const NFCameraModel *cams, nf_u32 n_cams);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_PIPELINE_H */
