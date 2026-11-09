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
#include "nfocc_voxel_filter.h"

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
    /**
     * **每路相机各自的**车身掩码 [4][img_h*img_w]，NULL = 该路不启用。
     *
     * 为什么必须分开：4 路相机的车身投影位置完全不同（FV 看到车头，
     * RV 看到车尾，ML/MR 看到车侧）。共用一个指针会让三路相机
     * 用错掩码 —— 表现是"车身掩码没效果"或把大片正常区域剔掉。
     *
     * 掩码尺寸是**原图尺寸**，由 `nf_mask_build_vehicle` 生成。
     */
    const nf_u8 *body_mask[NF_NUM_CAMERAS];

    /**
     * σ 硬截断门限 (cm)，0 = 用 NF_SIGMA_MAX_CM(80)，负值 = 关闭截断。
     *
     * 超过门限的点直接丢弃，不进体素化。用途是切掉 SGBM 锁错周期产生
     * 的远处幽灵点（虚假小视差 -> 深度爆炸 -> σ 平方反比膨胀）。
     *
     * 设负值只为**做负对照**（验证截断真的在起作用），生产配置不该关。
     */
    nf_f32 sigma_max_cm;

    /**
     * 启用 REV-003 五阶段稠密体素精简（NEW_MOD §5）。
     *
     * 0 = 关闭（默认，保持旧行为）；1 = 开启，输出经过地面剥离 +
     * 极坐标外壳 + 配额熔断，点数上限 15000。
     *
     * 关闭时 `out.grid` 仍会填充（REV-002 的柱状排布），只是不做精简。
     */
    nf_u8  enable_voxel_filter;
    nf_u8  _pad_filter[3];
    NFVoxelFilterConfig filter_cfg;   /**< enable_voxel_filter=1 时生效 */

    /* ---- NEW_MOD §2.2.1 NFORM 产品化增强项 ---- */

    /**
     * D4 双基线交叉验证（NEW_MOD §1.3）。
     *
     * 1 = 启用：每个三角化点先查体素表，同格已有观测则做 3D 残差检验。
     *      通过 -> 融合质心、σ 减半；超限 -> 衰减权重。
     * 0 = 关闭：直接插入（旧行为）。
     *
     * 代价是每点一次哈希查找，实测约 1.3 倍插入耗时。
     */
    nf_u8  enable_d4;
    nf_u8  _pad_d4[3];

    /**
     * D4 检验门限下限 (mm)。0 = 用 50mm。
     *
     * **实际门限随距离自适应**（nf_verify_thresh_mm），这个值只是下限：
     * 近场 σ 小的时候防止门限退化到 0，远场会被 2*sqrt(σ1²+σ2²) 顶上去。
     */
    nf_f32 d4_tau_min_mm;

    /**
     * 双基线状态机替代单帧门控（NEW_MOD §1.1）。
     *
     * 1 = 启用：用 NFBaselineManager 累积历史帧，长基线达 b_target 才产出，
     *      纯旋转时冻结（FROZEN_ROT）。
     * 0 = 关闭：用旧的 nf_gate_decide 单帧门控。
     *
     * ## 两者的本质区别
     * 旧门控只看**当帧位移**：帧间 5km/h 走 278mm，刚够 b_min，长基线
     * 永远用不上，深度精度被卡死在短基线上。
     *
     * 状态机累积**多帧位移**：同样 5km/h，等 2 帧就凑到 b_target=500mm，
     * 精度提升约 1.8 倍。代价是要多存几帧历史。
     */
    nf_u8  enable_baseline_mgr;
    nf_u8  _pad_bl[3];

    /** 长基线触发目标 (mm)，0 -> NF_B_TARGET */
    nf_f32 b_target_mm;

    /**
     * 相机使能掩码（bit0=FV bit1=RV bit2=MVL bit3=MVR）。
     *
     * 0 = 四路全开（默认，兼容旧调用方）。用于"前视 90° / 侧视 150°"
     * 分视口验证：一个 pipeline 实例只处理掩码内的相机，另一实例处理
     * 其余相机，两者各自用设计视口，不再被一个全局 150° 绑死。
     */
    nf_u32 cam_enable_mask;

    /**
     * FORWARD 径向匹配加速（P1）：
     *   forward_sample_step = 0/1 -> 全分辨率；2 -> 每 2 行/列采样
     *   forward_coarse_step = 0 -> 2.0 px 粗搜，再 0.5 px 细化
     * 0/0 保持旧的全质量路径（测试默认），nfocc_run 生产默认 2/2.0。
     */
    nf_u32 forward_sample_step;
    nf_f32 forward_coarse_step;
    nf_f32 forward_ncc_min;      /* 0 -> 0.6；快速路径可收紧到 0.7 */
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

    /*
     * ---- REV-003 五阶段精简输出（enable_voxel_filter=1 时有效）----
     *
     * 与 `cloud` **并存**而不是替换：`cloud` 是完整点云（15 字节/点），
     * 这里是精简后的下游结构（10 字节/点，<= 15000）。两条路各自有用途：
     *   - 调试/可视化用 cloud（全量）
     *   - 下游接口用 dvf_points + dvf_zindex（配额内）
     *
     * 指针的生命周期到下一次 `nf_pipeline_process` 或 destroy 为止。
     */
    nf_u8   filter_enabled;     /**< 1=本次输出经过精简 */
    nf_u8   _pad_dvf[3];
    NFVoxelFilterStats filter_stats;  /**< 各阶段点数，供诊断 */
    const NFDownstreamPoint *dvf_points;  /**< [dvf_n_points] */
    nf_u32  dvf_n_points;       /**< 精简后点数（<= 15000） */
    nf_u32  dvf_n_cells;        /**< 底面网格数 */
    const nf_u32 *dvf_zmap;     /**< [dvf_n_cells] 高程位图 */
    const nf_u32 *dvf_zindex;   /**< [dvf_n_cells+1] 前缀和 */
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
