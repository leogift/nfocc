/**
 * @file pipeline.cpp
 * @brief Pipeline 编排实现。
 *
 * ## 为什么核心库不直接调用 nf_engine_*
 * SPEC-BUILD §3 与 ADR-006 要求 **核心库不链接 OpenCV**(V-BUILD-2)。
 * 但 pipeline 需要跑引擎。解法: 通过**函数指针表**注入引擎实现 ——
 * 上层(nfocc_run)链接 engine 库并调用 nf_pipeline_bind_engine() 注册。
 * 未注册时 pipeline 仍可跑前向路径(不依赖引擎)。
 */
#include "nfocc/nfocc_pipeline.h"
#include "nfocc/nfocc_match.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

/** 引擎晚绑定接口 */
struct NFEngineOps {
    NFStatus (*init)(NFEngineCtx **, const NFEngineConfig *);
    NFStatus (*run)(NFEngineCtx *, const NFImageView *, const NFImageView *,
                    const NFDisparityRange *, NFDisparityMap *);
    NFStatus (*deinit)(NFEngineCtx *);
};

namespace {
NFEngineOps g_engine_ops = { nullptr, nullptr, nullptr };
}

/** 由宿主程序注册引擎实现(见 tools/nfocc_run.cpp) */
extern "C" NFStatus nf_pipeline_bind_engine(const void *ops) {
    if (!ops) return NF_E_PARAM_NULL;
    g_engine_ops = *(const NFEngineOps *)ops;
    return NF_OK;
}

struct NFPipeline {
    NFPipelineConfig cfg;
    NFCameraModel    cams[NF_NUM_CAMERAS];
    NFPose           poses[NF_NUM_CAMERAS];

    /* 每相机固定缓冲, init 期一次分配 (SPEC-BUILD §5) */
    NFViewLut        lut[NF_NUM_CAMERAS];
    /**
     * 每相机**实际使用**的视口参数（含虚拟相机的 yaw_deg）。
     *
     * ## 为什么必须存下来
     * LUT 是按 `vp_c`（带偏转）构建的，但三角化时若回头去用
     * `cfg.view`（不带偏转），视口射线方向就与 LUT 的采样方向不一致。
     * 症状是深度系统性偏大、点云沉到地面以下 —— 实测四路中位误差
     * 352~645mm，91% 的点 z<300（应为 z≈0）。
     * 这个错误在"纹理横移"假场景里看不出来，因为那里没有几何约束。
     */
    NFViewParams     view_cam[NF_NUM_CAMERAS];
    nf_u8           *view_img[NF_NUM_CAMERAS * 2];   /* [左, 右] */
    nf_u8           *view_val[NF_NUM_CAMERAS * 2];
    NFDisparityMap   dmap[NF_NUM_CAMERAS];
    NFEngineCtx     *engine[NF_NUM_CAMERAS];

    /* 前向路径的对应点缓冲(径向膨胀匹配用)。
       与 SGBM 的 dmap 分开: 前向后向的中间量语义不同,
       共用一块会让"前向产出的点"和"侧向产出的视差"互相污染。 */
    nf_f32          *m_pts;      /* 2*w*h, u,v 交替 */
    nf_f32          *m_conf;     /* w*h */
    nf_u8           *m_valid;    /* w*h */

    NFVoxelCtx      *voxel;
    NFPointCloud     cloud;

    /*
     * REV-003 五阶段精简器。enable_voxel_filter=1 时才创建。
     *
     * 三块输出缓冲在这里预分配而不是逐帧 malloc：
     *   zmap   [n_cells]
     *   zindex [n_cells+1]
     *   dvf_pts[max_output]
     * 逐帧分配会产生堆碎片（实测帧耗时 P99 明显高于中位数）。
     */
    NFVoxelFilterCtx *filter;
    nf_u32           *dvf_zmap;
    nf_u32           *dvf_zindex;
    NFDownstreamPoint *dvf_pts;
    nf_u32            dvf_max_out;
    nf_u32            dvf_n_cells;
    NFVoxelFilterStats dvf_stats;

    nf_u32           last_frame_id;
    nf_u8            has_ref;
    nf_u8            cam_disabled;   /**< bit i = 相机 i 外参自检失败 */
};

namespace {

/** 视口像素 -> 视口系射线(归一化) */
void view_pixel_dir(const NFViewParams *vp, float u, float v, float *dir) {
    nf_view_pixel_to_ray(vp, u, v, dir);
}

/** 视口等效焦距。B_MIN/B_MAX/d_max 的反推都用它 (PARAM B.1/B.2)。 */
inline float cfg_view_f(const NFPipeline *p) { return p->cfg.view.f; }

} // namespace

/**
 * 本地 π 常量。
 *
 * **不要用 M_PI**：MinGW-W64 在 `-std=c++11` 严格模式下 `<cmath>` 不会
 * 暴露 POSIX 宏，编译直接报 "M_PI was not declared"（陷阱 5）。
 * 自己定义一份，跨工具链一致。
 */
static const float kPiF = 3.14159265358979323846f;

extern "C" {

NFStatus nf_params_validate(const NFPipelineConfig *cfg,
                            const NFCameraModel *cams, nf_u32 n_cams) {
    if (!cfg || !cams) return NF_E_PARAM_NULL;
    if (n_cams != NF_NUM_CAMERAS) return NF_E_PARAM_RANGE;

    /* R1: 视口自洽 */
    NFStatus st = nf_view_params_validate(&cfg->view);
    if (st != NF_OK) return st;

    /* R2: B_MAX 必须能覆盖引擎容量 (反推关系见 PARAM B.2)
       B_MAX = numDisp * B_ZMIN_EFF / VF_VIEW */
    const float b_max_from_engine =
        (float)cfg->engine.max_disp_px * NF_B_ZMIN_EFF / cfg->view.f;
    if (cfg->b_max_mm > b_max_from_engine * 1.05f) return NF_E_PARAM_RANGE;

    /* R3: B_MIN < B_TARGET < B_MAX */
    if (!(cfg->b_min_mm < cfg->b_max_mm)) return NF_E_PARAM_RANGE;

    /* R4: 体素网格自洽 */
    if (cfg->grid.nx == 0 || cfg->grid.ny == 0 || cfg->grid.nz == 0)
        return NF_E_PARAM_RANGE;
    if (cfg->grid.nz > 31) return NF_E_PARAM_RANGE;   /* zMap bit31 保留 */

    /* R5: 内参必须是本项目的 f=550 基准(否则 B_MIN/B_MAX 需重算) */
    for (nf_u32 i = 0; i < n_cams; ++i) {
        if (nf_cam_validate(&cams[i]) != NF_OK) return NF_E_CAM_NOT_CALIBRATED;
        if (cams[i].max_theta < 2.0f) return NF_E_PARAM_RANGE;  /* 防抄库默认 1.65 */
    }

    /* R6: numDisp 必须是 16 的倍数 */
    if (cfg->engine.max_disp_px % 16 != 0) return NF_E_PARAM_RANGE;
    return NF_OK;
}

NFStatus nf_pipeline_mem_bytes(const NFPipelineConfig *cfg, nf_u64 *out_bytes) {
    if (!cfg || !out_bytes) return NF_E_PARAM_NULL;
    const nf_u64 vw = cfg->view.width, vh = cfg->view.height;
    /* 视口图 2 张/相机 + 有效性 2 张 + 视差图 3 张(disp/valid/conf) */
    nf_u64 per_cam = vw * vh * 2ull            /* view_img x2 */
                   + vw * vh * 2ull            /* view_val x2 */
                   + vw * vh * (4ull + 1ull + 4ull);  /* dmap */
    *out_bytes = per_cam * NF_NUM_CAMERAS;

    /* 引擎 cost volume: 串行时只需 1 份, 并行 N 份 */
    const nf_u64 vol = (nf_u64)cfg->engine.max_disp_px * vw * vh * 2ull;
    *out_bytes += vol * (cfg->thread_cameras ? cfg->thread_cameras : 1);

    /* 体素表 */
    *out_bytes += (nf_u64)NF_G_HASH_CAPACITY * 64ull;
    return NF_OK;
}

NFStatus nf_pipeline_create(NFPipeline **pp, const NFPipelineConfig *cfg,
                            const NFCameraModel *cams,
                            const NFPose *poses) {
    if (!pp || !cfg || !cams || !poses) return NF_E_PARAM_NULL;
    *pp = nullptr;

    NFStatus st = nf_params_validate(cfg, cams, NF_NUM_CAMERAS);
    if (st != NF_OK) return st;

    NFPipeline *p = (NFPipeline *)std::calloc(1, sizeof(NFPipeline));
    if (!p) return NF_E_ENGINE_OOM;
    p->cfg = *cfg;
    for (nf_u32 i = 0; i < NF_NUM_CAMERAS; ++i) {
        p->cams[i] = cams[i];
        p->poses[i] = poses[i];
        /* 外参自检失败 -> 该相机禁用(SPEC-ICD 失效语义表)。
           用位掩码记录, process 时跳过该相机。 */
        if (nf_pose_validate(&poses[i]) != NF_OK) p->cam_disabled |= (nf_u8)(1u << i);
    }

    const nf_u32 vw = cfg->view.width, vh = cfg->view.height;

    /* 视锥张角可在 cfg 里覆盖(0 = 用默认 NF_V_VIEW_HFOV_DEG)。
     * 张角决定等效焦距 f = (W/2)/tan(hfov/2), 进而决定视差像素数 ——
     * 150° 比 120° 的 f 小 54%, 同一基线的视差只剩 46%。
     * 因此这是关键可调项, 不能被硬编码覆盖。 */
    NFViewParams view_base = cfg->view;
    /* **只在调用方显式要求改张角时才重建**（view_hfov_deg > 0）。
     *
     * 早期无条件重建, 用 NF_V_VIEW_HFOV_DEG 覆盖了 cfg->view 的张角。
     * 测试里用 480 宽 + 特定张角建好后, 重建把它换成默认值,
     * f 与 fov_h 不再自洽 —— nf_view_params_validate 报 NF_E_PARAM_RANGE。
     * 实际影响: integration_pipeline / system_run_synthetic 全挂。
     *
     * cfg->view 是调用方给的完整视口参数, 默认应当尊重它。 */
    if (cfg->view_hfov_deg > 0.0f) {
        NFViewParams derived;
        /* 虚拟相机: 显式 150°x120°, 平视 (pitch=0, roll=0) */
        if (nf_view_params_derive_fov(&derived, vw, cfg->view_hfov_deg,
                                      NF_V_VIEW_VFOV_DEG) == NF_OK) {
            view_base = derived;
        }
    }

    for (nf_u32 c = 0; c < NF_NUM_CAMERAS; ++c) {
        st = nf_lut_alloc(&p->lut[c], vw, vh);
        if (st != NF_OK) { nf_pipeline_destroy(p); return st; }

        /* 每相机的虚拟视口朝向由 cfg->view_yaw_deg 指定。
         *
         * 偏转的物理意义: 前后相机直行时平移沿光轴、横向基线为 0,
         * 视差深度消失。把该相机视口整体偏 30° 后, 平移在视口系的
         * 横向分量 = T·sin(30°) = 0.5T, Z=3m 得 9.24 px 视差。
         *
         * 用本地副本: cfg 是 const, 且各相机的偏转角不同。 */
        NFViewParams vp_c = view_base;
        vp_c.yaw_deg = cfg->view_yaw_deg[c];
        p->view_cam[c] = vp_c;

        NFLutStats stats;
        /* **D1 车身掩码接线**：此前这里传 nullptr，导致车体反光、车标、
         * 保险杠在点云里形成一层贴着车身的假障碍（实测 BEV 上车体轮廓
         * 一圈恒亮）。掩码在 LUT 构建期就把这些像素判为无效 —— 源头掐灭，
         * 下游零代价。cfg.body_mask[c] 为 NULL 时行为不变。 */
        st = nf_lut_build(&p->lut[c], &p->cams[c], &p->poses[c],
                          &vp_c, p->cfg.body_mask[c], &stats);
        if (st == NF_E_LUT_EMPTY) {
            /* 该虚拟视口完全落在本相机视场外 —— 正常工况。
             * 例: RV 相机 yaw=0（朝车头）时，车尾鱼眼根本拍不到。
             * 记为该相机禁用，process 时跳过，不影响其它相机。 */
            nf_log_msg(NF_LOG_WARN,
                       "cam%u 虚拟视口 yaw=%.0f° 完全不可见，本路禁用",
                       c, (double)vp_c.yaw_deg);
            p->cam_disabled |= (nf_u8)(1u << c);
            st = NF_OK;
        } else if (st != NF_OK) {
            nf_log_msg(NF_LOG_ERROR,
                       "cam%u LUT 构建失败: %s (视口 %ux%u f=%.2f yaw=%.1f)",
                       c, nf_status_str(st), vp_c.width, vp_c.height,
                       (double)vp_c.f, (double)vp_c.yaw_deg);
            nf_pipeline_destroy(p); return st;
        }

        for (int k = 0; k < 2; ++k) {
            const size_t idx = c * 2 + k;
            p->view_img[idx] = (nf_u8 *)std::malloc((size_t)vw * vh);
            p->view_val[idx] = (nf_u8 *)std::malloc((size_t)vw * vh);
            if (!p->view_img[idx] || !p->view_val[idx]) {
                nf_pipeline_destroy(p);
                return NF_E_ENGINE_OOM;
            }
        }
        p->dmap[c].version = NF_DISPARITY_MAP_VERSION;
        st = nf_dmap_alloc(&p->dmap[c], vw, vh);
        if (st != NF_OK) { nf_pipeline_destroy(p); return st; }
    }

    /* 前向路径的对应点缓冲: 每相机一份 */
    {
        const size_t np = (size_t)vw * vh;
        p->m_pts   = (nf_f32 *)std::calloc(np * 2, sizeof(nf_f32));
        p->m_conf  = (nf_f32 *)std::calloc(np, sizeof(nf_f32));
        p->m_valid = (nf_u8  *)std::calloc(np, 1);
        if (!p->m_pts || !p->m_conf || !p->m_valid) {
            nf_pipeline_destroy(p);
            return NF_E_ENGINE_OOM;
        }
    }

    st = nf_voxel_create(&p->voxel, &cfg->grid, NF_G_HASH_CAPACITY, NF_V_N_EFF_MAX);
    if (st != NF_OK) { nf_pipeline_destroy(p); return st; }

    st = nf_pointcloud_alloc(&p->cloud, cfg->max_points);
    if (st != NF_OK) { nf_pipeline_destroy(p); return st; }

    /*
     * REV-003 精简器初始化（仅 enable_voxel_filter=1 时）。
     *
     * 三块缓冲一次分配，运行期零堆分配。失败时整个 pipeline 创建失败 ——
     * 不能"过滤器没起来但 pipeline 能用"，那样下游会拿到未精简的点
     * 却以为是精简过的。
     */
    if (cfg->enable_voxel_filter) {
        st = nf_voxel_filter_create(&p->filter, &cfg->filter_cfg);
        if (st != NF_OK) { nf_pipeline_destroy(p); return st; }

        nf_u32 nx = 0, ny = 0, ncell = 0;
        nf_voxel_filter_grid_size(p->filter, &nx, &ny, &ncell);
        p->dvf_n_cells = ncell;
        p->dvf_max_out = cfg->filter_cfg.max_output_points
                        ? cfg->filter_cfg.max_output_points
                        : NF_DVF_HARD_CAP_MAX_POINTS;

        p->dvf_zmap   = (nf_u32 *)std::calloc(ncell, sizeof(nf_u32));
        p->dvf_zindex = (nf_u32 *)std::calloc((size_t)ncell + 1u, sizeof(nf_u32));
        p->dvf_pts    = (NFDownstreamPoint *)std::calloc(p->dvf_max_out,
                                                         sizeof(NFDownstreamPoint));
        if (!p->dvf_zmap || !p->dvf_zindex || !p->dvf_pts) {
            nf_pipeline_destroy(p);
            return NF_E_ENGINE_OOM;
        }
    }

    *pp = p;
    return NF_OK;
}

NFStatus nf_pipeline_destroy(NFPipeline *p) {
    if (!p) return NF_E_PARAM_NULL;

    /* REV-003 精简器与输出缓冲 */
    nf_voxel_filter_destroy(p->filter);
    std::free(p->dvf_zmap);
    std::free(p->dvf_zindex);
    std::free(p->dvf_pts);
    for (nf_u32 c = 0; c < NF_NUM_CAMERAS; ++c) {
        nf_lut_destroy(&p->lut[c]);
        for (int k = 0; k < 2; ++k)
            std::free(p->view_img[c * 2 + k]), std::free(p->view_val[c * 2 + k]);
        std::free(p->dmap[c].disp);
        std::free(p->dmap[c].valid);
        std::free(p->dmap[c].conf);
        if (p->engine[c] && g_engine_ops.deinit) g_engine_ops.deinit(p->engine[c]);
    }
    nf_voxel_destroy(p->voxel);
    nf_pointcloud_destroy(&p->cloud);
    std::free(p);
    return NF_OK;
}

NFStatus nf_pipeline_process(NFPipeline *p, const NFFrameInput *in,
                             NFFrameOutput *out) {
    if (!p || !in || !out) return NF_E_PARAM_NULL;
    if (out->version != NF_POINT_CLOUD_VERSION) out->version = NF_POINT_CLOUD_VERSION;

    std::memset(out->mode, 0, sizeof(out->mode));
    std::memset(out->cam_status, 0, sizeof(out->cam_status));
    std::memset(out->num_points_per_cam, 0, sizeof(out->num_points_per_cam));
    out->cloud.num_points = 0;
    out->cloud.overflow = 0;
    out->is_frozen = 0;
    out->frame_id = in->frame_id;

    nf_voxel_clear(p->voxel);

    const nf_u32 vw = p->cfg.view.width, vh = p->cfg.view.height;
    nf_u32 n_frozen = 0;

    /* ---------------------------------------------------------------
     * 阶段 1: 四路 warp 并行
     *
     * warp 是**纯函数**(读原图+LUT, 写自己那一路的视口缓冲), 没有共享
     * 写状态, 因此可以安全并行。这是整个流程里唯一值得并行的部分 ——
     * SGBM 每路要 127MB 工作内存, 4 路并行需要 507MB, 超出上板预算
     * (SPEC-BUILD §4 的实算结论), 所以引擎仍串行。
     *
     * OpenMP 不可用时这里是普通的串行循环, 结果一致。
     * --------------------------------------------------------------- */
    NFStatus warp_st[NF_NUM_CAMERAS];
    nf_u8 cam_active[NF_NUM_CAMERAS];
    for (nf_u32 c = 0; c < NF_NUM_CAMERAS; ++c) {
        warp_st[c] = NF_OK;
        cam_active[c] = (p->cam_disabled & (nf_u8)(1u << c)) ? 0u : 1u;
    }

#ifdef NF_HAS_OPENMP
#pragma omp parallel for num_threads(NF_NUM_CAMERAS) schedule(static)
#endif
    for (int ci = 0; ci < (int)NF_NUM_CAMERAS; ++ci) {
        const nf_u32 c = (nf_u32)ci;
        if (!cam_active[c] || !in->img[c]) continue;
        NFImageView src;
        src.version = NF_IMAGE_VIEW_VERSION;
        src.data = in->img[c];
        src.width = in->width;
        src.height = in->height;
        src.stride = in->stride;
        src.valid_x0 = nullptr;
        src.timestamp_us = in->timestamp_us;
        src.frame_id = in->frame_id;
        warp_st[c] = nf_view_warp(&p->lut[c], src.data, src.width, src.height,
                                  src.stride, p->view_img[c * 2],
                                  p->view_val[c * 2]);
    }

    /**
     * σ 硬截断门限由配置解析一次，两个分支（前向/侧向）共用。
     * 0 -> 默认 NF_SIGMA_MAX_CM；负值 -> 关闭（仅用于负对照）。
     */
    const nf_f32 sigma_cap = (p->cfg.sigma_max_cm == 0.0f)
                           ? (nf_f32)NF_SIGMA_MAX_CM : p->cfg.sigma_max_cm;

    for (nf_u32 c = 0; c < NF_NUM_CAMERAS; ++c) {
        /* 本相机被 σ 截断的点数，供诊断输出 */
        nf_u32 n_sigma_cut = 0;
        /* 外参自检失败的相机: 整个相机禁用 (SPEC-ICD 失效语义表) */
        if (p->cam_disabled & (nf_u8)(1u << c)) {
            out->cam_status[c] = NF_E_EXTRINSIC_INVALID;
            ++n_frozen;
            continue;
        }
        /* ---- 门控 (ADR-003) ---- */
        NFGateDecision dec;
        /* ---- 把平移旋到**视口系**再判工况 ----
         *
         * 这是 6 视口方案成立的关键。门控判的是"平移在成像坐标系里
         * 有多大的横向分量", 而这个量**随视口偏转改变**:
         *   直行 t=(0,0,T), 视口偏 +30° -> 视口系 t=(T·sin30, 0, T·cos30)
         *   sigma 从 0.000 变成 0.500, 直接从 FORWARD 跳到可供视差的区间。
         *
         * 若仍用车系 t 判, 偏转完全不产生作用 —— 实测确认过:
         * 偏 30° 后 cam0 仍报 FORWARD、点云与不偏转时几乎一样。
         *
         * 旋转方向: 视口绕相机 y 轴偏 yaw 角, 等价于把 t 反向旋转。
         */
        /**
         * **t_cam 已经是视口系，直接用，不能再旋转**
         * （SPEC-CONVENTION §2）。
         *
         * 历史: 这里曾把车系 t 旋转到视口系。后来把换算下沉到输入源后，
         * 若这里再转一次就是"转两次"，门控判据反而不对。
         * 输入源负责换算，pipeline 只消费。
         */
        const float *t_view = in->t_cam[c];

        NFStatus st = nf_gate_decide(t_view, in->frame_gap,
                                     p->cfg.b_min_mm, p->cfg.b_max_mm,
                                     NF_SIGMA_STEREO_MIN, NF_SIGMA_FORWARD_MAX,
                                     p->cfg.gap_max_frames, &dec);
        if (st != NF_OK) { out->cam_status[c] = st; continue; }
        out->mode[c] = dec.mode;

        /* 冻结: **不产出点云** */
        if (!nf_mode_produces_depth(dec.mode)) {
            out->cam_status[c] = (dec.mode == NF_MODE_FROZEN_NEAR) ? NF_E_BASELINE_IDLE
                               : (dec.mode == NF_MODE_FROZEN_FAR)  ? NF_E_BASELINE_OVER
                               : NF_E_BASELINE_DEGRADED;
            ++n_frozen;
            continue;
        }

        if (!in->img[c]) { out->cam_status[c] = NF_E_PARAM_NULL; continue; }

        NFImageView src;
        src.version = NF_IMAGE_VIEW_VERSION;
        src.data = in->img[c];
        src.width = in->width;
        src.height = in->height;
        src.stride = in->stride;
        src.valid_x0 = nullptr;
        src.timestamp_us = in->timestamp_us;
        src.frame_id = in->frame_id;

        /* ---- Warp 已在阶段 1 并行完成, 这里只取结果 ---- */
        if (warp_st[c] != NF_OK) { out->cam_status[c] = warp_st[c]; continue; }

        /* ---- 前向路径: 径向膨胀解析测距 (ADR-003) ----
           注意: 真实系统里对应点须由光流给出。本实现用"视口图上的像素匹配"
           作为占位 —— 它的精度直接决定测距精度(1px -> 110mm)。
           此处只验证链路, 像素匹配的实现在下一阶段补齐。 */
        if (dec.mode == NF_MODE_FORWARD) {
            /* ---- 前向: 径向膨胀匹配 + 解析测距 (SPEC-FORWARD) ----
             *
             * 1) 在视口图上找对应点(径向搜索, 不依赖极线平行)
             * 2) 沿 FoE 计算膨胀率 s = rho_ref/rho_cur
             * 3) 反射线→车系, 用 nf_forward_range 精确求解
             *
             * 为什么不用 Z = f*b/d: 该式只在平移垂直于视线时成立,
             * 前向工况视线与平移夹角 0~90°, 实测光轴附近误差 10.3 倍。
             */
            if (!in->img_ref[c]) {
                out->cam_status[c] = NF_E_BASELINE_IDLE;
                out->num_points_per_cam[c] = 0;
                continue;
            }

            /* 参考帧 warp(前向路径用左缓冲做当前帧, 右缓冲做参考帧) */
            NFStatus st_r = nf_view_warp(&p->lut[c], in->img_ref[c],
                                         src.width, src.height, src.stride,
                                         p->view_img[c * 2 + 1],
                                         p->view_val[c * 2 + 1]);

            /* FoE: 纯前向平移时膨胀中心即光心 */
            NFFoe foe = { (nf_f32)vw * 0.5f, (nf_f32)vh * 0.5f };

            /* 搜索范围由基线与深度范围推:
             * 位移 ≈ T·rho/Z_min 到 T·rho/Z_max。
             * 用视口角落的最大半径算上界。 */
            const float tmm = std::sqrt(in->t_cam[c][0] * in->t_cam[c][0]
                                      + in->t_cam[c][1] * in->t_cam[c][1]
                                      + in->t_cam[c][2] * in->t_cam[c][2]);
            const float rho_max = std::sqrt((nf_f32)vw * vw + (nf_f32)vh * vh) * 0.5f;
            const float r_lo = 0.5f;
            float r_hi = tmm * rho_max / NF_B_ZMIN_EFF;
            if (r_hi < r_lo + 1.0f) r_hi = r_lo + 1.0f;
            if (r_hi > 64.0f) r_hi = 64.0f;   /* 上限防搜索过慢 */

            nf_u32 fwd_pts = 0;
            if (st_r == NF_OK && tmm > 1e-3f) {
                NFStatus ms = nf_match_radial(p->view_img[c * 2],
                                              p->view_img[c * 2 + 1],
                                              p->view_val[c * 2],
                                              vw, vh, foe, r_lo, r_hi,
                                              2, 0.6f,
                                              p->m_pts, p->m_conf, p->m_valid);
                if (ms != NF_OK) {
                    out->cam_status[c] = ms;
                    out->num_points_per_cam[c] = 0;
                    continue;
                }

                /* 逐点: 视口像素 → 视口系方向 → 两帧射线 → 解析测距 */
                const NFPose *pose = &p->poses[c];
                for (nf_u32 yy = 0; yy < vh; ++yy) {
                    for (nf_u32 xx = 0; xx < vw; ++xx) {
                        const nf_u32 i = yy * vw + xx;
                        if (!p->m_valid[i]) continue;

                        float d0[3], d1[3];
                        view_pixel_dir(&p->view_cam[c], (nf_f32)xx + 0.5f,
                                       (nf_f32)yy + 0.5f, d0);
                        view_pixel_dir(&p->view_cam[c],
                                       p->m_pts[i * 2], p->m_pts[i * 2 + 1], d1);

                        /* 两帧射线视口系 → 车系方向 */
                        float a[3], b[3];
                        for (int k = 0; k < 3; ++k) {
                            a[k] = pose->R[k * 3 + 0] * d0[0]
                                 + pose->R[k * 3 + 1] * d0[1]
                                 + pose->R[k * 3 + 2] * d0[2];
                            b[k] = pose->R[k * 3 + 0] * d1[0]
                                 + pose->R[k * 3 + 1] * d1[1]
                                 + pose->R[k * 3 + 2] * d1[2];
                        }

                        /* 解析测距: 给两帧的方向向量与平移, 返回沿 b 的距离。
                         * 用 b(参考帧方向)作为基准, 因为参考帧是"已知"的那一帧,
                         * 它到物点的距离即所需深度。 */
                        /* **必须用视口系的 t**。a/b 是视口系射线;
                         * 视口偏转后视口系 != 车系, 混用车系 t 会让解出的
                         * 距离落到几十米外被下面的 50000mm 门限拒掉 ——
                         * 实测 45° 偏转时 cam0 产出从 1380 掉到 0。 */
                        /**
                         * **t 必须转到车系再用**（陷阱 1 的后半段）。
                         *
                         * `t_view` 是视口系位移，而 `pose->t` 是车系光心、
                         * `a`/`b` 也是车系射线。直接 `pose->t - t_view`
                         * 是在**混基相减**，偏转 0° 时恰好相等（不暴露），
                         * 偏转后顶点会整体偏一个基线向量。
                         */
                        const float yaw_f = p->cfg.view_yaw_deg[c] * (kPiF / 180.0f);
                        const float cyf = std::cos(yaw_f), syf = std::sin(yaw_f);
                        const float tt[3] = {
                            t_view[0]*cyf - t_view[1]*syf,
                            t_view[0]*syf + t_view[1]*cyf,
                            t_view[2],
                        };
                        float dist = 0.0f;
                        nf_u8 okp = 0;
                        nf_forward_range(a, b, 1, tt, &dist, &okp);
                        if (!okp || dist <= 0.0f || dist > 50000.0f) continue;

                        /* 物点 = 参考帧光心 + b*dist = 当前光心 + a*(dist - 沿a的分量)。
                         * 直接由 b 构造: 参考帧光心 = 当前光心 - t */
                        /* P = 参考帧光心 + b*dist，其中参考帧光心 = 当前光心 - t_veh。
                         * 注意 b 是**参考帧射线**，不是当前帧射线 —— 两者不要混用，
                         * 否则会得到 `C_curr - t + d_curr*dist` 那种物理上错的式子。 */
                        const float P[3] = {
                            pose->t[0] - tt[0] + b[0] * dist,
                            pose->t[1] - tt[1] + b[1] * dist,
                            pose->t[2] - tt[2] + b[2] * dist
                        };

                        /* ---- D3: FoE 发散锥过滤 ----
                         * 前向工况平移方向≈视线方向，沿轴线的点深度退化，
                         * 会解出几十米外的"拉丝"幽灵点。锥内直接丢弃。 */
                        const float tn = std::sqrt(tt[0]*tt[0] + tt[1]*tt[1] + tt[2]*tt[2]);
                        if (tn > 1e-3f) {
                            const float dir[3] = { tt[0]/tn, tt[1]/tn, tt[2]/tn };
                            if (nf_pt_in_foe(P, pose->t, dir, 0.9806f)) continue;
                        }

                        /* ---- 方差传播: 由距离反推等效视差，再算 sigma ---- */
                        const float b_len = tn;
                        const nf_u8 sig = (b_len > 1e-3f)
                            ? nf_compute_spatial_uncertainty_cm(
                                  cfg_view_f(p), b_len,
                                  cfg_view_f(p) * b_len / dist, 0.5f, 0.02f)
                            : 255u;
                        /* σ 硬截断：幽灵点的 σ 会爆到几百 cm，直接丢 */
                        if (sigma_cap > 0.0f && (nf_f32)sig > sigma_cap) {
                            ++n_sigma_cut;
                            continue;
                        }
                        if (nf_voxel_insert_with_sigma(p->voxel, P[0], P[1], P[2],
                                                       p->m_conf[i], (nf_u8)(1u << c),
                                                       sig) == NF_OK)
                            ++fwd_pts;
                    }
                }
            }

            out->cam_status[c] = NF_OK;
            out->num_points_per_cam[c] = fwd_pts;
            continue;
        }

        /* ---- 侧向: SGBM ---- */
        if (!g_engine_ops.init) {
            out->cam_status[c] = NF_E_ENGINE_NOT_INIT;
            continue;
        }
        if (!p->engine[c]) {
            st = g_engine_ops.init(&p->engine[c], &p->cfg.engine);
            if (st != NF_OK) {
                nf_log_msg(NF_LOG_ERROR, "cam%u 引擎初始化失败: %s", c, nf_status_str(st));
                out->cam_status[c] = st;
                continue;
            }
        }

        /* 侧向匹配需要**两帧**: 当前帧视口图 与 参考帧视口图。
         *
         * 本实现只有单帧输入, 因此第二张图取自 `in->img_ref`(参考帧原图)。
         * 若未提供, 用当前帧填充 —— 此时两图相同, SGBM 输出全无效,
         * 这是**正确行为**(无从匹配), 不是缺陷。
         * 系统测试要看到非零点, 必须提供 img_ref。 */
        /* 镜像判据必须用**视口系**横向分量。用车系 t_cam[0] 时,
         * 视口偏转后左右方向可能反了 -> SGBM 在错误方向搜索 -> 全无效。
         * 实测 45° 偏转时 cam0 产出 0 点, 就是这个原因。 */
        const bool mirror = t_view[0] > 0.0f;
        /* **判的是 in->img_ref[c] 而不是 in->img_ref**:
           后者是数组, 地址恒非空, 布尔判断恒真(编译告警 -Wpointer-bool-conversion)。
           写错时"没有参考帧"会走入有参考帧分支, 拿 nullptr 去 warp。 */
        const nf_u8 *src_ref = in->img_ref[c] ? in->img_ref[c] : in->img[c];
        if (in->img_ref[c]) {
            NFStatus st_r = nf_view_warp(&p->lut[c], src_ref, src.width,
                                         src.height, src.stride,
                                         p->view_img[c * 2 + 1],
                                         p->view_val[c * 2 + 1]);
            if (st_r != NF_OK) { out->cam_status[c] = st_r; continue; }
        } else {
            std::memcpy(p->view_img[c * 2 + 1], p->view_img[c * 2], (size_t)vw * vh);
        }
        if (mirror) {
            /* 水平镜像后匹配: SGBM 假定 right(x) = left(x - d) */
            for (nf_u32 y = 0; y < vh; ++y) {
                nf_u8 *a = p->view_img[c * 2] + (size_t)y * vw;
                nf_u8 *b = p->view_img[c * 2 + 1] + (size_t)y * vw;
                for (nf_u32 x = 0; x < vw / 2; ++x) {
                    const nf_u8 t1 = a[x]; a[x] = a[vw - 1 - x]; a[vw - 1 - x] = t1;
                    const nf_u8 t2 = b[x]; b[x] = b[vw - 1 - x]; b[vw - 1 - x] = t2;
                }
            }
        }

        NFImageView lv = src, rv = src;
        lv.data = p->view_img[c * 2];
        lv.width = vw; lv.height = vh; lv.stride = vw; lv.valid_x0 = nullptr;
        rv.data = p->view_img[c * 2 + 1];
        rv.width = vw; rv.height = vh; rv.stride = vw; rv.valid_x0 = nullptr;

        NFDisparityRange dr;
        dr.version = NF_DISPARITY_RANGE_VERSION;
        dr.d_min = 0.0f;
        /* d_max 由基线与该档最近距离反推: d = f·b/Z_min (SPEC-ICD §4.1) */
        dr.d_max = cfg_view_f(p) * dec.baseline_mm / NF_B_ZMIN_EFF;
        dr.b_mm = dec.baseline_mm;

        st = g_engine_ops.run(p->engine[c], &lv, &rv, &dr, &p->dmap[c]);
        if (st != NF_OK) { out->cam_status[c] = st; continue; }

        /* ---- 三角化 -> 体素插入 ---- */
        nf_u32 inserted = 0;
        for (nf_u32 y = 0; y < vh; ++y) {
            for (nf_u32 x = 0; x < vw; ++x) {
                const size_t i = (size_t)y * vw + x;
                if (!p->dmap[c].valid[i] || !p->view_val[c * 2][i]) continue;
                const float d = p->dmap[c].disp[i];
                if (d < dr.d_min || d > dr.d_max) continue;

                /* 参考帧方向 */
                float r0[3], r1[3];
                view_pixel_dir(&p->view_cam[c], (float)x + 0.5f, (float)y + 0.5f, r0);
                const float u2 = mirror ? ((float)(vw - 1 - x) + 0.5f + d)
                                        : ((float)x + 0.5f - d);
                view_pixel_dir(&p->view_cam[c], u2, (float)y + 0.5f, r1);

                float s0 = 0.0f;
                nf_u8 ok = 0;
                const float c0[3] = { 0, 0, 0 };
                /* **c1 = -t_view**, 两个要点:
                 *
                 * 1) 必须是**视口系**的 t。r0/r1 是视口系射线,
                 *    视口偏转后视口系 != 车系, 混用会让解出几十米外的点。
                 *
                 * 2) **必须取负**。t_cam 的语义是"相机相对参考帧的位移",
                 *    而三角化里 c0=0(当前光心), c1 应是**参考帧光心相对当前帧**
                 *    的位置, 即 -t。
                 *    实测反例: 45° 偏转时取 +t_view 得 s0=-364.6(物点在相机后方),
                 *    取 -t_view 得 s0=+364.6 (物理正确)。
                 *    此前偏转为 0 时没暴露, 是因为横向/前向两条路径对 t 的
                 *    符号约定不同, 恰好各自抵消。 */
                /**
                 * **c1 必须和 r0/r1 在同一个基（车系）**。
                 *
                 * r0/r1 由 `view_pixel_dir` 给出，虚拟相机模型下它们
                 * 已经是**车系**单位向量。而 t_view 是**视口系**的位移。
                 * 直接拿视口系的 c1 配车系的射线 = 基不一致，
                 * 解出的 s0 会被视口 yaw 旋转一个角度 ——
                 * 偏转 0° 时恰好相等（不暴露），偏 45° 时完全错。
                 *
                 * 故这里把 t_view 转到车系: v_V = Rz(yaw) · v_S
                 */
                const float yaw_r = p->cfg.view_yaw_deg[c] * (kPiF / 180.0f);
                const float cyr = std::cos(yaw_r), syr = std::sin(yaw_r);
                const float tV_car[3] = {
                    t_view[0]*cyr - t_view[1]*syr,
                    t_view[0]*syr + t_view[1]*cyr,
                    t_view[2],
                };
                const float c1[3] = { -tV_car[0], -tV_car[1], -tV_car[2] };
                nf_triangulate_rays(c0, r0, c1, r1, 1, &s0, &ok);
                if (!ok) continue;

                /* 视口系 -> 车系 (视口轴与相机轴同向) */
                const NFPose *pose = &p->poses[c];
                /**
                 * **r0 已经是车系方向，不要再乘 pose->R。**
                 *
                 * 虚拟相机（pitch=0, roll=0）的基向量直接建在车系上，
                 * `nf_view_pixel_to_ray` 返回的就是车系单位向量。
                 * 三角化得到 Pc = r0·s0 也就是**车系**坐标，只需加光心平移。
                 *
                 * 历史 bug: 这里曾把 Pc 当相机系乘 R 再转车系 ——
                 * 那是"视口射线在相机系"时代的写法。改成虚拟相机后
                 * 变成**转两次**：得到的点绕光心转了一个安装角，
                 * 表现为沿视线方向 20%~45% 的深度偏差（实测中位 991mm），
                 * 而 SGBM 视差本身是对的（单独验证误差 0.9%）。
                 */
                const float Pv[3] = {
                    r0[0]*s0 + pose->t[0],
                    r0[1]*s0 + pose->t[1],
                    r0[2]*s0 + pose->t[2],
                };
                /* ---- D3: FoE 锥过滤（用已转车系的 tV_car）---- */
                const float tn2 = std::sqrt(tV_car[0]*tV_car[0] + tV_car[1]*tV_car[1]
                                            + tV_car[2]*tV_car[2]);
                if (tn2 > 1e-3f) {
                    const float dir2[3] = { tV_car[0]/tn2, tV_car[1]/tn2, tV_car[2]/tn2 };
                    if (nf_pt_in_foe(Pv, pose->t, dir2, 0.9806f)) continue;
                }

                /* ---- 方差传播：真实视差 + 真实基线 ---- */
                const nf_u8 sig2 = nf_compute_spatial_uncertainty_cm(
                    cfg_view_f(p), dec.baseline_mm, d, 0.5f, 0.02f);

                /**
                 * σ 硬截断（NEW_MOD §1.2.2）。
                 *
                 * SGBM 在规则纹理上会锁到错误的视差周期 —— 实测真值
                 * 140.8px 报成 2.2px。Z=f·b/d 下 d 变小则 Z 爆炸，
                 * 这些点会散到几十米外，把整片 BEV 糊掉。
                 *
                 * 方差传播正好识别它们：d 极小时 σ ∝ 1/d² 迅速膨胀。
                 * 所以按 σ 截断比按距离截断更有依据 —— 后者会误杀
                 * "远处但测量可靠"的点，而 σ 只看"这个测量可不可信"。
                 */
                if (sigma_cap > 0.0f && (nf_f32)sig2 > sigma_cap) {
                    ++n_sigma_cut;
                    continue;
                }

                if (nf_voxel_insert_with_sigma(p->voxel, Pv[0], Pv[1], Pv[2],
                                               p->dmap[c].conf[i], (nf_u8)(1u << c),
                                               sig2) == NF_OK)
                    ++inserted;
            }
        }
        out->cam_status[c] = NF_OK;
        out->num_points_per_cam[c] = inserted;
        if (n_sigma_cut) {
            nf_log_msg(NF_LOG_INFO, "cam%u σ截断剔除 %u 点 (门限 %.0fcm)",
                       c, n_sigma_cut, (double)sigma_cap);
        }
    }

    /* 主导模式: 取产出点数最多的相机 */
    nf_u32 best = 0;
    for (nf_u32 c = 1; c < NF_NUM_CAMERAS; ++c)
        if (out->num_points_per_cam[c] > out->num_points_per_cam[best]) best = c;
    out->dominant_mode = out->mode[best];

    /* 全部冻结 -> 本帧冻结, cloud 不可用 (ADR-003) */
    if (n_frozen == NF_NUM_CAMERAS) {
        out->is_frozen = 1;
        out->cloud.num_points = 0;
        return NF_E_BASELINE_IDLE;
    }

    /*
     * ==================== REV-003: 五阶段精简 ====================
     *
     * **先导出点再让过滤器处理**，而不是让过滤器直接读体素表。
     *
     * 原因：评审 §5.1 指出阶段 4 需要 sqrt(x²+y²)，位图只有"某层有没有点"，
     * 恢复不出坐标。所以顺序必须是「导出点数组 -> 过滤器按数组算」，
     * 位图只作索引。把这一步写在这里，防止后来者"优化"成直接读位图。
     */
    NFStatus st = nf_voxel_export_points(p->voxel, &p->cloud);

    if (p->filter && p->cloud.num_points > 0) {
        /*
         * 自车运动方向给 FoE 锥判别用。
         *
         * 取相机 0 的视口平移作为近似运动方向 —— 视口系 z 轴 = 车系前进，
         * 所以 (t_view[0], t_view[2]) 转到车系就是前进分量。静止时
         * 传 (0,0)，过滤器会跳过 FoE 过滤（静止没有退化方向）。
         */
        {
            const nf_f32 tvz = in->t_cam[0][2];
            const nf_f32 tvx = in->t_cam[0][0];
            nf_voxel_filter_update_ego_motion(p->filter, tvz, -tvx);
        }

        const NFStatus fs = nf_voxel_filter_compress(
            p->filter,
            (const NFVoxelEntry *)p->cloud.points, p->cloud.num_points,
            p->dvf_zmap, p->dvf_zindex, p->dvf_pts, &p->dvf_stats);

        if (fs == NF_OK) {
            /*
             * 精简后点云回写到 out->cloud。
             *
             * **不能复用 p->cloud.points 的原有类型**：下游结构体是
             * NFDownstreamPoint（10 字节），而 p->cloud 是 NFPoint（15 字节）。
             * 这里把精简结果放在 p->dvf_pts，通过 out->grid 暴露给调用方，
             * 原有 out->cloud 保持完整点云不变 —— 两条路并存，
             * 下游要精简版读 grid，要全量读 cloud。
             */
            out->filter_enabled = 1u;
            out->filter_stats = p->dvf_stats;
            out->dvf_points = p->dvf_pts;
            out->dvf_n_points = p->dvf_stats.n_output;
            out->dvf_n_cells = p->dvf_n_cells;
            out->dvf_zmap = p->dvf_zmap;
            out->dvf_zindex = p->dvf_zindex;
        }
    }
    /* 把内部缓冲借给输出(生命周期到下一次 process) */
    out->cloud.points = p->cloud.points;
    out->cloud.num_points = p->cloud.num_points;
    out->cloud.max_points = p->cloud.max_points;
    out->cloud.overflow = p->cloud.overflow;
    p->last_frame_id = in->frame_id;
    return st;
}

} /* extern "C" */
