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
#include "nfocc/nfocc_baseline.h"
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

/** 一个虚拟视口的运行期状态 */
struct NFViewState {
    nf_u32          cam_id;
    NFViewParams    vp;
    NFViewLut       lut;
    nf_u8          *img[2];   /* [当前, 参考] */
    nf_u8          *val[2];
    NFDisparityMap  dmap;
    nf_u8           usable;   /* LUT 有有效像素 */
    nf_u8           _pad[3];
};

struct NFPipeline {
    NFPipelineConfig cfg;
    NFCameraModel    cams[NF_NUM_CAMERAS];
    NFPose           poses[NF_NUM_CAMERAS];

    /*
     * 虚拟视口（6 视口模式）或相机（旧模式）的固定缓冲。
     *
     * 旧结构是每相机一套 lut/img/dmap，隐含"一个相机只有一个视口"。
     * 6 视口要求同一相机（FV/RV）出两个 yaw 不同的视口，因此按**视口**
     * 存：每个视口有自己的 LUT、当前/参考图、有效位图、视差图。
     * 引擎只保留一个：SGBM 的 compute 不绑定图像尺寸，串行复用即可，
     * 比 6 份 SGBM 工作内存省得多。
     */
    NFViewState      views[NF_MAX_VIEWS];
    nf_u32           n_views;

    /* 前向路径的对应点缓冲(径向膨胀匹配用)。按最大视口面积分配，
       逐视口串行复用；与 SGBM 的 dmap 分开，避免语义污染。 */
    nf_f32          *m_pts;      /* 2*m_w*m_h, u,v 交替 */
    nf_f32          *m_conf;     /* m_w*m_h */
    nf_u8           *m_valid;    /* m_w*m_h */
    nf_u32           m_w, m_h;

    /* SGBM 引擎：全 pipeline 一个，串行跑各视口 */
    NFEngineCtx     *engine;

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

    /*
     * 双基线状态机（NEW_MOD §1.1）。
     *
     * 需要**累积绝对位姿**：状态机要比较"当前帧"与"若干帧之前的某帧"
     * 之间的基线长度，而 pipeline 每帧只拿到相对的 t_cam。这里按
     * t_cam 逐帧积分出 curr_pose。
     *
     * 精度说明：积分用的是每帧相对位移，长期会漂移（2% 量级的里程计
     * 误差会累积）。但基线长度只用到最近几帧的相对关系，漂移可忽略。
     */
    NFBaselineManager baseline_mgr;
    NFBaselinePose    curr_pose;
    nf_u8             has_pose_init;
    nf_u32            bl_frame_id;
    /** cam0 本帧算出的决策，其余三路直接复用（同一辆车的同一次运动） */
    NFBaselineDecision last_bl_dec;

    nf_u32           last_frame_id;
    nf_u8            has_ref;
    nf_u8            cam_disabled;   /**< bit i = 相机 i 外参自检失败 */
    nf_u32           cam_mask;       /**< bit i = 本实例处理相机 i */
};


namespace {

/**
 * 把一点写入体素表，可选 D4 交叉验证（NEW_MOD §1.3）。
 *
 * ## 为什么封成函数
 * 前向分支和侧向分支的插入逻辑必须完全一致，否则同一份点云走两条路
 * 会得到不同结果。D4 的门限又要按各自的距离算，所以参数化。
 *
 * ## D4 的判定
 *   - 体素表里该格没有观测  -> 首次出现，正常插入
 *   - 已有观测且残差 <= τ   -> 融合质心、σ 除以 sqrt(2)
 *   - 已有观测且残差 >  τ   -> 判为动态拖影或错配，衰减权重
 *
 * τ = max(tau_min, 2*sqrt(σ1² + σ2²))，σ 由距离与基线给出。
 * **不能用固定 50mm**：2m 处短基线自身的 σ 就有 66mm，固定门限会把
 * 长基线的有效点一起衰减（评审 #2，已用数据证实）。
 */
/**
 * D4 融合计数（用于诊断"开了 D4 但点数没变"这类疑问）。
 * 真实稀疏点云每格常只有 1 个点，没有可融合的第二次观测 ——
 * 此时点数不变是**正确行为**，不是 D4 失效。
 */
static nf_u32 g_d4_fused = 0;
static nf_u32 g_d4_rejected = 0;

NFStatus insert_point(NFPipeline *p, nf_u32 cam, nf_f32 x, nf_f32 y, nf_f32 z,
                      nf_f32 conf, nf_u8 sigma_cm, nf_f32 dist_mm,
                      nf_f32 baseline_mm, nf_f32 view_f, nf_u8 *out_verdict) {
    if (out_verdict) *out_verdict = 0u;

    if (p->cfg.enable_d4) {
        const nf_f32 tau = nf_verify_thresh_mm(dist_mm, view_f,
                                               baseline_mm, baseline_mm * 0.556f,
                                               p->cfg.d4_tau_min_mm);
        nf_u8 verdict = 0;
        const NFStatus vs = nf_voxel_verify_cross(p->voxel, x, y, z,
                                                  (nf_u8)(1u << cam), tau,
                                                  &verdict);
        if (vs == NF_OK && verdict == 1u) {
            ++g_d4_fused;
            if (out_verdict) *out_verdict = 1u;
            return NF_OK;      /* 融合成功，不再单独插入 */
        }
        if (vs == NF_OK && verdict == 0u && p->cfg.enable_d4) {
            /* 走到这里说明该格有旧观测但残差超限，已衰减 */
            ++g_d4_rejected;
        }
        if (vs == NF_OK) {
            /* 该格没有已有观测，或残差超限已被衰减 —— 正常插入 */
        }
    }

    return nf_voxel_insert_with_sigma(p->voxel, x, y, z, conf,
                                      (nf_u8)(1u << cam), sigma_cm);
}

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
    /* R7: 相机掩码不得越界（0 = 全开） */
    if (cfg->cam_enable_mask & ~0xFu) return NF_E_PARAM_RANGE;

    /* R8: 6 视口规划必须完整；避免 n_views>0 但 plan 未初始化时读垃圾 */
    if (cfg->n_views > NF_MAX_VIEWS) return NF_E_PARAM_RANGE;
    for (nf_u32 v = 0; v < cfg->n_views; ++v) {
        const NFViewPlanEntry &e = cfg->view_plan[v];
        if (e.cam_id >= NF_NUM_CAMERAS) return NF_E_PARAM_RANGE;
        if (e.width_px == 0u || !(e.hfov_deg > 0.0f) || e.hfov_deg >= 180.0f)
            return NF_E_PARAM_RANGE;
    }
    return NF_OK;
}

NFStatus nf_pipeline_mem_bytes(const NFPipelineConfig *cfg, nf_u64 *out_bytes) {
    if (!cfg || !out_bytes) return NF_E_PARAM_NULL;

    nf_u64 total = 0;
    nf_u64 max_vol = 0;
    nf_u64 max_area = 0;

    if (cfg->n_views > 0u) {
        for (nf_u32 v = 0; v < cfg->n_views && v < NF_MAX_VIEWS; ++v) {
            NFViewParams vp;
            if (nf_view_params_derive_fov(&vp, cfg->view_plan[v].width_px,
                                          cfg->view_plan[v].hfov_deg,
                                          (cfg->view_plan[v].vfov_deg > 0.0f)
                                              ? cfg->view_plan[v].vfov_deg
                                              : NF_V_VIEW_VFOV_DEG) != NF_OK)
                continue;
            const nf_u64 area = (nf_u64)vp.width * vp.height;
            /* 视口图 2 + 有效位 2 + 视差 3 张 = 12 字节/像素 */
            total += area * 12ull;
            if (area > max_area) max_area = area;
            const nf_u64 vol = (nf_u64)cfg->engine.max_disp_px * area * 2ull;
            if (vol > max_vol) max_vol = vol;
        }
    } else {
        const nf_u64 area = (nf_u64)cfg->view.width * cfg->view.height;
        total = area * 12ull * NF_NUM_CAMERAS;
        max_area = area;
        max_vol = (nf_u64)cfg->engine.max_disp_px * area * 2ull;
    }

    /* 前向匹配缓冲按最大视口一份（2*4 + 4 + 1 = 13 字节/像素） */
    total += max_area * 13ull;
    /* SGBM cost volume：串行一份 */
    total += max_vol * (cfg->thread_cameras ? cfg->thread_cameras : 1);
    /* 体素表 */
    total += (nf_u64)NF_G_HASH_CAPACITY * 64ull;
    *out_bytes = total;
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
    p->cam_mask = (cfg->cam_enable_mask != 0u) ? cfg->cam_enable_mask : 0xFu;
    for (nf_u32 i = 0; i < NF_NUM_CAMERAS; ++i) {
        p->cams[i] = cams[i];
        p->poses[i] = poses[i];
        /* 外参自检失败 -> 该相机禁用(SPEC-ICD 失效语义表)。
           用位掩码记录, process 时跳过该相机。 */
        if (nf_pose_validate(&poses[i]) != NF_OK) p->cam_disabled |= (nf_u8)(1u << i);
    }

    /*
     * 视口表：n_views=0 时每相机一个视口（旧行为）；n_views>0 时用
     * cfg->view_plan 的 6 个虚拟视口。两条路共用同一套按视口的缓冲，
     * 避免"旧路径一套、6 视口另一套"导致行为分叉。
     */
    p->n_views = (cfg->n_views > 0u) ? cfg->n_views : NF_NUM_CAMERAS;
    if (p->n_views > NF_MAX_VIEWS) {
        nf_pipeline_destroy(p);
        return NF_E_PARAM_RANGE;
    }

    /* 旧路径的基准视口；显式给 view_hfov_deg 时重建（保持原语义）。 */
    NFViewParams view_base = cfg->view;
    if (cfg->view_hfov_deg > 0.0f) {
        NFViewParams derived;
        if (nf_view_params_derive_fov(&derived, cfg->view.width,
                                      cfg->view_hfov_deg,
                                      NF_V_VIEW_VFOV_DEG) == NF_OK) {
            view_base = derived;
        }
    }

    for (nf_u32 v = 0; v < p->n_views; ++v) {
        NFViewState &vs = p->views[v];
        vs.cam_id = (cfg->n_views > 0u) ? cfg->view_plan[v].cam_id : v;
        if (vs.cam_id >= NF_NUM_CAMERAS) {
            nf_pipeline_destroy(p);
            return NF_E_PARAM_RANGE;
        }

        if (cfg->n_views > 0u) {
            const NFViewPlanEntry &e = cfg->view_plan[v];
            NFViewParams vp;
            st = nf_view_params_derive_fov(&vp, e.width_px, e.hfov_deg,
                                           (e.vfov_deg > 0.0f)
                                               ? e.vfov_deg
                                               : NF_V_VIEW_VFOV_DEG);
            if (st != NF_OK) { nf_pipeline_destroy(p); return st; }
            vp.yaw_deg = e.yaw_deg;
            vs.vp = vp;
        } else {
            vs.vp = view_base;
            vs.vp.yaw_deg = cfg->view_yaw_deg[vs.cam_id];
        }

        /* 本视口的相机必须被使能，且外参自检通过 */
        const nf_u32 cam_bit = 1u << vs.cam_id;
        if (!(p->cam_mask & cam_bit) ||
            (p->cam_disabled & (nf_u8)cam_bit)) {
            vs.usable = 0u;
            continue;
        }

        const nf_u32 vw = vs.vp.width, vh = vs.vp.height;
        st = nf_lut_alloc(&vs.lut, vw, vh);
        if (st != NF_OK) { nf_pipeline_destroy(p); return st; }

        NFLutStats stats;
        st = nf_lut_build(&vs.lut, &p->cams[vs.cam_id], &p->poses[vs.cam_id],
                          &vs.vp, p->cfg.body_mask[vs.cam_id], &stats);
        if (st == NF_E_LUT_EMPTY) {
            /* 该虚拟视口完全落在本相机视场外 —— 正常工况。
             * 例：RV 相机 yaw=0（朝车头）时车尾鱼眼根本拍不到。
             * 只禁这个视口，不牵连同相机的另一个视口。 */
            nf_log_msg(NF_LOG_WARN,
                       "view%u (cam%u) yaw=%.0f° 完全不可见，本视口禁用",
                       v, vs.cam_id, (double)vs.vp.yaw_deg);
            nf_lut_destroy(&vs.lut);
            vs.usable = 0u;
            continue;
        } else if (st != NF_OK) {
            nf_log_msg(NF_LOG_ERROR,
                       "view%u (cam%u) LUT 构建失败: %s (视口 %ux%u f=%.2f yaw=%.1f)",
                       v, vs.cam_id, nf_status_str(st), vw, vh,
                       (double)vs.vp.f, (double)vs.vp.yaw_deg);
            nf_pipeline_destroy(p); return st;
        }

        for (int k = 0; k < 2; ++k) {
            vs.img[k] = (nf_u8 *)std::malloc((size_t)vw * vh);
            vs.val[k] = (nf_u8 *)std::malloc((size_t)vw * vh);
            if (!vs.img[k] || !vs.val[k]) {
                nf_pipeline_destroy(p);
                return NF_E_ENGINE_OOM;
            }
        }
        vs.dmap.version = NF_DISPARITY_MAP_VERSION;
        st = nf_dmap_alloc(&vs.dmap, vw, vh);
        if (st != NF_OK) { nf_pipeline_destroy(p); return st; }

        vs.usable = 1u;
        if (vw > p->m_w) p->m_w = vw;
        if (vh > p->m_h) p->m_h = vh;
    }

    /* 前向路径的对应点缓冲：按最大视口面积分配，逐视口串行复用。 */
    if (p->m_w == 0u || p->m_h == 0u) {
        p->m_w = cfg->view.width;
        p->m_h = cfg->view.height;
    }
    {
        const size_t np = (size_t)p->m_w * p->m_h;
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

    /* 双基线状态机初始化（NEW_MOD §1.1） */
    if (cfg->enable_baseline_mgr) {
        const nf_f32 b_tgt = (cfg->b_target_mm > 0.0f) ? cfg->b_target_mm
                                                       : (nf_f32)NF_B_TARGET;
        nf_baseline_manager_init(&p->baseline_mgr, cfg->b_min_mm,
                                 cfg->b_max_mm, b_tgt);
        p->has_pose_init = 0u;
        p->bl_frame_id = 0u;
        std::memset(&p->curr_pose, 0, sizeof(p->curr_pose));
        p->curr_pose.R[0] = p->curr_pose.R[4] = p->curr_pose.R[8] = 1.0f;
        nf_log_msg(NF_LOG_INFO, "双基线状态机: b_min=%.0f b_target=%.0f b_max=%.0f",
                   (double)cfg->b_min_mm, (double)b_tgt, (double)cfg->b_max_mm);
    }

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
    for (nf_u32 v = 0; v < p->n_views; ++v) {
        NFViewState &vs = p->views[v];
        nf_lut_destroy(&vs.lut);
        for (int k = 0; k < 2; ++k)
            std::free(vs.img[k]), std::free(vs.val[k]);
        std::free(vs.dmap.disp);
        std::free(vs.dmap.valid);
        std::free(vs.dmap.conf);
    }
    if (p->engine && g_engine_ops.deinit) g_engine_ops.deinit(p->engine);
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
    std::memset(out->view_mode, 0, sizeof(out->view_mode));
    std::memset(out->view_status, 0, sizeof(out->view_status));
    std::memset(out->num_points_per_view, 0, sizeof(out->num_points_per_view));
    std::memset(out->view_cam, 0, sizeof(out->view_cam));
    out->n_views = p->n_views;
    out->cloud.num_points = 0;
    out->cloud.overflow = 0;
    out->is_frozen = 0;
    out->filter_enabled = 0u;
    out->frame_id = in->frame_id;

    nf_voxel_clear(p->voxel);

    nf_u32 n_frozen = 0;
    nf_u32 n_enabled = 0;
    for (nf_u32 v = 0; v < p->n_views; ++v) {
        const NFViewState &vs = p->views[v];
        if (vs.usable && (p->cam_mask & (1u << vs.cam_id)) &&
            !(p->cam_disabled & (nf_u8)(1u << vs.cam_id)))
            ++n_enabled;
    }

    /* ---------------------------------------------------------------
     * 阶段 1: 逐视口 warp，OpenMP 按视口并行
     *
     * warp 是纯函数（读原图 + 本视口 LUT，写本视口缓冲），各视口无共享
     * 写状态。6 个视口里 FV/RV 各有两个 yaw 不同的 LUT，必须分别 warp；
     * 复用同一相机原图，不重复解码。
     * --------------------------------------------------------------- */
    NFStatus warp_st[NF_MAX_VIEWS];
    for (nf_u32 v = 0; v < p->n_views; ++v) warp_st[v] = NF_OK;

#ifdef NF_HAS_OPENMP
#pragma omp parallel for num_threads(NF_MAX_VIEWS) schedule(static)
#endif
    for (int vi = 0; vi < (int)p->n_views; ++vi) {
        const nf_u32 v = (nf_u32)vi;
        NFViewState &vs = p->views[v];
        if (!vs.usable || !(p->cam_mask & (1u << vs.cam_id)) ||
            (p->cam_disabled & (nf_u8)(1u << vs.cam_id)) ||
            !in->img[vs.cam_id])
            continue;
        NFImageView src;
        src.version = NF_IMAGE_VIEW_VERSION;
        src.data = in->img[vs.cam_id];
        src.width = in->width;
        src.height = in->height;
        src.stride = in->stride;
        src.valid_x0 = nullptr;
        src.timestamp_us = in->timestamp_us;
        src.frame_id = in->frame_id;
        warp_st[v] = nf_view_warp(&vs.lut, src.data, src.width, src.height,
                                  src.stride, vs.img[0], vs.val[0]);
    }

    const nf_f32 sigma_cap = (p->cfg.sigma_max_cm == 0.0f)
                           ? (nf_f32)NF_SIGMA_MAX_CM : p->cfg.sigma_max_cm;

    /*
     * 帧间位移：优先用车体系 t_veh，6 视口按各自 yaw 投影；旧调用方
     * 没有 t_veh 时回退到 t_cam（此时 6 视口只应配合同一 yaw 使用）。
     */
    const bool has_tveh =
        (std::fabs(in->t_veh[0]) + std::fabs(in->t_veh[1]) +
         std::fabs(in->t_veh[2])) > 1e-6f;
    nf_f32 tveh[3] = { in->t_veh[0], in->t_veh[1], in->t_veh[2] };
    if (!has_tveh && p->n_views > 0u) {
        /* 旧路径：t_cam[0] 是 cam0 视口系，转回车系（基向量见
           view_lut.cpp：f=(cos,sin), r=(sin,-cos)）。 */
        const nf_f32 yaw = p->views[0].vp.yaw_deg * kPiF / 180.0f;
        const nf_f32 cy = std::cos(yaw), sy = std::sin(yaw);
        tveh[0] = in->t_cam[0][0] * sy + in->t_cam[0][2] * cy;
        tveh[1] = -in->t_cam[0][0] * cy + in->t_cam[0][2] * sy;
        tveh[2] = 0.0f;
    }

    /* 6 视口即使调用方没给 t_veh，也必须用从 cam0 反算出的车体 t，
       否则同一相机的两个视口会共用同一个 t_cam（±45 会错）。 */
    const bool use_vehicle_t = has_tveh || (p->n_views > NF_NUM_CAMERAS);

    /* 双基线状态机每帧只推一次（同一辆车的同一次运动），逐视口复用决策。 */
    bool bl_ready = false;
    if (p->cfg.enable_baseline_mgr) {
        if (!p->has_pose_init) {
            std::memset(&p->curr_pose, 0, sizeof(p->curr_pose));
            p->curr_pose.R[0] = p->curr_pose.R[4] = p->curr_pose.R[8] = 1.0f;
            p->has_pose_init = 1u;
        } else {
            p->curr_pose.t[0] += tveh[0];
            p->curr_pose.t[1] += tveh[1];
            p->curr_pose.t[2] += tveh[2];
        }
        nf_baseline_manager_push(&p->baseline_mgr, &p->curr_pose,
                                 (nf_u64)in->frame_id * 200000ull,
                                 in->frame_id);
        nf_baseline_manager_decide(&p->baseline_mgr, &p->last_bl_dec);
        ++p->bl_frame_id;
        bl_ready = true;
    }

    for (nf_u32 v = 0; v < p->n_views; ++v) {
        NFViewState &vs = p->views[v];
        const nf_u32 cam = vs.cam_id;
        out->view_cam[v] = cam;

        /* 外参自检失败优先于 LUT 空：否则坏位姿会被报成"视口不可见" */
        if (p->cam_disabled & (nf_u8)(1u << cam)) {
            out->view_status[v] = NF_E_EXTRINSIC_INVALID;
            out->cam_status[cam] = NF_E_EXTRINSIC_INVALID;
            ++n_frozen;
            continue;
        }
        if (!vs.usable) {
            out->view_status[v] = NF_E_LUT_EMPTY;
            if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = NF_E_LUT_EMPTY;
            continue;
        }
        if (!(p->cam_mask & (1u << cam))) {
            out->view_status[v] = NF_E_CAM_NOT_CALIBRATED;
            continue;
        }

        const nf_u32 vw = vs.vp.width, vh = vs.vp.height;
        const nf_f32 yaw = vs.vp.yaw_deg * kPiF / 180.0f;
        const nf_f32 cy = std::cos(yaw), sy = std::sin(yaw);

        /* 视口系 t：t_x = t·r, t_z = t·f（r=(sin,-cos), f=(cos,sin)） */
        nf_f32 t_view[3] = { 0.0f, 0.0f, 0.0f };
        if (use_vehicle_t) {
            t_view[0] = tveh[0] * sy - tveh[1] * cy;
            t_view[2] = tveh[0] * cy + tveh[1] * sy;
        } else {
            t_view[0] = in->t_cam[cam][0];
            t_view[1] = in->t_cam[cam][1];
            t_view[2] = in->t_cam[cam][2];
        }

        NFGateDecision dec;
        NFStatus st = NF_OK;
        if (bl_ready) {
            dec.mode = p->last_bl_dec.mode;
            dec.baseline_mm = p->last_bl_dec.baseline_mm;
            /* 方向兜底：近前向视口强制 FORWARD；±45° 的 σ=0.707 仍走 STEREO */
            const nf_f32 tvn = std::sqrt(t_view[0] * t_view[0] +
                                        t_view[1] * t_view[1] +
                                        t_view[2] * t_view[2]);
            if (tvn > 1e-6f) {
                const nf_f32 sig = std::sqrt(t_view[0] * t_view[0] +
                                             t_view[1] * t_view[1]) / tvn;
                if (dec.mode == NF_MODE_STEREO && sig <= NF_SIGMA_FORWARD_MAX)
                    dec.mode = NF_MODE_FORWARD;
            }
        } else {
            st = nf_gate_decide(t_view, in->frame_gap,
                                p->cfg.b_min_mm, p->cfg.b_max_mm,
                                NF_SIGMA_STEREO_MIN, NF_SIGMA_FORWARD_MAX,
                                p->cfg.gap_max_frames, &dec);
        }
        if (st != NF_OK) {
            out->view_status[v] = st;
            if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = st;
            continue;
        }

        /*
         * SfM 极线判据：纯平移下所有对应点都在以**极点**为中心的射线上。
         * 极点由 t_view 投影得到：u_e = cx + f·t_x/t_z。
         *   |t_z|/|t| > 0.25  -> 极点在有限位置（前后 ±45/±135、正视前向）
         *                        用径向膨胀/收缩匹配，FoE=极点；
         *   |t_z|≈0          -> 极点在无穷远（左右 ±90），极线近水平，
         *                        才用 SGBM。
         * 不能用 σ（横向分量）直接判：±45 的 σ=0.707，会被误判成 SGBM，
         * 而它的极线是 45° 斜线，SGBM 的水平搜索结构性错配。
         */
        const float tvn0 = std::sqrt(t_view[0] * t_view[0] +
                                     t_view[1] * t_view[1] +
                                     t_view[2] * t_view[2]);
        const bool epi_forward =
            (tvn0 > 1e-6f) &&
            (std::fabs(t_view[2]) / tvn0 > 0.25f);
        if (epi_forward && (dec.mode == NF_MODE_STEREO ||
                            dec.mode == NF_MODE_HYBRID))
            dec.mode = NF_MODE_FORWARD;

        out->view_mode[v] = dec.mode;
        if (out->mode[cam] == NF_MODE_STEREO) out->mode[cam] = dec.mode;

        if (!nf_mode_produces_depth(dec.mode)) {
            out->view_status[v] = (dec.mode == NF_MODE_FROZEN_NEAR) ? NF_E_BASELINE_IDLE
                                 : (dec.mode == NF_MODE_FROZEN_FAR)  ? NF_E_BASELINE_OVER
                                 : NF_E_BASELINE_DEGRADED;
            out->cam_status[cam] = out->view_status[v];
            ++n_frozen;
            continue;
        }
        if (!in->img[cam]) {
            out->view_status[v] = NF_E_PARAM_NULL;
            if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = NF_E_PARAM_NULL;
            continue;
        }
        if (warp_st[v] != NF_OK) {
            out->view_status[v] = warp_st[v];
            if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = warp_st[v];
            continue;
        }

        const NFPose *pose = &p->poses[cam];

        /* ---------------- 前向：径向膨胀 + 解析测距 ---------------- */
        if (dec.mode == NF_MODE_FORWARD) {
            if (!in->img_ref[cam]) {
                out->view_status[v] = NF_E_BASELINE_IDLE;
                if (out->cam_status[cam] == NF_OK)
                    out->cam_status[cam] = NF_E_BASELINE_IDLE;
                continue;
            }
            NFStatus st_r = nf_view_warp(&vs.lut, in->img_ref[cam],
                                         in->width, in->height, in->stride,
                                         vs.img[1], vs.val[1]);
            if (st_r != NF_OK) {
                out->view_status[v] = st_r;
                if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = st_r;
                continue;
            }

            /*
             * 极点 = 平移方向在视口上的投影。t_view z>0 时在图像前方，
             * z<0（后视 ±135）时在图像后方；径向匹配两种都能处理，
             * 只是 rho 会膨胀/收缩。z≈0 时不应走到这里（上面已判 SGBM）。
             */
            NFFoe foe = { vs.vp.cx, vs.vp.cy };
            if (std::fabs(t_view[2]) > 1e-6f) {
                foe.u = vs.vp.cx + vs.vp.f * t_view[0] / t_view[2];
                foe.v = vs.vp.cy + vs.vp.f * t_view[1] / t_view[2];
            }
            const float tmm = std::sqrt(t_view[0] * t_view[0] +
                                        t_view[1] * t_view[1] +
                                        t_view[2] * t_view[2]);
            /* 极点到四角的最大半径；极点可能在图外，不能用半对角线 */
            float rho_max = 0.0f;
            const float corner_u[2] = { 0.0f, (float)vw - 1.0f };
            const float corner_v[2] = { 0.0f, (float)vh - 1.0f };
            for (int cu = 0; cu < 2; ++cu)
                for (int cv = 0; cv < 2; ++cv) {
                    const float du = corner_u[cu] - foe.u;
                    const float dv = corner_v[cv] - foe.v;
                    const float r = std::sqrt(du * du + dv * dv);
                    if (r > rho_max) rho_max = r;
                }
            const float r_lo = 0.5f;
            float r_hi = tmm * rho_max / NF_B_ZMIN_EFF;
            if (r_hi < r_lo + 1.0f) r_hi = r_lo + 1.0f;
            /* 上限 64 px：加大到 256 会在周期纹理上引入远端假峰，
               前视 90° 表面中位从 223 涨到 499 mm（实测）。近场宁可
               少出点，也不能用错误匹配换覆盖。 */
            if (r_hi > 64.0f) r_hi = 64.0f;

            nf_u32 fwd_pts = 0;
            if (tmm > 1e-3f) {
                NFStatus ms;
                if (p->cfg.forward_sample_step >= 2u) {
                    ms = nf_match_radial_fast(
                        vs.img[0], vs.img[1], vs.val[0], vw, vh, foe,
                        r_lo, r_hi, 2,
                        (p->cfg.forward_ncc_min > 0.0f)
                            ? p->cfg.forward_ncc_min : 0.6f,
                        p->cfg.forward_sample_step,
                        (p->cfg.forward_coarse_step > 0.0f)
                            ? p->cfg.forward_coarse_step : 2.0f,
                        p->m_pts, p->m_conf, p->m_valid);
                } else {
                    ms = nf_match_radial(vs.img[0], vs.img[1], vs.val[0],
                                         vw, vh, foe, r_lo, r_hi, 2, 0.6f,
                                         p->m_pts, p->m_conf, p->m_valid);
                }
                if (ms != NF_OK) {
                    out->view_status[v] = ms;
                    if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = ms;
                    continue;
                }

                /*
                 * 视口系 -> 车系：v_V = t_x·r + t_z·f。
                 * r=(sin,-cos)、f=(cos,sin) 是 view_lut 的基向量；
                 * 写成 Rz(+yaw) 会把 ±45° 的位移方向转错 45°。
                 */
                const float tt[3] = {
                    t_view[0] * sy + t_view[2] * cy,
                    -t_view[0] * cy + t_view[2] * sy,
                    0.0f
                };
                for (nf_u32 yy = 0; yy < vh; ++yy) {
                    for (nf_u32 xx = 0; xx < vw; ++xx) {
                        const nf_u32 i = yy * vw + xx;
                        if (!p->m_valid[i]) continue;

                        float d0[3], d1[3];
                        view_pixel_dir(&vs.vp, (nf_f32)xx + 0.5f,
                                       (nf_f32)yy + 0.5f, d0);
                        view_pixel_dir(&vs.vp, p->m_pts[i * 2],
                                       p->m_pts[i * 2 + 1], d1);
                        /* view_pixel_dir 已经返回**车系**方向（虚拟相机
                           基向量建在车系），不能再乘 pose->R；乘了就是把
                           FV 的 +X 光轴转成 -Y（实测前视 90° 表面中位
                           从 223 退化到 309）。 */
                        const float *a = d0;
                        const float *b = d1;
                        float dist = 0.0f;
                        nf_u8 okp = 0;
                        nf_forward_range(a, b, 1, tt, &dist, &okp);
                        if (!okp || dist <= 0.0f || dist > 50000.0f) continue;

                        const float P[3] = {
                            pose->t[0] - tt[0] + b[0] * dist,
                            pose->t[1] - tt[1] + b[1] * dist,
                            pose->t[2] - tt[2] + b[2] * dist
                        };
                        const float tn = std::sqrt(tt[0] * tt[0] +
                                                   tt[1] * tt[1] +
                                                   tt[2] * tt[2]);
                        if (tn > 1e-3f) {
                            const float dir[3] = { tt[0] / tn, tt[1] / tn,
                                                   tt[2] / tn };
                            if (nf_pt_in_foe(P, pose->t, dir, 0.9806f)) continue;
                        }
                        const nf_u8 sig = (tn > 1e-3f)
                            ? nf_compute_spatial_uncertainty_cm(
                                  vs.vp.f, tn, vs.vp.f * tn / dist, 0.5f, 0.02f)
                            : 255u;
                        if (sigma_cap > 0.0f && (nf_f32)sig > sigma_cap) continue;
                        if (insert_point(p, cam, P[0], P[1], P[2],
                                         p->m_conf[i], sig, dist, tn,
                                         vs.vp.f, nullptr) == NF_OK)
                            ++fwd_pts;
                    }
                }
            }
            out->view_status[v] = NF_OK;
            out->num_points_per_view[v] = fwd_pts;
            out->num_points_per_cam[cam] += fwd_pts;
            continue;
        }

        /* ---------------- 侧向/斜视：SGBM ---------------- */
        if (!g_engine_ops.init) {
            out->view_status[v] = NF_E_ENGINE_NOT_INIT;
            if (out->cam_status[cam] == NF_OK)
                out->cam_status[cam] = NF_E_ENGINE_NOT_INIT;
            continue;
        }
        if (!p->engine) {
            st = g_engine_ops.init(&p->engine, &p->cfg.engine);
            if (st != NF_OK) {
                out->view_status[v] = st;
                if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = st;
                continue;
            }
        }

        const bool mirror = t_view[0] > 0.0f;
        const nf_u8 *src_ref = in->img_ref[cam] ? in->img_ref[cam] : in->img[cam];
        if (in->img_ref[cam]) {
            NFStatus st_r = nf_view_warp(&vs.lut, src_ref, in->width,
                                         in->height, in->stride,
                                         vs.img[1], vs.val[1]);
            if (st_r != NF_OK) {
                out->view_status[v] = st_r;
                if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = st_r;
                continue;
            }
        } else {
            std::memcpy(vs.img[1], vs.img[0], (size_t)vw * vh);
        }
        if (mirror) {
            for (nf_u32 y = 0; y < vh; ++y) {
                nf_u8 *a = vs.img[0] + (size_t)y * vw;
                nf_u8 *b = vs.img[1] + (size_t)y * vw;
                for (nf_u32 x = 0; x < vw / 2; ++x) {
                    const nf_u8 t1 = a[x]; a[x] = a[vw - 1 - x]; a[vw - 1 - x] = t1;
                    const nf_u8 t2 = b[x]; b[x] = b[vw - 1 - x]; b[vw - 1 - x] = t2;
                }
            }
        }

        NFImageView lv, rv;
        lv.version = rv.version = NF_IMAGE_VIEW_VERSION;
        lv.data = vs.img[1]; lv.width = vw; lv.height = vh;
        lv.stride = vw; lv.valid_x0 = nullptr; lv.timestamp_us = in->timestamp_us;
        lv.frame_id = in->frame_id;
        rv = lv; rv.data = vs.img[0];

        NFDisparityRange dr;
        dr.version = NF_DISPARITY_RANGE_VERSION;
        dr.d_min = 0.0f;
        dr.d_max = vs.vp.f * dec.baseline_mm / NF_B_ZMIN_EFF;
        dr.b_mm = dec.baseline_mm;
        if ((nf_f32)p->cfg.engine.max_disp_px < dr.d_max) {
            out->view_status[v] = NF_E_ENGINE_RANGE;
            if (out->cam_status[cam] == NF_OK)
                out->cam_status[cam] = NF_E_ENGINE_RANGE;
            continue;
        }

        st = g_engine_ops.run(p->engine, &lv, &rv, &dr, &vs.dmap);
        if (st != NF_OK) {
            out->view_status[v] = st;
            if (out->cam_status[cam] == NF_OK) out->cam_status[cam] = st;
            continue;
        }

        nf_u32 inserted = 0;
        for (nf_u32 y = 0; y < vh; ++y) {
            for (nf_u32 x = 0; x < vw; ++x) {
                const size_t i = (size_t)y * vw + x;
                if (!vs.dmap.valid[i] || !vs.val[0][i]) continue;
                const float d = vs.dmap.disp[i];
                if (d < dr.d_min || d > dr.d_max) continue;

                float r0[3], r1[3];
                view_pixel_dir(&vs.vp, (float)x + 0.5f, (float)y + 0.5f, r0);
                const float u2 = mirror ? ((float)(vw - 1 - x) + 0.5f + d)
                                        : ((float)x + 0.5f - d);
                view_pixel_dir(&vs.vp, u2, (float)y + 0.5f, r1);

                /* 视口系 -> 车系（同一基向量逆变换） */
                const float tV_car[3] = {
                    t_view[0] * sy + t_view[2] * cy,
                    -t_view[0] * cy + t_view[2] * sy,
                    0.0f
                };
                const float c1[3] = { -tV_car[0], -tV_car[1], -tV_car[2] };
                float s0 = 0.0f;
                nf_u8 ok = 0;
                const float c0[3] = { 0, 0, 0 };
                nf_triangulate_rays(c0, r0, c1, r1, 1, &s0, &ok);
                if (!ok) continue;

                const float Pv[3] = {
                    r0[0] * s0 + pose->t[0],
                    r0[1] * s0 + pose->t[1],
                    r0[2] * s0 + pose->t[2]
                };
                const float tn2 = std::sqrt(tV_car[0] * tV_car[0] +
                                            tV_car[1] * tV_car[1] +
                                            tV_car[2] * tV_car[2]);
                if (tn2 > 1e-3f) {
                    const float dir2[3] = { tV_car[0] / tn2,
                                            tV_car[1] / tn2,
                                            tV_car[2] / tn2 };
                    if (nf_pt_in_foe(Pv, pose->t, dir2, 0.9806f)) continue;
                }
                const nf_u8 sig2 = nf_compute_spatial_uncertainty_cm(
                    vs.vp.f, dec.baseline_mm, d, 0.5f, 0.02f);
                if (sigma_cap > 0.0f && (nf_f32)sig2 > sigma_cap) continue;
                if (insert_point(p, cam, Pv[0], Pv[1], Pv[2],
                                 vs.dmap.conf[i], sig2, s0,
                                 dec.baseline_mm, vs.vp.f, nullptr) == NF_OK)
                    ++inserted;
            }
        }
        out->view_status[v] = NF_OK;
        out->num_points_per_view[v] = inserted;
        out->num_points_per_cam[cam] += inserted;
    }

    /* 主导模式：按相机聚合点数最多的一路 */
    nf_i32 best = -1;
    for (nf_u32 c = 0; c < NF_NUM_CAMERAS; ++c) {
        if (!(p->cam_mask & (1u << c))) continue;
        if (best < 0 ||
            out->num_points_per_cam[c] > out->num_points_per_cam[best])
            best = (nf_i32)c;
    }
    out->dominant_mode = (best >= 0) ? out->mode[best] : NF_MODE_STEREO;

    if (n_enabled > 0u && n_frozen == n_enabled) {
        out->is_frozen = 1;
        out->cloud.num_points = 0;
        return NF_E_BASELINE_IDLE;
    }

    NFStatus st = nf_voxel_export_points(p->voxel, &p->cloud);

    if (p->filter && p->cloud.num_points > 0) {
        nf_voxel_filter_update_ego_motion(p->filter, tveh[0], -tveh[1]);
        const NFStatus fs = nf_voxel_filter_compress(
            p->filter,
            (const NFVoxelEntry *)p->cloud.points, p->cloud.num_points,
            p->dvf_zmap, p->dvf_zindex, p->dvf_pts, &p->dvf_stats);
        if (fs == NF_OK) {
            out->filter_enabled = 1u;
            out->filter_stats = p->dvf_stats;
            out->dvf_points = p->dvf_pts;
            out->dvf_n_points = p->dvf_stats.n_output;
            out->dvf_n_cells = p->dvf_n_cells;
            out->dvf_zmap = p->dvf_zmap;
            out->dvf_zindex = p->dvf_zindex;
        }
    }
    out->cloud.points = p->cloud.points;
    out->cloud.num_points = p->cloud.num_points;
    out->cloud.max_points = p->cloud.max_points;
    out->cloud.overflow = p->cloud.overflow;

    if (p->cfg.enable_d4) {
        nf_log_msg(NF_LOG_INFO,
                   "D4: 融合 %u 次, 残差超限 %u 次 (本帧累计)",
                   g_d4_fused, g_d4_rejected);
        g_d4_fused = 0;
        g_d4_rejected = 0;
    }

    p->last_frame_id = in->frame_id;
    return st;
}

} /* extern "C" */
