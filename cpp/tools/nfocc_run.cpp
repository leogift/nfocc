/**
 * @file nfocc_run.cpp
 * @brief 主程序: 读配置 -> 建 pipeline -> 回放/合成 -> 输出。
 *
 * 用途:
 *   - SIL 调试(macOS/Linux 上位机)
 *   - 录包重放回归
 *   - 系统测试的内存与耗时统计(V-BUILD-3/7)
 *
 * 用法:
 *   nfocc_run --playback run.nfp --frames 100 --stats
 *   nfocc_run --synthetic --frames 10 --stats        (无录包时的自测)
 */
#include "nfocc/nfocc_pipeline.h"
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_io.h"
#include "nfocc/util/input_source.h"
#include "nfocc/hanqtech_loader.h"

#ifdef NF_HAS_ZMQ
#include <zmq.h>
#endif

#include <memory>
#include <vector>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>

#ifdef NF_WITH_SGBM
#include "nfocc/nfocc_engine_bind.h"
#endif

/* ---- 平台内存统计: macOS 用 mach, Linux 用 /proc ---- */
#if defined(__APPLE__)
#include <mach/mach.h>
static double rss_mb() {
    struct mach_task_basic_info info;
    mach_msg_type_number_t cnt = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  (task_info_t)&info, &cnt) != KERN_SUCCESS) return -1.0;
    return (double)info.resident_size / (1024.0 * 1024.0);
}
#elif defined(__linux__)
#include <unistd.h>
#include <cstdio>
static double rss_mb() {
    FILE *f = std::fopen("/proc/self/statm", "r");
    if (!f) return -1.0;
    long total = 0, resident = 0;
    if (std::fscanf(f, "%ld %ld", &total, &resident) != 2) { std::fclose(f); return -1.0; }
    std::fclose(f);
    return (double)resident * (double)sysconf(_SC_PAGESIZE) / (1024.0 * 1024.0);
}
#else
static double rss_mb() { return -1.0; }
#endif

static double now_ms() {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* ---- 默认相机与位姿(SPEC-CALIB-CFG 定案值) ---- */
static void make_default_rig(NFCameraModel cams[4], NFPose poses[4]) {
    /* R 的列 = 相机轴在车系 (v_V = R·v_C) */
    static const float axis[4][3][3] = {
        /* FV: 朝 +X */ { {0,-1,0}, {0,0,-1}, {1,0,0} },
        /* RV: 朝 -X */ { {0, 1,0}, {0,0,-1}, {-1,0,0} },
        /* ML: 朝 +Y */ { {1, 0,0}, {0,0,-1}, {0,1,0} },
        /* MR: 朝 -Y */ { {-1,0,0}, {0,0,-1}, {0,-1,0} },
    };
    static const float pos[4][3] = {
        { NF_VEH_REAR_TO_FRONT, 0.0f, NF_POS_CAM_Z },
        { -(NF_VEH_LEN - NF_VEH_REAR_TO_FRONT), 0.0f, NF_POS_CAM_Z },
        { 0.0f,  NF_VEH_WID * 0.5f, NF_POS_CAM_Z },
        { 0.0f, -NF_VEH_WID * 0.5f, NF_POS_CAM_Z },
    };

    for (int c = 0; c < 4; ++c) {
        cams[c].version = NF_CAMERA_MODEL_VERSION;
        cams[c].kind = NF_CAM_MODEL_KB;
        cams[c].cam_id = (nf_u8)c;
        cams[c].f = NF_INF_F;
        cams[c].cx = NF_INF_CX;
        cams[c].cy = NF_INF_CY;
        cams[c].k1 = NF_INF_K1; cams[c].k2 = NF_INF_K2;
        cams[c].k3 = NF_INF_K3; cams[c].k4 = NF_INF_K4;
        cams[c].r_valid = NF_INF_R_VALID;
        cams[c].max_theta = NF_INF_MAX_THETA;

        poses[c].version = NF_POSE_VERSION;
        poses[c].cam_id = (nf_u8)c;
        /* 行优先: poses[c].R[row*3+col] = axis[c][col][row] */
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                poses[c].R[row * 3 + col] = axis[c][col][row];
        poses[c].t[0] = pos[c][0];
        poses[c].t[1] = pos[c][1];
        poses[c].t[2] = pos[c][2];
        poses[c].timestamp_us = 0;
    }
}

/* ---- 合成纹理: 必须是 3D 的确定性函数 ----
   随机噪声会让同一物点在两帧灰度不同, 匹配器"成功返回但全错"(实测 133px -> 3.6px)。 */
static void synth_image(nf_u8 *buf, nf_u32 w, nf_u32 h, float phase) {
    for (nf_u32 y = 0; y < h; ++y) {
        for (nf_u32 x = 0; x < w; ++x) {
            double v = 128.0
                + 60.0 * std::sin(x * 0.031 + phase)
                + 50.0 * std::sin(y * 0.024 - phase)
                + 40.0 * std::sin((x + y) * 0.017);
            v = v < 0 ? 0 : (v > 255 ? 255 : v);
            buf[(size_t)y * w + x] = (nf_u8)(v + 0.5);
        }
    }
}

static void log_to_stdout(NFLogLevel lv, const char *txt, void *u) {
    (void)u;
    static const char *nm[] = {"ERR ", "WARN", "INFO", "DBG "};
    std::printf("    [%s] %s\n", nm[lv], txt);
}

int main(int argc, char **argv) {
    int  n_frames   = 5;
    bool do_stats   = false;
    bool synthetic  = false;
    const char *playback = nullptr;
    const char *ws_base = nullptr;
    nf_u32 ws_frame = 164;
    const char *zmq_in = nullptr;
    const char *zmq_out = nullptr;
    float move_mm = NF_B_MIN;
    bool forward = false;
    bool viewplan = false;
    float hfov_deg = NF_V_VIEW_HFOV_DEG;
    bool no_body_mask = false;       /* --no-body-mask: 关闭 D1 */
    float sigma_max_cm = 0.0f;       /* --sigma-max: σ 截断门限(cm)，负=关闭 */
    bool  filter_on = false;         /* --filter: 启用 REV-003 五阶段精简 */
    nf_u32 filter_cap = 0;           /* --filter-cap: 覆盖配额(默认 15000) */
    bool d4_on = false;              /* --d4: 启用双基线 + D4 */
    bool bl_on = false;              /* --baseline-mgr: 启用双基线状态机 */
    float speed_kmh = 5.0f;          /* --speed: 车速, 驱动 yaw_rate 积分 */
    float yaw_rate_dps = 0.0f;       /* --yaw-rate: 偏航率 deg/s */
    std::vector<nf_u8> mask_buf;     /* D1 掩码存储，4 路连续 */

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) n_frames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--stats")) do_stats = true;
        else if (!std::strcmp(argv[i], "--synthetic")) synthetic = true;
        else if (!std::strcmp(argv[i], "--playback") && i + 1 < argc) playback = argv[++i];
        else if (!std::strcmp(argv[i], "--hanqtech") && i + 1 < argc)
            ws_base = argv[++i];
        else if (!std::strcmp(argv[i], "--ws-frame") && i + 1 < argc)
            ws_frame = (nf_u32)atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--zmq-in") && i + 1 < argc) zmq_in = argv[++i];
        else if (!std::strcmp(argv[i], "--zmq-out") && i + 1 < argc) zmq_out = argv[++i];
        else if (!std::strcmp(argv[i], "--move") && i + 1 < argc) move_mm = (float)atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--forward")) forward = true;
        else if (!std::strcmp(argv[i], "--viewplan")) viewplan = true;
        else if (!std::strcmp(argv[i], "--no-body-mask")) no_body_mask = true;
        else if (!std::strcmp(argv[i], "--sigma-max") && i + 1 < argc)
            sigma_max_cm = (float)atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--filter")) filter_on = true;
        else if (!std::strcmp(argv[i], "--filter-cap") && i + 1 < argc)
            filter_cap = (nf_u32)atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--d4")) d4_on = true;
        else if (!std::strcmp(argv[i], "--baseline-mgr")) bl_on = true;
        else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc) speed_kmh = (float)atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--yaw-rate") && i + 1 < argc) yaw_rate_dps = (float)atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hfov") && i + 1 < argc)
            hfov_deg = (float)atof(argv[++i]);
        else { std::printf("未知参数: %s\n", argv[i]); return 2; }
    }

    NFCameraModel cams[4];
    NFPose poses[4];
    make_default_rig(cams, poses);

    NFPipelineConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.version = 1;
    cfg.engine.version = NF_ENGINE_CONFIG_VERSION;
    cfg.engine.kind = NF_ENGINE_SGBM;
    cfg.engine.max_disp_px = NF_E_MAX_DISP;
    cfg.engine.downscale = NF_E_DOWNSCALE;
    cfg.engine.conf_thresh = 0.0f;
    cfg.engine.block_size = NF_E_BLOCK;
    cfg.engine.uniqueness = NF_E_UNIQUENESS;
    cfg.engine.speckle_window = NF_E_SPECKLE_WINDOW;

    /* 视口宽度必须与**图像宽度 / 降采样倍数**一致。
     * 早期固定 960（对应理想 1920 图降采样 2 倍）; 接入真实 1280x966 后
     * 应为 640, 否则 LUT 采样位置与实际图像网格错位。
     * 注意 cfg.view 在此处定稿, 而输入源要到后面才创建, 故先用常量,
     * 待 --hanqtech 分支拿到真实尺寸后再覆盖(见下方)。 */
    const nf_u32 view_w0 = NF_V_VIEW_W;
    NFViewParams vp;
    NFStatus st = nf_view_params_derive_fov(&vp, view_w0, hfov_deg,
                                            NF_V_VIEW_VFOV_DEG);
    if (st != NF_OK) { std::printf("视口参数非法: %s\n", nf_status_str(st)); return 1; }
    cfg.view = vp;
    cfg.view_hfov_deg = hfov_deg;

    /* **B_MAX 必须与当前视口 f 自洽**: B_MAX = numDisp * Z_MIN / f。
     * 视口尺寸/张角一变, f 就变, B_MAX 的可用上限跟着变。
     * 这里先按默认视口算一次; --hanqtech 分支按真实图像再算一次。
     * 漏了这一步, nf_params_validate 的 R2 判据会直接拒绝创建 pipeline。 */
    {
        NFViewParams vb;
        if (nf_view_params_derive_fov(&vb, NF_V_VIEW_W, hfov_deg,
                                      NF_V_VIEW_VFOV_DEG) == NF_OK) {
            const float lim = (float)NF_E_MAX_DISP * NF_B_ZMIN_EFF / vb.f;
            if ((float)NF_B_MAX > lim) {
                cfg.b_max_mm = lim * 0.98f;
                std::printf("B_MAX 由 %.0f 调到 %.0f（受 numDisp/f 限制）\n",
                            (double)NF_B_MAX, (double)cfg.b_max_mm);
            }
        }
    }

    st = nf_voxel_grid_init(&cfg.grid, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                            NF_G_Y_MIN, NF_G_Y_MAX, NF_G_Z_MIN, NF_G_Z_MAX);
    if (st != NF_OK) { std::printf("体素网格非法: %s\n", nf_status_str(st)); return 1; }

    cfg.b_min_mm = NF_B_MIN;
    if (cfg.b_max_mm <= 0.0f) cfg.b_max_mm = NF_B_MAX;
    cfg.sigma_d_px = NF_T_SIGMA_D;
    cfg.sigma_b_ratio = NF_T_SIGMA_B_RATIO;
    cfg.gap_max_frames = NF_GAP_MAX_FRAMES;
    cfg.max_points = NF_MAX_POINTS;
    cfg.thread_cameras = 1;

    std::printf("=== NFOCC ===\n");
    std::printf("视口 %ux%u  f=%.2f  环带 %.0f-%.0f°\n",
                cfg.view.width, cfg.view.height, cfg.view.f,
                NF_V_RING_EL_MIN_DEG, NF_V_RING_EL_MAX_DEG);
    std::printf("体素 %ux%ux%u  vox=%dmm\n", cfg.grid.nx, cfg.grid.ny, cfg.grid.nz,
                cfg.grid.vox_mm);

    nf_log_set(log_to_stdout, nullptr);

    nf_u64 mem_est = 0;
    nf_pipeline_mem_bytes(&cfg, &mem_est);
    std::printf("内存预算(解析): %.1f MB\n", (double)mem_est / 1e6);
    const double rss0 = rss_mb();
    if (rss0 >= 0) std::printf("启动 RSS: %.1f MB\n", rss0);

#ifdef NF_WITH_SGBM
    nf_pipeline_bind_sgbm();
    std::printf("引擎: SGBM 已绑定\n");
#else
    std::printf("引擎: 未绑定(仅前向路径可用)\n");
#endif

    NFPipeline *pipe = nullptr;
    /* --viewplan: 6 视口规划 -> 前后相机各偏 30°(取 +30 一侧为主组) */
    if (viewplan) {
        /* 虚拟相机绝对 yaw（cam_id: 0=FV 1=RV 2=ML 3=MR）
         *   FV +45°  RV +135°  ML +90°  MR −90°
         * 取每组的一个代表；完整 6 视口表见 nfocc_viewplan。 */
        const nf_f32 yaws[4] = { 45.0f, 135.0f, 90.0f, -90.0f };
        for (int c = 0; c < 4; ++c) cfg.view_yaw_deg[c] = yaws[c];
        std::printf("视口规划(虚拟相机): FV+45 RV+135 ML+90 MR-90, 视锥 %.0f°\n", hfov_deg);
    }

    /* ============================================================
     * 输入源必须在 pipeline **之前**创建。
     *
     * 原因: hanqtech 的真实标定会覆盖 cams/poses, 且真实图像尺寸
     * (1280x966) 决定视口宽度(640) 与 B_MAX —— 这些都必须在
     * nf_pipeline_create 定稿 cfg 之前完成。
     * 早期顺序反了, 结果 --hanqtech 的标定与视口设置全部被忽略,
     * 但程序照常运行、照常出点, 只有深度整体错位。
     * ============================================================ */
    /* ---- 输入源: 合成 / 文件 / ZMQ 三选一 ---- */
    const nf_u32 W = NF_IMG_W, H = NF_IMG_H;
    std::unique_ptr<nf::InputSource> source;
    if (zmq_in) {
#ifdef NF_HAS_ZMQ
        source = nf::MakeZmqSource(zmq_in, W, H, 2000);
        if (!source) { std::printf("ZMQ 源打开失败: %s\n", zmq_in); return 1; }
#else
        std::printf("构建时未启用 ZeroMQ\n");
        return 1;
#endif
    } else if (ws_base) {
        NFWSCalib wscal;
        std::memset(&wscal, 0, sizeof(wscal));
        source = nf::MakehanqtechSource(ws_base, ws_frame, n_frames, &wscal);
        if (!source) { std::printf("hanqtech 数据加载失败: %s\n", ws_base); return 1; }
        /* **用真实标定覆盖理想外参** —— 这是接入真实数据的关键一步。
         * hanqtech 的相机安装角与理想外参差别很大:
         *   FV 俯角 22.7°, MVL/MVR 俯角 74°, RV 俯角 37°。 */
        for (int c = 0; c < 4; ++c) {
            cams[c] = wscal.cam[c];
            poses[c] = wscal.pose[c];
        }
        std::printf("真实标定: ");
        for (int c = 0; c < 4; ++c) std::printf("%s ", wscal.name[c]);
        std::printf("\n");
        /* **视口宽度必须按真实图像重算**: 1280/2 = 640。
         * 若仍用 960, LUT 会在错误位置采样, warp 出来的视口图是乱的 ——
         * 且不会报错, 只会让深度整体偏掉。
         *
         * 图像尺寸取自**标定的 width/height**（x2 因 cx 里已含 w/2 的换算），
         * 而不是 source->Width() —— 后者要到 ReadFrame 才填, 此刻是 0。 */
        /* hanqtech 图像固定 1280x966。loader 里已按此换算 cx/cy,
         * 此处需要原始宽高来定视口宽度。用 source->Width() 会得 0
         * （尚未 ReadFrame）, 故让 loader 暴露尺寸: 由 cx 反推不可靠,
         * 直接由加载器提供更清晰 —— 见 NFWSCalib.image_w/h。 */
        const nf_u32 rw = wscal.image_w;
        if (rw == 0) {
            std::printf("标定缺少图像尺寸\n");
            return 1;
        }
        /* 视口宽用常量 NF_V_VIEW_W（1024），不再按 rw/2 降到 640 ——
         * 那会让 f 掉到 85.7，5m 精度直接不达标。源图 1280 > 1024 仍够采样。 */
        const nf_u32 vw = (rw < (nf_u32)NF_V_VIEW_W) ? rw : (nf_u32)NF_V_VIEW_W;
        NFViewParams vp2;
        if (nf_view_params_derive_fov(&vp2, vw, hfov_deg,
                                      NF_V_VIEW_VFOV_DEG) == NF_OK) {
            cfg.view = vp2;
            std::printf("真实图像 %ux%u -> 视口 %ux%u (f=%.2f)\n",
                        wscal.image_w, wscal.image_h,
                        vp2.width, vp2.height, (double)vp2.f);
        }

        /**
         * **D1 车身掩码**：车体轮廓（矩形占位 5000x2000）投影到每路原图，
         * 生成逐像素掩码，LUT 构建期据此剔除车身像素。
         *
         * 用矩形占位而不是真实车体轮廓：真实轮廓要整车尺寸图，
         * 当前没有。矩形会**过度屏蔽**（后视镜、轮眉凸起处本不是车身
         * 却在内），所以开启后点数会偏少 —— 这是已知代价，
         * 换成真实多边形后消失。
         */
        if (!no_body_mask) {
            const float half_l = NF_VEH_LEN * 0.5f;
            const float front  = NF_VEH_REAR_TO_FRONT;
            const float rear   = front - NF_VEH_LEN;
            const float half_w = NF_VEH_WID * 0.5f;
            /* 车体系俯视矩形，顺时针 */
            const nf_f32 poly[8] = {
                front,  -half_w,   /* 右前 */
                front,   half_w,   /* 左前 */
                rear,    half_w,   /* 左后 */
                rear,   -half_w,   /* 右后 */
            };
            const size_t msz = (size_t)wscal.image_w * wscal.image_h;
            mask_buf.assign(msz * 4u, 0);
            nf_u32 tot_cov = 0;
            for (int c = 0; c < 4; ++c) {
                nf_u32 cov = 0;
                nf_mask_build_vehicle(&cams[c], &poses[c],
                                      wscal.image_w, wscal.image_h,
                                      poly, 4u, mask_buf.data() + (size_t)c * msz,
                                      &cov);
                tot_cov += cov;
                std::printf("  D1 cam%d 车身掩码覆盖 %u 像素 (%.1f%%)\n",
                            c, cov, 100.0 * (double)cov / (double)msz);
            }
            (void)half_l;
            for (int c = 0; c < 4; ++c)
                cfg.body_mask[c] = tot_cov
                    ? mask_buf.data() + (size_t)c * msz : nullptr;
            std::printf("D1 车身掩码已生成 (矩形 %.0fx%.0f mm, 合计 %u 像素)\n",
                        (double)NF_VEH_LEN, (double)NF_VEH_WID, tot_cov);
        } else {
            std::printf("D1 车身掩码已关闭 (--no-body-mask)\n");
        }

        /* σ 硬截断门限：与 D1 无关，必须在 if/else 之外设置 */
        cfg.sigma_max_cm = sigma_max_cm;

        /* 双基线状态机（NEW_MOD §1.1） */
        if (bl_on) {
            cfg.enable_baseline_mgr = 1u;
            cfg.b_target_mm = (float)NF_B_TARGET;
            std::printf("双基线状态机已启用 (b_target=%.0f)\n",
                        (double)cfg.b_target_mm);
        }

        /* D4 双基线交叉验证（NEW_MOD §1.3） */
        if (d4_on) {
            cfg.enable_d4 = 1u;
            cfg.d4_tau_min_mm = 50.0f;
            std::printf("D4 交叉验证已启用 (门限下限 50mm, 实际随距离自适应)\n");
        }

        /* REV-003 五阶段精简 */
        if (filter_on) {
            nf_voxel_filter_default_config(&cfg.filter_cfg);
            if (filter_cap) cfg.filter_cfg.max_output_points = filter_cap;
            cfg.enable_voxel_filter = 1u;
            std::printf("REV-003 精简已启用 (配额 %u)\n",
                        cfg.filter_cfg.max_output_points);
        }
        std::printf("σ 截断门限 %.0f cm%s\n",
                    (double)(sigma_max_cm == 0.0f ? NF_SIGMA_MAX_CM : sigma_max_cm),
                    sigma_max_cm < 0.0f ? " (已关闭)" : "");
        /* B_MAX 必须随 f 重算: B_MAX = numDisp*Z_MIN/f */
        const float bmax_new = (float)NF_E_MAX_DISP * NF_B_ZMIN_EFF / cfg.view.f;
        if (cfg.b_max_mm > bmax_new) {
            std::printf("B_MAX %.0f -> %.0f (受 numDisp 限制)\n",
                        (double)cfg.b_max_mm, (double)bmax_new);
            cfg.b_max_mm = bmax_new * 0.98f;
        }
    } else if (playback) {
        source = nf::MakeFileSource(playback);
        if (!source) { std::printf("回放文件打开失败: %s\n", playback); return 1; }
    } else {
        source = nf::MakeSyntheticSource(W, H, n_frames, move_mm, forward);
    }
    std::printf("输入源: %s  %ux%u  每帧位移 %.0f mm\n",
                source->Name(), source->Width(), source->Height(), move_mm);

    st = nf_pipeline_create(&pipe, &cfg, cams, poses);
    if (st != NF_OK) {
        std::printf("pipeline 创建失败: %s\n", nf_status_str(st));
        return 1;
    }
    const double rss1 = rss_mb();
    if (rss1 >= 0) std::printf("初始化后 RSS: %.1f MB (增加 %.1f MB)\n",
                               rss1, rss1 - rss0);


    /* ---- 下游: 可选 ZMQ PUB ---- */
    void *out_ctx = nullptr;
    void *out_pub = nullptr;
    if (zmq_out) {
#ifdef NF_HAS_ZMQ
        out_ctx = zmq_ctx_new();
        out_pub = zmq_socket(out_ctx, ZMQ_PUB);
        int hwm = 4;
        zmq_setsockopt(out_pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));
        if (zmq_bind(out_pub, zmq_out) != 0) {
            std::printf("下游 bind 失败: %s\n", zmq_out);
            return 1;
        }
        std::printf("下游 PUB: %s\n", zmq_out);
#else
        std::printf("构建时未启用 ZeroMQ\n");
        return 1;
#endif
    }

    double t_total = 0.0;
    nf_u32 total_points = 0;
    nf_u8  last_filter_enabled = 0u;
    NFVoxelFilterStats last_filter_stats;
    std::memset(&last_filter_stats, 0, sizeof(last_filter_stats));
    for (int f = 0; f < n_frames; ++f) {
        NFFrameInput in;
        std::memset(&in, 0, sizeof(in));
        bool finished = false;
        const NFStatus src_st = source->ReadFrame(&in, &finished);
        if (finished) {
            std::printf("输入源结束于第 %d 帧\n", f);
            break;
        }
        if (src_st != NF_OK) {
            std::printf("帧 %d 读取失败: %s\n", f, nf_status_str(src_st));
            continue;      /* 单帧错误不中断整条流 */
        }

        NFFrameOutput out;
        std::memset(&out, 0, sizeof(out));
        out.version = NF_POINT_CLOUD_VERSION;

        const double t0 = now_ms();
        st = nf_pipeline_process(pipe, &in, &out);
        const double dt = now_ms() - t0;
        t_total += dt;

        if (do_stats || f == 0) {
            std::printf("帧 %3d: %s  主导模式=%-8s  点=%6u  %.1f ms%s\n",
                        f, nf_status_str(st), nf_mode_str(out.dominant_mode),
                        out.cloud.num_points, dt,
                        out.is_frozen ? "  [冻结]" : "");
            if (do_stats) {
                for (int c = 0; c < 4; ++c)
                    std::printf("         cam%d mode=%-8s status=%-24s pts=%u\n",
                                c, nf_mode_str(out.mode[c]),
                                nf_status_str(out.cam_status[c]),
                                out.num_points_per_cam[c]);
            }
        }
        total_points += out.cloud.num_points;
        if (out.filter_enabled) {
            last_filter_enabled = 1u;
            last_filter_stats = out.filter_stats;
        }

        /* DEPTH_STAT: 逐帧 z 统计 —— 判断点云深度是否合理, 而不只是点数 */
        if (out.cloud.num_points > 0) {
            nf_i32 zmin = 1 << 30, zmax = -(1 << 30);
            long long zsum = 0;
            for (nf_u32 q = 0; q < out.cloud.num_points; ++q) {
                const nf_i32 z = out.cloud.points[q].z_mm;
                if (z < zmin) zmin = z;
                if (z > zmax) zmax = z;
                zsum += z;
            }
            /* 同时统计到后轴中心的水平距离, 这才是判断"看到哪儿"的指标。
               z 在视口环带 10-60° 下本来就在地面附近(0.4~4m 处的地面)，
               单看 z 无法判断点云对错。 */
            double dsum = 0.0;
            nf_i32 dmin = 1 << 30, dmax = -(1 << 30);
            for (nf_u32 q = 0; q < out.cloud.num_points; ++q) {
                const double dx = out.cloud.points[q].x_mm;
                const double dy = out.cloud.points[q].y_mm;
                const int d = (int)std::sqrt(dx * dx + dy * dy);
                dsum += d;
                if (d < dmin) dmin = d;
                if (d > dmax) dmax = d;
            }
            std::printf("DEPTH_STAT f=%d: n=%u z=[%d,%d] mean=%.0f | 水平距=[%d,%d] mean=%.0f mm\n",
                        f, out.cloud.num_points, zmin, zmax,
                        (double)zsum / out.cloud.num_points,
                        dmin, dmax, dsum / out.cloud.num_points);
        }

        /* ---- 下游发布: 冻结也照常发（SPEC-DOWNSTREAM §1） ---- */
        if (out_pub) {
#ifdef NF_HAS_ZMQ
            NFOutHeader oh;
            std::memset(&oh, 0, sizeof(oh));
            oh.magic = NF_OUT_MAGIC;
            oh.version = NF_OUT_VERSION;
            oh.frame_id = out.frame_id;
            oh.timestamp_us = in.timestamp_us;     /* 曝光时刻, 不是完成时刻 */
            oh.latency_us = (nf_u64)(dt * 1000.0);
            oh.is_valid = out.is_frozen ? 0u : 1u;
            oh.status = (nf_u8)(st & 0xFF);
            oh.mode = (nf_u8)out.dominant_mode;
            nf_u8 ok_cams = 0;
            for (int c = 0; c < 4; ++c)
                if (out.num_points_per_cam[c] > 0) ++ok_cams;
            oh.num_cameras_ok = ok_cams;
            oh.num_points = out.cloud.num_points;
            oh.points_bytes = out.cloud.num_points * (nf_u32)sizeof(NFPoint);
            /* 基线与 σ: 下游据此判断精度（SPEC-DOWNSTREAM §2.1）。
               取四路里的中位数；全冻结时为 0，下游看到 base=0 即知不可用。 */
            {
                float bl = 0.0f;
                int nb = 0;
                for (int c = 0; c < 4; ++c) {
                    if (out.num_points_per_cam[c] > 0) {
                        bl += (float)std::sqrt(
                            (double)in.t_cam[c][0] * in.t_cam[c][0] +
                            (double)in.t_cam[c][1] * in.t_cam[c][1] +
                            (double)in.t_cam[c][2] * in.t_cam[c][2]);
                        ++nb;
                    }
                }
                oh.baseline_mm = nb ? bl / (float)nb : 0.0f;
                /* σ_Z = Z²·σ_d/(f·b_perp)。视口中心附近 b_perp≈b，
                   故用 σ_Z ≈ Z²·σ_d/(VF_VIEW·b) 在中距离 Z=2m 处的量级
                   作为"本帧精度"的代表值。下游只需要量级来设膨胀半径，
                   逐点 σ 在点云里另有承载（NFPoint.sigma_cm）。 */
                if (nb > 0 && oh.baseline_mm > 1e-3f) {
                    const float z_rep = 2000.0f;   /* mm */
                    oh.sigma_med_mm = z_rep * z_rep * NF_T_SIGMA_D /
                                      (NF_VF_VIEW * oh.baseline_mm);
                }
            }

            nf_u32 total_bytes = 0;
            if (nf_out_size(&oh, &total_bytes) == NF_OK) {
                zmq_send(out_pub, &oh, sizeof(oh), ZMQ_SNDMORE);
                zmq_send(out_pub, out.cloud.points,
                         (size_t)oh.points_bytes, 0);
            }
#endif
        }
    }

    std::printf("\n--- 统计 ---\n");
    std::printf("帧数 %d, 平均耗时 %.1f ms/帧\n", n_frames,
                n_frames ? t_total / n_frames : 0.0);
        if (last_filter_enabled) {
            std::printf("REV-003 精简: %u 点 -> 输出 %u 点, 占用柱体 %u%s\n",
                        last_filter_stats.n_input, last_filter_stats.n_output,
                        last_filter_stats.n_cells,
                        last_filter_stats.truncated ? " (已截断)" : "");
            std::printf("  各阶段: AABB %u -> FoE %u -> 地面 %u -> 外壳 %u\n",
                        last_filter_stats.n_after_roi,
                        last_filter_stats.n_after_foe,
                        last_filter_stats.n_after_ground,
                        last_filter_stats.n_after_shell);
        }
        std::printf("累计点数 %u (平均 %.0f/帧)\n", total_points,
                n_frames ? (double)total_points / n_frames : 0.0);
    const double rss2 = rss_mb();
    if (rss2 >= 0) std::printf("稳态 RSS: %.1f MB (预算 %.1f MB, 偏差 %.1f%%)\n",
                               rss2, (double)mem_est / 1e6,
                               mem_est ? ((rss2 * 1e6 - (double)mem_est) / (double)mem_est * 100.0) : 0.0);

    nf_pipeline_destroy(pipe);
#ifdef NF_HAS_ZMQ
    if (out_pub) { zmq_close(out_pub); zmq_ctx_destroy(out_ctx); }
#endif
    return 0;
}
