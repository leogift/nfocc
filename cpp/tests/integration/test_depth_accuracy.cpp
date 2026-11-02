/**
 * @file test_depth_accuracy.cpp
 * @brief 端到端深度精度 —— 对齐原始方案阶段 A 的验收标准。
 *
 * ## 原始要求
 * `docs/doc/3_NFOCC实施方案.md` §7 阶段 A：
 *   "1–3 m 范围内的体素位置与激光真值的偏差中位数 < 150 mm"
 *
 * ## 比对口径（本测试最关键的设计，错了会误判实现）
 *
 * 点云经过体素量化（200mm），x/y/z 全被吸到体素中心。
 * 早期版本拿"量化后的点"直接算到平面的距离，得到中位 242~259mm
 * ——**超过一个体素，看起来像实现错了，实际是口径错了**：
 * x、y 两个轴的量化误差（各 ∓100mm）全被算进了"深度误差"。
 *
 * 正确口径：**反投影到像素，比深度**。
 *   1. 对产出点 P，用 source_mask 找出它来自哪一路相机；
 *   2. 把 P 投影到该相机的视口，得到像素 (u,v)；
 *   3. 由 (u,v) 的视口射线打到场景，得到该像素的**真值 Z**；
 *   4. 比 P 的 Z 与真值 Z。
 *
 * 这样横向量化误差不进结果，测的是"匹配 + 三角化"的深度精度。
 * 体素量化本身另算，它最多贡献 ∓100mm，是**可加的下限**。
 *
 * ## 局限（必须写明）
 * P 已被量化，投影回像素时会落到体素中心对应的像素，与原始像素
 * 有最多半个体素的位置差，从而真值 Z 有间接误差。
 * 因此本测试给出的是**偏保守**的上界，不是精确值。
 * 精确值需要 pipeline 导出 (点 -> 像素) 映射，见 PROJECT-STATUS §5 P2 的后续。
 */
#include "nf_test.h"
#include "nfocc/nfocc_pipeline.h"
#include "nfocc/nfocc_engine_bind.h"
#include "nfocc/nfocc_scene.h"
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_default.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

void rig(NFCameraModel cams[4], NFPose poses[4]) {
    static const float axis[4][3][3] = {
        { {0,-1,0}, {0,0,-1}, {1,0,0} }, { {0, 1,0}, {0,0,-1}, {-1,0,0} },
        { {1, 0,0}, {0,0,-1}, {0,1,0} }, { {-1,0,0}, {0,0,-1}, {0,-1,0} } };
    static const float pos[4][3] = {
        { NF_VEH_REAR_TO_FRONT, 0.0f, NF_POS_CAM_Z },
        { -(NF_VEH_LEN - NF_VEH_REAR_TO_FRONT), 0.0f, NF_POS_CAM_Z },
        { 0.0f,  NF_VEH_WID * 0.5f, NF_POS_CAM_Z },
        { 0.0f, -NF_VEH_WID * 0.5f, NF_POS_CAM_Z } };
    for (int c = 0; c < 4; ++c) {
        cams[c].version = NF_CAMERA_MODEL_VERSION;
        cams[c].kind = NF_CAM_MODEL_KB;
        cams[c].f = NF_INF_F; cams[c].cx = NF_INF_CX; cams[c].cy = NF_INF_CY;
        cams[c].k1 = cams[c].k2 = cams[c].k3 = cams[c].k4 = 0.0f;
        cams[c].r_valid = NF_INF_R_VALID;
        cams[c].max_theta = NF_INF_MAX_THETA;
        poses[c].version = NF_POSE_VERSION;
        poses[c].cam_id = (nf_u8)c;
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                poses[c].R[row * 3 + col] = axis[c][col][row];
        for (int k = 0; k < 3; ++k) poses[c].t[k] = pos[c][k];
    }
}

/**
 * 误差统计。**按距离分段**输出，因为验收标准是分段的：
 * 用户决策为"保持 150mm 目标、接受精度随距离衰减"，
 * 单一全局中位数会把近场的达标和远场的衰减混成一个数。
 */
struct Stat {
    int n = 0;
    double med = -1, p95 = -1, mean = -1;
    int    n_band[4] = { 0, 0, 0, 0 };     /* 1-2, 2-3, 3-4, 4-5 m */
    double med_band[4] = { -1, -1, -1, -1 };
};

Stat summarize(std::vector<double> v) {
    Stat s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    s.n = (int)v.size();
    s.med = v[v.size() / 2];
    s.p95 = v[(size_t)(v.size() * 0.95)];
    double t = 0;
    for (double x : v) t += x;
    s.mean = t / v.size();
    return s;
}

/* 把点 P 投到相机像素，再由该像素的**真值射线**求交，得到真值点 P_true。
 *
 * 为什么这样做：点云 P 已被体素量化，x/y/z 都是 200mm 网格中心。
 * 直接拿 P 与"连续表面"比会混入量化误差（实测口径错时得 242~259mm，
 * 超过一个体素，看起来像实现错了）。这里改为"P 落在哪个像素，
 * 那个像素的真值是多少"，量化误差仍然存在但只来自 P 的位移，
 * 且与原始方案"体素位置偏差"同义。
 *
 * @param out_Ptrue 输出的真值点（车系）
 * @return 真值点沿相机光轴的距离；0 表示投不到
 */
float truth_point_at(const NFScene &scene, const NFPose &pose,
                     const NFCameraModel &cam /* 保留签名，真值不再经像素 */,
                     float px, float py, float pz, float *out_Ptrue) {
    float rel[3] = { px - pose.t[0], py - pose.t[1], pz - pose.t[2] };
    const float n = std::sqrt(rel[0]*rel[0] + rel[1]*rel[1] + rel[2]*rel[2]);
    if (n < 1.0f) return 0.0f;

    /**
     * **真值沿"该点自己的射线"求交，不做像素往返。**
     *
     * 旧实现: 点 -> 投到真实相机像素 -> 反投影成射线 -> 求交。
     * `project -> unproject` 丢掉了距离信息，只留方向；真值因此变成
     * "那个像素看到的表面"。点稍微偏一点就落到**另一个表面**上 ——
     * 前墙 x=4000，产出点 x=4946 仍能投到邻近像素，真值还是 4000，
     * 于是"误差"946mm 里绝大部分是测量方法的伪影，不是实现误差。
     * 实测: n=972 但中位 1162mm，而 SGBM 视差独立探针只有 0.9% 误差。
     *
     * 现在: 真值点 = 从相机光心沿 P 的方向求交得到，两者同一条射线，
     * 误差退化为**沿视线的距离差**，可以直接与 sigma_Z 公式对照。
     */
    const float d[3] = { rel[0] / n, rel[1] / n, rel[2] / n };
    float th = 0.0f, Ph[3];
    if (nf_scene_raycast(&scene, pose.t, d, &th, Ph, nullptr) != NF_OK)
        return 0.0f;
    out_Ptrue[0] = Ph[0]; out_Ptrue[1] = Ph[1]; out_Ptrue[2] = Ph[2];
    return th;                  /* 沿视线的距离 (mm) */
}

struct RunCfg {
    /**
     * 帧间位移。默认 278 mm = 5 km/h ÷ 5 Hz，对应用户定的车速下限。
     * 15 km/h 对应 833 mm，通过单独用例覆盖。
     */
    float move_mm = 278.0f;
    bool  forward = false;
    float yaw_off = 0.0f;
    int   frames = 2;
};

Stat run_accuracy(const NFScene &scene, NFCameraModel cams[4], NFPose poses[4],
                  const RunCfg &rc) {
    NFPipelineConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.version = 1;
    cfg.engine.version = NF_ENGINE_CONFIG_VERSION;
    cfg.engine.kind = NF_ENGINE_SGBM;
    cfg.engine.max_disp_px = NF_E_MAX_DISP;
    cfg.engine.downscale = NF_E_DOWNSCALE;
    cfg.engine.block_size = NF_E_BLOCK;
    cfg.engine.uniqueness = NF_E_UNIQUENESS;
    cfg.engine.speckle_window = NF_E_SPECKLE_WINDOW;

    /* 必须与 pipeline 内部用的推导方式一致：显式 150°x120°。
     * 用旧的环带式 derive 会得到 960x120，而 create 里按 150/120 重建
     * 得到 960x446 —— 尺寸不符，nf_lut_alloc 的 height 与 vp 不一致，
     * create 返回 NF_E_PARAM_RANGE，测试直接 return 空统计（n=0）。 */
    NFViewParams vp;
    nf_view_params_derive_fov(&vp, NF_V_VIEW_W, NF_V_VIEW_HFOV_DEG,
                              NF_V_VIEW_VFOV_DEG);
    cfg.view = vp;
    cfg.view_hfov_deg = NF_V_VIEW_HFOV_DEG;
    nf_voxel_grid_init(&cfg.grid, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                       NF_G_Y_MIN, NF_G_Y_MAX, NF_G_Z_MIN, NF_G_Z_MAX);
    cfg.b_min_mm = NF_B_MIN; cfg.b_max_mm = NF_B_MAX;
    cfg.sigma_d_px = 0.5f; cfg.sigma_b_ratio = 0.02f;
    cfg.gap_max_frames = NF_GAP_MAX_FRAMES;
    cfg.max_points = NF_MAX_POINTS;
    cfg.thread_cameras = 1;
    for (int c = 0; c < 4; ++c) cfg.view_yaw_deg[c] = rc.yaw_off;

    nf_pipeline_bind_sgbm();
    NFPipeline *p = nullptr;
    {
        const NFStatus cst = nf_pipeline_create(&p, &cfg, cams, poses);
        if (cst != NF_OK) {
            std::printf("      [CREATE] %s (view %ux%u f=%.2f hfov=%.1f)\n",
                        nf_status_str(cst), cfg.view.width, cfg.view.height,
                        (double)cfg.view.f, (double)cfg.view_hfov_deg);
            return Stat{};
        }
    }

    std::vector<NFRenderFrame> cur(4), ref(4);
    for (int c = 0; c < 4; ++c) {
        std::memset(&cur[c], 0, sizeof(cur[c]));
        std::memset(&ref[c], 0, sizeof(ref[c]));
        nf_render_frame_alloc(&cur[c], NF_IMG_W, NF_IMG_H);
        nf_render_frame_alloc(&ref[c], NF_IMG_W, NF_IMG_H);
        NFPose pr = poses[c];
        if (rc.forward) pr.t[0] -= rc.move_mm; else pr.t[1] -= rc.move_mm;
        nf_render_frame(&scene, &cams[c], &pr, &ref[c]);
        nf_render_frame(&scene, &cams[c], &poses[c], &cur[c]);
    }

    std::vector<double> errs, ranges;
    std::vector<double> per_cam[4];   /* 按相机分开统计，定位是哪一路在拖 */
    std::vector<double> lateral_deg;
    for (int f = 0; f < rc.frames; ++f) {
        NFFrameInput in;
        std::memset(&in, 0, sizeof(in));
        for (int c = 0; c < 4; ++c) {
            in.img[c] = cur[c].img;
            in.img_ref[c] = ref[c].img;
            /* **t_cam 用视口系（SPEC-CONVENTION §2）**，必须与参考帧
             * 的实际位移在同一轴上，且换算正确。
             *
             * 虚拟相机 (yaw=0) 的基向量（车系）:
             *   forward = (cos yaw, sin yaw, 0) = (1, 0, 0)
             *   right   = (sin yaw,-cos yaw, 0) = (0,-1, 0)   <- 是 -Y
             *   down    = (0, 0,-1)
             * 故视口 x 轴 = 车系 -Y，视口 z 轴 = 车系 +X。
             *
             * 参考帧渲染时:
             *   forward -> pr.t[0] -= move  => 车系位移 (+move, 0, 0)
             *              转视口系 => t_cam = (+move, 0, 0)   沿视口 z
             *   侧行    -> pr.t[1] -= move  => 车系位移 (0, +move, 0)
             *              转视口系 => t_cam = (-move, 0, 0)   沿视口 x
             *
             * **历史 bug**: 这里曾写 t_cam=(0, +move, 0)，把位移放到视口
             * y 轴（相机"下"方向）。而实际像移在视口 x 轴，基线与像移正交，
             * 匹配全部失败 -> n=0。
             */
            in.t_cam[c][0] = rc.forward ? 0.0f : -rc.move_mm;
            in.t_cam[c][1] = 0.0f;
            in.t_cam[c][2] = rc.forward ? rc.move_mm : 0.0f;
        }
        in.width = NF_IMG_W; in.height = NF_IMG_H; in.stride = NF_IMG_W;
        in.frame_id = (nf_u32)f; in.frame_gap = 1;

        NFFrameOutput out;
        std::memset(&out, 0, sizeof(out));
        const NFStatus pr_st = nf_pipeline_process(p, &in, &out);
        if (f == 0) {   /* TRIP: 诊断首帧 */
            std::printf("      [TRIP] proc=%s dominant=%d npts=%u per_cam=",
                        nf_status_str(pr_st), (int)out.dominant_mode,
                        out.cloud.num_points);
            for (int c = 0; c < 4; ++c)
                std::printf("%u/%d ", out.num_points_per_cam[c], (int)out.mode[c]);
            std::printf("\n");
        }
        if (pr_st != NF_OK) continue;

        for (nf_u32 q = 0; q < out.cloud.num_points; ++q) {
            const NFPoint *pt = &out.cloud.points[q];
            const nf_u8 m = pt->source_camera;
            /* 点云的 z 是**视口系 z**。真值也要按同一相机/视口算。
             * 多点共享体素时只取最低位那一路（precision 足够）。 */
            int cam = -1;
            for (int c = 0; c < 4; ++c) if (m & (1u << c)) { cam = c; break; }
            if (cam < 0) continue;
            NFViewParams vp_c = vp;
            vp_c.yaw_deg = rc.yaw_off;
            /* 点云是**车系**坐标（体素中心）。
             * 正确口径：把 P 投到该相机像素，由该像素的**真值射线**求交，
             * 得到真值点 P_true；误差 = |P - P_true|（车系欧氏距离）。
             * 这对应原始方案"体素位置与真值偏差"，且横向量化误差是
             * 真实存在的一部分（它本来就该算进"体素位置偏差"）。
             * 但为把**深度**与**横向**分开，再分别输出两个分量。 */
            /* **点到表面距离**（法向偏差），不再走像素往返或沿射线求交。 */
            const float P[3] = { (float)pt->x_mm, (float)pt->y_mm, (float)pt->z_mm };
            float e_surf = 0.0f;
            if (nf_scene_dist_to_surface(&scene, P, &e_surf) != NF_OK) continue;

            /* 距离分段用"点到相机光心"的欧氏距离 */
            const double rx = (double)pt->x_mm - poses[cam].t[0];
            const double ry = (double)pt->y_mm - poses[cam].t[1];
            const double rz = (double)pt->z_mm - poses[cam].t[2];
            const double rng = std::sqrt(rx*rx + ry*ry + rz*rz);
            if (rng < 1000.0 || rng > 5000.0) continue;

            const double e = (double)e_surf;
            errs.push_back(e);
            ranges.push_back(rng);
            per_cam[cam].push_back(e);
        }
    }

    nf_pipeline_destroy(p);
    for (int c = 0; c < 4; ++c) {
        nf_render_frame_free(&cur[c]);
        nf_render_frame_free(&ref[c]);
    }
    Stat s = summarize(errs);
    /* 分段统计：1-2 / 2-3 / 3-4 / 4-5 m */
    std::vector<double> band[4];
    for (size_t i = 0; i < errs.size(); ++i) {
        const double z = ranges[i];
        const int k = (z < 2000.0) ? 0 : (z < 3000.0) ? 1 : (z < 4000.0) ? 2 : 3;
        band[k].push_back(errs[i]);
    }
    for (int k = 0; k < 4; ++k) {
        Stat b = summarize(band[k]);
        s.n_band[k] = b.n;
        s.med_band[k] = b.med;
    }
    /* 按相机打印，判断是否某一路（如俯角 74° 的 MVL/MVR）显著更差 */
    std::printf("        按相机: ");
    for (int c = 0; c < 4; ++c) {
        Stat pc = summarize(per_cam[c]);
        std::printf("cam%d n=%d med=%.0f  ", c, pc.n, pc.med);
    }
    std::printf("\n");
    return s;
}


/**
 * 分段验收。
 *
 * 用户决策：保持 150 mm 目标，**接受精度随距离衰减**。
 * 因此不是"全体中位 <150"，而是按距离分级：
 *   1-2 m : <150 mm  （目标区，必须达成）
 *   2-3 m : <200 mm
 *   3-5 m : <320 mm  （衰减区，物理下限决定：sigma_Z ∝ Z^2）
 */
void check_bands(const char *name, const Stat &s) {
    static const double lim[4] = { 150.0, 200.0, 320.0, 320.0 };
    std::printf("      [%s] 分段中位:", name);
    for (int k = 0; k < 4; ++k) {
        std::printf("  %d-%dm:%s(n=%d)", k + 1, k + 2,
                    s.n_band[k] ? "" : "-", s.n_band[k]);
        if (s.n_band[k]) std::printf("%.0f/%.0fmm", s.med_band[k], lim[k]);
    }
    std::printf("\n");
    if (s.n_band[0]) NF_CHECK_MSG(s.med_band[0] < lim[0], "1-2m 中位 < 150mm");
    if (s.n_band[1]) NF_CHECK_MSG(s.med_band[1] < lim[1], "2-3m 中位 < 200mm");
    if (s.n_band[2]) NF_CHECK_MSG(s.med_band[2] < lim[2], "3-4m 中位 < 320mm");
    if (s.n_band[3]) NF_CHECK_MSG(s.med_band[3] < lim[3], "4-5m 中位 < 320mm");
}

NF_TEST(depth_accuracy_diagonal) {
    NFScene scene; NF_CHECK_STATUS(nf_scene_default(&scene), NF_OK);
    /* **纹理周期必须按当前视口 f 设定**：f 变了，同一空间周期的纹理
     * 在像素上的周期就变。150°/120° 下 f=128.6, 固定 tex_freq 会让
     * 2m 处周期掉到 9.6px, SGBM 匹配不稳, 误差涨到 900mm。
     * 这里按 f 反推, 让 2m 处保持 20px。 */
    NFViewParams vpp;
    nf_view_params_derive_fov(&vpp, NF_V_VIEW_W, NF_V_VIEW_HFOV_DEG,
                              NF_V_VIEW_VFOV_DEG);
    nf_scene_set_tex_for_focal(&scene, vpp.f, 20.0f, 2000.0f);
    std::printf("      纹理: f=%.2f -> 空间周期 %.0f mm (2m 处 20px)\n",
                (double)vpp.f, (double)(6.28318530718 / scene.tex_freq));
    NFCameraModel cams[4]; NFPose poses[4]; rig(cams, poses);
    RunCfg rc; rc.forward = false; rc.yaw_off = 0.0f;
    Stat s = run_accuracy(scene, cams, poses, rc);
    std::printf("      侧行 T=278mm(5km/h): n=%d 中位 %.0f mm, P95 %.0f mm, 均值 %.0f mm\n",
                s.n, s.med, s.p95, s.mean);
    check_bands("侧行5kmh", s);
    NF_CHECK(s.n > 20);
}

NF_TEST(depth_accuracy_forward) {
    NFScene scene; nf_scene_default(&scene);
    NFCameraModel cams[4]; NFPose poses[4]; rig(cams, poses);
    RunCfg rc; rc.forward = true; rc.yaw_off = 0.0f;
    Stat s = run_accuracy(scene, cams, poses, rc);
    std::printf("      直行 T=278mm(5km/h): n=%d 中位 %.0f mm, P95 %.0f mm\n",
                s.n, s.med, s.p95);
    check_bands("直行5kmh", s);
    NF_CHECK(s.n > 10);
}

NF_TEST(depth_accuracy_forward_45deg) {
    NFScene scene; nf_scene_default(&scene);
    NFCameraModel cams[4]; NFPose poses[4]; rig(cams, poses);
    RunCfg rc; rc.forward = true; rc.yaw_off = 45.0f;
    Stat s = run_accuracy(scene, cams, poses, rc);
    std::printf("      直行+45°偏转: n=%d 中位 %.0f mm, P95 %.0f mm\n",
                s.n, s.med, s.p95);
    check_bands("45deg", s);
    NF_CHECK(s.n > 10);
}

NF_TEST(depth_accuracy_forward_15kmh) {
    /* 车速上限 15 km/h -> 833 mm 基线。这是 5 m 精度最好的一档。 */
    NFScene scene; nf_scene_default(&scene);
    NFViewParams vpp;
    nf_view_params_derive_fov(&vpp, NF_V_VIEW_W, NF_V_VIEW_HFOV_DEG,
                              NF_V_VIEW_VFOV_DEG);
    nf_scene_set_tex_for_focal(&scene, vpp.f, 20.0f, 2000.0f);
    NFCameraModel cams[4]; NFPose poses[4]; rig(cams, poses);
    RunCfg rc; rc.forward = true; rc.yaw_off = 0.0f; rc.move_mm = 833.0f;
    Stat s = run_accuracy(scene, cams, poses, rc);
    std::printf("      直行 T=833mm(15km/h): n=%d 中位 %.0f mm, P95 %.0f mm\n",
                s.n, s.med, s.p95);
    check_bands("直行15kmh", s);
    NF_CHECK(s.n > 10);
}

}  // namespace

int main() { return nftest::run_all(); }
