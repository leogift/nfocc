/**
 * @file probe_sfm6.cpp
 * @brief 6 视口 SFM 几何探针：同一解析场景，4 视口 vs 6 视口。
 *
 * 目的不是"跑通"，而是量出：
 *   - 每个虚拟视口是否真的出点（view_mode / num_points_per_view）
 *   - 点云到场景表面的距离（毫米，中位/P95）
 *   - 1-5m 水平距离内的点、z< -100mm 的地下点比例
 * 场景是解析平面（地面 + 两面墙），真值可闭式求；同一物点两帧灰度
 * 由 3D 坐标决定，匹配器不会因纹理变化而假匹配。
 */
#include "nfocc/nfocc_pipeline.h"
#include "nfocc/nfocc_scene.h"
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_engine_bind.h"
#include "nfocc/nfocc_default.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

void make_rig(NFCameraModel cams[4], NFPose poses[4]) {
    static const float axis[4][3][3] = {
        { {0,-1,0}, {0,0,-1}, {1,0,0} },
        { {0, 1,0}, {0,0,-1}, {-1,0,0} },
        { {1, 0,0}, {0,0,-1}, {0,1,0} },
        { {-1,0,0}, {0,0,-1}, {0,-1,0} },
    };
    static const float pos[4][3] = {
        { NF_VEH_REAR_TO_FRONT, 0.0f, NF_POS_CAM_Z },
        { -(NF_VEH_LEN - NF_VEH_REAR_TO_FRONT), 0.0f, NF_POS_CAM_Z },
        { 0.0f,  NF_VEH_WID * 0.5f, NF_POS_CAM_Z },
        { 0.0f, -NF_VEH_WID * 0.5f, NF_POS_CAM_Z },
    };
    for (int c = 0; c < 4; ++c) {
        std::memset(&cams[c], 0, sizeof(cams[c]));
        cams[c].version = NF_CAMERA_MODEL_VERSION;
        cams[c].kind = NF_CAM_MODEL_KB;
        cams[c].cam_id = (nf_u8)c;
        cams[c].f = NF_INF_F; cams[c].cx = NF_INF_CX; cams[c].cy = NF_INF_CY;
        cams[c].k1 = NF_INF_K1; cams[c].k2 = NF_INF_K2;
        cams[c].k3 = NF_INF_K3; cams[c].k4 = NF_INF_K4;
        cams[c].r_valid = NF_INF_R_VALID; cams[c].max_theta = NF_INF_MAX_THETA;
        std::memset(&poses[c], 0, sizeof(poses[c]));
        poses[c].version = NF_POSE_VERSION;
        poses[c].cam_id = (nf_u8)c;
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                poses[c].R[row * 3 + col] = axis[c][col][row];
        poses[c].t[0] = pos[c][0];
        poses[c].t[1] = pos[c][1];
        poses[c].t[2] = pos[c][2];
    }
}

double pct(std::vector<double> &v, double p) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    const size_t k = (size_t)(p * (double)(v.size() - 1) + 0.5);
    return v[k];
}

struct Stat {
    int n_points = 0, n_band = 0, below = 0;
    double med = -1, p95 = -1, med_v = -1, zmin = 0, zmax = 0, hmean = 0;
};

Stat run(bool six, NFScene &scene, const NFCameraModel cams[4],
         const NFPose poses[4], int W, int H,
         const std::vector<NFRenderFrame> &cur,
         const std::vector<NFRenderFrame> &ref, nf_u32 cam_mask) {
    NFPipelineConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.version = 1;
    cfg.engine.version = NF_ENGINE_CONFIG_VERSION;
    cfg.engine.kind = NF_ENGINE_SGBM;
    cfg.engine.max_disp_px = 256;
    cfg.engine.downscale = 1;
    cfg.engine.block_size = 5;
    cfg.engine.uniqueness = 10;
    cfg.engine.speckle_window = 100;
    nf_view_params_derive_fov(&cfg.view, 1024, NF_V_VIEW_HFOV_DEG, NF_V_VIEW_VFOV_DEG);
    cfg.b_min_mm = 200.0f;
    cfg.b_max_mm = 833.0f;
    cfg.b_target_mm = 500.0f;
    cfg.sigma_d_px = 0.5f;
    cfg.sigma_b_ratio = 0.02f;
    cfg.sigma_max_cm = 80.0f;
    cfg.gap_max_frames = 4;
    cfg.max_points = 60000;
    cfg.cam_enable_mask = cam_mask;
    nf_voxel_grid_init(&cfg.grid, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                       NF_G_Y_MIN, NF_G_Y_MAX, NF_G_Z_MIN, NF_G_Z_MAX);
    cfg.forward_sample_step = 2u;
    cfg.forward_coarse_step = 1.0f;
    cfg.forward_ncc_min = 0.7f;

    if (six) {
        cfg.n_views = nf_viewplan_default(cfg.view_plan, NF_MAX_VIEWS);
        /* 使用产品默认的 110° 成对视口（中心仍 ±45，间隔 90°） */
    } else {
        cfg.view_yaw_deg[0] = 0.0f;
        cfg.view_yaw_deg[1] = 180.0f;
        cfg.view_yaw_deg[2] = 90.0f;
        cfg.view_yaw_deg[3] = -90.0f;
    }

    NFPipeline *p = nullptr;
    NFStatus st = nf_pipeline_create(&p, &cfg, cams, poses);
    if (st != NF_OK) {
        std::printf("  create 失败: %s\n", nf_status_str(st));
        return Stat{};
    }

    NFFrameInput in;
    std::memset(&in, 0, sizeof(in));
    for (int c = 0; c < 4; ++c) {
        in.img[c] = cur[c].img;
        in.img_ref[c] = ref[c].img;
    }
    in.width = (nf_u32)W; in.height = (nf_u32)H; in.stride = (nf_u32)W;
    in.frame_id = 1; in.frame_gap = 1;
    in.t_veh[0] = 278.0f;

    NFFrameOutput out;
    std::memset(&out, 0, sizeof(out));
    out.version = NF_POINT_CLOUD_VERSION;
    st = nf_pipeline_process(p, &in, &out);

    Stat s;
    s.n_points = (int)out.cloud.num_points;
    std::vector<double> err, err_v;
    double hsum = 0.0;
    s.zmin = 1e9; s.zmax = -1e9;
    for (nf_u32 i = 0; i < out.cloud.num_points; ++i) {
        const NFPoint &q = out.cloud.points[i];
        const double h = std::sqrt((double)q.x_mm * q.x_mm +
                                   (double)q.y_mm * q.y_mm);
        if (h >= 500.0 && h <= 5000.0) {
            float d = 0.0f;
            const float P[3] = { (float)q.x_mm, (float)q.y_mm, (float)q.z_mm };
            if (nf_scene_dist_to_surface(&scene, P, &d) == NF_OK) {
                err.push_back((double)d);
                if (q.z_mm > 100) err_v.push_back((double)d);
                ++s.n_band;
            }
            hsum += h;
        }
        if (q.z_mm < -100) ++s.below;
        if (q.z_mm < s.zmin) s.zmin = q.z_mm;
        if (q.z_mm > s.zmax) s.zmax = q.z_mm;
    }
    s.med = pct(err, 0.50);
    s.p95 = pct(err, 0.95);
    s.med_v = pct(err_v, 0.50);
    s.hmean = s.n_band ? hsum / s.n_band : 0.0;

    std::printf("  status=%-22s n=%d 1-5m=%d 表面中位=%.0f 竖面中位=%.0f P95=%.0f mm "
                "z=[%.0f,%.0f] hmean=%.0f 地下=%d\n",
                nf_status_str(st), s.n_points, s.n_band, s.med, s.med_v, s.p95,
                s.zmin, s.zmax, s.hmean, s.below);
    if (six) {
        for (nf_u32 v = 0; v < out.n_views; ++v)
            std::printf("    view%u cam%u mode=%-8s status=%-20s pts=%u\n",
                        v, out.view_cam[v], nf_mode_str(out.view_mode[v]),
                        nf_status_str(out.view_status[v]),
                        out.num_points_per_view[v]);
    }

    nf_pipeline_destroy(p);
    return s;
}

} // namespace

int main() {
    NFScene scene;
    nf_scene_default(&scene);
    /* 前视 90° 的 f=295；20px@2m 对侧视 150° 会偏小，但两模式同场景可比 */
    nf_scene_set_tex_for_focal(&scene, 295.0f, 20.0f, 2000.0f);

    NFCameraModel cams[4];
    NFPose poses[4];
    make_rig(cams, poses);
    nf_pipeline_bind_sgbm();

    const int W = 1280, H = 966;
    std::vector<NFRenderFrame> cur(4), ref(4);
    for (int c = 0; c < 4; ++c) {
        nf_render_frame_alloc(&cur[c], (nf_u32)W, (nf_u32)H);
        nf_render_frame_alloc(&ref[c], (nf_u32)W, (nf_u32)H);
        NFPose pr = poses[c];
        pr.t[0] -= 278.0f;
        nf_render_frame(&scene, &cams[c], &pr, &ref[c]);
        nf_render_frame(&scene, &cams[c], &poses[c], &cur[c]);
    }

    const nf_u32 masks[2] = { 0x3u, 0xCu };
    const char *names[2] = { "前向 FV+RV", "侧向 ML+MR" };
    for (int k = 0; k < 2; ++k) {
        std::printf("\n[%s] 4 视口（每相机 1 个，150°）:\n", names[k]);
        Stat s4 = run(false, scene, cams, poses, W, H, cur, ref, masks[k]);
        std::printf("[%s] 6 视口（前后 ±45/±135，左右 ±90）:\n", names[k]);
        Stat s6 = run(true, scene, cams, poses, W, H, cur, ref, masks[k]);
        std::printf("对比[%s]: 4视口 n=%d med=%.0f p95=%.0f below=%d | "
                    "6视口 n=%d med=%.0f p95=%.0f below=%d\n",
                    names[k], s4.n_points, s4.med, s4.p95, s4.below,
                    s6.n_points, s6.med, s6.p95, s6.below);
    }
    for (int c = 0; c < 4; ++c) {
        nf_render_frame_free(&cur[c]);
        nf_render_frame_free(&ref[c]);
    }
    return 0;
}
