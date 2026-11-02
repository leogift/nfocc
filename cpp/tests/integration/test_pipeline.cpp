/**
 * @file test_pipeline.cpp
 * @brief 集成测试: Warp -> Engine -> Triangulate -> Voxel 全链路。
 *
 * 对应 PH1-ACCEPTANCE 的 V-PROD-* 与 V-BUILD-3/7。
 */
#include "nf_test.h"
#include "nfocc/nfocc_pipeline.h"
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>
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
        cams[c].version = NF_CAMERA_MODEL_VERSION;
        cams[c].kind = NF_CAM_MODEL_KB;
        cams[c].cam_id = (nf_u8)c;
        cams[c].f = NF_INF_F; cams[c].cx = NF_INF_CX; cams[c].cy = NF_INF_CY;
        cams[c].k1 = cams[c].k2 = cams[c].k3 = cams[c].k4 = 0.0f;
        cams[c].r_valid = NF_INF_R_VALID;
        cams[c].max_theta = NF_INF_MAX_THETA;

        poses[c].version = NF_POSE_VERSION;
        poses[c].cam_id = (nf_u8)c;
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                poses[c].R[row * 3 + col] = axis[c][col][row];
        poses[c].t[0] = pos[c][0];
        poses[c].t[1] = pos[c][1];
        poses[c].t[2] = pos[c][2];
        poses[c].timestamp_us = 0;
    }
}

NFPipelineConfig make_cfg() {
    NFPipelineConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.version = 1;
    /* **每路指定虚拟相机朝向**。不设就全是 yaw=0（都朝车头），
     * 只有 FV 看得到，RV/ML/MR 的视口全空被禁用 ——
     * 门控与点云都只剩一路，测试语义就变了。 */
    cfg.view_yaw_deg[0] = 0.0f;     /* FV 朝前 */
    cfg.view_yaw_deg[1] = 180.0f;   /* RV 朝后 */
    cfg.view_yaw_deg[2] = 90.0f;    /* ML 朝左 */
    cfg.view_yaw_deg[3] = -90.0f;   /* MR 朝右 */
    cfg.engine.version = NF_ENGINE_CONFIG_VERSION;
    cfg.engine.kind = NF_ENGINE_SGBM;
    cfg.engine.max_disp_px = 128;
    cfg.engine.downscale = 2;
    cfg.engine.block_size = 5;
    cfg.engine.uniqueness = 10;
    cfg.engine.speckle_window = 100;

    nf_view_params_derive(&cfg.view, 480, NF_V_RING_EL_MIN_DEG,
                          NF_V_RING_EL_MAX_DEG, 120.0f);
    nf_voxel_grid_init(&cfg.grid, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                       NF_G_Y_MIN, NF_G_Y_MAX, NF_G_Z_MIN, NF_G_Z_MAX);
    cfg.b_min_mm = NF_B_MIN;
    /**
     * **b_max 必须按本测试自己的引擎/视口容量推导**，不能直接抄全局 NF_B_MAX。
     *
     * 容量约束: b_max <= max_disp * Z_MIN / f。
     * 本测试用 480 宽视口 + max_disp=128 -> 上限约 462mm；
     * 全局 NF_B_MAX 已提到 833（对应 1024 宽视口 + 256 视差），
     * 直接抄会让 validate 判 RANGE —— 这个拒绝是对的，说明判据有效。
     */
    {
        const float cap = (float)cfg.engine.max_disp_px * NF_B_ZMIN_EFF / cfg.view.f;
        cfg.b_max_mm = (cap * 0.98f < NF_B_MAX) ? (cap * 0.98f) : NF_B_MAX;
        if (cfg.b_max_mm < cfg.b_min_mm * 1.5f) cfg.b_max_mm = NF_B_MAX;
    }
    cfg.sigma_d_px = NF_T_SIGMA_D;
    cfg.sigma_b_ratio = NF_T_SIGMA_B_RATIO;
    cfg.gap_max_frames = NF_GAP_MAX_FRAMES;
    cfg.max_points = 4096;
    cfg.thread_cameras = 1;
    return cfg;
}

/* 视口尺寸必须与视口参数自洽 */
NF_TEST(pipe_view_selfconsistent) {
    NFPipelineConfig cfg = make_cfg();
    NF_CHECK_STATUS(nf_view_params_validate(&cfg.view), NF_OK);
    /* height 由 fov_v 反算, 不能独立指定 */
    NF_CHECK(cfg.view.height > 0);
    NF_CHECK(cfg.view.height < 1536);
}

/* 参数自检: 合法配置通过 */
NF_TEST(pipe_params_ok) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    NFPipelineConfig cfg = make_cfg();
    NF_CHECK_STATUS(nf_params_validate(&cfg, cams, 4), NF_OK);
}

/* 参数自检: max_theta 抄库默认 1.65 必须被拒(会砍掉 5.5° 视场) */
NF_TEST(pipe_reject_small_max_theta) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    for (int c = 0; c < 4; ++c) cams[c].max_theta = 1.65f;
    NFPipelineConfig cfg = make_cfg();
    NF_CHECK_STATUS(nf_params_validate(&cfg, cams, 4), NF_E_PARAM_RANGE);
}

/* 参数自检: B_MAX 超过引擎容量反推值必须被拒 */
NF_TEST(pipe_reject_bmax) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    NFPipelineConfig cfg = make_cfg();
    cfg.b_max_mm = 2000.0f;   /* 远超 numDisp 能覆盖的范围 */
    NF_CHECK_STATUS(nf_params_validate(&cfg, cams, 4), NF_E_PARAM_RANGE);
}

/* 创建 + 销毁: 全部内存在 init 期分配 */
NF_TEST(pipe_create_destroy) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    NFPipelineConfig cfg = make_cfg();

    NFPipeline *p = nullptr;
    NF_CHECK_STATUS(nf_pipeline_create(&p, &cfg, cams, poses), NF_OK);
    NF_CHECK(p != nullptr);
    NF_CHECK_STATUS(nf_pipeline_destroy(p), NF_OK);
}

/* 内存预算: 解析值应在上板可接受范围 */
NF_TEST(pipe_mem_budget) {
    NFPipelineConfig cfg = make_cfg();
    nf_u64 bytes = 0;
    NF_CHECK_STATUS(nf_pipeline_mem_bytes(&cfg, &bytes), NF_OK);
    const double mb = (double)bytes / 1e6;
    NF_CHECK_MSG(mb > 0.0, "内存预算不应为 0");
    NF_CHECK_MSG(mb < 800.0, "串行模式下内存预算超出上板阈值");
}

/* 冻结语义: 基线不足时**不产出点云**(不是空点云) */
NF_TEST(pipe_frozen_no_points) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    NFPipelineConfig cfg = make_cfg();

    NFPipeline *p = nullptr;
    NF_CHECK_STATUS(nf_pipeline_create(&p, &cfg, cams, poses), NF_OK);

    const nf_u32 w = 480, h = 320;
    std::vector<nf_u8> buf((size_t)w * h, 128);
    NFFrameInput in;
    std::memset(&in, 0, sizeof(in));
    in.width = w; in.height = h; in.stride = w;
    for (int c = 0; c < 4; ++c) {
        in.img[c] = buf.data();
        in.t_cam[c][0] = 2.0f;      /* 远小于 B_MIN=12 -> FROZEN_NEAR */
    }
    in.frame_gap = 1;

    NFFrameOutput out;
    std::memset(&out, 0, sizeof(out));
    out.version = NF_POINT_CLOUD_VERSION;

    NFStatus st = nf_pipeline_process(p, &in, &out);
    NF_CHECK_EQ_INT(st, NF_E_BASELINE_IDLE);
    NF_CHECK_EQ_INT(out.is_frozen, 1);
    NF_CHECK_EQ_INT(out.cloud.num_points, 0);
    for (int c = 0; c < 4; ++c) {
        NF_CHECK_EQ_INT(out.mode[c], NF_MODE_FROZEN_NEAR);
        NF_CHECK_EQ_INT(out.num_points_per_cam[c], 0);
    }
    nf_pipeline_destroy(p);
}

/* 门控在全链路中生效: 侧移 -> STEREO, 前向 -> FORWARD */
NF_TEST(pipe_gate_integrated) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    NFPipelineConfig cfg = make_cfg();

    NFPipeline *p = nullptr;
    NF_CHECK_STATUS(nf_pipeline_create(&p, &cfg, cams, poses), NF_OK);

    const nf_u32 w = 480, h = 320;
    std::vector<nf_u8> buf((size_t)w * h, 128);

    {
        NFFrameInput in;
        std::memset(&in, 0, sizeof(in));
        in.width = w; in.height = h; in.stride = w;
        for (int c = 0; c < 4; ++c) {
            in.img[c] = buf.data();
            in.t_cam[c][0] = 300.0f;    /* 纯侧移, 落在 [B_MIN,B_MAX] */
        }
        in.frame_gap = 1;
        NFFrameOutput out; std::memset(&out, 0, sizeof(out));
        out.version = NF_POINT_CLOUD_VERSION;
        nf_pipeline_process(p, &in, &out);
        NF_CHECK_EQ_INT(out.mode[0], NF_MODE_STEREO);
        NF_CHECK_EQ_INT(out.is_frozen, 0);
    }
    {
        NFFrameInput in;
        std::memset(&in, 0, sizeof(in));
        in.width = w; in.height = h; in.stride = w;
        for (int c = 0; c < 4; ++c) {
            in.img[c] = buf.data();
            in.t_cam[c][2] = 300.0f;    /* 纯前向 */
        }
        in.frame_gap = 1;
        NFFrameOutput out; std::memset(&out, 0, sizeof(out));
        out.version = NF_POINT_CLOUD_VERSION;
        nf_pipeline_process(p, &in, &out);
        NF_CHECK_EQ_INT(out.mode[0], NF_MODE_FORWARD);
    }
    nf_pipeline_destroy(p);
}

/* 外参自检失败的相机必须被整个禁用 */
NF_TEST(pipe_bad_pose_disables_cam) {
    NFCameraModel cams[4]; NFPose poses[4];
    make_rig(cams, poses);
    poses[2].R[0] = 0.5f;   /* 破坏 ML 的正交性 */
    NFPipelineConfig cfg = make_cfg();

    NFPipeline *p = nullptr;
    NF_CHECK_STATUS(nf_pipeline_create(&p, &cfg, cams, poses), NF_OK);

    const nf_u32 w = 480, h = 320;
    std::vector<nf_u8> buf((size_t)w * h, 128);
    NFFrameInput in;
    std::memset(&in, 0, sizeof(in));
    in.width = w; in.height = h; in.stride = w;
    for (int c = 0; c < 4; ++c) { in.img[c] = buf.data(); in.t_cam[c][0] = 300.0f; }
    in.frame_gap = 1;

    NFFrameOutput out; std::memset(&out, 0, sizeof(out));
    out.version = NF_POINT_CLOUD_VERSION;
    nf_pipeline_process(p, &in, &out);
    NF_CHECK_EQ_INT(out.cam_status[2], NF_E_EXTRINSIC_INVALID);
    NF_CHECK_EQ_INT(out.num_points_per_cam[2], 0);

    nf_pipeline_destroy(p);
}

/* NULL 安全 */
NF_TEST(pipe_null_safety) {
    NF_CHECK_STATUS(nf_pipeline_create(nullptr, nullptr, nullptr, nullptr),
                    NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_pipeline_process(nullptr, nullptr, nullptr), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_pipeline_destroy(nullptr), NF_E_PARAM_NULL);
    nf_u64 b = 0;
    NF_CHECK_STATUS(nf_pipeline_mem_bytes(nullptr, &b), NF_E_PARAM_NULL);
}

} // namespace

int main() { return nftest::run_all(); }
