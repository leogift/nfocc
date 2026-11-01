/**
 * @file test_hanqtech.cpp
 * @brief hanqtech 真实标定加载器测试。
 *
 * 数据在 /Volumes/Workspace/NFOCC/datasets/fisheye。
 * 若目录不存在，测试 SKIP（不能因为"没数据"就报 PASS）。
 */
#include "nf_test.h"
#include "nfocc/hanqtech_loader.h"
#include "nfocc/nfocc_camera.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

namespace {

const char *kBase = "/Volumes/Workspace/NFOCC/datasets/fisheye";
const char *kCalibDir = "/Volumes/Workspace/NFOCC/datasets/fisheye/calibration/calibration";

bool dir_exists(const char *p) {
    struct stat st;
    return stat(p, &st) == 0 && S_ISDIR(st.st_mode);
}

}  // namespace

NF_TEST(ws_calib_loads_four_cams) {
    if (!dir_exists(kCalibDir)) {
        std::printf("      SKIP: 数据集不存在\n");
        return;
    }
    NFWSCalib c;
    const NFStatus st = nf_ws_load_frame(kCalibDir, 164, &c);
    NF_CHECK_STATUS(st, NF_OK);
    NF_CHECK_EQ_INT((int)c.n_cams, 4);
    /* 名字映射: cam0=FV cam1=RV cam2=ML cam3=MR */
    std::printf("      cam0=%s cam1=%s cam2=%s cam3=%s\n",
                c.name[0], c.name[1], c.name[2], c.name[3]);
    NF_CHECK_MSG(std::strcmp(c.name[0], "FV") == 0, "cam0 应为 FV");
    NF_CHECK_MSG(std::strcmp(c.name[1], "RV") == 0, "cam1 应为 RV");
    NF_CHECK_MSG(std::strcmp(c.name[2], "MVL") == 0, "cam2 应为 MVL");
    NF_CHECK_MSG(std::strcmp(c.name[3], "MVR") == 0, "cam3 应为 MVR");
}

NF_TEST(ws_intrinsic_matches_known_values) {
    if (!dir_exists(kCalibDir)) { std::printf("      SKIP\n"); return; }
    NFWSCalib c;
    if (nf_ws_load_frame(kCalibDir, 164, &c) != NF_OK) { NF_CHECK(false); return; }
    const NFCameraModel *fv = &c.cam[0];
    NF_CHECK_EQ_INT((int)fv->kind, (int)NF_CAM_MODEL_RADIAL_POLY);
    NF_CHECK_NEAR(fv->k1, 339.749f, 1e-3);
    NF_CHECK_NEAR(fv->k2, -31.988f, 1e-3);
    NF_CHECK_NEAR(fv->k3, 48.275f, 1e-3);
    NF_CHECK_NEAR(fv->k4, -7.201f, 1e-3);
    /* 主点 = cx_offset + w/2 - 0.5 = 3.942 + 640 - 0.5 = 643.442 */
    NF_CHECK_NEAR(fv->cx, 643.442f, 1e-2);
    NF_CHECK_NEAR(fv->cy, 479.407f, 1e-2);
    NF_CHECK_STATUS(nf_cam_validate(fv), NF_OK);
}

NF_TEST(ws_extrinsic_matches_known_values) {
    if (!dir_exists(kCalibDir)) { std::printf("      SKIP\n"); return; }
    NFWSCalib c;
    if (nf_ws_load_frame(kCalibDir, 164, &c) != NF_OK) { NF_CHECK(false); return; }
    const NFPose *fv = &c.pose[0];
    /* 光心 (3.7484, 0, 0.68133) m -> mm */
    NF_CHECK_NEAR(fv->t[0], 3748.4f, 0.5);
    NF_CHECK_NEAR(fv->t[1], 0.0f, 0.5);
    NF_CHECK_NEAR(fv->t[2], 681.33f, 0.5);
    /* 光轴 = R 的第 3 列 ≈ (+0.923, -0.001, -0.385) */
    NF_CHECK_NEAR(fv->R[0*3 + 2], 0.922874f, 1e-3);
    NF_CHECK_NEAR(fv->R[1*3 + 2], -0.001128f, 1e-3);
    NF_CHECK_NEAR(fv->R[2*3 + 2], -0.385101f, 1e-3);
    /* 相机 y 轴(第2列)在车系 z 应为负 = 朝下 */
    NF_CHECK(fv->R[2*3 + 1] < 0.0f);
    NF_CHECK_STATUS(nf_pose_validate(fv), NF_OK);
}

NF_TEST(ws_all_cams_pose_valid) {
    if (!dir_exists(kCalibDir)) { std::printf("      SKIP\n"); return; }
    NFWSCalib c;
    if (nf_ws_load_frame(kCalibDir, 164, &c) != NF_OK) { NF_CHECK(false); return; }
    for (nf_u32 i = 0; i < 4; ++i) {
        const NFPose *p = &c.pose[i];
        /* 正交 */
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) {
                float s = 0.0f;
                for (int k = 0; k < 3; ++k) s += p->R[a*3+k] * p->R[b*3+k];
                NF_CHECK_NEAR(s, a == b ? 1.0f : 0.0f, 1e-5);
            }
        }
        /* y 轴朝下 */
        NF_CHECK_MSG(p->R[2*3+1] < 0.0f, "相机 y 轴必须朝下");
        NF_CHECK_STATUS(nf_pose_validate(p), NF_OK);
    }
}

NF_TEST(ws_ego_loads) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/vehicle_info/rgb_images/00164_FV.json", kBase);
    struct stat st_;
    if (stat(path, &st_) != 0) { std::printf("      SKIP\n"); return; }
    NFWSEgo e;
    NF_CHECK_STATUS(nf_ws_load_ego(path, &e), NF_OK);
    std::printf("      speed=%.2f m/s yawRate=%.3f dist=%.4f m gear=%s\n",
                (double)e.speed_mps, (double)e.yaw_rate,
                (double)e.distance_m, e.gear);
    NF_CHECK_NEAR(e.speed_mps, 29.5f, 1e-3);
    NF_CHECK_NEAR(e.yaw_rate, 0.43f, 1e-3);
    NF_CHECK_NEAR(e.distance_m, 0.273028f, 1e-5);
    NF_CHECK_MSG(std::strcmp(e.gear, "D") == 0, "档位应为 D");
}

NF_TEST(ws_project_into_image) {
    if (!dir_exists(kCalibDir)) { std::printf("      SKIP\n"); return; }
    NFWSCalib c;
    if (nf_ws_load_frame(kCalibDir, 164, &c) != NF_OK) { NF_CHECK(false); return; }

    /* 把车系中相机正前方 5m、低于光心 1m 的点投到 FV 图像，应落在图内 */
    const NFPose *p = &c.pose[0];
    const float zw[3] = { p->R[0*3+2], p->R[1*3+2], p->R[2*3+2] };
    float P[3];
    for (int k = 0; k < 3; ++k) P[k] = p->t[k] + zw[k] * 5000.0f;
    P[2] -= 1000.0f;                       /* 往地面压一点 */

    float rel[3] = { P[0]-p->t[0], P[1]-p->t[1], P[2]-p->t[2] };
    float dc[3];
    for (int k = 0; k < 3; ++k)
        dc[k] = p->R[0*3+k]*rel[0] + p->R[1*3+k]*rel[1] + p->R[2*3+k]*rel[2];

    float u = 0, v = 0;
    nf_u8 ok = 0;
    NF_CHECK_STATUS(nf_cam_project_c(&c.cam[0], dc[0], dc[1], dc[2], &u, &v, &ok),
                    NF_OK);
    std::printf("      投影到 (%.1f, %.1f) ok=%d\n", (double)u, (double)v, (int)ok);
    NF_CHECK(ok == 1);
    NF_CHECK(u > 0.0f && u < 1280.0f);
    NF_CHECK(v > 0.0f && v < 966.0f);
}

int main() { return nftest::run_all(); }
