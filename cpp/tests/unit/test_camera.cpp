/**
 * @file test_camera.cpp
 * @brief 相机模型单元测试。对应 SPEC-VERIFY V-CO-2/3/4/6。
 */
#include "nf_test.h"
#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdlib>

namespace {

NFCameraModel default_cam() {
    NFCameraModel c;
    c.version = NF_CAMERA_MODEL_VERSION;
    /* **必须显式设 kind**: 新增该字段后, 不设会读到栈垃圾,
     * 使 nf_cam_validate 返回 NF_E_PARAM_VERSION。 */
    c.kind = NF_CAM_MODEL_KB;
    c.cam_id = 0;
    c.f = NF_INF_F;
    c.cx = NF_INF_CX;
    c.cy = NF_INF_CY;
    c.k1 = NF_INF_K1; c.k2 = NF_INF_K2; c.k3 = NF_INF_K3; c.k4 = NF_INF_K4;
    c.r_valid = NF_INF_R_VALID;
    c.max_theta = NF_INF_MAX_THETA;
    return c;
}

/* FV 相机位姿: 光轴指向 +X(车头), y 朝下。
 *
 * 由 v_V = R·v_C 得 **R 的列**是相机轴在车系:
 *   列0 (x_cam) = (0,-1, 0)   车系 -Y (朝右)
 *   列1 (y_cam) = (0, 0,-1)   车系 -Z (朝下)
 *   列2 (z_cam) = (1, 0, 0)   车系 +X (车头)
 */
NFPose default_pose(unsigned cam_id) {
    NFPose p;
    p.version = NF_POSE_VERSION;
    p.cam_id = (nf_u8)cam_id;
    const float R[9] = { 0.0f,  0.0f,  1.0f,
                        -1.0f,  0.0f,  0.0f,
                         0.0f, -1.0f,  0.0f };
    for (int i = 0; i < 9; ++i) p.R[i] = R[i];
    p.t[0] = NF_VEH_REAR_TO_FRONT;
    p.t[1] = 0.0f;
    p.t[2] = NF_POS_CAM_Z;
    p.timestamp_us = 0;
    return p;
}

/* V-CO-3: 光轴上的点投影到主点 */
NF_TEST(co3_axis_to_principal) {
    NFCameraModel cam = default_cam();
    NFPose pose = default_pose(0);

    /* 相机系 (0,0,1000) -> 车系 */
    NFPoint3f P;
    P.x = pose.t[0] + 1000.0f;
    P.y = pose.t[1];
    P.z = pose.t[2];

    nf_f32 uv[2]; nf_u8 ok = 0;
    NF_CHECK_STATUS(nf_cam_project(&cam, &pose, &P, uv, &ok), NF_OK);
    NF_CHECK_EQ_INT(ok, 1);
    NF_CHECK_NEAR(uv[0], cam.cx, 1e-3f);
    NF_CHECK_NEAR(uv[1], cam.cy, 1e-3f);
}

/* V-CO-4: 图像轴未翻转 —— 相机系 +y 投影后 v 增大, +x 投影后 u 增大 */
NF_TEST(co4_axes_not_flipped) {
    NFCameraModel cam = default_cam();
    nf_f32 u = 0, v = 0; nf_u8 ok = 0;
    NF_CHECK_STATUS(nf_cam_project_c(&cam, 0.0f, 100.0f, 1000.0f,
                                     &u, &v, &ok), NF_OK);
    NF_CHECK_EQ_INT(ok, 1);
    NF_CHECK_MSG(v > NF_INF_CY, "相机 +y 应投影到 v > cy");

    NF_CHECK_STATUS(nf_cam_project_c(&cam, 100.0f, 0.0f, 1000.0f,
                                     &u, &v, &ok), NF_OK);
    NF_CHECK_MSG(u > NF_INF_CX, "相机 +x 应投影到 u > cx");
}

/* V-CO-2: 正投影 -> 反投影往返 < 0.01 px (k=0 时) */
NF_TEST(co2_roundtrip) {
    NFCameraModel cam = default_cam();

    /* 覆盖到接近 r_valid 的多个半径 */
    const float radii[6] = { 10.0f, 100.0f, 300.0f, 600.0f, 900.0f, 950.0f };
    float worst = 0.0f;

    for (int i = 0; i < 6; ++i) {
        for (int a = 0; a < 8; ++a) {
            const float ang = (float)a * 3.14159265f / 4.0f;
            const float u = cam.cx + radii[i] * std::cos(ang);
            const float v = cam.cy + radii[i] * std::sin(ang);

            float ray[3];
            NF_CHECK_STATUS(nf_cam_unproject_c(&cam, u, v, ray), NF_OK);

            float u2, v2; nf_u8 ok = 0;
            NF_CHECK_STATUS(nf_cam_project_c(&cam, ray[0], ray[1], ray[2],
                                             &u2, &v2, &ok), NF_OK);
            if (!ok) continue;   /* 超出 r_valid 的边界点 */
            const float e = std::sqrt((u2-u)*(u2-u) + (v2-v)*(v2-v));
            if (e > worst) worst = e;
        }
    }
    NF_CHECK_MSG(worst < 0.01f, "往返误差超过 0.01 px");
}

/* V-CO-6: 非单调的畸变系数必须被拒绝 */
NF_TEST(co6_nonmonotonic_rejected) {
    NFCameraModel cam = default_cam();
    cam.k1 = -0.5f;    /* 会使 theta_d 非单调 */
    NF_CHECK_STATUS(nf_cam_validate(&cam), NF_E_CAM_NOT_CALIBRATED);
}

/* 合法内参通过自检 */
NF_TEST(cam_validate_ok) {
    NFCameraModel cam = default_cam();
    NF_CHECK_STATUS(nf_cam_validate(&cam), NF_OK);
}

/* 外参自检: 正确外参通过 */
NF_TEST(pose_validate_ok) {
    NFPose p = default_pose(0);
    NF_CHECK_STATUS(nf_pose_validate(&p), NF_OK);
}

/* 外参自检: y 轴朝上(相机倒装)必须被拒 */
NF_TEST(pose_validate_flipped) {
    NFPose p = default_pose(0);
    p.R[3] = 0.0f; p.R[4] = 0.0f; p.R[5] = 1.0f;   /* y 朝上 */
    NF_CHECK_STATUS(nf_pose_validate(&p), NF_E_EXTRINSIC_INVALID);
}

/* 外参自检: 非正交必须被拒 */
NF_TEST(pose_validate_nonortho) {
    NFPose p = default_pose(0);
    p.R[0] = 0.2f;    /* 破坏正交性 */
    NF_CHECK_STATUS(nf_pose_validate(&p), NF_E_EXTRINSIC_INVALID);
}

/* max_theta 太小会砍掉视场 —— 通过投影判据体现 */
NF_TEST(max_theta_too_small) {
    NFCameraModel cam = default_cam();
    /* 本项目左右边缘 theta = 100°, 若 max_theta 用第三方默认 1.65rad(94.5°)
       则左右边缘被静默判无效 */
    cam.max_theta = 1.65f;
    nf_f32 u, v; nf_u8 ok = 0;
    const float th = 100.0f * 3.14159265f / 180.0f;
    NF_CHECK_STATUS(nf_cam_project_c(&cam, std::sin(th), 0.0f, std::cos(th),
                                     &u, &v, &ok), NF_OK);
    NF_CHECK_EQ_INT(ok, 0);   /* 应为无效 */

    /* 用正确的 2.235rad 则有效 */
    cam.max_theta = 2.235f;
    NF_CHECK_STATUS(nf_cam_project_c(&cam, std::sin(th), 0.0f, std::cos(th),
                                     &u, &v, &ok), NF_OK);
    NF_CHECK_MSG(ok == 1, "max_theta=2.235 时 100° 应有效");
}

/* theta_d 导数系数 1,3,5,7,9 —— 用 k=0 时 d'=1 验证 */
NF_TEST(theta_d_prime_coeffs) {
    NFCameraModel cam = default_cam();
    NF_CHECK_NEAR(nf_cam_theta_d_prime(&cam, 0.5f), 1.0f, 1e-6f);

    /* 单系数验证: k1=0.1 时 d' = 1 + 3*0.1*θ² */
    cam.k1 = 0.1f;
    const float th = 0.5f;
    NF_CHECK_NEAR(nf_cam_theta_d_prime(&cam, th), 1.0f + 3.0f*0.1f*th*th, 1e-6f);
    NF_CHECK_NEAR(nf_cam_theta_d(&cam, th), th*(1.0f + 0.1f*th*th), 1e-6f);
}

/* NULL 与 version 安全 */
NF_TEST(cam_null_safety) {
    NF_CHECK_STATUS(nf_cam_validate(nullptr), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_pose_validate(nullptr), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_cam_project(nullptr, nullptr, nullptr, nullptr, nullptr),
                    NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_cam_unproject_c(nullptr, 0, 0, nullptr), NF_E_PARAM_NULL);

    NFCameraModel cam = default_cam();
    cam.version = 999;
    NF_CHECK_STATUS(nf_cam_validate(&cam), NF_E_PARAM_VERSION);
}

} // namespace

int main() { return nftest::run_all(); }
