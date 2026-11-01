/**
 * @file test_scene.cpp
 * @brief 真值场景的自校验 —— **先证明真值本身是对的**，再拿它去测实现。
 *
 * 待测代码错了要能看出来；而"真值"错了会让所有比对失去意义。
 * 所以本测试的第一要务是：地面深度的闭式解必须与解析式吻合。
 */
#include "nf_test.h"
#include "nfocc/nfocc_scene.h"
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace {

/* 理想外参: FV 朝 +X, 高 700mm */
void fv_pose(NFPose *p) {
    /* R 的列 = 相机轴在车系 */
    static const float axis[3][3] = { {0,-1,0}, {0,0,-1}, {1,0,0} };
    std::memset(p, 0, sizeof(*p));
    p->version = NF_POSE_VERSION;
    for (int col = 0; col < 3; ++col)
        for (int row = 0; row < 3; ++row)
            p->R[row * 3 + col] = axis[col][row];
    p->t[0] = 0.0f; p->t[1] = 0.0f; p->t[2] = NF_POS_CAM_Z;
}

void ideal_cam(NFCameraModel *c) {
    std::memset(c, 0, sizeof(*c));
    c->version = NF_CAMERA_MODEL_VERSION;
    c->f = NF_INF_F; c->cx = NF_INF_CX; c->cy = NF_INF_CY;
    c->k1 = c->k2 = c->k3 = c->k4 = 0.0f;
    c->r_valid = NF_INF_R_VALID;
    c->max_theta = NF_INF_MAX_THETA;
}

/* 地面真值校验: 视线俯角 el 处, 沿光轴深度应为 H/tan(el)。
 * 用相机系射线直接算, 绕过像素, 避免倒投影误差混进来。 */
NF_TEST(scene_ground_depth_analytic) {
    NFScene s;
    NF_CHECK_STATUS(nf_scene_default(&s), NF_OK);

    const float H = NF_POS_CAM_Z;      /* 700 */
    const float o[3] = { 0.0f, 0.0f, H };

    int n = 0;
    double max_rel = 0.0;
    for (int el_deg = 10; el_deg <= 60; el_deg += 5) {
        for (int az_deg = -50; az_deg <= 50; az_deg += 25) {
            const float el = el_deg * 3.14159265358979f / 180.0f;
            const float az = az_deg * 3.14159265358979f / 180.0f;
            /* **直接在车系里表达射线**, 绕开坐标系转换 —— 测试的目的是校验
             * 场景的闭式解, 不是校验坐标变换。转换写错过两次, 都是测试自身
             * 的 bug（第一版相机系方向直接喂进世界系求交 -> 全落空;
             * 第二版转错轴 -> 误差 35.7%）。
             *
             * 车系: x 前, y 左, z 上。相机在 (0,0,H) 朝 +x。
             * 方位角 az (绕 z 轴, 0=+x), 俯角 el (向下):
             *   d = (cos(az)·cos(el), sin(az)·cos(el), -sin(el))
             * 沿光轴(即车系 +x)的深度 = t_hit · d[0] = H / tan(el)。 */
            const float d[3] = { std::cos(az) * std::cos(el),
                                 std::sin(az) * std::cos(el),
                                 -std::sin(el) };
            float t_hit = 0.0f, P[3];
            nf_u32 pl = 0;
            NFStatus st = nf_scene_raycast(&s, o, d, &t_hit, P, &pl);
            if (st != NF_OK) continue;      /* 打到墙上或框外, 跳过 */

            /* 打到地面(plane 0)时: 沿**光轴**(相机 z)的深度 = H / tan(el)。
             * t_hit 是射线距离, 沿光轴的分量 = t_hit * dc[2]（相机系 z 分量）。 */
            if (pl != 0) continue;
            const float z_axis = t_hit * d[0];      /* 沿光轴(车系 +x) */
            /* **期望值必须含方位角投影**。
             * 沿光轴深度 = t·cos(az)·cos(el)，而 t = H/sin(el)，
             * 故 = H·cos(az)/tan(el)。
             * 第一版漏了 cos(az)，az=±50° 处差 1.556 倍，
             * 使一条本来正确的实现被判成"误差 35.7%"。 */
            const float z_expect = H * std::cos(az) / std::tan(el);
            const double rel = std::fabs((double)z_axis - z_expect) / z_expect;
            if (rel > max_rel) max_rel = rel;
            ++n;
        }
    }
    std::printf("      地面采样 %d 点, 最大相对误差 %.3e\n", n, max_rel);
    NF_CHECK(n > 50);
    NF_CHECK_MSG(max_rel < 1e-6, "地面真值必须与 H/tan(el) 闭式解吻合");
}

/* 射线求交本身: 垂直向下打地面, 距离应等于相机高度 */
NF_TEST(scene_raycast_down) {
    NFScene s;
    nf_scene_default(&s);
    const float o[3] = { 0.0f, 0.0f, 700.0f };
    const float d[3] = { 0.0f, 0.0f, -1.0f };
    float t = 0.0f, P[3];
    nf_u32 pl = 99;
    NF_CHECK_STATUS(nf_scene_raycast(&s, o, d, &t, P, &pl), NF_OK);
    NF_CHECK_NEAR(t, 700.0f, 1e-3);
    NF_CHECK_EQ_INT(pl, 0);
    NF_CHECK_NEAR(P[2], 0.0f, 1e-3);
}

/* 打前方墙: 沿 +x 水平, 应命中 x=4000 的墙 */
NF_TEST(scene_raycast_wall) {
    NFScene s;
    nf_scene_default(&s);
    const float o[3] = { 0.0f, 0.0f, 1000.0f };
    const float d[3] = { 1.0f, 0.0f, 0.0f };
    float t = 0.0f, P[3];
    nf_u32 pl = 99;
    NFStatus st = nf_scene_raycast(&s, o, d, &t, P, &pl);
    NF_CHECK_STATUS(st, NF_OK);
    if (st == NF_OK) {
        NF_CHECK_NEAR(t, 4000.0f, 1.0f);
        NF_CHECK_EQ_INT(pl, 1);
    }
}

/* 朝上打: 无平面可命中, 必须返回失败而不是给出巨大深度 */
NF_TEST(scene_raycast_sky) {
    NFScene s;
    nf_scene_default(&s);
    const float o[3] = { 0.0f, 0.0f, 700.0f };
    const float d[3] = { 1.0f, 0.0f, 1.0f };
    float t = 0.0f;
    NFStatus st = nf_scene_raycast(&s, o, d, &t, nullptr, nullptr);
    /* 斜上打: 墙高 2000, 起点 700, 可能命中墙。用正上方保证落空 */
    const float up[3] = { 0.0f, 0.0f, 1.0f };
    NFStatus st2 = nf_scene_raycast(&s, o, up, &t, nullptr, nullptr);
    NF_CHECK_MSG(st2 != NF_OK, "正上方必须无命中");
    (void)st;
}

/* 纹理必须是 3D 坐标的确定性函数: 同一点两次着色相同 */
NF_TEST(scene_shade_deterministic) {
    NFScene s;
    nf_scene_default(&s);
    const float p[3] = { 1234.5f, -678.9f, 0.0f };
    const nf_u8 a = nf_scene_shade(&s, p, 0);
    const nf_u8 b = nf_scene_shade(&s, p, 0);
    NF_CHECK_EQ_INT(a, b);
    /* 不同平面同一点应不同（相位不同） */
    const nf_u8 c = nf_scene_shade(&s, p, 1);
    NF_CHECK_MSG(a != c, "不同平面应有不同相位");
}

/* 渲染出的真值深度必须与射线的解析深度一致 */
NF_TEST(render_depth_matches_raycast) {
    NFScene s;
    nf_scene_default(&s);
    NFCameraModel cam; ideal_cam(&cam);
    NFPose pose; fv_pose(&pose);

    NFRenderFrame f;
    std::memset(&f, 0, sizeof(f));
    /* 用较小分辨率加快测试, 但保持主点居中 */
    NF_CHECK_STATUS(nf_render_frame_alloc(&f, 320, 256), NF_OK);
    cam.cx = 160.0f; cam.cy = 128.0f;

    NF_CHECK_STATUS(nf_render_frame(&s, &cam, &pose, &f), NF_OK);

    /* 抽样比对: 深度 > 0 的像素, 其真值应等于该像素射线的解析 Z */
    int n = 0, mism = 0;
    double worst = 0.0;
    for (nf_u32 y = 8; y < f.height; y += 37) {
        for (nf_u32 x = 8; x < f.width; x += 41) {
            const nf_u32 i = y * f.width + x;
            const float z = f.depth[i];
            if (z <= 0.0f) continue;
            /* 重算: 像素 -> 相机系射线 -> 车系 -> 求交 */
            float dc[3];
            if (nf_cam_unproject_c(&cam, (nf_f32)x + 0.5f, (nf_f32)y + 0.5f, dc) != NF_OK)
                continue;
            float dw[3];
            for (int k = 0; k < 3; ++k)
                dw[k] = pose.R[k*3+0]*dc[0] + pose.R[k*3+1]*dc[1] + pose.R[k*3+2]*dc[2];
            float th = 0.0f;
            if (nf_scene_raycast(&s, pose.t, dw, &th, nullptr, nullptr) != NF_OK) continue;
            const float z_ref = th * dc[2];
            const double rel = std::fabs((double)z - z_ref) / z_ref;
            if (rel > worst) worst = rel;
            if (rel > 1e-5) ++mism;
            ++n;
        }
    }
    std::printf("      抽样 %d 点, 最大相对误差 %.3e, 不一致 %d\n", n, worst, mism);
    NF_CHECK(n > 20);
    NF_CHECK_MSG(mism == 0, "渲染真值与逐射线解析值必须一致");
    nf_render_frame_free(&f);
}

}  // namespace

int main() { return nftest::run_all(); }
