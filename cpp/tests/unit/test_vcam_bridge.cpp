/**
 * @file test_vcam_bridge.cpp
 * @brief vcam 桥接的交叉校验。
 *
 * ## 这个测试要回答的唯一问题
 * 把投影从 NFOCC 原生实现换成 vcam，会不会改变结果？
 *
 * 两边的公式是同一套（KB / radial_poly），但实现语言、精度（float32 vs
 * float64）、求根迭代次数都可能不同。所以这里**不比"看起来对"，
 * 比"两条路算出来的射线夹角"** —— 折算成像素偏差，给出一个可判的数。
 *
 * ## 判据
 * - vcam 未编译：`nf_vcam_backend()` 返回 NONE，测试跳过（不算失败）
 * - vcam 已编译：最大偏差 < 0.5 px
 *
 * 0.5 px 这个门限来自本项目对亚像素精度的要求：
 * σ_d 预算本身就 0.5 px，后端差异占满全部预算就说明不能换。
 */
#include <cmath>
#include <cstring>

#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_vcam_bridge.h"
#include "nf_test.h"

namespace {

/** 构造一个 hanqtech 风格的 radial_poly 相机（k1 就是小角度焦距） */
NFCameraModel make_radial(float k1, float k2, float k3, float k4) {
    NFCameraModel c;
    std::memset(&c, 0, sizeof(c));
    c.version = NF_CAMERA_MODEL_VERSION;
    c.kind = NF_CAM_MODEL_RADIAL_POLY;
    c.cam_id = 0;
    c.cx = 640.0f; c.cy = 483.0f;
    c.k1 = k1; c.k2 = k2; c.k3 = k3; c.k4 = k4;
    c.r_valid = 802.0f;
    c.max_theta = 1.9f;
    return c;
}

NFCameraModel make_kb(float f) {
    NFCameraModel c;
    std::memset(&c, 0, sizeof(c));
    c.version = NF_CAMERA_MODEL_VERSION;
    c.kind = NF_CAM_MODEL_KB;
    c.cam_id = 0;
    c.f = f;
    c.cx = 640.0f; c.cy = 483.0f;
    c.k1 = 0.0f; c.k2 = 0.0f; c.k3 = 0.0f; c.k4 = 0.0f;
    c.r_valid = 802.0f;
    c.max_theta = 1.9f;
    return c;
}

}  // namespace

/* 后端可用性查询在两种构建下都必须有确定答案 */
NF_TEST(vcam_backend_query) {
    const NFVcamBackend b = nf_vcam_backend();
    NF_CHECK(b == NF_VCAM_BACKEND_NONE || b == NF_VCAM_BACKEND_LINKED);
}

/* radial_poly: 两个后端的射线夹角折算像素应 < 0.5 px */
NF_TEST(vcam_crosscheck_radial) {
    if (nf_vcam_backend() != NF_VCAM_BACKEND_LINKED) {
        std::printf("      (vcam 未启用，跳过)\n");
        return;
    }
    /* 用 hanqtech FV 量级的系数：k1≈340 px/rad */
    const NFCameraModel cam = make_radial(340.0f, 0.0f, 0.0f, 0.0f);
    nf_f32 max_dev = -1.0f;
    nf_u32 n_cmp = 0;
    const NFStatus st = nf_vcam_crosscheck(&cam, 400, &max_dev, &n_cmp);
    NF_CHECK_STATUS(st, NF_OK);
    NF_CHECK(n_cmp > 100);
    std::printf("      radial_poly: 比较 %u 点, 最大偏差 %.6f px\n",
                n_cmp, (double)max_dev);
    NF_CHECK_MSG(max_dev < 0.5, "radial_poly 两后端偏差 < 0.5 px");
}

/* KB 模型同样要一致 */
NF_TEST(vcam_crosscheck_kb) {
    if (nf_vcam_backend() != NF_VCAM_BACKEND_LINKED) {
        std::printf("      (vcam 未启用，跳过)\n");
        return;
    }
    const NFCameraModel cam = make_kb(550.0f);
    nf_f32 max_dev = -1.0f;
    nf_u32 n_cmp = 0;
    NF_CHECK_STATUS(nf_vcam_crosscheck(&cam, 400, &max_dev, &n_cmp), NF_OK);
    NF_CHECK(n_cmp > 100);
    std::printf("      KB: 比较 %u 点, 最大偏差 %.6f px\n",
                n_cmp, (double)max_dev);
    NF_CHECK_MSG(max_dev < 0.5, "KB 两后端偏差 < 0.5 px");
}

/**
 * 无效输入必须被两边**一致地**判为无效。
 *
 * 只比"都有效"的点不够 —— 若一端把图像外的点当成有效，
 * 生产路径会拿一个物理上不存在的像素去采样。
 */
NF_TEST(vcam_invalid_consistency) {
    if (nf_vcam_backend() != NF_VCAM_BACKEND_LINKED) {
        std::printf("      (vcam 未启用，跳过)\n");
        return;
    }
    const NFCameraModel cam = make_kb(550.0f);

    /*
     * 原生实现已删，这里只验 vcam 的不可见判定。
     * 判据是"后半球必须 ok=0" —— 少了它，反投影会在车后造出镜像点。
     */
    nf_f32 u = 0, v = 0;
    nf_u8 ok_vc = 1;
    const NFStatus st = nf_vcam_project(&cam, 0.0f, 0.0f, -1000.0f, &u, &v, &ok_vc);
    NF_CHECK_STATUS(st, NF_OK);
    NF_CHECK_EQ_INT(ok_vc, 0);

    /* 光轴前方：应为有效 */
    float u2 = 0, v2 = 0;
    nf_u8 ok_vc2 = 0;
    nf_vcam_project(&cam, 0.0f, 0.0f, 1000.0f, &u2, &v2, &ok_vc2);
    NF_CHECK_EQ_INT(ok_vc2, 1);
}

/* 空指针保护 */
NF_TEST(vcam_null_safety) {
    nf_f32 u = 0, v = 0;
    nf_u8 ok = 0;
    NF_CHECK_EQ_INT(nf_vcam_project(nullptr, 0, 0, 1, &u, &v, &ok), NF_E_PARAM_NULL);
    nf_f32 ray[3];
    NF_CHECK_EQ_INT(nf_vcam_unproject(nullptr, 0, 0, ray, nullptr, nullptr),
                    NF_E_PARAM_NULL);
    nf_f32 dev;
    NF_CHECK_EQ_INT(nf_vcam_crosscheck(nullptr, 100, &dev, nullptr), NF_E_PARAM_NULL);
}

int main() { return nftest::run_all(); }
