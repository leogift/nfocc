/**
 * @file vcam_bridge.cpp
 * @brief vcam 后端桥接实现。
 *
 * 核心工作是把 `NFCameraModel`（float32、两种模型）转成
 * `VCCameraModel`（float64、三种模型 + 仿射），再转发调用。
 *
 * ## 精度注意
 * NFOCC 用 float32，vcam 用 float64。转换是**升宽**（无损），
 * 但 vcam 返回 float64 后要压回 float32。两者的差异应在 1e-3 px 量级，
 * `nf_vcam_crosscheck` 就是用来量化这个数的。
 *
 * ## 为什么必须有回退
 * vcam 是可选依赖。`NF_WITH_VCAM` 未定义时本文件编成"全部返回
 * UNSUPPORTED"的桩，让调用方能统一按返回值判断，不需要 `#ifdef`
 * 散落在业务代码里。
 */
#include "nfocc/nfocc_vcam_bridge.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstring>

#ifdef NF_WITH_VCAM
#include "vcam/vc_camera.h"
#endif

extern "C" {

NFVcamBackend nf_vcam_backend(void) {
#ifdef NF_WITH_VCAM
    return NF_VCAM_BACKEND_LINKED;
#else
    return NF_VCAM_BACKEND_NONE;
#endif
}

#ifdef NF_WITH_VCAM

namespace {

/**
 * NFCameraModel -> VCCameraModel。
 *
 * 字段映射（易错处都标出来了）：
 *   KB:          f  -> fx=fy=f,  k1..k4 -> k[0..3]
 *   RADIAL_POLY: k1..k4 是 rho(theta) 的系数，**k1 就是小角度等效焦距**
 *                （像素/弧度），不是畸变系数。填 k[0..3] 时顺序不能错，
 *                否则 vc_cam_solve_theta 会求出错误的 theta。
 *
 * @return 0 = 不支持该模型
 */
int to_vcam(const NFCameraModel *src, VCCameraModel *dst) {
    std::memset(dst, 0, sizeof(*dst));
    dst->version = VC_CAMERA_MODEL_VERSION;

    if (src->kind == NF_CAM_MODEL_KB) {
        /* NFOCC 的 NFCameraModel 不含图像尺寸，用 r_valid 反推一个保守值。
         * 这里只需要 project/unproject，尺寸只影响 validate 的边界检查。 */
        vc_f64 f = (vc_f64)src->f;
        if (vc_cam_make_kb(dst, 4096, 4096, f, f,
                           (vc_f64)src->cx, (vc_f64)src->cy,
                           (vc_f64)src->k1, (vc_f64)src->k2,
                           (vc_f64)src->k3, (vc_f64)src->k4) != VC_OK)
            return 0;
        return 1;
    }

    if (src->kind == NF_CAM_MODEL_RADIAL_POLY) {
        const vc_f64 k[4] = { (vc_f64)src->k1, (vc_f64)src->k2,
                              (vc_f64)src->k3, (vc_f64)src->k4 };
        if (vc_cam_make_radial_poly(dst, 4096, 4096, k,
                                    (vc_f64)src->cx, (vc_f64)src->cy,
                                    1.0 /* aspect_ratio */) != VC_OK)
            return 0;
        return 1;
    }
    return 0;
}

}  // namespace

NFStatus nf_vcam_project(const NFCameraModel *cam, nf_f32 X, nf_f32 Y, nf_f32 Z,
                         nf_f32 *u, nf_f32 *v, nf_u8 *ok) {
    if (!cam || !u || !v || !ok) return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    *ok = 0;

    VCCameraModel vc;
    if (!to_vcam(cam, &vc)) return NF_E_ENGINE_UNSUPPORTED;

    vc_f64 uo = -1.0, vo = -1.0;
    const VCStatus st = vc_cam_project(&vc, (vc_f64)X, (vc_f64)Y, (vc_f64)Z,
                                       &uo, &vo);
    if (st != VC_OK) return NF_E_ENGINE_UNSUPPORTED;

    /* vc_cam_project 在"不可见"时返回 u=v=-1 且 VC_OK，与 NFOCC 的 ok 语义一致 */
    if (uo < 0.0 || vo < 0.0) { *ok = 0; return NF_OK; }

    /**
     * **必须补上 max_theta 检查**。
     *
     * vcam 的 `vc_project_fisheye` 用 `atan2(r_xy, Z)` 求入射角，
     * Z<0 时给出钝角 —— 数学上正确（鱼眼确实能拍到后半球），
     * 但 **NFOCC 的可见性判据是 `theta <= max_theta`**
     * （见 `nf_cam_project_c` 的 ok 条件），不是 Z>0。
     *
     * 实测差异：X=Y=0, Z=-1000 时
     *   NFOCC: theta = atan2(0,-1000) = π ≈ 3.14 > max_theta(1.9) -> 无效
     *   vcam : 不带这个检查, 直接按 theta 求 rho 并投影 -> 报"有效"
     * 少了这一步，后方视口会拿物理上不存在的像素去采样。
     */
    {
        const vc_f64 chi = std::sqrt((vc_f64)X * X + (vc_f64)Y * Y);
        const vc_f64 th  = std::atan2(chi, (vc_f64)Z);
        if (th > (vc_f64)cam->max_theta) { *ok = 0; return NF_OK; }

        /**
         * **r_img 的量纲要按 NFOCC 的约定算**。
         *
         * `nf_cam_project_c` 里 `rho = nf_cam_theta_d(cam, theta)` 返回的是
         * **归一化**角度（KB: theta + k1θ³...，RADIAL_POLY: 除以 k1 后的值），
         * 然后 `r_img = K * rho` 才得到像素半径。
         *
         * 而 vcam 的 `vc_cam_rho` 返回的是**已经带量纲**的 rho
         * （RADIAL_POLY 下直接是像素）。再乘一次 K 会把半径放大 K 倍，
         * 于是每个点都判"超 r_valid" —— 实测三路相机全被判不可见。
         *
         * 所以这里直接用 vcam 的 rho，**不再乘 K**。
         */
        const vc_f64 r_img = vc_cam_rho(&vc, th);
        if (cam->r_valid > 0.0f && r_img > (vc_f64)cam->r_valid) {
            *ok = 0; return NF_OK;
        }
    }

    *u = (nf_f32)uo;
    *v = (nf_f32)vo;
    *ok = 1;
    return NF_OK;
}

NFStatus nf_vcam_unproject(const NFCameraModel *cam, nf_f32 u, nf_f32 v,
                           nf_f32 ray[3], nf_f32 *theta_deg, nf_u8 *converged) {
    if (!cam || !ray) return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    if (converged) *converged = 0;

    VCCameraModel vc;
    if (!to_vcam(cam, &vc)) return NF_E_ENGINE_UNSUPPORTED;

    vc_f64 r[3] = { 0.0, 0.0, 0.0 };
    vc_f64 th = 0.0;
    vc_i32 cv = 0;
    const VCStatus st = vc_cam_unproject(&vc, (vc_f64)u, (vc_f64)v, r, &th, &cv);
    if (st != VC_OK) return NF_E_ENGINE_UNSUPPORTED;

    ray[0] = (nf_f32)r[0];
    ray[1] = (nf_f32)r[1];
    ray[2] = (nf_f32)r[2];
    if (theta_deg) *theta_deg = (nf_f32)th;
    if (converged) *converged = (cv != 0) ? 1u : 0u;
    return NF_OK;
}

NFStatus nf_vcam_crosscheck(const NFCameraModel *cam, nf_u32 n_samples,
                            nf_f32 *out_max_dev_px, nf_u32 *out_n_compared) {
    if (!cam || !out_max_dev_px) return NF_E_PARAM_NULL;
    if (n_samples == 0) return NF_E_PARAM_RANGE;
    *out_max_dev_px = -1.0f;
    if (out_n_compared) *out_n_compared = 0;

    /* 在图像上取 n x n 网格（n_samples 是总点数，向上取整到方阵） */
    nf_u32 side = (nf_u32)std::ceil(std::sqrt((double)n_samples));
    if (side < 1) side = 1;

    /* NFOCC 的 NFCameraModel 不含图像尺寸，用 r_valid 作采样半径 */
    const float rc = (cam->r_valid > 0.0f) ? cam->r_valid : 800.0f;

    float max_dev = 0.0f;
    nf_u32 compared = 0;

    for (nf_u32 iy = 0; iy < side; ++iy) {
        for (nf_u32 ix = 0; ix < side; ++ix) {
            const float fu = (side > 1) ? (float)ix / (float)(side - 1) : 0.5f;
            const float fv = (side > 1) ? (float)iy / (float)(side - 1) : 0.5f;

            /* 采样点覆盖整个有效圆 */
            const float u = cam->cx + (fu * 2.0f - 1.0f) * rc;
            const float v = cam->cy + (fv * 2.0f - 1.0f) * rc;

            /* 两条路各自 unproject 出射线，再各自 project 回去，比像素差 */
            float r_a[3] = { 0, 0, 0 }, r_b[3] = { 0, 0, 0 };
            nf_u8 ok_a = 0, ok_b = 0;
            if (nf_cam_unproject_c(cam, u, v, r_a) != NF_OK) continue;
            if (nf_vcam_unproject(cam, u, v, r_b, nullptr, &ok_b) != NF_OK) continue;
            if (!ok_b) continue;

            /* 两条射线的夹角（度）—— 比再投影更直接反映模型差异 */
            const float dot = r_a[0]*r_b[0] + r_a[1]*r_b[1] + r_a[2]*r_b[2];
            const float ang = std::acos(dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot));
            /* 折算成像素：用该点附近的局部焦距 */
            const float f_scale = (cam->kind == NF_CAM_MODEL_KB)
                                ? cam->f : cam->k1;
            const float dev_px = std::fabs(ang) * f_scale;
            if (dev_px > max_dev) max_dev = dev_px;
            ++compared;
            (void)ok_a;
        }
    }

    *out_max_dev_px = max_dev;
    if (out_n_compared) *out_n_compared = compared;
    return (compared > 0) ? NF_OK : NF_E_CAM_NOT_CALIBRATED;
}

#else /* !NF_WITH_VCAM —— 桩实现 */

/*
 * **参数检查优先于"功能不可用"**。
 *
 * 顺序反了会出现：`nf_vcam_project(nullptr, ...)` 返回 UNSUPPORTED 而不是
 * PARAM_NULL —— 调用方拿到 UNSUPPORTED 会去回退到原生实现，
 * 而原生实现又会拿到空指针。让空指针先报错，故障定位才落在正确的层。
 */
NFStatus nf_vcam_project(const NFCameraModel *cam, nf_f32, nf_f32, nf_f32,
                         nf_f32 *u, nf_f32 *v, nf_u8 *ok) {
    if (!cam || !u || !v || !ok) return NF_E_PARAM_NULL;
    return NF_E_ENGINE_UNSUPPORTED;
}

NFStatus nf_vcam_unproject(const NFCameraModel *cam, nf_f32, nf_f32,
                           nf_f32 *ray, nf_f32 *, nf_u8 *) {
    if (!cam || !ray) return NF_E_PARAM_NULL;
    return NF_E_ENGINE_UNSUPPORTED;
}

NFStatus nf_vcam_crosscheck(const NFCameraModel *cam, nf_u32 n,
                            nf_f32 *dev, nf_u32 *) {
    if (!cam || !dev) return NF_E_PARAM_NULL;
    if (n == 0) return NF_E_PARAM_RANGE;
    return NF_E_ENGINE_UNSUPPORTED;
}

#endif /* NF_WITH_VCAM */

} /* extern "C" */
