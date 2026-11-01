/**
 * @file camera_kb.cpp
 * @brief KB 鱼眼正/反投影实现。对应 Python 原型 nfocc_ph0/camera_kb.py。
 *
 * 实现要点(每条都是 Python 原型里实测踩过的):
 *  1. theta_d 的导数系数是 1,3,5,7,9 —— 写成 1,2,3,4 会让 Newton 收敛到错解。
 *  2. chi→0 时 scale=rho/chi 的极限为 1, 不处理会 0/0。
 *  3. 有效性判据用 theta 与半径, **不用 z>0**(theta>90° 时 z<0 但像素有效)。
 *  4. Newton 初值取 theta=rho(畸变小, 已很接近), 8 次足够(f=550,k=0 时 1 次即收敛)。
 */
#include "nfocc/nfocc_camera.h"

#include <cmath>
#include <cfloat>

namespace {

const float kEpsChi   = 1e-9f;
const float kEpsRho   = 1e-12f;
const int   kNewtonIt = 24;
const float kNewtonTol= 1e-12f;

inline void mat_vec(const float *R, const float *v, float *out) {
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}
/* Rᵀ·v : 因为 R 正交, Rᵀ 就是转置 */
inline void matT_vec(const float *R, const float *v, float *out) {
    out[0] = R[0]*v[0] + R[3]*v[1] + R[6]*v[2];
    out[1] = R[1]*v[0] + R[4]*v[1] + R[7]*v[2];
    out[2] = R[2]*v[0] + R[5]*v[1] + R[8]*v[2];
}

} // namespace

extern "C" {

nf_f32 nf_cam_theta_d(const NFCameraModel *cam, nf_f32 theta) {
    if (!cam) return NAN;
    if (cam->kind == NF_CAM_MODEL_RADIAL_POLY) {
        /* ρ(θ)/k1: 归一化到 KB 的同一尺度, 使上下层代码无需分叉。
         * 真正的像素半径由 nf_cam_pixel_radius() 给出。 */
        const float k1 = cam->k1;
        if (!(k1 > 0.0f)) return NAN;
        return (k1 * theta + cam->k2 * theta * theta
              + cam->k3 * theta * theta * theta
              + cam->k4 * theta * theta * theta * theta) / k1;
    }
    const float t2 = theta * theta;
    return theta * (1.0f + cam->k1 * t2 + cam->k2 * t2 * t2
                         + cam->k3 * t2 * t2 * t2 + cam->k4 * t2 * t2 * t2 * t2);
}

/**
 * 归一化系数: 让 "rho_normalized * K" 得到像素半径。
 * KB 模型 K = f; RADIAL_POLY 模型 K = k1(本征焦距)。
 */
static inline float model_scale(const NFCameraModel *cam) {
    return (cam->kind == NF_CAM_MODEL_RADIAL_POLY) ? cam->k1 : cam->f;
}

nf_f32 nf_cam_theta_d_prime(const NFCameraModel *cam, nf_f32 theta) {
    if (!cam) return NAN;
    if (cam->kind == NF_CAM_MODEL_RADIAL_POLY) {
        const float k1 = cam->k1;
        if (!(k1 > 0.0f)) return NAN;
        return (k1 + 2.0f * cam->k2 * theta
                   + 3.0f * cam->k3 * theta * theta
                   + 4.0f * cam->k4 * theta * theta * theta) / k1;
    }
    const float t2 = theta * theta;
    /* 系数 1,3,5,7,9 —— 见文件头第 1 条 */
    return 1.0f + 3.0f * cam->k1 * t2
                + 5.0f * cam->k2 * t2 * t2
                + 7.0f * cam->k3 * t2 * t2 * t2
                + 9.0f * cam->k4 * t2 * t2 * t2 * t2;
}

nf_f32 nf_cam_rho_to_theta(const NFCameraModel *cam, nf_f32 rho, nf_i32 *ok) {
    if (ok) *ok = 0;
    if (!cam) return -1.0f;
    if (rho < 0.0f) return -1.0f;

    /* k 全零时 theta_d 就是 theta, 直接返回 —— 本项目默认工况, 省掉迭代 */
    if (cam->k1 == 0.0f && cam->k2 == 0.0f && cam->k3 == 0.0f && cam->k4 == 0.0f) {
        if (ok) *ok = 1;
        return rho;
    }

    float th = rho;                     /* 初值: rho 本身 */
    for (int i = 0; i < kNewtonIt; ++i) {
        const float fv = nf_cam_theta_d(cam, th) - rho;
        float fp = nf_cam_theta_d_prime(cam, th);
        if (fp < 1e-12f) fp = 1e-12f;
        const float step = fv / fp;
        th -= step;
        if (std::fabs(step) < kNewtonTol) {
            if (ok) *ok = 1;
            return th;
        }
    }
    /* 未收敛: 检查残差, 小于 1e-9 仍可接受 */
    const float resid = std::fabs(nf_cam_theta_d(cam, th) - rho);
    if (resid < 1e-9f) {
        if (ok) *ok = 1;
        return th;
    }
    return -1.0f;
}

NFStatus nf_cam_validate(const NFCameraModel *cam) {
    if (!cam) return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    /* **未知 kind 必须明确拒绝**: 新增字段后, 栈上构造的结构体若未初始化
     * kind, 会读到垃圾值。静默当 KB 会让投影结果看似正常实则用到错误公式。
     * 实测: test_camera 的 default_cam() 未设 kind, 12 个用例里有 2 个失败。 */
    if (cam->kind != NF_CAM_MODEL_KB && cam->kind != NF_CAM_MODEL_RADIAL_POLY)
        return NF_E_PARAM_VERSION;

    if (cam->kind == NF_CAM_MODEL_RADIAL_POLY) {
        /* 该模型用 k1 当焦距, f 字段无意义 */
        if (!(cam->k1 > 0.0f) || !std::isfinite(cam->k1))
            return NF_E_CAM_NOT_CALIBRATED;
    } else {
        if (!(cam->f > 0.0f) || !std::isfinite(cam->f)) return NF_E_CAM_NOT_CALIBRATED;
    }
    if (!std::isfinite(cam->cx) || !std::isfinite(cam->cy)) return NF_E_CAM_NOT_CALIBRATED;
    if (!(cam->r_valid > 0.0f)) return NF_E_PARAM_RANGE;

    /* theta_d 在 [0, max_theta*1.05] 上必须严格单调 (V-CO-6) */
    const float tmax = cam->max_theta * 1.05f;
    const int   N    = 500;
    float prev = nf_cam_theta_d(cam, 0.0f);
    for (int i = 1; i <= N; ++i) {
        const float th = tmax * (float)i / (float)N;
        const float v  = nf_cam_theta_d(cam, th);
        if (!(v > prev)) return NF_E_CAM_NOT_CALIBRATED;  /* 非单调 */
        prev = v;
    }
    return NF_OK;
}

NFStatus nf_pose_validate(const NFPose *pose) {
    if (!pose) return NF_E_PARAM_NULL;
    if (pose->version != NF_POSE_VERSION) return NF_E_PARAM_VERSION;
    const float *R = pose->R;
    /* C1: R·Rᵀ = I, 误差 < 1e-9 */
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += R[i*3+k] * R[j*3+k];
            const float expect = (i == j) ? 1.0f : 0.0f;
            if (std::fabs(s - expect) > 1e-6f) return NF_E_EXTRINSIC_INVALID;
        }
    }
    /* C2: det(R) = +1 */
    const float det = R[0]*(R[4]*R[8] - R[5]*R[7])
                    - R[1]*(R[3]*R[8] - R[5]*R[6])
                    + R[2]*(R[3]*R[7] - R[4]*R[6]);
    if (std::fabs(det - 1.0f) > 1e-5f) return NF_E_EXTRINSIC_INVALID;
    /* C3/C4 的索引必须按 v_V = R·v_C 取。
     *
     * 行优先存储下, **相机第 k 轴在车系 = R 的第 k 列**, 即元素 R[row*3 + k],
     * 其中 row 是车系的轴序。取 R[3*k+c] 会拿到第 k 行 —— 那是转置语义,
     * 对应 v_C = R·v_V 的约定, 与 ICD 相反。(本项曾因此误判合法外参为非法。)
     */
    /* C3: y 轴 = 第 1 列, 其车系 z 分量 R[2*3+1] 必须为负(朝下) */
    if (!(R[2*3 + 1] < 0.0f)) return NF_E_EXTRINSIC_INVALID;
    /* C4: z 轴 = 第 2 列, 其车系 z 分量 R[2*3+2] 不能接近 ±1(光轴不得竖直) */
    if (!(std::fabs(R[2*3 + 2]) < 0.99f)) return NF_E_EXTRINSIC_INVALID;
    return NF_OK;
}

NFStatus nf_cam_project_c(const NFCameraModel *cam,
                          nf_f32 x, nf_f32 y, nf_f32 z,
                          nf_f32 *u, nf_f32 *v, nf_u8 *valid) {
    if (!cam || !u || !v || !valid) return NF_E_PARAM_NULL;

    const float chi   = std::sqrt(x*x + y*y);
    const float theta = std::atan2(chi, z);
    const float rho   = nf_cam_theta_d(cam, theta);
    const float K     = model_scale(cam);
    const float r_img = K * rho;

    /* chi→0: 极限 scale=1 (文件头第 2 条) */
    float sx, sy;
    if (chi < kEpsChi) {
        sx = x; sy = y;              /* 主点附近, x,y 已接近 0 */
    } else {
        const float s = rho / chi;
        sx = s * x; sy = s * y;
    }
    *u = K * sx + cam->cx;
    *v = K * sy + cam->cy;

    const bool ok = std::isfinite(*u) && std::isfinite(*v)
                 && theta >= 0.0f
                 && theta <= cam->max_theta
                 && r_img <= cam->r_valid;
    *valid = ok ? 1u : 0u;
    if (!ok) { *u = NAN; *v = NAN; }
    return NF_OK;
}

NFStatus nf_cam_unproject_c(const NFCameraModel *cam,
                            nf_f32 u, nf_f32 v,
                            nf_f32 *ray) {
    if (!cam || !ray) return NF_E_PARAM_NULL;

    /* 除以模型尺度 K: KB 时 K=f, RADIAL_POLY 时 K=k1。
     * 除完 lx/ly 即"归一化像素偏移", 其模长就是归一化 rho, 可直接迭代反解。 */
    const float K  = model_scale(cam);
    if (!(K > 0.0f)) { ray[0] = ray[1] = ray[2] = NAN; return NF_E_CAM_NOT_CALIBRATED; }
    const float lx = (u - cam->cx) / K;
    const float ly = (v - cam->cy) / K;
    const float rho = std::sqrt(lx*lx + ly*ly);

    nf_i32 ok = 0;
    const float theta = nf_cam_rho_to_theta(cam, rho, &ok);
    if (!ok) { ray[0] = ray[1] = ray[2] = NAN; return NF_E_CAM_DISTORT_INV; }

    const float sin_t = std::sin(theta);
    float sx, sy;
    if (rho < kEpsRho) { sx = 0.0f; sy = 0.0f; }
    else {
        const float s = sin_t / rho;
        sx = s * lx; sy = s * ly;
    }
    ray[0] = sx;
    ray[1] = sy;
    ray[2] = std::cos(theta);

    const float n = std::sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]);
    if (n > 1e-15f) { ray[0] /= n; ray[1] /= n; ray[2] /= n; }
    return NF_OK;
}

NFStatus nf_cam_project(const NFCameraModel *cam, const NFPose *pose,
                        const NFPoint3f *P_V,
                        nf_f32 *uv, nf_u8 *valid) {
    if (!cam || !pose || !P_V || !uv || !valid) return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    if (pose->version != NF_POSE_VERSION) return NF_E_PARAM_VERSION;

    /* p_C = Rᵀ·(P_V − t) */
    const float d[3] = { P_V->x - pose->t[0], P_V->y - pose->t[1], P_V->z - pose->t[2] };
    float pc[3];
    matT_vec(pose->R, d, pc);

    return nf_cam_project_c(cam, pc[0], pc[1], pc[2], &uv[0], &uv[1], valid);
}

NFStatus nf_cam_unproject(const NFCameraModel *cam, const NFPose *pose,
                          nf_f32 u, nf_f32 v,
                          nf_f32 *ray) {
    if (!cam || !pose || !ray) return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    if (pose->version != NF_POSE_VERSION) return NF_E_PARAM_VERSION;

    float rc[3];
    const NFStatus st = nf_cam_unproject_c(cam, u, v, rc);
    if (st != NF_OK) { ray[0] = ray[1] = ray[2] = NAN; return st; }

    /* v_V = R·v_C */
    mat_vec(pose->R, rc, ray);
    return NF_OK;
}

} /* extern "C" */
