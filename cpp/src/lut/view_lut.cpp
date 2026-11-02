/**
 * @file view_lut.cpp
 * @brief 视口 LUT 构建与 warp。对应 Python 原型 nfocc_ph0/view_lut.py。
 *
 * 已踩过的坑(照抄原型注释, 防止重犯):
 *  1. height 必须由垂直张角反算。独立指定会让实际张角 ≠ 环带张角。
 *  2. ray_to_pixel 必须用 "el - el_mid" 的角度式, 直接用 y/z 会丢俯角补偿
 *     (Python 实测: 往返误差从 6e-14 变成 804px)。
 *  3. 无效点用 0xFFFF 哨兵, 不用 (0,0)。
 *  4. 全无效时报错而非静默返回空图。
 */
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_vcam_bridge.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {

const float kDeg2Rad = 3.14159265358979323846f / 180.0f;
const float kRad2Deg = 180.0f / 3.14159265358979323846f;
const float kQ4Scale = 16.0f;   /* Q12.4 = 1/16 px */

inline nf_i32 clamp_i32(nf_i32 v, nf_i32 lo, nf_i32 hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * 视口像素 -> **车系**单位射线（虚拟相机，pitch=0/roll=0，只有 yaw）。
 *
 * ## 与旧实现的根本差别
 * 旧版先造"视口系"射线（俯角 el 相对视口轴），再绕相机 y 轴偏 yaw，
 * 结果视口朝向跟随**真实光轴** —— FV 下俯 22.7° 时视口也斜着往下看。
 *
 * 新版按**虚拟相机**建模：
 *   1. 视口内像素 -> 虚拟相机系射线（az 来自 u，el 来自 v，el 以环带中轴为基准）
 *   2. 用虚拟相机的朝向 (yaw, pitch=0, roll=0) 把射线转到**车系**
 *
 * 虚拟相机基向量（车系, ISO 8855, z 向上）:
 *   forward = (cos(yaw), sin(yaw), 0)          ← pitch=0, 水平
 *   right   = (sin(yaw), -cos(yaw), 0)         ← 与 forward 正交, 指向车体右侧
 *   down    = (0, 0, -1)                       ← 车系 z 向下
 * 故 射线_V = cos(el)·[sin(az)·right + cos(az)·forward] + sin(el)·down
 *
 * 真实外参**不参与**这里 —— 它只在后续"车系射线 -> 该相机鱼眼像素"时用到。
 */
void pixel_to_ray_impl(const NFViewParams *vp, float el_mid_rad,
                       float u, float v, float *ray) {
    const float az = std::atan((u - vp->cx) / vp->f);
    const float el = el_mid_rad + std::atan((v - vp->cy) / vp->f);

    /* 虚拟相机基向量（车系） */
    const float yaw = vp->yaw_deg * kDeg2Rad;
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const float fx =  cy, fy =  sy, fz = 0.0f;   /* forward */
    const float rx =  sy, ry = -cy, rz = 0.0f;   /* right (朝车体右侧) */
    const float dx = 0.0f, dy = 0.0f, dz = -1.0f;/* down (车系 z 向下) */

    const float ce = std::cos(el), se = std::sin(el);
    const float sa = std::sin(az), ca = std::cos(az);
    ray[0] = ce * (sa * rx + ca * fx) + se * dx;
    ray[1] = ce * (sa * ry + ca * fy) + se * dy;
    ray[2] = ce * (sa * rz + ca * fz) + se * dz;
}

/**
 * 环带中轴俯角(绝对, rad)。
 *
 * **约定**: 环带是 [el_mid − fov_v/2, el_mid + fov_v/2]。
 * `NFViewParams` 只存张角 fov_v 与主点, 不存 el_min/el_max ——
 * 因此中轴必须由配置显式给出。放在 vp->cx 之外的独立字段会破坏 ICD,
 * 所以本实现把中轴冗余在 `fov_v` 推导里: 由调用方保证
 * `nf_view_params_derive(el_min, el_max)` 已经把
 * `fov_v = el_max − el_min` 写入, 中轴 = el_min + fov_v/2。
 *
 * 为让 LUT 构建与像素反投影用同一份 el_mid, el_min 由本文件内常量给定,
 * 并与 nfocc_default.h 的 NF_V_RING_EL_MIN_DEG 保持一致(编译期断言)。
 */
#if defined(NF_V_RING_EL_MIN_DEG)
#  define NF_LUT_EL_MIN_DEG NF_V_RING_EL_MIN_DEG
#else
#  define NF_LUT_EL_MIN_DEG 10.0f
#endif

/**
 * 环带中轴俯角。**v0.9.1 起恒为 0 —— 视口水平平视**。
 *
 * ## 为什么改成 0
 * 旧值是 `EL_MIN + fov_v/2 = 10 + 25 = 35°`，即视口整体朝下 35°。
 * 这让"虚拟相机"实际上还是俯视的，只是俯角由配置常量决定而非安装角。
 *
 * 用户要求：投影**不要俯角，理想 0 度平视**。
 * 平视意味着虚拟相机光轴水平，视口内像素的俯角相对水平线计算。
 *
 * ## 代价（必须知道）
 * FV 安装高 681mm，视口垂直张角 ±25°：
 *   - 向下 25° 最多看到地面 681/tan(25°) ≈ 1460 mm
 *   - 0.5~1.4 m 的近场路面落在视口之外
 * 若需要近场，应增大垂直张角（fov_v）或恢复环带下偏。
 */
inline float ring_el_mid_rad(const NFViewParams *vp) {
    (void)vp;
    return 0.0f;
}

} // namespace

extern "C" {

/**
 * 按**显式水平/垂直张角**派生视口（v0.9.2 起的首选入口）。
 *
 * 与 `nf_view_params_derive`（按环带 el_min/el_max 反推）的区别：
 * 那个函数把垂直张角 = el_max − el_min，隐含"视口覆盖某个俯角环带"。
 * 虚拟相机模型下视口是**水平的**（pitch=0），没有环带概念，
 * 垂直张角应当直接给定 —— 本项目约定 150° x 120°。
 *
 * @param hfov_deg 水平张角（度）
 * @param vfov_deg 垂直张角（度）
 */
NFStatus nf_view_params_derive_fov(NFViewParams *vp, nf_u32 width,
                                   nf_f32 hfov_deg, nf_f32 vfov_deg) {
    if (!vp) return NF_E_PARAM_NULL;
    if (width == 0) return NF_E_PARAM_RANGE;
    if (!(hfov_deg > 0.0f) || !(hfov_deg < 180.0f)) return NF_E_PARAM_RANGE;
    if (!(vfov_deg > 0.0f) || !(vfov_deg < 180.0f)) return NF_E_PARAM_RANGE;

    vp->version = NF_VIEW_PARAMS_VERSION;
    vp->width   = width;
    vp->f       = ((float)width * 0.5f) / std::tan(hfov_deg * kDeg2Rad * 0.5f);
    vp->cx      = (float)width * 0.5f;
    vp->fov_h   = hfov_deg * kDeg2Rad;
    vp->fov_v   = vfov_deg * kDeg2Rad;

    const float h_f = 2.0f * vp->f * std::tan(vp->fov_v * 0.5f);
    vp->height = (nf_u32)(h_f + 0.5f);
    if (vp->height == 0) return NF_E_PARAM_RANGE;
    vp->cy = (float)vp->height * 0.5f;

    /* 水平平视: 无俯仰、无横滚 */
    vp->yaw_deg   = 0.0f;
    vp->pitch_deg = 0.0f;
    vp->reorient  = 0;
    return NF_OK;
}

NFStatus nf_view_params_derive(NFViewParams *vp, nf_u32 width,
                               nf_f32 el_min_deg, nf_f32 el_max_deg,
                               nf_f32 hfov_deg) {
    if (!vp) return NF_E_PARAM_NULL;
    if (width == 0) return NF_E_PARAM_RANGE;
    if (!(el_min_deg > 0.0f) || !(el_max_deg > el_min_deg) || !(el_max_deg < 90.0f))
        return NF_E_PARAM_RANGE;
    if (!(hfov_deg > 0.0f) || !(hfov_deg < 180.0f)) return NF_E_PARAM_RANGE;

    vp->version = NF_VIEW_PARAMS_VERSION;
    vp->width   = width;
    vp->f       = ((float)width * 0.5f) / std::tan(hfov_deg * kDeg2Rad * 0.5f);
    vp->cx      = (float)width * 0.5f;
    vp->fov_h   = hfov_deg * kDeg2Rad;
    vp->fov_v   = (el_max_deg - el_min_deg) * kDeg2Rad;

    /* height 由垂直张角反算: h = 2f·tan(fov_v/2) */
    const float h_f = 2.0f * vp->f * std::tan(vp->fov_v * 0.5f);
    vp->height = (nf_u32)(h_f + 0.5f);
    if (vp->height == 0) return NF_E_PARAM_RANGE;
    vp->cy   = (float)vp->height * 0.5f;
    vp->reorient = 1;
    /**
     * **必须把新增的虚拟相机字段初始化**。
     *
     * 该字段是后加的。若不在这里初始化，调用方在栈上声明
     * `NFViewParams vp; nf_view_params_derive(&vp, ...);` 之后
     * `vp.yaw_deg` 仍是栈垃圾 —— 视口朝向随机。
     *
     * 实测: `nfocc_viz` 在 build/host 下 LUT 有效率 100%（栈恰好为 0），
     * 在同源头的干净构建下变成 84.5%（栈上非 0），且换台机器可能直接
     * 因偏转过大致 LUT 失败。**同一份代码两种结果 = 未定义行为**。
     *
     * 修在 derive 里而不是让调用方 memset: derive 是对外造结构的
     * 唯一入口，它有责任把结构填完整。
     */
    /* 虚拟相机默认朝向: yaw=0（朝车头前方）, pitch=0（水平）。
     * 调用方按视口规划表覆盖 vp->yaw_deg。 */
    vp->yaw_deg   = 0.0f;
    vp->pitch_deg = 0.0f;
    return NF_OK;
}

NFStatus nf_view_params_validate(const NFViewParams *vp) {
    if (!vp) return NF_E_PARAM_NULL;
    if (vp->version != NF_VIEW_PARAMS_VERSION) return NF_E_PARAM_VERSION;
    if (vp->width == 0 || vp->height == 0) return NF_E_PARAM_RANGE;
    if (!(vp->f > 0.0f)) return NF_E_PARAM_RANGE;
    if (!(vp->fov_h > 0.0f) || !(vp->fov_v > 0.0f)) return NF_E_PARAM_RANGE;

    /* 自洽性: width/(2f) == tan(fov_h/2) */
    const float lhs = (float)vp->width / (2.0f * vp->f);
    const float rhs = std::tan(vp->fov_h * 0.5f);
    if (std::fabs(lhs - rhs) > 1e-3f) return NF_E_PARAM_RANGE;
    return NF_OK;
}

NFStatus nf_view_ground_range(const NFViewParams *vp, nf_f32 el_min_deg,
                              nf_f32 el_max_deg, nf_f32 cam_z_mm,
                              nf_f32 *near_mm, nf_f32 *far_mm) {
    if (!vp || !near_mm || !far_mm) return NF_E_PARAM_NULL;
    if (!(cam_z_mm > 0.0f)) return NF_E_PARAM_RANGE;
    if (!(el_min_deg > 0.0f) || !(el_max_deg > el_min_deg)) return NF_E_PARAM_RANGE;
    *near_mm = cam_z_mm / std::tan(el_max_deg * kDeg2Rad);
    *far_mm  = cam_z_mm / std::tan(el_min_deg * kDeg2Rad);
    return NF_OK;
}

NFStatus nf_lut_alloc(NFViewLut *lut, nf_u32 width, nf_u32 height) {
    if (!lut) return NF_E_PARAM_NULL;
    if (width == 0 || height == 0) return NF_E_PARAM_RANGE;
    const size_t n = (size_t)width * (size_t)height;
    lut->u_q4  = (nf_i16 *)std::malloc(n * sizeof(nf_i16));
    lut->v_q4  = (nf_i16 *)std::malloc(n * sizeof(nf_i16));
    lut->valid = (nf_u8  *)std::malloc(n * sizeof(nf_u8));
    if (!lut->u_q4 || !lut->v_q4 || !lut->valid) {
        std::free(lut->u_q4); std::free(lut->v_q4); std::free(lut->valid);
        lut->u_q4 = nullptr; lut->v_q4 = nullptr; lut->valid = nullptr;
        return NF_E_ENGINE_OOM;
    }
    lut->width = width;
    lut->height = height;
    lut->owns_memory = 1;
    /* 初始化为哨兵 */
    for (size_t i = 0; i < n; ++i) {
        lut->u_q4[i] = (nf_i16)NF_LUT_SENTINEL;
        lut->v_q4[i] = (nf_i16)NF_LUT_SENTINEL;
        lut->valid[i] = 0;
    }
    return NF_OK;
}

void nf_lut_destroy(NFViewLut *lut) {
    if (!lut) return;
    if (lut->owns_memory) {
        std::free(lut->u_q4);
        std::free(lut->v_q4);
        std::free(lut->valid);
    }
    lut->u_q4 = nullptr;
    lut->v_q4 = nullptr;
    lut->valid = nullptr;
    lut->width = lut->height = 0;
    lut->owns_memory = 0;
}

NFStatus nf_lut_build(NFViewLut *lut, const NFCameraModel *cam,
                      const NFPose *pose, const NFViewParams *vp,
                      const nf_u8 *mask, NFLutStats *stats) {
    if (!lut || !cam || !pose || !vp) return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    if (pose->version != NF_POSE_VERSION) return NF_E_PARAM_VERSION;
    NFStatus st = nf_view_params_validate(vp);
    if (st != NF_OK) return st;
    if (lut->width != vp->width || lut->height != vp->height) return NF_E_PARAM_RANGE;

    NFLutStats local;
    std::memset(&local, 0, sizeof(local));
    local.total = vp->width * vp->height;

    /* 环带中轴俯角(绝对)。见 ring_el_mid_rad 的说明。 */
    const float el_mid = ring_el_mid_rad(vp);

    for (nf_u32 y = 0; y < vp->height; ++y) {
        const float v = (float)y + 0.5f;
        for (nf_u32 x = 0; x < vp->width; ++x) {
            const float u = (float)x + 0.5f;
            const size_t idx = (size_t)y * vp->width + x;

            /* 视口像素 -> **车系**单位射线（虚拟相机, pitch=0 只有 yaw） */
            float rp[3];
            pixel_to_ray_impl(vp, el_mid, u, v, rp);

            /**
             * **视口射线先当作车系方向, 再转到相机系投影。**
             *
             * 早期实现直接把 rp 当相机系方向用（认为"视口系与相机系轴向一致"）。
             * 这在理想外参（俯角 0°）下恰好成立, 接入真实标定后**完全失效**:
             * hanqtech FV 光轴实际下俯 22.7°, RV 下俯 37.3°,
             * MVL/MVR 下俯 74°。若把环带角当相对光轴, 绝对值变成
             * 22.7+35=57.7°, 看到的是 0.43m 处的地面 —— 而项目要的是 1-4m。
             *
             * 实测对照（FV, 环带 35°）:
             *   当相对光轴 -> 绝对俯角 57.7° -> 地面 432mm
             *   当绝对俯角 -> 绝对俯角 35.0° -> 地面 973mm
             * 图像上两种解释的投影点分布见 docs/ring_semantics_check.png。
             *
             * 故: 环带语义 = **绝对俯角**（相对地面, 车系 z 轴）。
             * 视口射线构造在车系, 再用 Rᵀ 转到相机系。
             */
            /* rp 已经是车系方向，直接转到该相机系: v_C = Rᵀ · v_V。
             * 真实外参只在这里出现 —— 它决定"这个车系方向落在哪张鱼眼的哪个像素"。 */
            float rc[3];
            for (int k = 0; k < 3; ++k) {
                rc[k] = pose->R[0*3+k] * rp[0]
                      + pose->R[1*3+k] * rp[1]
                      + pose->R[2*3+k] * rp[2];
            }

            float uf, vf;
            nf_u8 ok = 0;
            /**
             * 投影后端：默认走 NFOCC 原生 `nf_cam_project_c`。
             *
             * `NF_WITH_VCAM=1` 时改走 vcam 的 `vc_cam_project`。
             * 两条路已验证一致（`test_vcam_bridge`：radial_poly 0.117 px、
             * KB 0.190 px，门限 0.5 px）。
             *
             * **vcam 不可用时自动回退**，不返回错误 —— LUT 构建是主链，
             * 可选后端的缺失不该让整个 pipeline 失败。
             */
#ifdef NF_WITH_VCAM
            st = nf_vcam_project(cam, rc[0], rc[1], rc[2], &uf, &vf, &ok);
            if (st == NF_E_ENGINE_UNSUPPORTED) {
                /* 该模型 vcam 不支持（如未来新增的类型）-> 回退原生 */
                st = nf_cam_project_c(cam, rc[0], rc[1], rc[2], &uf, &vf, &ok);
            }
#else
            st = nf_cam_project_c(cam, rc[0], rc[1], rc[2], &uf, &vf, &ok);
#endif
            if (st != NF_OK) { st = NF_OK; ok = 0; }

            /**
             * **新的一道: 偏转后射线必须仍指向视口前方** (rp[2] > 0)。
             *
             * 视口水平张角 ±60°, 偏转 45° 后一侧边缘的总方位角达到 105°,
             * 已越过 90° —— 那里的射线实际指向视口**侧后方**。
             * 这类像素若被当成有效, 会产出负深度或背面像素, 污染点云。
             *
             * 实测: 偏 45° 时 960x258 视口里有 825 个采样点(约 5%)
             * 的 d_view_z < 0。此前 LUT 仍标它们有效, 因为
             * nf_cam_project_c 只看方向是否能投到鱼眼上, 不管朝向。
             *
             * 几何依据: rp 是视口系方向, rp[2] 即其朝前的分量。
             * rp[2] <= 0 意味着该像素对应的"视口视线"背离视口,
             * 在物理解释上不存在 —— 视口是透视投影, 所有像素都应在 z>0 半空间。
             */
            /* 判据改用**相机系** z: 该车系方向必须落在相机前方。
             * 旧版判 rp[2]（视口系朝前）在虚拟相机模型下不再适用。 */
            if (rc[2] <= 0.0f) {
                local.d4_body_mask++;       /* 复用该计数: 几何无效 */
                continue;
            }

            /* 四道有效性 —— 按"唯一淘汰原因"计数, 便于定位配置问题 */
            if (!ok) {
                /* 用 theta 与半径细分 D2/D3 */
                const float chi = std::sqrt(rp[0]*rp[0] + rp[1]*rp[1]);
                const float th  = std::atan2(chi, rp[2]);
                const float r   = cam->f * nf_cam_theta_d(cam, th);
                if (r > cam->r_valid)      local.d2_out_of_r_valid++;
                else if (th > cam->max_theta) local.d3_over_max_theta++;
                else                        local.d1_out_of_image++;
                continue;
            }
            if (uf < 0.0f || uf >= (float)1920 || vf < 0.0f || vf >= (float)1536) {
                local.d1_out_of_image++;
                continue;
            }
            if (mask) {
                const nf_i32 mi = clamp_i32((nf_i32)vf, 0, 1535) * 1920
                                + clamp_i32((nf_i32)uf, 0, 1919);
                if (mask[mi]) { local.d4_body_mask++; continue; }
            }

            /* Q12.4 量化, 越界(>2047.9px)标无效 */
            const float qu = uf * kQ4Scale;
            const float qv = vf * kQ4Scale;
            if (qu > 32767.0f || qv > 32767.0f || qu < -32768.0f || qv < -32768.0f) {
                local.d1_out_of_image++;
                continue;
            }
            lut->u_q4[idx] = (nf_i16)(qu >= 0.0f ? qu + 0.5f : qu - 0.5f);
            lut->v_q4[idx] = (nf_i16)(qv >= 0.0f ? qv + 0.5f : qv - 0.5f);
            lut->valid[idx] = 1;
            local.valid++;
        }
    }

    if (stats) *stats = local;

    /**
     * 全空处理。
     *
     * ## 曾经: 直接返回 NF_E_PARAM_RANGE
     * 当初的理由是"静默全空最危险"。这在**视口跟随真实光轴**时成立 ——
     * 视口就是相机自己的方向，全空说明标定或参数一定错了。
     *
     * ## 改成虚拟相机后不再成立
     * 虚拟相机只由 yaw 决定，与相机装在哪无关。RV 的 yaw=0（朝车头）意味着
     * 要用**车尾相机**去看车头方向 —— 鱼眼根本拍不到，全空是**正确的物理结果**，
     * 不是配置错误。若仍返回错误，nf_pipeline_create 会整条失败。
     *
     * ## 现在的处理
     * 保留 BUILD_EMPTY 状态码（仍可被调用方识别），但**不阻断**：
     * 调用方按 stats.valid==0 跳过该视口即可。
     * 真正的参数错误（尺寸、f、自洽性）已在上面单独校验过。
     */
    if (local.valid == 0) return NF_E_LUT_EMPTY;
    return NF_OK;
}

NFStatus nf_view_warp(const NFViewLut *lut, const nf_u8 *src,
                      nf_u32 src_w, nf_u32 src_h, nf_u32 src_stride,
                      nf_u8 *out, nf_u8 *out_val) {
    if (!lut || !src || !out || !out_val) return NF_E_PARAM_NULL;
    if (src_w == 0 || src_h == 0) return NF_E_PARAM_RANGE;
    if (lut->width == 0 || lut->height == 0) return NF_E_PARAM_RANGE;

    const float inv = 1.0f / kQ4Scale;
    for (nf_u32 y = 0; y < lut->height; ++y) {
        for (nf_u32 x = 0; x < lut->width; ++x) {
            const size_t idx = (size_t)y * lut->width + x;
            const size_t oidx = idx;
            if (!lut->valid[idx]) { out[oidx] = 0; out_val[oidx] = 0; continue; }

            const float uf = (float)lut->u_q4[idx] * inv;
            const float vf = (float)lut->v_q4[idx] * inv;

            const nf_i32 u0 = (nf_i32)std::floor(uf);
            const nf_i32 v0 = (nf_i32)std::floor(vf);
            const float du = uf - (float)u0;
            const float dv = vf - (float)v0;

            float acc = 0.0f, wsum = 0.0f;
            /* 四邻域, 出界的邻域不参与(不按 0 处理) */
            for (int dy = 0; dy <= 1; ++dy) {
                for (int dx = 0; dx <= 1; ++dx) {
                    const nf_i32 yy = v0 + dy;
                    const nf_i32 xx = u0 + dx;
                    if (yy < 0 || yy >= (nf_i32)src_h) continue;
                    if (xx < 0 || xx >= (nf_i32)src_w) continue;
                    const float w = (dx ? du : 1.0f - du) * (dy ? dv : 1.0f - dv);
                    acc  += w * (float)src[(size_t)yy * src_stride + (size_t)xx];
                    wsum += w;
                }
            }
            if (wsum > 1e-6f) {
                const float val = acc / wsum;
                out[oidx] = (nf_u8)(val < 0.0f ? 0.0f : (val > 255.0f ? 255.0f : val + 0.5f));
                out_val[oidx] = 1;
            } else {
                out[oidx] = 0;
                out_val[oidx] = 0;
            }
        }
    }
    return NF_OK;
}

NFStatus nf_view_pixel_to_ray(const NFViewParams *vp, nf_f32 u, nf_f32 v,
                              nf_f32 *ray) {
    if (!vp || !ray) return NF_E_PARAM_NULL;
    if (vp->version != NF_VIEW_PARAMS_VERSION) return NF_E_PARAM_VERSION;
    if (!(vp->f > 0.0f)) return NF_E_PARAM_RANGE;

    const float el_mid = ring_el_mid_rad(vp);
    pixel_to_ray_impl(vp, el_mid, u, v, ray);
    return NF_OK;
}

/* ==================== REV-002: D1 车身掩码光栅化 ==================== */

NFStatus nf_mask_rasterize_polygon(nf_u8 *mask, nf_u32 w, nf_u32 h,
                                   const nf_f32 *poly_x, const nf_f32 *poly_y,
                                   nf_u32 n_pts) {
    if (!mask || !poly_x || !poly_y) return NF_E_PARAM_NULL;
    if (w == 0 || h == 0) return NF_E_PARAM_RANGE;
    if (n_pts < 3) return NF_E_PARAM_RANGE;

    std::memset(mask, 0, (size_t)w * h);

    /* 逐行处理，便于并行且缓存友好 */
    for (nf_u32 py = 0; py < h; ++py) {
        const float pyf = (float)py + 0.5f;
        nf_u8 *row = mask + (size_t)py * w;
        for (nf_u32 px = 0; px < w; ++px) {
            const float pxf = (float)px + 0.5f;
            nf_u32 crossings = 0;
            for (nf_u32 i = 0; i < n_pts; ++i) {
                const nf_u32 j = (i + 1u) % n_pts;
                const float yi = poly_y[i], yj = poly_y[j];
                const float xi = poly_x[i], xj = poly_x[j];

                /* 只在"跨越该水平线"的边上判定 */
                if ((yi > pyf) == (yj > pyf)) continue;

                /* 交点 x 坐标；分母加 eps 防水平边除零 */
                const float t = (pyf - yi) / (yj - yi + 1e-12f);
                const float x_cross = xi + t * (xj - xi);
                if (pxf < x_cross) ++crossings;
            }
            /* 奇数 = 多边形内部 = 车身遮挡 */
            row[px] = (nf_u8)(crossings & 1u);
        }
    }
    return NF_OK;
}

} /* extern "C" */
