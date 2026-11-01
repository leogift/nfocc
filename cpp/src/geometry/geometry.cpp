/**
 * @file geometry.cpp
 * @brief 三角化 / 门控 / σ 传播 / 前向解析测距。
 *        对应 Python 原型 nfocc_ph1/triangulate.py、gating.py、forward_range.py。
 */
#include "nfocc/nfocc_geometry.h"
#include "nfocc/nfocc_default.h"

#include <cmath>

namespace {

const float kPi = 3.14159265358979323846f;

inline void cross3(const float *a, const float *b, float *o) {
    o[0] = a[1]*b[2] - a[2]*b[1];
    o[1] = a[2]*b[0] - a[0]*b[2];
    o[2] = a[0]*b[1] - a[1]*b[0];
}
inline float dot3(const float *a, const float *b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
inline float norm3(const float *a) { return std::sqrt(dot3(a, a)); }

} // namespace

extern "C" {

nf_u8 nf_mode_produces_depth(NFMode m) {
    return (m == NF_MODE_STEREO || m == NF_MODE_FORWARD || m == NF_MODE_HYBRID) ? 1u : 0u;
}

const char *nf_mode_str(NFMode m) {
    switch (m) {
    case NF_MODE_STEREO:      return "STEREO";
    case NF_MODE_FORWARD:     return "FORWARD";
    case NF_MODE_HYBRID:      return "HYBRID";
    case NF_MODE_FROZEN_NEAR: return "FROZEN_NEAR";
    case NF_MODE_FROZEN_FAR:  return "FROZEN_FAR";
    case NF_MODE_FROZEN_GAP:  return "FROZEN_GAP";
    }
    return "UNKNOWN";
}

nf_f32 nf_epi_angle_deg(const nf_f32 *t_cam) {
    if (!t_cam) return NAN;
    /* 像移竖直分量来自 t_y, 但"偏离横轴多少"由 (t_y, t_z) 共同决定:
       t 沿 z(光轴) -> 视差趋零 -> FoE 退化, 同样判不可用 */
    const float perp = std::sqrt(t_cam[1]*t_cam[1] + t_cam[2]*t_cam[2]);
    const float tx   = std::fabs(t_cam[0]);
    return std::atan2(perp, tx) * 180.0f / kPi;
}

NFStatus nf_gate_decide(const nf_f32 *t_cam, nf_u32 frame_gap,
                        nf_f32 b_min_mm, nf_f32 b_max_mm,
                        nf_f32 sigma_stereo_min, nf_f32 sigma_forward_max,
                        nf_u32 gap_max_frames,
                        NFGateDecision *out) {
    if (!t_cam || !out) return NF_E_PARAM_NULL;

    const float nt = norm3(t_cam);
    out->baseline_mm = nt;
    out->sigma = (nt > 1e-9f)
               ? std::sqrt(t_cam[0]*t_cam[0] + t_cam[1]*t_cam[1]) / nt
               : 0.0f;
    out->epi_angle_deg = nf_epi_angle_deg(t_cam);

    /* 顺序不可换: 丢帧 -> 基线不足 -> 超限 -> 方向分流 */
    if (frame_gap > gap_max_frames) { out->mode = NF_MODE_FROZEN_GAP;  return NF_OK; }
    if (nt < b_min_mm)              { out->mode = NF_MODE_FROZEN_NEAR; return NF_OK; }
    if (nt > b_max_mm)              { out->mode = NF_MODE_FROZEN_FAR;  return NF_OK; }

    if (out->sigma >= sigma_stereo_min)       out->mode = NF_MODE_STEREO;
    else if (out->sigma <= sigma_forward_max) out->mode = NF_MODE_FORWARD;
    else                                      out->mode = NF_MODE_HYBRID;
    return NF_OK;
}

NFStatus nf_triangulate_rays(const nf_f32 *c0, const nf_f32 *r0,
                             const nf_f32 *c1, const nf_f32 *r1,
                             nf_u32 n, nf_f32 *s0, nf_u8 *ok) {
    if (!c0 || !r0 || !c1 || !r1 || !s0 || !ok) return NF_E_PARAM_NULL;

    /* **用叉积闭式解, 不用 2×2 正规方程。**
     *
     * 正规方程解 [a,−b][s0,s1]ᵀ = t 时, det = a11·a22 − a12² 是"两个约 1 的数
     * 相减"。本项目 120mm 基线 / 5000mm 距离下两视线夹角仅 0.33°,
     * det ≈ 3e-5, float32 的 6e-8 相对误差被放大到 ~1e-2。
     * 实测: float32 输入下, 正规方程误差 3.87e-03。
     *
     * 叉积闭式解避免了这个相减:
     *     s0 = |t × b|² / ((a × b)·(t × b))
     * 同一输入实测误差 5.04e-06 —— **好 768 倍**, 且无需提升到 double。
     *
     * 推导: 由 P = c0 + s0·a = c1 + s1·b 得 (c1−c0) = s0·a − s1·b。
     * 两边叉乘 b 消去 s1: t × b = s0·(a × b)。再与 (a×b) 点乘取最小二乘:
     *     (t×b)·(a×b) = s0·|a×b|²   但这样依赖 |a×b|² 的绝对值;
     * 更稳的形式是再用 t×b 投影:  s0 = |t×b|² / ((a×b)·(t×b))。
     */
    const float t[3] = { c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2] };

    for (nf_u32 i = 0; i < n; ++i) {
        const float *a = r0 + (size_t)i * 3;
        const float *b = r1 + (size_t)i * 3;

        float ab[3], tb[3];
        cross3(a, b, ab);        /* a × b */
        cross3(t, b, tb);        /* t × b */

        const float den = dot3(ab, tb);
        /* den = (a×b)·(t×b) = |t|·|a×b|·cos(...) , 视线接近平行时 -> 0 */
        if (std::fabs(den) < 1e-12f) { s0[i] = NAN; ok[i] = 0; continue; }

        const float num = dot3(tb, tb);      /* |t×b|² */
        const float s = num / den;
        if (!std::isfinite(s) || s <= 0.0f) { s0[i] = NAN; ok[i] = 0; continue; }
        s0[i] = s;
        ok[i] = 1;
    }
    return NF_OK;
}

NFStatus nf_effective_baseline(const nf_f32 *r, nf_u32 n,
                               const nf_f32 *t,
                               nf_f32 *b_perp) {
    if (!r || !t || !b_perp) return NF_E_PARAM_NULL;
    for (nf_u32 i = 0; i < n; ++i) {
        const float *ri = r + (size_t)i * 3;
        /* perp = t − (r·t)·r ,  取模 */
        const float proj = dot3(ri, t);
        const float px = t[0] - proj * ri[0];
        const float py = t[1] - proj * ri[1];
        const float pz = t[2] - proj * ri[2];
        b_perp[i] = std::sqrt(px*px + py*py + pz*pz);
    }
    return NF_OK;
}

NFStatus nf_sigma_depth(const nf_f32 *z_mm, const nf_f32 *b_perp_mm,
                        nf_u32 n, nf_f32 f_px, nf_f32 sigma_d_px,
                        nf_f32 sigma_b_ratio, nf_f32 *sigma_out_mm) {
    if (!z_mm || !b_perp_mm || !sigma_out_mm) return NF_E_PARAM_NULL;
    if (!(f_px > 0.0f)) return NF_E_PARAM_RANGE;

    for (nf_u32 i = 0; i < n; ++i) {
        const float z  = z_mm[i];
        const float bp = (b_perp_mm[i] > 1e-9f) ? b_perp_mm[i] : 1e-9f;
        const float s_disp = (z * z) / (f_px * bp) * sigma_d_px;
        const float s_base = z * sigma_b_ratio;
        sigma_out_mm[i] = std::sqrt(s_disp * s_disp + s_base * s_base);
    }
    return NF_OK;
}

NFStatus nf_forward_range(const nf_f32 *u_hat, const nf_f32 *v_hat,
                          nf_u32 n, const nf_f32 *t,
                          nf_f32 *z_out, nf_u8 *ok) {
    if (!u_hat || !v_hat || !t || !z_out || !ok) return NF_E_PARAM_NULL;

    for (nf_u32 i = 0; i < n; ++i) {
        const float *u = u_hat + (size_t)i * 3;
        const float *v = v_hat + (size_t)i * 3;

        float uv[3], tv[3];
        cross3(u, v, uv);       /* û × v̂ */
        cross3(t, v, tv);       /* t × v̂ */

        const float num = dot3(uv, tv);
        const float den = dot3(uv, uv);
        /* den = sin²(两方向夹角), 过小表示共线, 无法定深度 */
        if (den < 1e-8f) { z_out[i] = NAN; ok[i] = 0; continue; }

        const float z = num / den;
        if (!std::isfinite(z) || z <= 0.0f) { z_out[i] = NAN; ok[i] = 0; continue; }
        z_out[i] = z;
        ok[i] = 1;
    }
    return NF_OK;
}

} /* extern "C" */
