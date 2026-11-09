/**
 * @file match_radial.cpp
 * @brief 径向膨胀匹配 + 亚像素细化。
 */
#include "nfocc/nfocc_match.h"

#include <cmath>
#include <cstring>

namespace {

/** 双线性采样，越界返回 −1（用于判定无效） */
inline float SampleBilinear(const nf_u8 *img, nf_u32 w, nf_u32 h,
                            float x, float y) {
    if (x < 0.0f || y < 0.0f) return -1.0f;
    const int x0 = (int)x, y0 = (int)y;
    if (x0 < 0 || y0 < 0 || x0 + 1 >= (int)w || y0 + 1 >= (int)h) return -1.0f;
    const float fx = x - (float)x0, fy = y - (float)y0;
    const nf_u8 *r0 = img + (size_t)y0 * w + x0;
    const nf_u8 *r1 = r0 + w;
    const float a = (float)r0[0] * (1.0f - fx) + (float)r0[1] * fx;
    const float b = (float)r1[0] * (1.0f - fx) + (float)r1[1] * fx;
    return a * (1.0f - fy) + b * fy;
}

/**
 * 归一化互相关（NCC）。
 * 用 NCC 而不是 SAD 的原因：前向膨胀会改变尺度，亮度均值也随面积变化，
 * SAD 对这两者都敏感；NCC 对线性亮度变化不变。
 */
float Ncc(const nf_u8 *a, nf_u32 aw, nf_u32 ah,
          float ax, float ay,
          const nf_u8 *b, nf_u32 bw, nf_u32 bh,
          float bx, float by, nf_u32 rad) {
    const int n = (int)(2 * rad + 1);
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    int cnt = 0;
    for (int dy = -(int)rad; dy <= (int)rad; ++dy) {
        for (int dx = -(int)rad; dx <= (int)rad; ++dx) {
            const float va = SampleBilinear(a, aw, ah, ax + dx, ay + dy);
            if (va < 0.0f) continue;
            const float vb = SampleBilinear(b, bw, bh, bx + dx, by + dy);
            if (vb < 0.0f) continue;
            sa += va; sb += vb;
            saa += va * va; sbb += vb * vb; sab += va * vb;
            ++cnt;
        }
    }
    if (cnt < (int)(0.6 * n * n)) return -1.0f;     // 有效样本太少
    const double dn = (double)cnt;
    const double va = saa - sa * sa / dn;
    const double vb = sbb - sb * sb / dn;
    if (va <= 1e-9 || vb <= 1e-9) return -1.0f;     // 无纹理
    const double cov = sab - sa * sb / dn;
    return (float)(cov / std::sqrt(va * vb));
}

}  // namespace

extern "C" {

NFStatus nf_match_radial(const nf_u8 *cur, const nf_u8 *ref,
                         const nf_u8 *valid_cur,
                         nf_u32 w, nf_u32 h, NFFoe foe,
                         nf_f32 r_min, nf_f32 r_max,
                         nf_u32 patch, nf_f32 ncc_min,
                         nf_f32 *out_cur, nf_f32 *out_conf, nf_u8 *out_valid) {
    if (!cur || !ref || !out_cur || !out_conf || !out_valid) return NF_E_PARAM_NULL;
    if (w == 0 || h == 0) return NF_E_PARAM_RANGE;
    if (r_max <= r_min) return NF_E_PARAM_RANGE;
    if (patch < 1 || patch > 4) return NF_E_PARAM_RANGE;
    if (ncc_min < -1.0f || ncc_min > 1.0f) return NF_E_PARAM_RANGE;

    std::memset(out_valid, 0, (size_t)w * h);

    for (nf_u32 y = 0; y < h; ++y) {
        for (nf_u32 x = 0; x < w; ++x) {
            const nf_u32 idx = y * w + x;
            if (valid_cur && !valid_cur[idx]) continue;

            /* **边界检查: 整块 patch 必须在图内**。
             * 不做这一步时, 靠近图像边缘的点因为 NCC 采样不足而给出
             * 虚假的高相关 —— 实测半径 58~64 处误差 1.9 px、最大 5.9 px,
             * 而内部区域误差只有 0.03~0.05 px。这是纯边界效应, 应当直接拒绝,
             * 而不是让它污染深度图。 */
            const int pr = (int)patch;
            if ((int)x < pr || (int)y < pr
                || (int)x + pr + 1 >= (int)w || (int)y + pr + 1 >= (int)h)
                continue;

            /* 相对 FoE 的径向方向（前向膨胀沿此方向移动） */
            const float dx = (float)x - foe.u;
            const float dy = (float)y - foe.v;
            const float rho = std::sqrt(dx * dx + dy * dy);
            /* 太靠近 FoE: 方向不稳定，径向位移也趋近 0，无法定深度 */
            if (rho < 4.0f) continue;
            const float ux = dx / rho, uy = dy / rho;

            /* 沿径向搜索最佳位移 */
            float best_r = 0.0f, best_ncc = -2.0f;
            /* 半像素步长：粗搜精度 0.5 px，细化后可达 ~0.05 px。 */
            const float step = 0.5f;
            /* 次峰统计: 用于"峰值显著性"判据(见下) */
            float second_ncc = -2.0f;
            const float min_sep = 2.0f * (float)patch + 1.0f;

            for (float r = r_min; r <= r_max; r += step) {
                const float ncc = Ncc(cur, w, h, (float)x, (float)y,
                                      ref, w, h,
                                      (float)x + ux * r, (float)y + uy * r, patch);
                if (ncc > best_ncc) { best_ncc = ncc; best_r = r; }
            }
            /* 第二遍: 统计 (a) 离主峰足够远的次峰, (b) 整个搜索范围的均值/标准差。
             *
             * 只用"主峰 - 次峰"不够: 两张**完全无关**的高频纹理图, 在 24 个候选
             * 位移里总能碰上一个局部 NCC 0.7+ 的位置(实测无关图仍有 43% 被
             * 判为有效)。加"主峰必须显著高于搜索范围的统计分布"才拦得住:
             * 真实匹配的 NCC 曲线是单峰尖峰, 无关图是噪声。
             */
            double sum_ncc = 0.0, sum_sq = 0.0;
            int n_cand = 0;
            for (float r = r_min; r <= r_max; r += step) {
                const float ncc = Ncc(cur, w, h, (float)x, (float)y,
                                      ref, w, h,
                                      (float)x + ux * r, (float)y + uy * r, patch);
                if (ncc < -1.5f) continue;      /* 采样不足的候选不计入统计 */
                sum_ncc += ncc;
                sum_sq += (double)ncc * ncc;
                ++n_cand;
                if (std::fabs(r - best_r) < min_sep) continue;
                if (ncc > second_ncc) second_ncc = ncc;
            }

            if (best_ncc < ncc_min) continue;
            /* 峰值显著性: 主峰须明显高于次峰(重复纹理判据) */
            if (second_ncc > -2.0f && best_ncc - second_ncc < 0.15f) continue;
            /* 峰值突出度: 主峰须高于全范围均值若干标准差。
             *
             * **阈值要随 best_ncc 自适应**。固定 2σ 会误杀真实匹配:
             * 前向膨胀时像块被放大, NCC 曲线在主峰附近是**宽峰**而非尖峰,
             * 多候选都接近主峰 -> 标准差大 -> 2σ 判据拒掉(实测只剩 92 点)。
             * 而无关图的 best_ncc 本身就不高。
             *
             * 折中: best_ncc 越高, 允许的标准差倍数越小(因为它已经足够可信);
             * best_ncc 中等时要求严格(防止噪声凑出来的 0.6)。
             *   0.6 -> 2.0σ   (严格, 主要拦无关图)
             *   0.9 -> 0.8σ   (宽松, 已是很强的匹配)
             */
            if (n_cand >= 8) {
                const double mean = sum_ncc / n_cand;
                const double var = sum_sq / n_cand - mean * mean;
                const double sd = var > 0.0 ? std::sqrt(var) : 0.0;
                double k = 2.0 - 4.0 * ((double)best_ncc - 0.6);   // 0.6->2.0, 0.9->0.8
                if (k < 1.2) k = 1.2;   // 下界 1.2σ: 实测 0.8σ 会让无关噪声图 40% 通过
                if (k > 2.0) k = 2.0;
                if ((double)best_ncc - mean < k * sd) continue;
            }

            /* 亚像素细化：三点抛物线顶点
             *   delta = 0.5*(n_prev - n_next) / (n_prev - 2*n_0 + n_next)
             * 仅当峰值在中点、且分母非退化时才可信。 */
            float r_fine = best_r;
            if (best_r > r_min + step && best_r < r_max - step) {
                const float n0 = Ncc(cur, w, h, (float)x, (float)y, ref, w, h,
                                     (float)x + ux * best_r,
                                     (float)y + uy * best_r, patch);
                const float np = Ncc(cur, w, h, (float)x, (float)y, ref, w, h,
                                     (float)x + ux * (best_r - step),
                                     (float)y + uy * (best_r - step), patch);
                const float nn = Ncc(cur, w, h, (float)x, (float)y, ref, w, h,
                                     (float)x + ux * (best_r + step),
                                     (float)y + uy * (best_r + step), patch);
                const float den = np - 2.0f * n0 + nn;
                if (std::fabs(den) > 1e-6f) {
                    float delta = 0.5f * (np - nn) / den;
                    if (delta > 1.0f) delta = 1.0f;
                    if (delta < -1.0f) delta = -1.0f;
                    r_fine = best_r + delta * step;
                }
            }

            /* 终点同样要留出 patch 余量, 否则细化时采到边界外 */
            {
                const float qx = (float)x + ux * r_fine;
                const float qy = (float)y + uy * r_fine;
                if (qx < (float)pr || qy < (float)pr
                    || qx + (float)pr >= (float)(w - 1)
                    || qy + (float)pr >= (float)(h - 1))
                    continue;
            }

            out_cur[idx * 2 + 0] = (float)x + ux * r_fine;
            out_cur[idx * 2 + 1] = (float)y + uy * r_fine;
            out_conf[idx] = best_ncc;
            out_valid[idx] = 1;
        }
    }
    return NF_OK;
}

NFStatus nf_depth_from_radial(const nf_f32 *pts, const nf_u8 *valid,
                              nf_u32 w, nf_u32 h, NFFoe foe,
                              nf_f32 t_forward_mm,
                              nf_f32 *out_depth, nf_f32 *out_sigma) {
    if (!pts || !valid || !out_depth) return NF_E_PARAM_NULL;
    if (w == 0 || h == 0) return NF_E_PARAM_RANGE;
    if (t_forward_mm <= 0.0f) return NF_E_PARAM_RANGE;

    for (nf_u32 y = 0; y < h; ++y) {
        for (nf_u32 x = 0; x < w; ++x) {
            const nf_u32 idx = y * w + x;
            out_depth[idx] = 0.0f;
            if (out_sigma) out_sigma[idx] = 0.0f;
            if (!valid[idx]) continue;

            /* ρ0 = 当前帧半径；ρ1 = 对应帧半径。s = ρ1/ρ0 > 1 */
            const float dx0 = (float)x - foe.u, dy0 = (float)y - foe.v;
            const float rho0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
            const float dx1 = pts[idx * 2 + 0] - foe.u;
            const float dy1 = pts[idx * 2 + 1] - foe.v;
            const float rho1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            if (rho0 < 1e-3f) continue;

            const float s = rho1 / rho0;
            /* 前向平移 T 后，物点深度从 Z 变成 Z − T，像点半径按 Z/(Z−T) 放大。
               由 s = Z/(Z−T) 得 Z = T·s/(s−1)。s<=1 说明方向反了或退化。 */
            if (s <= 1.0f + 1e-4f) continue;
            const float Z = t_forward_mm * s / (s - 1.0f);
            out_depth[idx] = Z;
            if (out_sigma) {
                /* σ_Z ≈ T·σ_r/(s−1)², 单位像素误差 σ_r≈0.5 px */
                const float dZds = -t_forward_mm / ((s - 1.0f) * (s - 1.0f));
                const float ds = 0.5f / rho0;
                out_sigma[idx] = std::fabs(dZds * ds);
            }
        }
    }
    return NF_OK;
}

}  /* extern "C" */

/* ==================== P1: 快速径向匹配 ==================== */
/**
 * 与 nf_match_radial 同一判据，但按 sample_step 采样、coarse_step 粗搜、
 * 单遍统计、OpenMP 按行并行。块内邻居复制采样点的对应点；深度解算仍按
 * 每个全分辨率像素自己的射线做，所以不会把邻居的像素坐标当成自己的。
 */
NFStatus nf_match_radial_fast(const nf_u8 *cur, const nf_u8 *ref,
                              const nf_u8 *valid_cur,
                              nf_u32 w, nf_u32 h, NFFoe foe,
                              nf_f32 r_min, nf_f32 r_max,
                              nf_u32 patch, nf_f32 ncc_min,
                              nf_u32 sample_step, nf_f32 coarse_step,
                              nf_f32 *out_cur, nf_f32 *out_conf,
                              nf_u8 *out_valid) {
    if (!cur || !ref || !out_cur || !out_conf || !out_valid) return NF_E_PARAM_NULL;
    if (w == 0 || h == 0) return NF_E_PARAM_RANGE;
    if (r_max <= r_min) return NF_E_PARAM_RANGE;
    if (patch < 1 || patch > 4) return NF_E_PARAM_RANGE;
    if (ncc_min < -1.0f || ncc_min > 1.0f) return NF_E_PARAM_RANGE;

    if (sample_step < 1) sample_step = 1;
    if (coarse_step < 0.5f) coarse_step = 0.5f;
    const int step = (int)sample_step;
    const int pr = (int)patch;
    const float min_sep = 2.0f * (float)patch + 1.0f;

    std::memset(out_valid, 0, (size_t)w * h);
    std::memset(out_cur, 0, (size_t)w * h * 2 * sizeof(nf_f32));
    std::memset(out_conf, 0, (size_t)w * h * sizeof(nf_f32));

#ifdef NF_HAS_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int yy = 0; yy < (int)h; yy += step) {
        const nf_u32 y = (nf_u32)yy;
        for (int xx = 0; xx < (int)w; xx += step) {
            const nf_u32 x = (nf_u32)xx;
            const nf_u32 idx = y * w + x;
            if (valid_cur && !valid_cur[idx]) continue;

            if ((int)x < pr || (int)y < pr
                || (int)x + pr + 1 >= (int)w || (int)y + pr + 1 >= (int)h)
                continue;

            const float dx = (float)x - foe.u;
            const float dy = (float)y - foe.v;
            const float rho = std::sqrt(dx * dx + dy * dy);
            if (rho < 4.0f) continue;
            const float ux = dx / rho, uy = dy / rho;

            /* ---- 粗搜 + 单遍统计 ---- */
            float best_r = 0.0f, best_ncc = -2.0f, second_ncc = -2.0f;
            double sum_ncc = 0.0, sum_sq = 0.0;
            int n_cand = 0;
            for (float r = r_min; r <= r_max; r += coarse_step) {
                const float ncc = Ncc(cur, w, h, (float)x, (float)y,
                                      ref, w, h,
                                      (float)x + ux * r, (float)y + uy * r, patch);
                if (ncc >= -1.5f) {
                    sum_ncc += ncc;
                    sum_sq += (double)ncc * ncc;
                    ++n_cand;
                }
                if (ncc > best_ncc) {
                    if (std::fabs(r - best_r) >= min_sep) second_ncc = best_ncc;
                    best_ncc = ncc; best_r = r;
                } else if (std::fabs(r - best_r) >= min_sep && ncc > second_ncc) {
                    second_ncc = ncc;
                }
            }

            /* ---- 主峰附近 0.5 px 细化 ---- */
            {
                const float lo = std::fmax(r_min, best_r - 2.0f * coarse_step);
                const float hi = std::fmin(r_max, best_r + 2.0f * coarse_step);
                for (float r = lo; r <= hi; r += 0.5f) {
                    const float ncc = Ncc(cur, w, h, (float)x, (float)y,
                                          ref, w, h,
                                          (float)x + ux * r, (float)y + uy * r, patch);
                    if (ncc > best_ncc) { best_ncc = ncc; best_r = r; }
                }
            }

            if (best_ncc < ncc_min) continue;
            if (second_ncc > -2.0f && best_ncc - second_ncc < 0.15f) continue;
            if (n_cand >= 8) {
                const double mean = sum_ncc / n_cand;
                const double var = sum_sq / n_cand - mean * mean;
                const double sd = var > 0.0 ? std::sqrt(var) : 0.0;
                double k = 2.0 - 4.0 * ((double)best_ncc - 0.6);
                if (k < 1.2) k = 1.2;
                if (k > 2.0) k = 2.0;
                if ((double)best_ncc - mean < k * sd) continue;
            }

            /* 三点抛物线亚像素 */
            float r_fine = best_r;
            if (best_r > r_min + 0.5f && best_r < r_max - 0.5f) {
                const float n0 = Ncc(cur, w, h, (float)x, (float)y, ref, w, h,
                                     (float)x + ux * best_r,
                                     (float)y + uy * best_r, patch);
                const float np = Ncc(cur, w, h, (float)x, (float)y, ref, w, h,
                                     (float)x + ux * (best_r - 0.5f),
                                     (float)y + uy * (best_r - 0.5f), patch);
                const float nn = Ncc(cur, w, h, (float)x, (float)y, ref, w, h,
                                     (float)x + ux * (best_r + 0.5f),
                                     (float)y + uy * (best_r + 0.5f), patch);
                const float den = np - 2.0f * n0 + nn;
                if (std::fabs(den) > 1e-6f) {
                    float delta = 0.5f * (np - nn) / den;
                    if (delta > 1.0f) delta = 1.0f;
                    if (delta < -1.0f) delta = -1.0f;
                    r_fine = best_r + delta * 0.5f;
                }
            }

            const float qx = (float)x + ux * r_fine;
            const float qy = (float)y + uy * r_fine;
            if (qx < (float)pr || qy < (float)pr
                || qx + (float)pr >= (float)(w - 1)
                || qy + (float)pr >= (float)(h - 1))
                continue;

            out_cur[idx * 2 + 0] = qx;
            out_cur[idx * 2 + 1] = qy;
            out_conf[idx] = best_ncc;
            out_valid[idx] = 1;
        }
    }

    /* ---- 采样点之间的径向位移做双线性插值 ----
     *
     * 最初这里直接把采样点的 q 复制到 2x2 块，结果 2-3 m 表面中位
     * 从 222 涨到 261 mm：四个像素共用同一个对应点，块边缘的深度
     * 被拉偏。改成插值"径向位移 r"（而不是 q）—— 每个全分辨率像素
     * 仍沿自己的射线 u 算 q = p + u·r，深度用各自射线解算。
     */
    if (step > 1) {
        for (nf_u32 y = 0; y < h; ++y) {
            for (nf_u32 x = 0; x < w; ++x) {
                if ((x % (nf_u32)step) == 0u && (y % (nf_u32)step) == 0u)
                    continue;
                const nf_u32 x0 = (x / (nf_u32)step) * (nf_u32)step;
                const nf_u32 y0 = (y / (nf_u32)step) * (nf_u32)step;
                const nf_u32 x1 = x0 + (nf_u32)step;
                const nf_u32 y1 = y0 + (nf_u32)step;
                if (x1 >= w || y1 >= h) continue;
                const nf_u32 i00 = y0 * w + x0, i10 = y0 * w + x1;
                const nf_u32 i01 = y1 * w + x0, i11 = y1 * w + x1;
                if (!out_valid[i00] || !out_valid[i10] ||
                    !out_valid[i01] || !out_valid[i11]) continue;

                /* 采样点的径向位移 r = |q − a| − |p − a| */
                auto rad_at = [&](nf_u32 px, nf_u32 py, nf_u32 idx) {
                    const float qx = out_cur[idx * 2 + 0];
                    const float qy = out_cur[idx * 2 + 1];
                    const float dqx = qx - foe.u, dqy = qy - foe.v;
                    const float dpx = (float)px - foe.u, dpy = (float)py - foe.v;
                    return std::sqrt(dqx * dqx + dqy * dqy)
                         - std::sqrt(dpx * dpx + dpy * dpy);
                };
                const float r00 = rad_at(x0, y0, i00);
                const float r10 = rad_at(x1, y0, i10);
                const float r01 = rad_at(x0, y1, i01);
                const float r11 = rad_at(x1, y1, i11);
                const float fx = (float)(x - x0) / (float)step;
                const float fy = (float)(y - y0) / (float)step;
                const float r0 = r00 * (1.0f - fx) + r10 * fx;
                const float r1 = r01 * (1.0f - fx) + r11 * fx;
                const float r  = r0 * (1.0f - fy) + r1 * fy;

                const float dx = (float)x - foe.u, dy = (float)y - foe.v;
                const float rho = std::sqrt(dx * dx + dy * dy);
                if (rho < 1e-3f) continue;
                const float qx = (float)x + dx / rho * r;
                const float qy = (float)y + dy / rho * r;
                if (qx < (float)pr || qy < (float)pr
                    || qx + (float)pr >= (float)(w - 1)
                    || qy + (float)pr >= (float)(h - 1))
                    continue;

                const nf_u32 didx = y * w + x;
                out_cur[didx * 2 + 0] = qx;
                out_cur[didx * 2 + 1] = qy;
                const float c0 = out_conf[i00] * (1.0f - fx) + out_conf[i10] * fx;
                const float c1 = out_conf[i01] * (1.0f - fx) + out_conf[i11] * fx;
                out_conf[didx] = c0 * (1.0f - fy) + c1 * fy;
                out_valid[didx] = 1;
            }
        }
    }
    return NF_OK;
}
