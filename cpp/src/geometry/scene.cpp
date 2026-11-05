/**
 * @file scene.cpp
 * @brief 解析真值场景实现。
 */
#include "nfocc/nfocc_scene.h"

#include <cmath>
#include <cstring>

namespace {

inline float dot3(const float *a, const float *b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline bool inside_box(const NFPlane *p, const float *q) {
    if (!p->use_box) return true;
    for (int k = 0; k < 3; ++k) {
        if (q[k] < p->box_min[k] || q[k] > p->box_max[k]) return false;
    }
    return true;
}

}  // namespace

extern "C" {

NFStatus nf_scene_set_tex_for_focal(NFScene *s, nf_f32 focal_px,
                                    nf_f32 target_px, nf_f32 ref_z_mm) {
    if (!s) return NF_E_PARAM_NULL;
    if (!(focal_px > 0.0f) || !(target_px > 0.0f) || !(ref_z_mm > 0.0f))
        return NF_E_PARAM_RANGE;
    /* 像素周期 P_px = P_mm · f / Z  =>  P_mm = P_px · Z / f */
    const float p_mm = target_px * ref_z_mm / focal_px;
    s->tex_freq = 6.28318530718f / p_mm;       /* 2π / 周期 */
    return NF_OK;
}

NFStatus nf_scene_default(NFScene *s) {
    if (!s) return NF_E_PARAM_NULL;
    std::memset(s, 0, sizeof(*s));
    s->version = NF_SCENE_VERSION;
    /* 纹理基频: 每 mm 的相位 (rad/mm)。
     *
     * **这个值必须与 SGBM 的块大小匹配**, 否则匹配不可靠而深度系统性偏大。
     * 视口里纹理周期(px) = 周期(mm) * f / Z, f=277.13。
     *   0.004 rad/mm -> 周期 1571mm -> 2m 处 218px: 5x5 窗口看到近常数, 匹配失败
     *   0.042 rad/mm -> 周期  150mm -> 2m 处  21px, 1m 处 42px, 4m 处 10px
     * 实测: 用 0.004 时四路中位误差 352~645mm; 改用 0.042 后待验证。 */
    s->tex_freq = 0.042f;

    /* 平面 0: 地面 z = 0, 法向 (0,0,1)。
     * 用大范围框限制, 避免"无限地面"在远处产生不现实的深度。
     * 范围覆盖感知区 ±9m / ±7.5m。 */
    {
        NFPlane *p = &s->planes[0];
        p->normal[0] = 0.0f; p->normal[1] = 0.0f; p->normal[2] = 1.0f;
        p->point[0] = 0.0f; p->point[1] = 0.0f; p->point[2] = 0.0f;
        p->use_box = 1;
        p->box_min[0] = -9000.0f; p->box_max[0] = 9000.0f;
        p->box_min[1] = -7500.0f; p->box_max[1] = 7500.0f;
        p->box_min[2] = -10.0f;   p->box_max[2] = 10.0f;
        p->is_ground = 1;
    }

    /* 平面 1: 正前方墙 x = 4000, 法向 (-1,0,0), 高 0..2000, 宽 ±4000 */
    {
        NFPlane *p = &s->planes[1];
        p->normal[0] = -1.0f; p->normal[1] = 0.0f; p->normal[2] = 0.0f;
        p->point[0] = 4000.0f; p->point[1] = 0.0f; p->point[2] = 1000.0f;
        p->use_box = 1;
        p->box_min[0] = 3990.0f; p->box_max[0] = 4010.0f;
        p->box_min[1] = -4000.0f; p->box_max[1] = 4000.0f;
        p->box_min[2] = 0.0f;    p->box_max[2] = 2000.0f;
    }

    /* 平面 2: 左侧墙 y = 2500, 法向 (0,-1,0), 高 0..2000, 前后 ±4000 */
    {
        NFPlane *p = &s->planes[2];
        p->normal[0] = 0.0f; p->normal[1] = -1.0f; p->normal[2] = 0.0f;
        p->point[0] = 0.0f; p->point[1] = 2500.0f; p->point[2] = 1000.0f;
        p->use_box = 1;
        p->box_min[0] = -4000.0f; p->box_max[0] = 4000.0f;
        p->box_min[1] = 2490.0f;  p->box_max[1] = 2510.0f;
        p->box_min[2] = 0.0f;     p->box_max[2] = 2000.0f;
    }

    /*
     * ---- 精度验收专用的竖直结构（2026-11 重建）----
     *
     * ## 为什么要加
     * 原来只有 地面 + x=4000 前墙 + y=2500 左墙。探针实测发现：
     * 采样点**全部落在 776 mm 附近的地面**，2-3m/3-4m/4-5m 段样本为 0。
     * 地面是平坦大面，SGBM 的 5x5 窗口在里面找不到水平结构，匹配
     * 必然失败 —— 那不是算法问题，是场景测不出东西。
     *
     * 竖直平面提供**水平方向的灰度梯度**（沿 x 变化），正是 SGBM 需要的。
     * 按 1.2 / 2.2 / 3.4 / 4.6 m 四档布置，让 0.5-5 m 每一段都有样本。
     *
     * ## 为什么用细长平面而不是柱子
     * NFScene 只有平面图元。用"窄条平面"（宽 200mm）即可模拟立柱，
     * 且求交仍是解析的，不会引入数值噪声。
     */
    {
        /* 平面 3: 正前方 1.2m 处的窄障碍（模拟前车尾部） */
        NFPlane *p = &s->planes[3];
        p->normal[0] = -1.0f; p->normal[1] = 0.0f; p->normal[2] = 0.0f;
        p->point[0] = 1200.0f; p->point[1] = 0.0f; p->point[2] = 500.0f;
        p->use_box = 1;
        p->box_min[0] = 1195.0f; p->box_max[0] = 1205.0f;
        p->box_min[1] = -800.0f; p->box_max[1] = 800.0f;
        p->box_min[2] = 0.0f;    p->box_max[2] = 1000.0f;
    }
    {
        /* 平面 4: 前方 2.2m 处的立柱 */
        NFPlane *p = &s->planes[4];
        p->normal[0] = -1.0f; p->normal[1] = 0.0f; p->normal[2] = 0.0f;
        p->point[0] = 2200.0f; p->point[1] = 300.0f; p->point[2] = 700.0f;
        p->use_box = 1;
        p->box_min[0] = 2195.0f; p->box_max[0] = 2205.0f;
        p->box_min[1] = 200.0f;  p->box_max[1] = 400.0f;
        p->box_min[2] = 0.0f;    p->box_max[2] = 1400.0f;
    }
    {
        /* 平面 5: 前方 3.4m 处的墙段 */
        NFPlane *p = &s->planes[5];
        p->normal[0] = -1.0f; p->normal[1] = 0.0f; p->normal[2] = 0.0f;
        p->point[0] = 3400.0f; p->point[1] = -500.0f; p->point[2] = 800.0f;
        p->use_box = 1;
        p->box_min[0] = 3395.0f; p->box_max[0] = 3405.0f;
        p->box_min[1] = -1200.0f; p->box_max[1] = 200.0f;
        p->box_min[2] = 0.0f;    p->box_max[2] = 1600.0f;
    }
    {
        /* 平面 6: 前方 4.6m 处的墙段 */
        NFPlane *p = &s->planes[6];
        p->normal[0] = -1.0f; p->normal[1] = 0.0f; p->normal[2] = 0.0f;
        p->point[0] = 4600.0f; p->point[1] = 600.0f; p->point[2] = 900.0f;
        p->use_box = 1;
        p->box_min[0] = 4595.0f; p->box_max[0] = 4605.0f;
        p->box_min[1] = -800.0f; p->box_max[1] = 1600.0f;
        p->box_min[2] = 0.0f;    p->box_max[2] = 1800.0f;
    }
    {
        /* 平面 7: 右侧 1.5m 处的侧墙，给左右视口提供近场样本 */
        NFPlane *p = &s->planes[7];
        p->normal[0] = 0.0f; p->normal[1] = -1.0f; p->normal[2] = 0.0f;
        p->point[0] = 0.0f; p->point[1] = -1500.0f; p->point[2] = 600.0f;
        p->use_box = 1;
        p->box_min[0] = -3000.0f; p->box_max[0] = 3000.0f;
        p->box_min[1] = -1505.0f; p->box_max[1] = -1495.0f;
        p->box_min[2] = 0.0f;     p->box_max[2] = 1200.0f;
    }

    s->n_planes = 8;
    return NF_OK;
}

NFStatus nf_scene_raycast(const NFScene *s, const nf_f32 *o, const nf_f32 *d,
                          nf_f32 *hit_dist, nf_f32 *hit_p, nf_u32 *hit_plane) {
    if (!s || !o || !d || !hit_dist) return NF_E_PARAM_NULL;
    if (s->version != NF_SCENE_VERSION) return NF_E_PARAM_VERSION;

    float best = 1e30f;
    float best_p[3] = { 0, 0, 0 };
    nf_u32 best_i = 0xFFFFFFFFu;

    for (nf_u32 i = 0; i < s->n_planes; ++i) {
        const NFPlane *pl = &s->planes[i];
        const float den = dot3(d, pl->normal);
        /* 射线与平面平行: 无交点。用 1e-9 而不是 0 判,
         * 避免近掠射角算出巨大的 t。 */
        if (std::fabs(den) < 1e-9f) continue;

        const float num = dot3(pl->point, pl->normal) - dot3(o, pl->normal);
        const float t = num / den;
        if (t <= 1e-3f || t >= best) continue;

        float q[3];
        for (int k = 0; k < 3; ++k) q[k] = o[k] + d[k] * t;
        if (!inside_box(pl, q)) continue;

        best = t;
        best_i = i;
        best_p[0] = q[0]; best_p[1] = q[1]; best_p[2] = q[2];
    }

    if (best_i == 0xFFFFFFFFu) return NF_E_PARAM_RANGE;
    *hit_dist = best;
    if (hit_p) { hit_p[0] = best_p[0]; hit_p[1] = best_p[1]; hit_p[2] = best_p[2]; }
    if (hit_plane) *hit_plane = best_i;
    return NF_OK;
}

nf_u8 nf_scene_shade(const NFScene *s, const nf_f32 *p, nf_u32 plane_id) {
    if (!s || !p) return 128;
    const float f = s->tex_freq;
    /* 三个不同方向的正弦叠加, 每个平面的相位基调不同 —— 这样相邻平面
     * 在交界处纹理不连续, 便于发现"把墙当成地面"这类错误。
     * **纯 3D 坐标函数**: 同一物点在任何一帧都得同一灰度。 */
    const float phase = (float)plane_id * 1.7f;
    double v = 128.0
             + 55.0 * std::sin(p[0] * f * 1.0 + phase)
             + 50.0 * std::sin(p[1] * f * 1.3 - phase)
             + 45.0 * std::sin((p[0] + p[1]) * f * 0.7 + p[2] * f * 1.1);

    /**
     * ## 确定性高频散斑（NEW_MOD §2.4）
     *
     * ### 为什么必须加
     * 上面三项都是**规则周期函数**。SGBM 在纯周期纹理上会锁到错误的周期：
     * 实测墙面理论视差 140.8px，SGBM 报 2.2px —— 差 64 倍。这不是算法
     * 能力问题，是场景本身缺少可唯一匹配的特征。
     *
     * ### 为什么用哈希而不是加噪声
     * 场景必须是**视角不变**的：同一物点在任何一帧、任何相机里都得是同一
     * 灰度，否则 SGBM 匹配的是"两帧不同的图案"，误差本身就是假的。
     * 用世界坐标做哈希天然满足这一点 —— 同一个 3D 点永远算出同一个值。
     *
     * ### 量化尺度 0.02 (=50mm)
     * 散斑格子取 50mm。太小（1mm）时视口重采样会把格子平均掉；太大
     * （200mm）时近场一个 SGBM 窗口内只有常数，又退化成无特征。50mm 在
     * 1-4m 处对应 2-7px，正好落在 SGBM 窗口尺度里。
     *
     * 幅度 ±31.5 是在正弦叠加后的余量内，不会把结果推出 [0,255]，
     * 但又足以打破周期局部极小的平局。
     */
    {
        const int qx = (int)std::floor(p[0] * 0.02f);
        const int qy = (int)std::floor(p[1] * 0.02f);
        const int qz = (int)std::floor(p[2] * 0.02f);
        unsigned h = (unsigned)qx * 73856093u
                   ^ (unsigned)qy * 19349663u
                   ^ (unsigned)qz * 83492791u;
        h = (h ^ (h >> 13)) * 1274126177u;
        h = h ^ (h >> 16);
        /* plane_id 混进去，避免相邻平面在交界处散斑巧合相同 */
        h ^= plane_id * 2654435761u;
        const double speckle = (double)(h & 0x3Fu) - 31.5;
        v += speckle;
    }

    v = v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v);
    return (nf_u8)(v + 0.5);
}

/* ==================== 精度评估：点到表面距离 ==================== */

NFStatus nf_scene_dist_to_surface(const NFScene *scene, const nf_f32 *P,
                                  nf_f32 *out_dist) {
    if (!scene || !P || !out_dist) return NF_E_PARAM_NULL;

    nf_f32 best = 1e30f;
    nf_u8 found = 0;

    for (nf_u32 i = 0; i < scene->n_planes; ++i) {
        const NFPlane *pl = &scene->planes[i];
        /* 有符号距离: 点到平面的垂直距离 */
        const nf_f32 d = pl->normal[0] * (P[0] - pl->point[0])
                       + pl->normal[1] * (P[1] - pl->point[1])
                       + pl->normal[2] * (P[2] - pl->point[2]);
        const nf_f32 ad = (d < 0.0f) ? -d : d;

        if (pl->use_box) {
            /* 投影点落在该平面片段的范围内才算数 */
            const nf_f32 proj[3] = { P[0] - d * pl->normal[0],
                                     P[1] - d * pl->normal[1],
                                     P[2] - d * pl->normal[2] };
            nf_u8 inside = 1;
            for (int k = 0; k < 3; ++k) {
                if (proj[k] < pl->box_min[k] || proj[k] > pl->box_max[k]) {
                    inside = 0; break;
                }
            }
            if (!inside) continue;
        }
        if (ad < best) { best = ad; found = 1; }
    }

    if (!found) return NF_E_PARAM_RANGE;
    *out_dist = best;
    return NF_OK;
}

} /* extern "C" */
