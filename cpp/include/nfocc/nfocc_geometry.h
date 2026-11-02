/**
 * @file nfocc_geometry.h
 * @brief 三角化、基线方向判定、σ 传播。
 *
 * 规范: docs/SPEC-FORWARD.md §2/§4, docs/SPEC-EPIPOLAR.md §4
 *
 * ## 为什么不是 Z = f·b/d (ADR-002)
 * 该式只在**平移垂直于视线**时成立。本项目相机俯角 0°、车辆直行,
 * 视线与平移夹角从 0° 到近 90°。Python 原型实测: 光轴附近相对误差 **10.3 倍**。
 * 生产路径用射线最小二乘:
 *     P = C0 + s0·r0 = C1 + s1·r1  =>  [r0, −r1]·[s0,s1]ᵀ = t
 */
#ifndef NFOCC_GEOMETRY_H
#define NFOCC_GEOMETRY_H

#include "nfocc_types.h"
#include "nfocc_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 运行模式 (SPEC-FORWARD §4) */
typedef enum {
    NF_MODE_STEREO       = 0,   /**< 侧向, 走 SGBM */
    NF_MODE_FORWARD      = 1,   /**< 前向, 走解析测距 */
    NF_MODE_HYBRID       = 2,   /**< 中间带, 两条都跑 */
    NF_MODE_FROZEN_NEAR  = 3,   /**< B < B_MIN */
    NF_MODE_FROZEN_FAR   = 4,   /**< B > B_MAX */
    NF_MODE_FROZEN_GAP   = 5,   /**< 丢帧 */
    NF_MODE_FROZEN_ROT   = 6    /**< 急转/原地掉头: 两帧 yaw 漂移 > 15° */
} NFMode;

/** 模式是否产出深度 */
nf_u8 nf_mode_produces_depth(NFMode m);
const char *nf_mode_str(NFMode m);

/** 门控决策 */
typedef struct {
    NFMode  mode;
    nf_f32  baseline_mm;   /**< |t| */
    nf_f32  sigma;          /**< |t_xy| / |t|, 侧向程度 */
    nf_f32  epi_angle_deg;  /**< 极线与图像横轴夹角 */
} NFGateDecision;

/**
 * 由相机系平移给出模式 (SPEC-FORWARD §4)。
 * @param t_cam 相机系平移 [3], mm
 * @param frame_gap 距参考帧的帧间隔
 */
NFStatus nf_gate_decide(const nf_f32 *t_cam, nf_u32 frame_gap,
                        nf_f32 b_min_mm, nf_f32 b_max_mm,
                        nf_f32 sigma_stereo_min, nf_f32 sigma_forward_max,
                        nf_u32 gap_max_frames,
                        NFGateDecision *out);

/**
 * 射线最小二乘三角化(批量)。
 * @param c0  参考帧光心 [3] (相机系, 通常为原点)
 * @param r0  参考帧单位射线 [n*3] (相机系)
 * @param c1  当前帧光心 [3] (同一坐标系)
 * @param r1  当前帧单位射线 [n*3]
 * @param s0  输出: 参考帧距离 [n], 无效为 NaN
 * @param ok  输出: 有效性 [n]
 */
NFStatus nf_triangulate_rays(const nf_f32 *c0, const nf_f32 *r0,
                             const nf_f32 *c1, const nf_f32 *r1,
                             nf_u32 n, nf_f32 *s0, nf_u8 *ok);

/**
 * 有效基线: t 在**垂直于视线**方向的分量大小。
 * 前向平移时 b_perp << |t|, 直接拿 |t| 会把 σ_Z 高估数倍。
 */
NFStatus nf_effective_baseline(const nf_f32 *r, nf_u32 n,
                               const nf_f32 *t,
                               nf_f32 *b_perp);

/**
 * σ 传播。σ_Z = sqrt( (Z²/(f·b_perp))²·σ_d² + (Z·σ_b_ratio)² )
 * 第一项是视差误差传播, 第二项是基线尺度误差(相对)。
 */
NFStatus nf_sigma_depth(const nf_f32 *z_mm, const nf_f32 *b_perp_mm,
                        nf_u32 n, nf_f32 f_px, nf_f32 sigma_d_px,
                        nf_f32 sigma_b_ratio, nf_f32 *sigma_out_mm);

/**
 * 前向运动的径向膨胀解析测距 (SPEC-FORWARD §2)。
 *
 *   Z = Σ[(û×v̂)·(t×v̂)] / Σ|û×v̂|²
 *
 * **对任意平移方向成立**, 不要求极线平行。
 * @param u_hat 参考帧单位方向 [n*3]
 * @param v_hat 当前帧对应点单位方向 [n*3]
 * @param t     平移(同一坐标系) [3]
 * @param z_out 输出距离 |q0| [n]
 * **注意 Z 是距离 |q0|, 不是光轴深度 q0.z** —— 混用会让结果差数倍。
 */
NFStatus nf_forward_range(const nf_f32 *u_hat, const nf_f32 *v_hat,
                          nf_u32 n, const nf_f32 *t,
                          nf_f32 *z_out, nf_u8 *ok);

/** 极线与图像横轴夹角(度)。判据见 SPEC-EPIPOLAR §1 */
nf_f32 nf_epi_angle_deg(const nf_f32 *t_cam);

/**
 * D3: FoE 发散锥代数过滤。
 *
 * 平移运动产生的像移全部从运动极点 (FoE) 辐射出去，因此**沿平移方向**
 * 的视线在立体匹配里几乎不产生视差 —— 那里的深度是退化的，
 * 硬解会得到几十米外的"前向拉丝"幽灵点。
 *
 * ## 判据
 * ```
 * h     = P - apex                      // apex 必须是**相机光心**
 * delta = h · dir_hat                   // 沿平移方向的投影
 * 落在锥内 <=>  delta > 0 && delta^2 >= |h|^2 * cos^2(aperture)
 * ```
 * 等价于「视线与平移方向夹角 <= aperture」。默认 aperture = 8°。
 *
 * ## apex 的物理含义（易错点）
 * `apex` = **当前帧相机光心**在车体系的位置 (mm)，**不是**车辆平移向量，
 * 也不是参考帧光心。传入 pose->t 才是对的；传 t_veh 会让锥顶点整体偏移，
 * 过滤边界跟着歪。
 *
 * @param P            候选物点 (车系, mm)
 * @param apex         相机光心 (车系, mm)
 * @param dir          平移方向单位向量 (车系)，由 t_veh 归一化得到
 * @param cos_ap_sq    cos^2(半张角)。8° -> 0.9806
 * @return 1 = 落在发散锥内（应剔除）, 0 = 锥外（保留）
 */
nf_u8 nf_pt_in_foe(const nf_f32 *P, const nf_f32 *apex,
                   const nf_f32 *dir, nf_f32 cos_ap_sq);

/**
 * D2: 双向视差一致性软抑制权重。
 *
 * 前后向匹配残差越大，越可能是遮挡/重复纹理导致的误匹配。
 * 用**平滑衰减**而非硬阈值，避免体素边界出现锯齿。
 *
 * ```
 * r = |d_fwd - d_bwd|
 * w = 1.0                               , r <= tau
 * w = 1/(1 + lambda*(r/tau - 1))        , r >  tau
 * ```
 *
 * ## 符号约定（易错点）
 * `d_bwd` 必须与 `d_fwd` **同号约定**。若后向视差定义为反向搜索得到的
 * 负值，调用方应先取反再加，否则残差恒为两值之和，权重全部误判。
 * 本实现按「同号」约定，与 SGBM 的 disp12MaxDiff 语义一致。
 *
 * @param tau     容差 (px)，默认 1.0
 * @param lambda  惩罚系数，默认 14.0
 * @return 权重 [0, 1]
 */
nf_f32 nf_occlusion_weight(nf_f32 d_fwd, nf_f32 d_bwd,
                           nf_f32 tau, nf_f32 lambda);

/**
 * 物理空间不确定度（cm，1..254，255=哨兵）。
 *
 * ## 采用 L2 平方和（严格方差传播）
 * ```
 * Z    = f*b/d
 * sZ   = sqrt( (f*b/d^2 * sig_d)^2 + (f/d * sig_b)^2 )
 * ```
 * **原规范写的是 L1 线性相加**（`f*(sig_d*b/d^2 + b*sig_b/d)`），
 * 那是保守上界，比严格值大 sqrt(2) 倍。文档若称"严格偏导传播"
 * 就与实现矛盾。此处实现 L2，并在文档中注明。
 *
 * @param f_px    视口等效焦距 (px)
 * @param b_mm    基线 (mm)
 * @param d_px    视差 (px)，<=0 返回哨兵 255
 * @param sig_d   像素匹配标准差 (px)，典型 0.5
 * @param sig_b   基线**相对**误差率（无量纲），典型 0.02
 * @return 不确定度 (cm)，[1,254]，无效返回 255
 */
nf_u8 nf_compute_spatial_uncertainty_cm(nf_f32 f_px, nf_f32 b_mm,
                                        nf_f32 d_px, nf_f32 sig_d,
                                        nf_f32 sig_b);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_GEOMETRY_H */
