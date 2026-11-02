/**
 * @file nfocc_scene.h
 * @brief 解析真值场景：射线-平面求交得到**闭式**真值深度。
 *
 * ## 为什么需要这个模块
 * 现有合成源用「纹理横移」模拟运动（`synth_image` 里 sin(x*0.031+phase)），
 * 那不是真实 3D 投影 —— 没有地面、没有物体、没有真值。
 * 因此在它上面测出的任何数字都只是**中间量**精度（匹配 0.045px、
 * 解析式自校 0.000mm），无法回答「端到端深度准不准」。
 *
 * 原始方案阶段 A 的验收标准是「1–3 m 与激光真值偏差中位数 < 150 mm」，
 * 至今未验证，根本原因就是缺这个模块。
 *
 * ## 场景定义
 * 世界 = 车体系（x 前, y 左, z 上, 后轴中心为原点）。由若干**平面**组成：
 *   - 地面：z = 0
 *   - 竖直平面：过点 p0、法向 n 的半无限面（用外接框限定范围）
 *
 * 对每条射线，取**最近的交点**作为真值。求交是闭式的：
 *     t_hit = ((p0 − o)·n) / (d·n)
 * 无解（d·n ≈ 0）或交点在框外则跳过，继续下一个平面。
 *
 * ## 为什么用平面而不是网格
 * 平面的真值可闭式算出，误差只来自浮点；网格需要插值，会引入
 * 本身的不确定度，反而无法判断被测实现的误差。
 */
#ifndef NFOCC_SCENE_H
#define NFOCC_SCENE_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_SCENE_MAX_PLANES 8

/** 一个平面（法向 + 过点 + 可选范围框） */
typedef struct {
    nf_f32 normal[3];      /**< 单位法向 */
    nf_f32 point[3];       /**< 平面上一点 */
    /** 范围框（世界系 AABB）。若 use_box=0 则视为无限大平面 */
    nf_u8  use_box;
    nf_f32 box_min[3];
    nf_f32 box_max[3];
    nf_u8  is_ground;      /**< 1=地面（用于自校验） */
    nf_u8  _pad[3];
} NFPlane;

typedef struct {
    nf_u32  version;                 /**< = NF_SCENE_VERSION */
    nf_u32  n_planes;
    NFPlane planes[NF_SCENE_MAX_PLANES];
    /** 纹理基频（每 mm 的相位），决定表面花纹密度 */
    nf_f32  tex_freq;
} NFScene;

#define NF_SCENE_VERSION 1u

/** 默认场景：地面 + 正前方一面墙 + 左侧一面墙 */
NFStatus nf_scene_default(NFScene *s);

/**
 * 按**相机焦距与参考距离**设置纹理空间频率。
 *
 * ## 为什么必须按 f 设
 * 纹理在视口里的像素周期 = 空间周期 · f / Z。
 * f 变了（视口张角变），同一空间周期的纹理在像素上的周期就变了。
 * 写死 tex_freq 会让"纹理周期(px)"随 f 漂移 —— 张角 150° 时
 * f 只有 128.6（120°/50° 时为 184.75），2m 处周期从 13.8px 掉到 9.6px，
 * 低于 SGBM 5x5 窗口能稳定捕捉的下限，深度误差从百毫米级涨到 900mm。
 *
 * @param target_px  期望的像素周期（推荐 16~24，默认 20）
 * @param ref_z_mm   参考距离（默认 2000mm）
 */
NFStatus nf_scene_set_tex_for_focal(NFScene *s, nf_f32 focal_px,
                                    nf_f32 target_px, nf_f32 ref_z_mm);

/**
 * 射线求交，返回最近的交点距离（沿 d，d 必须单位化）。
 * @param o,d     起点与单位方向（世界系）
 * @param hit_p   输出交点坐标（可为 nullptr）
 * @param hit_plane 输出命中的平面序号（可为 nullptr）
 * @return NF_OK 命中；NF_E_PARAM_RANGE 未命中
 */
NFStatus nf_scene_raycast(const NFScene *s, const nf_f32 *o, const nf_f32 *d,
                          nf_f32 *hit_dist, nf_f32 *hit_p, nf_u32 *hit_plane);

/**
 * 表面纹理亮度：把交点映射到 0..255。
 * **必须是 3D 坐标的确定性函数** —— 同一物点在两帧必须得同一灰度，
 * 否则匹配器会"正常返回但全错"（Python 原型踩过：逐像素随机噪声
 * 使 SGBM 输出 3.6px 而真值 133px）。
 */
nf_u8 nf_scene_shade(const NFScene *s, const nf_f32 *p, nf_u32 plane_id);

/**
 * 点到场景表面的**最小距离**（mm）。用于精度评估。
 *
 * ## 为什么用这个口径而不是"沿射线求交"
 * 沿射线求交对**掠射面**（地面那样几乎与视线平行的面）极不稳定：
 * 点有微小横向偏差，射线就会打到很远的另一处，距离差被放大几十倍。
 * 实测用它评 1-2 m 段得到 1540 mm，而同一批点的视差误差对应的
 * 深度理论误差只有 ~30 mm —— 差 50 倍，全是度量方法的伪影。
 *
 * 点到表面距离对横向偏差不敏感（横向移动沿平面滑动，距离不变），
 * 只反映**法向**偏离，也就是"这个体素离真实表面多远"，
 * 正是"体素位置偏差"的定义。
 *
 * @param P        点 (车系, mm)
 * @param out_dist 输出最小距离 (mm, >=0)；无有效表面时返回 NF_E_PARAM_RANGE
 */
NFStatus nf_scene_dist_to_surface(const NFScene *scene, const nf_f32 *P,
                                  nf_f32 *out_dist);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_SCENE_H */
