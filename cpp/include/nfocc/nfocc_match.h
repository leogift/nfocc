/**
 * @file nfocc_match.h
 * @brief 前向运动的对应点提取（径向膨胀测距的前置）。
 *
 * 规范: docs/SPEC-FORWARD.md
 *
 * ## 为什么不用 SGBM 找前向对应点
 * SPEC-FORWARD §1 的三条实测事实已经说明：前向平移时极线从 FoE 辐射、
 * 同一行内极线方向差 175.9°、像移左右异号。SGBM 按行单向水平搜索
 * **结构上不适用**，调参无效。
 *
 * ## 本模块的做法
 * 按"径向"搜索：对每个视口像素 p，沿它相对 FoE 的射线方向搜，
 * 因为纯前向平移时像点只沿该方向移动（径向膨胀）。
 *
 * 精度要求来自 SPEC-FORWARD §4：1 px 对应 110 mm 深度误差，
 * 要做到 200 mm 体素必须 **≤0.5 px**。所以匹配后必须做亚像素细化。
 *
 * ## 已知限制
 * - 只在**纯前向**（|t_perp|/|t| 小）时有效；侧向走 SGBM。
 * - 需要足够纹理；无纹理区域匹配失败并标 valid=0。
 * - 视口边缘处径向位移可能超出搜索范围（大位移），标 valid=0。
 */
#ifndef NFOCC_MATCH_H
#define NFOCC_MATCH_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** FoE（膨胀中心，聚焦点）在视口坐标中的位置。前向平移时 FoE = 光心。 */
typedef struct {
    nf_f32 u;        /**< FoE 列坐标(像素, 可为小数) */
    nf_f32 v;        /**< FoE 行坐标 */
} NFFoe;

/**
 * 径向膨胀匹配：在当前视口图中找 ref 视口图每一像素的对应点。
 *
 * @param cur        当前帧视口图(灰度, w*h)
 * @param ref        参考帧视口图(灰度, w*h)
 * @param valid_cur  当前帧视口有效性(w*h, 非 0 表示可用)
 * @param w, h       视口尺寸
 * @param foe        膨胀中心
 * @param r_min,r_max 搜索的径向位移范围(像素)。
 *                    由基线与深度范围推出：r = |p-a|·(Z/(Z-T)-1)
 * @param patch      匹配块半径(1=3x3, 2=5x5)
 * @param ncc_min    NCC 相似度下限，低于此判匹配失败
 * @param out_cur    输出：当前帧对应点(2*w*h, u,v 交替)
 * @param out_conf   输出：NCC 置信度(0..1)
 * @param out_valid  输出：是否匹配成功
 * @return NF_OK；参数错误返回对应状态码
 */
NFStatus nf_match_radial(const nf_u8 *cur, const nf_u8 *ref,
                         const nf_u8 *valid_cur,
                         nf_u32 w, nf_u32 h, NFFoe foe,
                         nf_f32 r_min, nf_f32 r_max,
                         nf_u32 patch, nf_f32 ncc_min,
                         nf_f32 *out_cur, nf_f32 *out_conf, nf_u8 *out_valid);

/**
 * 由匹配结果算深度：沿 FoE 射线做一维膨胀求解。
 *
 * 对每个有效点：径向半径 ρ0 = |p − a|，ρ1 = |q − a|，
 * 则 s = ρ1/ρ0，深度 Z = T·ρ0/(ρ1 − ρ0)（T 为前向位移）。
 *
 * **注意**：这是 SPEC-FORWARD §3 的**近似式**（假设纯前向、垂直于像面的
 * 深度）。精确解由 nf_forward_range 用射线最小二乘给出。本函数用于
 * 快速筛查，最终结果应走精确解。
 */
NFStatus nf_depth_from_radial(const nf_f32 *pts, const nf_u8 *valid,
                              nf_u32 w, nf_u32 h, NFFoe foe,
                              nf_f32 t_forward_mm,
                              nf_f32 *out_depth, nf_f32 *out_sigma);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_MATCH_H */
