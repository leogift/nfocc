/**
 * @file nfocc_viewplan.h
 * @brief 视口规划表：哪个相机生成几个视口、各自偏转多少。
 *
 * ## 为什么需要 6 视口
 * 前后相机在直行时，平移沿光轴，横向基线为 0 —— 视差深度**结构性消失**。
 * 把前后相机各生成 ±30° 两个偏转视口后，平移在视口系有横向分量
 * T·sin(30°) = 0.5T，Z=3m 处得到 9.24 px 视差（原先 0.00 px）。
 *
 * 左右相机在直行时横向基线 = 整车宽 2000 mm，本来就充足，不需要偏转。
 *
 * ## 规划表（唯一的定义处）
 * | 视口 | 相机 | 光轴 yaw | 偏转 | 视口 yaw |
 * |---|---|---|---|---|
 * | 0 | ML | −90° | 0 | −90° |
 * | 1 | MR | +90° | 0 | +90° |
 * | 2 | FV | 0° | −30° | −30° |
 * | 3 | FV | 0° | +30° | +30° |
 * | 4 | RV | 180° | −30° | +150° |
 * | 5 | RV | 180° | +30° | −150° |
 *
 * 覆盖检查：每视口水平 ±60°，6 个视口在全部方位角上至少 2 重覆盖，
 * 无空洞（见 docs/SPEC-VIEWPLAN.md）。
 *
 * ## 为什么 ML/MR 排在前面
 * 它们同属"侧向"工况，SGBM 直接可用；FV/RV 的偏转视口极线不平行，
 * 交给径向膨胀匹配。按工况排序让 pipeline 可以先跑确定能用的那几路。
 */
#ifndef NFOCC_VIEWPLAN_H
#define NFOCC_VIEWPLAN_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_MAX_VIEWS 6

/** 单个视口的规划项 */
typedef struct {
    nf_u8  cam_id;          /**< 对应哪个相机 0=FV 1=RV 2=ML 3=MR */
    nf_f32 yaw_deg;         /**< 虚拟相机的**车系绝对 yaw**（deg，0=+X 前） */
    nf_u8  use_sgbm;        /**< 1=SGBM 可用(极线近似水平); 0=走径向匹配 */
} NFViewEntry;

/** 默认 6 视口规划。返回项数。 */
nf_u32 nf_viewplan_default(NFViewEntry *out, nf_u32 cap);

/**
 * 计算某视口中心的**车系方位角**(deg, ISO 8855, 0=+X 前)。
 * 用于覆盖检查与调试。
 */
nf_f32 nf_viewplan_azimuth_deg(nf_f32 cam_yaw_deg, nf_f32 offset_deg);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_VIEWPLAN_H */
