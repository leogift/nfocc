/**
 * @file nfocc_viewplan.h
 * @brief 6 视口规划表（分类张角）。
 *
 * ## 为什么前后和左右的张角不同
 * 前后视口**成对使用**：FV 出 ±45° 两个，中心相隔 90°，各 110° 张角。
 * 90° 张角时两个视口只在 0° 交于一点，正前方目标同时落在两个视口的
 * 边缘，匹配质量最差；加到 110° 后留 20° 重叠，正前方不再贴边。
 * 实测（解析场景、前向 FV+RV）：竖面中位 1213 mm（90°）-> 791 mm（110°）。
 *
 * 110° 时 f = (590/2)/tan55° ≈ 206.6 px/rad，仍是侧视 150°（137）的 1.5 倍。
 * 覆盖范围 [-100°, +100°]，两侧各多出 10°，与左右视口有少量重叠。
 *
 * 左右视口**单个使用**：一个视口要覆盖整侧，必须 150°。
 *
 * 这是"用覆盖换角分辨率"的直接应用：前后有重叠，张角可比 150° 窄；
 * 左右没有，只能放宽。
 */
#ifndef NFOCC_VIEWPLAN_H
#define NFOCC_VIEWPLAN_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_VIEWPLAN_VERSION  1u

/** 单个视口的规划条目 */
typedef struct {
    nf_u32 cam_id;          /**< 0=FV 1=RV 2=MVL 3=MVR */
    nf_f32 yaw_deg;         /**< 车系绝对 yaw (ISO 8855, 逆时针正) */
    nf_f32 hfov_deg;        /**< 该视口的水平张角 */
    nf_f32 vfov_deg;        /**< 垂直张角 */
    /**
     * 视口宽 (px)。**按类别不同**：
     *   左右 150° -> 1024（f=137.2）
     *   前后 110° -> 590 （f=206.6，垂直 120° 高度约 716）
     * 高度不由这里给，由 vfov 与 f 派生。
     */
    nf_u32 width_px;
    nf_u8  is_side;         /**< 1 = 侧向视口 (ML/MR) */
    nf_u8  _pad[3];
} NFViewPlanEntry;

/**
 * 6 视口规划：
 *   FV  -45° / +45°   110° 张角（中心相隔 90°）
 *   RV -135° / +135°  110° 张角
 *   ML  +90°          150° 张角
 *   MR  -90°          150° 张角
 *
 * @return 写入的条目数（固定 6）
 */
nf_u32 nf_viewplan_default(NFViewPlanEntry *out, nf_u32 max_entries);

/** 按 cam_id 与 yaw 查条目；找不到返回 NULL */
const NFViewPlanEntry *nf_viewplan_find(const NFViewPlanEntry *plan, nf_u32 n,
                                        nf_u32 cam_id, nf_f32 yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_VIEWPLAN_H */
