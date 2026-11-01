/**
 * @file viewplan.cpp
 * @brief 6 视口规划表实现。
 */
#include "nfocc/nfocc_viewplan.h"

#include <cmath>

extern "C" {

nf_u32 nf_viewplan_default(NFViewEntry *out, nf_u32 cap) {
    if (!out || cap < NF_MAX_VIEWS) return 0;

    /* 顺序: 侧向先行(直接可用), 再前后偏转视口。
     * use_sgbm=1 需满足: 平移在视口系的横向分量占比高且极线离散小。
     * 实测(SPEC-VIEWPLAN §3): ML/MR 在直行时极线离散最小;
     * FV/RV 偏 30° 后一行内极差仍 80°, SGBM 不可用 -> use_sgbm=0。 */
    /* 需求定案 (v0.7.0): ML/MR 保持单视口 0°; FV/RV 各 ±45°; 视锥 120°。
     *
     * 偏 45° 的 LUT 有效率是 94.3%, 不是 100% —— 几何必然:
     * 视口水平 ±60°, 偏转后边缘到光轴 105°, 超出鱼眼水平视场 100°,
     * 约 5.7% 像素落在有效圆外。实测曲线 0°/30°->100%, 45°->94.3%, 60°->76.0%。
     *
     * 偏转角选 45° 的依据:
     *   - 横向基线 = T·sin(45°) = 0.707T, 比 30° 的 0.5T 高 41%
     *   - 30° 时 Z=3m 视差 9.24px(120° 视锥), 实测深度误差 -11%~+21% 抖动
     *   - 45° 的视差更大, 但极线离散也更差, 是否更准需实测(见 verify_viewplan)
     *
     * use_sgbm=0 的原因与偏转角无关: FV/RV 偏转后一行内极线方向跨度
     * 仍在 80° 量级(SPEC-VIEWPLAN §3), SGBM 的"按行水平搜索"假设不成立,
     * 只能走径向膨胀匹配。
     */
    /* v0.9.0: 视口改为**虚拟相机**（pitch=0, roll=0），yaw 用**车系绝对角**。
     *   ML  +90°    MR  −90°
     *   FV  −45° / +45°
     *   RV  −135° / +135°   （= 180° ± 45°）
     * 前值 0/0/±45/±45 是"相对真实光轴"的偏移，随安装角变化，
     * 多相机间不共面；改成绝对角后所有视口统一在同一理想相机模型下。 */
    out[0].cam_id = 2;  out[0].yaw_deg = +90.0f;  out[0].use_sgbm = 1;  /* ML */
    out[1].cam_id = 3;  out[1].yaw_deg = -90.0f;  out[1].use_sgbm = 1;  /* MR */
    out[2].cam_id = 0;  out[2].yaw_deg = -45.0f;  out[2].use_sgbm = 0;  /* FV 左前 */
    out[3].cam_id = 0;  out[3].yaw_deg = +45.0f;  out[3].use_sgbm = 0;  /* FV 右前 */
    out[4].cam_id = 1;  out[4].yaw_deg = -135.0f; out[4].use_sgbm = 0;  /* RV 左后 */
    out[5].cam_id = 1;  out[5].yaw_deg = +135.0f; out[5].use_sgbm = 0;  /* RV 右后 */
    return NF_MAX_VIEWS;
}

nf_f32 nf_viewplan_azimuth_deg(nf_f32 cam_yaw_deg, nf_f32 offset_deg) {
    /* 视口偏转绕相机 y 轴(向下)。车系 yaw 逆时针为正(从上方看)，
     * 而相机 y 轴朝下，故绕它转正角在车系里是**顺时针**，即 yaw 减小。
     * 实测对照: FV(0°) 偏 +30 后视口朝 +30(右) -> 车系 yaw 应为 −30。 */
    nf_f32 a = cam_yaw_deg - offset_deg;
    while (a <= -180.0f) a += 360.0f;
    while (a > 180.0f) a -= 360.0f;
    return a;
}

} /* extern "C" */
