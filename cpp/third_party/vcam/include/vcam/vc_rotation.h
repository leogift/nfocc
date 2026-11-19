/**
 * @file vc_rotation.h
 * @brief 姿态旋转与欧拉角 (纯 C, 无依赖)。
 *
 * R 一律是 **world/vehicle -> cam** 的 3x3 行优先矩阵。
 *
 * ## 坐标约定 (全库唯一来源)
 *   车体系 ISO 8855: X 前, Y 左, Z 上
 *   相机系:          X 右, Y 下, Z 前(光轴)
 *
 * ## 方向语义 (有数值断言, 见 tests/test_rotation.cpp)
 *   Valeo:  +yaw = 向左看   +pitch = 抬头   +roll = 图像顺时针
 *   camera: 相机本体系 Tait-Bryan, 无预旋转
 */
#ifndef VCAM_ROTATION_H
#define VCAM_ROTATION_H

#include "vcam/vc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** (yaw,pitch,roll)[deg] -> R[9] (world->cam, 行优先)。 */
void vc_rot_ypr_to_mat(vc_f64 yaw_deg, vc_f64 pitch_deg, vc_f64 roll_deg,
                       VCConventionKind conv, vc_f64 R[9]);

/** R[9] -> (yaw,pitch,roll)[deg]。是 vc_rot_ypr_to_mat 的严格逆。 */
void vc_rot_mat_to_ypr(const vc_f64 R[9], VCConventionKind conv, vc_f64 *yaw_deg,
                       vc_f64 *pitch_deg, vc_f64 *roll_deg);

/**
 * **增量**旋转 -> R[9]，零增量给出单位阵。
 *
 * 与 vc_rot_ypr_to_mat 的区别: 后者是绝对姿态(零姿态 = 车载相机默认朝向,
 * Valeo 下等于 PRE_ROT), 前者是"在现有姿态上再转一点"。虚拟相机用这个:
 * 恒等映射必须是 R=I, 而不是 PRE_ROT。
 */
void vc_rot_delta_to_mat(vc_f64 yaw_deg, vc_f64 pitch_deg, vc_f64 roll_deg,
                         VCConventionKind conv, vc_f64 R[9]);

/**
 * 相机系里的相对旋转 -> 车体系里可对绝对姿态**左乘**的增量:
 *     R_delta_world = PRE_ROT^T @ R_rel @ PRE_ROT
 *
 * 用途: 已知源相机绝对姿态 R_src 与相机系相对旋转 R_rel, 求目标绝对姿态
 *     R_dst = R_delta_world @ R_src
 * 只用于打印/元数据 (几何本身走 R_rel)。
 */
void vc_rot_conjugate_delta(const vc_f64 R_rel_cam[9], vc_f64 R_delta_world[9]);

/** ZXZ 欧拉角 -> R[9]。R = Rz(rotz2) @ Rx(rotx+180°) @ Rz(rotz1)。 */
void vc_rot_zxz_to_mat(vc_f64 rotz1_deg, vc_f64 rotx_deg, vc_f64 rotz2_deg,
                       vc_f64 R[9]);

/** R_rel = R_src @ R_dst^T, 满足 v_src = R_rel @ v_dst。 */
void vc_rot_relative(const vc_f64 R_src[9], const vc_f64 R_dst[9], vc_f64 out[9]);

/** 光轴 (cam +Z) 在车体系下的单位向量。 */
void vc_rot_optical_axis(const vc_f64 R[9], vc_f64 axis[3]);

void vc_rot_transpose(const vc_f64 R[9], vc_f64 out[9]);
void vc_rot_matvec(const vc_f64 R[9], const vc_f64 v[3], vc_f64 out[3]);
int vc_rot_is_identity(const vc_f64 R[9], vc_f64 tol);

/** 把光轴翻译成人类可读朝向, 例如 "left 45.0deg, down 22.7deg"。 */
void vc_rot_facing(const vc_f64 R[9], char *buf, int buflen);

#ifdef __cplusplus
}
#endif

#endif /* VCAM_ROTATION_H */
