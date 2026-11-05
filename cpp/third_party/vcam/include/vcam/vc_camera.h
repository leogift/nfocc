/**
 * @file vc_camera.h
 * @brief 相机模型: 内参 + 正/反投影 + 有效性判定。
 *
 * 规范出处: 与 Python 版 vcam/models/ 一一对应, 公式完全相同。
 *
 * ## 坐标约定 (与 NFOCC/cpp nfocc_camera.h 一致)
 *   相机系: X 右, Y 下, Z 前 (光学轴)。所有模型都吃这个系。
 *   车体系: ISO 8855, X 前, Y 左, Z 上。
 *
 * ## 有效性判据
 *   鱼眼: **只看是否落在图像内**, 不限制 theta<=90°。
 *     实测 WoodScape FV: rho(90°)=598px, rho(95°)=641px, 图像主点到左边界
 *     644px —— 图像里本来就有 theta>90° 的像素。用 90° 截断会错杀它们
 *     (Python 版初版犯过, 2000 个图像内像素错杀 370 个)。
 *   针孔: Z>0 (除法 x=X/Z 的物理边界) 且落在图像内。
 *
 * ## 反投影的收敛
 *   所有求根都返回 converged 标记; 调用方必须检查, 不能把未收敛值当结果用。
 */
#ifndef VCAM_CAMERA_H
#define VCAM_CAMERA_H

#include "vc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 相机模型。字段按 kind 解释, 不用到的一律填 0。 */
typedef struct {
    vc_u32 version;    /**< = VC_CAMERA_MODEL_VERSION */
    VCModelKind kind;

    vc_i32 width;      /**< 图像宽(像素) */
    vc_i32 height;

    /* --- 针孔 / KB: 焦距与主点 --- */
    vc_f64 fx, fy;
    vc_f64 cx, cy;

    /* --- KB: rho = theta*(1+k1 t^2+k2 t^4+k3 t^6+k4 t^8) ---
     * --- RADIAL_POLY: rho = k1 t+k2 t^2+k3 t^3+k4 t^4+k5 t^5 ---
     *     (k1 的单位是"像素/弧度", 即小角度等效焦距)
     * --- EFTHETA: 同上, 另加仿射斜切 --- */
    vc_f64 k[5];

    /* --- 针孔: Brown-Conrady 切向项 --- */
    vc_f64 p1, p2;

    /* --- EFTHETA: 仿射 A=[[c,d],[e,1]], det=c-d*e 必须非奇异 --- */
    vc_f64 efc, efd, efe;

    /* --- RADIAL_POLY: v 方向额外缩放 (WoodScape aspect_ratio) --- */
    vc_f64 aspect_ratio;
} VCCameraModel;

/**
 * 构造径向多项式模型 (WoodScape / NFOCC)。
 * k[0] 即小角度焦距, 必须为正。
 */
VCStatus vc_cam_make_radial_poly(VCCameraModel *out, vc_i32 width, vc_i32 height,
                                 const vc_f64 k[4], vc_f64 cx, vc_f64 cy,
                                 vc_f64 aspect_ratio);

/** 构造针孔模型 (可带畸变)。 */
VCStatus vc_cam_make_pinhole(VCCameraModel *out, vc_i32 width, vc_i32 height,
                             vc_f64 fx, vc_f64 fy, vc_f64 cx, vc_f64 cy);

/** 构造 KB 模型。 */
VCStatus vc_cam_make_kb(VCCameraModel *out, vc_i32 width, vc_i32 height,
                        vc_f64 fx, vc_f64 fy, vc_f64 cx, vc_f64 cy,
                        vc_f64 k1, vc_f64 k2, vc_f64 k3, vc_f64 k4);

/** 构造 e-ftheta 模型。k 长度 n_k (2..5), 仿射 c/d/e。 */
VCStatus vc_cam_make_eftheta(VCCameraModel *out, vc_i32 width, vc_i32 height,
                             const vc_f64 *k, vc_i32 n_k, vc_f64 c, vc_f64 d,
                             vc_f64 e, vc_f64 cx, vc_f64 cy);

/** 参数自检: 尺寸>0, 焦距/k1 有效, 仿射非奇异。 */
VCStatus vc_cam_validate(const VCCameraModel *cam);

/**
 * 像素 -> 相机系单位射线。
 * @param ray_out 长度 3 的数组 (x,y,z), 单位长度
 * @param theta_deg_out 可选, 入射角(度)
 * @param converged_out 可选, 0/1
 */
VCStatus vc_cam_unproject(const VCCameraModel *cam, vc_f64 u, vc_f64 v,
                          vc_f64 ray_out[3], vc_f64 *theta_deg_out,
                          vc_i32 *converged_out);

/**
 * 相机系向量 -> 像素。**落在图像外或后半球返回 u=v=-1**。
 * @return VC_OK 正常 (即使返回了 -1, 因为"不可见"不是错误)
 */
VCStatus vc_cam_project(const VCCameraModel *cam, vc_f64 X, vc_f64 Y, vc_f64 Z,
                        vc_f64 *u_out, vc_f64 *v_out);

/** 小角度等效焦距 (radial_poly 的 k1, 其余模型为 fx)。 */
vc_f64 vc_cam_focal_scale(const VCCameraModel *cam);

/** rho(theta) 及其导数, 供 LUT 构建与自检。 */
vc_f64 vc_cam_rho(const VCCameraModel *cam, vc_f64 theta);
vc_f64 vc_cam_rho_prime(const VCCameraModel *cam, vc_f64 theta);

/** 由 rho 反解 theta。不收敛返回负值并置 *ok=0。 */
vc_f64 vc_cam_solve_theta(const VCCameraModel *cam, vc_f64 rho, vc_i32 *ok);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* VCAM_CAMERA_H */
