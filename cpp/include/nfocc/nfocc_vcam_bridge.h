/**
 * @file nfocc_vcam_bridge.h
 * @brief vcam 后端桥接：把 NFOCC 的相机模型/视口参数转成 vcam 的等价表示。
 *
 * ## 为什么是"桥接"而不是"替换"
 *
 * `virtualcamera/cpp` 的相机模型比 NFOCC 的通用（多 KB 独立 fx/fy、
 * e-ftheta、Brown-Conrady 切向项），但它**没有** NFOCC 这几层语义：
 *
 *   1. 环带角是**绝对俯角**（相对地面），不是相对光轴 ——
 *      hanqtech MVL 光轴下俯 74°，把环带当相对角会看到 0.43 m 处的地面
 *      而不是 1-4 m；
 *   2. 四道有效性淘汰要**分开计数**（出图/超半径/超角度/车身掩码），
 *      混成一个数就没法定位配置问题；
 *   3. 车身掩码在 LUT 构建期生效，不是后处理。
 *
 * 直接换成 vc_conv_build_maps 会把这三样丢掉。所以这里只借 vcam 的
 * **相机模型层**（unproject / project / rho 求根），LUT 网格构建仍走
 * NFOCC 自己的实现。
 *
 * ## 开启条件
 * 编译期 `NF_WITH_VCAM=1`。未开启时本文件不参与构建，
 * `nf_vcam_available()` 由 CMake 决定返回常量。
 */
#ifndef NFOCC_VCAM_BRIDGE_H
#define NFOCC_VCAM_BRIDGE_H

#include "nfocc/nfocc_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_VCAM_BRIDGE_VERSION  1u

/** 后端类型 */
typedef enum {
    NF_VCAM_BACKEND_NONE = 0,   /**< 未编译进 vcam，走 NFOCC 原生实现 */
    NF_VCAM_BACKEND_LINKED = 1  /**< vcam 可用 */
} NFVcamBackend;

/** 查询本二进制是否链接了 vcam */
NFVcamBackend nf_vcam_backend(void);

/**
 * 用 vcam 的相机模型做**相机系射线 -> 像素**投影。
 *
 * 与已删除的原生实现语义相同（落在图像外或后半球返回 ok=0），
 * 但内部走 vcam 的 `vc_cam_project`。
 *
 * @return NF_OK 正常（即使 ok=0）；vcam 不可用时返回
 *         NF_E_ENGINE_UNSUPPORTED，调用方应回退到 `nf_cam_project_c`。
 */
NFStatus nf_vcam_project(const NFCameraModel *cam, nf_f32 X, nf_f32 Y, nf_f32 Z,
                         nf_f32 *u, nf_f32 *v, nf_u8 *ok);

/**
 * 用 vcam 做**像素 -> 相机系单位射线**。
 *
 * @param theta_deg 可选，输出入射角
 * @param converged 可选，0 = 求根未收敛（调用方不得使用该射线）
 */
NFStatus nf_vcam_unproject(const NFCameraModel *cam, nf_f32 u, nf_f32 v,
                           nf_f32 ray[3], nf_f32 *theta_deg, nf_u8 *converged);

/**
 * 交叉校验：用两个后端各算一遍，报告最大偏差（px）。
 *
 * 这不是生产路径，是**验证工具** —— 换后端时先跑它，
 * 确认两条路在同一批采样点上一致，才敢让 vcam 接管投影。
 *
 * @param n_samples 采样数（在图像上均匀取）
 * @param out_max_dev_px 输出最大偏差
 * @param out_n_compared 输出实际比较的点数（排除双方都判无效的）
 */
NFStatus nf_vcam_crosscheck(const NFCameraModel *cam, nf_u32 n_samples,
                            nf_f32 *out_max_dev_px, nf_u32 *out_n_compared);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_VCAM_BRIDGE_H */
