/**
 * @file vc_io_ws.h
 * @brief WoodScape / NFOCC 标定 JSON 的最小读取器。
 *
 * 为什么自己写一个 JSON 解析: 核心库要零依赖, 而这格式只有一层嵌套、
 * 字段固定 (extrinsic.quaternion/translation + intrinsic.k1..k4/cx_offset/
 * cy_offset/width/height/aspect_ratio)。引 RapidJSON 只为读 12 个字段
 * 不划算, 而且引入后核心库就不再零依赖了。
 *
 * 限制: 只吃**本格式**, 不做通用 JSON。遇到不认识的字段直接忽略; 必需字段
 * 缺失则报错并指出缺哪个 —— 不做"缺了就填默认值"这种事(旧实现填 1920x1080)。
 */
#ifndef VCAM_IO_WS_H
#define VCAM_IO_WS_H

#include "vcam/vc_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char name[32];
    /* 内参 */
    vc_i32 width, height;
    vc_f64 k1, k2, k3, k4;
    vc_f64 cx_offset, cy_offset;
    vc_f64 aspect_ratio;
    vc_i32 poly_order;
    /* 外参: 四元数 (x,y,z,w) + 平移, 语义是 **cam -> vehicle** */
    vc_f64 quat[4];
    vc_f64 trans[3];
    vc_i32 has_extrinsic;
} VCWoodScapeCalib;

/**
 * 读一个 WoodScape 标定 JSON。
 * @return VC_OK / VC_E_PARAM_NULL / VC_E_CAM_NOT_CALIBRATED(缺字段)
 */
VCStatus vc_ws_load(const char *path, VCWoodScapeCalib *out);

/**
 * 标定 -> 径向多项式相机模型。
 * 主点 = width/2 + cx_offset (不减 0.5; 见 README 的"主点约定"一节)。
 */
VCStatus vc_ws_to_camera(const VCWoodScapeCalib *calib, VCCameraModel *out);

/**
 * 四元数 (cam->vehicle) -> R (vehicle->cam), 行优先。
 * 标定给的是 v_veh = R_c2v v_cam, 本库要的是 R_w2c = R_c2v^T。
 */
void vc_ws_pose(const VCWoodScapeCalib *calib, vc_f64 R_out[9]);

/** 单位四元数 (x,y,z,w) -> 3x3 旋转矩阵。 */
void vc_quat_to_mat(const vc_f64 q[4], vc_f64 R_out[9]);

#ifdef __cplusplus
}
#endif

#endif /* VCAM_IO_WS_H */
