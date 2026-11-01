/**
 * @file hanqtech_loader.h
 * @brief hanqtech 数据集加载器：标定 JSON + PNG 图像 + 车体运动信息。
 *
 * ## 数据集约定（来自 calibration/readme.txt，已实测核对）
 *
 * 内参：四次多项式 ρ(θ) = k1θ + k2θ² + k3θ³ + k4θ⁴
 *   像素坐标：u = ρ·X/χ + cx_offset + width/2 − 0.5
 *             v = ρ·Y/χ·aspect + cy_offset + height/2 − 0.5
 *   因此**主点像素坐标** = cx_offset + width/2 − 0.5（不是 cx_offset 本身）。
 *
 * 外参：camera → vehicle 的变换，与项目约定 v_V = R·v_C 完全一致，
 *   可**直接**填入 NFPose（实测：正交、det=+1、y 轴朝下、光轴朝外）。
 *
 * 安装角（实测，与理想外参差别很大）：
 *   FV  方位 −0.1°  俯角 22.7°  光心 (3748, 0, 681)
 *   MVL 方位 +105.9° 俯角 73.8°  光心 (1945, +957, 955)
 *   MVR 方位 −107.9° 俯角 74.2°  光心 (1945, −957, 975)
 *   RV  方位 +178.7° 俯角 37.3°  光心 (−1068, +55, 909)
 *   —— MVL/MVR 是**侧下方俯视**，不是侧视。
 */
#ifndef NFOCC_HANQTECH_LOADER_H
#define NFOCC_HANQTECH_LOADER_H

#include "nfocc/nfocc_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_WS_MAX_CAMS 4

/** 一组同 timestamp 的四路标定 */
typedef struct {
    nf_u32       n_cams;
    NFCameraModel cam[NF_WS_MAX_CAMS];
    NFPose        pose[NF_WS_MAX_CAMS];
    /** 相机名: "FV"/"MVL"/"MVR"/"RV"，用于日志 */
    char          name[NF_WS_MAX_CAMS][8];
    /** 帧号: 00164 等 */
    nf_u32        frame_id;
    /** 原始图像尺寸（px）。视口宽度必须按它算: width/downscale */
    nf_u32        image_w, image_h;
} NFWSCalib;

/**
 * 从标定 JSON 加载。
 * @param json_path  形如 .../calibration/00164_FV.json
 * @param cam_id     填入的相机序号 (0..3)
 * @param out_calib  累积写入
 */
NFStatus nf_ws_load_calib(const char *json_path, nf_u32 cam_id,
                          NFWSCalib *out_calib);

/**
 * 按帧号加载四路标定（自动按 FV/MVL/MVR/RV 排序）。
 * @param calib_dir   .../calibration/calibration
 * @param frame_id    00164
 */
NFStatus nf_ws_load_frame(const char *calib_dir, nf_u32 frame_id,
                          NFWSCalib *out_calib);

/** 车辆运动信息（从 vehicle_info 的 JSON 解析） */
typedef struct {
    nf_u32   frame_id;
    nf_u64   timestamp_us;      /**< 原始 timestamp 是整数, 单位未标注 */
    nf_f32   speed_mps;         /**< ego_speed */
    nf_f32   yaw_rate;          /**< ego_yawRate */
    nf_f32   steering;          /**< ego_steering */
    nf_f32   acceleration;      /**< ego_acceleration */
    nf_f32   distance_m;        /**< ego_distance, 本帧行驶距离 */
    char     gear[4];
} NFWSEgo;

NFStatus nf_ws_load_ego(const char *json_path, NFWSEgo *out);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_HANQTECH_LOADER_H */
