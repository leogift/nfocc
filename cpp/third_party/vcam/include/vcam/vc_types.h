/**
 * @file vc_types.h
 * @brief vcam C++ 实现的基础类型与状态码。纯 C 头, C/C++ 均可包含。
 *
 * 命名与 NFOCC/cpp 保持一致: 类型 VC*_t / 函数 vc_* / 枚举 VC_* / 状态码 VC_E_*。
 * 理由: 这两个工程最终会跑在同一块板子上、由同一套上层代码调用, 接口风格
 * 分裂会让集成时多写一层胶水, 也就多一处出错的地方。
 *
 * ## 与 Python 版 (vcam/) 的对应关系
 *   rotation.py      -> vc_rotation.h/.c
 *   models 各文件     -> vc_camera.h/.c       (pinhole / kb / radial_poly / eftheta)
 *   pipeline 各文件   -> vc_virtual.h/.c      (CameraPose / 网格构建)
 *
 * ## 失败表示
 *   每个可能失败的函数都返回 VCStatus。禁止用 NaN / 空指针 / 静默 0 表示失败
 *   (NFOCC SPEC-ICD §1.2 的同一条约定, 这里照搬)。
 */
#ifndef VCAM_TYPES_H
#define VCAM_TYPES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 基础类型 */
typedef int8_t vc_i8;
typedef uint8_t vc_u8;
typedef int16_t vc_i16;
typedef uint16_t vc_u16;
typedef int32_t vc_i32;
typedef uint32_t vc_u32;
typedef int64_t vc_i64;
typedef uint64_t vc_u64;
typedef float vc_f32;
typedef double vc_f64;

/* 结构体只允许在尾部追加字段, 入口先校验 version */
#define VC_CAMERA_MODEL_VERSION 1u
#define VC_POSE_VERSION 1u
#define VC_VIRTUAL_CONVERSION_VERSION 1u
#define VC_MAP_STATS_VERSION 1u

/* ---------------------------------------------------------------- 状态码 */
typedef enum {
    VC_OK = 0,
    /* --- 输入 --- */
    VC_E_PARAM_NULL = 1,
    VC_E_PARAM_RANGE = 2,
    VC_E_PARAM_VERSION = 3,
    VC_E_UNKNOWN_MODEL = 4,
    /* --- 相机几何 --- */
    VC_E_CAM_NOT_CALIBRATED = 10, /* k1 为 0 / 仿射矩阵奇异 */
    VC_E_CAM_INVERSE_DIVERGED = 11, /* theta 反解不收敛 */
    VC_E_CAM_SINGULAR = 12,         /* e-ftheta 的 det(c-d*e) 近 0 */
    /* --- 外参 --- */
    VC_E_EXTRINSIC_INVALID = 13,
    /* --- 网格 --- */
    VC_E_MAP_EMPTY = 20,     /* 有效像素为 0 */
    VC_E_MAP_NO_OVERLAP = 21,/* 虚拟相机完全看不到源相机视野 */
    VC_E_BUFFER_SMALL = 30,
    VC_E_IMAGE_SHAPE = 31,   /* 图像尺寸与标定不符 */
} VCStatus;

/** 相机模型种类。与 Python 的 model_name 一一对应。 */
typedef enum {
    VC_MODEL_PINHOLE = 0,     /* 针孔 + Brown-Conrady */
    VC_MODEL_KB = 1,          /* Kannala-Brandt */
    VC_MODEL_RADIAL_POLY = 2, /* 径向多项式 rho=k1θ+k2θ²+... (WoodScape) */
    VC_MODEL_EFTHETA = 3      /* 增强型多项式 + 仿射斜切 */
} VCModelKind;

/** 欧拉角约定。 */
typedef enum {
    VC_CONV_VALEO = 0, /* 车体系, 预旋转 X_cam=-Y_w, Y_cam=-Z_w, Z_cam=+X_w */
    VC_CONV_CAMERA = 1 /* 相机本体系 Tait-Bryan, 无预旋转 */
} VCConventionKind;

const char *vc_status_str(VCStatus st);
const char *vc_model_str(VCModelKind m);
const char *vc_convention_str(VCConventionKind c);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* VCAM_TYPES_H */
