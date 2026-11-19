/**
 * @file nfocc_types.h
 * @brief NFOCC 基础类型与状态码。纯 C 头, 可被 C++ 与 C 同时包含。
 *
 * 规范出处: docs/SPEC-ICD.md §1
 * 约定: 类型 NF*_t / 函数 nf_* / 枚举 NF_* / 状态码 NF_E_*
 */
#ifndef NFOCC_TYPES_H
#define NFOCC_TYPES_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 基础类型 */
typedef int8_t    nf_i8;
typedef uint8_t   nf_u8;
typedef int16_t   nf_i16;
typedef uint16_t  nf_u16;
typedef int32_t   nf_i32;
typedef uint32_t  nf_u32;
typedef int64_t   nf_i64;   /* 时间戳, 微秒 */
typedef uint64_t  nf_u64;
typedef float     nf_f32;

/* 版本宏: 结构体只允许在尾部追加字段, 入口先校验 version (SPEC-ICD §8) */
#define NF_IMAGE_VIEW_VERSION      1u
#define NF_CAMERA_MODEL_VERSION    1u
#define NF_POSE_VERSION            1u
#define NF_DISPARITY_MAP_VERSION   1u
#define NF_DISPARITY_RANGE_VERSION 1u
#define NF_ENGINE_CONFIG_VERSION   1u
#define NF_VIEW_PARAMS_VERSION     1u
#define NF_VOXEL_GRID_VERSION      1u
#define NF_POINT_CLOUD_VERSION     1u
#define NF_GRID_OUTPUT_VERSION     1u

/* ---------------------------------------------------------------- 状态码 */
/**
 * 状态码。每一个可能失败的函数都必须返回它。
 * 禁止用 NaN / 空指针 / 静默 0 表示失败 (SPEC-ICD §1.2)。
 */
typedef enum {
    NF_OK                    = 0,
    /* --- 输入/配置 --- */
    NF_E_PARAM_NULL          = 1,   /* 指针为空 */
    NF_E_PARAM_RANGE         = 2,   /* 数值越界 */
    NF_E_PARAM_VERSION       = 3,   /* 结构体 version 不匹配 */
    /* --- 相机/几何 --- */
    NF_E_CAM_NOT_CALIBRATED  = 10,  /* 内参缺失 */
    NF_E_CAM_DISTORT_INV     = 11,  /* theta_d 反解不收敛 */
    NF_E_CAM_OUT_OF_VALID    = 12,  /* 像素超出有效半径 */
    NF_E_EXTRINSIC_INVALID   = 13,  /* 外参自检失败 */
    /* --- 基线 --- */
    NF_E_BASELINE_IDLE       = 20,  /* 基线不足 */
    NF_E_BASELINE_OVER       = 21,  /* 超上限 */
    NF_E_BASELINE_ROTATION   = 22,  /* 纯旋转 */
    NF_E_BASELINE_DEGRADED   = 23,  /* 位姿源降级 */
    /* --- 引擎 --- */
    NF_E_ENGINE_NOT_INIT     = 30,
    NF_E_ENGINE_RANGE        = 31,  /* numDisparities < d_max */
    NF_E_ENGINE_OOM          = 32,
    NF_E_ENGINE_UNSUPPORTED  = 33,
    /* --- 体素/输出 --- */
    NF_E_VOXEL_FULL          = 40,
    NF_E_OUTPUT_OVERFLOW     = 41,

    NF_E_IO                  = 50,  /* 文件/JSON 读写或解析失败 */
    NF_E_LUT_EMPTY           = 51   /* 视口全部落在相机视场外（虚拟相机正常工况） */
} NFStatus;

/** 状态码转字符串。永不返回 NULL。 */
const char *nf_status_str(NFStatus s);

/** 日志级别 (SPEC-BUILD §6) */
typedef enum {
    NF_LOG_ERROR = 0,
    NF_LOG_WARN  = 1,
    NF_LOG_INFO  = 2,
    NF_LOG_DEBUG = 3
} NFLogLevel;

/** 日志回调。text 生命周期仅限调用期, 实现方不得保存指针。 */
typedef void (*NFLogFn)(NFLogLevel level, const char *text, void *user);

/** 全局日志设置。fn 为 NULL 时不输出。 */
void nf_log_set(NFLogFn fn, void *user);
NFLogFn nf_log_get(void);
void *nf_log_get_user(void);
/** 按级别格式化输出(内部用 snprintf, 可被实时路径关闭)。 */
void nf_log_msg(NFLogLevel level, const char *fmt, ...);

/* ------------------------------------------------------------- 几何基础 */
/** 3D 点, 单位 mm。所属坐标系由使用处声明。 */
typedef struct { nf_f32 x, y, z; } NFPoint3f;

/** 体素点, 必须 9 字节无填充 (V-ICD-3 要求 sizeof == 9)。
 *
 * **packed 的放置位置很关键**, 实测踩了两次:
 *   1. `typedef struct {...} __attribute__((packed)) NFPoint;`
 *      —— 对匿名 struct 不生效, sizeof 仍是 16。
 *   2. `typedef struct __attribute__((packed)) {...} NFPoint;`
 *      —— clang 接受该语法但**只作用于后面的成员声明**,
 *         结果偏移被压紧(x=0,y=4,z=8,src=12,sig=13,prob=14)
 *         而 sizeof 仍是 15, 比不加还难排查。
 *   正确写法: 属性放在标签位(__attribute__ 后紧跟 {), 见下方 NF_PACK_TAG。
 */
#if defined(_MSC_VER)
#  pragma pack(push, 1)
#  define NF_PACK_TAG
#  define NF_PACK_TAIL
#else
#  define NF_PACK_TAG
#  define NF_PACK_TAIL __attribute__((packed))
#endif
typedef struct NF_PACK_TAG {
    nf_i32 x_mm, y_mm, z_mm;   /* 体素中心 */
    nf_u8  source_camera;      /* bit0=FV bit1=RV bit2=ML bit3=MR */
    nf_u8  sigma_cm;           /* 空间不确定度, 255 = 超限哨兵 */
    nf_u8  existence_prob;     /* 0..100 */
} NF_PACK_TAIL NFPoint;
#if defined(_MSC_VER)
#  pragma pack(pop)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_TYPES_H */
