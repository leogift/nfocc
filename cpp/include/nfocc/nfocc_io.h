/**
 * @file nfocc_io.h
 * @brief 上游输入与下游输出的**线上格式**结构体。
 *
 * 规范出处: docs/SPEC-UPSTREAM.md（输入）、docs/SPEC-DOWNSTREAM.md（输出）
 *
 * ## 重要：这里的结构体是**线上格式**，不是内存映射格式
 * 跨进程传输必须按小端、显式偏移解析，**不能**直接 `memcpy` 到本地 struct
 * 再 cast —— 不同编译器/版本的填充规则可能不同。
 * 本文件用 packed + 显式顺序把布局钉死，并提供 static_assert 校验尺寸。
 *
 * ## 与 nfocc_types.h 的关系
 * `NFPoint` / `NFFrameInput` 是**内部**结构(内存布局, 供算法直接访问)。
 * 本文件的 `NFUpStreamHeader` / `NFOutHeader` 是**跨进程**结构(线上格式)。
 * 两者不要混用。
 */
#ifndef NFOCC_IO_H
#define NFOCC_IO_H

#include "nfocc/nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NF_UP_MAGIC   0x4E465550u   /* "NFUP" */
#define NF_OUT_MAGIC  0x4E464F55u   /* "NFOU" */
#define NF_UP_VERSION   1u
#define NF_OUT_VERSION  1u

/* ==================================================================
 * 上游（相机 / ego motion 发送方 -> 本系统）
 * ================================================================== */

/** 位姿 / ego motion。SPEC-UPSTREAM §2.2 */
#if defined(_MSC_VER)
#  pragma pack(push, 1)
#  define NF_PACKED_IO
#else
#  define NF_PACKED_IO __attribute__((packed))
#endif

typedef struct NF_PACKED_IO {
    nf_u32 version;       /**< = NF_UP_VERSION 内嵌版本(与头部同步) */
    nf_u64 timestamp_us;  /**< 与图像同一时钟源, **单调时钟**非墙钟 */
    nf_f32 pos_mm[3];     /**< 后轴中心位置, x 前 y 左 z 上 (ISO 8855) */
    nf_f32 quat[4];       /**< 单位四元数 w,x,y,z (避免欧拉角万向锁) */
    nf_f32 vel_mmps[3];   /**< 后轴中心速度, mm/s */
    nf_f32 yaw_rate_dps;  /**< 横摆角速度, deg/s */
    nf_u8  source;        /**< 0=轮速 1=IMU 2=视觉 3=融合 */
    nf_u8  quality;       /**< 0..100, <50 视为降级 */
    nf_u8  reserved[2];
} NFUpEgoMotion;

/**
 * 上游消息头（multipart 的段 0 起始部分）。
 * 之后紧跟 n_cameras 段"NFUpImageHeader + 像素"，每段一条 ZMQ 消息。
 */
typedef struct NF_PACKED_IO {
    nf_u32 magic;         /**< = NF_UP_MAGIC */
    nf_u32 version;       /**< = NF_UP_VERSION */
    nf_u32 frame_id;      /**< 单调递增, 丢帧时跳号 */
    nf_u64 timestamp_us;  /**< 曝光中点(单调时钟), 四路共用 */
    nf_u32 n_cameras;     /**< = 4 */
    nf_u32 reserved;
    NFUpEgoMotion ego;    /**< 本帧位姿 */
} NFUpHeader;

/** 单个相机的图像头（multipart 段 1..4 的起始部分）。 */
typedef struct NF_PACKED_IO {
    nf_u8  cam_id;        /**< 0=FV 1=RV 2=ML 3=MR */
    nf_u8  format;        /**< 0=GRAY8（唯一支持） */
    nf_u16 reserved;
    nf_u32 width;         /**< 必为 1920 */
    nf_u32 height;        /**< 必为 1536 */
    nf_u32 stride;        /**< 字节/行, 可 >= width */
    nf_u64 timestamp_us;  /**< 该相机自己的曝光时刻, 用于同步检查 */
    nf_u32 frame_id;
    nf_u32 data_bytes;    /**< **必须** == stride*height, 否则拒绝 */
} NFUpImageHeader;

/* ==================================================================
 * 下游（本系统 -> 规划/可视化）
 * ================================================================== */

/**
 * 下游消息头（multipart 段 0）。
 *
 * 设计要点:
 *  - timestamp_us 是**上游曝光时刻**, 不是处理完成时刻。下游做时空对齐用前者;
 *    完成时刻另由 latency_us 表达。混用会引入一整个帧周期的系统性偏差。
 *  - 冻结时本头仍完整发出, is_valid=0 且 status 说明原因。
 *    **不能不发消息** —— 下游无法区分"没障碍"和"系统挂了"。
 */
typedef struct NF_PACKED_IO {
    nf_u32 magic;           /**< = NF_OUT_MAGIC */
    nf_u32 version;         /**< = NF_OUT_VERSION */
    nf_u32 frame_id;        /**< 回抄上游 frame_id, 便于对账 */
    nf_u64 timestamp_us;    /**< 上游曝光时刻(单调时钟) */
    nf_u64 latency_us;      /**< 曝光 -> 本消息发出的耗时 */
    nf_u8  is_valid;        /**< 1=结果可信; 0=冻结/降级 */
    nf_u8  status;          /**< NFStatus 低 8 位, is_valid=0 时说明原因 */
    nf_u8  mode;            /**< NFMode: 0=STEREO 1=FORWARD 2=HYBRID 3..5=FROZEN */
    nf_u8  num_cameras_ok;  /**< 参与本帧的有效相机数 0..4 */
    nf_u32 num_points;
    nf_u32 zmap_cells;      /**< nx*ny; 0 表示不带高程 */
    nf_u32 zmap_bytes;      /**< = zmap_cells*4 */
    nf_u32 zindex_bytes;    /**< = (zmap_cells+1)*4 */
    nf_u32 points_bytes;    /**< = num_points*sizeof(NFPoint) */
    nf_f32 baseline_mm;     /**< 本帧实际基线, 下游据此判断精度 */
    nf_f32 sigma_med_mm;    /**< σ 中位数, 下游据此设膨胀半径 */
    nf_u32 reserved;
} NFOutHeader;

#if defined(_MSC_VER)
#  pragma pack(pop)
#endif

/* ---- 布局校验: 尺寸错了跨进程解析必然错乱, 编译期就该拦住 ---- */
#ifdef __cplusplus
static_assert(sizeof(NFPoint) == 15, "NFPoint 线上格式必须 15 字节");
static_assert(sizeof(NFOutHeader) == 64, "NFOutHeader 尺寸变了, 需同步下游");
static_assert(sizeof(NFUpEgoMotion) == 60, "NFUpEgoMotion 尺寸变了");
static_assert(sizeof(NFUpHeader) == 88, "NFUpHeader 尺寸变了");
#endif

/* ==================================================================
 * 收发函数
 * ================================================================== */

#define NF_MAX_CAMERAS 4

/** 上游一帧的解包结果（指向消息缓冲内部, 不拥有内存） */
typedef struct {
    NFUpHeader       header;
    nf_u8            cam_id[NF_MAX_CAMERAS];
    const nf_u8     *pixels[NF_MAX_CAMERAS];   /* 指向各段像素起始 */
    nf_u32           width, height;            /* 从首段图像头抄出, 4 路必须一致 */
} NFUpFrameView;

/**
 * 解析上游 multipart 消息。
 * @param frames  每路一段(段 1..n_cameras)
 * @param n_frames 段数
 * @return NF_E_PARAM_RANGE 表示格式不符(尺寸/字节数/相机数)
 */
NFStatus nf_up_parse(const void *head, nf_u32 head_bytes,
                     const void *const *frames, const nf_u32 *frame_bytes,
                     nf_u32 n_frames, NFUpFrameView *out);

/** 序列化下游消息：返回需要的总字节数(供调用方分配)。
 *  layout: [NFOutHeader][NFPoint * n][zmap][zindex] */
NFStatus nf_out_size(const NFOutHeader *h, nf_u32 *out_bytes);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_IO_H */
