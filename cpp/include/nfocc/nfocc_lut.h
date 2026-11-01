/**
 * @file nfocc_lut.h
 * @brief 视口 LUT 构建与重采样。规范出处: docs/SPEC-LUT.md
 *
 * 视口 = 一个虚拟针孔相机, 覆盖相机视场中的**俯角环带**(默认 10°–60°)。
 * 不直接用整个视场的原因: f=550/H=1536 时垂直半视场 80°, 全视场会让
 * 下边缘看到 123mm、上边缘看到地平线(视差为零)。
 *
 * 存储: 每栅格点 5 字节 (u:Q12.4 int16, v:Q12.4 int16, valid:u8)。
 * 无效点的 u/v 写 0xFFFF 哨兵 —— **不能用 (0,0)**, 那是合法的左上角像素。
 */
#ifndef NFOCC_LUT_H
#define NFOCC_LUT_H

#include "nfocc_types.h"
#include "nfocc_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 视口参数 (SPEC-ICD §5.2) */
typedef struct {
    nf_u32 version;        /**< = NF_VIEW_PARAMS_VERSION */
    nf_u32 width, height;  /**< 视口栅格尺寸 px */
    nf_f32 f;              /**< 等效焦距 px */
    nf_f32 cx, cy;         /**< 主点 px */
    nf_f32 fov_h, fov_v;   /**< 张角 rad; 必须满足 width/(2f)=tan(fov_h/2) */

    /**
     * **虚拟相机的车系绝对偏航角** (deg, ISO 8855: 0=+X 前, 逆时针为正)。
     *
     * ## 这是虚拟相机，不是"相对真实光轴偏转"
     *
     * 视口按**理想俯仰/横滚**构造: `pitch = 0`, `roll = 0`，
     * 只有 yaw 变化。真实相机的安装角（hanqtech FV 下俯 22.7°、
     * MVL/MVR 下俯 74°）**不参与**视口朝向，只用于把车系射线
     * 反投影到该相机的鱼眼像素。
     *
     * ## 为什么必须这样
     * 若视口朝向跟随真实光轴，不同相机的视口就不共面 ——
     * 同一个方向在天上看是"俯视"、在侧视相机看是"斜视"，
     * 多相机融合时体素无法对齐，也无法与下游的 ego 系 BEV 直接比对。
     * 虚拟相机把所有视口统一到**同一个理想针孔相机模型**下，
     * 只是朝向不同。
     *
     * ## 约定值（6 视口规划）
     *   ML  +90°    MR  −90°
     *   FV  ±45°    RV  ±45°（相对 180°）
     * 完整表见 `nfocc_viewplan.h::nf_viewplan_default()`。
     */
    nf_f32 yaw_deg;

    /**
     * 虚拟相机俯仰角 (deg)。**约定恒为 0**（理想水平光轴）。
     *
     * 保留该字段是为了让"虚拟相机"的完整朝向可表达，
     * 而不是把它隐含在别处。当前实现只接受 0，非 0 会被 validate 拒绝。
     */
    nf_f32 pitch_deg;

    nf_u8  reorient;       /**< 1=按 t 重排视口 (预留, 当前未使用) */
    nf_u8  _pad[3];
} NFViewParams;

/** 无效哨兵 */
#define NF_LUT_SENTINEL 0xFFFFu

/** 视口 LUT。u/v 为 Q12.4 (1/16 px), 范围 ±2048 px。 */
typedef struct {
    nf_i16 *u_q4;     /**< [height*width] */
    nf_i16 *v_q4;     /**< [height*width] */
    nf_u8  *valid;    /**< [height*width] 1=有效 */
    nf_u32  width, height;
    nf_u8   owns_memory;  /**< 1=本结构分配, nf_lut_destroy 负责释放 */
} NFViewLut;

/** LUT 构建统计: 各道有效性淘汰计数, 用于定位配置问题。 */
typedef struct {
    nf_u32 total;
    nf_u32 d1_out_of_image;   /**< 超出鱼眼图像边界 */
    nf_u32 d2_out_of_r_valid; /**< 超出有效半径 */
    nf_u32 d3_over_max_theta; /**< 超出角度上限 */
    nf_u32 d4_body_mask;      /**< 被车身掩码剔除 */
    nf_u32 valid;
} NFLutStats;

/**
 * 反向推导视口参数: 由环带角与水平张角算出 f/cx/cy/height。
 * **height 必须由垂直张角反算**, 独立指定会让实际垂直张角与环带不符
 * (Python 原型实测: height=384 配 f=277 得到 69.6°, 而环带是 50°)。
 *
 * @param width    视口宽 px (>0)
 * @param el_min   环带下界 (绝对俯角, deg)
 * @param el_max   环带上界
 * @param hfov_deg 水平张角
 */
/**
 * 按显式水平/垂直张角派生虚拟视口（平视, pitch=0, roll=0）。
 * 本项目约定 150° x 120°。
 */
NFStatus nf_view_params_derive_fov(NFViewParams *vp, nf_u32 width,
                                   nf_f32 hfov_deg, nf_f32 vfov_deg);

NFStatus nf_view_params_derive(NFViewParams *vp, nf_u32 width,
                               nf_f32 el_min_deg, nf_f32 el_max_deg,
                               nf_f32 hfov_deg);

/** 校验视口自洽性 (V-ICD-5): width/(2f) 与 tan(fov_h/2) 误差 >1e-3 拒绝 */
NFStatus nf_view_params_validate(const NFViewParams *vp);

/** 环带对应的地面距离范围 (mm)。相机俯角不影响 —— 环带是绝对俯角。 */
NFStatus nf_view_ground_range(const NFViewParams *vp, nf_f32 el_min_deg,
                              nf_f32 el_max_deg, nf_f32 cam_z_mm,
                              nf_f32 *near_mm, nf_f32 *far_mm);

/** LUT 分配(不构建)。全部内存在此处一次性分配 (SPEC-BUILD §5)。 */
NFStatus nf_lut_alloc(NFViewLut *lut, nf_u32 width, nf_u32 height);
void     nf_lut_destroy(NFViewLut *lut);

/**
 * 构建 LUT。
 * @param mask 车身掩码 [h*w], 1=车身(剔除); NULL 表示不启用
 * @return NF_OK / NF_E_PARAM_* / NF_E_CAM_DISTORT_INV
 * @note 若全部点都无效会返回 NF_E_PARAM_RANGE 并附带统计 ——
 *       静默全空是最危险的失败模式(Python 原型踩过)。
 */
NFStatus nf_lut_build(NFViewLut *lut, const NFCameraModel *cam,
                      const NFPose *pose, const NFViewParams *vp,
                      const nf_u8 *mask, NFLutStats *stats);

/**
 * 按 LUT 把原图重采样到视口。
 * 双线性, **无效邻域不参与权重**(不按 0 处理 —— 否则边缘会出现假梯度,
 * SGBM 会把它当纹理去匹配)。
 *
 * @param src    [h*w] 单通道 8bit
 * @param out    [vp.height*vp.width] 输出
 * @param out_val[同 out] 输出有效性
 */
NFStatus nf_view_warp(const NFViewLut *lut, const nf_u8 *src,
                      nf_u32 src_w, nf_u32 src_h, nf_u32 src_stride,
                      nf_u8 *out, nf_u8 *out_val);

/** 视口像素 -> 视口系单位射线 (针孔)。供三角化与预测使用。 */
NFStatus nf_view_pixel_to_ray(const NFViewParams *vp, nf_f32 u, nf_f32 v,
                              nf_f32 *ray);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_LUT_H */
