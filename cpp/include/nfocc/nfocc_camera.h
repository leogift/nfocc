/**
 * @file nfocc_camera.h
 * @brief KB 鱼眼相机模型、位姿、正/反投影。
 *
 * 规范出处: docs/SPEC-ICD.md §2.2/§2.3, docs/SPEC-COORD.md §2/§3
 *
 * ## 坐标约定 (全项目唯一来源, ADR-007)
 *   NFPose::R 满足  v_V = R · v_C     (行优先存储的 3×3)
 *   因此投影:       p_C = Rᵀ · (P_V − t)
 *
 *   **要点**: 由 v_V = R·v_C 可知, **R 的第 k 列**才是相机第 k 轴在车系中的方向。
 *   例: 相机 z 轴(光轴)在车系 = R·(0,0,1) = R 的第 3 列。
 *   (曾误注为"行是相机轴" —— 那是 v_C = R·v_V 的约定, 与 ICD 相反。)
 *
 *   **与 Python 原型的 R_cv(p_cam = R·(P−C)) 互为转置**,
 *   转换只在 pose_util 里做一次。
 *
 * ## KB 模型
 *   theta   = atan2(sqrt(x²+y²), z)
 *   theta_d = theta·(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)
 *   u = cx + f·theta_d·x/chi,  v = cy + f·theta_d·y/chi
 *
 * 有效性判据用 theta 与半径, **不用 z>0** —— 鱼眼 theta 可 >90°,
 * 此时 z<0 但图像上仍有有效像素。
 */
#ifndef NFOCC_CAMERA_H
#define NFOCC_CAMERA_H

#include "nfocc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** KB 鱼眼内参 (SPEC-ICD §2.2) */
/**
 * 鱼眼投影模型类型。
 *
 * ## 为什么需要两种
 * 两个模型的数学形式不同, **不能用同一组字段糊弄**:
 *
 * KB (本项目的理想基线, f=550):
 *      r = f · [θ + k1·θ³ + k2·θ⁵ + k3·θ⁷ + k4·θ⁹]
 *      └ 只含奇次项, f 单独给出
 *
 * RADIAL_POLY (hanqtech 标定):
 *      ρ = k1·θ + k2·θ² + k3·θ³ + k4·θ⁴
 *      └ 含偶次项, **k1 本身就是焦距(px/rad)**, 没有单独的 f
 *
 * 实测 hanqtech FV: k1=339.75, k2=-31.99, k3=48.28, k4=-7.20。
 * 若强行套 KB 公式 (k1=339 当畸变系数), 投影会在 θ>0.2rad 处完全失真。
 */
typedef enum {
    NF_CAM_MODEL_KB = 0,            /**< Kannala-Brandt (奇次多项式) */
    NF_CAM_MODEL_RADIAL_POLY = 1    /**< hanqtech 四次多项式 ρ(θ) */
} NFCameraModelKind;

typedef struct {
    nf_u32 version;        /**< = NF_CAMERA_MODEL_VERSION */
    NFCameraModelKind kind;/**< 投影模型; 决定 k1..k4 的语义 */
    nf_u8  cam_id;         /**< 0=FV 1=RV 2=ML 3=MR */
    nf_f32 f;              /**< 焦距, px。**仅 KB 用**; RADIAL_POLY 时忽略 */
    nf_f32 cx, cy;         /**< 主点, px (允许小数) */
    nf_f32 k1, k2, k3, k4; /**< KB: 畸变系数; RADIAL_POLY: ρ(θ) 的四次系数 */
    nf_f32 r_valid;        /**< 有效半径, px */
    nf_f32 max_theta;      /**< 角度上限, rad。**必须显式给, 不用库默认值**
                            *   本项目 f=550/1920x1536 时为 2.235 rad (128.1°)。
                            *   抄 OpenCV/第三方的 1.65 rad 会静默砍掉 5.5° 视场。 */
} NFCameraModel;

/** 相机位姿 (SPEC-ICD §2.3) */
typedef struct {
    nf_u32 version;        /**< = NF_POSE_VERSION */
    nf_u8  cam_id;
    nf_f32 R[9];           /**< 行优先 3x3, 满足 v_V = R·v_C(列=相机轴在车系) */
    nf_f32 t[3];           /**< 光心在 V 系, mm */
    nf_u64 timestamp_us;
} NFPose;

/* ----------------------------------------------------------- 内参自检 */
/**
 * 校验内参合法性: f>0、r_valid∈(0,r_corner]、max_theta 覆盖四角、
 * theta_d 在 [0, max_theta] 上单调 (V-CO-6)。
 * @return NF_OK / NF_E_PARAM_NULL / NF_E_PARAM_RANGE / NF_E_CAM_NOT_CALIBRATED
 */
NFStatus nf_cam_validate(const NFCameraModel *cam);

/** 外参自检 C1–C4: R 正交 (‖RRᵀ−I‖<1e-9)、det(R)=1、y 轴朝下、光轴朝外。
 *  @return NF_OK / NF_E_EXTRINSIC_INVALID */
NFStatus nf_pose_validate(const NFPose *pose);

/* -------------------------------------------------------------- 投影 */
/**
 * 3D 点 (V 系, mm) -> 像素。
 * @param uv  输出 [2]; 无效时写 NaN
 * @param valid 输出 [1]; 1=有效
 * @return NF_OK 或错误码。**超出 r_valid/max_theta 不算错误**, 只置 valid=0
 *         (逐点失效用掩码, 不用状态码 —— SPEC-ICD §7 的 nf_triangulate 语义)。
 */
NFStatus nf_cam_project(const NFCameraModel *cam, const NFPose *pose,
                        const NFPoint3f *P_V,
                        nf_f32 *uv, nf_u8 *valid);

/**
 * 像素 -> 单位射线 (V 系)。
 * @param ray 输出 [3], 单位向量
 * @return NF_E_CAM_DISTORT_INV 表示 Newton 不收敛
 */
NFStatus nf_cam_unproject(const NFCameraModel *cam, const NFPose *pose,
                          nf_f32 u, nf_f32 v,
                          nf_f32 *ray);

/* ------------------------------------------------- 内部: 相机系下的一步 */
/** 相机系 3D -> 像素 (不含外参) */
NFStatus nf_cam_project_c(const NFCameraModel *cam,
                          nf_f32 x, nf_f32 y, nf_f32 z,
                          nf_f32 *u, nf_f32 *v, nf_u8 *valid);
/** 像素 -> 相机系单位射线 (不含外参) */
NFStatus nf_cam_unproject_c(const NFCameraModel *cam,
                            nf_f32 u, nf_f32 v,
                            nf_f32 *ray);

/** theta_d(theta) 及其导数, 供 LUT 构建与自检使用 */
nf_f32 nf_cam_theta_d(const NFCameraModel *cam, nf_f32 theta);
nf_f32 nf_cam_theta_d_prime(const NFCameraModel *cam, nf_f32 theta);
/** 由 rho = theta_d 反解 theta。不收敛返回负值并置 *ok=0。 */
nf_f32 nf_cam_rho_to_theta(const NFCameraModel *cam, nf_f32 rho, nf_i32 *ok);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NFOCC_CAMERA_H */
