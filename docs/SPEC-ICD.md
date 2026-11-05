# SPEC-ICD — 接口控制文档（结构体 + 函数签名 + 失效语义）

**本文档是所有代码符号的唯一接口定义处。** 其他文档引用类型/函数时只写名字，不重复定义。
命名约定：类型 `NF*` 用 `_t` 后缀的 C 风格；函数 `nf_*`；枚举 `NF_*`；状态码 `NF_E_*`。

坐标系、单位、矩阵约定见 `SPEC-COORD.md`，此处不重复。

---

## 1. 基础类型与状态码

### 1.1 标量

```c
typedef int32_t   nf_i32;    // 长度/坐标, mm 或 px
typedef uint32_t  nf_u32;
typedef uint16_t  nf_u16;
typedef uint8_t   nf_u8;
typedef float     nf_f32;
typedef int64_t   nf_i64;    // 时间戳, 微秒
```

### 1.2 状态码 `NFStatus`

**每一个可能失败的函数必须返回它。禁止用 NaN / 空指针 / 静默 0 表示失败。**

```c
typedef enum {
    NF_OK                      = 0,
    /* --- 输入/配置 --- */
    NF_E_PARAM_NULL            = 1,   // 指针为空
    NF_E_PARAM_RANGE           = 2,   // 数值越界(如 W/H 不符)
    NF_E_PARAM_VERSION         = 3,   // 结构体 version 不匹配
    /* --- 相机/几何 --- */
    NF_E_CAM_NOT_CALIBRATED    = 10,  // 内参缺失
    NF_E_CAM_DISTORT_INV       = 11,  // theta_d 反解不收敛(SPEC-COORD 3.3)
    NF_E_CAM_OUT_OF_VALID      = 12,  // 像素超出有效半径 R_valid
    NF_E_EXTRINSIC_INVALID     = 13,  // 外参自检 C1–C4 失败
    /* --- 基线 --- */
    NF_E_BASELINE_IDLE         = 20,  // 基线不足, 不可输出
    NF_E_BASELINE_OVER         = 21,  // 超上限
    NF_E_BASELINE_ROTATION     = 22,  // 纯旋转, 无平移
    NF_E_BASELINE_DEGRADED     = 23,  // 位姿源降级
    /* --- 引擎 --- */
    NF_E_ENGINE_NOT_INIT       = 30,
    NF_E_ENGINE_RANGE          = 31,  // numDisparities < range.d_max (SPEC-ENGINE S-2)
    NF_E_ENGINE_OOM            = 32,  // 内存不足 (SPEC-ENGINE S-4)
    NF_E_ENGINE_UNSUPPORTED    = 33,  // 平台不支持该 kind
    /* --- 体素/输出 --- */
    NF_E_VOXEL_FULL            = 40,  // 哈希表满(负载因子超限)
    NF_E_OUTPUT_OVERFLOW       = 41,  // 输出点数超过上限
} NFStatus;

const char *nf_status_str(NFStatus s);
```

### 1.3 失效语义总表

每一行对应一个 `NF_E_*`，规定**上层必须怎么反应**。这张表是 `SPEC-BASELINE-SM` 与下游契约的依据。

| 状态码 | 触发条件 | 上层必须的动作 | 下游可观测的后果 |
|---|---|---|---|
| `NF_E_CAM_DISTORT_INV` | Newton 10 次不收敛 | 该 LUT 点标为无效，继续其余点 | 该方向缺少体素 |
| `NF_E_CAM_OUT_OF_VALID` | `r_d·f > R_valid` | 该点标无效 | 同上 |
| `NF_E_EXTRINSIC_INVALID` | 外参自检失败 | **整个相机禁用**，记故障 | 该相机无贡献；`source_camera` 不含该位 |
| `NF_E_BASELINE_*` | 见 `SPEC-BASELINE-SM` §2 | **不调用引擎**，输出"冻结"帧 | `is_valid=0`，保留上一帧体素（可选） |
| `NF_E_ENGINE_RANGE` | `numDisp < d_max` | 编程错误，**不得静默**；返回错误 | 无输出 |
| `NF_E_ENGINE_OOM` | 分配失败 | 降档（downscale×2）重试一次，仍失败则报错 | 可能降低分辨率 |
| `NF_E_VOXEL_FULL` | 负载因子 > 上限 | 丢弃新点，计数上报 | 点云稀疏化 |
| `NF_E_OUTPUT_OVERFLOW` | 点数 > `MAX_POINTS` | 按置信度截断 | 只输出置信度最高的 N 个 |

---

## 2. 图像与视图

### 2.1 `NFImageView`（零拷贝视图）

```c
typedef struct {
    nf_u32 version;        // = NF_IMAGE_VIEW_VERSION
    const nf_u8 *data;     // 单 Y 通道, 8bit
    nf_u32 width;          // 必为 1920
    nf_u32 height;         // 必为 1536
    nf_u32 stride;         // 字节/行, 可 >= width
    const nf_i32 *valid_x0;// 长度 = height; 每行第 0 个有效列; NULL 表示整行 [0,width)
    nf_u64 timestamp_us;
    nf_u32 frame_id;
} NFImageView;
```

| 约束 | 值/说明 |
|---|---|
| `width/height` | **必须** 1920 / 1536，否则 `NF_E_PARAM_RANGE` |
| `valid_x0` | 由视口重排产生（`SPEC-EPIPOLAR` §3.3），允许每行不同 |
| 生命周期 | 调用方持有；引擎不得保存指针超出该次调用 |

### 2.2 `NFCameraModel`（KB 鱼眼内参）

```c
typedef struct {
    nf_u32 version;        // = NF_CAMERA_MODEL_VERSION
    nf_u8  cam_id;         // 0=FV 1=RV 2=ML 3=MR
    nf_f32 f;              // px
    nf_f32 cx, cy;         // px, 允许小数
    nf_f32 k1, k2, k3, k4; // KB 畸变系数
    nf_f32 r_valid;        // 有效半径 px (SPEC-COORD 3.4)
} NFCameraModel;
```

### 2.3 `NFPose`（相机位姿）

```c
typedef struct {
    nf_u32 version;
    nf_u8  cam_id;
    nf_f32 R[9];           // R_cv, 行优先, 满足 v_V = R·v_C  (SPEC-COORD §2)
    nf_f32 t[3];           // 相机光心在 V 系, mm
    nf_u64 timestamp_us;
} NFPose;
```

---

## 3. 基线状态机接口

```c
NFStatus nf_baseline_init(NFBaselineCtx **ctx, const NFBaselineParams *p);
NFStatus nf_baseline_update(NFBaselineCtx *ctx,
                            const NFBaselineInput *in,
                            NFBaselineOutput *out);
void     nf_baseline_reset(NFBaselineCtx *ctx);
```

`NFBaselineInput` / `NFBaselineOutput` 的字段在 `SPEC-BASELINE-SM.md` §3 定义（状态机文档是这两个结构体的唯一定义处）。

---

## 4. 视差引擎

### 4.1 `NFDisparityRange`

```c
typedef struct {
    nf_u32 version;
    nf_f32 d_min;          // px, 通常 0
    nf_f32 d_max;          // px, 由基线 b 与 Z_min 反推
    nf_f32 b_mm;           // 该档基线, 仅用于三角化, 引擎不用
} NFDisparityRange;
```

### 4.2 `NFDisparityMap`

```c
typedef struct {
    nf_u32 version;
    nf_f32 *disp;          // [height*width], 单位 px, 符号见 SPEC-COORD 4.2
    nf_u8  *valid;         // [height*width], 1=有效; 与 d=0 不混淆 (SPEC-ENGINE E-6)
    nf_f32 *conf;          // [height*width], 0..1; SGBM 实现可填 1.0
    nf_u32 width, height;
    nf_i32 stride_f;       // 元素数/行
    nf_i32 stride_b;
} NFDisparityMap;
```

### 4.3 `NFEngineConfig`
```c
typedef enum { NF_ENGINE_SGBM = 0, NF_ENGINE_MODEL = 1 } NFEngineKind;

typedef struct {
    nf_u32 version;
    NFEngineKind kind;
    nf_i32 max_disp_px;    // 必须 >= range.d_max (SPEC-ENGINE S-2)
    nf_i32 downscale;      // 1 | 2 | 4
    nf_f32 conf_thresh;    // 仅 MODEL 用
    /* SGBM 专用, kind==SGBM 时生效 */
    nf_i32 block_size;
    nf_i32 uniqueness;
    nf_i32 speckle_window;
} NFEngineConfig;
```

函数签名见 `SPEC-ENGINE.md` §2.1。

---

## 5. 几何基础类型与视口参数

### 5.1 `NFPoint3f`

```c
typedef struct { nf_f32 x, y, z; } NFPoint3f;     // 单位 mm, 系由使用处声明
```

### 5.2 `NFViewParams`（视口 = 展开图）

视口是一个针孔相机系（`SPEC-COORD` §1.4）。**8 个字段全部是标定/配置量，不在运行时猜。**

```c
typedef struct {
    nf_u32 version;        // = NF_VIEW_PARAMS_VERSION
    nf_u32 width, height;  // 视口栅格尺寸, px (= 引擎输入图尺寸)
    nf_f32 f;              // 视口等效焦距, px/rad  (PARAM P.VIEW_F)
    nf_f32 cx, cy;         // 视口主点, px
    nf_f32 fov_h, fov_v;   // 视口张角, rad; f = (width/2)/tan(fov_h/2) 必须自洽
    nf_u8  reorient;       // 1=按 t 重排视口 (SPEC-EPIPOLAR §3), 0=固定视口
} NFViewParams;
```

| 约束 | 说明 |
|---|---|
| 自洽性 | 必须满足 `width/(2·f) = tan(fov_h/2)`，误差 > 1e-3 返回 `NF_E_PARAM_RANGE` |
| `f` 与 `width` | 降采样时两者同时减半（`SPEC-ENGINE` S-4 的应对） |

### 5.3 其余体素类型见 §6

---

## 6. 体素与输出

### 6.1 `NFVoxelGridParams`

```c
typedef struct {
    nf_u32 version;
    nf_i32 vox_mm;                     // 体素边长
    nf_i32 x_min, x_max;               // mm
    nf_i32 y_min, y_max;
    nf_i32 z_min, z_max;
    nf_u32 nx, ny, nz;                 // 由上面推出, 必须自洽 (SPEC-COORD 5.1)
} NFVoxelGridParams;
```

### 6.2 `NFPoint`

```c
typedef struct {
    nf_i32 x_mm, y_mm, z_mm;   // 体素中心 (SPEC-COORD 5.2)
    nf_u8  source_camera;      // 位图: bit0=FV bit1=RV bit2=ML bit3=MR
    nf_u8  sigma_cm;           // 空间不确定度, 255 = 超限哨兵
    nf_u8  existence_prob;     // 0..100
} NFPoint;                     // 15 字节(3*int32 + 3*uint8), packed 无填充
```

### 6.3 `NFPointCloud`

```c
typedef struct {
    nf_u32 version;
    NFPoint *points;
    nf_u32  num_points;
    nf_u32  max_points;        // 默认 15000
} NFPointCloud;
```

### 6.4 `NFGridOutput`（2D 高程柱）

```c
typedef struct {
    nf_u32 version;
    nf_u32 n_cells;            // = nx*ny
    nf_u32 *zmap;              // [n_cells], bit k = 第 k 层体素存在
    nf_u32 *zindex;            // [n_cells+1], 前缀和 (SPEC-COORD 的 O(1) 寻址)
    nf_u32 *point_offset_dummy;// 保留, 置 NULL
} NFGridOutput;
```

`zmap` 用 `uint32`；`nz ≤ 31`，**bit31 保留为 0**（见 `SPEC-VERIFY.md::V-VOX-3`）。

---

## 7. 函数清单（全部需要实现的接口）

> **2026-11 变更**：原生的 `nf_cam_project` / `nf_cam_unproject`（含 `_c` 变体）
> 已删除，投影与反投影统一走 vcam（`nfocc_vcam_bridge.h`）。
> 只保留模型本身的数学定义 `nf_cam_theta_d` / `nf_cam_rho_to_theta`，
> LUT 构建与自检要用。带外参的版本由调用方先做 `v_cam = Rᵀ(P − t)` 再投。

| 函数 | 输入 | 输出 | 可能返回的失败码 |
|---|---|---|---|
| `nf_vcam_project` | `NFCameraModel`, 相机系 `(x,y,z)` | `(u,v)`, 有效标志 | `NF_E_ENGINE_UNSUPPORTED` |
| `nf_vcam_unproject` | `NFCameraModel`, `(u,v)` | 单位射线 (相机系), 收敛标志 | 同上 |
| `nf_lut_build` | `NFCameraModel`, `NFPose`, 视口参数, `t_dir` | LUT + 有效性 | `NF_E_CAM_DISTORT_INV`, `NF_E_PARAM_RANGE` |
| `nf_view_warp` | 原图 + LUT | `NFImageView`（极线对齐视口图） | `NF_E_PARAM_RANGE` |
| `nf_engine_init/run/deinit` | 见 `SPEC-ENGINE` | `NFDisparityMap` | `NF_E_ENGINE_*` |
| `nf_triangulate` | `NFDisparityMap`, `NFDisparityRange`, 视口参数 | 3D 点 + `sigma_Z` | 无（逐点失效用 `valid`） |
| `nf_voxel_insert` | 3D 点 + `sigma` | 哈希插入/CMA 更新 | `NF_E_VOXEL_FULL` |
| `nf_grid_flatten` | 体素表 | `NFGridOutput` | `NF_E_OUTPUT_OVERFLOW` |
| `nf_baseline_*` | 见 `SPEC-BASELINE-SM` | — | `NF_E_BASELINE_*` |

### 7.1 `nf_triangulate` 精确签名

```c
NFStatus nf_triangulate(const NFDisparityMap   *dm,
                        const NFDisparityRange *range,
                        const NFViewParams     *vp,
                        const NFPose           *pose,
                        NFPoint3f              *out_pts,
                        nf_f32                 *out_sigma_mm,
                        nf_u32                 *out_n);
```

- 逐点：`d = disp[i]`，若 `valid[i]==0` 或 `d < d_min` 或 `d > d_max` → 跳过。
- `Z = VIEW_F · b / d`（`SPEC-EPIPOLAR` 4.1），`X,Y` 同理。
- `sigma_Z` 按 `SPEC-EPIPOLAR` (4.3)，`sigma_d` 取 `PARAM.md::T_SIGMA_D`。
- 输出坐标从视口系经 `pose` 转到 V 系（`SPEC-COORD` 2.1）。

---

## 8. 版本与 ABI

| 结构体 | 版本宏 | 增长策略 |
|---|---|---|
| `NFImageView` | `NF_IMAGE_VIEW_VERSION` | 只追加尾部字段 |
| `NFCameraModel` | `NF_CAMERA_MODEL_VERSION` | 同上 |
| `NFPose` | `NF_POSE_VERSION` | 同上 |
| `NFDisparityMap` | `NF_DISPARITY_MAP_VERSION` | 同上 |
| `NFEngineConfig` | `NF_ENGINE_CONFIG_VERSION` | 同上 |

所有函数入口先校验 `version`，不匹配返回 `NF_E_PARAM_VERSION`。

---

## 9. 本规范的验收

| 编号 | 验收 | 方法 |
|---|---|---|
| V-ICD-1 | 所有公开函数对 NULL 入参返回 `NF_E_PARAM_NULL` 而非崩溃 | 对每个函数注入 NULL |
| V-ICD-2 | 每个 `NF_E_*` 至少有一个测试用例触发 | 覆盖表见 `SPEC-VERIFY.md` 附录 |
| V-ICD-3 | `NFPoint` 尺寸为 **15 字节**（packed 无填充） | `static_assert(sizeof(NFPoint)==15)` |
| V-ICD-4 | 版本不匹配被拒绝 | 传入错误 version |
