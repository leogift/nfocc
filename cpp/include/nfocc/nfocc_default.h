/**
 * @file nfocc_default.h
 * @brief 参数默认值。**唯一来源是 docs/PARAM.md**, 本文件由脚本生成。
 *
 * 生成命令:
 *   python3 tools/param_gen.py --src docs/PARAM.md --out cpp/include/nfocc/nfocc_default.h
 *
 * CI 检查: 重新生成后 git diff 必须为空 (V-BUILD-5)。
 * 手改本文件无效 —— 下次生成会覆盖。
 */
#ifndef NFOCC_DEFAULT_H
#define NFOCC_DEFAULT_H

/* ---------------------------------------------------------------- 相机内参 */
#define NF_INF_F               550.0f   /* KB 焦距 px (用户给定) */
#define NF_INF_CX              960.0f   /* 主点 */
#define NF_INF_CY              768.0f
#define NF_INF_K1              0.0f     /* KB 畸变, 默认等距模型 */
#define NF_INF_K2              0.0f
#define NF_INF_K3              0.0f
#define NF_INF_K4              0.0f
#define NF_INF_R_VALID         960.0f   /* 取长边半宽, 不是短边 768 */
#define NF_INF_MAX_THETA       2.235f   /* 128.1°, 覆盖四角; 必须显式给 */

/* ---------------------------------------------------------------- 图像/视口 */
#define NF_IMG_W               1920
#define NF_IMG_H               1536
#define NF_VF_VIEW             550.0f   /* 视口等效焦距 px/rad (= INF_F, 视口 1:1) */
#define NF_V_REORIENT          1
#define NF_V_REORIENT_EPS      1e-3f
#define NF_V_RING_EL_MIN_DEG   10.0f    /* 视口环带下界 */
#define NF_V_RING_EL_MAX_DEG   60.0f    /* 视口环带上界 */
/**
 * 视口宽度 (px)。水平张角 150° 固定，故 f = (W/2)/tan(75°)。
 *
 * W=1024 -> f=137.2 px/rad，源图角分辨率约 340 px/rad，视口为其 0.40×。
 * 这个取值由精度需求反解确定（见 docs/DECISION-DEPTH-ACCURACY.md）：
 * 5 m 处中位 150 mm 需要 f·b 足够大，1024 已把 f 抬到 640 的 1.6 倍。
 *
 * **再往上加收益递减**：要到源图极限需 W≈2560，计算量约 5 倍，
 * 超出 200 ms 时间片，故停在 1024。
 */
#define NF_V_VIEW_W            1024     /* 左侧视口宽（150°） */
/**
 * **视口宽按类别分**（C 方案，用户决策）。
 *
 * 150° 左右视口：1024 宽 -> f=137.2。
 * 90° 前后视口：557 宽 -> f=278.5，垂直 120° 自然高度
 *   2*278.5*tan(60°) = 965 ≈ 966（正好落在源图高度内）。
 *
 * 判据是"视口像素角分辨率不得细于源图"：
 *   557x966 -> 3.59 mrad/px，源图 340 px/rad = 2.94 mrad
 *   1024x1774 -> 1.95 mrad/px，比源图还细 = 凭空插值
 * 所以前后视口必须**同时**降宽和高，只降高会让横向过细。
 */
#define NF_V_VIEW_W_FRONT      590      /* 前后视口宽（90°），高 1024 由派生算出 */
/**
 * 前后视口尺寸（用户决策，v2）。
 *
 * 590 宽 / 90° -> f = 295/2/tan(45°) = 295.0
 * 该 f 下 120° 纵向自然高度 = 2*295*tan(60°) = 1021.9 ≈ 1024 ✓
 * 像素角分辨率 3.39 mrad，粗于源图 2.94 mrad（不凭空插值）。
 *
 * 相比上一版 557x966：f 278.5 -> 295.0（+5.9%），角分辨率 3.59 -> 3.39 mrad，
 * 视口覆盖 90°x120° 不变。源图 1280x966 仍**不欠采样**：
 * 视口 590x1024 的对角 1179 < 源图对角 1602。
 */
#define NF_V_VIEW_H            258
/* 虚拟视锥水平张角 (deg)。
 *
 * **保持 120°, 不要加大**: f = (W/2)/tan(hfov/2) 与张角成反比。
 *   120° -> f = 277.13 px/rad
 *   150° -> f = 128.62 px/rad  (视差像素数只剩 46%, 角分辨率劣化 115%)
 * 而覆盖上 6 视口方案在 120° 已做到全域 2~3 重, 加宽不增加覆盖,
 * 只损失视差分辨率。实测 150° 时直行+45°偏转的点云从 103 掉到 85。 */
/**
 * 虚拟相机水平张角，**按视口类别分**（用户决策）。
 *
 * 左右视口：侧向大视场，单视口要覆盖车的整侧 -> 150°。
 * 前后视口：每侧两个 ±45° 视口互相重叠，单个只需 90° 即可拼出
 *           180° 的前/后覆盖，且 f 更大（角分辨率更高）。
 *
 * 代价对比（W=1024）:
 *   150° -> f=137.2    90° -> f=512.0
 * 前后视口用 90° 换取 3.7 倍的角分辨率，对 5 m 精度是决定性的。
 */
#define NF_V_VIEW_HFOV_DEG_SIDE    150.0f   /* 左右视口 (ML/MR) */
#define NF_V_VIEW_HFOV_DEG_FRONT   90.0f    /* 前后视口 (FV/RV 的 ±45° 两路) */
/* 兼容旧代码：默认按侧向 */
#define NF_V_VIEW_HFOV_DEG     NF_V_VIEW_HFOV_DEG_SIDE
#define NF_V_VIEW_VFOV_DEG     120.0f      /* 虚拟相机垂直张角 (v0.9.2) */

/* ------------------------------------------------------------------ 车辆 */
#define NF_VEH_LEN             5000.0f  /* mm */
#define NF_VEH_WID             2000.0f
#define NF_VEH_REAR_TO_FRONT   3500.0f
#define NF_POS_CAM_Z           700.0f   /* 相机安装高度 mm */
#define NF_POS_PITCH_DEG       0.0f     /* 安装俯角 deg */

/* ------------------------------------------------------------ 基线状态机 */
/* **B_MIN 由车速下限反推, 不是拍的**:
 *   车速下限 3.6 km/h = 1.000 m/s, 处理频率 5 Hz
 *   => 每帧位移 200 mm, 这就是最短可用基线。
 * 原值 12mm 是旧参数(未考虑车速)下的数, 会得到 <0.1px 的视差, 无法三角化。
 * 基线下限抬到 200mm 后, 深度量化误差显著下降(见 PARAM B.1)。 */
/**
 * 基线下限 = 5 km/h (1.389 m/s) ÷ 5 Hz = 278 mm。
 *
 * 决策依据：支持 5–15 km/h 车速区间（用户定），下限对应最慢车速。
 * 低于此值视差不足，宁可冻结也不出低质量点。
 */
#define NF_B_MIN               278.0f
/* B_MAX 受**引擎容量**约束: d_max = VF_VIEW·b/Z_MIN ≤ numDisparities
 *   b_max = numDisp·Z_MIN/VF_VIEW = 256×500/277.13 = 461.9 mm
 * 取 460 留余量。原 233 是按旧 B_MIN 算的, 已作废。 */
/**
 * 基线上限 = 15 km/h (4.167 m/s) ÷ 5 Hz = 833 mm。
 *
 * 容量校验: numDisp 256、Z_MIN_EFF 500 -> B_MAX ≤ 256*500/137.2 = 933，
 * 833 在容量内。注意 0.5 m 处视差会到 229 px，SGBM 成本显著上升
 * （实测见 docs/DECISION-DEPTH-ACCURACY.md）。
 */
#define NF_B_MAX               833.0f
/* 期望基线: 200~460 的几何中位, 兼顾精度与状态机滞后 */
/* 长基线触发目标：取 5–15 km/h 区间的中段，约 9 km/h */
#define NF_B_TARGET            500.0f
#define NF_B_OMEGA_DEG         12.0f    /* deg/s, 纯旋转判据 */
#define NF_B_DT_MAX_MS         5.0f     /* 位姿与图像允许失步 */
#define NF_B_LAT_MAX_MS        50.0f
#define NF_B_ZMIN_EFF          500.0f   /* mm, 该档最近距离(用户定: 地下 0.5m 起的近场) */

/* ------------------------------------------------------------ 前向门控 */
#define NF_SIGMA_STEREO_MIN    0.80f    /* σ≥此值走立体 */
#define NF_SIGMA_FORWARD_MAX   0.35f    /* σ≤此值走前向解析测距 */

/* ------------------------------------------------------------------ 引擎 */
/* numDisparities: 必须覆盖**最大基线 B_MAX** 下的视差, 不是最小基线。
 *   d_max = VF_VIEW·B_MAX/Z_MIN = 277.13×460/500 = 255.0
 *   -> 256 (16×16), 恰好用满
 *
 * **曾误设为 112**(按 B_MIN=200 算), 会导致 B_MAX=460 的帧被
 * nf_params_validate 的 R2 判据拒绝 —— 那条校验正是为了抓这个错。
 * 若将来把 B_MAX 降到 202mm, 才可以用 112 档, 内存从 127MB 降到 56MB。
 *
 * 内存: 256 × 960 × 258 × 2B = 126.8 MB (单相机; 4 路串行时只用 1 份)。 */
#define NF_E_MAX_DISP          256
#define NF_E_BLOCK             5
#define NF_E_DOWNSCALE         2
#define NF_E_UNIQUENESS        10
#define NF_E_SPECKLE_WINDOW    100

/* ------------------------------------------------------------------ 体素 */
/* 感知范围(用户定案): **以后轴中心为原点**
 *   X: 前后各 9 m     Y: 左右各 7.5 m
 *   Z: 地下 0.5 m ~ 地上 2.5 m
 * 与旧值的差异: X 原 -5~+10m(不对称) -> ±9m(对称); Y 原 ±6m -> ±7.5m;
 * Z 原 -1.0~+2.5m -> -0.5~+2.5m。 */
#define NF_G_VOX               200
#define NF_G_X_MIN             (-9000)
#define NF_G_X_MAX             9000
#define NF_G_Y_MIN             (-7500)
#define NF_G_Y_MAX             7500
#define NF_G_Z_MIN             (-500)
#define NF_G_Z_MAX             2500
#define NF_G_NX                90      /* ceil(18000/200) */
#define NF_G_NY                75      /* ceil(15000/200) */
#define NF_G_NZ                15      /* ceil(3000/200), 整除无半格 */
/* 索引位宽: nx=90, ny=75, nz=15 均 < 128, 原本的 8/8/8 仍然够用,
 * 但 ny=75 接近 8 位上限, 这里显式记下: 若 Y 范围再扩到 ±12.8m(ny=128)
 * 就会溢出, 届时必须改散列。 */
#define NF_G_HASH_BITS_X       8
#define NF_G_HASH_BITS_Y       8
#define NF_G_HASH_BITS_Z       8
/* 哈希表槽数。**必须是 2 的幂** —— 开放寻址用掩码代替取模(& (cap-1))。
   200000 不是 2 的幂, 实测会让 nf_voxel_create 返回 NF_E_PARAM_RANGE,
   进而 pipeline 创建失败。2^18 = 262144 槽, 条目 34B -> 8.9 MB。 */
#define NF_G_HASH_CAPACITY     262144
#define NF_V_N_EFF_MAX         64      /* CMA 有效观测数饱和 */
#define NF_V_HASH_PROBE_MAX    4
/* 点数上限: 网格柱数 90×75=6750, 每柱最多 15 层。
 * 实际有效点受近场可见面限制, 取 32768 覆盖"每柱 4~5 层"的典型情形。 */
#define NF_MAX_POINTS          32768

/* ------------------------------------------------------------ 防御层 */
#define NF_MASK_DILATE         2
/* ==================== 聚类（网格哈希 + DBSCAN） ====================
 *
 * ## 格边长的选择依据
 * 150k 点在 9x7.5x2 m 的 5% 体积 ≈ 6.75 m³，平均点间距 ≈ 36 mm。
 * 格边长取 40mm（略大于平均间距），绝大多数格只有 1-2 点。
 *
 * 曾试过 200mm（与体素网格对齐）：实测 150k 点时单格 148 点，
 * DBSCAN 邻域查询退化成格内暴力，857 ms。**格边长必须按点密度定，
 * 不能跟着体素网格走** —— 两者服务的对象不同：体素要融合，
 * 索引要查询。
 */
/**
 * ## 为什么取 120mm（等于 eps）
 *
 * 实测三种分布下的 DBSCAN 耗时（150k 点）：
 *
 * | 格边长 | 均匀分布 格数/均点 | 均匀 dbscan | 极密团簇 dbscan |
 * |---|---|---|---|
 * | 40mm  | 143956 / 1.0 | 313 ms | 335 ms |
 * | 80mm  | 116261 / 1.3 |  79 ms | 250 ms |
 * | **120mm** | **83017 / 1.8** | **45 ms** | 283 ms |
 *
 * 格 = eps 时，查询半径恰好覆盖 3x3x3=27 个格，是**最小安全覆盖**；
 * 再大就要扫更多格，再小则格数暴涨、哈希开销反超。
 *
 * 极密团簇那列仍慢是**算法本身的性质**：5000 点挤在半径 300mm 的球里时，
 * eps 球内点数上千，任何索引都救不了。真实 150k 点云不会出现这种分布
 * （实测密度对应平均间距 36mm，eps 球内约 161 点）。
 */
#define NF_HASH_CELL_MM         120.0f

#define NF_DBSCAN_EPS_MM        120.0f
#define NF_DBSCAN_MIN_PTS       3u

/* 单帧最多聚类数（超过则合并到"其他"） */
#define NF_MAX_CLUSTERS         256u

/* ==================== 动静分离 ==================== */

/** 判为静止的速度门限 (mm/s)。泊车场景 30mm/s 约等于"没动" */
#define NF_STATIC_SPEED_MMPS    30.0f
/** 聚类级动静投票：动态点占比超过此值则整簇判动态 */
#define NF_DYNAMIC_VOTE_RATIO   0.5f
/** 点级速度估计需要的帧历史长度 */
#define NF_VELOCITY_HISTORY     4u

#define NF_OCC_TAU             1.0f
#define NF_OCC_LAMBDA          14.0f
#define NF_FOE_D_MIN           1.0f
#define NF_FOE_APEX_SIGMA      2.0f

/* -------------------------------------------------------------- 不确定度 */
#define NF_T_SIGMA_D           0.25f    /* px, 视差匹配标准差(待实测替换) */
#define NF_T_SIGMA_B_RATIO     0.02f    /* 基线相对不确定度 */

/* -------------------------------------------------------------- 时间同步 */
#define NF_SYNC_MAX_SKEW_MS    5.0f     /* 四路时间戳极差上限 */
/* 处理频率 5 Hz -> 帧间隔 200 ms。位姿与图像的最大允许失步
 * 必须远小于帧间隔, 取 5ms(= 2.5%)。 */
#define NF_FRAME_PERIOD_MS     200.0f
#define NF_MIN_SPEED_KMH       3.6f
#define NF_GAP_MAX_FRAMES      2

#endif /* NFOCC_DEFAULT_H */
