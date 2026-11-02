# SPEC-POSE — 位姿接入规范（轮速/IMU → NFPose）

> **状态：降级方案已实现** —— `nfocc_odom`（speed + yaw_rate 积分）。
> **无 IMU 通路**：接口留了 `nf_odom_push_pose` 供外部注入真位姿。
> 打滑时轮速不可信，产品必须接真实定位。

补齐审计缺口 **G-6**。本文档定义：位姿从哪来、怎么变成 `NFPose`、误差怎么进基线。
坐标系与矩阵约定见 `SPEC-COORD.md`，状态机见 `SPEC-BASELINE-SM.md`。

---

## 1. 为什么这一层必须先定义

运动双目的**每一个深度值都正比于基线**，而基线 = 两帧位姿之差：

$$Z = \frac{f\,b}{d},\qquad b = \lVert t_2 - t_1 \rVert$$

所以位姿误差按 **1:1 进入深度相对误差**：基线差 2%，深度就差 2%。
原 NFOCC 用 SfM 自己估位姿，本项目**依赖外部位姿源**，那么"外部位姿怎么变成
`t_2 - t_1`"就是精度的第一道关。

现状：`docs/` 定义了 `NFPose` 结构体，却没定义它的来源。这是审计里最致命的一条——
工程师会各自写一套积分，误差特性完全不同且无法比较。

---

## 2. 位姿源的三种形态（必须显式声明是哪一种）

```c
typedef enum {
    NF_POSE_SRC_EGO_MOTION = 0,  // 域控给的融合位姿(视觉/轮速/IMU 融合), 最准
    NF_POSE_SRC_WHEEL_IMU  = 1,  // 轮速脉冲 + 陀螺/IMU, 本模块自己积分
    NF_POSE_SRC_WHEEL_ONLY = 2,  // 只有轮速, 无横摆角速度(退化)
} NFPoseSource;
```

| 来源 | 能给的量 | 缺失的量 | 近场可用性 |
|---|---|---|---|
| `EGO_MOTION` | 完整 6DoF | — | 首选 |
| `WHEEL_IMU` | 纵向位移 + 横摆角 | 俯仰/侧倾、z | 可接受（泊车平面运动） |
| `WHEEL_ONLY` | 纵向位移 | 横摆角 | **不可用于运动双目**（转弯时基线方向错） |

**规则**：`WHEEL_ONLY` 时基线状态机必须常驻 `DEGRADED`（`SPEC-BASELINE-SM` T-5），
不输出体素。理由：没有横摆角就无法知道基线方向，`t_rel` 全错。

---

## 3. 平面运动假设（必须写明，不能隐含）

泊车工况下采用平面运动假设：

| 量 | 假设 | 违反时的后果 |
|---|---|---|
| 俯仰/侧倾 | 0（车辆在水平面） | 坡度会让基线有 z 分量，见 §5 |
| 高度 z | 常数 | 相机高度变化直接错进体素高程 |
| 运动 | 平面内 (x, y, yaw) | 轮胎打滑时位移估计偏 |

**这条假设必须写进代码注释与 `scene.json`**，因为一旦上了坡道，所有体素的 z 都系统性偏移。

---

## 4. 从轮速/IMU 到位姿的积分链（唯一实现流程）

### 4.1 输入

```c
typedef struct {
    nf_u32 version;
    nf_u64 ts_us;
    nf_f32 wheel_dist_mm[4];   // 四轮累计里程, mm (左前/右前/左后/右后)
    nf_f32 yaw_rate_rps;       // 横摆角速度, rad/s; 无效时置 NAN
    nf_u8  yaw_rate_valid;
    nf_f32 v_mps;              // 纵向速度, m/s; 无效时置 NAN
    nf_u8  v_valid;
} NFChassisState;
```

### 4.2 积分步骤（逐步，顺序不可调换）

```
Step 1  时间差:  dt = (t_k − t_{k−1}) / 1e6    [s]
         若 dt ≤ 0 或 dt > B_DT_MAX_GAP → 置 DEGRADED, 丢弃该帧

Step 2  纵向位移: ds = (Δwheel_L + Δwheel_R) / 2      [mm]
         左/右分别取前后轴平均: L = (ΔLF+ΔLR)/2, R = (ΔRF+ΔRR)/2

Step 3  横摆增量: dψ = yaw_rate · dt                  [rad]
         若 yaw_rate 无效: dψ 由左右轮差反解 dψ = (R − L) / track_width

Step 4  中值积分(关键: 用中点航向, 不用端点)
         ψ_mid = ψ_{k−1} + dψ/2
         dx = ds · cos(ψ_mid)
         dy = ds · sin(ψ_mid)
         ψ_k = ψ_{k−1} + dψ

Step 5  位置: X_k = X_{k−1} + dx ,  Y_k = Y_{k−1} + dy ,  Z_k = Z_{k−1}
```

**为什么必须中值积分**：用端点航向（`ψ_{k-1}`）在转弯时会系统性低估横向位移，
曲率越大误差越大。泊车恰好是低曲率半径工况，这个误差会被放大。

### 4.3 尺度因子（必须显式，不可默认 1.0）

轮速脉冲当量受胎压、载荷、轮胎磨损影响。定义：

```
ds_corrected = ds · scale_factor          (4.1)
```

`scale_factor` 初值 1.0，由 `SPEC-BASELINE-SM` §7 的尺度自检在线校正。
**出厂标定值必须写入参数表**（`PARAM.md` 的 `POS.SCALE`），不能靠默认值跑。

---

## 5. 车辆位姿 → 相机位姿（`NFPose` 的唯一生成方式）

`NFPose` 需要的是**相机光心在 V 系的位姿**，而位姿源给的是**车辆位姿**。变换链：

$$R_{cv} = R_{vc}^{\top} \qquad t_{c} = \big(R_{vc}^{\top}\big)\,(t_{v} - t_{vc})$$

其中：
- `R_vc`、`t_vc` 是**标定外参**（相机相对车辆，`SPEC-COORD` §2 约定）
- `t_v` 是车辆位姿（后轴中心）

展开成可直接编码的形式：

```
R_w = Rz(ψ_k) · Ry(θ) · Rx(φ)                 # 车辆在世界系, 平面假设下 θ=φ=0
R_cv_k = R_w^T · R_cv_0                        # 当前帧相机旋转(SPEC-COORD §2 的约定)
t_c_k  = R_w^T · (t_c0 − t_w_k)               # 注意: R^T 作用在差值上
```

**`R_cv_0` / `t_c0` 是"零位"相机位姿**（车辆位于原点时的相机位姿），由标定给出。

> 这里是本项目最容易写错的地方：`t_c_k = R_w^T · (t_c0 − t_w_k)` 里的 `R_w^T` 是
> "把世界系差值转回车辆系"，**不是** `R_w · (...)`。写反会导致转弯时相机位置转错方向，
> 而直行时完全看不出来（`R_w = I`）。**验收必须包含转弯场景（V-POSE-4）。**

### 5.1 相对位姿（状态机真正用的量）

`SPEC-BASELINE-SM` 需要的是参考帧到当前帧的相对位姿。由 `NFPose` 直接算：

```
R_rel = R_cv_ref^T · R_cv_cur                 (5.1)
t_rel = R_cv_ref^T · (t_c_cur − t_c_ref)      (5.2)
b     = ‖t_c_cur − t_c_ref‖                   (5.3)
```

**注意 `b` 用的是 V 系下的差值的模长**，与 (5.2) 的旋转无关——这不是巧合，
因为旋转不改变长度。但如果 `t_rel` 与 `b` 从不同地方算，会出现"状态机认为基线够、
而 `t_rel` 模长对不上"的内部矛盾。**必须由同一个差值算出来。**

---

## 6. 误差模型（进 σ_Z 的那一项）

`SPEC-EPIPOLAR` (4.3) 里的 `σ_b` 由这里定义：

$$\sigma_b^2 = \underbrace{(\Delta t\cdot\sigma_v)^2}_{\text{速度噪声}} +
\underbrace{(b\cdot\sigma_{\text{scale}})^2}_{\text{尺度}} +
\underbrace{(\sigma_{\text{quant}})^2}_{\text{轮速脉冲量化}}$$

| 项 | 默认 | 来源 |
|---|---|---|
| `σ_v` | 0.02 m/s | 轮速噪声（`PARAM.md::T_SIGMA_B` 的 2% 由此推） |
| `σ_scale` | 0.02 | 尺度因子不确定度 |
| `σ_quant` | 1 个脉冲当量 / 2 | 轮速脉冲计数量化 |

**分档规则**：`EGO_MOTION` 源时 `σ_b` 取融合输出的协方差（若有），否则按上式；
`WHEEL_IMU` 源时上式满额；`WHEEL_ONLY` 时不计算（状态机常驻 DEGRADED）。

---

## 7. 失效与降级（与 `SPEC-ICD` 的错误码对应）

| 条件 | 状态码 | 动作 |
|---|---|---|
| `dt` 超限 | `NF_E_BASELINE_DEGRADED` | 丢弃该帧，参考帧重置 |
| `yaw_rate` 无效且轮差不可用 | `NF_E_BASELINE_DEGRADED` | 冻结 |
| 位姿源 = `WHEEL_ONLY` | `NF_E_BASELINE_DEGRADED` | 常驻冻结 |
| 积分漂移超 `POS.DRIFT_MAX` | `NF_E_BASELINE_DEGRADED` + 重置 | 防累积发散 |
| 相机位姿自检失败（`R_cv` 非正交） | `NF_E_EXTRINSIC_INVALID` | 该相机禁用 |

---

## 8. 验收

| 编号 | 级别 | 判据 | 负例（必须失败） |
|---|---|---|---|
| V-POSE-1 | L0 | 静止 1000 帧，位置漂移 < 1 mm | 注入零偏 → 漂移超限 |
| V-POSE-2 | L0 | 直线 10 m，终点位置误差 < 0.1% | 把中值积分改端点积分 → 转弯用例超限 |
| V-POSE-3 | L0 | 90° 转弯，终点位置误差 < 1% | — |
| V-POSE-4 | **L0** | **转弯工况下 `t_c_k` 方向正确**（与解析值比 < 1 mm） | **把 `R_w^T` 写成 `R_w` → 直行仍过、转弯必错** |
| V-POSE-5 | L0 | `t_rel` 模长 == `b`（同一差值算，误差 < 1e-6） | 两者分别计算 → 不一致 |
| V-POSE-6 | L0 | `WHEEL_ONLY` 源 → 状态机常驻 DEGRADED | 去掉该判断 → 错位地基线被采用 |

**V-POSE-4 是本文档最重要的一条验收**：它专门抓 §5 的转置错误。
若只测直行，这个 bug 可以一路带到实车。
