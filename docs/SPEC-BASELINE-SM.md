# SPEC-BASELINE-SM — 基线状态机（状态表 + 迁移 + 输出契约）

本文档是 `NFBaselineInput` / `NFBaselineOutput` 的**唯一定义处**，也是状态机的唯一规范。
视差量级与分档的算术依据见 `SPEC-EPIPOLAR` §4.2；参数值见 `PARAM.md` 的 B 节（`B_MIN`/`B_MAX`/`B_TARGET`/`B_OMEGA`/`B_DT_MAX`/`B_LAT_MAX`/`B_ZMIN_EFF`）。

---

## 1. 为什么需要状态机

外部位姿基线下，基线由"位移"产生。三种情况下基线**在物理上不存在或不可信**：

1. **静止**：`b → 0`，三角化深度发散（`Z = f·b/d`，`b→0` 则 `Z→0` 或无穷）。
2. **纯旋转**（原地打方向）：角速度大、平移近零，平移方向无定义，极线方向无定义。
3. **超基线**：位移过大，两帧共视区不足，匹配失效；且视差可能超过引擎容量。

外加位姿质量下降（陀螺/轮速失步、尺度漂移）。这四种情况必须**显式判定并冻结输出**，
而不是让下游拿到"看起来像体素但其实是噪声"的东西。

---

## 2. 状态定义与控制表

### 2.1 状态集合

```c
typedef enum {
    NF_BS_IDLE          = 0,   // 基线未建立(静止或位移不足)
    NF_BS_VALID         = 1,   // 基线可用, 正常输出
    NF_BS_ROTATION_ONLY = 2,   // 纯旋转: 无平移, 冻结
    NF_BS_DEGRADED      = 3,   // 位姿源质量下降: 冻结
    NF_BS_OVER_BASELINE = 4,   // 超上限: 重置基线
} NFBaselineState;
```

### 2.2 状态表（每个状态的输出契约）

| 状态 | `is_valid` | `b_mm` | `t_rel` | `R_rel` | 允许调用引擎 | 下游语义 |
|---|---|---|---|---|---|---|
| `NF_BS_IDLE` | 0 | 0 | 未定义 | 未定义 | **否** | 无新观测，保留上一帧 |
| `NF_BS_VALID` | 1 | 当前 `b` | 有效 | 有效 | **是** | 正常帧 |
| `NF_BS_ROTATION_ONLY` | 0 | 0 | (0,0,0) | 有效 | **否** | 冻结；`R_rel` 仍可用于补偿 |
| `NF_BS_DEGRADED` | 0 | 0 | 未定义 | 未定义 | **否** | 冻结 + 故障上报 |
| `NF_BS_OVER_BASELINE` | 0 | 0 | 未定义 | 未定义 | **否** | 重置参考帧 |

**"允许调用引擎"这一列是硬约束**：状态非 `VALID` 时，管线必须跳过 `nf_engine_run`，
不得为了"输出点云"而强行调用（会产生系统性错误体素，比无输出更坏）。

### 2.3 状态机图

```
                 ┌──────────────────────────────────────────┐
                 │                                          │
                 ▼                                          │
   ┌──────────┐  b>=b_min, |ω|<ω_th, pose_ok   ┌────────────┴──┐
   │   IDLE   │───────────────────────────────►│    VALID      │
   └──────────┘                                └───────┬───────┘
        ▲   ▲                                      ... │
        │   │  b<b_min 持续                          ...
        │   └──────────────────────────────────────────┤
        │                                              │
        │  |ω|>=ω_th 且 b<b_min                        │ b>b_max
        ├──────────────────► ROTATION_ONLY  ──────────┤
        │  b>=b_min (恢复平移)                          │
        │                                              │
        │  pose 质量下降                                │
        ├──────────────────► DEGRADED  ────────────────┤
        │  质量恢复                                     │
        │                                              │
        └──────────────────► OVER_BASELINE ◄───────────┘
                (重置参考帧后回 IDLE)
```

---

## 3. 输入/输出结构（唯一定义处）

### 3.1 `NFBaselineInput`

```c
typedef struct {
    nf_u32 version;        // = NF_BASELINE_VERSION
    nf_u64 ts_us;          // 当前帧时间戳
    /* 位姿: 优先用视觉里程计/融合位姿; 缺省时由 v, ω 积分 */
    nf_u8  pose_valid;     // 1 = (R_cv, t_v) 可用
    nf_f32 R_cv[9];        // 当前帧相机旋转 (SPEC-COORD §2)
    nf_f32 t_v[3];         // 当前帧相机光心在 V 系, mm
    /* 运动学(用于判定退化与尺度自检) */
    nf_f32 v_mps;          // 纵向速度, m/s
    nf_f32 omega_rps;      // 横摆角速度, rad/s
    nf_u8  v_valid, omega_valid;
    nf_u8  odom_quality;   // 0=丢失 1=降级 2=正常
} NFBaselineInput;
```

### 3.2 `NFBaselineOutput`

```c
typedef struct {
    nf_u32 version;
    NFBaselineState state;
    nf_u8  is_valid;       // 与状态表的"允许调用引擎"一致
    /* 仅在 VALID 时有效 */
    nf_f32 b_mm;           // 基线长度
    nf_f32 t_rel[3];       // 平移在参考帧相机系, mm  (SPEC-COORD 4.2)
    nf_f32 R_rel[9];       // 参考帧→当前帧的旋转, 行优先
    nf_f32 d_max_px;       // 建议的视差上限 = f_view·b/Z_min_effective
    /* 参考帧管理 */
    nf_u32 ref_frame_id;
    nf_u32 cur_frame_id;
    /* 诊断 */
    NFStatus diag;         // NF_OK 或对应的 NF_E_BASELINE_*
} NFBaselineOutput;
```

### 3.3 字段约束

| 字段 | 约束 | 违反时 |
|---|---|---|
| `state == VALID` → `is_valid == 1` | 强一致 | 断言失败 |
| `is_valid == 0` → `b_mm == 0` | 强一致 | 断言失败 |
| `d_max_px` 仅在 `VALID` 时有意义 | — | 其余状态置 0 |
| `t_rel` 仅在 `VALID` / `ROTATION_ONLY` 时有意义 | `ROTATION_ONLY` 时全 0 | — |

---

## 4. 迁移条件（每条可判定）

设 `b = ‖t_now − t_ref‖`（mm），`ω = |omega_rps|`（rad/s）。阈值见 `PARAM.md` 的 B 节。

| 编号 | 从 | 到 | 条件（全部满足） |
|---|---|---|---|
| T-1 | IDLE | VALID | `b ≥ B_MIN` ∧ `ω < B_OMEGA` ∧ `odom_quality==2` ∧ `pose_valid` |
| T-2 | VALID | IDLE | `b < B_MIN` ∧ `ω < B_OMEGA`（静止或慢速，非旋转） |
| T-3 | VALID | ROTATION_ONLY | `ω ≥ B_OMEGA` ∧ `b < B_MIN` |
| T-4 | VALID | OVER_BASELINE | `b > B_MAX` |
| T-5 | VALID | DEGRADED | `odom_quality < 2` ∨ `pose_valid==0` ∨ 时间戳失步 > `B_DT_MAX` |
| T-6 | ROTATION_ONLY | IDLE | `ω < B_OMEGA` ∧ `b < B_MIN` |
| T-7 | ROTATION_ONLY | VALID | `b ≥ B_MIN`（恢复平移） |
| T-8 | DEGRADED | IDLE | `odom_quality==2` ∧ `pose_valid` ∧ `b < B_MIN` |
| T-9 | DEGRADED | VALID | `odom_quality==2` ∧ `pose_valid` ∧ `b ≥ B_MIN` |
| T-10 | OVER_BASELINE | IDLE | **无条件**（先重置参考帧为当前帧，再进 IDLE） |

### 4.1 T-3 与 T-2 的优先级

`ω ≥ B_OMEGA` 时，即使 `b` 略大于 `B_MIN` 也不进入 `VALID` —— 因为纯旋转下平移分量不可靠。
实现顺序：**先判旋转，再判基线长度**。

### 4.2 T-10 的重置语义

`OVER_BASELINE` 必须**原子地**把参考帧置为当前帧（`ref_frame_id = cur_frame_id`，`t_ref = t_now`），
否则下一帧仍会立即再次超限，形成抖动。

---

## 5. 参考帧管理

| 事件 | 动作 |
|---|---|
| 进入 VALID | 记录 `ref = 当前帧`（若尚无参考帧） |
| VALID 持续 | `ref` 不动，`b` 随 `t_now − t_ref` 增长 |
| b 超过 `B_TARGET` 的 1.5 倍 | 在下一帧把 `ref` 前移（滑动参考帧） |
| OVER_BASELINE | `ref = 当前帧` |
| DEGRADED→VALID | `ref = 当前帧`（位姿不可信期间的位移不能用） |

`B_TARGET` 见 `PARAM.md::B_TARGET`。滑动参考帧的目的是**让 `b` 稳定在可用区间中段**，
避免"刚进入 VALID 就接近 OVER_BASELINE"的锯齿。

---

## 6. `d_max_px` 的计算（与引擎容量的桥梁）

```
d_max_px = f_view · b_mm / Z_min_eff          (6.1)
```

- `f_view`：视口等效焦距（`NFViewParams::f`）
- `Z_min_eff`：本档愿意测量的最近距离（`PARAM.md::B_ZMIN_EFF`）

这个值直接写进 `NFDisparityRange::d_max`，引擎必须 `numDisparities ≥ d_max_px`（`SPEC-ENGINE` S-2）。
**若 `d_max_px` 超过引擎能承受的上限（`PARAM.md::E_MAX_DISP`），状态机必须拒绝进入 VALID**，
返回 `NF_E_BASELINE_OVER`——这是"引擎容量反压基线选择"的唯一入口。

---

## 7. 规模尺度自检（可选，P2 之后）

`B.SCALE_CHECK=1` 时，在直线段用已知宽度目标（车道线/车位线）反算基线尺度，
偏差 > `B.SCALE_TOL` 时对 `t_rel` 施乘性校正，并置诊断位。
**该功能默认关闭**，开启条件是已有实车数据（`APPENDIX-OPEN` O-6）。

---

## 8. 本规范的验收

| 编号 | 验收 | 方法 |
|---|---|---|
| V-BS-1 | 静止输入连续 100 帧，状态始终 IDLE，输出 `is_valid=0` | 单元测试 |
| V-BS-2 | 纯旋转输入（ω 大、b=0），进入 ROTATION_ONLY 且**不调用引擎** | 打桩计数 |
| V-BS-3 | 直线加速，b 单调增长，在 `b∈[B_MIN,B_MAX]` 内保持 VALID | 单元测试 |
| V-BS-4 | b 超 `B_MAX` 后一帧内进入 OVER_BASELINE，且随后回 IDLE 不抖动 | 边界测试 |
| V-BS-5 | `d_max_px > E.MAX_DISP` 时拒绝进入 VALID，返回 `NF_E_BASELINE_OVER` | 边界测试 |
| V-BS-6 | 状态与输出字段的一致性约束（§3.3）全部成立 | 断言注入 |
| V-BS-7 | 状态迁移延迟 ≤ `B.LAT_MAX`（从条件成立到状态改变） | 逐帧记录 |
