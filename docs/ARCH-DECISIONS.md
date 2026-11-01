# NFOCC C++11 产品实现 — 架构决策记录 (ADR)

本文记录把 Python 原型落到 C++11 stream pipeline 时的**关键决策与理由**，
以及它们与既有规范（`SPEC-BUILD`）的冲突处理。

---

## ADR-001：语言从 C99 改为 C++11（覆盖 `SPEC-BUILD` §1）

| | 原规范 | 本次要求 |
|---|---|---|
| 语言 | C99 + 少量 C++17（仅测试） | **C++11**（全工程） |
| 禁用 | C++ 异常、RTTI、STL | 允许 STL，但禁异常/RTTI |
| 命名 | C 风格 `nf_*` | 保留 C 兼容头 + C++ 内部实现 |

**决策**：以本次要求为准。理由：
1. 用户明确要求 C++11 与 ZeroMQ/onnxruntime/pipeline 架构；
2. ZeroMQ 与 onnxruntime 的 C++ 绑定天然是 C++ 接口；
3. `std::shared_ptr`/`std::atomic` 显著降低 pipeline 生命周期管理的出错面。

**保留的约束**（来自原规范，仍然有效）：
- `include/nfocc/` 之外的头文件不得被外部 include；
- 错误一律用 `NFStatus` 返回，**不用异常**；
- 编译期 `-Wall -Wextra -Werror`。

**仍然禁用**：异常、RTTI（`-fno-exceptions -fno-rtti`）。

---

## ADR-002：三角化改用射线最小二乘（覆盖 `SPEC-ICD` §7.1）

`SPEC-ICD` §7.1 写的是 `Z = VIEW_F · b / d`。

**该式只在平移垂直于视线时成立。** Python 原型实测（`out/probe_disp.py`）：
光轴附近（`|u−cx| < 100`）相对误差 **10.3 倍**，边缘才降到 0.095。

**决策**：C++ 实现用射线最小二乘：

```
P = C0 + s0·r0 = C1 + s1·r1
[r0, −r1]·[s0, s1]ᵀ = t     (2×2 正规方程)
```

`Z = VIEW_F·b/d` 仅保留为 `nf_triangulate_fast()` 的量级估计函数，不用于生产路径。
理由与推导见 `docs/SPEC-EPIPOLAR.md` §4 与 `docs/SPEC-FORWARD.md` §2。

**验收**：`V-TRI-1` 用真值视差三角化，相对误差 < 1e-6（Python 实测 1.4e-12）。

---

## ADR-003：前向运动走解析测距，不走立体匹配

Python 原型证明纯前向平移下 SGBM 结构性不可用（像移方向左右翻转、
`|dv|/|du| = 0.23`、极线行内离散 175.9°）。

**决策**：pipeline 里有明确的分支：

```
σ = |t_cam.xy| / |t_cam|
σ ≥ 0.80  -> SgbmStage        (侧向)
σ ≤ 0.35  -> ForwardRangeStage (前向，解析测距)
中间       -> 两条都跑，按 σ 加权融合
σ 无效 / B 超限 -> 冻结，不产出
```

**冻结时输出 `points = nullptr, num_points = 0` 且带 `NF_E_BASELINE_*`**，
不是输出空点云 —— 下游必须能区分"没结果"与"没障碍"。

---

## ADR-004：stream pipeline 的分级结构

```
┌─────────────────────────────────────────────────────────────┐
│  Source (ZMQ SUB / 文件回放 / 合成)                          │
└───────────────────┬─────────────────────────────────────────┘
                    │ FrameBundle (4 路 + 位姿 + 时间戳)
┌───────────────────▼─────────────────────────────────────────┐
│  SyncStage: 时间戳校验(极差 ≤5ms)、帧配对、丢帧检测            │
└───────────────────┬─────────────────────────────────────────┘
                    │
┌───────────────────▼─────────────────────────────────────────┐
│  BaselineStage: 状态机 -> STEREO / FORWARD / HYBRID / FROZEN  │
└───────────────────┬─────────────────────────────────────────┘
                    │  (按相机并行或串行, 由 NF_THREAD_CAMERAS 决定)
        ┌───────────┼───────────┬───────────┐
        ▼           ▼           ▼           ▼
   ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐
   │ Cam FV │  │ Cam RV │  │ Cam ML │  │ Cam MR │
   ├────────┤  ├────────┤  ├────────┤  ├────────┤
   │ Warp   │  │ Warp   │  │ Warp   │  │ Warp   │
   │ Detect │  │ Detect │  │ Detect │  │ Detect │
   │ Tri    │  │ Tri    │  │ Tri    │  │ Tri    │
   └───┬────┘  └───┬────┘  └───┬────┘  └───┬────┘
       └───────────┴─────┬─────┴───────────┘
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  FusionStage: 体素 Hash 插入 + CMA (单线程, 无锁)             │
└───────────────────┬─────────────────────────────────────────┘
                    ▼
┌─────────────────────────────────────────────────────────────┐
│  Sink (ZMQ PUB / 文件 / 可视化)                              │
└─────────────────────────────────────────────────────────────┘
```

**为什么 Fusion 单线程**：`SPEC-BUILD` §4 的实算 —— 4 路并行 SGBM 峰值 1.93 GB，
M57 不容；串行 420 MB。体素表单线程写避免加锁。

---

## ADR-005：Stage 的统一接口

```cpp
class Stage {
public:
    virtual ~Stage() = default;
    virtual NFStatus Init(const StageConfig&) = 0;
    // 同步执行: 便于 SIL 单步调试与确定性重放
    virtual NFStatus Process(const FrameContext& in, FrameContext& out) = 0;
    virtual const char* Name() const = 0;
};
```

**关键决策：Stage 是同步的，pipeline 才决定并发。**

理由：`V-ENG-2` 要求"同输入两次调用逐位相同"。若 Stage 内部自带线程池，
确定性无法保证，录包重放会漂。并发只在 Pipeline 层按相机维度开。

---

## ADR-006：依赖裁剪

| 依赖 | 用途 | 是否进入核心库 |
|---|---|---|
| OpenCV | SGBM、图像 warp、可视化 | **否**，只在 `engine_sgbm` 与 `tools` |
| ZeroMQ | 帧输入/点云输出 | 否，只在 `io` |
| onnxruntime | 模型引擎（占位，PH-4） | 否，只在 `engine_model` |

**核心库（`libnfocc_core`）零第三方依赖**，便于 Aarch64 交叉编译与
`V-BUILD-2`（`ldd` 检查核心库不链接 OpenCV）。

---

## ADR-007：坐标约定的唯一来源

本项目在坐标系上踩错过的次数最多（Python 原型里 4 次）。C++ 里定为：

| 量 | 约定 | 出处 |
|---|---|---|
| `NFPose::R` | `v_V = R · v_C`，行优先 | `SPEC-ICD` §2.3 |
| 投影 | `p_C = Rᵀ · (P_V − t)` | 由上一行推出 |
| 视口 | 针孔，`p = f·(q.xy/q.z) + c` | `SPEC-COORD` §1.4 |
| 体素 | `ix = (X_MAX − X)/vox`（**X/Y 反向**） | `SPEC-COORD` §5 |

`NFPose::R` 与 Python 的 `R_cv`（`p_cam = R·(P−C)`）**互为转置**。
C++ 侧统一用 `SPEC-ICD` 的约定，转换只在一个地方做（`pose_util.h::fv_from_cv`）。

---

## ADR-008：测试分层

| 层 | 内容 | 对应 |
|---|---|---|
| 单元 | 每个 `nf_*` 函数 + 负对照 | `SPEC-VERIFY` V-CO/V-ENG/V-TRI/V-VOX |
| 集成 | Warp→Engine→Triangulate 链路 | `V-ENG-3`、`V-TRI-1` |
| 系统 | 全 pipeline + 合成数据 + RSS 统计 | `V-BUILD-3/7`、`V-PROD-*` |
| 跨平台 | 同一用例在 macOS / Aarch64 各跑一次，比对输出 | `V-ENG-5` |

测试框架用**自带的轻量断言**（不引入 gtest），理由是 Aarch64 交叉编译时
少一个依赖，且失败信息需要中文。
