# SPEC-ENGINE — 视差引擎接口（SGBM / 模型双实现，接口兼容）

本文档定义唯一的引擎接口 `IDisparityEngine`。SGBM 实现与模型实现**只能**通过此接口与系统交互。
上层模块不得感知具体引擎。符号见 `SPEC-COORD.md`，结构体见 `SPEC-ICD.md`。

---

## 1. 设计约束（来自用户拍板）

| 约束 | 影响 |
|---|---|
| 不用硬件 SDE | 全部引擎在 CPU/NPU 上跑，必须自己管 `numDisparities` 与内存 |
| RK3588 + Axera M57 | 模型实现必须能导出（ONNX/RKNN/AXMODEL），且尺寸静态 |
| SGBM 与模型可替换 | 接口只暴露"视差 + 有效性 + 置信度"，不暴露引擎内部状态 |
| 单 Y 通道 1920×1536 | 引擎输入是单通道，不需要颜色；模型训练也按单通道 |
| 视差量级 10–580 px | SGBM 的 `numDisparities`、模型的 `max_disp` 都必须能到这个量级 |

---

## 2. 接口定义

### 2.1 核心函数签名

```c
NFStatus nf_engine_init   (NFEngineCtx **ctx, const NFEngineConfig *cfg);
NFStatus nf_engine_run    (NFEngineCtx *ctx,
                           const NFImageView *left,
                           const NFImageView *right,
                           const NFDisparityRange *range,
                           NFDisparityMap *out);
NFStatus nf_engine_deinit (NFEngineCtx *ctx);
```

`NFImageView` / `NFDisparityRange` / `NFDisparityMap` 的定义见 `SPEC-ICD.md` §4。

### 2.2 语义要点（逐条可验收）

| 编号 | 约束 | 理由 |
|---|---|---|
| E-1 | 输入图像 `W=1920, H=1536`，单通道 8 bit，`stride` 可与 `W` 不同 | 平台对齐要求 |
| E-2 | `left` 与 `right` 必须**已经极线对齐且行对齐**（由 `nf_view_warp` 产出，见 `SPEC-ICD`） | 引擎不做几何校正 |
| E-3 | 允许每行有效起点不同：`NFImageView::valid_x0[y]` 给出该行第 0 个有效列 | 视口重排后是弧形行（`SPEC-EPIPOLAR` §3.3） |
| E-4 | `range` 由基线状态机给出，引擎**不得**自行扩大或缩小 | 搜索范围与基线必须同源 |
| E-5 | 输出视差单位为 px，符号按 `SPEC-COORD` §4.2（**d > 0**） | 统一符号，避免适配层反复取反 |
| E-6 | 无效像素必须由 `valid` 掩码显式标出，**不得用 0 表示无效** | 0 是合法视差（d=0 对应无穷远） |
| E-7 | 失败必须返回 `NFStatus`，不得返回 NaN 或空指针 | 失效语义统一 |
| E-8 | 引擎必须幂等：同输入同配置，两次调用结果逐位相同（确定性） | 便于回归与录包重放 |

---

## 3. SGBM 实现的约束（由实测得出，不是建议）

以下四条是 `tools/sgbm_bench.py` 在 1280×720、真实视差 180 px 下实测的结果，
直接构成 SGBM 实现的实现约束。

| 编号 | 事实 | 实测数据 | 约束 |
|---|---|---|---|
| S-1 | `numDisparities` 单位是**像素** | `compute()` 返回 CV_16S，÷16 得像素视差 | 不要写成"128 即 8 px" |
| S-2 | 容量 < 真实视差 → **完全失效**（不是精度差） | 真实 180 px、`numDisp=128` 时中位误差 89 px | `numDisparities ≥ d_max(range)`，硬断言 |
| S-3 | 有效覆盖 = `(W − numDisparities)/W`，无效列是最左 `numDisparities` 列 | 1920 宽、`numDisp=512` → 覆盖 73% | 与 E-3 的 `valid_x0[y]` 对接 |
| S-4 | 内存 = `numDisparities × W × H × 2 B` | 1280×720/1024 → 1.89 GB，**2048 直接 OOM** | 1920×1536 下必须按档限制 |

**S-4 的量化**：1920×1536 时
`mem = numDisp × 1920 × 1536 × 2 = numDisp × 5.9 MB`。

| numDisp | 内存 | 能覆盖的最近距离（b=100，f_view=579） |
|---|---|---|
| 64 | 0.38 GB | 905 mm |
| 128 | 0.75 GB | 452 mm |
| 256 | 1.51 GB | 226 mm |
| 512 | 3.02 GB | 113 mm |

**结论：SGBM 在 1920×1536 全分辨率下，`numDisp` 超过 256 就吃不消。**
必须靠**降采样视口**或**分块**解决，这是 `APPENDIX-OPEN.md` O-3 的决策项。

### 3.1 SGBM 参数默认表

| 参数 | 默认 | 依据 |
|---|---|---|
| `minDisparity` | 0 | 视差下限由 `range` 表达，不用 minDisparity |
| `numDisparities` | `ceil16(d_max(range))` | S-2；OpenCV 要求 16 的整数倍 |
| `blockSize` | 5 | 近场纹理尺度小；见 `PARAM.md::E_BLOCK` |
| `P1 / P2` | `8·bs²`, `32·bs²` | OpenCV 惯例，弱纹理场景用标准值 |
| `disp12MaxDiff` | 1 | 提供左右一致性无效标记 |
| `uniquenessRatio` | 10 | 抑制重复纹理误配 |
| `speckleWindowSize` | 100 | 滤除小连通域噪声 |
| `speckleRange` | 2 | 同上 |
| `mode` | `SGBM_3WAY` | 速度与质量折中 |

---

## 4. 模型实现的约束

| 编号 | 要求 | 说明 |
|---|---|---|
| M-1 | 输入 1×1×H×W 单通道，静态尺寸 | 平台导出要求，不允许动态 shape |
| M-2 | 输出视差图 + 置信度图（两张） | 无置信度的模型不可用（无法做 CMA 加权） |
| M-3 | 输出必须为 **float**，不得在模型内取整 | 亚像素是近场精度的必要条件 |
| M-4 | `max_disp` ≥ 该档最大视差 | 同上 S-2 的教训：容量不足即失效 |
| M-5 | 必须能表达"无效" | 用置信度阈值替代显式掩码，但接口仍要填 `valid` |
| M-6 | 导出后板端精度损失 < 1% 视差 | 见 `SPEC-VERIFY.md::V-ENG-5` |

### 4.1 对现有候选的结论（不重复评估，只记结论）

工作区 `stereodisp`（DispMap）**不满足 M-2、M-3、M-4**：无置信度、`to(torch.int32)` 截断亚像素、
`max_disp=96`。若采用必须重设计 + 重训，不属于"换引擎"范畴。详见 `notes/03-disparity-engine.md`。

---

## 5. 两实现的等价性要求

同一个 `NFEngineConfig::kind` 之外，两实现必须满足：

| 编号 | 要求 | 验收 |
|---|---|---|
| X-1 | 对同一合成场景（已知真值），两实现的中位误差均在 1 px 内 | V-ENG-3 |
| X-2 | 无效掩码的语义一致（都表示"该像素视差不可信"） | 代码审查 + V-ENG-4 |
| X-3 | 上传/下载开销之外，接口层无可观测差异 | 上层代码不出现 `kind` 分支 |

**X-3 是"接口兼容"的实质检验：如果上层代码需要 `if kind == SGBM` 才能工作，接口就没做成。**

---

## 6. 配置结构

`NFEngineConfig` 字段定义收敛在 `SPEC-ICD.md` §4.3，本文只规定语义：

| 字段 | 语义 | 谁填 |
|---|---|---|
| `kind` | `NF_ENGINE_SGBM` / `NF_ENGINE_MODEL` | 系统配置 |
| `max_disp_px` | 本档视差上限，必须 ≥ `range.d_max` | 基线状态机 → 配置器 |
| `downscale` | 视口降采样倍率（1/2/4） | `APPENDIX-OPEN` O-3 决策后固定 |
| `conf_thresh` | 置信度阈值（仅模型） | 参数手册 |

---

## 7. 本规范的验收

| 编号 | 验收 | 方法 |
|---|---|---|
| V-ENG-1 | 容量不足时必须返回 `NF_E_ENGINE_RANGE`，不得静默输出 | 故意传小 `numDisp` |
| V-ENG-2 | 相同输入两次调用逐位相同 | 跑两遍比对 |
| V-ENG-3 | 合成场景（已知真值 180 px）中位误差 ≤ 1 px | 两实现各跑一次 |
| V-ENG-4 | 无效掩码不与 `d=0` 混淆 | 构造全黑图，检查 `valid` 全 0 而视差全 0 |
| V-ENG-5 | 板端与 PC 端输出的中位误差差 < 1% | 导出模型后对拍 |
