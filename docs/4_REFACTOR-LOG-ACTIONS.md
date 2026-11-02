# NFOCC REFACTOR LOG & REPRODUCIBLE ACTIONS
================================================================================
Format: Compressed Semantic Action Manifest (No raw source code dumps)
Reproducibility: 100% Deterministic & Complete
Compiler/Runtime: MinGW GCC 13.2.0, CMake 4.4.0, ZeroMQ, OpenCV SGBM, OpenMP
================================================================================

## [REV-001] 四重遮挡防御 (D1-D4)、空间方差传播与坐标系修复 (v0.5.0)
--------------------------------------------------------------------------------
### 1. 修改目标与符号契约
- **cpp/include/nfocc/nfocc_geometry.h** & **cpp/src/geometry/geometry.cpp**:
  - 新增 `nf_pt_in_foe(P, apex, dir, cos_aperture_sq)`: D3 FoE 极点开锥代数过滤。判据: h = P - apex, dot = h·dir > 0 且 dot^2 >= |h|^2 * cos^2(8°)。
  - 新增 `nf_occlusion_weight(d_fwd, d_bwd, tau, lambda)`: D2 双向一致性软抑制。公式: w = 1 / (1 + lambda * max(0, r/tau - 1)), lambda=14.0, tau=1.0px。
  - 新增 `nf_compute_spatial_uncertainty_cm(f, b, d, sig_d, sig_b)`: 严格偏导误差传播。公式: sigma_mm = f * (sig_d * (b/d^2) + (b*sig_b)/d), 转为 cm 输出并钳位至 [1, 254]。
- **cpp/include/nfocc/nfocc_voxel.h** & **cpp/src/voxel/voxel.cpp**:
  - `NFVoxelEntry` 增加 `sigma_cm` 字段。
  - 新增 `nf_voxel_insert_with_sigma(...)`: CMA 聚合时更新并保存最小不确定度。
  - 新增 `nf_voxel_verify_cross(...)`: D4 第二基线交叉验证。若残差 |P1 - P2| <= thresh，则 `sigma_cm >>= 1` (方差减半)；若残差超限，大幅削减权重 (`n_eff *= 0.2`) 直至剔除。
  - 零堆分配改进: 在 `NFVoxelCtx` 初始化中预分配 `scratch_items` (大小等于 capacity)，消除 `nf_voxel_export_points` 每帧的 `std::malloc` 与 `std::free`。
- **cpp/src/pipeline/pipeline.cpp**:
  - 修复前向路径“转两次”坐标 Bug: 虚拟视口射线已经在车系方向，移除多余的 `pose->R` 旋转；将视口系平移 `t_view` 经 `Rz(yaw)` 严格转换至车系后再输入 `nf_forward_range`。
  - 接驳 D1 车身遮蔽: 将 `p->cfg.body_mask` 传入 `nf_lut_build`。
  - 接驳 D3 FoE 过滤与方差传播至点云插入流水线。
- **构建系统**:
  - 修复 `CMakeLists.txt` 中缺失的 ZLIB 探测，添加 Windows/MinGW 便携路径 fallback。

--------------------------------------------------------------------------------
## [REV-002]参考量产系统算法体系深度对齐：自适应多帧双基线管理器与 2D 高程连续柱状压缩 (v0.6.0)
--------------------------------------------------------------------------------
### 1. 业务痛点与改进动因
- **基线状态机脆弱**: 原 `nf_gate_decide` 为单帧无状态瞬时判定，车速波动时极易频繁进入 FROZEN 冻结，缺乏历史平滑与多帧自适应能力。
- **高程图未排序**: 原 `nf_grid_flatten` 仅压平数组，下游提取任意网格柱体点云仍需遍历整体哈希表，无法实现参考量产系统的 O(1) 连续寻址。
- **掩码生成繁琐**: 缺乏从车体几何多边形到点阵掩码的轻量级自动化生成机制。

### 2. 修改目标与符号契约
- **cpp/include/nfocc/nfocc_baseline.h** (新增) & **cpp/src/geometry/baseline_manager.cpp** (新增):
  - 实现结构体 `NFHistoryFrame` 与 `NFBaselineManager`: 维护容量 16 的自适应历史环形队列。
  - 实现 `nf_baseline_manager_init(mgr, b_min, b_max, b_target)`: 参数默认 [200mm, 460mm, 300mm]。
  - 实现 `nf_baseline_manager_push(mgr, pos_mm, rot, ts_us, frame_id)`: 记录车体大地系位姿并维护时钟。
  - 实现 `nf_baseline_manager_decide(mgr, out_dec)`:
    - 采用车体基准位移 (Datum Delta) 统一各相机触发门限，解决转弯时外内侧基线差异导致的失步。
    - 航向角微分保护: 若两帧间 yaw 漂移超过 `max_yaw_deg` (15°)，判定为 `NF_MODE_FROZEN_ROT` 并重置参考帧。
    -参考量产系统并行双基线机制 (Parallel Dual Baselines): 同时维护 `prev_idx_short` 与 `prev_idx_long`。当长基线累积达 `b_target` 触发产出时，自动将短基线晋升为下一轮长基线，彻底消除速度变化时的感知空白期。
- **cpp/include/nfocc/nfocc_voxel.h** & **cpp/src/voxel/voxel.cpp**:
  - 新增 API: `nf_voxel_transform_columnar(ctx, grid_out, pc)`
    - 算法步骤 1: 扫描哈希表有效条目，在 `grid_out->zmap` 对应列置位 `1 << iz`。
    - 算法步骤 2: 顺序遍历每一个地表二维柱体 `(ix, iy)`，累加填充单调前缀和 `zindex`。
    - 算法步骤 3: 提取该柱体内垂直 iz = 0 .. nz-1 的所有体素，严格按高度递增顺序连续写入 `pc->points` 缓冲区。
    - 效益: 下游规划/避障算法仅需通过 `zindex[col] ~ zindex[col+1]-1` 即可 O(1) 索引该网格的所有三维物理点。
- **cpp/include/nfocc/nfocc_lut.h** & **cpp/src/lut/view_lut.cpp**:
  - 新增 API: `nf_mask_rasterize_polygon(mask, w, h, poly_x, poly_y, n_pts)`
    - 采用 OpenMP 并行加速的多边形射线交叉法 (Ray-Casting Algorithm)，将车辆外形轮廓顶点毫秒级光栅化为 binary mask，直接驱动 D1 遮挡防御。
- **cpp/include/nfocc/nfocc_geometry.h**:
  - `NFMode` 枚举扩展 `NF_MODE_FROZEN_ROT = 6`，配套完善 `nf_mode_str` 转换。
- **cpp/CMakeLists.txt**:
  - `nfocc_core` 目标添加 `src/geometry/baseline_manager.cpp` 编译单元。

### 3. 验证与回归基线 (CTest 全量通过)
- **test_geometry**:
  - `baseline_manager_multi_frame`: 验证历史帧累积、单帧短距离 hold、250mm 触发 STEREO 及 350mm 双基线交替晋升。
  - `foe_cone_filtering`: 验证发散锥内硬剔除与锥外保留。
  - `occlusion_soft_weight`: 验证一致区 w=1.0 与失配区软抑制。
  - `spatial_uncertainty_cm`: 验证随视差与基线的单调方差响应。
- **test_voxel**:
  - `voxel_transform_columnar_sorted`: 验证 zMap 位图精确匹配、zindex 前缀和区间与柱体点云按垂直高度递增排序。
  - `mask_rasterize_polygon_test`: 验证多边形内部标记为 1、外部标记为 0。
  - `voxel_verify_cross_and_sigma`: 验证双基线残差符合预期时方差减半、严重失配时权重归零/物理剔除。
- **总通过率**: CTest 9/9 套测试套件 100% 全部通过 (耗时约 19.65s)。

# 解读
# 对 `NFOCC REFACTOR LOG & REPRODUCIBLE ACTIONS` 的验证与改进报告

## 总体结论

该日志以压缩语义动作清单的形式记录了两个版本的重构，**格式清晰、可读性高、测试覆盖较完整**。但存在 **3 处致命问题会导致重现结果继承原规范错误**、**5 处关键定义缺失**、**3 处描述与实现不一致**。作为“可复现 100% 确定性”的文档，当前版本尚不满足该标准，需修正后重新冻结。

---

## 一、致命问题（必须修正，否则重现结果错误）

### 问题 1：物点反解公式未在 patch 中修正

REV-001 的 `pipeline.cpp` 条目只写“将视口系平移 `t_view` 经 `Rz(yaw)` 严格转换至车系后再输入 `nf_forward_range`”，**未提及反解公式本身的修正**。

上一轮验证已明确指出：原规范公式 $\mathbf{P} = \mathbf{P}_{curr} - \mathbf{t}_{veh} + \mathbf{d}_1 \cdot dist$ 存在物理错误——射线 $\mathbf{d}_1$ 从当前光心发出，不应再减去 $\mathbf{t}_{veh}$。若 patch 仅修了“转两次”但没有修正这一项，**重现结果仍会在车体系下偏移一个基线长度**。

**修正建议**：在 REV-001 的 `pipeline.cpp` 条目中显式写明：

> 物点反解统一为 $\mathbf{P}_{veh} = \mathbf{C}_{curr} + \mathbf{d}_1 \cdot dist$，其中 $\mathbf{C}_{curr}$ 为当前帧相机光心在车体系的位置，$\mathbf{d}_1$ 为车体系单位射线。删除原公式中的 $-\mathbf{t}_{veh}$ 项。

---

### 问题 2：`nf_pt_in_foe` 的 `apex` 参数物理意义未定义

REV-001 写“D3 FoE 极点开锥代数过滤，判据 h = P - apex”，但**未定义 apex 是什么**。上一轮验证已指出：apex 必须是**相机光心**，不是车辆平移向量 `pose->t`。若不注明，实现者极易误传 `pose->t`，导致 FoE 锥顶点偏离相机光学中心，过滤边界整体偏移。

**修正建议**：在符号契约中补充：

> `apex` = 当前帧相机光心在车体系的位置 $\mathbf{C}_{curr}$（mm），**不是**车辆平移向量。`dir` 由 $\mathbf{t}_{veh}$ 归一化得到，且 $\mathbf{t}_{veh}$ 方向定义为“当前帧光心 − 参考帧光心”。

---

### 问题 3：`nf_compute_spatial_uncertainty_cm` 的描述与实现不符

REV-001 写“**严格偏导误差传播**”，但给出的公式是线性相加：

$$\sigma_{mm} = f \cdot \left(\sigma_d \cdot \frac{b}{d^2} + \frac{b \sigma_b}{d}\right)$$

这是**保守 $L_1$ 上界**，不是严格偏导传播（严格应为平方和开根）。上一轮验证已明确指出这一矛盾。作为工程近似可以接受，但**描述与实现不符会误导后续维护者**。

**修正建议**：将描述改为：

> 采用保守 $L_1$ 上界合成（非严格统计方差），形式为 $\sigma_{mm} = f \cdot (\sigma_d b/d^2 + b \sigma_b/d)$。严格 $L_2$ 传播应为 $\sqrt{(\cdot)^2 + (\cdot)^2}$，本实现选择 $L_1$ 以覆盖最坏情况，标定阶段需验证其保守性。

同时明确 $\sigma_b$ 的语义：是**相对误差率**（无量纲，如 0.02）还是**绝对标准差**（mm）。从公式量纲看必须是相对误差率，但日志未注明。

---

## 二、关键定义缺失（会导致实现歧义）

### 缺失 1：`nf_baseline_manager_decide` 缺 `b_max` 超限处理

REV-002 定义了 `b_max = 460mm`，但**决定逻辑中未见使用**。若 $\Delta_{long} > b_{max}$，重叠视场不足，匹配失效率陡增。

**补充**：增加迁移规则——`若 Δ_long > b_max，强制重置 prev_idx_long = prev_idx_short = curr_slot，并清空中间历史`。

### 缺失 2：短基线未触发时的更新规则

REV-002 写“当长基线累积达 `b_target` 触发产出时，自动将短基线晋升为下一轮长基线”，但**未说明长基线未触发时短基线是否每帧更新**。若不更新，短基线会越来越老，恢复时基线过长。

**补充**：明确“每帧到达时，`prev_idx_short` 按固定策略滚动（保持最近有效帧或按时间衰减），与长基线是否触发解耦”。

### 缺失 3：`sig_b` 的语义与量纲

`nf_compute_spatial_uncertainty_cm(f, b, d, sig_d, sig_b)` 中 `sig_b` 未定义。从公式 `b*sig_b` 的量纲看，`sig_b` 是**相对误差率**，但日志未注明。实现者若误传绝对 mm 值，结果量纲错乱。

### 缺失 4：`nf_occlusion_weight` 中 $r$ 的符号约定

REV-001 写“$w = 1/(1 + \lambda \cdot \max(0, r/\tau - 1))$”，但未定义 $r$。若 $r = |d_{fwd} - d_{bwd}|$，则依赖视差符号约定（后向视差是否翻转）。需与 SGBM 的 `disp12MaxDiff` 语义对齐。

**补充**：明确“$r = |d_{fwd} - d_{bwd}|$，其中 $d_{fwd}$ 为左→右视差，$d_{bwd}$ 为右→左视差的**同号化版本**（即若 SGBM 输出的后向视差符号相反，须先取负）”。

### 缺失 5：`scratch_items` 的 `capacity` 定义与溢出处理

REV-001 写“预分配 `scratch_items`（大小等于 capacity）”，但 `capacity` 来源未说明，且**未定义实际输出点数超过 capacity 时的处理**。

**补充**：`capacity` 应定义为“单帧最大体素输出点数上限”，由网格尺寸与最大点数推导（建议 `capacity = max_voxels_per_frame`），并在 `nf_voxel_export_points` 中做溢出检查：超出则截断并递增溢出计数器，而非越界写入。

### 缺失 6：`nf_mask_rasterize_polygon` 的坐标系

REV-002 未注明多边形顶点与查询点所在坐标系。射线法要求二者一致，否则判定错误。

**补充**：“多边形顶点与 mask 像素均定义在**虚拟视口图像坐标系**下”。

---

## 三、描述与实现不一致（需澄清，不影响正确性）

### 不一致 1：`sigma_cm >>= 1` 未标注经验近似

REV-001 写“残差达标则 `sigma_cm >>= 1`（方差减半）”。上一轮验证已指出：除以 2 比统计最优（除以 $\sqrt{2}$）更乐观，可能低估不确定度。作为工程近似可保留，但**必须标注**。

**修正**：改为“`sigma_cm >>= 1` 为经验保守近似（严格贝叶斯融合应为 $\sigma/\sqrt{2}$），标定阶段需验证其不低估不确定度”。

### 不一致 2：CMA 的“最小不确定度”聚合规则需明确

REV-001 写“`nf_voxel_insert_with_sigma(...)`: CMA 聚合时更新并保存**最小不确定度**”。若取最小，多个观测的不确定度可能都很大，但最小那个不代表整体——这与 CMA 质心聚合的统计意义不一致。

**建议**：明确为“保存**加权融合后的不确定度**（按各观测的 $1/\sigma^2$ 加权），而非简单取最小值。若实现选择最小值，需注明这是保守下界估计”。

### 不一致 3：CMake 4.4.0 版本号仍可疑

与上轮验证一致：截至 2025 年 CMake 主流为 3.30+，4.4.0 是否存在需确认。建议改为 `CMake ≥ 3.20` 或明确来源。

---

## 四、测试覆盖的补充建议

当前测试列表覆盖了主要路径，但**缺少以下关键断言**：

| 建议新增测试 | 验证目标 |
| :--- | :--- |
| `test_geometry: foe_apex_is_camera_center` | 验证 FoE 锥顶点传入的是相机光心而非平移向量 |
| `test_pipeline: forward_range_point_in_vehicle_frame` | 验证物点反解结果在车体系下与真值偏差 ≤ 1 mm（标定场景） |
| `test_geometry: baseline_bmax_overflow_resets` | 验证 $\Delta_{long} > b_{max}$ 时队列强制重置 |
| `test_voxel: sigma_cm_never_underestimates` | 验证双基线验证后的 $\sigma_{cm}$ 不低于 $\sigma/\sqrt{2}$ 的理论下界 |
| `test_voxel: scratch_items_overflow_truncates` | 验证输出点数超过 capacity 时截断而非越界 |

另外，`baseline_manager_multi_frame` 测试写“250mm 触发 STEREO 及 350mm 双基线交替晋升”——**250mm < b_target=300mm，不应触发 STEREO**。需确认该测试的预期：是验证“未达 b_target 时保持 hold”，还是文档笔误（应为 350mm 触发）？建议明确。

---

## 五、改进后的修订建议汇总

| 编号 | 位置 | 问题 | 严重度 | 修正动作 |
| :--- | :--- | :--- | :--- | :--- |
| 1 | REV-001 pipeline | 物点反解公式未修正 | **致命** | 显式写入 $\mathbf{P}=\mathbf{C}_{curr}+\mathbf{d}_1\cdot dist$ |
| 2 | REV-001 FoE | apex 物理意义未定义 | **致命** | 注明 apex = 相机光心，非平移向量 |
| 3 | REV-001 不确定度 | “严格偏导”与线性相加不符 | **致命** | 改为“保守 $L_1$ 上界”，明确 $\sigma_b$ 为相对误差率 |
| 4 | REV-002 baseline | 缺 $b_{max}$ 超限重置 | 高 | 补充迁移规则 |
| 5 | REV-002 baseline | 短基线更新规则缺失 | 高 | 明确与长基线触发解耦 |
| 6 | REV-001 occlusion | $r$ 符号约定未定义 | 中 | 明确同号化规则 |
| 7 | REV-001 scratch | capacity 定义与溢出处理缺失 | 中 | 定义容量来源与截断策略 |
| 8 | REV-002 mask | 坐标系未注明 | 低 | 标注为视口图像坐标系 |
| 9 | REV-001 sigma>>=1 | 未标注经验近似 | 中 | 注明保守近似及理论下界 |
| 10 | REV-001 CMA | “最小不确定度”聚合规则需明确 | 中 | 建议改为 $1/\sigma^2$ 加权融合 |
| 11 | 环境 | CMake 4.4.0 版本可疑 | 低 | 改为 CMake ≥ 3.20 或注明来源 |
| 12 | 测试 | 缺 FoE apex、反解车系、b_max、sigma 下界、scratch 溢出测试 | 中 | 补入测试列表 |

---

## 六、最终判断

该 patch 日志**记录了正确的重构方向**——四重防御、双基线状态机、O(1) 高程压缩、零堆分配等改进均与 NFOCC 量产化目标一致，测试覆盖也较为完整。但**作为“100% 确定性可复现”的文档，当前版本尚不达标**：致命问题 1–3 若不修正，重现结果会继承原规范的坐标系错误与描述误导；关键定义缺失 1–6 会导致不同实现者产出不同结果，破坏可复现性。

**建议**：按上表修正后发布 **v0.6.1-Corrected**，将“致命问题”三项作为 patch 的显式修正条目写入，并在文档头部增加“**已知继承问题修正公告**”段落，说明本 patch 相对原规范的数学修正点。修正后即可冻结为开发基线。