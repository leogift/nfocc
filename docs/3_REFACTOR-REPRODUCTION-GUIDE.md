# REFACTOR-REPRODUCTION-GUIDE

> **文档代号**：`REFACTOR-REPRODUCTION-GUIDE`  
> **适用基线**：原版 NFOCC (C++11) $\to$ 现代工业量产增强版 NFOCC v0.6.0+  

---

## 1. 环境准备与便携工具链环境变配

编译环境要求：
*   **C++ 编译器**：GCC 13.2.0 (MinGW-W64) 或更高，支持 C++11、OpenMP 4.5 与 AVX2；
*   **构建工具**：CMake 4.4.0 或更高；
*   **依赖库**：ZLIB、OpenCV 4.8.0、ZeroMQ (cppzmq)。

在 Windows PowerShell 会话中设置环境：
```powershell
$env:PATH = "C:\Users\bcai5\Workspace\PortableGit\cmd;C:\Users\bcai5\Workspace\cmake-4.4.0\bin;C:\Users\bcai5\Workspace\mingw64\bin;" + $env:PATH
```

---

## 2. 代码重构改造工程步骤清单

### 步骤 1：新建移动双目多帧历史与双基线状态机模块
1.  **创建头文件**：`cpp/include/nfocc/nfocc_baseline.h`
    *   定义容量为 16 的环形队列 `NFHistoryFrame history[16]`；
    *   定义 `NFBaselineManager` 结构体，包含 `prev_idx_short` 与 `prev_idx_long` 双索引，以及 `b_min_mm` (200), `b_max_mm` (460), `b_target_mm` (300)；
    *   声明 API：`nf_baseline_manager_init`、`nf_baseline_manager_push`、`nf_baseline_manager_decide`。
2.  **创建实现文件**：`cpp/src/geometry/baseline_manager.cpp`
    *   计算车体系相对平移：$\mathbf{t}_{	ext{rel}} = R_{	ext{curr}}^T (\mathbf{P}_{	ext{curr}} - \mathbf{P}_{	ext{long}})$；
    *   计算相对航向角 $\Delta\psi = |\operatorname{atan2}(R_{	ext{rel}}(1, 0), R_{	ext{rel}}(0, 0))| 	imes rac{180}{\pi}$，超 $15^\circ$ 判定 `NF_MODE_FROZEN_ROT`；
    *   位移达 `b_target` 时晋升短基线：`prev_idx_long = prev_idx_short`，`prev_idx_short = curr_slot`。
3.  **配置 CMake**：
    *   在 `cpp/CMakeLists.txt` 的 `NF_CORE_SRC` 列表中加入 `src/geometry/baseline_manager.cpp`。

### 步骤 2：落地四重遮挡防御与不确定度计算（修改 geometry 模块）
1.  **修改 `cpp/include/nfocc/nfocc_geometry.h`**：
    *   在 `NFMode` 枚举尾部追加 `NF_MODE_FROZEN_ROT = 6`；
    *   声明 `nf_pt_in_foe(P, apex, dir, cos_aperture_sq)`；
    *   声明 `nf_occlusion_weight(d_fwd, d_bwd, tau, lambda)`；
    *   声明 `nf_compute_spatial_uncertainty_cm(f, b, d, sig_d, sig_b)`。
2.  **修改 `cpp/src/geometry/geometry.cpp`**：
    *   实现 `nf_pt_in_foe`：计算点积与模长平方判别式 $(\mathbf{h}\cdot\mathbf{d})^2 \ge |\mathbf{h}|^2 \cos^2(8^\circ)$；
    *   实现 `nf_occlusion_weight`：残差超出 1.0px 时按 $1/(1 + 14 \cdot 	ext{excess})$ 软性压低置信度；
    *   实现 `nf_compute_spatial_uncertainty_cm`：按 $f \cdot ( \sigma_{	ext{pixel}} rac{b}{d^2} + rac{b \cdot \sigma_{	ext{odom}}}{d} )$ 换算为厘米并返回 `[1, 254]`。

### 步骤 3：体素网格结构扩展与 O(1) 连续高程压缩（修改 voxel 模块）
1.  **修改 `cpp/include/nfocc/nfocc_voxel.h`**：
    *   在 `NFVoxelEntry` 中增加 `nf_u8 sigma_cm;`；
    *   声明 `nf_voxel_insert_with_sigma`；
    *   声明 `nf_voxel_verify_cross`；
    *   **【关键顺序】** 将 `typedef struct { ... } NFPointCloud;` 结构体定义移至 `nf_voxel_transform_columnar` 声明之前，避免类型未决错误；
    *   声明 `nf_voxel_transform_columnar(ctx, grid_out, pc)`。
2.  **修改 `cpp/src/voxel/voxel.cpp`**：
    *   **零堆分配改造**：在 `struct NFVoxelCtx` 中增加 `void *scratch_items;`，在 `nf_voxel_create` 中一次性分配 `capacity * sizeof(ExportItem)`，并在 `nf_voxel_destroy` 中释放。彻底移除 `nf_voxel_export_points` 中的每帧动态 malloc/free；
    *   在 `nf_voxel_insert_with_sigma` 中实现方差融合（保留更优测量）与格内质心钳位；
    *   在 `nf_voxel_verify_cross` 中实现残差小于门限时方差减半（`sigma_cm >>= 1`）、残差过大时大幅衰减（`n_eff *= 0.2`）；
    *   实现 `nf_voxel_transform_columnar`：按地表网格 `(ix, iy)` 遍历，生成位图 `zmap` 与前缀和 `zindex`，并将点按垂直高度递增顺序连续紧凑装配至 `pc->points`。

### 步骤 4：车身掩码光栅化与视口接驳（修改 lut 模块）
1.  **修改 `cpp/include/nfocc/nfocc_lut.h`**：
    *   声明 `nf_mask_rasterize_polygon(mask, w, h, poly_x, poly_y, n_pts)`。
2.  **修改 `cpp/src/lut/view_lut.cpp`**：
    *   使用 OpenMP 并行多边形射线相交法实现 `nf_mask_rasterize_polygon`。
3.  **修改 `cpp/include/nfocc/nfocc_pipeline.h`**：
    *   在 `NFPipelineConfig` 中增加 `const nf_u8 *body_mask;` 配置项。

### 步骤 5：Pipeline 坐标变换修复与全链路串联（修改 pipeline 模块）
1.  **修改 `cpp/src/pipeline/pipeline.cpp`**：
    *   **接驳 D1 掩码**：在 `nf_pipeline_create` 中将 `p->cfg.body_mask` 传递给 `nf_lut_build`；
    *   **修复前向“转两次”Bug**：
        *   移除循环中 `d0` 和 `d1` 乘以 `pose->R` 的冗余旋转语句（直接使用车系方向 `a = d0; b = d1;`）；
        *   将视口平移通过偏航角转换至车系：`t_veh = { t_view[0]*cos(yaw) - t_view[1]*sin(yaw), ... }`；
        *   输入 `nf_forward_range(a, b, 1, t_veh, &dist, &okp)` 并反解 `P`；
    *   **串接 D3 FoE 过滤与方差传播**：
        *   在前向和侧向三角化产生点后，调用 `nf_pt_in_foe(P, pose->t, dir_t, 0.9806f)`，在发散锥内则跳过；
        *   调用 `nf_compute_spatial_uncertainty_cm` 计算真实方差，并传入 `nf_voxel_insert_with_sigma`。

### 步骤 6：构建系统健壮性加固（修改 CMakeLists.txt）
在 `CMakeLists.txt` 中增加 ZLIB 探测回退机制，并在 `nfocc_core` 引入路径中加入 `${ZLIB_INCLUDE_DIR}`，确保在 Windows MinGW 平台下无痛编译。

---

## 3. 六大高频已知陷阱（Pitfalls）与排错备忘录

| 序号 | 故障表现 | 根本诱因 | 规避与修复方案 |
|---|---|---|---|
| **1** | 前向测距深度误差高达数米甚至点云全被 50m 门限过滤 | 视口射线本身已在车系，代码中又乘了一次 `pose->R`（转两次），且视口平移未转车系 | 保持射线为车系基底，平移向量左乘 $R_z(	ext{yaw})$ 后再代入解析方程。 |
| **2** | 编译报错 `conflicting declaration 'typedef struct NFPointCloud NFPointCloud'` | `nfocc_voxel.h` 中前置声明了类型，后面又使用了匿名结构体 `typedef struct { ... } NFPointCloud;` | 调整结构体定义顺序，先定义具名结构体 `NFPointCloud`，再声明相关导出 API。 |
| **3** | 车身周围出现大量静态假体素 | `pipeline.cpp` 中调用 `nf_lut_build` 时传了 `nullptr` | 接驳 `NFPipelineConfig::body_mask`，在 LUT 生成期直接令其 `valid = 0`。 |
| **4** | 点云导出时 `pt->sigma_cm` 恒为 255 | 原代码将该字段作为固定占位符硬编码为 255 | 改为输出体素结构体中动态维护的 `entry->sigma_cm`。 |
| **5** | 编译报错 `M_PI was not declared in this scope` | MinGW C++11 严格模式下 `<cmath>` 不默认暴露 POSIX 宏 | 在源码文件头部显式定义 `static const float kPi = 3.14159265358979323846f;`。 |
| **6** | 频繁运行出现内存碎片与随机延迟增加 | `nf_voxel_export_points` 每帧在堆上 malloc 和 free `ExportItem` 缓冲 | 纳入 `NFVoxelCtx` 初始化时静态预分配的 `scratch_items` 池化管理。 |

---

## 4. 自动化回归测试执行与验收准则

在构建目录下执行验证：
```powershell
cd cpp
cmake -S . -B build_test -G "MinGW Makefiles" -DNF_DEPS=system -DNF_WITH_OPENCV=OFF -DNF_WITH_ZMQ=OFF -DNF_WITH_ORT=OFF -DNF_USE_OPENMP=OFF -DNF_BUILD_TOOLS=OFF
cmake --build build_test -j
cd build_test
ctest --output-on-failure
```

**验收标准（必须 100% 通过）**：
```text
1/9 Test #1: unit_camera ......................   Passed
2/9 Test #2: unit_geometry ....................   Passed  (含 baseline_manager、FoE开锥、双向软抑制、方差模型)
3/9 Test #3: unit_voxel .......................   Passed  (含 2D柱状连续排序、O(1)前缀和、多边形光栅化、双基线验证)
4/9 Test #4: unit_concurrency .................   Passed
5/9 Test #5: unit_io_protocol .................   Passed
6/9 Test #6: unit_match .......................   Passed
7/9 Test #7: unit_hanqtech ....................   Passed
8/9 Test #8: unit_scene .......................   Passed
9/9 Test #9: integration_pipeline .............   Passed
100% tests passed out of 9
```

# 解读
# 对 `REFACTOR-REPRODUCTION-GUIDE` 的验证报告

## 总体结论

该重现方案**整体可执行**，步骤划分清晰，陷阱表覆盖了主要已知坑点，回归测试验收标准明确。但存在 **3 处数学/物理错误会直接导致重现结果偏离预期**、**4 处状态机与接口定义缺失**、**2 处工程实现细节需补充**。建议修正后再作为开发基线冻结。

---

## 逐步骤验证

### 环境准备

| 项目 | 验证结果 |
| :--- | :--- |
| GCC 13.2.0 + OpenMP 4.5 + AVX2 | ✅ 合理，MinGW-W64 13.2 支持 |
| **CMake 4.4.0** | ⚠️ **版本号可疑**。截至 2025 年 CMake 主流为 3.30+，4.0 刚发布不久，4.4.0 是否存在需确认。建议改为 **CMake ≥ 3.20** 或明确来源 |
| ZLIB / OpenCV 4.8 / cppzmq | ✅ 合理，`-DNF_DEPS=system` 需 CMakeLists 支持 `find_package` 回退 |
| PowerShell PATH 设置 | ✅ 可行，但需确保路径中无空格问题 |

---

### 步骤 1：双基线状态机模块

**合理项**：环形队列容量 16、双索引 `prev_idx_short/long`、阈值 `b_min=200 / b_target=300 / b_max=460` 与工程经验一致。

**需补充/修正**：

1. **`t_rel` 公式的坐标系未明确**  
   文档写 $\mathbf{t}_{rel} = R_{curr}^T(\mathbf{P}_{curr} - \mathbf{P}_{long})$。若 $\mathbf{P}$ 为车体系位置、$R$ 为车辆姿态，则 $R_{curr}^T(\cdot)$ 得到的是**当前帧坐标系下**的相对平移；若 $\mathbf{P}$ 为大地系位置，则 $R_{curr}^T$ 将其转到当前车体系。两种解释结果不同，**必须在头文件中注明 $\mathbf{P}$ 与 $R$ 的参考系**。

2. **缺少 `b_max` 超限处理**  
   文档只写了“位移达 `b_target` 时晋升”，未说明 $\Delta_{long} > b_{max}$ 时如何处理。必须补充：**超 `b_max` 时强制重置队列**（`prev_idx_long = prev_idx_short = curr_slot`），否则重叠视场不足会导致匹配失效率陡增。

3. **短基线未触发时的更新规则缺失**  
   若 $\Delta_{long} < b_{target}$，`prev_idx_short` 是否每帧更新？若不更新，短基线会越来越老，恢复时基线过长。建议明确：**每帧到达时短基线参考帧按固定策略滚动**（如保持最近有效帧或按时间衰减）。

4. **历史队列满时的覆盖策略**  
   环形队列 16 帧，满时覆盖最老帧。需在 `push` 中显式处理，并保证 `prev_idx_*` 不指向被覆盖槽位。

---

### 步骤 2：四重防御与不确定度

**合理项**：`NF_MODE_FROZEN_ROT = 6`、`nf_pt_in_foe` 判据 $(h\cdot d)^2 \ge |h|^2\cos^2(8^\circ)$、软权重函数形式均正确。

**需修正**：

1. **`nf_compute_spatial_uncertainty_cm` 沿用线性相加**  
   文档写 $f \cdot (\sigma_{pixel} b/d^2 + b\sigma_{odom}/d)$，这是**线性上界**而非方差传播。作为重现可保留（与原规范一致），但**函数注释必须声明“保守 $L_1$ 上界，非统计方差”**，避免下游误读为置信区间。

2. **`nf_occlusion_weight` 的 `excess` 定义缺失**  
   文档写 $1/(1+14\cdot excess)$，但未定义 $excess$。必须明确 $excess = \max(0, r/\tau - 1)$，否则实现者可能误用 $r-\tau$。

3. **量化与哨兵值未在步骤中落地**  
   函数应显式实现：`sigma_cm = round(sigma_spatial*0.1)`，钳位 `[1,254]`，当 $d\le0$ 或三角化奇异时返回 `255`。文档未写，易漏。

---

### 步骤 3：体素结构与高程压缩

**合理项**：`sigma_cm` 字段、零堆分配 `scratch_items`、`NFPointCloud` 定义顺序修正、`transform_columnar` 的 O(1) 前缀和设计均正确。

**需修正/补充**：

1. **`sigma_cm >>= 1` 过于乐观**  
   双基线验证通过后算术右移 1 位（除以 2）比统计最优（除以 $\sqrt{2}$）更激进，可能低估不确定度。作为重现沿用原规范可接受，但**必须在代码注释中标明“经验保守近似，待标定验证”**。

2. **`n_eff` 的类型与释放条件未定义**  
   `n_eff *= 0.2` 后“当 $n_{eff}<0.2$ 时物理释放该体素”——`n_eff` 是 float 还是整数？释放是置计数为零还是从 Hash 表删除？需在头文件中定义 `nf_f32 n_eff` 及释放语义。

3. **`scratch_items` 容量未定义**  
   `capacity * sizeof(ExportItem)` 中 `ExportItem` 结构体大小与 `capacity` 来源未说明。建议：`capacity = max_voxels_per_frame`（由网格尺寸与最大点数推导），并在 `nf_voxel_create` 中做溢出检查。

4. **`transform_columnar` 的并行安全**  
   若多线程写 `pc->points`，必须保证**按 `col` 递增顺序分段写入**，或单线程串行装配。文档未说明，易引入竞态。

---

### 步骤 4：车身掩码光栅化

**合理项**：OpenMP 并行射线相交法、`body_mask` 传入 LUT 构建。

**需补充**：

1. **多边形顶点坐标系未注明**  
   射线法要求多边形与查询点在同一坐标系。应明确：**多边形顶点在虚拟视口图像坐标系**，查询点为视口像素 $(p_x,p_y)$，而非车体系。

2. **mask 初始化与边界**  
   并行前需 `memset(mask, 0, w*h)`，且多边形顶点超出图像边界时射线法仍应正确（奇数/偶数规则不受影响，但需 `epsilon` 防除零，文档已含 `+ε`）。

---

### 步骤 5：Pipeline 修复与串联

**合理项**：移除 `d0/d1` 冗余旋转、视口平移按偏航角转车系、串接 FoE 与方差传播——方向正确。

**需修正（严重）**：

1. **物点反解公式仍继承原规范错误**  
   文档写“反解 `P`”，但未给出公式。若沿用原规范的 $\mathbf{P} = \mathbf{P}_{curr} - \mathbf{t}_{veh} + \mathbf{d}_1\cdot dist$，则物理错误（射线从当前光心发出，不应减去 $\mathbf{t}_{veh}$）。**必须明确修正为**：
   $$\mathbf{P}_{veh} = \mathbf{C}_{curr} + \mathbf{d}_1\cdot dist$$
   其中 $\mathbf{C}_{curr}$ 为当前帧相机光心在车体系的位置，$\mathbf{d}_1$ 为车体系单位射线。

2. **FoE 的 `apex` 参数用 `pose->t` 错误**  
   `nf_pt_in_foe(P, apex, dir, cos²)` 中 `apex` 应为**相机光心**，而非车辆平移向量 `pose->t`。应改为当前帧相机光心 $\mathbf{C}_{curr}$。`dir_t` 需由 $\mathbf{t}_{veh}$ 归一化得到。

3. **`nf_forward_range` 参数语义未定义**  
   `nf_forward_range(a, b, 1, t_veh, &dist, &okp)` 中 `1` 是什么？`a,b` 是两帧射线还是其他？需在头文件中给出完整签名与语义，否则无法实现。

---

### 步骤 6：CMake 健壮性

✅ 合理。ZLIB 探测回退机制必要，`${ZLIB_INCLUDE_DIR}` 加入 include 路径正确。

---

## 陷阱表验证

| 序号 | 验证结果 |
| :--- | :--- |
| 1 转两次 | ✅ 正确，但修复描述“平移向量左乘 $R_z(yaw)$”需补充：**射线保持车系，平移转车系后代入修正后的反解公式** |
| 2 typedef 冲突 | ✅ 正确 |
| 3 body_mask nullptr | ✅ 正确 |
| 4 sigma_cm 恒 255 | ✅ 正确 |
| 5 M_PI | ✅ 正确，MinGW C++11 严格模式确实不暴露 |
| 6 内存碎片 | ✅ 正确 |

**缺失陷阱**：未覆盖 **`b_max` 超限未重置**、**FoE apex 误用平移向量**、**物点反解符号错误**——这三项是比表中部分项更致命的坑，建议补入。

---

## 回归测试验证

命令与测试列表基本合理，但需确认：

1. `-DNF_DEPS=system` 等选项需与 CMakeLists 中的 `option()` 名称完全一致；
2. 9 个测试中 `unit_geometry` 需覆盖 `baseline_manager`，但步骤 1 新建的 `baseline_manager.cpp` 若未加入测试源列表，可能不被执行；
3. 验收标准写“必须 100% 通过”合理，但建议增加**关键断言说明**：如 `unit_voxel` 中验证 `zindex` 单调性、`unit_geometry` 中验证 FoE 锥内点被剔除、`integration_pipeline` 中验证体素坐标与真值偏差中位数 ≤ 150 mm。

---

## 关键问题汇总

| 编号 | 位置 | 问题 | 严重度 | 修正建议 |
| :--- | :--- | :--- | :--- | :--- |
| 1 | 步骤 5 | 物点反解公式继承原规范物理错误 | **致命** | 明确 $\mathbf{P}=\mathbf{C}_{curr}+\mathbf{d}_1\cdot dist$ |
| 2 | 步骤 5 | FoE apex 误用 `pose->t` | **高** | 改为相机光心 $\mathbf{C}_{curr}$，`dir_t` 归一化 |
| 3 | 步骤 2 | 不确定度线性相加未声明为 $L_1$ 上界 | 中 | 注释声明，避免误读 |
| 4 | 步骤 1 | 缺 `b_max` 超限重置、短基线更新规则 | 中 | 补充状态机迁移表 |
| 5 | 步骤 3 | `sigma >>=1` 过于乐观、`n_eff` 类型未定义 | 中 | 注释经验近似；定义 `nf_f32 n_eff` 及释放语义 |
| 6 | 步骤 3 | `scratch_items` 容量与 `transform_columnar` 并行安全未说明 | 中 | 明确容量推导与写入顺序 |
| 7 | 步骤 2 | `excess` 定义缺失 | 低 | 明确 $excess=\max(0,r/\tau-1)$ |
| 8 | 步骤 4 | 多边形坐标系未注明 | 低 | 标注为视口图像坐标系 |

---

## 最终判断

该重现方案**可以作为 NFOCC v0.6.0+ 的开发操作指南**，但**步骤 5 的物点反解公式与 FoE apex 参数必须在编码前修正**，否则重现结果会继承原规范中的坐标系错误，导致 3D 点云系统性偏移。建议在文档头部增加“**数学修正公告**”，将上述第 1、2 项修正显式写入，并同步更新陷阱表。其余问题为定义澄清与工程近似声明，可在实现过程中通过头文件注释与测试断言补足。修正后即可冻结为 **v1.0.0-Reproduction-Validated**。