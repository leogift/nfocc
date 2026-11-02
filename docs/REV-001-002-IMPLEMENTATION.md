# REV-001 / REV-002 实施记录

依据 `docs/4_REFACTOR-LOG-ACTIONS.md` 与 `docs/2_SPEC-ALGORITHM-DERIVATION.md`，
本文件记录**实际落地**的内容、与规范的偏离、以及踩到的坑。

---

## 已实现（9 个 API）

| API | 位置 | 说明 |
|---|---|---|
| `nf_pt_in_foe` | `nfocc_geometry.h` | D3 FoE 发散锥代数过滤 |
| `nf_occlusion_weight` | `nfocc_geometry.h` | D2 双向一致性软权重 |
| `nf_compute_spatial_uncertainty_cm` | `nfocc_geometry.h` | 空间不确定度（**L2**，见偏离 1） |
| `nf_voxel_insert_with_sigma` | `nfocc_voxel.h` | 带不确定度的 CMA 插入（取最小） |
| `nf_voxel_verify_cross` | `nfocc_voxel.h` | D4 双基线交叉验证（**σ/√2**，见偏离 2） |
| `nf_baseline_manager_*` | `nfocc_baseline.h` | 自适应多帧历史 + 并行双基线状态机 |
| `nf_voxel_transform_columnar` | `nfocc_voxel.h` | 柱体连续排布，支撑 O(1) 访问定理 |
| `nf_mask_rasterize_polygon` | `nfocc_lut.h` | D1 车身掩码光栅化 |
| `NF_MODE_FROZEN_ROT` | `nfocc_geometry.h` | 急转冻结模式 |

测试：`tests/unit/test_algo_rev.cpp`，**12 用例 / 69 断言全过**。

---

## 与规范的偏离（三处，都是有理由的）

### 偏离 1：空间不确定度用 L2 而非 L1

规范 3.2 写的是：

```
σ_spatial = max(|f|,1.0) · ( σ_pixel·b/d² + b·σ_odom/d )
```

这是两项**线性相加**（L1），比标准方差传播大 √2 倍。规范标题却叫
"全微分方差传播模型"——名实不符。

**本实现用 L2**：

```
σ_Z = √[(f·b/d²·σ_d)² + (f/d·σ_b)²]
```

理由：规范自带的验证报告（`2_SPEC-ALGORITHM-DERIVATION.md` 第 228 行）
已明确指出这是错误并要求改为平方和开根。实现取修正后的形式。

实测差异：`f=184.75, b=273, d=8, σ_d=0.5, σ_odom=0.02`
- L2 → **41 cm**
- L1 → 52 cm

### 偏离 2：D4 方差减半用 ÷√2 而非 ÷2

规范写 `σ_cm >>= 1`（除以 2）。两次独立等精度观测的信息融合：

```
1/σ² = 1/σ₁² + 1/σ₂²   =>   σ = σ₁/√2  (等方差)
```

除以 2 比统计最优更乐观，会低估不确定度。本实现用 `/1.41421356`。

### 偏离 3：双基线补两条规范遗漏的规则

规范 2.3 没说清两件事，不补会导致错误：

**a) 必须设 b_max 超限处理。** 高速时 `Δ_long` 可能到 1 m，重叠视场不足、
匹配陡崩。实现：`Δ_long > b_max` 时先尝试降级用短基线，短基线也不行才冻结，
两种情况都重置队列。

**b) `Δ_long < b_target` 时短基线如何更新。** 若短基线不随帧推进，
队列永远停在初始状态，双基线无法晋升。实现：**每帧 push 即更新短基线参考帧**。

---

## 实施中踩到的坑（都是实测出来的）

### 坑 1：短基线指向了当前帧自己

最初把 `push` 写成 `idx_short = 当前帧`。结果 decide 时短基线长度是
`|cur - cur| = 0`，永远达不到 b_min，只能靠长基线产出；而长基线刚晋升成
当前帧后，当帧必然冻结。

实测 150 mm/帧步进：

```
改前: frame 2 STEREO(300) -> frame 3 FROZEN -> frame 4 STEREO(300)  // 交替
改后: frame 2 STEREO(300) -> frame 3 STEREO(300) -> frame 4 STEREO(300)
```

修正：短基线指向"上一个 head"，即相邻两帧。

### 坑 2：测试里坐标不落同一体素

索引是 `ix = floor((x_max - x) / 200)`，边界很隐蔽。
`x=1000` 落在 ix=5，`x=1000.5` 落在 ix=4 —— 差了 0.5 mm 就跨格。
测试选点必须先手算索引区间。

### 坑 3：σ 与基线的单调性判据写反

我最初写"基线越长越准 → 不确定度越小"，测试失败后才发现：
同**视差**下 `Z = f·b/d`，b 翻倍则 Z 翻倍，2% 相对基线误差在绝对量上也翻倍。
所以**同视差下基线越长不确定度越大**。"基站长更准"是**同一距离**下的结论
（此时 d 与 b 成正比）。这是物理事实，不是实现错误。

### 坑 4：核心库禁用 STL 容器

`nf_voxel_transform_columnar` 最初用了 `std::vector`，
核心库带 `-fno-exceptions`，改用 `malloc/free`。

---

## 尚未接入的部分

这些 API 目前是**独立可用**，但还没接进 `pipeline.cpp` 的主链路：

| 待接入 | 影响 |
|---|---|
| D3 FoE → 三角化前过滤 | 直行时前向拉丝幽灵点仍未剔除 |
| D4 交叉验证 → 双基线结果融合 | 双基线目前只有状态机，未真正跑两路 |
| sigma_cm → 点云输出 | `NFPoint.sigma_cm` 字段存在但主链路填 0 |
| D2 权重 → SGBM 后处理 | 需先有反向视差图 |
| 双基线管理器 → 替代 nf_gate_decide | 现有门控仍是单帧无状态 |
| D1 车身掩码 → nf_lut_build | 接口已通，`nfgocc_run` 仍传 nullptr |

**下一步**应优先接 D3 和 sigma_cm —— 它们不依赖额外数据，改动面小，
且直接改善点云质量。

---

## 回归状态

```
ctest: 13/14 通过（depth_accuracy 未过，既有问题，与本次改动无关）
test_algo_rev: 12/12 用例，69 断言，0 失败
干净构建: 零错误（独立 tmpdir 验证）
```
