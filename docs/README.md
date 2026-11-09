# NFOCC 开发文档集（NFOCC-DOC）

面向实现的规范文档。**规则：每一处代码符号（函数 / 结构体 / 状态机 / 参数）在本目录内只有一个定义处，
其余位置只能引用、不得重复定义。** 验收条件必须可执行（给出命令或可判定的数值）。

体系代号：**NFOCC-DOC v0.3**（2026-11-03 同步工程现状）。任何一处的修改都要同步改 `APPENDIX-CHANGELOG`。

---

## 现状速览

| 项 | 值 | 来源 |
|---|---|---|
| 代码规模 | 11004 行（`cpp/src` + `cpp/include`） | `wc -l` |
| 测试 | ctest 20/21；`depth_accuracy` 未达标 | 见下"已知未达标" |
| 单元测试 | `test_cluster` 10/10、`test_d4_baseline` 6/6、`test_algo_rev` 12/12、`test_vcam_bridge` 5/5、`test_voxel_filter` 10/10、`test_newmod` 10/10、`test_zmq_io` 8/8 | 各测试文件 |
| 文档自检 | `python3 tools/docs_check.py` 42 篇全绿 | — |
| 相机后端 | **vcam 静态库**（`cpp/third_party/vcam`），投影/反投影唯一路径 | `BUILD-TOOLCHAIN.md` |
| 真实数据 | hanqtech 1280×966 radial_poly；3 组 × 4 路，**含 ego_distance / ego_yawRate 真值** | `SPEC-DATASET.md` |

### 已知未达标

| 项 | 状态 | 说明 |
|---|---|---|
| `test_depth_accuracy` | **失败** | 侧行 1-2/2-3/3-4/4-5m 中位 291/256/433/896mm（目标 150/200/320/320）。合成正弦纹理的周期性与立体匹配唯一性天然冲突，见 `PROBE-RESULTS.md` |
| 前向径向匹配性能 | **回退** | 真实直行数据走 FORWARD 分支，3429 ms/帧（STEREO 路径 35 ms）。径向匹配是手写标量循环，未做降采样/并行 |
| 150mm@5m | **未证实** | 需要 σ_d ≤ 0.34 px，理论不可达。分距离段的验收标准见 `SPEC-DEPTH-ACCEPTANCE.md` |
| 聚类实用性 | **受限** | 真实数据点数随工况 195–2077/帧波动，聚不出物体。见 `CLUSTERING-IMPLEMENTATION.md` |
| 负深度 | **未查明** | 真实数据点云出现 z=-498mm（点在相机后方），物理不可能 |

---

## 阅读顺序

### 第一层：决策与计划（先读这些）

| 顺序 | 文档 | 回答什么问题 |
|---|---|---|
| A0 | `NEW_MOD.md` | **产品化演进**：双基线/不确定度/D4/zMap 压缩 + 已知限制（必读 §0） |
| A0b | `TODO.md` | **待办**：当前阻塞项与外部依赖 |
| A0c | `IMPROVEMENT-PLAN.md` | **改进方案**：P0–P4 优先级、0_SKILL 六模块映射、本轮已落地修复 |
| A1 | `ROADMAP.md` | **研发计划**：按当前代码重盘的缺口与 6 阶段迭代 |
| A2 | `SPEC-DEPTH-ACCEPTANCE.md` | **验收标准**：150mm 的分距离段适用范围 |
| A3 | `DECISION-DEPTH-ACCURACY.md` | **精度决策**：150°/1024, 5-15km/h 的参数取舍与实测 |
| A4 | `APPENDIX-OPEN.md` | 未决项：每题给选项，等用户决定 |

### 报告（PDF）

| 文件 | 内容 |
|---|---|
| `report/NFOCC-项目总览.pdf` | 9 页：项目定位 / 业务逻辑 / 架构 / 数据流 / 算法 / 亮点 / 局限 / 进展 / 待办 / 选型权衡 |
| `report/REPORT-FULL.md` | 上者正文源 |
| `report/NFOCC-项目报告-2026-11.pdf` | 31 页：业务/架构/数据流/算法/性能/局限/进展，含 15 张矢量图 + 5 张实测图 |

生成流程：改 `report/REPORT-FULL.md` → 重跑 `tools/build_report_full.py`（程序化 SVG + pandoc + Chromium 打印）。
新报告：`docs/report/v2/report.md` + `tools/report_v2/build_report.py`（同上，另含 PyMuPDF 页脚/页码）。
7 张图全部内联 SVG，保持矢量；不依赖任何图片预处理脚本。

### 第二层：已落地的实现记录

| 顺序 | 文档 | 回答什么问题 |
|---|---|---|
| B1 | `CLUSTERING-IMPLEMENTATION.md` | **聚类落地**：纯网格哈希选型（六次迭代）、DBSCAN 实测、动静分离 |
| B2 | `REV-001-002-IMPLEMENTATION.md` | D1–D4、双基线、柱体排布的实施记录（**来源文档已删，仅存结论**） |
| B3 | `PLAN-DEFENSE-INTEGRATION.md` | 防御接入：哪些已接、哪些缺外部支持 |
| B4 | `VALIDATION-REPORT.md` | 三份外部文档合入审计（**审计对象已删，仅存结论**） |

### 第三层：规范（实现依据）

| 顺序 | 文档 | 回答什么问题 |
|---|---|---|
| 1 | `SPEC-COORD.md` | 点、向量、矩阵、旋转怎么定义？谁是谁的转置？ |
| 2 | `SPEC-CONVENTION.md` | **坐标系与 t_cam 语义（唯一权威）** |
| 2b | `SPEC-CAMERA.md` | 4 路鱼眼的内参模型、图像格式、有效像素区 |
| 2c | `SPEC-CALIB-CFG.md` | 相机内外参配置文件格式与默认值 |
| 2d | `SPEC-DATASET.md` | hanqtech 真实数据接入：内参/外参/环带语义 |
| 3a | `SPEC-LUT.md` | 视口 LUT 的构建链、Q12.4 存储、有效性四道检查 |
| 3b | `SPEC-FORWARD.md` | **前向运动的径向膨胀解析测距** |
| 3c | `SPEC-EPIPOLAR.md` | 运动双目的极线/视差几何 |
| 4 | `SPEC-ENGINE.md` | 视差引擎接口（SGBM 与模型双实现） |
| 5 | `SPEC-POSE.md` | 位姿接入：轮速/IMU → `NFPose` 的积分链 |
| 6 | `SPEC-ICD.md` | 所有结构体、函数签名、失效语义（唯一接口定义处） |
| 7 | `SPEC-BASELINE-SM.md` | 基线状态机：状态表、迁移、输出契约 |
| 8 | `SPEC-VOXEL.md` | 体素融合（Hash + CMA）、zMap/zIndex 压缩 |
| 8a | `SPEC-MASK.md` | 车身 ROI 掩码：源头剔除自车反射 |
| 8b | `SPEC-OCCLUSION.md` | 双向光流遮挡检测与软性权重 |
| 8c | `SPEC-FOE.md` | FoE 发散锥过滤（逐相机） |
| 10a | `SPEC-UPSTREAM.md` | 上游接入：图像/位姿结构、时间片、失步语义 |
| 10b | `SPEC-DOWNSTREAM.md` | 下游输出：消息格式、冻结也发 |
| 10c | `SPEC-P0-SYNTH.md` | P0 合成数据生成器规范 |
| 10 | `PARAM.md` | 参数手册：唯一符号、默认值、反推依据 |
| 11 | `SPEC-VERIFY.md` | 每个代码单元的验收条件与方法 |

### 第四层：构建与工具

| 顺序 | 文档 | 回答什么问题 |
|---|---|---|
| C1 | `BUILD-CROSSPLATFORM.md` | **构建调试**：macOS/Linux 统一 CMake、三方依赖、vcam 后端 |
| C2 | `SPEC-BUILD.md` | 工程骨架：语言、目录、线程、内存、日志 |
| C3 | `BUILD-TOOLCHAIN.md` | 三套目标构建：macOS / Linux / Aarch64 交叉 |
| C4 | `ARCH-DECISIONS.md` | C++11 落地时的架构决策（ADR-001..008） |
| C5 | `ADR-009-zltoolkit-port.md` | ZLToolKit 移植取舍 |
| C6 | `PHASE.md` | 阶段划分、依赖关系、当前阻塞项 |

### 第五层：过程记录（历史存档，结论有效但不可回查来源）

| 文档 | 说明 |
|---|---|
| `PROBE-FINDINGS.md` | 视差探针诊断：前后视口 90×120 不自洽 |
| `PROBE-RESULTS.md` | 探针实际结果：六视口视差误差 |
| `PLAN-ONLINE-CALIBRATION.md` | 自校准方案三种路线（**未实现**，方案有效） |
| `APPENDIX-CHANGELOG.md` | 变更记录 + 关键推导与"不可信项"清单 |

### 已删除（2026-11-02）

| 文档 | 删除理由 |
|---|---|
| 1_NFOCC待办.txt、2_SPEC-ALGORITHM-DERIVATION.md、3_REFACTOR-REPRODUCTION-GUIDE.md、4_REFACTOR-LOG-ACTIONS.md | 外部方案文档，其内容已并入本目录规范或成为历史记录 |
| PH1-ACCEPTANCE.md | 验收对象是已删除的 Python 原型 `nfocc_ph1` |
| PROJECT-STATUS.md | 停在 11-01，被 `ROADMAP.md` 取代 |
| PLAN-CLUSTERING.md | 调研稿，已被 `CLUSTERING-IMPLEMENTATION.md`（落地结果）取代 |
| `docs/report/` 整目录 | 架构报告 PDF 与图 |
| `nfocc_ph0/`、`nfocc_ph1/` | Python 原型，已被 C++ 实现取代 |
| `docs/*.png`（viewplan_bev_* 等） | 对应脚本产物，可重新生成 |

---

## 由当前约束直接导出的量

| 量 | 值 | 推导 |
|---|---|---|
| 源图 | 1280 × 966，单 Y 通道 8 bit | hanqtech 实测 |
| 内参 | `radial_poly` ρ(θ)=Σkᵢθⁱ，k1≈340 px/rad | `SPEC-DATASET.md` |
| 左右视口 | 1024 × 475，150°H，f=137.2 | `nfocc_default.h` |
| 前后视口 | 590 × 1024，90°H，f=295.0 | 同上（90°×120° 的几何解） |
| 基线区间 | b=278 / 500 / 833 mm（5 / 9 / 15 km/h @5Hz） | `DECISION-DEPTH-ACCURACY.md` |
| 视差动态范围 | `numDisp=256` 容量支持 b≤933 mm | `B_MAX = numDisp·Z_MIN/f` |

> 分档基线的算术依据见 `SPEC-BASELINE-SM.md`：单一 `numDisparities`
> 不可能同时覆盖 0.5 m 与 5 m。

---

## 文档纪律

1. **符号唯一**：一个物理量只有一个符号，在 `PARAM.md` 的符号表登记。
2. **单位显式**：长度默认 mm，派生量在符号后缀标注（`_mm`、`_px`、`_rad`）。
3. **矩阵约定只有一套**：见 `SPEC-COORD.md`，禁止别处另立。
4. **失效必须有名字**：函数失败必须返回 `SPEC-ICD.md` 登记的 `NFStatus` 码。
5. **验收可执行**：每条验收对应一个可运行命令或可判定的数值，不接受"目测正常"。
6. **"能编译"不算完成**：交付要给出真实执行的输出，见 `SPEC-VERIFY.md`。

自检：`python3 tools/docs_check.py`（检查交叉引用、参数唯一性、编号、索引完整性）。
