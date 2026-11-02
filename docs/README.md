# NFOCC 开发文档集（NFOCC-DOC）

面向实现的规范文档。**规则：每一处代码符号（函数 / 结构体 / 状态机 / 参数）在本目录内只有一个定义处，
其余位置只能引用、不得重复定义。** 验收条件必须可执行（给出命令或可判定的数值）。

体系代号：**NFOCC-DOC v0.1**。任何一处的修改都要同步改 `APPENDIX-CHANGELOG`。

---

## 阅读顺序

| 顺序 | 文档 | 回答什么问题 |
|---|---|---|
| 0 | `PROJECT-STATUS.md` | **进度唯一入口**：与原始方案对照、缺口、后续计划 |
| 0b | `SPEC-DATASET.md` | hanqtech 真实数据接入：内参/外参/环带语义/实测 |
| 0c | `SPEC-CONVENTION.md` | **坐标系与 t_cam 语义（唯一权威）** |
| 0d | `REV-001-002-IMPLEMENTATION.md` | REV-001/002 实施记录：D1-D4、双基线、柱体排布 |
| 1a | `0_SKILL.txt` | 近场鱼眼感知开发技能（参考量产系统六模块精要） |
| 1b | `1_NFOCC待办.txt` | 与量产系统的六项差距与弥合路径 |
| 1c | `2_SPEC-ALGORITHM-DERIVATION.md` | 算法改进的数学依据（含自带验证报告） |
| 1d | `3_REFACTOR-REPRODUCTION-GUIDE.md` | 工程改进复现指南 |
| 1e | `4_REFACTOR-LOG-ACTIONS.md` | REV-001/REV-002 修改记录 |
| 2a | `VALIDATION-REPORT.md` | 三份文档合入审计 + 真实数据各阶段验证 |
| 2b | `report/ARCH-REPORT.md` | 架构/Pipeline/流程解析（PDF: `report/NFOCC-架构与流程解析.pdf`） |
| 2c | `DECISION-DEPTH-ACCURACY.md` | **精度目标决策**：150°/1024, 5-15km/h, 参数与实测 |
| 2d | `PROBE-FINDINGS.md` | **视差探针结论**：前后视口 90x120 不自洽，待决策 |
| 2e | `PROBE-RESULTS.md` | **探针实测**：C 方案 557x966 落地，六视口视差误差 |
| 2f | `ROADMAP.md` | **研发计划**：按当前代码重盘的缺口与 6 阶段迭代 |
| 2g | `PLAN-DEFENSE-INTEGRATION.md` | **防御接入**：D1/D2/D4/双基线 的外部依赖与方案 |
| 2h | `PLAN-ONLINE-CALIBRATION.md` | **自校准方案**：三种路线对比与可行性 |
| 2i | `PLAN-CLUSTERING.md` | **聚类与动静分离**：算法选型与数据规模分析 |
| 2j | `SPEC-DEPTH-ACCEPTANCE.md` | **验收标准**：150mm 的分距离段适用范围 |
| 1 | `SPEC-COORD.md` | 点、向量、矩阵、旋转到底怎么定义？谁是谁的转置？ |
| 2 | `SPEC-CAMERA.md` | 4 路鱼眼的内参模型（KB）、图像格式、有效像素区 |
| 2a | `SPEC-CALIB-CFG.md` | 相机内外参配置文件格式与默认值（f=550、车 5000×2000） |
| 3a | `SPEC-LUT.md` | 视口 LUT 的构建链、Q12.4 存储、有效性四道检查 |
| 3b | `SPEC-FORWARD.md` | **前向运动的径向膨胀解析测距**（前向工况的唯一正解） |
| 3 | `SPEC-EPIPOLAR.md` | 运动双目的极线/视差几何，重定向能做到什么、做不到什么 |
| 3c | `PH1-ACCEPTANCE.md` | PH-1 验收协议（引擎契约 / 三角化 / 产品链路） |
| 4 | `SPEC-ENGINE.md` | 视差引擎接口（SGBM 与模型双实现），输入输出与失效语义 |
| 5 | `SPEC-POSE.md` | 位姿接入：轮速/IMU → `NFPose` 的积分链与误差 |
| 6 | `SPEC-ICD.md` | 所有结构体、函数签名、失效语义（唯一的接口定义处） |
| 7 | `SPEC-BASELINE-SM.md` | 基线状态机：状态表、迁移、输出契约 |
| 8 | `SPEC-VOXEL.md` | 体素融合（Hash + CMA）、zMap/zIndex 压缩 |
| 8a | `SPEC-MASK.md` | 车身 ROI 掩码：源头剔除自车反射 |
| 8b | `SPEC-OCCLUSION.md` | 双向光流遮挡检测与软性权重 |
| 8c | `SPEC-FOE.md` | FoE 发散锥过滤（逐相机） |
| 9 | `SPEC-BUILD.md` | 工程骨架：语言、构建、目录、线程、内存、日志 |
| 9a | `ARCH-DECISIONS.md` | C++11 落地时的架构决策（ADR-001..008），与 SPEC-BUILD 的冲突处理 |
| 9b | `ADR-009-zltoolkit-port.md` | ZLToolKit 移植取舍：哪些用、哪些不用、为什么 |
| 9c | `BUILD-TOOLCHAIN.md` | 三套目标构建：macOS / Linux PC / Aarch64 交叉，含国内网络注意 |
| 10a | `SPEC-UPSTREAM.md` | 上游接入：图像/位姿结构、时间片(5Hz/200ms)、失步语义 |
| 10b | `SPEC-DOWNSTREAM.md` | 下游输出：消息格式、冻结也发、下游如何判系统状态 |
| — | `cpp/include/nfocc/nfocc_match.h` | 前向对应点提取（径向膨胀 + NCC），实测 0.045px |
| — | `cpp/include/nfocc/nfocc_viewplan.h` | 6 视口规划表：ML/MR 0°，FV/RV ±45° |
| 10 | `PARAM.md` | 参数手册：每个参数的唯一符号、默认值、反推依据、整定方法 |
| 11 | `SPEC-VERIFY.md` | 每个代码单元的验收条件与验收方法 |
| 12 | `SPEC-P0-SYNTH.md` | P0 合成数据生成器规范 |
| 13 | `PHASE.md` | 阶段划分、依赖关系、当前阻塞项 |
| 附 | `APPENDIX-OPEN.md` | 未决项：每题给选项，等用户决定 |
| 附 | `APPENDIX-CHANGELOG.md` | 变更记录 + 本轮所有关键推导与"不可信项"清单 |

自检：`python tools/docs_check.py`（检查交叉引用、参数唯一性、编号、索引完整性）。

---

## 已定约束（用户拍板，不再讨论）

| 项 | 值 |
|---|---|
| 相机 | 4 路环视鱼眼：FV / RV / ML / MR |
| 图像格式 | 单 Y 通道，8 bit |
| 图像分辨率 | 1920 × 1536（W × H），四路一致 |
| 内参模型 | **KB 鱼眼**（Kannala-Brandt，θ 的 1/3/5/7/9 次多项式） |
| 车辆坐标系 | **ISO 8855**（X 前、Y 左、Z 上） |
| 图像/相机坐标系 | **OpenCV**（x 右、y 下、z 前） |
| 计算平台 | RK3588、Axera M57 |
| 视差引擎 | SGBM（纯 CV）与模型方案（StereoNetPlus 类），**接口兼容、可替换** |
| 硬件 SDE | **不使用**（TDA4 SDE 等专用加速器排除） |
| 原始 NFOCC 源码 | **不可得**，全部模块按本文档重写 |
| P0 数据 | 合成数据；后续替换为实车数据 |

---

## 由上述约束直接导出的量（已算，文档各处引用这些值）

| 量 | 值 | 推导 |
|---|---|---|
| 单帧图像大小 | 1920×1536×1 B = **2.95 MB** | 4 路 = 11.8 MB/帧 |
| 图像长宽比 | 1.25（5:4） | 1536/1920 |
| 中心角分辨率（等距近似） | f ≈ 579 px/rad @ 190° 水平 FOV；611 @ 180° | `f = (W/2)/tan⁻¹` 反推 |
| 单像素角分辨率 | ≈ **0.099°/px**（@190° FOV） | 190°/1920 |
| 展开视口有效焦距 | 见 `PARAM.md` 的 `P.F_VIEW` | 由视口张角与栅格宽度决定 |
| 近场最小视差（b=100 mm，Z=5 m，f=579） | 11.6 px | `d = f·b/Z` |
| 近场最大视差（b=500 mm，Z=0.5 m，f=579） | 579 px | 同上 |
| 视差动态范围 | **约 50:1** | 这决定了必须分档基线，见 `SPEC-BASELINE-SM.md` |

> 视差动态范围 50:1 是整套设计的核心矛盾。任何单一 `numDisparities` 都不可能同时覆盖
> 0.5 m 与 5 m，这是 `SPEC-BASELINE-SM` 必须分档的算术依据，不是工程偏好。

---

## 文档纪律

1. **符号唯一**：一个物理量只有一个符号，在 `PARAM.md` 的符号表登记。文档别处出现新符号即为错误。
2. **单位显式**：所有长度默认 mm，派生量在符号后缀标注（`_mm`、`_px`、`_rad`）。
3. **矩阵约定只有一套**：见 `SPEC-COORD.md` §2，禁止别处另立。
4. **失效必须有名字**：任何函数失败不能返回 NaN 了事，必须返回 `SPEC-ICD.md` 里登记的 `NFStatus` 码。
5. **验收可执行**：`SPEC-VERIFY.md` 里每条验收都对应一个可运行命令或可判定的数值，不接受"目测正常"。

---

## 未决项（待用户决定，见 `APPENDIX-OPEN.md`）

在未决项落定前，相关实现按文中标注的**默认值**推进，不阻塞。
