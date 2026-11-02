# NFOCC

多相机鱼眼近场稠密深度与体素重建系统。4 路环视鱼眼 + 运动双目，覆盖 0.5–5 m，
输出点云与 2D 高程柱（zMap）。

C++11 产品实现，面向 RK3588 / Axera M57 边缘 NPU。

---

## 当前状态（2026-11-02）

| 项 | 状态 |
|---|---|
| 代码规模 | 约 7900 行（`cpp/src` + `cpp/include`） |
| 构建 | macOS / Linux 均通过，aarch64 交叉编译产物已出（未上板） |
| 测试 | `ctest` **17/18**，唯一失败 `depth_accuracy`（见「已知问题」） |
| 文档 | `docs/` 40 篇，`python3 tools/docs_check.py` 全绿 |

### 已实现并接入主链

相机模型（KB / 径向多项式）、4 路鱼眼外参、6 视口 LUT、SGBM 视差、
前向径向测距、三角化、体素融合（Hash + CMA）、门控、FoE 锥过滤、
σ 传播与落库、车身掩码 D1、ZMQ 零拷贝收发、网格哈希 + DBSCAN 聚类、
动静分离（位姿补偿 + 点级速度 + 簇级投票）、双基线状态机、里程计积分。

### 已实现但未接入主链

| 项 | 原因 |
|---|---|
| D2 双向一致性 | 需引擎跑两次匹配，耗时 1.8 倍 |
| D4 交叉验证 | 依赖双基线晋升，需真实定位数据 |
| 双基线状态机 | pipeline 仍用旧的 `nf_gate_decide` |
| 在线自校准 | 方案已定（地平面单应 + IMU），未实现 |

### 已知问题

**`test_depth_accuracy` 失败。** 合成场景只有地面 + 一面墙，纹理是规则周期
函数，SGBM 锁错周期，实测视差误差 14–42 px。这是**场景缺陷不是算法能力**——
墙面理论视差 140.8 px 时 SGBM 给出 2.2 px，属于匹配失败。修法见
`docs/PROBE-RESULTS.md`：给场景加随机相位纹理后重跑探针。

**150 mm @ 5 m 未证实。** 5 km/h 时理论需要 σ_d ≤ 0.34 px，SGBM 典型值
0.3–0.5 px。分距离段的验收标准见 `docs/SPEC-DEPTH-ACCEPTANCE.md`。

**聚类在真实数据上实用性受限。** 帧 164 只有 1256 点 → 82 簇、噪声 926、
每簇 4 点。密度不足以聚出物体，根因是上面的精度问题。算法正确性由合成
场景的 10 条用例守着。

---

## 快速开始

### 依赖

| 库 | 版本 | 必需 |
|---|---|---|
| CMake | ≥ 3.16 | 是 |
| C++11 编译器 | clang / gcc | 是 |
| OpenCV | 5.0（SGBM 模块） | 否，`NF_WITH_OPENCV=OFF` 可关 |
| ZeroMQ | 4.3.5 | 否 |
| onnxruntime | 1.31 | 否 |
| Eigen / OpenBLAS / libomp / zlib | — | 核心库只需 zlib |

### 构建

```bash
cd cpp

./scripts/build.sh host       # 本机 Release + 跑测试
./scripts/build.sh debug      # -O0 -g + compile_commands.json
./scripts/build.sh core       # 只编核心库（零三方依赖）
./scripts/build.sh aarch64    # 交叉编译 ARM
./scripts/build.sh clean
```

`scripts/build.sh` 支持三种依赖来源：

```bash
cmake -S . -B build -DNF_DEPS=vendored   # 默认，优先 cpp/third_party/
cmake -S . -B build -DNF_DEPS=managed    # 系统路径 / find_package
cmake -S . -B build -DNF_DEPS=off        # 不引入三方库
```

可选 vcam 后端（用外部 virtualcamera 工程的相机模型）：

```bash
NF_VCAM_DIR=/Volumes/Workspace/virtualcamera/cpp ./scripts/build.sh host
```

详见 `docs/BUILD-CROSSPLATFORM.md`。

### 测试

```bash
cd cpp

# 全部（含已知失败的 depth_accuracy）
ctest --test-dir build/host --output-on-failure

# 排除已知失败
ctest --test-dir build/host -E depth_accuracy

# 单个测试
./build/host/tests/test_cluster
./build/host/tests/test_zmq_io
ctest --test-dir build/host -R unit_cluster -V
```

| 测试 | 覆盖 |
|---|---|
| `test_geometry` / `test_camera` / `test_voxel` / `test_lut` | 基础几何与数据结构 |
| `test_algo_rev` | D1–D4 等 12 个算法的独立验证 |
| `test_cluster` | 八叉树半径查询（对拍暴力）、DBSCAN、动静分离 |
| `test_d4_baseline` | 里程计积分、双基线状态机（含负对照） |
| `test_zmq_io` | 零拷贝生命周期、尺寸白名单、所有权转移 |
| `test_vcam_bridge` | vcam 后端与原生实现的一致性 |
| `test_pipeline` / `test_depth_accuracy` | 端到端 |
| `system_run_synthetic` | 系统级冒烟 |

### 运行

```bash
cd cpp

# 真实数据（hanqtech 鱼眼，结构见 docs/SPEC-DATASET.md）
./build/host/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164 --frames 3 --stats

# 关闭车身掩码做负对照（点数应回升）
./build/host/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164 --frames 3 --stats --no-body-mask

# 中间结果可视化（Y 通道 / 视口 / 视差 / BEV / 车身掩码 / 聚类）
./build/host/nfocc_viz ../datasets/fisheye 164 ../viz
```

`nfocc_run` 常用参数：

| 参数 | 说明 |
|---|---|
| `--frames N` | 处理帧数 |
| `--ws-frame N` | 起始帧号 |
| `--speed KMH` | 车速，驱动 yaw_rate 积分 |
| `--yaw-rate DPS` | 偏航率 (deg/s) |
| `--hfov DEG` | 侧视口水平张角（默认 150） |
| `--d4` | 启用双基线 |
| `--no-body-mask` | 关闭 D1 车身掩码 |

### 验证

```bash
# 文档一致性与交叉引用
python3 tools/docs_check.py

# 文档声明 vs 代码实况（含"说已接入的确实有调用"的双向校验）
./scripts/verify_docs.sh

# ZMQ 零拷贝（含负对照与内存不泄漏）
./scripts/verify_zmq.sh
```

---

## 核心设计

**虚拟相机视口**：所有视口按 pitch=0、roll=0 构造，只有 yaw。真实外参
只决定「这个车系方向落在哪张鱼眼的哪个像素」，不参与视口朝向。

| 视口 | 尺寸 | 水平张角 | f (px/rad) |
|---|---|---|---|
| 左右 (ML/MR) | 1024 × 475 | 150° | 137.2 |
| 前后 (FV/RV ±45°) | 590 × 1024 | 90° | 295.0 |

前后视口用 90° 而非 150°，是因为 f 直接决定深度精度：同样基线 278 mm，
f=295 的 5 m 处视差 16.4 px，f=137 只有 7.6 px。

前后各两个（±45°）是为了直行时获得**横向基线**——单前向视口横向基线为 0，
视差深度在纯前向平移下结构上消失。

**坐标约定**：车体 ISO 8855（X 前 Y 左 Z 上）、相机 OpenCV（x 右 y 下 z 前）、
虚拟视口以其自身基向量表达。`t_cam` 用**视口系**，唯一权威定义在
`docs/SPEC-CONVENTION.md`。

**基线分档**：b = 278 / 500 / 833 mm 对应 5 / 9 / 15 km/h @5 Hz。
视差动态范围决定必须分档——单一 `numDisparities` 无法同时覆盖 0.5 m 与 5 m。

---

## 目录

```
cpp/
  include/nfocc/     公共头（核心库零第三方依赖）
  src/               实现
    camera/          相机模型、vcam 桥接
    lut/             视口 LUT、视口规划
    geometry/        三角化、门控、FoE、径向测距、场景
    engine/          SGBM / 模型双引擎
    voxel/           体素融合（Hash + CMA）
    cluster/         网格哈希索引 + DBSCAN + 动静分离
    baseline/        双基线状态机、里程计积分
    io/              文件 / hanqtech / ZMQ（零拷贝）
    pipeline/        主链
    util/            线程池、帧池、日志、计时
  tools/             nfocc_run / nfocc_viz / nfocc_gen / nfocc_sink
                     nfocc_disp_probe / nfocc_disturb_gen
  tests/             单元 + 集成 + 系统
  scripts/           build.sh / verify_*.sh
  cmake/             三方依赖与交叉编译工具链
docs/                设计文档集（索引见 docs/README.md）
datasets/            真实数据（不入库）
viz/                 可视化输出（不入库）
tools/               文档自检与绘图脚本
scripts/             工程级验证脚本
CHANGES.md           变更记录
```

## 文档

| 想了解 | 看 |
|---|---|
| 研发计划与缺口 | `docs/ROADMAP.md` |
| 精度目标与验收标准 | `docs/DECISION-DEPTH-ACCURACY.md`、`docs/SPEC-DEPTH-ACCEPTANCE.md` |
| 坐标系与 t_cam 语义 | `docs/SPEC-CONVENTION.md`（唯一权威） |
| 真实数据接入 | `docs/SPEC-DATASET.md` |
| 聚类与动静分离 | `docs/CLUSTERING-IMPLEMENTATION.md` |
| 防御接入现状 | `docs/PLAN-DEFENSE-INTEGRATION.md` |
| 构建与调试 | `docs/BUILD-CROSSPLATFORM.md` |
| 全部文档索引 | `docs/README.md` |

## 许可

内部项目，未授权勿外传。
