# NFOCC

多相机鱼眼近场稠密深度与体素重建系统。4 路环视鱼眼 + 运动双目，覆盖 0.5–5 m，
输出点云与 2D 高程柱（zMap）。

C++11 产品实现，面向 RK3588 / Axera M57 边缘 NPU。

---

## 快速开始

### 依赖

| 库 | 版本 | 必需 |
|---|---|---|
| CMake | ≥ 3.16 | 是 |
| C++11 编译器 | clang / gcc | 是 |
| OpenCV | 4.x/5.x（SGBM 模块） | 否，`NF_WITH_OPENCV=OFF` 可关 |
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

> `build.sh host` 会跑全量 ctest；`depth_accuracy` 的 4 个旧验收用例已知失败，
> 脚本会以非零退出。只看其余测试用 `ctest --test-dir build/host -E depth_accuracy`。

`cpp/scripts/build.sh` 支持三种依赖来源：

```bash
cmake -S . -B build -DNF_DEPS=vendored   # 默认，优先 cpp/third_party/
cmake -S . -B build -DNF_DEPS=managed    # 系统路径 / find_package
cmake -S . -B build -DNF_DEPS=off        # 不引入三方库
```

vcam 是投影后端，默认由 `third_party/vcam` 提供；`NF_WITH_VCAM=OFF` 只能编核心库，
投影/LUT/端到端测试会失败。若要重新生成 vcam 静态库：

```bash
python3 tools/sync_vcam.py --src /Volumes/Workspace/virtualcamera/cpp
```

再构建。详见 `docs/BUILD-TOOLCHAIN.md`。

### 测试

```bash
cd cpp

# 全部（20/21；已知失败的是 depth_accuracy）
ctest --test-dir build/host --output-on-failure

# 排除已知失败
ctest --test-dir build/host -E depth_accuracy

# 单个测试
./build/host/tests/test_cluster
./build/host/tests/test_zmq_io
ctest --test-dir build/host -R unit_cluster -V
```

当前实测：21 个目标 20 个通过，180 用例 / 2,361 断言；`depth_accuracy` 的
6 个用例中 4 个旧验收用例失败（前视 90° 表面中位 307/305/460/857 mm）。

| 测试 | 覆盖 |
|---|---|
| `test_geometry` / `test_camera` / `test_voxel` / `test_scene` | 基础几何、相机、场景与数据结构 |
| `test_algo_rev` | D1–D4 等 12 个算法的独立验证 |
| `test_cluster` | 八叉树半径查询（对拍暴力）、DBSCAN、动静分离 |
| `test_d4_baseline` | 里程计积分、双基线状态机（含负对照） |
| `test_zmq_io` | 零拷贝生命周期、尺寸白名单、所有权转移 |
| `test_vcam_bridge` | vcam 后端与原生实现的一致性 |
| `test_pipeline` / `test_depth_accuracy` | 端到端；`test_pipeline` 含 6 视口映射，`test_depth_accuracy` 的 4 个旧验收用例仍失败 |
| `system_run_synthetic` | 系统级冒烟 |

### 运行

```bash
cd cpp

# 真实数据（hanqtech 鱼眼，结构见 docs/SPEC-DATASET.md）
./build/host/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164 --frames 3 --stats

# 6 视口：FV ±45 / RV ±135 / ML +90 / MR -90
./build/host/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164 --frames 3 --stats --sfm6

# 关闭车身掩码做负对照（点数应回升）
./build/host/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164 --frames 3 --stats --no-body-mask

# 中间结果可视化（Y 通道 / 视口 / 视差 / BEV / 车身掩码 / 聚类）
# 注意：nfocc_viz 仍是旧的全局 1024 宽 6 视口实现，FV/RV 成对视口会 LUT 参数不匹配；
# 6 视口验收请用 ./build/host/probe_sfm6 或 nfocc_run --sfm6。
./build/host/nfocc_viz ../datasets/fisheye 164 ../viz
```

`nfocc_run` 常用参数：

| 参数 | 说明 |
|---|---|
| `--frames N` | 处理帧数 |
| `--ws-frame N` | 起始帧号 |
| `--speed KMH` | 车速，驱动 yaw_rate 积分 |
| `--yaw-rate DPS` | 偏航率 (deg/s) |
| `--hfov DEG` | 4 视口模式的全局水平张角（默认 150）；`--sfm6` 用规划表自己的张角 |
| `--sfm6` | 6 视口：FV ±45、RV ±135、ML +90、MR -90 |
| `--forward-sample N` | 前向径向匹配采样步长（6 视口默认 4） |
| `--forward-coarse PX` | 前向粗搜步长（6 视口默认 4.0） |
| `--body-mask-dir DIR` | 每路 `<FV|RV|MVL|MVR>.png` 车身掩码 |
| `--baseline-mgr` | 双基线状态机 |
| `--d4` | 启用双基线 |
| `--no-body-mask` | 关闭 D1 车身掩码 |

### 验证

```bash
# vcam 对拍 / ZMQ 零拷贝（当前工程内实际存在的两个脚本）
./scripts/verify_vcam.sh
./scripts/verify_zmq.sh
```

---

## 核心设计

**虚拟相机视口**：所有视口按 pitch=0、roll=0 构造，只有 yaw。真实外参
只决定「这个车系方向落在哪张鱼眼的哪个像素」，不参与视口朝向。

| 视口 | 尺寸 | 水平张角 | f (px/rad) |
|---|---|---|---|
| 左右 (ML/MR) | 1024 × 475 | 150° | 137.2 |
| 前后 (FV ±45° / RV ±135°) | 590 × 716 | 110° | 206.6 |

前后视口成对使用：中心 ±45°（两组中心相隔 90°），各 110° 张角，留 20° 重叠。
90° 张角时两个视口只在 0° 交于一点，正前方目标落在两边边缘；实测竖面中位
1213 mm（90°）→ 791 mm（110°）。110° 的 f=206.6，仍是侧视 150°（137.2）的 1.5 倍。

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
  tests/            单元 + 集成 + 系统
  scripts/           build.sh / verify_*.sh
  cmake/             三方依赖与交叉编译工具链
docs/                设计文档集（索引见 docs/README.md）
datasets/            真实数据（不入库）
viz/                 可视化输出（不入库）
tools/               build_report_full.py / sync_vcam.py（根 tools）
scripts/             工程级验证脚本
CHANGES.md           变更记录
```

## 文档

| 想了解 | 看 |
|---|---|
| 研发计划与缺口 | `docs/IMPROVEMENT-PLAN.md` |
| SFM 6 视口设计与实测 | `docs/SFM-6VIEW.md` |
| 坐标系与 t_cam/t_veh | `docs/SPEC-CONVENTION.md`（唯一权威） |
| 真实数据接入 | `docs/SPEC-DATASET.md` |
| 接口与失效语义 | `docs/SPEC-ICD.md` |
| 视口 LUT | `docs/SPEC-LUT.md` |
| 验收规范 | `docs/SPEC-VERIFY.md` |
| 防御接入现状 | `docs/PLAN-DEFENSE-INTEGRATION.md` |
| 构建与调试 | `docs/BUILD-TOOLCHAIN.md` |
| 全部文档索引 | `docs/README.md` |

## 许可

内部项目，未授权勿外传。
