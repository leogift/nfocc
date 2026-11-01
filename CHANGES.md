# CHANGES

## v0.4.0 — 并发基础、依赖治理、参数定案

### 用户定案的新参数（已落到 `nfocc_default.h`，每项都带反推依据）

| 参数 | 原值 | 新值 | 依据 |
|---|---|---|---|
| `B_MIN` | 12 mm | **200 mm** | 车速下限 3.6 km/h = 1.0 m/s，5 Hz → 每帧位移 200 mm |
| `B_MAX` | 233 mm | **460 mm** | `numDisp×Z_MIN/VF_VIEW = 256×500/277.13 = 461.9` |
| `B_TARGET` | 120 mm | 300 mm | [200,460] 的几何中位 |
| `G_X` | −5000~+10000 | **−9000~+9000** | 用户定：后轴为原点，前后各 9 m |
| `G_Y` | ±6000 | **±7500** | 左右各 7.5 m |
| `G_Z` | −1000~+2500 | **−500~+2500** | 地下 0.5 m ~ 地上 2.5 m |
| `nx/ny/nz` | 75/60/18 | **90/75/15** | 由上推导；Z 3000/200 整除，无半格 |
| `NF_MAX_POINTS` | 15000 | 32768 | 柱数 90×75=6750，覆盖每柱 4~5 层 |

**B_MIN 从 12 抬到 200 是最大的行为变化**：旧值下 9 m 处视差 0.06 px，
根本无法三角化；200 mm 下 9 m 处 6.16 px，但仍只有 1.8 个体素的不确定度
（σ_Z=365 mm），**远场必须靠轮速/里程计，不能靠双目**。

### 新增：ZLToolKit 移植（`docs/ADR-009-zltoolkit-port.md`）

读了 `src/Thread`、`src/Poller`、`src/Util` 后按需求取舍，不是照搬：

| 组件 | 处置 | 理由 |
|---|---|---|
| `TaskExecutor` | 移植精简版 | 保留 `may_sync`（防线程内自投递死锁）、`AsyncFirst`（故障优先）、线程名 |
| `FramePool` | 借思路重写 | 只用它的引用计数+缓冲复用，不要它的 GOP 缓存（5 Hz 下缓存历史帧=延迟） |
| `EventBus` | 移植并改类型安全 | NoticeCenter 用 string 键，改成 enum class |
| `StageStats` | 移植精简版 | P95 比均值有判别力 |
| `EventPoller` | **不移植** | 本项目没有大量 fd；ZMQ 内部管理 fd，5 Hz 定频不需要事件驱动 |

### 新增：并发与工具模块

- `include/nfocc/util/task_executor.h` — 线程绑定 FIFO 队列
- `include/nfocc/util/frame_pool.h` — 帧缓冲池，运行期零 malloc
- `include/nfocc/util/event_bus.h` — pipeline 与日志/监控解耦
- `include/nfocc/util/timing.h` — RAII 计时 + 滑窗分位统计
- `tests/unit/test_concurrency.cpp` — **19 个并发用例全过**

### 新增：OpenMP 并行 warp

四路 warp 是纯函数（各写各的缓冲），可安全并行；引擎仍串行
（4 路并行 SGBM 需 507 MB，超上板预算）。

**实测**：3.2 ms/帧 → **1.2 ms/帧**（2.7×）。`ctest` 7/7 仍全过。

macOS 上 `#pragma omp` 会被静默忽略 —— 必须 `-Xpreprocessor -fopenmp`
加链接 `libomp`，已写进 `cmake/nf_deps.cmake`。

### 新增：依赖治理

`cmake/nf_deps.cmake` + `tools/sync_third_party.py` + `vcpkg.json`：

- 三层策略 `-DNF_DEPS=managed|vendored|system`
- `third_party/` 软链模式 **8 MB**（拷贝模式实测 424 MB）
- RPATH 写 `@loader_path/../lib` 相对路径

### 修复

| 问题 | 现象 | 根因 |
|---|---|---|
| Eigen include 未生效 | 日志打印"Eigen 5.0.1 <- 系统"，但 `flags.make` 里没有 eigen 路径 | Eigen 5.0 只导出 `Eigen3::Eigen` 目标，不设 `EIGEN3_INCLUDE_DIR`。改用 imported target |
| `numDisp` 设成 112 | pipeline 创建失败 `NF_E_PARAM_RANGE` | 112 是按 B_MIN 算的，B_MAX=460 需要 256。R2 校验正确地抓住了这个错 |
| `NFPoint` 尺寸 | 规范要求 9 字节，实际 15 | 3×int32+3×uint8 = 15，规范算术错误。已改规范并加 `offsetof` 断言 |
| OpenCV 异常 | `nf_engine_init` SIGTRAP | OpenCV 5.0 用 C++17 编译，跨标准边界 `catch` ABI 不匹配。改为调用前自查 `CV_Assert` 条件 |
| `NF_G_HASH_CAPACITY` | 200000 非 2 的幂 | 开放寻址用 `&(cap-1)`，非 2 的幂会让掩码失效 |
| `nf_dmap_alloc` 位置 | 单独链接核心库报 undefined | 定义在引擎库里，违反"核心库零依赖" |
| macOS `._*` 文件 | GLOB 把它们当源文件编译 | 加 `list(FILTER ... EXCLUDE REGEX)` |


### 修复（关键）：引擎符号冲突导致全链路 0 点

**现象**：四路相机全部 `NF_E_ENGINE_UNSUPPORTED`，点云恒为 0。日志显示
`nf_engine_init` 被正常调用了 —— 从日志完全看不出调错了实现。

**根因**：`nf_sgbm_engine_init` 与 `nf_model_engine_init` 原本都叫
`nf_engine_init`，两个引擎库各导出一份。静态链接时**先满足者胜出**，
而 `libnfocc_engine_model.a` 在链接行里排在 `libnfocc_engine_sgbm.a` 前面，
所以注册进 pipeline 的恒是**模型引擎**，它对 `kind==SGBM` 的配置直接拒绝。

**修法**：各引擎库导出带后缀的符号（`nf_sgbm_engine_*` / `nf_model_engine_*`），
公共名 `nf_engine_*` 保留给"只链一个引擎"的场合。新增
`tests/unit/test_engine_bind.cpp` 用 4 条判据锁死语义，其中两条是负对照：
绑模型后 SGBM 配置必须返回 `UNSUPPORTED`（这正是原故障的表现），
两个 `init` 的函数指针必须不同。

**验证**：修复后 `cam0 status=NF_OK pts=104462`，端到端累计 1224 点。

**这条链路上我猜错了三次**（以为是 `raw.empty()`、`kind` 传错、链接顺序），
都没修好。最后靠"在 `nf_engine_init` 入口加日志"定位 —— 日志打出
`UNSUPPORTED` 后，用 `nm` 比对两个库的导出符号才确认是覆盖。
**教训：静默错误要用观测点定位，不要靠推理链**。

### 验证方式（都可复跑）

```bash
python3 tools/sync_third_party.py --link
cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
ctest --test-dir build --output-on-failure      # 7/7
```

依赖验证脚本曾写错过：第一版读 `CMakeCache.txt` 取证，三个变量全 `None`——
它们是普通 `set()` 变量，不进缓存。换成正确定证（编译命令行 / `otool`）后
才抓到 Eigen 的真 bug。**探针错了不代表实现没问题**。

---


## v0.5.0 — 上下游协议、三目标工具链、ZMQ 端到端

### 新增：上游/下游协议（`docs/SPEC-UPSTREAM.md`、`SPEC-DOWNSTREAM.md`）

补齐 `SPEC-ICD` 只有结构体、没有传输层与时间片的缺口。

**时间片定案**：5 Hz → 帧周期 200 ms → 最短基线 200 mm。四路时间戳极差
≤5 ms、位姿-图像失步 ≤5 ms，超了冻结不发。丢弃超过 2 帧判长丢帧。

**上游结构**（`nfocc_io.h`，packed，尺寸被 `static_assert` 钉死）：
- `NFUpEgoMotion` **60 B**：四元数而非欧拉角（避万向锁），同时给速度（滑动参考帧用）
- `NFUpHeader` **88 B**
- `NFUpImageHeader` **32 B**

**下游结构**：`NFOutHeader` **64 B**。`timestamp_us` 是**曝光时刻**不是完成时刻——
混用会引入整个帧周期的系统偏差；完成时刻由 `latency_us` 单独承载。

**核心原则**：冻结也必须发消息。不发，下游无法区分"没障碍"和"感知挂了"。

### 新增：ZMQ 端到端（实测收发成功）

`nfocc_gen`（模拟上游 PUB）→ `nfocc_run`（`--zmq-in` / `--zmq-out`）→ `nfocc_sink`（下游 SUB）。

实测下游收到 3 条有效消息，点数 436/376/305，延迟 66 ms。

### 新增：三种输入源统一抽象（`nf::InputSource`）

合成 / 文件回放 / ZMQ 三选一，上层 pipeline 不感知。`--playback` 从空壳变成真实现。

### 新增：三目标构建（`scripts/build.sh`）

`host` / `linux` / `aarch64` / `core` 四个目标，参数固化。

- `core` 实测验证**核心库零第三方依赖**（M57 上没 OpenCV 也能编）
- `aarch64` 交叉编译后自动 `file` 校验架构，不对就退出非零

### 修复：合成源参考帧指针别名导致 0 点

`in->img_ref[c] = ref_[c].data()` 在 `memcpy(ref_, cur_)` **之前**执行，
存的是指针；下一帧 `ref_` 已被覆盖，SGBM 拿到两张相同的图 → 视差恒 0。

修法：先拷上一帧进 ref_，再生成当前帧。修后第 1、2 帧产出 80871 / 77767 点。

### 修复：`in->img_ref` 数组判空恒真

`in->img_ref` 是数组，地址永远非空，`if (in->img_ref)` 恒真 ——
"没有参考帧"会走有参考帧分支，拿 nullptr 去 warp。改判 `in->img_ref[c]`。
编译器给了 `-Wpointer-bool-conversion` 告警，是它指出来的。

### 修复：`test_engine_bind` 无条件构建

关闭所有引擎时该测试仍尝试链接 SGBM，导致 `core` 目标构建失败。
改为 `if(TARGET nfocc_engine_sgbm)` 保护。


### Aarch64 交叉编译：已实测通过

工具链 GCC 15.2.0（messense v15.2.0，184MB）。全部目标产出 aarch64 ELF：

```
nfocc_run: ELF 64-bit LSB executable, ARM aarch64, ...
  架构正确: aarch64
  依赖审计通过 (无 macOS 库)
  NEEDED: libpthread / libgomp / libstdc++ / libm / libgcc_s / libc
```

**未在真机运行** —— 本机是 macOS，板端执行待验证。产物架构与依赖已审计。

踩了两个坑，都写进了 `docs/BUILD-TOOLCHAIN.md` §3.5：

1. **工具链文件被 include 两次**（主工程 + `try_compile` 子工程），
   第二次缓存变量不可见，提前 `FATAL_ERROR` 会报"编译器明明识别成功却说找不到"。
2. **宿主依赖静默混入**：配置阶段打印 `Eigen <- 系统`、`OpenBLAS <- /opt/homebrew/...dylib`，
   链接期才炸。要在 **三处**探测分支都加 `AND NOT NF_CROSS_COMPILING`——
   第一次只改了一处，系统分支仍然找到宿主库。

国内网络：`ghproxy.net`/`ghfast.top` 对该仓库 502，`gh-proxy.com` 可用；
`brew tap` 走 github.com 全超时，需手工 clone 到 Taps 目录。


## v0.6.0 — 对应点提取（前向测距落地）

### 新增：径向膨胀匹配（`nfocc_match.h` / `match_radial.cpp`）

**这是前向测距缺的最后一环。** 此前 pipeline 的前向分支直接返回 0 点，
`nf_forward_range` 虽已实现但没有任何调用者。

按 SPEC-FORWARD §1 的结论，前向平移时极线从 FoE 辐射、行内方向差 175.9°，
SGBM 结构上不适用。改为**沿 FoE 径向搜索** + NCC 匹配。

**实测精度**（`tests/unit/test_match.cpp`）：

| 指标 | 值 | 判据 |
|---|---|---|
| 径向误差均值 | **0.045 px** | <0.5 px |
| 最大误差 | 0.600 px | — |
| >0.5px 占比 | 0.6% | — |
| 无关图误匹配率 | **3.3%** | <10% |
| 纯色图 | 0% | 必须 0 |

0.5 px 是 200 mm 体素的要求（SPEC-FORWARD §4：1 px ↔ 110 mm）。

### 三个判据都是被实测逼出来的

1. **边界拒绝**：patch 必须完整在图内（起点和终点都查）。
   不加时半径 58~64 处误差 1.9 px、最大 5.9 px，而内部只有 0.03 px —— 纯边界效应。

2. **峰值显著性**（主峰 − 次峰 > 0.15）：防重复纹理。

3. **峰值突出度**（主峰超过搜索范围均值 k·σ）：防无关纹理。
   实测只用"主峰−次峰"时，两张无关的高频图仍有 **42.9%** 判为有效 ——
   24 个候选位移里总能碰上一个局部 NCC 0.7+ 的位置。
   加突出度判据后降到 3.3%。
   **k 必须随 best_ncc 自适应**：固定 2σ 会误杀真实匹配（前向膨胀时 NCC
   是宽峰不是尖峰，多候选接近主峰 → σ 大），实测只剩 92 点。

### 修掉 4 个自己的错误（都在测试或参数上，不是算法）

| 问题 | 症状 | 原因 |
|---|---|---|
| 几何方向反了 | 误差 7.1 px | 让 `cur` 是放大版，则对应点在 −径向，而搜索是 +径向。正确是 `ref` 放大（车前进→物点更近→半径变大） |
| 源和目标同内存 | 误差 0.75 px / 333 点 | `warped(ref, ref, ...)` 边读边写，图像被破坏。诊断程序用独立缓冲是 0.045 px / 911 点，靠这个对比才找到 |
| 搜索范围含负值 | 剩 106 点 | 前向膨胀只让半径变大，负半边全是无意义候选，把 NCC 统计分布拉平，突出度判据误杀 |
| 无关图用正弦 | 40% 误匹配 | 正弦在 5×5 窗口内近似平面，任意两张低频纹理局部都相关。换真噪声才对 |

### 新增：`nfocc_run --forward`

切前向工况（位移在 z 轴）。实测：前向 335 点 / 3 帧，侧向 812 点 / 3 帧，
`ctest` **10/10** 通过。

## v0.8.0 — 真实数据接入（hanqtech）

首次用真实鱼眼数据跑通全链路。数据集在 `datasets/fisheye`，格式为 hanqtech hanqtech。

### 新增

| 模块 | 说明 |
|---|---|
| `NFCameraModelKind` | KB / RADIAL_POLY 两种投影模型 |
| 四次多项式内参 | `ρ(θ)=k1θ+k2θ²+k3θ³+k4θ⁴`，**k1 即焦距** |
| `hanqtech_loader` | 标定/ego 的 JSON 解析（手写，不引第三方库） |
| `ws_png` | 极简 PNG 解码（zlib，不引 libpng） |
| `ws_source` | 真实数据输入源，读 PNG + 算位移 |
| `test_hanqtech` | 6 用例 / 75 断言，用真实标定数值校验 |
| `verify_hanqtech_calib.py` | C++ vs Python 对拍，四路投影偏差 ≤9.8e-05 px |

### 实测确认的三件事

**一、外参约定与项目完全一致。** `det(R)=+1`、`‖RRᵀ−I‖<1e-6`、
相机 y 轴朝下、光轴朝外。`NFPose.R = M` 直接可用，无需变换。

**二、真实安装角与理想外参差别很大。** FV 俯角 22.7°、RV 37.3°、
**MVL/MVR 74°（侧下方俯视，不是侧视）**。图像已核对。

**三、环带语义必须是绝对俯角。** 早期实现当"相对光轴"，
在理想外参下恰好成立，接入真实数据后 FV 实际看到 432 mm（车头正下方）
而非 973 mm。改后水平距回到 579–2762 mm 的合理区间。

### 修掉的问题

| 位置 | 问题 | 症状 |
|---|---|---|
| `nf_ws_load_frame` | 四路用同一 frame_id | 只有 FV 加载成功，其余静默失败 |
| `view_lut.cpp` | 视口射线当相机系方向 | 真实外参下深度整体错位 |
| `nfocc_run` | 输入源在 pipeline **之后**创建 | 真实标定与视口设置全被忽略，程序照常出点 |
| `NF_CAMERA_MODEL_VERSION` 构造点 | 新增 `kind` 字段未初始化 | 6 个测试与 nfocc_run 全部失败 |

最后一条值得记：新增字段后，所有 `memset` 之外逐字段赋值的构造点都变得不确定。
`nf_cam_validate` 对未知 `kind` 明确报错，反而把这些点全暴露了出来 ——
这个设计是对的，静默当 KB 会让投影用到错误公式却不报错。

### 实测结果

```
nfocc_run --hanqtech datasets/fisheye --ws-frame 164 --frames 2 --stats
```

- 视口 640×172，f=184.75（按真实 1280×966 重算，不再固定 960）
- 四路全部 NF_OK；候选点 4858–63151；融合 416 点
- 水平距 579–2762 mm，均值 1897 mm
- 60.7 ms/帧，RSS 128.8 MB
- `ctest` 12/13（`depth_accuracy` 仍红，见下）

### 仍未解决

**这些点的精度没有真值可比。** 真实场景无激光标注，只能验证"落在合理范围"。
`depth_accuracy` 的三个用例仍是红的 —— 它用的是解析合成场景，
问题在于 `t_cam` 坐标系语义未定（`PROJECT-STATUS` G2），与真实数据无关。

## v0.7.0 — 视口偏转实施（ML/MR 单视口 + FV/RV ±45°）

### 定案

| 相机 | 视口数 | 偏转 | 视口方位角 |
|---|---|---|---|
| ML | 1 | 0° | +90° |
| MR | 1 | 0° | −90° |
| FV | 2 | ±45° | +45° / −45° |
| RV | 2 | ±45° | −135° / +135° |

合计 **6 视口**。视锥水平张角 **120°**（不是 150°）。

### 为什么视锥保持 120° 而不是 150°

`f = (W/2)/tan(hfov/2)`，张角与焦距成反比：

| 张角 | f (px/rad) | 角分辨率 | 同基线视差像素 | 直行+45° 点云 |
|---|---|---|---|---|
| 120° | 277.13 | 0.2067 °/px | 100% | 103 点 |
| 150° | 128.62 | 0.4455 °/px | **46%** | 85 点 |

覆盖上两者**完全相同**（6 视口下 az 全域已 2~3 重覆盖），
150° 不增加覆盖、只损失视差分辨率。实测点云 103 → 85。

### 新增的配置项

- `NFViewParams.yaw_offset_deg`：视口绕相机 y 轴偏转
- `NFPipelineConfig.view_yaw_offset_deg[4]`：每相机主偏转角
- `NFPipelineConfig.view_hfov_deg`：视锥张角可覆盖
- `nf_viewplan_default()`：6 视口规划表（唯一定义处）
- `nfocc_run --viewplan --hfov <deg>`：命令行切换

### 修掉三个真 bug（都是"视口系 vs 车系"混用）

偏转引入了一个系统性风险：**视口系与车系不再重合**。
三处代码此前默认两者相同，偏转后逐一出错。

| 位置 | 错误 | 症状 |
|---|---|---|
| 门控 `nf_gate_decide` | 用车系 `t_cam` 判工况 | 偏 30° 后仍报 FORWARD，偏转**完全不生效** |
| 前向分支 `nf_forward_range` | 用车系 `t_cam` | 45° 时解出几十米外，被 50000mm 门限全拒 |
| SGBM `mirror` 判据 | 用车系 `t_cam[0]` | 偏转后左右反了，SGBM 在错误方向搜索 |

还有一个**符号错误**：三角化的 `c1` 应为 `-t_view`（参考帧光心相对当前帧），
此前写成 `+t_view`。偏转为 0 时被其他符号抵消没暴露；
45° 时实测 `s0 = -364.6`（物点在相机后方，物理不可能），
取负后 `s0 = +364.6` 正确。**修正后 cam0 从 0 点变为 14970 点。**

### 实测（`scripts/verify_viewplan.py`，13 项全过）

| 工况 | cam0 模式 | 点云 | 水平距范围 / 均值 |
|---|---|---|---|
| 侧行 + 4 视口 | STEREO | 322 | 1128–7071 / 3247 mm |
| 侧行 + 6 视口 | HYBRID | 436 | 1128–7435 / 3506 mm |
| 直行 + 4 视口 | FORWARD | 156 | 1006–4314 / 2414 mm |
| 直行 + 6 视口 | HYBRID | 189 | 1006–6852 / 2918 mm |

**LUT 有效率随偏转角下降，是几何必然**：

| 偏转 | 有效率 | 出图外 | 超 r_valid |
|---|---|---|---|
| 0° / 30° | 100.0% | 0 | 0 |
| **45°** | **94.3%** | 3219 | 10786 |
| 60° | 76.0% | 3166 | 56363 |

视口水平 ±60°，偏 45° 后边缘到光轴 105°，超出鱼眼水平视场 100°，
约 5.7% 像素落在有效圆外。验收判据按偏转角设下限（≥90%），不是一刀切 100%。

### 未验证的

**45° 偏转的深度精度还没有真值比对。** 上面只有点数和距离范围，
没有"测出的距离 vs 真实距离"的误差。上一轮测 30° 时误差在 −11%~+21% 抖动，
45° 是否收敛未知。这需要合成场景的精确真值（不是纹理漂移近似）。

## 未完成

| 项 | 阻塞 |
|---|---|

| 模型引擎 | 骨架，返回 `UNSUPPORTED`；缺模型权重 |
| 安装目录可重定位 | RPATH 已设相对路径，但依赖库自身 install_name 仍是 Homebrew 绝对路径 |
| Aarch64 板端运行 | 产物已审计，未在 RK3588/M57 上执行 |
| ZeroMQ 零拷贝 | 当前每帧 memcpy 12MB（60MB/s @5Hz），RK3588 带宽够用，暂不优化 |
| 真实数据验证 | 用户明确排除，等真实图像/外参/KB 系数 |
