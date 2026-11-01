# SPEC-BUILD — 工程骨架（语言 / 构建 / 目录 / 线程 / 内存 / 日志）

补齐审计缺口 **G-7**。没有这份文档，每个工程师会按自己习惯搭一套，集成时无法合并。

---

## 1. 语言与标准

| 项 | 决定 | 理由 |
|---|---|---|
| 语言 | **C99 + 少量 C++17（仅测试与工具）** | RK3588/M57 的 SDK 都是 C 接口；C99 有 `stdint.h`/`restrict`，够用 |
| 禁用 | C++ 异常、RTTI、STL 容器（库内部） | 嵌入式内存可控性；错误一律用 `NFStatus` |
| 头文件 | 纯 C，可被 C++ 编译（`extern "C"`） | 便于在测试里用 C++17 |
| 定点/浮点 | 几何链路 **float32**；索引/计数 **int32/uint32** | float64 在 M57 上会掉速 |

> 为什么不用纯 C++：用户平台是 RK3588 + M57，两者商用车 SDK 以 C ABI 为主；
> C++ 的 ABI 不稳定会带来跨模块链接问题。

---

## 2. 目录结构

```
nfocc/
├── CMakeLists.txt
├── include/nfocc/          # 唯一对外头文件目录
│   ├── nfocc_types.h       # SPEC-ICD §1-1.2
│   ├── nfocc_camera.h      # NFCameraModel / NFPose / nf_cam_*
│   ├── nfocc_lut.h         # NFViewParams / nf_lut_* / nf_view_warp
│   ├── nfocc_engine.h      # NFEngineConfig / NFDisparityRange / NFDisparityMap
│   ├── nfocc_baseline.h    # SPEC-BASELINE-SM §3
│   ├── nfocc_pose.h        # SPEC-POSE
│   ├── nfocc_voxel.h       # SPEC-VOXEL
│   └── nfocc_status.h      # NFStatus + nf_status_str
├── src/
│   ├── camera/             # KB 正反投影
│   ├── lut/                # 视口 LUT 与 warp
│   ├── engine/
│   │   ├── engine_sgbm.c   # SGBM 实现
│   │   └── engine_model.c  # 模型实现（平台后端可插拔）
│   ├── baseline/           # 状态机
│   ├── pose/               # 位姿积分
│   ├── voxel/              # Hash + CMA + zMap
│   └── util/               # 日志、内存池、时间戳
├── tests/
│   ├── unit/               # L0，对应 SPEC-VERIFY 每条判据
│   ├── synth/              # L1，接 SPEC-P0-SYNTH
│   └── negative/           # 负对照用例（SPEC-VERIFY §10）
├── tools/                  # 宿主侧：合成数据生成、探针、文档自检
├── config/
│   ├── nfocc_default.h     # 参数唯一来源，由 PARAM.md 生成
│   └── platform/           # rk3588.h / axera_m57.h
└── docs/                   # 本规范集
```

**规则：`include/nfocc/` 之外的任何头文件都不允许被外部 include。**

---

## 3. 构建

| 项 | 决定 |
|---|---|
| 系统 | CMake ≥ 3.16 |
| 最低目标 | 交叉编译 `aarch64-linux-gnu` |
| 选项 | `NF_BUILD_TESTS`(默认 ON)、`NF_BUILD_TOOLS`(ON)、`NF_ENGINE_SGBM`(ON)、`NF_ENGINE_MODEL`(OFF) |
| 依赖 | OpenCV **仅**在 `engine_sgbm` 与 `tools` 里；核心库不依赖 OpenCV |
| 警告 | `-Wall -Wextra -Werror`（CI 严格） |
| 参数来源 | `config/nfocc_default.h` **由 `PARAM.md` 脚本生成**，手改无效 |

> **核心库不依赖 OpenCV** 是硬要求：M57 上不一定有 OpenCV；而且一旦依赖，
> "换引擎不改上层"的承诺就破了。

---

## 4. 线程模型（由内存预算决定，不是自由选择）

审计 §G-10 实算：

| 模式 | 峰值内存 | 结论 |
|---|---|---|
| 4 路并行 | ~1.93 GB | RK3588 可行但挤；**M57 很可能不行** |
| **4 路串行** | **~420 MB** | 两台平台都宽裕 |

**决定：默认串行（`NF_THREAD_CAMERAS=1`），提供并行选项。**

```
主线程:  采集同步(4路时间戳校验) → 基线状态机 → 分发
工作线程 1..N:  单路 (warp → engine → triangulate → voxel_insert)
主线程:  等全部相机完成 → grid_flatten → 输出
```

| 规则 | 说明 |
|---|---|
| 体素表**单线程写** | 各相机的结果先落到 per-camera 暂存，最后由主线程统一插入（避免加锁） |
| 引擎实例 per-thread | SGBM 的 `numDisp` 决定内存，N 个线程就要 N 份 |
| 禁止在中断/回调里分配内存 | 全部在 init 期分配 |

**若将来要开 4 路并行**：条件是 `4 × numDisp × W_view × H_view × 2 B + 其它 < 平台可用内存 × 0.6`，
不满足则自动降为串行并记 `NF_E_ENGINE_OOM`（不静默降级）。

---

## 5. 内存策略

| 项 | 规则 |
|---|---|
| 分配时机 | **全部在 `nf_init` 一次分配**，运行期零 malloc |
| 对齐 | 64 字节（NEON/Cache line） |
| 池 | 每个相机一个 buffer pool：原图 1 + 视口图 1 + 视差图 1 + 置信度 1 |
| 复用 | 帧间复用，不释放 |
| 大小 | 见审计 G-10 的表；由 `nf_mem_plan()` 在 init 期计算并返回总需求 |

`nf_mem_plan()` 必须能在**不分配**的情况下算出所需字节数，供上层在启动时做预算检查。

---

## 6. 日志与故障

| 级别 | 用途 |
|---|---|
| `NF_LOG_ERROR` | 数据不可用（相机失效、引擎失败） |
| `NF_LOG_WARN` | 降级（丢帧、体素满、截断） |
| `NF_LOG_INFO` | 每帧摘要（状态、点数、耗时） |
| `NF_LOG_DEBUG` | 逐阶段耗时、参数 |

- 日志**不得**在中断/实时路径里做格式化字符串解析（预格式化，或仅在 DEBUG 编译期开）。
- 每帧必带 `frame_id` 与 `state`，便于与下游对账。

---

## 7. 平台适配层

```c
typedef struct {
    void *(*alloc)(size_t, size_t align);
    void  (*free)(void *);
    nf_u64 (*now_us)(void);
    int   (*sgbm_submit)(...);   // 可选：平台加速的 SGBM
    int   (*model_infer)(...);   // 可选：NPU 推理
} NFPlatformOps;
```

| 平台 | 提供什么 |
|---|---|
| RK3588 | `alloc`/`now_us` + 可选 `sgbm_submit`（用 NEON 优化版） |
| Axera M57 | `alloc`/`now_us` + `model_infer`（AXMODEL） |
| 宿主（PC） | 全部软件实现，供 CI |

**规则：`NFPlatformOps` 是唯一允许出现平台分支的地方。** 上层代码不得出现 `#ifdef RK3588`。

---

## 8. 参数生成（防止文档与代码漂移）

`PARAM.md` 的符号表 → `config/nfocc_default.h`：

```
python tools/param_gen.py --src docs/PARAM.md --out config/nfocc_default.h
```

- 生成的头文件带 `/* AUTO-GENERATED FROM docs/PARAM.md, DO NOT EDIT */`
- CI 检查：重新生成后 `git diff` 为空，否则失败
- **这是 V-PARAM-2 的落地手段**：人工保证不住，必须脚本化

---

## 9. 验收

| 编号 | 判据 |
|---|---|
| V-BUILD-1 | `cmake -DCMAKE_BUILD_TYPE=Release` 零警告（`-Werror`） |
| V-BUILD-2 | 交叉编译出 aarch64 产物，且**核心库不链接 OpenCV**（`ldd` 检查） |
| V-BUILD-3 | `nf_mem_plan()` 返回值与实测峰值偏差 < 5% |
| V-BUILD-4 | 运行期 malloc 计数为 0（插桩验证） |
| V-BUILD-5 | `param_gen.py` 重跑后无 diff |
| V-BUILD-6 | 上层源码中 `#ifdef RK3588` 出现次数为 0 |
| V-BUILD-7 | 4 路串行模式峰值内存 < 450 MB（实测 RSS） |
