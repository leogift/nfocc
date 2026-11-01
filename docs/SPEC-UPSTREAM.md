# SPEC-UPSTREAM — 上游数据接口与时间片

补齐 `SPEC-ICD` 缺失的**上游接入**部分：从谁拿数据、什么结构、什么时间片、
失步怎么办。下游输出协议见 `SPEC-ICD` §6，本文只覆盖输入端。

---

## 1. 时间片（TDD 帧周期）的唯一来源

**用户定案**：车速下限 3.6 km/h，处理频率 **5 Hz**。
由此推出本系统所有与时间相关的常量：

| 量 | 值 | 推导 | 出处 |
|---|---|---|---|
| 帧周期 `T_FRAME` | **200 ms** | 1 / 5 Hz | `NF_FRAME_PERIOD_MS` |
| 帧间隔（20 ms 网格） | 200 = 10×20 | 上游相机以 20 ms 为节拍出图 | — |
| 最短基线 `B_MIN` | **200 mm** | 3.6 km/h = 1.0 m/s ÷ 5 Hz | `NF_B_MIN` |
| 基线上限 | 见 `PARAM.md` B.2 | `numDisp×Z_MIN/VF_VIEW` | `NF_B_MAX` |
| 位姿-图像最大失步 | **5 ms** | 2.5% × T_FRAME | `NF_SYNC_MAX_SKEW_MS` |
| 四路图像时间戳极差 | **5 ms** | 同上，四路必须同步曝光 | `NF_SYNC_MAX_SKEW_MS` |
| 丢帧容忍 | **2 帧** | 400 ms 内可接受 | `NF_GAP_MAX_FRAMES` |
| 状态机迁移延迟 | **50 ms** | 1/4 周期 | `NF_B_LAT_MAX_MS` |

### 1.1 为什么 200 ms 是硬约束，不是建议

- **小于 200 ms**：基线下限不变（由车速决定），但计算量翻倍，且 5 Hz 是
  用户给的整车节拍，快于它没有额外信息。
- **大于 200 ms**：1 m/s 时基线 > 200 mm，`d_max` 很快撞上 `numDisp` 上限
  （基线上限见 `PARAM.md` B.2，对应约 2.3 m/s @5Hz）。超过就必须降档。

### 1.2 时间轴对齐规则

```
上游相机曝光     t0      t0+200   t0+400     (各相机独立触发)
                  |        |        |
四路时间戳极差   ≤5ms  ── 超过则本帧判 INVALID, 不发下游
                  |
位姿(ego motion) 必须覆盖 [t0-5, t0+5] 区间, 否则用最近邻并记
                  NF_E_BASELINE_DEGRADED
```

**决策**：**不做插值**。位姿与图像失步超过 5 ms 时，用最近邻位姿并降级标记。
理由：5 Hz 下位姿变化很小（1 m/s × 5 ms = 5 mm），插值收益不足以抵消
引入的时序复杂度；而降级标记能让下游看到"这一帧的基线不够准"。

---

## 2. 上游数据结构

### 2.1 图像帧（每相机一帧）

```c
/* 与 nfocc_io.h::NFImageFrame 一致 */
typedef struct {
    nf_u32 version;       /* = NF_IMAGE_FRAME_VERSION */
    nf_u8  cam_id;        /* 0=FV 1=RV 2=ML 3=MR */
    nf_u8  format;        /* 0=GRAY8 (本项目唯一支持) */
    nf_u16 reserved;
    nf_u32 width;         /* 必为 1920 */
    nf_u32 height;        /* 必为 1536 */
    nf_u32 stride;        /* 字节/行, 可 >= width */
    nf_u64 timestamp_us;  /* **曝光中点**的单调时钟, 微秒 */
    nf_u32 frame_id;      /* 单调递增, 丢帧时跳号(下游据此发现丢帧) */
    nf_u32 data_bytes;    /* = stride * height */
    /* 之后紧跟 data_bytes 字节像素 */
} NFImageFrame;
```

**约束**：
| 项 | 要求 | 违反后果 |
|---|---|---|
| `width/height` | 严格 1920×1536 | `NF_E_PARAM_RANGE` |
| `timestamp_us` | 单调时钟(uptime), **不是墙钟** | 墙钟会被 NTP 跳变, 导致基线算出负数 |
| `frame_id` | 单调递增，允许跳号 | 跳号 = 丢帧，连续跳 >2 帧冻结 |
| `data_bytes` | 必须等于 `stride*height` | 防止上游截断 |

**为什么时间戳要单调时钟**：系统墙钟会被 NTP 校正，一次跳变可能是几十毫秒，
比帧周期还大。用墙钟算基线会得到负值或突变，`t_cam` 直接错。

### 2.2 位姿 / ego motion

```c
typedef struct {
    nf_u32 version;       /* = NF_EGO_MOTION_VERSION */
    nf_u64 timestamp_us;  /* 与图像同一时钟源 */
    /* 车辆后轴中心在世界系下的位姿(ISO 8855) */
    nf_f32 pos_mm[3];     /* x 前, y 左, z 上 */
    nf_f32 quat[4];       /* w,x,y,z 单位四元数 */
    /* 车身系速度 */
    nf_f32 vel_mmps[3];   /* 后轴中心速度 */
    nf_f32 yaw_rate_dps;  /* 横摆角速度, deg/s */
    nf_u8  source;        /* 0=轮速 1=IMU 2=视觉 3=融合 */
    nf_u8  quality;       /* 0..100, <50 视为降级 */
} NFEgoMotion;
```

**为什么给四元数而不是欧拉角**：横滚/俯仰接近 ±90° 时欧拉角有万向锁。
四元数没有这个问题，且插值稳定（本项目虽不插值，但下游可能要用）。

**为什么同时给速度**：`B_TARGET`（见 `PARAM.md` B.3）的滑动参考帧需要「预计下一帧位移」，
用速度比用相邻帧位姿差分更稳（差分对噪声敏感）。

### 2.3 一帧的完整打包（ZeroMQ 单条消息）

```
偏移   长度     内容
0      4        magic "NFUP" (0x4E465550)
4      4        version (u32) = 1
8      4        frame_id (u32)
12     8        timestamp_us (u64)
20     4        n_cameras (u32) = 4
24     4        reserved
28     60       NFUpEgoMotion (packed, 见下)
88     ---      multipart: 段 1..4 各为 (NFUpImageHeader 32B + 像素)
```

`NFUpEgoMotion` 实测 **60 字节**（packed: 4+8+12+16+12+4+1+1+2=60），
`NFUpHeader` 合计 **88 字节**（4+4+4+8+4+4+60，packed）。
尺寸由 `nfocc_io.h` 的 `static_assert` 钉死，改字段会立刻编译失败。

### 2.4 为什么用多段消息（multipart）而不是单块

ZeroMQ 支持 multipart。**建议用 5 段消息**：

```
段 0: 头部(28 字节) + NFEgoMotion
段 1..4: 每段 = NFImageFrame 头部 + 像素
```

理由：下游（或中间代理）可以只读段 0 拿到时间戳与位姿做调度，不必拷贝
4×3MB 像素。ZMQ 的 multipart **零拷贝**转发，价格与单块相同。

---

## 3. 时间片内的处理预算

5 Hz → 每帧 200 ms。预算分配（目标，非实测）：

| 阶段 | 预算 | 说明 |
|---|---|---|
| 收帧 + 解包 | 10 ms | 4×3 MB = 12 MB 拷贝；建议零拷贝(见下) |
| 四路 warp | 8 ms | 实测 1.0 ms/帧(OpenMP 4 线程) |
| SGBM × 4 | 100 ms | **最大头**，单路 127 MB 工作内存, 串行 |
| 三角化 + 体素融合 | 30 ms | 单线程(体素表无锁) |
| 输出序列化 | 10 ms | 取决于点数 |
| 余量 | 42 ms | 抖动与 GC/调度 |

**结论**：SGBM 占一半预算。若后续要上 4 路并行，每路 127 MB × 4 = 507 MB
（`SPEC-BUILD` §4 的实算），RK3588 勉强、M57 不行。

---

## 4. 失步与异常语义（上游侧）

| 情况 | 判定 | 动作 | 下游可见 |
|---|---|---|---|
| 四路时间戳极差 > 5 ms | 不同步 | 本帧不处理 | 无输出（`frame_id` 断档）|
| 位姿与图像失步 > 5 ms | 降级 | 用最近邻位姿 | `NF_E_BASELINE_DEGRADED` |
| `frame_id` 跳号 ≤ 2 | 丢帧 | 正常处理，基线按实际间隔算 | 正常输出 |
| `frame_id` 跳号 > 2 | 长丢帧 | 冻结 | `NF_E_BASELINE_DEGRADED` |
| 图像尺寸不符 | 配置错 | 拒绝整帧 | 无输出 + 错误日志 |
| `quality < 50` | 位姿降级 | 冻结 | `NF_E_BASELINE_DEGRADED` |
| 车速 < 3.6 km/h | 基线不足 | 冻结 | `NF_E_BASELINE_IDLE` |

**关键：冻结时下游收到的仍是完整消息**，只是 `is_valid=0`、`num_points=0`。
**不能不发消息** —— 下游无法区分"没障碍"和"系统挂了"。

---

## 5. 上游接入方式

### 5.1 ZeroMQ（推荐，实时）

| 项 | 值 | 理由 |
|---|---|---|
| 模式 | `SUB` 连上游 `PUB` | 感知是"消费最新帧" |
| `RCVHWM` | 2 | 只保留最新，避免积压（实时系统的正确行为） |
| 订阅 | 空串（收全部） | 若上游分相机 topic，可在此过滤 |
| 传输 | `tcp://` 或 `ipc://` | 同机用 `ipc://` 省协议栈开销 |
| 超时 | 200 ms × 3 = 600 ms 无帧 | 判上游断流，报错但不退出 |

### 5.2 文件回放（SIL 调试）

`.nfp` 容器（见 `src/io/file_io.cpp`），格式与 ZMQ 单条消息**同构**，
只是 4 路图像按顺序排列。这样同一份录包既能回放也能走网络。

### 5.3 零拷贝建议（未实现，记为待办）

当前实现每条消息 `memcpy` 一次（12 MB / 帧 = 60 MB/s @5Hz）。
可改为：
1. ZMQ 用 `zmq_msg` 直接在接收缓冲上构造 `NFImageView`，生命周期绑定到消息；
2. 帧池 `nf::FramePool` 直接接管该缓冲（需自定义 deleter）。

收益：省 60 MB/s 的拷贝。代价：缓冲生命周期与 ZMQ 消息绑定，出错易 use-after-free。
**当前不做的理由**：60 MB/s 对 RK3588 的 DDR 带宽不构成瓶颈，先保证正确性。

---

## 6. 验收

| 编号 | 判据 | 方法 |
|---|---|---|
| V-UP-1 | 收到的帧 `data_bytes == stride*height`，否则拒绝 | 注入截断消息 |
| V-UP-2 | 时间戳用单调时钟：人为把墙钟回拨，基线不得为负 | 注入跳变时间戳 |
| V-UP-3 | 四路极差 > 5 ms 时不产出、不停机 | 注入失同步帧 |
| V-UP-4 | `frame_id` 跳号 > 2 时冻结并报 `DEGRADED` | 注入跳号序列 |
| V-UP-5 | 冻结帧**仍然发出**完整下游消息（`is_valid=0`） | 检查下游收到条数 |
