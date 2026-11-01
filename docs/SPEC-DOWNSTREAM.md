# SPEC-DOWNSTREAM — 下游输出协议

补齐 `SPEC-ICD` §6 只有结构体、没有传输层的问题。本文定义：
发什么、怎么序列化、什么节奏、下游怎么判断"系统挂了"。

上游接入见 `SPEC-UPSTREAM.md`。

---

## 1. 输出内容

一帧的输出 = **点云** + **高程柱(zMap)** + **状态**。三者绑在同一条消息里。

| 部分 | 用途 | 是否可省略 |
|---|---|---|
| 点云 `NFPoint[]` | 下游避障/规划直接用 | 可（冻结时为空） |
| zMap + zIndex | 高程图消费（可视图/代价地图） | 可（下游不关心高程时） |
| 状态头 | **不可省** —— 下游据此判断是否可信 | 不可 |

**核心原则：冻结也必须发消息。** 不发消息，下游无法区分"没有障碍物"和
"感知挂了"。这是安全关键系统的硬要求。

---

## 2. 消息结构（ZeroMQ 单条 / 两段 multipart）

```
段 0: NFOutHeader (固定 64 字节)
段 1: 变长负载 = 点云 + zMap + zIndex
```

### 2.1 `NFOutHeader`

```c
typedef struct {
    nf_u32 magic;         /* 0x4E464F55 = "NFOU" */
    nf_u32 version;       /* = 1 */
    nf_u32 frame_id;      /* 回抄上游 frame_id, 便于对账 */
    nf_u64 timestamp_us;  /* 上游曝光时刻(单调时钟), 不是处理完成时刻 */
    nf_u64 latency_us;    /* 从曝光到本消息发出的耗时 */
    nf_u8  is_valid;      /* 1=结果可信; 0=冻结/降级 */
    nf_u8  status;        /* NFStatus 的低 8 位; is_valid=0 时说明原因 */
    nf_u8  mode;          /* NFMode: 0=STEREO 1=FORWARD 2=HYBRID 3..5=FROZEN */
    nf_u8  num_cameras_ok;/* 参与本帧的有效相机数 0..4 */
    nf_u32 num_points;    /* 点云点数 */
    nf_u32 zmap_cells;    /* zMap 单元数 (nx*ny); 0 表示不带高程 */
    nf_u32 zmap_bytes;    /* = zmap_cells * 4 */
    nf_u32 zindex_bytes;  /* = (zmap_cells+1) * 4 */
    nf_u32 points_bytes;  /* = num_points * sizeof(NFPoint) */
    nf_f32 baseline_mm;   /* 本帧实际基线, 供下游判断精度 */
    nf_f32 sigma_med_mm;  /* σ 中位数, 供下游设膨胀半径 */
    nf_u32 reserved;
} NFOutHeader;            /* 64 字节(packed) */
```

**`timestamp_us` 必须是曝光时刻，不是处理完成时刻**。下游做时空对齐用的是
"这个结果描述的是哪一刻的世界"，用完成时刻会引入 200 ms 的系统性偏差。

**`latency_us` 单独给**，供下游监控实时性。两者不可混用。

### 2.2 负载布局

```
偏移 0                    : NFPoint[num_points]        (9 字节/点, packed)
偏移 num_points*9         : nf_u32[zmap_cells]        zMap
偏移 + zmap_bytes         : nf_u32[zmap_cells+1]      zIndex 前缀和
```

点云在前，因为它是最高频消费的部分；下游可以只读头部+点云段就返回。

### 2.3 为什么点云是 15 字节不是 9

`SPEC-ICD` 原写 9 字节是算术错误（3×int32 + 3×uint8 = 15）。
`NFPoint` 用 `__attribute__((packed))` 保证无填充，实测 `sizeof==15`，
`offsetof` 依次 0/4/8/12/13/14。详见 `APPENDIX-CHANGELOG` v0.4.0。

---

## 3. 输出节奏

| 项 | 规则 | 理由 |
|---|---|---|
| 频率 | **严格跟随输入帧**，不自行定频 | 输入 5 Hz，输出也 5 Hz |
| 丢帧 | 输入丢帧则输出也跳号 | 下游靠 `frame_id` 断档发现 |
| 冻结 | **照常发**，`is_valid=0` | 见 §1 核心原则 |
| 首帧 | 等基线达到 `B_MIN`(200 mm) 才首次 `is_valid=1` | 冷启动前 1~2 帧冻结 |
| 背压 | ZMQ `SNDHWM=4`，满则丢最旧 | 下游慢时丢帧优于积压 |

**为什么 SNDHWM=4 而不是更大**：感知结果是"越快越有用"，积压 4 帧
（800 ms）已经超出任何避障系统的容忍。积压更多只会让下游拿到过期数据。

---

## 4. 下游如何判断系统状态

下游**只靠 header 就该能判断**，不需要解析负载：

```
is_valid=1                      -> 正常使用
is_valid=0, status=IDLE(20)     -> 车速过低, 无深度, 属预期
is_valid=0, status=DEGRADED(23) -> 位姿降级/长丢帧, 精度不可信
is_valid=0, status=OVER(21)     -> 基线超限, 上游节拍有问题
num_cameras_ok < 4              -> 部分相机失效, 覆盖不全
latency_us > 200000             -> 超时, 下游应作废本帧
frame_id 断档 > 2               -> 上游丢帧
```

**下游不得用"点数多少"判断有效性** —— 前方空旷时点数天然很少，
那不代表系统异常。

---

## 5. zMap 的语义（防误用）

| 项 | 值 | 易错点 |
|---|---|---|
| 单元编号 | `cell = iy*nx + ix` | **不是 `ix*ny+iy`**，用错会让高程图转置 |
| 位序 | `bit k = 第 k 层体素存在` | nz=15 时只用 bit0..bit14 |
| bit31 | **恒 0** | 下游若拿 bit31 做标志位会与保留位冲突 |
| zIndex | 前缀和，`zIndex[c]..zIndex[c+1]` 是本柱点数区间 | 长度 `n_cells+1` |
| 单元中心 | 由 `SPEC-COORD` §5 的索引反算 | **X/Y 反向**：ix=0 靠近 X_MAX |

nx=90, ny=75, nz=15（感知范围 18m×15m×3m，vox=200mm）。

---

## 6. 序列化

### 6.1 字节序

**一律小端**（x86/Aarch64 都是小端）。若将来上大端平台，需在序列化层转换，
不在结构体里做 —— 结构体是内存布局，不是线上格式。

### 6.2 结构体对齐

`NFOutHeader` 用 packed 钉死布局，**64 字节无填充**（实测 static_assert 校验）。
`NFPoint` 用 `packed`，15 字节。接收方必须按小端、无填充解析，
不能用本地的 `struct` 定义去 `memcpy` 映射（除非确认 ABI 一致）。

**建议下游用显式偏移解析**，而不是直接 cast —— 跨编译器/跨版本更稳。

### 6.3 大小估算

| 内容 | 典型 | 上限 |
|---|---|---|
| 头部 | 64 B | 64 B |
| 点云 | 5000 × 15 = 75 KB | 32768 × 15 = 492 KB |
| zMap | 6750 × 4 = 27 KB | 27 KB |
| zIndex | 6751 × 4 = 27 KB | 27 KB |
| **合计** | **~129 KB/帧** | ~546 KB/帧 |

5 Hz 下典型带宽 **0.65 MB/s**，上限 2.7 MB/s。对 `ipc://` 或千兆网都宽裕。

---

## 7. 输出方式

### 7.1 ZeroMQ（推荐）

| 项 | 值 | 理由 |
|---|---|---|
| 模式 | `PUB` bind | 支持多个下游订阅者 |
| `SNDHWM` | 4 | 见 §3 |
| 传输 | `tcp://*:port` 或 `ipc:///tmp/nfocc.out` | 同机用 ipc |
| multipart | 是（段 0 头 / 段 1 负载） | 下游可只读头做调度 |

### 7.2 文件（SIL 回归）

`.nfo` 容器：magic + version + 每帧 (NFOutHeader + 负载)。
用于录下结果做逐位比对（V-ENG-2 的确定性要求）。

---

## 8. 验收

| 编号 | 判据 | 方法 |
|---|---|---|
| V-DS-1 | `NFOutHeader` sizeof == 64，无隐式填充 | `static_assert` |
| V-DS-2 | 冻结帧仍发出完整消息，`is_valid=0` | 检查下游收到条数 == 输入帧数 |
| V-DS-3 | `timestamp_us` == 上游曝光时刻（不是完成时刻） | 注入已知时间戳比对 |
| V-DS-4 | zMap 的 `cell=iy*nx+ix`，非方阵网格下可验证 | nx≠ny 时故意测 |
| V-DS-5 | bit31 恒 0，bit≥nz 恒 0 | 全量扫描 |
| V-DS-6 | 点数超 `MAX_POINTS` 时按存在概率截断且置 `overflow` | 注入超量点 |
| V-DS-7 | `latency_us` 反映真实耗时（±10%） | 与外部计时比对 |
