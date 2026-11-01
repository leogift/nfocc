# 构建与工具链

三套目标：**本机 macOS**、**Linux PC**、**Aarch64 嵌入式**。
一条命令覆盖，参数固化在 `scripts/build.sh`。

```bash
cd cpp
./scripts/build.sh host      # 本机 + 测试
./scripts/build.sh linux     # Linux PC（在 Linux 机器上执行）
./scripts/build.sh aarch64   # Aarch64 交叉编译
./scripts/build.sh core      # 只编核心库（零第三方依赖，用于验证解耦）
./scripts/build.sh clean
```

---

## 1. 本机 macOS

| 项 | 值 |
|---|---|
| 编译器 | Apple Clang（C++11） |
| 依赖 | Homebrew：opencv 5.0 / zeromq 4.3.5 / onnxruntime 1.31 / eigen 5.0 / openblas / libomp |
| 实测 | `ctest` **9/9 通过**，端到端产出点云 |

**macOS 特有的三个坑（都已在 `cmake/nf_deps.cmake` 里处理）**：

1. **`#pragma omp` 会被静默忽略**。Apple Clang 不带 libomp，只加 `#pragma omp`
   编译通过但完全串行。必须同时加 `-Xpreprocessor -fopenmp` 和链接 `libomp`。
   实测开关 OpenMP：3.1 ms/帧 → 1.0 ms/帧。
2. **`find_library` 不跟随符号链接**。Homebrew 的 `libopenblas.dylib` 是软链，
   直接找会失败。改用 glob 取带版本号的实体文件。
3. **`._*` 资源分支被 GLOB 当源文件**。`file(GLOB_RECURSE)` 会收进这些二进制
   垃圾，编译报 `expected unqualified-id`。用 `list(FILTER ... EXCLUDE REGEX)` 剔除。

---

## 2. Linux PC

| 项 | 值 |
|---|---|
| 编译器 | GCC ≥ 7 或 Clang ≥ 6（需 C++11） |
| 依赖 | 系统包：`libopencv-dev` `libzmq3-dev` `libeigen3-dev` `libopenblas-dev` `libomp-dev` |

```bash
sudo apt install build-essential cmake pkg-config \
     libopencv-dev libzmq3-dev libeigen3-dev libopenblas-dev libomp-dev
./scripts/build.sh linux
```

**Linux 与 macOS 的差异**：

| 项 | Linux | macOS |
|---|---|---|
| OpenMP | GCC 自带，`-fopenmp` 即可 | 需手工装 libomp |
| 头文件目录 | `opencv4/` | `opencv5/`（OpenCV 5） |
| RPATH | `$ORIGIN/../lib` | `@loader_path/../lib` |
| 内存统计 | `/proc/self/statm` | `mach_task_basic_info` |

两处平台差异都在 `nf_deps.cmake` 与 `nfocc_run.cpp` 里用 `#if defined(__APPLE__)`
处理，**上层代码没有平台分支**（`SPEC-BUILD` §7 的要求）。

---

## 3. Aarch64 交叉编译

### 3.1 工具链

| 宿主 | 工具链 | 安装 |
|---|---|---|
| macOS | `messense/macos-cross-toolchains/aarch64-unknown-linux-gnu` | 见下 |
| Linux | `g++-aarch64-linux-gnu` | `apt install g++-aarch64-linux-gnu` |

**macOS 上为什么不能用系统 clang**：Apple Clang 只能产 Darwin 目标，
无法链接 Linux 的 glibc。必须用这套 GNU 工具链（它跑在 macOS 上，
产出 Linux 的 aarch64 ELF）。

**国内网络注意（实测结论，别照抄网上的做法）**：

`brew install messense/macos-cross-toolchains/aarch64-unknown-linux-gnu`
**在国内网络下会失败** —— 它直接从 github.com 拉 184 MB 发布包，
报 `curl: (92) HTTP/2 stream 1 was not closed cleanly: PROTOCOL_ERROR`。
即使用代理也没成（等了近 8 分钟无输出）。

**可行路径：绕过 brew，手动下载解压。**

```bash
mkdir -p ~/toolchains && cd /tmp
curl -L --max-time 1800 -o aarch64-toolchain.tar.gz \
  "https://gh-proxy.com/https://github.com/messense/homebrew-macos-cross-toolchains/releases/download/v15.2.0/aarch64-unknown-linux-gnu-aarch64-darwin.tar.gz"
tar -xzf aarch64-toolchain.tar.gz -C ~/toolchains/
~/toolchains/aarch64-unknown-linux-gnu/bin/aarch64-unknown-linux-gnu-g++ --version
```

实测：184 MB 下载约 2.5 分钟，解压后 583 MB。
`cmake/toolchain-aarch64.cmake` 的 `HINTS` 已包含
`$ENV{HOME}/toolchains/aarch64-unknown-linux-gnu/bin`，
因此**不需要设任何环境变量**，`./scripts/build.sh aarch64` 直接可用。

镜像代理实测对比：

| 代理 | 该仓库 | 说明 |
|---|---|---|
| `gh-proxy.com` | **可用** | 184 MB 正常下载 |
| `ghproxy.net` | 502 | git clone 与 API 都失败 |
| `ghfast.top` | 超时 | — |
| 直连 github.com | PROTOCOL_ERROR / 超时 | 国内基本不可用 |

若已配好代理（如 `export https_proxy=http://127.0.0.1:10927 ...`），
brew 本体与 tap 可以走代理，但**那个 184 MB 的 release 资源仍可能被中断**，
手动下载更可靠。

### 3.2 编译

```bash
./scripts/build.sh aarch64
```

等价于：

```bash
cmake -S . -B build/arm \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake \
      -DNF_DEPS=vendored -DNF_WITH_OPENCV=OFF -DNF_WITH_ZMQ=OFF -DNF_WITH_ORT=OFF
```

**默认关闭三个第三方引擎**，因为交叉编译时没有 aarch64 的 OpenCV/ZeroMQ/ORT。
需要时提供其一：

- **vcpkg**：`-DNF_VCPKG_ROOT=/path/to/vcpkg`（用 `arm64-linux` triplet）
- **sysroot**：`-DNF_SYSROOT=/path/to/aarch64-sysroot`

`cmake/toolchain-aarch64.cmake` 里 `CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY`
**强制只在目标前缀找库**。不加这条会静默链到宿主的 arm64-darwin `.dylib`，
链接期报格式不兼容，或者更糟——侥幸链上但运行崩。

### 3.3 目标平台特性

```
-march=armv8-a+simd -mtune=cortex-a55
```

RK3588 = 4×Cortex-A76 + 4×Cortex-A55，M57 = Cortex-A55。
`armv8-a+simd` 是两者共同下限；`-mtune=cortex-a55` 按更弱的核调优，
在 A76 上只会更快不会错。

**不用 `-mcpu=native`** —— 交叉编译时它指的是宿主 CPU，结果对目标机是错的。

### 3.4 实测结果（已跑通）

```
==> 产物架构校验
build/arm/nfocc_run: ELF 64-bit LSB executable, ARM aarch64, version 1 (SYSV),
                     dynamically linked, interpreter /lib/ld-linux-aarch64.so.1
  架构正确: aarch64
  依赖审计通过 (无 macOS 库)
  NEEDED               libpthread.so.0
  NEEDED               libgomp.so.1
  NEEDED               libstdc++.so.6
  NEEDED               libm.so.6
  NEEDED               libgcc_s.so.1
  NEEDED               libc.so.6
```

工具链 GCC 15.2.0，全部目标（`nfocc_run` / `nfocc_gen` / `nfocc_sink` / 6 个测试）
都成功产出 aarch64 ELF。动态依赖只有 glibc / libstdc++ / libgomp，**没有误链
宿主 macOS 的 dylib**。产物未在真机运行（本机是 macOS），板端执行待验证。

### 3.5 交叉编译的两个坑（都实际踩过）

**坑 1：工具链文件被 include 两次。**
CMake 在主工程配置时 include 一次，之后 `try_compile` 的子工程里**再 include 一次**。
第二次时 `-D` 传的缓存变量不可见，若此时 `FATAL_ERROR` 直接退出，
症状是"编译器明明识别成功了（`The CXX compiler identification is GNU 15.2.0`），
紧接着却报找不到编译器"。
修法：不加提前的 FATAL，只在 `CMAKE_CXX_COMPILER` 未设置时才搜索。

**坑 2：宿主依赖静默混入。**
配置阶段 `Eigen 5.0.1 <- 系统`、`OpenBLAS <- /opt/homebrew/.../libopenblasp-r0.3.34.dylib`
看起来一切正常，直到链接期才炸；更糟的情况是侥幸链上但运行崩。
修法：`nf_deps.cmake` 开头设 `NF_CROSS_COMPILING`，Eigen 与 BLAS 的
**三处**探测分支全部加 `AND NOT NF_CROSS_COMPILING`。
只加在一处会漏——实测第一次只改了 vendored 分支，系统分支仍然找到宿主的库。

交叉编译时 `third_party/` 里的软链指向 Homebrew，同样不可用，
所以 vendored 模式在交叉编译下也一律跳过。

---

## 4. 核心库的零依赖约束

`SPEC-BUILD` §3 要求核心库不链接 OpenCV/ZeroMQ/ORT（V-BUILD-2）。
`./scripts/build.sh core` 会实测验证：

```
==> 核心库依赖检查 (应无 OpenCV/ZeroMQ/ORT)
  无第三方依赖 (符合预期)
```

**为什么这条重要**：M57 上不一定有 OpenCV；一旦核心库依赖它，
"换引擎不改上层"的承诺就破了。当前 `libnfocc_core.a` 只依赖:
- libc / libm
- pthread（任务执行器）
- libomp（OpenMP 运行时，编译器提供）

Eigen 是 header-only，不算链接依赖。

---

## 5. 依赖治理（三方库放工程内）

`tools/sync_third_party.py` 把系统库同步到 `third_party/`：

```bash
python3 tools/sync_third_party.py --link      # 软链，8 MB
python3 tools/sync_third_party.py            # 拷贝，424 MB
```

**软链模式改的是系统库吗**：不是。`--link` 时脚本会**跳过** `install_name` 改写，
否则通过软链改的就是 Homebrew 的原库（破坏性）。脚本里有这条保护并实测过。

**RPATH**：`cmake/nf_deps.cmake` 给可执行文件写相对 RPATH
（macOS `@loader_path/../lib`，Linux `$ORIGIN/../lib`），
`otool -l` 可验证已写入。

**已知限制（诚实记录）**：RPATH 指向的是 `install` 目录下的 `lib/`，
但依赖库本身仍带着 Homebrew 的**绝对路径** install_name。
所以「把 install 目录拷到别的机器就能跑」**目前不成立**——
要真可重定位，需在安装阶段对每个 dylib 跑 `install_name_tool -change`，
或改用静态链接。这条没做。

---

## 6. CI 建议

```bash
# 三套目标都要过
./scripts/build.sh core
./scripts/build.sh host
./scripts/build.sh aarch64        # 需要交叉工具链
```

严格模式加 `-DNF_STRICT_WERROR=ON`，把警告当错误。
当前代码在这个模式下零警告（实测）。
