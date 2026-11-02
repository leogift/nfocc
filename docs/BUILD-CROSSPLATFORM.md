# 构建与调试（macOS / Linux 统一 CMake）

日期：2026-11-02

---

## 1. 一条命令

```bash
cd cpp
./scripts/build.sh host        # 本机 Release + 跑测试
./scripts/build.sh debug       # 本机 Debug（-O0 -g，assert 生效）
./scripts/build.sh core        # 只编核心库（零三方依赖）
./scripts/build.sh aarch64     # 交叉编译 ARM
./scripts/build.sh clean
```

脚本内部只是把下面这些 cmake 参数固化，CI 与本地用同一份。

---

## 2. 三方依赖：三种来源

```bash
cmake -S . -B build -DNF_DEPS=vendored   # 默认：优先 cpp/third_party/
cmake -S . -B build -DNF_DEPS=managed    # 只用 find_package / 系统路径
cmake -S . -B build -DNF_DEPS=off        # 不引入三方库（核心库仍可构建）
```

| 模式 | 适用 | 说明 |
|---|---|---|
| vendored | 本机开发 | `third_party/` 里是符号链接，由 `tools/sync_third_party.py` 建立 |
| managed | Linux CI | 依赖装在系统路径，`apt install libopencv-dev` 之类 |
| off | 交叉编译 / 最小验证 | 只编 `nfocc_core`，无 OpenCV/ZeroMQ/ORT |

`NF_DEPS=off` 时 SGBM 引擎、ZMQ IO、ORT 模型引擎全部跳过，
但核心几何、体素、门控仍可编可测 —— 这是板端最小镜像的基础。

---

## 3. macOS 特有处理

### 3.1 `._` 文件

macOS 在 exFAT / 网络卷 / 部分 U 盘上会为**每个文件**生成 `._xxx` 伴随文件。
GLOB 收源文件时会把它们当 `.cpp` 收进去，报一堆错。

CMake 里已经过滤：

```cmake
list(FILTER NF_CORE_SRC EXCLUDE REGEX "/\\._[^/]*$")
```

`.gitignore` 里也排除了 `._*`。实测本项目工作区曾一次产生 1573 个。

**已清理**：`find . -name "._*" -delete`，当前为 0。

### 3.2 OpenMP

AppleClang **不带** libomp。不加 `-Xpreprocessor -fopenmp` 的话
`#pragma omp` 会被**静默忽略** —— 编译通过、完全串行。这是 macOS 上
最容易漏的一条。CMake 里已处理，并会打印 OpenMP 来源。

验证方式：

```bash
./build/host/nfocc_run --stats
# 若耗时与单线程完全相同，检查是否串行编译
```

### 3.3 RPATH

部署时 `libopencv_*.dylib` / `libomp.dylib` 不在系统默认搜索路径。
不写 RPATH 就要用户手工设 `DYLD_LIBRARY_PATH`，量产镜像不可接受。
CMake 用**相对** RPATH：

```
macOS: @loader_path/../lib
Linux: $ORIGIN/../lib
```

整个目录拷过去就能跑。

---

## 4. Linux

```bash
# 依赖（Debian/Ubuntu）
sudo apt install build-essential cmake libopencv-dev libzmq3-dev \
                 libeigen3-dev libopenblas-dev libomp-dev zlib1g-dev

cd cpp && ./scripts/build.sh linux
```

与 macOS 的差异：

| 项 | macOS | Linux |
|---|---|---|
| OpenMP | 手工找 libomp | `find_package(OpenMP)` 直接可用 |
| 动态库扩展 | `.dylib` | `.so` |
| RPATH 变量 | `@loader_path` | `$ORIGIN` |
| 调试器 | lldb | gdb / lldb |
| 架构检查 | `file` + `otool` | `file` + `ldd` |

CMake 里这些都用 `if(APPLE)` / `else()` 分支处理，无需手工切换。

---

## 5. 调试

```bash
# Debug 构建
./scripts/build.sh debug

# lldb (macOS / Linux)
lldb build/debug/nfocc_run
(lldb) run --hanqtech ../datasets/fisheye --ws-frame 164 --frames 3

# gdb (Linux)
gdb --args build/debug/nfocc_run --hanqtech ../datasets/fisheye --ws-frame 164

# 单个测试
./build/debug/tests/test_geometry
ctest --test-dir build/debug -R unit_geometry -V

# 内存检查
valgrind --leak-check=full ./build/debug/tests/test_voxel   # Linux
```

`build/debug/compile_commands.json` 可直接给 VS Code / clangd / CLion 用。

---

## 6. vcam 后端（可选）

用 `/Volumes/Workspace/virtualcamera/cpp` 的相机模型做投影。

```bash
# 开启
cmake -S . -B build -DNF_WITH_VCAM=ON \
      -DNF_VCAM_DIR=/Volumes/Workspace/virtualcamera/cpp
cmake --build build -j

# 或用环境变量走脚本
NF_VCAM_DIR=/Volumes/Workspace/virtualcamera/cpp ./scripts/build.sh host
```

**默认 OFF。** 关闭时 `vcam_bridge.cpp` 编成桩，
`nf_vcam_backend()` 返回 `NF_VCAM_BACKEND_NONE`，业务代码按返回值判断。

### 6.1 为什么用 add_subdirectory 而不是拷贝

vcam 是独立工程，自带 CMakeLists 与测试。拷贝进 `third_party/` 会产生
两份会漂移的副本。`add_subdirectory` 让它作为子工程参与同一次构建，
版本由 `NF_VCAM_DIR` 决定（可以是 git submodule）。

子工程里 `VCAM_BUILD_TOOLS=OFF`、`VCAM_BUILD_TESTS=OFF`，
避免拉进 vcam 的 OpenCV 工具层。

### 6.2 为什么是"桥接"不是"替换"

vcam 的相机模型更通用（KB 独立 fx/fy、e-ftheta、Brown-Conrady），
但它**没有** NFOCC 这几层语义：

1. 环带角是**绝对俯角**（相对地面），不是相对光轴；
2. 四道有效性淘汰要分开计数；
3. 车身掩码在 LUT 构建期生效。

直接换 `vc_conv_build_maps` 会丢掉这三样。所以只借相机模型层，
LUT 网格构建仍走 NFOCC 自己的实现。

### 6.3 一致性验证

```bash
./build/host/tests/test_vcam_bridge
```

实测结果：

| 模型 | 比较点数 | 最大偏差 |
|---|---|---|
| radial_poly | 338 | **0.117 px** |
| KB | 400 | **0.190 px** |

门限 0.5 px。两者远低于门限，说明换后端不改变几何结果。

### 6.4 一个必须补的检查

vcam 的 `vc_project_fisheye` 用 `atan2(r_xy, Z)` 求入射角，
Z<0 时给钝角 —— 数学上正确（鱼眼能拍后半球），但 **NFOCC 的可见性
判据是 `theta <= max_theta`**，不是 Z>0。

实测差异：`X=Y=0, Z=-1000` 时 NFOCC 判无效（θ=π > 1.9），
vcam 报有效。桥接层已补上 `max_theta` 与 `r_valid` 检查
（`vcam_bridge.cpp` 的 `nf_vcam_project`），否则后方视口会拿
物理上不存在的像素去采样。

---

## 7. 已知限制

| 项 | 说明 |
|---|---|
| aarch64 未上板 | 只验证了产物架构与依赖，未在 RK3588/M57 运行 |
| vcam 只接管 project | unproject 仍走 NFOCC，因为 LUT 只用 project |
| Debug 构建未跑满测试 | depth_accuracy 在 -O0 下很慢，按需单独跑 |
