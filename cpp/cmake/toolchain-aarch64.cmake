# aarch64-linux-gnu 交叉编译工具链
#
# ## macOS 上为什么需要它
# Apple Clang 只能产 Darwin 目标, 无法链接 Linux 的 glibc。macOS 上做
# aarch64-linux 交叉编译必须用 messense/macos-cross-toolchains 的
# aarch64-unknown-linux-gnu (GNU binutils + glibc, 跑在 macOS 上)。
#
# ## 用法
#   cmake -S . -B build_arm \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake \
#         -DNF_DEPS=vendored
#
# ## 关键: 依赖必须也是 aarch64 的
# 宿主机 Homebrew 里的库是 arm64-**darwin**, 不能链接到 linux 目标。
# 因此交叉编译时:
#   - 有 vcpkg: 用 arm64-linux triplet (见 NF_VCPKG_TRIPLET)
#   - 否则: 必须提供 aarch64 的 sysroot 或预编译库
# 本文件会在找不到 aarch64 依赖时**明确报错**, 而不是悄悄链接错架构。

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ---------------------------------------------------------------- 编译器
# 优先用 PATH 里的, 其次用 Homebrew 的固定路径
# ## 工具链文件会被 include 两次
# CMake 在主工程配置时 include 一次, 之后 try_compile 的子工程里**再 include 一次**。
# 第二次时 -D 传的缓存变量不可见(缓存里只有 CMAKE_ 前缀的), 于是 NF_CROSS_CXX 为空。
# 若此时直接 FATAL_ERROR, 症状是"编译器明明识别成功了却报找不到编译器"。
#
# 正确做法: 只在**还没设 CMAKE_CXX_COMPILER** 时才去找, 并改用 CACHE 存放,
# 这样两次 include 都能读到同一份。
# 只在还没确定编译器时才搜索。**不清 CACHE**, 不加 FATAL_ERROR 早退 ——
# try_compile 的子工程会重新 include 本文件, 那时缓存变量不可见,
# 提前 FATAL 会导致"编译器明明可用却报找不到"。
if(NOT CMAKE_CXX_COMPILER)
  # 搜索顺序: 显式变量 > PATH > 常见安装位置
  if(NOT NF_CROSS_CXX)
    find_program(NF_CROSS_CXX NAMES aarch64-unknown-linux-gnu-g++
                              aarch64-linux-gnu-g++
               HINTS $ENV{HOME}/toolchains/aarch64-unknown-linux-gnu/bin
                     /opt/homebrew/opt/aarch64-unknown-linux-gnu/bin
                     /usr/local/opt/aarch64-unknown-linux-gnu/bin)
  endif()
  if(NOT NF_CROSS_C)
    find_program(NF_CROSS_C NAMES aarch64-unknown-linux-gnu-gcc
                            aarch64-linux-gnu-gcc
               HINTS $ENV{HOME}/toolchains/aarch64-unknown-linux-gnu/bin
                     /opt/homebrew/opt/aarch64-unknown-linux-gnu/bin
                     /usr/local/opt/aarch64-unknown-linux-gnu/bin)
  endif()
endif()

if(NF_CROSS_CXX)
  set(CMAKE_CXX_COMPILER "${NF_CROSS_CXX}")
  set(CMAKE_C_COMPILER   "${NF_CROSS_C}")
  message(STATUS "NFOCC/cross: CXX=${NF_CROSS_CXX}")
endif()

# ---------------------------------------------------------------- 查找策略
# **只在目标前缀与 sysroot 下找库, 绝不回退宿主** —— 否则会静默链到
# arm64-darwin 的 .dylib, 链接期报"格式不兼容", 或者更糟: 侥幸链上但运行崩。
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)   # 工具要在宿主上跑
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

if(NF_SYSROOT)
  set(CMAKE_SYSROOT "${NF_SYSROOT}")
  list(APPEND CMAKE_FIND_ROOT_PATH "${NF_SYSROOT}")
endif()

# ---------------------------------------------------------------- 目标特性
# RK3588 = 4×Cortex-A76 + 4×Cortex-A55, M57 = Cortex-A55
# armv8-a + NEON 是两者共同下限; 不加 -mcpu=native(交叉编译无意义)
set(NF_ARM_ARCH_FLAGS "-march=armv8-a+simd -mtune=cortex-a55")
set(CMAKE_C_FLAGS_INIT   "${NF_ARM_ARCH_FLAGS} -O3 -fPIC")
set(CMAKE_CXX_FLAGS_INIT "${NF_ARM_ARCH_FLAGS} -O3 -fPIC")

# ---------------------------------------------------------------- 依赖
# 交叉编译时的依赖来源, 按优先级:
#   1. vcpkg + arm64-linux triplet
#   2. NF_SYSROOT 下的预编译库
#   3. 全部关闭(NF_DEPS=system 且不启用第三方引擎) -> 只编核心库
if(NF_VCPKG_ROOT)
  set(VCPKG_TARGET_TRIPLET "arm64-linux" CACHE STRING "" FORCE)
  set(CMAKE_TOOLCHAIN_FILE
      "${NF_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "" FORCE)
  message(STATUS "NFOCC/cross: 使用 vcpkg (${VCPKG_TARGET_TRIPLET})")
endif()

# OpenMP 交叉编译: GCC 的 libgomp 随编译器提供, 不需要额外找
set(NF_CROSS_OPENMP 1)
