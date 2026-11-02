#!/usr/bin/env bash
# NFOCC 构建脚本 — 覆盖本机(macOS)、Linux PC、Aarch64 三种目标
#
# 用法:
#   ./scripts/build.sh host          # 本机 Release + 测试
#   ./scripts/build.sh linux         # Linux PC (在 Linux 上执行)
#   ./scripts/build.sh aarch64       # Aarch64 交叉编译
#   ./scripts/build.sh core          # 只编核心库(零第三方依赖)
#   ./scripts/build.sh clean
#
# ## 为什么要有这个脚本
# 三套目标的 cmake 参数差异大(工具链文件、依赖来源、是否带引擎),
# 靠记忆敲命令必然出错。参数在这里固化, CI 与本地用同一份。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

JOBS="${NF_JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)}"
TARGET="${1:-host}"

# vcam 后端（可选）。设置 NF_VCAM_DIR 环境变量即可启用：
#   NF_VCAM_DIR=/Volumes/Workspace/virtualcamera/cpp ./scripts/build.sh host
EXTRA_VCAM=()
if [ -n "${NF_VCAM_DIR:-}" ]; then
  EXTRA_VCAM+=("-DNF_WITH_VCAM=ON" "-DNF_VCAM_DIR=$NF_VCAM_DIR")
fi

msg() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }

case "$TARGET" in
  host)
    msg "本机构建 (${JOBS} 线程)"
    # macOS 用 sysctl, Linux 用 nproc —— 已在顶部做回退
    cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Release \
          ${EXTRA_VCAM[@]+"${EXTRA_VCAM[@]}"}
    cmake --build build/host -j"$JOBS"
    msg "测试"
    ctest --test-dir build/host --output-on-failure
    ;;

  linux)
    msg "Linux PC 构建"
    # Linux 上依赖装在系统路径, 用 managed 模式
    cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Release -DNF_DEPS=managed \
          ${EXTRA_VCAM[@]+"${EXTRA_VCAM[@]}"}
    cmake --build build/linux -j"$JOBS"
    ctest --test-dir build/linux --output-on-failure
    ;;

  debug)
    # 调试构建：-O0 -g，关闭 NDEBUG 让 assert 生效。
    # 与 Release 分开目录，避免 cmake 缓存里 BUILD_TYPE 打架。
    msg "本机调试构建 (Debug, 无优化 + 调试符号)"
    cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug \
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
          ${EXTRA_VCAM[@]+"${EXTRA_VCAM[@]}"}
    cmake --build build/debug -j"$JOBS"
    msg "调试构建完成。调试方式："
    echo "  lldb build/debug/nfocc_run -- --hanqtech ../datasets/fisheye --ws-frame 164"
    echo "  gdb  build/debug/nfocc_run   (Linux)"
    echo "  或 VS Code: 用 build/debug/compile_commands.json"
    ;;

  aarch64)
    msg "Aarch64 交叉编译"
    TOOLCHAIN="$ROOT/cmake/toolchain-aarch64.cmake"
    if [ ! -f "$TOOLCHAIN" ]; then
      echo "缺少 $TOOLCHAIN" >&2; exit 1
    fi
    # 交叉编译时没有 aarch64 的 OpenCV/ZeroMQ, 默认只编核心库 + 测试
    # 若有 arm64 sysroot 或 vcpkg, 加 NF_SYSROOT=... / NF_VCPKG_ROOT=...
    EXTRA=()
    [ -n "${NF_SYSROOT:-}" ]    && EXTRA+=("-DNF_SYSROOT=$NF_SYSROOT")
    [ -n "${NF_VCPKG_ROOT:-}" ] && EXTRA+=("-DNF_VCPKG_ROOT=$NF_VCPKG_ROOT")
    # 编译器: 优先用 NF_CROSS_CXX/NF_CROSS_C, 否则由 toolchain 文件在 PATH 里找
    if [ -n "${NF_CROSS_CXX:-}" ]; then
      EXTRA+=("-DNF_CROSS_CXX=$NF_CROSS_CXX")
      [ -n "${NF_CROSS_C:-}" ] && EXTRA+=("-DNF_CROSS_C=$NF_CROSS_C")
    fi

    cmake -S . -B build/arm \
          -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
          -DCMAKE_BUILD_TYPE=Release \
          -DNF_DEPS=vendored \
          -DNF_WITH_OPENCV=OFF \
          -DNF_WITH_ZMQ=OFF \
          -DNF_WITH_ORT=OFF \
          ${EXTRA[@]+"${EXTRA[@]}"}
    cmake --build build/arm -j"$JOBS"
    msg "产物架构校验"
    FILE_BIN=$(find build/arm -name 'nfocc_run' -type f -perm -u+x | head -1)
    [ -z "$FILE_BIN" ] && FILE_BIN=$(find build/arm -name 'test_geometry' -type f -perm -u+x | head -1)
    if [ -n "$FILE_BIN" ]; then
      file "$FILE_BIN" | sed 's/, BuildID.*//'
      if file "$FILE_BIN" | grep -qi "ARM aarch64"; then
        printf '\033[1;32m  架构正确: aarch64\033[0m\n'
      else
        printf '\033[1;31m  架构错误: 不是 aarch64\033[0m\n'; exit 1
      fi
      # 依赖审计: 不得出现 macOS 的 .dylib
      OBJDUMP="$(dirname "${NF_CROSS_CXX:-aarch64-unknown-linux-gnu-g++}")/aarch64-unknown-linux-gnu-objdump"
      if [ -x "$OBJDUMP" ]; then
        if "$OBJDUMP" -p "$FILE_BIN" | grep -qiE 'darwin|\.dylib'; then
          printf '\033[1;31m  依赖审计失败: 误链了 macOS 库\033[0m\n'; exit 1
        fi
        printf '  依赖审计通过 (无 macOS 库)\n'
        "$OBJDUMP" -p "$FILE_BIN" | grep NEEDED | head -6
      fi
    else
      printf '\033[1;31m  未找到可执行产物\033[0m\n'; exit 1
    fi
    ;;

  core)
    msg "仅核心库 (零第三方依赖)"
    cmake -S . -B build/core -DCMAKE_BUILD_TYPE=Release \
          -DNF_WITH_OPENCV=OFF -DNF_WITH_ZMQ=OFF -DNF_WITH_ORT=OFF \
          -DNF_BUILD_TOOLS=OFF
    cmake --build build/core -j"$JOBS"
    msg "核心库依赖检查 (应无 OpenCV/ZeroMQ/ORT)"
    if command -v ldd >/dev/null 2>&1; then
      ldd build/core/libnfocc_core.a 2>/dev/null || echo "  静态库无动态依赖 (符合预期)"
    elif command -v otool >/dev/null 2>&1; then
      otool -L build/core/libnfocc_core.a 2>/dev/null | grep -iE "opencv|zmq|onnx" \
        && { echo "  核心库不应依赖第三方"; exit 1; } || echo "  无第三方依赖 (符合预期)"
    fi
    ;;

  clean)
    msg "清理构建目录"
    rm -rf build/host build/linux build/arm build/core build/debug
    msg "完成"
    ;;

  *)
    echo "用法: $0 {host|linux|debug|aarch64|core|clean}" >&2
    exit 2
    ;;
esac

msg "完成"
