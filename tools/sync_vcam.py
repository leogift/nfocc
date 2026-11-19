#!/usr/bin/env python3
"""把 vcam 编成静态库并放进 cpp/third_party/vcam，供 NFOCC 直接链接。

## 为什么预编译而不是 add_subdirectory
- NFOCC 的第三方依赖统一放在 `third_party/<name>/{include,lib}`，
  源码树里不出现外部工程的 CMakeLists，交叉编译时不会意外带进
  vcam 的工具层（那层依赖 OpenCV）。
- 预编译库的体积和 ABI 可控，交付时不用附带 vcam 全套源码。

## 编码/分支坑
vcam 的源码在 macOS 上会生成 `._xxx` AppleDouble 文件，它的 CMakeLists
已经用 list(FILTER ... EXCLUDE REGEX) 排除。同步时也要跳过，
否则复制过去会让下游的 GLOB 报编译错。

用法:
    python3 tools/sync_vcam.py                    # 用默认路径
    python3 tools/sync_vcam.py --src /path/to/virtualcamera/cpp
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DST = ROOT / "cpp" / "third_party" / "vcam"
DEFAULT_SRC = Path("/Volumes/Workspace/virtualcamera/cpp")


def run(cmd, cwd=None):
    print("  $", " ".join(str(c) for c in cmd))
    r = subprocess.run(cmd, cwd=cwd)
    if r.returncode != 0:
        sys.exit(f"失败: {cmd[0]} 退出码 {r.returncode}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(DEFAULT_SRC))
    ap.add_argument("--build-dir", default=None)
    args = ap.parse_args()

    src = Path(args.src)
    if not (src / "CMakeLists.txt").exists():
        sys.exit(f"找不到 vcam 源码: {src}")

    build = Path(args.build_dir) if args.build_dir else src / "build"

    print("[1/4] 编译 vcam_core")
    run(["cmake", "-S", str(src), "-B", str(build),
         "-DCMAKE_BUILD_TYPE=Release",
         "-DVCAM_BUILD_TOOLS=OFF", "-DVCAM_BUILD_TESTS=OFF"])
    run(["cmake", "--build", str(build), "--target", "vcam_core", "-j", "8"])

    lib = build / "libvcam_core.a"
    if not lib.exists():
        sys.exit(f"没找到产物: {lib}")
    kb = lib.stat().st_size / 1024
    print(f"      libvcam_core.a  {kb:.0f} KB")

    print(f"[2/4] 复制到 {DST}")
    if DST.exists():
        shutil.rmtree(DST)
    (DST / "lib").mkdir(parents=True)
    (DST / "include").mkdir(parents=True)

    shutil.copy2(lib, DST / "lib" / "libvcam_core.a")
    # 头文件：跳过 AppleDouble
    n = 0
    for f in (src / "include").rglob("*.h"):
        if f.name.startswith("._"):
            continue
        rel = f.relative_to(src / "include")
        (DST / "include" / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, DST / "include" / rel)
        n += 1
    print(f"      {n} 个头文件")

    print("[3/4] 生成 CMake 导入配置")
    (DST / "vcam.cmake").write_text(
        "# 由 tools/sync_vcam.py 生成，勿手改。\n"
        "# NFOCC 直接链接这个预编译库，不用 add_subdirectory。\n"
        'set(NF_VCAM_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")\n'
        'set(NF_VCAM_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/libvcam_core.a")\n',
        encoding="utf-8")

    print("[4/4] 记录版本")
    ver = "unknown"
    for line in (src / "CMakeLists.txt").read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("project("):
            parts = line.split("VERSION")
            if len(parts) > 1:
                ver = parts[1].split(")")[0].strip()
            break
    (DST / "VERSION.txt").write_text(
        f"vcam {ver}\n"
        f"source: {src}\n"
        f"lib: {kb:.0f} KB\n"
        f"built by: tools/sync_vcam.py\n", encoding="utf-8")
    print(f"      vcam {ver}")

    print(f"\n完成: {DST}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
