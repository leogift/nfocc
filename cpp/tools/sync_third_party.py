# 把系统已安装的第三方库同步到工程内 `third_party/`
#
# 用途: 满足"三方依赖放进工程目录, 用 CMake + RPATH 管理"的要求,
#       使工程自包含 —— 换机器/上 Aarch64 板端时整个目录拷过去即可运行。
#
# 用法:
#   python3 tools/sync_third_party.py            # 同步(拷贝)
#   python3 tools/sync_third_party.py --dry-run  # 只看会拷什么
#   python3 tools/sync_third_party.py --link     # 建软链(省磁盘, 不可移植)
#
# 拷什么:
#   eigen/        -> 纯头文件
#   openblas/     -> dylib/so + 头文件
#   libomp/       -> dylib/so (macOS 上 AppleClang 必须显式链接它)
#   opencv/       -> 只拷实际用到的模块(core/imgproc/stereo)及其依赖链
#   zeromq/       -> libzmq + 头文件
#   onnxruntime/  -> libonnxruntime + 头文件
#
# 注意: **不拷系统框架**(Accelerate/CoreFoundation 等), 那些由 OS 提供。
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
THIRD = ROOT / "third_party"

# 每个条目: (名字, [头文件目录], [需要拷的动态库 glob], [pkg-config 名])
PKGS = [
    ("eigen",       ["include/eigen3"],                       [],                                    None),
    ("openblas",    ["include"],  ["lib/libopenblas*.dylib", "lib/libopenblas*.a"],                 "openblas"),
    ("libomp",      ["include"],  ["lib/libomp.dylib", "lib/libomp.so"],                            None),
    ("zeromq",      ["include"],  ["lib/libzmq*.dylib", "lib/libzmq.so*"],                          "libzmq"),
    ("onnxruntime", ["include"],  ["lib/libonnxruntime*.dylib", "lib/libonnxruntime*.so*"],          None),
    ("opencv",      ["include/opencv5", "include/opencv4"],
     ["lib/libopencv_core*.dylib", "lib/libopencv_imgproc*.dylib",
      "lib/libopencv_stereo*.dylib", "lib/libopencv_calib3d*.dylib",
      "lib/libopencv_core*.so*", "lib/libopencv_imgproc*.so*",
      "lib/libopencv_stereo*.so*", "lib/libopencv_calib3d*.so*"], None),
]


def brew_prefix(name: str):
    """查 Homebrew 安装前缀。优先用 brew --prefix, 失败则扫常见路径。"""
    try:
        out = subprocess.run(["brew", "--prefix", name], capture_output=True,
                             text=True, timeout=10)
        if out.returncode == 0:
            p = Path(out.stdout.strip())
            if p.is_dir():
                return p
    except (OSError, subprocess.SubprocessError):
        pass
    for base in ("/opt/homebrew/opt", "/usr/local/opt", "/usr"):
        p = Path(base) / name
        if p.is_dir():
            return p
    return None


def sync_one(name, incs, libs, dry, link):
    src_root = brew_prefix(name)
    if src_root is None:
        print(f"  [跳过] {name}: 未找到安装前缀")
        return 0
    dst = THIRD / name
    copied = 0
    for inc in incs:
        s = src_root / inc
        if not s.is_dir():
            continue
        d = dst / inc
        print(f"    {s} -> {d}")
        if not dry:
            d.parent.mkdir(parents=True, exist_ok=True)
            if link:
                if d.exists() or d.is_symlink():
                    pass
                else:
                    d.symlink_to(s)
            else:
                if d.exists():
                    shutil.rmtree(d)
                shutil.copytree(s, d, symlinks=False)
        copied += 1
    for pat in libs:
        for s in sorted(src_root.glob(pat)):
            if s.name.startswith("._"):
                continue      # macOS 资源分支, 不是库
            if s.is_symlink() and not link:
                continue      # 拷模式下拷真实文件, 不留符号链接(可移植)
            if s.name.endswith(".dylib") and s.is_symlink():
                continue
            d = dst / "lib" / s.name
            print(f"    {s} -> {d}")
            if not dry:
                d.parent.mkdir(parents=True, exist_ok=True)
                if link:
                    if not (d.exists() or d.is_symlink()):
                        d.symlink_to(s)
                else:
                    shutil.copy2(s, d)
            copied += 1
    return copied


def fix_rpath_macos(dry):
    """把拷进来的 dylib 的依赖路径改成本目录(@loader_path), 使其自包含。"""
    if sys.platform != "darwin":
        return
    libdir = THIRD
    dylibs = [d for d in libdir.glob("*/lib/*.dylib")
              if not d.is_symlink() and not d.name.startswith("._")]
    if not dylibs:
        print("  (软链接模式, 跳过 install_name 修改 —— 改符号链接会破坏系统原库)")
        return
    print(f"  修正 {len(dylibs)} 个 dylib 的 install_name/RPATH ...")
    for d in dylibs:
        if dry:
            continue
        # 1) 改自己的 install_name 为 @rpath/<basename>
        subprocess.run(["install_name_tool", "-id", f"@rpath/{d.name}", str(d)],
                       capture_output=True)
        # 2) 若引用了 /opt/homebrew 下的同级库, 改成本目录相对
        out = subprocess.run(["otool", "-L", str(d)], capture_output=True, text=True)
        for line in out.stdout.splitlines()[1:]:
            dep = line.strip().split(" ")[0]
            base = Path(dep).name
            if (libdir.name in dep or "homebrew" in dep) and base != d.name:
                subprocess.run(["install_name_tool", "-change", dep,
                                f"@rpath/{base}", str(d)], capture_output=True)


def main():
    ap = argparse.ArgumentParser(description="同步第三方库到 third_party/")
    ap.add_argument("--dry-run", action="store_true", help="只打印, 不拷")
    ap.add_argument("--link", action="store_true",
                    help="建软链接而非拷贝(省空间, 但不可移植)")
    a = ap.parse_args()

    THIRD.mkdir(parents=True, exist_ok=True)
    total = 0
    for name, incs, libs, _pc in PKGS:
        print(f"[{name}]")
        n = sync_one(name, incs, libs, a.dry_run, a.link)
        total += n
        if n == 0:
            print(f"  (无内容)")

    fix_rpath_macos(a.dry_run)

    # 写一个清单, 供 CMake 校验与文档追溯
    if not a.dry_run:
        with open(THIRD / "MANIFEST.txt", "w", encoding="utf-8") as f:
            f.write("# 由 tools/sync_third_party.py 生成\n")
            for name, _i, _l, _p in PKGS:
                d = THIRD / name
                if d.is_dir():
                    f.write(f"{name}\n")
    print(f"\n完成: {total} 项 -> {THIRD}")
    if a.dry_run:
        print("(dry-run, 未实际拷贝)")


if __name__ == "__main__":
    main()
