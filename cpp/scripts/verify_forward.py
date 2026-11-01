#!/usr/bin/env python3
"""前向匹配与前向路径的验收脚本（工程内常驻，可反复复跑）。

    python3 scripts/verify_forward.py

## 为什么做成常驻脚本
先前写成临时脚本，验证干净后就被清理掉，导致"改过但无有效证据"。
验收脚本本身是可交付物的一部分，和 tests/ 一样应当长期存在。

## 判据来源
- 亚像素误差 <0.5 px：SPEC-FORWARD §4（200mm 体素要求，1px ↔ 110mm）
- 无关纹理 <10%：无匹配结构时不能编造深度
- 前向非零：此前该路径恒返回 0 点
- 侧向不回归：改动前是 812 点量级
"""
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
results = []


def check(name, ok, detail):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def sh(*a, **kw):
    return subprocess.run(a, capture_output=True, text=True, timeout=kw.pop("t", 600))


def main():
    b = Path(tempfile.mkdtemp(prefix="nfocc-yfwd-"))
    try:
        print("[1] 构建")
        r = sh("cmake", "-S", str(ROOT), "-B", str(b), "-DCMAKE_BUILD_TYPE=Release")
        check("配置成功", r.returncode == 0, f"rc={r.returncode}")
        if r.returncode != 0:
            return 1
        r = sh("cmake", "--build", str(b), "-j8")
        check("构建零错误", r.returncode == 0,
              "ok" if r.returncode == 0 else r.stderr[-600:])
        if r.returncode != 0:
            return 1

        print("\n[2] 匹配精度")
        r = sh(str(b / "tests" / "test_match"), t=300)
        out = r.stdout
        check("test_match 全过", r.returncode == 0, out.strip().splitlines()[-1])
        m = re.search(r"径向误差 均值 ([\d.]+) px, 最大 ([\d.]+) px, >0\.5px 占 ([\d.]+)%", out)
        if m:
            check("均值 <0.5px", float(m.group(1)) < 0.5, f"{m.group(1)} px")
            check("超 0.5px 占比 <10%", float(m.group(3)) < 10.0, f"{m.group(3)}%")
        m2 = re.search(r"无关图: 有效匹配 \d+ / \d+ \(([\d.]+)%\)", out)
        if m2:
            check("无关图 <10%", float(m2.group(1)) < 10.0, f"{m2.group(1)}%")

        exe = str(b / "nfocc_run")

        print("\n[3] 前向路径非零（改前恒为 0）")
        fwd = sh(exe, "--synthetic", "--forward", "--frames", "4", "--stats", t=300).stdout
        mf = re.search(r"累计点数\s*(\d+)", fwd)
        pts = int(mf.group(1)) if mf else -1
        check("前向产出非零", pts > 0, f"{pts} 点")
        cams = re.findall(r"cam\d mode=FORWARD\s+status=(\S+)", fwd)
        check("前向状态全 NF_OK", bool(cams) and all(c == "NF_OK" for c in cams),
              f"{len(cams)} 条")

        print("\n[4] 侧向不回归")
        side = sh(exe, "--synthetic", "--frames", "4", "--stats", t=300).stdout
        ms = re.search(r"累计点数\s*(\d+)", side)
        ps = int(ms.group(1)) if ms else -1
        check("侧向产出非零", ps > 0, f"{ps} 点")
        cams2 = re.findall(r"cam\d mode=STEREO\s+status=(\S+)", side)
        check("侧向状态全 NF_OK", bool(cams2) and all(c == "NF_OK" for c in cams2),
              f"{len(cams2)} 条")

        print("\n[5] 位移真的进了三角化（接线错时点云会看着正常）")
        far = sh(exe, "--synthetic", "--forward", "--frames", "4",
                 "--move", "400", t=300).stdout
        mff = re.search(r"累计点数\s*(\d+)", far)
        pf = int(mff.group(1)) if mff else -1
        print(f"      200mm: {pts} 点, 400mm: {pf} 点")
        check("两档位移都产出", pts > 0 and pf > 0, f"{pts} vs {pf}")

        print("\n[6] 全套测试")
        r = sh("ctest", "--test-dir", str(b), "--output-on-failure", t=600)
        m = re.search(r"(\d+)% tests passed, (\d+) tests failed out of (\d+)", r.stdout)
        if m:
            check(f"ctest {m.group(1)}% ({m.group(3)}-{m.group(2)})",
                  m.group(2) == "0", f"{m.group(2)} failed")
        else:
            check("ctest 可解析", False, r.stdout[-300:])

        print("\n[7] 核心库零第三方")
        link = b / "CMakeFiles" / "nfocc_core.dir" / "link.txt"
        if link.exists():
            bad = [k for k in ("opencv", "zmq", "onnx") if k in link.read_text().lower()]
            check("无第三方", not bad, "干净" if not bad else str(bad))
    finally:
        shutil.rmtree(b, ignore_errors=True)

    print("\n" + "=" * 62)
    fails = [x for x in results if not x[1]]
    print(f"{len(results)} 项, 失败 {len(fails)}")
    for n, _, d in fails:
        print(f"  FAIL {n}: {d}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
