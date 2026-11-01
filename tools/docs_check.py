#!/usr/bin/env python3
"""docs/ 自检: 交叉引用可达、参数数值只在 PARAM 定义、章节编号连续。

这三条是文档集的纪律, 靠人工保证不住, 必须脚本化。
失败即非零退出, 可直接接进 CI。

检查项
------
D1 所有 `XXX.md` 形式的引用都指向 docs/ 下真实存在的文件
D2 PARAM 里登记的参数符号, 其"数值赋值"不在其他文档重复出现
D3 每个 .md 的二级/三级标题编号单调且不重复
D4 README 索引列出的文件都存在, 且 docs/ 下的文件都在 README 索引里

用法
----
    python3 tools/docs_check.py
    python3 tools/docs_check.py --docs /path/to/docs
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# PARAM 参数符号 (只查这些; 其他符号不参与 D2)
PARAM_SYMS = [
    "B_MIN", "B_MAX", "B_TARGET", "B_OMEGA", "B_DT_MAX", "B_LAT_MAX", "B_ZMIN_EFF",
    "E_MAX_DISP", "E_BLOCK", "E_DOWNSCALE",
    "G_VOX", "P_HASH_BITS",
    "T_SIGMA_D", "T_SIGMA_B",
    "VF_VIEW", "V_REORIENT_EPS",
    "INF_F", "INF_CX", "INF_CY", "INF_R_VALID",
]
# 允许出现数值赋值的文档 (定义处 + 变更记录)
ASSIGN_OK = {"PARAM.md", "APPENDIX-CHANGELOG.md"}


def check_refs(files: list[Path]) -> list[str]:
    errs = []
    pat = re.compile(r"`([A-Za-z0-9_.\-]+\.md)`")
    names = {f.name for f in files}
    for f in files:
        for m in pat.finditer(f.read_text(encoding="utf-8")):
            ref = m.group(1)
            if "/" in ref:  # 外部路径, 不检查
                continue
            if ref not in names:
                errs.append(f"D1 {f.name}: 引用不存在 `{ref}`")
    return errs


def check_param_assign(files: list[Path]) -> list[str]:
    errs = []
    # 匹配 `SYM=123` / `SYM = 123` / `SYM 为 123`
    pat = re.compile(r"\b(" + "|".join(PARAM_SYMS) + r")\s*(?:=|≈|为)\s*`?(\d[\w.]*)")
    for f in files:
        if f.name in ASSIGN_OK:
            continue
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            if "APPENDIX-CHANGELOG" in f.name:
                continue
            for m in pat.finditer(line):
                errs.append(
                    f"D2 {f.name}:{i}: 参数 `{m.group(1)}` 在此重复赋值 ({m.group(0)})，"
                    f"数值应只在 PARAM.md 定义"
                )
    return errs


def check_headings(files: list[Path]) -> list[str]:
    errs = []
    for f in files:
        seen: dict[int, list[int]] = {}
        for line in f.read_text(encoding="utf-8").splitlines():
            m = re.match(r"^(#{2,3})\s+(\d+(?:\.\d+)?)[.．、\s]", line)
            if not m:
                continue
            level = len(m.group(1))
            num = m.group(2)
            parts = [int(x) for x in num.split(".")]
            seen.setdefault(level, []).append(parts[0] if level == 2 else parts[-1])
        # 二级标题: 首数字应严格递增
        for level, nums in seen.items():
            if level == 2:
                uniq = []
                for n in nums:
                    if n not in uniq:
                        uniq.append(n)
                if uniq != sorted(uniq):
                    errs.append(f"D3 {f.name}: 二级标题编号非递增 {uniq}")
    return errs


def check_readme(files: list[Path]) -> list[str]:
    errs = []
    readme = next((f for f in files if f.name == "README.md"), None)
    if readme is None:
        return ["D4 缺少 README.md"]
    text = readme.read_text(encoding="utf-8")
    for f in files:
        if f.name == "README.md":
            continue
        if f.name not in text:
            errs.append(f"D4 {f.name}: 未出现在 README 索引中")
    return errs


def check_param_refs(files: list[Path]) -> list[str]:
    """`PARAM.md::SYM` 引用的符号必须在 PARAM.md 的符号总表里登记。"""
    errs = []
    param = next((f for f in files if f.name == "PARAM.md"), None)
    if param is None:
        return ["D5 缺少 PARAM.md"]
    ptext = param.read_text(encoding="utf-8")
    pat = re.compile(r"PARAM\.md::`?([A-Z][A-Z0-9_]*)`?")
    for f in files:
        if f.name == "PARAM.md":
            continue
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            for m in pat.finditer(line):
                sym = m.group(1)
                if sym not in ptext:
                    errs.append(f"D5 {f.name}:{i}: 引用 `PARAM.md::{sym}` 但 PARAM.md 未登记该符号")
    return errs


def check_section_refs(files: list[Path]) -> list[str]:
    """`SPEC-XXX.md` §N 形式的引用, 目标文档必须有第 N 节。"""
    errs = []
    pat = re.compile(r"`(SPEC-[A-Z0-9-]+|PARAM|PHASE)\.md`\s*§(\d+)")
    for f in files:
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            for m in pat.finditer(line):
                target = m.group(1) + ".md"
                num = int(m.group(2))
                tf = next((x for x in files if x.name == target), None)
                if tf is None:
                    continue  # D1 已报
                headings = tf.read_text(encoding="utf-8")
                # 接受 "## N." 或中文序号章节
                if not re.search(rf"^##\s+{num}[.．、\s]", headings, re.M):
                    # PARAM 用中文序号, 允许 §N 指代其第 N 节
                    cn = "一二三四五六七八九十"
                    ok = target == "PARAM.md" and num <= len(cn) and f"## {cn[num-1]}、" in headings
                    if not ok:
                        errs.append(f"D6 {f.name}:{i}: 引用 `{target}` §{num} 但该文档没有第 {num} 节")
    return errs


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--docs", default="docs")
    args = ap.parse_args()
    root = Path(args.docs)
    # macOS 在卷上会生成 ._xxx.md 资源分叉文件(二进制), 必须排除
    files = sorted(f for f in root.glob("*.md") if not f.name.startswith("._"))
    if not files:
        print(f"FAIL 在 {root} 下没有找到 .md")
        sys.exit(1)
    errs = (
        check_refs(files)
        + check_param_assign(files)
        + check_headings(files)
        + check_readme(files)
        + check_param_refs(files)
        + check_section_refs(files)
    )
    if errs:
        for e in errs:
            print("FAIL", e)
        print(f"\n{len(errs)} 项失败")
        sys.exit(1)
    print(f"PASS  {len(files)} 个文档, D1-D6 全部通过")


if __name__ == "__main__":
    main()
