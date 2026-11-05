#!/bin/sh
# docs 同步现状的定向验证。
#
# 检查器只验"文档互引不悬空"，不验"文档说的符号在代码里存在"。
# 这个脚本补上后者 —— 那才是"同步工程现状"的实质。
set -u
ROOT=/Volumes/Workspace/NFOCC
D=$ROOT/docs
C=$ROOT/cpp
fail=0
ok()  { printf '  [PASS] %-24s %s\n' "$1" "$2"; }
bad() { printf '  [FAIL] %-24s %s\n' "$1" "$2"; fail=$((fail+1)); }

echo "== [1] docs_check =="
OUT=$(cd "$ROOT" && python3 tools/docs_check.py 2>&1); RC=$?
printf '%s\n' "$OUT" | tail -1 | sed 's/^/    /'
[ "$RC" -eq 0 ] && ok docs_check "全绿" || bad docs_check "$(printf '%s' "$OUT" | grep FAIL | head -3)"

echo "== [2] 已删文档确实不在 =="
for f in PH1-ACCEPTANCE.md PROJECT-STATUS.md PLAN-CLUSTERING.md \
         1_NFOCC待办.txt 2_SPEC-ALGORITHM-DERIVATION.md; do
    [ -f "$D/$f" ] && bad "已删 $f" "仍存在" || ok "已删 $f" "不存在"
done
[ -d "$ROOT/nfocc_ph0" ] && bad "nfocc_ph0" "仍存在" || ok nfocc_ph0 "不存在"
[ -d "$ROOT/nfocc_ph1" ] && bad "nfocc_ph1" "仍存在" || ok nfocc_ph1 "不存在"

echo "== [3] README 列出的文档全部存在 =="
MISS=""
# 从 README 的表格里抽 `xxx.md` 形式的条目（反引号包裹 = 真实引用）
for n in $(grep -oE '`[A-Z0-9a-z_.-]+\.md`' "$D/README.md" | tr -d '`' | sort -u); do
    [ -f "$D/$n" ] || MISS="$MISS $n"
done
if [ -z "$MISS" ]; then ok README引用 "全部存在"; else bad README引用 "缺失:$MISS"; fi

echo "== [4] 反向：实际存在的 .md 是否都在 README 里登记 =="
LISTED=$(grep -oE '`[A-Z0-9a-z_.-]+\.md`' "$D/README.md" | tr -d '`')
ORPHAN=""
for f in "$D"/*.md; do
    b=$(basename "$f")
    [ "$b" = "README.md" ] && continue
    printf '%s\n' "$LISTED" | grep -qx "$b" || ORPHAN="$ORPHAN $b"
done
if [ -z "$ORPHAN" ]; then ok 索引完整 "无遗漏"; else bad 索引完整 "未登记:$ORPHAN"; fi

echo "== [5] 文档声明已接入的符号，代码里真的在调用 =="
# 文档说"已接入"的符号，就必须在 pipeline 里能找到调用点。
# D4 与双基线 2026-11-02 已接入，从下面"未接入"组移到这一组。
for sym in nf_pt_in_foe nf_compute_spatial_uncertainty_cm \
           nf_voxel_verify_cross nf_verify_thresh_mm \
           nf_baseline_manager_push nf_baseline_manager_decide; do
    N=$(grep -c "$sym" "$C/src/pipeline/pipeline.cpp" 2>/dev/null)
    [ "$N" -gt 0 ] && ok "$sym" "pipeline 调用 $N 处" || bad "$sym" "文档说已接入但 0 处"
done
# 文档说"未接入"的符号，必须确实是 0 处
for sym in nf_occlusion_weight; do
    N=$(grep -c "$sym" "$C/src/pipeline/pipeline.cpp" 2>/dev/null)
    [ "$N" -eq 0 ] && ok "$sym" "确认未接入(0)" || bad "$sym" "文档说未接入但有 $N 处"
done

echo "== [6] 状态行覆盖 =="
NOSTAT=""
for f in SPEC-MASK.md SPEC-OCCLUSION.md SPEC-BASELINE-SM.md SPEC-FOE.md SPEC-POSE.md; do
    head -6 "$D/$f" 2>/dev/null | grep -q "状态：" || NOSTAT="$NOSTAT $f"
done
if [ -z "$NOSTAT" ]; then ok 状态行 "5 篇均有"; else bad 状态行 "缺:$NOSTAT"; fi

echo "== [7] 判据自证伪 =="
[ -n "${NF_CANARY:-}" ] && bad "canary(故意)" "本段可失败"

echo
echo "失败 $fail 项"
[ "$fail" -eq 0 ] || exit 1
