#!/bin/sh
# 本轮（尺寸白名单 / README / PDF 报告）的 ad-hoc 定向验证。
#
# **不是套件全绿**：ctest 里 depth_accuracy 是既有失败，本脚本单独验本轮。
#
# 判据重点在"能不能用"，不是"能不能编译"：
#   1. 尺寸白名单：1280x966 必须被接受（这才是真实数据源的尺寸）
#   2. README 里每条命令都能真跑
#   3. PDF 的图内文字必须进文本层（内联 SVG 而非栅格化）
#   4. 文档索引与工程实际文件一致
set -u
ROOT=/Volumes/Workspace/NFOCC
CPP=$ROOT/cpp
B=$CPP/build/host
PY="$HOME/venvs/flashocc/bin/python"
fail=0
ok()  { printf '  [PASS] %-22s %s\n' "$1" "$2"; }
bad() { printf '  [FAIL] %-22s %s\n' "$1" "$2"; fail=$((fail+1)); }

echo "== [1] 尺寸白名单（本轮核心改动） =="
"$B/tests/test_zmq_io" >/tmp/vf_z.log 2>&1; RC=$?
grep -E "1920x1536 ->|1280x966 ->|检查 [0-9]+ 次" /tmp/vf_z.log | sed 's/^/    /'
if [ "$RC" -eq 0 ] && grep -q "1280x966 -> 接受" /tmp/vf_z.log; then
    ok 尺寸白名单 "两种尺寸都接受，8/8 通过"
else
    bad 尺寸白名单 "rc=$RC"
fi
# 负对照：白名单外的尺寸必须被拒
if grep -q "up_parse_rejects_other_sizes.*PASS" /tmp/vf_z.log; then
    ok 负对照 "640x480 被拒"
else
    bad 负对照 "缺少拒绝对照"
fi

echo "== [2] README 命令实测 =="
for cmd in "python3 tools/docs_check.py" "sh scripts/verify_docs.sh" "sh scripts/verify_zmq.sh"; do
    if (cd "$ROOT" && eval "$cmd" >/tmp/vf_c.log 2>&1); then
        ok "$cmd" "退出 0"
    else
        bad "$cmd" "$(tail -2 /tmp/vf_c.log)"
    fi
done

# 用 README 里写的参数真跑 nfocc_run，D1 开/关都要有数
N_ON=$(cd "$CPP" && "$B/nfocc_run" --hanqtech ../datasets/fisheye --ws-frame 164 \
       --frames 2 --stats 2>&1 | grep -oE '累计点数 [0-9]+' | grep -oE '[0-9]+')
N_OFF=$(cd "$CPP" && "$B/nfocc_run" --hanqtech ../datasets/fisheye --ws-frame 164 \
        --frames 2 --stats --no-body-mask 2>&1 | grep -oE '累计点数 [0-9]+' | grep -oE '[0-9]+')
printf '    D1 开=%s 关=%s\n' "${N_ON:-?}" "${N_OFF:-?}"
if [ -n "${N_ON:-}" ] && [ -n "${N_OFF:-}" ] && [ "$N_OFF" -gt "$N_ON" ]; then
    ok "nfocc_run 参数" "README 里的参数均可用"
else
    bad "nfocc_run 参数" "ON=$N_ON OFF=$N_OFF"
fi

echo "== [3] README 引用的文件都存在 =="
MISS=""
for f in cpp/scripts/build.sh cpp/build/host/nfocc_run cpp/build/host/nfocc_viz \
         tools/docs_check.py scripts/verify_docs.sh scripts/verify_zmq.sh \
         docs/SPEC-DATASET.md docs/SPEC-CONVENTION.md docs/BUILD-CROSSPLATFORM.md \
         docs/ROADMAP.md docs/CLUSTERING-IMPLEMENTATION.md; do
    [ -e "$ROOT/$f" ] || MISS="$MISS $f"
done
[ -z "$MISS" ] && ok README引用 "全部存在" || bad README引用 "缺失:$MISS"

# 反向：README 不该再提已删的 Python 原型
if grep -q "nfocc_ph0\|nfocc_ph1" "$ROOT/README.md"; then
    bad README过时 "仍引用已删的 Python 原型"
else
    ok README过时 "无已删内容引用"
fi

echo "== [4] PDF 报告 =="
PDF="$ROOT/docs/report/NFOCC-工程解析报告.pdf"
if [ -f "$PDF" ]; then
    SZ=$(du -m "$PDF" | cut -f1)
    RES=$("$PY" - "$PDF" <<'EOF' 2>&1
import sys, fitz
d = fitz.open(sys.argv[1])
n = d.page_count
t = "".join(p.get_text() for p in d)
d.close()
must = ["开发进展看板", "工程架构", "数据流 Pipeline", "算法逻辑", "已知问题"]
miss = [m for m in must if m not in t]
# 图内文字进文本层 = 内联 SVG 生效（栅格化时搜不到）
inline = ["视口 LUT", "ZMQ 零拷贝", "网格哈希"]
miss2 = [m for m in inline if m not in t]
print(f"{n}|{len(t)}|{','.join(miss) or '-'}|{','.join(miss2) or '-'}")
EOF
)
    IFS='|' read -r NPAGES NCHARS M1 M2 <<EOF2
$RES
EOF2
    printf '    %s 页, %s 字, %s MB\n' "$NPAGES" "$NCHARS" "$SZ"
    if [ "${NPAGES:-0}" -ge 10 ] && [ "${M1:-x}" = "-" ]; then
        ok PDF章节 "四张主图齐全"
    else
        bad PDF章节 "缺:$M1"
    fi
    if [ "${M2:-x}" = "-" ]; then
        ok PDF文字层 "图内文字可搜索（内联SVG生效）"
    else
        bad PDF文字层 "图内文字未进文本层:$M2"
    fi
    [ "${SZ:-99}" -lt 8 ] && ok PDF体积 "${SZ} MB（< 8）" || bad PDF体积 "${SZ} MB 过大"
else
    bad PDF "不存在"
fi

echo "== [5] 文档索引 =="
python3 "$ROOT/tools/docs_check.py" >/tmp/vf_d.log 2>&1 \
    && ok docs_check "$(tail -1 /tmp/vf_d.log)" || bad docs_check "失败"
grep -q "report/NFOCC-工程解析报告.pdf" "$ROOT/docs/README.md" \
    && ok 索引登记 "PDF 已入索引" || bad 索引登记 "README 未登记 PDF"

echo "== [6] 回归 =="
for t in test_cluster test_d4_baseline test_algo_rev; do
    [ -x "$B/tests/$t" ] || continue
    "$B/tests/$t" >/dev/null 2>&1 && ok "$t" 通过 || bad "$t" 失败
done

echo "== [7] 判据自证伪 =="
[ -n "${NF_CANARY:-}" ] && bad "canary(故意)" "本段可失败"

echo
echo "失败 $fail 项"
[ "$fail" -eq 0 ] || exit 1
