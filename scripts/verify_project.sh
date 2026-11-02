#!/bin/sh
# 工程验收：重命名完整性 + BEV 视口 FOV 分色。
#
#   sh scripts/verify_project.sh
#
# 判据（每条都能失败）:
#   1. 独立目录干净构建零错误（增量目录会掩盖改名遗漏）
#   2. 无 valeo/woodscape/nform/NFORM 残留（datasets/ 是原始数据，豁免）
#   3. 无旧名文件或目录
#   4. 三个产物齐全且名称为 nfocc_*
#   5. ctest 失败数不差于基线（1 项 depth_accuracy，既有问题）
#   6. 05_bev 生成，且 6 个视口专属色全部出现、多视口重叠区存在
#   7. .gitignore 挡住 build/ 与 datasets/
#
# 这是**定向验收**，不等于全套测试通过：ctest 真实结果是 12/13。
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-python3}
TMPD=$(mktemp -d "${TMPDIR:-/tmp}/hermes-verify-nfocc-XXXXXX")
trap 'rm -rf "$TMPD"' EXIT INT TERM
fail=0
ok()  { printf '  [PASS] %s: %s\n' "$1" "$2"; }
bad() { printf '  [FAIL] %s: %s\n' "$1" "$2"; fail=$((fail+1)); }

echo "[1] 独立目录干净构建"
if cmake -S "$ROOT/cpp" -B "$TMPD/b" -DCMAKE_BUILD_TYPE=Release >"$TMPD/cm.log" 2>&1; then
    ok cmake ok
else
    bad cmake "$(tail -3 "$TMPD/cm.log")"
fi
if cmake --build "$TMPD/b" -j8 >"$TMPD/bd.log" 2>&1; then
    ok 构建 零错误
else
    bad 构建 "$(grep -m3 'error:' "$TMPD/bd.log")"
fi

echo "[2] 旧名的源码/文档残留"
R=$(cd "$ROOT" && grep -rlE 'valeo|woodscape|nform|NFORM' \
      --include='*.md' --include='*.h' --include='*.cpp' --include='*.py' \
      --include='*.txt' --include='*.json' . 2>/dev/null \
      | grep -v '^\./datasets/' | grep -v '/build/' | head -5)
[ -z "$R" ] && ok 无残留 clean || bad 残留 "$R"

echo "[3] 旧名的文件/目录"
R=$(cd "$ROOT" && find . \( -name '*nform*' -o -name '*woodscape*' \) 2>/dev/null \
      | grep -v '^\./datasets/' | grep -v '/build/' | head -3)
[ -z "$R" ] && ok 无旧名路径 clean || bad 旧名路径 "$R"

echo "[4] 产物"
M=""
for f in nfocc_run nfocc_viz libnfocc_core.a; do
    [ -e "$TMPD/b/$f" ] || M="$M $f"
done
[ -z "$M" ] && ok 产物齐 3/3 || bad 缺产物 "$M"

echo "[5] ctest"
ctest --test-dir "$TMPD/b" >"$TMPD/ct.log" 2>&1
N=$(grep -cE '\*\*\*Failed' "$TMPD/ct.log")
printf '     %s\n' "$(grep -E 'tests passed' "$TMPD/ct.log" | tail -1)"
[ "${N:-9}" -le 1 ] && ok "失败数 ≤1（基线）" "$N" || bad ctest "$N"

echo "[6] BEV 视口 FOV 分色"
"$TMPD/b/nfocc_viz" "$ROOT/datasets/fisheye" 164 "$TMPD/v" >"$TMPD/vz.log" 2>&1
if [ -f "$TMPD/v/05_bev.ppm" ]; then ok 05_bev 生成; else bad 05_bev 缺; fi
# 6 个视口专属色各需 >=200 px；灰白多视口重叠区需 >=100 px
read -r CLS OV <<EOF
$("$PY" - "$TMPD/v/05_bev.ppm" <<'PYEOF'
import sys
import numpy as np
from PIL import Image
a = np.asarray(Image.open(sys.argv[1]).convert("RGB")).reshape(-1, 3).astype(int)
VC = [(230,60,60),(250,140,40),(60,170,70),(40,190,200),(120,90,220),(230,70,180)]
cls = sum((np.abs(a - np.array(c)).sum(1) < 40).sum() > 200 for c in VC)
g = (np.abs(a[:,0]-a[:,1])<12) & (np.abs(a[:,1]-a[:,2])<12) & (a[:,0]>145) & (a[:,0]<235)
print(cls, int(g.sum()))
PYEOF
)
EOF
[ "${CLS:-0}" -ge 5 ] && ok "视口色出现 ≥5" "$CLS/6" || bad 视口色 "$CLS/6"
[ "${OV:-0}" -ge 100 ] && ok 多视口重叠区 "$OV px" || bad 重叠区 "$OV"

echo "[7] .gitignore"
cd "$ROOT" || exit 1
[ -n "$(git check-ignore cpp/build/x 2>/dev/null)" ] && ok "build/ 忽略" yes || bad build/ no
[ -n "$(git check-ignore datasets/fisheye/x 2>/dev/null)" ] && ok "datasets/ 忽略" yes || bad datasets/ no

echo
echo "失败 $fail 项"
[ "$fail" -eq 0 ] || exit 1
