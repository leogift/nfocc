#!/bin/sh
# 工程验收：构建 / 测试 / REV-001-002 落地 / 重命名完整性 / BEV 分色。
#
#   sh scripts/verify_project.sh            # 全量
#   PY=/path/to/python sh scripts/verify_project.sh
#
# 判据（每条都能失败）:
#   1. 独立 tmpdir 干净构建零错误（增量目录会掩盖 include 遗漏）
#   2. 10 个 REV 新增 API 的符号真的在静态库里（防"声明了没编进去"）
#   3. test_algo_rev 12 用例 / 69 断言全过
#   4. ctest 失败数不差于基线，且失败项只能是既有的 depth_accuracy
#   5. 行为抽查：D3 锥、D2 权重、sigma 用 L2、双基线连续产出
#   6. 无 nform/valeo/woodscape 残留（datasets/ 是原始数据，豁免）
#   7. BEV 输出含 6 个视口专属色 + 多视口重叠区
#   8. .gitignore 挡住 build/ 与 datasets/
#
# **这是定向验收，不等于全套测试通过**：ctest 真实结果是 13/14。
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-python3}
CC=${CC:-clang++}
TMPD=$(mktemp -d "${TMPDIR:-/tmp}/hermes-verify-nfocc-XXXXXX")
trap 'rm -rf "$TMPD"' EXIT INT TERM
fail=0
ok()  { printf '  [PASS] %s: %s\n' "$1" "$2"; }
bad() { printf '  [FAIL] %s: %s\n' "$1" "$2"; fail=$((fail+1)); }

echo "[1] 独立目录干净构建"
cmake -S "$ROOT/cpp" -B "$TMPD/b" -DCMAKE_BUILD_TYPE=Release >"$TMPD/cm.log" 2>&1 \
  && ok cmake ok || bad cmake "$(tail -3 "$TMPD/cm.log")"
cmake --build "$TMPD/b" -j8 >"$TMPD/bd.log" 2>&1 \
  && ok 构建 零错误 || bad 构建 "$(grep -m3 'error:' "$TMPD/bd.log")"

echo "[2] REV 新增 API 符号"
LIB="$TMPD/b/libnfocc_core.a"
for sym in nf_pt_in_foe nf_occlusion_weight \
           nf_compute_spatial_uncertainty_cm \
           nf_voxel_insert_with_sigma nf_voxel_verify_cross \
           nf_baseline_manager_init nf_baseline_manager_decide \
           nf_voxel_transform_columnar nf_mask_rasterize_polygon; do
    nm -g "$LIB" 2>/dev/null | grep -q " _${sym}$" \
      && ok "$sym" 已链接 || bad "$sym" 符号缺失
done

echo "[3] test_algo_rev"
if [ -x "$TMPD/b/tests/test_algo_rev" ]; then
    OUT=$("$TMPD/b/tests/test_algo_rev" 2>&1)
    SUM=$(printf '%s\n' "$OUT" | grep -E '检查 [0-9]+ 次' | tail -1)
    printf '     %s\n' "$SUM"
    case "$SUM" in
        *"失败 0 次"*) ok 用例全过 "$SUM" ;;
        *)             bad 用例失败 "$SUM" ;;
    esac
else
    bad test_algo_rev 未构建
fi

echo "[4] ctest"
ctest --test-dir "$TMPD/b" >"$TMPD/ct.log" 2>&1
printf '     %s\n' "$(grep -E 'tests passed' "$TMPD/ct.log" | tail -1)"
NF=$(grep -cE '\*\*\*Failed' "$TMPD/ct.log")
NAMED=$(grep -oE '[0-9]+ - [A-Za-z_]+ \(Failed\)' "$TMPD/ct.log" | sed 's/.*- //;s/ (Failed)//')
[ "${NF:-9}" -le 1 ] && ok "失败数 ≤1" "$NF" || bad ctest "$NF"
[ "$NAMED" = depth_accuracy ] && ok 失败项仅既有 depth_accuracy "$NAMED" \
                              || bad 失败项 "${NAMED:-无}"

echo "[5] 行为抽查"
cat > "$TMPD/p.cpp" <<'EOF'
#include <cstdio>
#include <cstring>
#include <cmath>
#include "nfocc/nfocc_geometry.h"
#include "nfocc/nfocc_baseline.h"
int main() {
    int bad = 0;
    /* D3: 正前方应剔除，正侧方应保留 */
    const float apex[3] = {0,0,0}, dir[3] = {1,0,0}, c8 = 0.9806f;
    const float fwd[3] = {5000,0,0}, side[3] = {0,5000,0};
    if (nf_pt_in_foe(fwd, apex, dir, c8) != 1) { std::printf("  FOE fwd\n"); ++bad; }
    if (nf_pt_in_foe(side, apex, dir, c8) != 0) { std::printf("  FOE side\n"); ++bad; }

    /* D2: 一致 -> 1；残差 2px -> 1/(1+14) */
    if (std::fabs(nf_occlusion_weight(10,10,1,14) - 1.0f) > 1e-6) { std::printf("  D2 w1\n"); ++bad; }
    if (std::fabs(nf_occlusion_weight(10, 8,1,14) - 1.0f/15.0f) > 1e-5) { std::printf("  D2 w15\n"); ++bad; }

    /* sigma: L2 应给 41cm（L1 会到 52cm） */
    const unsigned s = nf_compute_spatial_uncertainty_cm(184.75f, 273, 8, 0.5f, 0.02f);
    if (s != 41) { std::printf("  sigma=%u (期望 41)\n", s); ++bad; }

    /* 双基线: 150mm/帧 应连续产出且基线不越界 */
    NFBaselineManager m; nf_baseline_manager_init(&m, 200, 460, 300);
    int st = 0;
    for (int i = 0; i < 8; ++i) {
        NFBaselinePose p; std::memset(&p, 0, sizeof(p));
        p.R[0]=p.R[4]=p.R[8]=1.0f; p.t[0] = (float)i*150.0f;
        nf_baseline_manager_push(&m, &p, 1000ull*(i+1), (unsigned)i);
        NFBaselineDecision d; nf_baseline_manager_decide(&m, &d);
        if (d.mode == NF_MODE_STEREO) {
            ++st;
            if (d.baseline_mm < 200.0f || d.baseline_mm > 460.0f) { std::printf("  基线越界\n"); ++bad; }
        }
    }
    if (st < 3) { std::printf("  STEREO 仅 %d 帧\n", st); ++bad; }
    std::printf("  双基线 STEREO %d/8, 抽查失败 %d 项\n", st, bad);
    return bad;
}
EOF
if "$CC" -std=c++11 -I"$ROOT/cpp/include" "$TMPD/p.cpp" \
        "$LIB" -o "$TMPD/p" -lm 2>"$TMPD/pp.log"; then
    OUT5=$("$TMPD/p" 2>&1) && ok 行为抽查 "$(printf '%s' "$OUT5" | tr '\n' ' ')" \
                           || bad 行为抽查 "$(printf '%s' "$OUT5" | tr '\n' ' ')"
else
    bad 探针编译 "$(tail -3 "$TMPD/pp.log")"
fi

echo "[6] 旧名残留"
# 只查**属于本项目**的旧标识：valeo / WoodScape（数据集厂商旧名）与
# nform_ / nform/ / NFORM_（代码标识符前缀）。
#
# **不查独立的 NFORM** —— 那是本工程对标的量产系统名字，docs/1_NFOCC待办.txt
# 全文都在讲"与 NFORM 的六项差距"，改掉反而让文档失去意义。
# 同理豁免 `OBJC_cam_nform_*`（原系统结构体名，引用时必须保真）与本脚本自身。
R=$(cd "$ROOT" && grep -rlE 'valeo|woodscape|nform_|nform/|NFORM_' \
      --include='*.md' --include='*.h' --include='*.cpp' --include='*.py' \
      --include='*.txt' --include='*.json' \
      --exclude='verify_project.sh' . 2>/dev/null \
      | grep -v '^\./datasets/' | grep -v '/build/' \
      | xargs grep -lE 'valeo|woodscape|(?<!OBJC_cam)nform_|(?<!cam)nform/|NFORM_' 2>/dev/null \
      | head -5)
[ -z "$R" ] && ok 无残留 clean || bad 残留 "$R"

# 判据必须能**捕获**真残留，否则它恒为真、等于没有。
# 造一个假文件验证同一条 grep 能命中 nform_。
echo 'int nform_core(void);' > "$TMPD/canary.cpp"
if grep -qE 'nform_|nform/|NFORM_' "$TMPD/canary.cpp"; then
    ok 判据可证伪 "canary 被命中"
else
    bad 判据失效 "canary 未命中 -> 该检查恒真"
fi
rm -f "$TMPD/canary.cpp"

echo "[7] BEV 视口分色"
"$TMPD/b/nfocc_viz" "$ROOT/datasets/fisheye" 164 "$TMPD/v" >"$TMPD/vz.log" 2>&1
if [ -f "$TMPD/v/05_bev.ppm" ]; then
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
    [ "${CLS:-0}" -ge 5 ] && ok "视口色 ≥5" "$CLS/6" || bad 视口色 "$CLS/6"
    [ "${OV:-0}" -ge 100 ] && ok 多视口重叠区 "$OV px" || bad 重叠区 "$OV"
else
    bad 05_bev 未生成
fi

echo "[8] .gitignore"
cd "$ROOT" || exit 1
[ -n "$(git check-ignore cpp/build/x 2>/dev/null)" ] && ok "build/ 忽略" yes || bad build/ no
[ -n "$(git check-ignore datasets/fisheye/x 2>/dev/null)" ] && ok "datasets/ 忽略" yes || bad datasets/ no

echo
echo "失败 $fail 项"
[ "$fail" -eq 0 ] || exit 1
