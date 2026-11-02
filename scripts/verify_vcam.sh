#!/bin/sh
# vcam 后端 + 工程规范的本轮定向验证（ad-hoc，非测试套件全绿）。
#
# **ctest 真实结果**：vcam=ON 13/15、vcam=OFF 13/15，
# 唯一失败是既有的 depth_accuracy（见 docs/PROBE-RESULTS.md）。
# 本脚本只验本轮改动，不吃这个既有失败。
#
# 判据（每条都能失败）:
#   1. vcam=OFF 干净构建 + test_vcam_bridge 全过（桩实现路径）
#   2. vcam=ON  干净构建 + test_vcam_bridge 全过（真后端路径）
#   3. 交叉校验偏差 < 0.5 px（两个后端几何一致）
#   4. vcam=ON 与 OFF 端到端点数**完全相同**（接入未改变结果）
#   5. .gitignore 生效：._ 文件/viz/build 被忽略，已跟踪文件无误伤
#   6. 判据自证伪：NF_CANARY=1 必须让本段失败
set -u
ROOT=/Volumes/Workspace/NFOCC
CPP=$ROOT/cpp
VCAM_DIR=/Volumes/Workspace/virtualcamera/cpp
TMPD=$(mktemp -d "${TMPDIR:-/tmp}/hermes-verify-vcam-XXXXXX")
trap 'rm -rf "$TMPD"' EXIT INT TERM
fail=0
ok()  { printf '  [PASS] %s: %s\n' "$1" "$2"; }
bad() { printf '  [FAIL] %s: %s\n' "$1" "$2"; fail=$((fail+1)); }
JOBS=8

echo "[1] vcam=OFF 构建 + 桥接测试"
if cmake -S "$CPP" -B "$TMPD/off" -DCMAKE_BUILD_TYPE=Release \
        -DNF_WITH_VCAM=OFF >"$TMPD/off.log" 2>&1 &&
   cmake --build "$TMPD/off" -j$JOBS >>"$TMPD/off.log" 2>&1; then
    ok 构建 "零错误"
    if OUT=$("$TMPD/off/tests/test_vcam_bridge" 2>&1); then
        ok 桥接测试 "$(printf '%s' "$OUT" | grep -oE '失败 [0-9]+ 次' | head -1)"
    else
        bad 桥接测试 "$(printf '%s' "$OUT" | grep -E 'FAIL' | head -2)"
    fi
else
    bad 构建 "$(grep -m2 'error:' "$TMPD/off.log")"
fi

echo "[2] vcam=ON 构建 + 桥接测试"
if cmake -S "$CPP" -B "$TMPD/on" -DCMAKE_BUILD_TYPE=Release \
        -DNF_WITH_VCAM=ON -DNF_VCAM_DIR="$VCAM_DIR" >"$TMPD/on.log" 2>&1 &&
   cmake --build "$TMPD/on" -j$JOBS >>"$TMPD/on.log" 2>&1; then
    grep -q "vcam 后端已启用" "$TMPD/on.log" \
        && ok 构建 "vcam 已启用且零错误" \
        || bad 构建 "未打印启用信息"
    OUT=$("$TMPD/on/tests/test_vcam_bridge" 2>&1)
    RC=$?
    if [ "$RC" -eq 0 ]; then
        ok 桥接测试 "$(printf '%s' "$OUT" | grep -oE '失败 [0-9]+ 次' | head -1)"
    else
        bad 桥接测试 "$(printf '%s' "$OUT" | grep -E 'FAIL' | head -2)"
    fi
else
    bad 构建 "$(grep -m2 'error:' "$TMPD/on.log")"
fi

echo "[3] 交叉校验偏差"
# 判据值是 0.5 px；脚本只确认测试真的跑了两个模型并给出数字
for model in radial_poly KB; do
    LINE=$(printf '%s' "${OUT:-}" | grep "$model:" | grep -oE '最大偏差 [0-9.]+ px')
    if [ -n "$LINE" ]; then
        DEV=$(printf '%s' "$LINE" | grep -oE '[0-9]+\.[0-9]+')
        if awk "BEGIN{exit !($DEV < 0.5)}"; then
            ok "$model" "$LINE (< 0.5 px)"
        else
            bad "$model" "$LINE 超门限"
        fi
    else
        bad "$model" "未输出偏差"
    fi
done

echo "[4] 端到端等价（vcam=ON 与 OFF 点数必须相同）"
ARGS="--hanqtech ../datasets/fisheye --ws-frame 164 --frames 3 --stats"
# nfocc_run 要在 cpp/ 下跑（相对路径 ../datasets）
N_OFF=$(cd "$CPP" && "$TMPD/off/nfocc_run" $ARGS 2>&1 | grep -oE '累计点数 [0-9]+' | grep -oE '[0-9]+')
N_ON=$(cd  "$CPP" && "$TMPD/on/nfocc_run"  $ARGS 2>&1 | grep -oE '累计点数 [0-9]+' | grep -oE '[0-9]+')
printf '     OFF=%s  ON=%s\n' "${N_OFF:-?}" "${N_ON:-?}"
if [ -n "${N_OFF:-}" ] && [ "$N_OFF" = "${N_ON:-}" ]; then
    ok 等价 "两边都是 $N_OFF 点"
else
    bad 等价 "ON=$N_ON != OFF=$N_OFF -- 接入改变了结果"
fi
# 相机不得被禁用（r_img 量纲错误的症状就是三路全禁用）
if (cd "$CPP" && "$TMPD/on/nfocc_run" $ARGS 2>&1 | grep -q "完全不可见"); then
    bad 相机可见性 "有相机被禁用（疑似 max_theta/r_valid 判据过严）"
else
    ok 相机可见性 "无相机被禁用"
fi

echo "[5] .gitignore 生效"
cd "$ROOT" || exit 1
for p in "._foo" "viz/x.ppm" "cpp/build/x.o" ".wip/d2/x.cpp"; do
    git check-ignore -q "$p" && ok "$p" "已忽略" || bad "$p" "未被忽略"
done
for p in "cpp/third_party/MANIFEST.txt" "docs/viewplan_bev_current.png"; do
    git check-ignore -q "$p" && bad "$p" "被误忽略" || ok "$p" "仍跟踪"
done
# 已跟踪文件不得被新规则误伤
N_HIT=$(git ls-files -i -c --exclude-standard 2>/dev/null | wc -l | tr -d ' ')
[ "$N_HIT" = "0" ] && ok 无误伤 "0 个已跟踪文件被忽略" \
                   || bad 无误伤 "$N_HIT 个已跟踪文件被误忽略"
# ._ 文件必须已清空
N=$(find . -name "._*" -not -path "./.git/*" 2>/dev/null | wc -l | tr -d ' ')
[ "$N" = "0" ] && ok "._ 文件" "已清空" || bad "._ 文件" "仍有 $N 个"

echo "[6] 判据自证伪"
[ -n "${NF_CANARY:-}" ] && bad "canary(故意)" "本段可失败"

echo
echo "失败 $fail 项"
[ "$fail" -eq 0 ] || exit 1
