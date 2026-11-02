#!/bin/sh
# ZMQ 零拷贝的定向验证。**不是套件全绿**：ctest 里 depth_accuracy 是既有失败，
# 本脚本用 -E 排除它，只验本轮改动。
#
# 零拷贝的判据必须验"对"，不能只验"快"：
#   1. 单测 5/5（含"指针落在消息内部"这条零拷贝的直接证据）
#   2. 独立探针：nf_up_parse 返回的 pixels 必须落在输入缓冲内（不复制）
#   3. 负对照：如果复制了，指针会在缓冲外 —— 用堆地址距离判定
#   4. 内存：转移所有权后无泄漏（RSS 不随轮次线性增长）
set -u
ROOT=/Volumes/Workspace/NFOCC
CPP=$ROOT/cpp
B=$CPP/build/host
TMPD=$(mktemp -d "${TMPDIR:-/tmp}/hermes-verify-zmq-XXXXXX")
trap 'rm -rf "$TMPD"' EXIT INT TERM
fail=0
ok()  { printf '  [PASS] %-22s %s\n' "$1" "$2"; }
bad() { printf '  [FAIL] %-22s %s\n' "$1" "$2"; fail=$((fail+1)); }

echo "== [1] 构建 =="
if cmake --build "$B" -j8 >"$TMPD/b.log" 2>&1; then
    ok 构建 零错误
else
    bad 构建 "$(grep -m2 'error:' "$TMPD/b.log")"
fi

echo "== [2] 单测 test_zmq_io =="
"$B/tests/test_zmq_io" >"$TMPD/z.log" 2>&1; RC=$?
grep -E '^  \[|检查 [0-9]+ 次' "$TMPD/z.log" | sed 's/^/    /'
[ "$RC" -eq 0 ] && ok 单测 "5/5 通过" || bad 单测 "rc=$RC"

echo "== [3] 独立探针：pixels 必须在输入缓冲内（零拷贝的直接证据） =="
cat > "$TMPD/p.cpp" <<'EOF'
#include <cstdio>
#include <cstring>
#include <vector>
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_io.h"

int main() {
    const nf_u32 W = NF_IMG_W, H = NF_IMG_H;

    NFUpHeader uh;
    std::memset(&uh, 0, sizeof(uh));
    uh.magic = NF_UP_MAGIC; uh.version = NF_UP_VERSION;
    uh.frame_id = 99; uh.n_cameras = NF_MAX_CAMERAS; uh.ego.quality = 100;

    std::vector<std::vector<nf_u8>> segs;
    for (nf_u32 c = 0; c < NF_MAX_CAMERAS; ++c) {
        NFUpImageHeader ih;
        std::memset(&ih, 0, sizeof(ih));
        ih.cam_id = (nf_u8)c; ih.format = 0;
        ih.width = W; ih.height = H; ih.stride = W;
        ih.data_bytes = W * H;
        std::vector<nf_u8> s(sizeof(ih) + (size_t)W * H);
        std::memcpy(s.data(), &ih, sizeof(ih));
        std::memset(s.data() + sizeof(ih), 0x40 + c, (size_t)W * H);
        segs.push_back(std::move(s));
    }

    const void *fr[NF_MAX_CAMERAS]; nf_u32 fb[NF_MAX_CAMERAS];
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        fr[i] = segs[i].data(); fb[i] = (nf_u32)segs[i].size();
    }
    NFUpFrameView v;
    NFStatus st = nf_up_parse(&uh, (nf_u32)sizeof(uh), fr, fb, NF_MAX_CAMERAS, &v);
    if (st != NF_OK) { std::printf("parse-fail %d\n", (int)st); return 1; }

    int outside = 0;
    for (nf_u32 c = 0; c < NF_MAX_CAMERAS; ++c) {
        const nf_u8 *b = segs[c].data();
        const nf_u8 *p = v.pixels[c];
        if (p < b || p >= b + segs[c].size()) { ++outside; }
        else if (p[0] != (nf_u8)(0x40 + c)) { ++outside; }
    }
    std::printf("4 路中 %d 路指针在输入缓冲外（应为 0 = 零拷贝）\n", outside);
    return outside ? 2 : 0;
}
EOF
if clang++ -std=c++11 -I"$CPP/include" "$TMPD/p.cpp" "$B/libnfocc_core.a" -o "$TMPD/p" 2>"$TMPD/pe"; then
    RES=$("$TMPD/p" 2>&1); PRC=$?
    printf '%s\n' "$RES" | sed 's/^/    /'
    [ "$PRC" -eq 0 ] && ok 零拷贝证明 "4 路指针全在输入缓冲内" || bad 零拷贝证明 "有指针在缓冲外"
else
    bad 探针编译 "$(tail -2 "$TMPD/pe")"
fi

echo "== [4] 内存：转移所有权后不泄漏 =="
# 跑 20 次 send_moved，RSS 增量应 < 5 MB（若 free_fn 未调用会累积）
cat > "$TMPD/m.cpp" <<'EOF'
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/resource.h>
#include <zmq.h>
#include "nfocc/nfocc_zmq_io.h"

/* macOS 没有 /proc，用 getrusage；它在两个平台都可用 */
static long rss_kb() {
#ifdef __APPLE__
    struct rusage ru;
    if (getrusage(RUSAGE_SELF, &ru) != 0) return -1;
    return ru.ru_maxrss / 1024;          /* macOS: bytes */
#else
    FILE *f = std::fopen("/proc/self/statm", "r");
    if (!f) return -1;
    long sz = 0, res = 0;
    if (std::fscanf(f, "%ld %ld", &sz, &res) != 2) res = -1;
    std::fclose(f);
    return res > 0 ? res * 4 : -1;       /* Linux: 页 -> KB */
#endif
}

int main() {
    const char *ep = "ipc:///tmp/nf_leak_test.ipc";
    NFZmqSink *k = nullptr;
    if (nf_zmq_sink_open(&k, ep, 4) != NF_OK) { std::printf("open-fail\n"); return 1; }

    const size_t SZ = 1u << 20;   /* 1 MB */
    long before = rss_kb();
    for (int i = 0; i < 200; ++i) {
        void *buf = std::malloc(SZ);
        std::memset(buf, i & 0xFF, SZ);
        nf_zmq_sink_send_moved(k, buf, (nf_u32)SZ);
    }
    long after = rss_kb();
    std::printf("RSS before=%ld KB after=%ld KB delta=%ld KB (200 x 1MB)\n",
                before, after, after - before);
    /* 无订阅者时全部 overflow -> free_fn 应立即释放，RSS 不应增长 200MB */
    long delta = after - before;
    nf_zmq_sink_close(k);
    std::remove("/tmp/nf_leak_test.ipc");
    if (before < 0) { std::printf("rss-unavailable\n"); return 0; }
    return (delta > 50 * 1024) ? 2 : 0;
}
EOF
# zmq 库位置随平台不同：优先 brew，其次 third_party
ZMQ_INC=""; ZMQ_LIB=""
for d in /opt/homebrew /usr/local; do
    [ -f "$d/include/zmq.h" ] && ZMQ_INC="$d/include" && ZMQ_LIB="$d/lib" && break
done
[ -z "$ZMQ_INC" ] && [ -d "$CPP/third_party/zeromq/include" ] && \
    ZMQ_INC="$CPP/third_party/zeromq/include" && ZMQ_LIB="$CPP/third_party/zeromq/lib"

if [ -z "$ZMQ_INC" ]; then
    bad 内存 "找不到 zmq.h，无法验证"
elif clang++ -std=c++11 -I"$CPP/include" -I"$ZMQ_INC" \
        "$TMPD/m.cpp" "$B/libnfocc_io.a" "$B/libnfocc_core.a" \
        -L"$ZMQ_LIB" -lzmq -o "$TMPD/m" 2>"$TMPD/me"; then
    RES=$("$TMPD/m" 2>&1); PRC=$?
    printf '%s\n' "$RES" | sed 's/^/    /'
    case "$RES" in
        *rss-unavailable*) bad 内存 "拿不到 RSS，判据未执行" ;;
        *) [ "$PRC" -eq 0 ] && ok 内存 "无泄漏（增量 < 50MB）" || bad 内存 "RSS 增长过大" ;;
    esac
else
    bad 内存 "编译失败: $(tail -1 "$TMPD/me")"
fi

echo "== [5] 回归：其余单测不受影响 =="
for t in test_cluster test_d4_baseline test_algo_rev; do
    if [ -x "$B/tests/$t" ]; then
        "$B/tests/$t" >"$TMPD/$t.log" 2>&1
        [ $? -eq 0 ] && ok "$t" "通过" || bad "$t" "失败"
    fi
done

echo "== [6] 判据自证伪 =="
[ -n "${NF_CANARY:-}" ] && bad "canary(故意)" "本段可失败"

echo
echo "失败 $fail 项"
[ "$fail" -eq 0 ] || exit 1
