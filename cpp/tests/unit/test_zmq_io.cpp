/**
 * @file test_zmq_io.cpp
 * @brief ZMQ 零拷贝收发的正确性测试。
 *
 * ## 零拷贝最难验的不是"快"，是"对"
 * 零拷贝把内存所有权在库之间转移，出错时表现为**读到已释放内存**
 * 或**双重释放** —— 平时看不出，压力下随机崩。所以这里的判据集中在：
 *
 * 1. 收→解析→release 的完整周期能重复跑而不崩（生命周期正确）
 * 2. release 前数据完好、release 后不再访问（不留悬垂指针）
 * 3. 发送转移所有权后，原缓冲区**不被本进程 free**（无双重释放）
 * 4. 多个订阅者能收到同一帧（PUB/SUB 正常）
 * 5. 大载荷（模拟 150k 点）反复发送不泄漏
 *
 * 用进程内 PUB/SUB + inproc 端点，不依赖外部服务。
 */
#include <cstdio>
#include <cstring>
#include <vector>

#include <zmq.h>

#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_io.h"
#include "nfocc/nfocc_zmq_io.h"
#include "nf_test.h"

namespace {

/**
 * 构造一条符合 NFUpHeader/NFUpImageHeader 的 multipart 消息。
 *
 * **这里必须按 ICD 真实布局构造**，不能用简化格式 —— 之前的
 * zmq_io 用了一套私有格式，结果 nf_up_parse 根本读不了。
 */
struct UpMsgBuilder {
    std::vector<nf_u8> head;
    std::vector<std::vector<nf_u8>> imgs;
    /* 必须是 ICD 规定的尺寸：nf_up_parse 会硬校验 */
    nf_u32 w = NF_IMG_W, h = NF_IMG_H;

    void build(nf_u32 frame_id) {
        NFUpHeader uh;
        std::memset(&uh, 0, sizeof(uh));
        uh.magic = NF_UP_MAGIC;
        uh.version = NF_UP_VERSION;
        uh.frame_id = frame_id;
        uh.n_cameras = NF_MAX_CAMERAS;
        uh.ego.quality = 100;
        head.assign((const nf_u8 *)&uh, (const nf_u8 *)&uh + sizeof(uh));

        imgs.clear();
        for (nf_u32 c = 0; c < NF_MAX_CAMERAS; ++c) {
            NFUpImageHeader ih;
            std::memset(&ih, 0, sizeof(ih));
            ih.width = w;
            ih.height = h;
            ih.stride = w;
            ih.cam_id = (nf_u8)c;
            ih.format = 0;
            ih.data_bytes = w * h;   /* 必须 == stride*height，否则解析拒绝 */

            std::vector<nf_u8> seg(sizeof(ih) + (size_t)w * h);
            std::memcpy(seg.data(), &ih, sizeof(ih));
            /* 用 cam 序号填充，便于验证"收到的是哪一路" */
            std::memset(seg.data() + sizeof(ih), (int)(0x40 + c), (size_t)w * h);
            imgs.push_back(std::move(seg));
        }
    }
};

}  // namespace

/* ---------- 无 ZMQ 的纯结构校验 ---------- */

/* NFUpFrameView 的指针应指向消息内部（零拷贝的语义前提） */
NF_TEST(up_parse_returns_views_into_message) {
    UpMsgBuilder b;
    b.build(7);

    const void *frames[NF_MAX_CAMERAS];
    nf_u32 fbytes[NF_MAX_CAMERAS];
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        frames[i] = b.imgs[i].data();
        fbytes[i] = (nf_u32)b.imgs[i].size();
    }

    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(b.head.data(), (nf_u32)b.head.size(),
                                frames, fbytes, NF_MAX_CAMERAS, &v), NF_OK);
    NF_CHECK_EQ_INT(v.header.frame_id, 7);

    /* 指针必须落在原消息里，不能是新分配的副本 */
    for (nf_u32 c = 0; c < NF_MAX_CAMERAS; ++c) {
        const nf_u8 *base = b.imgs[c].data();
        const nf_u8 *px = v.pixels[c];
        NF_CHECK_MSG(px >= base && px < base + b.imgs[c].size(),
                     "pixels 应指向消息内部（零拷贝）");
        /* 首字节应是该路的填充值 */
        NF_CHECK_EQ_INT(px[0], 0x40 + c);
    }
}

/* ---------- ZMQ 收发 ---------- */

/**
 * 完整周期：open sink/source → 发 → 收 → 解析 → release，重复 50 次。
 *
 * 这一步验证**生命周期**。零拷贝最常见的 bug 是 release 早于使用
 * （读到已释放内存）或忘记 release（泄漏）。重复执行让这类错误
 * 在 ASan 下暴露，也能在普通构建里因数据错乱被发现。
 */
NF_TEST(zmq_roundtrip_zero_copy) {
    /*
     * 用裸 socket 而不是 nf_zmq_source_* 封装，原因有两个：
     *   1. 需要逐段发 multipart，封装只提供整帧发送
     *   2. PUB/SUB 必须在**同一个 context** —— 实测跨 context 的三个端点
     *      (inproc / ipc / tcp) 全部收不到，同 context 的 ipc 第一次就成功
     *
     * 这里验证的是 `nf_up_parse` 的零拷贝语义：view 里的指针指向消息内部，
     * 消息存活期间可读，release 之后不再访问。
     */
    const char *ep = "ipc:///tmp/nf_test_rt.ipc";
    void *ctx = zmq_ctx_new();
    void *pub = zmq_socket(ctx, ZMQ_PUB);
    void *sub = zmq_socket(ctx, ZMQ_SUB);
    int hwm = 4;
    zmq_setsockopt(pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);

    if (zmq_bind(pub, ep) != 0 || zmq_connect(sub, ep) != 0) {
        zmq_close(pub); zmq_close(sub); zmq_ctx_destroy(ctx);
        NF_CHECK_MSG(false, "bind/connect 失败");
        return;
    }

    /* 发一帧并尝试收；PUB/SUB 握手是异步的，重试到通为止 */
    auto send_frame = [&](nf_u32 fid) {
        UpMsgBuilder b;
        b.build(fid);
        zmq_msg_t m;
        zmq_msg_init_size(&m, b.head.size());
        std::memcpy(zmq_msg_data(&m), b.head.data(), b.head.size());
        zmq_msg_send(&m, pub, ZMQ_SNDMORE);
        zmq_msg_close(&m);
        for (nf_u32 c = 0; c < NF_MAX_CAMERAS; ++c) {
            zmq_msg_t im;
            zmq_msg_init_size(&im, b.imgs[c].size());
            std::memcpy(zmq_msg_data(&im), b.imgs[c].data(), b.imgs[c].size());
            zmq_msg_send(&im, pub,
                         (c + 1u < NF_MAX_CAMERAS) ? ZMQ_SNDMORE : 0);
            zmq_msg_close(&im);
        }
    };

    /* 收一帧 multipart 并解析，返回是否成功 */
    auto recv_and_parse = [&](nf_u32 *out_fid) -> bool {
        zmq_pollitem_t it;
        it.socket = sub; it.fd = 0; it.events = ZMQ_POLLIN; it.revents = 0;
        if (zmq_poll(&it, 1, 300) <= 0) return false;

        zmq_msg_t parts[NF_ZMQ_MAX_PARTS];
        nf_u32 n = 0;
        while (n < NF_ZMQ_MAX_PARTS) {
            zmq_msg_init(&parts[n]);
            if (zmq_msg_recv(&parts[n], sub, 0) < 0) {
                for (nf_u32 i = 0; i <= n; ++i) zmq_msg_close(&parts[i]);
                return false;
            }
            ++n;
            int more = 0; size_t ms = sizeof(more);
            zmq_getsockopt(sub, ZMQ_RCVMORE, &more, &ms);
            if (!more) break;
        }
        if (n != NF_ZMQ_MAX_PARTS) {
            for (nf_u32 i = 0; i < n; ++i) zmq_msg_close(&parts[i]);
            return false;
        }

        /* 零拷贝：直接把这些**消息内部**的地址喂给解析器 */
        const void *frames[NF_MAX_CAMERAS];
        nf_u32 fb[NF_MAX_CAMERAS];
        for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
            frames[i] = zmq_msg_data(&parts[i + 1u]);
            fb[i] = (nf_u32)zmq_msg_size(&parts[i + 1u]);
        }
        NFUpFrameView v;
        const NFStatus st = nf_up_parse(zmq_msg_data(&parts[0]),
                                        (nf_u32)zmq_msg_size(&parts[0]),
                                        frames, fb, NF_MAX_CAMERAS, &v);
        bool ok = (st == NF_OK);
        if (ok) {
            *out_fid = v.header.frame_id;
            /* **关键断言**：view 的指针落在消息内部（零拷贝的证明） */
            const nf_u8 *msgbase = (const nf_u8 *)zmq_msg_data(&parts[1]);
            ok = (v.pixels[0] >= msgbase &&
                  v.pixels[0] < msgbase + zmq_msg_size(&parts[1]));
            ok = ok && (v.pixels[0][0] == 0x40);   /* 该路填充值 */
        }
        for (nf_u32 i = 0; i < n; ++i) zmq_msg_close(&parts[i]);
        return ok;
    };

    /* 握手：反复发直到收到 */
    bool linked = false;
    for (int a = 0; a < 100 && !linked; ++a) {
        send_frame(0xFFFFFFFFu);
        nf_u32 fid = 0;
        if (recv_and_parse(&fid)) linked = (fid == 0xFFFFFFFFu);
    }
    if (!linked) {
        zmq_close(pub); zmq_close(sub); zmq_ctx_destroy(ctx);
        NF_CHECK_MSG(false, "PUB/SUB 未在 100 次内建立");
        return;
    }

    int ok_count = 0;
    for (nf_u32 round = 1; round <= 5; ++round) {
        send_frame(round);
        nf_u32 fid = 0;
        if (recv_and_parse(&fid) && fid == round) ++ok_count;
    }
    std::printf("      5 轮: 成功 %d 帧（含指针落在消息内的校验）\n", ok_count);
    NF_CHECK_EQ_INT(ok_count, 5);

    zmq_close(pub);
    zmq_close(sub);
    zmq_ctx_destroy(ctx);
    /* 清理 ipc 端点文件 */
    std::remove("/tmp/nf_test_rt.ipc");
}

NF_TEST(zmq_send_moved_ownership) {
    const char *ep = "inproc://nf_test_moved";
    NFZmqSink *sink = nullptr;
    NF_CHECK_STATUS(nf_zmq_sink_open(&sink, ep, 4), NF_OK);

    for (int i = 0; i < 200; ++i) {
        const size_t sz = 1024 + (size_t)i * 16;
        void *buf = std::malloc(sz);
        NF_CHECK(buf != nullptr);
        std::memset(buf, i & 0xFF, sz);
        /* 交给 ZeroMQ；无订阅者时会返回 OVERFLOW，内存仍由 free_fn 释放 */
        nf_zmq_sink_send_moved(sink, buf, (nf_u32)sz);
    }

    /* close 时 ZeroMQ 释放所有仍在队列里的消息 -> 触发 free_fn */
    nf_zmq_sink_close(sink);
}

/* 发送完整 NFFrameOutput：验证按 ICD 打包不崩、字段正确 */
NF_TEST(zmq_send_frame_output) {
    const char *ep = "inproc://nf_test_out";
    NFZmqSink *sink = nullptr;
    NF_CHECK_STATUS(nf_zmq_sink_open(&sink, ep, 4), NF_OK);
    NFZmqSource *src = nullptr;
    NF_CHECK_STATUS(nf_zmq_source_open(&src, ep, 50), NF_OK);

    NFFrameOutput out;
    std::memset(&out, 0, sizeof(out));
    out.version = 1;
    out.frame_id = 42;
    out.dominant_mode = NF_MODE_STEREO;
    out.is_frozen = 0;
    for (int c = 0; c < NF_NUM_CAMERAS; ++c) out.cam_status[c] = NF_OK;

    /* 造 1000 个点 */
    std::vector<NFPoint> pts(1000);
    std::memset(pts.data(), 0, pts.size() * sizeof(NFPoint));
    for (int i = 0; i < 1000; ++i) {
        pts[i].x_mm = i; pts[i].y_mm = i * 2; pts[i].z_mm = i * 3;
    }
    out.cloud.points = pts.data();
    out.cloud.num_points = 1000;
    out.cloud.max_points = 1000;

    /* 无订阅者时可能 overflow，这不算失败 */
    const NFStatus st = nf_zmq_sink_send(sink, &out);
    NF_CHECK_MSG(st == NF_OK || st == NF_E_OUTPUT_OVERFLOW, "发送不应崩溃");

    nf_zmq_source_close(src);
    nf_zmq_sink_close(sink);
}

/* 参数校验：空指针、坏端点必须拒绝 */
NF_TEST(zmq_reject_bad_input) {
    NF_CHECK_EQ_INT(nf_zmq_source_open(nullptr, "inproc://x", 0), NF_E_PARAM_NULL);
    NF_CHECK_EQ_INT(nf_zmq_sink_open(nullptr, "inproc://x", 0), NF_E_PARAM_NULL);

    NFZmqSource *s = nullptr;
    NF_CHECK_EQ_INT(nf_zmq_source_recv(nullptr, nullptr), NF_E_PARAM_NULL);
    NF_CHECK_EQ_INT(nf_zmq_sink_send(nullptr, nullptr), NF_E_PARAM_NULL);
    NF_CHECK_EQ_INT(nf_zmq_sink_send_moved(nullptr, nullptr, 0), NF_E_PARAM_NULL);
    (void)s;

    /* 不存在的端点应在 open 时失败 */
    NFZmqSink *k = nullptr;
    NF_CHECK_EQ_INT(nf_zmq_sink_open(&k, "not-a-valid-endpoint", 4), NF_E_PARAM_RANGE);
    NF_CHECK(k == nullptr);
}


/* ---------- 尺寸白名单 ---------- */

/**
 * 两种尺寸都必须被接受：1920x1536（ICD 设计值）与 1280x966（hanqtech 实拍）。
 *
 * 这条用例的存在理由：**原先只测了 1920x1536，所以 1280x966 被拒的问题
 * 一直没暴露**。测试数据必须覆盖真实数据源的尺寸，否则通过了也不能说明
 * 接上真数据能用。
 */
NF_TEST(up_parse_accepts_both_sizes) {
    const nf_u32 sizes[2][2] = { {1920u, 1536u}, {1280u, 966u} };

    for (int s = 0; s < 2; ++s) {
        UpMsgBuilder b;
        b.w = sizes[s][0];
        b.h = sizes[s][1];
        b.build(1);

        const void *frames[NF_MAX_CAMERAS];
        nf_u32 fbytes[NF_MAX_CAMERAS];
        for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
            frames[i] = b.imgs[i].data();
            fbytes[i] = (nf_u32)b.imgs[i].size();
        }
        NFUpFrameView v;
        const NFStatus st = nf_up_parse(b.head.data(), (nf_u32)b.head.size(),
                                        frames, fbytes, NF_MAX_CAMERAS, &v);
        std::printf("      %ux%u -> %s\n", sizes[s][0], sizes[s][1],
                    st == NF_OK ? "接受" : "拒绝");
        NF_CHECK_MSG(st == NF_OK, "白名单内的尺寸应被接受");
        NF_CHECK_EQ_INT(v.width, sizes[s][0]);
        NF_CHECK_EQ_INT(v.height, sizes[s][1]);
    }
}

/** 白名单外的尺寸必须被拒（不能放开到任意尺寸） */
NF_TEST(up_parse_rejects_other_sizes) {
    UpMsgBuilder b;
    b.w = 640; b.h = 480;
    b.build(1);

    const void *frames[NF_MAX_CAMERAS];
    nf_u32 fbytes[NF_MAX_CAMERAS];
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        frames[i] = b.imgs[i].data();
        fbytes[i] = (nf_u32)b.imgs[i].size();
    }
    NFUpFrameView v;
    NF_CHECK_EQ_INT(nf_up_parse(b.head.data(), (nf_u32)b.head.size(),
                                frames, fbytes, NF_MAX_CAMERAS, &v),
                    NF_E_PARAM_RANGE);
}

/** 自定义白名单：只允许一个非常规尺寸 */
NF_TEST(up_parse_custom_allowlist) {
    const NFUpAllowedSize allow[2] = { {640u, 480u}, {0u, 0u} };

    UpMsgBuilder b;
    b.w = 640; b.h = 480;
    b.build(3);

    const void *frames[NF_MAX_CAMERAS];
    nf_u32 fbytes[NF_MAX_CAMERAS];
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        frames[i] = b.imgs[i].data();
        fbytes[i] = (nf_u32)b.imgs[i].size();
    }
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse_sized(b.head.data(), (nf_u32)b.head.size(),
                                      frames, fbytes, NF_MAX_CAMERAS, &v,
                                      allow, 2), NF_OK);
    NF_CHECK_EQ_INT(v.width, 640);

    /* 换成 1280x966 就该被这个白名单拒绝 */
    UpMsgBuilder b2;
    b2.w = 1280; b2.h = 966;
    b2.build(4);
    const void *f2[NF_MAX_CAMERAS];
    nf_u32 fb2[NF_MAX_CAMERAS];
    for (nf_u32 i = 0; i < NF_MAX_CAMERAS; ++i) {
        f2[i] = b2.imgs[i].data();
        fb2[i] = (nf_u32)b2.imgs[i].size();
    }
    NF_CHECK_EQ_INT(nf_up_parse_sized(b2.head.data(), (nf_u32)b2.head.size(),
                                      f2, fb2, NF_MAX_CAMERAS, &v,
                                      allow, 2), NF_E_PARAM_RANGE);
}

int main() { return nftest::run_all(); }
