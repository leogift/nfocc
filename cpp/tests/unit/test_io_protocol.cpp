/**
 * @file test_io_protocol.cpp
 * @brief 上下游消息解析测试。对应 SPEC-UPSTREAM §6 与 SPEC-DOWNSTREAM §8。
 *
 * 跨进程来的数据一律当**不可信输入**, 所以这里大半是负对照:
 * 伪造长度、截断、尺寸不符、时间戳失同步 —— 每一种都必须被拒绝。
 */
#include "nf_test.h"
#include "nfocc/nfocc_io.h"
#include "nfocc/nfocc_default.h"

#include <cstring>
#include <vector>

namespace {

/* 构造一条合法的上游 multipart 消息 */
struct UpMsg {
    std::vector<nf_u8> head;
    std::vector<std::vector<nf_u8>> frames;
    std::vector<const void *> ptrs;
    std::vector<nf_u32> sizes;
};

UpMsg make_up(int w = 1920, int h = 1536, nf_u64 ts = 1000000) {
    UpMsg m;
    m.head.resize(sizeof(NFUpHeader), 0);
    NFUpHeader *hd = (NFUpHeader *)m.head.data();
    hd->magic = NF_UP_MAGIC;
    hd->version = NF_UP_VERSION;
    hd->frame_id = 42;
    hd->timestamp_us = ts;
    hd->n_cameras = 4;
    hd->ego.version = 1;
    hd->ego.timestamp_us = ts;
    hd->ego.quat[0] = 1.0f;
    hd->ego.source = 0;
    hd->ego.quality = 90;

    m.frames.resize(4);
    for (int i = 0; i < 4; ++i) {
        const nf_u32 bytes = (nf_u32)w * h;
        m.frames[i].resize(sizeof(NFUpImageHeader) + bytes, 0);
        NFUpImageHeader *ih = (NFUpImageHeader *)m.frames[i].data();
        ih->cam_id = (nf_u8)i;
        ih->format = 0;
        ih->width = (nf_u32)w;
        ih->height = (nf_u32)h;
        ih->stride = (nf_u32)w;
        ih->timestamp_us = ts + i * 1000;    /* 极差 3ms < 5ms */
        ih->frame_id = 42;
        ih->data_bytes = bytes;
        m.ptrs.push_back(m.frames[i].data());
        m.sizes.push_back((nf_u32)m.frames[i].size());
    }
    return m;
}

/* 正例: 合法消息应解析成功且指针指向像素起始 */
NF_TEST(up_parse_ok) {
    UpMsg m = make_up();
    NFUpFrameView v;
    NFStatus st = nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                              m.ptrs.data(), m.sizes.data(), 4, &v);
    NF_CHECK_STATUS(st, NF_OK);
    NF_CHECK_EQ_INT(v.header.frame_id, 42);
    NF_CHECK_EQ_INT(v.width, 1920);
    NF_CHECK_EQ_INT(v.height, 1536);
    for (int i = 0; i < 4; ++i) {
        NF_CHECK(v.pixels[i] != nullptr);
        /* 像素指针必须是段起点 + 头长度 */
        const nf_u8 *expect = (const nf_u8 *)m.frames[i].data() + sizeof(NFUpImageHeader);
        NF_CHECK_MSG(v.pixels[i] == expect, "像素指针偏移错误");
    }
}

/* 负对照: magic / version 错 */
NF_TEST(up_bad_magic) {
    UpMsg m = make_up();
    ((NFUpHeader *)m.head.data())->magic = 0xDEADBEEF;
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_VERSION);
}

/* 负对照: n_cameras 不是 4 */
NF_TEST(up_bad_cam_count) {
    UpMsg m = make_up();
    ((NFUpHeader *)m.head.data())->n_cameras = 3;
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_RANGE);
}

/* 负对照: 图像尺寸不符(不是 1920x1536) */
NF_TEST(up_bad_dims) {
    UpMsg m = make_up(1280, 720);
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_RANGE);
}

/* 负对照: data_bytes 与 stride*height 不符 —— 上游截断的典型表现 */
NF_TEST(up_truncated_databytes) {
    UpMsg m = make_up();
    NFUpImageHeader *ih = (NFUpImageHeader *)m.frames[2].data();
    ih->data_bytes -= 100;      /* 声明比实际少 */
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_RANGE);
}

/* 负对照: 段实际长度装不下声明的像素 */
NF_TEST(up_short_frame_buffer) {
    UpMsg m = make_up();
    m.sizes[1] = sizeof(NFUpImageHeader) + 1000;   /* 远小于 data_bytes */
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_RANGE);
}

/* 负对照: 四路时间戳极差 > 5ms —— 不同步 */
NF_TEST(up_timestamp_skew) {
    UpMsg m = make_up();
    NFUpImageHeader *ih = (NFUpImageHeader *)m.frames[3].data();
    ih->timestamp_us += 20000;   /* 20 ms 偏差 */
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_RANGE);
}

/* 位姿降级: quality < 50 应返回 DEGRADED, 但不是格式错误 */
NF_TEST(up_pose_degraded) {
    UpMsg m = make_up();
    ((NFUpHeader *)m.head.data())->ego.quality = 30;
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_BASELINE_DEGRADED);
    NF_CHECK_EQ_INT(v.header.ego.quality, 30);
}

/* 负对照: 相机 id 重复 */
NF_TEST(up_dup_cam_id) {
    UpMsg m = make_up();
    ((NFUpImageHeader *)m.frames[1].data())->cam_id = 0;   /* 与 cam0 重复 */
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(m.head.data(), (nf_u32)m.head.size(),
                                m.ptrs.data(), m.sizes.data(), 4, &v),
                    NF_E_PARAM_RANGE);
}

/* NULL 安全 */
NF_TEST(up_null_safety) {
    NFUpFrameView v;
    NF_CHECK_STATUS(nf_up_parse(nullptr, 0, nullptr, nullptr, 4, &v),
                    NF_E_PARAM_NULL);
}

/* ---------- 下游 ---------- */

NFOutHeader make_out(nf_u32 n_pts, nf_u32 cells) {
    NFOutHeader h;
    std::memset(&h, 0, sizeof(h));
    h.magic = NF_OUT_MAGIC;
    h.version = NF_OUT_VERSION;
    h.num_points = n_pts;
    h.points_bytes = n_pts * (nf_u32)sizeof(NFPoint);
    h.zmap_cells = cells;
    h.zmap_bytes = cells * 4u;
    h.zindex_bytes = cells ? (cells + 1u) * 4u : 0u;
    return h;
}

/* 正例: 总字节数 = 头 + 点云 + zMap + zIndex */
NF_TEST(out_size_ok) {
    NFOutHeader h = make_out(1000, 6750);
    nf_u32 total = 0;
    NF_CHECK_STATUS(nf_out_size(&h, &total), NF_OK);
    const nf_u32 expect = (nf_u32)sizeof(NFOutHeader) + 1000u * 15u
                        + 6750u * 4u + 6751u * 4u;
    NF_CHECK_EQ_INT(total, expect);
}

/* 冻结时无点无高程: 总长就是头 */
NF_TEST(out_size_frozen) {
    NFOutHeader h = make_out(0, 0);
    nf_u32 total = 0;
    NF_CHECK_STATUS(nf_out_size(&h, &total), NF_OK);
    NF_CHECK_EQ_INT(total, sizeof(NFOutHeader));
}

/* 负对照: points_bytes 与 num_points 不符 —— 会让下游按错偏移读 zMap */
NF_TEST(out_bad_points_bytes) {
    NFOutHeader h = make_out(1000, 6750);
    h.points_bytes = 999 * 15;
    nf_u32 total = 0;
    NF_CHECK_STATUS(nf_out_size(&h, &total), NF_E_PARAM_RANGE);
}

/* 负对照: zindex_bytes 不是 (cells+1)*4 */
NF_TEST(out_bad_zindex_bytes) {
    NFOutHeader h = make_out(100, 6750);
    h.zindex_bytes = 6750 * 4;    /* 少了一个 */
    nf_u32 total = 0;
    NF_CHECK_STATUS(nf_out_size(&h, &total), NF_E_PARAM_RANGE);
}

/* 负对照: magic/version 错 */
NF_TEST(out_bad_magic) {
    NFOutHeader h = make_out(10, 100);
    h.magic = 0;
    nf_u32 total = 0;
    NF_CHECK_STATUS(nf_out_size(&h, &total), NF_E_PARAM_VERSION);
}

/* 尺寸契约: 头必须是 64 字节, 点 15 字节 (V-DS-1) */
NF_TEST(out_layout_fixed) {
    NF_CHECK_EQ_INT(sizeof(NFOutHeader), 64);
    NF_CHECK_EQ_INT(sizeof(NFPoint), 15);
    NF_CHECK_EQ_INT(sizeof(NFUpEgoMotion), 60);
    NF_CHECK_EQ_INT(sizeof(NFUpHeader), 88);
    NF_CHECK_EQ_INT(sizeof(NFUpImageHeader), 32);
}

}  // namespace

int main() { return nftest::run_all(); }
