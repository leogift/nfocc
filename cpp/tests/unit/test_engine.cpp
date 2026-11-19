/**
 * @file test_engine.cpp
 * @brief 引擎单元测试。对应 SPEC-VERIFY 的 V-ENG-1..7。
 */
#include "nf_test.h"
/* 本测试只链接 SGBM 引擎库。公共名 nf_engine_* 现在由 bind 层在运行时选择,
 * 实现侧导出的是 nf_sgbm_engine_*。这里在编译期映射, 测试代码不必改调用点。 */
#include "nfocc/nfocc_engine.h"
#define nf_engine_init   nf_sgbm_engine_init
#define nf_engine_run    nf_sgbm_engine_run
#define nf_engine_deinit nf_sgbm_engine_deinit
extern "C" {
NFStatus nf_sgbm_engine_init  (NFEngineCtx **ctx, const NFEngineConfig *cfg);
NFStatus nf_sgbm_engine_run   (NFEngineCtx *ctx, const NFImageView *left,
                               const NFImageView *right,
                               const NFDisparityRange *range, NFDisparityMap *out);
NFStatus nf_sgbm_engine_deinit(NFEngineCtx *ctx);
}
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace {

/* 构造一对有纹理的左右图: right(x) = left(x + shift) */
void make_stereo_pair(std::vector<nf_u8> &L, std::vector<nf_u8> &R,
                      int w, int h, int shift) {
    L.assign((size_t)w * h, 0);
    R.assign((size_t)w * h, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            /* 多频正弦, 保证 SGBM 有稳定代价谷 */
            const double v = 128.0
                + 70.0 * std::sin(x * 0.27)
                + 60.0 * std::sin(y * 0.19)
                + 40.0 * std::sin((x + y) * 0.11);
            const double v2 = 128.0
                + 70.0 * std::sin((x + shift) * 0.27)
                + 60.0 * std::sin(y * 0.19)
                + 40.0 * std::sin((x + shift + y) * 0.11);
            double c = v;  c = c < 0 ? 0 : (c > 255 ? 255 : c);
            double c2 = v2; c2 = c2 < 0 ? 0 : (c2 > 255 ? 255 : c2);
            L[(size_t)y * w + x] = (nf_u8)(c + 0.5);
            R[(size_t)y * w + x] = (nf_u8)(c2 + 0.5);
        }
    }
}

NFImageView view_of(const std::vector<nf_u8> &buf, int w, int h) {
    NFImageView v;
    v.version = NF_IMAGE_VIEW_VERSION;
    v.data = buf.data();
    v.width = (nf_u32)w;
    v.height = (nf_u32)h;
    v.stride = (nf_u32)w;
    v.valid_x0 = nullptr;
    v.timestamp_us = 0;
    v.frame_id = 0;
    return v;
}

/* V-ENG-1: 容量不足必须拒绝, **且不写输出** (S-2) */
NF_TEST(eng1_range_reject) {
    NFEngineConfig cfg;
    cfg.version = NF_ENGINE_CONFIG_VERSION;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = 64;
    cfg.downscale = 1;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 100;

    NFEngineCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_engine_init(&ctx, &cfg), NF_OK);
    NF_CHECK(ctx != nullptr);

    const int w = 320, h = 128;
    std::vector<nf_u8> L, R;
    make_stereo_pair(L, R, w, h, 8);
    NFImageView lv = view_of(L, w, h);
    NFImageView rv = view_of(R, w, h);

    NFDisparityMap dm;
    dm.version = NF_DISPARITY_MAP_VERSION;
    NF_CHECK_STATUS(nf_dmap_alloc(&dm, w, h), NF_OK);

    /* 预填哨兵值, 用于确认未被写入 */
    const float sentinel = -12345.0f;
    for (size_t i = 0; i < (size_t)w * h; ++i) dm.disp[i] = sentinel;

    NFDisparityRange range;
    range.version = NF_DISPARITY_RANGE_VERSION;
    range.d_min = 0.0f;
    range.d_max = 128.0f;      /* 容量 64 < 128 */
    range.b_mm = 120.0f;

    NF_CHECK_STATUS(nf_engine_run(ctx, &lv, &rv, &range, &dm), NF_E_ENGINE_RANGE);
    /* 输出必须保持哨兵 —— 这是 S-2 的核心 */
    int touched = 0;
    for (size_t i = 0; i < (size_t)w * h; ++i)
        if (dm.disp[i] != sentinel) ++touched;
    NF_CHECK_EQ_INT(touched, 0);

    nf_dmap_destroy(&dm);
    nf_engine_deinit(ctx);
}

/* V-ENG-2: 确定性 —— 两次调用逐位相同 (E-8) */
NF_TEST(eng2_deterministic) {
    NFEngineConfig cfg;
    cfg.version = NF_ENGINE_CONFIG_VERSION;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = 64;
    cfg.downscale = 1;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 100;

    NFEngineCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_engine_init(&ctx, &cfg), NF_OK);

    const int w = 320, h = 128;
    std::vector<nf_u8> L, R;
    make_stereo_pair(L, R, w, h, 8);
    NFImageView lv = view_of(L, w, h);
    NFImageView rv = view_of(R, w, h);

    NFDisparityMap d1, d2;
    d1.version = NF_DISPARITY_MAP_VERSION;
    d2.version = NF_DISPARITY_MAP_VERSION;
    nf_dmap_alloc(&d1, w, h);
    nf_dmap_alloc(&d2, w, h);

    NFDisparityRange range;
    range.version = NF_DISPARITY_RANGE_VERSION;
    range.d_min = 0.0f;
    range.d_max = 64.0f;
    range.b_mm = 120.0f;

    NF_CHECK_STATUS(nf_engine_run(ctx, &lv, &rv, &range, &d1), NF_OK);
    NF_CHECK_STATUS(nf_engine_run(ctx, &lv, &rv, &range, &d2), NF_OK);

    const size_t n = (size_t)w * h;
    NF_CHECK_EQ_INT(std::memcmp(d1.disp, d2.disp, n * sizeof(nf_f32)), 0);
    NF_CHECK_EQ_INT(std::memcmp(d1.valid, d2.valid, n * sizeof(nf_u8)), 0);

    nf_dmap_destroy(&d1);
    nf_dmap_destroy(&d2);
    nf_engine_deinit(ctx);
}

/* V-ENG-4: 全黑输入 -> valid 全 0, 且不得用 disp=0 冒充有效 (E-6) */
NF_TEST(eng4_black_input) {
    NFEngineConfig cfg;
    cfg.version = NF_ENGINE_CONFIG_VERSION;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = 64;
    cfg.downscale = 1;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 100;

    NFEngineCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_engine_init(&ctx, &cfg), NF_OK);

    const int w = 320, h = 128;
    std::vector<nf_u8> L((size_t)w * h, 0), R((size_t)w * h, 0);
    NFImageView lv = view_of(L, w, h);
    NFImageView rv = view_of(R, w, h);

    NFDisparityMap dm;
    dm.version = NF_DISPARITY_MAP_VERSION;
    nf_dmap_alloc(&dm, w, h);

    NFDisparityRange range;
    range.version = NF_DISPARITY_RANGE_VERSION;
    range.d_min = 0.0f;
    range.d_max = 64.0f;
    range.b_mm = 120.0f;

    NF_CHECK_STATUS(nf_engine_run(ctx, &lv, &rv, &range, &dm), NF_OK);
    nf_u32 n_valid = 0;
    for (size_t i = 0; i < (size_t)w * h; ++i) n_valid += dm.valid[i];
    NF_CHECK_MSG(n_valid == 0, "全黑输入不应有任何有效像素");

    nf_dmap_destroy(&dm);
    nf_engine_deinit(ctx);
}

/* V-ENG-7: 覆盖率 = (W - numDisp)/W (S-3) */
NF_TEST(eng7_coverage) {
    const int w = 640, h = 128, nd = 64;
    NFEngineConfig cfg;
    cfg.version = NF_ENGINE_CONFIG_VERSION;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = nd;
    cfg.downscale = 1;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 30;

    NFEngineCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_engine_init(&ctx, &cfg), NF_OK);

    std::vector<nf_u8> L, R;
    make_stereo_pair(L, R, w, h, 8);
    NFImageView lv = view_of(L, w, h);
    NFImageView rv = view_of(R, w, h);

    NFDisparityMap dm;
    dm.version = NF_DISPARITY_MAP_VERSION;
    nf_dmap_alloc(&dm, w, h);

    NFDisparityRange range;
    range.version = NF_DISPARITY_RANGE_VERSION;
    range.d_min = 0.0f;
    range.d_max = (nf_f32)nd;
    range.b_mm = 120.0f;

    NF_CHECK_STATUS(nf_engine_run(ctx, &lv, &rv, &range, &dm), NF_OK);

    /* 左 nd 列必须全无效 */
    int bad_left = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < nd; ++x)
            bad_left += dm.valid[(size_t)y * w + x];
    NF_CHECK_EQ_INT(bad_left, 0);

    /* 解析覆盖率 */
    const float expect = nf_engine_expected_valid_ratio((nf_u32)w, nd);
    NF_CHECK_NEAR(expect, (float)(w - nd) / (float)w, 1e-6f);

    nf_dmap_destroy(&dm);
    nf_engine_deinit(ctx);
}

/* V-ENG-6: 内存核算 (S-4) */
NF_TEST(eng6_memory) {
    NFEngineConfig cfg;
    cfg.version = NF_ENGINE_CONFIG_VERSION;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = 256;
    cfg.downscale = 2;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 100;

    nf_u64 bytes = 0;
    NF_CHECK_STATUS(nf_engine_mem_bytes(&cfg, 960, 768, &bytes), NF_OK);
    /* 256*960*768*2 = 377 MB */
    const double mb = (double)bytes / 1e6;
    NF_CHECK_MSG(mb > 370.0 && mb < 385.0, "内存估算偏离 PARAM E.1");
    NF_CHECK_MSG(mb < 450.0, "超出上板预算");
}

/* numDisparities 必须是 16 的整数倍 */
NF_TEST(eng_init_param_check) {
    NFEngineConfig cfg;
    cfg.version = NF_ENGINE_CONFIG_VERSION;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = 100;   /* 不是 16 的倍数 */
    cfg.downscale = 1;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 100;

    NFEngineCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_engine_init(&ctx, &cfg), NF_E_PARAM_RANGE);
    NF_CHECK(ctx == nullptr);
}

/* version 不匹配必须被拒 (V-ICD-4) */
NF_TEST(eng_version_check) {
    NFEngineConfig cfg;
    cfg.version = 999;
    cfg.kind = NF_ENGINE_SGBM;
    cfg.max_disp_px = 64;
    cfg.downscale = 1;
    cfg.conf_thresh = 0.0f;
    cfg.block_size = 5;
    cfg.uniqueness = 10;
    cfg.speckle_window = 100;
    NFEngineCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_engine_init(&ctx, &cfg), NF_E_PARAM_VERSION);
    NF_CHECK(ctx == nullptr);
}

/* NULL 安全 */
NF_TEST(eng_null_safety) {
    NF_CHECK_STATUS(nf_engine_init(nullptr, nullptr), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_engine_run(nullptr, nullptr, nullptr, nullptr, nullptr),
                    NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_engine_deinit(nullptr), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_dmap_alloc(nullptr, 10, 10), NF_E_PARAM_NULL);
}

} // namespace

int main() { return nftest::run_all(); }
