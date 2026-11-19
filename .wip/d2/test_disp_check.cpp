/**
 * @file test_disp_check.cpp
 * @brief D2 双向一致性的单元测试。
 *
 * 重点验证**符号约定** —— 实测符号搞反会让中位误差从 36px 涨到 85px，
 * 而这种错误在端到端测试里表现为"精度不够"，很难定位。
 */
#include <cmath>
#include <cstring>
#include <vector>

#include "nfocc/nfocc_disp_check.h"
#include "nfocc/nfocc_default.h"
#include "nf_test.h"

namespace {

constexpr nf_u32 W = 64, H = 8;

NFDispCheckConfig make_cfg(nf_u8 enable = 1) {
    NFDispCheckConfig c;
    std::memset(&c, 0, sizeof(c));
    c.version = NF_DISP_CHECK_VERSION;
    c.tau = 1.0f;
    c.lambda = 14.0f;
    c.w_min = 0.1f;
    c.enable = enable;
    return c;
}

}  // namespace

/**
 * 完全一致的正反向视差 -> 全部保留。
 *
 * **只在 x >= d 的区域断言全保留**：左图 x < d_fwd 的像素在右图里
 * 没有对应（xr = x - d < 0），判为遮挡是正确的几何行为，不是缺陷。
 */
NF_TEST(dc_consistent_all_kept) {
    const nf_f32 d = 10.0f;
    std::vector<nf_f32> df(W * H, d);
    std::vector<nf_f32> db(W * H, d);
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 0);
    NFDispCheckConfig c = make_cfg();

    nf_u32 occ = 0, kept = 0;
    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), &occ, &kept), NF_OK);

    /* 左边界 d 列判遮挡，其余保留 */
    const nf_u32 expect_occ = (nf_u32)d * H;
    NF_CHECK_EQ_INT(occ, (int)expect_occ);
    NF_CHECK_EQ_INT(kept, (int)(W * H - expect_occ));

    for (nf_u32 y = 0; y < H; ++y)
        for (nf_u32 x = (nf_u32)d; x < W; ++x)
            NF_CHECK_EQ_INT(keep[y * W + x], 1);
}

/* 反向视差差异巨大 -> 判为遮挡 */
NF_TEST(dc_inconsistent_occluded) {
    std::vector<nf_f32> df(W * H, 10.0f);
    std::vector<nf_f32> db(W * H, 60.0f);   /* 差 50px，远超 tau */
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 0);
    NFDispCheckConfig c = make_cfg();

    nf_u32 occ = 0, kept = 0;
    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), &occ, &kept), NF_OK);
    NF_CHECK_EQ_INT(kept, 0);
    NF_CHECK_EQ_INT(occ, (int)(W * H));
}

/**
 * **符号回归测试**。
 *
 * SGBM 输出恒为正值（engine_sgbm.cpp:147 用 `r > 0` 判有效），
 * 几何上的 d_bwd = -d_fwd 这个符号在引擎里丢了。所以反向图给出的
 * 数值就是 |d|，实现里**不该再取负**。
 *
 * 若实现误加了取负，残差会变成 d_fwd + d_bwd = 20，几乎全部判遮挡。
 * 本用例守的就是这个。
 */
NF_TEST(dc_sign_convention) {
    std::vector<nf_f32> df(W * H, 10.0f);
    std::vector<nf_f32> db(W * H, 10.0f);
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 0);
    NFDispCheckConfig c = make_cfg();

    nf_u32 kept = 0;
    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), nullptr, &kept), NF_OK);
    /* 同上：左边界 10 列判遮挡，其余应保留。漏了取负会保留 0 个 */
    const nf_u32 expect_kept = (W - 10u) * H;
    NF_CHECK_MSG(kept == expect_kept,
                 "同号视差应保留 (仅左边界遮挡)；若保留远少于预期说明漏了取负");
}

/* 关闭 D2 -> 只要有正向有效就保留 */
NF_TEST(dc_disabled_all_kept) {
    std::vector<nf_f32> df(W * H, 10.0f), db(W * H, 60.0f);
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 0);
    NFDispCheckConfig c = make_cfg(0);

    nf_u32 kept = 0;
    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), nullptr, &kept), NF_OK);
    NF_CHECK_EQ_INT(kept, (int)(W * H));
}

/* 正向无效 -> 剔除，且不依赖反向 */
NF_TEST(dc_fwd_invalid_dropped) {
    std::vector<nf_f32> df(W * H, 10.0f), db(W * H, 10.0f);
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 1);
    NFDispCheckConfig c = make_cfg();
    vf[0] = 0;

    nf_u32 occ = 0;
    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), &occ, nullptr), NF_OK);
    /* 该点在 x=0，本就落在左边界遮挡区内（x < d_fwd），
     * 所以 occ 仍是左边界 10 列 = 80，不会额外加 1。 */
    NF_CHECK_EQ_INT(keep[0], 0);
    NF_CHECK_EQ_INT(occ, 10 * (int)H);
}

/* 反向无效 -> 剔除（遮挡区在右图里找不到对应） */
NF_TEST(dc_bwd_invalid_dropped) {
    std::vector<nf_f32> df(W * H, 10.0f), db(W * H, 10.0f);
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 1);
    NFDispCheckConfig c = make_cfg();
    /* 采样索引是 xr = x - d_fwd = 20 - 10 = 10，
     * 要让它落空必须把 vb[y*W+10] 置 0，而不是 vb[y*W+20]。 */
    vb[5 * W + 10] = 0;

    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), nullptr, nullptr), NF_OK);
    NF_CHECK_EQ_INT(keep[5 * W + 20], 0);
}

/* 视差超出图像边界 -> 剔除 */
NF_TEST(dc_out_of_bound_dropped) {
    std::vector<nf_f32> df(W * H, 10.0f), db(W * H, 10.0f);
    std::vector<nf_u8> vf(W * H, 1), vb(W * H, 1), keep(W * H, 1);
    NFDispCheckConfig c = make_cfg();
    /* 第 0 行第 2 列，视差 10 -> xr = -8，越界 */
    df[2] = 10.0f;

    NF_CHECK_STATUS(nf_disp_check_combine(df.data(), vf.data(), db.data(), vb.data(),
                                          W, H, &c, keep.data(), nullptr, nullptr), NF_OK);
    NF_CHECK_EQ_INT(keep[2], 0);
}

/* 空指针保护 */
NF_TEST(dc_null_safety) {
    NFDispCheckConfig c = make_cfg();
    nf_u8 keep[8];
    NF_CHECK_EQ_INT(nf_disp_check_combine(nullptr, nullptr, nullptr, nullptr,
                                          W, H, &c, keep, nullptr, nullptr),
                    NF_E_PARAM_NULL);
}

int main() { return nftest::run_all(); }
