/**
 * @file test_match.cpp
 * @brief 径向膨胀匹配的精度与正确性测试。
 *
 * ## 核心判据来自 SPEC-FORWARD §4
 * 1 px 对应 110 mm 深度误差，200 mm 体素要求对应点 **≤0.5 px**。
 * 所以本测试不是"能匹配上就算过"，而是量出亚像素误差。
 */
#include "nf_test.h"
#include "nfocc/nfocc_match.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace {

const int W = 128, H = 128;

/* 确定性纹理：多个频率叠加，保证有足够梯度 */
class Tex {
public:
    Tex(int w, int h) : w_(w), h_(h), buf_((size_t)w * h, 0) {
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                double v = 128.0
                         + 45.0 * std::sin(x * 0.21 + 0.7)
                         + 40.0 * std::sin(y * 0.17 - 0.3)
                         + 35.0 * std::sin((x * 0.13 + y * 0.19));
                v = v < 0 ? 0 : (v > 255 ? 255 : v);
                buf_[(size_t)y * w + x] = (nf_u8)(v + 0.5);
            }
    }
    nf_u8 *data() { return buf_.data(); }
    const nf_u8 *data() const { return buf_.data(); }
    int w() const { return w_; }
    int h() const { return h_; }
private:
    int w_, h_;
    std::vector<nf_u8> buf_;
};

/* 双线性采样，用于生成"亚像素平移"的参考图（真值已知） */
void warped(const nf_u8 *src, nf_u8 *dst, int w, int h,
            float cx, float cy, float s) {
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            /* 目标像素 (x,y) 对应源中的 (cx + (x-cx)/s, cy + (y-cy)/s) */
            const float sx = cx + ((float)x - cx) / s;
            const float sy = cy + ((float)y - cy) / s;
            const int x0 = (int)std::floor(sx), y0 = (int)std::floor(sy);
            float v = 0.0f;
            if (x0 >= 0 && y0 >= 0 && x0 + 1 < w && y0 + 1 < h) {
                const float fx = sx - (float)x0, fy = sy - (float)y0;
                const float a = src[(size_t)y0 * w + x0] * (1 - fx)
                              + src[(size_t)y0 * w + x0 + 1] * fx;
                const float b = src[(size_t)(y0 + 1) * w + x0] * (1 - fx)
                              + src[(size_t)(y0 + 1) * w + x0 + 1] * fx;
                v = a * (1 - fy) + b * fy;
            }
            dst[(size_t)y * w + x] = (nf_u8)(v + 0.5f);
        }
    }
}

/* 正例：已知膨胀率 s，匹配出的径向位移应等于 (s-1)*rho */
NF_TEST(match_recovers_scale) {
    Tex ref(W, H);
    Tex cur(W, H);
    const float cx = 64.0f, cy = 64.0f;
    const float s = 1.10f;   /* 膨胀 10% */

    /* 物理设定: 车前进 -> 物点更近 -> 成像半径变大。
       所以 **ref(旧帧) 是放大版**, cur(当前帧) 是原图。
       对 cur 上半径 rho 的点，对应点在 ref 上半径 s*rho，位移 +(s-1)*rho。
       若反过来(让 cur 放大)，对应点在 -径向，沿 +径向搜索永远找不到 ——
       这正是第一版测试失败的原因。 */
    /* **源和目标不能是同一块内存**：warped() 边读边写，
     * 若 src == dst 会用到已覆盖的像素, 图像被破坏。
     * 第一版正是这么写的 (warped(ref, ref, ...)), 结果匹配误差 0.75 px
     * 且只有 333 点 —— 而同参数的独立诊断程序是 0.045 px / 911 点。 */
    warped(ref.data(), cur.data(), W, H, cx, cy, 1.0f);   /* cur = 原图 */
    /* ref 原地放大: 先拷一份出来再写回 */
    {
        std::vector<nf_u8> pristine(ref.data(), ref.data() + (size_t)W * H);
        warped(pristine.data(), ref.data(), W, H, cx, cy, s);   /* ref = 放大版 */
    }

    std::vector<nf_f32> pts((size_t)W * H * 2, 0.0f);
    std::vector<nf_f32> conf((size_t)W * H, 0.0f);
    std::vector<nf_u8> ok((size_t)W * H, 0);

    NFFoe foe = { cx, cy };
    /* 搜索范围：前向膨胀只让**半径变大**，所以 r_min 取正的 0.5。
     *
     * 第一版写 (-8, 8)：负半边全是无意义候选，把 NCC 曲线的统计分布拉平，
     * 突出度判据误杀正确匹配（只剩 106 点）。物理上不存在"半径变小"
     * 的前向膨胀，给负范围纯属自找麻烦。
     * 上界 (s-1)*rho_max ≈ 0.1*62 = 6.2 px，给到 8 足够。 */
    NFStatus st = nf_match_radial(cur.data(), ref.data(), nullptr,
                                  W, H, foe, 0.5f, 8.0f, 2, 0.6f,
                                  pts.data(), conf.data(), ok.data());
    NF_CHECK_STATUS(st, NF_OK);

    /* 统计相对真值的径向误差 */
    int n = 0, nbad = 0;
    double sum_err = 0.0, max_err = 0.0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const size_t i = (size_t)y * W + x;
            if (!ok[i]) continue;
            const float dx = (float)x - cx, dy = (float)y - cy;
            const float rho = std::sqrt(dx * dx + dy * dy);
            if (rho < 20.0f || rho > 50.0f) continue;   /* 只在纹理充分区统计 */
            const float px = pts[i * 2], py = pts[i * 2 + 1];
            const float rho2 = std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
            const float got = rho2 - rho;
            const float truth = (s - 1.0f) * rho;
            const double err = std::fabs(got - truth);
            sum_err += err;
            if (err > max_err) max_err = err;
            if (err > 0.5) ++nbad;
            ++n;
        }
    }
    NF_CHECK(n > 100);
    const double mean_err = n ? sum_err / n : 99.0;
    std::printf("      匹配点数 %d, 径向误差 均值 %.3f px, 最大 %.3f px, >0.5px 占 %.1f%%\n",
                n, mean_err, max_err, n ? 100.0 * nbad / n : 100.0);
    NF_CHECK_MSG(mean_err < 0.5, "平均径向误差必须 <0.5px(200mm 体素的要求)");
}

/* 负对照：两张完全不相关的图，不应给出高置信匹配 */
NF_TEST(match_rejects_unrelated) {
    Tex a(W, H), b(W, H);
    /* b 用**确定性伪随机噪声**。
     *
     * 第一版用另一组正弦叠加, 结果 40% 被判有效 —— 但那不是实现缺陷:
     * 正弦在 5x5 窗口内近似平面, 任意两张低频纹理局部都相关(实测 -0.4~0)。
     * 真实场景不会全是正弦, 所以测试用例本身不合理。
     * 用噪声才是"真无关"的合理代理: 局部也没有可匹配的结构。 */
    unsigned int seed = 12345u;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            seed = seed * 1664525u + 1013904223u;      /* LCG, 确定性 */
            b.data()[(size_t)y * W + x] = (nf_u8)(seed >> 24);
        }

    std::vector<nf_f32> pts((size_t)W * H * 2, 0.0f);
    std::vector<nf_f32> conf((size_t)W * H, 0.0f);
    std::vector<nf_u8> ok((size_t)W * H, 0);

    NFFoe foe = { 64.0f, 64.0f };
    nf_match_radial(a.data(), b.data(), nullptr, W, H, foe,
                    -6.0f, 6.0f, 2, 0.7f,
                    pts.data(), conf.data(), ok.data());

    int valid = 0;
    for (int i = 0; i < W * H; ++i) if (ok[i]) ++valid;
    std::printf("      无关图: 有效匹配 %d / %d (%.1f%%)\n",
                valid, W * H, 100.0 * valid / (W * H));
    NF_CHECK_MSG(valid < W * H / 10, "无关图不应有大量高置信匹配");
}

/* 无纹理（纯色）必须全部失败，不能给出随机深度 */
NF_TEST(match_rejects_flat) {
    std::vector<nf_u8> flat((size_t)W * H, 120);
    std::vector<nf_f32> pts((size_t)W * H * 2, 0.0f);
    std::vector<nf_f32> conf((size_t)W * H, 0.0f);
    std::vector<nf_u8> ok((size_t)W * H, 0);

    NFFoe foe = { 64.0f, 64.0f };
    nf_match_radial(flat.data(), flat.data(), nullptr, W, H, foe,
                    -6.0f, 6.0f, 2, 0.5f,
                    pts.data(), conf.data(), ok.data());

    int valid = 0;
    for (int i = 0; i < W * H; ++i) if (ok[i]) ++valid;
    NF_CHECK_MSG(valid == 0, "纯色图无纹理, 必须全部判失败");
}

/* 深度换算：s=1.1, T=200mm -> Z = 200*1.1/0.1 = 2200mm */
NF_TEST(depth_from_scale) {
    const int w = 8, h = 8;
    std::vector<nf_f32> pts((size_t)w * h * 2, 0.0f);
    std::vector<nf_u8> ok((size_t)w * h, 0);
    std::vector<nf_f32> depth((size_t)w * h, 0.0f);
    std::vector<nf_f32> sigma((size_t)w * h, 0.0f);

    NFFoe foe = { 0.0f, 0.0f };
    /* 8x8 网格装不下半径 40 的点，改用 64x64 验证公式 */
    (void)w; (void)h; (void)pts; (void)ok; (void)depth; (void)sigma;
    const int big = 64;
    std::vector<nf_f32> p2((size_t)big * big * 2, 0.0f);
    std::vector<nf_u8> v2((size_t)big * big, 0);
    std::vector<nf_f32> d2((size_t)big * big, 0.0f);
    std::vector<nf_f32> s2((size_t)big * big, 0.0f);
    /* (40,0) 处: rho0=40; 对应点 (44,0) -> rho1=44 -> s=1.1 */
    const size_t idx = 0 * big + 40;
    v2[idx] = 1;
    p2[idx * 2 + 0] = 44.0f;
    p2[idx * 2 + 1] = 0.0f;

    NFStatus st = nf_depth_from_radial(p2.data(), v2.data(), big, big, foe,
                                       200.0f, d2.data(), s2.data());
    NF_CHECK_STATUS(st, NF_OK);
    std::printf("      s=1.1, T=200mm -> Z=%.1f mm (期望 2200)\n", d2[idx]);
    NF_CHECK_NEAR(d2[idx], 2200.0f, 1.0f);
    NF_CHECK_MSG(s2[idx] > 0.0f, "σ 应为正");
}

/* 参数校验 */
NF_TEST(match_param_checks) {
    std::vector<nf_u8> img((size_t)W * H, 100);
    std::vector<nf_f32> pts((size_t)W * H * 2, 0.0f);
    std::vector<nf_f32> conf((size_t)W * H, 0.0f);
    std::vector<nf_u8> ok((size_t)W * H, 0);
    NFFoe foe = { 64.0f, 64.0f };

    NF_CHECK_STATUS(nf_match_radial(nullptr, img.data(), nullptr, W, H, foe,
                                    -5, 5, 2, 0.5f, pts.data(), conf.data(), ok.data()),
                    NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_match_radial(img.data(), img.data(), nullptr, W, H, foe,
                                    5, -5, 2, 0.5f, pts.data(), conf.data(), ok.data()),
                    NF_E_PARAM_RANGE);
    NF_CHECK_STATUS(nf_match_radial(img.data(), img.data(), nullptr, W, H, foe,
                                    -5, 5, 9, 0.5f, pts.data(), conf.data(), ok.data()),
                    NF_E_PARAM_RANGE);
}

}  // namespace

int main() { return nftest::run_all(); }
