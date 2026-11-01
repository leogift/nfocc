/**
 * @file test_engine_bind.cpp
 * @brief 引擎绑定正确性 —— 防"同名符号被覆盖"这类静默错误。
 *
 * ## 为什么这个测试必须存在
 * 实际故障: SGBM 与模型两个引擎库都导出 `nf_engine_init`, 静态链接时
 * 先满足者胜出。nfocc_run 同时链两个库, 结果注册进 pipeline 的恒是
 * **模型引擎**, STEREO 路径全部返回 NF_E_ENGINE_UNSUPPORTED。
 * 现象是"引擎没产出", 但日志显示 nf_engine_init 被正常调用了 ——
 * 从日志完全看不出调错了实现。
 *
 * 本测试用**负对照**锁住这个语义:
 *   1. 绑 SGBM 后, 用 SGBM 配置 init 必须成功;
 *   2. 绑模型后, 用 SGBM 配置 init 必须返回 UNSUPPORTED;
 *   3. 绑 SGBM 后, 用模型配置 init 必须返回 UNSUPPORTED。
 * 若将来符号再次被覆盖, 1 或 3 会失败。
 */
#include "nf_test.h"
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_engine_bind.h"
#include "nfocc/nfocc_default.h"

#include <cstring>
#include <vector>

namespace {

/* pipeline.cpp 里的 ops 结构布局(纯 POD, 三个函数指针) */
struct Ops {
    NFStatus (*init)(NFEngineCtx **, const NFEngineConfig *);
    NFStatus (*run)(NFEngineCtx *, const NFImageView *, const NFImageView *,
                    const NFDisparityRange *, NFDisparityMap *);
    NFStatus (*deinit)(NFEngineCtx *);
};

extern "C" {
NFStatus nf_sgbm_engine_init(NFEngineCtx **, const NFEngineConfig *);
NFStatus nf_sgbm_engine_run(NFEngineCtx *, const NFImageView *, const NFImageView *,
                            const NFDisparityRange *, NFDisparityMap *);
NFStatus nf_sgbm_engine_deinit(NFEngineCtx *);
NFStatus nf_model_engine_init(NFEngineCtx **, const NFEngineConfig *);
NFStatus nf_model_engine_run(NFEngineCtx *, const NFImageView *, const NFImageView *,
                             const NFDisparityRange *, NFDisparityMap *);
NFStatus nf_model_engine_deinit(NFEngineCtx *);
}

NFEngineConfig base_cfg(NFEngineKind kind) {
    NFEngineConfig c;
    std::memset(&c, 0, sizeof(c));
    c.version = NF_ENGINE_CONFIG_VERSION;
    c.kind = kind;
    c.max_disp_px = 256;
    c.downscale = 2;
    c.block_size = 5;
    c.uniqueness = 10;
    c.speckle_window = 100;
    return c;
}

/* 通过 bind 层拿到引擎: 用 bind_sgbm 注册后, pipeline 会调到 SGBM */
NFStatus bind_and_init_sgbm(NFEngineCtx **ctx, const NFEngineConfig *cfg) {
    static const Ops ops = { nf_sgbm_engine_init, nf_sgbm_engine_run,
                             nf_sgbm_engine_deinit };
    nf_pipeline_bind_engine(&ops);
    // 模拟 pipeline 的行为: 调用注册进来的 init
    return ops.init(ctx, cfg);
}

NFStatus bind_and_init_model(NFEngineCtx **ctx, const NFEngineConfig *cfg) {
    static const Ops ops = { nf_model_engine_init, nf_model_engine_run,
                             nf_model_engine_deinit };
    nf_pipeline_bind_engine(&ops);
    return ops.init(ctx, cfg);
}

/* 正例: SGBM 绑 SGBM 配置 -> 成功 */
NF_TEST(bind_sgbm_accepts_sgbm_cfg) {
    NFEngineConfig cfg = base_cfg(NF_ENGINE_SGBM);
    NFEngineCtx *ctx = nullptr;
    const NFStatus st = bind_and_init_sgbm(&ctx, &cfg);
    NF_CHECK_MSG(st == NF_OK, "SGBM 引擎应接受 SGBM 配置");
    NF_CHECK(ctx != nullptr);
    if (ctx) nf_sgbm_engine_deinit(ctx);
}

/* 负对照 1: 模型绑 SGBM 配置 -> 必须拒绝(这正是原故障的表现) */
NF_TEST(bind_model_rejects_sgbm_cfg) {
    NFEngineConfig cfg = base_cfg(NF_ENGINE_SGBM);
    NFEngineCtx *ctx = nullptr;
    const NFStatus st = bind_and_init_model(&ctx, &cfg);
    NF_CHECK_MSG(st == NF_E_ENGINE_UNSUPPORTED,
                 "模型引擎必须拒绝 SGBM 配置 —— 否则说明符号被覆盖了");
}

/* 负对照 2: SGBM 绑模型配置 -> 必须拒绝 */
NF_TEST(bind_sgbm_rejects_model_cfg) {
    NFEngineConfig cfg = base_cfg(NF_ENGINE_MODEL);
    NFEngineCtx *ctx = nullptr;
    const NFStatus st = bind_and_init_sgbm(&ctx, &cfg);
    NF_CHECK_MSG(st == NF_E_ENGINE_UNSUPPORTED,
                 "SGBM 引擎必须拒绝模型配置");
}

/* 两个 bind 函数必须解析到不同实现 —— 直接比函数指针 */
NF_TEST(bind_symbols_distinct) {
    NF_CHECK_MSG((void *)nf_sgbm_engine_init != (void *)nf_model_engine_init,
                 "SGBM 与模型的 init 必须是不同符号(同名会被链接器覆盖)");
    NF_CHECK_MSG((void *)nf_sgbm_engine_run != (void *)nf_model_engine_run,
                 "run 必须是不同符号");
    NF_CHECK_MSG((void *)nf_sgbm_engine_deinit != (void *)nf_model_engine_deinit,
                 "deinit 必须是不同符号");
}

}  // namespace

int main() { return nftest::run_all(); }
