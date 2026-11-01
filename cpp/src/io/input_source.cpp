/**
 * @file input_source.cpp
 * @brief 三种输入源的实现。
 *
 * ## 时间戳纪律（SPEC-UPSTREAM §2.1）
 * 一律用**单调时钟**。合成/文件源的 ts 是合成出来的(按 5Hz 递增)；
 * ZMQ 源用上游发来的 ts。绝不用墙钟 —— NTP 校正一次几十毫秒，比帧周期还大。
 */
#include "nfocc/util/input_source.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef NF_HAS_ZMQ
#include <zmq.h>
#endif

namespace nf {

namespace {

const uint32_t kW = NF_IMG_W;
const uint32_t kH = NF_IMG_H;

/** 确定性纹理：同一 3D 点在两帧必须得同一灰度 */
void synth_image(uint8_t *buf, uint32_t w, uint32_t h, float phase) {
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            double v = 128.0
                     + 60.0 * std::sin(x * 0.031 + phase)
                     + 50.0 * std::sin(y * 0.024 - phase)
                     + 40.0 * std::sin((x + y) * 0.017);
            v = v < 0 ? 0 : (v > 255 ? 255 : v);
            buf[(size_t)y * w + x] = (uint8_t)(v + 0.5);
        }
    }
}

/* ---------------------------------------------------------------- 合成 */

class SyntheticSource : public InputSource {
public:
    SyntheticSource(uint32_t w, uint32_t h, int n, float move_mm, bool fwd)
        : w_(w), h_(h), n_(n), move_(move_mm), fwd_(fwd) {
        for (int c = 0; c < 4; ++c) {
            cur_[c].assign((size_t)w * h, 0);
            ref_[c].assign((size_t)w * h, 0);
            /* 参考帧 = 第 0 帧前一步: 这样第 0 帧就有合法基线 */
            synth_image(ref_[c].data(), w, h, (float)(-PhaseStep() + c * 0.5f));
        }
    }

    NFStatus ReadFrame(NFFrameInput *in, bool *finished) override {
        if (idx_ >= n_) { *finished = true; return NF_E_BASELINE_IDLE; }
        *finished = false;

        /* 相位步长由"目标深度 3m"反推, 保证点落在网格内 */
        const float step = PhaseStep();
        for (int c = 0; c < 4; ++c) {
            /* **先把上一帧拷进参考, 再生成当前帧**。
             * 反过来的话 in->img_ref 指向的是 ref_, 而 ref_ 随后被本帧覆盖,
             * 于是 SGBM 拿到两张完全相同的图 -> 视差恒 0 -> 0 点。
             * 这是"指针别名"型错误: 代码看起来每组都赋值了, 实际读到时已是新内容。 */
            std::memcpy(ref_[c].data(), cur_[c].data(), (size_t)w_ * h_);
            synth_image(cur_[c].data(), w_, h_, (float)(idx_ * step + c * 0.5f));
            in->img[c] = cur_[c].data();
            in->img_ref[c] = ref_[c].data();
            /* **t_cam 用视口系（SPEC-CONVENTION §2）**。
             * 合成源直接按"视口系"给位移：
             *   fwd_=false -> 沿视口 x 横移（侧行, 用于测 STEREO）
             *   fwd_=true  -> 沿视口 z 纵移（直行, 用于测 FORWARD）
             * 调用方须保证各相机视口 yaw 与这里的假设一致（默认全 0/车系对齐）。 */
            in->t_cam[c][0] = fwd_ ? 0.0f : move_;
            in->t_cam[c][1] = 0.0f;
            in->t_cam[c][2] = fwd_ ? move_ : 0.0f;
        }
        in->width = w_;
        in->height = h_;
        in->stride = w_;
        in->frame_id = (uint32_t)idx_;
        in->timestamp_us = 1000000ull + (uint64_t)idx_ * 200000ull;
        in->frame_gap = 1;
        in->body_mask = nullptr;
        ++idx_;
        return NF_OK;
    }

    const char *Name() const override { return "synthetic"; }
    uint32_t Width() const override { return w_; }
    uint32_t Height() const override { return h_; }
    bool HasPose() const override { return true; }

private:
    float PhaseStep() const {
        /* 视差 d = f*move/Z_target, 相位 = d*0.031 */
        const double f = NF_VF_VIEW;
        const double z = 3000.0;
        return (float)(f * move_ / z * 0.031);
    }
    uint32_t w_, h_;
    int n_;
    float move_;
    bool fwd_;
    int idx_ = 0;
    std::vector<uint8_t> cur_[4], ref_[4];
};

/* ---------------------------------------------------------------- 文件 */

class FileSource : public InputSource {
public:
    explicit FileSource(const std::string &path) {
        fp_ = std::fopen(path.c_str(), "rb");
        if (!fp_) return;
        uint32_t magic = 0, ver = 0;
        if (std::fread(&magic, 4, 1, fp_) != 1 || magic != 0x4E46504Bu) return;
        if (std::fread(&ver, 4, 1, fp_) != 1 || ver != 1) return;
        if (std::fread(&w_, 4, 1, fp_) != 1) return;
        if (std::fread(&h_, 4, 1, fp_) != 1) return;
        if (std::fread(&n_, 4, 1, fp_) != 1) return;
        ok_ = (w_ > 0 && h_ > 0);
        for (int c = 0; c < 4; ++c) {
            cur_[c].assign((size_t)w_ * h_, 0);
            ref_[c].assign((size_t)w_ * h_, 0);
        }
    }
    ~FileSource() override { if (fp_) std::fclose(fp_); }

    bool ok() const { return ok_ && fp_ != nullptr; }

    NFStatus ReadFrame(NFFrameInput *in, bool *finished) override {
        if (!ok_ || idx_ >= n_) { *finished = true; return NF_E_BASELINE_IDLE; }
        *finished = false;

        uint32_t fid = 0;
        uint64_t ts = 0;
        float t[12];
        if (std::fread(&fid, 4, 1, fp_) != 1) { *finished = true; return NF_E_PARAM_RANGE; }
        if (std::fread(&ts, 8, 1, fp_) != 1) { *finished = true; return NF_E_PARAM_RANGE; }
        if (std::fread(t, 4, 12, fp_) != 12) { *finished = true; return NF_E_PARAM_RANGE; }

        for (int c = 0; c < 4; ++c) {
            /* 上一帧成为参考（第 0 帧用自己的副本，基线由 t_cam 表达） */
            if (idx_ > 0) std::memcpy(ref_[c].data(), cur_[c].data(), (size_t)w_ * h_);
            const size_t sz = (size_t)w_ * h_;
            if (std::fread(cur_[c].data(), 1, sz, fp_) != sz) {
                *finished = true;
                return NF_E_PARAM_RANGE;
            }
            in->img[c] = cur_[c].data();
            in->img_ref[c] = (idx_ > 0) ? ref_[c].data() : cur_[c].data();
            for (int k = 0; k < 3; ++k) in->t_cam[c][k] = t[c * 3 + k];
        }
        in->width = w_;
        in->height = h_;
        in->stride = w_;
        in->frame_id = fid;
        in->timestamp_us = ts;
        in->frame_gap = 1;
        in->body_mask = nullptr;
        ++idx_;
        return NF_OK;
    }

    const char *Name() const override { return "file"; }
    uint32_t Width() const override { return w_; }
    uint32_t Height() const override { return h_; }
    bool HasPose() const override { return true; }

private:
    FILE *fp_ = nullptr;
    bool ok_ = false;
    uint32_t w_ = 0, h_ = 0, n_ = 0, idx_ = 0;
    std::vector<uint8_t> cur_[4], ref_[4];
};

#ifdef NF_HAS_ZMQ
/* ---------------------------------------------------------------- ZMQ */

class ZmqSource : public InputSource {
public:
    ZmqSource(const std::string &ep, uint32_t w, uint32_t h, int timeout_ms)
        : w_(w), h_(h), tmo_(timeout_ms) {
        ctx_ = zmq_ctx_new();
        if (!ctx_) return;
        sock_ = zmq_socket(ctx_, ZMQ_SUB);
        if (!sock_) return;
        int hwm = 2;                                  /* 只留最新帧 */
        zmq_setsockopt(sock_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
        zmq_setsockopt(sock_, ZMQ_SUBSCRIBE, "", 0);
        if (zmq_connect(sock_, ep.c_str()) != 0) { ok_ = false; return; }
        ok_ = true;
        for (int c = 0; c < 4; ++c) {
            cur_[c].assign((size_t)w * h, 0);
            ref_[c].assign((size_t)w * h, 0);
        }
    }
    ~ZmqSource() override {
        if (sock_) zmq_close(sock_);
        if (ctx_) zmq_ctx_destroy(ctx_);
    }

    bool ok() const { return ok_; }

    NFStatus ReadFrame(NFFrameInput *in, bool *finished) override {
        if (!ok_) { *finished = true; return NF_E_ENGINE_NOT_INIT; }
        *finished = false;

        /* 段 0: 头 */
        zmq_pollitem_t item = { sock_, 0, ZMQ_POLLIN, 0 };
        if (zmq_poll(&item, 1, tmo_) == 0) { *finished = true; return NF_E_BASELINE_IDLE; }

        zmq_msg_t m0;
        zmq_msg_init(&m0);
        if (zmq_msg_recv(&m0, sock_, 0) < 0) {
            zmq_msg_close(&m0);
            *finished = true;
            return NF_E_PARAM_RANGE;
        }
        /* 收齐 4 段图像 */
        std::vector<zmq_msg_t> msgs(4);
        for (int c = 0; c < 4; ++c) zmq_msg_init(&msgs[c]);
        for (int c = 0; c < 4; ++c) {
            if (zmq_msg_recv(&msgs[c], sock_, 0) < 0) {
                for (int k = 0; k < 4; ++k) zmq_msg_close(&msgs[k]);
                zmq_msg_close(&m0);
                *finished = true;
                return NF_E_PARAM_RANGE;
            }
        }

        const void *ptrs[4];
        uint32_t sizes[4];
        for (int c = 0; c < 4; ++c) {
            ptrs[c] = zmq_msg_data(&msgs[c]);
            sizes[c] = (uint32_t)zmq_msg_size(&msgs[c]);
        }

        NFUpFrameView v;
        NFStatus st = nf_up_parse(zmq_msg_data(&m0),
                                  (uint32_t)zmq_msg_size(&m0),
                                  ptrs, sizes, 4, &v);
        zmq_msg_close(&m0);
        for (int c = 0; c < 4; ++c) zmq_msg_close(&msgs[c]);

        if (st != NF_OK && st != NF_E_BASELINE_DEGRADED) {
            /* 格式错误: 跳过这一帧, 不中断整条流 */
            return st;
        }

        /* 上游给的是 ego 位姿; 这里用相邻帧位移近似 t_cam（相机系 = 车系前向）。
           真实系统应由上游直接给各相机 t_cam，见 SPEC-UPSTREAM §2.2。 */
        for (int c = 0; c < 4; ++c) {
            if (idx_ > 0) std::memcpy(ref_[c].data(), cur_[c].data(), (size_t)w_ * h_);
            std::memcpy(cur_[c].data(), v.pixels[c], (size_t)w_ * h_);
            in->img[c] = cur_[c].data();
            in->img_ref[c] = (idx_ > 0) ? ref_[c].data() : cur_[c].data();
            in->t_cam[c][0] = t_move_;
            in->t_cam[c][1] = 0.0f;
            in->t_cam[c][2] = 0.0f;
        }
        in->width = v.width;
        in->height = v.height;
        in->stride = v.width;
        in->frame_id = v.header.frame_id;
        in->timestamp_us = v.header.timestamp_us;
        in->frame_gap = 1;
        in->body_mask = nullptr;
        ++idx_;
        return NF_OK;
    }

    void SetMove(float m) { t_move_ = m; }

    const char *Name() const override { return "zmq"; }
    uint32_t Width() const override { return w_; }
    uint32_t Height() const override { return h_; }
    bool HasPose() const override { return true; }

private:
    void *ctx_ = nullptr;
    void *sock_ = nullptr;
    bool ok_ = false;
    uint32_t w_, h_;
    int tmo_;
    uint32_t idx_ = 0;
    float t_move_ = NF_B_MIN;
    std::vector<uint8_t> cur_[4], ref_[4];
};
#endif  // NF_HAS_ZMQ

}  // namespace

std::unique_ptr<InputSource> MakeSyntheticSource(uint32_t w, uint32_t h,
                                                 int n_frames, float move_mm,
                                                 bool forward) {
    return std::unique_ptr<InputSource>(
        new SyntheticSource(w, h, n_frames, move_mm, forward));
}

std::unique_ptr<InputSource> MakeFileSource(const std::string &path) {
    std::unique_ptr<FileSource> s(new FileSource(path));
    if (!s->ok()) return nullptr;
    return std::unique_ptr<InputSource>(s.release());
}

#ifdef NF_HAS_ZMQ
std::unique_ptr<InputSource> MakeZmqSource(const std::string &endpoint,
                                           uint32_t w, uint32_t h,
                                           int timeout_ms) {
    std::unique_ptr<ZmqSource> s(new ZmqSource(endpoint, w, h, timeout_ms));
    if (!s->ok()) return nullptr;
    return std::unique_ptr<InputSource>(s.release());
}
#endif

}  // namespace nf
