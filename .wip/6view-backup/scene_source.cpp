/**
 * @file scene_source.cpp
 * @brief 真值场景输入源：渲染两帧 + 附带参考帧真值。
 *
 * 与 SyntheticSource 的区别（这是 P1/P2 的核心）：
 *   SyntheticSource: 纹理横移，无几何、无真值 -> 只能测中间量
 *   SceneSource:     解析场景渲染，**每像素真值可闭式算出** -> 能测端到端深度误差
 */
#include "nfocc/util/input_source.h"
#include "nfocc/nfocc_scene.h"
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_default.h"

#include <cstring>
#include <vector>

namespace nf {

namespace {

class SceneSource : public InputSource {
public:
    SceneSource(const NFScene &scene, const NFCameraModel *cams,
                const NFPose *poses, uint32_t w, uint32_t h,
                int n_frames, float move_mm, bool forward)
        : scene_(scene), w_(w), h_(h), n_(n_frames),
          move_(move_mm), fwd_(forward) {
        for (int c = 0; c < 4; ++c) {
            cams_[c] = cams[c];
            poses_[c] = poses[c];
        }
    }

    NFStatus ReadFrame(NFFrameInput *in, bool *finished) override {
        if (idx_ >= n_) { *finished = true; return NF_E_BASELINE_IDLE; }
        *finished = false;
        if (!ready_) {
            for (int c = 0; c < 4; ++c) {
                if (nf_render_frame_alloc(&cur_[c], w_, h_) != NF_OK ||
                    nf_render_frame_alloc(&ref_[c], w_, h_) != NF_OK)
                    return NF_E_ENGINE_OOM;
            }
            /* 参考帧: 相机后退 move（等价于当前帧前进 move） */
            for (int c = 0; c < 4; ++c) {
                NFPose p = poses_[c];
                if (fwd_) p.t[0] -= move_;      /* 参考帧更靠后 */
                else     p.t[1] -= move_;
                nf_render_frame(&scene_, &cams_[c], &p, &ref_[c]);
            }
            ready_ = true;
        }

        for (int c = 0; c < 4; ++c) {
            NFPose p = poses_[c];
            in->img[c] = cur_[c].img;
            in->img_ref[c] = ref_[c].img;
            /* t_cam 语义: 相机相对参考帧的位移(视口/相机系) */
            /* t_cam 用视口系，见 SPEC-CONVENTION §2 */
            in->t_cam[c][0] = fwd_ ? 0.0f : move_;
            in->t_cam[c][1] = 0.0f;
            in->t_cam[c][2] = fwd_ ? move_ : 0.0f;
            (void)p;
        }
        for (int c = 0; c < 4; ++c)
            nf_render_frame(&scene_, &cams_[c], &poses_[c], &cur_[c]);

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

    const char *Name() const override { return "scene"; }
    uint32_t Width() const override { return w_; }
    uint32_t Height() const override { return h_; }
    bool HasPose() const override { return true; }

    const float *TruthDepth(int c) const override {
        return (c >= 0 && c < 4) ? cur_[c].depth : nullptr;
    }
    const uint8_t *TruthImage(int c) const override {
        return (c >= 0 && c < 4) ? cur_[c].img : nullptr;
    }

    ~SceneSource() override {
        for (int c = 0; c < 4; ++c) {
            nf_render_frame_free(&cur_[c]);
            nf_render_frame_free(&ref_[c]);
        }
    }

private:
    NFScene scene_;
    NFCameraModel cams_[4];
    NFPose poses_[4];
    NFRenderFrame cur_[4], ref_[4];
    uint32_t w_, h_;
    int n_;
    float move_;
    bool fwd_;
    int idx_ = 0;
    bool ready_ = false;
};

}  // namespace

std::unique_ptr<InputSource> MakeSceneSource(const void *scene,
                                             const void *cams, const void *poses,
                                             uint32_t w, uint32_t h,
                                             int n_frames, float move_mm,
                                             bool forward) {
    const NFScene *s = (const NFScene *)scene;
    const NFCameraModel *c = (const NFCameraModel *)cams;
    const NFPose *p = (const NFPose *)poses;
    if (!s || !c || !p) return nullptr;
    return std::unique_ptr<InputSource>(
        new SceneSource(*s, c, p, w, h, n_frames, move_mm, forward));
}



}  // namespace nf
