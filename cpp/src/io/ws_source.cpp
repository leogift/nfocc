/**
 * @file ws_source.cpp
 * @brief hanqtech 真实数据输入源：读 PNG，配标定，算 ego 位移。
 *
 * ## 数据布局（已实测）
 *   rgb_images/00164_FV.png ...           当前帧
 *   previous_images/00164_FV_prev.png ... 上一帧
 *   calibration/calibration/00164_FV.json
 *   vehicle_info/rgb_images/00164_FV.json ego
 *   vehicle_info/previous_images/00164_FV_prev.json
 *
 * **序号不是同步 ID**：每路相机独立递增，FV=164, MVL=165, MVR=166, RV=167。
 *
 * ## 位移从哪来
 * ego JSON 只有 speed/yawRate/steering/distance，没有位姿。
 * 两帧之间的平移用 `ego_distance`（本帧行驶距离, m），方向按 gear 取正负，
 * 车体系下即 (distance, 0, 0)——行驶方向。
 *
 * **这是简化**：真实位姿还含横摆。本数据集的 yawRate≈0.43°/s、
 * 帧间隔未知，横摆位移极小，先按纯纵向处理，并在日志里说明。
 */
#include "nfocc/util/input_source.h"
#include "nfocc/hanqtech_loader.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace nf {

namespace {

}  // namespace

class WSSource : public InputSource {
public:
    WSSource(const char *base, uint32_t frame_id, int n_frames)
        : base_(base), start_(frame_id), n_(n_frames) {}

    NFStatus Init(NFWSCalib *out_calib) {
        char dir[512];
        std::snprintf(dir, sizeof(dir), "%s/calibration/calibration", base_);
        const NFStatus st = nf_ws_load_frame(dir, start_, &calib_);
        if (st != NF_OK) return st;
        for (int c = 0; c < 4; ++c) { cur_[c] = Png{}; ref_[c] = Png{}; }
        if (out_calib) *out_calib = calib_;
        return NF_OK;
    }

    NFStatus ReadFrame(NFFrameInput *in, bool *finished) override {
        if (idx_ >= n_) { *finished = true; return NF_E_BASELINE_IDLE; }
        *finished = false;

        const uint32_t fid = start_ + (uint32_t)idx_ * 4u;   /* 每帧 4 路 */
        static const char *kNames[4] = { "FV", "MVL", "MVR", "RV" };
        char p[512];

        /* 读四路当前帧 + 上一帧 */
        for (int c = 0; c < 4; ++c) {
            std::snprintf(p, sizeof(p), "%s/rgb_images/%05u_%s.png",
                          base_, fid + (uint32_t)c, kNames[c]);
            if (!png_read_gray(p, &cur_[c])) {
                nf_log_msg(NF_LOG_ERROR, "读图失败: %s", p);
                return NF_E_IO;
            }
            std::snprintf(p, sizeof(p), "%s/previous_images/%05u_%s_prev.png",
                          base_, fid + (uint32_t)c, kNames[c]);
            if (!png_read_gray(p, &ref_[c])) {
                nf_log_msg(NF_LOG_ERROR, "读参考图失败: %s", p);
                return NF_E_IO;
            }
            if (cur_[c].w != ref_[c].w || cur_[c].h != ref_[c].h) return NF_E_IO;
        }

        /* ego: 取 FV 那一路的距离 */
        std::snprintf(p, sizeof(p), "%s/vehicle_info/rgb_images/%05u_FV.json",
                      base_, fid);
        NFWSEgo ego;
        if (nf_ws_load_ego(p, &ego) != NF_OK) return NF_E_IO;
        /* 位移: 车体系纵向。gear=R 时为负(倒车) */
        const float dist_mm = ego.distance_m * 1000.0f;
        const float sign = (ego.gear[0] == 'R') ? -1.0f : 1.0f;
        const float move = dist_mm * sign;

        /* hanqtech 的 cam_id 排布: 0=FV 1=RV 2=MVL 3=MVR,
         * 与 pipeline 的 0=FV 1=RV 2=ML 3=MR 一致。 */
        for (int c = 0; c < 4; ++c) {
            in->img[c]     = cur_[c].data.data();
            in->img_ref[c] = ref_[c].data.data();
            /*
             * **t_cam 用视口系（SPEC-CONVENTION §2）**。
             *
             * 行驶方向是车体 +X。视口基向量（SPEC-CONVENTION §2）：
             *   forward = (cos yaw, sin yaw, 0)   <- 视口 z 轴
             *   right   = (sin yaw,-cos yaw, 0)   <- 视口 x 轴
             * 车体位移 d·(1,0,0) 投影到视口系：
             *   沿视口 z = d·cos(yaw)
             *   沿视口 x = d·sin(yaw)
             *
             * 历史 bug：原来写成 `t_S[0] = d·cos(yaw)`、`t_S[2] = 0`，
             * 把**前进位移放到了视口横向**。直行时正确值应是
             * t_S[2] = d（yaw=0），原式给 0 —— 基线为零，视差恒 0；
             * 而 t_S[0] 却拿到 d，门控按"横向位移"判成侧行。
             *
             * 实测症状（真实 hanqtech 数据）：三角化出**负深度**
             * （z 最小 -497mm）、点云水平距 3.7-8.9m 全在车外、
             * 16k 插入点最后只剩 55 个。
             */
            const float yaw_c = yaw_[c] * (float)(3.14159265358979323846 / 180.0);
            const float cy = std::cos(yaw_c), sy = std::sin(yaw_c);
            in->t_cam[c][0] = move * sy;    /* 视口 x = right 分量 */
            in->t_cam[c][1] = 0.0f;
            in->t_cam[c][2] = move * cy;    /* 视口 z = forward 分量 */
        }
        in->width  = (nf_u32)cur_[0].w;
        in->height = (nf_u32)cur_[0].h;
        in->stride = (nf_u32)cur_[0].w;
        in->frame_id = (nf_u32)idx_;
        in->timestamp_us = ego.timestamp_us;
        in->frame_gap = 1;
        in->body_mask = nullptr;

        last_ego_ = ego;
        last_move_mm_ = move;
        ++idx_;
        return NF_OK;
    }

    const char *Name() const override { return "hanqtech"; }
    uint32_t Width()  const override { return cur_[0].w ? (uint32_t)cur_[0].w : 0; }
    uint32_t Height() const override { return cur_[0].h ? (uint32_t)cur_[0].h : 0; }
    bool HasPose() const override { return true; }

    const NFWSCalib &Calib() const { return calib_; }
    float LastMoveMm() const { return last_move_mm_; }

private:
    const char *base_;
    uint32_t start_;
    int n_;
    int idx_ = 0;
    NFWSCalib calib_;
    Png cur_[4], ref_[4];
    NFWSEgo last_ego_{};
    float yaw_[4] = { 0.0f, 180.0f, 90.0f, -90.0f };  /* 虚拟视口 yaw, 与 cfg 对应 */
    float last_move_mm_ = 0.0f;
};

std::unique_ptr<InputSource> MakehanqtechSource(const char *base,
                                                 uint32_t frame_id,
                                                 int n_frames,
                                                 void *out_calib) {
    if (!base) return nullptr;
    WSSource *s = new WSSource(base, frame_id, n_frames);
    NFWSCalib cal;
    if (s->Init(&cal) != NF_OK) { delete s; return nullptr; }
    if (out_calib) *(NFWSCalib *)out_calib = cal;
    return std::unique_ptr<InputSource>(s);
}

}  // namespace nf
