/**
 * @file input_source.h
 * @brief 上游输入端抽象：合成 / 文件回放 / ZeroMQ。
 *
 * ## 为什么要抽象
 * SIL 调试用文件、端到端联调用 ZMQ、单元/系统测试用合成。三者接口一致，
 * 上层 pipeline 不必知道自己吃的从哪来 —— 否则每加一种源就要改主循环。
 *
 * ## 冻结语义（SPEC-DOWNSTREAM §1）
 * 源"读完/断流"与"取到冻结帧"是两件事：
 *   - 源结束 -> ReadFrame 返回 NF_E_BASELINE_IDLE 且 finished=true
 *   - 取到帧但基线不足 -> 由 pipeline 判冻结，源照常返回 NF_OK
 */
#ifndef NFOCC_INPUT_SOURCE_H
#define NFOCC_INPUT_SOURCE_H

#include "nfocc/nfocc_pipeline.h"
#include "nfocc/nfocc_io.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nf {

/** 8-bit 灰度图（PNG 解码结果） */
struct Png {
    int w = 0, h = 0, channels = 0;
    std::vector<unsigned char> data;
};

/** 读 PNG 并转 8-bit 灰度。只支持非隔行 8-bit。 */
bool png_read_gray(const char *path, Png *out);


class InputSource {
public:
    virtual ~InputSource() = default;
    /** 取下一帧。返回 NF_OK / NF_E_BASELINE_IDLE(结束或超时) / 其他错误 */
    virtual NFStatus ReadFrame(NFFrameInput *in, bool *finished) = 0;
    virtual const char *Name() const = 0;
    virtual uint32_t Width() const = 0;
    virtual uint32_t Height() const = 0;
    /** 上一帧的位姿(用于算 t_cam)；无位姿源时返回 false */
    virtual bool HasPose() const = 0;

    /**
     * 当前帧的**真值深度图**(沿光轴, mm; 0=未命中)。
     * 只有真值场景源提供；其余源返回 nullptr。
     * 用虚函数而不是 dynamic_cast —— 本项目编译时 -fno-rtti。
     */
    virtual const float *TruthDepth(int /*cam*/) const { return nullptr; }
    /** 当前帧渲染图(灰度)。真值场景源提供, 其余返回 nullptr。 */
    virtual const uint8_t *TruthImage(int /*cam*/) const { return nullptr; }
};

/** 合成源：纹理随相位漂移，产生确定视差 */
std::unique_ptr<InputSource> MakeSyntheticSource(uint32_t w, uint32_t h,
                                                 int n_frames, float move_mm,
                                                 bool forward = false);

/** 文件回放源：读 .nfp（SPEC-UPSTREAM §5.2） */
std::unique_ptr<InputSource> MakeFileSource(const std::string &path);

/**
 * 解析真值场景源：渲染 + 逐像素真值深度（P1/P2 用）。
 * @param scene  NFScene*, cams/poses 各 4 个
 */
std::unique_ptr<InputSource> MakeSceneSource(const void *scene,
                                             const void *cams,
                                             const void *poses,
                                             uint32_t w, uint32_t h,
                                             int n_frames, float move_mm,
                                             bool forward);

/**
 * hanqtech 真实数据源。
 *
 * @param base      数据集根（含 rgb_images/、previous_images/ 等）
 * @param frame_id  **FV 那一路的帧号**；其余三路为 +1/+2/+3
 *                  （实测：00164_FV, 00165_MVL, 00166_MVR, 00167_RV）
 * @param n_frames  读多少帧
 * @param out_calib 输出四路真实标定（NFWSCalib*，可为 nullptr）
 */
std::unique_ptr<InputSource> MakehanqtechSource(const char *base,
                                                 uint32_t frame_id,
                                                 int n_frames,
                                                 void *out_calib);

#ifdef NF_HAS_ZMQ
/** ZeroMQ 源：SUB 连上游 PUB（SPEC-UPSTREAM §5.1） */
std::unique_ptr<InputSource> MakeZmqSource(const std::string &endpoint,
                                           uint32_t w, uint32_t h,
                                           int timeout_ms);
#endif

}  // namespace nf

#endif  // NFOCC_INPUT_SOURCE_H
