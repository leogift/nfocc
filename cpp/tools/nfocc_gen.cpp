/**
 * @file nfocc_gen.cpp
 * @brief 生成上游测试数据：合成 4 路图像 + ego motion，走 ZMQ 或写文件。
 *
 * 用途：
 *   - 端到端联调（模拟上游 PUB）
 *   - SIL 回归的输入录制
 *
 * 用法：
 *   nfocc_gen --zmq tcp://*:5555 --frames 20 --move 300
 *   nfocc_gen --file run.nfp  --frames 20 --move 300
 *
 * `--move` 是每帧位移(mm)。5 Hz 下 200mm = 3.6 km/h, 300mm = 5.4 km/h。
 */
#include "nfocc/nfocc_io.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifdef NF_HAS_ZMQ
#include <zmq.h>
#endif

static void synth_image(nf_u8 *buf, nf_u32 w, nf_u32 h, float phase) {
    /* 确定性纹理: 同一 3D 点在两帧必须得同一灰度, 否则匹配器"成功返回但全错" */
    for (nf_u32 y = 0; y < h; ++y) {
        for (nf_u32 x = 0; x < w; ++x) {
            double v = 128.0
                     + 60.0 * std::sin(x * 0.031 + phase)
                     + 50.0 * std::sin(y * 0.024 - phase)
                     + 40.0 * std::sin((x + y) * 0.017);
            v = v < 0 ? 0 : (v > 255 ? 255 : v);
            buf[(size_t)y * w + x] = (nf_u8)(v + 0.5);
        }
    }
}

int main(int argc, char **argv) {
    const char *endpoint = nullptr;
    const char *file = nullptr;
    int frames = 10;
    double move_mm = NF_B_MIN;     /* 默认最小基线 */

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--zmq") && i + 1 < argc) endpoint = argv[++i];
        else if (!std::strcmp(argv[i], "--file") && i + 1 < argc) file = argv[++i];
        else if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--move") && i + 1 < argc) move_mm = std::atof(argv[++i]);
        else { std::printf("未知参数: %s\n", argv[i]); return 2; }
    }
    if (!endpoint && !file) {
        std::printf("用法: nfocc_gen --zmq tcp://*:5555 | --file run.nfp [--frames N] [--move mm]\n");
        return 2;
    }

    const nf_u32 W = NF_IMG_W, H = NF_IMG_H;
    const size_t bytes = (size_t)W * H;

    std::vector<nf_u8> head(sizeof(NFUpHeader));
    std::vector<std::vector<nf_u8>> frames_buf(4);

    /* 文件模式: 写 .nfp（复用 file_io 的容器格式） */
    FILE *fp = nullptr;
    if (file) {
        fp = std::fopen(file, "wb");
        if (!fp) { std::printf("无法写 %s\n", file); return 1; }
        const nf_u32 magic = 0x4E46504Bu;  /* NFPK */
        const nf_u32 ver = 1;
        std::fwrite(&magic, 4, 1, fp);
        std::fwrite(&ver, 4, 1, fp);
        std::fwrite(&W, 4, 1, fp);
        std::fwrite(&H, 4, 1, fp);
        const nf_u32 n = (nf_u32)frames;
        std::fwrite(&n, 4, 1, fp);
    }

#ifdef NF_HAS_ZMQ
    void *zctx = nullptr, *pub = nullptr;
    if (endpoint) {
        zctx = zmq_ctx_new();
        pub = zmq_socket(zctx, ZMQ_PUB);
        int hwm = 4;
        zmq_setsockopt(pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));
        if (zmq_bind(pub, endpoint) != 0) {
            std::printf("bind 失败: %s\n", endpoint);
            return 1;
        }
        std::printf("PUB 已绑定 %s\n", endpoint);
    }
#endif

    /* 位移->相位: 图案横移 = phase/0.031 像素。目标视差 d 需要 phase = d*0.031。
       这里按"目标深度 3 m"反推视差, 再换算相位, 保证点落在网格内。 */
    const double f = NF_VF_VIEW;
    const double target_z = 3000.0;          /* mm */
    const double disp_px = f * move_mm / target_z;
    const double phase_step = disp_px * 0.031;
    std::printf("每帧位移 %.0f mm -> 视差 %.2f px @%.1fm -> 相位步长 %.4f\n",
                move_mm, disp_px, target_z / 1000.0, phase_step);

    for (int fi = 0; fi < frames; ++fi) {
        const nf_u64 ts = 1000000ull + (nf_u64)fi * 200000ull;   /* 5 Hz = 200ms */

        NFUpHeader *hd = (NFUpHeader *)head.data();
        std::memset(hd, 0, sizeof(NFUpHeader));
        hd->magic = NF_UP_MAGIC;
        hd->version = NF_UP_VERSION;
        hd->frame_id = (nf_u32)fi;
        hd->timestamp_us = ts;
        hd->n_cameras = 4;
        hd->ego.version = 1;
        hd->ego.timestamp_us = ts;
        hd->ego.quat[0] = 1.0f;
        hd->ego.pos_mm[0] = (nf_f32)(move_mm * fi);
        hd->ego.vel_mmps[0] = (nf_f32)(move_mm * 5.0);   /* mm/s @5Hz */
        hd->ego.source = 0;
        hd->ego.quality = 95;

        for (int c = 0; c < 4; ++c) {
            const size_t fb = sizeof(NFUpImageHeader) + bytes;
            frames_buf[c].resize(fb);
            NFUpImageHeader *ih = (NFUpImageHeader *)frames_buf[c].data();
            std::memset(ih, 0, sizeof(NFUpImageHeader));
            ih->cam_id = (nf_u8)c;
            ih->format = 0;
            ih->width = W;
            ih->height = H;
            ih->stride = W;
            ih->timestamp_us = ts + (nf_u64)c * 500ull;   /* 极差 1.5ms */
            ih->frame_id = (nf_u32)fi;
            ih->data_bytes = (nf_u32)bytes;
            synth_image(frames_buf[c].data() + sizeof(NFUpImageHeader),
                        W, H, (float)(fi * phase_step + c * 0.5));
        }

        if (fp) {
            /* .nfp 帧体: frame_id + ts + t_cam[4][3] + 4 路像素 */
            std::fwrite(&fi, 4, 1, fp);
            std::fwrite(&ts, 8, 1, fp);
            nf_f32 t_cam[4][3];
            for (int c = 0; c < 4; ++c)
                for (int k = 0; k < 3; ++k)
                    t_cam[c][k] = (k == 0) ? (nf_f32)move_mm : 0.0f;
            std::fwrite(t_cam, 4, 12, fp);
            for (int c = 0; c < 4; ++c)
                std::fwrite(frames_buf[c].data() + sizeof(NFUpImageHeader),
                            1, bytes, fp);
        }

#ifdef NF_HAS_ZMQ
        if (pub) {
            /* multipart: 段 0 = 头, 段 1..4 = 图像 */
            zmq_send(pub, head.data(), head.size(), ZMQ_SNDMORE);
            for (int c = 0; c < 4; ++c) {
                const int more = (c < 3) ? ZMQ_SNDMORE : 0;
                zmq_send(pub, frames_buf[c].data(), frames_buf[c].size(), more);
            }
        }
#endif
        if ((fi % 5) == 0) std::printf("  发帧 %d\r", fi);
    }
    std::printf("\n共 %d 帧完成\n", frames);

    if (fp) std::fclose(fp);
#ifdef NF_HAS_ZMQ
    if (pub) { zmq_close(pub); zmq_ctx_destroy(zctx); }
#endif
    return 0;
}
