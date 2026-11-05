/**
 * @file scene_render.cpp
 * @brief 解析场景渲染。每个像素沿真实相机射线求交，写入灰度与真值 Z。
 */
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_vcam_bridge.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

extern "C" {

NFStatus nf_render_frame_alloc(NFRenderFrame *f, nf_u32 w, nf_u32 h) {
    if (!f || w == 0 || h == 0) return NF_E_PARAM_NULL;
    f->img = (nf_u8 *)std::calloc((size_t)w * h, 1);
    f->depth = (nf_f32 *)std::calloc((size_t)w * h, sizeof(nf_f32));
    if (!f->img || !f->depth) {
        std::free(f->img); std::free(f->depth);
        f->img = nullptr; f->depth = nullptr;
        return NF_E_ENGINE_OOM;
    }
    f->width = w;
    f->height = h;
    return NF_OK;
}

void nf_render_frame_free(NFRenderFrame *f) {
    if (!f) return;
    std::free(f->img);
    std::free(f->depth);
    f->img = nullptr;
    f->depth = nullptr;
}

NFStatus nf_render_frame(const NFScene *scene, const NFCameraModel *cam,
                         const NFPose *pose, NFRenderFrame *frame) {
    if (!scene || !cam || !pose || !frame || !frame->img || !frame->depth)
        return NF_E_PARAM_NULL;
    if (cam->version != NF_CAMERA_MODEL_VERSION) return NF_E_PARAM_VERSION;
    if (pose->version != NF_POSE_VERSION) return NF_E_PARAM_VERSION;

    const nf_u32 w = frame->width, h = frame->height;

    /* 无命中像素的灰度: 用固定暗值。**不能随机** ——
     * 随机噪声会让匹配器把噪声当纹理，产出看似正常实则全错的视差。 */
    const nf_u8 kNoHit = 20;

    for (nf_u32 y = 0; y < h; ++y) {
        for (nf_u32 x = 0; x < w; ++x) {
            const nf_u32 idx = y * w + x;

            /* 像素 -> 相机系单位射线（走 vcam，原生实现已删除） */
            float dc[3];
            nf_u8 conv = 0;
            /* 像素中心在 (x+0.5, y+0.5)，与标定一致 */
            NFStatus st_r = nf_vcam_unproject(cam, (nf_f32)x + 0.5f,
                                              (nf_f32)y + 0.5f, dc,
                                              nullptr, &conv);
            if (st_r != NF_OK || !conv) {
                frame->img[idx] = kNoHit;
                frame->depth[idx] = 0.0f;
                continue;
            }

            /* 相机系 -> 车系: v_V = R · v_C */
            float dw[3];
            for (int k = 0; k < 3; ++k) {
                dw[k] = pose->R[k * 3 + 0] * dc[0]
                      + pose->R[k * 3 + 1] * dc[1]
                      + pose->R[k * 3 + 2] * dc[2];
            }

            float t_hit = 0.0f, P[3];
            nf_u32 plane = 0;
            NFStatus st = nf_scene_raycast(scene, pose->t, dw, &t_hit, P, &plane);
            if (st != NF_OK) {
                frame->img[idx] = kNoHit;
                frame->depth[idx] = 0.0f;
                continue;
            }

            frame->img[idx] = nf_scene_shade(scene, P, plane);
            /* 真值存**沿相机光轴的距离** Z = 交点相对光心的相机系 z。
             * 与 pipeline 输出的点云 z 同义（点云也是相机/视口 z）。 */
            const float rel[3] = { P[0] - pose->t[0],
                                   P[1] - pose->t[1],
                                   P[2] - pose->t[2] };
            const float zc = pose->R[0 * 3 + 2] * rel[0]
                           + pose->R[1 * 3 + 2] * rel[1]
                           + pose->R[2 * 3 + 2] * rel[2];
            frame->depth[idx] = zc > 0.0f ? zc : 0.0f;
        }
    }
    return NF_OK;
}

} /* extern "C" */
