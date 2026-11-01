/**
 * @file scene_render.h
 * @brief 按真实相机模型渲染解析场景，同时输出**逐像素真值深度**。
 *
 * 与 `SyntheticSource` 的根本区别：
 *   SyntheticSource: 纹理横移 -> 没有几何，没有真值
 *   本模块:          射线-平面求交 -> 每个像素的真值深度可闭式算出
 *
 * ## 两帧的生成
 * 相机在 t0 与 t1 两个位置各渲染一次。**同一物点两侧的灰度相同**
 * （`nf_scene_shade` 是 3D 坐标的确定性函数），所以匹配器配对的是真对应点。
 */
#ifndef NFOCC_SCENE_RENDER_H
#define NFOCC_SCENE_RENDER_H

#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_scene.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 渲染一帧 */
typedef struct {
    nf_u8  *img;         /**< [h*w] 灰度 */
    nf_f32 *depth;       /**< [h*w] 真值深度（沿光轴 z，mm）；0 = 未命中 */
    nf_u32  width, height;
} NFRenderFrame;

/**
 * 渲染一帧。
 *
 * @param scene    场景
 * @param cam      KB 相机模型
 * @param pose     相机位姿（R: v_V = R·v_C；t: 光心在车系）
 * @param frame    (in) img/depth 缓冲已分配; (out) 填充
 * @return NF_OK
 *
 * ## 真值深度的定义
 * 存的是**沿相机 z 轴的距离**（即 Z），不是射线距离。
 * 因为后续 pipeline 输出的点云也是 (x,y,z)，比对 Z 才同义。
 * 换算: Z = t_hit * d_cam_z，其中 d_cam_z 是射线方向的相机系 z 分量。
 */
NFStatus nf_render_frame(const NFScene *scene, const NFCameraModel *cam,
                         const NFPose *pose, NFRenderFrame *frame);

NFStatus nf_render_frame_alloc(NFRenderFrame *f, nf_u32 w, nf_u32 h);
void     nf_render_frame_free(NFRenderFrame *f);

#ifdef __cplusplus
}
#endif

#endif /* NFOCC_SCENE_RENDER_H */
