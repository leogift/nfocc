#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include "nfocc/nfocc_scene.h"
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_vcam_bridge.h"
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_default.h"
extern "C" NFStatus nf_sgbm_engine_init(NFEngineCtx**, const NFEngineConfig*);
extern "C" NFStatus nf_sgbm_engine_run(NFEngineCtx*, const NFImageView*,
                                       const NFImageView*, const NFDisparityRange*,
                                       NFDisparityMap*);
int main(){
    NFScene sc; nf_scene_default(&sc);
    NFViewParams vp;
    nf_view_params_derive_fov(&vp, NF_V_VIEW_W, NF_V_VIEW_HFOV_DEG, NF_V_VIEW_VFOV_DEG);
    nf_scene_set_tex_for_focal(&sc, vp.f, 20.0f, 2000.0f);
    printf("视口 %ux%u f=%.2f\n", vp.width, vp.height, (double)vp.f);

    NFCameraModel cam; std::memset(&cam,0,sizeof(cam));
    cam.version=NF_CAMERA_MODEL_VERSION; cam.kind=NF_CAM_MODEL_KB; cam.cam_id=0;
    cam.f=NF_INF_F; cam.cx=NF_INF_CX; cam.cy=NF_INF_CY;
    cam.r_valid=NF_INF_R_VALID; cam.max_theta=NF_INF_MAX_THETA;
    NFPose pose; std::memset(&pose,0,sizeof(pose)); pose.version=NF_POSE_VERSION;
    float ax[3][3]={{0,-1,0},{0,0,-1},{1,0,0}};
    for(int k=0;k<3;k++)for(int r=0;r<3;r++) pose.R[r*3+k]=ax[k][r];
    pose.t[0]=NF_VEH_REAR_TO_FRONT; pose.t[2]=NF_POS_CAM_Z;
    float b=278.0f;
    NFPose pr=pose; pr.t[1]-=b;   /* 侧行 */

    NFRenderFrame fc,fr;
    nf_render_frame_alloc(&fc,NF_IMG_W,NF_IMG_H);
    nf_render_frame_alloc(&fr,NF_IMG_W,NF_IMG_H);
    nf_render_frame(&sc,&cam,&pose,&fc);
    nf_render_frame(&sc,&cam,&pr,&fr);

    NFViewLut lut; std::memset(&lut,0,sizeof(lut));
    nf_lut_alloc(&lut, vp.width, vp.height);
    nf_lut_build(&lut,&cam,&pose,&vp,nullptr,nullptr);

    size_t np=(size_t)vp.width*vp.height;
    std::vector<nf_u8> li(np),ri(np),lv(np),rv(np);
    /* pipeline: 参考帧作左图 */
    nf_view_warp(&lut,fr.img,NF_IMG_W,NF_IMG_H,NF_IMG_W,li.data(),lv.data());
    nf_view_warp(&lut,fc.img,NF_IMG_W,NF_IMG_H,NF_IMG_W,ri.data(),rv.data());

    NFEngineConfig ec; std::memset(&ec,0,sizeof(ec));
    ec.version=NF_ENGINE_CONFIG_VERSION; ec.kind=NF_ENGINE_SGBM;
    ec.downscale=1; ec.block_size=NF_E_BLOCK; ec.uniqueness=NF_E_UNIQUENESS;
    ec.speckle_window=NF_E_SPECKLE_WINDOW; ec.max_disp_px=NF_E_MAX_DISP;
    NFEngineCtx *eng=nullptr;
    if(nf_sgbm_engine_init(&eng,&ec)!=NF_OK){printf("engine fail\n");return 1;}

    NFImageView L{},R{};
    L.version=R.version=NF_IMAGE_VIEW_VERSION;
    L.data=li.data(); L.width=vp.width; L.height=vp.height; L.stride=vp.width;
    R=L; R.data=ri.data();
    NFDisparityRange dr; std::memset(&dr,0,sizeof(dr));
    dr.version=NF_DISPARITY_RANGE_VERSION; dr.d_min=0;
    dr.d_max=256.0f; dr.b_mm=b;
    std::vector<float> dm_(np,0), cf(np,0); std::vector<nf_u8> dv(np,0);
    NFDisparityMap dm; std::memset(&dm,0,sizeof(dm));
    dm.version=NF_DISPARITY_MAP_VERSION;
    dm.disp=dm_.data(); dm.valid=dv.data(); dm.conf=cf.data();
    dm.width=vp.width; dm.height=vp.height;
    dm.stride_f=(nf_i32)vp.width; dm.stride_b=(nf_i32)vp.width;
    if(nf_sgbm_engine_run(eng,&L,&R,&dr,&dm)!=NF_OK){printf("run fail\n");return 1;}

    /* 逐像素: 用参考帧深度算 d_true, 与 d_meas 比 */
    std::vector<double> err;
    long n=0;
    for(nf_u32 y=0;y<vp.height;y+=3){
        for(nf_u32 x=0;x<vp.width;x+=3){
            size_t i=(size_t)y*vp.width+x;
            if(!dv[i]||!lv[i]) continue;
            int su=(int)lut.u_q4[i]>>4, sv=(int)lut.v_q4[i]>>4;
            if(su<0||sv<0||su>=NF_IMG_W||sv>=NF_IMG_H) continue;
            float zc=fr.depth[(size_t)sv*NF_IMG_W+su];
            if(zc<=0) continue;
            /*
             * fr.depth 存的就是**沿相机光轴的深度**（见 scene_render.cpp
             * 第 85 行的注释），而视差公式 d = f*b/Z 里的 Z 正是它。
             * 早先这里写成 zv = zc / dc[2]（除一次射线 z 分量）是错的 ——
             * 等于把沿光轴深度又折算了一遍，真值整体偏小，
             * 表现为"误差中位 70px"这种夸张数字。
             */
            (void)0;
            float zv = zc;
            static int dbg = 0;
            if (dbg < 8) { printf("    [dbg] su=%d sv=%d zc=%.1f\n", su, sv, (double)zc); ++dbg; }
            if(zv<800||zv>6000) continue;
            double d_true=(double)vp.f*b/zv;
            err.push_back((double)dm_[i]-d_true);
            n++;
        }
    }
    if(err.empty()){printf("无样本\n");return 1;}
    std::sort(err.begin(),err.end());
    auto pct=[&](double q){return err[(size_t)(q*(err.size()-1))];};
    printf("n=%ld  d_err 中位=%.2f  P25=%.2f  P75=%.2f  P95=%.2f\n",
           n, pct(0.5), pct(0.25), pct(0.75), pct(0.95));
    double s=0; for(double e:err) s+=e;
    printf("均值=%.2f\n", s/err.size());
    return 0;
}
