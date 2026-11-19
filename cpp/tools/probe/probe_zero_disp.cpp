#include <cstdio>
#include <cstring>
#include <vector>
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_default.h"

/* 引擎层用专属符号名，头文件里的通用名没实现（与 viz 一致） */
extern "C" NFStatus nf_sgbm_engine_init(NFEngineCtx **ctx, const NFEngineConfig *cfg);
extern "C" NFStatus nf_sgbm_engine_run(NFEngineCtx *ctx, const NFImageView *l,
                                       const NFImageView *r,
                                       const NFDisparityRange *rg,
                                       NFDisparityMap *out);
int main(){
    const int W=512,H=256;
    std::vector<unsigned char> a((size_t)W*H);
    for(int y=0;y<H;y++)for(int x=0;x<W;x++){
        unsigned h=(unsigned)x*73856093u ^ (unsigned)y*19349663u;
        h=(h^(h>>13))*1274126177u;
        a[(size_t)y*W+x]=(unsigned char)(128+((int)(h&0x3F)-31));
    }
    NFEngineConfig cfg; std::memset(&cfg,0,sizeof(cfg));
    cfg.version=NF_ENGINE_CONFIG_VERSION; cfg.kind=NF_ENGINE_SGBM;
    cfg.max_disp_px=128; cfg.downscale=1; cfg.block_size=5;
    cfg.uniqueness=1; cfg.speckle_window=0;
    NFEngineCtx *e=nullptr;
    if(nf_sgbm_engine_init(&e,&cfg)!=NF_OK){printf("init fail\n");return 1;}
    NFImageView v; std::memset(&v,0,sizeof(v));
    v.version=NF_IMAGE_VIEW_VERSION; v.data=a.data(); v.width=W; v.height=H; v.stride=W;
    NFDisparityRange r; std::memset(&r,0,sizeof(r));
    r.version=NF_DISPARITY_RANGE_VERSION; r.d_min=0; r.d_max=128; r.b_mm=278;
    std::vector<float> d((size_t)W*H,0);
    std::vector<unsigned char> val((size_t)W*H,0);
    std::vector<float> cf((size_t)W*H,0);
    NFDisparityMap dm; std::memset(&dm,0,sizeof(dm));
    dm.version=NF_DISPARITY_MAP_VERSION;
    dm.disp=d.data(); dm.valid=val.data(); dm.conf=cf.data();
    dm.width=W; dm.height=H; dm.stride_f=W; dm.stride_b=W;
    /*
     * **零位移测不出问题**：SGBM 用 raw>0 判有效（engine_sgbm.cpp），
     * 真实视差 0 会被判失败。所以这里同时验"平移 4px"的情形：
     * 右图左移 4 像素后，左图每个像素应匹配到视差 4。
     */
    int shift=4;
    std::vector<unsigned char> b((size_t)W*H);
    for(int y=0;y<H;y++)
        for(int x=0;x<W;x++){
            int sx=x+shift;             /* 右图取右移后的采样 -> 视差=+shift */
            b[(size_t)y*W+x] = (sx<W) ? a[(size_t)y*W+sx] : 128;
        }
    NFImageView vb; std::memset(&vb,0,sizeof(vb));
    vb.version=NF_IMAGE_VIEW_VERSION; vb.data=b.data(); vb.width=W; vb.height=H; vb.stride=W;
    int rc=nf_sgbm_engine_run(e,&v,&vb,&r,&dm);
    long nv=0; double sum=0; float mn=1e9f,mx=-1e9f;
    for(size_t i=0;i<d.size();i++) if(val[i]){nv++;sum+=d[i]; if(d[i]<mn)mn=d[i]; if(d[i]>mx)mx=d[i];}
    printf("人为平移 %d px: rc=%d 有效=%ld/%d\n",shift,rc,nv,W*H);
    if(nv) printf("  视差 min=%.3f max=%.3f 均值=%.4f (应=%d)\n",(double)mn,(double)mx,sum/nv,shift);

    return 0;
}
