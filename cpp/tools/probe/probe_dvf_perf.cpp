#include <cstdio>
#include <cstring>
#include <vector>
#include <chrono>
#include "nfocc/nfocc_voxel_filter.h"
int main(){
    NFVoxelFilterConfig c; nf_voxel_filter_default_config(&c);
    NFVoxelFilterCtx *ctx=nullptr;
    if(nf_voxel_filter_create(&ctx,&c)!=NF_OK){printf("create fail\n");return 1;}
    nf_u32 nc=0; nf_voxel_filter_grid_size(ctx,nullptr,nullptr,&nc);
    std::vector<nf_u32> zm(nc,0), zi(nc+1,0);
    std::vector<NFDownstreamPoint> out(NF_DVF_HARD_CAP_MAX_POINTS);
    /* 15 万点铺满 ROI */
    std::vector<NFVoxelEntry> pts(150000);
    for(size_t i=0;i<pts.size();i++){
        std::memset(&pts[i],0,sizeof(NFVoxelEntry));
        pts[i].cx = -5000.f + (float)(i%300)*50.f;
        pts[i].cy = -6000.f + (float)((i/300)%200)*60.f;
        pts[i].cz = -1000.f + (float)((i/60000)%35)*100.f;
        pts[i].n_eff=4.f; pts[i].source_mask=1; pts[i].sigma_cm=20; pts[i].used=1;
    }
    nf_u32 npt=(nf_u32)pts.size();
    NFVoxelFilterStats st;
    /* 预热 */
    nf_voxel_filter_compress(ctx,pts.data(),npt,zm.data(),zi.data(),out.data(),&st);
    double best=1e9;
    for(int r=0;r<5;r++){
        auto t0=std::chrono::steady_clock::now();
        nf_voxel_filter_compress(ctx,pts.data(),npt,zm.data(),zi.data(),out.data(),&st);
        auto t1=std::chrono::steady_clock::now();
        double ms=std::chrono::duration<double,std::milli>(t1-t0).count();
        if(ms<best) best=ms;
    }
    printf("15万点: %.2f ms, 输入 %u -> 输出 %u, 截断=%u\n",
           best, st.n_input, st.n_output, st.truncated);
    nf_voxel_filter_destroy(ctx);
    return 0;
}
