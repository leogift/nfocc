/**
 * @file nfocc_viz.cpp
 * @brief 真实数据的中间结果可视化：Y 通道 / 视口投影 / 对齐 / 视差 / Voxel BEV。
 *
 * ## 为什么要独立实现而不从 pipeline 取
 * pipeline 只对外给融合后的点云（NFFrameOutput.cloud），视口图与视差图
 * 是内部缓冲。要可视化它们有两条路：
 *   a) 给 pipeline 加调试导出接口 —— 会把调试逻辑塞进产品代码
 *   b) 本工具按相同顺序调用同一套库函数 —— 不改产品代码
 * 选 b。所有几何计算都走 nfocc_core / nfocc_engine_sgbm 的同一实现，
 * 不存在"可视化用一套、产品用另一套"的分叉。
 *
 * ## 输出
 *   viz/01_y_<cam>.png        Y 通道灰度原图
 *   viz/02_view_<cam>.png     视口投影（当前帧）
 *   viz/03_view_ref_<cam>.png 前一帧，已对齐到当前帧视口
 *   viz/04_disp_<cam>.png     视差图（彩色）
 *   viz/05_bev.png            Voxel 分层 BEV
 */
#include "nfocc/nfocc_camera.h"
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_voxel.h"
#include "nfocc/nfocc_geometry.h"
#include "nfocc/nfocc_default.h"
#include "nfocc/hanqtech_loader.h"
#include "nfocc/util/input_source.h"

/* 引擎符号已带 sgbm 后缀（防止与模型引擎同名覆盖），先声明再映射 */
extern "C" {
NFStatus nf_sgbm_engine_init(NFEngineCtx **ctx, const NFEngineConfig *cfg);
NFStatus nf_sgbm_engine_run(NFEngineCtx *ctx, const NFImageView *l,
                            const NFImageView *r,
                            const NFDisparityRange *range, NFDisparityMap *out);
NFStatus nf_sgbm_engine_deinit(NFEngineCtx *ctx);
}
#define nf_engine_init   nf_sgbm_engine_init
#define nf_engine_run    nf_sgbm_engine_run
#define nf_engine_deinit nf_sgbm_engine_deinit

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <sys/stat.h>

namespace {

/* 极简 PPM 写出（P6 彩色 / P5 灰度），再由外部转 PNG。
 * 不引 libpng —— 写 PPM 十几行就够。 */
void write_ppm_gray(const std::string &path, const unsigned char *d,
                    int w, int h) {
    std::FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    std::fprintf(f, "P5\n%d %d\n255\n", w, h);
    std::fwrite(d, 1, (size_t)w * h, f);
    std::fclose(f);
}

void write_ppm_rgb(const std::string &path, const unsigned char *d,
                   int w, int h) {
    std::FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    std::fwrite(d, 1, (size_t)w * h * 3, f);
    std::fclose(f);
}

/* 视差 -> 彩色（近红远蓝），无效为黑 */
void disp_to_rgb(const nf_f32 *disp, const nf_u8 *valid, int w, int h,
                 float dmax, unsigned char *rgb) {
    for (int i = 0; i < w * h; ++i) {
        if (!valid[i] || !(disp[i] > 0.0f)) {
            rgb[i*3+0] = rgb[i*3+1] = rgb[i*3+2] = 0;
            continue;
        }
        float t = disp[i] / (dmax > 0.0f ? dmax : 1.0f);
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        /* 红(近) -> 黄 -> 绿 -> 青 -> 蓝(远) */
        const float r = t < 0.5f ? 1.0f : 2.0f * (1.0f - t);
        const float g = t < 0.5f ? 2.0f * t : 2.0f * (1.0f - t);
        const float b = t < 0.5f ? 0.0f : 2.0f * (t - 0.5f);
        rgb[i*3+0] = (unsigned char)(r * 255.0f);
        rgb[i*3+1] = (unsigned char)(g * 255.0f);
        rgb[i*3+2] = (unsigned char)(b * 255.0f);
    }
}

/* 视口图：灰度 + 无效像素染红，便于看出有效域 */
void view_to_rgb(const nf_u8 *img, const nf_u8 *valid, int w, int h,
                 unsigned char *rgb) {
    for (int i = 0; i < w * h; ++i) {
        if (valid && !valid[i]) {
            rgb[i*3+0] = 200; rgb[i*3+1] = 30; rgb[i*3+2] = 30;
        } else {
            rgb[i*3+0] = rgb[i*3+1] = rgb[i*3+2] = img[i];
        }
    }
}

}  // namespace

int main(int argc, char **argv) {
    const char *base = (argc > 1) ? argv[1]
                                  : "/Volumes/Workspace/NFOCC/datasets/fisheye";
    const nf_u32 frame = (argc > 2) ? (nf_u32)std::atoi(argv[2]) : 164u;
    const char *outdir = (argc > 3) ? argv[3] : "/Volumes/Workspace/NFOCC/viz";

    mkdir(outdir, 0755);

    /* ---------- 1. 载入标定与图像 ---------- */
    char dir[512];
    std::snprintf(dir, sizeof(dir), "%s/calibration/calibration", base);
    NFWSCalib cal;
    if (nf_ws_load_frame(dir, frame, &cal) != NF_OK) {
        std::printf("标定加载失败\n");
        return 1;
    }
    const int ncam = 4;
    static const char *kNames[4] = { "FV", "MVL", "MVR", "RV" };
    /**
     * **文件序 -> cam_id**（不是 cam_id -> 文件序）。
     *
     * `nf_ws_load_frame` 内部按 FV/MVL/MVR/RV 的文件序加载，
     * 但写入的下标是 cam_id（0=FV 1=RV 2=ML 3=MR）。
     * 所以 cal.cam[1] 是 **RV**，cal.cam[2] 才是 **MVL**。
     *
     * 早期这里写成 `kFileIdx[cam_id]` 并拿它去索引文件名数组，
     * 方向反了：cam_id=1(RV) 取到 kNames[2]="MVR"，
     * 结果 **RV 的标定配上了 MVR 的图像**，02 之后的输出全部错位。
     * 01_y_* 没暴露是因为那里按文件序遍历，恰好是 0,1,2,3。
     */
    static const int kFileOfCam[4] = { 0, 3, 1, 2 };  /* cam_id -> 文件序 */

    const int W = (int)cal.image_w, H = (int)cal.image_h;
    std::vector<nf::Png> cur(ncam), ref(ncam);
    for (int c = 0; c < ncam; ++c) {
        char p[512];
        const int fi = kFileOfCam[c];
        const nf_u32 fid = frame + (nf_u32)fi;
        std::snprintf(p, sizeof(p), "%s/rgb_images/%05u_%s.png",
                      base, fid, kNames[fi]);
        if (!nf::png_read_gray(p, &cur[c])) {
            std::printf("读图失败 %s\n", p); return 1;
        }
        std::snprintf(p, sizeof(p), "%s/previous_images/%05u_%s_prev.png",
                      base, fid, kNames[fi]);
        if (!nf::png_read_gray(p, &ref[c])) {
            std::printf("读参考失败 %s\n", p); return 1;
        }
    }
    std::printf("载入 %d 路, 图像 %dx%d\n", ncam, W, H);

    /* 1) Y 通道 */
    for (int c = 0; c < ncam; ++c) {
        char p[512];
        /* 用 cal.name[c] 而不是文件序 —— 与 02/03/04 的命名保持一致,
         * 避免两处映射口径不一（这正是 02 之后错位的成因）。 */
        std::snprintf(p, sizeof(p), "%s/01_y_%s.ppm", outdir, cal.name[c]);
        write_ppm_gray(p, cur[c].data.data(), W, H);
    }
    std::printf("已输出 01_y_*.ppm\n");

    /* ---------- 2. 视口 ---------- */
    /**
     * 视口宽度用**常量 NF_V_VIEW_W**，不再由图像宽/E_DOWNSCALE 推导。
     *
     * 原来 1280/2=640，f 只有 85.7，5 m 处中位误差 ~490mm。
     * 现在固定 1024（f=137.2），是精度需求的直接结果。
     * 源图 1280 宽仍大于视口，采样不欠采样。
     */
    const nf_u32 vw = (W < NF_V_VIEW_W) ? W : (nf_u32)NF_V_VIEW_W;
    NFViewParams vp;
    if (nf_view_params_derive_fov(&vp, vw, NF_V_VIEW_HFOV_DEG,
                                  NF_V_VIEW_VFOV_DEG) != NF_OK) {
        std::printf("视口参数失败\n"); return 1;
    }
    /**
     * **6 个虚拟视口**（与 `nfocc_viewplan` 一致）。
     *
     * 前后相机各出两个：FV 偏 ±45°，RV 偏 ±45°（相对 180°）。
     * 左右各一个：ML +90°、MR −90°。
     *
     * 为什么前后要两个: 直行时平移沿光轴，单一前向视口的横向基线为 0，
     * 视差深度消失；偏 ±45° 后横向基线 = T·sin45° = 0.707T。
     *
     * **历史**: 早期这里每路只取一个代表朝向（FV 0°、RV 180°），
     * 于是调试图只有 4 张视口，看不到 ±45° 的两个 —— 与本方案不符。 */
    /**
     * 视口槽带**各自的张角**：前后 90°、左右 150°。
     *
     * 前后成对使用（±45° 各 90°，合起来 180° 前向覆盖），
     * 收窄后 f 从 137 提到 512，角分辨率 3.7 倍。
     * 左右单个使用，必须 150° 才能覆盖整侧。
     */
    struct ViewSlot { int cam; float yaw; float hfov; const char *tag; };
    static const ViewSlot kViews[6] = {
        { 0,  -45.0f, NF_V_VIEW_HFOV_DEG_FRONT, "FV_p-45" },
        { 0,  +45.0f, NF_V_VIEW_HFOV_DEG_FRONT, "FV_p+45" },
        { 1, -135.0f, NF_V_VIEW_HFOV_DEG_FRONT, "RV_p-45" },
        { 1, +135.0f, NF_V_VIEW_HFOV_DEG_FRONT, "RV_p+45" },
        { 2,  +90.0f, NF_V_VIEW_HFOV_DEG_SIDE,  "ML" },
        { 3,  -90.0f, NF_V_VIEW_HFOV_DEG_SIDE,  "MR" },
    };
    const int nview = 6;
    std::printf("视口 %ux%u f=%.2f（虚拟相机 pitch=0, roll=0）\n",
                vp.width, vp.height, (double)vp.f);

    std::vector<nf_u8>  vimg((size_t)vp.width * vp.height);
    std::vector<nf_u8>  vval((size_t)vp.width * vp.height);
    std::vector<nf_u8>  rimg((size_t)vp.width * vp.height);
    std::vector<nf_u8>  rval((size_t)vp.width * vp.height);
    std::vector<unsigned char> rgb((size_t)vp.width * vp.height * 3);

    NFViewLut lut;
    std::memset(&lut, 0, sizeof(lut));

    /* ---------- 引擎 ---------- */
    NFEngineConfig ec;
    std::memset(&ec, 0, sizeof(ec));
    ec.version = NF_ENGINE_CONFIG_VERSION;
    ec.kind = NF_ENGINE_SGBM;
    ec.max_disp_px = NF_E_MAX_DISP;
    ec.downscale = 1;              /* 视口已按 downscale 建好，引擎不再缩 */
    ec.block_size = NF_E_BLOCK;
    ec.uniqueness = NF_E_UNIQUENESS;
    ec.speckle_window = NF_E_SPECKLE_WINDOW;

    /* ---------- 体素 ---------- */
    NFVoxelGridParams gp;
    nf_voxel_grid_init(&gp, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                       NF_G_Y_MIN, NF_G_Y_MAX, NF_G_Z_MIN, NF_G_Z_MAX);
    NFVoxelCtx *vox = nullptr;
    nf_voxel_create(&vox, &gp, (nf_u32)NF_G_HASH_CAPACITY, (nf_f32)NF_V_N_EFF_MAX);

    /* ego 位移 */
    char ep[512];
    std::snprintf(ep, sizeof(ep), "%s/vehicle_info/rgb_images/%05u_FV.json",
                  base, frame);
    NFWSEgo ego;
    nf_ws_load_ego(ep, &ego);
    const float move_mm = ego.distance_m * 1000.0f *
                          (ego.gear[0] == 'R' ? -1.0f : 1.0f);
    std::printf("ego: speed=%.2f m/s dist=%.4f m -> 位移 %.1f mm\n",
                (double)ego.speed_mps, (double)ego.distance_m, (double)move_mm);

    /* 每格记录**来源视口位掩码**（哪个视口的点落进来的）。
     * 用于 BEV 上按视口着色，看清各视口的覆盖贡献。 */
    const nf_u32 viz_n_cells = gp.nx * gp.ny;
    std::vector<nf_u8> cell_view(viz_n_cells, 0);

    nf_u32 n_raw = 0, n_ins = 0, n_full = 0, n_out = 0, n_foe = 0;
    /* sigma 直方图: <10, <30, <60, <100, >=100 cm */
    nf_u32 sig_hist[5] = { 0, 0, 0, 0, 0 };
    float pmin[3] = { 1e9f, 1e9f, 1e9f }, pmax[3] = { -1e9f, -1e9f, -1e9f };
    nf_u32 bins[4] = { 0, 0, 0, 0 };
    nf_u32 n_far = 0;
    /* 近场系统: 20m 之外一律视为退化解 */
    const float NF_VIZ_MAX_RANGE_MM = 20000.0f;
    for (int vi = 0; vi < nview; ++vi) {
        const int c = kViews[vi].cam;
        const char *tag = kViews[vi].tag;
        const NFCameraModel *cam = &cal.cam[c];
        const NFPose *pose = &cal.pose[c];

        /* 该视口的虚拟相机朝向（绝对 yaw, pitch=0） */
        /* 按该视口自己的 hfov 重建，不能沿用全局 vp（那是侧向 150°） */
        NFViewParams vpc;
        if (nf_view_params_derive_fov(&vpc, vw, kViews[vi].hfov,
                                      NF_V_VIEW_VFOV_DEG) != NF_OK) {
            std::printf("view%d 参数失败\n", vi); continue;
        }
        vpc.yaw_deg = kViews[vi].yaw;
        vpc.pitch_deg = 0.0f;

        {
            const NFStatus sa = nf_lut_alloc(&lut, vp.width, vp.height);
            if (sa != NF_OK) { std::printf("view%d(%s) LUT alloc 失败: %s\n", vi, tag, nf_status_str(sa)); continue; }
        }
        NFLutStats st;
        const NFStatus sl = nf_lut_build(&lut, cam, pose, &vpc, nullptr, &st);
        if (sl != NF_OK) {
            std::printf("view%d(%s) LUT 失败: %s\n", vi, tag, nf_status_str(sl));
            continue;
        }
        std::printf("view%-6d %-9s (cam%d %s yaw=%+6.1f hfov=%3.0f f=%6.1f) LUT 有效 %.1f%%\n",
                    vi, tag, c, cal.name[c], (double)kViews[vi].yaw,
                    100.0 * st.valid / st.total);

        /* 2) 当前帧视口 */
        nf_view_warp(&lut, cur[c].data.data(), W, H, W, vimg.data(), vval.data());
        {
            char p[512];
            std::snprintf(p, sizeof(p), "%s/02_view_%s.ppm",
                          outdir, tag);
            view_to_rgb(vimg.data(), vval.data(), (int)vp.width,
                        (int)vp.height, rgb.data());
            write_ppm_rgb(p, rgb.data(), (int)vp.width, (int)vp.height);
        }

        /* 3) 前一帧 -> 同一视口（对齐） */
        nf_view_warp(&lut, ref[c].data.data(), W, H, W, rimg.data(), rval.data());
        {
            char p[512];
            std::snprintf(p, sizeof(p), "%s/03_view_ref_%s.ppm",
                          outdir, tag);
            view_to_rgb(rimg.data(), rval.data(), (int)vp.width,
                        (int)vp.height, rgb.data());
            write_ppm_rgb(p, rgb.data(), (int)vp.width, (int)vp.height);
        }

        /* 4) 视差 */
        NFEngineCtx *eng = nullptr;
        if (nf_engine_init(&eng, &ec) != NF_OK) {
            std::printf("cam%d 引擎失败\n", c);
            nf_lut_destroy(&lut);
            continue;
        }
        /* SGBM 要求 left(x) 对应 right(x-d)。t 在视口系 x 分量为正时
         * 像点右移，需要镜像后匹配（与 pipeline 同一判据）。 */
        const float tw[3] = { move_mm, 0.0f, 0.0f };
        /**
         * t_cam = **视口系**位移（SPEC-CONVENTION §2）。
         *
         * 虚拟相机的视口系是车系绕 Z 轴转 yaw，故:
         *   t_S = Rz(-yaw) · t_V
         * 车系位移 tw = (move,0,0)（车辆前进方向）。
         * 早期这里先转到相机系再绕相机 y 轴转 —— 那是"视口跟随光轴"的写法，
         * 用真实外参转一次等于把安装角也带进来了。
         */
        float tv[3];
        {
            /**
             * **t_cam = 车系位移在视口三轴上的投影**。
             *
             * 视口系的三个基向量（由 `view_lut.cpp::pixel_to_ray_impl` 定义）:
             *   forward = ( cos yaw,  sin yaw, 0)   -> 视口 +z
             *   right   = ( sin yaw, -cos yaw, 0)   -> 视口 +x
             *   down    = ( 0, 0, -1)               -> 视口 +y
             *
             * 注意这不是标准右手系: right 的 y 分量是 **-cos**，
             * 于是 (right, down, forward) 构成左手排列。所以**不能**
             * 直接用 Rz(±yaw) 转。
             *
             * **历史 bug**: 这里曾写 `tv = Rz(-yaw)·tw`，等价于把车系 x
             * 当成视口 x。实测 yaw=0 时给出横向基线 273（应为 0）、
             * yaw=90 时给出 0（应为 273）——**横向与纵向完全错位**。
             * 后果: 直行时 FV 视口拿到的是"纯横向"基线，视差方向与
             * 实际像移正交，三角化解出天量退化解，
             * 点云范围 x 达 8.9e11，BEV 只剩单一方向有深度。
             */
            const float yaw = vpc.yaw_deg * (float)(M_PI / 180.0);
            const float cy = std::cos(yaw), sy = std::sin(yaw);
            const float rx =  sy, ry = -cy, rz = 0.0f;   /* right */
            const float dx = 0.0f, dy = 0.0f, dz = -1.0f;/* down  */
            const float fx =  cy, fy =  sy, fz = 0.0f;   /* forward */
            tv[0] = tw[0]*rx + tw[1]*ry + tw[2]*rz;      /* 横向基线 */
            tv[1] = tw[0]*dx + tw[1]*dy + tw[2]*dz;
            tv[2] = tw[0]*fx + tw[1]*fy + tw[2]*fz;      /* 纵向 */
        }
        const bool mirror = tv[0] > 0.0f;
        if (mirror) {
            for (nf_u32 y = 0; y < vp.height; ++y) {
                nf_u8 *a = &vimg[(size_t)y * vp.width];
                nf_u8 *b = &rimg[(size_t)y * vp.width];
                for (nf_u32 x = 0; x < vp.width / 2; ++x) {
                    std::swap(a[x], a[vp.width-1-x]);
                    std::swap(b[x], b[vp.width-1-x]);
                }
            }
        }

        NFImageView lv, rv;
        std::memset(&lv, 0, sizeof(lv)); std::memset(&rv, 0, sizeof(rv));
        lv.version = rv.version = NF_IMAGE_VIEW_VERSION;
        lv.data = vimg.data(); lv.width = vp.width; lv.height = vp.height;
        lv.stride = vp.width;
        rv.data = rimg.data(); rv.width = vp.width; rv.height = vp.height;
        rv.stride = vp.width;

        NFDisparityRange dr;
        std::memset(&dr, 0, sizeof(dr));
        dr.version = NF_DISPARITY_RANGE_VERSION;
        dr.d_min = 0.5f;
        dr.d_max = vp.f * move_mm / NF_B_ZMIN_EFF;

        NFDisparityMap dm;
        std::memset(&dm, 0, sizeof(dm));
        dm.version = NF_DISPARITY_MAP_VERSION;
        dm.width = vp.width; dm.height = vp.height;
        dm.stride_f = (nf_i32)vp.width; dm.stride_b = (nf_i32)vp.width;
        const size_t npx = (size_t)vp.width * vp.height;
        dm.disp  = (nf_f32 *)std::calloc(npx, sizeof(nf_f32));
        dm.valid = (nf_u8 *)std::calloc(npx, 1);
        dm.conf  = (nf_f32 *)std::calloc(npx, sizeof(nf_f32));

        const NFStatus se = nf_engine_run(eng, &lv, &rv, &dr, &dm);
        nf_u32 nval = 0;
        for (size_t q = 0; q < npx; ++q) if (dm.valid[q]) ++nval;
        std::printf("cam%d(%s) SGBM=%s d_max=%.1f 有效视差 %u (%.1f%%)\n",
                    c, cal.name[c], nf_status_str(se), (double)dr.d_max,
                    nval, 100.0 * nval / npx);

        {
            char p[512];
            std::snprintf(p, sizeof(p), "%s/04_disp_%s.ppm",
                          outdir, tag);
            disp_to_rgb(dm.disp, dm.valid, (int)vp.width, (int)vp.height,
                        dr.d_max, rgb.data());
            write_ppm_rgb(p, rgb.data(), (int)vp.width, (int)vp.height);
        }

        /* 5) 三角化 -> 体素
         *
         * **c1 必须与射线同一个基（车系）**。
         * tv 是视口系的位移，而 `nf_view_pixel_to_ray(&vpc, ...)` 返回的是
         * **车系**方向（虚拟相机基向量建在车系上）。基不一致时
         * 解出的 s0 会被视口 yaw 拧一个角度。
         * 把 tv 转到车系: v_V = (right,down,forward) 基的逆变换，
         * 即 tv_x·right + tv_y·down + tv_z·forward。
         */
        float tVcar[3];
        {
            const float yaw_r = vpc.yaw_deg * (float)(M_PI / 180.0);
            const float cyr = std::cos(yaw_r), syr = std::sin(yaw_r);
            const float rx = syr, ry = -cyr;
            const float fx = cyr, fy = syr;
            tVcar[0] = tv[0]*rx + tv[2]*fx;
            tVcar[1] = tv[0]*ry + tv[2]*fy;
            tVcar[2] = -tv[1];              /* down = (0,0,-1) */
        }
        const float c1[3] = { -tVcar[0], -tVcar[1], -tVcar[2] };
        const float c0[3] = { 0, 0, 0 };
        for (nf_u32 y = 0; y < vp.height; ++y) {
            for (nf_u32 x = 0; x < vp.width; ++x) {
                const size_t i = (size_t)y * vp.width + x;
                if (!dm.valid[i]) continue;
                const float d = dm.disp[i];
                if (d < dr.d_min || d > dr.d_max) continue;

                /* **必须用 vpc（该视口的 yaw）而不是 vp（yaw=0）**。
                 * 用 vp 时六个视口的射线全部按车头方向生成，
                 * 于是所有点都挤到同一侧 —— BEV 上表现为只有单一方向有深度
                 * （实测 前 14220 / 左 18700 / 后 2243 / 右 531）。 */
                float r0[3], r1[3];
                nf_view_pixel_to_ray(&vpc, (float)x + 0.5f, (float)y + 0.5f, r0);
                const float u2 = mirror ? ((float)(vp.width - 1 - x) + 0.5f + d)
                                        : ((float)x + 0.5f - d);
                nf_view_pixel_to_ray(&vpc, u2, (float)y + 0.5f, r1);

                float s0 = 0.0f; nf_u8 ok = 0;
                nf_triangulate_rays(c0, r0, c1, r1, 1, &s0, &ok);
                if (!ok || s0 <= 0.0f) continue;
                /* 诊断: s0 分布 */
                if (s0 < 1e3f) ++bins[0];
                else if (s0 < 1e4f) ++bins[1];
                else if (s0 < 1e6f) ++bins[2];
                else ++bins[3];

                /**
                 * **s0 必须做物理上界检查**。
                 *
                 * 两条射线近平行时，《叉积闭式》解给出的 den 极小，
                 * s0 会被放大到 1e6~1e11 —— 数值上"收敛"，物理上是退化解。
                 * 实测未加上界时: 40968 个候选里只有 31% 落在 1-10m，
                 * 其余 69% 飞出感知范围（点云 x 达 8.9e11）。
                 * 这些越界点全部堵在网格边界，使 BEV 只显示单一方向的深度。
                 *
                 * 上界取 "该视口能看到的最远距离": 网格对角线的 1.5 倍足够；
                 * 这里用 20m（近场系统，2 倍网格半宽）。
                 */
                if (s0 > NF_VIZ_MAX_RANGE_MM) { ++n_far; continue; }

                /* r0/s0 给出的是**车系**坐标（虚拟相机射线就是车系方向），
                 * 只需加上相机光心平移。旧版还要乘 pose->R，是因为旧视口
                 * 射线在相机系；改成虚拟相机后这一步必须去掉，否则会
                 * 把车系向量再当相机系转一次 —— 点云会整体歪掉。 */
                const float Pv[3] = { r0[0]*s0 + pose->t[0],
                                      r0[1]*s0 + pose->t[1],
                                      r0[2]*s0 + pose->t[2] };
                /* 记录该格被哪个视口覆盖（在插入前算格索引） */
                {
                    const nf_i32 cx = (nf_i32)std::floor((Pv[0] - gp.x_min) / gp.vox_mm);
                    const nf_i32 cy = (nf_i32)std::floor((Pv[1] - gp.y_min) / gp.vox_mm);
                    const nf_i32 cz = (nf_i32)std::floor((Pv[2] - gp.z_min) / gp.vox_mm);
                    if (cx >= 0 && cx < (nf_i32)gp.nx && cy >= 0 && cy < (nf_i32)gp.ny
                        && cz >= 0 && cz < (nf_i32)gp.nz) {
                        const nf_u32 cell = (nf_u32)cy * gp.nx + (nf_u32)cx;
                        cell_view[cell] |= (nf_u8)(1u << vi);
                    }
                }
                /**
                 * **D3: FoE 发散锥过滤**。
                 *
                 * 平移方向上的点在立体匹配里几乎不产生视差，深度是退化的；
                 * 硬解会得到几十米外的"拉丝"幽灵点。锥内直接丢弃。
                 * apex 必须是**相机光心**（pose->t），不是平移向量。
                 */
                const float tvn = std::sqrt(tVcar[0]*tVcar[0] + tVcar[1]*tVcar[1]
                                            + tVcar[2]*tVcar[2]);
                if (tvn > 1e-3f) {
                    const float dv[3] = { tVcar[0]/tvn, tVcar[1]/tvn, tVcar[2]/tvn };
                    if (nf_pt_in_foe(Pv, pose->t, dv, 0.9806f)) { ++n_foe; continue; }
                }

                /* ---- 方差传播：真实视差 + 真实基线 -> sigma_cm ---- */
                const nf_u8 sig = nf_compute_spatial_uncertainty_cm(
                    vp.f, tvn, d, 0.5f, 0.02f);

                const NFStatus ins = nf_voxel_insert_with_sigma(vox, Pv[0], Pv[1], Pv[2],
                                                                1.0f, (nf_u8)(1u<<c), sig);
                ++n_raw;
                if (sig < 10u) ++sig_hist[0];
                else if (sig < 30u) ++sig_hist[1];
                else if (sig < 60u) ++sig_hist[2];
                else if (sig < 100u) ++sig_hist[3];
                else ++sig_hist[4];
                for (int k = 0; k < 3; ++k) {
                    if (Pv[k] < pmin[k]) pmin[k] = Pv[k];
                    if (Pv[k] > pmax[k]) pmax[k] = Pv[k];
                }
                if (ins == NF_OK) ++n_ins;
                else if (ins == NF_E_VOXEL_FULL) ++n_full;
                else if (n_out < 3) {
                    std::printf("      [OUT] view=%s P=(%.0f,%.0f,%.0f) %s\n",
                                tag, (double)Pv[0], (double)Pv[1],
                                (double)Pv[2], nf_status_str(ins));
                    ++n_out;
                }
            }
        }

        { nf_u32 ne = 0, ns = 0, nc = 0;
          if (nf_voxel_stats(vox, &ne, &ns, &nc) == NF_OK)
              std::printf("  [VOX] view=%-9s 累计条目 %u / 槽 %u  (本视口插入 %u)\n",
                          tag, ne, ns, n_raw); }

        std::free(dm.disp); std::free(dm.valid); std::free(dm.conf);
        nf_engine_deinit(eng);
        nf_lut_destroy(&lut);
    }

    std::printf("三角化: 候选 %u, 插入成功 %u, 超距丢弃 %u, 表满 %u, 网格外 %u\n",
                n_raw, n_ins, n_far, n_full, n_out);
    std::printf("s0 分布: <1m %u | 1-10m %u | 10m-1km %u | >1km %u\n",
                bins[0], bins[1], bins[2], bins[3]);
    std::printf("D3 FoE 锥内剔除: %u\n", n_foe);
    std::printf("sigma_cm 分布: <10 %u | <30 %u | <60 %u | <100 %u | >=100 %u\n",
                sig_hist[0], sig_hist[1], sig_hist[2], sig_hist[3], sig_hist[4]);
    std::printf("点云范围 x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f]\n",
                (double)pmin[0], (double)pmax[0], (double)pmin[1], (double)pmax[1],
                (double)pmin[2], (double)pmax[2]);

    /* ---------- 6) Voxel 分层 BEV ---------- */
    {
        const int bw = 900, bh = 750;
        /* 车系 X ±9m, Y ±7.5m -> 像素 */
        const float xmin = -9000.0f, xmax = 9000.0f;
        const float ymin = -7500.0f, ymax = 7500.0f;
        std::vector<unsigned char> img((size_t)bw * bh * 3, 245);
        auto put = [&](int px, int py, unsigned char r, unsigned char g, unsigned char b) {
            if (px < 0 || py < 0 || px >= bw || py >= bh) return;
            unsigned char *q = &img[((size_t)py * bw + px) * 3];
            q[0] = r; q[1] = g; q[2] = b;
        };
        /* 坐标轴与网格 */
        for (int px = 0; px < bw; px += 50) for (int py = 0; py < bh; ++py)
            put(px, py, 225, 225, 225);
        for (int py = 0; py < bh; py += 50) for (int px = 0; px < bw; ++px)
            put(px, py, 225, 225, 225);

        NFGridOutput go;
        std::memset(&go, 0, sizeof(go));
        /* grid_output 需要 n_cells; 由 nx*ny */
        const nf_u32 n_cells = gp.nx * gp.ny;
        /* NFGridOutput 只含 zmap/zindex，网格尺寸由 gp 提供。 */
        go.version = NF_GRID_OUTPUT_VERSION;
        go.n_cells = n_cells;
        go.zmap   = (nf_u32 *)std::calloc(n_cells, sizeof(nf_u32));
        /* zindex 是 n_cells+1 个 u32 的前缀和 */
        go.zindex = (nf_u32 *)std::calloc(n_cells + 1, sizeof(nf_u32));
        if (go.zmap && go.zindex) {
            nf_grid_flatten(vox, &go);
            int cells = 0;
            for (nf_u32 cy = 0; cy < gp.ny; ++cy) {
                for (nf_u32 cx = 0; cx < gp.nx; ++cx) {
                    const nf_u32 cell = cy * gp.nx + cx;
                    if (!go.zmap[cell]) continue;
                    ++cells;
                    /* 该格最低占用的层 -> 用颜色深浅表示高度 */
                    int lowest = -1, highest = -1, nz = 0;
                    for (int k = 0; k < 32; ++k) {
                        if (go.zmap[cell] & (1u << k)) {
                            if (lowest < 0) lowest = k;
                            highest = k;
                            ++nz;
                        }
                    }
                    const float wx = gp.x_min + ((float)cx + 0.5f) * gp.vox_mm;
                    const float wy = gp.y_min + ((float)cy + 0.5f) * gp.vox_mm;
                    const int px = (int)((wx - xmin) / (xmax - xmin) * bw);
                    const int py = (int)((ymax - wy) / (ymax - ymin) * bh);
                    /**
                     * **按来源视口着色**（而不是按占用层数）。
                     *
                     * 这样一眼能看出：哪些区域是哪个视口贡献的、哪里是
                     * 多视口重叠（重叠处更可信）、哪里是单一视口独撑。
                     *
                     * 同一格被多个视口覆盖 -> 混成白色系；
                     * 单视口 -> 用该视口的专用色。
                     */
                    const nf_u8 vm = cell_view[cell];
                    int nv_cell = 0, last_v = -1;
                    for (int k = 0; k < 8; ++k)
                        if (vm & (1u << k)) { ++nv_cell; last_v = k; }
                    unsigned char r, g, b;
                    if (nv_cell >= 2) {
                        /* 多视口重叠：浅灰白，加亮表示重叠 */
                        const unsigned char v = (unsigned char)(150 + 20 * (nv_cell > 4 ? 4 : nv_cell));
                        r = g = b = v;
                    } else if (last_v >= 0 && last_v < 6) {
                        static const unsigned char VC[6][3] = {
                            {230,  60,  60},   /* 0 FV_p-45  红 */
                            {250, 140,  40},   /* 1 FV_p+45  橙 */
                            { 60, 170,  70},   /* 2 RV_p-45  绿 */
                            { 40, 190, 200},   /* 3 RV_p+45  青 */
                            {120,  90, 220},   /* 4 ML       紫 */
                            {230,  70, 180},   /* 5 MR       品红 */
                        };
                        r = VC[last_v][0]; g = VC[last_v][1]; b = VC[last_v][2];
                        /* 高度做轻微明暗（层数多则更亮），保留高度信息 */
                        const int lift = nz > 1 ? 20 : 0;
                        r = (unsigned char)(r + lift > 255 ? 255 : r + lift);
                        g = (unsigned char)(g + lift > 255 ? 255 : g + lift);
                        b = (unsigned char)(b + lift > 255 ? 255 : b + lift);
                    } else {
                        r = g = b = 160;
                    }
                    for (int dy = -2; dy <= 2; ++dy)
                        for (int dx = -2; dx <= 2; ++dx)
                            put(px + dx, py + dy, r, g, b);
                }
            }
            std::printf("BEV: %d 个占用栅格 (nx=%u ny=%u nz=%u vox=%dmm)\n",
                        cells, gp.nx, gp.ny, gp.nz, gp.vox_mm);
        }

        /* 车体轮廓 (后轴原点) */
        const float bx0 = NF_VEH_REAR_TO_FRONT - NF_VEH_LEN;
        const float bx1 = NF_VEH_REAR_TO_FRONT;
        const float by0 = -NF_VEH_WID * 0.5f, by1 = NF_VEH_WID * 0.5f;
        for (float x = bx0; x <= bx1; x += 20.0f) {
            put((int)((x-xmin)/(xmax-xmin)*bw), (int)((ymax-by1)/(ymax-ymin)*bh), 0, 0, 0);
            put((int)((x-xmin)/(xmax-xmin)*bw), (int)((ymax-by0)/(ymax-ymin)*bh), 0, 0, 0);
        }
        for (float y = by0; y <= by1; y += 20.0f) {
            put((int)((bx0-xmin)/(xmax-xmin)*bw), (int)((ymax-y)/(ymax-ymin)*bh), 0, 0, 0);
            put((int)((bx1-xmin)/(xmax-xmin)*bw), (int)((ymax-y)/(ymax-ymin)*bh), 0, 0, 0);
        }

        char p[512];
        /* ---------- FOV 扇形：每个视口的水平覆盖范围 ---------- */
        {
            /**
             * 每个视口画一条**扇形边界**（同色系但更暗，避免盖住占用格）。
             *
             * 扇形数据来源：虚拟相机的水平张角 ±hfov/2，绕 yaw 展开。
             * 地面可见距离不是固定值（pitch=0 时向下 60° 看 393mm，
             * 水平方向看无穷远），故半径统一按视口中心到网格边缘画，
             * 仅表达**方位覆盖**，不代表距离。
             */
            static const unsigned char VC2[6][3] = {
                {150,  30,  30}, {170,  95,  25}, { 40, 120,  50},
                { 25, 130, 140}, { 80,  60, 155}, {155,  45, 125},
            };
            const float hfov_half = NF_V_VIEW_HFOV_DEG * 0.5f;
            for (int vi2 = 0; vi2 < nview; ++vi2) {
                const float yawc = kViews[vi2].yaw;
                const int camv = kViews[vi2].cam;
                /* 以该相机光心为扇形顶点 */
                const float ox = cal.pose[camv].t[0];
                const float oy = cal.pose[camv].t[1];
                const int pox = (int)((ox - xmin) / (xmax - xmin) * bw);
                const int poy = (int)((ymax - oy) / (ymax - ymin) * bh);
                const float R = 420.0f;      /* 像素半径 */
                auto draw_ray = [&](float ang_deg, int step) {
                    const float a = ang_deg * (float)(M_PI / 180.0);
                    for (float rr = 0; rr < R; rr += 2.0f) {
                        const int px = pox + (int)(rr * std::cos(a));
                        const int py = poy - (int)(rr * std::sin(a));
                        if (step == 1 && ((int)rr % 6) < 3) continue;  /* 虚线 */
                        put(px, py, VC2[vi2][0], VC2[vi2][1], VC2[vi2][2]);
                    }
                };
                draw_ray(yawc - hfov_half, 0);
                draw_ray(yawc + hfov_half, 0);
                draw_ray(yawc, 1);                       /* 中轴虚线 */
            }
        }

        std::snprintf(p, sizeof(p), "%s/05_bev.ppm", outdir);
        /* ---------- 图例：视口 -> 颜色 ---------- */
        {
            static const unsigned char VC3[6][3] = {
                {230,  60,  60}, {250, 140,  40}, { 60, 170,  70},
                { 40, 190, 200}, {120,  90, 220}, {230,  70, 180},
            };
            const int lx = 12, ly = 12, lw = 120, lh = 6 * 22 + 30;
            /* 白底框 */
            for (int yy = 0; yy < lh; ++yy)
                for (int xx = 0; xx < lw; ++xx) {
                    const int px = lx + xx, py = ly + yy;
                    if (px < 0 || py < 0 || px >= bw || py >= bh) continue;
                    unsigned char *q = &img[((size_t)py * bw + px) * 3];
                    const bool border = (xx < 1 || yy < 1 || xx >= lw - 1 || yy >= lh - 1);
                    q[0] = q[1] = q[2] = border ? 40 : 252;
                }
            /* 6 个色块 */
            for (int k = 0; k < 6; ++k) {
                const int sy = ly + 8 + k * 22;
                for (int yy = 0; yy < 14; ++yy)
                    for (int xx = 0; xx < 14; ++xx) {
                        const int px = lx + 8 + xx, py = sy + yy;
                        if (px < 0 || py < 0 || px >= bw || py >= bh) continue;
                        unsigned char *q = &img[((size_t)py * bw + px) * 3];
                        q[0] = VC3[k][0]; q[1] = VC3[k][1]; q[2] = VC3[k][2];
                    }
                /* 色块右侧的简易标记（用点阵表示视口编号 1..6） */
                const int nx = lx + 8 + 20;
                for (int d = 0; d < k + 1; ++d) {
                    const int px = nx + d * 5;
                    const int py = sy + 7;
                    for (int yy = -1; yy <= 1; ++yy)
                        for (int xx = -1; xx <= 1; ++xx) {
                            const int qx = px + xx, qy = py + yy;
                            if (qx < 0 || qy < 0 || qx >= bw || qy >= bh) continue;
                            unsigned char *q = &img[((size_t)qy * bw + qx) * 3];
                            q[0] = q[1] = q[2] = 30;
                        }
                }
            }
            /* 图例底部的"多视口重叠"色块 */
            const int oy = ly + 8 + 6 * 22 - 4;
            for (int yy = 0; yy < 8; ++yy)
                for (int xx = 0; xx < 14; ++xx) {
                    const int px = lx + 8 + xx, py = oy + yy;
                    if (px < 0 || py < 0 || px >= bw || py >= bh) continue;
                    unsigned char *q = &img[((size_t)py * bw + px) * 3];
                    q[0] = q[1] = q[2] = 170;
                }
        }

        write_ppm_rgb(p, img.data(), bw, bh);
        std::printf("已输出 05_bev.ppm\n");

        std::free(go.zmap); std::free(go.zindex);
    }

    nf_voxel_destroy(vox);
    return 0;
}
