/**
 * @file nfocc_disp_probe.cpp
 * @brief 视差级探针：把"匹配误差"与"几何/融合误差"分开。
 *
 * ## 为什么需要它
 * 端到端 1-2 m 段中位误差 365-429 mm。按 sigma_Z = Z^2*sigma_d/(f*b)
 * 反推，50 mm 只需 sigma_d ~5 px —— SGBM 在合成纹理上不该这么差。
 * 说明中间某一步在放大误差，但端到端数字区分不出是哪一步。
 *
 * 本工具**绕开三角化与体素融合**，只做一件事：比较
 *   d_meas (SGBM 给出的视差)  与  d_true = f * b / Z_view
 *
 * ## 真值怎么来
 * 合成场景渲染时给了逐像素的**相机系 Z**。视口像素 (x,y) 经 LUT 得到
 * 对应的原图像素 (u,v)，取该处真值 Z；再把该 Z 换算到**视口光轴方向**：
 *   Z_view = Z_cam * cos(视口光轴与相机光轴夹角)
 * 这个夹角由视口射线与相机光轴的内积给出，逐像素不同。
 *
 * ## 判据
 *   sigma_d ≈ 0.3-0.6 px  -> 匹配没问题，误差出在三角化/融合
 *   sigma_d  > 2 px        -> 匹配就是瓶颈，要改纹理/参数/算法
 *
 * 用法:
 *   nfocc_disp_probe [--b <mm>] [--all] [--cam <0-3>]
 */
#include "nfocc/nfocc_default.h"
#include "nfocc/nfocc_engine.h"
#include "nfocc/nfocc_lut.h"
#include "nfocc/nfocc_scene.h"
#include "nfocc/nfocc_scene_render.h"
#include "nfocc/nfocc_viewplan.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>

/* 引擎符号带 sgbm 后缀（防止与模型引擎同名覆盖），先声明再映射 */
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

namespace {

double pctl(const std::vector<double> &v, double q) {
    if (v.empty()) return -1.0;
    size_t k = (size_t)(q * (double)(v.size() - 1) + 0.5);
    if (k >= v.size()) k = v.size() - 1;
    return v[k];
}

const char *view_tag(const NFViewPlanEntry &pe) {
    if (pe.cam_id == 2u) return "ML";
    if (pe.cam_id == 3u) return "MR";
    if (pe.cam_id == 0u) return (pe.yaw_deg < 0) ? "FV_p-45" : "FV_p+45";
    return (pe.yaw_deg < 0) ? "RV_p-45" : "RV_p+45";
}

}  // namespace

int main(int argc, char **argv) {
    float b_mm = 278.0f;
    int cam_filter = -1;
    bool all = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--b") && i + 1 < argc) b_mm = (float)std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--cam") && i + 1 < argc) cam_filter = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--all")) all = true;
    }
    std::printf("=== 视差级探针 ===  基线 %.0f mm (%.1f km/h @5Hz)\n",
                (double)b_mm, (double)(b_mm / 1000.0 * 5.0 * 3.6));

    NFScene scene;
    if (nf_scene_default(&scene) != NF_OK) { std::printf("场景初始化失败\n"); return 1; }

    NFViewPlanEntry plan[6];
    const nf_u32 nplan = nf_viewplan_default(plan, 6u);
    std::printf("视口规划: %u 条\n", nplan);
    const nf_u32 vw = (nf_u32)NF_V_VIEW_W;

    static const float kCamX[4] = { 3500.0f, -1500.0f, 0.0f, 0.0f };
    static const float kCamY[4] = { 0.0f, 0.0f, 1000.0f, -1000.0f };
    static const float kCamYaw[4] = { 0.0f, 180.0f, 90.0f, -90.0f };

    int n_fail = 0;
    int n_ok = 0;   /* 真正跑出样本的视口数，防"全部跳过也报通过" */

    for (nf_u32 pi = 0; pi < nplan; ++pi) {
        const NFViewPlanEntry &pe = plan[pi];
        std::printf("  -> 处理 %u: cam=%u yaw=%.0f hfov=%.0f\n", pi, pe.cam_id,
                    (double)pe.yaw_deg, (double)pe.hfov_deg);
        if (!all && cam_filter >= 0 && (int)pe.cam_id != cam_filter) continue;
        /* 默认只跑 FV 的 +45° 视口，跑全量要 --all */
        if (!all && cam_filter < 0 && !(pe.cam_id == 0u && pe.yaw_deg > 0.0f)) continue;

        const int c = (int)pe.cam_id;

        NFCameraModel cam;
        std::memset(&cam, 0, sizeof(cam));
        cam.version = NF_CAMERA_MODEL_VERSION;
        cam.kind = NF_CAM_MODEL_KB;
        /* NFCameraModel 不含图像尺寸，单独定义 */
        const nf_u32 imgW = 1280, imgH = 966;
        cam.f = 550.0f; cam.cx = 640.0f; cam.cy = 483.0f;
        /**
         * **r_valid 必须按图像对角外接圆给**，不能留 0。
         * memset 后为 0 时每个像素都判"超半径"，LUT 全空 —— 实测
         * 1816576 个采样全部落在 d2_out_of_r_valid。
         * 1280x966 对角 = sqrt(640^2+483^2) = 801.7。
         */
        cam.r_valid = 802.0f;
        cam.max_theta = 1.9f;

        NFPose pose;
        std::memset(&pose, 0, sizeof(pose));
        pose.version = NF_POSE_VERSION;   /* 渲染会校验，不设报 NF_E_PARAM_VERSION */
        pose.t[0] = kCamX[c]; pose.t[1] = kCamY[c]; pose.t[2] = 700.0f;
        {
            const float a = kCamYaw[c] * 3.14159265f / 180.0f;
            const float ca = std::cos(a), sa = std::sin(a);
            /* R 列 = 相机三轴在车系下的方向（v_V = R·v_C）
             * 相机 z(前)=(ca,sa,0)  x(右)=(sa,-ca,0)  y(下)=(0,0,-1) */
            pose.R[0*3+0] = sa;  pose.R[1*3+0] = -ca; pose.R[2*3+0] = 0.0f;
            pose.R[0*3+1] = 0.0f; pose.R[1*3+1] = 0.0f; pose.R[2*3+1] = -1.0f;
            pose.R[0*3+2] = ca;  pose.R[1*3+2] = sa;  pose.R[2*3+2] = 0.0f;
        }

        NFViewParams vp;
        {
            const NFStatus vst = nf_view_params_derive_fov(&vp, vw, pe.hfov_deg, pe.vfov_deg);
            if (vst != NF_OK) {
                std::printf("[%s] 视口参数失败: %s\n", view_tag(pe), nf_status_str(vst));
                continue;
            }
        }
        std::printf("     [A 视口参数OK]\n");
        vp.yaw_deg = pe.yaw_deg;

        /* 纹理周期按视口 f 定，保证 SGBM 有高频可匹配 */
        nf_scene_set_tex_for_focal(&scene, vp.f, 20.0f, 2000.0f);

        NFRenderFrame fc, fr;
        std::memset(&fc, 0, sizeof(fc));
        std::memset(&fr, 0, sizeof(fr));
        nf_render_frame_alloc(&fc, imgW, imgH);
        nf_render_frame_alloc(&fr, imgW, imgH);

        {
            const NFStatus rst = nf_render_frame(&scene, &cam, &pose, &fc);
            if (rst != NF_OK) {
                std::printf("[%s] 渲染失败: %s\n", view_tag(pe), nf_status_str(rst));
                nf_render_frame_free(&fc); nf_render_frame_free(&fr);
                continue;
            }
        }
        NFPose pref = pose;
        pref.version = NF_POSE_VERSION;
        pref.t[0] -= b_mm * pose.R[0*3+2];   /* 相机沿光轴后退 b */
        pref.t[1] -= b_mm * pose.R[1*3+2];
        pref.t[2] -= b_mm * pose.R[2*3+2];
        {
            const NFStatus rst = nf_render_frame(&scene, &cam, &pref, &fr);
            if (rst != NF_OK) {
                std::printf("[%s] 参考帧渲染失败: %s\n", view_tag(pe), nf_status_str(rst));
                nf_render_frame_free(&fc); nf_render_frame_free(&fr);
                continue;
            }
        }

        std::printf("     [B 渲染帧OK]\n");
        /* ---------- 视口 LUT ---------- */
        NFViewLut lut;
        {
            const NFStatus ast = nf_lut_alloc(&lut, vp.width, vp.height);
            if (ast != NF_OK) {
                std::printf("[%s] LUT alloc 失败: %s\n", view_tag(pe), nf_status_str(ast));
                nf_render_frame_free(&fc); nf_render_frame_free(&fr);
                continue;
            }
        }
        NFLutStats lst;
        std::memset(&lst, 0, sizeof(lst));
        {
            const NFStatus bst = nf_lut_build(&lut, &cam, &pose, &vp, nullptr, &lst);
            if (bst != NF_OK) {
                std::printf("[%s] LUT build 失败: %s  total=%u 出图=%u 超半径=%u "
                            "超角度=%u 掩码=%u valid=%u\n",
                            view_tag(pe), nf_status_str(bst), lst.total,
                            lst.d1_out_of_image, lst.d2_out_of_r_valid,
                            lst.d3_over_max_theta, lst.d4_body_mask, lst.valid);
                nf_lut_destroy(&lut);
                nf_render_frame_free(&fc); nf_render_frame_free(&fr);
                continue;
            }
        }

        std::printf("     [C LUT构建OK]\n");
        const size_t np = (size_t)vp.width * vp.height;
        std::vector<nf_u8> vimg(np), rimg(np), vval(np), rval(np);
        nf_view_warp(&lut, fc.img, imgW, imgH, imgW, vimg.data(), vval.data());
        nf_view_warp(&lut, fr.img, imgW, imgH, imgW, rimg.data(), rval.data());

        /* ---------- SGBM ---------- */
        NFEngineConfig ec;
        std::memset(&ec, 0, sizeof(ec));
        ec.version = NF_ENGINE_CONFIG_VERSION;
        ec.kind = NF_ENGINE_SGBM;
        ec.downscale = 1;
        ec.block_size = 5;
        ec.uniqueness = 10;
        ec.speckle_window = 100;
        const float d_far  = vp.f * b_mm / 5000.0f;
        const float d_near = vp.f * b_mm / 500.0f;
        ec.max_disp_px = (nf_i32)(std::ceil(d_near / 16.0f) * 16.0f);
        if (ec.max_disp_px > 512) ec.max_disp_px = 512;

        NFEngineCtx *eng = nullptr;
        {
            const NFStatus isr = nf_engine_init(&eng, &ec);
            if (isr != NF_OK) {
                std::printf("[%s] 引擎 init 失败: %s (numDisp=%d)\n",
                            view_tag(pe), nf_status_str(isr), (int)ec.max_disp_px);
                nf_lut_destroy(&lut);
                nf_render_frame_free(&fc); nf_render_frame_free(&fr);
                continue;
            }
        }
        if (false) {
            std::printf("[%s] 引擎创建失败\n", view_tag(pe)); continue;
        }
        NFImageView lv{}, rv{};
        std::memset(&lv, 0, sizeof(lv));
        lv.version = NF_IMAGE_VIEW_VERSION;   /* 引擎会校验，不设报 NF_E_PARAM_VERSION */
        lv.data = vimg.data(); lv.width = vp.width; lv.height = vp.height;
        lv.stride = vp.width;
        rv = lv; rv.data = rimg.data();

        std::vector<float> disp(np, 0.0f), conf(np, 0.0f);
        std::vector<nf_u8> valid(np, 0);
        NFDisparityMap dm;
        std::memset(&dm, 0, sizeof(dm));
        dm.version = NF_DISPARITY_MAP_VERSION;
        dm.width = vp.width; dm.height = vp.height;
        dm.disp = disp.data(); dm.valid = valid.data(); dm.conf = conf.data();
        NFDisparityRange dr;
        std::memset(&dr, 0, sizeof(dr));
        dr.version = NF_DISPARITY_RANGE_VERSION;
        dr.d_min = 0.0f;
        dr.d_max = (float)ec.max_disp_px;
        dr.b_mm = b_mm;
        const NFStatus est = nf_engine_run(eng, &lv, &rv, &dr, &dm);
        nf_engine_deinit(eng);
        if (est != NF_OK) {
            std::printf("[%s] 引擎运行失败: %s (numDisp=%d d=[%.1f,%.1f] 视口=%ux%u)\n",
                        view_tag(pe), nf_status_str(est), (int)ec.max_disp_px,
                        (double)dr.d_min, (double)dr.d_max, vp.width, vp.height);
            nf_lut_destroy(&lut);
            nf_render_frame_free(&fc); nf_render_frame_free(&fr);
            continue;
        }

        {
            int nv = 0, nz = 0;
            float dmin = 1e9f, dmax = -1e9f;
            for (size_t k = 0; k < np; ++k) {
                if (!valid[k]) continue;
                ++nv;
                if (disp[k] <= 0.0f) { ++nz; continue; }
                if (disp[k] < dmin) dmin = disp[k];
                if (disp[k] > dmax) dmax = disp[k];
            }
            std::printf("     [D SGBM] 视口%ux%u valid=%d 零视差=%d 范围=[%.2f,%.2f] "
                        "输入L=%u R=%u 前256字节L均值=%.0f R均值=%.0f\n",
                        vp.width, vp.height, nv, nz,
                        nv > nz ? (double)dmin : -1.0, nv > nz ? (double)dmax : -1.0,
                        lst.valid, lst.valid,
                        (double)vimg[100], (double)rimg[100]);
        }
        /* ---------- 逐像素比较 d_meas vs d_true ----------
         *
         * d_true = f * b / Z_view
         * Z_view = 该视口像素看到的真值点，沿**视口光轴**的深度。
         *
         * 求 Z_view：视口像素 (x,y) 经 LUT 得到原图 (u,v)，
         * 原图 (u,v) 反投影出相机射线 d_cam；真值点 P = C + d_cam * t，
         * 其中 t 由渲染深度 Z_cam 换算（P = C + d_cam * (Z_cam / d_cam.z)）。
         * 再取 P 在视口 forward 轴上的投影。
         */
        const float yawv = vp.yaw_deg * 3.14159265f / 180.0f;
        const float fwd[3] = { std::cos(yawv), std::sin(yawv), 0.0f };
        const float cxv = pose.t[0], cyv = pose.t[1], czv = pose.t[2];

        std::vector<double> errs, zview;
        int n_gt_miss = 0;
        int n_disp_valid = 0;

        for (nf_u32 y = 0; y < vp.height; ++y) {
            for (nf_u32 x = 0; x < vp.width; ++x) {
                const size_t i = (size_t)y * vp.width + x;
                if (!valid[i] || !vval[i]) continue;
                const float d = disp[i];
                if (d <= 0.0f || d > dr.d_max) continue;
                ++n_disp_valid;

                /* LUT 给的原图坐标（Q12.4） */
                const int su4 = (int)lut.u_q4[i];
                const int sv4 = (int)lut.v_q4[i];
                const int su = su4 >> 4, sv = sv4 >> 4;
                if (su < 0 || sv < 0 || su >= (int)imgW || sv >= (int)imgH) continue;

                const float z_cam = fc.depth[(size_t)sv * imgW + su];
                if (z_cam <= 0.0f) { ++n_gt_miss; continue; }

                /* 相机射线（相机系） */
                float dc[3];
                if (nf_cam_unproject_c(&cam, (float)su + 0.5f, (float)sv + 0.5f, dc) != NF_OK)
                    continue;
                if (dc[2] <= 1e-6f) continue;

                /* 真值点（车系）= 光心 + R * dc * (Z_cam / dc_z) */
                const float t = z_cam / dc[2];
                float P[3];
                for (int k = 0; k < 3; ++k)
                    P[k] = pose.t[k] + (pose.R[k*3+0]*dc[0] + pose.R[k*3+1]*dc[1]
                                        + pose.R[k*3+2]*dc[2]) * t;

                /* 视口光轴上的深度 */
                const float zv = (P[0]-cxv)*fwd[0] + (P[1]-cyv)*fwd[1];
                if (zv < 500.0f || zv > 5000.0f) continue;   /* 只看 0.5-5m */

                const float d_true = vp.f * b_mm / zv;
                errs.push_back((double)(d - d_true));
                zview.push_back((double)zv);
            }
        }
        nf_lut_destroy(&lut);
        nf_render_frame_free(&fc);
        nf_render_frame_free(&fr);

        /* ---------- 报告 ---------- */
        std::printf("\n[%-8s] yaw=%+5.0f  hfov=%3.0f  f=%6.1f  numDisp=%d"
                    "  LUT有效=%.1f%%\n",
                    view_tag(pe), (double)pe.yaw_deg, (double)pe.hfov_deg,
                    (double)vp.f, (int)ec.max_disp_px,
                    100.0 * (double)lst.valid / (double)(lst.valid + lst.d1_out_of_image
                            + lst.d2_out_of_r_valid + lst.d3_over_max_theta
                            + lst.d4_body_mask + 1));
        if (errs.empty()) {
            std::printf("  无有效样本: LUT有效=%u 视差有效=%d 真值缺失=%d "
                        "([%d,%d] 中视差 [%.1f,%.1f])\n",
                        lst.valid, n_disp_valid, n_gt_miss,
                        (int)dr.d_min, (int)dr.d_max,
                        (double)vp.f * b_mm / 5000.0, (double)vp.f * b_mm / 500.0);
            ++n_fail;
            continue;
        }

        std::vector<double> ae;
        ae.reserve(errs.size());
        double sum = 0.0, ss = 0.0;
        for (double e : errs) { ae.push_back(std::fabs(e)); sum += e; ss += e*e; }
        std::vector<double> sorted_ae = ae;
        std::sort(sorted_ae.begin(), sorted_ae.end());
        const double bias = sum / (double)errs.size();
        const double rms  = std::sqrt(ss / (double)errs.size());

        std::printf("  样本 %zu  视差误差: 中位 %.3f px  P95 %.3f px  偏置 %+.3f  RMS %.3f\n",
                    errs.size(), pctl(sorted_ae, 0.5), pctl(sorted_ae, 0.95), bias, rms);

        /* 按深度分段 */
        static const char *band[4] = { "1-2m", "2-3m", "3-4m", "4-5m" };
        std::printf("  按深度: ");
        for (int k = 0; k < 4; ++k) {
            std::vector<double> sub;
            for (size_t j = 0; j < zview.size(); ++j) {
                const double z = zview[j];
                const int kk = (z < 2000.0) ? 0 : (z < 3000.0) ? 1 : (z < 4000.0) ? 2 : 3;
                if (kk == k) sub.push_back(std::fabs(errs[j]));
            }
            if (sub.empty()) { std::printf("%s:n=0  ", band[k]); continue; }
            std::sort(sub.begin(), sub.end());
            std::printf("%s:n=%zu med=%.3fpx  ", band[k], sub.size(), pctl(sub, 0.5));
            /* 该段 sigma_Z 由 sigma_d 推算，用于和端到端对比 */
            const double zmid = 1500.0 + 1000.0 * k;
            const double sz = zmid * zmid * pctl(sub, 0.5) / ((double)vp.f * b_mm);
            std::printf("->sigmaZ=%.0fmm  ", sz);
        }
        std::printf("\n");

        /* 判据：中位视差误差 <= 0.6 px 视为匹配可用 */
        if (pctl(sorted_ae, 0.5) > 0.6) ++n_fail;
        else ++n_ok;
    }

    std::printf("\n跑出样本的视口: %d 个，超限 %d 个\n", n_ok + n_fail, n_fail);
    if (n_ok + n_fail == 0) {
        std::printf("**无任何样本 —— 探针本身失效，不能判通过**\n");
        return 2;
    }
    std::printf("%s\n", n_fail ? "存在视口匹配精度不足（中位 > 0.6 px）"
                               : "全部视口中位视差误差 <= 0.6 px");
    return n_fail ? 1 : 0;
}
