/**
 * @file test_voxel.cpp
 * @brief 体素单元测试。对应 SPEC-VERIFY V-VOX-1..6。
 */
#include "nf_test.h"
#include "nfocc/nfocc_voxel.h"
#include "nfocc/nfocc_default.h"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <vector>

namespace {

NFVoxelGridParams default_grid() {
    NFVoxelGridParams g;
    nf_voxel_grid_init(&g, NF_G_VOX, NF_G_X_MIN, NF_G_X_MAX,
                       NF_G_Y_MIN, NF_G_Y_MAX, NF_G_Z_MIN, NF_G_Z_MAX);
    return g;
}

/* V-VOX-1: nx/ny/nz 与末格半格标记 */
NF_TEST(vox1_grid_dims) {
    NFVoxelGridParams g = default_grid();
    /* 感知范围(用户定案): X ±9m, Y ±7.5m, Z -0.5~+2.5m, vox=200 */
    NF_CHECK_EQ_INT(g.nx, 90);      /* ceil(18000/200) */
    NF_CHECK_EQ_INT(g.ny, 75);      /* ceil(15000/200) */
    NF_CHECK_EQ_INT(g.nz, 15);      /* ceil(3000/200), 整除无半格 */

    nf_i32 sx = 0, sy = 0, sz = 0;
    NF_CHECK_STATUS(nf_voxel_last_cell_partial(&g, &sx, &sy, &sz), NF_OK);
    NF_CHECK_EQ_INT(sx, 200);
    NF_CHECK_EQ_INT(sy, 200);
    NF_CHECK_EQ_INT(sz, 200);       /* Z 3000/200 整除, 末格是满格 */
}

/* V-CO-5: 索引 <-> 中心往返一致, **含 X/Y 反向** */
NF_TEST(vox_index_roundtrip) {
    NFVoxelGridParams g = default_grid();

    /* X/Y 反向: X 更大 -> ix 更小 */
    nf_i32 ix1, iy1, iz1, ix2, iy2, iz2;
    nf_u8 in1 = 0, in2 = 0;
    nf_voxel_index(&g, 9000.0f, 0.0f, 1000.0f, &ix1, &iy1, &iz1, &in1);
    nf_voxel_index(&g, -4000.0f, 0.0f, 1000.0f, &ix2, &iy2, &iz2, &in2);
    NF_CHECK_EQ_INT(in1, 1);
    NF_CHECK_EQ_INT(in2, 1);
    NF_CHECK_MSG(ix1 < ix2, "X 方向必须反向");

    /* 往返 */
    for (nf_i32 i = 0; i < (nf_i32)g.nx; i += 7) {
        for (nf_i32 j = 0; j < (nf_i32)g.ny; j += 11) {
            for (nf_i32 k = 0; k < (nf_i32)g.nz; k += 5) {
                nf_f32 cx, cy, cz;
                NF_CHECK_STATUS(nf_voxel_center(&g, i, j, k, &cx, &cy, &cz), NF_OK);
                nf_i32 a, b, c; nf_u8 inside = 0;
                nf_voxel_index(&g, cx, cy, cz, &a, &b, &c, &inside);
                NF_CHECK_EQ_INT(inside, 1);
                NF_CHECK_EQ_INT(a, i);
                NF_CHECK_EQ_INT(b, j);
                NF_CHECK_EQ_INT(c, k);
            }
        }
    }
}

/* 越界点不钳位 */
NF_TEST(vox_out_of_range) {
    NFVoxelGridParams g = default_grid();
    nf_i32 ix, iy, iz; nf_u8 inside = 1;
    nf_voxel_index(&g, 20000.0f, 0.0f, 0.0f, &ix, &iy, &iz, &inside);
    NF_CHECK_EQ_INT(inside, 0);
}

/* V-VOX-3: zMap bit31 恒 0, nz=18 时 bit18+ 必须为 0 */
NF_TEST(vox3_zmap_bits) {
    NFVoxelGridParams g = default_grid();
    NFVoxelCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_create(&ctx, &g, 4096, NF_V_N_EFF_MAX), NF_OK);

    /* 在第 0 层与第 17 层各插一个点 */
    nf_f32 cx, cy, cz;
    nf_voxel_center(&g, 10, 10, 0, &cx, &cy, &cz);
    nf_voxel_insert(ctx, cx, cy, cz, 1.0f, 0x1u);
    nf_voxel_center(&g, 10, 10, 14, &cx, &cy, &cz);   /* 最后一层 iz=14 */
    nf_voxel_insert(ctx, cx, cy, cz, 1.0f, 0x1u);

    const nf_u32 n_cells = g.nx * g.ny;
    NFGridOutput out;
    out.version = NF_GRID_OUTPUT_VERSION;
    out.n_cells = n_cells;
    out.zmap = (nf_u32 *)std::calloc(n_cells, sizeof(nf_u32));
    out.zindex = (nf_u32 *)std::calloc(n_cells + 1, sizeof(nf_u32));
    NF_CHECK_STATUS(nf_grid_flatten(ctx, &out), NF_OK);

    int bad_high = 0, bad_sign = 0;
    for (nf_u32 c = 0; c < n_cells; ++c) {
        if (out.zmap[c] & (1u << 31)) ++bad_sign;
        if (out.zmap[c] & ~((1u << NF_G_NZ) - 1u)) ++bad_high;
    }
    NF_CHECK_EQ_INT(bad_sign, 0);
    NF_CHECK_EQ_INT(bad_high, 0);

    /* 柱体编号 = iy*nx+ix, 不是 ix*ny+iy */
    const nf_u32 cell = (nf_u32)10 * g.nx + 10;
    NF_CHECK_EQ_INT(out.zmap[cell] & 1u, 1u);
    NF_CHECK_EQ_INT((out.zmap[cell] >> 14) & 1u, 1u);

    nf_grid_output_destroy(&out);
    nf_voxel_destroy(ctx);
}

/* V-VOX-4: zindex 前缀和单调且末项 == 总点数 */
NF_TEST(vox4_zindex_prefix) {
    NFVoxelGridParams g = default_grid();
    NFVoxelCtx *ctx = nullptr;
    nf_voxel_create(&ctx, &g, 4096, NF_V_N_EFF_MAX);

    for (nf_i32 k = 0; k < 5; ++k) {
        nf_f32 x, y, z;
        nf_voxel_center(&g, 20, 20, k, &x, &y, &z);
        nf_voxel_insert(ctx, x, y, z, 1.0f, 0x1u);
    }

    const nf_u32 n_cells = g.nx * g.ny;
    NFGridOutput out;
    out.version = NF_GRID_OUTPUT_VERSION;
    out.n_cells = n_cells;
    out.zmap = (nf_u32 *)std::calloc(n_cells, sizeof(nf_u32));
    out.zindex = (nf_u32 *)std::calloc(n_cells + 1, sizeof(nf_u32));
    nf_grid_flatten(ctx, &out);

    int non_mono = 0;
    for (nf_u32 c = 0; c < n_cells; ++c)
        if (out.zindex[c] > out.zindex[c + 1]) ++non_mono;
    NF_CHECK_EQ_INT(non_mono, 0);
    NF_CHECK_EQ_INT(out.zindex[n_cells], 5);

    nf_grid_output_destroy(&out);
    nf_voxel_destroy(ctx);
}

/* V-VOX-2: 同一点重复插入 N 次, 质心漂移 < 1mm (CMA 正确) */
NF_TEST(vox2_cma_stable) {
    NFVoxelGridParams g = default_grid();
    NFVoxelCtx *ctx = nullptr;
    nf_voxel_create(&ctx, &g, 4096, NF_V_N_EFF_MAX);

    nf_f32 cx, cy, cz;
    nf_voxel_center(&g, 30, 30, 5, &cx, &cy, &cz);
    const nf_f32 px = cx + 20.0f, py = cy - 15.0f, pz = cz + 10.0f;

    for (int i = 0; i < 1000; ++i) nf_voxel_insert(ctx, px, py, pz, 1.0f, 0x1u);

    nf_u32 n = 0, slots = 0, coll = 0;
    nf_voxel_stats(ctx, &n, &slots, &coll);
    NF_CHECK_EQ_INT(n, 1);   /* 只应产生一个体素 */

    NFPointCloud pc;
    nf_pointcloud_alloc(&pc, 16);
    nf_voxel_export_points(ctx, &pc);
    NF_CHECK_EQ_INT(pc.num_points, 1);
    if (pc.num_points == 1) {
        const float dx = (float)(pc.points[0].x_mm - px);
        const float dy = (float)(pc.points[0].y_mm - py);
        const float dz = (float)(pc.points[0].z_mm - pz);
        const float drift = std::sqrt(dx*dx + dy*dy + dz*dz);
        NF_CHECK_MSG(drift < 1.0f, "同点重复插入质心漂移超过 1mm");
    }
    nf_pointcloud_destroy(&pc);
    nf_voxel_destroy(ctx);
}

/* 哈希位宽: 索引 > 255 必须报错 */
NF_TEST(vox_hash_width) {
    nf_i32 ix[1] = { 300 }, iy[1] = { 0 }, iz[1] = { 0 };
    nf_u32 h[1] = { 0 };
    NF_CHECK_STATUS(nf_voxel_pack_hash(ix, iy, iz, 1, h), NF_E_PARAM_RANGE);
}

/* 容量非 2 的幂必须拒绝(用掩码代替取模) */
NF_TEST(vox_capacity_pow2) {
    NFVoxelGridParams g = default_grid();
    NFVoxelCtx *ctx = nullptr;
    NF_CHECK_STATUS(nf_voxel_create(&ctx, &g, 1000, 64.0f), NF_E_PARAM_RANGE);
    NF_CHECK(ctx == nullptr);
}

/* NULL 安全 */
NF_TEST(vox_null_safety) {
    NF_CHECK_STATUS(nf_voxel_create(nullptr, nullptr, 16, 64.0f), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_voxel_insert(nullptr, 0, 0, 0, 1, 1), NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_voxel_index(nullptr, 0, 0, 0, nullptr, nullptr, nullptr, nullptr),
                    NF_E_PARAM_NULL);
    NF_CHECK_STATUS(nf_pointcloud_alloc(nullptr, 10), NF_E_PARAM_NULL);
}

/* V-ICD-3: NFPoint 必须无填充。
 *
 * **注意 15 而不是 9**: 3 个 int32 + 3 个 uint8 = 15 字节。
 * SPEC-ICD 原写 9 字节是文档自身的算术错误(已修正), 不加 packed 时
 * 编译器会把末尾补到 16。本测试同时检查这两件事。 */
NF_TEST(point_size_packed) {
    NF_CHECK_EQ_INT(sizeof(NFPoint), 15);
    NF_CHECK_EQ_INT((int)offsetof(NFPoint, y_mm), 4);
    NF_CHECK_EQ_INT((int)offsetof(NFPoint, z_mm), 8);
    NF_CHECK_EQ_INT((int)offsetof(NFPoint, source_camera), 12);
    NF_CHECK_EQ_INT((int)offsetof(NFPoint, sigma_cm), 13);
    NF_CHECK_EQ_INT((int)offsetof(NFPoint, existence_prob), 14);
}

} // namespace

int main() { return nftest::run_all(); }
