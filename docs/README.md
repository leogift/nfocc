# 文档索引

当前工程状态：C++11 产品树，21 个 ctest 目标通过 20 个；180 用例 / 2,361 断言；
`depth_accuracy` 的 6 个用例中 4 个旧验收用例仍失败。最近一次大改是 2026-11-19 的
SFM 6 视口（见 `SFM-6VIEW.md`）。

## 当前有效

| 文档 | 内容 | 与代码的对应 |
|---|---|---|
| [SFM-6VIEW.md](SFM-6VIEW.md) | 6 视口设计、`t_view` 投影、极点判据、4 vs 6 实测 | `NFPipelineConfig::view_plan`、`nf_pipeline_process`、`probe_sfm6` |
| [SPEC-CONVENTION.md](SPEC-CONVENTION.md) | 相机/车体/视口坐标、`t_cam`/`t_veh` 语义 | `nfocc_viewplan.h`、`view_lut.cpp`、pipeline |
| [SPEC-ICD.md](SPEC-ICD.md) | 公共结构体、状态码、失效语义 | `cpp/include/nfocc/*.h` |
| [SPEC-LUT.md](SPEC-LUT.md) | 视口 LUT 构建、Q12.4、有效性判定 | `cpp/src/lut/view_lut.cpp` |
| [SPEC-DATASET.md](SPEC-DATASET.md) | hanqtech 数据、内参/外参、环带、位移 | `cpp/src/io/ws_*` |
| [SPEC-VERIFY.md](SPEC-VERIFY.md) | 当前可执行的验收矩阵与判定 | `cpp/tests/`、`probe_sfm6` |
| [BUILD-TOOLCHAIN.md](BUILD-TOOLCHAIN.md) | macOS/Linux/Aarch64 构建、依赖治理 | `cpp/scripts/build.sh`、`cpp/cmake/` |
| [IMPROVEMENT-PLAN.md](IMPROVEMENT-PLAN.md) | 当前基线、剩余缺口、P2–P4 计划 | 计划文档，不是规范 |

## 根目录

- [README.md](../README.md)：安装、构建、运行、测试入口。
- [CHANGES.md](../CHANGES.md)：2026-11-19 SFM 6 视口变更。

## 验证

```bash
cd cpp
ctest --test-dir build/host --output-on-failure          # 20/21
ctest --test-dir build/host -E depth_accuracy            # 排除已知失败
NF_TEST_FILTER=front90 ./build/host/tests/test_depth_accuracy
./build/host/probe_sfm6                                  # 4 vs 6 解析场景
```

## 维护规则

`docs/` 只保留与当前代码/实测一致的文档。历史实验、已废弃方案、旧报告不在本树保留；
需要追溯时查提交历史，不把旧结论混进当前规范。
