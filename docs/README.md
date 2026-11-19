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
| [SPEC-VERIFY.md](SPEC-VERIFY.md) | 验收判据、状态码覆盖、负对照 | `cpp/tests/`；部分 V-* 编号仍待补 |
| [BUILD-TOOLCHAIN.md](BUILD-TOOLCHAIN.md) | macOS/Linux/Aarch64 构建、依赖治理 | `cpp/scripts/build.sh`、`cpp/cmake/` |
| [IMPROVEMENT-PLAN.md](IMPROVEMENT-PLAN.md) | P0–P4 计划与 2026-11-19 完成情况 | 计划文档，不是规范 |

## 历史 / 计划记录

| 文档 | 状态 |
|---|---|
| [PLAN-DEFENSE-INTEGRATION.md](PLAN-DEFENSE-INTEGRATION.md) | 防御接入的外部支持清单；D1 已文件化、D4 同帧残差检验已接入（完整 D4 需双基线）、D2 未接入，正文保留当时的实施判断 |
| [PROBE-RESULTS.md](PROBE-RESULTS.md) | 2026-11-02/03 的视差探针实验记录；数字是当时基线，不代表当前 6 视口 |
| [APPENDIX-CHANGELOG.md](APPENDIX-CHANGELOG.md) | 文档变更史；包含已删除文档和旧报告的记录 |

## 根目录

- [README.md](../README.md)：安装、构建、运行、测试入口。
- [CHANGES.md](../CHANGES.md)：2026-11-19 SFM 6 视口变更。

## 已不在当前树中的引用

旧 README/文档曾引用以下文件，当前树没有；根 README 已改指现存文档：

- docs/ROADMAP.md、docs/DECISION-DEPTH-ACCURACY.md、docs/SPEC-DEPTH-ACCEPTANCE.md、
  docs/CLUSTERING-IMPLEMENTATION.md、docs/BUILD-CROSSPLATFORM.md
- docs/report/NFOCC-项目报告-2026-11.pdf
- tools/docs_check.py、tools/calib_gen.py、tools/sgbm_bench.py、
  tools/sync_third_party.py、scripts/verify_docs.sh

这些文件的结论若仍有效，已合并到 `IMPROVEMENT-PLAN.md`、`SPEC-VERIFY.md` 或
`BUILD-TOOLCHAIN.md`；没有合并的按历史记录处理。

## 验证

```bash
cd cpp
ctest --test-dir build/host --output-on-failure          # 20/21
ctest --test-dir build/host -E depth_accuracy            # 排除已知失败
NF_TEST_FILTER=front90 ./build/host/tests/test_depth_accuracy
./build/host/probe_sfm6                                  # 4 vs 6 解析场景
```
