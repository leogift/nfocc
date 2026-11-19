# SPEC-VERIFY — 当前验收矩阵

本文件只记录**当前树里能执行、能判定**的验收项。历史 V-* 编号和已删除规范的引用不再保留。

## 0. 验收级别

| 级别 | 含义 | 当前执行 |
|---|---|---|
| L0 单元 | 单函数/数学关系 | 每次构建 |
| L1 模块 | 模块级合成数据 | 每次合并 |
| L2 链路 | 4/6 视口 + 引擎 + 体素 | 每个里程碑 |
| L3 实车 | 真实数据/真值 | 未具备 |

## 1. 当前测试矩阵

| 能力 | 测试 | 状态 |
|---|---|---|
| 相机模型、外参、KB 往返 | `test_camera`、`test_vcam_bridge` | 通过 |
| 几何、三角化、体素索引 | `test_geometry`、`test_voxel` | 通过 |
| 视口 LUT / 场景 | `test_scene`、`test_pipeline::pipe_view_selfconsistent` | 通过 |
| 引擎容量/确定性/绑定 | `test_engine`、`test_engine_bind` | 通过 |
| 门控、双基线、新模块 | `test_d4_baseline`、`test_newmod` | 通过 |
| 4 视口端到端 | `test_pipeline`、`test_newmod_integration` | 通过 |
| 6 视口映射与 `t_veh` | `test_pipeline::pipe_six_view_plan` | 通过 |
| 体素精简/聚类/IO/并发 | `test_voxel_filter`、`test_cluster`、`test_io_protocol`、`test_concurrency`、`test_zmq_io` | 通过 |
| 前向精度 | `test_depth_accuracy` | **失败**：6 用例中 4 个旧验收用例失败 |

`ctest`：21 个目标 **20 通过**；180 用例 / 2,361 断言。

## 2. 已知失败

`depth_accuracy` 的 4 个旧验收用例（侧行/直行/45°/15 km/h）仍失败。当前前视 90°
分段表面中位：

| 工况 | 1–2 m | 2–3 m | 3–4 m | 4–5 m |
|---|---:|---:|---:|---:|
| 前视 90°，278 mm 基线 | 307 | 305 | 460 | 857 |
| 目标 | 150 | 200 | 320 | 320 |

真实数据没有点云真值，不能把“落在合理范围”当成精度通过。

## 3. 6 视口验收

| 项 | 判据 | 证据 |
|---|---|---|
| 视口表 | 6 项，cam 映射 `0,0,1,1,2,3` | `nf_viewplan_default`、`pipe_six_view_plan` |
| 前后 | 中心相隔 90°，各 110° 张角，590×716 | `SPEC-LUT.md` |
| 左右 | ±90°，150°，1024×475 | `SPEC-LUT.md` |
| 路径 | 前后 `|t_z|/|t|>0.25` 走径向，左右走 SGBM | `pipeline.cpp` |
| 真实数据 | 2,019 点 / 104.6 ms / RSS 165.3 MB | 帧 164，`--sfm6` |
| 解析场景 | 6 视口前向表面 354 mm，4 视口 402 mm | `probe_sfm6` |

## 4. 负对照

当前保留的负对照：

| 项 | 正例 | 负例 |
|---|---|---|
| 外参 | 正常 `R` | 非正交 `R` → `NF_E_EXTRINSIC_INVALID` |
| 引擎容量 | `numDisp >= d_max` | `numDisp < d_max` → `NF_E_ENGINE_RANGE` |
| 冻结 | 侧移/前向有效帧 | 静止/近场 → 冻结且 0 点 |
| 6 视口 | `t_veh` 投影 | 同一相机两个视口共用 `t_cam` 会错（文档禁止） |
| 掩码 | 空掩码出点 | 全掩码禁用对应视口 |

## 5. 复现

```bash
cd cpp
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Release \
      -DNF_WITH_VCAM=ON -DNF_BUILD_TESTS=ON
cmake --build build/host -j8
ctest --test-dir build/host --output-on-failure
NF_TEST_FILTER=front90 ./build/host/tests/test_depth_accuracy
./build/host/probe_sfm6
```
