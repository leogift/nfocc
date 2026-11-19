# CHANGES

## 2026-11-19 — SFM 6 视口

### 新增

- `docs/SFM-6VIEW.md`：6 视口设计、`t_view` 投影/反投影、极点判据、4 vs 6 实测与限制。
- `cpp/tools/probe_sfm6.cpp`：解析场景真值探针（4/6 视口、前向/侧向分组、表面/竖面中位）。
- `test_pipeline::pipe_six_view_plan`：6 视口映射、`t_veh` 路径与模式断言。
- `NFPipelineConfig::view_plan[6] / n_views`；`NFFrameInput::t_veh[3]`；
  `NFFrameOutput::n_views / view_mode / view_status / num_points_per_view / view_cam`。

### 修改

- pipeline 从“每相机一个视口”改为“每视口一套 LUT/当前参考图/有效位/视差图”；
  SGBM 引擎全 pipeline 一个、串行复用。
- 视口：FV ±45°、RV ±135°（中心间隔 90°，各 110° 张角）；ML +90°、MR -90°（150°）。
- 前后斜视按 SfM 极线走径向匹配（FoE=极点），左右 |t_z|≈0 才走 SGBM。
- 修正 `t_view` 投影：`f=(cos,sin)`、`r=(sin,-cos)`，不再用 `Rz`；
  前向路径不再对车系射线二次乘 `pose->R`。
- 输入源（synthetic/scene/hanqtech）填 `t_veh`；`nfocc_run --sfm6` 启用 6 视口。

### 实测

- 真实 hanqtech 帧 164：6 视口 2,019 点 / 104.6 ms / RSS 165.3 MB / 地下 11.5%；
  4 视口 1,999 点 / 179.4 ms / 地下 12.4%。
- 解析场景前向：4 视口表面 402 mm、竖面 410 mm；6 视口 354 mm、382 mm。
- 侧视两模式逐项相同（720 点、672/718 mm）。
- ctest 20/21；180 用例 / 2,361 断言；`depth_accuracy` 6 个用例中 4 个失败。

### 注意

- 前视 1–2 m 仍 307 mm，未达 150；径向搜索上限 64 px 卡住近场。
- 旧报告 223/222/367/408 与 P1 206/231/347/385 是在前向 `t`/射线基不一致的代码下测的，
  已作废，不能与当前数比较。
