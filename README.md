# NFOCC

多相机鱼眼近场稠密深度与体素重建系统。4 路环视鱼眼 + 运动双目，覆盖 0.5–5 m，
输出点云与 2D 高程柱（zMap）。

C++11 产品实现，面向 RK3588 / Axera M57 边缘 NPU；另附 Python 原型（`nfocc_ph0/`、`nfocc_ph1/`）。

## 目录

```
cpp/            C++11 产品实现（核心库 + 工具 + 测试）
  include/nfocc/     公共头（零第三方依赖）
  src/               实现
  tools/             nfocc_run / nfocc_viz / nfocc_gen / nfocc_sink
  tests/             单元 + 集成 + 系统测试
  scripts/           构建与验收脚本
  cmake/             三方依赖与交叉编译工具链
nfocc_ph0/      Python 原型（相机模型、视口 LUT、体素）
nfocc_ph1/      Python 原型（管线、门控、三角化）
docs/           设计文档集（31 篇，`docs/README.md` 为索引）
docs/doc/       原始需求与技术方案（留档）
tools/          文档自检、可视化、探针脚本
CHANGES.md      变更记录
```

## 快速开始

依赖：OpenCV 5.0、ZeroMQ 4.3.5、Eigen 5.0、OpenBLAS、libomp、zlib。

```bash
cd cpp
./scripts/build.sh host          # 构建
ctest --test-dir build/host      # 测试
./build/host/nfocc_run --synthetic --frames 10 --stats
```

真实数据（hanqtech 鱼眼格式）需放到 `datasets/fisheye/`，结构见
`docs/SPEC-DATASET.md`：

```bash
./build/host/nfocc_run --hanqtech datasets/fisheye --ws-frame 164 --frames 2 --stats
./build/host/nfocc_viz  datasets/fisheye 164 viz     # 中间结果可视化
```

## 核心设计

**虚拟相机视口**：所有视口按 pitch=0、roll=0 构造，只有 yaw。真实外参
只决定「这个车系方向落在哪张鱼眼的哪个像素」，不参与视口朝向。

6 视口规划：ML +90°、MR −90°、FV ±45°、RV ±135°。前后各两个是为了
直行时获得横向基线（单前向视口横向基线为 0，视差深度消失）。

**坐标约定**：车体 ISO 8855、相机 OpenCV、虚拟视口右手系。
`t_cam` 用**视口系**表达，定义见 `docs/SPEC-CONVENTION.md`。

## 状态

`ctest` 12/13（`depth_accuracy` 未通过，`t_cam` 语义已定但测量口径待修）。
详见 `docs/PROJECT-STATUS.md` 与 `CHANGES.md`。

## 许可

内部项目，未授权勿外传。
