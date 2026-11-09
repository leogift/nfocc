#!/usr/bin/env python3
"""NFOCC 报告图表生成器。

每个数字都来自 2026-11-09 的实测（ctest / nfocc_run --stats / probe / wc）。
计算值在图中明确标注"计算"，不与实测混在一起。
"""
from __future__ import annotations

import math
from pathlib import Path

from svgkit import (ACC, AMBER, FAINT, FILL_AMBER, FILL_BLUE, FILL_GREEN,
                    FILL_GREY, FILL_PURPLE, FILL_RED, GREEN, INK, MUTED,
                    PURPLE, RED, TEAL, WHITE, Fig)

OUT = Path(__file__).resolve().parent / "fig"
OUT.mkdir(exist_ok=True)

F_PX = 137.19          # 1024x475 @150° 视口等效焦距
SIG_D = 0.5            # px，SGBM 典型亚像素精度（计算假设）
TARGET_MM = 150.0


def sigma_z(z_mm: float, b_mm: float, sig_d: float = SIG_D) -> float:
    return z_mm * z_mm * sig_d / (F_PX * b_mm)


# ---------------------------------------------------------------- 图 1 物理约束
def fig_physics():
    W, H = 1080, 500
    f = Fig(W, H)
    f.title("速度决定精度：基线 → 视差 → 深度误差",
            "σ_Z = Z²·σ_d /(f·b)，f=137.19 px，σ_d=0.5 px；曲线为计算值，实测见 §7")

    x0, y0, pw, ph = 92, 105, 610, 300
    f.axes(x0, y0, pw, ph, 0.5, 6.0, 0, 500,
           [1, 2, 3, 4, 5, 6], [0, 100, 200, 300, 400, 500],
           "目标距离 Z（m）", "深度误差 σ_Z（mm）")

    curves = [(278, AMBER, "5 km/h · b=278 mm"),
              (500, TEAL, "9 km/h · b=500 mm"),
              (833, ACC, "15 km/h · b=833 mm")]
    for b, color, _ in curves:
        pts = []
        for i in range(0, 221):
            z = 0.5 + i * (6.0 - 0.5) / 220
            s = sigma_z(z * 1000.0, b)
            if s > 500:
                break
            pts.append((x0 + (z - 0.5) / 5.5 * pw, y0 + ph - s / 500 * ph))
        f.parts.append(
            '<polyline points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in pts) +
            f'" fill="none" stroke="{color}" stroke-width="2.6"/>')

    # 150 mm 验收线
    yt = y0 + ph - TARGET_MM / 500 * ph
    f.line(x0, yt, x0 + pw, yt, RED, 1.4, "6 4")
    f.text(x0 + 8, yt - 7, "150 mm 验收线", 13, RED, weight="600")

    # 交点
    for b, color in ((278, AMBER), (500, TEAL), (833, ACC)):
        zc = math.sqrt(TARGET_MM * F_PX * b / SIG_D) / 1000.0
        if 0.5 <= zc <= 6:
            px = x0 + (zc - 0.5) / 5.5 * pw
            f.circle(px, yt, 4.2, WHITE, color, 1.8)
            f.text(px, yt + 20, f"{zc:.1f} m", 12, color, "middle", "600")

    # 右侧速度档卡
    bx, bw = 748, 300
    cards = [
        (278, AMBER, "5 km/h · 278 mm", "5 m 处 σ 328 mm · 150 mm 仅到 3.4 m"),
        (500, TEAL, "9 km/h · 500 mm", "5 m 处 σ 182 mm · 达标到 4.5 m"),
        (833, ACC, "15 km/h · 833 mm", "5 m 处 σ 109 mm · 达标到 5.9 m"),
    ]
    for i, (b, color, t1, t2) in enumerate(cards):
        y = 105 + i * 78
        f.rect(bx, y, bw, 66, WHITE, color, 7, 1.3)
        f.rect(bx, y, 5, 66, color, None, 2)
        f.text(bx + 18, y + 25, t1, 15, color, weight="600")
        f.text(bx + 18, y + 47, t2, 12.5, MUTED)

    f.rect(748, 352, bw, 82, FILL_RED, RED, 7, 1.2)
    f.text(764, 376, "5 m 处要达 150 mm：b ≥ 607 mm", 14.5, RED, weight="600")
    f.text(764, 398, "≈ 11 km/h @ 5 Hz。低于此速度，", 12.5, "#7a3b33")
    f.text(764, 416, "150 mm 不是调参能解决的。", 12.5, "#7a3b33")

    f.note(30, 462, "帧间位移 b = v/(5 Hz)；σ_d 取 0.5 px 是 SGBM 的典型值，非本项目实测。"
                    "误差按 Z² 增长，是几何决定的。")
    f.note(30, 484, "里程计项 σ_b=2% 未计入；计入后远距离误差更大。")
    f.save(OUT / "fig01_physics.svg")


# ---------------------------------------------------------------- 图 2 架构
def fig_arch():
    W, H = 1080, 660
    f = Fig(W, H)
    f.title("四层架构：应用 / 主链 / 算法 / 基础设施",
            "核心库 11,004 行（src+include），仅依赖 vcam 静态库；引擎与相机后端均为运行期绑定")

    layers = [
        ("应用层", 96, FILL_GREY, [
            ("nfocc_run", "CLI · 端到端", "613 行"),
            ("nfocc_viz", "中间结果可视化", "929 行"),
            ("nfocc_gen / sink", "合成 / 回放", ""),
            ("探针工具", "视差 / DVF / 扰动", ""),
        ]),
        ("主链层", 224, FILL_GREEN, [
            ("pipeline", "6 阶段调度 · 门控 · 冻结", "1,017 行"),
            ("input_source + io", "文件 / ZMQ / 真实数据", "1,608 行"),
            ("frame_pool", "帧缓冲复用", "395 行"),
        ]),
        ("算法层", 352, FILL_AMBER, [
            ("geometry", "三角化 · 径向测距 · FoE", "938 行"),
            ("voxel", "Hash + CMA · zMap", "1,081 行"),
            ("engine", "SGBM / 模型双引擎", "416 行"),
            ("lut", "虚拟视口 · 车身掩码", "657 行"),
        ]),
        ("基础设施", 480, FILL_PURPLE, [
            ("camera / vcam", "KB · 径向多项式", "433 行"),
            ("cluster", "网格哈希 + DBSCAN", "647 行"),
            ("baseline", "双基线 · 里程计", "383 行"),
            ("util", "线程池 · 日志 · 计时", "395 行"),
        ]),
    ]

    content_x, content_w = 132, W - 162
    for name, y, fill, items in layers:
        f.rect(24, y - 26, W - 48, 96, fill, "#d8e0e8", 9, 1.0)
        f.text(38, y + 26, name, 15, MUTED, weight="600")
        n = len(items)
        gap = 12
        iw = (content_w - (n - 1) * gap) / n
        for i, (nm, sub, ln) in enumerate(items):
            x = content_x + i * (iw + gap)
            f.box(x, y - 14, iw, 70, nm, sub, ln, WHITE, "#93a7bb", 15, 12.5)

    for y in (174, 302, 430):
        f.arrow(W / 2, y, W / 2, y + 18, "#93a4b5")

    # 依赖边界
    f.rect(24, 556, W - 48, 76, FILL_BLUE, ACC, 8, 1.1)
    f.text(40, 580, "依赖边界", 14, ACC, weight="600")
    f.text(40, 603, "核心库不链接 OpenCV：SGBM 经 NFEngineOps 函数表在 nfocc_run 侧注入；"
                    "相机投影只走 vcam 静态库。", 12.5, "#33475b")
    f.text(40, 622, "代价：NF_WITH_VCAM=OFF 仍能编过，但投影返回 UNSUPPORTED —— "
                    "实测 21 项测试会多失败 5 项。", 12.5, "#7a3b33")
    f.save(OUT / "fig02_arch.svg")


# ---------------------------------------------------------------- 图 3 代码规模
def fig_codesize():
    W, H = 1080, 470
    f = Fig(W, H)
    f.title("代码规模与工程构成", "cpp/src 实现 7,575 行，公共头 3,429 行；测试代码 5,494 行，与实现比 0.73")

    mods = [("io", 1608), ("voxel", 1081), ("pipeline", 1017), ("geometry", 938),
            ("lut", 657), ("cluster", 647), ("camera", 433), ("engine", 416),
            ("util", 395), ("baseline", 383)]
    x0, y0, w, rh = 110, 100, 420, 30
    mx = 1700
    f.text(x0, 84, "各模块实现行数（cpp/src）", 14, INK, weight="600")
    for i, (name, v) in enumerate(mods):
        y = y0 + i * rh
        f.text(x0 - 12, y + 19, name, 13.5, MUTED, "end")
        bw = v / mx * w
        f.rect(x0, y + 5, w, 19, "#eef1f4", None, 3)
        f.rect(x0, y + 5, bw, 19, ACC, None, 3)
        f.text(x0 + bw + 8, y + 20, f"{v:,}", 12.5, "#33475b", weight="600")

    # 右：构成
    rx = 640
    f.text(rx, 84, "工程构成（实测）", 14, INK, weight="600")
    stats = [
        ("核心实现", "7,575", "cpp/src，10 个模块"),
        ("公共头文件", "3,429", "cpp/include，契约与文档"),
        ("合计", "11,004", "src + include"),
        ("测试", "5,494", "21 个 ctest 目标"),
        ("工具", "2,555", "run / viz / probe"),
        ("文档", "42 篇", "docs/*.md，自检全绿"),
        ("提交", "42 次", "2026-11-01 → 11-03"),
    ]
    for i, (k, v, note) in enumerate(stats):
        y = 106 + i * 44
        f.rect(rx, y, 410, 36, WHITE, "#dfe5ec", 5, 1)
        f.text(rx + 12, y + 23, k, 13.5, MUTED)
        f.text(rx + 118, y + 23, v, 15, ACC, weight="600")
        f.text(rx + 215, y + 23, note, 12, FAINT)

    f.note(30, 426, "行数含空行与注释；不含 third_party。测试/实现 = 5,494/7,575 = 0.73。")
    f.save(OUT / "fig03_codesize.svg")


# ---------------------------------------------------------------- 图 4 调用关系
def fig_callgraph():
    W, H = 1080, 740
    f = Fig(W, H)
    f.title("代码调用关系：从 CLI 到体素输出",
            "实线=主链调用；虚线=离线诊断。C 接口 + 函数表，唯一的 C++ 继承是 InputSource")

    # 入口
    f.box(24, 92, 190, 64, "nfocc_run", "参数解析 / 主循环", "", FILL_GREY, "#93a7bb", 15, 12)
    f.box(24, 176, 190, 64, "nfocc_viz", "离线诊断", "", FILL_GREY, "#93a7bb", 15, 12)

    # 输入
    f.box(262, 92, 180, 64, "InputSource", "虚基类 · ReadFrame", "", FILL_BLUE, ACC, 15, 12)
    f.text(262, 174, "Synthetic / File / ZMQ / hanqtech", 11.8, MUTED)
    f.text(262, 190, "NFFrameInput: img[4] / img_ref[4]", 10.8, FAINT)
    f.text(262, 206, "t_cam[4]（视口系）", 10.8, FAINT)

    f.text(228, 84, "build", 11.5, ACC, "middle")
    f.arrow(214, 124, 260, 124, ACC, marker="acc")

    # 主链大框
    f.rect(470, 80, 340, 512, "#f7fafd", ACC, 10, 1.4)
    f.text(640, 104, "NFPipeline", 17, ACC, "middle", "600")
    f.text(640, 123, "nf_pipeline_process()", 12, MUTED, "middle")
    rows = [
        ("① nf_view_warp ×4", "LUT 重采样 / 掩码"),
        ("② nf_baseline_manager_decide", "或 nf_gate_decide"),
        ("③ nf_match_radial | SGBM", "前向 / 侧向两分支"),
        ("④ nf_forward_range | 三角化", "像素 → 3D 点"),
        ("⑤ FoE · σ 传播 · σ 截断 · D4", "逐点防御"),
        ("⑥ nf_voxel_insert → export", "Hash + CMA 融合"),
        ("⑦ nf_voxel_filter_compress", "五阶段精简（可选）"),
    ]
    for i, (main, sub) in enumerate(rows):
        y = 140 + i * 62
        f.rect(485, y, 310, 50, WHITE, "#c9d6e3", 6, 1.1)
        f.text(495, y + 21, main, 13.5, INK, weight="600")
        f.text(495, y + 39, sub, 11.5, MUTED)

    # 输入到主链
    f.arrow(442, 124, 468, 124, ACC, marker="acc")

    # 右侧模块
    mods = [
        ("nf_lut_build", "nf_vcam_project"),
        ("NFBaselineManager", "hist[16] · 双基线"),
        ("NFEngineOps", "SGBM / MODEL 函数表"),
        ("nfocc_geometry", "纯函数 · 无状态"),
        ("NFVoxelCtx", "开放寻址 · 质心 CMA"),
        ("NFVoxelFilterCtx", "位图 / 扇区 / 配额"),
    ]
    for i, (main, sub) in enumerate(mods):
        y = 140 + i * 62
        f.box(830, y, 220, 50, main, sub, "", FILL_AMBER, "#c99a55", 13, 10.8)
        f.arrow(796, y + 25, 828, y + 25, "#9aa8b6", marker="ar")

    # 输出
    f.arrow(640, 592, 640, 610, GREEN, marker="green")
    f.box(470, 612, 340, 52, "NFFrameOutput",
          "cloud · dvf_points · zmap/zindex · 冻结码", "", FILL_GREEN, GREEN, 14, 11)

    # 离线聚类
    f.rect(24, 470, 400, 124, FILL_PURPLE, PURPLE, 8, 1.2)
    f.text(40, 494, "离线诊断链（nfocc_viz）", 13.5, PURPLE, weight="600")
    f.text(40, 516, "nf_octree_build → nf_dbscan → nf_point_velocity", 12, "#4a3a66")
    f.text(40, 536, "→ nf_cluster_vote_motion → 簇级动静", 12, "#4a3a66")
    f.text(40, 560, "网格哈希索引 120 mm；150k 点 54 ms（均匀）", 11.5, MUTED)
    f.arrow(214, 240, 214, 466, PURPLE, "6 4", marker="ar")

    f.note(24, 704, "调用点以 pipeline.cpp 为准；引擎经 nf_pipeline_bind_engine() 注入，核心库不链接 OpenCV。")
    f.note(24, 724, "num_points_per_cam 统计的是插入调用次数，越界点也返回 OK 并计数 —— 与最终体素数不等。")
    f.save(OUT / "fig04_callgraph.svg")


