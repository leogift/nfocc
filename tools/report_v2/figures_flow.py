#!/usr/bin/env python3
"""数据流 / 门控 / 几何 / 防御 四张图。"""
from __future__ import annotations

from svgkit import (ACC, AMBER, FAINT, FILL_AMBER, FILL_BLUE, FILL_GREEN,
                    FILL_GREY, FILL_PURPLE, FILL_RED, GREEN, INK, MUTED,
                    PURPLE, RED, WHITE, Fig)

from figures import OUT


def fig_pipeline():
    W, H = 1080, 590
    f = Fig(W, H)
    f.title("单帧数据流：4 路鱼眼 → 点云 / 高程柱",
            "状态码贯穿每一级；冻结时输出 num_points=0 + NF_E_BASELINE_*，不是空点云")

    f.box(24, 92, 222, 62, "① 输入", "4×Y 1280×966 + prev + ego", "", FILL_GREY, "#93a7bb", 15, 11.5)
    f.box(24, 180, 222, 62, "② 视口重采样", "4×1024×475，f=137.19", "", FILL_BLUE, ACC, 15, 11.5)
    f.box(24, 268, 222, 76, "③ 工况决策", "nf_gate_decide（默认）", "或双基线状态机", FILL_BLUE, ACC, 15, 11.5)
    f.arrow(135, 154, 135, 178, ACC, marker="acc")
    f.arrow(135, 242, 135, 266, ACC, marker="acc")
    f.text(30, 366, "缓冲：view_img[2] / view_val[2] / dmap", 10.8, FAINT)

    f.text(300, 112, "FORWARD  σ≤0.35", 13, GREEN, weight="600")
    f.box(300, 126, 210, 60, "④a 径向匹配", "nf_match_radial", "", FILL_GREEN, GREEN, 14, 11)
    f.box(540, 126, 210, 60, "⑤a 解析测距", "nf_forward_range", "", FILL_GREEN, GREEN, 14, 11)
    f.arrow(510, 156, 538, 156, GREEN, marker="green")

    f.text(300, 232, "STEREO  σ≥0.80", 13, ACC, weight="600")
    f.box(300, 246, 210, 60, "④b SGBM", "NFEngineOps.run", "", FILL_BLUE, ACC, 14, 11)
    f.box(540, 246, 210, 60, "⑤b 三角化", "nf_triangulate_rays", "", FILL_BLUE, ACC, 14, 11)
    f.arrow(510, 276, 538, 276, ACC, marker="acc")

    f.path("M246 306 C270 306 270 156 298 156", stroke=GREEN, sw=1.7, marker="arg")
    f.path("M246 306 C270 306 270 276 298 276", stroke=ACC, sw=1.7, marker="arb")

    f.box(790, 120, 260, 112, "⑥ 逐点防御", "D3 FoE · σ 传播 · σ 截断 · D4", "不通过则丢弃", FILL_AMBER, "#c99a55", 15, 11.3)
    f.box(790, 252, 260, 76, "⑦ 体素融合", "Hash + CMA · 质心 / σ 取最小", "", FILL_AMBER, "#c99a55", 15, 11.3)
    f.box(790, 348, 260, 76, "⑧ 导出 / 精简", "点云 → DVF 五阶段（可选）", "", FILL_PURPLE, PURPLE, 15, 11.3)
    f.box(790, 444, 260, 66, "⑨ 输出", "cloud · dvf · zMap/zIndex · ZMQ", "", FILL_GREY, "#93a7bb", 15, 11)
    f.arrow(752, 176, 788, 176, "#9aa8b6")
    f.arrow(752, 276, 788, 290, "#9aa8b6")
    f.arrow(920, 232, 920, 250, "#9aa8b6")
    f.arrow(920, 328, 920, 346, "#9aa8b6")
    f.arrow(920, 424, 920, 442, "#9aa8b6")

    f.rect(300, 390, 450, 70, FILL_RED, RED, 7, 1.1)
    f.text(316, 414, "冻结旁路（四路全冻结 → is_frozen=1）", 13, RED, weight="600")
    f.text(316, 436, "FROZEN_NEAR / FAR / ROT / GAP：不产出点，但仍返回状态码", 11.5, "#7a3b33")
    f.arrow(246, 344, 298, 410, RED, "4 4", marker="arr")

    f.note(24, 500, "实测有效帧：FORWARD 11.2 s（4 路中 2 路走前向）· STEREO 104 ms（4 路 SGBM）· 冻结帧 1.2–2.5 ms。")
    f.note(24, 522, "两条路径都先产出视口系射线，再转车系；t_cam 的基与符号在 pipeline.cpp 中有三处显式换算。")
    f.note(24, 544, "D4 与 DVF 均为可选：D4 默认关，DVF 需 enable_voxel_filter=1 或 --filter。")
    f.save(OUT / "fig05_pipeline.svg")


def fig_gate():
    W, H = 1080, 560
    f = Fig(W, H)
    f.title("工况判定：默认单帧门控，可选双基线状态机",
            "b = |t_cam|（帧间位移）；σ = |t_xy|/|t|（平移偏离光轴的程度）")

    f.text(24, 100, "A. 默认：nf_gate_decide（无状态）", 15, INK, weight="600")
    rules = [
        ("frame_gap > gap_max", "FROZEN_GAP", RED),
        ("b < b_min（278 mm）", "FROZEN_NEAR", RED),
        ("b > b_max（833 mm）", "FROZEN_FAR", RED),
        ("σ ≥ 0.80", "STEREO（SGBM）", ACC),
        ("σ ≤ 0.35", "FORWARD（径向）", GREEN),
        ("0.35 < σ < 0.80", "HYBRID（实走 STEREO）", AMBER),
    ]
    for i, (cond, mode, color) in enumerate(rules):
        y = 118 + i * 52
        f.rect(24, y, 490, 42, WHITE, "#dfe5ec", 5, 1)
        f.text(36, y + 26, cond, 13.5, INK)
        f.arrow(300, y + 21, 330, y + 21, color, marker="ar", sw=1.4)
        f.text(340, y + 26, mode, 13.5, color, weight="600")
    f.text(24, 452, "顺序即代码顺序：丢帧 → 基线不足 → 超限 → 方向分流。", 11.5, MUTED)

    f.text(560, 100, "B. --baseline-mgr：双基线状态机", 15, INK, weight="600")
    f.box(560, 120, 150, 64, "FROZEN_NEAR", "历史不足或 b<278", "", FILL_RED, RED, 13, 10.5)
    f.box(760, 120, 150, 64, "STEREO 短", "b ≥ 278 mm", "", FILL_GREEN, GREEN, 14, 10.5)
    f.box(760, 230, 150, 64, "STEREO 长", "b ≥ 500 mm", "", FILL_GREEN, GREEN, 14, 10.5)
    f.box(560, 230, 150, 64, "重置队列", "yaw>15° / b>833", "", FILL_RED, RED, 13, 10.5)
    f.arrow(712, 152, 758, 152, GREEN, marker="green")
    f.arrow(835, 186, 835, 228, GREEN, marker="green", label="晋升", lx=862, ly=212)
    f.path("M758 262 C700 262 660 262 662 296", stroke=RED, sw=1.6, marker="arr")
    f.arrow(636, 232, 636, 186, RED, marker="red", label="reset", lx=600, ly=210)
    f.text(560, 330, "短基线 b=278 → 长基线 b=556（单测连续 8 次长基线输出）", 11.5, MUTED)
    f.text(560, 352, "历史队列容量 16 帧；只在 cam0 推进，四路共享决策。", 11.5, MUTED)

    f.rect(560, 378, 490, 112, FILL_AMBER, "#c99a55", 7, 1.1)
    f.text(576, 402, "关键差异", 13.5, AMBER, weight="600")
    f.text(576, 424, "状态机只按 |t| 判，不判方向；直行数据 b≥500 时被判 STEREO，", 11.5, "#6b4a10")
    f.text(576, 444, "SGBM 在纯前向平移下结构性失效 —— 实测 46,821 次插入、最终 0 点。", 11.5, "#6b4a10")
    f.text(576, 464, "默认门控会判 FORWARD，同一数据产出 2,077 点。", 11.5, "#6b4a10")

    f.note(24, 520, "b_min=278 / b_max=833 / b_target=500 mm 对应 5 / 15 / 9 km/h @5 Hz；阈值改动需同步 PARAM 反推。")
    f.save(OUT / "fig06_gate.svg")


def fig_forward():
    W, H = 1080, 520
    f = Fig(W, H)
    f.title("前向为什么不能用 SGBM：径向膨胀的解析测距",
            "纯前向平移下，像移从 FoE 辐射、跨 FoE 方向翻转；水平视差搜索失去意义")

    O0, O1, P = (110, 400), (340, 400), (300, 150)
    f.line(O0[0], O0[1], O1[0], O1[1], MUTED, 2.2)
    f.arrow(O0[0], O0[1] + 18, O1[0], O1[1] + 18, MUTED, marker="ar", sw=1.6)
    f.text(225, 442, "t（帧间位移）", 12.5, MUTED, "middle")
    f.line(O0[0], O0[1], P[0], P[1], ACC, 2.4)
    f.line(O1[0], O1[1], P[0], P[1], GREEN, 2.4)
    f.circle(O0[0], O0[1], 7, WHITE, INK, 1.8)
    f.circle(O1[0], O1[1], 7, WHITE, INK, 1.8)
    f.circle(P[0], P[1], 6, RED, RED, 1)
    f.text(O0[0] - 20, O0[1] + 24, "O0", 14, INK, weight="600")
    f.text(O1[0] + 10, O1[1] + 24, "O1", 14, INK, weight="600")
    f.text(P[0] + 12, P[1] - 8, "P", 14, RED, weight="600")
    f.text(185, 258, "u（参考帧）", 12.5, ACC)
    f.text(325, 258, "v（当前帧）", 12.5, GREEN)
    f.text(58, 120, "q0 = Z·u，q1 = q0 − t", 13, MUTED)

    f.rect(590, 92, 460, 128, FILL_GREEN, GREEN, 9, 1.4)
    f.text(612, 118, "解析解（对任意平移方向成立）", 13, GREEN, weight="600")
    f.text(612, 156, "Z = Σ[(u×v)·(t×v)] / Σ‖u×v‖²", 21, INK, weight="600")
    f.text(612, 186, "无需迭代；u、v 为两帧同一像素的视线单位向量", 11.5, MUTED)

    f.rect(590, 240, 460, 96, FILL_RED, RED, 8, 1.1)
    f.text(606, 264, "SGBM 失效的实测证据", 13, RED, weight="600")
    f.text(606, 286, "合成墙面真值 140.8 px → SGBM 输出 2.2 px", 11.8, "#7a3b33")
    f.text(606, 306, "偏置随基线严格成正比（278→−34 px，556→−61 px）", 11.8, "#7a3b33")
    f.text(606, 326, "→ 锁到固定错误周期，不是随机噪声", 11.8, "#7a3b33")

    f.rect(590, 352, 460, 96, FILL_BLUE, ACC, 8, 1.1)
    f.text(606, 376, "实现链", 13, ACC, weight="600")
    f.text(606, 398, "径向一维搜索 → 膨胀率 → 两帧射线 → 闭式解 → FoE 锥过滤", 11.5, "#33475b")
    f.text(606, 418, "输出距离是 |q0|，不是光轴深度 q0.z —— 混用会差数倍", 11.5, "#33475b")
    f.text(606, 438, "合成单测：6 个距离点全部通过（forward_range_exact）", 11.5, "#33475b")

    f.note(24, 458, "注：SGBM 失效与左右图顺序的数据来自 2026-11-02 视差探针，本次未复测。", 11.5)
    f.note(24, 482, "真实直行数据走此分支，有效帧 11.2 s；耗时在径向标量搜索，不在解析解本身。")
    f.save(OUT / "fig07_forward.svg")


def fig_defense():
    W, H = 1080, 520
    f = Fig(W, H)
    f.title("四道已接入防御 + 一道未接入",
            "按「拦截什么」分工；D2 函数已实现但主链 0 处调用")

    cards = [
        ("D1 车身掩码", "LUT 构建期剔除", "自车车体 / 车标 / 保险杠", "已接入", True,
         "矩形占位：侧视挖掉 76.6%"),
        ("D3 FoE 锥", "平移方向 8° 锥内剔除", "前向拉丝幽灵点", "已接入", True,
         "纯代数，无开方无三角函数"),
        ("D4 交叉验证", "同体素残差 τ 自适应", "动态拖影 / 错配", "已接入（默认关）", True,
         "τ 随距离自适应（≥50 mm）"),
        ("σ 硬截断", "σ>80 cm 直接丢弃", "锁错周期的远端点", "已接入", True,
         "实测单野值即可造一个体素"),
        ("D2 双向一致性", "前后向视差残差加权", "遮挡断裂带伪视差", "未接入", False,
         "函数已有，未接主链"),
    ]
    for i, (name, mech, kill, status, on, note) in enumerate(cards):
        x = 24 + i * 208
        fill = FILL_GREY if not on else (FILL_BLUE if i % 2 == 0 else FILL_GREEN)
        stroke = "#b9c4cf" if not on else (ACC if i % 2 == 0 else GREEN)
        f.rect(x, 96, 196, 292, fill, stroke, 8, 1.4 if on else 1.2)
        if not on:
            f.rect(x, 96, 196, 292, "none", stroke, 8, 1.2, "6 4")
        f.text(x + 14, 126, name, 14.5, INK if on else FAINT, weight="600")
        f.pill(x + 14, 138, 128 if on else 96, 24, status, WHITE, stroke,
               stroke if on else FAINT, 10.5)
        f.text(x + 14, 190, "机制", 11.5, MUTED, weight="600")
        f.text(x + 14, 208, mech, 11.2, "#33475b")
        f.text(x + 14, 240, "拦截", 11.5, MUTED, weight="600")
        f.text(x + 14, 258, kill, 11.2, "#33475b")
        f.text(x + 14, 292, "备注", 11.5, MUTED, weight="600")
        words = note.split("：")
        f.text(x + 14, 310, words[0][:14], 10.6, "#5b6b7c")
        if len(words) > 1:
            f.text(x + 14, 326, words[1][:14], 10.6, "#5b6b7c")
        f.text(x + 14, 362, "↓ 进入体素前", 10.8, stroke, weight="600")

    f.rect(24, 410, W - 48, 70, FILL_AMBER, "#d8b271", 8, 1.1)
    f.text(40, 434, "体素入口的四道闸门", 13.5, AMBER, weight="600")
    f.text(40, 456, "σ 截断先按「测量可信度」筛，D4 再按「与已有观测的一致性」筛；"
                    "两者都不能替代精度验收。", 12, "#6b4a10")
    f.text(40, 474, "D1 是唯一在 LUT 期生效的防御（零运行时代价），其余都在逐点循环里。", 12, "#6b4a10")

    f.note(24, 502, "D2 的缺失意味着遮挡断裂带没有专门防护；NEW_MOD §0.1 建议接 SGBM 的 disp12MaxDiff 作为轻量替代。")
    f.save(OUT / "fig08_defense.svg")
