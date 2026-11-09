#!/usr/bin/env python3
"""精简 / 性能 / 测试 / 进展 / 完整度 / 局限 / 时间线。"""
from __future__ import annotations

import math

from svgkit import (ACC, AMBER, FAINT, FILL_AMBER, FILL_BLUE, FILL_GREEN,
                    FILL_GREY, FILL_PURPLE, FILL_RED, GREEN, INK, MUTED,
                    PURPLE, RED, TEAL, WHITE, Fig)

from figures import OUT


def fig_dvf():
    W, H = 1080, 430
    f = Fig(W, H)
    f.title("五阶段稠密体素精简：2,077 → 1,352 点",
            "全部整数 / 位运算，无除法无三角函数；实测单帧（hanqtech 帧 164，DVF 输出 1,352）")

    stages = [
        ("① AABB + FoE", "ROI 裁剪 + 方向锥", "2,077 → 1,534 → 1,361"),
        ("② 柱体位图", "每 cell 一个 uint32", "1,361"),
        ("③ 地面剥离", "CTZ32 找最低层", "1,356"),
        ("④ 极坐标外壳", "720 扇区 · 30 cm 壳", "1,352"),
        ("⑤ 配额熔断", "近场优先 · ≤15,000", "1,352"),
    ]
    x0, y0, cw, gap = 24, 150, 196, 10
    for i, (name, desc, cnt) in enumerate(stages):
        x = x0 + i * (cw + gap)
        tip = 18
        if i == 0:
            pts = [(x, y0), (x + cw - tip, y0), (x + cw, y0 + 70),
                   (x + cw - tip, y0 + 140), (x, y0 + 140)]
        else:
            pts = [(x, y0), (x + cw - tip, y0), (x + cw, y0 + 70),
                   (x + cw - tip, y0 + 140), (x, y0 + 140), (x + tip, y0 + 70)]
        f.poly(pts, FILL_AMBER if i < 4 else FILL_GREEN,
               "#c99a55" if i < 4 else GREEN, 1.4)
        cx = x + cw / 2 + (8 if i else 0)
        f.text(cx, y0 + 34, name, 13.5, "#6b4a10" if i < 4 else GREEN, "middle", "600")
        f.text(cx, y0 + 58, desc, 10.8, "#6b4a10" if i < 4 else "#1e5a30", "middle")
        f.text(cx, y0 + 108, cnt, 11.5, INK, "middle", "600")

    f.pill(24, 96, 150, 30, "输入 2,077 点", WHITE, ACC, ACC, 12.5)
    f.arrow(99, 128, 99, 148, ACC, marker="acc")
    f.text(178, 116, "体素表导出点（NFVoxelEntry）", 11.5, MUTED)

    f.rect(24, 318, 512, 76, FILL_GREEN, GREEN, 7, 1.1)
    f.text(40, 342, "实测产出", 13, GREEN, weight="600")
    f.text(40, 364, "1,352 点 / 1,352 个底面柱；各阶段只剔除 2.3%（1534→1352）", 11.5, "#1e5a30")
    f.text(40, 382, "说明当前真实点云本身已很稀疏，精简器的价值在稠密场景", 11.5, "#1e5a30")

    f.rect(552, 318, 504, 76, FILL_BLUE, ACC, 7, 1.1)
    f.text(568, 342, "性能（probe_dvf_perf，15 万合成点）", 13, ACC, weight="600")
    f.text(568, 364, "1.83 ms，输出 2,070 点，截断 0", 11.5, "#33475b")
    f.text(568, 382, "墙钟随负载波动：此前同一探针记录为 1.06 ms", 11.5, "#33475b")

    f.note(24, 416, "阶段 4 必须回到原始点数组取坐标：位图只记「层是否存在」，恢复不出 sqrt(x²+y²)。")
    f.save(OUT / "fig09_dvf.svg")


def fig_perf():
    W, H = 1080, 540
    f = Fig(W, H)
    f.title("实测延时的两种口径", "「有效帧」是真正跑完匹配的那一帧；「摊薄」把冻结帧也算进平均")

    rows = [
        ("FORWARD 有效帧", 11188, RED, "2 路前向"),
        ("FORWARD 3 帧摊薄", 3730, AMBER, "11.2s+2ms×2"),
        ("STEREO 有效帧", 104, GREEN, "4 路 SGBM"),
        ("STEREO 3 帧摊薄", 36, TEAL, "104ms+2ms"),
        ("合成 STEREO", 75, ACC, "合成场景"),
        ("DBSCAN 150k", 54, PURPLE, "均匀 150k"),
        ("DVF 150k", 1.83, "#8e6bb0", "五阶段"),
    ]
    x0, y0, pw, rh = 270, 92, 600, 48
    log_max = math.log10(20000)
    f.text(x0, 82, "耗时（ms，对数轴）", 13, MUTED)
    for v in (1, 10, 100, 1000, 10000):
        px = x0 + math.log10(v) / log_max * pw
        f.line(px, y0, px, y0 + rh * len(rows), "#e5eaf0", 0.9, "3 3")
        f.text(px, y0 + rh * len(rows) + 18, str(v), 12, MUTED, "middle")

    for i, (name, v, color, note) in enumerate(rows):
        y = y0 + i * rh
        f.text(x0 - 14, y + 22, name, 13, INK, "end", "600")
        bw = max(3, math.log10(v) / log_max * pw)
        f.rect(x0, y + 9, bw, 24, color, None, 3, 0.9)
        f.text(x0 + bw + 8, y + 26, f"{v:,} ms" if v >= 10 else f"{v} ms", 12, color, weight="600")
        f.text(x0 + pw + 70, y + 26, note, 10, FAINT)

    px = x0 + math.log10(200) / log_max * pw
    f.line(px, y0 - 10, px, y0 + rh * len(rows) + 4, RED, 1.4, "5 4")
    f.text(px + 6, y0 - 14, "5 Hz 预算 200 ms", 12, RED, weight="600")

    f.rect(24, 464, W - 48, 46, FILL_RED, RED, 7, 1.1)
    f.text(40, 492, "FORWARD 超预算 56×；STEREO 有效帧 104 ms，在 200 ms 内但余量只剩 48%。",
           12.5, "#7a3b33", weight="600")
    f.note(24, 524, "所有数值同一台机器、Release、同日实测；未做负载隔离，绝对值为量级参考。")
    f.save(OUT / "fig10_perf.svg")


def fig_test():
    W, H = 1080, 430
    f = Fig(W, H)
    f.title("测试现状：20 / 21 通过", "唯一失败是端到端深度精度；其余 177 个用例、2,343 次断言全部通过")

    cards = [
        ("21", "ctest 目标", ACC), ("20", "通过", GREEN),
        ("177", "用例", ACC), ("2,343", "断言", PURPLE),
    ]
    for i, (v, label, color) in enumerate(cards):
        x = 24 + i * 262
        f.rect(x, 90, 246, 96, WHITE, color, 8, 1.3)
        f.text(x + 20, 140, v, 34, color, weight="600")
        f.text(x + 20, 168, label, 13, MUTED)

    bx, by, bw = 24, 220, 1032
    f.rect(bx, by, bw, 40, FILL_GREY, None, 4)
    f.rect(bx, by, bw * 20 / 21, 40, GREEN, None, 4)
    f.text(bx + 12, by + 26, "20 个目标通过", 13, WHITE, weight="600")
    f.rect(bx + bw * 20 / 21, by, bw / 21, 40, RED, None, 4)
    f.text(bx + bw, by + 56, "失败 1 项：depth_accuracy", 11, RED, "end", "600")

    f.rect(24, 286, 1032, 80, FILL_RED, RED, 7, 1.1)
    f.text(40, 312, "depth_accuracy：4/4 用例失败，16/21 条断言失败", 13.5, RED, weight="600")
    f.text(40, 334, "侧行 5 km/h 分段中位 291 / 256 / 433 / 896 mm（目标 150 / 200 / 320 / 320）", 12, "#7a3b33")
    f.text(40, 354, "直行 5 km/h 为 399 / 1,101 / 548 / 477 mm；45° 偏转与 15 km/h 同样全段未达。", 12, "#7a3b33")

    f.note(24, 394, "断言总数 = 20 个目标的 2,313 次 + depth_accuracy 的 21 次。"
                    "「通过」不等于需求满足：D2 未接入、精度未达标都不体现在失败数里。")
    f.save(OUT / "fig11_test.svg")


def fig_status():
    W, H = 1080, 790
    f = Fig(W, H)
    f.title("开发进展看板", "2026-11-09 复核；「部分」= 代码已实现且单测通过，但主链未接入或依赖外部数据")

    items = [
        ("相机模型 KB / 径向多项式", "vcam 交叉校验 0.00008 px", "done"),
        ("虚拟视口 LUT", "4 路 150°×1024，f=137.19", "done"),
        ("SGBM 引擎", "确定性、容量断言、无效语义", "done"),
        ("前向径向解析测距", "合成单测 6/6；真实直行在用", "done"),
        ("射线三角化", "闭式解，float32 稳定", "done"),
        ("体素 Hash + CMA", "开放寻址、σ 取最小", "done"),
        ("σ 传播 + 80 cm 截断", "1 m 2 cm → 8 m 85 cm", "done"),
        ("D4 交叉验证", "已接主链，默认关闭", "done"),
        ("双基线状态机", "278→556 mm 晋升，默认关闭", "done"),
        ("DVF 五阶段精简", "1,352 点 / 1.83 ms@150k", "done"),
        ("网格哈希 + DBSCAN", "150k 均匀 54 ms", "done"),
        ("动静分离", "点级速度 + 簇级投票", "done"),
        ("ZMQ 零拷贝", "收发双向，含负对照", "done"),
        ("跨平台构建", "macOS / Linux / aarch64", "done"),
        ("D1 车身掩码", "矩形占位，侧视挖 77%", "part"),
        ("六视口规划", "规划表有，主链仍 4 视口", "part"),
        ("D2 双向一致性", "函数有，主链 0 处调用", "part"),
        ("aarch64 上板", "产物已出，未在真机运行", "part"),
        ("精度验收", "4/4 段未达，唯一失败测试", "todo"),
        ("FORWARD 性能", "有效帧 11.2 s，超预算 56×", "todo"),
        ("地面以下点 -498 mm", "真实数据可复现，未查明", "todo"),
        ("在线自校准", "方案已定，未实现", "todo"),
        ("NPU 后端", "模型引擎仍 UNSUPPORTED", "todo"),
        ("深度真值", "数据集只有 ego 与图像", "todo"),
    ]
    col = {"done": (FILL_GREEN, GREEN, "已完成"),
           "part": (FILL_AMBER, AMBER, "部分"),
           "todo": (FILL_RED, RED, "未做")}
    lx = 760
    for k in ("done", "part", "todo"):
        fill, bc, lb = col[k]
        f.rect(lx, 24, 14, 14, fill, bc, 3, 1.3)
        f.text(lx + 19, 36, lb, 12, MUTED)
        lx += 92

    bw, bh, gap = 505, 48, 8
    for i, (name, desc, st) in enumerate(items):
        fill, bc, _ = col[st]
        cx = 24 + (i // 12) * (bw + 20)
        cy = 76 + (i % 12) * (bh + gap)
        f.rect(cx, cy, bw, bh, fill, bc, 5, 1.2)
        f.text(cx + 12, cy + 20, name, 13, INK, weight="600")
        f.text(cx + 12, cy + 38, desc, 10.8, "#5b6b7c")

    f.note(24, 776, "统计口径：24 项中 14 项完成、4 项部分、6 项未做。「部分」最容易被误报为完成 —— 函数在、测试绿、系统不调用。")
    f.save(OUT / "fig12_status.svg")


def fig_complete():
    W, H = 1080, 590
    f = Fig(W, H)
    f.title("功能完成度（工程自评）", "口径：代码 + 单测 100%；主链接入 80%；真实数据验证 60%；精度验收 30%；非目标项 0%")

    rows = [
        ("相机与几何", 100, "KB / 径向多项式；vcam 交叉校验"),
        ("视口与 LUT", 90, "4 路 150°；6 视口规划未全接"),
        ("视差引擎", 95, "SGBM + 模型接口；模型未实现"),
        ("三角化 / 前向", 95, "两条路径均通；前向性能未优化"),
        ("体素与融合", 100, "Hash + CMA + zMap 全通"),
        ("防御体系", 80, "D1/D3/D4/σ；D2 未接"),
        ("基线状态机", 90, "双基线 + 里程计；真实数据默认未开"),
        ("聚类与动静", 85, "算法通过；真实点云太稀"),
        ("IO / ZMQ", 100, "零拷贝收发 + 协议校验"),
        ("测试", 90, "20/21；精度用例失败"),
        ("精度验收", 30, "4 段全未达；场景纹理缺陷"),
        ("真机部署", 20, "aarch64 产物未上板"),
        ("在线自校准", 0, "方案已定，未实现"),
    ]
    for i, (name, pct, note) in enumerate(rows):
        y = 96 + i * 34
        color = GREEN if pct >= 90 else (AMBER if pct >= 60 else RED)
        f.text(24, y + 16, name, 13, INK, weight="600")
        f.rect(150, y, 440, 20, "#eef1f4", None, 3)
        if pct:
            f.rect(150, y, 440 * pct / 100, 20, color, None, 3)
        f.text(620, y + 15, f"{pct}%", 12.5, color, "end", "600")
        f.text(642, y + 15, note, 11, MUTED)

    f.rect(24, 542, W - 48, 30, FILL_BLUE, ACC, 6, 1)
    f.text(40, 562, "整体：链路已打通，精度未证实。完成度不是加权总分 —— 精度验收决定能否交付。",
           12, "#33475b", weight="600")
    f.save(OUT / "fig13_complete.svg")


def fig_limits():
    W, H = 1080, 640
    f = Fig(W, H)
    f.title("已知局限：影响 × 证据 × 对策", "按影响排序；「证据」列是本次实测值，不是估计")

    # (局限, 影响, 证据1, 证据2, 对策1, 对策2)
    rows = [
        ("精度未证实", "高", "侧行 291/256/433/896 mm", "目标 150/200/320/320 mm",
         "重建随机相位场景（P0）", "当前纹理让 SGBM 锁错周期"),
        ("FORWARD 性能", "高", "有效帧 11.2 s", "5 Hz 预算 200 ms",
         "降采样 / 金字塔 / 并行化", "径向搜索（P1）"),
        ("地面以下点 -498 mm", "高", "z 最小 -498 mm；236/2,077 点", "cam0 占 212，集中在前向",
         "未查明；先加 z>0 断言", "网格外点单独计数"),
        ("双基线不判方向", "中", "直行强制 STEREO", "46,821 次插入 / 0 点",
         "加 σ/FoE 方向判据", "或仅对侧向相机启用"),
        ("车身掩码粗糙", "中", "MVL 76.6% / MVR 77.3%", "画面被判为车身",
         "改用标定式视图掩码", "不能用矩形投影（TODO-1）"),
        ("D2 未接入", "中", "主链 0 处调用", "nf_occlusion_weight 已实现",
         "接 SGBM disp12MaxDiff", "作为轻量替代（NEW_MOD §0.1）"),
        ("无深度真值", "高", "数据集仅 24 图 + 36 json", "有 ego 无点云",
         "需激光/点云真值或高精地图", "否则只能验自洽"),
        ("真机未验证", "高", "aarch64 产物已出", "RK3588 / M57 未运行",
         "上板实测 + INT8 量化", "291 MB 预算待复核"),
        ("薄障碍漏检", "低", "720 扇区 @5 m 弧宽 44 mm", "",
         "<50 mm 立柱依赖融合", "超声波 / 毫米波"),
    ]
    cx = [24, 190, 250, 660]
    heads = ["局限", "影响", "证据（实测）", "对策 / 状态"]
    for i, h in enumerate(heads):
        f.text(cx[i] + 8, 92, h, 12.5, MUTED, weight="600")
    sev = {"高": RED, "中": AMBER, "低": FAINT}
    for r, (name, level, ev1, ev2, fix1, fix2) in enumerate(rows):
        y = 102 + r * 56
        f.rect(24, y, W - 48, 50, WHITE, "#e2e8ee", 5, 1)
        f.rect(24, y, 5, 50, sev[level], None, 2)
        f.text(cx[0] + 12, y + 21, name, 12.5, INK, weight="600")
        f.text(cx[1] + 12, y + 21, level, 12.5, sev[level], weight="600")
        f.text(cx[2] + 8, y + 21, ev1, 10.9, "#33475b")
        if ev2:
            f.text(cx[2] + 8, y + 38, ev2, 10.9, "#33475b")
        f.text(cx[3] + 8, y + 21, fix1, 10.9, "#33475b")
        if fix2:
            f.text(cx[3] + 8, y + 38, fix2, 10.9, "#33475b")

    f.note(24, 622, "影响「高」= 直接阻断交付或真机运行；「中」= 特定工况失效；「低」= 已知边界。")
    f.save(OUT / "fig14_limits.svg")


def fig_timeline():
    W, H = 1080, 420
    f = Fig(W, H)
    f.title("研发时间线：从首次入库到真实数据闭环", "42 次提交（2026-11-01 → 11-03）+ 11-09 本轮独立复核")

    bands = [
        (60, 215, "骨架 / 几何 / 重命名", FILL_BLUE, ACC),
        (283, 315, "视口决策 + 防御 + 聚类", FILL_GREEN, GREEN),
        (606, 220, "vcam 统一 + 真实数据", FILL_AMBER, AMBER),
        (834, 200, "文档 v0.3 + 复核", FILL_PURPLE, PURPLE),
    ]
    for x, w, label, fill, stroke in bands:
        f.rect(x, 88, w, 34, fill, stroke, 6, 1.1)
        f.text(x + w / 2, 111, label, 12, stroke, "middle", "600")

    f.line(60, 230, 1030, 230, MUTED, 2)

    miles = [
        ("11-01", "首次入库", "NFOCC 重命名 / 清理", 80),
        ("11-02", "视口 150°×1024", "精度口径修正 + 探针", 300),
        ("11-02", "D1–D4 / 聚类", "纯网格哈希 + 双基线", 460),
        ("11-03", "真实数据 t_cam", "修正前进位移被放到横向", 620),
        ("11-03", "文档 v0.3", "vcam 唯一投影路径", 780),
        ("11-09", "本轮复核", "6 项失败→20/21；性能重测", 940),
    ]
    for i, (date, title, desc, x) in enumerate(miles):
        f.circle(x, 230, 7, WHITE, ACC, 2)
        up = (i % 2 == 0)
        y1 = 190 if up else 246
        y2 = 166 if up else 270
        f.line(x, 230, x, y1, ACC, 1.2)
        f.text(x, y2 - 8, date, 11.5, ACC, "middle", "600")
        f.text(x, y2 + 10, title, 12.5, INK, "middle", "600")
        f.text(x, y2 + 27, desc, 10.5, MUTED, "middle")

    f.rect(24, 330, W - 48, 58, FILL_RED, RED, 7, 1.1)
    f.text(40, 354, "11-09 复核的关键差异", 13, RED, weight="600")
    f.text(40, 374, "文档写 ctest 20/21，但本机旧 build 目录是 NF_WITH_VCAM=OFF："
                    "实测 15/21，5 项投影相关测试失败。", 11.5, "#7a3b33")

    f.note(24, 408, "时间线只取 git 提交与本次实测；文档 mtime 不作为研发进度依据。")
    f.save(OUT / "fig15_timeline.svg")
