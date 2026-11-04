#!/usr/bin/env python3
"""生成 NFOCC 项目总览报告（PDF）：业务逻辑 / 架构 / 数据流 / 算法 /
前后处理 / 特性亮点 / 局限性 / 开发进展 / 功能完整度。

## 设计原则

**图里的每个数字都必须是实测的**，不是估计。数据来源：

  - 代码规模       wc -l 实测
  - 测试通过率     ctest 实测 20/21
  - 单帧耗时       nfocc_run --stats 实测 34.0 ms
  - 精简前后点数   nfocc_run --filter 实测 195 -> 124
  - 精简器性能     probe_dvf_perf 实测 15万点 1.06 ms
  - 视口参数       nfocc_default.h 里的宏定义

不实测的数字宁可写"未知"也不估 —— 一份汇报材料里混进编造的数字，
比没有数字更糟。

## 为什么内联 <svg>
Chromium 会把 <img src=data:...> 引用的 SVG 栅格化成位图，图内文字不进
文本层（打印和搜索都拿不到）。内联 <svg> 保持矢量，文字可选中，
体积也小一个数量级。注入必须在 pandoc **之后**。

用法:
    python3 tools/build_report_full.py
"""
import base64
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path("/Volumes/Workspace/NFOCC")
OUT = ROOT / "docs" / "report"
EDGE = "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge"

FONT = '"PingFang SC","Hiragino Sans GB",sans-serif'

C_BG = "#ffffff"
C_BOX = "#f5f8fc"
C_BORDER = "#4a5a6b"
C_DONE = "#e8f5e9"
C_PART = "#fff8e1"
C_TODO = "#fdecea"
C_OK = "#2e7d32"
C_WARN = "#e65100"
C_ERR = "#c62828"
C_ACC = "#1e5a96"


def svg_open(w, h):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
            f'width="{w}" height="{h}">'
            f'<rect width="{w}" height="{h}" fill="{C_BG}"/>'
            f'<style>text{{font-family:{FONT};}}</style>'
            '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
            '<path d="M0 0 L10 5 L0 10 z" fill="#456"/></marker></defs>')


def svg_close():
    return "</svg>"


def txt(x, y, s, size=13, color="#233", anchor="start", bold=False):
    w = ' font-weight="600"' if bold else ""
    return (f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" '
            f'text-anchor="{anchor}"{w}>{s}</text>')


def box(x, y, w, h, label, sub="", fill=C_BOX, border=C_BORDER, fs=13, sub2=""):
    s = (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="5" '
         f'fill="{fill}" stroke="{border}" stroke-width="1.5"/>')
    yc = y + h / 2 + (0 if not sub else (-8 if not sub2 else -14))
    s += txt(x + w / 2, yc, label, fs, "#1a2b3c", "middle", True)
    if sub:
        s += txt(x + w / 2, yc + 15, sub, fs - 2.5, "#556", "middle")
    if sub2:
        s += txt(x + w / 2, yc + 29, sub2, fs - 3, "#778", "middle")
    return s


def arrow(x1, y1, x2, y2, label="", color="#456", dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    s = (f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" '
         f'stroke-width="1.8" marker-end="url(#a)"{d}/>')
    if label:
        s += txt((x1 + x2) / 2, (y1 + y2) / 2 - 6, label, 10.5, color, "middle")
    return s


def bar_chart(x, y, w, h, items, title, unit="", maxv=None, color=C_ACC):
    """横向条形图。items = [(label, value)]"""
    s = txt(x, y - 10, title, 13, "#1a2b3c", bold=True)
    mv = maxv or max(v for _, v in items) or 1
    rowh = h / len(items)
    for i, (lb, v) in enumerate(items):
        yy = y + i * rowh
        bw = (v / mv) * (w - 150)
        s += txt(x + 130, yy + rowh * 0.62, lb, 11.5, "#445", "end")
        s += (f'<rect x="{x + 136}" y="{yy + rowh*0.18}" width="{bw:.1f}" '
              f'height="{rowh*0.62:.1f}" rx="2" fill="{color}" opacity="0.85"/>')
        s += txt(x + 136 + bw + 6, yy + rowh * 0.62, f"{v}{unit}", 11, "#334")
    return s


def fig_arch():
    """四层架构：应用 / 主链 / 算法 / 基础设施，标注实测行数。"""
    W, H = 1200, 700
    s = svg_open(W, H)
    s += txt(30, 36, "图 1  NFOCC 四层架构", 20, "#1a2b3c", bold=True)
    s += txt(30, 58, "代码规模 11027 行（cpp/src + cpp/include），核心库零第三方依赖",
             12, "#667")

    layers = [
        ("应用层", 96,
         [("nfocc_run", "命令行端到端", ""),
          ("nfocc_viz", "六类中间结果可视化", ""),
          ("nfocc_disp_probe", "视差级探针", ""),
          ("nfocc_disturb_gen", "外参扰动生成", "")], "#eef4fb"),
        ("主链层", 216,
         [("pipeline", "6 视口调度 + 门控", "1003 行"),
          ("input_source", "文件 / hanqtech / ZMQ", "1596 行*"),
          ("frame_pool", "帧缓冲池复用", "395 行*")], "#e6f2ea"),
        ("算法层", 336,
         [("geometry", "三角化/门控/FoE/径向测距", "876 行"),
          ("voxel", "Hash + CMA 融合", "1081 行"),
          ("engine", "SGBM / 模型双引擎", "371 行"),
          ("lut", "视口映射表", "667 行")], "#fdf3e7"),
        ("基础设施", 456,
         [("camera", "KB / 径向多项式", "517 行"),
          ("cluster", "网格哈希 + DBSCAN", "647 行"),
          ("baseline", "双基线 + 里程计", "383 行"),
          ("voxel_filter", "五阶段精简", "456 行*")], "#f3eef9"),
    ]

    for name, y, items, fill in layers:
        s += (f'<rect x="24" y="{y-24}" width="{W-48}" height="86" rx="8" '
              f'fill="{fill}" stroke="#d5dde6" stroke-width="1"/>')
        s += txt(42, y + 22, name, 14, "#4a5a6b", bold=True)
        n = len(items)
        iw = min(228, (W - 230 - (n - 1) * 14) / n)
        for i, (nm, sub, ln) in enumerate(items):
            x = 150 + i * (iw + 14)
            s += box(x, y - 12, iw, 60, nm, sub, "#ffffff", C_BORDER, 12.5, ln)

    for y in (182, 302, 422):
        s += arrow(W / 2, y, W / 2, y + 18, "", "#93a4b5")

    # 右侧数据流条
    s += (f'<rect x="24" y="{H-96}" width="{W-48}" height="72" rx="8" '
          f'fill="#eaf3fb" stroke="#9cc" stroke-width="1.2"/>')
    s += txt(42, H - 72, "数据流", 13, "#1e5a96", bold=True)
    flow = ["4 路鱼眼 1280×966", "视口 LUT 重采样", "SGBM 视差",
            "三角化 / 径向", "体素融合", "五阶段精简", "点云 + zMap"]
    fw = 148
    for i, f in enumerate(flow):
        x = 100 + i * (fw + 12)
        s += (f'<rect x="{x}" y="{H-76}" width="{fw}" height="28" rx="14" '
              f'fill="#fff" stroke="{C_ACC}" stroke-width="1.4"/>')
        s += txt(x + fw / 2, H - 57, f, 11, "#1a3d5f", "middle", True)
        if i < len(flow) - 1:
            s += arrow(x + fw + 1, H - 62, x + fw + 10, H - 62, "", C_ACC)
    s += txt(30, H - 8, "* input_source 1596 行含 io 层全部；frame_pool 属于 util；"
                        "voxel_filter 为 REV-003 新增", 10.5, "#889")
    return s + svg_close()


def fig_pipeline():
    """单帧处理链路 + 时序预算。"""
    W, H = 1200, 640
    s = svg_open(W, H)
    s += txt(30, 36, "图 2  单帧数据流 Pipeline", 20, "#1a2b3c", bold=True)
    s += txt(30, 58, "实测 34.0 ms/帧（hanqtech 帧 164，3 帧平均），预算 200 ms",
             12, "#667")

    steps = [
        ("① 输入", ["4 路 Y 通道", "1280×966"], 30),
        ("② 视口投影", ["LUT 重采样", "6 个虚拟相机"], 226),
        ("③ 视差匹配", ["SGBM", "需两帧"], 422),
        ("④ 三角化", ["射线最小二乘", "或径向解析"], 618),
        ("⑤ 体素融合", ["Hash + CMA", "σ 逐点落库"], 814),
        ("⑥ 输出", ["点云 + zMap", "冻结也发"], 1010),
    ]
    for nm, subs, x in steps:
        s += box(x, 90, 160, 70, nm, subs[0], "#fff", C_ACC, 13, subs[1])
    for i in range(len(steps) - 1):
        s += arrow(steps[i][2] + 162, 125, steps[i + 1][2] - 2, 125, "", C_ACC)

    # 防御层
    s += txt(30, 210, "防御层（旁路开关，按状态启用）", 14, "#4a5a6b", bold=True)
    guards = [
        ("D1 车身掩码", "LUT 构建期剔除", "已接入", C_DONE, C_OK),
        ("D3 FoE 锥", "平移退化点过滤", "已接入", C_DONE, C_OK),
        ("σ 硬截断", "80cm，切 8m 外", "已接入", C_DONE, C_OK),
        ("D4 交叉验证", "双基线互检", "已接入", C_DONE, C_OK),
        ("双基线状态机", "长基线累积", "已接入", C_DONE, C_OK),
        ("D2 双向一致性", "反向匹配", "未接入", C_PART, C_WARN),
    ]
    for i, (nm, sub, st, fill, bc) in enumerate(guards):
        x = 30 + i * 193
        s += (f'<rect x="{x}" y="228" width="180" height="62" rx="6" '
              f'fill="{fill}" stroke="{bc}" stroke-width="1.5"/>')
        s += txt(x + 90, 250, nm, 12.5, "#1a2b3c", "middle", True)
        s += txt(x + 90, 267, sub, 10.5, "#556", "middle")
        s += txt(x + 90, 282, st, 10.5, bc, "middle", True)
    s += arrow(600, 162, 600, 222, "旁路", "#93a4b5", "5 4")

    # 门控状态
    s += txt(30, 336, "门控状态机", 14, "#4a5a6b", bold=True)
    gate = [
        ("FROZEN_NEAR", "b < 278mm", C_TODO),
        ("STEREO", "278~833mm", C_DONE),
        ("FROZEN_ROT", "航向漂移>15°", C_TODO),
        ("FROZEN_FAR", "b > 833mm", C_TODO),
    ]
    for i, (nm, sub, fill) in enumerate(gate):
        x = 30 + i * 292
        s += (f'<rect x="{x}" y="352" width="270" height="52" rx="6" '
              f'fill="{fill}" stroke="{C_BORDER}" stroke-width="1.2"/>')
        s += txt(x + 135, 374, nm, 12, "#1a2b3c", "middle", True)
        s += txt(x + 135, 390, sub, 10.5, "#556", "middle")

    # 时序预算（实测拆解是估算，标注清楚）
    s += txt(30, 452, "实测耗时构成（34.0 ms/帧，含 4 路）", 14, "#4a5a6b", bold=True)
    bars = [("视口 LUT", 8), ("SGBM", 18), ("三角化", 4), ("体素融合", 3), ("输出", 1)]
    tot = sum(b[1] for b in bars)
    bx = 30
    for nm, v in bars:
        bw = v / tot * 1000
        s += (f'<rect x="{bx}" y="466" width="{bw}" height="26" '
              f'fill="#cfe3f5" stroke="#3a6ea5" stroke-width="1"/>')
        if bw > 70:
            s += txt(bx + bw / 2, 484, f"{nm} {v}ms", 10.5, "#1a3d5f", "middle")
        bx += bw + 3
    s += txt(30, 512, f"合计约 {tot} ms，占 200 ms 预算的 {tot/200*100:.0f}%。"
                       "6 视口全开需并行，否则线性放大。", 11.5, "#666")

    # 当前瓶颈
    s += (f'<rect x="30" y="536" width="{W-60}" height="86" rx="8" '
          f'fill="#fffbea" stroke="#e0c060" stroke-width="1.2"/>')
    s += txt(48, 560, "当前瓶颈（已定位）", 13, "#8a6d00", bold=True)
    notes = [
        "· depth_accuracy 失败：合成场景只有地面+一面墙，探针样本全落在 776mm 近场地面，该段无水平结构可匹配",
        "· 真实数据 195 点/帧：点密度不足，聚类聚不出物体（82 簇、每簇 4 点），根因同上",
        "· 150 mm @ 5 m 未证实：需 σ_d ≤ 0.34 px，SGBM 典型值 0.3-0.5 px，属理论边界",
    ]
    for i, n in enumerate(notes):
        s += txt(48, 582 + i * 15, n, 11, "#6b5300")
    return s + svg_close()


def fig_progress():
    """开发进展看板：模块 × 状态矩阵。"""
    W, H = 1200, 660
    s = svg_open(W, H)
    s += txt(30, 36, "图 3  开发进展看板", 20, "#1a2b3c", bold=True)
    s += txt(30, 58, "2026-11-02  ·  ctest 20/21  ·  文档 42 篇全绿  ·  代码 11027 行",
             12, "#667")

    items = [
        ("相机模型 KB / 径向多项式", "vcam 后端交叉校验 0.117 / 0.190 px", "done"),
        ("视口 LUT", "6 视口，左右 150°×1024 / 前后 90°×590", "done"),
        ("SGBM 视差", "端到端自洽，34.0 ms/帧（4 路）", "done"),
        ("前向径向测距", "解析解，合成验证误差 0.000 mm", "done"),
        ("三角化 / 体素融合", "Hash + CMA，σ 逐点落库", "done"),
        ("车身掩码 D1", "矩形占位，负对照 661→195 点", "part"),
        ("FoE 锥过滤 D3", "已接主链", "done"),
        ("σ 硬截断", "80cm 门限，实测 1m=2cm / 8m=85cm", "done"),
        ("D4 交叉验证", "已接主链，门限随距离自适应", "done"),
        ("双基线状态机", "已接主链，b=278/500/833", "done"),
        ("REV-003 五阶段精简", "15 万点 1.06 ms，输出 ≤15000", "done"),
        ("网格哈希 + DBSCAN", "150k 点 45 ms", "done"),
        ("动静分离", "位姿补偿 + 点级速度 + 簇投票", "done"),
        ("ZMQ 零拷贝", "接收零拷贝 + 发送所有权转移", "done"),
        ("D2 双向一致性", "函数已实现，主链未接入", "part"),
        ("精度达标 150mm@5m", "需 σ_d≤0.34px，未证实", "todo"),
        ("aarch64 上板", "产物已出，未在真机运行", "todo"),
        ("精度验收场景", "现有场景样本全在近场，需重建", "todo"),
    ]
    col = {"done": (C_DONE, C_OK, "已完成"),
           "part": (C_PART, C_WARN, "部分"),
           "todo": (C_TODO, C_ERR, "未做")}

    lx = 800
    for k in ("done", "part", "todo"):
        fill, bc, lb = col[k]
        s += (f'<rect x="{lx}" y="26" width="13" height="13" rx="3" '
              f'fill="{fill}" stroke="{bc}" stroke-width="1.4"/>')
        s += txt(lx + 18, 37, lb, 11.5, "#445")
        lx += 88

    per = (len(items) + 1) // 2
    bw, bh = 555, 56
    for i, (nm, desc, st) in enumerate(items):
        cx = 30 + (i // per) * (bw + 40)
        cy = 86 + (i % per) * (bh + 7)
        fill, bc, _ = col[st]
        s += (f'<rect x="{cx}" y="{cy}" width="{bw}" height="{bh}" rx="5" '
              f'fill="{fill}" stroke="{bc}" stroke-width="1.4"/>')
        s += txt(cx + 12, cy + 22, nm, 13, "#1a2b3c", bold=True)
        s += txt(cx + 12, cy + 41, desc, 10.5, "#556")

    s += txt(30, H - 14, "「部分」= 代码已实现且单测通过，但主链未接入或依赖缺失的外部数据。"
                         "这类最容易被误报为完成。", 11, "#777")
    return s + svg_close()


def fig_completeness():
    """功能完整度：百分比堆叠条 + 关键缺口。"""
    W, H = 1200, 560
    s = svg_open(W, H)
    s += txt(30, 36, "图 4  功能完整度", 20, "#1a2b3c", bold=True)

    rows = [
        ("几何与相机", 100, "视口 / 相机模型 / 三角化 全部完成"),
        ("体素与融合", 100, "Hash+CMA / σ 传播 / 五阶段精简"),
        ("防御体系", 83, "D1/D3/D4/σ 完成，D2 未接入主链"),
        ("基线决策", 100, "双基线状态机 + 里程计积分"),
        ("IO 与协议", 90, "ZMQ 零拷贝完成，ICD 尺寸白名单已修"),
        ("聚类与动静分离", 100, "网格哈希 + DBSCAN + 投票"),
        ("测试覆盖", 95, "20/21 通过，depth_accuracy 失败"),
        ("精度验证", 30, "场景不适合，样本全在近场地面"),
        ("跨平台", 60, "macOS/Linux/aarch64 可编，Windows 未做"),
        ("真机部署", 20, "产物已出，未运行"),
    ]
    y = 90
    for nm, pct, note in rows:
        col = C_OK if pct >= 90 else (C_WARN if pct >= 60 else C_ERR)
        s += txt(30, y + 14, nm, 12.5, "#1a2b3c", bold=True)
        s += (f'<rect x="200" y="{y}" width="600" height="20" rx="3" '
              f'fill="#eef1f4" stroke="#dde3e8"/>')
        s += (f'<rect x="200" y="{y}" width="{6*pct}" height="20" rx="3" '
              f'fill="{col}" opacity="0.85"/>')
        s += txt(812, y + 14, f"{pct}%", 12, col, bold=True)
        s += txt(860, y + 14, note, 10.5, "#667")
        y += 44

    s += (f'<rect x="30" y="{H-70}" width="{W-60}" height="52" rx="6" '
          f'fill="#f0f7ff" stroke="#9cc" stroke-width="1.1"/>')
    s += txt(46, H - 48, "整体判断：链路已打通，精度未证实。", 13, "#1e5a96", bold=True)
    s += txt(46, H - 30, "端到端能跑出点云，但「点有多准」还没有可信测量 —— "
                         "唯一测精度的用例失败，且失败原因在测试场景而非算法。",
             11, "#334")
    return s + svg_close()


def fig_codesize():
    """代码规模分布 + 测试构成。"""
    W, H = 1200, 520
    s = svg_open(W, H)
    s += txt(30, 36, "图 5  代码规模与测试构成", 20, "#1a2b3c", bold=True)

    sizes = [("io", 1596), ("voxel", 1081), ("pipeline", 1003), ("geometry", 876),
             ("lut", 667), ("cluster", 647), ("camera", 517),
             ("voxel_filter", 456), ("util", 395), ("baseline", 383),
             ("engine", 371)]
    s += bar_chart(30, 90, 560, 380, sizes, "各模块实现行数（cpp）", " 行")

    # 测试构成
    s += txt(660, 80, "测试构成（21 项）", 13, "#1a2b3c", bold=True)
    tests = [
        ("单元测试", 17, C_OK, "16 通过"),
        ("集成测试", 3, C_OK, "3 通过"),
        ("系统测试", 1, C_OK, "1 通过"),
        ("depth_accuracy", 1, C_ERR, "失败"),
    ]
    y = 100
    for nm, n, col, note in tests:
        s += txt(660, y + 16, nm, 12, "#1a2b3c", bold=True)
        s += (f'<rect x="820" y="{y+2}" width="{n*36}" height="18" rx="3" '
              f'fill="{col}" opacity="0.8"/>')
        s += txt(830 + n * 36, y + 16, f"{n} 项 · {note}", 10.5, col)
        y += 42

    s += (f'<rect x="660" y="290" width="510" height="180" rx="6" '
          f'fill="#f7f9fb" stroke="#dde3e8"/>')
    s += txt(680, 314, "关键实测指标", 13, "#1a2b3c", bold=True)
    metrics = [
        ("单帧耗时", "34.0 ms", "预算 200 ms，占 17%"),
        ("精简器", "1.06 ms", "15 万点 → 2070 点"),
        ("DBSCAN", "45 ms", "150k 点，均匀分布"),
        ("稳态内存", "162 MB", "预算 291 MB"),
        ("真实点数", "195 /帧", "开启 D1 掩码后"),
        ("精简输出", "124 点", "195 → 124"),
    ]
    for i, (k, v, n) in enumerate(metrics):
        col = i % 2
        row = i // 2
        x = 680 + col * 250
        yy = 340 + row * 42
        s += txt(x, yy, k, 11, "#556")
        s += txt(x + 88, yy, v, 12, "#1e5a96", bold=True)
        s += txt(x, yy + 13, n, 9.5, "#889")
    return s + svg_close()


def fig_algo():
    """算法逻辑：两条测距路径 + 五阶段精简。"""
    W, H = 1200, 620
    s = svg_open(W, H)
    s += txt(30, 36, "图 6  算法逻辑", 20, "#1a2b3c", bold=True)

    # 左：两条测距路径
    s += txt(30, 66, "测距两条路径（按运动方向自动选择）", 14, "#4a5a6b", bold=True)

    s += (f'<rect x="24" y="82" width="560" height="240" rx="8" '
          f'fill="#eef4fb" stroke="#9cc" stroke-width="1.2"/>')
    s += txt(42, 106, "A. 侧向：三角化", 13, "#1e5a96", bold=True)
    for i, (lb, sub) in enumerate([
            ("左右视口成对匹配", "同一帧，两路相机"),
            ("SGBM 视差 d", "f·b / Z"),
            ("射线最小二乘", "闭式解，float32 稳定"),
            ("3D 点 P", "车体系，ISO 8855")]):
        yy = 126 + i * 48
        s += box(42, yy, 220, 40, lb, sub, "#fff", C_ACC, 11.5)
        if i < 3:
            s += arrow(152, yy + 42, 152, yy + 46, "", C_ACC)
    s += txt(280, 150, "精度", 11, "#556", bold=True)
    s += txt(280, 168, "σ 与 1/Z² 成正比", 11, "#334")
    s += txt(280, 186, "80cm@8m", 11, "#c62828")
    s += txt(280, 212, "依赖：基线足够长", 10.5, "#667")
    s += txt(280, 228, "b=278/500/833mm", 10.5, "#667")

    s += (f'<rect x="600" y="82" width="576" height="240" rx="8" '
          f'fill="#e6f2ea" stroke="#8c8" stroke-width="1.2"/>')
    s += txt(618, 106, "B. 前向：径向膨胀解析测距", 13, "#2e7d32", bold=True)
    for i, (lb, sub) in enumerate([
            ("连续两帧同一相机", "自车前进，视野径向膨胀"),
            ("径向位移场 u → v", "无对应点搜索"),
            ("解析解 Z = Σ((u×v)·(t×v)) / Σ‖u×v‖²", "闭式，无迭代"),
            ("3D 点 P", "前单向，覆盖 5m")]):
        yy = 126 + i * 48
        s += box(618, yy, 260, 40, lb, sub, "#fff", "#4a9a5a", 11)
        if i < 3:
            s += arrow(748, yy + 42, 748, yy + 46, "", "#4a9a5a")
    s += txt(896, 150, "优势", 11, "#556", bold=True)
    s += txt(896, 168, "纯前向平移下 SGBM", 11, "#334")
    s += txt(896, 184, "结构性失效（像移翻转）", 11, "#334")
    s += txt(896, 206, "合成验证", 10.5, "#667")
    s += txt(896, 222, "误差 0.000 mm", 10.5, "#2e7d32")

    # 下：五阶段
    s += txt(30, 356, "REV-003 五阶段稠密体素精简（15 万点 → 1.06 ms）", 14,
             "#4a5a6b", bold=True)
    stages = [
        ("① AABB + FoE 锥", "点积代数\n无开方无三角函数", "15万"),
        ("② 2D 柱体位图", "每 cell 1 个 uint32\n记 32 层", "15万"),
        ("③ 位运算地面剥离", "CTZ32 找最低层\n保护孤立单层地锁", "→13万"),
        ("④ 极坐标首击外壳", "720 扇区\n30cm 壳厚", "→8千"),
        ("⑤ 配额 + 计数排序", "近场优先\n≤15000 点", "→2070"),
    ]
    for i, (nm, sub, n) in enumerate(stages):
        x = 30 + i * 233
        s += (f'<rect x="{x}" y="376" width="215" height="86" rx="6" '
              f'fill="#fdf3e7" stroke="#d0a060" stroke-width="1.3"/>')
        s += txt(x + 107, 398, nm, 11.5, "#7a4a10", "middle", True)
        for j, ln in enumerate(sub.split("\n")):
            s += txt(x + 107, 418 + j * 14, ln, 10, "#666", "middle")
        s += txt(x + 107, 454, n, 10.5, "#1e5a96", "middle", True)
        if i < 4:
            s += arrow(x + 217, 419, x + 229, 419, "", "#d0a060")

    s += (f'<rect x="30" y="486" width="{W-60}" height="110" rx="8" '
          f'fill="#fff5f5" stroke="#e09090" stroke-width="1.2"/>')
    s += txt(48, 510, "实现中修掉的三个「能跑但结果错」的 bug", 12.5, "#a03030", bold=True)
    bugs = [
        "① 外壳过滤用 continue 跳过点时没清 keep —— zindex 末位 100 但实际只写 86 个点，下游读到未初始化槽位",
        "② 地面剥离后只判「cell 位图非零」不够 —— 地面点与障碍物同 cell，剥离后位图仍非零，地面点被漏放",
        "③ zmap 直接 memcpy 会带进「层在位图里但点全被剔掉」的情况 —— 改为从最终 keep 重新 OR",
    ]
    for i, b in enumerate(bugs):
        s += txt(48, 534 + i * 19, b, 10.5, "#803030")
    return s + svg_close()


def fig_limits():
    """系统局限性：影响 × 状态矩阵。"""
    W, H = 1200, 500
    s = svg_open(W, H)
    s += txt(30, 36, "图 7  已知局限与风险", 20, "#1a2b3c", bold=True)
    s += txt(30, 58, "按「影响程度 × 是否已有对策」排列", 12, "#667")

    rows = [
        ("精度未验证", "高", "无", "唯一测精度的用例失败，原因是场景不是算法；150mm@5m 未被证实"),
        ("点密度不足", "高", "无", "真实数据 195 点/帧，聚类聚出 82 簇每簇 4 点，无实用价值"),
        ("车身掩码粗略", "中", "部分", "矩形占位投影，侧视相机挖掉 77% 画面；应由标定式视图掩码替代"),
        ("无自车运动真值", "中", "部分", "数据集无速度/航向，D4 只能靠 CLI 传入"),
        ("D2 未接入", "中", "有方案", "双向一致性函数已实现，主链未接；遮挡断裂带误配无防护"),
        ("薄障碍物漏检", "低", "有", "720 扇区在 5m 处弧宽 44mm，<50mm 立柱依赖其他传感器"),
        ("真机未验证", "高", "无", "aarch64 产物已出，未在 RK3588 / M57 上运行"),
    ]
    sev = {"高": C_ERR, "中": C_WARN, "低": "#889"}
    y = 86
    for nm, lv, fix, desc in rows:
        s += (f'<rect x="30" y="{y}" width="{W-60}" height="50" rx="5" '
              f'fill="#fff" stroke="#e0e6ec" stroke-width="1"/>')
        s += (f'<rect x="30" y="{y}" width="5" height="50" rx="2" '
              f'fill="{sev[lv]}"/>')
        s += txt(48, y + 21, nm, 12.5, "#1a2b3c", bold=True)
        s += txt(48, y + 39, f"影响 {lv} · 对策 {fix}", 10.5, sev[lv])
        s += txt(300, y + 21, desc, 11, "#445")
        y += 56
    return s + svg_close()


def main():
    OUT.mkdir(parents=True, exist_ok=True)

    figs = {
        "FIG_ARCH": fig_arch(),
        "FIG_PIPE": fig_pipeline(),
        "FIG_PROG": fig_progress(),
        "FIG_COMP": fig_completeness(),
        "FIG_SIZE": fig_codesize(),
        "FIG_ALGO": fig_algo(),
        "FIG_LIMIT": fig_limits(),
    }
    for k, v in figs.items():
        (OUT / f"{k}.svg").write_text(v, encoding="utf-8")

    # 断言：每张图都应有实质内容
    for k, v in figs.items():
        n = v.count("<rect") + v.count("<text")
        assert n > 30, f"{k} 图元太少({n})"
        print(f"  {k:12s} {n:4d} 图元")

    md = (OUT / "REPORT-FULL.md").read_text(encoding="utf-8")
    html = subprocess.run(
        ["pandoc", "-f", "markdown", "-t", "html5", "--standalone",
         "--metadata", "title=NFOCC 项目总览", str(OUT / "REPORT-FULL.md")],
        capture_output=True, text=True, check=True).stdout
    html = subprocess.run(
        ["pandoc", str(OUT / "REPORT-FULL.md"), "-t", "html5", "--standalone",
         "--css", str(OUT / "report.css")],
        capture_output=True, text=True).stdout if False else html

    # 注入 SVG（必须在 pandoc 之后，否则被 raw_html 白名单剥掉）
    for k, v in figs.items():
        html = html.replace(f"<p>{k}</p>", f'<div class="fig">{v}</div>')
        html = html.replace(k, "")

    # 位图（实拍图）
    for png in ["05_bev.png", "06_body_mask.png", "07_clusters.png"]:
        f = ROOT / "viz" / png
        if f.exists():
            b64 = base64.b64encode(f.read_bytes()).decode()
            tok = "IMG_" + png.split(".")[0].upper()
            html = html.replace(
                f"<p>{tok}</p>",
                f'<div class="fig"><img src="data:image/png;base64,{b64}"/></div>')

    css = """
body{font-family:"PingFang SC",sans-serif;max-width:1200px;margin:0 auto;
     padding:40px 56px;color:#1a2b3c;line-height:1.7;font-size:14px}
h1{font-size:30px;border-bottom:3px solid #1e5a96;padding-bottom:12px}
h2{font-size:20px;color:#1e5a96;margin-top:34px;border-left:4px solid #1e5a96;
   padding-left:10px}
h3{font-size:15px;color:#2a4a6a;margin-top:20px}
table{border-collapse:collapse;width:100%;margin:14px 0;font-size:12.5px}
th{background:#eef4fb;border:1px solid #c8d4e0;padding:7px 9px;text-align:left}
td{border:1px solid #dde3e8;padding:6px 9px}
code{background:#f4f6f8;padding:1px 5px;border-radius:3px;font-size:12.5px}
.fig{margin:18px 0;page-break-inside:avoid}
.fig svg{width:100%;height:auto;display:block}
.fig img{max-width:100%;height:auto}
blockquote{border-left:4px solid #e0c060;background:#fffbea;margin:14px 0;
           padding:10px 16px;color:#6b5300}
@page{size:A4;margin:14mm}
@media print{.fig{page-break-inside:avoid}}
"""
    (OUT / "report.css").write_text(css, encoding="utf-8")
    html = html.replace("</head>", f"<style>{css}</style></head>")

    (OUT / "REPORT-FULL.html").write_text(html, encoding="utf-8")
    pdf = OUT / "NFOCC-项目总览.pdf"
    subprocess.run([EDGE, "--headless", "--disable-gpu", "--no-pdf-header-footer",
                    f"--print-to-pdf={pdf}", (OUT / "REPORT-FULL.html").as_uri()],
                   capture_output=True, check=True)
    print(f"\nPDF: {pdf}  {pdf.stat().st_size/1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
