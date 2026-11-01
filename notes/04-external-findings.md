# 外部调研结论（本轮补全）

早前三个调研任务已返回。本文记录**已核实**与**仍未核实**的边界，并列出它们对 `docs/` 的影响。
凡未核实的，不写进规范，只在此备注。

---

## 1. StereoNetPlus —— 查不到这篇论文

调研结论：**arXiv 中没有 "StereoNet Plus"**。

- arXiv ID 2309.12121 实际是《A Multiscale Autoencoder (MSAE) Framework for End-to-End
  Neural Network Speech Enhancement》（语音增强），与立体匹配无关。
- arXiv 全库 `ti:"StereoNet"` 检索只返回原版 **StereoNet (arXiv:1807.08865)**，无 Plus 版本。
- Crossref 按标题检索无匹配记录。
- 相关的衍生工作只有 **Compact StereoNet**（IEEE Access 2020, DOI 10.1109/access.2020.3029832，
  知识蒸馏方向），不是 Harman/Samsung 的 Plus。

**影响**：`notes/03-disparity-engine.md` 里"候选 3：StereoNetPlus"整节的前提不成立。
`docs/SPEC-ENGINE.md` 只按"模型方案"抽象描述，未引用任何具体论文指标，**不受影响**。
若确有该工作，需要提供正确的 arXiv ID / DOI / 会议名才能再核。

---

## 2. TDA4 SDE —— 已核实，但本项目已排除该硬件

来源：TI E2E 员工回帖 + `edgeai-gst-plugins` 源码 `ext/tiovx/gsttiovxsde.c`。

| 项 | 核实结果 |
|---|---|
| 最大视差 | **191**，且是**离散档位**（`disparity_max_63 / 127 / 191`），不是任意可配；min 只有 `0` 或 `-3` |
| 输入要求 | 必须是**已整流、行对齐**的同尺寸灰度图；SDE 不吃鱼眼展开图 |
| 前级链路 | 鱼眼 → LDC 去畸变 → 双目极线校正（每目一个 LDC mesh）→ SDE |
| 最大分辨率 | 2048×1024，**不支持 1080 高度** |
| 置信度 | 支持输出 |

**影响**：用户已明确**放弃硬件 SDE**，故此节仅作背景记录。
但"**SDE 需要先做双目极线校正**"这一条有普适价值：它从第三方硬件角度印证了
`SPEC-EPIPOLAR` 的结论——**整流是前置必需的，不能指望匹配器自己适应**。

---

## 3. 鱼眼近场稠密立体：公开工作基本空白

| 类型 | 有无公开工作 |
|---|---|
| 鱼眼/ERP 展开图上跑 SGBM 做车载近场 0.3–5 m 稠密深度 | **没有找到** |
| 鱼眼环视近场深度 | 全是**自监督单目**（SVDistNet T-ITS'21 / FisheyeDistanceNet / SynDistNet） |
| 全向立体 | OmniSLAM (ICRA'20)、DFI-OmniStereo、Orthogonally-Divergent Fisheye Stereo |
| 鱼眼立体精度分析 | *On the Accuracy of Dense Fisheye Stereo* (RA-L 2016) |
| 停车场/车位 + 鱼眼立体 | **空白** |
| RK3588 / M57 上的鱼眼立体 benchmark | **查不到** |

**影响**：本项目没有现成参数可抄，`PARAM.md` 的反推是唯一依据。这既是风险也是差异点。

---

## 4. 已核实的技术要点（可写进规范）

| 要点 | 来源 | 落点 |
|---|---|---|
| ERP 只在两相机光轴平行且基线水平时使极线严格水平；否则残差是**竖直位移**，需 v-视差校正 | arXiv:2606.24847 | 与 `SPEC-EPIPOLAR` (1.1) 一致 |
| ERP 极区（上下极）采样密度崩溃，应只在中纬度环带内匹配 | 同上 | 强化了 `SPEC-COORD` 3.5 的"垂直半视场"结论 |
| 鱼眼边缘畸变直接降低匹配精度，替代做法是**抽小视场针孔块**而非全图整流 | arXiv:2307.03602 (ECCV-W'18) | 支持 `SPEC-EPIPOLAR` §3 的"视口重排"思路 |
| SGM 在 ERP 接缝处聚合不连续，需环形 wrap | 调研者的推断，**无文献证实** | 若将来改用全景 ERP 才需要 |

---

## 5. 仍未核实（不要引用）

| 项 | 状态 |
|---|---|
| Oram 2002「Rectification for any epipolar geometry」原文 | 未取得 |
| Abraham / Förstner omnidirectional rectification | 未查到可验证条目 |
| Caruso et al. RSS 2010（uni-sphere 统一球面模型） | 未取得原文 |
| OmniSLAM PDF 细节 | 仅确认存在 |
| StereoNetPlus 全部技术细节 | **论文本身查不到** |
| TDA4VH TRM (SPRUJ52) 的 SDE 章节 | 未取得原文，191 档位为高置信推断 |

---

## 6. 对 `docs/` 的实际改动

| 改动 | 位置 |
|---|---|
| 新增 §3.0：实测 `fisheye::stereoRectify` 的焦距压缩与单位陷阱 | `SPEC-EPIPOLAR.md` |
| 澄清"视差恒为 0"是单位错误导致的误判，不是函数缺陷 | 同上 |
| 上游调研"视差恒为 0"的说法**已证伪**，不采纳 | 本文 §7 |

---

## 7. 一处需要记录的自我纠错

调研子任务报告称 `fisheye::stereoRectify` 在 >180° 时"视差恒为 0.00"。
我按它给的方式复现失败后，用正确签名 `(K,D,K,D,size,R,tvec,flags,...)` 重测：

- `tvec` 按 **mm** 传（t=120）：视差 94.5 / 23.6 / 9.4 px，**完全正确**；
- `tvec` 按 **m** 传（t=0.12）：视差 0.094 / 0.024 / 0.009 px —— 这才是"恒为 0"的来源。

所以那条结论是**单位错误造成的假阳性**。真实结论是"几何正确但焦距被压缩到 56%"。
这类"看起来很专业的错误结论"必须复算，不能直接采信。
