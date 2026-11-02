# SPEC-ALGORITHM-DERIVATION
> **规范代号**：`SPEC-ALGORITHM-DERIVATION`  
> **适用版本**：NFOCC v0.6.0+  

---

## 目录
- [SPEC-ALGORITHM-DERIVATION](#spec-algorithm-derivation)
  - [目录](#目录)
  - [1. 坐标系统一与虚拟视口运动几何学](#1-坐标系统一与虚拟视口运动几何学)
    - [1.1 坐标系定义与对齐准则](#11-坐标系定义与对齐准则)
    - [1.2 虚拟视口光线采样（Ray Sampling）与防“转两次”几何定理](#12-虚拟视口光线采样ray-sampling与防转两次几何定理)
    - [1.3 视口系平移向车体系的严格旋转变换](#13-视口系平移向车体系的严格旋转变换)
  - [2. 移动双目自适应多帧历史与并行双基线状态机模型](#2-移动双目自适应多帧历史与并行双基线状态机模型)
    - [2.1 车体基准位移（Datum Delta）同步机制](#21-车体基准位移datum-delta同步机制)
    - [2.2 航向角微分与转向防失真门控](#22-航向角微分与转向防失真门控)
    - [2.3 并行双基线交替迭代算法（Parallel Dual Baselines）](#23-并行双基线交替迭代算法parallel-dual-baselines)
  - [3. 物理空间不确定度全微分方差传播模型](#3-物理空间不确定度全微分方差传播模型)
    - [3.1 测距公式的偏导推导](#31-测距公式的偏导推导)
    - [3.2 像素匹配误差与里程计漂移的协方差合成](#32-像素匹配误差与里程计漂移的协方差合成)
    - [3.3 厘米级量化、饱和截断与哨兵值定义](#33-厘米级量化饱和截断与哨兵值定义)
  - [4. 四重遮挡与伪影防御体系闭式数学判据（D1-D4）](#4-四重遮挡与伪影防御体系闭式数学判据d1-d4)
    - [4.1 D1：自车几何轮廓光栅化与射线剔除（Ray-Casting Algorithm）](#41-d1自车几何轮廓光栅化与射线剔除ray-casting-algorithm)
    - [4.2 D2：双向视差一致性软性抑制函数（Smooth Occlusion Weighting）](#42-d2双向视差一致性软性抑制函数smooth-occlusion-weighting)
    - [4.3 D3：运动极点（FoE）发散锥代数判据（Algebraic Cone Filtering）](#43-d3运动极点foe发散锥代数判据algebraic-cone-filtering)
    - [4.4 D4：双基线空间重投影一致性检验与方差减半模型](#44-d4双基线空间重投影一致性检验与方差减半模型)
  - [5. 2D 柱状高程压缩与 O(1) 拓扑存储规范](#5-2d-柱状高程压缩与-o1-拓扑存储规范)
    - [5.1 地表二维网格与 3D 离散化映射](#51-地表二维网格与-3d-离散化映射)
    - [5.2 32 位位图 zMap 编码规则](#52-32-位位图-zmap-编码规则)
    - [5.3 单调递增前缀和 zIndex 与 O(1) 局部切片访问定理](#53-单调递增前缀和-zindex-与-o1-局部切片访问定理)
- [解读](#解读)
- [对 `SPEC-ALGORITHM-DERIVATION` 的验证报告](#对-spec-algorithm-derivation-的验证报告)
  - [总体评价](#总体评价)
  - [逐章验证](#逐章验证)
    - [第 1 章 坐标系统一与虚拟视口运动几何学](#第-1-章-坐标系统一与虚拟视口运动几何学)
    - [第 2 章 移动双目自适应多帧历史与并行双基线状态机模型](#第-2-章-移动双目自适应多帧历史与并行双基线状态机模型)
    - [第 3 章 物理空间不确定度全微分方差传播模型](#第-3-章-物理空间不确定度全微分方差传播模型)
    - [第 4 章 四重遮挡与伪影防御体系](#第-4-章-四重遮挡与伪影防御体系)
    - [第 5 章 2D 柱状高程压缩与 O(1) 拓扑存储](#第-5-章-2d-柱状高程压缩与-o1-拓扑存储)
  - [需修正项汇总](#需修正项汇总)
  - [结论](#结论)

---

## 1. 坐标系统一与虚拟视口运动几何学

### 1.1 坐标系定义与对齐准则
系统统一使用右手笛卡尔坐标系，各参考系定义如下：
*   **车辆 ISO 8855 坐标系（Vehicle / Ego Frame）**：原点位于后轴中心地面投影点；$+X$ 指向车头前向，$+Y$ 指向车辆左侧，$+Z$ 垂直地面向上。
*   **相机光学物理坐标系（Camera Optical Frame）**：原点位于镜头光学中心；$+x$ 指向图像右方，$+y$ 指向图像下方，$+z$ 沿光轴指向被摄景物。
*   **虚拟视口坐标系（Virtual Viewport Frame）**：理想针孔投影模型，水平光轴无俯仰无横滚（$	ext{pitch} = 0, 	ext{roll} = 0$），仅沿车辆 $Z$ 轴存在预定偏航角 $\psi$（ISO 8855 逆时针为正）。

### 1.2 虚拟视口光线采样（Ray Sampling）与防“转两次”几何定理
**【核心定理】** 虚拟视口通过 `pixel_to_ray_impl(vp, el_mid, u, v, ray)` 生成的单位视线向量 $\mathbf{d} = (d_x, d_y, d_z)^T$，**在数学上本身已经处于车体 ISO 8855 坐标系中**。

**证明**：
视口像素 $(u, v)$ 计算的方位角 $az$ 与仰角 $el$ 以车体系主轴建立基底：
$$	ext{right} = (\sin\psi, -\cos\psi, 0)^T, \quad 	ext{forward} = (\cos\psi, \sin\psi, 0)^T, \quad 	ext{down} = (0, 0, -1)^T$$
$$\mathbf{r}_{	ext{veh}} = \cos(el) \cdot [\sin(az)\cdot	ext{right} + \cos(az)\cdot	ext{forward}] - \sin(el)\cdot	ext{down}$$
由于基向量已经直接嵌入了车辆偏航角 $\psi$，输出射线方向即在车身坐标系内。真实相机外参矩阵 $R_{	ext{cam}	o	ext{veh}}$ 仅在“车身射线反投影至鱼眼曲面像素”时参与运算。

**【禁忌】** 严禁在前向测距或空间三角化阶段对视口射线 $\mathbf{d}$ 左乘相机安装姿态矩阵 $R_{	ext{cam}	o	ext{veh}}$。若左乘，将导致射线被二次叠加安装角（“转两次”缺陷），使得解算的 3D 深度产生 20%~45% 的灾难性发散误差。

### 1.3 视口系平移向车体系的严格旋转变换
输入源提供的相机间位移 $\mathbf{t}_{	ext{view}} = (t_{v,x}, t_{v,y}, t_{v,z})^T$ 是在视口坐标系内度量的。为了将其与车身坐标系下的视线 $\mathbf{d}_0, \mathbf{d}_1$ 联立求解，必须按视口偏航角 $\psi$ 施加刚体旋转：
$$\mathbf{t}_{	ext{veh}} = R_z(\psi) \cdot \mathbf{t}_{	ext{view}} = egin{bmatrix} \cos\psi & -\sin\psi & 0 \ \sin\psi & \cos\psi & 0 \ 0 & 0 & 1 \end{bmatrix} egin{bmatrix} t_{v,x} \ t_{v,y} \ t_{v,z} \end{bmatrix}$$
物点 3D 车系物理坐标闭式反解公式为：
$$\mathbf{P}_{	ext{veh}} = \mathbf{P}_{	ext{curr\_cam}} - \mathbf{t}_{	ext{veh}} + \mathbf{d}_1 \cdot 	ext{dist}$$
其中 $	ext{dist}$ 为沿参考帧射线 $\mathbf{d}_1$ 的欧氏距离，由解析测距公式给出。

---

## 2. 移动双目自适应多帧历史与并行双基线状态机模型

### 2.1 车体基准位移（Datum Delta）同步机制
多相机环视系统中，转弯时外侧相机线速度显著高于内侧相机。若按单个相机位移作为门控，会导致四路相机深度计算异步。系统引入**后轴车体基准位移（Datum Delta）**作为统一时钟门控：
$$\Delta_{	ext{veh}} = \|\mathbf{P}_{	ext{curr\_rear}} - \mathbf{P}_{	ext{ref\_rear}}\|_2 = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$$
四路相机共享同一个基准触发判定，确保下游接收到的 3D 体素在时间戳与参考基线上绝对同步。

### 2.2 航向角微分与转向防失真门控
两帧间大地系旋转矩阵相对变化量 $R_{	ext{rel}} = R_{	ext{curr}}^T \cdot R_{	ext{ref}}$。其车身航向漂移角 $\Delta\psi$ 为：
$$\Delta\psi = \left|\operatorname{atan2}\left(R_{	ext{rel}}(1, 0), R_{	ext{rel}}(0, 0)
ight)
ight| \cdot rac{180^\circ}{\pi}$$
*   **纯旋转保护判据**：若 $\Delta\psi > 15^\circ$，表明车辆处于剧烈急转弯或原地掉头工况，此时极线搜索完全失效，状态机强制置入 `NF_MODE_FROZEN_ROT`，重置参考帧并清空中间位移，防止生成横向弧形拉丝伪点。

### 2.3 并行双基线交替迭代算法（Parallel Dual Baselines）
为解决单一参考帧在低速行驶或加减速时产生的周期性感知静默，系统维护双参考帧队列（`PREV_1` 与 `PREV_2`）：
1.  同时计算两路基线：$\Delta_{	ext{short}} = \|\mathbf{P}_{	ext{curr}} - \mathbf{P}_{	ext{prev\_short}}\|$，$\Delta_{	ext{long}} = \|\mathbf{P}_{	ext{curr}} - \mathbf{P}_{	ext{prev\_long}}\|$；
2.  当 $\Delta_{	ext{long}} \ge b_{	ext{target}}$（默认 300 mm）时，以长基线执行高精度立体解算输出；
3.  长基线输出后立即退役，短基线自动晋升为下一次的长基线：$	ext{idx}_{	ext{long}} \leftarrow 	ext{idx}_{	ext{short}}$；
4.  将当前帧作为新的短基线参考帧：$	ext{idx}_{	ext{short}} \leftarrow 	ext{idx}_{	ext{curr}}$。
该交替状态机消除了固定基线等待周期，使系统始终处于高置信度感知产出状态。

---

## 3. 物理空间不确定度全微分方差传播模型

### 3.1 测距公式的偏导推导
由针孔立体视觉几何深度公式 $Z = rac{f \cdot b}{d}$，求全微分：
$$\mathrm{d}Z = rac{\partial Z}{\partial d} \mathrm{d}d + rac{\partial Z}{\partial b} \mathrm{d}b = -rac{f \cdot b}{d^2} \mathrm{d}d + rac{f}{d} \mathrm{d}b$$

### 3.2 像素匹配误差与里程计漂移的协方差合成
定义像素匹配标准差为 $\sigma_{	ext{pixel}}$（典型值 $0.5	ext{ px}$），里程计运动相对漂移率为 $\sigma_{	ext{odom}}$（典型值 $0.02$，即 $2\%$ 相对误差）。假设两项误差源相互独立，物理空间深度测量方差为：
$$\sigma_{	ext{spatial}} = \max(|f_{	ext{px}}|, 1.0) \cdot \left( \sigma_{	ext{pixel}} \cdot rac{b}{d^2} + rac{b \cdot \sigma_{	ext{odom}}}{d} 
ight)$$
其中 $f_{	ext{px}}$ 为视口等效焦距，$b$ 为实际基线长度（mm），$d$ 为匹配视差（px）。

### 3.3 厘米级量化、饱和截断与哨兵值定义
*   **单位转换**：$\sigma_{	ext{cm}} = 	ext{round}(\sigma_{	ext{spatial}} 	imes 0.1)$；
*   **数值范围约束**：
    $$\sigma_{	ext{cm}} = egin{cases} 1, & \sigma_{	ext{cm}} < 1 \ 254, & \sigma_{	ext{cm}} > 254 \ \sigma_{	ext{cm}}, & 	ext{otherwise} \end{cases}$$
*   **哨兵值 255**：当 $d \le 0$、视差无效或三角化求交奇异时，$\sigma_{	ext{cm}} \equiv 255$，标识该点空间不确定度不可知或超限。

---

## 4. 四重遮挡与伪影防御体系闭式数学判据（D1-D4）

### 4.1 D1：自车几何轮廓光栅化与射线剔除（Ray-Casting Algorithm）
对于 2D 凸/非凸车身遮蔽多边形顶点序列 $\{(x_i, y_i)\}_{i=0}^{N-1}$，对视口像素点 $(p_x, p_y)$ 发射水平射线：
$$	ext{intersect} = \left( (y_i > p_y) 
eq (y_j > p_y) 
ight) \land \left( p_x < rac{(x_j - x_i)(p_y - y_i)}{y_j - y_i + \epsilon} + x_i 
ight)$$
相交次数为奇数时判定位于车身遮挡区，该光线在 LUT 中 `valid` 标志直接置 0，源头掐灭自车反光点云。

### 4.2 D2：双向视差一致性软性抑制函数（Smooth Occlusion Weighting）
前向视差 $d_{	ext{fwd}}$ 与反向验证视差 $d_{	ext{bwd}}$ 的绝对残差为 $r = |d_{	ext{fwd}} - d_{	ext{bwd}}|$。
置信度衰减软权重函数为：
$$w_{	ext{occ}} = egin{cases} 1.0, & r \le 	au_{	ext{occ}} \ rac{1}{1 + \lambda_{	ext{occ}} \left( rac{r}{	au_{	ext{occ}}} - 1 
ight)}, & r > 	au_{	ext{occ}} \end{cases}$$
其中容差阈值 $	au_{	ext{occ}} = 1.0	ext{ px}$，惩罚系数 $\lambda_{	ext{occ}} = 14.0$。平滑衰减避免了硬阈值切除带来的体素边界锯齿。

### 4.3 D3：运动极点（FoE）发散锥代数判据（Algebraic Cone Filtering）
平移向量单位方向 $\hat{\mathbf{d}} = \mathbf{t}_{	ext{veh}} / \|\mathbf{t}_{	ext{veh}}\|$。
对于候选 3D 物点 $\mathbf{P}$ 与相机光心 $\mathbf{P}_{	ext{apex}}$，构造视线向量 $\mathbf{h} = \mathbf{P} - \mathbf{P}_{	ext{apex}}$。
定义开锥半张角 $	heta_{	ext{cone}} = 8^\circ$（$\cos^2(8^\circ) pprox 0.9806$）。
**【发散锥剔除判据】**：
$$\delta = \mathbf{h} \cdot \hat{\mathbf{d}} > 0 \quad \land \quad \delta^2 \ge \|\mathbf{h}\|_2^2 \cdot \cos^2(8^\circ)$$
满足上述两式即判定物点落在平移运动盲区内，三角化阶段硬剔除，彻底根治直行工况下的前向拉丝幽灵点。

### 4.4 D4：双基线空间重投影一致性检验与方差减半模型
主基线观测点 $\mathbf{P}_1$ 与辅助验证基线反投影点 $\mathbf{P}_2$ 之间的三维物理残差：
$$\Delta^2 = \|\mathbf{P}_1 - \mathbf{P}_2\|_2^2 = (x_1 - x_2)^2 + (y_1 - y_2)^2 + (z_1 - z_2)^2$$
设定空间检验门限 $	ext{Thresh}_{	ext{dist}} = 50	ext{ mm}$：
*   **残差达标（$\Delta^2 \le 	ext{Thresh}_{	ext{dist}}^2$）**：两组测量一致，融合质心 $\mathbf{P}_{	ext{merged}} = 0.5(\mathbf{P}_1 + \mathbf{P}_2)$，且**空间不确定度算术右移 1 位减半**：
    $$\sigma_{	ext{cm}} \leftarrow \max(1, \sigma_{	ext{cm}} \gg 1)$$
*   **残差超限（$\Delta^2 > 	ext{Thresh}_{	ext{dist}}^2$）**：判定为动态干扰或假阳性，大幅削弱权重 $n_{	ext{eff}} \leftarrow n_{	ext{eff}} 	imes 0.2$。当有效观测数 $n_{	ext{eff}} < 0.2$ 时物理释放该体素。

---

## 5. 2D 柱状高程压缩与 O(1) 拓扑存储规范

### 5.1 地表二维网格与 3D 离散化映射
给定体素分辨率 $\Delta_{	ext{vox}} = 200	ext{ mm}$（乘法因子 $	ext{mmPerVoxelInv} = 0.005$），连续物理坐标离散化公式为：
$$i_x = \lfloor (X_{\max} - x) \cdot 0.005 
floor, \quad i_y = \lfloor (Y_{\max} - y) \cdot 0.005 
floor, \quad i_z = \lfloor (z - Z_{\min}) \cdot 0.005 
floor$$
二维地表柱体一维线性索引为：
$$	ext{col} = i_y \cdot n_x + i_x, \quad (0 \le 	ext{col} < n_x \cdot n_y)$$

### 5.2 32 位位图 zMap 编码规则
每个地表柱体分配一个 32 位无符号整数 `zmap[col]`。
若该柱体在垂直第 $i_z$ 层（$0 \le i_z < n_z \le 31$）存在被占据的体素，则置位第 $i_z$ 位：
$$	ext{zmap}[	ext{col}] \leftarrow 	ext{zmap}[	ext{col}] \mid (1	ext{U} \ll i_z)$$
32 位整数最高位恒为 0，有效层高上限支持到 31 层（在 200mm 分辨率下覆盖 6.2 米垂直净空）。

### 5.3 单调递增前缀和 zIndex 与 O(1) 局部切片访问定理
一维数组 `zindex` 长度为 $n_{	ext{cells}} + 1$。
定义 `zindex[col]` 为前 $	ext{col}$ 个柱体内所含全部体素点累计数量：
$$	ext{zindex}[0] = 0, \quad 	ext{zindex}[	ext{col} + 1] = 	ext{zindex}[	ext{col}] + \operatorname{popcount}(	ext{zmap}[	ext{col}])$$

**【O(1) 访问定理】** 
点云缓冲区 `pc->points` 中的点严格按照 `col` 递增排列，在同一个 `col` 内部严格按照垂直层高 $i_z$ 升序紧凑排布。
下游路径规划器查询任意地表网格 $(i_x, i_y)$ 处的障碍物时，无需任何空间遍历或散列查找：
$$	ext{start} = 	ext{zindex}[	ext{col}], \quad 	ext{end} = 	ext{zindex}[	ext{col} + 1]$$
所有该柱体上的障碍物物理点严格位于连续区间 `pc->points[start ... end-1]` 内，时间复杂度为严格的 $O(1)$。

# 解读
# 对 `SPEC-ALGORITHM-DERIVATION` 的验证报告

## 总体评价

该规范整体架构清晰，坐标系统一、虚拟视口防“转两次”定理、并行双基线状态机、四重防御体系、2D 高程压缩等核心设计在数学与工程逻辑上基本成立，可以作为 NFOCC 算法实现的依据。但存在 **2 处数学错误、3 处定义不严谨、2 处工程近似需显式声明**，建议修正后再冻结为开发基线。

---

## 逐章验证

### 第 1 章 坐标系统一与虚拟视口运动几何学

**1.1 坐标系定义**：✅ 正确。车辆 ISO 8855（X 前、Y 左、Z 上，原点后轴中心地面投影）、相机光学系（x 右、y 下、z 前）、虚拟视口系（仅绕 Z 轴偏航 ψ）定义清晰，符合行业惯例。

**1.2 防“转两次”几何定理**：✅ 数学正确。  
基向量 $\text{right}=(\sin\psi,-\cos\psi,0)^T$、$\text{forward}=(\cos\psi,\sin\psi,0)^T$、$\text{down}=(0,0,-1)^T$ 在 $\psi=0$ 时分别对应车体 $-Y$、$+X$、$-Z$，与“右、前、下”一致。合成的 $\mathbf{r}_{\text{veh}}$ 确实已在车体系，禁止再左乘 $R_{\text{cam}\to\text{veh}}$ 的禁忌合理。

**1.3 视口系平移向车体系的旋转变换与物点反解**：⚠️ **存在物理错误**。  
旋转部分 $\mathbf{t}_{\text{veh}} = R_z(\psi)\mathbf{t}_{\text{view}}$ 正确。  
但物点反解公式：
$$\mathbf{P}_{\text{veh}} = \mathbf{P}_{\text{curr\_cam}} - \mathbf{t}_{\text{veh}} + \mathbf{d}_1 \cdot \text{dist}$$
若 $\mathbf{t}_{\text{veh}}$ 定义为“参考帧到当前帧的位移”，则 $\mathbf{P}_{\text{curr\_cam}} - \mathbf{t}_{\text{veh}}$ 等于参考帧光心 $\mathbf{P}_{\text{ref\_cam}}$，而 $\mathbf{d}_1$ 是当前帧射线，从当前光心发出，不能从参考光心出发。正确公式应为：
$$\mathbf{P}_{\text{veh}} = \mathbf{P}_{\text{curr\_cam}} + \mathbf{d}_1 \cdot \text{dist}$$
或若要用参考帧表达：
$$\mathbf{P}_{\text{veh}} = \mathbf{P}_{\text{ref\_cam}} + \mathbf{d}_0 \cdot \text{dist}_0$$
**修正建议**：明确 $\mathbf{t}_{\text{veh}}$ 的方向定义（$\mathbf{C}_{\text{curr}}-\mathbf{C}_{\text{ref}}$ 还是反向），并统一为 $\mathbf{P} = \mathbf{C}_{\text{curr}} + \mathbf{d}_1 \cdot \text{dist}$，避免符号混淆。

---

### 第 2 章 移动双目自适应多帧历史与并行双基线状态机模型

**2.1 车体基准位移同步**：✅ 合理。用后轴中心位移模长作为四路相机统一门控，保证时间戳与基线同步。

**2.2 航向角微分与转向防失真门控**：✅ 正确。  
$R_{\text{rel}} = R_{\text{curr}}^T R_{\text{ref}}$ 提取 $\Delta\psi$ 的方法标准；$15^\circ$ 冻结阈值在工程上合理。建议补充：冻结时清空参考帧队列，避免恢复后使用过期参考帧。

**2.3 并行双基线交替迭代**：⚠️ **缺少基线上限与未输出时的更新规则**。  
交替逻辑本身成立，但：
- 未设 $b_{\max}$。高速时 $\Delta_{\text{long}}$ 可能远超 $300\text{ mm}$（如 $1\text{ m}$），导致重叠视场不足、匹配失效率陡增。建议增加 $b_{\max} \approx 600\text{ mm}$，超限则强制重置队列。
- 未说明 $\Delta_{\text{long}} < b_{\text{target}}$ 时短基线如何更新。应明确：每帧到达时先更新短基线参考帧（或保持原短基线直到长基线触发），否则帧队列可能停滞。

---

### 第 3 章 物理空间不确定度全微分方差传播模型

**3.1 偏导推导**：✅ 正确。$Z = fb/d$ 的全微分无误。

**3.2 方差合成**：❌ **公式错误（线性相加而非平方和）**。  
标准方差传播应为：
$$\sigma_Z = \sqrt{\left(\frac{f b}{d^2}\sigma_d\right)^2 + \left(\frac{f}{d}\sigma_b\right)^2}$$
代入 $\sigma_d = \sigma_{\text{pixel}}$，$\sigma_b = b \cdot \sigma_{\text{odom}}$，得：
$$\sigma_Z = \frac{f b}{d} \sqrt{\left(\frac{\sigma_{\text{pixel}}}{d}\right)^2 + \sigma_{\text{odom}}^2}$$
规范中写为：
$$\sigma_{\text{spatial}} = \max(|f_{\text{px}}|,1.0) \cdot \left(\sigma_{\text{pixel}}\frac{b}{d^2} + \frac{b\sigma_{\text{odom}}}{d}\right)$$
这是将两项**线性相加**，且多乘了 $f_{\text{px}}$ 的提取形式，与标准方差传播不符。若作为保守上界可保留，但应显式声明“采用 $L_1$ 上界而非 $L_2$ 方差合成”，否则与标题“方差传播”矛盾。  
**修正建议**：改为平方和开根形式，或明确标注为“保守线性上界”。

**3.3 厘米量化与哨兵值**：✅ 正确。`round(σ_spatial*0.1)`、`[1,254]` 钳位、`255` 哨兵定义清晰。

---

### 第 4 章 四重遮挡与伪影防御体系

**4.1 D1 自车多边形射线剔除**：✅ 射线法判据正确。  
需明确：多边形顶点 $\{(x_i,y_i)\}$ 与查询点 $(p_x,p_y)$ 应统一在**虚拟视口图像坐标系**下，而非车体系。建议在实现中标注坐标系。

**4.2 D2 双向视差一致性软性抑制**：⚠️ **残差符号需与视差定义对齐**。  
规范写 $r = |d_{\text{fwd}} - d_{\text{bwd}}|$。若后向视差定义为从右图到左图的视差（符号与前向相反），则一致性应为 $|d_{\text{fwd}} + d_{\text{bwd}}|$。需在 ICD 中明确视差符号约定，并与 SGBM 的 `disp12MaxDiff` 语义对齐。软权重函数形式合理。

**4.3 D3 FoE 发散锥判据**：✅ 代数判据正确。  
$\delta = \mathbf{h}\cdot\hat{\mathbf{d}} > 0$ 且 $\delta^2 \ge \|\mathbf{h}\|^2\cos^2(8^\circ)$ 等价于视线与平移方向夹角 $\le 8^\circ$，剔除逻辑成立。

**4.4 D4 双基线重投影验证与方差减半**：⚠️ **方差减半用算术右移过于乐观**。  
$\sigma_{\text{cm}} \gg 1$ 是除以 $2$。两次独立观测的信息融合方差应为 $\sigma^2_{\text{verified}} = \sigma_1^2\sigma_2^2/(\sigma_1^2+\sigma_2^2)$，当 $\sigma_1\approx\sigma_2$ 时 $\sigma_{\text{verified}} \approx \sigma_1/\sqrt{2}$。除以 $2$ 比统计最优更乐观，可能低估不确定度。  
**修正建议**：改为 $\sigma_{\text{cm}} \leftarrow \max(1, \text{round}(\sigma_{\text{cm}}/\sqrt{2}))$，或显式声明“采用保守工程近似，减半系数为经验值，需在标定中验证”。

---

### 第 5 章 2D 柱状高程压缩与 O(1) 拓扑存储

**5.1 离散化映射**：✅ 正确。  
$i_x = \lfloor (X_{\max}-x)\cdot0.005\rfloor$、$i_y = \lfloor (Y_{\max}-y)\cdot0.005\rfloor$、$i_z = \lfloor (z-Z_{\min})\cdot0.005\rfloor$ 量纲一致，索引范围正确。需注意 $X_{\max}/Y_{\max}$ 是网格边界，实现中应确保 $x$ 在 $[X_{\min},X_{\max})$ 内，否则索引越界。

**5.2 zMap 编码**：✅ 正确。`1U << i_z` 使用无符号类型，最高位恒 0，$i_z \le 31$ 约束明确。

**5.3 zIndex 前缀和与 O(1) 访问定理**：✅ 定理成立。  
前提是点云缓冲区严格按 `col` 递增、同 `col` 内按 `i_z` 升序紧凑排列。实现中需保证填充顺序与 `zindex` 构建顺序一致。`popcount` 计算每柱体点数正确。

---

## 需修正项汇总

| 编号 | 位置 | 问题 | 严重度 | 修正建议 |
| :--- | :--- | :--- | :--- | :--- |
| 1 | 1.3 物点反解公式 | $\mathbf{P}=\mathbf{C}_{\text{curr}}-\mathbf{t}_{\text{veh}}+\mathbf{d}_1\cdot\text{dist}$ 物理错误 | **高** | 改为 $\mathbf{P}=\mathbf{C}_{\text{curr}}+\mathbf{d}_1\cdot\text{dist}$，并明确 $\mathbf{t}$ 方向 |
| 2 | 3.2 方差合成 | 线性相加而非平方和，与“方差传播”标题矛盾 | **高** | 改为平方和开根，或声明为保守 $L_1$ 上界 |
| 3 | 4.2 双向视差残差 | $|d_{\text{fwd}}-d_{\text{bwd}}|$ 符号依赖视差定义 | 中 | 明确视差符号约定，与 `disp12MaxDiff` 对齐 |
| 4 | 4.4 方差减半 | $\sigma \gg 1$ 除以 2 过于乐观 | 中 | 改为除以 $\sqrt{2}$，或声明为经验保守近似 |
| 5 | 2.3 双基线状态机 | 缺 $b_{\max}$ 与未触发时的更新规则 | 中 | 增加 $b_{\max}$ 强制重置；明确每帧短基线更新逻辑 |
| 6 | 4.1 车身掩码 | 未明确多边形与查询点的坐标系 | 低 | 标注为虚拟视口图像坐标系 |

---

## 结论

该规范**可以作为 NFOCC v0.6.0+ 的算法开发依据**，但第 1.3 节的物点反解公式和第 3.2 节的方差合成公式必须在编码前修正，否则会直接导致 3D 点坐标系统偏差和不确定度低估。其余问题为定义澄清与工程近似声明，可在 ICD 与 DDD 中补充后冻结。建议修正后发布 **v1.0.0-Validated** 版本，作为代码评审的基线。