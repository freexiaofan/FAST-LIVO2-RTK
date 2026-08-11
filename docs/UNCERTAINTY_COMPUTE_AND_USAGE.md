# FAST-LIVO2-RTK 中 uncertainty 的计算与使用

## 1. 目标

本文说明 `FAST-LIVO2-RTK` 在前端 `VoxelMapManager` 中，不确定性（uncertainty）是如何：
1. 被构造（传感器噪声、状态协方差、平面参数协方差）
2. 被传播（IMU 传播 + 点协方差映射）
3. 被使用（残差加权、数据关联门限、EKF 求解、RTK 融合）

核心代码位置：
- `src/voxel_map.cpp`
- `src/IMU_Processing.cpp`
- `include/common_lib.h`
- `include/voxel_map.h`

---

## 2. uncertainty 的三层来源

## 2.1 点测量噪声（LiDAR 原始测量层）

函数 `calcBodyCov(...)` 直接构造单点协方差：
- 位置：`src/voxel_map.cpp:15`
- 关键公式：`src/voxel_map.cpp:33`

其形式为：

$$
\Sigma_{p,b} = \mathbf{d}\,\sigma_r^2\,\mathbf{d}^T + \mathbf{A}\,\Sigma_{\text{dir}}\,\mathbf{A}^T
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\Sigma_{p,b} = \mathbf{d}\sigma_r^2\mathbf{d}^T + \mathbf{A}\Sigma_{\text{dir}}\mathbf{A}^T$ | [src/voxel_map.cpp#L33](../src/voxel_map.cpp#L33) |

其中：
- $\sigma_r^2$ 来自 `range_inc`（距离噪声）
- $\Sigma_{dir}$ 来自 `degree_inc`（角度噪声）
- 结果是体坐标系下点不确定性

这部分在代码里进入 `body_cov_list_`：
- `src/voxel_map.cpp:354`
- `src/voxel_map.cpp:355`

---

## 2.2 状态不确定性（ESKF 状态层）

### 2.2.1 状态向量与协方差结构

状态协方差定义在 `StatesGroup::cov`：
- 维度：`DIM_STATE=19`（[include/common_lib.h#L31](../include/common_lib.h#L31)）
- 成员：[include/common_lib.h#L232](../include/common_lib.h#L232)
- 初始化：[include/common_lib.h#L147](../include/common_lib.h#L147)

19 维状态分块含义（按行/列索引）：

| 索引 | 状态分量 | 含义 |
|---|---|---|
| 0–2 | $\delta\boldsymbol{\theta}$ | 姿态误差（so(3)扰动） |
| 3–5 | $\delta\mathbf{t}$ | 位置误差 |
| 6 | $\delta e$ | 曝光时间误差（可选） |
| 7–9 | $\delta\mathbf{v}$ | 速度误差 |
| 10–12 | $\delta\mathbf{b}_g$ | 陀螺偏置误差 |
| 13–15 | $\delta\mathbf{b}_a$ | 加计偏置误差 |
| 16–18 | $\delta\mathbf{g}$ | 重力误差（可选估计） |

### 2.2.2 离散时间协方差传播公式

每个 IMU 时间步 $\Delta t$ 做一次一阶线性化传播：

$$
\mathbf{P}_{k|k-1} = \mathbf{F}_x\,\mathbf{P}_{k-1|k-1}\,\mathbf{F}_x^T + \mathbf{Q}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{P}_{k\|k-1} = \mathbf{F}_x\mathbf{P}_{k-1\|k-1}\mathbf{F}_x^T + \mathbf{Q}$ | [src/IMU_Processing.cpp#L401](../src/IMU_Processing.cpp#L401) |

### 2.2.3 状态转移矩阵 $\mathbf{F}_x$ 的构造

`F_x` 初始为单位阵，再按物理耦合关系填入非零块（[src/IMU_Processing.cpp#L385](../src/IMU_Processing.cpp#L385)–[L392](../src/IMU_Processing.cpp#L392)）：

$$
\mathbf{F}_x = \mathbf{I} + \begin{bmatrix}
\text{Exp}(-\boldsymbol{\omega}\Delta t) & & & -\mathbf{I}\Delta t & & \\
& \mathbf{I} & \mathbf{I}\Delta t & & & \\
-\mathbf{R}\lfloor\mathbf{a}\rfloor_\times\Delta t & & \mathbf{I} & & -\mathbf{R}\Delta t & \mathbf{I}\Delta t \\
& & & \mathbf{I} & & \\
& & & & \mathbf{I} & \\
& & & & & \mathbf{I}
\end{bmatrix}
$$

各非零块对应代码语句：

| 块位置 | 物理含义 | 源码 |
|---|---|---|
| `F_x.block<3,3>(0,0)` = $\text{Exp}(-\boldsymbol{\omega}\Delta t)$ | 姿态误差自传播 | [src/IMU_Processing.cpp#L385](../src/IMU_Processing.cpp#L385) |
| `F_x.block<3,3>(0,10)` = $-\mathbf{I}\Delta t$ | 陀螺偏置对姿态误差的影响 | [src/IMU_Processing.cpp#L386](../src/IMU_Processing.cpp#L386) |
| `F_x.block<3,3>(3,7)` = $\mathbf{I}\Delta t$ | 速度积分到位置 | [src/IMU_Processing.cpp#L388](../src/IMU_Processing.cpp#L388) |
| `F_x.block<3,3>(7,0)` = $-\mathbf{R}\lfloor\mathbf{a}\rfloor_\times\Delta t$ | 姿态误差对速度误差的影响 | [src/IMU_Processing.cpp#L389](../src/IMU_Processing.cpp#L389) |
| `F_x.block<3,3>(7,13)` = $-\mathbf{R}\Delta t$ | 加计偏置对速度误差的影响 | [src/IMU_Processing.cpp#L390](../src/IMU_Processing.cpp#L390) |
| `F_x.block<3,3>(7,16)` = $\mathbf{I}\Delta t$ | 重力误差对速度误差的影响 | [src/IMU_Processing.cpp#L391](../src/IMU_Processing.cpp#L391) |

### 2.2.4 过程噪声矩阵 $\mathbf{Q}$ 的构造

`cov_w`（即 $\mathbf{Q}$）按各传感器噪声密度×时间步长填入对角块：

$$
\mathbf{Q} = \mathrm{blockdiag}\!\left(
\sigma_g^2\Delta t^2\,\mathbf{I}_3,\;
0,\;
\mathbf{R}\,\mathrm{diag}(\sigma_a^2)\,\mathbf{R}^T\Delta t^2,\;
\sigma_{bg}^2\Delta t^2\,\mathbf{I}_3,\;
\sigma_{ba}^2\Delta t^2\,\mathbf{I}_3
\right)
$$

各块对应代码：

| $\mathbf{Q}$ 块 | 物理含义 | 源码 |
|---|---|---|
| `cov_w.block<3,3>(0,0)` = $\sigma_g^2\Delta t^2\mathbf{I}$ | 陀螺测量白噪声 | [src/IMU_Processing.cpp#L396](../src/IMU_Processing.cpp#L396) |
| `cov_w.block<3,3>(7,7)` = $\mathbf{R}\,\mathrm{diag}(\sigma_a^2)\,\mathbf{R}^T\Delta t^2$ | 加计测量白噪声（旋转到世界系） | [src/IMU_Processing.cpp#L397](../src/IMU_Processing.cpp#L397) |
| `cov_w.block<3,3>(10,10)` = $\sigma_{bg}^2\Delta t^2\mathbf{I}$ | 陀螺偏置随机游走 | [src/IMU_Processing.cpp#L398](../src/IMU_Processing.cpp#L398) |
| `cov_w.block<3,3>(13,13)` = $\sigma_{ba}^2\Delta t^2\mathbf{I}$ | 加计偏置随机游走 | [src/IMU_Processing.cpp#L399](../src/IMU_Processing.cpp#L399) |

注意加计噪声块乘了 $\mathbf{R}$，因为 acc 在 body 系测量但 $\delta\mathbf{v}$ 误差在 world 系定义。

### 2.2.5 传播结果如何流向后续使用

每帧 IMU 传播结束后，`state_inout.cov` 就是先验协方差 $\mathbf{P}_{k|k-1}$，进入 `StateEstimation` 后被用于：
1. 点协方差的世界系映射（第 3 节中的 $\Sigma_{rot}$、$\Sigma_t$）
2. EKF 法方程中的先验约束项 $\mathbf{P}^{-1}$（第 6 节）

---

## 2.3 平面参数不确定性（地图几何层）

### 2.3.1 存储结构

平面结构中存储了 6×6 的参数协方差矩阵 `plane_var_`（前 3 维对应法向 $\mathbf{n}$，后 3 维对应中心 $\mathbf{q}$）：
- `VoxelPlane::plane_var_`：[include/voxel_map.h#L76](../include/voxel_map.h#L76)
- `PointToPlane::plane_var_`：[include/voxel_map.h#L60](../include/voxel_map.h#L60)

### 2.3.2 平面拟合过程（`init_plane`）

函数入口：[src/voxel_map.cpp#L55](../src/voxel_map.cpp#L55)

**Step 1：计算点云中心与散布矩阵**（[src/voxel_map.cpp#L63](../src/voxel_map.cpp#L63)–[L69](../src/voxel_map.cpp#L69)）

$$
\mathbf{q} = \frac{1}{N}\sum_i \mathbf{p}_i, \qquad
\mathbf{C} = \frac{1}{N}\sum_i \mathbf{p}_i\mathbf{p}_i^T - \mathbf{q}\mathbf{q}^T
$$

**Step 2：特征值分解**（[src/voxel_map.cpp#L70](../src/voxel_map.cpp#L70)–[L79](../src/voxel_map.cpp#L79)）

$$
\mathbf{C} = \mathbf{V}\,\mathrm{diag}(\lambda_{\min},\lambda_{\text{mid}},\lambda_{\max})\,\mathbf{V}^T
$$

- 最小特征值对应特征向量 $\mathbf{v}_{\min}$ 即为法向量 $\mathbf{n}$
- 平面判据：$\lambda_{\min} < \text{planer\_threshold}$（[src/voxel_map.cpp#L86](../src/voxel_map.cpp#L86)）

### 2.3.3 Jacobian 的构造与含义

**Step 3：对每个点构造 $\mathbf{J}_i \in \mathbb{R}^{6\times 3}$**（[src/voxel_map.cpp#L88](../src/voxel_map.cpp#L88)–[L109](../src/voxel_map.cpp#L109)）

$\mathbf{J}_i$ 分两块：

$$
\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{i,n} \\ \mathbf{J}_{i,q} \end{bmatrix}
= \begin{bmatrix} \mathbf{V}\,\mathbf{F}_i \\ \frac{1}{N}\mathbf{I}_3 \end{bmatrix}
$$

- 下半块 $\mathbf{J}_{i,q} = \frac{1}{N}\mathbf{I}_3$：中心 $\mathbf{q}$ 对第 $i$ 个点的梯度，即平均算子（[src/voxel_map.cpp#L108](../src/voxel_map.cpp#L108) `J.block<3,3>(3,0) = J_Q`）
- 上半块 $\mathbf{J}_{i,n} = \mathbf{V}\mathbf{F}_i$：法向 $\mathbf{n}$ 对第 $i$ 个点的梯度，通过特征向量扰动推导（[src/voxel_map.cpp#L107](../src/voxel_map.cpp#L107) `J.block<3,3>(0,0) = evecs.real() * F`）

其中矩阵 $\mathbf{F}_i$ 的第 $m$ 行（$m \neq \min$）为：

$$
\mathbf{F}_{i,m} = \frac{(\mathbf{p}_i - \mathbf{q})^T}{N(\lambda_{\min}-\lambda_m)}
\left(\mathbf{v}_m\mathbf{v}_{\min}^T + \mathbf{v}_{\min}\mathbf{v}_m^T\right)
$$

（[src/voxel_map.cpp#L96](../src/voxel_map.cpp#L96)–[L101](../src/voxel_map.cpp#L101)）

第 $\min$ 行置零（法向分量对自身扰动的一阶项为零）。

**Step 4：协方差累积**（[src/voxel_map.cpp#L110](../src/voxel_map.cpp#L110)）

$$
\Sigma_{\pi} = \sum_{i=1}^{N} \mathbf{J}_i\,\Sigma_{p,i}\,\mathbf{J}_i^T
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{q}=\frac{1}{N}\sum_i\mathbf{p}_i$ | [src/voxel_map.cpp#L68](../src/voxel_map.cpp#L68) |
| $\mathbf{C}=\frac{1}{N}\sum_i\mathbf{p}_i\mathbf{p}_i^T-\mathbf{q}\mathbf{q}^T$ | [src/voxel_map.cpp#L69](../src/voxel_map.cpp#L69) |
| 特征值分解取最小特征向量为 $\mathbf{n}$ | [src/voxel_map.cpp#L70](../src/voxel_map.cpp#L70)–[L79](../src/voxel_map.cpp#L79) |
| $\mathbf{F}_{i,m}$ 行推导 | [src/voxel_map.cpp#L96](../src/voxel_map.cpp#L96)–[L101](../src/voxel_map.cpp#L101) |
| `J.block<3,3>(0,0)` = $\mathbf{V}\mathbf{F}_i$ | [src/voxel_map.cpp#L107](../src/voxel_map.cpp#L107) |
| `J.block<3,3>(3,0)` = $\frac{1}{N}\mathbf{I}$ | [src/voxel_map.cpp#L108](../src/voxel_map.cpp#L108) |
| $\Sigma_{\pi} \mathrel{+}= \mathbf{J}_i\Sigma_{p,i}\mathbf{J}_i^T$ | [src/voxel_map.cpp#L110](../src/voxel_map.cpp#L110) |

### 2.3.4 物理意义

- $\Sigma_{\pi}$ 的前 3×3 块描述**法向量 $\mathbf{n}$ 的估计不确定性**：点测量噪声越大、点数越少、点云越薄，法向越不准
- $\Sigma_{\pi}$ 的后 3×3 块描述**中心点 $\mathbf{q}$ 的估计不确定性**：等于所有点协方差的平均 $\frac{1}{N^2}\sum_i\Sigma_{p,i}$
- 该矩阵会在第 4 节残差加权时通过 $\mathbf{J}_{nq}$ 再次投影，最终影响每个约束的权重

---

## 3. 点协方差如何从 body 映射到 world

### 3.1 坐标系链与变量说明

本系统涉及三个坐标系：

| 坐标系 | 记号 | 说明 |
|---|---|---|
| LiDAR 传感器系 | $\mathcal{L}$ | `calcBodyCov` 输出的 $\Sigma_{p,b}$ 在此系 |
| IMU/body 系 | $\mathcal{B}$ | 外参 `extR_`, `extT_` 将 $\mathcal{L}$ 变换到此系 |
| 世界系 | $\mathcal{W}$ | `state_.rot_end`（$\mathbf{R}$）和 `state_.pos_end`（$\mathbf{t}$）将 $\mathcal{B}$ 变换到此系 |

**预计算阶段**（进入迭代前，对每帧只做一次）：

1. 在 LiDAR 系下计算点协方差 $\Sigma_{p,b}$ → `body_cov_list_[i]`（[src/voxel_map.cpp#L353](../src/voxel_map.cpp#L353)）
2. 将点变换到 IMU 系：$\mathbf{p}_{\mathcal{B}} = \mathbf{R}_{ext}\mathbf{p}_{\mathcal{L}} + \mathbf{t}_{ext}$（[src/voxel_map.cpp#L356](../src/voxel_map.cpp#L356)）
3. 计算 IMU 系下点的叉乘矩阵 $[\mathbf{p}_{\mathcal{B}}]_\times$ → `cross_mat_list_[i]`（[src/voxel_map.cpp#L358](../src/voxel_map.cpp#L358)）

### 3.2 误差传播推导

世界系下点坐标：

$$
\mathbf{p}_{\mathcal{W}} = \mathbf{R}\,\mathbf{p}_{\mathcal{B}} + \mathbf{t}
$$

对状态误差 $(\delta\boldsymbol{\theta},\,\delta\mathbf{t})$ 和点测量误差 $\delta\mathbf{p}_{\mathcal{B}}$ 做一阶扰动：

$$
\delta\mathbf{p}_{\mathcal{W}}
= \underbrace{\mathbf{R}\,\delta\mathbf{p}_{\mathcal{B}}}_{\text{点测量误差项}}
\underbrace{- [\mathbf{p}_{\mathcal{B}}]_\times\,\delta\boldsymbol{\theta}}_{\text{姿态误差项}}
+ \underbrace{\delta\mathbf{t}}_{\text{位移误差项}}
$$

三项相互独立，因此协方差叠加：

$$
\Sigma_{p,w}
= \underbrace{\mathbf{R}\,\Sigma_{p,b}\,\mathbf{R}^T}_{\text{点测量噪声}}
+ \underbrace{[\mathbf{p}_{\mathcal{B}}]_\times\,\Sigma_{rot}\,[\mathbf{p}_{\mathcal{B}}]_\times^T}_{\text{姿态不确定性}}
+ \underbrace{\Sigma_t}_{\text{位移不确定性}}
$$

**公式→源码行号定位表（可点击）**

| 公式项 | 物理含义 | 源码定位 |
|---|---|---|
| $\mathbf{p}_{\mathcal{W}}=\mathbf{R}\mathbf{p}_{\mathcal{B}}+\mathbf{t}$ | 点坐标变换 | [src/voxel_map.cpp#L376](../src/voxel_map.cpp#L376) |
| `rot_var = state_.cov.block<3,3>(0,0)` | 从状态协方差取姿态块 $\Sigma_{rot}$ | [src/voxel_map.cpp#L377](../src/voxel_map.cpp#L377) |
| `t_var = state_.cov.block<3,3>(3,3)` | 从状态协方差取位移块 $\Sigma_t$ | [src/voxel_map.cpp#L378](../src/voxel_map.cpp#L378) |
| $\mathbf{R}\Sigma_{p,b}\mathbf{R}^T$ | 点测量噪声旋转到世界系 | [src/voxel_map.cpp#L387](../src/voxel_map.cpp#L387) |
| $[\mathbf{p}_{\mathcal{B}}]_\times\Sigma_{rot}[\mathbf{p}_{\mathcal{B}}]_\times^T$ | 姿态不确定性通过力矩臂放大 | [src/voxel_map.cpp#L387](../src/voxel_map.cpp#L387) |
| $+\Sigma_t$ | 直接叠加位移不确定性 | [src/voxel_map.cpp#L387](../src/voxel_map.cpp#L387) |
| `pv.var = cov` | 写回世界系总协方差 | [src/voxel_map.cpp#L388](../src/voxel_map.cpp#L388) |
| `pv.body_var = body_cov_list_[i]` | 保留体坐标系原始协方差 | [src/voxel_map.cpp#L389](../src/voxel_map.cpp#L389) |

### 3.3 代码实现（迭代内每轮重新计算）

```cpp
M3D rot_var = state_.cov.block<3, 3>(0, 0);   // Σ_rot
M3D t_var   = state_.cov.block<3, 3>(3, 3);   // Σ_t

M3D cov = body_cov_list_[i];                  // Σ_{p,b}（LiDAR 系）
M3D point_crossmat = cross_mat_list_[i];       // [p_B]×（IMU 系预计算）

cov = state_.rot_end * cov * state_.rot_end.transpose()           // R Σ_{p,b} R^T
  + (-point_crossmat) * rot_var * (-point_crossmat).transpose()  // [p_B]× Σ_rot [p_B]×^T
  + t_var;                                                        // Σ_t

pv.var      = cov;               // 世界系总协方差
pv.body_var = body_cov_list_[i]; // 原始体坐标系协方差（供后续加权备用）
```

### 3.4 几个关键细节

**① 姿态项的力矩臂效应**

$[\mathbf{p}_{\mathcal{B}}]_\times\Sigma_{rot}[\mathbf{p}_{\mathcal{B}}]_\times^T$ 的大小正比于点到原点距离的平方。距传感器越远的点，姿态误差导致的世界系不确定性越大。

**② 为什么叉乘矩阵用 IMU 系而非 LiDAR 系**

姿态误差 $\delta\boldsymbol{\theta}$ 定义在 IMU/body 系，力矩臂也必须用 IMU 系下的点坐标 $\mathbf{p}_{\mathcal{B}}$。`cross_mat_list_` 在预计算时已完成 `extR_` 变换，迭代中可直接复用。

**③ 每轮迭代重新计算**

`state_.rot_end`、`rot_var`、`t_var` 随迭代状态更新而变化，因此 $\Sigma_{p,w}$ 在每轮开始时重新计算，确保线性化点始终基于最新状态。

**④ 最终效果**

同一个 LiDAR 点，$\Sigma_{p,b}$ 固定，但：
- $\Sigma_{rot}$、$\Sigma_t$ 越大 → $\Sigma_{p,w}$ 越大 → 残差权重越小 → 该点对 EKF 更新贡献越弱
- 随迭代收敛，位姿协方差缩小，对应点权重回升

---

## 4. uncertainty 如何进入残差权重（最关键）

### 4.1 残差方差的组合与投影

每个点到平面的残差是一个**标量**，其总方差包含**三个独立的不确定性来源**：

#### 4.1.1 平面参数不确定性向法向方向的投影

平面用参数 $(\mathbf{n}, d)$ 表示，点到平面的距离为 $d_{plane} = \mathbf{n}^T\mathbf{p} + d$。  
平面参数的不确定性表现为 $6 \times 6$ 协方差矩阵 $\Sigma_\pi$（法向量 3 维 + 中心位置 3 维）。

映射到残差的Jacobian为：

$$
\mathbf{J}_{nq} = \begin{bmatrix} \mathbf{p} - \mathbf{c} & -\mathbf{n} \end{bmatrix} \in \mathbb{R}^{1 \times 6}
$$

其中 $\mathbf{c}$ 是平面中心，$\mathbf{p}$ 是世界坐标系下的点。

平面参数不确定性对残差的贡献表现为：

$$
\sigma_{plane}^2 = \mathbf{J}_{nq} \Sigma_\pi \mathbf{J}_{nq}^T
$$

**公式→源码行号定位表（可点击）**

| 公式部分 | 含义 | 源码定位 |
|---|---|---|
| $\mathbf{J}_{nq}$ | 点到平面距离对平面参数的梯度 | [src/voxel_map.cpp#L795-L796](../src/voxel_map.cpp#L795-L796) |
| $\Sigma_\pi$ | 平面的 $6 \times 6$ 协方差矩阵 | [src/voxel_map.cpp#L797](../src/voxel_map.cpp#L797)（`plane.plane_var_`） |
| $\mathbf{J}_{nq}\Sigma_\pi\mathbf{J}_{nq}^T$ | 方差投影结果（`J_nq * plane.plane_var_ * J_nq.transpose()`) | [src/voxel_map.cpp#L797](../src/voxel_map.cpp#L797) |

代码实现（[src/voxel_map.cpp#L797](../src/voxel_map.cpp#L797)）：
```cpp
double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
```

#### 4.1.2 点测量不确定性向法向方向的投影

每个点在世界坐标系下的协方差为 $\Sigma_{p,w}$ （来自第3节身体系→世界系的变换）。

点到平面的残差只依赖于点在平面**法向方向**上的位置，法向方向上的方差为：

$$
\sigma_{point}^2 = \mathbf{n}^T \Sigma_{p,w} \mathbf{n}
$$

其中 $\mathbf{n}$ 是平面的单位法向量。

**公式→源码行号定位表（可点击）**

| 公式部分 | 含义 | 源码定位 |
|---|---|---|
| $\mathbf{n}^T\Sigma_{p,w}\mathbf{n}$ | 点协方差在法向方向的投影 | [src/voxel_map.cpp#L792](../src/voxel_map.cpp#L792) |
| $\Sigma_{p,w}$ | 点的世界坐标系协方差（来自[第3节](##-3-点协方差如何从-body-映射到-world)) | [src/voxel_map.cpp#L792](../src/voxel_map.cpp#L792) |

代码实现（[src/voxel_map.cpp#L792](../src/voxel_map.cpp#L792)）：
```cpp
sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
```

#### 4.1.3 总残差方差

两项**独立**，因此直接相加：

$$
\sigma_l = \sigma_{plane}^2 + \sigma_{point}^2 = \mathbf{J}_{nq}\Sigma_\pi\mathbf{J}_{nq}^T + \mathbf{n}^T\Sigma_{p,w}\mathbf{n}
$$

**公式→源码行号定位表（可点击）**

| 合并公式 | 源码定位 |
|---|---|
| $\sigma_l = \mathbf{J}_{nq}\Sigma_\pi\mathbf{J}_{nq}^T + \mathbf{n}^T\Sigma_{p,w}\mathbf{n}$ | [src/voxel_map.cpp#L797-L798](../src/voxel_map.cpp#L797-L798) |

---

### 4.2 数据关联门限：不确定性驱动的匹配

在多个潜在平面中选择最合适的匹配时，系统不仅考虑残差大小，还考虑该残差相对于其不确定性的**归一化**程度。

#### 4.2.1 $\sigma$ 门限检验

判断一个点是否与某个平面"匹配"的第一步是：

$$
|d_{plane}| < \sigma_{num} \cdot \sqrt{\sigma_l}
$$

其中：
- $d_{plane}$ 是点到平面的**有向距离**（残差）
- $\sigma_l$ 是残差的**标准差**
- $\sigma_{num}$ 是用户设置的门限倍数（通常 2.5–3.0，对应 95%–99.7% 置信度）

**几何含义**：残差必须在距离平面 $\sigma_{num}$ 倍标准差的"走廊"内，才被认为可能是合法匹配。

**公式→源码行号定位表（可点击）**

| 公式 | 物理含义 | 源码定位 |
|---|---|---|
| $\sigma_{num}$ | 门限倍数（配置参数） | [src/voxel_map.cpp#L779](../src/voxel_map.cpp#L779) |
| $\sqrt{\sigma_l}$ | 残差标准差 | [src/voxel_map.cpp#L799](../src/voxel_map.cpp#L799) |
| $\|d_{plane}\| < \sigma_{num}\sqrt{\sigma_l}$ | 门限检验（高斯分布3-sigma规则） | [src/voxel_map.cpp#L799](../src/voxel_map.cpp#L799) |

代码实现（[src/voxel_map.cpp#L799](../src/voxel_map.cpp#L799)）：
```cpp
if (dis_to_plane < sigma_num * sqrt(sigma_l))
{
  is_sucess = true;
  // ... 继续处理这个匹配
}
```

#### 4.2.2 不确定性越大，门限越宽松

这是关键的自适应机制：
- **不确定性大**（$\sigma_l$ 大）→ **门限宽松** → 更容易接受这个点作为"潜在匹配"
- **不确定性小**（$\sigma_l$ 小）→ **门限严格** → 只有非常小的残差才被接受

物理解释：高不确定性的测量本身就"模糊"，所以不应该要求它与平面完美对齐。

---

### 4.3 概率权重的计算：高斯分布似然

在通过 $\sigma$ 门限检验后，系统计算每个潜在匹配的**相对概率**。

在多个候选平面中，选择概率最高的那个（最大似然）。

#### 4.3.1 高斯分布的概率密度函数

假设残差 $d_{plane}$ 服从高斯分布 $\mathcal{N}(0, \sigma_l^2)$，其概率密度为：

$$
p(d_{plane}) = \frac{1}{\sqrt{2\pi}\cdot\sigma_l} \exp\left(-\frac{d_{plane}^2}{2\sigma_l^2}\right)
$$

提取常数因子，归一化后得到**相对概率**：

$$
p \propto \frac{1}{\sqrt{\sigma_l}} \exp\left(-\frac{d_{plane}^2}{2\sigma_l^2}\right)
$$

**公式→源码行号定位表（可点击）**

| 公式部分 | 含义 | 源码定位 |
|---|---|---|
| $\mathcal{N}(0,\sigma_l^2)$ | 残差的高斯假设 | [src/voxel_map.cpp#L802](../src/voxel_map.cpp#L802) |
| $\frac{1}{\sqrt{\sigma_l}}$ | 正规化因子（方差小时高峰更尖锐） | [src/voxel_map.cpp#L802](../src/voxel_map.cpp#L802) |
| $\exp\left(-\frac{d_{plane}^2}{2\sigma_l^2}\right)$ | 指数衰减项（残差越大，越不可能） | [src/voxel_map.cpp#L802](../src/voxel_map.cpp#L802) |

#### 4.3.2 最大似然匹配选择

系统对一个点尝试所有候选平面，计算每个平面的概率：

$$
\text{prob}_i = \frac{1}{\sqrt{\sigma_{l,i}}} \exp\left(-\frac{d_{plane,i}^2}{2\sigma_{l,i}^2}\right)
$$

选择使 $\text{prob}_i$ 最大的平面作为**最终匹配平面**。

代码实现（[src/voxel_map.cpp#L802](../src/voxel_map.cpp#L802)）：
```cpp
double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
if (this_prob > prob)
{
  prob = this_prob;
  // 保存这个平面作为当前最佳匹配
  single_ptpl.dis_to_plane_ = ...
  single_ptpl.normal_ = plane.normal_;
  single_ptpl.plane_var_ = plane.plane_var_;
  // ...
}
```

**物理含义**：
- 不确定性**小**（$\sigma_l$ 小）且残差**小**（$d_{plane}$ 小）→ 概率高 → 置信度高 ✓
- 不确定性**大**（$\sigma_l$ 大），即使残差中等 → 概率相对较高 → 不算太坏 ✓
- 不确定性**大**且**残差巨大** → 概率低 → 不是好匹配 ✗

---

### 4.4 测量权重的构造：反演协方差

最终，每个**有效匹配**的点都分配一个**权重**，用于加权最小二乘法。

权重与**方差成反比**，体现"低方差（高确定性）的约束更强"的原理。

$$
w_i = \frac{1}{\sigma_l^2 + \epsilon}
$$

其中 $\epsilon = 0.001$ 是防止数值奇异的小常数。

**公式→源码行号定位表（可点击）**

| 概念 | 含义 | 源码定位 |
|---|---|---|
| $w_i \propto \frac{1}{\sigma_l^2}$ | 权重与方差的倒数成正比 | 后续EKF求解  |
| $\epsilon = 0.001$ | 数值稳定性的正则化常数 | [src/voxel_map.cpp#L456](../src/voxel_map.cpp#L456) |
| $R^{-1}$ 矩阵 | 所有权重构成的对角阵 | 后续送入H^T R^{-1} H |

**核心原理**：uncertainty 大 → 权重小 → 残差对状态更新的影响变弱 → 系统自动降低可疑约束的贡献。

---

### 4.5 加权残差进入EKF求解

一旦所有有效点的权重 $w_i$ 计算完毕，它们直接进入**加权最小二乘**的正规方程：

#### 4.5.1 加权Jacobian矩阵

$$
\tilde{\mathbf{H}} = \sqrt{\mathbf{W}} \cdot \mathbf{H}
$$

其中 $\mathbf{W} = \text{diag}(w_1, w_2, \ldots, w_n)$ 是对角权重矩阵。

代码在构建 `Hsub_T_R_inv` 时实现（[src/voxel_map.cpp#L462-L463](../src/voxel_map.cpp#L462-L463)）：
```cpp
for (int i = 0; i < effective_num; i++) {
  Hsub_T_R_inv.col(i) = Hsub.col(i) * R_inv(i);  // R_inv(i) = w_i
}
```

#### 4.5.2 加权右端项

$$
\tilde{\mathbf{z}} = \sqrt{\mathbf{W}} \cdot \mathbf{z}
$$

$$
\mathbf{b} = \tilde{\mathbf{H}}^T \tilde{\mathbf{z}} = \mathbf{H}^T \mathbf{W} \mathbf{z}
$$

代码（[src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521)）：
```cpp
HTz = Hsub_T_R_inv * meas_vec;  // meas_vec 包含所有残差 z_i
```

#### 4.5.3 加权Hessian矩阵

$$
\mathbf{A} = \tilde{\mathbf{H}}^T \tilde{\mathbf{H}} = \mathbf{H}^T \mathbf{W} \mathbf{H}
$$

代码（[src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523)）：
```cpp
Hsub_T_R_inv_Hsub = Hsub_T_R_inv * Hsub.transpose();  // H^T W H
```

#### 4.5.4 求解增量

$$
\Delta\mathbf{x} = \mathbf{A}^{-1} \mathbf{b} = (\mathbf{H}^T\mathbf{W}\mathbf{H})^{-1} \mathbf{H}^T\mathbf{W}\mathbf{z}
$$

**核心作用链**：不确定性大的点 → 权重小 → 在 Hessian 和右端项中贡献少 → 状态更新时被"轻视" → 系统更稳健。

这就是 FAST-LIVO2-RTK 自适应减重的精髓。

---

## 5. uncertainty 如何用于数据关联门限

在 `build_single_residual(...)` 里，匹配并非只看距离，也看方差归一化：

- 组合方差：
  - `src/voxel_map.cpp:790`
  - `src/voxel_map.cpp:791`

- 统计门限（$\sigma$ 门限）：
$$
|d_{plane}| < \sigma_{num}\sqrt{\sigma_l}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $|d_{plane}| < \sigma_{num}\sqrt{\sigma_l}$ | [src/voxel_map.cpp#L799](../src/voxel_map.cpp#L799) |

对应：`src/voxel_map.cpp:792`

- 似然概率：
$$
p \propto \frac{1}{\sqrt{\sigma_l}}\exp\left(-\frac{d^2}{2\sigma_l}\right)
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $p \propto \frac{1}{\sqrt{\sigma_l}}\exp\left(-\frac{d^2}{2\sigma_l}\right)$ | [src/voxel_map.cpp#L802](../src/voxel_map.cpp#L802) |

对应：`src/voxel_map.cpp:795`

这表示系统会优先保留“残差小且不确定性合理”的对应关系，而不是仅仅绝对距离最小。

---

## 5. RTK uncertainty 的进入方式

当 `rtk_good` 时追加 3 个观测（x/y/z）：

- 加权 Jacobian：`Hsub_T_R_inv.col(i)`
  - `src/voxel_map.cpp:460`

- 右端项：
$$
\mathbf{H}^T\mathbf{W}\mathbf{z}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{H}^T\mathbf{W}\mathbf{z}$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521) |

对应：`HTz = Hsub_T_R_inv * meas_vec`（`src/voxel_map.cpp:519`）

- Hessian 近似：
$$
\mathbf{H}^T\mathbf{W}\mathbf{H}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{H}^T\mathbf{W}\mathbf{H}$ | [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523) |

对应：`src/voxel_map.cpp:521`

- 先验融合求解：
$$
(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})^{-1}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})^{-1}$ | [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |

对应 `K_1`：`src/voxel_map.cpp:523`

- 协方差后验更新：
  - `src/voxel_map.cpp:544`

因此 uncertainty 最终直接决定：
1. 每个约束在法方程里的权重
2. 后验协方差收缩的速度

---

## 6. uncertainty 的可视化使用

当 `rtk_good` 时追加 3 个观测（x/y/z）：
- 开关与维度扩展：`src/voxel_map.cpp:411`

RTK 标准差设定：
- `rtk_std << 0.003, 0.003, 0.1`（`src/voxel_map.cpp:476`）

每个维度权重：
$$
w_{rtk,k} = \frac{1}{\sigma_{rtk,k}^2}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $w_{rtk,k} = \frac{1}{\sigma_{rtk,k}^2}$ | [src/voxel_map.cpp#L498](../src/voxel_map.cpp#L498) |

对应：`src/voxel_map.cpp:496`

这意味着：
- x/y（3mm）权重大
- z（10cm）权重小

即 RTK 在平面更强约束，竖直相对弱。

---

## 7. 从参数调节角度理解 uncertainty

平面可视化时，使用 `plane_var_` 的对角线迹作为不确定性指标：
- `plane_cov = plane_var_.block<3,3>(0,0).diagonal()`：`src/voxel_map.cpp:858`
- `trace = plane_cov.sum()`：`src/voxel_map.cpp:859`

trace 越大，表示平面参数越不确定。

---

## 8. 一句话总结

这些参数直接控制 uncertainty：
- `beam_err_`, `dept_err_`：点测量噪声（`calcBodyCov` 输入）
- `sigma_num_`：关联门限严格度（`src/voxel_map.cpp:792`）
- IMU 噪声参数（`cov_gyr`, `cov_acc`, `cov_bias_*`）影响 `state_.cov` 传播

调参方向：
- 如果误匹配多：减小 `sigma_num_` 或增大测量噪声建模一致性
- 如果收敛过慢：检查是否把不确定性设得过大（权重过小）
- 如果抖动大：检查 IMU 过程噪声是否过小（导致过度自信）

---

---

该系统的 uncertainty 机制是一个完整闭环：

**传感器噪声建模（点） + 状态协方差传播（IMU） + 地图几何方差（平面） -> 统一形成残差权重与门限 -> 决定关联质量与 EKF 更新强度。**

所以它不仅“算了方差”，而是把方差贯穿到了**匹配、加权、求解、可视化**全流程。