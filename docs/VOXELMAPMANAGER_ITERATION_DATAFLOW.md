# VoxelMapManager 每轮迭代的矩阵构建与求解数据流

## 1. 适用范围

本文对应 `VoxelMapManager::StateEstimation(...)` 的单帧 EKF 迭代流程，核心代码在：
- `src/voxel_map.cpp`（函数 `StateEstimation`）
- `include/voxel_map.h`（类与数据结构定义）

目标：说明每轮迭代里 `Hsub`、`R_inv`、`meas_vec`、`H_T_H`、`K_1`、`G`、`solution` 的构建、流转和更新关系。

---

## 2. 单帧总体流程（进入迭代前）

输入：
- 预测状态 `state_propagat`（来自 IMU 传播）
- 当前状态 `state_`（上一帧/上一轮结果）
- 当前降采样点云 `feats_down_body_`
- 地图结构 `voxel_map_`
- `rtk_good` 与 `rtk_means`（可选）

进入迭代前做两件事：
1. 为每个点计算体坐标噪声协方差 `body_cov_list_`
2. 预计算每个点的叉乘矩阵 `cross_mat_list_`

这两步的目的是减少每轮迭代中的重复计算。

---

## 3. 每轮迭代数据流图（核心）

```text
for iterCount in [0, max_iterations):

  A) 用当前 state_ 把点云变换到世界系
     world_lidar = TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_)

  B) 构建 pv_list_（每个点的世界点 + 不确定性）
     pv.var = R * body_cov * R^T + 姿态噪声项 + 平移噪声项

  C) 建立几何残差集合（并行）
     BuildResidualListOMP(pv_list_, ptpl_list_)
     -> ptpl_list_ = 有效点到平面约束集合

  D) 决定测量维度
     effct_feat_num_ = ptpl_list_.size()
     if (rtk_good) effct_feat_num_ += 3

  E) 分配测量相关矩阵/向量
     Hsub            : [effct_feat_num_ x 6]
     Hsub_T_R_inv    : [6 x effct_feat_num_]
     R_inv           : [effct_feat_num_]
     meas_vec        : [effct_feat_num_]

  F) 填充 LIO 约束（前 N 行）
     对每个 point-to-plane 约束 i:
       - 计算测量方差（平面参数方差 + 点协方差投影）
       - R_inv(i) = 1 / sigma_i^2
       - Hsub(i,:) = [旋转雅可比(3), 平移雅可比(3)]
       - Hsub_T_R_inv(:,i) = Hsub(i,:)^T * R_inv(i)
       - meas_vec(i) = -dis_to_plane

  G) 填充 RTK 约束（最后 3 行，可选）
     if (rtk_good):
       - rtk_residual = rtk_means - (state_.pos_end + state_.rot_end * rtk_ext_t)
       - 对 x/y/z 三个分量分别写入 Hsub、R_inv、meas_vec

  H) 形成法方程的关键项
     HTz   = Hsub_T_R_inv * meas_vec
     H_T_H(0:6,0:6) = Hsub_T_R_inv * Hsub

  I) 融合先验协方差并求解增量
     K_1 = (H_T_H + P^{-1})^{-1}
     G   = K_1 * H_T_H

     vec = state_propagat - state_
     solution = K_1 * HTz + vec - G * vec(前6维)

  J) 状态更新
     state_ += solution

  K) 收敛判断
     若旋转增量和位移增量足够小 -> 记录收敛

  L) 达到停止条件后更新后验协方差
     P = (I - G) * P

end for
```

---

## 4. 矩阵/向量维度与物理意义

### 4.1 测量模型

线性化后统一写成：

$$
\mathbf{z} \approx \mathbf{H}\,\delta \mathbf{x}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{z} \approx \mathbf{H}\delta \mathbf{x}$ | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464), [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521) |

其中：
- $\delta \mathbf{x}$ 是状态增量（这里实际只显式用到位姿相关 6 维）
- $\mathbf{H}$ 对应代码中的 `Hsub`
- $\mathbf{z}$ 对应代码中的 `meas_vec`

### 4.2 代码中的对象

- `Hsub`：`[M x 6]`，M 为有效约束数（LIO 点数 + 可选 RTK 三轴）
- `R_inv`：`[M]`，每个约束对应的逆方差（信息量）
- `Hsub_T_R_inv`：`[6 x M]`，等价于 `H^T * W`（W 为对角权重）
- `HTz`：`[6 x 1]`，右端项
- `H_T_H`：`[DIM_STATE x DIM_STATE]`，其中前 6x6 块放 `H^TWH`
- `K_1`：近似后验信息矩阵的逆
- `G`：用于协方差更新的增益相关项
- `solution`：`[DIM_STATE x 1]`，本轮求得状态增量

---

## 5. LIO 约束是如何写入矩阵的

对每个点到平面约束：
1. 取法向 `normal`、平面中心 `center`、点坐标 `point_this`
2. 计算测量噪声：
   - 平面参数不确定性项：`J_nq * plane_var * J_nq^T`
   - 点不确定性在法向投影项：`normal^T * var * normal`
3. 令：
   - `R_inv(i) = 1 / (0.001 + 上述两项)`
4. 写入雅可比：
   - `Hsub.row(i) = [A(0),A(1),A(2), nx, ny, nz]`
5. 写入残差：
   - `meas_vec(i) = -dis_to_plane`

这一步本质上是加权点到平面最小二乘。

---

## 6. RTK 约束是如何拼接的（`rtk_good=true`）

当 `rtk_good` 为真时：
- 将测量维度增加 3（对应 x/y/z）
- 在 `Hsub` 最后 3 行追加 RTK 观测方程
- 在 `R_inv` 最后 3 个元素写入 RTK 逆方差
- 在 `meas_vec` 最后 3 个元素写入 RTK 位置残差

当前实现中 `rtk_ext_t = [0,0,0]`，因此旋转雅可比被置零，RTK 主要直接约束平移分量。

---

## 7. 求解器视角：为什么这么写

代码实现的是“迭代 EKF / 信息形式高斯牛顿”的混合写法：

$$
\delta x = (H^T W H + P^{-1})^{-1} H^T W z \quad (\text{并叠加传播先验项修正})
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\delta x = (H^TWH + P^{-1})^{-1}H^TWz$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521), [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523), [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |

其中：
- $H^TWH$ 来自当前帧几何约束
- $P^{-1}$ 来自传播先验（抑制病态和过拟合）
- 多轮迭代通过不断更新 `state_` 重新线性化

所以每轮本质是：
1. 线性化
2. 加权最小二乘
3. 状态增量更新
4. 判断收敛

---

## 8. 为什么这套每轮数据流高效

1. **局部线性化**：每轮只围绕当前 `state_` 线性化，矩阵规模受控（6 维主块）
2. **稀疏观测压缩**：点云先经过体素/平面筛选，只保留有效约束
3. **并行构残差**：`BuildResidualListOMP` 将最重的对应构建并行化
4. **先验融合稳定**：`+ P^{-1}` 提升可解性和数值稳定性
5. **早停机制**：收敛阈值触发后不再做无效迭代

---

## 9. 关键实现位置索引

- `VoxelMapManager` 类定义：`include/voxel_map.h`
- 迭代主循环：`src/voxel_map.cpp` 中 `StateEstimation(...)`
- 残差构建（并行）：`BuildResidualListOMP(...)`
- 单点递归匹配：`build_single_residual(...)`
- 地图更新：`BuildVoxelMap(...)` / `UpdateVoxelMap(...)`

---

## 10. 一页版结论

`StateEstimation` 的每轮迭代可以概括为：

**“当前状态重投影 -> 构建加权几何/RTK观测 -> 形成法方程 -> 与先验融合求增量 -> 更新状态与协方差 -> 收敛判定”**。

这也是 FAST-LIVO2-RTK 在前端实现稳定、实时位姿优化的核心数据流。

---

## 11. ESKF 构造过程（从系统模型到实现）

> 2026-07-28 补记：`voxel_map.cpp` 在本节最初写完之后新增了一处 `// todo 可能有越界？？？` 注释
> （[src/voxel_map.cpp#L414](../src/voxel_map.cpp#L414)，对应 `rtk_good` 时 `ptpl_list_[i]` 可能越界的问题），
> 插入了 2 行，本节及第12/15/16节此前引用的行号已按当前代码重新核对更正。

这一节给出该前端优化里 ESKF 的标准构造方式，并与当前实现一一对应。

> **完全没接触过 ESKF？先读文末的 [§18 误差状态从零详解](#18-误差状态从零详解eskf-入门必读)**——那里把
> “误差状态到底是什么、为什么要有它、$\boxplus/\boxminus$ 怎么算、姿态为什么不能直接相加” 一步步讲清楚，
> 读完再回来看 11/12/15/17 节的公式会顺很多。本节（11.1）只做最小定义。

### 11.1 名义状态与误差状态

名义状态（可按 19 维理解）可写作：

$$
\mathbf{x} = \left(\mathbf{R},\ \mathbf{p},\ \mathbf{v},\ \mathbf{b}_g,\ \mathbf{b}_a,\ \mathbf{g},\ \eta\right)
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{x}=(\mathbf{R},\mathbf{p},\mathbf{v},\mathbf{b}_g,\mathbf{b}_a,\mathbf{g},\eta)$ | [include/common_lib.h#L31](../include/common_lib.h#L31), [include/common_lib.h#L232](../include/common_lib.h#L232) |

其中：
- $\mathbf{R}\in SO(3)$：姿态
- $\mathbf{p},\mathbf{v}\in\mathbb{R}^3$：位置、速度
- $\mathbf{b}_g,\mathbf{b}_a\in\mathbb{R}^3$：陀螺/加计偏置
- $\mathbf{g}\in\mathbb{R}^3$：重力
- $\eta$：扩展标量状态（项目里保留该扩展维）

误差状态采用左乘误差：

$$
\delta \mathbf{x} = \left(\delta\boldsymbol{\theta},\ \delta\mathbf{p},\ \delta\mathbf{v},\ \delta\mathbf{b}_g,\ \delta\mathbf{b}_a,\ \delta\mathbf{g},\ \delta\eta\right)
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\delta\mathbf{x}=(\delta\boldsymbol{\theta},\delta\mathbf{p},\delta\mathbf{v},\delta\mathbf{b}_g,\delta\mathbf{b}_a,\delta\mathbf{g},\delta\eta)$ | [include/common_lib.h#L232](../include/common_lib.h#L232), [src/IMU_Processing.cpp#L401](../src/IMU_Processing.cpp#L401) |

姿态注入使用：

$$
\mathbf{R} \leftarrow \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\theta})
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{R} \leftarrow \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\theta})$ | [src/voxel_map.cpp#L531](../src/voxel_map.cpp#L531) |

这与代码中的 `state_ += solution` 一致（旋转在李群上更新）。

### 11.2 预测（Propagation）

IMU 传播阶段给出先验：

$$
\mathbf{x}_{k|k-1},\ \mathbf{P}_{k|k-1}
$$

在本函数内，这个先验通过 `state_propagat`（名义先验）和 `state_.cov`（协方差）进入测量更新。

### 11.3 观测模型（LIO + 可选 RTK）

统一线性化模型：

$$
\mathbf{r}(\mathbf{x}) \approx \mathbf{r}(\hat{\mathbf{x}}) + \mathbf{H}\,\delta\mathbf{x}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{r}(\mathbf{x}) \approx \mathbf{r}(\hat{\mathbf{x}})+\mathbf{H}\delta\mathbf{x}$ | [src/voxel_map.cpp#L461](../src/voxel_map.cpp#L461), [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464) |

在实现中：
- 残差向量：`meas_vec`（等价 $\mathbf{z}$）
- 雅可比：`Hsub`（等价 $\mathbf{H}$ 的位姿块）
- 权重：`R_inv`（对角信息阵）

LIO 点到平面残差：

$$
r_i = -d_i
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $r_i=-d_i$ | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464) |

对应每行雅可比（对位姿 6 维）：

$$
\mathbf{H}_i = \begin{bmatrix} \mathbf{J}_{\theta,i} & \mathbf{n}_i^T \end{bmatrix}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{H}_i=[\mathbf{J}_{\theta,i},\mathbf{n}_i^T]$ | [src/voxel_map.cpp#L460](../src/voxel_map.cpp#L460), [src/voxel_map.cpp#L461](../src/voxel_map.cpp#L461) |

RTK 开启时，再拼接 3 条位置约束（x/y/z）。

---

## 12. ESKF 更新公式（与代码变量逐项对应）

### 12.1 信息形式的加权最小二乘

令测量噪声协方差为 $\mathbf{R}_m$，其逆为 $\mathbf{W}=\mathbf{R}_m^{-1}$（对角）。

法方程：

$$
\left(\mathbf{H}^T\mathbf{W}\mathbf{H} + \mathbf{P}^{-1}\right)\delta\mathbf{x}
= \mathbf{H}^T\mathbf{W}\mathbf{z}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})\delta\mathbf{x}=\mathbf{H}^T\mathbf{W}\mathbf{z}$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521), [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523), [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |

与代码映射：
- $\mathbf{H}^T\mathbf{W}\mathbf{z}$ ↔ `HTz = Hsub_T_R_inv * meas_vec`
- $\mathbf{H}^T\mathbf{W}\mathbf{H}$ ↔ `H_T_H.block<6,6>(0,0) = Hsub_T_R_inv * Hsub`
- $\left(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1}\right)^{-1}$ ↔ `K_1`

### 12.2 本实现中的迭代增量写法

代码采用“先验偏移补偿”项：

$$
\delta\mathbf{x} = \mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z}
+ \mathbf{v}
- \mathbf{G}\,\mathbf{v}_{pose}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\delta\mathbf{x}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z}+\mathbf{v}-\mathbf{G}\mathbf{v}_{pose}$ | [src/voxel_map.cpp#L526](../src/voxel_map.cpp#L526)（`G`）, [src/voxel_map.cpp#L527](../src/voxel_map.cpp#L527)（`vec`）, [src/voxel_map.cpp#L529](../src/voxel_map.cpp#L529)（`solution`赋值） |

其中：
- $\mathbf{v}=\mathbf{x}_{k|k-1}\ominus\mathbf{x}$ ↔ `vec = state_propagat - state_`
- $\mathbf{G}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{H}$ ↔ `G`
- `solution` 即 $\delta\mathbf{x}$

对应实现语句：

`solution = K_1 * HTz + vec - G * vec(0:5)`

### 12.3 状态注入与协方差更新

状态注入：

$$
\hat{\mathbf{x}} \leftarrow \hat{\mathbf{x}} \boxplus \delta\mathbf{x}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\hat{\mathbf{x}} \leftarrow \hat{\mathbf{x}} \boxplus \delta\mathbf{x}$ | [src/voxel_map.cpp#L531](../src/voxel_map.cpp#L531) |

代码中为：`state_ += solution`。

后验协方差（近似 Joseph 简化形式之一）：

$$
\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{G})\mathbf{P}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{G})\mathbf{P}$ | [src/voxel_map.cpp#L546](../src/voxel_map.cpp#L546) |

代码中为：

`state_.cov = (I_STATE - G) * state_.cov`

---

## 13. 从 ESKF 角度理解每轮迭代的意义

每一轮迭代都在做同一件事：
1. 用当前 $\hat{\mathbf{x}}$ 重建残差与雅可比（重新线性化）
2. 解带先验约束的增量 $\delta\mathbf{x}$
3. 注入到李群状态并检查收敛

因此它是“迭代 ESKF”（IEKF）而不是一次性 EKF。

---

## 14. 最小复现版公式清单

给实现最关键的 5 条公式如下：

$$
\mathbf{z} \approx \mathbf{H}\delta\mathbf{x}
$$

$$
\mathbf{A}=\mathbf{H}^T\mathbf{W}\mathbf{H},\quad
\mathbf{b}=\mathbf{H}^T\mathbf{W}\mathbf{z}
$$

$$
\mathbf{K}_1=(\mathbf{A}+\mathbf{P}^{-1})^{-1}
$$

$$
\delta\mathbf{x}=\mathbf{K}_1\mathbf{b}+\mathbf{v}-\mathbf{G}\mathbf{v}_{pose}
$$

$$
\hat{\mathbf{x}}\leftarrow\hat{\mathbf{x}}\boxplus\delta\mathbf{x},\quad
\mathbf{P}\leftarrow(\mathbf{I}-\mathbf{G})\mathbf{P}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{z} \approx \mathbf{H}\delta\mathbf{x}$ | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464), [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521) |
| $\mathbf{A}=\mathbf{H}^T\mathbf{W}\mathbf{H}$ | [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523) |
| $\mathbf{b}=\mathbf{H}^T\mathbf{W}\mathbf{z}$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521) |
| $\mathbf{K}_1=(\mathbf{A}+\mathbf{P}^{-1})^{-1}$ | [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |
| $\delta\mathbf{x}=\mathbf{K}_1\mathbf{b}+\mathbf{v}-\mathbf{G}\mathbf{v}_{pose}$ | [src/voxel_map.cpp#L529](../src/voxel_map.cpp#L529) |
| $\hat{\mathbf{x}}\leftarrow\hat{\mathbf{x}}\boxplus\delta\mathbf{x}$ | [src/voxel_map.cpp#L531](../src/voxel_map.cpp#L531) |
| $\mathbf{P}\leftarrow(\mathbf{I}-\mathbf{G})\mathbf{P}$ | [src/voxel_map.cpp#L546](../src/voxel_map.cpp#L546) |

按这 5 条读代码，就能把该函数的 ESKF 主干完整对上。

---

## 15. 推导版附录（与代码一一对应）

本附录目标：把 `StateEstimation(...)` 里的每个核心矩阵公式，从数学推导到实现变量名完整对齐。

对应函数：
- `src/voxel_map.cpp` 的 `VoxelMapManager::StateEstimation(...)`

### 15.1 几何残差定义（点到平面）

对第 $i$ 个匹配点，平面写成：

$$
\pi_i: \mathbf{n}_i^T\mathbf{x} + d_i = 0
$$

点 $\mathbf{p}_i^w$ 到平面的有符号距离残差：

$$
r_i = \mathbf{n}_i^T\mathbf{p}_i^w + d_i
$$

代码里采用 `meas_vec(i) = -dis_to_plane_`，所以更新时等价于逼近：

$$
\mathbf{H}\,\delta\mathbf{x} \approx \mathbf{z},\quad z_i=-r_i
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\pi_i: \mathbf{n}_i^T\mathbf{x}+d_i=0$ | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464) |
| $r_i=\mathbf{n}_i^T\mathbf{p}_i^w+d_i$ | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464) |
| $\mathbf{H}\delta\mathbf{x}\approx\mathbf{z},\ z_i=-r_i$ | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464), [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521) |

对应代码变量：
- `ptpl_list_[i].normal_` ↔ $\mathbf{n}_i$
- `ptpl_list_[i].d_` ↔ $d_i$
- `ptpl_list_[i].dis_to_plane_` ↔ $r_i$
- `meas_vec(i)` ↔ $z_i$

### 15.2 位姿扰动线性化

设当前位姿扰动为 $(\delta\boldsymbol{\theta},\delta\mathbf{t})$，点的世界坐标一阶近似：

$$
\mathbf{p}' \approx \mathbf{p} + [\mathbf{p}]_\times\,\delta\boldsymbol{\theta} + \delta\mathbf{t}
$$

残差增量：

$$
\delta r_i \approx \underbrace{\mathbf{n}_i^T[\mathbf{p}_i]_\times}_{J_{\theta,i}}\delta\boldsymbol{\theta}
+ \underbrace{\mathbf{n}_i^T}_{J_{t,i}}\delta\mathbf{t}
$$

因此每行雅可比：

$$
\mathbf{H}_i = \begin{bmatrix}J_{\theta,i} & J_{t,i}\end{bmatrix}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{p}' \approx \mathbf{p} + [\mathbf{p}]_\times\delta\boldsymbol{\theta}+\delta\mathbf{t}$ | [src/voxel_map.cpp#L460](../src/voxel_map.cpp#L460) |
| $\delta r_i \approx \mathbf{n}_i^T[\mathbf{p}_i]_\times\delta\boldsymbol{\theta}+\mathbf{n}_i^T\delta\mathbf{t}$ | [src/voxel_map.cpp#L460](../src/voxel_map.cpp#L460) |
| $\mathbf{H}_i=[J_{\theta,i},J_{t,i}]$ | [src/voxel_map.cpp#L461](../src/voxel_map.cpp#L461) |

对应代码：
- `A = point_crossmat * state_.rot_end.transpose() * normal`
- `Hsub.row(i) << A[0],A[1],A[2], n_x,n_y,n_z`

### 15.3 测量权重推导（异方差加权）

实现中每个残差方差采用：

$$
\sigma_i^2 \approx \sigma_{plane,i}^2 + \sigma_{point\to n,i}^2 + \epsilon
$$

其中：

$$
\sigma_{plane,i}^2 = J_{nq,i}\,\Sigma_{\pi,i}\,J_{nq,i}^T
$$

$$
\sigma_{point\to n,i}^2 = \mathbf{n}_i^T\Sigma_{p,i}\mathbf{n}_i
$$

于是权重（信息量）：

$$
w_i = \frac{1}{\sigma_i^2}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\sigma_i^2 \approx \sigma_{plane,i}^2+\sigma_{point\to n,i}^2+\epsilon$ | [src/voxel_map.cpp#L454](../src/voxel_map.cpp#L454), [src/voxel_map.cpp#L456](../src/voxel_map.cpp#L456) |
| $\sigma_{plane,i}^2 = J_{nq,i}\Sigma_{\pi,i}J_{nq,i}^T$ | [src/voxel_map.cpp#L454](../src/voxel_map.cpp#L454) |
| $\sigma_{point\to n,i}^2 = \mathbf{n}_i^T\Sigma_{p,i}\mathbf{n}_i$ | [src/voxel_map.cpp#L452](../src/voxel_map.cpp#L452)（`var`）, [src/voxel_map.cpp#L456](../src/voxel_map.cpp#L456)（投影到法向） |
| $w_i=\frac{1}{\sigma_i^2}$ | [src/voxel_map.cpp#L456](../src/voxel_map.cpp#L456) |

对应代码：
- `sigma_l = J_nq * plane_var * J_nq.transpose()`
- `R_inv(i) = 1 / (0.001 + sigma_l + normal^T var normal)`

### 15.4 组装全局加权最小二乘

把全部残差堆叠：

$$
\mathbf{z}=\begin{bmatrix}z_1 & \cdots & z_M\end{bmatrix}^T,
\quad
\mathbf{H}=\begin{bmatrix}\mathbf{H}_1\\ \vdots \\ \mathbf{H}_M\end{bmatrix}
$$

令 $\mathbf{W}=\mathrm{diag}(w_1,\dots,w_M)$，最小化：

$$
\min_{\delta\mathbf{x}}\ \|\mathbf{H}\delta\mathbf{x}-\mathbf{z}\|_{\mathbf{W}}^2
$$

法方程：

$$
\mathbf{H}^T\mathbf{W}\mathbf{H}\,\delta\mathbf{x}
=\mathbf{H}^T\mathbf{W}\mathbf{z}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{z}=[z_1,\cdots,z_M]^T,\ \mathbf{H}=[\mathbf{H}_1^T,\cdots,\mathbf{H}_M^T]^T$ | [src/voxel_map.cpp#L461](../src/voxel_map.cpp#L461), [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464) |
| $\min_{\delta\mathbf{x}}\|\mathbf{H}\delta\mathbf{x}-\mathbf{z}\|_{\mathbf{W}}^2$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521), [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523) |
| $\mathbf{H}^T\mathbf{W}\mathbf{H}\delta\mathbf{x}=\mathbf{H}^T\mathbf{W}\mathbf{z}$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521), [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523) |

对应代码：
- `Hsub_T_R_inv.col(i) = H_i^T * w_i`
- `HTz = Hsub_T_R_inv * meas_vec`
- `H_T_H(0:6,0:6) = Hsub_T_R_inv * Hsub`

### 15.5 融合先验（ESKF 核心）

ESKF 不是只看当前测量，还融合传播先验 $(\hat x_{k|k-1},P_{k|k-1})$。

写成 MAP 形式：

$$
\min_{\delta\mathbf{x}}
\underbrace{\|\mathbf{H}\delta\mathbf{x}-\mathbf{z}\|_{\mathbf{W}}^2}_{\text{测量项}}
+
\underbrace{\|\delta\mathbf{x}-\mathbf{v}\|_{\mathbf{P}^{-1}}^2}_{\text{先验项}}
$$

其中 $\mathbf{v}=\hat x_{k|k-1}\ominus \hat x$（代码里的 `vec`）。

正规解：

$$
(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})\delta\mathbf{x}
=\mathbf{H}^T\mathbf{W}\mathbf{z}+\mathbf{P}^{-1}\mathbf{v}
$$

实现里等价写成（把先验偏移分成 `vec - G*vec_pose` 结构）：

$$
\delta\mathbf{x}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z}
+\mathbf{v}-\mathbf{G}\mathbf{v}_{pose}
$$

$$
\mathbf{K}_1=(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})^{-1},
\quad
\mathbf{G}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{H}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\min_{\delta\mathbf{x}}\|\mathbf{H}\delta\mathbf{x}-\mathbf{z}\|_{\mathbf{W}}^2+\|\delta\mathbf{x}-\mathbf{v}\|_{\mathbf{P}^{-1}}^2$ | [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525), [src/voxel_map.cpp#L529](../src/voxel_map.cpp#L529) |
| $(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})\delta\mathbf{x}=\mathbf{H}^T\mathbf{W}\mathbf{z}+\mathbf{P}^{-1}\mathbf{v}$ | [src/voxel_map.cpp#L521](../src/voxel_map.cpp#L521), [src/voxel_map.cpp#L523](../src/voxel_map.cpp#L523), [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |
| $\delta\mathbf{x}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z}+\mathbf{v}-\mathbf{G}\mathbf{v}_{pose}$ | [src/voxel_map.cpp#L529](../src/voxel_map.cpp#L529) |
| $\mathbf{K}_1=(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})^{-1}$ | [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |
| $\mathbf{G}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{H}$ | [src/voxel_map.cpp#L526](../src/voxel_map.cpp#L526) |

对应代码：
- `K_1 = (H_T_H + P^{-1})^{-1}`
- `G = K_1 * H_T_H`
- `solution = K_1*HTz + vec - G*vec(前6维)`

### 15.6 迭代更新（IEKF）

每轮都重新线性化（因为残差关于姿态/位置是非线性的）：

$$
\hat x \leftarrow \hat x \boxplus \delta\mathbf{x}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\hat x \leftarrow \hat x \boxplus \delta\mathbf{x}$ | [src/voxel_map.cpp#L531](../src/voxel_map.cpp#L531) |

代码：`state_ += solution`。

这一步之后，再用新 `state_` 进入下一轮，直到收敛阈值满足：
- 旋转增量很小
- 平移增量很小

### 15.7 协方差后验更新

当前实现采用简化形式：

$$
\mathbf{P}\leftarrow(\mathbf{I}-\mathbf{G})\mathbf{P}
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{P}\leftarrow(\mathbf{I}-\mathbf{G})\mathbf{P}$ | [src/voxel_map.cpp#L546](../src/voxel_map.cpp#L546) |

代码：`state_.cov = (I_STATE - G) * state_.cov`。

说明：这里 `G` 仅在位姿相关块上显式构造，符合该函数“位姿主更新”的实现策略。

### 15.8 RTK 约束的推导拼接

RTK 残差（3 维）：

$$
\mathbf{r}_{rtk} = \mathbf{p}_{rtk}^{meas} - \left(\mathbf{p} + \mathbf{R}\mathbf{t}_{ext}\right)
$$

若当前实现取 `rtk_ext_t = 0`，则：

$$
\frac{\partial \mathbf{r}_{rtk}}{\partial \delta\boldsymbol{\theta}} = \mathbf{0},
\quad
\frac{\partial \mathbf{r}_{rtk}}{\partial \delta\mathbf{t}} = \mathbf{I}_3
$$

**公式→源码行号定位表（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{r}_{rtk}=\mathbf{p}_{rtk}^{meas}-(\mathbf{p}+\mathbf{R}\mathbf{t}_{ext})$ | [src/voxel_map.cpp#L480](../src/voxel_map.cpp#L480)-[L481](../src/voxel_map.cpp#L481) |
| $\frac{\partial \mathbf{r}_{rtk}}{\partial \delta\boldsymbol{\theta}}=\mathbf{0}$ | [src/voxel_map.cpp#L489](../src/voxel_map.cpp#L489) |
| $\frac{\partial \mathbf{r}_{rtk}}{\partial \delta\mathbf{t}}=\mathbf{I}_3$ | [src/voxel_map.cpp#L491](../src/voxel_map.cpp#L491) |

因此 RTK 三行雅可比就是 `[0_{3x3} , I_{3x3}]`，直接加到 `Hsub` 尾部 3 行。

对应代码：
- `rtk_rot_jacobian.setZero()`
- `rtk_pos_jacobian = I`
- 对 `k=0..2` 写入 `Hsub(row_idx,:)`、`R_inv(row_idx)`、`meas_vec(row_idx)`

---

## 16. 公式到代码速查表（附录）

| 推导对象 | 数学式 | 代码变量/语句 |
|---|---|---|
| 点面残差 | $r_i=\mathbf{n}_i^T\mathbf{p}_i+d_i$ | `dis_to_plane_` |
| 观测向量 | $z_i=-r_i$ | `meas_vec(i) = -dis_to_plane_` |
| 雅可比行 | $H_i=[J_{\theta,i},J_{t,i}]$ | `Hsub.row(i) << A, n` |
| 权重 | $w_i=1/\sigma_i^2$ | `R_inv(i)=...` |
| 右端项 | $H^TWz$ | `HTz = Hsub_T_R_inv * meas_vec` |
| 海森近似 | $H^TWH$ | `H_T_H = Hsub_T_R_inv * Hsub` |
| 先验融合逆 | $(H^TWH+P^{-1})^{-1}$ | `K_1 = (...)^{-1}` |
| 增益块 | $G=K_1H^TWH$ | `G = K_1 * H_T_H` |
| 增量 | $\delta x=K_1H^TWz+v-Gv_{pose}$ | `solution = ...` |
| 状态注入 | $x\leftarrow x\boxplus\delta x$ | `state_ += solution` |
| 协方差更新 | $P\leftarrow(I-G)P$ | `state_.cov = (I_STATE - G) * state_.cov` |

到这里为止，推导链条已经和实现完全对齐：

**残差定义 → 雅可比线性化 → 异方差加权 → 先验融合 → 迭代注入 → 协方差更新**。

---

## 17. 误差状态雅可比矩阵详细推导（对照 FAST-LIVO2 论文）

> 参考文献：`Supplementary/2408.14035v2.pdf`（*FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual
> Odometry*, Zheng et al.）。对应章节：
> - **Sec. IV-A《Notations and State Transition Model》**：状态流形、名义状态与连续运动学（论文式(1)(2)、Table I）
> - **Sec. IV-C《Propagation》/ IV-D《Sequential Update》**：ESIKF 传播与更新（论文式(9)(10)(11)）
> - **Sec. VI-A《Point-to-plane LiDAR Measurement Model》**：LiDAR 点面测量模型（论文式(16)–(19)）
>
> 前面 11.2 节只说“预测给出先验 $(\hat{\mathbf{x}},\hat{\mathbf{P}})$”而没展开**传播侧**的误差状态雅可比；
> 本节补齐 $\mathbf{F}_{\tilde{\mathbf{x}}}$（状态转移雅可比）、$\mathbf{F}_{\mathbf{w}}$（过程噪声雅可比）的
> 逐块推导，并把 11.3/15.2 节的**测量侧**雅可比 $\mathbf{H}$ 从论文式(19) 严格重推一遍。传播侧代码在
> `src/IMU_Processing.cpp::UndistortPcl(...)`（而不是 `voxel_map.cpp`），本节行号据此标注。

### 17.1 状态流形与名义状态（论文式(1)(2)）

论文把状态定义在流形 $\mathcal{M}\triangleq SO(3)\times\mathbb{R}^{16}$，$\dim\mathcal{M}=19$：

$$
\mathbf{x}\triangleq\begin{bmatrix}{}^{G}\mathbf{R}_I & {}^{G}\mathbf{p}_I & {}^{G}\mathbf{v}_I & \mathbf{b}_g & \mathbf{b}_a & {}^{G}\mathbf{g} & \tau\end{bmatrix}^T
$$

输入、噪声与连续运动学 $f(\mathbf{x},\mathbf{u},\mathbf{w})$（论文式(2)）：

$$
\mathbf{u}=\begin{bmatrix}\boldsymbol{\omega}_m\\ \mathbf{a}_m\end{bmatrix},\quad
\mathbf{w}=\begin{bmatrix}\mathbf{n}_g\\ \mathbf{n}_a\\ \mathbf{n}_{bg}\\ \mathbf{n}_{ba}\\ n_\tau\end{bmatrix},\qquad
f(\mathbf{x},\mathbf{u},\mathbf{w})=
\begin{bmatrix}
\boldsymbol{\omega}_m-\mathbf{b}_g-\mathbf{n}_g\\[2pt]
{}^{G}\mathbf{v}_I+\tfrac12\big({}^{G}\mathbf{R}_I(\mathbf{a}_m-\mathbf{b}_a-\mathbf{n}_a)+{}^{G}\mathbf{g}\big)\Delta t\\[2pt]
{}^{G}\mathbf{R}_I(\mathbf{a}_m-\mathbf{b}_a-\mathbf{n}_a)+{}^{G}\mathbf{g}\\[2pt]
\mathbf{n}_{bg}\\ \mathbf{n}_{ba}\\ \mathbf{0}_{3\times1}\\ n_\tau
\end{bmatrix}
$$

离散状态转移（论文式(1)）：$\mathbf{x}_{i+1}=\mathbf{x}_i\boxplus\big(\Delta t\, f(\mathbf{x}_i,\mathbf{u}_i,\mathbf{w}_i)\big)$。

**公式→源码定位（连续运动学的离散积分实现）**

| 公式 | 源码定位 |
|---|---|
| ${}^{G}\mathbf{R}_{I}\leftarrow{}^{G}\mathbf{R}_I\,\mathrm{Exp}((\boldsymbol{\omega}_m-\mathbf{b}_g)\Delta t)$ | [src/IMU_Processing.cpp#L412](../src/IMU_Processing.cpp#L412) |
| $\mathbf{a}^{G}={}^{G}\mathbf{R}_I(\mathbf{a}_m-\mathbf{b}_a)+{}^{G}\mathbf{g}$ | [src/IMU_Processing.cpp#L415](../src/IMU_Processing.cpp#L415) |
| ${}^{G}\mathbf{p}_I\leftarrow{}^{G}\mathbf{p}_I+{}^{G}\mathbf{v}_I\Delta t+\tfrac12\mathbf{a}^{G}\Delta t^2$ | [src/IMU_Processing.cpp#L418](../src/IMU_Processing.cpp#L418) |
| ${}^{G}\mathbf{v}_I\leftarrow{}^{G}\mathbf{v}_I+\mathbf{a}^{G}\Delta t$ | [src/IMU_Processing.cpp#L421](../src/IMU_Processing.cpp#L421) |

### 17.2 误差状态定义与本仓库的索引布局（重要）

误差状态用 $\boxminus$ 定义（论文式(3)：$\mathbf{x}\boxminus\hat{\mathbf{x}}\sim\mathcal{N}(\mathbf{0},\hat{\mathbf{P}})$）。姿态用**右扰动**、其余用加性：

$$
{}^{G}\mathbf{R}_I={}^{G}\hat{\mathbf{R}}_I\,\mathrm{Exp}(\delta\boldsymbol{\theta}),\qquad
(\cdot)=\hat{(\cdot)}+\delta(\cdot)\ \text{(其余分量)}
$$

**注意：代码里 19 维误差状态的索引顺序与论文书写顺序不同**——论文写作 $[\mathbf{R},\mathbf{p},\mathbf{v},\mathbf{b}_g,\mathbf{b}_a,\mathbf{g},\tau]$，
而代码把 $\tau$（逆曝光时间）放在下标 6（夹在 $\mathbf{p}$ 与 $\mathbf{v}$ 之间）。以 `StatesGroup::operator+`
（[include/common_lib.h](../include/common_lib.h)）的 `state_add.block` 下标为准：

| 误差分量 | 代码下标 | 注入方式 | common_lib.h |
|---|---|---|---|
| $\delta\boldsymbol{\theta}$（姿态） | `[0:2]` | ${}^{G}\mathbf{R}_I\!\cdot\!\mathrm{Exp}(\delta\boldsymbol\theta)$ | `rot_end = rot_end * Exp(add(0),add(1),add(2))` |
| $\delta\mathbf{p}$（位置） | `[3:5]` | 加性 | `pos_end += add.block<3,1>(3,0)` |
| $\delta\tau$（逆曝光） | `[6]` | 加性 | `inv_expo_time += add(6,0)` |
| $\delta\mathbf{v}$（速度） | `[7:9]` | 加性 | `vel_end += add.block<3,1>(7,0)` |
| $\delta\mathbf{b}_g$（陀螺零偏） | `[10:12]` | 加性 | `bias_g += add.block<3,1>(10,0)` |
| $\delta\mathbf{b}_a$（加计零偏） | `[13:15]` | 加性 | `bias_a += add.block<3,1>(13,0)` |
| $\delta\mathbf{g}$（重力） | `[16:18]` | 加性 | `gravity += add.block<3,1>(16,0)` |

下面所有块矩阵下标都按这张表来（这也是 `F_x`/`cov_w` 里 `block<3,3>(7,0)` 之类下标的由来）。

### 17.3 连续误差动力学 $\dot{\delta\mathbf{x}}=\mathbf{A}\,\delta\mathbf{x}+\mathbf{B}\,\mathbf{w}$

把名义量代入 $f$、减去名义轨迹、保留一阶，逐分量得到（记 $\hat{\mathbf{a}}\triangleq\mathbf{a}_m-\hat{\mathbf{b}}_a$，$\hat{\boldsymbol\omega}\triangleq\boldsymbol\omega_m-\hat{\mathbf{b}}_g$）：

**姿态**（标准 $SO(3)$ 右误差动力学）：
$$
\dot{\delta\boldsymbol{\theta}}=-\,\hat{\boldsymbol\omega}^{\wedge}\,\delta\boldsymbol{\theta}-\delta\mathbf{b}_g-\mathbf{n}_g
$$

**位置**（精确关系，非近似）：$\dot{\delta\mathbf{p}}=\delta\mathbf{v}$

**速度**：把 ${}^{G}\mathbf{R}_I={}^{G}\hat{\mathbf{R}}_I\mathrm{Exp}(\delta\boldsymbol\theta)\approx{}^{G}\hat{\mathbf{R}}_I(\mathbf{I}+\delta\boldsymbol\theta^\wedge)$ 代入 ${}^{G}\mathbf{R}_I\hat{\mathbf{a}}+{}^{G}\mathbf{g}$，
用叉乘反对称 $\mathbf{a}^\wedge\mathbf{b}=-\mathbf{b}^\wedge\mathbf{a}$：
$$
\dot{\delta\mathbf{v}}
={}^{G}\hat{\mathbf{R}}_I\,\delta\boldsymbol\theta^\wedge\hat{\mathbf{a}}-{}^{G}\hat{\mathbf{R}}_I\delta\mathbf{b}_a+\delta\mathbf{g}-{}^{G}\hat{\mathbf{R}}_I\mathbf{n}_a
=-{}^{G}\hat{\mathbf{R}}_I\,\hat{\mathbf{a}}^{\wedge}\,\delta\boldsymbol\theta-{}^{G}\hat{\mathbf{R}}_I\,\delta\mathbf{b}_a+\delta\mathbf{g}-{}^{G}\hat{\mathbf{R}}_I\mathbf{n}_a
$$

**其余**：$\dot{\delta\mathbf{b}}_g=\mathbf{n}_{bg}$，$\dot{\delta\mathbf{b}}_a=\mathbf{n}_{ba}$，$\dot{\delta\mathbf{g}}=\mathbf{0}$，$\dot{\delta\tau}=n_\tau$（随机游走）。

### 17.4 离散状态转移雅可比 $\mathbf{F}_{\tilde{\mathbf{x}}}$ 逐块（↔ 代码 `F_x`）

离散化 $\delta\mathbf{x}_{k+1}=\mathbf{F}_{\tilde{\mathbf{x}}}\,\delta\mathbf{x}_k+\mathbf{F}_{\mathbf{w}}\,\mathbf{w}_k$。除姿态-姿态块用指数精确形式外，其余块取 $\mathbf{I}+\mathbf{A}\Delta t$。逐块：

**公式→源码定位（可点击）**

| 块（行←列，代码下标） | 数学式 | 推导要点 | 源码定位 |
|---|---|---|---|
| $(\delta\boldsymbol\theta,\delta\boldsymbol\theta)$ = `(0,0)` | $\mathrm{Exp}(-\hat{\boldsymbol\omega}\Delta t)$ | 前后帧同一 IMU 增量做共轭 $\mathrm{Exp}(\delta\theta_{k+1})=\mathbf{U}^{-1}\mathrm{Exp}(\delta\theta_k)\mathbf{U}$，$\mathbf{U}=\mathrm{Exp}(\hat{\boldsymbol\omega}\Delta t)$，取伴随得 $\delta\theta_{k+1}=\mathbf{U}^{-1}\delta\theta_k$（**精确**，非一阶） | [src/IMU_Processing.cpp#L385](../src/IMU_Processing.cpp#L385) |
| $(\delta\boldsymbol\theta,\delta\mathbf{b}_g)$ = `(0,10)` | $-\mathbf{I}\,\Delta t$ | 严格应为 $-\mathbf{J}_r(\hat{\boldsymbol\omega}\Delta t)\Delta t$（右雅可比）；$\Delta t$ 很小时 $\mathbf{J}_r\approx\mathbf{I}$，代码取一阶近似 $-\mathbf{I}\Delta t$ | [src/IMU_Processing.cpp#L386](../src/IMU_Processing.cpp#L386) |
| $(\delta\mathbf{p},\delta\mathbf{v})$ = `(3,7)` | $\mathbf{I}\,\Delta t$ | $\dot{\delta\mathbf p}=\delta\mathbf v$ 直接离散 | [src/IMU_Processing.cpp#L388](../src/IMU_Processing.cpp#L388) |
| $(\delta\mathbf{v},\delta\boldsymbol\theta)$ = `(7,0)` | $-{}^{G}\hat{\mathbf{R}}_I\,\hat{\mathbf{a}}^{\wedge}\,\Delta t$ | 17.3 速度式的 $\delta\boldsymbol\theta$ 系数；代码 `acc_avr` 即已减零偏的 $\hat{\mathbf a}$ | [src/IMU_Processing.cpp#L389](../src/IMU_Processing.cpp#L389) |
| $(\delta\mathbf{v},\delta\mathbf{b}_a)$ = `(7,13)` | $-{}^{G}\hat{\mathbf{R}}_I\,\Delta t$ | 同上式 $\delta\mathbf b_a$ 系数 | [src/IMU_Processing.cpp#L390](../src/IMU_Processing.cpp#L390) |
| $(\delta\mathbf{v},\delta\mathbf{g})$ = `(7,16)` | $\mathbf{I}\,\Delta t$ | 重力直接进 $\dot{\delta\mathbf v}$ | [src/IMU_Processing.cpp#L391](../src/IMU_Processing.cpp#L391) |
| $(\delta\tau,\delta\tau),(\delta\mathbf{b}_g,\cdot),(\delta\mathbf{b}_a,\cdot),(\delta\mathbf{g},\cdot)$ 对角 | $\mathbf{I}$ | 随机游走/常量模型，自己对自己恒等 | `F_x.setIdentity()` [L382](../src/IMU_Processing.cpp#L382) |

其余块为零。$\mathbf{F}_{\tilde{\mathbf{x}}}$ 的非零稀疏结构（按代码下标）：

$$
\mathbf{F}_{\tilde{\mathbf{x}}}=\mathbf{I}_{19}+
\begin{bmatrix}
(\text{Exp}-\mathbf{I}) & & & & -\mathbf{I}\Delta t & & \\
 & \mathbf{0} & & \mathbf{I}\Delta t & & & \\
 & & 0 & & & & \\
-{}^{G}\hat{\mathbf{R}}_I\hat{\mathbf{a}}^\wedge\Delta t & & & \mathbf{0} & & -{}^{G}\hat{\mathbf{R}}_I\Delta t & \mathbf{I}\Delta t\\
 & & & & \mathbf{0} & & \\
 & & & & & \mathbf{0} & \\
 & & & & & & \mathbf{0}
\end{bmatrix}
\ \ (\text{行/列序：}\delta\boldsymbol\theta,\delta\mathbf p,\delta\tau,\delta\mathbf v,\delta\mathbf b_g,\delta\mathbf b_a,\delta\mathbf g)
$$

> 说明：本仓库 `UndistortPcl` 只显式写了上表这几块（`ba_bg_est_en`/`gravity_est_en` 关闭时对应块被跳过），
> 未显式写 $(\delta\boldsymbol\theta,\delta\boldsymbol\theta)$ 之外更高阶的耦合，属实现上的常见一阶近似。

### 17.5 过程噪声雅可比 $\mathbf{F}_{\mathbf{w}}$ 与离散过程噪声 $\mathbf{Q}$（↔ 代码 `cov_w`）

连续过程噪声经 $\mathbf{F}_{\mathbf{w}}$ 映射进误差状态，代码直接把离散协方差 $\mathbf{Q}=\mathbf{F}_{\mathbf{w}}\,\mathbf{Q}_c\,\mathbf{F}_{\mathbf{w}}^T$
以 `cov_w`（已乘 $\Delta t^2$）给出：

| 噪声→误差（代码下标块） | 数学式 | 源码定位 |
|---|---|---|
| $\mathbf{n}_g\to\delta\boldsymbol\theta$ = `(0,0)` | $\boldsymbol\Sigma_{gyr}\,\Delta t^2$ | [src/IMU_Processing.cpp#L396](../src/IMU_Processing.cpp#L396) |
| $\mathbf{n}_a\to\delta\mathbf{v}$ = `(7,7)` | ${}^{G}\hat{\mathbf{R}}_I\,\boldsymbol\Sigma_{acc}\,{}^{G}\hat{\mathbf{R}}_I^{T}\,\Delta t^2$ | [src/IMU_Processing.cpp#L397](../src/IMU_Processing.cpp#L397) |
| $\mathbf{n}_{bg}\to\delta\mathbf{b}_g$ = `(10,10)` | $\boldsymbol\Sigma_{bg}\,\Delta t^2$ | [src/IMU_Processing.cpp#L398](../src/IMU_Processing.cpp#L398) |
| $\mathbf{n}_{ba}\to\delta\mathbf{b}_a$ = `(13,13)` | $\boldsymbol\Sigma_{ba}\,\Delta t^2$ | [src/IMU_Processing.cpp#L399](../src/IMU_Processing.cpp#L399) |
| $n_\tau\to\delta\tau$ = `(6,6)` | $\sigma_{\tau}^2\,\Delta t^2$ | [src/IMU_Processing.cpp#L395](../src/IMU_Processing.cpp#L395) |

> 为什么加计噪声块是 ${}^{G}\hat{\mathbf{R}}_I\boldsymbol\Sigma_{acc}{}^{G}\hat{\mathbf{R}}_I^T$：$\mathbf{n}_a$ 定义在 IMU 系，
> 而 $\delta\mathbf{v}$ 在世界系，$\dot{\delta\mathbf v}$ 里 $\mathbf{n}_a$ 的系数是 $-{}^{G}\hat{\mathbf{R}}_I$，故其协方差被 ${}^{G}\hat{\mathbf{R}}_I(\cdot){}^{G}\hat{\mathbf{R}}_I^T$ 旋到世界系。

### 17.6 协方差传播（↔ 代码 `state_inout.cov = F_x * cov * F_x^T + cov_w`）

$$
\hat{\mathbf{P}}_{k|k-1}=\mathbf{F}_{\tilde{\mathbf{x}}}\,\mathbf{P}_{k-1}\,\mathbf{F}_{\tilde{\mathbf{x}}}^{T}+\mathbf{Q}
$$

**公式→源码定位**

| 公式 | 源码定位 |
|---|---|
| $\hat{\mathbf{P}}=\mathbf{F}_{\tilde{\mathbf{x}}}\mathbf{P}\mathbf{F}_{\tilde{\mathbf{x}}}^{T}+\mathbf{Q}$ | [src/IMU_Processing.cpp#L401](../src/IMU_Processing.cpp#L401) |

传播得到的 $(\hat{\mathbf{x}},\hat{\mathbf{P}})$ 即 `voxel_map.cpp::StateEstimation` 里的 `state_propagat` 与 `state_.cov`，
作为 12/15 节 ESIKF 更新的先验。

### 17.7 LiDAR 点面测量雅可比 $\mathbf{H}$ 的严格推导（论文式(16)–(19)）

论文把去畸变点投到世界系（式(16)）：${}^{G}\hat{\mathbf{p}}_j^{\kappa}={}^{G}\hat{\mathbf{T}}_I^{\kappa}\,{}^{I}\mathbf{T}_L\,{}^{L}\mathbf{p}_j$，
真值点落在体素平面上（式(18)）：$0=(\mathbf{n}_j^{gt})^T({}^{G}\mathbf{T}_I{}^{I}\mathbf{T}_L{}^{L}\mathbf{p}_j^{gt}-\mathbf{q}_j^{gt})$，
测量方程（式(19)）含点/法向/中心三类噪声 $\mathbf{v}_l=(\delta^{L}\mathbf{p}_j,\delta\mathbf{n}_j,\delta\mathbf{q}_j)$。

记 IMU 系下的点 ${}^{I}\mathbf{p}_j={}^{I}\mathbf{T}_L{}^{L}\mathbf{p}_j$（代码 `point_this = extR_*point_b_+extT_`），
世界点 ${}^{G}\mathbf{p}_j={}^{G}\mathbf{R}_I\,{}^{I}\mathbf{p}_j+{}^{G}\mathbf{p}_I$，残差就是点到面有符号距离：

$$
r_j=\mathbf{n}_j^T({}^{G}\mathbf{p}_j-\mathbf{q}_j)=\mathbf{n}_j^T{}^{G}\mathbf{p}_j+d_j\ \ (\text{代码 }\texttt{dis\_to\_plane\_},\ d_j=-\mathbf{n}_j^T\mathbf{q}_j)
$$

下面把“对位姿做右扰动、线性化出 $\mathbf{H}$”这一步**逐步展开**。

**先列出用到的公式/恒等式**（$(\cdot)^\wedge$ 表示 3 维向量到 $3\times3$ 反对称矩阵的映射）：

| 记号 | 公式 | 说明 |
|---|---|---|
| (I1) 右扰动模型 | ${}^{G}\mathbf{R}_I={}^{G}\hat{\mathbf{R}}_I\,\mathrm{Exp}(\delta\boldsymbol\theta)$，${}^{G}\mathbf{p}_I={}^{G}\hat{\mathbf{p}}_I+\delta\mathbf{p}$ | 姿态误差 $\delta\boldsymbol\theta$ 定义在机体系（右乘），位置加性 |
| (I2) $\mathrm{Exp}$ 小角度一阶 | $\mathrm{Exp}(\delta\boldsymbol\theta)=\mathbf{I}+\tfrac{\sin\phi}{\phi}\delta\boldsymbol\theta^\wedge+\tfrac{1-\cos\phi}{\phi^2}(\delta\boldsymbol\theta^\wedge)^2\approx\mathbf{I}+\delta\boldsymbol\theta^\wedge$ | $\phi=\|\delta\boldsymbol\theta\|\to0$ 时 $\tfrac{\sin\phi}{\phi}\to1,\ \tfrac{1-\cos\phi}{\phi^2}\to\tfrac12$，二阶项丢掉 |
| (I3) 反对称=叉乘 | $\mathbf{a}^\wedge\mathbf{b}=\mathbf{a}\times\mathbf{b}$ | 反对称矩阵乘向量 = 叉乘 |
| (I4) 叉乘反交换 | $\mathbf{a}^\wedge\mathbf{b}=-\,\mathbf{b}^\wedge\mathbf{a}$ | 即 $\mathbf{a}\times\mathbf{b}=-\mathbf{b}\times\mathbf{a}$ |
| (I5) 反对称转置 | $(\mathbf{a}^\wedge)^T=-\,\mathbf{a}^\wedge$ | 反对称矩阵转置=取负 |
| (I6) 一阶保留 | 丢弃 $\delta\boldsymbol\theta\!\cdot\!\delta\mathbf{p}$、$(\delta\boldsymbol\theta^\wedge)^2$ 等二阶小量 | 线性化 |

**起点**（把 ${}^{G}\mathbf{p}_j={}^{G}\mathbf{R}_I\,{}^{I}\mathbf{p}_j+{}^{G}\mathbf{p}_I$ 中的真值位姿写成“名义 $\boxplus$ 误差”）：

**① 代入右扰动 (I1)：**
$$
{}^{G}\mathbf{p}_j={}^{G}\hat{\mathbf{R}}_I\,\mathrm{Exp}(\delta\boldsymbol\theta)\,{}^{I}\mathbf{p}_j+\big({}^{G}\hat{\mathbf{p}}_I+\delta\mathbf{p}\big)
$$

**② 用 (I2) 把 $\mathrm{Exp}$ 一阶展开并乘开：**
$$
\approx{}^{G}\hat{\mathbf{R}}_I(\mathbf{I}+\delta\boldsymbol\theta^\wedge)\,{}^{I}\mathbf{p}_j+{}^{G}\hat{\mathbf{p}}_I+\delta\mathbf{p}
={}^{G}\hat{\mathbf{R}}_I\,{}^{I}\mathbf{p}_j+{}^{G}\hat{\mathbf{R}}_I\,\delta\boldsymbol\theta^\wedge\,{}^{I}\mathbf{p}_j+{}^{G}\hat{\mathbf{p}}_I+\delta\mathbf{p}
$$

**③ 归并名义世界点** ${}^{G}\hat{\mathbf{p}}_j\triangleq{}^{G}\hat{\mathbf{R}}_I\,{}^{I}\mathbf{p}_j+{}^{G}\hat{\mathbf{p}}_I$：
$$
={}^{G}\hat{\mathbf{p}}_j+{}^{G}\hat{\mathbf{R}}_I\,\delta\boldsymbol\theta^\wedge\,{}^{I}\mathbf{p}_j+\delta\mathbf{p}
$$

**④ 关键一步：用 (I4) 把 $\delta\boldsymbol\theta^\wedge\,{}^{I}\mathbf{p}_j$ 换成 $-({}^{I}\mathbf{p}_j)^\wedge\,\delta\boldsymbol\theta$**（把待求的 $\delta\boldsymbol\theta$ 提到最右边，才能读出雅可比）：
$$
\boxed{\ {}^{G}\mathbf{p}_j\approx{}^{G}\hat{\mathbf{p}}_j-{}^{G}\hat{\mathbf{R}}_I\,({}^{I}\mathbf{p}_j)^\wedge\,\delta\boldsymbol\theta+\delta\mathbf{p}\ }
$$

于是世界点的一阶扰动为 $\delta({}^{G}\mathbf{p}_j)={}^{G}\mathbf{p}_j-{}^{G}\hat{\mathbf{p}}_j=-{}^{G}\hat{\mathbf{R}}_I\,({}^{I}\mathbf{p}_j)^\wedge\,\delta\boldsymbol\theta+\delta\mathbf{p}$。

**⑤ 把残差看成“状态的函数”，在真值/名义两处各取一次值，相减。**

残差 $r_j$ 依赖状态，只通过世界点 ${}^{G}\mathbf{p}_j$ 依赖（法向 $\mathbf{n}_j$、平面中心 $\mathbf{q}_j$ 来自地图，本次线性化里视为**常量**）：

$$
r_j(\mathbf{x})=\mathbf{n}_j^T\big({}^{G}\mathbf{p}_j(\mathbf{x})-\mathbf{q}_j\big)
$$

- 在**真值**（含误差）位姿处：$\ r_j=\mathbf{n}_j^T({}^{G}\mathbf{p}_j-\mathbf{q}_j)$
- 在**名义**（估计）位姿处：$\ \hat r_j=\mathbf{n}_j^T({}^{G}\hat{\mathbf{p}}_j-\mathbf{q}_j)$（$=$ 代码 `dis_to_plane_`）

两者相减，**常量 $\mathbf{q}_j$（以及等价的 $d_j=-\mathbf{n}_j^T\mathbf{q}_j$）直接抵消**，只剩世界点的扰动：

$$
\delta r_j\triangleq r_j-\hat r_j
=\mathbf{n}_j^T\big({}^{G}\mathbf{p}_j-{}^{G}\hat{\mathbf{p}}_j\big)
=\mathbf{n}_j^T\,\delta({}^{G}\mathbf{p}_j)
$$

**⑥ 代入第 ④ 步得到的 $\delta({}^{G}\mathbf{p}_j)=-{}^{G}\hat{\mathbf{R}}_I({}^{I}\mathbf{p}_j)^\wedge\delta\boldsymbol\theta+\delta\mathbf{p}$，把 $\mathbf{n}_j^T$ 乘进去分成两项：**

$$
\delta r_j=\mathbf{n}_j^T\Big(-{}^{G}\hat{\mathbf{R}}_I({}^{I}\mathbf{p}_j)^\wedge\,\delta\boldsymbol\theta+\delta\mathbf{p}\Big)
=\underbrace{-\,\mathbf{n}_j^T\,{}^{G}\hat{\mathbf{R}}_I\,({}^{I}\mathbf{p}_j)^\wedge}_{\displaystyle \mathbf{H}_{\theta,j}\ (1\times3)}\ \delta\boldsymbol\theta
+\underbrace{\mathbf{n}_j^T}_{\displaystyle \mathbf{H}_{p,j}\ (1\times3)}\ \delta\mathbf{p}
$$

这一步本质就是**一阶泰勒展开** $r_j(\mathbf{x})\approx r_j(\hat{\mathbf{x}})+\mathbf{H}_j\,\delta\mathbf{x}$，因此 $\mathbf{H}_j=\dfrac{\partial r_j}{\partial\delta\mathbf{x}}\Big|_{\delta\mathbf{x}=0}$。每行雅可比：

$$
\mathbf{H}_j=[\,\mathbf{H}_{\theta,j}\ \ \mathbf{H}_{p,j}\,]=\big[-\mathbf{n}_j^T{}^{G}\hat{\mathbf{R}}_I({}^{I}\mathbf{p}_j)^\wedge\ \ \ \mathbf{n}_j^T\big]\quad(\text{对 6 维位姿 } [\delta\boldsymbol\theta,\ \delta\mathbf{p}])
$$

**⑦ 这个一阶增量怎么进 EKF（得到观测 $z_j$）。** 论文式(18) 的测量假设是“真值点恰好落在平面上”，即真值残差为零：$r_j(\mathbf{x})=0$。
把 ⑥ 的泰勒式令为 0：
$$
0=r_j(\mathbf{x})\approx \hat r_j+\mathbf{H}_j\,\delta\mathbf{x}
\quad\Longrightarrow\quad
\mathbf{H}_j\,\delta\mathbf{x}\approx\underbrace{-\hat r_j}_{\displaystyle z_j}
$$

于是**观测量 $z_j=-\hat r_j=-$`dis_to_plane_`**（代码 `meas_vec(i) = -dis_to_plane_`），线性观测模型即 $z_j=\mathbf{H}_j\,\delta\mathbf{x}$——正好对上 4.1 节的 $\mathbf{z}\approx\mathbf{H}\delta\mathbf{x}$、以及第 12/15 节把 $(\mathbf{H},\mathbf{z})$ 塞进法方程求 $\delta\mathbf{x}$ 的过程。

**与代码对齐**（`voxel_map.cpp`）：代码令
$\mathbf{A}\triangleq({}^{I}\mathbf{p}_j)^\wedge\,{}^{G}\hat{\mathbf{R}}_I^{T}\,\mathbf{n}_j$（`A = point_crossmat * state_.rot_end.transpose() * normal`）。
对它转置，并用 (I5) $\big(({}^{I}\mathbf{p}_j)^\wedge\big)^T=-({}^{I}\mathbf{p}_j)^\wedge$：
$$
\mathbf{A}^T=\mathbf{n}_j^T\,{}^{G}\hat{\mathbf{R}}_I\big(({}^{I}\mathbf{p}_j)^\wedge\big)^T
=-\,\mathbf{n}_j^T\,{}^{G}\hat{\mathbf{R}}_I\,({}^{I}\mathbf{p}_j)^\wedge=\mathbf{H}_{\theta,j}
$$
所以 `Hsub.row(i) << A[0],A[1],A[2], n_x,n_y,n_z` 正好是 $[\mathbf{H}_{\theta,j}\ \ \mathbf{H}_{p,j}]$（前 3 列 $=\mathbf{A}^T=\mathbf{H}_{\theta,j}$，后 3 列 $=\mathbf{n}_j^T=\mathbf{H}_{p,j}$），
`meas_vec(i) = -dis_to_plane_` 即 $z_j=-r_j$。

**公式→源码定位（可点击）**

| 公式 | 源码定位 |
|---|---|
| ${}^{I}\mathbf{p}_j={}^{I}\mathbf{T}_L{}^{L}\mathbf{p}_j$（`point_this = extR_*point_b_+extT_`） | [src/voxel_map.cpp#L425](../src/voxel_map.cpp#L425) |
| $\mathbf{A}=({}^{I}\mathbf{p}_j)^\wedge{}^{G}\hat{\mathbf{R}}_I^{T}\mathbf{n}_j$ | [src/voxel_map.cpp#L460](../src/voxel_map.cpp#L460) |
| $\mathbf{H}_j=[\mathbf{H}_{\theta,j},\ \mathbf{n}_j^T]$ | [src/voxel_map.cpp#L461](../src/voxel_map.cpp#L461) |
| $z_j=-r_j$（$r_j=$`dis_to_plane_`） | [src/voxel_map.cpp#L464](../src/voxel_map.cpp#L464) |

测量噪声 $\mathbf{R}$ 由式(19) 的三类噪声传播而来：平面参数 $(\delta\mathbf{n}_j,\delta\mathbf{q}_j)$ 经 $\mathbf{J}_{nq}$ 投到残差得 $\sigma_{plane}^2$，
点噪声 $\delta^{L}\mathbf{p}_j$（含论文式(20) 的光束发散项）经 ${}^{G}\hat{\mathbf{R}}_I$ 到世界系再投到法向得 $\sigma_{point}^2$——
这正是 15.3 节的 `sigma_l`（[src/voxel_map.cpp#L454](../src/voxel_map.cpp#L454)）与 `R_inv`（[src/voxel_map.cpp#L456](../src/voxel_map.cpp#L456)）。

> 实现细节：代码里算 $\mathbf{J}_{nq}$/`sigma_l`（权重）用的是 `state_propagat.rot_end`（先验位姿，
> [src/voxel_map.cpp#L432](../src/voxel_map.cpp#L432)），而算 $\mathbf{A}$（雅可比）用的是当前迭代 `state_.rot_end`
> （[src/voxel_map.cpp#L460](../src/voxel_map.cpp#L460)）——权重用先验位姿近似、雅可比用最新位姿，是常见的效率折中，不影响收敛。

### 17.8 ESIKF 更新与本仓库实现的等价对齐（论文式(11)）

论文式(11) 的标准迭代更新：

$$
\mathbf{K}=\big(\mathbf{H}^T\mathbf{R}^{-1}\mathbf{H}+\hat{\mathbf{P}}^{-1}\big)^{-1}\mathbf{H}^T\mathbf{R}^{-1},\qquad
\hat{\mathbf{x}}^{\kappa+1}=\hat{\mathbf{x}}^{\kappa}\boxplus\Big(-\mathbf{K}\mathbf{z}^{\kappa}-(\mathbf{I}-\mathbf{K}\mathbf{H})(\hat{\mathbf{x}}^{\kappa}\boxminus\hat{\mathbf{x}})\Big)
$$

本仓库 `voxel_map.cpp` 用**信息形式**实现（$\mathbf{W}=\mathbf{R}^{-1}$ 为对角权重）：

$$
\mathbf{K}_1=\big(\mathbf{H}^T\mathbf{W}\mathbf{H}+\hat{\mathbf{P}}^{-1}\big)^{-1},\quad
\mathbf{G}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{H},\quad
\delta\mathbf{x}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z}+\mathbf{v}-\mathbf{G}\mathbf{v}
$$

其中 `vec = state_propagat - state_` 即 $\mathbf{v}=\hat{\mathbf{x}}\boxminus\hat{\mathbf{x}}^{\kappa}=-(\hat{\mathbf{x}}^{\kappa}\boxminus\hat{\mathbf{x}})$。
逐项对照即可验证二者**代数等价**：

- $-\mathbf{K}\mathbf{z}^\kappa$（论文，$\mathbf{z}^\kappa=r$）$=-\mathbf{K}_1\mathbf{H}^T\mathbf{W}\,r=+\mathbf{K}_1\mathbf{H}^T\mathbf{W}\,(\underbrace{-r}_{\texttt{meas\_vec}})=$ 代码 `K_1*HTz`；
- $-(\mathbf{I}-\mathbf{K}\mathbf{H})(\hat{\mathbf{x}}^{\kappa}\boxminus\hat{\mathbf{x}})=(\mathbf{I}-\mathbf{G})\mathbf{v}=\mathbf{v}-\mathbf{G}\mathbf{v}=$ 代码 `vec - G*vec`。

**公式→源码定位（可点击）**

| 公式 | 源码定位 |
|---|---|
| $\mathbf{K}_1=(\mathbf{H}^T\mathbf{W}\mathbf{H}+\hat{\mathbf{P}}^{-1})^{-1}$ | [src/voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |
| $\mathbf{G}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{H}$ | [src/voxel_map.cpp#L526](../src/voxel_map.cpp#L526) |
| $\mathbf{v}=\hat{\mathbf{x}}\boxminus\hat{\mathbf{x}}^{\kappa}=$`state_propagat - state_` | [src/voxel_map.cpp#L527](../src/voxel_map.cpp#L527) |
| $\delta\mathbf{x}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z}+\mathbf{v}-\mathbf{G}\mathbf{v}$ | [src/voxel_map.cpp#L529](../src/voxel_map.cpp#L529) |
| $\hat{\mathbf{x}}^{\kappa+1}=\hat{\mathbf{x}}^{\kappa}\boxplus\delta\mathbf{x}$ | [src/voxel_map.cpp#L531](../src/voxel_map.cpp#L531) |
| $\hat{\mathbf{P}}\leftarrow(\mathbf{I}-\mathbf{G})\hat{\mathbf{P}}$（收敛后） | [src/voxel_map.cpp#L546](../src/voxel_map.cpp#L546) |

> 论文 Algorithm 1 强调 LiDAR/相机是**序贯更新**：先用 LiDAR 迭代收敛，再把收敛后的 $(\hat{\mathbf{x}},\hat{\mathbf{P}})$
> 作为视觉更新的先验（式(5)–(7) 的两步贝叶斯分解）。本仓库 RTK 观测是在 LiDAR 那步里作为额外 3 维直接拼进
> $\mathbf{H}/\mathbf{R}^{-1}/\mathbf{z}$（见 15.8 节），不是单独一步。

### 17.9 小结（传播侧 vs 更新侧雅可比）

| | 传播侧（本节 17.3–17.6） | 更新侧（本节 17.7–17.8 / 12、15 节） |
|---|---|---|
| 雅可比 | $\mathbf{F}_{\tilde{\mathbf{x}}}$（状态转移）、$\mathbf{F}_{\mathbf{w}}$（过程噪声） | $\mathbf{H}$（测量对误差状态） |
| 作用 | 把先验 $\mathbf{P}$ 往前推：$\hat{\mathbf{P}}=\mathbf{F}\mathbf{P}\mathbf{F}^T+\mathbf{Q}$ | 构造法方程、算增益 $\mathbf{K}$/增量 $\delta\mathbf{x}$ |
| 代码位置 | `IMU_Processing.cpp::UndistortPcl`（L382–L401） | `voxel_map.cpp::StateEstimation`（L460–L546） |
| 论文出处 | 式(1)(2)、Sec. IV-A/IV-C | 式(9)(10)(11)、式(16)–(19)，Sec. IV-D/VI-A |

---

## 18. 误差状态从零详解（ESKF 入门必读）

本节面向**完全没接触过误差状态卡尔曼滤波（ESKF）的读者**，把“误差状态”这个概念从动机到细节讲透。
读完后，前面 11/12/15/17 节里那些 $\delta\mathbf{x}$、$\boxplus$、$\mathrm{Exp}(\delta\boldsymbol\theta)$、$\mathbf{F}_{\tilde{\mathbf{x}}}$
就都有落脚点了。所有结论都对到本仓库 `include/common_lib.h` 的 `StatesGroup` 实现。

### 18.1 先说动机：为什么不直接对“状态本身”做卡尔曼滤波？

普通卡尔曼滤波（KF/EKF）默认状态是一个**欧氏向量**——能随便加减、能定义高斯分布、能算协方差。
但我们的状态里有**姿态** ${}^{G}\mathbf{R}_I\in SO(3)$（一个 $3\times3$ 旋转矩阵），它**不是欧氏向量**：

- 两个旋转矩阵相加 $\mathbf{R}_1+\mathbf{R}_2$ 一般**不再是旋转矩阵**（不正交、行列式不为 1），没有物理意义；
- 旋转有 3 个自由度，但用矩阵是 9 个数、用四元数是 4 个数（还带单位模长约束），**过参数化**——直接对 9/4 个数定义
  协方差会奇异、会漂出流形；
- 欧拉角虽然是 3 个数，但有**万向锁奇异**，不能全局用。

ESKF 的解法是把状态**拆成两层**：

1. **名义状态（nominal / 估计）** $\hat{\mathbf{x}}$：滤波器实际持有的量，姿态就存成正常的旋转矩阵，
   在流形上做大范围、非线性的 IMU 积分（`StatesGroup`，见 [common_lib.h#L225](../include/common_lib.h#L225)）。
2. **误差状态（error state）** $\delta\mathbf{x}$：真值相对名义的**一个很小的偏差**，它被刻意定义成一个
   **纯欧氏的小向量**（姿态误差只用 3 维），因此可以放心地对它做 EKF 的所有线性代数（加减、协方差、求逆）。

一句话：**名义状态负责“在流形上跑”，误差状态负责“在切空间里做卡尔曼滤波”。** 协方差、雅可比、增益，
全都是针对误差状态 $\delta\mathbf{x}$ 的，不是针对 $\hat{\mathbf{x}}$ 的。

### 18.2 三个状态的关系：真值 / 名义 / 误差

| 记号 | 名称 | 是什么 | 在代码里 |
|---|---|---|---|
| $\mathbf{x}$ | 真值（ground truth） | 我们想估计、但永远不知道的真实状态 | 不存在于代码，只在推导里出现 |
| $\hat{\mathbf{x}}$ | 名义 / 估计 | 滤波器持有的当前最优估计 | `StatesGroup state_` |
| $\delta\mathbf{x}$ | 误差状态 | 真值相对名义的小偏差（19 维欧氏向量） | 19 维向量 `solution` |

三者的定义关系（$\boxplus/\boxminus$ 见 18.3）：

$$
\boxed{\ \mathbf{x}=\hat{\mathbf{x}}\boxplus\delta\mathbf{x}\ }\qquad\Longleftrightarrow\qquad
\boxed{\ \delta\mathbf{x}=\mathbf{x}\boxminus\hat{\mathbf{x}}\ }
$$

论文式(3) 就是说这个 $\delta\mathbf{x}$ 服从零均值高斯：$\delta\mathbf{x}=\mathbf{x}\boxminus\hat{\mathbf{x}}\sim\mathcal{N}(\mathbf{0},\hat{\mathbf{P}})$。
**注意 $\hat{\mathbf{P}}$（`state_.cov`）是 $\delta\mathbf{x}$ 的协方差**，描述“真值大概离名义有多远”，不是名义本身的协方差。

### 18.3 $\boxplus$ 和 $\boxminus$ 到底怎么算（逐分量写清楚）

$\boxplus$（“流形加”，boxplus）把一个误差向量“加”回名义状态；$\boxminus$（“流形减”，boxminus）反过来求两个状态之间的误差向量。
对**不同分量规则不同**：

**(a) 欧氏分量**（位置 $\mathbf{p}$、速度 $\mathbf{v}$、零偏 $\mathbf{b}_g,\mathbf{b}_a$、重力 $\mathbf{g}$、逆曝光 $\tau$）——就是普通加减：

$$
\hat{\mathbf{p}}\boxplus\delta\mathbf{p}=\hat{\mathbf{p}}+\delta\mathbf{p},\qquad
\mathbf{p}\boxminus\hat{\mathbf{p}}=\mathbf{p}-\hat{\mathbf{p}}
$$

**(b) 姿态分量** ${}^{G}\mathbf{R}_I\in SO(3)$——**不能相加**，必须走指数/对数映射：

$$
\hat{\mathbf{R}}\boxplus\delta\boldsymbol\theta=\hat{\mathbf{R}}\cdot\mathrm{Exp}(\delta\boldsymbol\theta),\qquad
\mathbf{R}\boxminus\hat{\mathbf{R}}=\mathrm{Log}\!\big(\hat{\mathbf{R}}^{T}\mathbf{R}\big)
$$

也就是：姿态误差 $\delta\boldsymbol\theta$ 是“从 $\hat{\mathbf{R}}$ 转到 $\mathbf{R}$ 需要的那一小段旋转”，用一个 **3 维旋转向量**表示。

**与代码逐行对应**（`StatesGroup`）：

| 运算 | 数学 | 代码 | 行号 |
|---|---|---|---|
| $\boxplus$ 姿态 | $\hat{\mathbf{R}}\cdot\mathrm{Exp}(\delta\boldsymbol\theta)$ | `rot_end = rot_end * Exp(add(0),add(1),add(2))` | [common_lib.h#L194](../include/common_lib.h#L194) |
| $\boxplus$ 其余 | 加法 | `pos_end += add.block<3,1>(3,0)` 等 | [common_lib.h#L195](../include/common_lib.h#L195)–[L200](../include/common_lib.h#L200) |
| $\boxminus$ 姿态 | $\mathrm{Log}(\hat{\mathbf{R}}^{T}\mathbf{R})$ | `a.block<3,1>(0,0)=Log(b.rot_end.transpose()*this->rot_end)` | [common_lib.h#L207](../include/common_lib.h#L207)–[L208](../include/common_lib.h#L208) |
| $\boxminus$ 其余 | 减法 | `a.block<3,1>(3,0)=this->pos_end-b.pos_end` 等 | [common_lib.h#L209](../include/common_lib.h#L209)–[L214](../include/common_lib.h#L214) |

> 代码里 `state_ += solution` 就是在做 $\hat{\mathbf{x}}\leftarrow\hat{\mathbf{x}}\boxplus\delta\mathbf{x}$；
> `state_propagat - state_` 就是在做 $\hat{\mathbf{x}}_{\text{prior}}\boxminus\hat{\mathbf{x}}_{\text{当前}}$。
> 这就是为什么姿态那一项调 `Exp`/`Log` 而不是简单 `+`/`-`。

### 18.4 $\mathrm{Exp}$ / $\mathrm{Log}$ 是什么（旋转向量 ↔ 旋转矩阵）

- $\mathrm{Exp}(\delta\boldsymbol\theta)$：把一个**3 维旋转向量** $\delta\boldsymbol\theta\in\mathbb{R}^3$ 变成一个 $3\times3$ 旋转矩阵。
  几何含义：$\delta\boldsymbol\theta$ 的**方向 = 转轴**，**模长 $\|\delta\boldsymbol\theta\|$ = 转角（弧度）**。
  具体用罗德里格斯公式（代码 [so3_math.h#L44](../include/utils/so3_math.h#L44)）：
  $$
  \mathrm{Exp}(\delta\boldsymbol\theta)=\mathbf{I}+\sin\!\phi\,\mathbf{K}+(1-\cos\phi)\mathbf{K}^2,\quad
  \phi=\|\delta\boldsymbol\theta\|,\ \mathbf{K}=\Big(\tfrac{\delta\boldsymbol\theta}{\phi}\Big)^{\wedge}
  $$
  （$(\cdot)^\wedge$ 表示把 3 维向量变成 $3\times3$ 反对称矩阵；$\phi\to0$ 时 $\mathrm{Exp}\to\mathbf{I}$。）
- $\mathrm{Log}(\mathbf{R})$：$\mathrm{Exp}$ 的逆，把旋转矩阵变回那个 3 维旋转向量（代码 [so3_math.h#L61](../include/utils/so3_math.h#L61)）。

所以“姿态误差 $\delta\boldsymbol\theta=[\,\delta\theta_x,\ \delta\theta_y,\ \delta\theta_z\,]^T$”这三个数，合起来就是一个**微小转动**：
朝某个轴转了 $\|\delta\boldsymbol\theta\|$ 弧度。当估计很准时 $\delta\boldsymbol\theta\approx\mathbf{0}$，$\mathrm{Exp}(\mathbf{0})=\mathbf{I}$（不转）。

### 18.5 姿态为什么必须这么处理（一个反例）

假设名义姿态 $\hat{\mathbf{R}}$ 是绕 $z$ 轴转 $89^\circ$，真值 $\mathbf{R}$ 是绕 $z$ 轴转 $90^\circ$。

- **错误做法（直接减矩阵）**：$\mathbf{R}-\hat{\mathbf{R}}$ 得到的 9 个数拼成的矩阵**不是旋转矩阵**，也不好说“误差是多少”；
  而且要对 9 个数定义协方差，但真实自由度只有 1，协方差必然奇异。
- **正确做法（$\boxminus$）**：$\delta\boldsymbol\theta=\mathrm{Log}(\hat{\mathbf{R}}^T\mathbf{R})=[0,0,\ \tfrac{\pi}{180}]^T$，
  一个干净的 3 维小向量——“绕 $z$ 轴还差 $1^\circ$”。对它定义 $3\times3$ 协方差、做线性化，全都成立、且无奇异。

这就是 ESKF 相对“直接 EKF”的核心好处：**用切空间里的最小 3 维参数化描述姿态误差**，既无过参数化、又局部无奇异。

### 18.6 “右扰动”还是“左扰动”？本仓库用哪个

姿态误差可以定义在**机体系**（右乘，本仓库用的）或**世界系**（左乘）：

$$
\text{右扰动（本仓库）：}\ \mathbf{R}=\hat{\mathbf{R}}\,\mathrm{Exp}(\delta\boldsymbol\theta)
\qquad\text{左扰动：}\ \mathbf{R}=\mathrm{Exp}(\delta\boldsymbol\theta)\,\hat{\mathbf{R}}
$$

代码 `rot_end * Exp(...)`（`Exp` 在**右边**）说明本仓库用的是**右扰动**，即 $\delta\boldsymbol\theta$ 定义在 IMU/机体局部系。
这个选择不是随意的，它决定了后面所有姿态相关雅可比的具体形式：

- 传播雅可比里姿态-姿态块是 $\mathrm{Exp}(-\hat{\boldsymbol\omega}\Delta t)$（见 17.4）；
- 测量雅可比里出现的是 $-\mathbf{n}^T\hat{\mathbf{R}}({}^{I}\mathbf{p})^\wedge$（见 17.7）——那个 ${}^{I}\mathbf{p}$（IMU 系下的点）和 $\hat{\mathbf{R}}$ 的位置，正是右扰动推出来的。

换成左扰动，这些公式的 $\hat{\mathbf{R}}$ 位置和正负号都会变，代码也会长得不一样。**读雅可比时先认准“右扰动”**，否则会觉得符号对不上。

### 18.7 误差状态每个分量的精确定义（逐个写死）

本仓库误差状态 $\delta\mathbf{x}\in\mathbb{R}^{19}$，各分量定义、代码下标、物理意义如下（下标依据见 17.2；姿态用 $\boxminus$，其余用减法）：

| 分量 | 定义式 | 代码下标 | 物理意义 / 单位 / 典型量级 |
|---|---|---|---|
| $\delta\boldsymbol\theta$ | $\mathrm{Log}(\hat{\mathbf{R}}^{T}{}^{G}\mathbf{R}_I)$ | `[0:2]` | 机体系下姿态小旋转向量，rad；收敛后 $\sim10^{-3}$ |
| $\delta\mathbf{p}$ | ${}^{G}\mathbf{p}_I-{}^{G}\hat{\mathbf{p}}_I$ | `[3:5]` | 位置误差，m |
| $\delta\tau$ | $\tau-\hat\tau$ | `[6]` | 逆曝光时间误差，无量纲（视觉用） |
| $\delta\mathbf{v}$ | ${}^{G}\mathbf{v}_I-{}^{G}\hat{\mathbf{v}}_I$ | `[7:9]` | 速度误差，m/s |
| $\delta\mathbf{b}_g$ | $\mathbf{b}_g-\hat{\mathbf{b}}_g$ | `[10:12]` | 陀螺零偏误差，rad/s |
| $\delta\mathbf{b}_a$ | $\mathbf{b}_a-\hat{\mathbf{b}}_a$ | `[13:15]` | 加计零偏误差，m/s² |
| $\delta\mathbf{g}$ | ${}^{G}\mathbf{g}-{}^{G}\hat{\mathbf{g}}$ | `[16:18]` | 重力向量误差，m/s²（模长约束下只有 2 个真自由度） |

> 只有 $\delta\boldsymbol\theta$ 是“流形切空间坐标”（通过 $\mathrm{Log}$ 得到），其余 6 组都是朴素的“真值 − 估计”。
> 这也解释了为什么 `operator-`（[common_lib.h#L204](../include/common_lib.h#L204)）里只有姿态那一行用 `Log`，别的都是直接相减。

### 18.8 误差状态为什么“很小、且始终在 0 附近”——error reset

这是 ESKF 能用一阶线性化的关键。流程是（对应本仓库迭代更新）：

1. **传播**：IMU 积分只更新**名义** $\hat{\mathbf{x}}$（大范围非线性运动都由它吸收），误差 $\delta\mathbf{x}$ 名义上保持 0，只把协方差 $\hat{\mathbf{P}}$ 往前推（17.6）。
2. **更新**：用观测解出一个**小的** $\delta\mathbf{x}$（因为名义已经比较准，真值离名义不远）。
3. **注入 + 复位（reset）**：把 $\delta\mathbf{x}$ 通过 $\boxplus$ 折回名义 `state_ += solution`，然后**下一轮重新以 $\delta\mathbf{x}=0$ 为线性化点**。

因为每次都把误差“清回 0”，所以所有雅可比（$\mathbf{F}_{\tilde{\mathbf{x}}}$、$\mathbf{H}$）永远在 $\delta\mathbf{x}=0$ 处求导，一阶泰勒近似始终成立、精度高。
这正是 15.6/17 节反复强调的“每轮重新线性化”的含义——迭代 ESKF（IEKF）在**一帧内**就把“解小增量 → 注入 → 再以 0 为原点重解”重复到收敛。

> 严格说，reset 会给协方差带来一个雅可比（通常 $\approx\mathbf{I}$），多数实现（含本仓库）直接略去，影响可忽略。

### 18.9 协方差 $\mathbf{P}$ 到底是谁的协方差（最容易搞混的一点）

**$\mathbf{P}$（`state_.cov`，19×19）是误差状态 $\delta\mathbf{x}$ 的协方差，不是名义 $\hat{\mathbf{x}}$ 的。** 它回答的是
“真值 $\mathbf{x}$ 大概偏离当前估计 $\hat{\mathbf{x}}$ 多远、各方向偏多少、彼此相关性如何”。因此：

- 姿态那 $3\times3$ 块 `cov(0:2,0:2)` 是 $\delta\boldsymbol\theta$ 的协方差——“姿态还有多不确定”，单位 rad²；
- 传播时不确定度变大（$\hat{\mathbf{P}}=\mathbf{F}\mathbf{P}\mathbf{F}^T+\mathbf{Q}$，17.6）；更新后变小（$\mathbf{P}\leftarrow(\mathbf{I}-\mathbf{G})\mathbf{P}$，12.3/17.8）；
- 更新里的“先验项”$\|\delta\mathbf{x}-\mathbf{v}\|_{\mathbf{P}^{-1}}^2$（15.5）用的就是这个 $\mathbf{P}$——它衡量“偏离先验要付多大代价”。

### 18.10 和代码对象的总对照

| 概念 | 数学 | 代码 |
|---|---|---|
| 名义状态 $\hat{\mathbf{x}}$ | $[{}^{G}\mathbf{R}_I,{}^{G}\mathbf{p}_I,\dots]$ | `StatesGroup state_`（`rot_end` 是矩阵，其余是向量），[common_lib.h#L225](../include/common_lib.h#L225)–[L232](../include/common_lib.h#L232) |
| 误差状态 $\delta\mathbf{x}$ | 19 维欧氏小向量 | EKF 每轮解出的 `solution`（19×1） |
| $\boxplus$ 注入 | $\hat{\mathbf{x}}\boxplus\delta\mathbf{x}$ | `state_ += solution`（`operator+=`），[common_lib.h#L192](../include/common_lib.h#L192) |
| $\boxminus$ 求差 | $\mathbf{a}\boxminus\mathbf{b}$ | `a - b`（`operator-`），[common_lib.h#L204](../include/common_lib.h#L204) |
| 误差协方差 $\mathbf{P}$ | $\mathrm{Cov}(\delta\mathbf{x})$ | `state_.cov`（19×19），[common_lib.h#L232](../include/common_lib.h#L232) |
| 先验偏移 $\mathbf{v}$ | $\hat{\mathbf{x}}_{\text{prior}}\boxminus\hat{\mathbf{x}}_{\text{当前}}$ | `vec = state_propagat - state_`，[voxel_map.cpp#L527](../src/voxel_map.cpp#L527) |

### 18.11 一个具体小例子（把 $\delta\boldsymbol\theta$ 算出来）

设某轮迭代中：名义姿态 $\hat{\mathbf{R}}$ = 绕 $z$ 轴 $30.0^\circ$，观测更新解出的真值方向 $\mathbf{R}$ = 绕 $z$ 轴 $30.5^\circ$。

$$
\delta\boldsymbol\theta=\mathrm{Log}(\hat{\mathbf{R}}^{T}\mathbf{R})=\mathrm{Log}\big(\text{绕 }z\text{ 轴 }0.5^\circ\big)
=\Big[\,0,\ 0,\ 0.5\times\tfrac{\pi}{180}\,\Big]^T\approx[\,0,\ 0,\ 0.0087\,]^T\ \text{rad}
$$

它是个很小的 3 维向量（模长 $0.0087$ rad = $0.5^\circ$）。注入时 $\hat{\mathbf{R}}\leftarrow\hat{\mathbf{R}}\cdot\mathrm{Exp}([0,0,0.0087]^T)$，
名义姿态就从 $30.0^\circ$ 变成 $30.5^\circ$，随后 $\delta\boldsymbol\theta$ 复位为 0，进入下一轮。位置/速度等分量同理，只不过是直接加减，更直观。

---

> **读完本节再回看**：11.1 的“$\mathbf{R}\leftarrow\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta)$”、17.2 的索引表、
> 17.4 的 $\mathbf{F}_{\tilde{\mathbf{x}}}$ 姿态块为什么是 $\mathrm{Exp}$ 而非 $\mathbf{I}$、17.7 里 $\mathbf{H}$ 为什么带 $\hat{\mathbf{R}}$ 和 $(\cdot)^\wedge$——
> 都是“姿态误差用右扰动 $\delta\boldsymbol\theta$、其余用加性”这一个定义推出来的直接结果。
