# 预测（Propagation）与更新（Update）：公式与代码逐条对应

本文是对话式整理：回答"迭代 EKF 里预测和更新分别做什么、公式怎么落到代码里"这两个问题，
所有行号均按 2026-07-28 的代码状态核实（`voxel_map.cpp` 当天新增了一处 2 行的
`// todo 可能有越界？？？` 注释，本文行号已按此后的状态核对，不是之前的旧行号）。

更详细的推导可以看：
- [VOXELMAPMANAGER_ITERATION_DATAFLOW.md](VOXELMAPMANAGER_ITERATION_DATAFLOW.md) 第11-16节：`StateEstimation` 的完整公式推导与速查表
- [UNCERTAINTY_COMPUTE_AND_USAGE.md](UNCERTAINTY_COMPUTE_AND_USAGE.md)：uncertainty 怎么算、怎么进权重
- [EXECUTION_FLOW_AND_ALGORITHM.md](EXECUTION_FLOW_AND_ALGORITHM.md)：整个程序的调用链

本文只聚焦"预测 vs 更新"这一个问题，把两者各自的核心公式和代码行对齐。

---

## 1. 一句话区分

- **预测**：没有新观测的情况下，单靠 IMU 运动学模型，把状态和协方差往前推到这一时刻——只回答"应该在哪、有多不确定"。
- **更新**：有了这一时刻的真实观测（LiDAR 点到面残差、可选 RTK），反复修正预测值，直到收敛——而且是**迭代**做的，每轮都重新关联、重新线性化，不是解一次就完事。

对应函数：
- 预测：`ImuProcess::UndistortPcl(...)`，[src/IMU_Processing.cpp:237-541](../src/IMU_Processing.cpp#L237-L541)
- 更新：`VoxelMapManager::StateEstimation(...)`，[src/voxel_map.cpp:338-568](../src/voxel_map.cpp#L338-L568)

---

## 2. 预测：均值传播

标称运动学：

$$
\dot{\mathbf{R}} = \mathbf{R}(\boldsymbol{\omega}-\mathbf{b}_g)^\wedge,\qquad
\dot{\mathbf{p}} = \mathbf{v},\qquad
\dot{\mathbf{v}} = \mathbf{R}(\mathbf{a}-\mathbf{b}_a) + \mathbf{g}
$$

离散化（`dt` 为相邻两个 IMU 采样间隔）：

| 公式 | 代码 |
|---|---|
| $\mathbf{R}_{k+1}=\mathbf{R}_k\cdot\mathrm{Exp}((\boldsymbol{\omega}-\mathbf{b}_g)dt)$ | [src/IMU_Processing.cpp#L412](../src/IMU_Processing.cpp#L412) `R_imu = R_imu * Exp_f;` |
| $\mathbf{a}_{world}=\mathbf{R}(\mathbf{a}-\mathbf{b}_a)+\mathbf{g}$ | [src/IMU_Processing.cpp#L415](../src/IMU_Processing.cpp#L415) `acc_imu = R_imu * acc_avr + state_inout.gravity;` |
| $\mathbf{p}_{k+1}=\mathbf{p}_k+\mathbf{v}_k dt+\frac{1}{2}\mathbf{a}_{world}dt^2$ | [src/IMU_Processing.cpp#L418](../src/IMU_Processing.cpp#L418) `pos_imu = pos_imu + vel_imu*dt + 0.5*acc_imu*dt*dt;` |
| $\mathbf{v}_{k+1}=\mathbf{v}_k+\mathbf{a}_{world}dt$ | [src/IMU_Processing.cpp#L421](../src/IMU_Processing.cpp#L421) `vel_imu = vel_imu + acc_imu*dt;` |

这一步结束后的 `rot_end/vel_end/pos_end` 就是调用方（`voxel_map.cpp::StateEstimation`）拿到的
`state_propagat`——迭代更新公式（第4节）里"先验偏移"用的正是这个值。

---

## 3. 预测：协方差传播 `P = F_x·P·F_x^T + Q`

状态布局（19 维，`DIM_STATE`）：`[0-2]姿态误差 [3-5]位置 [6]曝光时间 [7-9]速度 [10-12]陀螺bias [13-15]加计bias [16-18]重力`
（见 [include/common_lib.h#L31](../include/common_lib.h#L31)）。

`F_x` 是误差状态对误差状态自身的雅可比，逐块对应各误差量的运动学导数：

| 代码 | 含义 |
|---|---|
| [IMU_Processing.cpp#L385](../src/IMU_Processing.cpp#L385) `F_x.block<3,3>(0,0) = Exp(angvel_avr,-dt);` | $\partial(\text{姿态误差})/\partial(\text{姿态误差})$ |
| [IMU_Processing.cpp#L386](../src/IMU_Processing.cpp#L386) `F_x.block<3,3>(0,10) = -Eye3d*dt;` | $\partial(\text{姿态误差})/\partial(\text{陀螺bias})$ |
| [IMU_Processing.cpp#L388](../src/IMU_Processing.cpp#L388) `F_x.block<3,3>(3,7) = Eye3d*dt;` | $\partial(\text{位置})/\partial(\text{速度})=dt$（$d\mathbf{p}=\mathbf{v}\,dt$） |
| [IMU_Processing.cpp#L389](../src/IMU_Processing.cpp#L389) `F_x.block<3,3>(7,0) = -R_imu*acc_avr_skew*dt;` | $\partial(\text{速度})/\partial(\text{姿态误差})=-\mathbf{R}(\mathbf{a}-\mathbf{b}_a)^\wedge dt$ |
| [IMU_Processing.cpp#L390](../src/IMU_Processing.cpp#L390) `F_x.block<3,3>(7,13) = -R_imu*dt;` | $\partial(\text{速度})/\partial(\text{加计bias})=-\mathbf{R}\,dt$ |
| [IMU_Processing.cpp#L391](../src/IMU_Processing.cpp#L391) `F_x.block<3,3>(7,16) = Eye3d*dt;` | $\partial(\text{速度})/\partial(\text{重力})=dt$（重力直接叠加进 $\dot{\mathbf{v}}$） |

其余块（bias、重力对自身）默认恒等（随机游走模型），来自 `F_x.setIdentity()`。

过程噪声 `Q`（`cov_w`）——各噪声源投影到误差状态的方差：

| 代码 | 含义 |
|---|---|
| [IMU_Processing.cpp#L396](../src/IMU_Processing.cpp#L396) `cov_w.block<3,3>(0,0).diagonal() = cov_gyr*dt*dt;` | 陀螺噪声 → 姿态误差 |
| [IMU_Processing.cpp#L397](../src/IMU_Processing.cpp#L397) `cov_w.block<3,3>(7,7) = R*cov_acc.asDiagonal()*R^T*dt*dt;` | 加计噪声（转世界系）→ 速度误差 |
| [IMU_Processing.cpp#L398](../src/IMU_Processing.cpp#L398) `cov_w.block<3,3>(10,10).diagonal() = cov_bias_gyr*dt*dt;` | 陀螺 bias 随机游走 |
| [IMU_Processing.cpp#L399](../src/IMU_Processing.cpp#L399) `cov_w.block<3,3>(13,13).diagonal() = cov_bias_acc*dt*dt;` | 加计 bias 随机游走 |

最终传播：

```cpp
state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;   // IMU_Processing.cpp:401
```

预测阶段还顺带做了运动畸变补偿（[IMU_Processing.cpp#L492-L539](../src/IMU_Processing.cpp#L492-L539)）：
把扫描期间每个点按其自身时刻的插值位姿，变换补偿回扫描结束时刻的统一位姿系——这是"预测"独有的
副产品（需要整段扫描的轨迹才能做），更新阶段不碰这一步。

---

## 4. 更新：观测方程、权重与雅可比

点到面残差：$h(\mathbf{x}) = \mathbf{n}\cdot(\mathbf{R}\mathbf{p}_L+\mathbf{t}) + d$（理想匹配时应为 0）。

| 公式 | 代码 |
|---|---|
| $\partial h/\partial \delta\boldsymbol{\theta}$（旋转扰动） | [voxel_map.cpp#L460](../src/voxel_map.cpp#L460) `V3D A(point_crossmat * state_.rot_end.transpose() * normal_);` |
| $\partial h/\partial \delta\mathbf{p}=\mathbf{n}^T$（平移扰动） | [voxel_map.cpp#L461](../src/voxel_map.cpp#L461) `Hsub.row(i) << A, normal_[0], normal_[1], normal_[2];` |
| 新息 $y=-h(\mathbf{x}_k)$ | [voxel_map.cpp#L464](../src/voxel_map.cpp#L464) `meas_vec(i) = -ptpl_list_[i].dis_to_plane_;` |

测量权重（每条残差各自的方差，取倒数即信息量）：

| 公式 | 代码 |
|---|---|
| $\sigma_l=\mathbf{J}_{nq}\Sigma_\pi\mathbf{J}_{nq}^T$（平面自身拟合不确定度投影） | [voxel_map.cpp#L454](../src/voxel_map.cpp#L454) `double sigma_l = J_nq * plane_var_ * J_nq.transpose();` |
| $R_i^{-1}=1/(0.001+\sigma_l+\mathbf{n}^T\Sigma_{p,w}\mathbf{n})$ | [voxel_map.cpp#L456](../src/voxel_map.cpp#L456) `R_inv(i) = 1.0/(0.001 + sigma_l + normal_.transpose()*var*normal_);` |

`rtk_good` 为真时，额外追加 3 维 RTK 位置观测（`Hsub`/`R_inv`/`meas_vec` 各补 3 行），
见 [voxel_map.cpp#L467-L511](../src/voxel_map.cpp#L467-L511)。

> 排查记录：这里 `effct_feat_num_` 在 `rtk_good` 时会 `+=3`（[voxel_map.cpp#L409-L412](../src/voxel_map.cpp#L409-L412)），
> 但填充 LIO 部分的循环 `for (int i=0;i<effct_feat_num_;i++)` 仍然直接读 `ptpl_list_[i]`
> （[voxel_map.cpp#L421-L465](../src/voxel_map.cpp#L421-L465)），`ptpl_list_` 并没有跟着扩容 3 个——
> 最后 3 次循环存在越界读取。代码里已经标了 `// todo 可能有越界？？？`（[voxel_map.cpp#L414](../src/voxel_map.cpp#L414)），
> 待确认修复方式（比如把循环上界改成 `ptpl_list_.size()`，RTK 那 3 行单独在 467 行之后的块里写）。

---

## 5. 更新：信息形式的卡尔曼增益 + 迭代修正项

标准卡尔曼增益要对 `effct_feat_num_ × effct_feat_num_` 的 `R` 求逆，代价太大；这里用矩阵求逆引理
等价变形，只求逆一个 `DIM_STATE × DIM_STATE` 的小矩阵：

| 公式 | 代码 |
|---|---|
| $\mathbf{H}^T\mathbf{W}\mathbf{z}$ | [voxel_map.cpp#L521](../src/voxel_map.cpp#L521) `auto &&HTz = Hsub_T_R_inv * meas_vec;` |
| $\mathbf{H}^T\mathbf{W}\mathbf{H}$ | [voxel_map.cpp#L523](../src/voxel_map.cpp#L523) `H_T_H.block<6,6>(0,0) = Hsub_T_R_inv * Hsub;` |
| $\mathbf{K}_1=(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})^{-1}$ | [voxel_map.cpp#L525](../src/voxel_map.cpp#L525) |
| $\mathbf{G}=\mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{H}$ | [voxel_map.cpp#L526](../src/voxel_map.cpp#L526) |

$\mathbf{K}_1=(\mathbf{H}^T\mathbf{W}\mathbf{H}+\mathbf{P}^{-1})^{-1}$ 的物理含义："先验精度 + 测量精度 = 后验精度"，
取逆就是后验协方差——和标准卡尔曼增益公式数学等价，只是换了个求逆代价小很多的等价形式。

**迭代修正公式**（区别单步 EKF 的关键）：

```cpp
auto vec = state_propagat - state_;                              // voxel_map.cpp:527
solution = K_1.block<DIM_STATE,6>(0,0) * HTz
         + vec.block<DIM_STATE,1>(0,0)
         - G.block<DIM_STATE,6>(0,0) * vec.block<6,1>(0,0);       // voxel_map.cpp:529
state_ += solution;                                                // voxel_map.cpp:531
```

对应公式 $\delta\mathbf{x} = \mathbf{K}_1\mathbf{H}^T\mathbf{W}\mathbf{z} + (\mathbf{I}-\mathbf{G})(\hat{\mathbf{x}}_{k|k-1}\ominus\mathbf{x}_k)$：
`vec` 是"当前迭代值相对最初 IMU 预测值的偏移"，`G` 在观测能约束到的方向上趋于 $\mathbf{I}$（此时修正几乎完全交给观测，`vec-G·vec`趋于0），
在观测约束不到的方向上趋于 $\mathbf{0}$（此时直接信最初的 IMU 预测值，不会被观测硬拽）——这就是为什么
`state_propagat` 必须单独传进来，不是随手存的调试变量。

收敛后（[voxel_map.cpp#L542](../src/voxel_map.cpp#L542) 判据满足）更新一次后验协方差：

```cpp
state_.cov.block<DIM_STATE,DIM_STATE>(0,0) =
    (I_STATE.block<DIM_STATE,DIM_STATE>(0,0) - G.block<DIM_STATE,DIM_STATE>(0,0))
    * state_.cov.block<DIM_STATE,DIM_STATE>(0,0);                  // voxel_map.cpp:546-547
```

---

## 6. 预测与更新怎么串起来（一轮迭代）

```
预测（每帧一次）：
  UndistortPcl: IMU积分 → state_propagat, 去畸变点云

更新（每帧内迭代 max_iterations_ 次）：
  for iterCount in [0, max_iterations):
    1. 用当前 state_ 把点云变换到世界系，重算每点世界协方差
    2. BuildResidualListOMP 重新关联匹配平面（位姿变了，关联结果也可能变）
    3. 构建 Hsub / R_inv / meas_vec（LIO + 可选 RTK）
    4. HTz, H_T_H → K_1 → G → vec → solution
    5. state_ += solution；增量够小则记收敛
    6. 收敛（连续2轮）或到达 max_iterations_ → 更新后验协方差，跳出循环
```

关键点：**更新阶段每轮都要重新做步骤1-2**，因为残差关于位姿是非线性的，状态变了、点落在哪个体素/
匹配哪个平面也可能变——这是"迭代"EKF（IEKF）区别于一次性 EKF 的地方。
